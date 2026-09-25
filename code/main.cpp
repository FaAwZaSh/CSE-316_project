/*
 * =====================================================================================
 *  PROJECT: VANGUARD - Event-Driven Multi-Sensor Surveillance System
 * =====================================================================================
 *  MCU:               ATmega32A (16.0 MHz External Crystal Oscillator)
 *  Baud Rate:         9600 Baud (8-N-1) on PD0(RXD)/PD1(TXD)
 * =====================================================================================
 *  BEHAVIOR SPECIFICATION:
 *  1. Standby: Pan parked at Center (1500 us). Quiet serial (no periodic spam).
 *  2. Detection: When PIR triggers, move in that direction and sweep 70° twice
 *     (Center <--> Limit: 4 passes total / 2 complete round trips).
 *  3. Target Lock: If Laser (< 1.2m) & Thermal (elevated signature) detect a
 * target during the sweep, STOP immediately at that position! Turn ON Alert
 * LED. Hold position until another PIR triggers.
 *  4. Safe Limits: Margins on both extremes (1000 us to 2000 us) are protected
 * to prevent physical stop collision and plastic gear stripping.
 *  5. Reporting: Terminal prints details ONLY when PIR triggers, during active
 *     searching, and upon target lock with full sensor pipeline data.
 * =====================================================================================
 *  PIN MAPPING (DIP-40):
 *  - PIR 0 (Left Sector)   : Pin 16 (PD2 / INT0) [External Interrupt 0]
 *  - PIR 1 (Right Sector)  : Pin 17 (PD3 / INT1) [External Interrupt 1]
 *  - Pan Servo (PD5 / OC1A): Pin 19 [Timer1 50 Hz Hardware Fast PWM]
 *  - Laser XSHUT (PA0)     : Pin 40 [VL53L0X Reset / Open-Drain Enable]
 *  - I2C Bus               : Pin 22 (PC0/SCL) & Pin 23 (PC1/SDA)
 *    * VL53L0X Laser ToF   : I2C Addr 0x29
 *    * GY-906 (MLX90614)   : I2C Addr 0x5A
 * =====================================================================================
 */

#include "Adafruit_VL53L0X.h"
#include <Arduino.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/io.h>

/* ---------- Pin Bit Definitions ---------- */
#define PIR0_BIT PD2        /* Pin 16: INT0 input (Left Sector) */
#define PIR1_BIT PD3        /* Pin 17: INT1 input (Right Sector) */
#define PAN_SERVO_BIT PD5   /* Pin 19: Timer1 OC1A Pan Servo Hardware PWM */
#define LASER_XSHUT_BIT PA0 /* Pin 40: VL53L0X Reset / Open-Drain Enable */

/* ---------- I2C Device Addresses & Registers ---------- */
#define MLX90614_I2CADDR 0x5A
#define MLX90614_TA 0x06    /* Ambient Temp Register */
#define MLX90614_TOBJ1 0x07 /* Object 1 Temp Register */
#define VL53L0X_I2CADDR 0x29

/* ---------- Tunables & Thresholds ---------- */
#define PIR_DEBOUNCE_MS 250UL   /* Min ms between PIR trigger events */
#define FEVER_THRESHOLD_C 37.5f /* Thermal alert threshold */
#define NEAR_THRESHOLD_MM 350   /* Max distance for target lock (350 mm / 35 cm) */

/* ---------- Hardware Timer Tick Conversion Macro ---------- */
#define US_TO_TICKS(us)                                                        \
  ((uint16_t)(((uint32_t)(us) * (F_CPU / 1000000UL)) / 8UL))

/* ---------- Pan Servo Tunables (Safe Margins to Prevent Over-Rotation)
 * ---------- */
#define SERVO_MIN_US 1000  /* Safe lower limit (0 deg) */
#define SERVO_MAX_US 2000  /* Safe upper limit (180 deg) */
#define PAN_CENTER_US 1500 /* Center / Neutral pulse (90 deg) */
#define PAN_LEFT_80_US                                                         \
  1050 /* Left Sector Limit (~15 deg, safe sweep without hitting stop) */
#define PAN_RIGHT_80_US                                                        \
  1950 /* Right Sector Limit (~165 deg, safe sweep without hitting stop) */

/* ---------- Sensor Hardware State ---------- */
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool vl53l0x_online = false;
bool mlx90614_online = false;

// Filtered Sensor Data
float filtered_dist_mm = 0.0f;
bool laser_first_read = true;
float filtered_obj_c = 0.0f;
bool thermal_first_read = true;
float current_amb_c = 25.0f;

/* ---------- Volatile Data Shared with ISRs ---------- */
volatile uint32_t pir0_trigger_count = 0;
volatile uint32_t pir1_trigger_count = 0;
volatile uint32_t pir0_last_trigger_ms = 0;
volatile uint32_t pir1_last_trigger_ms = 0;
volatile bool pir0_event_pending = false;
volatile bool pir1_event_pending = false;
static bool pir_active_high =
    true; // true = Active HIGH (3.3V/5V), false = Active LOW (0V)

/* ---------- Turret State Machine ---------- */
enum TurretState {
  TURRET_STANDBY,      // Center hold (1500 us). Silent, waiting for PIR motion.
  TURRET_SEARCHING,    // PIR detected motion: sweeping that sector (80°) twice.
  TURRET_TARGET_LOCKED // Target confirmed by Laser + Thermal: STOP immediately!
                       // Hold position.
};

enum SectorTarget {
  SECTOR_NONE = 0,
  SECTOR_LEFT, // 1500 us <--> 1050 us (Left 70° sweep)
  SECTOR_RIGHT // 1500 us <--> 1950 us (Right 70° sweep)
};

static TurretState turret_state = TURRET_STANDBY;
static SectorTarget active_sector = SECTOR_NONE;
static uint8_t sweep_passes_completed = 0; // 2 passes = 1 complete round trip
static uint32_t startup_stabilize_until =
    0; // Ignore warmup transients on power-up

static uint32_t sweep_cooldown_until =
    0; // 4.0s rest/settle duration at Center post-sweep

static bool lock_logic_enabled =
    true; // Autonomous target locking enabled by default ('k' toggles)

// Servo Motion Control (Timer1 Hardware 50 Hz Frame Synchronized)
volatile uint16_t current_pan_us =
    PAN_CENTER_US; // Active pulse width on OC1A (PD5)
volatile uint16_t target_pan_us = PAN_CENTER_US; // Slew destination pulse width
volatile uint8_t step_us_per_tick = 3; // 3 us per 20ms tick = ~150 us/s
volatile bool pan_is_slewing =
    false; // True while Timer1 ISR is actively stepping pulse
volatile bool turret_motion_paused =
    false; // Independent pause flag for command 'p'

enum SearchPhase { SEARCH_SLEWING, SEARCH_REVERSAL_PAUSE };
static SearchPhase search_phase = SEARCH_SLEWING;
static uint32_t search_pause_start_ms = 0;
const uint16_t reversal_pause_ms =
    300; // 300ms gentle pause before reversing direction

/* ---------- Forward Declarations ---------- */
void i2c_bus_recovery(void);
void scanI2CBus(void);
float readMLX90614TempC(uint8_t reg);
void initTimer1_TurretPWM(void);
void setPanPulse(uint16_t us);
void slewPanTo(uint16_t target_us);
uint16_t getPanPulse(void);
uint32_t getPir0Count(void);
uint32_t getPir1Count(void);
void triggerSectorSearch(SectorTarget sector);
void updateTurretSearch(uint32_t now);
void formatTime(uint32_t total_ms);
void printHelp(void);
void printDetailedStatus(void);
void resetStatistics(void);
void processSerialCommands(void);

/* ---------- External Interrupt Service Routines ---------- */
// INT0 (PD2 / Pin 16): PIR Sensor 0 (Left Sector) with Hardware Debounce
ISR(INT0_vect) {
  uint32_t ms = millis();
  if (ms - pir0_last_trigger_ms >= PIR_DEBOUNCE_MS) {
    pir0_trigger_count++;
    pir0_last_trigger_ms = ms;
    pir0_event_pending = true;
  }
}

// INT1 (PD3 / Pin 17): PIR Sensor 1 (Right Sector) with Hardware Debounce
ISR(INT1_vect) {
  uint32_t ms = millis();
  if (ms - pir1_last_trigger_ms >= PIR_DEBOUNCE_MS) {
    pir1_trigger_count++;
    pir1_last_trigger_ms = ms;
    pir1_event_pending = true;
  }
}

/* ---------- Setup Routine ---------- */
void setup() {
  /* 1. Configure PIR pins as inputs with internal pull-ups to prevent floating
   * pin noise */
  DDRD &= ~((1 << PIR0_BIT) | (1 << PIR1_BIT));
  PORTD |= ((1 << PIR0_BIT) | (1 << PIR1_BIT));

  /* 2. Initialize UART with internal pull-up on RXD (PD0) */
  PORTD |= (1 << PD0);
  Serial.begin(9600);
  Serial.setTimeout(100);
  while (!Serial && millis() < 500) {
  }

  Serial.println();
  Serial.println(
      F("=================================================================="));
  Serial.println(
      F("   PROJECT VANGUARD - EVENT-DRIVEN SURVEILLANCE TESTBENCH         "));
  Serial.println(
      F("=================================================================="));
  Serial.println(
      F(" Clock: 16.0 MHz (External Crystal) | Baud: 9600 | I2C: 100 kHz   "));
  Serial.println(F(" Mode : PIR-Triggered 70° Double Sweep (Pure Sweep | 'k' "
                   "toggles Lock)"));
  Serial.println(
      F(" Safe : Safe Buffers on both extremes (1000 us to 2000 us)         "));
  Serial.println(
      F(" Step 1: Initializing PIR External Interrupts (INT0, INT1)..."));
  Serial.flush();

  /* 3. Configure Hardware Interrupts INT0 & INT1 for RISING edge */
  MCUCR =
      (MCUCR & ~((1 << ISC01) | (1 << ISC00) | (1 << ISC11) | (1 << ISC10))) |
      ((1 << ISC01) | (1 << ISC00) | (1 << ISC11) | (1 << ISC10));
  GIFR = (1 << INTF0) |
         (1 << INTF1); // Direct assignment to clear only INTF0 and INTF1 flags
  GICR |= (1 << INT0) | (1 << INT1);
  sei();

  /* 4. Reset VL53L0X Laser ToF via XSHUT (PA0) */
  Serial.println(F(" Step 2: Resetting VL53L0X XSHUT Pin 40 (PA0)..."));
  Serial.flush();
  DDRA |= (1 << LASER_XSHUT_BIT);
  PORTA &= ~(1 << LASER_XSHUT_BIT); // Drive LOW (0V)
  delay(50);
  PORTA |= (1 << LASER_XSHUT_BIT); // Drive 5V HIGH output
  delay(250); // 250ms settling delay for VL53L0X internal MCU boot

  /* 6. Perform I2C Bus Recovery */
  Serial.println(
      F(" Step 3: Performing I2C Bus Clock Recovery & Starting Wire..."));
  Serial.flush();
  i2c_bus_recovery();

  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  PORTC |= (1 << PC0) | (1 << PC1); // Enable internal pull-ups
  delay(50);

  /* 8. Initialize VL53L0X Laser ToF directly */
  Serial.print(F(" Step 4: Initializing VL53L0X Laser Sensor at 0x29... "));
  _delay_ms(100);
  Wire.beginTransmission(VL53L0X_I2CADDR);
  if (Wire.endTransmission() == 0) {
    if (lox.begin(VL53L0X_I2CADDR, false, &Wire)) {
      vl53l0x_online = true;
      lox.setMeasurementTimingBudgetMicroSeconds(100000);
      Serial.println(F("[SUCCESS] Ready!"));
    } else {
      vl53l0x_online = false;
      Serial.println(
          F("[WARNING] Driver init failed. Retrying in background."));
    }
  } else {
    vl53l0x_online = false;
    Serial.println(F("[WARNING] Address 0x29 NACKed. Retrying in background."));
  }
  Serial.flush();

  /* 7. Scan I2C Bus */
  Serial.println(F(" Step 5: Probing I2C Bus for connected devices..."));
  Serial.flush();
  scanI2CBus();

  /* 9. Verify GY-906 (MLX90614) Thermal Sensor */
  Serial.print(F(" Step 6: Verifying GY-906 (MLX90614) Sensor at 0x5A... "));
  Wire.beginTransmission(MLX90614_I2CADDR);
  if (Wire.endTransmission() == 0) {
    mlx90614_online = true;
    Serial.println(F("[SUCCESS] Ready!"));
  } else {
    mlx90614_online = false;
    Serial.println(F("[WARNING] Not responding. Check power and pull-ups."));
  }
  Serial.flush();

  /* 8. Initialize Timer1 Hardware Fast PWM */
  Serial.println(
      F(" Step 7: Initializing Timer1 50 Hz Fast PWM on PD5 (Pan Pin 19)..."));
  initTimer1_TurretPWM();
  Serial.println(F("         [SUCCESS] Pan Servo parked at Center (1500 us)!"));
  Serial.flush();

  // Allow 2.5s for PIR sensors to stabilize after boot
  startup_stabilize_until = millis() + 2500;

  Serial.println();
  Serial.println(
      F("=================================================================="));
  Serial.println(
      F(" Status: SYSTEM READY IN QUIET STANDBY.                           "));
  Serial.println(
      F(" Pan   : Parked at Center (1500 us).                              "));
  Serial.println(
      F(" Serial: Output is silent until a PIR detects motion.             "));
  Serial.println(
      F(" Type 'h' for command menu | 's' for diagnostic snapshot.         "));
  Serial.println(
      F("=================================================================="));
  Serial.println();
}

/* ---------- Main Loop ---------- */
void loop() {
  uint32_t now = millis();

  /* ----- 1. Periodic Sensor Auto-Recovery Re-Probing (Every 5s if offline)
   * ----- */
  static uint32_t last_reprobe_ms = 0;
  if ((!vl53l0x_online || !mlx90614_online) &&
      (now - last_reprobe_ms >= 3000)) {
    last_reprobe_ms = now;
    if (!vl53l0x_online) {
      // Hard-reset VL53L0X MCU via XSHUT pin before re-initialization
      PORTA &= ~(1 << LASER_XSHUT_BIT);
      _delay_ms(20);
      PORTA |= (1 << LASER_XSHUT_BIT);
      _delay_ms(100);
      vl53l0x_online = lox.begin(VL53L0X_I2CADDR, false, &Wire);
      if (vl53l0x_online)
        lox.setMeasurementTimingBudgetMicroSeconds(100000);
    }
    if (!mlx90614_online) {
      Wire.beginTransmission(MLX90614_I2CADDR);
      mlx90614_online = (Wire.endTransmission() == 0);
    }
  }

  /* ----- 2. Poll PIR Levels for Motion Telemetry ----- */
  bool pir0_pin_high = (PIND & (1 << PIR0_BIT)) ? true : false;
  bool pir1_pin_high = (PIND & (1 << PIR1_BIT)) ? true : false;

  bool pir0_active = pir_active_high ? pir0_pin_high : !pir0_pin_high;
  bool pir1_active = pir_active_high ? pir1_pin_high : !pir1_pin_high;

  /* ----- 3. Pan Servo PWM Continuous Holding ----- */
  // Continuous 50 Hz PWM is maintained at all times by Timer1 hardware.
  // In Standby, OCR1A holds PAN_CENTER_US (1500 us) to keep the motor solidly
  // parked at Center.

  /* ----- 4. Process PIR Trigger Events ----- */

  if (now >= startup_stabilize_until && now >= sweep_cooldown_until) {

    if (turret_state == TURRET_SEARCHING) {
      pir0_event_pending = false;
      pir1_event_pending = false;
    } else if (turret_state == TURRET_STANDBY) {
      if (pir0_event_pending) {
        pir0_event_pending = false;
        pir1_event_pending = false;
        triggerSectorSearch(SECTOR_LEFT);
      } else if (pir1_event_pending) {
        pir0_event_pending = false;
        pir1_event_pending = false;
        triggerSectorSearch(SECTOR_RIGHT);
      }
    } else if (turret_state == TURRET_TARGET_LOCKED) {
      // Hold locked position until user presses 'u' over Serial
      pir0_event_pending = false;
      pir1_event_pending = false;
    }
  } else {
    pir0_event_pending = false;
    pir1_event_pending = false;
  }

  /* ----- 5. Update Non-Blocking Turret Sweep Motion ----- */
  if (turret_state == TURRET_SEARCHING) {
    updateTurretSearch(now);
  }

  /* ----- 5. Non-Blocking Sensor Acquisition (Sampled Every 120ms) ----- */
  static uint32_t last_sensor_poll_ms = 0;
  static uint16_t laser_raw_mm = 0;
  static bool laser_valid = false;
  static uint8_t laser_consecutive_fails = 0;
  static float amb_c = -999.0f;
  static float obj_c = -999.0f;
  static bool thermal_valid = false;

  if (now - last_sensor_poll_ms >= 120) {
    last_sensor_poll_ms = now;

    // Laser reading (VL53L0X)
    if (vl53l0x_online) {
      // Hardware probe check: ensure address 0x29 ACKs on I2C bus
      Wire.beginTransmission(VL53L0X_I2CADDR);
      if (Wire.endTransmission() != 0) {
        if (++laser_consecutive_fails >= 5) {
          laser_consecutive_fails = 0;
          vl53l0x_online =
              false; // Only mark offline if hardware I2C ACK fails!
        }
        laser_valid = false;
      } else {
        laser_consecutive_fails = 0;
        VL53L0X_RangingMeasurementData_t measure;
        lox.rangingTest(&measure, false);
        // RangeStatus != 4 means valid phase reading / target detected
        if (measure.RangeStatus != 4 && measure.RangeMilliMeter >= 15 &&
            measure.RangeMilliMeter <= 2500) {
          laser_raw_mm = measure.RangeMilliMeter;
          laser_valid = true;
          if (laser_first_read) {
            filtered_dist_mm = laser_raw_mm;
            laser_first_read = false;
          } else {
            filtered_dist_mm =
                (0.50f * laser_raw_mm) + (0.50f * filtered_dist_mm);
          }
        } else {
          laser_valid = false; // Out of range or no target present
        }
      }
    }

    // Thermal reading (MLX90614)
    if (mlx90614_online) {
      amb_c = readMLX90614TempC(MLX90614_TA);
      obj_c = readMLX90614TempC(MLX90614_TOBJ1);
      if (amb_c > -40.0f && amb_c < 125.0f && obj_c > -40.0f &&
          obj_c < 200.0f) {
        thermal_valid = true;
        current_amb_c = amb_c;
        if (thermal_first_read) {
          filtered_obj_c = obj_c;
          thermal_first_read = false;
        } else {
          filtered_obj_c = (0.35f * obj_c) + (0.65f * filtered_obj_c);
        }
      } else {
        thermal_valid = false;
      }
    }

    /* ----- 6. Target Lock Detection Logic during Search ----- */
    static uint8_t lock_consecutive_hits = 0;
    if (lock_logic_enabled && turret_state == TURRET_SEARCHING) {
      bool target_candidate = false;
      const __FlashStringHelper *lock_reason = nullptr;

      bool laser_hit = laser_valid && (filtered_dist_mm >= 80.0f) &&
                       (filtered_dist_mm <= (float)NEAR_THRESHOLD_MM);
      bool thermal_hit =
          thermal_valid && ((filtered_obj_c >= FEVER_THRESHOLD_C) ||
                            (filtered_obj_c - current_amb_c >= 1.0f));

      if (laser_hit && thermal_hit) {
        target_candidate = true;
        lock_reason = F("Laser Distance Target + Elevated Thermal "
                        "Signature Confirmed");
      } else if (laser_hit) {
        target_candidate = true;
        lock_reason = F("Laser ToF Proximity Target Detected");
      } else if (thermal_hit) {
        target_candidate = true;
        lock_reason = F("Thermal Heat Signature Detected");
      }

      if (target_candidate) {
        lock_consecutive_hits++;
      } else {
        lock_consecutive_hits = 0;
      }

      // Require 2 consecutive confirmations (~240ms sustained detection) to lock
      if (lock_consecutive_hits >= 2) {
        lock_consecutive_hits = 0;
        turret_state = TURRET_TARGET_LOCKED;
        // Freeze pan immediately and synchronize target_pan_us to
        // current_pan_us
        uint8_t sreg = SREG;
        cli();
        pan_is_slewing = false;
        target_pan_us = current_pan_us;
        SREG = sreg;

        formatTime(now);
        Serial.println();
        Serial.println(F("*****************************************************"
                         "*************"));
        Serial.println(F(" [!] TARGET LOCKED - SENSORS ACQUIRED SIGNATURE!     "
                         "             "));
        Serial.println(F("*****************************************************"
                         "*************"));
        Serial.print(F(" Sector     : "));
        Serial.println(active_sector == SECTOR_LEFT ? F("LEFT SECTOR")
                                                    : F("RIGHT SECTOR"));
        Serial.print(F(" Pan Angle  : "));
        Serial.print(getPanPulse());
        Serial.println(F(" us"));
        Serial.print(F(" Laser ToF  : "));
        if (laser_valid) {
          Serial.print((uint16_t)filtered_dist_mm);
          Serial.print(F(" mm ("));
          Serial.print(filtered_dist_mm / 25.4f, 1);
          Serial.println(F(" in)"));
        } else {
          Serial.println(F("N/A"));
        }
        Serial.print(F(" Thermal TA : "));
        Serial.print(current_amb_c, 1);
        Serial.println(F(" °C"));
        Serial.print(F(" Thermal TO : "));
        Serial.print(filtered_obj_c, 1);
        Serial.print(F(" °C ("));
        Serial.print((filtered_obj_c * 1.8f) + 32.0f, 1);
        Serial.println(F(" °F)"));
        if (thermal_valid) {
          Serial.print(F(" Delta Temp : +"));
          Serial.print(filtered_obj_c - current_amb_c, 1);
          Serial.println(F(" °C"));
        }
        Serial.print(F(" Lock Cause : "));
        Serial.println(lock_reason);
        Serial.println(F(" Turret Status: STOPPED & HOLDING until 'u' key is "
                         "pressed on keyboard."));
        Serial.println(F("*****************************************************"
                         "*************"));
        Serial.println();
      }
    }
  }

  /* ----- 7. Event-Driven Real-time Telemetry Output ----- */
  static uint32_t last_report_ms = 0;

  // During active search: print live scan pipeline every 350ms
  if (turret_state == TURRET_SEARCHING && (now - last_report_ms >= 350)) {
    last_report_ms = now;

    formatTime(now);
    Serial.print(F(" [SCAN "));
    Serial.print(active_sector == SECTOR_LEFT ? F("L") : F("R"));
    Serial.print(F("] Pass "));
    Serial.print(sweep_passes_completed + 1);
    Serial.print(F("/2 | Pan:"));
    Serial.print(getPanPulse());
    Serial.print(F("us | LASER:"));
    if (!vl53l0x_online) {
      Serial.print(F("[OFFLINE]"));
    } else if (laser_valid) {
      Serial.print((uint16_t)filtered_dist_mm);
      Serial.print(F("mm"));
    } else {
      Serial.print(F("[CLEAR]"));
    }
    Serial.print(F(" | THERM:"));
    if (!mlx90614_online) {
      Serial.print(F("[OFFLINE]"));
    } else if (thermal_valid) {
      Serial.print(filtered_obj_c, 1);
      Serial.print(F("C (dt:+"));
      Serial.print(filtered_obj_c - current_amb_c, 1);
      Serial.print(F("C)"));
    } else {
      Serial.print(F("[ERR]"));
    }
    Serial.println();
  }
  // While target is locked: print single status line every 1.5s
  else if (turret_state == TURRET_TARGET_LOCKED &&
           (now - last_report_ms >= 1500)) {
    last_report_ms = now;

    formatTime(now);
    Serial.print(F(" [TARGET LOCKED] Sector:"));
    Serial.print(active_sector == SECTOR_LEFT ? F("LEFT") : F("RIGHT"));
    Serial.print(F(" | Pan:"));
    Serial.print(getPanPulse());
    Serial.print(F("us | Dist:"));
    if (laser_valid) {
      Serial.print((uint16_t)filtered_dist_mm);
      Serial.print(F("mm"));
    } else {
      Serial.print(F("--"));
    }
    Serial.print(F(" | Obj:"));
    if (thermal_valid) {
      Serial.print(filtered_obj_c, 1);
      Serial.print(F("C"));
    } else {
      Serial.print(F("--"));
    }
    Serial.println(F(" | LOCKED (Press 'u' to release)"));
  }
  // In Standby: print periodic status line every 2.0s with live PIR logic
  // states
  else if (turret_state == TURRET_STANDBY && (now - last_report_ms >= 2000)) {
    last_report_ms = now;
    formatTime(now);
    Serial.print(F(" [STANDBY] PIR0:"));
    Serial.print(pir0_active ? F("ACTIVE") : F("IDLE"));
    Serial.print(F(" | PIR1:"));
    Serial.print(pir1_active ? F("ACTIVE") : F("IDLE"));
    Serial.print(F(" | Pan:Center("));
    Serial.print(getPanPulse());
    Serial.print(F("us) PWM:50Hz[HOLD]"));
    if (now < sweep_cooldown_until) {
      Serial.print(F(" [Resting: "));
      Serial.print((sweep_cooldown_until - now) / 1000.0f, 1);
      Serial.print(F("s]"));
    }
    Serial.println();
  }

  /* ----- 8. Handle Serial Commands ----- */
  if (Serial.available() > 0) {
    processSerialCommands();
  }
}

/* ---------- Trigger Sector Search on PIR Event ---------- */
void triggerSectorSearch(SectorTarget sector) {
  active_sector = sector;
  sweep_passes_completed = 0;
  turret_state = TURRET_SEARCHING;
  search_phase = SEARCH_SLEWING;

  uint16_t first_target =
      (sector == SECTOR_LEFT) ? PAN_LEFT_80_US : PAN_RIGHT_80_US;
  slewPanTo(first_target);

  pir0_event_pending = false;
  pir1_event_pending = false;

  formatTime(millis());
  Serial.println();
  Serial.println(
      F("=================================================================="));
  if (sector == SECTOR_LEFT) {
    Serial.println(
        F(" >>> [PIR TRIGGER: LEFT SECTOR] Motion detected on PIR 0 (INT0)!"));
    Serial.println(F("     Starting Gentle 70° Left Sector Search (1500 us <-> "
                     "1050 us, 1 round trip)..."));
  } else {
    Serial.println(
        F(" >>> [PIR TRIGGER: RIGHT SECTOR] Motion detected on PIR 1 (INT1)!"));
    Serial.println(F("     Starting Gentle 70° Right Sector Search (1500 us "
                     "<-> 1950 us, 1 round trip)..."));
  }
  Serial.println(
      F("=================================================================="));
}

/* ---------- Non-Blocking 70° Sector Sweep Engine (Frame-Synchronized State
 * Machine) ---------- */
void updateTurretSearch(uint32_t now) {
  if (turret_motion_paused)
    return;

  // 1. Gentle turnaround pause (300ms hold at limits/center before reversing
  // direction)
  if (search_phase == SEARCH_REVERSAL_PAUSE) {
    if (now - search_pause_start_ms >= reversal_pause_ms) {
      search_phase = SEARCH_SLEWING;
      uint16_t extremity =
          (active_sector == SECTOR_LEFT) ? PAN_LEFT_80_US : PAN_RIGHT_80_US;
      // Passes 0 and 2: slew towards sector extremity
      // Passes 1 and 3: slew back towards Center
      if (sweep_passes_completed % 2 == 0) {
        slewPanTo(extremity);
      } else {
        slewPanTo(PAN_CENTER_US);
      }
    }
    return;
  }

  // 2. Slewing phase: check if current slew waypoint reached
  if (search_phase == SEARCH_SLEWING) {
    if (!pan_is_slewing) {
      sweep_passes_completed++;

      if (sweep_passes_completed >= 2) {
        // Completed all 2 passes (1 round trip)
        turret_state = TURRET_STANDBY;
        active_sector = SECTOR_NONE;
        slewPanTo(PAN_CENTER_US);

        sweep_cooldown_until = now + 4000; // 4.0s settling duration at Center
        pir0_event_pending = false;
        pir1_event_pending = false;

        formatTime(now);
        Serial.println();
        Serial.println(F(" --- [SECTOR SEARCH COMPLETE] 1 round trip (2 "
                         "passes) finished."));
        Serial.println(F("     Turret PARKED at Center (1500 us). Resting "
                         "quietly for 4.0s settling delay."));
        Serial.println(F("-----------------------------------------------------"
                         "-------------"));
      } else {
        search_phase = SEARCH_REVERSAL_PAUSE;
        search_pause_start_ms = now;
      }
    }
  }
}

/* ---------- Timer1 Hardware Fast PWM for Pan Servo ---------- */
// Hardware-Synchronized Slew Engine: Executes exactly every 20.0ms at PWM TOP
ISR(TIMER1_OVF_vect) {
  if (pan_is_slewing && !turret_motion_paused) {
    if (current_pan_us < target_pan_us) {
      uint16_t diff = target_pan_us - current_pan_us;
      if (diff <= step_us_per_tick) {
        current_pan_us = target_pan_us;
        pan_is_slewing = false;
      } else {
        current_pan_us += step_us_per_tick;
      }
    } else if (current_pan_us > target_pan_us) {
      uint16_t diff = current_pan_us - target_pan_us;
      if (diff <= step_us_per_tick) {
        current_pan_us = target_pan_us;
        pan_is_slewing = false;
      } else {
        current_pan_us -= step_us_per_tick;
      }
    } else {
      pan_is_slewing = false;
    }
  }
  OCR1A = current_pan_us * 2; // Prescaler 8 @ 16 MHz: 1 us = 2 ticks (7.5% duty
                              // cycle at 1500 us = Center)
}

void initTimer1_TurretPWM(void) {
  // Set PD5 (Pan OC1A Pin 19) as output
  DDRD |= (1 << PAN_SERVO_BIT);
  PORTD &= ~(1 << PAN_SERVO_BIT);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  ICR1 = 39999; // 50.0 Hz PWM period (20.0 ms @ 16 MHz / 8)
  OCR1A = 3000; // 1500 us Center pulse (7.5% duty cycle = 1.5 ms HIGH pulse)

  // Mode 14: Fast PWM (ICR1 = TOP), Non-Inverting on OC1A (Clear on match, Set
  // at BOTTOM)
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

  setPanPulse(PAN_CENTER_US);

  // Enable Timer1 Overflow Interrupt for hardware-synchronized 50 Hz slew
  // stepping
  TIMSK |= (1 << TOIE1);
}

void setPanPulse(uint16_t us) {
  if (us < SERVO_MIN_US)
    us = SERVO_MIN_US;
  if (us > SERVO_MAX_US)
    us = SERVO_MAX_US;
  uint8_t sreg = SREG;
  cli();
  pan_is_slewing = false;
  current_pan_us = us;
  target_pan_us = us;
  OCR1A = US_TO_TICKS(us);
  SREG = sreg;
}

void slewPanTo(uint16_t target_us) {
  if (target_us < SERVO_MIN_US)
    target_us = SERVO_MIN_US;
  if (target_us > SERVO_MAX_US)
    target_us = SERVO_MAX_US;
  uint8_t sreg = SREG;
  cli();
  target_pan_us = target_us;
  pan_is_slewing = (current_pan_us != target_pan_us);
  SREG = sreg;
}

uint16_t getPanPulse(void) {
  uint8_t sreg = SREG;
  cli();
  uint16_t val = current_pan_us;
  SREG = sreg;
  return val;
}

uint32_t getPir0Count(void) {
  uint8_t sreg = SREG;
  cli();
  uint32_t val = pir0_trigger_count;
  SREG = sreg;
  return val;
}

uint32_t getPir1Count(void) {
  uint8_t sreg = SREG;
  cli();
  uint32_t val = pir1_trigger_count;
  SREG = sreg;
  return val;
}

/* ---------- I2C Bus Recovery Sequence ---------- */
void i2c_bus_recovery(void) {
  PORTC &= ~((1 << PC0) |
             (1 << PC1)); // Drive 0V when output, floating High-Z when input
  DDRC &= ~((1 << PC0) | (1 << PC1)); // High-Z release
  _delay_us(10);

  for (uint8_t i = 0; i < 9; i++) {
    DDRC |= (1 << PC0); // SCL forced LOW (0V)
    _delay_us(10);
    DDRC &= ~(1 << PC0); // SCL released HIGH (Float / Pull-up)
    _delay_us(10);
  }

  // Generate valid STOP condition: SCL LOW -> SDA LOW -> SCL HIGH -> SDA HIGH
  DDRC |= (1 << PC0); // SCL forced LOW
  _delay_us(10);
  DDRC |= (1 << PC1); // SDA forced LOW
  _delay_us(10);
  DDRC &= ~(1 << PC0); // SCL released HIGH
  _delay_us(10);
  DDRC &= ~(1 << PC1); // SDA released HIGH
  _delay_us(10);

  // Enable internal pull-ups on PC0 (SCL) and PC1 (SDA) to prevent I2C bus
  // floating/timeouts
  PORTC |= (1 << PC0) | (1 << PC1);
}

/* ---------- I2C Bus Scanner ---------- */
void scanI2CBus(void) {
  uint8_t devices_found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
#if defined(WIRE_TIMEOUT)
    if (Wire.getWireTimeoutFlag()) {
      Wire.clearWireTimeoutFlag();
      continue;
    }
#endif
    if (err == 0) {
      Serial.print(F("   -> Found device at address 0x"));
      if (addr < 16)
        Serial.print(F("0"));
      Serial.print(addr, HEX);
      if (addr == 0x29)
        Serial.print(F(" (VL53L0X Laser ToF)"));
      if (addr == 0x5A)
        Serial.print(F(" (GY-906 MLX90614 Thermal)"));
      Serial.println();
      devices_found++;
    }
  }
  if (devices_found == 0) {
    Serial.println(F("   -> [NONE] No I2C devices detected!"));
  }
  Serial.flush();
}

/* ---------- Read MLX90614 SMBus Register ---------- */
float readMLX90614TempC(uint8_t reg) {
  if (!mlx90614_online)
    return -999.0f;

  static uint8_t mlx_err_count = 0;
  static uint8_t mlx_req_err = 0;

  Wire.beginTransmission(MLX90614_I2CADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
#if defined(WIRE_TIMEOUT)
    if (Wire.getWireTimeoutFlag())
      Wire.clearWireTimeoutFlag();
#endif
    if (++mlx_err_count >= 3) {
      mlx90614_online = false;
    }
    return -999.0f;
  }

  if (Wire.requestFrom((uint8_t)MLX90614_I2CADDR, (uint8_t)3) != 3) {
#if defined(WIRE_TIMEOUT)
    if (Wire.getWireTimeoutFlag())
      Wire.clearWireTimeoutFlag();
#endif
    if (++mlx_req_err >= 3) {
      mlx90614_online = false;
    }
    return -999.0f;
  }

  uint8_t lsb = Wire.read();
  uint8_t msb = Wire.read();
  uint8_t pec = Wire.read();
  (void)pec;

  // Reset static error counters upon successful read transaction
  mlx_err_count = 0;
  mlx_req_err = 0;

  if (msb & 0x80)
    return -999.0f;

  uint16_t tempRaw = ((uint16_t)msb << 8) | lsb;
  float tempK = tempRaw * 0.02f;
  return tempK - 273.15f;
}

/* ---------- Helper: Formatted Timestamp Output [MM:SS.mmm] ---------- */
void formatTime(uint32_t total_ms) {
  uint32_t total_sec = total_ms / 1000UL;
  uint32_t ms_remainder = total_ms % 1000UL;
  uint32_t minutes = total_sec / 60UL;
  uint32_t seconds = total_sec % 60UL;

  Serial.print(F("["));
  if (minutes < 10)
    Serial.print(F("0"));
  Serial.print(minutes);
  Serial.print(F(":"));
  if (seconds < 10)
    Serial.print(F("0"));
  Serial.print(seconds);
  Serial.print(F("."));
  if (ms_remainder < 100)
    Serial.print(F("0"));
  if (ms_remainder < 10)
    Serial.print(F("0"));
  Serial.print(ms_remainder);
  Serial.print(F("]"));
}

/* ---------- Interactive Serial Commands ---------- */
void processSerialCommands(void) {
  char cmd = Serial.read();
  if (cmd < 32 || cmd > 126)
    return;

  Serial.println();
  Serial.print(F(">> Command: '"));
  Serial.print(cmd);
  Serial.println(F("'"));

  switch (cmd) {
  case 'h':
  case 'H':
  case '?':
    printHelp();
    break;

  case 's':
  case 'S':
  case 'b': // Tolerant alias for UART framing / clock shift
  case 'B':
    printDetailedStatus();
    break;

  case '1':
  case 'l':
  case 'L':
    triggerSectorSearch(SECTOR_LEFT);
    break;

  case '2':
  case 'r':
  case 'R':
    triggerSectorSearch(SECTOR_RIGHT);
    break;

  case 'u':
  case 'U':
  case 'c':
  case 'C':
  case '0':
  case ' ': // Spacebar to clear/center
    turret_state = TURRET_STANDBY;
    active_sector = SECTOR_NONE;
    slewPanTo(PAN_CENTER_US);
    Serial.println(F(">> [TARGET LOCK RELEASED] Turret slewing gently to "
                     "Center (1500 us) & Standby."));
    break;

  case 'i':
    GICR &= ~((1 << INT0) | (1 << INT1));
    pir_active_high = !pir_active_high;
    if (!pir_active_high) {
      // Active-LOW (0V = Motion): Enable pull-ups, set MCUCR for FALLING edge
      // trigger
      PORTD |= (1 << PIR0_BIT) | (1 << PIR1_BIT);
      MCUCR = (MCUCR & ~((1 << ISC00) | (1 << ISC10))) |
              ((1 << ISC01) | (1 << ISC11));
      Serial.println(
          F(">> [PIR POLARITY] Set to ACTIVE-LOW (Falling edge, Pull-ups ON)"));
    } else {
      // Active-HIGH (5V = Motion): Disable pull-ups, set MCUCR for RISING edge
      // trigger
      PORTD &= ~((1 << PIR0_BIT) | (1 << PIR1_BIT));
      MCUCR |= (1 << ISC01) | (1 << ISC00) | (1 << ISC11) | (1 << ISC10);
      Serial.println(F(
          ">> [PIR POLARITY] Set to ACTIVE-HIGH (Rising edge, Pull-ups OFF)"));
    }
    GIFR = (1 << INTF0) | (1 << INTF1);
    GICR |= (1 << INT0) | (1 << INT1);
    break;

  case 'k':
  case 'K':
    lock_logic_enabled = !lock_logic_enabled;
    if (!lock_logic_enabled && turret_state == TURRET_TARGET_LOCKED) {
      turret_state = TURRET_STANDBY;
      active_sector = SECTOR_NONE;
      slewPanTo(PAN_CENTER_US);
    }
    Serial.print(F(">> [TARGET LOCK LOGIC] "));
    Serial.println(lock_logic_enabled
                       ? F("ENABLED (Requires Laser + Elevated Thermal)")
                       : F("DISABLED (Pure 2-round-trip sweep)"));
    break;

  case 'p':
  case 'P':
    turret_motion_paused = !turret_motion_paused;
    Serial.print(F(">> [MOTION] "));
    Serial.println(turret_motion_paused ? F("PAUSED") : F("RESUMED"));
    break;

  case '[':
    turret_state = TURRET_STANDBY;
    slewPanTo(getPanPulse() > (50 + SERVO_MIN_US) ? (getPanPulse() - 50)
                                                  : SERVO_MIN_US);
    Serial.print(F(">> Pan Nudge Left to: "));
    Serial.print(target_pan_us);
    Serial.println(F(" us"));
    break;

  case ']':
    turret_state = TURRET_STANDBY;
    slewPanTo(getPanPulse() + 50 < SERVO_MAX_US ? (getPanPulse() + 50)
                                                : SERVO_MAX_US);
    Serial.print(F(">> Pan Nudge Right to: "));
    Serial.print(target_pan_us);
    Serial.println(F(" us"));
    break;

  case 'v':
  case 'V': {
    if (Serial.available() > 0 && isdigit(Serial.peek())) {
      int new_speed = Serial.parseInt();
      if (new_speed >= 1 && new_speed <= 10) {
        step_us_per_tick = new_speed;
        Serial.print(F(">> Sweep Speed set to: "));
        Serial.print(step_us_per_tick);
        Serial.print(F(" us/tick (~"));
        Serial.print(step_us_per_tick * 50);
        Serial.println(F(" us/s)"));
      } else {
        Serial.println(F(">> Invalid speed range (1-10 allowed)."));
      }
    } else {
      Serial.print(F(">> Current Speed: "));
      Serial.print(step_us_per_tick);
      Serial.println(F(" us/tick. Send 'v<1-10>' (e.g. 'v3') to set."));
    }
    break;
  }

  case 'f':
  case 'F':
    step_us_per_tick = 6;
    Serial.println(F(">> [FAST PRESET] Speed set to 6 us/tick (~300 us/s)"));
    break;

  case 'm':
  case 'M':
    step_us_per_tick = 4;
    Serial.println(F(">> [MEDIUM PRESET] Speed set to 4 us/tick (~200 us/s)"));
    break;

  case '+':
  case '=':
    if (step_us_per_tick < 10)
      step_us_per_tick++;
    Serial.print(F(">> Sweep Speed INCREASED: "));
    Serial.print(step_us_per_tick);
    Serial.print(F(" us/tick (~"));
    Serial.print(step_us_per_tick * 50);
    Serial.println(F(" us/s)"));
    break;

  case '-':
  case '_':
    if (step_us_per_tick > 1)
      step_us_per_tick--;
    Serial.print(F(">> Sweep Speed DECREASED: "));
    Serial.print(step_us_per_tick);
    Serial.print(F(" us/tick (~"));
    Serial.print(step_us_per_tick * 50);
    Serial.println(F(" us/s)"));
    break;

  case 'w':
  case 'W': {
    static bool pwm_inverted = false;
    pwm_inverted = !pwm_inverted;
    if (pwm_inverted) {
      TCCR1A = (1 << COM1A1) | (1 << COM1A0) |
               (1 << WGM11); // Inverting PWM (Active-LOW)
      Serial.println(
          F(">> [PWM POLARITY] Set to INVERTING (Active-LOW pulse width)"));
    } else {
      TCCR1A = (1 << COM1A1) | (1 << WGM11); // Non-Inverting PWM (Active-HIGH)
      Serial.println(F(
          ">> [PWM POLARITY] Set to NON-INVERTING (Active-HIGH pulse width)"));
    }
    break;
  }

  case 'x':
  case 'X':
    resetStatistics();
    break;

  default:
    Serial.println(F(">> Unknown command. Send 'h' for menu."));
    break;
  }
}

/* ---------- Help Menu ---------- */
void printHelp(void) {
  Serial.println(
      F("========================= COMMAND MENU ========================="));
  Serial.println(F("  'h' or '?' : Display this interactive help menu"));
  Serial.println(
      F("  's'        : Detailed diagnostic snapshot (Sensors & Turret)"));
  Serial.println(F("  '1' or 'l' : Manually trigger LEFT 70° double sweep"));
  Serial.println(F("  '2' or 'r' : Manually trigger RIGHT 70° double sweep"));
  Serial.println(F("  'p'        : Pause / Resume motion"));
  Serial.println(
      F("  'k'        : Toggle Target Lock ON / OFF (Laser + Thermal)"));
  Serial.println(
      F("  'u' or 'c' : RELEASE TARGET LOCK -> Center (1500 us) & Standby"));
  Serial.println(F(
      "  'i'        : Toggle PIR active polarity (Active-HIGH vs Active-LOW)"));
  Serial.println(F(
      "  'w'        : Toggle PWM output polarity (Active-HIGH vs Active-LOW)"));
  Serial.println(F("  '[' / ']'  : Nudge Pan Left / Right (-50 / +50 us)"));
  Serial.println(F("  '+' / '-'  : Speed Up / Down (+1 / -1 us/tick)"));
  Serial.println(F("  'v<1-10>'  : Set exact sweep speed in us/tick (e.g. 'v3' "
                   "= 150 us/s)"));
  Serial.println(
      F("  'm' / 'f'  : Medium (200 us/s) / Fast (300 us/s) preset"));
  Serial.println(F("  'x'        : Reset trigger counters"));
  Serial.println(
      F("================================================================"));
}

/* ---------- Diagnostic Status Report ---------- */
void printDetailedStatus(void) {
  uint32_t now = millis();
  bool p0 = (PIND & (1 << PIR0_BIT)) ? true : false;
  bool p1 = (PIND & (1 << PIR1_BIT)) ? true : false;

  Serial.println(
      F("-------------------- DIAGNOSTIC SNAPSHOT --------------------"));
  Serial.print(F(" Uptime       : "));
  formatTime(now);
  Serial.println();
  Serial.print(F(" Turret State : "));
  if (turret_state == TURRET_STANDBY)
    Serial.println(F("QUIET STANDBY (Parked at Center)"));
  else if (turret_state == TURRET_SEARCHING) {
    Serial.print(F("SEARCHING "));
    Serial.print(active_sector == SECTOR_LEFT ? F("LEFT") : F("RIGHT"));
    Serial.print(F(" SECTOR (Pass "));
    Serial.print(sweep_passes_completed);
    Serial.println(F("/2)"));
  } else {
    Serial.print(F("TARGET LOCKED at "));
    Serial.print(getPanPulse());
    Serial.println(F(" us (Alert Active)"));
  }

  Serial.print(F(" PIR 0 (Left) : "));
  Serial.print(p0 ? F("ACTIVE (HIGH)") : F("IDLE (LOW)"));
  Serial.print(F(" | Triggers: #"));
  Serial.println(getPir0Count());

  Serial.print(F(" PIR 1 (Right): "));
  Serial.print(p1 ? F("ACTIVE (HIGH)") : F("IDLE (LOW)"));
  Serial.print(F(" | Triggers: #"));
  Serial.println(getPir1Count());

  Serial.print(F(" Laser ToF    : "));
  if (vl53l0x_online) {
    Serial.print(F("ONLINE | "));
    Serial.print((uint16_t)filtered_dist_mm);
    Serial.println(F(" mm"));
  } else {
    Serial.println(F("OFFLINE"));
  }

  Serial.print(F(" Thermal IR   : "));
  if (mlx90614_online) {
    Serial.print(F("ONLINE | Amb: "));
    Serial.print(current_amb_c, 1);
    Serial.print(F(" C | Obj: "));
    Serial.print(filtered_obj_c, 1);
    Serial.println(F(" C"));
  } else {
    Serial.println(F("OFFLINE"));
  }

  Serial.print(F(" Pan Limits   : "));
  Serial.print(SERVO_MIN_US);
  Serial.print(F(" us (Safe Min) <-> "));
  Serial.print(PAN_CENTER_US);
  Serial.print(F(" us (Center) <-> "));
  Serial.print(SERVO_MAX_US);
  Serial.println(F(" us (Safe Max)"));
  Serial.print(F(" Slew Speed   : "));
  Serial.print(step_us_per_tick);
  Serial.print(F(" us/tick (~"));
  Serial.print(step_us_per_tick * 50);
  Serial.println(F(" us/s)"));
  Serial.print(F(" Lock Logic   : "));
  Serial.println(lock_logic_enabled ? F("ENABLED") : F("DISABLED"));
  Serial.println(
      F("-------------------------------------------------------------"));
}

/* ---------- Reset Counters ---------- */
void resetStatistics(void) {
  uint8_t sreg = SREG;
  cli();
  pir0_trigger_count = 0;
  pir1_trigger_count = 0;
  pir0_last_trigger_ms = 0;
  pir1_last_trigger_ms = 0;
  SREG = sreg;
  laser_first_read = true;
  thermal_first_read = true;
  Serial.println(F(">> [RESET] Statistics and filters reset."));
}
