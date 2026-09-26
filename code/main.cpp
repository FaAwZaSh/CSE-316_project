/*
 * =====================================================================================
 *  PROJECT VANGUARD: Dual PIR Surveillance Turret with Laser & Thermal Target Lock
 * =====================================================================================
 *  MCU:               ATmega32A (16.0 MHz External Crystal Oscillator)
 *  Baud Rate:         9600 Baud (8-N-1) on PD0(RXD)/PD1(TXD)
 *  I2C Bus:           SCL = Pin 22 (PC0), SDA = Pin 23 (PC1) @ 100 kHz
 * =====================================================================================
 *  BEHAVIOR & SPECIFICATION:
 *  1. Standby:
 *     - Turret parked and holding at Center (90° / 1500 µs).
 *     - Alert LED (Pin 33 / PA7) is OFF.
 *     - Sensor telemetry is passively read, but TARGET LOCK IS INACTIVE.
 *  2. PIR Detection & Sector Sweep:
 *     - PIR 1 (Left  / Pin 16 / PD2 / INT0): Sweeps 90° -> 0° -> 90°.
 *     - PIR 2 (Right / Pin 17 / PD3 / INT1): Sweeps 90° -> 180° -> 90°.
 *     - During an active sweep, all subsequent PIR interrupts are locked out.
 *  3. Continuous Sensor Acquisition during Sweep:
 *     - Distance (VL53L0X Laser ToF) and Temperature (GY-906 MLX90614 Thermal IR)
 *       are read continuously across the sweep trajectory (every 2 degrees).
 *  4. Real Turret Target Lock:
 *     - Triggered if:
 *         * Laser Distance <= 200 mm (LOCK_DISTANCE_MM), OR
 *         * Thermal Object Temp >= 32.0 °C (HEAT_THRESHOLD_C), OR
 *         * Thermal Temp Delta >= 2.0 °C above Ambient (DELTA_THRESHOLD_C).
 *     - Motor STOPS IMMEDIATELY and STAYS at that exact position/angle!
 *     - Alert LED (Pin 33 / PA7) turns ON.
 *     - Full telemetry report printed to Serial.
 *  5. Keypress Unlock & Return to Center:
 *     - Turret holds locked position indefinitely until ANY key is pressed over Serial.
 *     - Upon keypress: Alert LED turns OFF, motor slews smoothly back to Center (90°),
 *       stale interrupt flags are cleared, and system re-arms for future PIR triggers.
 * =====================================================================================
 *  ATMEGA32A CONNECTION CONFIGURATION (DIP-40 PINOUT):
 *  ---------------------------------------------------
 *                     +---[ \_/ ]---+
 *   (Heartbeat) PB0 1 |             | 40  PA0 (Laser XSHUT - Reset / Enable)
 *               PB1 2 |             | 39  PA1
 *               PB2 3 |             | 38  PA2
 *               PB3 4 |             | 37  PA3
 *               PB4 5 |   ATmega32A | 36  PA4
 *               PB5 6 |    DIP-40   | 35  PA5
 *               PB6 7 |             | 34  PA6
 *               PB7 8 |             | 33  PA7 (Target Lock Alert LED)
 *            !RESET 9 |             | 32  AREF
 *               VCC 10|             | 31  GND
 *               GND 11|             | 30  AVCC
 *             XTAL2 12|             | 29  PC7
 *             XTAL1 13|             | 28  PC6
 *        (RXD)  PD0 14|             | 27  PC5
 *        (TXD)  PD1 15|             | 26  PC4
 *  (PIR 1/INT0) PD2 16|             | 25  PC3
 *  (PIR 2/INT1) PD3 17|             | 24  PC2
 *  (Servo OC1B) PD4 18|             | 23  PC1 (SDA - Shared Laser & Thermal)
 *  (Servo OC1A) PD5 19|             | 22  PC0 (SCL - Shared Laser & Thermal)
 *               PD6 20|             | 21  PD7
 *                     +-------------+
 * =====================================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Adafruit_VL53L0X.h"

// ---------- Hardware Pin Definitions ----------
#define PIR1_BIT          PD2   // Pin 16: INT0 (PIR 1 - Left Sector)
#define PIR2_BIT          PD3   // Pin 17: INT1 (PIR 2 - Right Sector)
#define SERVO_PIN_A       PD5   // Pin 19: Timer1 OC1A (Pan Servo PWM)
#define SERVO_PIN_B       PD4   // Pin 18: Timer1 OC1B (Aux Servo PWM)
#define LED_HEARTBEAT     PB0   // Pin 1: Heartbeat Diagnostic LED
#define ALERT_LED_BIT     PA7   // Pin 33: Target Lock / Alert LED
#define LASER_XSHUT_BIT   PA0   // Pin 40: VL53L0X Hardware Reset / Enable

// ---------- I2C Device Addresses & Registers ----------
#define VL53L0X_I2CADDR   0x29  // Laser Distance Sensor
#define MLX90614_I2CADDR  0x5A  // Thermal IR Sensor
#define MLX90614_TA       0x06  // Ambient Temperature Register
#define MLX90614_TOBJ1    0x07  // Object 1 Temperature Register

// ---------- Target Lock Thresholds (from laser_thermal.cpp) ----------
#define LOCK_DISTANCE_MM  200   // Proximity target close threshold: <= 200 mm (~20 cm)
#define HEAT_THRESHOLD_C  32.0f // Elevated object temp threshold in Celsius
#define DELTA_THRESHOLD_C 2.0f  // Object temperature delta above Ambient

// ---------- MG995 Servo Pulse Limits (Microseconds) ----------
#define PULSE_MIN_US      500   //   0 degrees (~0.5 ms)
#define PULSE_CENTER_US   1500  //  90 degrees (~1.5 ms)
#define PULSE_MAX_US      2500  // 180 degrees (~2.5 ms)
#define STEP_DELAY_MS     12    // ms delay between 1-degree steps

// ---------- State Tracking & Filter Variables ----------
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool vl53l0x_online  = false;
bool mlx90614_online = false;

float filtered_obj_c  = 0.0f;
bool  first_temp_read = true;
float last_amb_c      = 0.0f;
uint16_t last_dist_mm = 0;

volatile bool sweeping = false; // True ONLY during an active sector sweep
volatile bool pir1_triggered = false;
volatile bool pir2_triggered = false;
volatile uint32_t pir1_trigger_count = 0;
volatile uint32_t pir2_trigger_count = 0;
volatile uint32_t pir1_last_ms = 0;
volatile uint32_t pir2_last_ms = 0;
#define PIR_DEBOUNCE_MS 250UL

uint32_t last_status_time = 0;

// ---------- Forward Declarations ----------
void initTimer1PWM();
void setServoPulse(uint16_t pulse_us);
void setServoAngle(uint8_t angle);
void i2c_bus_recovery();
float readMLX90614TempC(uint8_t reg);
void readSensorsPassive();
bool checkTargetLock(int current_angle, const char* sweep_dir);
void waitForUnlock(int locked_angle);
void sweepLeft();
void sweepRight();
void clearPendingInterrupts();

// =====================================================================================
//  External Interrupt Service Routines (INT0 = PIR 1, INT1 = PIR 2)
// =====================================================================================
ISR(INT0_vect) {
  uint32_t ms = millis();
  if (ms - pir1_last_ms >= PIR_DEBOUNCE_MS) {
    pir1_trigger_count++;
    pir1_last_ms = ms;
    if (!sweeping) {
      pir1_triggered = true;
    }
  }
}

ISR(INT1_vect) {
  uint32_t ms = millis();
  if (ms - pir2_last_ms >= PIR_DEBOUNCE_MS) {
    pir2_trigger_count++;
    pir2_last_ms = ms;
    if (!sweeping) {
      pir2_triggered = true;
    }
  }
}

// =====================================================================================
//  I2C Bus Recovery: Clears any hung slave from the bus (from laser_thermal.cpp)
// =====================================================================================
void i2c_bus_recovery(void) {
  DDRC |= (1 << PC0) | (1 << PC1);
  PORTC |= (1 << PC0) | (1 << PC1);
  _delay_us(10);

  for (uint8_t i = 0; i < 9; i++) {
    PORTC &= ~(1 << PC0);
    _delay_us(10);
    PORTC |= (1 << PC0);
    _delay_us(10);
  }

  // STOP condition
  PORTC &= ~(1 << PC1); _delay_us(10);
  PORTC |= (1 << PC0);  _delay_us(10);
  PORTC |= (1 << PC1);  _delay_us(10);

  DDRC &= ~((1 << PC0) | (1 << PC1));
  PORTC &= ~((1 << PC0) | (1 << PC1));
}

// =====================================================================================
//  Read raw 16-bit temperature from MLX90614 RAM over SMBus (from laser_thermal.cpp)
// =====================================================================================
float readMLX90614TempC(uint8_t reg) {
  Wire.beginTransmission(MLX90614_I2CADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { // Repeated START
    return -999.0f;
  }

  if (Wire.requestFrom((uint8_t)MLX90614_I2CADDR, (uint8_t)3) != 3) {
    return -999.0f;
  }

  uint8_t lsb = Wire.read();
  uint8_t msb = Wire.read();
  uint8_t pec = Wire.read();
  (void)pec;

  if (msb & 0x80) {
    return -999.0f; // Measurement error bit
  }

  uint16_t tempRaw = ((uint16_t)msb << 8) | lsb;
  return ((float)tempRaw * 0.02f) - 273.15f;
}

// =====================================================================================
//  Helper: Clear Pending Interrupts & Stale Triggers
// =====================================================================================
void clearPendingInterrupts() {
  uint8_t sreg = SREG;
  cli();
  GIFR = (1 << INTF0) | (1 << INTF1); // Clear hardware interrupt flags
  pir1_triggered = false;
  pir2_triggered = false;
  SREG = sreg;
}

// =====================================================================================
//  Setup Routine
// =====================================================================================
void setup() {
  // 1. Initialize UART at 9600 Baud with internal pull-up on RXD (PD0)
  PORTD |= (1 << PD0);
  Serial.begin(9600);
  while (!Serial && millis() < 1200);

  Serial.println();
  Serial.println(F("=================================================================="));
  Serial.println(F("   PROJECT VANGUARD: SURVEILLANCE TURRET SYSTEM (REAL TURRET)    "));
  Serial.println(F("=================================================================="));
  Serial.println(F(" MCU       : ATmega32A @ 16.0 MHz External Crystal Oscillator"));
  Serial.println(F(" Baud Rate : 9600 Baud (8-N-1) on PD0(RXD) / PD1(TXD)"));
  Serial.println(F(" PWM Timer : Timer1 50 Hz Hardware Fast PWM on Pin 19 (PD5 / OC1A)"));
  Serial.println(F(" PIR 1     : Pin 16 (PD2 / INT0) -> Left Sector Sweep (90° -> 0° -> 90°)"));
  Serial.println(F(" PIR 2     : Pin 17 (PD3 / INT1) -> Right Sector Sweep (90° -> 180° -> 90°)"));
  Serial.println(F(" Alert LED : Pin 33 (PA7) [Active HIGH Target Lock Indicator]"));
  Serial.println(F(" Laser ToF : VL53L0X (0x29) [XSHUT Pin 40 (PA0)] <= 200 mm Lock Limit"));
  Serial.println(F(" Therm IR  : GY-906 MLX90614 (0x5A) >= 32.0 °C / Delta >= 2.0 °C"));
  Serial.println(F(" Turret Op : Reads distance & thermal continuously during PIR motor sweep."));
  Serial.println(F("             If locked: holds angle until ANY key pressed -> centers."));
  Serial.println(F("=================================================================="));
  Serial.println();

  // 2. Configure PIR pins as inputs (no pull-up, PIR sensor drives active line)
  DDRD  &= ~((1 << PIR1_BIT) | (1 << PIR2_BIT));
  PORTD &= ~((1 << PIR1_BIT) | (1 << PIR2_BIT));

  // 3. Configure LEDs and XSHUT
  DDRB |= (1 << LED_HEARTBEAT);
  PORTB |= (1 << LED_HEARTBEAT); // Heartbeat ON during boot

  DDRA |= (1 << ALERT_LED_BIT) | (1 << LASER_XSHUT_BIT);
  PORTA &= ~(1 << ALERT_LED_BIT); // Alert LED OFF

  // 4. Hardware reset VL53L0X via XSHUT (PA0) with 250ms MCU boot delay
  Serial.print(F("[INIT] 1. Resetting VL53L0X Laser via XSHUT (PA0)... "));
  PORTA &= ~(1 << LASER_XSHUT_BIT);
  delay(50);
  PORTA |= (1 << LASER_XSHUT_BIT);
  delay(250);
  Serial.println(F("Done."));

  // 5. Recover I2C Bus & Initialize Wire at 100 kHz
  Serial.print(F("[INIT] 2. Performing I2C Bus Clock Recovery & Starting Wire... "));
  i2c_bus_recovery();
  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_TIMEOUT)
  Wire.setWireTimeout(0); // Disable timeout to prevent SMBus clock stretching cutoff
#endif
  PORTC |= (1 << PC0) | (1 << PC1); // Enable internal pull-ups
  Serial.println(F("Done."));

  // 6. Initialize VL53L0X Laser Distance Sensor
  Serial.print(F("[INIT] 3. Initializing VL53L0X Laser Sensor at 0x29... "));
  if (lox.begin(VL53L0X_I2CADDR, false, &Wire)) {
    vl53l0x_online = true;
    lox.setMeasurementTimingBudgetMicroSeconds(33000); // 33ms high-speed continuous budget
    Serial.println(F("[ONLINE / READY]"));
  } else {
    vl53l0x_online = false;
    Serial.println(F("[OFFLINE / CHECK WIRING]"));
  }

  // 7. Probe GY-906 (MLX90614) Thermal Sensor
  Serial.print(F("[INIT] 4. Probing GY-906 (MLX90614) Thermal Sensor at 0x5A... "));
  float test_amb = readMLX90614TempC(MLX90614_TA);
  if (test_amb > -100.0f) {
    mlx90614_online = true;
    Serial.print(F("[ONLINE / READY] (Ambient: "));
    Serial.print(test_amb, 1);
    Serial.println(F(" °C)"));
  } else {
    mlx90614_online = false;
    Serial.println(F("[OFFLINE / CHECK WIRING]"));
  }

  // 8. Initialize Timer1 50 Hz Hardware Fast PWM for Pan Servo
  Serial.print(F("[INIT] 5. Initializing Timer1 50 Hz Fast PWM on PD5 (OC1A)... "));
  initTimer1PWM();
  Serial.println(F("Done."));

  // 9. Park Servo at Center (90 deg)
  setServoAngle(90);
  Serial.println(F("[INIT] 6. Pan Servo parked at Center (90° / 1500 µs)."));
  delay(1000);

  // 10. Configure Hardware Interrupts INT0 & INT1 for RISING Edge
  Serial.print(F("[INIT] 7. Enabling PIR External Interrupts (INT0, INT1)... "));
  MCUCR |= (1 << ISC01) | (1 << ISC00) | (1 << ISC11) | (1 << ISC10); // Rising edge
  clearPendingInterrupts();
  GICR |= (1 << INT0) | (1 << INT1); // Enable INT0 and INT1
  sei(); // Enable global interrupts
  Serial.println(F("Done."));

  PORTB &= ~(1 << LED_HEARTBEAT); // Turn OFF heartbeat LED
  Serial.println();
  Serial.println(F("=================================================================="));
  Serial.println(F(" Status: SYSTEM ARMED IN QUIET STANDBY.                           "));
  Serial.println(F(" Pan   : Solidly parked at Center (90°).                          "));
  Serial.println(F(" Sensor: Telemetry passive in Standby (No false locks).            "));
  Serial.println(F(" Ready : Listening for PIR motion triggers...                     "));
  Serial.println(F(" Keys  : '1'=Manual Left Sweep | '2'=Manual Right Sweep | 'c'=Center"));
  Serial.println(F("=================================================================="));
  Serial.println();
}

// =====================================================================================
//  Main Loop
// =====================================================================================
void loop() {
  // Toggle Heartbeat LED
  PORTB ^= (1 << LED_HEARTBEAT);

  // --- 1. Check for Manual Commands over Serial Monitor ---
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '1' || cmd == 'l' || cmd == 'L') {
      Serial.println(F("\r\n>> [MANUAL] Left Sector Sweep initiated..."));
      sweeping = true;
      sweepLeft();
      clearPendingInterrupts();
      sweeping = false;
      Serial.println(F("[READY] Re-armed. Listening for PIR triggers...\r\n"));
      return;
    } else if (cmd == '2' || cmd == 'r' || cmd == 'R') {
      Serial.println(F("\r\n>> [MANUAL] Right Sector Sweep initiated..."));
      sweeping = true;
      sweepRight();
      clearPendingInterrupts();
      sweeping = false;
      Serial.println(F("[READY] Re-armed. Listening for PIR triggers...\r\n"));
      return;
    } else if (cmd == 'c' || cmd == 'C' || cmd == '0') {
      setServoAngle(90);
      PORTA &= ~(1 << ALERT_LED_BIT);
      clearPendingInterrupts();
      Serial.println(F(">> [MANUAL] Turret parked at Center (90°). Alert cleared."));
      return;
    }
  }

  // --- 2. Evaluate PIR Motion Triggers ---
  bool p1_active = pir1_triggered || ((PIND & (1 << PIR1_BIT)) != 0);
  bool p2_active = pir2_triggered || ((PIND & (1 << PIR2_BIT)) != 0);

  // Left Sector PIR Triggered
  if (p1_active && !sweeping) {
    clearPendingInterrupts();
    sweeping = true;

    Serial.println();
    Serial.println(F("=================================================================="));
    Serial.print(F(" >>> [PIR 1 TRIGGER: LEFT SECTOR] Motion Detected (Trigger #"));
    Serial.print(pir1_trigger_count);
    Serial.println(F(")"));
    Serial.println(F("     Initiating Left Sector Search Sweep (90° -> 0° -> 90°)..."));
    Serial.println(F("=================================================================="));

    sweepLeft();

    clearPendingInterrupts();
    sweeping = false;
    Serial.println(F("[READY] Re-armed. Waiting for next PIR trigger...\r\n"));
  }
  // Right Sector PIR Triggered
  else if (p2_active && !sweeping) {
    clearPendingInterrupts();
    sweeping = true;

    Serial.println();
    Serial.println(F("=================================================================="));
    Serial.print(F(" >>> [PIR 2 TRIGGER: RIGHT SECTOR] Motion Detected (Trigger #"));
    Serial.print(pir2_trigger_count);
    Serial.println(F(")"));
    Serial.println(F("     Initiating Right Sector Search Sweep (90° -> 180° -> 90°)..."));
    Serial.println(F("=================================================================="));

    sweepRight();

    clearPendingInterrupts();
    sweeping = false;
    Serial.println(F("[READY] Re-armed. Waiting for next PIR trigger...\r\n"));
  }

  // --- 3. Periodic Standby Status Telemetry (Every 3 seconds) ---
  // In Standby: reads sensor values passively for telemetry, BUT NEVER LOCKS!
  if (millis() - last_status_time >= 3000) {
    last_status_time = millis();
    readSensorsPassive();

    Serial.print(F("[STANDBY] Uptime: "));
    Serial.print(millis() / 1000);
    Serial.print(F("s | PIR1: "));
    Serial.print((PIND & (1 << PIR1_BIT)) ? F("ACTIVE") : F("IDLE"));
    Serial.print(F(" (#"));
    Serial.print(pir1_trigger_count);
    Serial.print(F(") | PIR2: "));
    Serial.print((PIND & (1 << PIR2_BIT)) ? F("ACTIVE") : F("IDLE"));
    Serial.print(F(" (#"));
    Serial.print(pir2_trigger_count);
    Serial.print(F(") | Laser: "));
    if (vl53l0x_online && last_dist_mm > 0) {
      Serial.print(last_dist_mm);
      Serial.print(F("mm"));
    } else if (!vl53l0x_online) {
      Serial.print(F("[OFFLINE]"));
    } else {
      Serial.print(F("[CLEAR]"));
    }
    Serial.print(F(" | Therm: Obj="));
    if (mlx90614_online && filtered_obj_c > -50.0f) {
      Serial.print(filtered_obj_c, 1);
      Serial.print(F("°C (Amb="));
      Serial.print(last_amb_c, 1);
      Serial.print(F("°C)"));
    } else if (!mlx90614_online) {
      Serial.print(F("[OFFLINE]"));
    } else {
      Serial.print(F("[ERR]"));
    }
    Serial.println(F(" | State: STANDBY (Armed)"));
  }

  delay(40); // Debounce loop delay
}

// =====================================================================================
//  Passive Sensor Reading (Used only for idle telemetry — NEVER triggers target lock)
// =====================================================================================
void readSensorsPassive() {
  if (vl53l0x_online) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter >= 15 && measure.RangeMilliMeter <= 2500) {
      last_dist_mm = measure.RangeMilliMeter;
    } else {
      last_dist_mm = 0;
    }
  }

  if (mlx90614_online) {
    float amb = readMLX90614TempC(MLX90614_TA);
    float obj = readMLX90614TempC(MLX90614_TOBJ1);
    if (amb > -40.0f && amb < 125.0f && obj > -40.0f && obj < 200.0f) {
      last_amb_c = amb;
      if (first_temp_read) {
        filtered_obj_c = obj;
        first_temp_read = false;
      } else {
        filtered_obj_c = (0.35f * obj) + (0.65f * filtered_obj_c);
      }
    }
  }
}

// =====================================================================================
//  Target Lock Evaluation (ONLY called during active motor sweep)
// =====================================================================================
bool checkTargetLock(int current_angle, const char* sweep_dir) {
  uint16_t dist_mm = 0;
  bool laser_valid = false;

  // 1. Read Laser Distance
  if (vl53l0x_online) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter >= 15 && measure.RangeMilliMeter <= 2500) {
      dist_mm = measure.RangeMilliMeter;
      last_dist_mm = dist_mm;
      laser_valid = true;
    }
  }

  // 2. Read Thermal Temperatures
  float amb_c = -999.0f;
  float obj_c = -999.0f;
  float delta_c = 0.0f;
  bool thermal_valid = false;

  if (mlx90614_online) {
    amb_c = readMLX90614TempC(MLX90614_TA);
    obj_c = readMLX90614TempC(MLX90614_TOBJ1);

    if (amb_c > -40.0f && amb_c < 125.0f && obj_c > -40.0f && obj_c < 200.0f) {
      thermal_valid = true;
      last_amb_c = amb_c;
      if (first_temp_read) {
        filtered_obj_c = obj_c;
        first_temp_read = false;
      } else {
        filtered_obj_c = (0.35f * obj_c) + (0.65f * filtered_obj_c);
      }
      delta_c = filtered_obj_c - amb_c;
    }
  }

  // 3. Evaluate Lock Conditions
  bool proximity_hit = laser_valid && (dist_mm <= LOCK_DISTANCE_MM);
  bool thermal_hit   = thermal_valid && ((filtered_obj_c >= HEAT_THRESHOLD_C) || (delta_c >= DELTA_THRESHOLD_C));

  // --- TARGET LOCKED! ---
  if (proximity_hit || thermal_hit) {
    Serial.println();
    Serial.println(F("******************************************************************"));
    Serial.println(F(" [!] >>> TARGET LOCKED DURING SECTOR SWEEP! <<<                   "));
    Serial.println(F("******************************************************************"));
    Serial.print(F(" Sector & Sweep : ")); Serial.println(sweep_dir);
    Serial.print(F(" Locked Angle   : ")); Serial.print(current_angle); Serial.println(F("°"));

    if (proximity_hit && thermal_hit) {
      Serial.println(F(" Lock Cause     : [DUAL LOCK: PROXIMITY + HEAT DETECTED!]"));
    } else if (proximity_hit) {
      Serial.print(F(" Lock Cause     : [PROXIMITY LOCK (Distance <= "));
      Serial.print(LOCK_DISTANCE_MM);
      Serial.println(F(" mm)]"));
    } else {
      Serial.print(F(" Lock Cause     : [HEAT SIGNATURE DETECTED (Obj >= "));
      Serial.print(HEAT_THRESHOLD_C, 1);
      Serial.println(F(" °C)]"));
    }

    Serial.print(F(" Laser Distance : "));
    if (laser_valid) {
      Serial.print(dist_mm); Serial.println(F(" mm"));
    } else {
      Serial.println(F("[CLEAR / NO REFLECTION]"));
    }

    Serial.print(F(" Thermal Data   : "));
    if (thermal_valid) {
      Serial.print(F("Obj=")); Serial.print(filtered_obj_c, 1);
      Serial.print(F(" °C | Amb=")); Serial.print(amb_c, 1);
      Serial.print(F(" °C (dt:+")); Serial.print(delta_c, 1); Serial.println(F(" °C)"));
    } else {
      Serial.println(F("[SENSOR ERROR / OFFLINE]"));
    }

    Serial.println(F(" Alert LED      : Pin 33 (PA7) -> ON"));
    Serial.println(F(" Turret Status  : STOPPED & HOLDING POSITION."));
    Serial.println(F(" Action         : Press ANY KEY in Serial Monitor to return to Center."));
    Serial.println(F("******************************************************************\r\n"));
    return true;
  }

  // Live telemetry line during active sweep
  Serial.print(F(" [SWEEPING] Angle: "));
  if (current_angle < 100) Serial.print(F(" "));
  if (current_angle < 10)  Serial.print(F(" "));
  Serial.print(current_angle);
  Serial.print(F("° | Laser: "));
  if (laser_valid) {
    Serial.print(dist_mm); Serial.print(F("mm"));
  } else {
    Serial.print(F("CLEAR"));
  }
  Serial.print(F(" | Therm: "));
  if (thermal_valid) {
    Serial.print(filtered_obj_c, 1); Serial.print(F("°C"));
  } else {
    Serial.print(F("--"));
  }
  Serial.println();

  return false;
}

// =====================================================================================
//  Wait For Keypress to Unlock & Return to Center ("like a real turret system")
// =====================================================================================
void waitForUnlock(int locked_angle) {
  // Alert LED ON to signal target lock state
  PORTA |= (1 << ALERT_LED_BIT);

  // Flush any stale characters before waiting
  while (Serial.available() > 0) {
    Serial.read();
  }

  uint32_t last_lock_report = millis();
  while (true) {
    // Check if user pressed ANY key in Serial Monitor
    if (Serial.available() > 0) {
      delay(50); // Allow multicharacter bursts like \r\n to arrive
      while (Serial.available() > 0) {
        Serial.read(); // Consume key
      }
      break; // Exit lock hold loop!
    }

    // Periodic reminder every 2 seconds
    if (millis() - last_lock_report >= 2000) {
      last_lock_report = millis();
      Serial.print(F(" [LOCKED HOLD] Angle: "));
      Serial.print(locked_angle);
      Serial.println(F("° | Alert LED: ON | Press ANY KEY to return to Center (90°)..."));
    }

    delay(30);
  }

  // Key pressed: Turn off Alert LED and slew smoothly to Center (90°)
  PORTA &= ~(1 << ALERT_LED_BIT); // Alert LED OFF
  Serial.println();
  Serial.print(F(">> [KEY RECEIVED] Target Lock Released! Moving smoothly from "));
  Serial.print(locked_angle);
  Serial.println(F("° to Center (90°)..."));

  if (locked_angle < 90) {
    for (int a = locked_angle; a <= 90; a++) {
      setServoAngle(a);
      delay(STEP_DELAY_MS);
    }
  } else if (locked_angle > 90) {
    for (int a = locked_angle; a >= 90; a--) {
      setServoAngle(a);
      delay(STEP_DELAY_MS);
    }
  } else {
    setServoAngle(90);
  }

  delay(400); // Settle cleanly at Center
  Serial.println(F(">> [PARKED] Back at Center 90°. System unlocked."));
}

// =====================================================================================
//  Sweep Left: 90° -> 0° -> 90° (Continuous sensor read every 2 degrees)
// =====================================================================================
void sweepLeft() {
  Serial.println(F("    [SWEEP] Phase 1: 90° -> 0°..."));
  PORTB |= (1 << LED_HEARTBEAT);

  // 1. Outward sweep: 90° -> 0°
  for (int angle = 90; angle >= 0; angle--) {
    setServoAngle(angle);
    delay(STEP_DELAY_MS);

    // Continuous distance and thermal sensor acquisition every 2 degrees
    if ((angle % 2) == 0) {
      if (checkTargetLock(angle, "LEFT SECTOR (90° -> 0°)")) {
        waitForUnlock(angle);
        PORTB &= ~(1 << LED_HEARTBEAT);
        return; // Target lock handled, unlocked, returned to center
      }
    }
  }

  delay(250); // Brief settle at 0°

  // 2. Return sweep: 0° -> 90°
  Serial.println(F("    [SWEEP] Phase 2: 0° -> 90° (returning)..."));
  for (int angle = 0; angle <= 90; angle++) {
    setServoAngle(angle);
    delay(STEP_DELAY_MS);

    // Continuous distance and thermal sensor acquisition every 2 degrees
    if ((angle % 2) == 0) {
      if (checkTargetLock(angle, "LEFT SECTOR (0° -> 90°)")) {
        waitForUnlock(angle);
        PORTB &= ~(1 << LED_HEARTBEAT);
        return; // Target lock handled, unlocked, returned to center
      }
    }
  }

  PORTB &= ~(1 << LED_HEARTBEAT);
  Serial.println(F("    [PARKED] Back at Center 90° (Left Sector Clear)."));
}

// =====================================================================================
//  Sweep Right: 90° -> 180° -> 90° (Continuous sensor read every 2 degrees)
// =====================================================================================
void sweepRight() {
  Serial.println(F("    [SWEEP] Phase 1: 90° -> 180°..."));
  PORTB |= (1 << LED_HEARTBEAT);

  // 1. Outward sweep: 90° -> 180°
  for (int angle = 90; angle <= 180; angle++) {
    setServoAngle(angle);
    delay(STEP_DELAY_MS);

    // Continuous distance and thermal sensor acquisition every 2 degrees
    if ((angle % 2) == 0) {
      if (checkTargetLock(angle, "RIGHT SECTOR (90° -> 180°)")) {
        waitForUnlock(angle);
        PORTB &= ~(1 << LED_HEARTBEAT);
        return; // Target lock handled, unlocked, returned to center
      }
    }
  }

  delay(250); // Brief settle at 180°

  // 2. Return sweep: 180° -> 90°
  Serial.println(F("    [SWEEP] Phase 2: 180° -> 90° (returning)..."));
  for (int angle = 180; angle >= 90; angle--) {
    setServoAngle(angle);
    delay(STEP_DELAY_MS);

    // Continuous distance and thermal sensor acquisition every 2 degrees
    if ((angle % 2) == 0) {
      if (checkTargetLock(angle, "RIGHT SECTOR (180° -> 90°)")) {
        waitForUnlock(angle);
        PORTB &= ~(1 << LED_HEARTBEAT);
        return; // Target lock handled, unlocked, returned to center
      }
    }
  }

  PORTB &= ~(1 << LED_HEARTBEAT);
  Serial.println(F("    [PARKED] Back at Center 90° (Right Sector Clear)."));
}

// =====================================================================================
//  Timer1 Fast PWM (Mode 14) — 50 Hz on PD5 (OC1A) + PD4 (OC1B)
// =====================================================================================
void initTimer1PWM() {
  DDRD |= (1 << SERVO_PIN_A) | (1 << SERVO_PIN_B);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // ICR1 = (16,000,000 / (8 * 50)) - 1 = 39999 -> 20.0 ms period (50 Hz)
  ICR1 = 39999;

  // Initial pulse: Center (1500 µs * 2 ticks/µs = 3000)
  OCR1A = 3000;
  OCR1B = 3000;

  // Mode 14: Fast PWM, ICR1 as TOP, non-inverting OC1A/OC1B
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler = 8
}

// =====================================================================================
//  Servo Pulse & Angle Helpers
// =====================================================================================
void setServoPulse(uint16_t pulse_us) {
  if (pulse_us < PULSE_MIN_US) pulse_us = PULSE_MIN_US;
  if (pulse_us > PULSE_MAX_US) pulse_us = PULSE_MAX_US;

  uint16_t ticks = pulse_us * 2;
  OCR1A = ticks;
  OCR1B = ticks;
}

void setServoAngle(uint8_t angle) {
  if (angle > 180) angle = 180;
  uint16_t pulse_us = PULSE_MIN_US + ((uint32_t)angle * (PULSE_MAX_US - PULSE_MIN_US)) / 180;
  setServoPulse(pulse_us);
}