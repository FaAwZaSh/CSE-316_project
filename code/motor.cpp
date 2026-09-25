/*
 * =====================================================================================
 *  MG995 Servo Motor Test - ATmega32 / ATmega32A (16.0 MHz External Crystal)
 * =====================================================================================
 *  Semi-Autonomous Dual-Sector Sweep Testbench
 *  Hardware Fast PWM (Timer1 Mode 14) @ 50 Hz (20 ms period)
 * =====================================================================================
 *  BEHAVIOR:
 *    1. Startup: Park at Center 90 deg.
 *    2. Initial Sweep: 90 deg -> 0 deg -> 90 deg, then wait at 90 deg.
 *    3. Keyboard Trigger: When any key is sent via Serial, sweeps the other side
 *       (90 deg -> 180 deg -> 90 deg) and waits at 90 deg.
 *    4. Subsequent keys alternate between Left and Right sectors (or type 'l'/'r').
 *
 *  WIRING GUIDE (ATmega32 DIP-40):
 *  ------------------------------------------------------------------------------------
 *  Servo Signal (Orange / Yellow) --> Pin 19 (PD5 / OC1A)  [Also mirrored on Pin 18 (PD4 / OC1B)]
 *  Servo Power  (Red)             --> External 5V - 6V DC Supply (MG995 draws up to 1.2A!)
 *                                     * CAUTION: DO NOT power the servo from USB or MCU VCC!
 *  Servo Ground (Brown / Black)   --> COMMON GROUND (Connect Servo GND, MCU GND, Power GND)
 *  Heartbeat LED (Optional)       --> Pin 1 (PB0) with 330-ohm resistor to GND
 *  Serial Monitor (PL2303 / TTL)  --> 9600 Baud (PD0 = RXD, PD1 = TXD)
 * =====================================================================================
 */

#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>

// ---------- Pin Definitions ----------
#define SERVO_PIN_A       PD5   // Pin 19 (Timer1 OC1A - Primary Servo Signal)
#define SERVO_PIN_B       PD4   // Pin 18 (Timer1 OC1B - Secondary Servo Signal)
#define LED_HEARTBEAT     PB0   // Pin 1  (Heartbeat LED indicator)

// ---------- Full 180-Degree Pulse Limits for MG995 (Microseconds) ----------
#define PULSE_MIN_US      500   //   0 degrees (~0.5 ms) - Full Left Limit
#define PULSE_CENTER_US   1500  //  90 degrees (~1.5 ms) - Center Neutral
#define PULSE_MAX_US      2500  // 180 degrees (~2.5 ms) - Full Right Limit

// Slew speed: ms delay between 1-degree steps
#define STEP_DELAY_MS     12

// ---------- State Tracking ----------
enum NextSweepSide {
  SIDE_RIGHT,
  SIDE_LEFT
};

NextSweepSide next_side = SIDE_RIGHT; // Left runs on startup, so next is Right

// ---------- Function Declarations ----------
void initTimer1PWM();
void setServoPulse(uint16_t pulse_us);
void setServoAngle(uint8_t angle);
void sweepLeft();
void sweepRight();

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 1500);

  Serial.println(F("\r\n=========================================="));
  Serial.println(F("  MG995 Servo Sector Sweep Testbench      "));
  Serial.println(F("=========================================="));
  Serial.println(F("MCU     : ATmega32 @ 16.0 MHz"));
  Serial.println(F("PWM     : Timer1 50 Hz Hardware Fast PWM"));
  Serial.println(F("Signal  : Pin 19 (PD5) & Pin 18 (PD4)"));
  Serial.println(F("Baud    : 9600"));
  Serial.println(F("------------------------------------------"));
  Serial.println(F("Usage:"));
  Serial.println(F("  - Startup runs: 90 -> 0 -> 90 deg and waits"));
  Serial.println(F("  - Send ANY key (or Enter) to sweep the other side"));
  Serial.println(F("  - Or send 'l' for Left (90->0->90) / 'r' for Right (90->180->90)"));
  Serial.println(F("==========================================\r\n"));

  // Heartbeat LED Output
  DDRB |= (1 << LED_HEARTBEAT);
  PORTB &= ~(1 << LED_HEARTBEAT);

  // Initialize Timer1 50 Hz Hardware PWM
  initTimer1PWM();

  // Park at Center 90 deg
  setServoAngle(90);
  Serial.println(F("[INIT] Servo parked at Center (90 deg / 1500 us)"));
  delay(1200);

  // Initial startup action: Sweep 90 -> 0 -> 90 and wait
  sweepLeft();
}

void loop() {
  // Wait for keyboard input from Serial Monitor
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // Ignore standalone newline / carriage return characters
    if (cmd == '\r' || cmd == '\n') {
      return;
    }

    // Flush any remaining characters in the serial buffer
    while (Serial.available()) {
      Serial.read();
    }

    // Execute requested or alternating side
    if (cmd == 'l' || cmd == 'L' || cmd == '1') {
      sweepLeft();
      next_side = SIDE_RIGHT;
    } else if (cmd == 'r' || cmd == 'R' || cmd == '2') {
      sweepRight();
      next_side = SIDE_LEFT;
    } else {
      // Any other key triggers the other side
      if (next_side == SIDE_RIGHT) {
        sweepRight();
        next_side = SIDE_LEFT;
      } else {
        sweepLeft();
        next_side = SIDE_RIGHT;
      }
    }
  }

  delay(20);
}

/**
 * Sweep Left: 90 deg -> 0 deg -> 90 deg, then wait at 90 deg
 */
void sweepLeft() {
  Serial.println(F("\r\n>>> [SWEEP LEFT] Moving 90 deg -> 0 deg..."));
  PORTB |= (1 << LED_HEARTBEAT);
  for (int angle = 90; angle >= 0; angle--) {
    setServoAngle(angle);
    delay(STEP_DELAY_MS);
  }

  delay(300); // 300 ms settle time at 0 deg

  Serial.println(F("<<< [SWEEP LEFT] Returning 0 deg -> 90 deg (Center)..."));
  for (int angle = 0; angle <= 90; angle++) {
    setServoAngle(angle);
    delay(STEP_DELAY_MS);
  }

  PORTB &= ~(1 << LED_HEARTBEAT);
  Serial.println(F("[WAIT] Parked at Center 90 deg."));
  Serial.println(F("       ==> Send any key to sweep Right (90 -> 180 -> 90 deg)...\r\n"));
}

/**
 * Sweep Right: 90 deg -> 180 deg -> 90 deg, then wait at 90 deg
 */
void sweepRight() {
  Serial.println(F("\r\n>>> [SWEEP RIGHT] Moving 90 deg -> 180 deg..."));
  PORTB |= (1 << LED_HEARTBEAT);
  for (int angle = 90; angle <= 180; angle++) {
    setServoAngle(angle);
    delay(STEP_DELAY_MS);
  }

  delay(300); // 300 ms settle time at 180 deg

  Serial.println(F("<<< [SWEEP RIGHT] Returning 180 deg -> 90 deg (Center)..."));
  for (int angle = 180; angle >= 90; angle--) {
    setServoAngle(angle);
    delay(STEP_DELAY_MS);
  }

  PORTB &= ~(1 << LED_HEARTBEAT);
  Serial.println(F("[WAIT] Parked at Center 90 deg."));
  Serial.println(F("       ==> Send any key to sweep Left (90 -> 0 -> 90 deg)...\r\n"));
}

/**
 * Configure Timer1 for 50 Hz Fast PWM (Mode 14) on PD5 (OC1A) and PD4 (OC1B)
 * Clock = 16 MHz, Prescaler = 8 -> 2 MHz Timer Clock (0.5 us per tick)
 * ICR1 = 39999 -> 40,000 ticks = 20 ms period (50 Hz)
 */
void initTimer1PWM() {
  // Configure PD5 (OC1A) and PD4 (OC1B) as OUTPUT
  DDRD |= (1 << SERVO_PIN_A) | (1 << SERVO_PIN_B);

  // Clear Timer1 Control Registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // ICR1 defines the PWM period: (16,000,000 / (8 * 50)) - 1 = 39999 (20 ms / 50 Hz)
  ICR1 = 39999;

  // Set initial pulse to Center (1500 us * 2 ticks/us = 3000)
  OCR1A = 3000;
  OCR1B = 3000;

  // Mode 14: Fast PWM with ICR1 as TOP
  // Clear OC1A/OC1B on compare match, set at BOTTOM (non-inverting)
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler = 8
}

/**
 * Set the servo pulse width in microseconds (500 us - 2500 us)
 * At 16 MHz with prescaler 8, 1 us = 2 timer ticks.
 */
void setServoPulse(uint16_t pulse_us) {
  if (pulse_us < PULSE_MIN_US) pulse_us = PULSE_MIN_US;
  if (pulse_us > PULSE_MAX_US) pulse_us = PULSE_MAX_US;

  uint16_t ticks = pulse_us * 2;
  OCR1A = ticks;
  OCR1B = ticks;
}

/**
 * Set the servo position by angle (0 to 180 degrees)
 * Maps 0 deg to 500 us and 180 deg to 2500 us linearly.
 */
void setServoAngle(uint8_t angle) {
  if (angle > 180) angle = 180;
  uint16_t pulse_us = PULSE_MIN_US + ((uint32_t)angle * (PULSE_MAX_US - PULSE_MIN_US)) / 180;
  setServoPulse(pulse_us);
}
