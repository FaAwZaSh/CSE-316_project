/*
 * =====================================================================================
 *  PROJECT: VANGUARD / CSE-316 - MG996R 360 Continuous Rotation Servo Test
 * =====================================================================================
 *  MCU:           ATmega32A (8.0 MHz Internal RC Oscillator)
 *  Baud Rate:     9600 Baud (USB to TTL Converter on PD0/PD1)
 *  Motor:         MG996R 360-Degree Continuous Rotation Servo Motor
 *  PWM Pin:       PD5 (Pin 19 - Timer1 OC1A Hardware PWM)
 *  Status LED:    PB0 (Pin 1 - Heartbeat Indicator LED)
 * =====================================================================================
 *  CONTINUOUS ROTATION SERVO PRINCIPLE:
 *  ------------------------------------------------------------------------------------
 *  In a 360° continuous rotation servo:
 *    ~1500 µs        -> STOP / Neutral (motor stationary)
 *    > 1500 to 2400  -> Continuous Forward / CW (pulse width controls speed)
 *    < 1500 to 544   -> Continuous Reverse / CCW (pulse width controls speed)
 * =====================================================================================
 *  WIRING GUIDE:
 *  ------------------------------------------------------------------------------------
 *  MG996R Wire Color   -->  Connection Point
 *  ------------------------------------------------------------------------------------
 *  Orange / Yellow     -->  ATmega32 Pin 19 (PD5 / OC1A)  [PWM Signal]
 *  Red (VCC)           -->  EXTERNAL 5V - 6V Power Supply (+) [DO NOT use USB
 * 5V!] Brown / Black (GND) -->  COMMON GROUND (-) [Connect Power Supply GND,
 * MCU GND, USB-TTL GND]
 *
 *  USB to TTL Pin      -->  ATmega32 Pin
 *  ------------------------------------------------------------------------------------
 *  TXD                 -->  Pin 14 (PD0 / RXD)
 *  RXD                 -->  Pin 15 (PD1 / TXD)
 *  GND                 -->  Common Ground
 * =====================================================================================
 *  INTERACTIVE SERIAL COMMANDS (9600 Baud):
 *    f       : Continuous Forward / Clockwise rotation
 *    r       : Continuous Reverse / Counter-Clockwise rotation
 *    s       : Stop / Halt rotation (1500 us)
 *    demo    : Automatic Demo Mode (CW -> Pause -> CCW -> Pause)
 *    slow    : Run at Slow speed (~25%)
 *    med     : Run at Medium speed (~60%)
 *    fast    : Run at Maximum speed (100%)
 *    + / -   : Increase / Decrease speed
 *    trim+   : Adjust neutral stop calibration +5 us (if motor slowly drifts
 * when stopped) trim-   : Adjust neutral stop calibration -5 us <us>    : Send
 * direct pulse width in microseconds (544 to 2400) h       : Print command menu
 * and status
 * =====================================================================================
 */

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

// Pin Definitions
#define SERVO_PIN PD5     // Pin 19 on ATmega32 DIP (OC1A - Hardware PWM)
#define HEARTBEAT_LED PB0 // Pin 1 on ATmega32 DIP

// MG996R Pulse Limits
#define MIN_PULSE_US 544  // Full speed CCW limit
#define MAX_PULSE_US 2400 // Full speed CW limit

// Default stop point (can be calibrated via serial if motor has neutral drift)
int neutralStopUs = 1500;

// Operational States
enum ServoMode {
  MODE_CONTINUOUS_CW,
  MODE_CONTINUOUS_CCW,
  MODE_STOPPED,
  MODE_DEMO_CYCLE,
  MODE_DIRECT_PULSE
};

ServoMode currentMode =
    MODE_CONTINUOUS_CW;    // Default to continuous rotation on boot!
int currentPulseUs = 1800; // Default forward speed (~60%)
int speedPercent = 60;     // 0 to 100%

// Demo cycle state machine
int demoStep = 0;
unsigned long lastDemoStepTime = 0;
const unsigned long DEMO_RUN_DURATION_MS = 4000;   // Spin for 4 seconds
const unsigned long DEMO_PAUSE_DURATION_MS = 1500; // Pause for 1.5 seconds

// Forward declarations
void initTimer1_HardwarePWM();
void setPulseWidth(int pulseUs);
void setSpeedAndDirection(int speedPct, bool clockwise);
void stopServo();
void processSerialInput();
void runDemoCycle();
void printStatus();
void printHelp();

/**
 * @brief Initialize 16-bit Timer 1 for Hardware Fast PWM (Mode 14) on PD5
 * (OC1A) Clock = 8 MHz, Prescaler = 8 -> 1 MHz Timer Clock (1 tick = 1
 * microsecond) ICR1 = 19999 -> TOP gives 20,000 ticks = 20 ms period = 50 Hz
 * PWM.
 */
void initTimer1_HardwarePWM() {
  DDRD |= (1 << SERVO_PIN); // Output pin

  // Mode 14: Fast PWM with ICR1 as TOP, non-inverting on OC1A
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler 8
  ICR1 = 19999;                                       // 50 Hz

  // Start with default continuous CW rotation
  setSpeedAndDirection(speedPercent, true);
}

void setPulseWidth(int pulseUs) {
  if (pulseUs < MIN_PULSE_US)
    pulseUs = MIN_PULSE_US;
  if (pulseUs > MAX_PULSE_US)
    pulseUs = MAX_PULSE_US;
  currentPulseUs = pulseUs;
  OCR1A = pulseUs; // Set hardware PWM pulse width directly
}

/**
 * @brief Set continuous rotation speed (0 to 100%) and direction
 * @param speedPct 0 to 100%
 * @param clockwise true for CW, false for CCW
 */
void setSpeedAndDirection(int speedPct, bool clockwise) {
  if (speedPct < 0)
    speedPct = 0;
  if (speedPct > 100)
    speedPct = 100;
  speedPercent = speedPct;

  if (speedPct == 0) {
    stopServo();
    return;
  }

  if (clockwise) {
    currentMode = MODE_CONTINUOUS_CW;
    // Map 1% - 100% to (neutralStopUs + 30) - MAX_PULSE_US
    int pulse =
        neutralStopUs + map(speedPct, 1, 100, 40, MAX_PULSE_US - neutralStopUs);
    setPulseWidth(pulse);
  } else {
    currentMode = MODE_CONTINUOUS_CCW;
    // Map 1% - 100% to (neutralStopUs - 30) - MIN_PULSE_US
    int pulse =
        neutralStopUs - map(speedPct, 1, 100, 40, neutralStopUs - MIN_PULSE_US);
    setPulseWidth(pulse);
  }
}

void stopServo() {
  currentMode = MODE_STOPPED;
  setPulseWidth(neutralStopUs);
}

void setup() {
  // 1. Heartbeat LED Setup
  DDRB |= (1 << HEARTBEAT_LED);
  PORTB |= (1 << HEARTBEAT_LED);

  // 2. Serial USART @ 9600 Baud
  Serial.begin(9600);
  _delay_ms(100);

  Serial.println();
  Serial.println(F("===================================================="));
  Serial.println(F(" [BOOT] ATmega32A MG996R 360 Continuous Rotation Test"));
  Serial.println(F("===================================================="));
  Serial.println(F(" Hardware: Timer1 Fast PWM | 50 Hz | PD5 (Pin 19)"));
  Serial.println(F(" Default Action: Continuous 360 Rotation Started!"));
  Serial.println(F("===================================================="));
  Serial.flush();

  // 3. Initialize Timer 1 Hardware PWM (Starts continuous 360 rotation)
  initTimer1_HardwarePWM();

  // 4. Print instructions & status
  printHelp();
  printStatus();
}

void loop() {
  // 1. Heartbeat LED Indicator
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= 300) {
    lastHeartbeat = millis();
    PORTB ^= (1 << HEARTBEAT_LED);
  }

  // 2. Process Demo Mode if active
  if (currentMode == MODE_DEMO_CYCLE) {
    runDemoCycle();
  }

  // 3. Process Serial Commands
  processSerialInput();
}

void runDemoCycle() {
  unsigned long now = millis();
  switch (demoStep) {
  case 0: // Start CW rotation
    setSpeedAndDirection(speedPercent, true);
    currentMode = MODE_DEMO_CYCLE;
    Serial.println(F(" [DEMO] Rotating 360 CW (Forward)..."));
    printStatus();
    lastDemoStepTime = now;
    demoStep = 1;
    break;

  case 1: // Wait run duration
    if (now - lastDemoStepTime >= DEMO_RUN_DURATION_MS) {
      setPulseWidth(neutralStopUs);
      Serial.println(F(" [DEMO] Pausing..."));
      lastDemoStepTime = now;
      demoStep = 2;
    }
    break;

  case 2: // Wait pause duration, then start CCW
    if (now - lastDemoStepTime >= DEMO_PAUSE_DURATION_MS) {
      setSpeedAndDirection(speedPercent, false);
      currentMode = MODE_DEMO_CYCLE;
      Serial.println(F(" [DEMO] Rotating 360 CCW (Reverse)..."));
      printStatus();
      lastDemoStepTime = now;
      demoStep = 3;
    }
    break;

  case 3: // Wait run duration
    if (now - lastDemoStepTime >= DEMO_RUN_DURATION_MS) {
      setPulseWidth(neutralStopUs);
      Serial.println(F(" [DEMO] Pausing..."));
      lastDemoStepTime = now;
      demoStep = 4;
    }
    break;

  case 4: // Wait pause duration, then repeat
    if (now - lastDemoStepTime >= DEMO_PAUSE_DURATION_MS) {
      demoStep = 0; // Loop back
    }
    break;
  }
}

void processSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0)
      return;

    if (input.equalsIgnoreCase("f") || input.equalsIgnoreCase("cw")) {
      setSpeedAndDirection(speedPercent == 0 ? 60 : speedPercent, true);
      Serial.println(F("\r\n[ACTION] Continuous 360 Rotation -> FORWARD (CW)"));
      printStatus();
    } else if (input.equalsIgnoreCase("r") || input.equalsIgnoreCase("ccw")) {
      setSpeedAndDirection(speedPercent == 0 ? 60 : speedPercent, false);
      Serial.println(
          F("\r\n[ACTION] Continuous 360 Rotation -> REVERSE (CCW)"));
      printStatus();
    } else if (input.equalsIgnoreCase("s") || input.equalsIgnoreCase("stop")) {
      stopServo();
      Serial.println(F("\r\n[ACTION] Servo STOPPED (Neutral pulse)"));
      printStatus();
    } else if (input.equalsIgnoreCase("demo")) {
      currentMode = MODE_DEMO_CYCLE;
      demoStep = 0;
      Serial.println(
          F("\r\n[ACTION] DEMO CYCLE MODE Activated (CW <-> CCW with pauses)"));
    } else if (input.equalsIgnoreCase("slow")) {
      speedPercent = 25;
      if (currentMode == MODE_CONTINUOUS_CCW)
        setSpeedAndDirection(25, false);
      else
        setSpeedAndDirection(25, true);
      Serial.println(F("\r\n[SPEED] Set to SLOW (25%)"));
      printStatus();
    } else if (input.equalsIgnoreCase("med")) {
      speedPercent = 60;
      if (currentMode == MODE_CONTINUOUS_CCW)
        setSpeedAndDirection(60, false);
      else
        setSpeedAndDirection(60, true);
      Serial.println(F("\r\n[SPEED] Set to MEDIUM (60%)"));
      printStatus();
    } else if (input.equalsIgnoreCase("fast") ||
               input.equalsIgnoreCase("max")) {
      speedPercent = 100;
      if (currentMode == MODE_CONTINUOUS_CCW)
        setSpeedAndDirection(100, false);
      else
        setSpeedAndDirection(100, true);
      Serial.println(F("\r\n[SPEED] Set to MAXIMUM (100%)"));
      printStatus();
    } else if (input.equals("+")) {
      int newSpeed = speedPercent + 10;
      if (newSpeed > 100)
        newSpeed = 100;
      if (currentMode == MODE_CONTINUOUS_CCW)
        setSpeedAndDirection(newSpeed, false);
      else
        setSpeedAndDirection(newSpeed, true);
      Serial.print(F("\r\n[SPEED] Nudge Speed +10% -> "));
      printStatus();
    } else if (input.equals("-")) {
      int newSpeed = speedPercent - 10;
      if (newSpeed < 0)
        newSpeed = 0;
      if (currentMode == MODE_CONTINUOUS_CCW)
        setSpeedAndDirection(newSpeed, false);
      else
        setSpeedAndDirection(newSpeed, true);
      Serial.print(F("\r\n[SPEED] Nudge Speed -10% -> "));
      printStatus();
    } else if (input.equalsIgnoreCase("trim+")) {
      neutralStopUs += 5;
      if (currentMode == MODE_STOPPED)
        setPulseWidth(neutralStopUs);
      Serial.print(F("\r\n[TRIM] Neutral Stop calibrated to: "));
      Serial.print(neutralStopUs);
      Serial.println(F(" us"));
    } else if (input.equalsIgnoreCase("trim-")) {
      neutralStopUs -= 5;
      if (currentMode == MODE_STOPPED)
        setPulseWidth(neutralStopUs);
      Serial.print(F("\r\n[TRIM] Neutral Stop calibrated to: "));
      Serial.print(neutralStopUs);
      Serial.println(F(" us"));
    } else if (input.equalsIgnoreCase("h") || input.equalsIgnoreCase("help")) {
      printHelp();
      printStatus();
    } else {
      int val = input.toInt();
      // If value looks like a direct microsecond pulse (e.g. 500 to 2500)
      if (val >= MIN_PULSE_US && val <= MAX_PULSE_US) {
        currentMode = MODE_DIRECT_PULSE;
        setPulseWidth(val);
        Serial.print(F("\r\n[DIRECT] Custom Pulse Width applied: "));
        Serial.print(val);
        Serial.println(F(" us"));
        printStatus();
      } else if (val >= 0 && val <= 100) {
        // If value 0 to 100, treat as speed percent
        bool cw = (currentMode != MODE_CONTINUOUS_CCW);
        setSpeedAndDirection(val, cw);
        Serial.print(F("\r\n[SPEED] Speed set to "));
        Serial.print(val);
        Serial.println(F("%"));
        printStatus();
      } else {
        Serial.print(F("[UNKNOWN] '"));
        Serial.print(input);
        Serial.println(F("'. Send 'h' for command list."));
      }
    }
  }
}

void printStatus() {
  Serial.print(F(" [STATUS] Mode: "));
  switch (currentMode) {
  case MODE_CONTINUOUS_CW:
    Serial.print(F("CW Continuous 360"));
    break;
  case MODE_CONTINUOUS_CCW:
    Serial.print(F("CCW Continuous 360"));
    break;
  case MODE_STOPPED:
    Serial.print(F("STOPPED (Idle)"));
    break;
  case MODE_DEMO_CYCLE:
    Serial.print(F("Demo Cycle"));
    break;
  case MODE_DIRECT_PULSE:
    Serial.print(F("Direct Pulse"));
    break;
  }
  Serial.print(F(" | Speed: "));
  Serial.print(speedPercent);
  Serial.print(F("% | Pulse: "));
  Serial.print(currentPulseUs);
  Serial.print(F(" us | Neutral: "));
  Serial.print(neutralStopUs);
  Serial.println(F(" us"));
  Serial.flush();
}

void printHelp() {
  Serial.println(F("\r\n----------------------------------------------------"));
  Serial.println(F("       360 CONTINUOUS ROTATION COMMAND MENU"));
  Serial.println(F("----------------------------------------------------"));
  Serial.println(F("  f / cw   : Continuous Forward / Clockwise rotation"));
  Serial.println(
      F("  r / ccw  : Continuous Reverse / Counter-Clockwise rotation"));
  Serial.println(F("  s / stop : Stop motor rotation (Neutral 1500 us)"));
  Serial.println(
      F("  demo     : Auto Demo Cycle (CW -> Pause -> CCW -> Pause)"));
  Serial.println(F("  slow     : Run at 25% speed"));
  Serial.println(F("  med      : Run at 60% speed"));
  Serial.println(F("  fast     : Run at 100% max speed"));
  Serial.println(F("  + / -    : Speed up / slow down by 10%"));
  Serial.println(F("  0 - 100  : Set exact speed percentage"));
  Serial.println(F("  trim+/-  : Calibrate neutral zero-point (+/- 5us)"));
  Serial.println(F("  <544-2400> : Direct microsecond pulse width"));
  Serial.println(F("  h        : Show this help menu"));
  Serial.println(F("----------------------------------------------------"));
  Serial.flush();
}
