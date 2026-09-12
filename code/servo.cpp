/*
 * =====================================================================================
 *  PROJECT: VANGUARD / CSE-316 - 2-Axis Pan-Tilt Turret Dual Servo Test
 * =====================================================================================
 *  MCU:           ATmega32A (8.0 MHz Internal RC Oscillator)
 *  Baud Rate:     9600 Baud (USB to TTL Converter on PD0/PD1)
 *  Pan Motor:     MG996R Pan Servo on PD5 (Pin 19 - Timer1 OC1A Hardware PWM)
 *  Tilt Motor:    MG996R Tilt Servo on PD4 (Pin 18 - Timer1 OC1B Hardware PWM)
 *  Status LED:    PB0 (Pin 1 - Heartbeat Indicator LED)
 * =====================================================================================
 *  TEST SEQUENCE:
 *    1. Pan moves to Position A (1200 us / ~60 deg)
 *    2. Tilt moves to Position A (1300 us / ~70 deg)
 *    3. Rest 2.0s holding Position A
 *    4. Tilt reverses to Position B (1700 us / ~110 deg)
 *    5. Pan reverses to Position B (1800 us / ~120 deg)
 *    6. Rest 2.0s holding Position B
 *    7. Repeat in reverse continuously!
 * =====================================================================================
 *  WIRING GUIDE:
 *  Pan Servo Yellow/Orange (Signal)   --> ATmega32 Pin 19 (PD5 / OC1A)
 *  Tilt Servo Yellow/Orange (Signal)  --> ATmega32 Pin 18 (PD4 / OC1B)
 *  Both Servos Red (VCC)              --> EXTERNAL 5V - 6V Power Supply (+) [DO NOT use USB 5V!]
 *  Both Servos Brown/Black (GND)      --> COMMON GROUND (-) [Connect Power Supply GND, MCU GND, USB-TTL GND]
 *  USB to TTL Pin TXD                 --> ATmega32 Pin 14 (PD0 / RXD)
 *  USB to TTL Pin RXD                 --> ATmega32 Pin 15 (PD1 / TXD)
 * =====================================================================================
 */

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

// Pin Definitions
#define PAN_SERVO_PIN    PD5  // Pin 19 on ATmega32 DIP (OC1A - Hardware PWM)
#define TILT_SERVO_PIN   PD4  // Pin 18 on ATmega32 DIP (OC1B - Hardware PWM)
#define HEARTBEAT_LED    PB0  // Pin 1 on ATmega32 DIP

// MG996R Pulse Limits (Microseconds)
#define MIN_PULSE_US     544
#define MAX_PULSE_US     2400
#define SERVO_CENTER_US  1500

// Target Calibration Positions
#define PAN_POS_A_US     1200  // Pan Position A (Left / ~60 deg)
#define TILT_POS_A_US    1300  // Tilt Position A (Down / ~70 deg)
#define PAN_POS_B_US     1800  // Pan Position B (Right / ~120 deg)
#define TILT_POS_B_US    1700  // Tilt Position B (Up / ~110 deg)

#define SERVO_TRAVEL_MS  800UL  // Settle time for mechanical motion (ms)
#define TURRET_REST_MS   2000UL // Rest duration (2.0s) requested

// Current pulse widths
int currentPanPulseUs  = SERVO_CENTER_US;
int currentTiltPulseUs = SERVO_CENTER_US;

// Operational States
enum TurretState {
    TURRET_STATE_MOVE_PAN_A,
    TURRET_STATE_WAIT_PAN_A,
    TURRET_STATE_WAIT_TILT_A,
    TURRET_STATE_REST_A,
    TURRET_STATE_WAIT_TILT_B,
    TURRET_STATE_WAIT_PAN_B,
    TURRET_STATE_REST_B
};

TurretState currentTurretState = TURRET_STATE_MOVE_PAN_A;
unsigned long lastTransitionMs = 0;
bool autoCycleActive = true;

// Forward declarations
void initTimer1_DualServoPWM();
void setPanPulse(int pulseUs);
void setTiltPulse(int pulseUs);
void updateTurretSequence(unsigned long now);
void processSerialInput();
void handleCommand(const String &input);
void printStatus();
void printHelp();

/**
 * @brief Initialize 16-bit Timer 1 for Hardware Fast PWM (Mode 14) on PD5 (OC1A) and PD4 (OC1B)
 * Clock = 8 MHz, Prescaler = 8 -> 1 MHz Timer Clock (1 tick = 1 microsecond)
 * ICR1 = 19999 -> TOP gives 20,000 ticks = 20 ms period = 50 Hz PWM.
 */
void initTimer1_DualServoPWM() {
    // Set PD4 (pin 18) and PD5 (pin 19) as outputs
    DDRD |= (1 << PAN_SERVO_PIN) | (1 << TILT_SERVO_PIN);

    // Mode 14: Fast PWM with ICR1 as TOP, non-inverting on OC1A and OC1B
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler 8
    ICR1 = 19999;                                       // 50 Hz

    // Start with servos at Center (1500 us)
    setPanPulse(SERVO_CENTER_US);
    setTiltPulse(SERVO_CENTER_US);
}

void setPanPulse(int pulseUs) {
    if (pulseUs < MIN_PULSE_US) pulseUs = MIN_PULSE_US;
    if (pulseUs > MAX_PULSE_US) pulseUs = MAX_PULSE_US;
    currentPanPulseUs = pulseUs;
    OCR1A = pulseUs;
}

void setTiltPulse(int pulseUs) {
    if (pulseUs < MIN_PULSE_US) pulseUs = MIN_PULSE_US;
    if (pulseUs > MAX_PULSE_US) pulseUs = MAX_PULSE_US;
    currentTiltPulseUs = pulseUs;
    OCR1B = pulseUs;
}

void setup() {
    // 1. Heartbeat LED Setup
    DDRB |= (1 << HEARTBEAT_LED);
    PORTB |= (1 << HEARTBEAT_LED);

    // 2. Serial USART @ 9600 Baud
    Serial.begin(9600);
    _delay_ms(50);

    Serial.println();
    Serial.println(F("===================================================="));
    Serial.println(F(" [BOOT] ATmega32A 2-Axis Pan-Tilt Dual Servo Test   "));
    Serial.println(F("===================================================="));
    Serial.println(F(" Hardware: Timer1 Fast PWM | 50 Hz                  "));
    Serial.println(F(" Pan     : Pin 19 (PD5 / OC1A)                      "));
    Serial.println(F(" Tilt    : Pin 18 (PD4 / OC1B)                      "));
    Serial.println(F(" Motion  : Pan -> Tilt -> Rest 2s -> Repeat Reverse "));
    Serial.println(F("===================================================="));

    // 3. Initialize Timer 1 Hardware Dual PWM
    initTimer1_DualServoPWM();
    lastTransitionMs = millis();

    // 4. Print instructions & initial status
    printHelp();
    printStatus();
}

void loop() {
    unsigned long now = millis();

    // 1. Heartbeat LED Indicator (non-blocking)
    static unsigned long lastHeartbeat = 0;
    if (now - lastHeartbeat >= 300) {
        lastHeartbeat = now;
        PORTB ^= (1 << HEARTBEAT_LED);
    }

    // 2. Process Pan-Tilt Sequence State Machine
    updateTurretSequence(now);

    // 3. Non-blocking Serial Command Processing
    processSerialInput();
}

void updateTurretSequence(unsigned long now) {
    if (!autoCycleActive) return;

    switch (currentTurretState) {
        case TURRET_STATE_MOVE_PAN_A:
            setPanPulse(PAN_POS_A_US);
            Serial.print(F(" >>> [TURRET] (1/4) Pan moving to Pos A ("));
            Serial.print(PAN_POS_A_US);
            Serial.println(F(" us)..."));
            lastTransitionMs = now;
            currentTurretState = TURRET_STATE_WAIT_PAN_A;
            break;

        case TURRET_STATE_WAIT_PAN_A:
            if (now - lastTransitionMs >= SERVO_TRAVEL_MS) {
                setTiltPulse(TILT_POS_A_US);
                Serial.print(F(" >>> [TURRET] (2/4) Tilt moving to Pos A ("));
                Serial.print(TILT_POS_A_US);
                Serial.println(F(" us)..."));
                lastTransitionMs = now;
                currentTurretState = TURRET_STATE_WAIT_TILT_A;
            }
            break;

        case TURRET_STATE_WAIT_TILT_A:
            if (now - lastTransitionMs >= SERVO_TRAVEL_MS) {
                Serial.println(F(" === [TURRET] At Position A. Resting 2.0s ==="));
                lastTransitionMs = now;
                currentTurretState = TURRET_STATE_REST_A;
            }
            break;

        case TURRET_STATE_REST_A:
            if (now - lastTransitionMs >= TURRET_REST_MS) {
                Serial.println(F(" --- [TURRET] 2s Rest complete! Reversing sequence..."));
                setTiltPulse(TILT_POS_B_US);
                Serial.print(F(" >>> [TURRET] (3/4 REV) Tilt moving to Pos B ("));
                Serial.print(TILT_POS_B_US);
                Serial.println(F(" us)..."));
                lastTransitionMs = now;
                currentTurretState = TURRET_STATE_WAIT_TILT_B;
            }
            break;

        case TURRET_STATE_WAIT_TILT_B:
            if (now - lastTransitionMs >= SERVO_TRAVEL_MS) {
                setPanPulse(PAN_POS_B_US);
                Serial.print(F(" >>> [TURRET] (4/4 REV) Pan moving to Pos B ("));
                Serial.print(PAN_POS_B_US);
                Serial.println(F(" us)..."));
                lastTransitionMs = now;
                currentTurretState = TURRET_STATE_WAIT_PAN_B;
            }
            break;

        case TURRET_STATE_WAIT_PAN_B:
            if (now - lastTransitionMs >= SERVO_TRAVEL_MS) {
                Serial.println(F(" === [TURRET] At Position B. Resting 2.0s ==="));
                lastTransitionMs = now;
                currentTurretState = TURRET_STATE_REST_B;
            }
            break;

        case TURRET_STATE_REST_B:
            if (now - lastTransitionMs >= TURRET_REST_MS) {
                Serial.println(F(" --- [TURRET] 2s Rest complete! Repeating forward sequence..."));
                currentTurretState = TURRET_STATE_MOVE_PAN_A;
                lastTransitionMs = now;
            }
            break;
    }
}

/**
 * @brief Non-blocking serial input reader
 */
void processSerialInput() {
    static char inputBuffer[32];
    static uint8_t bufIdx = 0;

    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            inputBuffer[bufIdx] = '\0';
            String input = String(inputBuffer);
            input.trim();
            bufIdx = 0;
            if (input.length() > 0) {
                handleCommand(input);
            }
        } else if (bufIdx < sizeof(inputBuffer) - 1) {
            inputBuffer[bufIdx++] = c;
        }
    }
}

void handleCommand(const String &input) {
    if (input.equalsIgnoreCase("m") || input.equalsIgnoreCase("auto")) {
        autoCycleActive = !autoCycleActive;
        Serial.print(F("\r\n[ACTION] Turret Auto-Cycle: "));
        Serial.println(autoCycleActive ? F("ENABLED (Pan -> Tilt -> Rest 2s -> Reverse)") : F("PAUSED"));
        if (autoCycleActive) lastTransitionMs = millis();
        printStatus();
    }
    else if (input.equalsIgnoreCase("c") || input.equalsIgnoreCase("center")) {
        autoCycleActive = false;
        setPanPulse(SERVO_CENTER_US);
        setTiltPulse(SERVO_CENTER_US);
        Serial.println(F("\r\n[ACTION] Both Servos CENTERED (1500 us)"));
        printStatus();
    }
    else if (input.equalsIgnoreCase("1") || input.equalsIgnoreCase("a")) {
        autoCycleActive = false;
        setPanPulse(PAN_POS_A_US);
        setTiltPulse(TILT_POS_A_US);
        Serial.println(F("\r\n[ACTION] Jumped to Position A"));
        printStatus();
    }
    else if (input.equalsIgnoreCase("2") || input.equalsIgnoreCase("b")) {
        autoCycleActive = false;
        setPanPulse(PAN_POS_B_US);
        setTiltPulse(TILT_POS_B_US);
        Serial.println(F("\r\n[ACTION] Jumped to Position B"));
        printStatus();
    }
    else if (input.equalsIgnoreCase("s") || input.equalsIgnoreCase("stop")) {
        autoCycleActive = false;
        Serial.println(F("\r\n[ACTION] Motion PAUSED (Holding current pulse)"));
        printStatus();
    }
    else if (input.equals("[")) {
        autoCycleActive = false;
        setPanPulse(currentPanPulseUs - 50);
        printStatus();
    }
    else if (input.equals("]")) {
        autoCycleActive = false;
        setPanPulse(currentPanPulseUs + 50);
        printStatus();
    }
    else if (input.equals("{")) {
        autoCycleActive = false;
        setTiltPulse(currentTiltPulseUs - 50);
        printStatus();
    }
    else if (input.equals("}")) {
        autoCycleActive = false;
        setTiltPulse(currentTiltPulseUs + 50);
        printStatus();
    }
    else if (input.equalsIgnoreCase("h") || input.equalsIgnoreCase("help")) {
        printHelp();
        printStatus();
    }
    else {
        Serial.print(F("[UNKNOWN] '"));
        Serial.print(input);
        Serial.println(F("'. Send 'h' for command list."));
    }
}

void printStatus() {
    Serial.print(F(" [STATUS] Pan (PD5/19): "));
    Serial.print(currentPanPulseUs);
    Serial.print(F(" us | Tilt (PD4/18): "));
    Serial.print(currentTiltPulseUs);
    Serial.print(F(" us | Mode: "));
    Serial.println(autoCycleActive ? F("AUTO-CYCLE") : F("PAUSED/MANUAL"));
}

void printHelp() {
    Serial.println(F("\r\n----------------------------------------------------"));
    Serial.println(F("        PAN-TILT TURRET TEST COMMAND MENU"));
    Serial.println(F("----------------------------------------------------"));
    Serial.println(F("  m / auto : Toggle Auto Cycle (Pan->Tilt->Rest 2s->Reverse)"));
    Serial.println(F("  s / stop : Pause auto motion (hold position)"));
    Serial.println(F("  c / center: Center both servos (1500 us)"));
    Serial.println(F("  1 / a    : Jump to Position A (1200 us, 1300 us)"));
    Serial.println(F("  2 / b    : Jump to Position B (1800 us, 1700 us)"));
    Serial.println(F("  [ / ]    : Nudge Pan Left / Right (-50 / +50 us)"));
    Serial.println(F("  { / }    : Nudge Tilt Down / Up   (-50 / +50 us)"));
    Serial.println(F("  h        : Show this help menu"));
    Serial.println(F("----------------------------------------------------"));
}
