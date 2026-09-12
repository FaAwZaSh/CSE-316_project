/*
 * =====================================================================================
 *  PROJECT: VANGUARD - Pan Servo 0° -> 45° -> 90° -> 180° Low-Speed Sweep
 * =====================================================================================
 *  MCU:           ATmega32A (8.0 MHz Internal RC Oscillator)
 *  Baud Rate:     9600 Baud (8-N-1) on PD0(RXD)/PD1(TXD)
 *  Pan Motor:     MG996R Pan Servo on Pin 19 (PD5 / OC1A - Timer1 Hardware PWM)
 *  Status LED:    Pin 1 (PB0 - Heartbeat Indicator)
 * =====================================================================================
 *  SEQUENCE:
 *    0° (600 us)  --> 45° (1050 us) --> 90° (1500 us) --> 180° (2400 us)
 *    and reverse:
 *    180° (2400 us) --> 90° (1500 us) --> 45° (1050 us) --> 0° (600 us)
 *    repeating continuously at smooth low speed with a 1-second hold at each angle.
 * =====================================================================================
 *  WIRING:
 *    Pan Signal (Yellow/Orange)  --> ATmega32 Pin 19 (PD5 / OC1A)
 *    Pan VCC (Red)               --> External 5V - 6V Power Supply (+)
 *    Pan GND (Brown/Black)       --> Common GND (-) [Power Supply GND + MCU GND + TTL GND]
 * =====================================================================================
 */

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Hardware Pin Definitions
#define PAN_SERVO_PIN       PD5   // Pin 19: Timer1 OC1A Hardware PWM
#define HEARTBEAT_LED       PB0   // Pin 1: System Heartbeat LED

// MG996R Pulse Limits with Safety Buffers (Avoids hitting physical gear stops)
#define SERVO_MIN_US        700   // Hard software minimum limit (safely above physical stop)
#define SERVO_MAX_US        2300  // Hard software maximum limit (safely below physical stop)

#define POS_MIN_SAFE_US     750   // ~15° Safe Low Position (stops before plastic tab hits)
#define POS_45_DEG_US       1050  // 45°  Position (~1050 us)
#define POS_90_DEG_US       1500  // 90°  Center Position (~1500 us)
#define POS_MAX_SAFE_US     2250  // ~165° Safe High Position (stops before plastic tab hits)

// Waypoint Array: Safe Low -> 45° -> 90° -> Safe High -> 90° -> 45° -> (repeats)
const uint16_t WAYPOINTS_US[]   = { POS_MIN_SAFE_US, POS_45_DEG_US, POS_90_DEG_US, POS_MAX_SAFE_US, POS_90_DEG_US, POS_45_DEG_US };
const uint8_t  WAYPOINTS_DEG[]  = { 15,              45,             90,            165,              90,             45 };
const uint8_t  NUM_WAYPOINTS    = sizeof(WAYPOINTS_US) / sizeof(WAYPOINTS_US[0]);

// Low-Speed Motion Parameters
static uint8_t  step_us_per_tick = 3;     // Step size per 20ms tick (3 us = ~150 us/s: very gentle & smooth)
static uint16_t hold_time_ms     = 1000;  // Pause duration at each angle milestone (1.0 second)
const  uint8_t  tick_interval_ms = 20;    // 20ms tick interval (50 Hz frame sync)

// State Machine
enum MotionPhase {
    PHASE_SLEWING,   // Smoothly moving towards the target waypoint at low speed
    PHASE_HOLDING    // Resting at the target angle for hold_time_ms
};

static MotionPhase current_phase    = PHASE_SLEWING;
static uint8_t     waypoint_idx     = 0;                // Starts moving towards index 0 (0°)
static float       current_pulse_us = POS_90_DEG_US;    // Start physically centered
static uint32_t    phase_start_ms   = 0;
static bool        is_paused        = false;

// Forward Declarations
void initTimer1_PanPWM(void);
void setPanPulse(uint16_t pulse_us);
void updateMotion(uint32_t now);
void processSerialCommands(void);
void printHelp(void);
void printStatus(void);

/* ---------- Setup Routine ---------- */
void setup() {
    // 1. Heartbeat LED Setup
    DDRB |= (1 << HEARTBEAT_LED);
    PORTB |= (1 << HEARTBEAT_LED);

    // 2. Serial Communication @ 9600 Baud
    Serial.begin(9600);
    delay(50);

    Serial.println();
    Serial.println(F("================================================================"));
    Serial.println(F("   VANGUARD: LOW-SPEED PAN SERVO SWEEP BENCH                    "));
    Serial.println(F("================================================================"));
    Serial.println(F(" Hardware : ATmega32A 8.0 MHz | Timer1 50 Hz Fast PWM          "));
    Serial.println(F(" Pan Pin  : Pin 19 (PD5 / OC1A)                                "));
    Serial.println(F(" Sequence : 0° -> 45° -> 90° -> 180° -> 90° -> 45° -> Repeat   "));
    Serial.println(F(" Speed    : Gentle Low Speed (~150 us/s) with 1.0s Pause at Stop"));
    Serial.println(F("================================================================"));

    // 3. Initialize Timer 1 Hardware Fast PWM on PD5
    initTimer1_PanPWM();

    // 4. Start by moving towards first waypoint
    phase_start_ms = millis();
    Serial.print(F(">>> [START] Moving gently to Safe Min ("));
    Serial.print(WAYPOINTS_US[0]);
    Serial.println(F(" us)..."));

    printHelp();
}

/* ---------- Main Loop ---------- */
void loop() {
    uint32_t now = millis();

    // 1. Non-blocking Heartbeat Indicator (1 Hz)
    static uint32_t last_heartbeat_ms = 0;
    if (now - last_heartbeat_ms >= 500) {
        last_heartbeat_ms = now;
        PORTB ^= (1 << HEARTBEAT_LED);
    }

    // 2. Continuous Low-Speed Motion Engine
    if (!is_paused) {
        updateMotion(now);
    }

    // 3. Process Serial Input Commands
    if (Serial.available() > 0) {
        processSerialCommands();
    }
}

/* ---------- Timer1 Hardware Fast PWM on PD5 (OC1A) ---------- */
void initTimer1_PanPWM(void) {
    // Set PD5 (Pin 19 - Pan Servo OC1A) as output
    DDRD |= (1 << PAN_SERVO_PIN);

    // Mode 14: Fast PWM with ICR1 as TOP, Prescaler = 8
    // Clock: 8 MHz / 8 = 1 MHz (1 tick = 1 microsecond)
    // Non-inverting PWM on OC1A (PD5)
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 19999; // 20,000 ticks = 20,000 us = 20 ms = 50 Hz PWM

    // Initialize Pan at Center (1500 us)
    setPanPulse((uint16_t)current_pulse_us);
}

/* ---------- Direct Pulse Width Output to Hardware Register ---------- */
void setPanPulse(uint16_t pulse_us) {
    if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
    if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;
    current_pulse_us = pulse_us;
    OCR1A = pulse_us; // Direct hardware PWM register
}

/* ---------- Non-Blocking Low-Speed Motion State Machine ---------- */
void updateMotion(uint32_t now) {
    static uint32_t last_tick_ms = 0;

    if (current_phase == PHASE_SLEWING) {
        if (now - last_tick_ms >= tick_interval_ms) {
            last_tick_ms = now;

            uint16_t target_us = WAYPOINTS_US[waypoint_idx];

            if (current_pulse_us < target_us) {
                current_pulse_us += step_us_per_tick;
                if (current_pulse_us >= target_us) {
                    current_pulse_us = target_us;
                }
            } else if (current_pulse_us > target_us) {
                if (current_pulse_us - target_us <= step_us_per_tick) {
                    current_pulse_us = target_us;
                } else {
                    current_pulse_us -= step_us_per_tick;
                }
            }

            setPanPulse((uint16_t)current_pulse_us);

            // Reached target angle waypoint!
            if ((uint16_t)current_pulse_us == target_us) {
                current_phase = PHASE_HOLDING;
                phase_start_ms = now;

                Serial.print(F(" === [HOLD] Reached "));
                Serial.print(WAYPOINTS_DEG[waypoint_idx]);
                Serial.print(F("° ("));
                Serial.print(target_us);
                Serial.print(F(" us) | Resting "));
                Serial.print(hold_time_ms / 1000.0f, 1);
                Serial.println(F("s..."));
            }
        }
    } else if (current_phase == PHASE_HOLDING) {
        if (now - phase_start_ms >= hold_time_ms) {
            // Advance to next waypoint in the round-trip sequence
            waypoint_idx = (waypoint_idx + 1) % NUM_WAYPOINTS;
            current_phase = PHASE_SLEWING;

            Serial.print(F(" >>> [SLEW] Moving to "));
            Serial.print(WAYPOINTS_DEG[waypoint_idx]);
            Serial.print(F("° ("));
            Serial.print(WAYPOINTS_US[waypoint_idx]);
            Serial.println(F(" us)..."));
        }
    }
}

/* ---------- Interactive Serial Commands ---------- */
void processSerialCommands(void) {
    char cmd = Serial.read();
    if (cmd < 32 || cmd > 126) return;

    Serial.println();
    Serial.print(F(">> Command: '"));
    Serial.print(cmd);
    Serial.println(F("'"));

    switch (cmd) {
        case ' ':
        case 'p':
        case 'P':
            is_paused = !is_paused;
            Serial.print(F(">> Motion: "));
            Serial.println(is_paused ? F("PAUSED") : F("RESUMED"));
            break;

        case '+':
        case '=':
            if (step_us_per_tick < 30) step_us_per_tick++;
            Serial.print(F(">> Speed INCREASED: step = "));
            Serial.print(step_us_per_tick);
            Serial.println(F(" us/tick"));
            break;

        case '-':
        case '_':
            if (step_us_per_tick > 1) step_us_per_tick--;
            Serial.print(F(">> Speed DECREASED: step = "));
            Serial.print(step_us_per_tick);
            Serial.println(F(" us/tick (very slow & gentle)"));
            break;

        case '0':
            is_paused = false;
            waypoint_idx = 0; // 0°
            current_phase = PHASE_SLEWING;
            Serial.println(F(">> Target set to 0°"));
            break;

        case '4':
            is_paused = false;
            waypoint_idx = 1; // 45°
            current_phase = PHASE_SLEWING;
            Serial.println(F(">> Target set to 45°"));
            break;

        case '9':
        case 'c':
        case 'C':
            is_paused = false;
            waypoint_idx = 2; // 90° (Center)
            current_phase = PHASE_SLEWING;
            Serial.println(F(">> Target set to 90° (Center)"));
            break;

        case '8':
            is_paused = false;
            waypoint_idx = 3; // 180°
            current_phase = PHASE_SLEWING;
            Serial.println(F(">> Target set to 180°"));
            break;

        case '[':
            is_paused = true;
            setPanPulse((uint16_t)(current_pulse_us > (50 + SERVO_MIN_US) ? (current_pulse_us - 50) : SERVO_MIN_US));
            Serial.print(F(">> Nudged Left: "));
            Serial.print((uint16_t)current_pulse_us);
            Serial.println(F(" us (Paused)"));
            break;

        case ']':
            is_paused = true;
            setPanPulse((uint16_t)(current_pulse_us + 50 < SERVO_MAX_US ? (current_pulse_us + 50) : SERVO_MAX_US));
            Serial.print(F(">> Nudged Right: "));
            Serial.print((uint16_t)current_pulse_us);
            Serial.println(F(" us (Paused)"));
            break;

        case 's':
        case 'S':
            printStatus();
            break;

        case 'h':
        case 'H':
        case '?':
            printHelp();
            break;

        default:
            Serial.println(F(">> Unknown command. Send 'h' for menu."));
            break;
    }
}

void printStatus(void) {
    Serial.println(F("------------------- STATUS SNAPSHOT -------------------"));
    Serial.print(F(" Pan Pulse     : "));
    Serial.print((uint16_t)current_pulse_us);
    Serial.println(F(" us"));
    Serial.print(F(" Target        : "));
    Serial.print(WAYPOINTS_DEG[waypoint_idx]);
    Serial.print(F("° ("));
    Serial.print(WAYPOINTS_US[waypoint_idx]);
    Serial.println(F(" us)"));
    Serial.print(F(" Phase         : "));
    Serial.println(current_phase == PHASE_SLEWING ? F("SLEWING (Low Speed)") : F("HOLDING POSITION"));
    Serial.print(F(" Motion State  : "));
    Serial.println(is_paused ? F("PAUSED") : F("RUNNING CONTINUOUSLY"));
    Serial.print(F(" Slew Speed    : "));
    Serial.print(step_us_per_tick);
    Serial.print(F(" us/tick (~"));
    Serial.print(step_us_per_tick * 50);
    Serial.println(F(" us/s)"));
    Serial.println(F("-------------------------------------------------------"));
}

void printHelp(void) {
    Serial.println(F("-------------------- COMMAND MENU --------------------"));
    Serial.println(F("  'p' or Space : Pause / Resume continuous sweep"));
    Serial.println(F("  '+' / '-'    : Increase / Decrease slew speed"));
    Serial.println(F("  '0'          : Slew to 0°   (600 us)"));
    Serial.println(F("  '4'          : Slew to 45°  (1050 us)"));
    Serial.println(F("  '9' or 'c'   : Slew to 90°  (1500 us - Center)"));
    Serial.println(F("  '8'          : Slew to 180° (2400 us)"));
    Serial.println(F("  '[' / ']'    : Manual nudge Left / Right (-50 / +50 us)"));
    Serial.println(F("  's'          : Print current status snapshot"));
    Serial.println(F("  'h' or '?'   : Show this help menu"));
    Serial.println(F("------------------------------------------------------"));
}
