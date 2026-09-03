/*
 * =====================================================================================
 *  PROJECT: VANGUARD - Dual PIR Sensor Surveillance & Sector Monitor
 * =====================================================================================
 *  MCU:           ATmega32A (8.0 MHz Internal RC Oscillator)
 *  Baud Rate:     9600 Baud (8-N-1) over USB-to-TTL Converter
 * =====================================================================================
 *  HARDWARE PIN CONNECTIONS (DIP-40):
 * -------------------------------------------------------------------------------------
 *  PIR Sensor 0 (Left Sector):
 *    - OUT   --> ATmega32 Pin 16 (PD2 / INT0) [External Interrupt 0]
 *    - VCC   --> +5V
 *    - GND   --> Common GND
 *
 *  PIR Sensor 1 (Right Sector):
 *    - OUT   --> ATmega32 Pin 17 (PD3 / INT1) [External Interrupt 1]
 *    - VCC   --> +5V
 *    - GND   --> Common GND
 *
 *  USB-to-TTL Converter (PL2303 / CP2102 / FTDI):
 *    - TXD   --> ATmega32 Pin 14 (PD0 / RXD)
 *    - RXD   --> ATmega32 Pin 15 (PD1 / TXD)
 *    - GND   --> Common GND (MANDATORY)
 *    - VCC   --> DO NOT CONNECT if USBasp programmer is powering the board!
 *
 *  Status Indicators:
 *    - LED0 (PIR 0 Active)  --> ATmega32 Pin 40 (PA0) / Pin 39 (PA1) -> 330R -> GND
 *    - LED1 (PIR 1 Active)  --> ATmega32 Pin 33 (PA7) -> 330R -> GND
 *    - Heartbeat LED        --> ATmega32 Pin 1  (PB0) -> 330R -> GND
 * =====================================================================================
 */

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/* ---------- Pin Definitions (Direct Port Bit Offsets) ---------- */
#define PIR0_BIT        PD2    /* Pin 16: INT0 input */
#define PIR1_BIT        PD3    /* Pin 17: INT1 input */

#define LED0_PA0_BIT    PA0    /* Pin 40: Indicator for PIR 0 */
#define LED0_PA1_BIT    PA1    /* Pin 39: Alternate indicator for PIR 0 */
#define LED1_PA7_BIT    PA7    /* Pin 33: Indicator for PIR 1 */
#define HEARTBEAT_BIT   PB0    /* Pin 1:  System heartbeat LED */

/* ---------- Debounce & Timing Tunables ---------- */
#define PIR_DEBOUNCE_MS     150UL   /* Min ms between interrupt triggers to filter noise */
#define TELEMETRY_INTERVAL  1000UL  /* Periodic telemetry stream interval in ms */
#define HEARTBEAT_INTERVAL  500UL   /* Heartbeat LED toggle interval in ms */

/* ---------- Volatile Data Shared with ISRs ---------- */
volatile uint32_t pir0_trigger_count = 0;
volatile uint32_t pir1_trigger_count = 0;
volatile uint32_t pir0_last_trigger_ms = 0;
volatile uint32_t pir1_last_trigger_ms = 0;
volatile bool     pir0_event_pending = false;
volatile bool     pir1_event_pending = false;

/* ---------- Tracking Variables for Main Loop ---------- */
static bool pir0_was_high = false;
static bool pir1_was_high = false;
static uint32_t pir0_high_since_ms = 0;
static uint32_t pir1_high_since_ms = 0;

static uint32_t last_telemetry_ms = 0;
static uint32_t last_heartbeat_ms = 0;
static bool verbose_stream = true;

/* ---------- Forward Declarations ---------- */
void printBootBanner(void);
void printHelp(void);
void printDetailedStatus(void);
void runLedTest(void);
void resetStatistics(void);
void processSerialCommands(void);
void formatTime(uint32_t total_ms);

/* ---------- External Interrupt Service Routines ---------- */
// INT0 (PD2 / Pin 16): PIR Sensor 0 (Left Sector)
ISR(INT0_vect) {
    uint32_t now = millis();
    if ((now - pir0_last_trigger_ms) >= PIR_DEBOUNCE_MS) {
        pir0_trigger_count++;
        pir0_last_trigger_ms = now;
        pir0_event_pending = true;
    }
}

// INT1 (PD3 / Pin 17): PIR Sensor 1 (Right Sector)
ISR(INT1_vect) {
    uint32_t now = millis();
    if ((now - pir1_last_trigger_ms) >= PIR_DEBOUNCE_MS) {
        pir1_trigger_count++;
        pir1_last_trigger_ms = now;
        pir1_event_pending = true;
    }
}

/* ---------- Setup Routine ---------- */
void setup() {
    /* 1. Configure LED outputs */
    DDRA |= (1 << LED0_PA0_BIT) | (1 << LED0_PA1_BIT) | (1 << LED1_PA7_BIT);
    DDRB |= (1 << HEARTBEAT_BIT);

    /* Start with all LEDs OFF */
    PORTA &= ~((1 << LED0_PA0_BIT) | (1 << LED0_PA1_BIT) | (1 << LED1_PA7_BIT));
    PORTB &= ~(1 << HEARTBEAT_BIT);

    /* 2. Configure PIR pins as inputs without pull-up (PIR modules output active HIGH/LOW) */
    DDRD  &= ~((1 << PIR0_BIT) | (1 << PIR1_BIT));
    PORTD &= ~((1 << PIR0_BIT) | (1 << PIR1_BIT));

    /* 3. Initialize UART (USB-to-TTL on PD0/PD1) */
    PORTD |= (1 << PD0); /* Enable pull-up on RXD to prevent noise when line is idle/floating */
    Serial.begin(9600);
    while (!Serial && millis() < 500) {
        /* Allow USB-to-TTL bridge lines to settle */
    }

    /* 4. Configure Hardware Interrupts INT0 & INT1 for RISING edge
          MCUCR:
            ISC01 = 1, ISC00 = 1  -> INT0 on rising edge
            ISC11 = 1, ISC10 = 1  -> INT1 on rising edge
          GICR:
            INT0 = 1, INT1 = 1    -> Enable external interrupts
    */
    MCUCR |= (1 << ISC01) | (1 << ISC00) | (1 << ISC11) | (1 << ISC10);
    GIFR  |= (1 << INTF0) | (1 << INTF1);  // Clear any existing interrupt flags
    GICR  |= (1 << INT0)  | (1 << INT1);   // Enable INT0 and INT1

    sei(); // Enable global interrupts

    /* 5. Print startup diagnostics banner */
    printBootBanner();
}

/* ---------- Main Loop ---------- */
void loop() {
    uint32_t now = millis();

    /* ----- 1. Heartbeat LED Toggle ----- */
    if (now - last_heartbeat_ms >= HEARTBEAT_INTERVAL) {
        last_heartbeat_ms = now;
        PORTB ^= (1 << HEARTBEAT_BIT);
    }

    /* ----- 2. Poll instantaneous pin levels ----- */
    bool pir0_is_high = (PIND & (1 << PIR0_BIT)) ? true : false;
    bool pir1_is_high = (PIND & (1 << PIR1_BIT)) ? true : false;

    /* Update indicator LEDs in real-time */
    if (pir0_is_high) {
        PORTA |= (1 << LED0_PA0_BIT) | (1 << LED0_PA1_BIT);
    } else {
        PORTA &= ~((1 << LED0_PA0_BIT) | (1 << LED0_PA1_BIT));
    }

    if (pir1_is_high) {
        PORTA |= (1 << LED1_PA7_BIT);
    } else {
        PORTA &= ~(1 << LED1_PA7_BIT);
    }

    /* ----- 3. Process PIR 0 Interrupt Edge & Level Transitions ----- */
    if (pir0_event_pending) {
        pir0_event_pending = false;
        formatTime(pir0_last_trigger_ms);
        Serial.print(F(" >>> [PIR 0 / INT0 TRIGGER] Motion Detected (Left Sector) | Total: #"));
        Serial.print(pir0_trigger_count);
        Serial.print(F(" | Pin: "));
        Serial.println(pir0_is_high ? F("HIGH") : F("PULSE"));
    }

    // PIR 0 Level State Change Detection
    if (pir0_is_high && !pir0_was_high) {
        pir0_high_since_ms = now;
        pir0_was_high = true;
    } else if (!pir0_is_high && pir0_was_high) {
        uint32_t duration = now - pir0_high_since_ms;
        formatTime(now);
        Serial.print(F(" --- [PIR 0 / INT0 CLEARED] Motion Ended | Line held HIGH for "));
        Serial.print(duration);
        Serial.println(F(" ms | Status: IDLE"));
        pir0_was_high = false;
    }

    /* ----- 4. Process PIR 1 Interrupt Edge & Level Transitions ----- */
    if (pir1_event_pending) {
        pir1_event_pending = false;
        formatTime(pir1_last_trigger_ms);
        Serial.print(F(" >>> [PIR 1 / INT1 TRIGGER] Motion Detected (Right Sector) | Total: #"));
        Serial.print(pir1_trigger_count);
        Serial.print(F(" | Pin: "));
        Serial.println(pir1_is_high ? F("HIGH") : F("PULSE"));
    }

    // PIR 1 Level State Change Detection
    if (pir1_is_high && !pir1_was_high) {
        pir1_high_since_ms = now;
        pir1_was_high = true;
    } else if (!pir1_is_high && pir1_was_high) {
        uint32_t duration = now - pir1_high_since_ms;
        formatTime(now);
        Serial.print(F(" --- [PIR 1 / INT1 CLEARED] Motion Ended | Line held HIGH for "));
        Serial.print(duration);
        Serial.println(F(" ms | Status: IDLE"));
        pir1_was_high = false;
    }

    /* ----- 5. Periodic Live Telemetry (Every 1s) ----- */
    if (verbose_stream && (now - last_telemetry_ms >= TELEMETRY_INTERVAL)) {
        last_telemetry_ms = now;

        formatTime(now);
        Serial.print(F(" [STATUS] PIR0(INT0): "));
        if (pir0_is_high) {
            Serial.print(F("[ACTIVE] "));
        } else {
            Serial.print(F("[ IDLE ] "));
        }
        Serial.print(F("(Trigs: "));
        Serial.print(pir0_trigger_count);
        Serial.print(F(") | PIR1(INT1): "));
        if (pir1_is_high) {
            Serial.print(F("[ACTIVE] "));
        } else {
            Serial.print(F("[ IDLE ] "));
        }
        Serial.print(F("(Trigs: "));
        Serial.print(pir1_trigger_count);
        Serial.print(F(") | Active: "));

        if (pir0_is_high && pir1_is_high) {
            Serial.print(F("[DUAL / BOTH SECTORS]"));
        } else if (pir0_is_high) {
            Serial.print(F("[SECTOR 0 / LEFT]"));
        } else if (pir1_is_high) {
            Serial.print(F("[SECTOR 1 / RIGHT]"));
        } else {
            Serial.print(F("[NONE]"));
        }
        Serial.println();
    }

    /* ----- 6. Handle Incoming Serial Commands from USB-to-TTL ----- */
    if (Serial.available() > 0) {
        processSerialCommands();
    }
}

/* ---------- Helper: Formatted Timestamp Output [MM:SS.mmm] ---------- */
void formatTime(uint32_t total_ms) {
    uint32_t total_sec = total_ms / 1000UL;
    uint32_t ms_remainder = total_ms % 1000UL;
    uint32_t minutes = total_sec / 60UL;
    uint32_t seconds = total_sec % 60UL;

    Serial.print(F("["));
    if (minutes < 10) Serial.print(F("0"));
    Serial.print(minutes);
    Serial.print(F(":"));
    if (seconds < 10) Serial.print(F("0"));
    Serial.print(seconds);
    Serial.print(F("."));
    if (ms_remainder < 100) Serial.print(F("0"));
    if (ms_remainder < 10)  Serial.print(F("0"));
    Serial.print(ms_remainder);
    Serial.print(F("]"));
}

/* ---------- Interactive Serial Commands ---------- */
void processSerialCommands(void) {
    char cmd = Serial.read();

    // Ignore non-printable ASCII or control characters (newlines, carriage returns, noise)
    if (cmd < 32 || cmd > 126) {
        return;
    }

    Serial.println();
    Serial.print(F(">> Command received: '"));
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
            printDetailedStatus();
            break;

        case 'v':
        case 'V':
            verbose_stream = !verbose_stream;
            Serial.print(F(">> Periodic 1s Telemetry Stream is now: "));
            Serial.println(verbose_stream ? F("ENABLED") : F("DISABLED (Events Only)"));
            break;

        case 'r':
        case 'R':
            resetStatistics();
            break;

        case 't':
        case 'T':
            runLedTest();
            break;

        case 'p':
        case 'P': {
            bool p0 = (PIND & (1 << PIR0_BIT)) ? true : false;
            bool p1 = (PIND & (1 << PIR1_BIT)) ? true : false;
            Serial.println(F(">> Instantaneous Pin Readout:"));
            Serial.print(F("   Pin 16 (PD2 / INT0 / PIR 0): "));
            Serial.println(p0 ? F("HIGH (3.3V/5V - Motion)") : F("LOW (0V - Idle)"));
            Serial.print(F("   Pin 17 (PD3 / INT1 / PIR 1): "));
            Serial.println(p1 ? F("HIGH (3.3V/5V - Motion)") : F("LOW (0V - Idle)"));
            break;
        }

        default:
            Serial.print(F(">> Unknown command '"));
            Serial.print(cmd);
            Serial.println(F("'. Press 'h' or '?' for available commands."));
            break;
    }
}

/* ---------- Boot Banner ---------- */
void printBootBanner(void) {
    Serial.println();
    Serial.println(F("=================================================================="));
    Serial.println(F("   PROJECT VANGUARD - DUAL PIR SURVEILLANCE & SECTOR MONITOR      "));
    Serial.println(F("=================================================================="));
    Serial.println(F(" MCU:           ATmega32A @ 8.0 MHz"));
    Serial.println(F(" Serial:        9600 Baud (8-N-1) on PD0(RXD)/PD1(TXD)"));
    Serial.println(F(" PIR Sensor 0:  ATmega32 Pin 16 (PD2 / INT0) -> Sector 0 (Left)"));
    Serial.println(F(" PIR Sensor 1:  ATmega32 Pin 17 (PD3 / INT1) -> Sector 1 (Right)"));
    Serial.println(F(" Indicators:    PA0/PA1 (PIR0 LED), PA7 (PIR1 LED), PB0 (Heartbeat)"));
    Serial.println(F("------------------------------------------------------------------"));
    Serial.println(F(" Type 'h' or '?' in monitor for interactive commands menu."));
    Serial.println(F(" System initialized. Listening for motion interrupts..."));
    Serial.println(F("=================================================================="));
    Serial.println();
}

/* ---------- Help Menu ---------- */
void printHelp(void) {
    Serial.println(F("========================= COMMAND MENU ========================="));
    Serial.println(F("  'h' or '?' : Display this interactive help menu"));
    Serial.println(F("  's'        : Detailed status report (trigger counts, durations)"));
    Serial.println(F("  'p'        : Probe instantaneous PIR pin logic levels"));
    Serial.println(F("  'v'        : Toggle periodic 1s telemetry stream (ON / OFF)"));
    Serial.println(F("  't'        : Run LED hardware test cycle (PA0, PA7, PB0)"));
    Serial.println(F("  'r'        : Reset trigger counters and timers to zero"));
    Serial.println(F("================================================================"));
}

/* ---------- Detailed Status Report ---------- */
void printDetailedStatus(void) {
    uint32_t now = millis();
    bool p0 = (PIND & (1 << PIR0_BIT)) ? true : false;
    bool p1 = (PIND & (1 << PIR1_BIT)) ? true : false;

    Serial.println(F("--------------------- SYSTEM STATUS SNAPSHOT ---------------------"));
    Serial.print(F(" Uptime:               "));
    formatTime(now);
    Serial.println();

    Serial.print(F(" PIR 0 (PD2 / INT0):   "));
    Serial.print(p0 ? F("ACTIVE (HIGH)") : F("IDLE (LOW)"));
    Serial.print(F(" | Total Triggers: "));
    Serial.print(pir0_trigger_count);
    if (pir0_last_trigger_ms > 0) {
        Serial.print(F(" | Last Triggered: "));
        Serial.print((now - pir0_last_trigger_ms) / 1000UL);
        Serial.print(F("s ago"));
    } else {
        Serial.print(F(" | Never triggered"));
    }
    Serial.println();

    Serial.print(F(" PIR 1 (PD3 / INT1):   "));
    Serial.print(p1 ? F("ACTIVE (HIGH)") : F("IDLE (LOW)"));
    Serial.print(F(" | Total Triggers: "));
    Serial.print(pir1_trigger_count);
    if (pir1_last_trigger_ms > 0) {
        Serial.print(F(" | Last Triggered: "));
        Serial.print((now - pir1_last_trigger_ms) / 1000UL);
        Serial.print(F("s ago"));
    } else {
        Serial.print(F(" | Never triggered"));
    }
    Serial.println();

    Serial.print(F(" Active Sector:        "));
    if (p0 && p1) {
        Serial.println(F("DUAL SECTOR DETECTION (Both Left & Right)"));
    } else if (p0) {
        Serial.println(F("SECTOR 0 (Left Only)"));
    } else if (p1) {
        Serial.println(F("SECTOR 1 (Right Only)"));
    } else {
        Serial.println(F("CLEAR (No Motion)"));
    }

    Serial.print(F(" Telemetry Streaming:  "));
    Serial.println(verbose_stream ? F("ENABLED (1s interval)") : F("DISABLED (Events only)"));
    Serial.println(F("------------------------------------------------------------------"));
}

/* ---------- LED Test Cycle ---------- */
void runLedTest(void) {
    Serial.println(F(">> Running LED Test Cycle..."));

    // Turn all off
    PORTA &= ~((1 << LED0_PA0_BIT) | (1 << LED0_PA1_BIT) | (1 << LED1_PA7_BIT));
    PORTB &= ~(1 << HEARTBEAT_BIT);

    Serial.println(F("   [1/3] Testing LED0 (PA0 / PA1) - PIR 0 Indicator..."));
    PORTA |= (1 << LED0_PA0_BIT) | (1 << LED0_PA1_BIT);
    delay(600);
    PORTA &= ~((1 << LED0_PA0_BIT) | (1 << LED0_PA1_BIT));
    delay(200);

    Serial.println(F("   [2/3] Testing LED1 (PA7) - PIR 1 Indicator..."));
    PORTA |= (1 << LED1_PA7_BIT);
    delay(600);
    PORTA &= ~(1 << LED1_PA7_BIT);
    delay(200);

    Serial.println(F("   [3/3] Testing Heartbeat LED (PB0)..."));
    for (int i = 0; i < 4; i++) {
        PORTB |= (1 << HEARTBEAT_BIT);
        delay(150);
        PORTB &= ~(1 << HEARTBEAT_BIT);
        delay(150);
    }

    Serial.println(F(">> LED Test Complete! Restoring normal indicator operation."));
}

/* ---------- Reset Counters ---------- */
void resetStatistics(void) {
    pir0_trigger_count = 0;
    pir1_trigger_count = 0;
    pir0_last_trigger_ms = 0;
    pir1_last_trigger_ms = 0;
    pir0_was_high = false;
    pir1_was_high = false;
    Serial.println(F(">> [RESET] All trigger counters and timer statistics reset to 0."));
}