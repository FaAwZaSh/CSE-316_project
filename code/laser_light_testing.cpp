/*
 * =====================================================================================
 *  PROJECT VANGUARD: HW-493 650nm Red Laser Diode Module Testbench
 * =====================================================================================
 *  MCU:               ATmega32 / ATmega32A (16.0 MHz External Crystal)
 *  Baud Rate:         9600 Baud (8-N-1) over USB-to-TTL (PD0/PD1)
 *  Target Hardware:   HW-493 (KY-008 clone) 650nm 5V Red Laser Transmitter Module
 * =====================================================================================
 * 
 *  CAN I CHANGE THE COLOR OF THE HW-493 LASER?
 *  ------------------------------------------------------------------------------------
 *  NO. You CANNOT change the color of this laser module.
 * 
 *  TECHNICAL REASON:
 *  - Unlike RGB LEDs (which have three distinct colored dies inside that mix light),
 *    a semiconductor laser diode produces monochromatic (single-wavelength) coherent
 *    light governed strictly by the bandgap energy of its physical crystal material:
 *    Eg = (h * c) / lambda. For InGaAlP diodes, this is fixed at ~650 nm (Visible Red).
 *  - Applying PWM or changing the voltage will only alter the brightness / power output
 *    (or cause it to flicker below its threshold current); it CANNOT change the wavelength.
 *  - To get a different color (e.g., Green 520nm/532nm or Blue 450nm), you need a
 *    physically different laser emitter module.
 * 
 * =====================================================================================
 *  ATMEGA32A CONNECTION CONFIGURATION (DIP-40 PINOUT):
 * =====================================================================================
 * 
 *                     +---[ \_/ ]---+
 *     (Heartbeat)PB0 1 |             | 40  PA0
 *                PB1 2 |             | 39  PA1
 *                PB2 3 |             | 38  PA2
 *                PB3 4 |             | 37  PA3
 *                PB4 5 |             | 36  PA4
 *                PB5 6 |   ATmega32A | 35  PA5
 *                PB6 7 |    DIP-40   | 34  PA6
 *                PB7 8 |             | 33  PA7 (LASER PIN - Primary Output)
 *             !RESET 9 |             | 32  AREF
 *                VCC 10|             | 31  GND (Common Ground)
 *                GND 11|             | 30  AVCC (+5V Power)
 *              XTAL2 12|             | 29  PC7
 *              XTAL1 13|             | 28  PC6
 *         (RXD)  PD0 14|             | 27  PC5
 *         (TXD)  PD1 15|             | 26  PC4
 *                PD2 16|             | 25  PC3
 *                PD3 17|             | 24  PC2
 *                PD4 18|             | 23  PC1
 *                PD5 19|             | 22  PC0
 *                PD6 20|             | 21  PD7 (LASER PIN - Mirrored Output)
 *                     +-------------+
 * 
 * =====================================================================================
 *  HW-493 3-PIN LASER MODULE CONNECTION DIAGRAM:
 * =====================================================================================
 * 
 *         +--------------------------------+
 *         |     HW-493 / KY-008 LASER      |
 *         |   [ Brass Laser Diode Emitter ]|
 *         |                                |
 *         |     [ S ]     [   ]     [ - ]  |
 *         +-------+---------+---------+----+
 *                 |         |         |
 *                 |     (Middle)      |
 *                 |      No Connect   +--> Connect to ATmega32 Common GND (Pin 11 or 31)
 *                 |      (Unused)
 *                 +----------------------> Connect to ATmega32 PA7 (Pin 33) [or PD7 (Pin 21)]
 * 
 *  WIRING SUMMARY TABLE:
 *  +-------------------+--------------------+------------------------------------------+
 *  | HW-493 Pin Label  | ATmega32 DIP-40    | Function / Description                   |
 *  +-------------------+--------------------+------------------------------------------+
 *  | "S" (Signal / +)  | Pin 33 (PA7)       | Digital 5V Control (HIGH = ON, LOW = OFF)|
 *  | Middle Pin        | Not Connected (NC) | Leave floating / Unused                  |
 *  | "-" (Ground)      | Pin 11 or Pin 31   | Common System Ground (GND)               |
 *  +-------------------+--------------------+------------------------------------------+
 * 
 *  SAFETY WARNING:
 *  - This is a Class 3R / IIIa 650nm Red Laser (<= 5mW).
 *  - NEVER look directly into the laser beam or aim it at eyes or reflective surfaces!
 * =====================================================================================
 */

#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>

// ---------- Pin Definitions ----------
#define LASER_PRIMARY_PIN     PA7   // Pin 33 on ATmega32 DIP-40
#define LASER_MIRROR_PIN      PD7   // Pin 21 on ATmega32 DIP-40 (Mirrored output)
#define LED_INDICATOR         PB0   // Pin 1  (Onboard Heartbeat / Status LED)

// ---------- Operating Modes ----------
enum LaserMode {
  MODE_AUTO_DEMO,     // Cycles: Solid ON -> Strobe -> Beacon -> OFF
  MODE_ALWAYS_ON,     // Steady continuous beam
  MODE_ALWAYS_OFF,    // Laser disabled
  MODE_FAST_STROBE,   // 10 Hz tactical targeting strobe
  MODE_SLOW_BEACON    // 2 Hz slow warning blink
};

LaserMode current_mode = MODE_AUTO_DEMO;

// ---------- Helper Functions ----------
void setLaser(bool state) {
  if (state) {
    PORTA |= (1 << LASER_PRIMARY_PIN);
    PORTD |= (1 << LASER_MIRROR_PIN);
    PORTB |= (1 << LED_INDICATOR);
  } else {
    PORTA &= ~(1 << LASER_PRIMARY_PIN);
    PORTD &= ~(1 << LASER_MIRROR_PIN);
    PORTB &= ~(1 << LED_INDICATOR);
  }
}

void printHelp() {
  Serial.println(F("\r\n=========================================="));
  Serial.println(F("    HW-493 650nm Laser Diode Testbench    "));
  Serial.println(F("=========================================="));
  Serial.println(F("MCU     : ATmega32 @ 16.0 MHz"));
  Serial.println(F("Signal  : Pin 33 (PA7) & Pin 21 (PD7)"));
  Serial.println(F("Ground  : Pin 11 / 31 (GND)"));
  Serial.println(F("Baud    : 9600"));
  Serial.println(F("------------------------------------------"));
  Serial.println(F("Interactive Serial Commands:"));
  Serial.println(F("  '1' or 'o' -> Laser Solid ON"));
  Serial.println(F("  '0' or 'f' -> Laser Solid OFF"));
  Serial.println(F("  's'        -> Fast Tactical Strobe (10 Hz)"));
  Serial.println(F("  'b'        -> Slow Beacon Blink (2 Hz)"));
  Serial.println(F("  'p'        -> Single 500 ms Pulse"));
  Serial.println(F("  'd'        -> Return to Auto Demo Cycle"));
  Serial.println(F("  'h'        -> Show this Menu"));
  Serial.println(F("==========================================\r\n"));
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 1500);

  // Configure Laser Output Pins
  DDRA |= (1 << LASER_PRIMARY_PIN);
  DDRD |= (1 << LASER_MIRROR_PIN);
  DDRB |= (1 << LED_INDICATOR);

  // Ensure Laser starts OFF
  setLaser(false);

  printHelp();
  Serial.println(F("[INIT] Hardware ready. Starting Auto Demo Cycle...\r\n"));
}

void loop() {
  // Check for incoming keyboard commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == '\r' || cmd == '\n') return;
    while (Serial.available()) Serial.read(); // Flush buffer

    switch (cmd) {
      case '1':
      case 'o':
      case 'O':
        current_mode = MODE_ALWAYS_ON;
        setLaser(true);
        Serial.println(F("[COMMAND] Laser: SOLID ON"));
        break;

      case '0':
      case 'f':
      case 'F':
        current_mode = MODE_ALWAYS_OFF;
        setLaser(false);
        Serial.println(F("[COMMAND] Laser: SOLID OFF"));
        break;

      case 's':
      case 'S':
        current_mode = MODE_FAST_STROBE;
        Serial.println(F("[COMMAND] Mode: FAST TACTICAL STROBE (10 Hz)"));
        break;

      case 'b':
      case 'B':
        current_mode = MODE_SLOW_BEACON;
        Serial.println(F("[COMMAND] Mode: SLOW BEACON (2 Hz)"));
        break;

      case 'p':
      case 'P':
        Serial.println(F("[COMMAND] Single Pulse (500 ms)..."));
        setLaser(true);
        delay(500);
        setLaser(false);
        Serial.println(F("[COMMAND] Pulse Done."));
        break;

      case 'd':
      case 'D':
        current_mode = MODE_AUTO_DEMO;
        Serial.println(F("[COMMAND] Returned to AUTO DEMO CYCLE"));
        break;

      case 'h':
      case 'H':
        printHelp();
        break;

      default:
        Serial.print(F("[UNKNOWN] '"));
        Serial.print(cmd);
        Serial.println(F("' - Type 'h' for help menu."));
        break;
    }
  }

  // Execute current mode
  if (current_mode == MODE_AUTO_DEMO) {
    // 1. Solid ON for 2.0s
    Serial.println(F(">>> [DEMO] Stage 1: Solid Beam ON (2.0s)..."));
    setLaser(true);
    for (int i = 0; i < 20; i++) {
      delay(100);
      if (Serial.available()) return;
    }

    // 2. Fast Tactical Strobe for 1.5s
    Serial.println(F(">>> [DEMO] Stage 2: Tactical Fast Strobe (1.5s)..."));
    for (int i = 0; i < 15; i++) {
      setLaser(true);
      delay(50);
      setLaser(false);
      delay(50);
      if (Serial.available()) return;
    }

    // 3. Slow Beacon for 1.5s
    Serial.println(F(">>> [DEMO] Stage 3: Slow Beacon Blink (1.5s)..."));
    for (int i = 0; i < 3; i++) {
      setLaser(true);
      delay(250);
      setLaser(false);
      delay(250);
      if (Serial.available()) return;
    }

    // 4. Laser OFF Cooldown for 1.5s
    Serial.println(F(">>> [DEMO] Stage 4: Laser Beam OFF (1.5s Cooldown)...\r\n"));
    setLaser(false);
    for (int i = 0; i < 15; i++) {
      delay(100);
      if (Serial.available()) return;
    }
  }
  else if (current_mode == MODE_FAST_STROBE) {
    setLaser(true);
    delay(50);
    setLaser(false);
    delay(50);
  }
  else if (current_mode == MODE_SLOW_BEACON) {
    setLaser(true);
    delay(250);
    setLaser(false);
    delay(250);
  }
  else {
    delay(20);
  }
}
