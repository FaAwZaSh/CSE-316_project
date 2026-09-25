/*
 * =====================================================================================
 *  PROJECT: Dual PIR Motion Detector with USART Telemetry
 * =====================================================================================
 *  MCU:           ATmega32A (16.0 MHz External Crystal)
 *  Baud Rate:     9600 Baud (8-N-1)
 * =====================================================================================
 * 
 *  ATMEGA32A CONNECTION CONFIGURATION (DIP-40 PINOUT):
 *  ---------------------------------------------------
 * 
 *                     +---[ \_/ ]---+
 *     (XCK/T0) PB0  1 |             | 40  PA0 (ADC0)
 *         (T1) PB1  2 |             | 39  PA1 (ADC1)
 *       (INT2) PB2  3 |             | 38  PA2 (ADC2)
 *        (OC0) PB3  4 |             | 37  PA3 (ADC3)
 *        (!SS) PB4  5 |             | 36  PA4 (ADC4)
 *       (MOSI) PB5  6 |   ATmega32A | 35  PA5 (ADC5)
 *       (MISO) PB6  7 |    DIP-40   | 34  PA6 (ADC6)
 *        (SCK) PB7  8 |             | 33  PA7 (ADC7)
 *       !RESET     9 |             | 32  AREF
 *          VCC    10 |             | 31  GND
 *          GND    11 |             | 30  AVCC
 *        XTAL2    12 |             | 29  PC7 (TOSC2)
 *        XTAL1    13 |             | 28  PC6 (TOSC1)
 *   (RXD)  PD0    14 |             | 27  PC5 (TDI)
 *   (TXD)  PD1    15 |             | 26  PC4 (TDO)
 *  (INT0)  PD2    16 |             | 25  PC3 (TMS)
 *  (INT1)  PD3    17 |             | 24  PC2 (TCK)
 *  (OC1B)  PD4    18 |             | 23  PC1 (SDA)
 *  (OC1A)  PD5    19 |             | 22  PC0 (SCL)
 *  (ICP1)  PD6    20 |             | 21  PD7 (OC2)
 *                     +-------------+
 * 
 *  HARDWARE WIRING DETAILS:
 *  ------------------------
 *  1. Power & Clock:
 *     - Pin 10 (VCC)    --> +5V
 *     - Pin 11 (GND)    --> Common GND
 *     - Pin 30 (AVCC)   --> +5V
 *     - Pin 31 (GND)    --> Common GND
 *     - Pin 9  (!RESET) --> 10k resistor pull-up to +5V
 *     - Pin 12 (XTAL2)  --> 16MHz Crystal + 22pF cap to GND
 *     - Pin 13 (XTAL1)  --> 16MHz Crystal + 22pF cap to GND
 * 
 *  2. USB-to-TTL Serial Bridge (PL2303 / CP2102 / CH340 / FTDI):
 *     - USB-TTL TXD     --> ATmega32 Pin 14 (PD0 / RXD)
 *     - USB-TTL RXD     --> ATmega32 Pin 15 (PD1 / TXD)
 *     - USB-TTL GND     --> Common GND (Mandatory)
 *     - Baud Rate       --> 9600 Baud (8-N-1)
 * 
 *  3. PIR Sensors (HC-SR501 / AM312):
 *     - PIR 1 (Left Sector):
 *         * VCC         --> +5V
 *         * GND         --> Common GND
 *         * OUT (Signal)--> ATmega32 Pin 16 (PD2 / INT0)
 * 
 *     - PIR 2 (Right Sector):
 *         * VCC         --> +5V
 *         * GND         --> Common GND
 *         * OUT (Signal)--> ATmega32 Pin 17 (PD3 / INT1)
 * 
 *  4. (Optional) Diagnostic LEDs:
 *     - PIR 1 Motion LED --> ATmega32 Pin 40 (PA0) -> 330 ohm resistor -> GND
 *     - PIR 2 Motion LED --> ATmega32 Pin 33 (PA7) -> 330 ohm resistor -> GND
 *     - Heartbeat LED    --> ATmega32 Pin 1  (PB0) -> 330 ohm resistor -> GND
 * =====================================================================================
 */

#include <Arduino.h>
#include <avr/io.h>

// PIR Pin Definitions (Port D bit positions on ATmega32)
#define PIR1_BIT    PD2    // Pin 16 (PIR 1 - Left)
#define PIR2_BIT    PD3    // Pin 17 (PIR 2 - Right)

// Previous state tracking for edge detection
bool pir1_last_state = false;
bool pir2_last_state = false;

// Trigger event counters
uint32_t pir1_trigger_count = 0;
uint32_t pir2_trigger_count = 0;

// Periodic status timer
uint32_t last_status_time = 0;
const uint32_t STATUS_INTERVAL_MS = 2000; // Log status every 2 seconds

void setup() {
  // 1. Configure PIR pins as inputs (DDR = 0)
  DDRD &= ~((1 << PIR1_BIT) | (1 << PIR2_BIT));
  // Keep internal pull-ups disabled so PIR output determines logic level (LOW = 0V, HIGH = 3.3V/5V)
  PORTD &= ~((1 << PIR1_BIT) | (1 << PIR2_BIT));

  // 2. Enable internal pull-up on RXD (PD0) to suppress floating serial noise
  PORTD |= (1 << PD0);

  // 3. Initialize USART at 9600 Baud
  Serial.begin(9600);
  delay(200);

  // 4. Print Startup Banner
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("   ATmega32A Dual PIR Motion Monitor   "));
  Serial.println(F("========================================"));
  Serial.println(F(" USART : 9600 Baud (PD0=RXD, PD1=TXD)"));
  Serial.println(F(" PIR 1 : Pin 16 (PD2)"));
  Serial.println(F(" PIR 2 : Pin 17 (PD3)"));
  Serial.println(F(" System initialized and listening..."));
  Serial.println(F("========================================"));
  Serial.println();
}

void loop() {
  uint32_t now = millis();

  // Read current logic level of both PIR sensor pins
  bool pir1_current = (PIND & (1 << PIR1_BIT)) != 0;
  bool pir2_current = (PIND & (1 << PIR2_BIT)) != 0;

  // --- Check PIR 1 (Pin 16 / PD2) ---
  if (pir1_current != pir1_last_state) {
    pir1_last_state = pir1_current;
    if (pir1_current) {
      pir1_trigger_count++;
      Serial.print(F(">>> [PIR 1] Motion DETECTED! (Count: "));
      Serial.print(pir1_trigger_count);
      Serial.println(F(")"));
    } else {
      Serial.println(F("--- [PIR 1] Motion Ended (IDLE)"));
    }
  }

  // --- Check PIR 2 (Pin 17 / PD3) ---
  if (pir2_current != pir2_last_state) {
    pir2_last_state = pir2_current;
    if (pir2_current) {
      pir2_trigger_count++;
      Serial.print(F(">>> [PIR 2] Motion DETECTED! (Count: "));
      Serial.print(pir2_trigger_count);
      Serial.println(F(")"));
    } else {
      Serial.println(F("--- [PIR 2] Motion Ended (IDLE)"));
    }
  }

  // --- Periodic USART Status Log (Every 2 seconds) ---
  if (now - last_status_time >= STATUS_INTERVAL_MS) {
    last_status_time = now;
    Serial.print(F("[STATUS] Uptime: "));
    Serial.print(now / 1000);
    Serial.print(F("s | PIR 1 (Pin 16): "));
    Serial.print(pir1_current ? F("ACTIVE [HIGH]") : F("IDLE  [LOW] "));
    Serial.print(F(" | PIR 2 (Pin 17): "));
    Serial.println(pir2_current ? F("ACTIVE [HIGH]") : F("IDLE  [LOW] "));
  }

  // Small delay for contact debounce
  delay(30);
}