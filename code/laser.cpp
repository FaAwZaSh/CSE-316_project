/*
 * =====================================================================================
 *  PROJECT: VL53L0X Laser Distance Sensor with USART Telemetry & Proximity Lock
 * =====================================================================================
 *  MCU:           ATmega32A (16.0 MHz External Crystal)
 *  Baud Rate:     9600 Baud (8-N-1) on PD0(RXD) / PD1(TXD)
 *  Sensor:        VL53L0X Time-of-Flight (ToF) Laser Sensor (I2C: 0x29)
 *  Lock Limit:    <= 200 mm (Triggers TARGET LOCKED log)
 * =====================================================================================
 * 
 *  ATMEGA32A CONNECTION CONFIGURATION (DIP-40 PINOUT):
 *  ---------------------------------------------------
 * 
 *                     +---[ \_/ ]---+
 *     (XCK/T0) PB0  1 |             | 40  PA0 (XSHUT - Laser Reset / Wakeup)
 *         (T1) PB1  2 |             | 39  PA1 (ADC1)
 *       (INT2) PB2  3 |             | 38  PA2 (ADC2)
 *        (OC0) PB3  4 |             | 37  PA3 (ADC3)
 *        (!SS) PB4  5 |             | 36  PA4 (ADC4)
 *       (MOSI) PB5  6 |   ATmega32A | 35  PA5 (ADC5)
 *       (MISO) PB6  7 |    DIP-40   | 34  PA6 (ADC6)
 *        (SCK) PB7  8 |             | 33  PA7 (Alert LED - Optional)
 *       !RESET     9 |             | 32  AREF
 *          VCC    10 |             | 31  GND
 *          GND    11 |             | 30  AVCC
 *        XTAL2    12 |             | 29  PC7 (TOSC2)
 *        XTAL1    13 |             | 28  PC6 (TOSC1)
 *   (RXD)  PD0    14 |             | 27  PC5 (TDI)
 *   (TXD)  PD1    15 |             | 26  PC4 (TDO)
 *  (INT0)  PD2    16 |             | 25  PC3 (TMS)
 *  (INT1)  PD3    17 |             | 24  PC2 (TCK)
 *  (OC1B)  PD4    18 |             | 23  PC1 (SDA - Laser Data Line)
 *  (OC1A)  PD5    19 |             | 22  PC0 (SCL - Laser Clock Line)
 *  (ICP1)  PD6    20 |             | 21  PD7 (OC2)
 *                     +-------------+
 * 
 *  HARDWARE WIRING DETAILS:
 *  ------------------------
 *  1. Power & Clock:
 *     - Pin 10 (VCC)    --> +5V
 *     - Pin 11 (GND)    --> Common GND
 *     - Pin 30 (AVCC)   --> +5V (Mandatory for Port A)
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
 *  3. VL53L0X Laser Distance Sensor (I2C Interface):
 *     - VIN / VCC       --> +5V (or +3.3V)
 *     - GND             --> Common GND
 *     - SCL             --> ATmega32 Pin 22 (PC0 / SCL)  [+ 4.7k pull-up to +5V]
 *     - SDA             --> ATmega32 Pin 23 (PC1 / SDA)  [+ 4.7k pull-up to +5V]
 *     - XSHUT           --> ATmega32 Pin 40 (PA0)        [Reset / Active HIGH Enable]
 *     - GPIO1           --> Not connected (leave open)
 * 
 *  4. (Optional) Target Lock Alert LED:
 *     - Alert LED (+)   --> ATmega32 Pin 33 (PA7) -> 330 ohm resistor -> GND
 * =====================================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <util/delay.h>
#include "Adafruit_VL53L0X.h"

// Hardware Pin Definitions
#define LASER_XSHUT_BIT PA0   // Pin 40: VL53L0X Reset / Enable
#define ALERT_LED_BIT   PA7   // Pin 33: Optional Target Lock Alert LED
#define LOCK_THRESHOLD_MM 200 // Lock distance threshold: <= 200 mm (~20 cm)

// Sensor instance
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// I2C Bus Clock Recovery Sequence
void i2c_bus_recovery(void) {
  PORTC &= ~((1 << PC0) | (1 << PC1));
  DDRC &= ~((1 << PC0) | (1 << PC1));
  _delay_us(10);
  for (uint8_t i = 0; i < 9; i++) {
    DDRC |= (1 << PC0);
    _delay_us(10);
    DDRC &= ~(1 << PC0);
    _delay_us(10);
  }
  DDRC |= (1 << PC0); _delay_us(10);
  DDRC |= (1 << PC1); _delay_us(10);
  DDRC &= ~(1 << PC0); _delay_us(10);
  DDRC &= ~(1 << PC1); _delay_us(10);
  PORTC |= (1 << PC0) | (1 << PC1);
}

void setup() {
  // 1. Configure Alert LED and XSHUT
  DDRA |= (1 << ALERT_LED_BIT) | (1 << LASER_XSHUT_BIT);
  PORTA &= ~(1 << ALERT_LED_BIT); // Alert LED OFF

  // 2. Hardware reset cycle for VL53L0X with 250ms MCU boot delay
  PORTA &= ~(1 << LASER_XSHUT_BIT);
  delay(50);
  PORTA |= (1 << LASER_XSHUT_BIT);
  delay(250);

  // 3. Initialize UART at 9600 Baud with internal pull-up on RXD
  PORTD |= (1 << PD0);
  Serial.begin(9600);
  delay(100);

  Serial.println();
  Serial.println(F("=================================================================="));
  Serial.println(F("   ATmega32A VL53L0X LASER DISTANCE SENSOR MONITOR                "));
  Serial.println(F("=================================================================="));
  Serial.println(F(" Clock : 16.0 MHz | Baud: 9600 (PD0=RXD, PD1=TXD)                "));
  Serial.println(F(" I2C   : Pin 22 (PC0=SCL), Pin 23 (PC1=SDA)                       "));
  Serial.println(F(" Limit : <= 200 mm triggers TARGET LOCKED                         "));
  Serial.println(F("=================================================================="));
  Serial.print(F("Initializing VL53L0X Laser Sensor at 0x29... "));
  Serial.flush();

  // 4. Recover I2C bus & initialize Wire
  i2c_bus_recovery();
  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  PORTC |= (1 << PC0) | (1 << PC1);

  // 5. Initialize Adafruit VL53L0X driver
  if (lox.begin(0x29, false, &Wire)) {
    lox.setMeasurementTimingBudgetMicroSeconds(100000);
    Serial.println(F("[SUCCESS] Ready!"));
    Serial.println(F("Starting real-time distance measurements..."));
  } else {
    Serial.println(F("[FAILED!] Check wiring: SCL(Pin 22), SDA(Pin 23), 4.7k pullups."));
  }
  Serial.println(F("------------------------------------------------------------------"));
  Serial.flush();
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  // Take distance measurement
  lox.rangingTest(&measure, false);

  // RangeStatus != 4 indicates a valid distance measurement (4 = Out of Range / Phase Fail)
  if (measure.RangeStatus != 4 && measure.RangeMilliMeter >= 15 && measure.RangeMilliMeter <= 2500) {
    uint16_t dist_mm = measure.RangeMilliMeter;

    // Target Lock Check: <= 200 mm
    if (dist_mm <= LOCK_THRESHOLD_MM) {
      PORTA |= (1 << ALERT_LED_BIT); // Alert LED ON
      Serial.print(F("[!] >>> TARGET LOCKED! Distance: "));
      Serial.print(dist_mm);
      Serial.print(F(" mm ("));
      Serial.print(dist_mm / 10.0f, 1);
      Serial.println(F(" cm) - VERY CLOSE (<= 200mm)! <<<"));
    } else {
      PORTA &= ~(1 << ALERT_LED_BIT); // Alert LED OFF
      Serial.print(F("[SCAN] Distance: "));
      Serial.print(dist_mm);
      Serial.print(F(" mm ("));
      Serial.print(dist_mm / 10.0f, 1);
      Serial.println(F(" cm) | Status: CLEAR"));
    }
  } else {
    PORTA &= ~(1 << ALERT_LED_BIT);
    Serial.println(F("[SCAN] Distance: Out of Range / CLEAR"));
  }

  // Real-time measurement delay
  delay(150);
}
