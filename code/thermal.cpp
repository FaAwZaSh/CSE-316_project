/*
 * =====================================================================================
 *  PROJECT: GY-906 (MLX90614) Non-Contact Infrared Thermal Sensor Monitor
 * =====================================================================================
 *  MCU:           ATmega32A (16.0 MHz External Crystal)
 *  Baud Rate:     9600 Baud (8-N-1) on PD0(RXD) / PD1(TXD)
 *  Sensor:        GY-906 MLX90614 Contactless IR Thermometer (I2C: 0x5A)
 *  Alert Limit:   Object Temp >= 32.0 °C (or Delta >= 2.0 °C above Ambient)
 * =====================================================================================
 * 
 *  ATMEGA32A CONNECTION CONFIGURATION (DIP-40 PINOUT):
 *  ---------------------------------------------------
 * 
 *                     +---[ \_/ ]---+
 *     (Heartbeat)PB0 1 |             | 40  PA0
 *                PB1 2 |             | 39  PA1
 *                PB2 3 |             | 38  PA2
 *                PB3 4 |             | 37  PA3
 *                PB4 5 |             | 36  PA4
 *                PB5 6 |   ATmega32A | 35  PA5
 *                PB6 7 |    DIP-40   | 34  PA6
 *                PB7 8 |             | 33  PA7 (High Temp Alert LED)
 *             !RESET 9 |             | 32  AREF
 *                VCC 10|             | 31  GND
 *                GND 11|             | 30  AVCC
 *              XTAL2 12|             | 29  PC7
 *              XTAL1 13|             | 28  PC6
 *         (RXD)  PD0 14|             | 27  PC5
 *         (TXD)  PD1 15|             | 26  PC4
 *                PD2 16|             | 25  PC3
 *                PD3 17|             | 24  PC2
 *                PD4 18|             | 23  PC1 (SDA - Sensor Data Line)
 *                PD5 19|             | 22  PC0 (SCL - Sensor Clock Line)
 *                PD6 20|             | 21  PD7
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
 *  3. GY-906 (MLX90614) Thermal Sensor (I2C / SMBus Interface):
 *     - VIN / VCC       --> +5V or +3.3V
 *     - GND             --> Common GND
 *     - SCL             --> ATmega32 Pin 22 (PC0 / SCL)  [+ 4.7k pull-up to +5V]
 *     - SDA             --> ATmega32 Pin 23 (PC1 / SDA)  [+ 4.7k pull-up to +5V]
 * 
 *  4. (Optional) Diagnostic LEDs:
 *     - Heartbeat LED   --> ATmega32 Pin 1  (PB0) -> 330 ohm resistor -> GND
 *     - Alert LED (+)   --> ATmega32 Pin 33 (PA7) -> 330 ohm resistor -> GND
 * =====================================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <util/delay.h>

// Sensor I2C Address and RAM Registers
#define MLX90614_I2CADDR  0x5A
#define MLX90614_TA       0x06  // Ambient Temperature Register
#define MLX90614_TOBJ1    0x07  // Object 1 Temperature Register

// Hardware Pins
#define HEARTBEAT_LED_BIT PB0   // Pin 1: Heartbeat LED
#define ALERT_LED_BIT     PA7   // Pin 33: High Temp Alert LED

// Alert Thresholds (Human Body Heat Signature)
#define HEAT_THRESHOLD_C  32.0f // Object temperature threshold in Celsius
#define DELTA_THRESHOLD_C 2.0f  // Object elevated above Ambient by 2.0 °C

// Filtered object temperature
float filtered_obj_c = 0.0f;
bool first_temp_read = true;

// I2C Bus Recovery Sequence
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

// Read raw 16-bit temperature from MLX90614 RAM over SMBus
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

  // Check error bit
  if (msb & 0x80) {
    return -999.0f;
  }

  uint16_t tempRaw = ((uint16_t)msb << 8) | lsb;
  return ((float)tempRaw * 0.02f) - 273.15f;
}

void setup() {
  // 1. Configure Alert and Heartbeat LEDs
  DDRB |= (1 << HEARTBEAT_LED_BIT);
  PORTB |= (1 << HEARTBEAT_LED_BIT);

  DDRA |= (1 << ALERT_LED_BIT);
  PORTA &= ~(1 << ALERT_LED_BIT); // Alert LED OFF

  // 2. Configure USART @ 9600 Baud with pull-up on RXD
  PORTD |= (1 << PD0);
  Serial.begin(9600);
  delay(100);

  Serial.println();
  Serial.println(F("=================================================================="));
  Serial.println(F("   ATmega32A GY-906 (MLX90614) INFRARED THERMAL SENSOR MONITOR    "));
  Serial.println(F("=================================================================="));
  Serial.println(F(" Clock : 16.0 MHz | Baud: 9600 (PD0=RXD, PD1=TXD)                "));
  Serial.println(F(" I2C   : Pin 22 (PC0=SCL), Pin 23 (PC1=SDA)                       "));
  Serial.println(F(" Addr  : 0x5A | Alert: Obj >= 32.0 C or Delta >= +2.0 C           "));
  Serial.println(F("=================================================================="));
  Serial.print(F("Initializing I2C bus & probing GY-906 at 0x5A... "));
  Serial.flush();

  // 3. I2C Bus Recovery & Init
  i2c_bus_recovery();
  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_TIMEOUT)
  Wire.setWireTimeout(0); // Disable timeout to allow SMBus clock stretching
#endif
  PORTC |= (1 << PC0) | (1 << PC1); // Enable internal pull-ups

  // 4. Test initial read
  float test_amb = readMLX90614TempC(MLX90614_TA);
  if (test_amb > -100.0f) {
    Serial.println(F("[SUCCESS] Sensor Online!"));
    Serial.println(F("Starting real-time temperature measurements..."));
  } else {
    Serial.println(F("[FAILED!] Check wiring: SCL(Pin 22), SDA(Pin 23), 4.7k pullups."));
  }
  Serial.println(F("------------------------------------------------------------------"));
  Serial.flush();
}

void loop() {
  // Toggle Heartbeat LED
  PORTB ^= (1 << HEARTBEAT_LED_BIT);

  float amb_c = readMLX90614TempC(MLX90614_TA);
  float obj_c = readMLX90614TempC(MLX90614_TOBJ1);

  if (amb_c > -100.0f && obj_c > -100.0f) {
    // Exponential Moving Average filter on object temperature (smooth noise)
    if (first_temp_read) {
      filtered_obj_c = obj_c;
      first_temp_read = false;
    } else {
      filtered_obj_c = (0.35f * obj_c) + (0.65f * filtered_obj_c);
    }

    float amb_f = (amb_c * 1.8f) + 32.0f;
    float obj_f = (filtered_obj_c * 1.8f) + 32.0f;
    float delta_c = filtered_obj_c - amb_c;

    // Check if heat signature / elevated temperature is detected
    bool heat_alert = (filtered_obj_c >= HEAT_THRESHOLD_C) || (delta_c >= DELTA_THRESHOLD_C);

    if (heat_alert) {
      PORTA |= (1 << ALERT_LED_BIT); // Alert LED ON
      Serial.print(F("[!] >>> HEAT SIGNATURE DETECTED! "));
    } else {
      PORTA &= ~(1 << ALERT_LED_BIT); // Alert LED OFF
      Serial.print(F("[THERMAL] "));
    }

    Serial.print(F("Amb: "));
    Serial.print(amb_c, 1);
    Serial.print(F(" C ("));
    Serial.print(amb_f, 1);
    Serial.print(F(" F) | Obj: "));
    Serial.print(filtered_obj_c, 1);
    Serial.print(F(" C ("));
    Serial.print(obj_f, 1);
    Serial.print(F(" F) | Delta: "));
    if (delta_c >= 0) Serial.print(F("+"));
    Serial.print(delta_c, 1);
    Serial.print(F(" C"));

    if (heat_alert) {
      Serial.print(F(" <<<"));
    }
    Serial.println();
  } else {
    PORTA &= ~(1 << ALERT_LED_BIT);
    Serial.println(F("[ERROR] Failed to read from MLX90614!"));
  }

  delay(250); // Sample rate: ~4 readings per second
}
