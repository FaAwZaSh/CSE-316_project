/*
 * =====================================================================================
 *  PROJECT: Combined Dual-Sensor Test - VL53L0X Laser ToF + GY-906 (MLX90614) Thermal IR
 * =====================================================================================
 *  MCU:           ATmega32A (16.0 MHz External Crystal)
 *  Baud Rate:     9600 Baud (8-N-1) on PD0(RXD) / PD1(TXD)
 *  I2C Bus:       SCL = Pin 22 (PC0), SDA = Pin 23 (PC1) @ 100 kHz
 *  Sensors:       1. VL53L0X Laser Distance Sensor (I2C: 0x29, XSHUT: PA0)
 *                 2. GY-906 MLX90614 Contactless IR Thermometer (I2C: 0x5A)
 *  Alert Trigger: Distance <= 200 mm OR Object Temp >= 32.0 °C (Delta >= 2.0 °C)
 * =====================================================================================
 * 
 *  ATMEGA32A CONNECTION CONFIGURATION (DIP-40 PINOUT):
 *  ---------------------------------------------------
 * 
 *                     +---[ \_/ ]---+
 *     (Heartbeat)PB0 1 |             | 40  PA0 (XSHUT - Laser Reset / Wakeup)
 *                PB1 2 |             | 39  PA1
 *                PB2 3 |             | 38  PA2
 *                PB3 4 |             | 37  PA3
 *                PB4 5 |             | 36  PA4
 *                PB5 6 |   ATmega32A | 35  PA5
 *                PB6 7 |    DIP-40   | 34  PA6
 *                PB7 8 |             | 33  PA7 (Target Lock / Heat Alert LED)
 *             !RESET 9 |             | 32  AREF
 *                VCC 10|             | 31  GND
 *                GND 11|             | 30  AVCC
 *              XTAL2 12|             | 29  PC7
 *              XTAL1 13|             | 28  PC6
 *         (RXD)  PD0 14|             | 27  PC5
 *         (TXD)  PD1 15|             | 26  PC4
 *                PD2 16|             | 25  PC3
 *                PD3 17|             | 24  PC2
 *                PD4 18|             | 23  PC1 (SDA - Shared Laser & Thermal Data)
 *                PD5 19|             | 22  PC0 (SCL - Shared Laser & Thermal Clock)
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
 *  3. Shared I2C Bus (Both sensors connect in parallel to SCL & SDA):
 *     - SCL Line        --> ATmega32 Pin 22 (PC0) [+ 4.7k pull-up to +5V]
 *     - SDA Line        --> ATmega32 Pin 23 (PC1) [+ 4.7k pull-up to +5V]
 * 
 *  4. VL53L0X Laser Distance Sensor:
 *     - VIN / VCC       --> +5V (or +3.3V)
 *     - GND             --> Common GND
 *     - SCL             --> ATmega32 Pin 22 (PC0)
 *     - SDA             --> ATmega32 Pin 23 (PC1)
 *     - XSHUT           --> ATmega32 Pin 40 (PA0)
 * 
 *  5. GY-906 (MLX90614) Thermal Sensor:
 *     - VIN / VCC       --> +5V (or +3.3V)
 *     - GND             --> Common GND
 *     - SCL             --> ATmega32 Pin 22 (PC0)
 *     - SDA             --> ATmega32 Pin 23 (PC1)
 * 
 *  6. (Optional) Diagnostic LEDs:
 *     - Heartbeat LED   --> ATmega32 Pin 1  (PB0) -> 330 ohm resistor -> GND
 *     - Alert LED (+)   --> ATmega32 Pin 33 (PA7) -> 330 ohm resistor -> GND
 * =====================================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <util/delay.h>
#include "Adafruit_VL53L0X.h"

// Hardware Pin Definitions
#define LASER_XSHUT_BIT   PA0   // Pin 40: VL53L0X Hardware Reset / Enable
#define ALERT_LED_BIT     PA7   // Pin 33: Target Lock / Heat Alert LED
#define HEARTBEAT_LED_BIT PB0   // Pin 1: Heartbeat LED

// I2C Addresses
#define VL53L0X_I2CADDR   0x29  // Laser Sensor Address
#define MLX90614_I2CADDR  0x5A  // Thermal Sensor Address
#define MLX90614_TA       0x06  // Ambient Temperature Register
#define MLX90614_TOBJ1    0x07  // Object 1 Temperature Register

// Alert Thresholds
#define LOCK_DISTANCE_MM  200   // Target close threshold: <= 200 mm (~20 cm)
#define HEAT_THRESHOLD_C  32.0f // Elevated object temp threshold in Celsius
#define DELTA_THRESHOLD_C 2.0f  // Object temperature delta above Ambient

// Sensor instances and state
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool vl53l0x_online = false;
bool mlx90614_online = false;

// Filtered values
float filtered_obj_c = 0.0f;
bool first_temp_read = true;

// I2C Bus Recovery: Clears any hung slave from the bus
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

  if (msb & 0x80) {
    return -999.0f; // Measurement error
  }

  uint16_t tempRaw = ((uint16_t)msb << 8) | lsb;
  return ((float)tempRaw * 0.02f) - 273.15f;
}

void setup() {
  // 1. Configure LEDs and XSHUT pin
  DDRB |= (1 << HEARTBEAT_LED_BIT);
  PORTB |= (1 << HEARTBEAT_LED_BIT); // Heartbeat ON

  DDRA |= (1 << ALERT_LED_BIT) | (1 << LASER_XSHUT_BIT);
  PORTA &= ~(1 << ALERT_LED_BIT);   // Alert LED OFF

  // 2. Reset VL53L0X via XSHUT (PA0) with 250ms MCU boot delay
  PORTA &= ~(1 << LASER_XSHUT_BIT);
  delay(50);
  PORTA |= (1 << LASER_XSHUT_BIT);
  delay(250);

  // 3. Initialize UART at 9600 Baud with pull-up on RXD (PD0)
  PORTD |= (1 << PD0);
  Serial.begin(9600);
  delay(100);

  Serial.println();
  Serial.println(F("=================================================================="));
  Serial.println(F("   ATmega32A COMBINED SENSOR TEST: LASER ToF + THERMAL IR         "));
  Serial.println(F("=================================================================="));
  Serial.println(F(" Clock : 16.0 MHz | Baud: 9600 (PD0=RXD, PD1=TXD)                "));
  Serial.println(F(" I2C   : Pin 22 (PC0=SCL), Pin 23 (PC1=SDA) @ 100 kHz            "));
  Serial.println(F(" Laser : VL53L0X (0x29) | Lock Threshold: <= 200 mm               "));
  Serial.println(F(" Therm : MLX90614 (0x5A) | Alert Threshold: Obj >= 32.0 C         "));
  Serial.println(F("=================================================================="));
  Serial.flush();

  // 4. Recover I2C Bus & Initialize Wire
  i2c_bus_recovery();
  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_TIMEOUT)
  Wire.setWireTimeout(0); // 0 disables timeout to prevent SMBus clock stretching cutoff
#endif
  PORTC |= (1 << PC0) | (1 << PC1); // Enable internal pull-ups

  // 5. Initialize VL53L0X Laser Sensor
  Serial.print(F(" 1. Initializing VL53L0X Laser at 0x29... "));
  Serial.flush();
  if (lox.begin(VL53L0X_I2CADDR, false, &Wire)) {
    vl53l0x_online = true;
    lox.setMeasurementTimingBudgetMicroSeconds(100000);
    Serial.println(F("[SUCCESS] Ready!"));
  } else {
    vl53l0x_online = false;
    Serial.println(F("[FAILED!] Check Laser wiring and XSHUT."));
  }

  // 6. Initialize GY-906 (MLX90614) Thermal Sensor
  Serial.print(F(" 2. Probing GY-906 Thermal Sensor at 0x5A... "));
  Serial.flush();
  float test_amb = readMLX90614TempC(MLX90614_TA);
  if (test_amb > -100.0f) {
    mlx90614_online = true;
    Serial.println(F("[SUCCESS] Ready!"));
  } else {
    mlx90614_online = false;
    Serial.println(F("[FAILED!] Check Thermal sensor wiring."));
  }

  Serial.println(F("=================================================================="));
  Serial.println(F(" Starting continuous dual-sensor telemetry stream...             "));
  Serial.println(F("------------------------------------------------------------------"));
  Serial.flush();
}

void loop() {
  // Toggle Heartbeat LED
  PORTB ^= (1 << HEARTBEAT_LED_BIT);

  // --- 1. Read Laser Distance ---
  uint16_t dist_mm = 0;
  bool laser_valid = false;

  if (vl53l0x_online) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter >= 15 && measure.RangeMilliMeter <= 2500) {
      dist_mm = measure.RangeMilliMeter;
      laser_valid = true;
    }
  }

  // --- 2. Read Thermal Temperature ---
  float amb_c = -999.0f;
  float obj_c = -999.0f;
  float delta_c = 0.0f;
  bool thermal_valid = false;

  if (mlx90614_online) {
    amb_c = readMLX90614TempC(MLX90614_TA);
    obj_c = readMLX90614TempC(MLX90614_TOBJ1);

    if (amb_c > -40.0f && amb_c < 125.0f && obj_c > -40.0f && obj_c < 200.0f) {
      thermal_valid = true;
      if (first_temp_read) {
        filtered_obj_c = obj_c;
        first_temp_read = false;
      } else {
        filtered_obj_c = (0.35f * obj_c) + (0.65f * filtered_obj_c);
      }
      delta_c = filtered_obj_c - amb_c;
    }
  }

  // --- 3. Evaluate Target Lock / Heat Alert Conditions ---
  bool proximity_hit = laser_valid && (dist_mm <= LOCK_DISTANCE_MM);
  bool thermal_hit   = thermal_valid && ((filtered_obj_c >= HEAT_THRESHOLD_C) || (delta_c >= DELTA_THRESHOLD_C));

  if (proximity_hit || thermal_hit) {
    PORTA |= (1 << ALERT_LED_BIT); // Alert LED ON
    Serial.print(F("[!] >>> TARGET LOCKED! "));
  } else {
    PORTA &= ~(1 << ALERT_LED_BIT); // Alert LED OFF
    Serial.print(F("[DATA] "));
  }

  // --- 4. Format Combined Output ---
  // Laser telemetry
  Serial.print(F("Laser: "));
  if (laser_valid) {
    Serial.print(dist_mm);
    Serial.print(F(" mm"));
  } else if (!vl53l0x_online) {
    Serial.print(F("[OFFLINE]"));
  } else {
    Serial.print(F("[CLEAR]"));
  }

  Serial.print(F(" | Thermal: "));
  if (thermal_valid) {
    Serial.print(F("Amb="));
    Serial.print(amb_c, 1);
    Serial.print(F("C Obj="));
    Serial.print(filtered_obj_c, 1);
    Serial.print(F("C (dt:"));
    if (delta_c >= 0) Serial.print(F("+"));
    Serial.print(delta_c, 1);
    Serial.print(F("C)"));
  } else if (!mlx90614_online) {
    Serial.print(F("[OFFLINE]"));
  } else {
    Serial.print(F("[ERR]"));
  }

  // Status annotation
  if (proximity_hit && thermal_hit) {
    Serial.print(F(" | [DUAL LOCK: PROXIMITY + HEAT!] <<<"));
  } else if (proximity_hit) {
    Serial.print(F(" | [PROXIMITY LOCK (<=200mm)] <<<"));
  } else if (thermal_hit) {
    Serial.print(F(" | [HEAT SIGNATURE DETECTED] <<<"));
  } else {
    Serial.print(F(" | Status: CLEAR"));
  }

  Serial.println();

  // Sample delay: ~4 readings per second
  delay(200);
}
