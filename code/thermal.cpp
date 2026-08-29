/*
 * =====================================================================================
 *  PROJECT: VANGUARD / CSE-316 - GY-906 (MLX90614) Thermal IR Sensor Test
 * =====================================================================================
 *  MCU:           ATmega32A (8.0 MHz Internal RC Oscillator)
 *  Baud Rate:     9600 Baud
 *  Sensor:        GY-906 MLX90614 Contactless IR Temperature Sensor
 *  I2C Pins:      SCL = PC0 (Pin 22), SDA = PC1 (Pin 23)
 *  I2C Address:   0x5A (Default factory SMBus address)
 *  Indicators:    PB0 (Heartbeat LED), PA7 (High Temp / Fever Alert LED)
 * =====================================================================================
 *  WIRING GUIDE:
 *  GY-906 Module Pin  -->  ATmega32A Pin
 *  --------------------------------------------------
 *  VIN / VCC          -->  +5V or +3.3V Power Rail
 *  GND                -->  Common Ground (Pin 11 or 31)
 *  SCL                -->  PC0 (Pin 22) + 4.7k Pull-up to 5V
 *  SDA                -->  PC1 (Pin 23) + 4.7k Pull-up to 5V
 * =====================================================================================
 */

#include <Arduino.h>
#include <Wire.h>

#define MLX90614_I2CADDR 0x5A

// MLX90614 RAM Registers
#define MLX90614_TA   0x06  // Ambient Temperature
#define MLX90614_TOBJ1 0x07 // Object 1 Temperature

// High Temperature / Fever Alert Threshold (in Celsius)
#define FEVER_THRESHOLD_C 37.5f 

// Filtering factor for Exponential Moving Average (0.0 to 1.0)
#define EMA_ALPHA 0.30f

// Filtered temperature variables
float filtered_obj_c = 0.0f;
bool first_read = true;

// Function prototype to read raw temperature from MLX90614 SMBus
float readMLX90614TempC(uint8_t reg);

// Function to clear hung I2C bus by sending 9 SCL clock pulses
void i2c_bus_recovery() {
    DDRC |= (1 << PC0) | (1 << PC1); // SCL (PC0) and SDA (PC1) as outputs
    PORTC |= (1 << PC0) | (1 << PC1); // High
    _delay_us(10);

    for (uint8_t i = 0; i < 9; i++) {
        PORTC &= ~(1 << PC0); // SCL LOW
        _delay_us(10);
        PORTC |= (1 << PC0);  // SCL HIGH
        _delay_us(10);
    }

    // Generate I2C STOP condition
    PORTC &= ~(1 << PC1); // SDA LOW
    _delay_us(10);
    PORTC |= (1 << PC0);  // SCL HIGH
    _delay_us(10);
    PORTC |= (1 << PC1);  // SDA HIGH
    _delay_us(10);

    // Release pins back to floating input for Wire library
    DDRC &= ~((1 << PC0) | (1 << PC1));
    PORTC &= ~((1 << PC0) | (1 << PC1));
}

void setup() {
    // 1. Setup Status Indicators (PB0 = Heartbeat, PA7 = High Temp Alert)
    DDRB |= (1 << PB0);
    PORTB |= (1 << PB0); // Heartbeat ON

    DDRA |= (1 << PA7);
    PORTA &= ~(1 << PA7); // Alert LED OFF initially

    // 2. Initialize Serial USART @ 9600 Baud
    Serial.begin(9600);
    _delay_ms(100);

    Serial.println();
    Serial.println(F("===================================================="));
    Serial.println(F("  [MCU BOOT] ATmega32A GY-906 (MLX90614) Thermal Test"));
    Serial.println(F("===================================================="));
    Serial.println(F(" Clock: 8.0 MHz | Baud: 9600 | I2C: 100 kHz"));
    Serial.println(F(" Step 1: Performing I2C Bus Recovery & Initializing Wire..."));
    Serial.flush();

    // 3. Clear any stuck I2C bus state from previous session
    i2c_bus_recovery();

    Wire.begin();
    Wire.setClock(100000);
    _delay_ms(100);

    // 4. Probe I2C Bus for GY-906 MLX90614 Sensor at 0x5A
    Serial.println(F(" Step 2: Probing GY-906 MLX90614 at I2C address 0x5A..."));
    Serial.flush();

    Wire.beginTransmission(MLX90614_I2CADDR);
    uint8_t error = Wire.endTransmission();

    if (error != 0) {
        // Try bus recovery one more time before failing
        i2c_bus_recovery();
        Wire.beginTransmission(MLX90614_I2CADDR);
        error = Wire.endTransmission();
    }

    if (error != 0) {
        Serial.println(F(""));
        Serial.println(F("===================================================="));
        Serial.println(F(" [I2C ERROR] GY-906 (MLX90614) Sensor Not Responding!"));
        Serial.println(F("===================================================="));
        Serial.println(F(" Troubleshooting Checklist:"));
        Serial.println(F("   1. SCL -> ATmega32 Pin 22 (PC0) + 4.7k Pull-up to 5V"));
        Serial.println(F("   2. SDA -> ATmega32 Pin 23 (PC1) + 4.7k Pull-up to 5V"));
        Serial.println(F("   3. VIN -> 3.3V / 5V, GND -> Common GND"));
        Serial.println(F("   4. Verify I2C module address is 0x5A"));
        Serial.println(F("===================================================="));
        Serial.flush();

        // Rapid LED blink to indicate I2C sensor detection error
        while (1) {
            PORTB ^= (1 << PB0);
            _delay_ms(100);
        }
    }

    Serial.println(F(" Step 3: [SUCCESS] GY-906 MLX90614 Sensor Detected!"));
    Serial.println(F("----------------------------------------------------\r\n"));
    Serial.flush();
}

void loop() {
    PORTB ^= (1 << PB0); // Toggle Heartbeat LED

    // Read Ambient and Object Temperatures in Celsius
    float amb_c = readMLX90614TempC(MLX90614_TA);
    float obj_c = readMLX90614TempC(MLX90614_TOBJ1);

    if (amb_c < -100.0f || obj_c < -100.0f) {
        Serial.println(F("[ERROR] Failed to read SMBus data from MLX90614!"));
    } else {
        // Apply Exponential Moving Average (EMA) filter on Object Temperature
        if (first_read) {
            filtered_obj_c = obj_c;
            first_read = false;
        } else {
            filtered_obj_c = (EMA_ALPHA * obj_c) + ((1.0f - EMA_ALPHA) * filtered_obj_c);
        }

        // Convert to Fahrenheit
        float amb_f = (amb_c * 1.8f) + 32.0f;
        float obj_f = (filtered_obj_c * 1.8f) + 32.0f;

        // Output formatting over Serial UART
        Serial.print(F("Ambient: "));
        Serial.print(amb_c, 1);
        Serial.print(F(" C ("));
        Serial.print(amb_f, 1);
        Serial.print(F(" F) | Object: "));
        Serial.print(filtered_obj_c, 1);
        Serial.print(F(" C ("));
        Serial.print(obj_f, 1);
        Serial.print(F(" F)"));

        // High Temperature / Fever Alert Indicator
        if (filtered_obj_c >= FEVER_THRESHOLD_C) {
            PORTA |= (1 << PA7); // Turn ON Alert LED
            Serial.print(F("  <-- [ALERT: HIGH TEMP!]"));
        } else {
            PORTA &= ~(1 << PA7); // Turn OFF Alert LED
        }

        Serial.println();
    }

    delay(250); // Reading interval 250ms
}

/**
 * @brief Reads 16-bit raw data from MLX90614 RAM register over SMBus and converts to Celsius
 * @param reg RAM Register address (MLX90614_TA or MLX90614_TOBJ1)
 * @return Temperature in degrees Celsius, or -999.0f on error
 */
float readMLX90614TempC(uint8_t reg) {
    // 1. Send Register Read Command over SMBus
    Wire.beginTransmission(MLX90614_I2CADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { // Repeated START
        return -999.0f;
    }

    // 2. Request 3 bytes: LSB, MSB, PEC (Packet Error Code CRC-8)
    if (Wire.requestFrom((uint8_t)MLX90614_I2CADDR, (uint8_t)3) != 3) {
        return -999.0f;
    }

    uint8_t lsb = Wire.read();
    uint8_t msb = Wire.read();
    uint8_t pec = Wire.read(); // Read PEC CRC-8 byte
    (void)pec; // Silence unused warning

    // Check error bit (MSB bit 7 flags measurement errors)
    if (msb & 0x80) {
        return -999.0f;
    }

    // Combine MSB and LSB into 16-bit raw value
    uint16_t tempRaw = ((uint16_t)msb << 8) | lsb;

    // MLX90614 Resolution: 0.02 Kelvin per LSB
    float tempK = tempRaw * 0.02f;

    // Convert Kelvin to Celsius
    return tempK - 273.15f;
}
