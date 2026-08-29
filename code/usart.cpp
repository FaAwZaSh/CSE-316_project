/*
 * =====================================================================================
 *  PROJECT: VANGUARD - ATmega32A + VL53L0X Fail-Safe Distance Monitor
 * =====================================================================================
 *  MCU:           ATmega32A (8.0 MHz Internal RC Oscillator, lfuse = 0xE4)
 *  Baud Rate:     9600 Baud
 *  I2C Pins:      SCL = PC0 (Pin 22), SDA = PC1 (Pin 23)
 *  XSHUT Pin:     PA0 (Pin 40) driven HIGH
 *  Indicators:    PB0 (Heartbeat), PA1 (Sample Pulse), PA7 (Threshold Alert)
 * =====================================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Exponential Moving Average (EMA) Filter
float filtered_dist_mm = 0.0;
bool first_read = true;

void setup() {
    // 1. Immediately setup Heartbeat LED (PB0) and hard-reset VL53L0X (PA0)
    DDRB |= (1 << PB0);
    PORTB |= (1 << PB0); // Turn PB0 ON to signal power-on

    DDRA |= (1 << PA0) | (1 << PA1) | (1 << PA7);
    
    // Pulse XSHUT LOW -> HIGH to force hardware reset on VL53L0X
    PORTA &= ~(1 << PA0); // Reset pin LOW
    _delay_ms(50);
    PORTA |= (1 << PA0);  // Drive XSHUT HIGH to wake VL53L0X
    _delay_ms(50);

    // 2. IMMEDIATE SERIAL BOOT BANNER (Before I2C touches the bus!)
    Serial.begin(9600);
    _delay_ms(50);

    Serial.println();
    Serial.println(F("===================================================="));
    Serial.println(F("  [MCU BOOT] ATmega32 Powered On & Executing Setup   "));
    Serial.println(F("===================================================="));
    Serial.println(F(" Clock: 8.0 MHz Internal RC | Serial: 9600 Baud"));
    Serial.println(F(" Step 1: Serial UART Operational."));
    Serial.println(F(" Step 2: Initializing Hardware I2C (PC0=SCL, PC1=SDA)..."));
    Serial.flush();

    // 3. Initialize I2C Bus with 100kHz clock
    Wire.begin();
    Wire.setClock(100000);
    _delay_ms(100);

    Serial.println(F(" Step 3: Probing VL53L0X Sensor at I2C address 0x29..."));
    Serial.flush();

    if (!lox.begin(0x29, false, &Wire)) {
        Serial.println(F(""));
        Serial.println(F("===================================================="));
        Serial.println(F(" [I2C ERROR] VL53L0X Sensor Not Responding!"));
        Serial.println(F("===================================================="));
        Serial.println(F(" I2C Bus Freeze / Sensor Disconnected Checklist:"));
        Serial.println(F("   1. SCL -> ATmega32 Pin 22 (PC0) + 4.7k Pull-up to 5V"));
        Serial.println(F("   2. SDA -> ATmega32 Pin 23 (PC1) + 4.7k Pull-up to 5V"));
        Serial.println(F("   3. XSHUT -> ATmega32 Pin 40 (PA0) [HIGH]"));
        Serial.println(F("   4. VIN -> 5V / 3.3V, GND -> Common GND"));
        Serial.println(F("===================================================="));
        Serial.flush();

        // Rapid LED blink to indicate I2C bus error
        while (1) {
            PORTB ^= (1 << PB0);
            _delay_ms(100);
        }
    }

    // Set 100ms timing budget for high sensitivity
    lox.setMeasurementTimingBudgetMicroSeconds(100000);

    Serial.println(F(" Step 4: [SUCCESS] VL53L0X Detected & Ready!"));
    Serial.println(F("----------------------------------------------------\r\n"));
    Serial.flush();
}

void loop() {
    PORTB ^= (1 << PB0); // Toggle Heartbeat LED on every loop iteration

    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4 && measure.RangeMilliMeter > 10 && measure.RangeMilliMeter < 3000) {
        PORTA |= (1 << PA1); // Pulse sample LED

        uint16_t raw_mm = measure.RangeMilliMeter;

        if (first_read) {
            filtered_dist_mm = raw_mm;
            first_read = false;
        } else {
            filtered_dist_mm = (0.35f * raw_mm) + (0.65f * filtered_dist_mm);
        }

        float dist_inches = filtered_dist_mm / 25.4f;
        float dist_cm = filtered_dist_mm / 10.0f;

        Serial.print(F("Distance: "));
        if (dist_inches < 10.0f) Serial.print(F(" "));
        Serial.print(dist_inches, 2);
        Serial.print(F(" in  ("));
        Serial.print(dist_cm, 1);
        Serial.print(F(" cm / "));
        Serial.print((uint16_t)filtered_dist_mm);
        Serial.print(F(" mm) | Bar: ["));

        int bars = (int)(dist_inches / 2.0f);
        if (bars > 20) bars = 20;
        for (int i = 0; i < 20; i++) {
            if (i < bars) Serial.print(F("="));
            else Serial.print(F(" "));
        }
        Serial.print(F("]"));

        if (dist_inches <= 6.0f) {
            PORTA |= (1 << PA7);
            Serial.print(F("  <-- NEAR (<6 in)"));
        } else {
            PORTA &= ~(1 << PA7);
        }
        Serial.println();

        delay(10);
        PORTA &= ~(1 << PA1);
    } else {
        Serial.println(F("[SEARCHING] Target beyond range (>80 in) or signal blocked"));
        PORTA &= ~(1 << PA7);
    }

    delay(100);
}