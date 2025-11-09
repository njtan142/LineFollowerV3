#ifndef HAL_H
#define HAL_H

#include <Arduino.h>

/**
 * Hardware Abstraction Layer (HAL)
 * Centralizes all hardware pin definitions and low-level hardware operations.
 * This decouples hardware-specific details from the rest of the application.
 */
class HAL {
public:
    // Motor Driver Pins (TB6612FNG)
    struct MotorPins {
        static const uint8_t STBY = 7;
        
        // Temporarily swap just the PWM pins
        struct Left {
            static const uint8_t IN1 = 8;
            static const uint8_t IN2 = 12;
            static const uint8_t PWM = 10;  // Using right's PWM
        };
        
        struct Right {
            static const uint8_t IN1 = 13;  // Changed from 13 to avoid built-in LED
            static const uint8_t IN2 = 11;
            static const uint8_t PWM = 6;   // Using left's PWM
        };
    };
    
    // UI Input Pins
    struct UIPins {
        static const uint8_t BUTTON = 5;        // Button input
        static const uint8_t POT = A7;          // Potentiometer analog input (now dedicated, not shared)
        static const uint8_t POT_ENABLE = 9;    // Potentiometer enable (legacy, can be repurposed)
    };
    
    // Display Pins (Software I2C for SSD1306 OLED)
    struct DisplayPins {
        static const uint8_t SOFT_SDA = 2;      // Software I2C Data
        static const uint8_t SOFT_SCL = 3;      // Software I2C Clock
        static const int8_t OLED_RESET = -1;    // Reset pin (-1 if sharing Arduino reset)
        static const uint8_t OLED_ADDRESS = 0x3C;
        static const uint8_t SCREEN_WIDTH = 128;
        static const uint8_t SCREEN_HEIGHT = 32;
    };
    
    // Line Sensor Configuration
    // Note: Code always uses 8 channels for consistency
    // - NORMAL mode: Physical sensors on A0-A7 (8 real sensors)
    // - MODIFIED mode: Physical sensors on A1-A6 (6 real sensors)
    //   In MODIFIED mode:
    //   - Channel 0 (A0): Reads A1
    //   - Channel 1 (A1): Average of channels 0 and 2
    //   - Channels 2-5 (A2-A5): Direct reads
    //   - Channel 6 (A6): Average of channels 5 and 7
    //   - Channel 7 (A7): Reads A6
    struct SensorPins {
        static const uint8_t COUNT = 8;
        static const uint8_t FIRST_ANALOG_PIN = A0;  // Starting analog pin
    };
    
    // Initialize all hardware
    static void begin() {
        initMotors();
        initUI();
        // Display and sensors have their own initialization in their classes
    }
    
    // Motor Control Methods
    static void initMotors() {
        pinMode(MotorPins::STBY, OUTPUT);
        
        pinMode(MotorPins::Left::IN1, OUTPUT);
        pinMode(MotorPins::Left::IN2, OUTPUT);
        pinMode(MotorPins::Left::PWM, OUTPUT);
        
        pinMode(MotorPins::Right::IN1, OUTPUT);
        pinMode(MotorPins::Right::IN2, OUTPUT);
        pinMode(MotorPins::Right::PWM, OUTPUT);
        
        setMotorStandby(false);  // Motors active by default
    }
    
    static void setMotorStandby(bool standby) {
        digitalWrite(MotorPins::STBY, standby ? LOW : HIGH);
    }
    
    static void setMotorDirection(uint8_t in1Pin, uint8_t in2Pin, bool forward, bool brake = false) {
        if (brake) {
            digitalWrite(in1Pin, HIGH);
            digitalWrite(in2Pin, HIGH);
        } else if (forward) {
            digitalWrite(in1Pin, HIGH);
            digitalWrite(in2Pin, LOW);
        } else {
            digitalWrite(in1Pin, LOW);
            digitalWrite(in2Pin, HIGH);
        }
    }
    
    static void setMotorSpeed(uint8_t pwmPin, uint8_t speed) {
        analogWrite(pwmPin, speed);
    }
    
    // UI Input Methods
    static void initUI() {
        pinMode(UIPins::BUTTON, INPUT);
        pinMode(UIPins::POT_ENABLE, OUTPUT);
        enableLineSensors();  // Default to line sensors enabled
    }
    
    static bool readButton() {
        return digitalRead(UIPins::BUTTON) == LOW;  // Active low with pullup
    }
    
    static int readPotentiometer() {
        return analogRead(UIPins::POT);
    }
    
    // Potentiometer/Sensor Multiplexing Control
    // Note: POT_ENABLE is now legacy - A7 is dedicated to potentiometer in MODIFIED mode
    // In NORMAL mode, A7 can still be used for sensor 7
    static void enablePotentiometer() {
        digitalWrite(UIPins::POT_ENABLE, HIGH);
        delayMicroseconds(10);  // Brief settling time
    }
    
    static void enableLineSensors() {
        digitalWrite(UIPins::POT_ENABLE, LOW);
        delayMicroseconds(10);  // Brief settling time
    }
    
    // Sensor Methods
    static int readLineSensor(uint8_t index) {
        if (index >= SensorPins::COUNT) return 0;
        return analogRead(SensorPins::FIRST_ANALOG_PIN + index);
    }
    
    // General GPIO Methods (for any future needs)
    static void setPinMode(uint8_t pin, uint8_t mode) {
        pinMode(pin, mode);
    }
    
    static void writeDigital(uint8_t pin, bool value) {
        digitalWrite(pin, value ? HIGH : LOW);
    }
    
    static bool readDigital(uint8_t pin) {
        return digitalRead(pin) == HIGH;
    }
    
    static int readAnalog(uint8_t pin) {
        return analogRead(pin);
    }
    
    static void writeAnalog(uint8_t pin, uint8_t value) {
        analogWrite(pin, value);
    }
};

#endif
