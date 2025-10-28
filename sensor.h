#ifndef SENSOR_H
#define SENSOR_H

#include "hal.h"

/**
 * Line Sensor Module
 * Handles line sensor array reading and calibration
 * Optimized to reduce RAM usage
 */
class LineSensor {
private:
    // Store calibration as bytes (0-255 range) instead of uint16_t to save RAM
    static uint8_t minValues[HAL::SensorPins::COUNT];
    static uint8_t maxValues[HAL::SensorPins::COUNT];
    static uint16_t calibratedValues[HAL::SensorPins::COUNT];
    static bool isCalibrated;
    
public:
    static void begin() {
        // Initialize calibration arrays
        for (uint8_t i = 0; i < HAL::SensorPins::COUNT; i++) {
            minValues[i] = 255;
            maxValues[i] = 0;
            calibratedValues[i] = 0;
        }
        isCalibrated = false;
    }
    
    // Calibrate sensors (call this while moving over line)
    static void calibrate() {
        HAL::enableLineSensors();
        for (uint8_t i = 0; i < HAL::SensorPins::COUNT; i++) {
            // Read and immediately use, don't store raw values
            uint16_t reading = HAL::readLineSensor(i);
            uint8_t scaled = reading >> 2;  // Scale 0-1023 to 0-255
            
            if (scaled < minValues[i]) {
                minValues[i] = scaled;
            }
            if (scaled > maxValues[i]) {
                maxValues[i] = scaled;
            }
        }
        isCalibrated = true;
    }
    
    // Read calibrated sensor values (0-1000 scale)
    static void readCalibrated() {
        HAL::enableLineSensors();
        for (uint8_t i = 0; i < HAL::SensorPins::COUNT; i++) {
            uint16_t reading = HAL::readLineSensor(i);
            uint8_t scaled = reading >> 2;  // Scale 0-1023 to 0-255
            
            uint8_t denominator = maxValues[i] - minValues[i];
            if (denominator != 0) {
                calibratedValues[i] = (uint32_t)(scaled - minValues[i]) * 1000 / denominator;
            } else {
                calibratedValues[i] = 0;
            }
            // Clamp to valid range
            if (calibratedValues[i] > 1000) calibratedValues[i] = 1000;
        }
    }
    
    // Get line position (-3500 to 3500, or -1 if line not detected)
    // 0 means centered, negative means left, positive means right
    static int16_t getLinePosition() {
        readCalibrated();
        
        uint32_t avg = 0;
        uint32_t sum = 0;
        bool onLine = false;
        
        for (uint8_t i = 0; i < HAL::SensorPins::COUNT; i++) {
            uint16_t value = calibratedValues[i];
            
            // Consider sensor active if above threshold
            if (value > 200) {
                onLine = true;
            }
            
            // Weight calculation: position * sensor value
            avg += (uint32_t)value * (i * 1000);
            sum += value;
        }
        
        if (!onLine) {
            return -1;  // Line not detected
        }
        
        if (sum == 0) {
            return -1;  // Avoid division by zero
        }
        
        // Calculate position (0-7000 range)
        int16_t pos = avg / sum;
        
        // Convert to centered range (-3500 to 3500)
        return pos - 3500;
    }
    
    // Get individual sensor value (0-1000)
    static uint16_t getSensorValue(uint8_t index) {
        if (index >= HAL::SensorPins::COUNT) return 0;
        return calibratedValues[index];
    }
    
    // Check if sensors are calibrated
    static bool getCalibrationStatus() {
        return isCalibrated;
    }
    
    // Reset calibration
    static void resetCalibration() {
        begin();
    }
};

// Initialize static members - using uint8_t for min/max to save RAM
uint8_t LineSensor::minValues[HAL::SensorPins::COUNT];
uint8_t LineSensor::maxValues[HAL::SensorPins::COUNT];
uint16_t LineSensor::calibratedValues[HAL::SensorPins::COUNT];
bool LineSensor::isCalibrated = false;

#endif
