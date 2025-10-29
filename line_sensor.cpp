#include "line_sensor.h"
#include "motor.h"
#include "eeprom_manager.h"
#include "constants.h"

LineSensor::LineSensor(){
    // safe defaults
    lastPosition = 0;
    thresholdRatio = ThresholdRatio::RATIO_15_16;
    sensorMode = SensorMode::MODIFIED_6_CHANNEL;  // Default to 6-channel mode
    
    for (int i = 0; i < SENSORCOUNT; i++) {
        sensorValues[i] = 0;
        sensorMin[i] = Constants::LineSensor::ANALOG_MAX;
        sensorMax[i] = Constants::LineSensor::ANALOG_MIN;
        sensorThreshold[i] = Constants::LineSensor::DEFAULT_THRESHOLD;
    }
    
    // Load calibration from EEPROM
    loadCalibration();
}

void LineSensor::readSensorsRaw(){
    if (sensorMode == SensorMode::NORMAL_8_CHANNEL) {
        // Normal mode: Read all 8 sensors directly from A0-A7
        for(int i = 0; i < SENSORCOUNT; i++){
            sensorValues[i] = analogRead(i + A0);
        }
    } else {
        // Modified 6-channel mode: Physical sensors on A1-A6
        // Read physical sensors first
        sensorValues[0] = analogRead(A1);  // Channel 0 reads from A1
        sensorValues[2] = analogRead(A2);  // Channel 2 reads from A2
        sensorValues[3] = analogRead(A3);  // Channel 3 reads from A3
        sensorValues[4] = analogRead(A4);  // Channel 4 reads from A4
        sensorValues[5] = analogRead(A5);  // Channel 5 reads from A5
        sensorValues[7] = analogRead(A6);  // Channel 7 reads from A6
        
        // Calculate weighted averages for virtual sensors
        sensorValues[1] = (sensorValues[0] + sensorValues[2]) / 2;  // Average of channels 0 and 2
        sensorValues[6] = (sensorValues[5] + sensorValues[7]) / 2;  // Average of channels 5 and 7
    }
}

void LineSensor::calibrateSensors(){
    // Simplified calibration method (for backward compatibility)
    // This version doesn't control motors - motor control should be in Robot::calibrate()
    // It just reads sensors for the expected duration
    beginCalibration();
    
    int iterations = Constants::Calibration::ITERATIONS;
    int movement = Constants::Calibration::MOVEMENT_TIME_MS;
    int stopDelay = Constants::Calibration::STOP_DELAY_MS;
    int totalTime = iterations * (movement + stopDelay);
    
    unsigned long startTime = millis();
    while (millis() - startTime < totalTime) {
        updateCalibration();
        delay(10);  // Small delay between readings
    }
    
    endCalibration();
}

void LineSensor::beginCalibration() {
    // Initialize min/max values for calibration
    for (int i = 0; i < SENSORCOUNT; i++) {
        sensorMin[i] = Constants::LineSensor::ANALOG_MAX;
        sensorMax[i] = Constants::LineSensor::ANALOG_MIN;
    }
}

void LineSensor::updateCalibration() {
    // Read sensors and update min/max values
    readSensorsRaw();
    for (int i = 0; i < SENSORCOUNT; i++) {
        if (sensorValues[i] < sensorMin[i]) sensorMin[i] = sensorValues[i];
        if (sensorValues[i] > sensorMax[i]) sensorMax[i] = sensorValues[i];
    }
}

void LineSensor::endCalibration() {
    // Finalize calibration by computing thresholds and saving to EEPROM
    updateThresholds();
    saveCalibration();
}

void LineSensor::readSensors(){
    readSensorsRaw();
}

int LineSensor::getPosition(){
    // Safety check: If calibration is invalid, return center (safe default)
    if (!validateCalibration()) {
        lastPosition = 0;  // Center position
        return lastPosition;
    }
    
    long weighted = 0;
    long count = 0;
    for (int i = 0; i < SENSORCOUNT; i++) {
        bool onLine = sensorValues[i] > sensorThreshold[i];
        if (onLine) {
            weighted += (long)i * Constants::LineSensor::POSITION_SCALE;
            count++;
        }
    }
    if (count == 0) {
        return lastPosition; // keep last when line lost
    }
    int pos = (int)(weighted / count);
    pos -= Constants::LineSensor::CENTER_POSITION;  // center between sensors 3 and 4
    pos = constrain(pos, Constants::LineSensor::MIN_POSITION, Constants::LineSensor::MAX_POSITION);
    lastPosition = pos;
    return pos;
}

int LineSensor::getBlackSensorCount() {
    int count = 0;
    for (int i = 0; i < SENSORCOUNT; i++) {
        if (sensorValues[i] > sensorThreshold[i]) {
            count++;
        }
    }
    return count;
}

// EEPROM Functions
void LineSensor::saveCalibration() {
    // Write magic number
    EEPROMManager::write(EEPROMManager::Addresses::SENSOR_MAGIC, EEPROMManager::SENSOR_MAGIC_NUMBER);
    
    // Save sensor data
    for (int i = 0; i < SENSORCOUNT; i++) {
        EEPROMManager::write(EEPROMManager::Addresses::SENSOR_MIN + i * 2, sensorMin[i]);
        EEPROMManager::write(EEPROMManager::Addresses::SENSOR_MAX + i * 2, sensorMax[i]);
        EEPROMManager::write(EEPROMManager::Addresses::SENSOR_THRESHOLD + i * 2, sensorThreshold[i]);
    }
    EEPROMManager::write(EEPROMManager::Addresses::THRESHOLD_RATIO, (uint8_t)thresholdRatio);
}

void LineSensor::loadCalibration() {
    if (!EEPROMManager::isSensorCalibrationValid()) {
        // No valid calibration - use defaults
        resetCalibration();
        saveCalibration();
        return;
    }
    
    // Load sensor data
    for (int i = 0; i < SENSORCOUNT; i++) {
        sensorMin[i] = EEPROMManager::read<int>(EEPROMManager::Addresses::SENSOR_MIN + i * 2);
        sensorMax[i] = EEPROMManager::read<int>(EEPROMManager::Addresses::SENSOR_MAX + i * 2);
        sensorThreshold[i] = EEPROMManager::read<int>(EEPROMManager::Addresses::SENSOR_THRESHOLD + i * 2);
    }
    uint8_t ratioValue = EEPROMManager::read<uint8_t>(EEPROMManager::Addresses::THRESHOLD_RATIO);
    thresholdRatio = (ThresholdRatio)ratioValue;
}

void LineSensor::resetCalibration() {
    thresholdRatio = ThresholdRatio::RATIO_15_16;
    for (int i = 0; i < SENSORCOUNT; i++) {
        sensorMin[i] = Constants::LineSensor::ANALOG_MAX;
        sensorMax[i] = Constants::LineSensor::ANALOG_MIN;
        sensorThreshold[i] = Constants::LineSensor::DEFAULT_THRESHOLD;
    }
}

void LineSensor::setThresholdRatio(ThresholdRatio ratio) {
    thresholdRatio = ratio;
    updateThresholds();
    saveCalibration(); // Save immediately when changed
}

ThresholdRatio LineSensor::getThresholdRatio() const {
    return thresholdRatio;
}

void LineSensor::updateThresholds() {
    /*
        Threshold for flipping between black and white. 
        Set ratio of the way from min to max to avoid noise.
        Ratio is a/b where b = a + 1;
            15/16 -> 1/16 = 6.25%  (most sensitive)
            7/8   -> 1/8  = 12.5%
            3/4   -> 1/4  = 25%
            1/2   -> 1/2  = 50%    (most tolerant)
        
        This is useful for handling reading inconsistency that arises when moving
    */
    uint8_t numerator = (uint8_t)thresholdRatio;
    uint8_t denominator = numerator + 1;
    
    for (int i = 0; i < SENSORCOUNT; i++) {
        sensorThreshold[i] = (sensorMax[i] * numerator + sensorMin[i]) / denominator;
    }
}

void LineSensor::setSensorMode(SensorMode mode) {
    sensorMode = mode;
}

SensorMode LineSensor::getSensorMode() const {
    return sensorMode;
}

bool LineSensor::validateCalibration() {
    // Validate that calibration data is reasonable
    for (int i = 0; i < SENSORCOUNT; i++) {
        int range = sensorMax[i] - sensorMin[i];
        
        // Check 1: Range too small (sensor didn't see enough contrast)
        if (range < 150) {
            return false;
        }
        
        // Check 2: Min/max values out of reasonable bounds
        if (sensorMin[i] < 20 || sensorMax[i] > 1000) {
            return false;
        }
        
        // Check 3: Threshold makes sense
        if (sensorThreshold[i] <= sensorMin[i] || sensorThreshold[i] >= sensorMax[i]) {
            return false;
        }
    }
    return true;
}

int LineSensor::getCalibrationQuality() {
    // Calculate quality score 0-100 based on sensor ranges
    int totalQuality = 0;
    int validSensors = 0;
    
    for (int i = 0; i < SENSORCOUNT; i++) {
        int range = sensorMax[i] - sensorMin[i];
        
        // Quality based on range:
        // 150-250: Poor (50%)
        // 250-400: Good (75%)
        // 400+:    Excellent (100%)
        int quality;
        if (range < 150) {
            quality = 0;  // Invalid
        } else if (range < 250) {
            quality = 50;  // Poor
        } else if (range < 400) {
            quality = 75;  // Good
        } else {
            quality = 100;  // Excellent
        }
        
        if (quality > 0) {
            totalQuality += quality;
            validSensors++;
        }
    }
    
    if (validSensors == 0) return 0;
    
    // Average quality, with penalty for missing sensors
    int avgQuality = totalQuality / validSensors;
    
    // Penalty if not all sensors are valid
    if (validSensors < SENSORCOUNT) {
        avgQuality = (avgQuality * validSensors) / SENSORCOUNT;
    }
    
    return avgQuality;
}
