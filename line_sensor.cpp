#include "line_sensor.h"
#include "motor.h"
#include "eeprom_manager.h"
#include "constants.h"

/**
 * Constructor - Initializes the line sensor array with safe defaults
 * 
 * Sets up 8 sensor channels with initial calibration values and loads
 * any previously saved calibration data from EEPROM. Uses modified 6-channel
 * mode by default to work with hardware that has 6 physical sensors mapped
 * to 8 logical channels (with 2 virtual averaged sensors).
 */
LineSensor::LineSensor(){
    // Start at center position (no line detected yet)
    lastPosition = 0;
    
    // Use most sensitive threshold ratio (15/16) for optimal line detection
    thresholdRatio = ThresholdRatio::RATIO_15_16;
    
    // Default to 6-channel hardware mode with virtual sensors
    sensorMode = SensorMode::MODIFIED_6_CHANNEL;
    
    // Initialize all sensor arrays with safe default values
    // Min starts at max value, max starts at min value - will be updated during calibration
    for (int i = 0; i < SENSORCOUNT; i++) {
        sensorValues[i] = 0;
        sensorMin[i] = Constants::LineSensor::ANALOG_MAX;
        sensorMax[i] = Constants::LineSensor::ANALOG_MIN;
        sensorThreshold[i] = Constants::LineSensor::DEFAULT_THRESHOLD;
    }
    
    // Attempt to load previously saved calibration from EEPROM
    // If none exists, will use the defaults set above
    loadCalibration();
}

/**
 * Reads raw analog values from all sensor channels
 * 
 * Supports two modes:
 * 1. NORMAL_8_CHANNEL: Reads 8 physical sensors directly from A0-A7
 * 2. MODIFIED_6_CHANNEL: Reads 6 physical sensors from A1-A6 and creates
 *    2 virtual sensors by averaging adjacent pairs for smoother line detection
 * 
 * The virtual sensors (channels 1 and 6) help provide intermediate readings
 * between physical sensors, improving position resolution.
 */
void LineSensor::readSensorsRaw(){
    // NORMAL MODE: Direct 1:1 mapping of 8 physical sensors to 8 channels
    if (sensorMode == SensorMode::NORMAL_8_CHANNEL) {
        for(int i = 0; i < SENSORCOUNT; i++){
            sensorValues[i] = analogRead(i + A0);
        }
    } 
    // MODIFIED MODE: 6 physical sensors + 2 calculated virtual sensors
    // This compensates for hardware with only 6 sensors while maintaining
    // 8-channel position calculation algorithm
    else {
        // Read 6 physical sensors from A1-A6 into non-consecutive array positions
        sensorValues[0] = analogRead(A1);  // Leftmost sensor
        sensorValues[2] = analogRead(A2);
        sensorValues[3] = analogRead(A3);  // Center-left
        sensorValues[4] = analogRead(A4);  // Center-right
        sensorValues[5] = analogRead(A5);
        sensorValues[7] = analogRead(A6);  // Rightmost sensor
        
        // Create virtual sensors by averaging adjacent physical sensors
        // This provides smoother transitions and better position interpolation
        sensorValues[1] = (sensorValues[0] + sensorValues[2]) / 2;  // Virtual left
        sensorValues[6] = (sensorValues[5] + sensorValues[7]) / 2;  // Virtual right
    }
}

/**
 * Performs automatic sensor calibration over a fixed time period
 * 
 * This is a simplified calibration method that reads sensors continuously
 * for a predetermined duration. The robot should be moved over the line
 * during this time (controlled externally) so sensors see both black and white.
 * 
 * Note: This function only reads sensors - motor control must be handled
 * separately (typically in the main sketch's calibration routine).
 * Backwards compatible with older calibration code.
 */
void LineSensor::calibrateSensors(){
    beginCalibration();
    
    // Calculate total calibration time based on movement iterations
    int iterations = Constants::Calibration::ITERATIONS;
    int movement = Constants::Calibration::MOVEMENT_TIME_MS;
    int stopDelay = Constants::Calibration::STOP_DELAY_MS;
    int totalTime = iterations * (movement + stopDelay);
    
    // Continuously read and update min/max values for the calibration duration
    unsigned long startTime = millis();
    while (millis() - startTime < totalTime) {
        updateCalibration();
        delay(10);  // Small delay between readings to avoid excessive sampling
    }
    
    // Calculate thresholds and save calibration data to EEPROM
    endCalibration();
}

/**
 * Initializes calibration by resetting min/max tracking values
 * 
 * Sets min values to maximum possible (1023) and max values to minimum (0)
 * so that the first sensor readings will properly update these bounds.
 * This prepares the system to track the full range of values seen during calibration.
 */
void LineSensor::beginCalibration() {
    for (int i = 0; i < SENSORCOUNT; i++) {
        sensorMin[i] = Constants::LineSensor::ANALOG_MAX;  // 1023
        sensorMax[i] = Constants::LineSensor::ANALOG_MIN;  // 0
    }
}

/**
 * Updates calibration min/max values with current sensor readings
 * 
 * Called repeatedly during calibration while the robot moves over the line.
 * Tracks the darkest (min) and brightest (max) values seen by each sensor,
 * which will later be used to calculate detection thresholds.
 */
void LineSensor::updateCalibration() {
    readSensorsRaw();
    
    // Update min/max bounds for each sensor based on current reading
    for (int i = 0; i < SENSORCOUNT; i++) {
        if (sensorValues[i] < sensorMin[i]) sensorMin[i] = sensorValues[i];
        if (sensorValues[i] > sensorMax[i]) sensorMax[i] = sensorValues[i];
    }
}

/**
 * Finalizes calibration process
 * 
 * Calculates detection thresholds based on the min/max values collected
 * during calibration, then saves all calibration data to EEPROM for
 * persistence across power cycles.
 */
void LineSensor::endCalibration() {
    updateThresholds();
    saveCalibration();
}

/**
 * Public wrapper to read current sensor values
 * 
 * Simply calls readSensorsRaw() to update the sensorValues array.
 * Provided for API consistency and potential future extensions.
 */
void LineSensor::readSensors(){
    readSensorsRaw();
}

/**
 * Calculates the line position relative to the sensor array center
 * 
 * Uses a weighted average algorithm to determine where the line is positioned
 * under the sensor array. Returns a value from -3500 to +3500 where:
 *   -3500 = line is at far left (sensor 0)
 *       0 = line is centered (between sensors 3 and 4)
 *   +3500 = line is at far right (sensor 7)
 * 
 * This high-resolution position value allows for smooth PID control.
 * If no line is detected, returns the last known position to maintain
 * control stability during brief line losses (e.g., gaps in the track).
 * 
 * Safety: If calibration data is invalid, returns center position (0)
 * to prevent erratic behavior.
 */
int LineSensor::getPosition(){
    // Validate calibration before attempting position calculation
    // Invalid calibration could cause division by zero or incorrect thresholds
    if (!validateCalibration()) {
        lastPosition = 0;  // Return center as safe default
        return lastPosition;
    }
    
    // Calculate weighted average of sensors detecting the line
    long weighted = 0;
    long count = 0;
    for (int i = 0; i < SENSORCOUNT; i++) {
        // Sensor is "on line" if reading exceeds its threshold (darker = higher value)
        bool onLine = sensorValues[i] > sensorThreshold[i];
        if (onLine) {
            // Weight by sensor position (0-7) scaled by 1000 for precision
            weighted += (long)i * Constants::LineSensor::POSITION_SCALE;
            count++;
        }
    }
    
    // No sensors detecting line - return last known position
    // This provides continuity during brief line losses
    if (count == 0) {
        return lastPosition;
    }
    
    // Calculate average position from weighted sum
    int pos = (int)(weighted / count);
    
    // Shift to center the range: sensors 3-4 become position 0
    // Raw range: 0-7000 becomes centered range: -3500 to +3500
    pos -= Constants::LineSensor::CENTER_POSITION;
    
    // Clamp to valid range for safety
    pos = constrain(pos, Constants::LineSensor::MIN_POSITION, Constants::LineSensor::MAX_POSITION);
    
    // Store for use when line is lost
    lastPosition = pos;
    return pos;
}

/**
 * Counts how many sensors are currently detecting the line (black surface)
 * 
 * Returns the number of sensors with readings above their threshold value.
 * Useful for detecting special track features:
 *   - All sensors black: might be at finish line or intersection
 *   - No sensors black: line completely lost
 *   - Normal operation: 2-3 sensors black as line passes under array
 */
int LineSensor::getBlackSensorCount() {
    int count = 0;
    for (int i = 0; i < SENSORCOUNT; i++) {
        // Sensor detects black if reading exceeds its threshold
        if (sensorValues[i] > sensorThreshold[i]) {
            count++;
        }
    }
    return count;
}

/**
 * Saves complete calibration data to EEPROM for persistence
 * 
 * Stores all calibration parameters so the robot can skip calibration
 * on subsequent power-ups if the track/lighting conditions haven't changed.
 * 
 * Data saved:
 *   - Magic number (validates calibration exists)
 *   - Min values for all 8 sensors (darkest readings seen)
 *   - Max values for all 8 sensors (brightest readings seen)
 *   - Threshold values for all 8 sensors (calculated from min/max)
 *   - Threshold ratio setting (sensitivity level)
 */
void LineSensor::saveCalibration() {
    // Write magic number first - used to verify valid calibration data exists
    EEPROMManager::write(EEPROMManager::Addresses::SENSOR_MAGIC, EEPROMManager::SENSOR_MAGIC_NUMBER);
    
    // Write calibration data for all sensors
    for (int i = 0; i < SENSORCOUNT; i++) {
        EEPROMManager::write(EEPROMManager::Addresses::SENSOR_MIN + i * 2, sensorMin[i]);
        EEPROMManager::write(EEPROMManager::Addresses::SENSOR_MAX + i * 2, sensorMax[i]);
        EEPROMManager::write(EEPROMManager::Addresses::SENSOR_THRESHOLD + i * 2, sensorThreshold[i]);
    }
    
    // Save threshold ratio setting
    EEPROMManager::write(EEPROMManager::Addresses::THRESHOLD_RATIO, (uint8_t)thresholdRatio);
}

/**
 * Loads calibration data from EEPROM
 * 
 * Attempts to restore previously saved calibration. If no valid calibration
 * is found (first run or corrupted data), resets to factory defaults and
 * saves them to EEPROM.
 * 
 * This allows the robot to "remember" its calibration across power cycles,
 * saving time during startup.
 */
void LineSensor::loadCalibration() {
    // Check if valid calibration data exists in EEPROM
    if (!EEPROMManager::isSensorCalibrationValid()) {
        // No valid data found - use factory defaults and save them
        resetCalibration();
        saveCalibration();
        return;
    }
    
    // Load previously saved calibration data
    for (int i = 0; i < SENSORCOUNT; i++) {
        sensorMin[i] = EEPROMManager::read<int>(EEPROMManager::Addresses::SENSOR_MIN + i * 2);
        sensorMax[i] = EEPROMManager::read<int>(EEPROMManager::Addresses::SENSOR_MAX + i * 2);
        sensorThreshold[i] = EEPROMManager::read<int>(EEPROMManager::Addresses::SENSOR_THRESHOLD + i * 2);
    }
    uint8_t ratioValue = EEPROMManager::read<uint8_t>(EEPROMManager::Addresses::THRESHOLD_RATIO);
    thresholdRatio = (ThresholdRatio)ratioValue;
}

/**
 * Resets all calibration data to factory defaults
 * 
 * Used when no valid calibration exists or when user explicitly requests
 * a factory reset. Sets conservative default values that will work poorly
 * but safely until proper calibration is performed.
 */
void LineSensor::resetCalibration() {
    // Use most sensitive threshold ratio by default
    thresholdRatio = ThresholdRatio::RATIO_15_16;
    
    for (int i = 0; i < SENSORCOUNT; i++) {
        sensorMin[i] = Constants::LineSensor::ANALOG_MAX;       // 1023
        sensorMax[i] = Constants::LineSensor::ANALOG_MIN;       // 0
        sensorThreshold[i] = Constants::LineSensor::DEFAULT_THRESHOLD;  // 512
    }
}

/**
 * Updates the threshold ratio and recalculates all sensor thresholds
 * 
 * The threshold ratio controls sensitivity to line detection. After changing
 * the ratio, thresholds are immediately recalculated and saved to EEPROM.
 * 
 * Higher ratios (15/16) are more sensitive but may pick up noise.
 * Lower ratios (1/2) are more tolerant of variations but less precise.
 */
void LineSensor::setThresholdRatio(ThresholdRatio ratio) {
    thresholdRatio = ratio;
    updateThresholds();
    saveCalibration();  // Persist the change immediately
}

/**
 * Gets the current threshold ratio setting
 */
ThresholdRatio LineSensor::getThresholdRatio() const {
    return thresholdRatio;
}

/**
 * Recalculates detection thresholds for all sensors based on current ratio
 * 
 * The threshold determines the point at which a sensor reading is considered
 * "black" (line detected) vs "white" (no line). It's calculated as a ratio
 * between the min (white) and max (black) values seen during calibration.
 * 
 * Ratio meanings:
 *   15/16 (offset 1/16 = 6.25%) - Very sensitive, triggers easily
 *    7/8  (offset 1/8  = 12.5%) - Moderately sensitive
 *    3/4  (offset 1/4  = 25%)   - Less sensitive, more stable
 *    1/2  (offset 1/2  = 50%)   - Very stable, midpoint detection
 * 
 * The ratio compensates for sensor/reading variations during robot movement,
 * making detection more reliable in dynamic conditions.
 */
void LineSensor::updateThresholds() {
    // Extract numerator from ratio enum (15, 7, 3, or 1)
    uint8_t numerator = (uint8_t)thresholdRatio;
    // Denominator is always numerator + 1 (16, 8, 4, or 2)
    uint8_t denominator = numerator + 1;
    
    // Calculate threshold for each sensor based on its calibrated min/max
    // Formula: threshold = (max * numerator + min) / denominator
    // This places the threshold closer to max (black) for higher ratios
    for (int i = 0; i < SENSORCOUNT; i++) {
        sensorThreshold[i] = (sensorMax[i] * numerator + sensorMin[i]) / denominator;
    }
}

/**
 * Sets the sensor operating mode (8-channel normal or 6-channel modified)
 */
void LineSensor::setSensorMode(SensorMode mode) {
    sensorMode = mode;
}

/**
 * Gets the current sensor operating mode
 */
SensorMode LineSensor::getSensorMode() const {
    return sensorMode;
}

/**
 * Validates that the current calibration data is reasonable
 * 
 * Performs sanity checks on calibration to prevent using bad data that
 * could cause erratic robot behavior. Checks:
 * 
 * 1. Sufficient contrast: Each sensor must have seen at least 150 points
 *    difference between brightest and darkest readings. Less than this
 *    suggests the sensor didn't see both line and background properly.
 * 
 * 2. Reasonable bounds: Min/max values should be within expected ADC range
 *    (20-1000). Values outside this suggest sensor malfunction or
 *    electrical noise.
 * 
 * 3. Valid thresholds: Threshold must fall between min and max. If not,
 *    the threshold calculation failed or data is corrupted.
 * 
 * Returns: true if all sensors pass validation, false if any sensor fails
 */
bool LineSensor::validateCalibration() {
    for (int i = 0; i < SENSORCOUNT; i++) {
        int range = sensorMax[i] - sensorMin[i];
        
        // CHECK 1: Minimum contrast requirement
        // Range < 150 means sensor didn't see enough difference between line and background
        if (range < 150) {
            return false;
        }
        
        // CHECK 2: Sensor readings within physically possible bounds
        // Min too low or max too high suggests hardware/electrical issues
        if (sensorMin[i] < 20 || sensorMax[i] > 1000) {
            return false;
        }
        
        // CHECK 3: Threshold is logically between min and max
        // Threshold outside this range indicates calculation error or corruption
        if (sensorThreshold[i] <= sensorMin[i] || sensorThreshold[i] >= sensorMax[i]) {
            return false;
        }
    }
    return true;  // All sensors passed validation
}

/**
 * Calculates a quality score (0-100) for the current calibration
 * 
 * Evaluates how well the sensors were calibrated by analyzing the range
 * (contrast) seen by each sensor during calibration. Better calibration
 * comes from seeing a wide range of values.
 * 
 * Quality scoring per sensor:
 *   0-150:   Invalid (0%)   - Not enough contrast, unusable
 *   150-250: Poor (50%)     - Minimal contrast, will work but not optimal
 *   250-400: Good (75%)     - Decent contrast, reliable operation
 *   400+:    Excellent(100%)- Great contrast, optimal performance
 * 
 * Final score is the average of all valid sensors, with penalty if some
 * sensors failed to calibrate properly.
 * 
 * Returns: Quality score from 0 (completely failed) to 100 (perfect)
 */
int LineSensor::getCalibrationQuality() {
    int totalQuality = 0;
    int validSensors = 0;
    
    for (int i = 0; i < SENSORCOUNT; i++) {
        // Calculate contrast range for this sensor
        int range = sensorMax[i] - sensorMin[i];
        
        // Assign quality score based on contrast range
        int quality;
        if (range < 150) {
            quality = 0;    // Invalid - failed to calibrate
        } else if (range < 250) {
            quality = 50;   // Poor but usable
        } else if (range < 400) {
            quality = 75;   // Good performance expected
        } else {
            quality = 100;  // Excellent calibration
        }
        
        // Only include valid sensors in average
        if (quality > 0) {
            totalQuality += quality;
            validSensors++;
        }
    }
    
    // No valid sensors = complete failure
    if (validSensors == 0) return 0;
    
    // Calculate average quality across valid sensors
    int avgQuality = totalQuality / validSensors;
    
    // Apply penalty if not all sensors calibrated successfully
    // This ensures we can't get 100% with missing sensors
    if (validSensors < SENSORCOUNT) {
        avgQuality = (avgQuality * validSensors) / SENSORCOUNT;
    }
    
    return avgQuality;
}
