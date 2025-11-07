/**
 * Line Sensor Module - Header
 * 
 * Manages an array of 8 reflectance sensors for line detection.
 * Provides automatic calibration, position calculation, and EEPROM persistence.
 * 
 * Features:
 * - 8-channel sensor array (physical or virtual)
 * - Automatic calibration with min/max tracking
 * - Weighted average position calculation (-3500 to +3500)
 * - Adjustable threshold ratios for noise tolerance
 * - EEPROM storage for calibration persistence
 * - Calibration validation and quality scoring
 * - Support for 8-sensor and 6-sensor (with virtual) modes
 */

#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Arduino.h>
#include "motor.h"
#include "enums.h"

// Number of sensor channels (8 logical channels)
#define SENSORCOUNT 8

/**
 * LineSensor Class
 * 
 * Controls an array of reflectance sensors for line following.
 * Handles calibration, position calculation, and data persistence.
 */
class LineSensor {
    public:
        /**
         * Constructor - Initializes sensor array with defaults
         * Loads any existing calibration from EEPROM
         */
        LineSensor();
        
        /**
         * Reads raw analog values from all sensor channels
         * Supports both 8-channel physical and 6-channel (with virtual) modes
         */
        void readSensorsRaw();
        
        /**
         * Performs automatic calibration over a fixed time period
         * Robot should be swept over line during this time (controlled externally)
         * Simplified version - does not control motors
         */
        void calibrateSensors();
        
        /**
         * Initializes calibration process
         * Resets min/max tracking values to prepare for calibration
         */
        void beginCalibration();
        
        /**
         * Updates calibration with current sensor readings
         * Call repeatedly during calibration to track min/max values
         */
        void updateCalibration();
        
        /**
         * Finalizes calibration process
         * Calculates thresholds and saves data to EEPROM
         */
        void endCalibration();
        
        /**
         * Reads current sensor values
         * Wrapper for readSensorsRaw() for API consistency
         */
        void readSensors();
        
        /**
         * Calculates line position relative to sensor array center
         * Uses weighted average algorithm for high-resolution positioning
         * @return Position from -3500 (far left) to +3500 (far right), 0 = centered
         */
        int getPosition();
        
        /**
         * Counts how many sensors are currently detecting black (line)
         * @return Number of sensors above their threshold (0-8)
         */
        int getBlackSensorCount();
        
        // === EEPROM Persistence ===
        
        /**
         * Saves complete calibration data to EEPROM
         * Stores min/max values, thresholds, and ratio for all sensors
         */
        void saveCalibration();
        
        /**
         * Loads calibration data from EEPROM
         * If no valid data exists, uses factory defaults
         */
        void loadCalibration();
        
        /**
         * Resets calibration to factory defaults
         * Sets conservative values that work safely but not optimally
         */
        void resetCalibration();
        
        /**
         * Sets the threshold ratio and recalculates thresholds
         * Higher ratios are more sensitive, lower ratios more tolerant
         * @param ratio - ThresholdRatio enum value (RATIO_15_16, RATIO_7_8, etc.)
         */
        void setThresholdRatio(ThresholdRatio ratio);
        
        /**
         * Gets the current threshold ratio setting
         * @return Current ThresholdRatio enum value
         */
        ThresholdRatio getThresholdRatio() const;
        
        // === Calibration Validation ===
        
        /**
         * Validates that calibration data is reasonable
         * Checks for sufficient contrast, valid bounds, and logical thresholds
         * @return true if calibration passes all checks, false otherwise
         */
        bool validateCalibration();
        
        /**
         * Calculates a quality score for current calibration
         * Based on sensor contrast ranges seen during calibration
         * @return Quality score from 0 (failed) to 100 (excellent)
         */
        int getCalibrationQuality();
        
        // === Sensor Mode Configuration ===
        
        /**
         * Sets the sensor operating mode
         * @param mode - SensorMode enum (NORMAL_8_CHANNEL or MODIFIED_6_CHANNEL)
         */
        void setSensorMode(SensorMode mode);
        
        /**
         * Gets the current sensor operating mode
         * @return Current SensorMode enum value
         */
        SensorMode getSensorMode() const;
        
        // === Data Accessors (for debugging/display) ===
        
        /**
         * Gets array of minimum values seen during calibration
         * @return Pointer to 8-element array of min values
         */
        const int* getSensorMin() const { return sensorMin; }
        
        /**
         * Gets array of maximum values seen during calibration
         * @return Pointer to 8-element array of max values
         */
        const int* getSensorMax() const { return sensorMax; }
        
        /**
         * Gets array of calculated threshold values
         * @return Pointer to 8-element array of thresholds
         */
        const int* getSensorThreshold() const { return sensorThreshold; }
        
        /**
         * Gets array of current sensor readings
         * @return Pointer to 8-element array of current values
         */
        const int* getSensorValues() const { return sensorValues; }
        
    private:
        // Sensor data arrays (one entry per sensor channel)
        int sensorValues[SENSORCOUNT];      // Current readings (0-1023)
        int sensorMin[SENSORCOUNT];         // Minimum values from calibration (white)
        int sensorMax[SENSORCOUNT];         // Maximum values from calibration (black)
        int sensorThreshold[SENSORCOUNT];   // Calculated thresholds (black/white cutoff)
        
        // Position tracking
        int lastPosition;  // Last calculated position (-3500 to +3500)
        
        // Configuration
        ThresholdRatio thresholdRatio;  // Sensitivity setting
        SensorMode sensorMode;          // Hardware mode (8 or 6 channels)
        
        /**
         * Recalculates detection thresholds based on current ratio
         * Uses min/max values and threshold ratio to compute new thresholds
         * Private - called automatically when needed
         */
        void updateThresholds();
};

#endif