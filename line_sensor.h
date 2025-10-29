#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Arduino.h>
#include "motor.h"
#include "enums.h"

#define SENSORCOUNT 8

class LineSensor {
    public:
        LineSensor();
        void readSensorsRaw();
        void calibrateSensors();
        void beginCalibration();  // Initialize calibration (reset min/max)
        void updateCalibration();  // Update min/max with current sensor readings
        void endCalibration();  // Finalize calibration (compute thresholds, save to EEPROM)
        void readSensors();
        int getPosition();
        int getBlackSensorCount();  // Count how many sensors see black
        
        // EEPROM functions
        void saveCalibration();
        void loadCalibration();
        void resetCalibration();  // Reset to factory defaults
        void setThresholdRatio(ThresholdRatio ratio);
        ThresholdRatio getThresholdRatio() const;
        
        // Calibration validation
        bool validateCalibration();  // Check if calibration data is valid
        int getCalibrationQuality();  // Get quality score 0-100
        
        // Sensor mode methods
        void setSensorMode(SensorMode mode);
        SensorMode getSensorMode() const;
        
        // Accessor methods for calibration data (useful for debugging/UI)
        const int* getSensorMin() const { return sensorMin; }
        const int* getSensorMax() const { return sensorMax; }
        const int* getSensorThreshold() const { return sensorThreshold; }
        const int* getSensorValues() const { return sensorValues; }
        
    private:
        int sensorValues[SENSORCOUNT];
        int sensorMin[SENSORCOUNT];
        int sensorMax[SENSORCOUNT];
        int sensorThreshold[SENSORCOUNT];
        int lastPosition; // -3500..+3500
        ThresholdRatio thresholdRatio;
        SensorMode sensorMode;
        
        void updateThresholds(); // Recalculate thresholds based on ratio
};

#endif