#ifndef ENUMS_H
#define ENUMS_H

#include <Arduino.h>

enum class MotorState {
    FORWARD,
    BACKWARD,
    BRAKE,
    STANDBY
};

enum class MovementType {
    AGGRESSIVE,
    SMOOTH,
};

// PID Control mode
enum class PIDControlMode {
    NORMAL,            // Standard PID control
    OVERRIDE_ON_CENTER // Reset PID and go straight when centered
};

// Threshold ratio values (numerator/denominator where denominator = numerator + 1)
// 15/16 = 6.25%, 7/8 = 12.5%, 3/4 = 25%, 1/2 = 50%
enum class ThresholdRatio {
    RATIO_15_16 = 15,  // 6.25% leeway (most sensitive)
    RATIO_7_8 = 7,     // 12.5% leeway
    RATIO_3_4 = 3,     // 25% leeway
    RATIO_1_2 = 1      // 50% leeway (most tolerant)
};

// Sensor mode configuration
enum class SensorMode {
    NORMAL_8_CHANNEL,      // All 8 sensors physically connected (A0-A7)
    MODIFIED_6_CHANNEL     // 6 physical sensors (A1-A6), sensors 2 and 7 are weighted averages
};

#endif