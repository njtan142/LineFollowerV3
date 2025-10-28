#ifndef CONFIG_H
#define CONFIG_H

/**
 * Configuration Parameters
 * Centralized location for all tunable parameters
 */
struct Config {
    // PID Controller Settings
    static constexpr float PID_KP = 0.15;      // Proportional gain
    static constexpr float PID_KI = 0.0;       // Integral gain
    static constexpr float PID_KD = 0.5;       // Derivative gain
    
    // Motor Settings
    static constexpr uint8_t BASE_SPEED = 150;     // Base motor speed (0-255)
    static constexpr uint8_t MAX_SPEED = 255;      // Maximum motor speed
    static constexpr uint8_t MIN_SPEED = 0;        // Minimum motor speed
    static constexpr uint8_t TURN_SPEED = 120;     // Speed for sharp turns
    
    // Line Following Settings
    static constexpr int16_t LINE_SETPOINT = 0;    // Desired line position (centered)
    static constexpr uint16_t LINE_LOST_THRESHOLD = 100;  // Time (ms) before considering line lost
    
    // Calibration Settings
    static constexpr uint16_t CALIBRATION_TIME = 5000;  // Calibration duration (ms)
    static constexpr uint8_t CALIBRATION_SPEED = 100;   // Motor speed during calibration
    
    // UI Settings
    static constexpr uint16_t BUTTON_DEBOUNCE_MS = 50;   // Button debounce time
    static constexpr uint16_t DISPLAY_UPDATE_MS = 100;   // Display update interval
    
    // System States
    enum class Mode {
        IDLE,
        CALIBRATING,
        RUNNING,
        MENU,
        DEBUG
    };
};

#endif
