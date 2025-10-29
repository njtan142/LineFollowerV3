#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

/**
 * @file constants.h
 * @brief System-wide constants to replace magic numbers throughout the codebase
 * 
 * This file contains all configurable constants organized by component.
 * All values are compile-time constants to minimize RAM usage.
 */
namespace Constants {
    
    /**
     * @namespace LineSensor
     * @brief Line sensor array configuration and position calculation constants
     */
    namespace LineSensor {
        /** @brief Number of sensors in the array (always 8 for consistency) */
        static const uint8_t COUNT = 8;
        
        /** @brief Minimum analog reading value (10-bit ADC) */
        static const int ANALOG_MIN = 0;
        
        /** @brief Maximum analog reading value (10-bit ADC) */
        static const int ANALOG_MAX = 1023;
        
        /** @brief Default threshold for line detection (mid-scale) */
        static const int DEFAULT_THRESHOLD = 512;
        
        /** @brief Center position value (between sensors 3 and 4) */
        static const int CENTER_POSITION = 3500;
        
        /** @brief Minimum position value (far left) */
        static const int MIN_POSITION = -3500;
        
        /** @brief Maximum position value (far right) */
        static const int MAX_POSITION = 3500;
        
        /** @brief Scaling factor for weighted position calculation */
        static const int POSITION_SCALE = 1000;
    }
    
    /**
     * @namespace Motor
     * @brief Motor control constants for speed and movement
     */
    namespace Motor {
        /** @brief Minimum motor PWM speed (stopped) */
        static const uint8_t SPEED_MIN = 0;
        
        /** @brief Maximum motor PWM speed (8-bit PWM) */
        static const uint8_t SPEED_MAX = 255;
        
        /** @brief Default base speed for line following */
        static const uint8_t DEFAULT_BASE_SPEED = 150;
        
        /** @brief Default speed for motor testing */
        static const uint8_t DEFAULT_TEST_SPEED = 150;
        
        /** @brief Minimum step for smooth acceleration/deceleration */
        static const uint8_t MIN_ACCEL_DECEL_STEP = 1;
    }
    
    /**
     * @namespace Calibration
     * @brief Sensor calibration process parameters
     */
    namespace Calibration {
        /** @brief Number of readings to take during calibration */
        static const int ITERATIONS = 100;
        
        /** @brief Duration of each calibration movement step (ms) */
        static const unsigned long MOVEMENT_TIME_MS = 10;
        
        /** @brief Delay between movement steps during calibration (ms) */
        static const unsigned long STOP_DELAY_MS = 10;
    }
    
    /**
     * @namespace Display
     * @brief OLED display layout and rendering constants
     */
    namespace Display {
        /** @brief Width of a single character in pixels */
        static const uint8_t CHAR_WIDTH = 6;
        
        /** @brief Height of a single character in pixels */
        static const uint8_t CHAR_HEIGHT = 8;
        
        /** @brief Height of a text line in pixels */
        static const uint8_t LINE_HEIGHT = 8;
        
        /** @brief Height reserved for title text */
        static const uint8_t TITLE_HEIGHT = 12;
        
        /** @brief Y-coordinate for line 0 (top) */
        static const uint8_t Y_LINE_0 = 0;
        
        /** @brief Y-coordinate for line 1 */
        static const uint8_t Y_LINE_1 = 8;
        
        /** @brief Y-coordinate for line 2 */
        static const uint8_t Y_LINE_2 = 16;
        
        /** @brief Y-coordinate for line 3 (bottom) */
        static const uint8_t Y_LINE_3 = 24;
        
        /** @brief X-coordinate for menu selection arrow */
        static const uint8_t MENU_ARROW_X = 0;
        
        /** @brief X-coordinate for menu item text */
        static const uint8_t MENU_TEXT_X = 10;
        
        /** @brief X-coordinate for menu item values */
        static const uint8_t MENU_VALUE_X = 70;
        
        /** @brief Maximum characters that fit on one line */
        static const uint8_t MAX_CHARS_PER_LINE = 21;
    }
    
    /**
     * @namespace Timing
     * @brief System timing and delay constants
     */
    namespace Timing {
        /** @brief Startup delay before initialization (ms) */
        static const unsigned long STARTUP_DELAY_MS = 500;
        
        /** @brief Button debounce delay (ms) */
        static const unsigned long DEBOUNCE_DELAY_MS = 50;
        
        /** @brief Display auto-off timeout (ms) */
        static const unsigned long DISPLAY_AUTO_OFF_MS = 10000;
    }
    
    /**
     * @namespace PID
     * @brief PID controller tuning parameters and profiles
     */
    namespace PID {
        /** @brief Default proportional gain */
        static const int DEFAULT_KP = 12;
        
        /** @brief Default integral gain */
        static const int DEFAULT_KI = 0;
        
        /** @brief Default derivative gain */
        static const int DEFAULT_KD = 6;
        
        /** @brief Default PID scale factor for fixed-point math */
        static const int DEFAULT_SCALE = 1000;
        
        /** @brief Minimum allowed Kp value */
        static const int MIN_KP = 0;
        
        /** @brief Maximum allowed Kp value */
        static const int MAX_KP = 100;
        
        /** @brief Minimum allowed Ki value */
        static const int MIN_KI = 0;
        
        /** @brief Maximum allowed Ki value */
        static const int MAX_KI = 100;
        
        /** @brief Minimum allowed Kd value */
        static const int MIN_KD = 0;
        
        /** @brief Maximum allowed Kd value */
        static const int MAX_KD = 100;
        
        /** @brief Minimum PID scale factor */
        static const int MIN_SCALE = 1;
        
        /** @brief Maximum PID scale factor */
        static const int MAX_SCALE = 10000;
        
        /**
         * @namespace Aggressive
         * @brief Fast, responsive PID profile for high-speed runs
         */
        namespace Aggressive {
            static const int KP = 20;
            static const int KI = 0;
            static const int KD = 8;
            static const int SCALE = 1000;
        }
        
        /**
         * @namespace Balanced
         * @brief Balanced PID profile for general use
         */
        namespace Balanced {
            static const int KP = 12;
            static const int KI = 0;
            static const int KD = 6;
            static const int SCALE = 1000;
        }
        
        /**
         * @namespace Smooth
         * @brief Smooth, stable PID profile for precision
         */
        namespace Smooth {
            static const int KP = 8;
            static const int KI = 0;
            static const int KD = 10;
            static const int SCALE = 1000;
        }
    }
    
    /**
     * @namespace UI
     * @brief User interface input device constants
     */
    namespace UI {
        /**
         * @namespace Potentiometer
         * @brief Potentiometer analog input parameters
         */
        namespace Potentiometer {
            /** @brief Minimum potentiometer reading */
            static const int ANALOG_MIN = 0;
            
            /** @brief Maximum potentiometer reading */
            static const int ANALOG_MAX = 1023;
            
            /** @brief Threshold for detecting value changes (noise filtering) */
            static const int DEFAULT_CHANGE_THRESHOLD = 5;
            
            /** @brief Settling time after enabling potentiometer (microseconds) */
            static const int SETTLING_TIME_US = 10;
        }
        
        /**
         * @namespace Button
         * @brief Button input parameters
         */
        namespace Button {
            /** @brief Default debounce delay (milliseconds) */
            static const unsigned long DEFAULT_DEBOUNCE_MS = 50;
        }
    }
    
    /**
     * @namespace Movement
     * @brief Robot movement and motion control parameters
     */
    namespace Movement {
        /** @brief Default control loop timestep (ms) */
        static const int DEFAULT_TIMESTEP_MS = 20;
        
        /** @brief Default movement duration per step (ms) */
        static const int DEFAULT_MOVETIME_MS = 20;
        
        /** @brief Default brake timestep (ms) */
        static const int DEFAULT_BREAK_TIMESTEP_MS = 10;
        
        /** @brief Default acceleration rate */
        static const int DEFAULT_ACCELERATION = 150;
        
        /** @brief Default deceleration rate */
        static const int DEFAULT_DECELERATION = 150;
        
        /** @brief Default acceleration time (ms) */
        static const int DEFAULT_ACCELERATION_TIME_MS = 200;
        
        /** @brief Default deceleration time (ms) */
        static const int DEFAULT_DECELERATION_TIME_MS = 100;
    }
    
    /**
     * @namespace Statistics
     * @brief Runtime statistics and monitoring parameters
     */
    namespace Statistics {
        /** @brief Interval for speed sampling (every N updates) */
        static const uint8_t SPEED_SAMPLE_INTERVAL = 10;
    }
}

#endif
