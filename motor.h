/**
 * Motor Control Module - Header
 * 
 * Defines the Motor class for controlling a single DC motor through
 * an H-bridge driver (TB6612FNG). Provides smooth acceleration/deceleration,
 * state-based control, and alignment compensation.
 * 
 * Features:
 * - State machine (FORWARD, BACKWARD, BRAKE, STANDBY)
 * - Smooth speed ramping (prevents wheel slip)
 * - Aggressive mode (instant response)
 * - Alignment offset (compensates for motor differences)
 * - Time-based updates (consistent behavior regardless of loop speed)
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "enums.h"

// Forward declaration to avoid circular dependency
struct Settings;

/**
 * Motor Class
 * 
 * Controls a single DC motor with state-based operation and smooth ramping.
 * Each robot uses two Motor instances (left and right) for differential drive.
 */
class Motor {
    public:
        /**
         * Constructor - Initializes motor with pin assignments and settings
         * 
         * @param in1Pin - H-bridge input 1 (direction control)
         * @param in2Pin - H-bridge input 2 (direction control)
         * @param pwmPin - PWM output for speed control (0-255)
         * @param standbyPin - Driver enable/standby pin (shared between motors)
         * @param alignmentOffset - Speed adjustment to compensate for motor differences
         * @param settings - Pointer to global settings (acceleration, speeds, etc.)
         */
        Motor(int in1Pin, int in2Pin, int pwmPin, int standbyPin, int alignmentOffset, const Settings* settings);
        
        /**
         * Actively brakes the motor to a stop
         * Short-circuits motor terminals for maximum braking force
         */
        void brake();
        
        /**
         * Puts motor driver in standby (low-power) mode
         * Motor will coast to a stop
         */
        void standby();
        
        /**
         * Sets the motor's operational state
         * @param state - MotorState (FORWARD, BACKWARD, BRAKE, STANDBY)
         */
        void setState(MotorState state);
        
        /**
         * Updates motor speed based on current speed
         * Must be called regularly in loop
         */
        void update();
        
        /**
         * Checks if motor is currently moving
         * @return true if motor has any speed (forward or reverse)
         */
        bool isMoving();
        
        /**
         * Sets target speed with automatic alignment compensation
         * Speed will be adjusted by alignment offset before being applied
         * @param targetSpeed - Desired speed (0-255)
         */
        void setSpeed(int targetSpeed);
        
        /**
         * Updates the alignment offset value
         * Allows runtime adjustment of motor matching
         * @param offset - Speed adjustment (-255 to +255)
         */
        void setAlignmentOffset(int offset);
        
    private:
        // Target and current speeds
        int speed;        // Target speed after alignment (0..255)
        int direction;    // Reserved for future use
        
        // Hardware pin assignments
        int IN1_PIN;      // H-bridge input 1
        int IN2_PIN;      // H-bridge input 2
        int PWM_PIN;      // Speed control PWM
        int Standby;      // Driver enable/standby
        
        // Compensation and settings
        int alignmentOffset;        // Speed offset for motor matching
        const Settings* settings;   // Pointer to global configuration
        
        // Initialization flag
        bool pinsInitialized;      // True once pins are configured
        
        // Helper to ensure pins are initialized before first use
        void ensurePinsInitialized();
        
        // Low-level motor control functions (private - only called by update())
        void fwd(int speed);   // Drive forward at specified speed
        void rev(int speed);   // Drive reverse at specified speed
        
        // State tracking
        MotorState currentState;   // Current operational state
        int currentSpeed;          // Actual speed (signed: +forward, -reverse)
};

#endif