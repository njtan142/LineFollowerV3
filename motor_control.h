#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "hal.h"

/**
 * Motor Control Module - Header
 * 
 * Static utility class providing high-level motor control functions for
 * differential drive robots. Wraps HAL (Hardware Abstraction Layer) calls
 * with convenient movement primitives.
 * 
 * Features:
 * - Simple API for common movements (forward, backward, turns)
 * - Direct motor control with signed speed values
 * - Automatic direction handling based on speed sign
 * - Standby mode for power saving
 * 
 * Note: This is a simpler alternative to the Motor class for applications
 * that don't need smooth ramping or state machines. All methods are static.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "hal.h"

/**
 * MotorControl Class (Static)
 * 
 * Provides convenience functions for controlling a differential drive robot.
 * All methods are static - no instantiation needed.
 */
class MotorControl {
public:
    /**
     * Initializes motor hardware and stops motors
     * Call once during setup before using any motor functions
     */
    static void begin() {
        HAL::initMotors();
        stop();
    }
    
    /**
     * Sets left motor speed and direction
     * 
     * Automatically determines direction based on sign of speed value.
     * Positive speeds drive forward, negative speeds drive reverse.
     * Zero speed applies brake (both H-bridge inputs HIGH).
     * 
     * @param speed - Signed speed value (-255 to +255)
     *                Negative = reverse, Positive = forward, 0 = brake
     */
    static void setLeftMotor(int16_t speed) {
        // Zero speed - apply brake
        if (speed == 0) {
            HAL::setMotorDirection(HAL::MotorPins::Left::IN1, HAL::MotorPins::Left::IN2, true, true);
            HAL::setMotorSpeed(HAL::MotorPins::Left::PWM, 0);
        } 
        // Non-zero speed - determine direction and apply magnitude
        else {
            bool forward = (speed > 0);
            HAL::setMotorDirection(HAL::MotorPins::Left::IN1, HAL::MotorPins::Left::IN2, forward);
            HAL::setMotorSpeed(HAL::MotorPins::Left::PWM, abs(speed));
        }
    }
    
    /**
     * Sets right motor speed and direction
     * 
     * Automatically determines direction based on sign of speed value.
     * Positive speeds drive forward, negative speeds drive reverse.
     * Zero speed applies brake (both H-bridge inputs HIGH).
     * 
     * @param speed - Signed speed value (-255 to +255)
     *                Negative = reverse, Positive = forward, 0 = brake
     */
    static void setRightMotor(int16_t speed) {
        // Zero speed - apply brake
        if (speed == 0) {
            HAL::setMotorDirection(HAL::MotorPins::Right::IN1, HAL::MotorPins::Right::IN2, true, true);
            HAL::setMotorSpeed(HAL::MotorPins::Right::PWM, 0);
        } 
        // Non-zero speed - determine direction and apply magnitude
        else {
            bool forward = (speed > 0);
            HAL::setMotorDirection(HAL::MotorPins::Right::IN1, HAL::MotorPins::Right::IN2, forward);
            HAL::setMotorSpeed(HAL::MotorPins::Right::PWM, abs(speed));
        }
    }
    
    /**
     * Sets both motors simultaneously (differential drive control)
     * 
     * Allows independent control of each motor for differential steering.
     * Different left/right speeds create turning motion.
     * 
     * @param leftSpeed - Left motor speed (-255 to +255)
     * @param rightSpeed - Right motor speed (-255 to +255)
     */
    static void setMotors(int16_t leftSpeed, int16_t rightSpeed) {
        setLeftMotor(leftSpeed);
        setRightMotor(rightSpeed);
    }
    
    // === High-Level Movement Commands ===
    
    /**
     * Drives robot straight forward
     * Both motors run at same speed in forward direction
     * 
     * @param speed - Forward speed (0-255), default 200
     */
    static void forward(uint8_t speed = 200) {
        setMotors(speed, speed);
    }
    
    /**
     * Drives robot straight backward
     * Both motors run at same speed in reverse direction
     * 
     * @param speed - Backward speed (0-255), default 200
     */
    static void backward(uint8_t speed = 200) {
        setMotors(-speed, -speed);
    }
    
    /**
     * Turns robot left (pivot turn)
     * Left motor reverses, right motor forwards - robot spins in place
     * 
     * @param speed - Turn speed (0-255), default 150
     */
    static void turnLeft(uint8_t speed = 150) {
        setMotors(-speed, speed);
    }
    
    /**
     * Turns robot right (pivot turn)
     * Right motor reverses, left motor forwards - robot spins in place
     * 
     * @param speed - Turn speed (0-255), default 150
     */
    static void turnRight(uint8_t speed = 150) {
        setMotors(speed, -speed);
    }
    
    /**
     * Stops both motors immediately
     * Applies active brake (short-circuit) for maximum stopping force
     */
    static void stop() {
        setMotors(0, 0);
    }
    
    /**
     * Controls motor driver standby mode
     * 
     * Standby mode disables the motor driver to save power.
     * Motors will coast to a stop when entering standby.
     * 
     * @param enable - true to enable standby (low power), false to wake up
     */
    static void standby(bool enable) {
        HAL::setMotorStandby(enable);
    }
};

#endif
