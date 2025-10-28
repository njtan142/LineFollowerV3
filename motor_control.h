#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "hal.h"

/**
 * Motor Control Module
 * Handles all motor operations for differential drive robot
 */
class MotorControl {
public:
    static void begin() {
        HAL::initMotors();
        stop();
    }
    
    // Set left motor speed and direction
    // speed: -255 (full reverse) to 255 (full forward)
    static void setLeftMotor(int16_t speed) {
        if (speed == 0) {
            HAL::setMotorDirection(HAL::MotorPins::Left::IN1, HAL::MotorPins::Left::IN2, true, true);
            HAL::setMotorSpeed(HAL::MotorPins::Left::PWM, 0);
        } else {
            bool forward = (speed > 0);
            HAL::setMotorDirection(HAL::MotorPins::Left::IN1, HAL::MotorPins::Left::IN2, forward);
            HAL::setMotorSpeed(HAL::MotorPins::Left::PWM, abs(speed));
        }
    }
    
    // Set right motor speed and direction
    // speed: -255 (full reverse) to 255 (full forward)
    static void setRightMotor(int16_t speed) {
        if (speed == 0) {
            HAL::setMotorDirection(HAL::MotorPins::Right::IN1, HAL::MotorPins::Right::IN2, true, true);
            HAL::setMotorSpeed(HAL::MotorPins::Right::PWM, 0);
        } else {
            bool forward = (speed > 0);
            HAL::setMotorDirection(HAL::MotorPins::Right::IN1, HAL::MotorPins::Right::IN2, forward);
            HAL::setMotorSpeed(HAL::MotorPins::Right::PWM, abs(speed));
        }
    }
    
    // Set both motors (differential drive)
    static void setMotors(int16_t leftSpeed, int16_t rightSpeed) {
        setLeftMotor(leftSpeed);
        setRightMotor(rightSpeed);
    }
    
    // High-level movement commands
    static void forward(uint8_t speed = 200) {
        setMotors(speed, speed);
    }
    
    static void backward(uint8_t speed = 200) {
        setMotors(-speed, -speed);
    }
    
    static void turnLeft(uint8_t speed = 150) {
        setMotors(-speed, speed);
    }
    
    static void turnRight(uint8_t speed = 150) {
        setMotors(speed, -speed);
    }
    
    static void stop() {
        setMotors(0, 0);
    }
    
    static void standby(bool enable) {
        HAL::setMotorStandby(enable);
    }
};

#endif
