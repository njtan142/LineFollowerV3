#include "motor.h"
#include "Settings.h"
#include "constants.h"
#include <Arduino.h>

/**
 * Motor Constructor - Initializes a single motor instance
 */
Motor::Motor(int IN1_PIN, int IN2_PIN, int PWM_PIN, int Standby, int alignmentOffset, const Settings* settings) {
    this->IN1_PIN = IN1_PIN;
    this->IN2_PIN = IN2_PIN;
    this->PWM_PIN = PWM_PIN;
    this->Standby = Standby;
    this->alignmentOffset = alignmentOffset;
    this->settings = settings;
    speed = 0;
    currentSpeed = 0;
    currentState = MotorState::BRAKE;
    pinsInitialized = false;  // Pins will be initialized on first use
}

/**
 * Sets the motor's operational state
 */
void Motor::setState(MotorState state) {
    currentState = state;
}

/**
 * Ensures pins are initialized before first use
 * Called automatically by motor control functions
 */
void Motor::ensurePinsInitialized() {
    if (!pinsInitialized) {
        pinMode(IN1_PIN, OUTPUT);
        pinMode(IN2_PIN, OUTPUT);
        // pinMode(PWM_PIN, OUTPUT);
        pinMode(Standby, OUTPUT);
        
        // Initialize to safe state (motor off)
        // digitalWrite(IN1_PIN, LOW);
        // digitalWrite(IN2_PIN, LOW);
        // analogWrite(PWM_PIN, 0);
        // digitalWrite(Standby, LOW);
        
        pinsInitialized = true;
    }
}

/**
 * Sets target speed with alignment compensation
 * Positive speed = forward, negative speed = backward
 */
void Motor::setSpeed(int targetSpeed) {
    speed = constrain(targetSpeed, -255, 255);
}

/**
 * Updates the alignment offset value
 */
void Motor::setAlignmentOffset(int offset) {
    alignmentOffset = offset;
}

/**
 * Updates motor - simplified to just move forward or backward based on speed sign
 */
void Motor::update() {
    currentSpeed = speed;
    
    // Move forward if speed is positive, backward if negative, brake if zero
    if (currentSpeed > 0) {
        fwd(currentSpeed);
    } else if (currentSpeed < 0) {
        rev(abs(currentSpeed));
    } else {
        brake();
    }
}

/**
 * Drives motor forward at specified speed
 */
void Motor::fwd(int speed)
{
   ensurePinsInitialized();
   digitalWrite(Standby, HIGH);
   digitalWrite(IN1_PIN, HIGH);
   digitalWrite(IN2_PIN, LOW);
   analogWrite(PWM_PIN, speed);
}

/**
 * Drives motor in reverse at specified speed
 */
void Motor::rev(int speed)
{
   ensurePinsInitialized();
   digitalWrite(Standby, HIGH);
   digitalWrite(IN1_PIN, LOW);
   digitalWrite(IN2_PIN, HIGH);
   analogWrite(PWM_PIN, speed);
}

/**
 * Actively brakes the motor to a stop
 */
void Motor::brake()
{
   ensurePinsInitialized();
   digitalWrite(Standby, HIGH);
   digitalWrite(IN1_PIN, HIGH);
   digitalWrite(IN2_PIN, HIGH);
   analogWrite(PWM_PIN, 0);
}

/**
 * Puts motor driver into standby (low-power) mode
 */
void Motor::standby()
{
   ensurePinsInitialized();
   digitalWrite(Standby, LOW);
}

/**
 * Checks if motor is currently moving
 */
bool Motor::isMoving() {
    return currentSpeed != 0;
}