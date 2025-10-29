#include "motor.h"
#include "Settings.h"
#include "constants.h"
#include <Arduino.h>

Motor::Motor(int IN1_PIN, int IN2_PIN, int PWM_PIN, int Standby, int alignmentOffset, const Settings* settings) {
    this->IN1_PIN = IN1_PIN;
    this->IN2_PIN = IN2_PIN;
    this->PWM_PIN = PWM_PIN;
    this->Standby = Standby;
    this->alignmentOffset = alignmentOffset;
    this->settings = settings;

    // Pin initialization is now handled by HAL::initMotors()
    // Individual motor instances just store pin references

    speed = 0;            // target speed (0..255)
    currentSpeed = 0;     // signed
    currentState = MotorState::BRAKE;
}

void Motor::setState(MotorState state) {
    currentState = state;
}

void Motor::setSpeed(int targetSpeed) {
    // apply alignment on target; clamp to valid range
    int corrected = constrain(targetSpeed + alignmentOffset, 
                             Constants::Motor::SPEED_MIN, 
                             Constants::Motor::SPEED_MAX);
    speed = corrected;
}

void Motor::setAlignmentOffset(int offset) {
    alignmentOffset = offset;
}

void Motor::update(unsigned long deltaTime) {
    // compute ramp steps with proper integer math
    // step = (ACCELERATION * deltaTime) / ACCELERATION_TIME
    int accelStep = (settings->acceleration * (long)deltaTime) / settings->accelerationTime;
    int decelStep = (settings->deceleration * (long)deltaTime) / settings->decelerationTime;
    if (accelStep < Constants::Motor::MIN_ACCEL_DECEL_STEP) 
        accelStep = Constants::Motor::MIN_ACCEL_DECEL_STEP;  // ensure progress
    if (decelStep < Constants::Motor::MIN_ACCEL_DECEL_STEP) 
        decelStep = Constants::Motor::MIN_ACCEL_DECEL_STEP;

    switch(currentState) {
        case MotorState::FORWARD: {
            if(currentSpeed < 0 && settings->movementType == MovementType::SMOOTH){
                currentState = MotorState::BRAKE;
                update(deltaTime);
                break;
            }
            int target = constrain(speed, 0, settings->baseSpeed);
            if(settings->movementType == MovementType::AGGRESSIVE){
                currentSpeed = target;
            } else {
                if(currentSpeed < target) currentSpeed = min(currentSpeed + accelStep, target);
                if(currentSpeed > target) currentSpeed = max(currentSpeed - decelStep, target);
            }
            fwd(currentSpeed);
            break;
        }
        case MotorState::BACKWARD: {
            if(currentSpeed > 0 && settings->movementType == MovementType::SMOOTH){
                currentState = MotorState::BRAKE;
                update(deltaTime);
                break;
            }
            int target = constrain(speed, 0, settings->baseSpeed);
            if(settings->movementType == MovementType::AGGRESSIVE){
                currentSpeed = -target;
            } else {
                if(currentSpeed > -target) currentSpeed = max(currentSpeed - accelStep, -target);
                if(currentSpeed < -target) currentSpeed = min(currentSpeed + decelStep, -target);
            }
            rev(abs(currentSpeed));
            break;
        }
        case MotorState::BRAKE: {
            if(settings->movementType == MovementType::AGGRESSIVE){
                brake();
                currentSpeed = 0;
                break;
            }
            if(currentSpeed > 0){
                currentSpeed = max(0, currentSpeed - decelStep);
                if(currentSpeed == 0) { brake(); } else { fwd(currentSpeed); }
            } else if(currentSpeed < 0){
                currentSpeed = min(0, currentSpeed + decelStep);
                if(currentSpeed == 0) { brake(); } else { rev(abs(currentSpeed)); }
            } else {
                brake();
            }
            break;
        }
        case MotorState::STANDBY:
            standby();
            break;
    }
}

void Motor::fwd(int speed)
{
   digitalWrite(Standby, HIGH);
   digitalWrite(IN1_PIN, HIGH);
   digitalWrite(IN2_PIN, LOW);
   analogWrite(PWM_PIN, speed);
}

void Motor::rev(int speed)
{
   digitalWrite(Standby, HIGH);
   digitalWrite(IN1_PIN, LOW);
   digitalWrite(IN2_PIN, HIGH);
   analogWrite(PWM_PIN, speed);
}

void Motor::brake()
{
   digitalWrite(Standby, HIGH);
   digitalWrite(IN1_PIN, HIGH);
   digitalWrite(IN2_PIN, HIGH);
   analogWrite(PWM_PIN, 0);
}

void Motor::standby()
{
   digitalWrite(Standby, LOW);
}

bool Motor::isMoving() {
    return currentSpeed != 0;
}