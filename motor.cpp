#include "motor.h"
#include "Settings.h"
#include "constants.h"
#include <Arduino.h>

/**
 * Motor Constructor - Initializes a single motor instance
 * 
 * Sets up pin assignments and links to global settings for this motor.
 * Note: Actual pin initialization (pinMode, etc.) is handled by HAL::initMotors()
 * to centralize hardware setup. This constructor only stores references.
 * 
 * @param IN1_PIN - H-bridge input 1 pin (direction control)
 * @param IN2_PIN - H-bridge input 2 pin (direction control)
 * @param PWM_PIN - PWM pin for speed control (0-255)
 * @param Standby - Standby pin (shared between motors)
 * @param alignmentOffset - Speed offset to compensate for mechanical differences
 * @param settings - Pointer to global settings (acceleration, base speed, etc.)
 */
Motor::Motor(int IN1_PIN, int IN2_PIN, int PWM_PIN, int Standby, int alignmentOffset, const Settings* settings) {
    this->IN1_PIN = IN1_PIN;
    this->IN2_PIN = IN2_PIN;
    this->PWM_PIN = PWM_PIN;
    this->Standby = Standby;
    this->alignmentOffset = alignmentOffset;
    this->settings = settings;

    // Target speed (0..255) - the speed we want to reach
    speed = 0;
    // Current actual speed (signed: positive=forward, negative=reverse)
    currentSpeed = 0;
    // Start in brake state for safety
    currentState = MotorState::BRAKE;
}

/**
 * Sets the motor's operational state
 * 
 * Changes what the motor should be doing (FORWARD, BACKWARD, BRAKE, STANDBY).
 * Actual motor behavior is handled in update() based on this state.
 */
void Motor::setState(MotorState state) {
    currentState = state;
}

/**
 * Sets target speed with alignment compensation
 * 
 * The target speed is adjusted by the alignment offset to compensate for
 * mechanical differences between left and right motors. For example, if
 * one motor is slightly weaker, its offset can be positive to match the other.
 * 
 * @param targetSpeed - Desired speed (0-255), will be clamped to valid range
 */
void Motor::setSpeed(int targetSpeed) {
    // Apply alignment offset and ensure result is within valid motor speed range
    int corrected = constrain(targetSpeed + alignmentOffset, 
                             Constants::Motor::SPEED_MIN, 
                             Constants::Motor::SPEED_MAX);
    speed = corrected;
}

/**
 * Updates the alignment offset value
 * 
 * Allows runtime adjustment of motor alignment compensation.
 * Useful for tuning robot to drive straight.
 */
void Motor::setAlignmentOffset(int offset) {
    alignmentOffset = offset;
}

/**
 * Updates motor state and speed based on elapsed time
 * 
 * This is the main motor control function, called every loop iteration.
 * Handles smooth acceleration/deceleration ramps and state transitions.
 * 
 * Key features:
 * - Smooth ramping: Gradually changes speed to avoid wheel slip and jerky movement
 * - Aggressive mode: Instant speed changes for fastest response
 * - Auto-braking: When changing direction in smooth mode, automatically brakes first
 * - Time-based: Uses deltaTime for consistent behavior regardless of loop speed
 * 
 * @param deltaTime - Time elapsed since last update (milliseconds)
 */
void Motor::update(unsigned long deltaTime) {
    // Calculate acceleration/deceleration step sizes based on time elapsed
    // Formula: step = (max_speed_change * time_elapsed) / time_to_reach_max
    int accelStep = (settings->acceleration * (long)deltaTime) / settings->accelerationTime;
    int decelStep = (settings->deceleration * (long)deltaTime) / settings->decelerationTime;
    
    // Ensure minimum step size to guarantee progress (avoid getting stuck at 0)
    //TODO: review this as this somehow violates the time delta. it should increment gradually
    if (accelStep < Constants::Motor::MIN_ACCEL_DECEL_STEP) 
        accelStep = Constants::Motor::MIN_ACCEL_DECEL_STEP;
    if (decelStep < Constants::Motor::MIN_ACCEL_DECEL_STEP) 
        decelStep = Constants::Motor::MIN_ACCEL_DECEL_STEP;

    // Handle motor behavior based on current state
    switch(currentState) {
        // FORWARD STATE: Motor should spin forward
        case MotorState::FORWARD: {
            // Safety check: if currently going backward in smooth mode, brake first
            // This prevents mechanical stress from instant direction reversal
            if(currentSpeed < 0 && settings->movementType == MovementType::SMOOTH){
                currentState = MotorState::BRAKE;
                update(deltaTime);  // Recursive call to handle braking immediately
                break;
            }
            
            // Clamp target speed to configured base speed limit
            int target = constrain(speed, 0, settings->baseSpeed);
            
            // AGGRESSIVE MODE: Jump directly to target speed (no ramping)
            if(settings->movementType == MovementType::AGGRESSIVE){
                currentSpeed = target;
            } 
            // SMOOTH MODE: Gradually ramp to target speed
            else {
                // Accelerate if below target
                if(currentSpeed < target) currentSpeed = min(currentSpeed + accelStep, target);
                // Decelerate if above target
                if(currentSpeed > target) currentSpeed = max(currentSpeed - decelStep, target);
            }
            
            // Apply the calculated speed to hardware
            fwd(currentSpeed);
            break;
        }
        
        // BACKWARD STATE: Motor should spin in reverse
        case MotorState::BACKWARD: {
            // Safety check: if currently going forward in smooth mode, brake first
            if(currentSpeed > 0 && settings->movementType == MovementType::SMOOTH){
                currentState = MotorState::BRAKE;
                update(deltaTime);  // Recursive call to handle braking immediately
                break;
            }
            
            // Clamp target speed to configured base speed limit
            int target = constrain(speed, 0, settings->baseSpeed);
            
            // AGGRESSIVE MODE: Jump directly to target speed (negative for reverse)
            if(settings->movementType == MovementType::AGGRESSIVE){
                currentSpeed = -target;
            } 
            // SMOOTH MODE: Gradually ramp to target speed
            else {
                // Accelerate backward (make more negative)
                if(currentSpeed > -target) currentSpeed = max(currentSpeed - accelStep, -target);
                // Decelerate backward (make less negative)
                if(currentSpeed < -target) currentSpeed = min(currentSpeed + decelStep, -target);
            }
            
            // Apply the calculated speed to hardware (use absolute value)
            rev(abs(currentSpeed));
            break;
        }
        
        // BRAKE STATE: Bring motor to a stop
        case MotorState::BRAKE: {
            // AGGRESSIVE MODE: Instant stop with motor braking
            if(settings->movementType == MovementType::AGGRESSIVE){
                brake();
                currentSpeed = 0;
                break;
            }
            
            // SMOOTH MODE: Gradually decelerate to zero
            // If moving forward, slow down toward zero
            if(currentSpeed > 0){
                currentSpeed = max(0, currentSpeed - decelStep);
                // Once stopped, engage brake; otherwise keep coasting down
                if(currentSpeed == 0) { brake(); } else { fwd(currentSpeed); }
            } 
            // If moving backward, slow down toward zero
            else if(currentSpeed < 0){
                currentSpeed = min(0, currentSpeed + decelStep);
                // Once stopped, engage brake; otherwise keep coasting down
                if(currentSpeed == 0) { brake(); } else { rev(abs(currentSpeed)); }
            } 
            // Already stopped - just maintain brake
            else {
                brake();
            }
            break;
        }
        
        // STANDBY STATE: Put motor driver in low-power mode
        case MotorState::STANDBY:
            standby();
            break;
    }
}

/**
 * Drives motor forward at specified speed
 * 
 * Configures H-bridge pins for forward rotation and sets PWM speed.
 * IN1=HIGH, IN2=LOW creates forward current flow through motor.
 * 
 * @param speed - PWM duty cycle (0-255), higher = faster
 */
void Motor::fwd(int speed)
{
   digitalWrite(Standby, HIGH);    // Enable motor driver
   digitalWrite(IN1_PIN, HIGH);    // Forward direction
   digitalWrite(IN2_PIN, LOW);
   analogWrite(PWM_PIN, speed);    // Set speed via PWM
}

/**
 * Drives motor in reverse at specified speed
 * 
 * Configures H-bridge pins for reverse rotation and sets PWM speed.
 * IN1=LOW, IN2=HIGH creates reverse current flow through motor.
 * 
 * @param speed - PWM duty cycle (0-255), higher = faster reverse
 */
void Motor::rev(int speed)
{
   digitalWrite(Standby, HIGH);    // Enable motor driver
   digitalWrite(IN1_PIN, LOW);     // Reverse direction
   digitalWrite(IN2_PIN, HIGH);
   analogWrite(PWM_PIN, speed);    // Set speed via PWM
}

/**
 * Actively brakes the motor to a stop
 * 
 * Sets both H-bridge inputs HIGH, creating a short circuit across motor
 * terminals. This provides active braking (motor resists rotation) which
 * is more effective than just turning off power (coasting).
 * 
 * PWM is set to 0 for maximum braking effect.
 */
void Motor::brake()
{
   digitalWrite(Standby, HIGH);    // Keep driver enabled for braking
   digitalWrite(IN1_PIN, HIGH);    // Short circuit configuration
   digitalWrite(IN2_PIN, HIGH);    // Both HIGH = brake mode
   analogWrite(PWM_PIN, 0);        // No PWM during brake
}

/**
 * Puts motor driver into standby (low-power) mode
 * 
 * Disables the motor driver chip to save power. Motor will coast to a stop
 * (no active braking). Useful when robot is idle for extended periods.
 */
void Motor::standby()
{
   digitalWrite(Standby, LOW);     // Disable motor driver chip
}

/**
 * Checks if motor is currently moving
 * 
 * @return true if motor has any speed (forward or reverse), false if stopped
 */
bool Motor::isMoving() {
    return currentSpeed != 0;
}