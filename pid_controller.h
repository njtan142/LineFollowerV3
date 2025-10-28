#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

/**
 * PID Controller for Line Following
 * Implements Proportional-Integral-Derivative control
 */
class PIDController {
private:
    float kp;  // Proportional gain
    float ki;  // Integral gain
    float kd;  // Derivative gain
    
    float lastError;
    float integral;
    uint32_t lastTime;
    
    float maxIntegral;
    float maxOutput;
    
public:
    PIDController(float p = 0.1, float i = 0.0, float d = 0.0) 
        : kp(p), ki(i), kd(d), lastError(0), integral(0), lastTime(0) {
        maxIntegral = 1000.0;
        maxOutput = 255.0;
    }
    
    void setGains(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }
    
    void setMaxIntegral(float max) {
        maxIntegral = max;
    }
    
    void setMaxOutput(float max) {
        maxOutput = max;
    }
    
    float getKp() const { return kp; }
    float getKi() const { return ki; }
    float getKd() const { return kd; }
    
    // Calculate PID output
    float compute(float setpoint, float measured) {
        uint32_t now = millis();
        float dt = (now - lastTime) / 1000.0;  // Convert to seconds
        
        if (dt <= 0) dt = 0.01;  // Prevent division by zero
        if (lastTime == 0) dt = 0.01;  // First run
        
        lastTime = now;
        
        // Calculate error
        float error = setpoint - measured;
        
        // Proportional term
        float pTerm = kp * error;
        
        // Integral term
        integral += error * dt;
        // Anti-windup: clamp integral
        if (integral > maxIntegral) integral = maxIntegral;
        if (integral < -maxIntegral) integral = -maxIntegral;
        float iTerm = ki * integral;
        
        // Derivative term
        float derivative = (error - lastError) / dt;
        float dTerm = kd * derivative;
        
        // Save error for next iteration
        lastError = error;
        
        // Calculate total output
        float output = pTerm + iTerm + dTerm;
        
        // Clamp output
        if (output > maxOutput) output = maxOutput;
        if (output < -maxOutput) output = -maxOutput;
        
        return output;
    }
    
    // Reset PID state
    void reset() {
        lastError = 0;
        integral = 0;
        lastTime = 0;
    }
};

#endif
