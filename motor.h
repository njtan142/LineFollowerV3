#ifndef MOTOR_H
#define MOTOR_H

#include "enums.h"

// Forward declaration
struct Settings;

class Motor {
    public:
        Motor(int in1Pin, int in2Pin, int pwmPin, int standbyPin, int alignmentOffset, const Settings* settings);
        void brake();
        void standby();
        void setState(MotorState state);
        void update(unsigned long deltaTime);
        bool isMoving();
        void setSpeed(int targetSpeed); // NEW: set target speed 0..255
        void setAlignmentOffset(int offset); // Set alignment offset
    private:
        int speed;        // target speed (0..255)
        int direction;
        int IN1_PIN;
        int IN2_PIN;
        int PWM_PIN;
        int Standby;
        int alignmentOffset;
        const Settings* settings;  // Pointer to settings
        void fwd(int speed);
        void rev(int speed);
        MotorState currentState;
        int currentSpeed; // signed: +fwd, -rev
};

#endif