#ifndef LINE_FOLLOWER_CLASS_H
#define LINE_FOLLOWER_CLASS_H

#include <Arduino.h>
#include "pid_controller.h"
#include "line_sensor.h"
#include "motor.h"
#include "Settings.h"
#include "timer.h"
#include "hal.h"

// Forward declarations of external objects
extern U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8;
extern Settings settings;
extern LineSensor lineSensor;
extern Motor *leftMotor;
extern Motor *rightMotor;
extern Timer deltaTimer;
extern MenuSystem mainMenu;

/**
 * Simple Line Following Mode - PID Control
 */
class LineFollower {
private:
  // State data
  struct State {
    int linePosition;
    bool lastButtonState;
    int baseSpeed = 100;
  };

  State state;

  /**
   * Display the initial startup message
   */
  void displayStartupMessage() {
    u8x8.clear();
    u8x8.setCursor(0, 0);
    u8x8.print("LINE FOLLOW");
    u8x8.setCursor(0, 2);
    u8x8.print("Press to stop");
    delay(1000);
  }

  /**
   * Initialize the display labels
   */
  void initializeDisplay() {
    u8x8.clear();
    u8x8.setCursor(0, 0);
    u8x8.print("Pos:");
    u8x8.setCursor(0, 2);
    u8x8.print("Sensors:");
  }

  /**
   * Check if button was pressed to exit
   */
  bool checkForButtonPress() {
    bool currentButtonState = digitalRead(HAL::UIPins::BUTTON);
    if (state.lastButtonState == HIGH && currentButtonState == LOW) {
      state.lastButtonState = currentButtonState;
      return true;
    }
    state.lastButtonState = currentButtonState;
    return false;
  }

public:
  /**
   * Run simple line following with PID control
   */
  void run() {
    // Display startup message
    displayStartupMessage();

    // Initialize PID controller
    float kp = (float)10 / settings.pidScale;
    float ki = (float)0 / settings.pidScale;
    float kd = (float)0 / settings.pidScale;
    
    PIDController pid(kp, ki, kd);
    pid.setMaxOutput(255.0);

    // Setup button and sensors
    pinMode(HAL::UIPins::BUTTON, INPUT_PULLUP);
    state.lastButtonState = digitalRead(HAL::UIPins::BUTTON);
    
    // Initialize display
    initializeDisplay();

    // Main control loop
    while (true) {
      // Check for exit button
      if (checkForButtonPress()) {
        break;
      }

      // Read line position
      lineSensor.readSensors();
      state.linePosition = lineSensor.getPosition();
      int blackCount = lineSensor.getBlackSensorCount();

      // Calculate PID correction
      float correction = pid.compute(0, state.linePosition);
      
      // Apply differential steering
      int leftSpeed = state.baseSpeed + correction;
      int rightSpeed = state.baseSpeed - correction;
      
      // Clamp speeds
      leftSpeed = constrain(leftSpeed, -255, 255);
      rightSpeed = constrain(rightSpeed, -255, 255);

      // Set motor speeds
      leftMotor->setSpeed(leftSpeed);
      rightMotor->setSpeed(rightSpeed);
      leftMotor->update();
      rightMotor->update();

      delay(10);
    }

    // Stop motors and show exit message
    leftMotor->brake();
    rightMotor->brake();
    u8x8.clear();
    u8x8.setCursor(0, 3);
    u8x8.print("STOPPED");
    delay(1000);
    u8x8.clear();
    mainMenu.requestRedraw();
  }
};

#endif // LINE_FOLLOWER_CLASS_H
