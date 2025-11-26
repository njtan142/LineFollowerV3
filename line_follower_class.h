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
    int lastKnownPosition;
    bool lastButtonState;
    int baseSpeed = 60;
    int turnSpeed = 50;
    int backwardBias = -15;
    unsigned long turnTimeout = 1000;
    int minSensorsForLine = 2;
    unsigned long lineFoundDelay = 50;
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

  /**
   * Display line found information and wait for button press
   */
  void displayLineFoundInfo() {
    u8x8.clear();
    u8x8.setCursor(0, 0);
    u8x8.print("LINE FOUND!");
    
    u8x8.setCursor(0, 2);
    u8x8.print("POS:");
    u8x8.print(state.linePosition);
    u8x8.print(" S:");
    u8x8.print(lineSensor.getBlackSensorCount());
    
    u8x8.setCursor(0, 4);
    const int* sensorValues = lineSensor.getSensorValues();
    for (int i = 0; i < 8; i++) {
      u8x8.print(sensorValues[i] > lineSensor.getSensorThreshold()[i] ? "1" : "0");
    }
    
    // Wait for button press to continue
    state.lastButtonState = digitalRead(HAL::UIPins::BUTTON);
    while (true) {
      if (checkForButtonPress()) {
        break;
      }
      delay(50);
    }
    
    u8x8.clear();
  }

  /**
   * Execute aggressive turn until line is found
   */
  void executeAggressiveTurn(bool turnLeft, PIDController& pid) {
    int leftSpeed, rightSpeed;
    
    // Set turn direction with backward bias
    if (turnLeft) {
      leftSpeed = -state.turnSpeed + state.backwardBias;
      rightSpeed = state.turnSpeed + state.backwardBias;
    } else {
      rightSpeed = -state.turnSpeed + state.backwardBias;
      leftSpeed = state.turnSpeed + state.backwardBias;
    }
    
    leftMotor->setSpeed(leftSpeed);
    rightMotor->setSpeed(rightSpeed);
    leftMotor->update();
    rightMotor->update();
    
    Timer turnTimer;
    turnTimer.start();
    bool lineFound = false;
    
    while (!lineFound && turnTimer.elapsed() < state.turnTimeout) {
      lineSensor.readSensors();
      int tempPosition = lineSensor.getPosition();
      int tempBlackCount = lineSensor.getBlackSensorCount();
      
      // Exit turn if line is found with good sensor coverage and centered
      // Bias towards center sensors (position closer to 0)
      if (tempBlackCount >= state.minSensorsForLine && abs(tempPosition) < 2000) {
        state.linePosition = tempPosition;
        lineFound = true;
        pid.reset(); // Reset PID integral when back on line
        leftMotor->brake();
        rightMotor->brake();
        delay(state.lineFoundDelay);
        // displayLineFoundInfo();
        DEBUG_PRINTLN("Line found - PID reset");
      }
    }
    
    // If timeout, try opposite direction
    if (!lineFound) {
      DEBUG_PRINTLN("Turn timeout - trying opposite direction");
      
      // Reverse direction
      leftSpeed = -leftSpeed;
      rightSpeed = -rightSpeed;
      
      leftMotor->setSpeed(leftSpeed);
      rightMotor->setSpeed(rightSpeed);
      leftMotor->update();
      rightMotor->update();
      
      turnTimer.start();
      
      while (!lineFound) {
        lineSensor.readSensors();
        int tempPosition = lineSensor.getPosition();
        int tempBlackCount = lineSensor.getBlackSensorCount();
        
        // Exit turn if line is found with good sensor coverage and centered
        if (tempBlackCount >= state.minSensorsForLine && abs(tempPosition) < 2000) {
          state.linePosition = tempPosition;
          lineFound = true;
          leftMotor->brake();
          rightMotor->brake();
          delay(state.lineFoundDelay);
          pid.reset(); // Reset PID integral when back on line
          // displayLineFoundInfo();
          DEBUG_PRINTLN("Line found - PID reset");
        }
      }
    }
    
    leftMotor->brake();
    rightMotor->brake();
  }

public:
  /**
   * Run simple line following with PID control
   */
  void run() {
    // Display startup message
    displayStartupMessage();

    // Initialize PID controller
    float kp = (float)25 / settings.pidScale;
    float ki = (float)0 / settings.pidScale;
    float kd = (float)0 / settings.pidScale;
    
    PIDController pid(kp, ki, kd);
    pid.setMaxOutput(255.0);

    // Setup button and sensors
    pinMode(HAL::UIPins::BUTTON, INPUT_PULLUP);
    state.lastButtonState = digitalRead(HAL::UIPins::BUTTON);

    lineSensor.setThresholdRatio(ThresholdRatio::RATIO_15_16);
    
    // Initialize state
    state.lastKnownPosition = 0;
    
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

      // Check if line is lost
      if (blackCount == 0) {
        // Line lost - execute aggressive turn
        bool turnLeft = (state.lastKnownPosition > 0);
        executeAggressiveTurn(turnLeft, pid);
      } else {
        // Update last known position
        if (abs(state.linePosition) > 500) {
          state.lastKnownPosition = state.linePosition;
        }
        
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
      }

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
