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
    int baseSpeed = 110;
    int turnSpeed = 50;
    int backwardBias = -18;
    unsigned long turnTimeout = 1100;
    int minSensorsForLine = 2;
    unsigned long stateChangeDelay = 50;
    bool skipDisplayLineFoundInfo = true;
    bool skipDisplayLineLostInfo = true;
    int correctiveTurnMultiplier = 1.6;
    int maxCorrectiveTurnSpeed = 200;
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
   * Wait for button press
   */
  void waitForButtonPress() {
    state.lastButtonState = digitalRead(HAL::UIPins::BUTTON);
    while (true) {
      if (checkForButtonPress()) {
        break;
      }
      delay(50);
    }
  }

  /**
   * Display line found information and wait for button press
   */
  void displayLineFoundInfo() {
    if (state.skipDisplayLineFoundInfo) {
      return;
    }
    
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
    
    waitForButtonPress();
    u8x8.clear();
  }

  /**
   * Display line lost information and wait for button press
   */
  void displayLineLostInfo(bool turnLeft) {
    if (state.skipDisplayLineLostInfo) {
      return;
    }
    
    u8x8.clear();
    u8x8.setCursor(0, 0);
    u8x8.print("LINE LOST!");
    u8x8.setCursor(0, 2);
    u8x8.print("TURN ");
    u8x8.print(turnLeft ? "LEFT" : "RIGHT");
    u8x8.print(" ");
    u8x8.print(state.lastKnownPosition);
    u8x8.setCursor(0, 4);
    const int* sensorValues = lineSensor.getSensorValues();
    for (int i = 0; i < 8; i++) {
      u8x8.print(sensorValues[i] > lineSensor.getSensorThreshold()[i] ? "1" : "0");
    }
    
    waitForButtonPress();
  }

  /**
   * Check if line is found and perform sanity check before exiting turn
   */
  bool checkLineFoundWithSanityCheck(int blackCount, int position, PIDController& pid, bool isTurningLeft) {
    // Check if line detected with minimum sensors and centered position
    // Also verify position matches turn direction (left turn = negative position, right turn = positive position)
    // Reject if all sensors are black (11111111) - likely over the line or invalid reading
    bool positionMatchesTurnDirection = isTurningLeft ? (position < 0) : (position > 0);
    bool allSensorsBlack = (blackCount >= 6);
    
    if (blackCount >= state.minSensorsForLine && abs(position) < 2000 && positionMatchesTurnDirection && !allSensorsBlack) {
      leftMotor->brake();
      rightMotor->brake();
      delay(state.stateChangeDelay);
      
      // Sanity check: verify line is still visible
      lineSensor.readSensors();
      int sanityBlackCount = lineSensor.getBlackSensorCount();
      
      if (sanityBlackCount >= state.minSensorsForLine && sanityBlackCount < 8) {
        // Line still visible and not all sensors - confirmed found
        state.linePosition = lineSensor.getPosition();
        pid.reset(); // Reset PID integral when back on line
        displayLineFoundInfo();
        DEBUG_PRINTLN("Line found - PID reset");
        return true;
      } else {
        // Line lost during sanity check or all sensors black - false detection
        DEBUG_PRINTLN("Sanity check failed: Line lost or invalid, continuing turn");
        return false;
      }
    }
    return false;
  }

  /**
   * Execute turn in one direction with timeout
   */
  bool executeTurnDirection(int leftSpeed, int rightSpeed, PIDController& pid, bool isTurningLeft) {
    leftMotor->setSpeed(leftSpeed);
    rightMotor->setSpeed(rightSpeed);
    leftMotor->update();
    rightMotor->update();
    
    Timer turnTimer;
    turnTimer.start();
    
    while (turnTimer.elapsed() < state.turnTimeout) {
      lineSensor.readSensors();
      int tempPosition = lineSensor.getPosition();
      int tempBlackCount = lineSensor.getBlackSensorCount();
      
      if (checkLineFoundWithSanityCheck(tempBlackCount, tempPosition, pid, isTurningLeft)) {
        return true; // Line found
      }
    }
    
    return false; // Timeout - line not found
  }

  /**
   * Execute aggressive turn until line is found
   */
  void executeAggressiveTurn(bool turnLeft, PIDController& pid) {
    displayLineLostInfo(turnLeft);
    
    int leftSpeed, rightSpeed;
    
    // Set turn direction with backward bias
    if (turnLeft) {
      leftSpeed = -state.turnSpeed + state.backwardBias;
      rightSpeed = state.turnSpeed + state.backwardBias;
    } else {
      rightSpeed = -state.turnSpeed + state.backwardBias;
      leftSpeed = state.turnSpeed + state.backwardBias;
    }
    
    // Try first direction
    bool lineFound = executeTurnDirection(leftSpeed, rightSpeed, pid, turnLeft);
    
    // If timeout, try opposite direction
    if (!lineFound) {
      DEBUG_PRINTLN("Turn timeout - trying opposite direction");
      
      // Reverse direction with increased speed for corrective turn
      leftSpeed = -leftSpeed * state.correctiveTurnMultiplier;
      rightSpeed = -rightSpeed * state.correctiveTurnMultiplier;
      
      // Clamp to max corrective turn speed
      leftSpeed = constrain(leftSpeed, -state.maxCorrectiveTurnSpeed, state.maxCorrectiveTurnSpeed);
      rightSpeed = constrain(rightSpeed, -state.maxCorrectiveTurnSpeed, state.maxCorrectiveTurnSpeed);
      
      bool oppositeDirection = !turnLeft;
      
      // Try opposite direction with button check
      lineFound = executeTurnDirection(leftSpeed, rightSpeed, pid, oppositeDirection);
      
      // If still not found after both directions, keep searching but allow exit
      if (!lineFound) {
        leftMotor->setSpeed(leftSpeed);
        rightMotor->setSpeed(rightSpeed);
        leftMotor->update();
        rightMotor->update();
        
        while (!lineFound) {
          // Check for exit button even during search
          if (checkForButtonPress()) {
            leftMotor->brake();
            rightMotor->brake();
            return; // Exit aggressive turn
          }
          
          lineSensor.readSensors();
          int tempPosition = lineSensor.getPosition();
          int tempBlackCount = lineSensor.getBlackSensorCount();
          
          lineFound = checkLineFoundWithSanityCheck(tempBlackCount, tempPosition, pid, oppositeDirection);
        }
      }
    }
    
    // Motors are already braked by checkLineFoundWithSanityCheck
    // Main loop will resume PID control immediately
  }

public:
  /**
   * Run simple line following with PID control
   */
  void run() {
    // Display startup message
    displayStartupMessage();

    // Initialize PID controller
    float kp = (float)47.5 / settings.pidScale;
    float ki = (float)0 / settings.pidScale;
    float kd = (float)0.35 / settings.pidScale;
    
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

      // Read line position (already updated by aggressive turn if just recovered)
      lineSensor.readSensors();
      int blackCount = lineSensor.getBlackSensorCount();
      
      // Only update position if line is visible
      if (blackCount > 0) {
        state.linePosition = lineSensor.getPosition();
      }

      // Check if line is lost
      if (blackCount == 0) {
        // Line lost - brake and wait before state change
        leftMotor->brake();
        rightMotor->brake();
        delay(state.stateChangeDelay);
        
        // Sanity check: verify line is still lost
        lineSensor.readSensors();
        int sanityBlackCount = lineSensor.getBlackSensorCount();
        
        if (sanityBlackCount == 0) {
          // Line still lost - execute aggressive turn
          bool turnLeft = (state.lastKnownPosition > 0);
          executeAggressiveTurn(turnLeft, pid);
          
          // Move forward a bit to ensure proper line engagement
          leftMotor->setSpeed(state.baseSpeed);
          rightMotor->setSpeed(state.baseSpeed);
          leftMotor->update();
          rightMotor->update();
          delay(100);
          
          // After aggressive turn, continue to next iteration to read fresh sensors
          continue;
        } else {
          // Line found during sanity check - update position and continue normal operation
          state.linePosition = lineSensor.getPosition();
          DEBUG_PRINTLN("Sanity check: Line found, skipping aggressive turn");
        }
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
