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
 * Line Following Mode - Class-based Implementation
 */
class LineFollower {
private:
  // Turning states enum
  enum TurningState {
    PID_TURNING,      // Line is visible, use PID control
    TURN_LEFT,        // Line lost, turn left to find it
    TURN_RIGHT        // Line lost, turn right to find it
  };

  // State data
  struct State {
    int linePosition;
    int lastKnownPosition;
    int positionHistory[3];
    int historyIndex;
    TurningState currentTurningState;
    int leftTurnCount;
    int rightTurnCount;
    int forwardCount;
    bool lastButtonState;
    const int straightThreshold = 1000;
    const unsigned long movementDuration = 40;
    const unsigned long loopDelay = 50;
  };

  State state;

  /**
   * Display the initial startup message for line following mode
   */
  void displayStartupMessage() {
    u8x8.clear();
    u8x8.setCursor(0, 0);
    u8x8.print("LINE FOLLOW");
    u8x8.setCursor(0, 2);
    u8x8.print("PRESS TO STOP");
    u8x8.setCursor(0, 4);
    u8x8.print("STARTING...");
    delay(1000);
    delay(100);
  }

  /**
   * Initialize and display the runtime display labels
   */
  void initializeDisplay() {
    u8x8.clear();
    u8x8.setCursor(0, 0);
    u8x8.print("POS  S:   BLK");
    u8x8.setCursor(0, 4);
    u8x8.print("TL:  TR:  F:");
  }

  /**
   * Check if the user has pressed the button to exit line following
   * Returns true if button was pressed
   */
  bool checkForButtonPressToExit() {
    bool currentButtonState = digitalRead(HAL::UIPins::BUTTON);
    if (state.lastButtonState == HIGH && currentButtonState == LOW) {
      DEBUG_PRINTLN("Button pressed - exiting line following");
      state.lastButtonState = currentButtonState;
      return true;
    }
    state.lastButtonState = currentButtonState;
    return false;
  }

  /**
   * Apply curve correction logic to prevent oscillation on curves
   */
  void applyCurveCorrectionIfNeeded() {
    // Update position history (shift old values)
    state.positionHistory[0] = state.positionHistory[1]; // 2 iterations ago
    state.positionHistory[1] = state.positionHistory[2]; // 1 iteration ago
    state.positionHistory[2] = state.linePosition;       // current
    
    if (state.historyIndex >= 2) {
      bool wasTurning = abs(state.positionHistory[0]) > state.straightThreshold;
      bool wentStraight = abs(state.positionHistory[1]) < state.straightThreshold;
      bool nowOpposite = (state.positionHistory[0] > 0 && state.positionHistory[2] < -state.straightThreshold) ||
                         (state.positionHistory[0] < 0 && state.positionHistory[2] > state.straightThreshold);
      
      if (wasTurning && wentStraight && nowOpposite) {
        // Override: continue in the original curve direction
        state.linePosition = state.positionHistory[0];
        state.positionHistory[2] = state.linePosition;
        
        DEBUG_PRINTLN("CURVE CORRECTION APPLIED!");
        DEBUG_PRINT("  Was turning: ");
        DEBUG_PRINT(state.positionHistory[0]);
        DEBUG_PRINT(" -> Straight: ");
        DEBUG_PRINT(state.positionHistory[1]);
        DEBUG_PRINT(" -> Corrected from: ");
        DEBUG_PRINT(state.positionHistory[2]);
        DEBUG_PRINT(" to: ");
        DEBUG_PRINTLN(state.linePosition);
      }
    }
    
    state.historyIndex++;
  }

  /**
   * Determine which turning state the robot should be in based on sensor readings
   */
  void determineTurningState(int blackCount) {
    if (blackCount > 0) {
      // Line is visible - use PID control
      state.currentTurningState = PID_TURNING;
      state.lastKnownPosition = state.linePosition;
    } else {
      // Line is lost - turn in direction of last known position
      if (state.lastKnownPosition < 0) {
        state.currentTurningState = TURN_RIGHT;
      } else {
        state.currentTurningState = TURN_LEFT;
      }
    }
  }

  /**
   * Calculate motor speeds using PID control for line following
   */
  void calculatePIDMotorSpeeds(PIDController& pid, int& leftSpeed, int& rightSpeed,
                                bool& isTurningLeft, bool& isTurningRight, bool& isGoingForward) {
    // Calculate PID correction (setpoint is 0 = line centered)
    float correction = pid.compute(0, state.linePosition);
    
    // Apply correction using differential steering
    leftSpeed = settings.baseSpeed + correction;
    rightSpeed = settings.baseSpeed - correction;
    
    // Clamp speeds to valid range
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // Determine turn direction for counting
    if (state.linePosition > state.straightThreshold) {
      isTurningLeft = true;
    } else if (state.linePosition < -state.straightThreshold) {
      isTurningRight = true;
    } else {
      isGoingForward = true;
    }
  }

  /**
   * Execute aggressive left turn until line is found
   * Returns true if should exit line following, false otherwise
   */
  bool executeAggressiveLeftTurnUntilLineFound() {
    int leftSpeed = -settings.baseSpeed;
    int rightSpeed = settings.baseSpeed;
    
    leftMotor->setSpeed(leftSpeed);
    rightMotor->setSpeed(rightSpeed);
    leftMotor->update();
    rightMotor->update();
    
    bool lineFound = false;
    const int minSensorsForLine = 3;
    
    while (!lineFound) {
      // Check for button press to exit
      if (checkForButtonPressToExit()) {
        leftMotor->brake();
        rightMotor->brake();
        return true; // Signal to exit line following
      }
      
      lineSensor.readSensors();
      int tempPosition = lineSensor.getPosition();
      int tempBlackCount = lineSensor.getBlackSensorCount();
      
      // Exit turn if line is centered or we have good sensor coverage
      if ((abs(tempPosition) < state.straightThreshold && tempBlackCount >= minSensorsForLine) || 
          tempBlackCount >= 4) {
        state.linePosition = tempPosition;
        lineFound = true;
      }
      
      delay(10);
    }
    
    leftMotor->brake();
    rightMotor->brake();
    return false; // Continue line following
  }

  /**
   * Execute aggressive right turn until line is found
   * Returns true if should exit line following, false otherwise
   */
  bool executeAggressiveRightTurnUntilLineFound() {
    int leftSpeed = settings.baseSpeed;
    int rightSpeed = -settings.baseSpeed;
    
    leftMotor->setSpeed(leftSpeed);
    rightMotor->setSpeed(rightSpeed);
    leftMotor->update();
    rightMotor->update();
    
    bool lineFound = false;
    const int minSensorsForLine = 3;
    
    while (!lineFound) {
      // Check for button press to exit
      if (checkForButtonPressToExit()) {
        leftMotor->brake();
        rightMotor->brake();
        return true; // Signal to exit line following
      }
      
      lineSensor.readSensors();
      int tempPosition = lineSensor.getPosition();
      int tempBlackCount = lineSensor.getBlackSensorCount();
      
      // Exit turn if line is centered or we have good sensor coverage
      if ((abs(tempPosition) < state.straightThreshold && tempBlackCount >= minSensorsForLine) || 
          tempBlackCount >= 4) {
        state.linePosition = tempPosition;
        lineFound = true;
      }
      
      delay(10);
    }
    
    leftMotor->brake();
    rightMotor->brake();
    return false; // Continue line following
  }

  /**
   * Update turn counters based on current movement direction
   */
  void updateTurnCounters(bool isTurningLeft, bool isTurningRight, bool isGoingForward) {
    if (isTurningLeft) {
      state.leftTurnCount++;
      if (state.leftTurnCount > 3) {
        state.rightTurnCount = 0;
        state.forwardCount = 0;
      }
    } else if (isTurningRight) {
      state.rightTurnCount++;
      if (state.rightTurnCount > 3) {
        state.leftTurnCount = 0;
        state.forwardCount = 0;
      }
    } else if (isGoingForward) {
      state.forwardCount++;
      if (state.forwardCount > 3) {
        state.leftTurnCount = 0;
        state.rightTurnCount = 0;
      }
    }
  }

  /**
   * Execute PID motor movement and average sensor readings during movement
   */
  void executePIDMotorMovementAndReadSensors(int leftSpeed, int rightSpeed) {
    leftMotor->setSpeed(leftSpeed);
    rightMotor->setSpeed(rightSpeed);
    leftMotor->update();
    rightMotor->update();
    
    Timer movementTimer;
    movementTimer.start();
    long positionSum = 0;
    int readingCount = 0;
    
    while (movementTimer.elapsed() < state.movementDuration) {
      lineSensor.readSensors();
      positionSum += lineSensor.getPosition();
      readingCount++;
    }
    
    state.linePosition = (readingCount > 0) ? (positionSum / readingCount) : state.linePosition;
    
    leftMotor->brake();
    rightMotor->brake();
  }

  /**
   * Update the OLED display with current line following status
   */
  void updateDisplay(int avgBlackCount) {
    // Row 0 (line 2): Values for POS, State, BLK
    u8x8.setCursor(0, 2);
    u8x8.print("     ");
    u8x8.setCursor(0, 2);
    u8x8.print(state.linePosition);
    
    u8x8.setCursor(5, 2);
    switch (state.currentTurningState) {
      case PID_TURNING:
        if (state.linePosition > state.straightThreshold) {
          u8x8.print("PL ");
        } else if (state.linePosition < -state.straightThreshold) {
          u8x8.print("PR ");
        } else {
          u8x8.print("PF ");
        }
        break;
      case TURN_LEFT:
        u8x8.print("LFT");
        break;
      case TURN_RIGHT:
        u8x8.print("RGT");
        break;
    }
    
    u8x8.setCursor(10, 2);
    u8x8.print("   ");
    u8x8.setCursor(10, 2);
    u8x8.print(avgBlackCount);
    
    // Row 2 (line 6): Turn counters
    u8x8.setCursor(3, 6);
    u8x8.print("   ");
    u8x8.setCursor(3, 6);
    u8x8.print(state.leftTurnCount);
    
    u8x8.setCursor(8, 6);
    u8x8.print("   ");
    u8x8.setCursor(8, 6);
    u8x8.print(state.rightTurnCount);
    
    u8x8.setCursor(13, 6);
    u8x8.print("  ");
    u8x8.setCursor(13, 6);
    u8x8.print(state.forwardCount);
  }

  /**
   * Output debug information about current line following state
   */
  void printDebugInfo(int avgBlackCount, int leftSpeed, int rightSpeed) {
    DEBUG_PRINT("STATE: ");
    switch (state.currentTurningState) {
      case PID_TURNING:
        if (state.linePosition > state.straightThreshold) {
          DEBUG_PRINT("PID_LEFT");
        } else if (state.linePosition < -state.straightThreshold) {
          DEBUG_PRINT("PID_RIGHT");
        } else {
          DEBUG_PRINT("PID_FWD");
        }
        break;
      case TURN_LEFT:
        DEBUG_PRINT("LEFT");
        break;
      case TURN_RIGHT:
        DEBUG_PRINT("RIGHT");
        break;
    }
    DEBUG_PRINT(" | POS: ");
    DEBUG_PRINT(state.linePosition);
    DEBUG_PRINT(" | BLACK: ");
    DEBUG_PRINT(avgBlackCount);
    DEBUG_PRINT(" | LAST: ");
    DEBUG_PRINT(state.lastKnownPosition);
    DEBUG_PRINT(" | L: ");
    DEBUG_PRINT(leftSpeed);
    DEBUG_PRINT(" R: ");
    DEBUG_PRINT(rightSpeed);
    DEBUG_PRINT(" | TL: ");
    DEBUG_PRINT(state.leftTurnCount);
    DEBUG_PRINT(" TR: ");
    DEBUG_PRINT(state.rightTurnCount);
    DEBUG_PRINT(" F: ");
    DEBUG_PRINTLN(state.forwardCount);
  }

  /**
   * Display exit message and return to main menu
   */
  void displayExitMessage() {
    leftMotor->brake();
    rightMotor->brake();
    u8x8.setPowerSave(0);
    u8x8.clear();
    u8x8.setCursor(0, 3);
    u8x8.print("STOPPED");
    delay(1000);
    u8x8.clear();
    mainMenu.requestRedraw();
    DEBUG_PRINTLN("Line following complete");
  }

public:
  /**
   * Run the line following algorithm
   */
  void run() {
    DEBUG_PRINTLN("Starting line following mode...");

    // Display startup message
    displayStartupMessage();

    // Initialize PID controller with settings from EEPROM
    float kp = (float)settings.kp / settings.pidScale;
    float ki = (float)settings.ki / settings.pidScale;
    float kd = (float)settings.kd / settings.pidScale;
    
    PIDController pid(kp, ki, kd);
    pid.setMaxOutput(510.0);
    pid.setMaxIntegral(10000.0);

    DEBUG_PRINT("PID Gains - Kp: ");
    DEBUG_PRINT(kp);
    DEBUG_PRINT(" Ki: ");
    DEBUG_PRINT(ki);
    DEBUG_PRINT(" Kd: ");
    DEBUG_PRINTLN(kd);

    settings.movementType = MovementType::AGGRESSIVE;
    deltaTimer.start();

    // Setup button for exit detection
    pinMode(HAL::UIPins::BUTTON, INPUT_PULLUP);

    // Initialize line following state
    lineSensor.readSensors();
    state.linePosition = lineSensor.getPosition();
    state.lastKnownPosition = 0;
    state.positionHistory[0] = 0;
    state.positionHistory[1] = 0;
    state.positionHistory[2] = 0;
    state.historyIndex = 0;
    state.currentTurningState = PID_TURNING;
    state.leftTurnCount = 0;
    state.rightTurnCount = 0;
    state.forwardCount = 0;
    state.lastButtonState = digitalRead(HAL::UIPins::BUTTON);

    // Initialize display
    initializeDisplay();

    // Main control loop - runs until button is pressed
    while (true)
    {
      // Check for button press to exit
      if (checkForButtonPressToExit()) {
        break;
      }

      // Calculate time since last update
      deltaTimer.elapsed();
      deltaTimer.start();

      // Apply curve correction to prevent oscillation
      applyCurveCorrectionIfNeeded();
      
      // Read sensor data and determine turning state
      int blackCount = lineSensor.getBlackSensorCount();
      determineTurningState(blackCount);

      int leftSpeed = 0, rightSpeed = 0;
      bool isTurningLeft = false;
      bool isTurningRight = false;
      bool isGoingForward = false;
      bool skipNormalControl = false;
      
      // Execute behavior based on current state
      switch (state.currentTurningState) {
        case PID_TURNING:
          calculatePIDMotorSpeeds(pid, leftSpeed, rightSpeed, 
                                  isTurningLeft, isTurningRight, isGoingForward);
          break;
          
        case TURN_LEFT:
          if (executeAggressiveLeftTurnUntilLineFound()) {
            goto exitLineFollowing; // Exit if button pressed during turn
          }
          isTurningLeft = true;
          skipNormalControl = true;
          break;
          
        case TURN_RIGHT:
          if (executeAggressiveRightTurnUntilLineFound()) {
            goto exitLineFollowing; // Exit if button pressed during turn
          }
          isTurningRight = true;
          skipNormalControl = true;
          break;
      }
      
      // Update turn counters based on movement direction
      updateTurnCounters(isTurningLeft, isTurningRight, isGoingForward);

      // Execute PID motor movement (only if not in aggressive turn mode)
      if (!skipNormalControl) {
        executePIDMotorMovementAndReadSensors(leftSpeed, rightSpeed);
      }

      // Ensure motors are stopped before updating display
      leftMotor->brake();
      rightMotor->brake();

      // Get current black sensor count for display
      int avgBlackCount = lineSensor.getBlackSensorCount();

      // Update display with current status
      updateDisplay(avgBlackCount);

      // Print debug information
      printDebugInfo(avgBlackCount, leftSpeed, rightSpeed);

      delay(state.loopDelay);
    }

exitLineFollowing:
    // Display exit message and return to menu
    displayExitMessage();
  }
};

#endif // LINE_FOLLOWER_CLASS_H
