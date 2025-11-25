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
    bool skipDisplay = true;
    bool useAggressiveTurning = true;
    bool useCurveCorrection = true;
    const int straightThreshold = 750;
    const unsigned long movementDuration = 10;
    const unsigned long loopDelay = 0;
    const unsigned long stateChangeDelay = 1000;
    
    // Dynamic speed control
    int currentBaseSpeed = 255;
    bool hasEnteredAggressiveTurn = false;
    unsigned long lastStraightSpeedIncrease = 0;
    
    // Last displayed values for change detection
    int lastDisplayedPosition = -99999;
    TurningState lastDisplayedState = PID_TURNING;
    int lastDisplayedBlackCount = -1;
    int lastDisplayedLeftTurnCount = -1;
    int lastDisplayedRightTurnCount = -1;
    int lastDisplayedForwardCount = -1;
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
    
    if (state.useCurveCorrection && state.historyIndex >= 2) {
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
  void determineTurningState(int blackCount, Motor* leftMotor, Motor* rightMotor) {
    TurningState previousState = state.currentTurningState;
    if (blackCount > 0) {
      // Line is visible - use PID control
      state.currentTurningState = PID_TURNING;
      state.lastKnownPosition = state.linePosition;
    } else {
      // Line is lost
      if (state.useAggressiveTurning) {
        // Use aggressive turning - turn in direction of last known position
        if (state.lastKnownPosition < 0) {
          state.currentTurningState = TURN_RIGHT;
        } else {
          state.currentTurningState = TURN_LEFT;
        }
        
        // Handle speed reduction when entering aggressive turn
        if (previousState == PID_TURNING) {
          if (!state.hasEnteredAggressiveTurn) {
            // First time entering aggressive turn - set speed to 170
            state.currentBaseSpeed = 170;
            state.hasEnteredAggressiveTurn = true;
            DEBUG_PRINTLN("First aggressive turn - speed set to 170");
          } else {
            // Subsequent turns - reduce by 5
            state.currentBaseSpeed -= 5;
            state.currentBaseSpeed = constrain(state.currentBaseSpeed, 150, 255);
            DEBUG_PRINT("Aggressive turn - speed reduced to: ");
            DEBUG_PRINTLN(state.currentBaseSpeed);
          }
          state.lastStraightSpeedIncrease = 0; // Reset straight speed timer
        }
      } else {
        // Keep using PID turning even when line is lost
        state.currentTurningState = PID_TURNING;
      }
    }
    if(previousState != state.currentTurningState){
        leftMotor->brake();
        rightMotor->brake();
        delay(state.stateChangeDelay);
    }
  }

  /**
   * Calculate motor speeds using PID control for line following
   */
  void calculatePIDMotorSpeeds(PIDController& pid, int& leftSpeed, int& rightSpeed,
                                bool& isTurningLeft, bool& isTurningRight, bool& isGoingForward) {
    // Calculate PID correction (setpoint is 0 = line centered)
    float correction = pid.compute(0, state.linePosition);
    
    // Check if position is within straight threshold and increase speed progressively
    if (abs(state.linePosition) < state.straightThreshold) {
      unsigned long currentTime = millis();
      if (state.lastStraightSpeedIncrease == 0) {
        state.lastStraightSpeedIncrease = currentTime;
      }
      
      // Add 10 speed per second (1000ms)
      if (currentTime - state.lastStraightSpeedIncrease >= 1000) {
        state.currentBaseSpeed += 10;
        state.currentBaseSpeed = constrain(state.currentBaseSpeed, 0, 255);
        state.lastStraightSpeedIncrease = currentTime;
        DEBUG_PRINT("Speed increased to: ");
        DEBUG_PRINTLN(state.currentBaseSpeed);
      }
      isGoingForward = true;
    } else {
      // Reset timer when not going straight
      state.lastStraightSpeedIncrease = 0;
    }
    
    // Apply correction using differential steering
    leftSpeed = state.currentBaseSpeed + correction;
    rightSpeed = state.currentBaseSpeed - correction;
    
    // Clamp speeds to valid range
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // Determine turn direction for counting
    if (state.linePosition > state.straightThreshold) {
      isTurningLeft = true;
    } else if (state.linePosition < -state.straightThreshold) {
      isTurningRight = true;
    }
  }

  /**
   * Execute aggressive turn until line is found
   * Returns true if should exit line following, false otherwise
   */
  bool executeAggressiveTurnUntilLineFound(TurningState direction) {
    int leftSpeed, rightSpeed;
    int correctionLeftSpeed, correctionRightSpeed;
    int minSensorsForLine;
    int turnSpeed = 200;
    int backwardBias = -20; // Backward movement component
    const unsigned long turnTimeout = 400; // Maximum time to turn in one direction (ms)
    
    // Set turn direction and correction direction with backward bias
    if (direction == TURN_LEFT) {
      // Turn left while moving backward: left motor more negative, right motor less positive
      leftSpeed = -turnSpeed + backwardBias;
      rightSpeed = turnSpeed + backwardBias;
      correctionLeftSpeed = turnSpeed + backwardBias;   // Correction: turn right while backward
      correctionRightSpeed = -turnSpeed + backwardBias;
      minSensorsForLine = 3;
    } else { // TURN_RIGHT
      // Turn right while moving backward: right motor more negative, left motor less positive
      leftSpeed = turnSpeed + backwardBias;
      rightSpeed = -turnSpeed + backwardBias;
      correctionLeftSpeed = -turnSpeed + backwardBias;  // Correction: turn left while backward
      correctionRightSpeed = turnSpeed + backwardBias;
      minSensorsForLine = 3;
    }
    
    leftMotor->setSpeed(leftSpeed);
    rightMotor->setSpeed(rightSpeed);
    leftMotor->update();
    rightMotor->update();
    
    bool lineFound = false;
    Timer turnTimer;
    turnTimer.start();
    
    while (!lineFound) {
      lineSensor.readSensors();
      int tempPosition = lineSensor.getPosition();
      int tempBlackCount = lineSensor.getBlackSensorCount();
      
      // Exit turn if line is centered or we have good sensor coverage
      if ((abs(tempPosition) < state.straightThreshold && tempBlackCount >= minSensorsForLine) || 
          tempBlackCount >= 4) {
        state.linePosition = tempPosition;
        lineFound = true;
        DEBUG_PRINTLN("Line found during turn!");
      }
      
      // If timeout reached, switch direction
      if (turnTimer.elapsed() >= turnTimeout) {
        DEBUG_PRINTLN("Turn timeout - switching direction!");
        leftMotor->brake();
        rightMotor->brake();
        delay(50);
        
        // Reverse direction
        leftMotor->setSpeed(correctionLeftSpeed);
        rightMotor->setSpeed(correctionRightSpeed);
        leftMotor->update();
        rightMotor->update();
        
        // Update last known position to opposite direction
        state.lastKnownPosition = -state.lastKnownPosition;
        
        turnTimer.start(); // Reset timer for the new direction
      }
    }
    
    return false; // Continue line following
  }

  /**
   * Update turn counters based on current movement direction
   */
  void updateTurnCounters(bool isTurningLeft, bool isTurningRight, bool isGoingForward) {
    int countLimit = 40; // Increased count limit for more stability
    if (isTurningLeft) {
      state.leftTurnCount++;
      if (state.leftTurnCount > countLimit) {
        state.rightTurnCount = 0;
        state.forwardCount = 0;
      }
    } else if (isTurningRight) {
      state.rightTurnCount++;
      if (state.rightTurnCount > countLimit) {
        state.leftTurnCount = 0;
        state.forwardCount = 0;
      }
    } else if (isGoingForward) {
      state.forwardCount++;
      if (state.forwardCount > countLimit) {
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
    // Skip display updates if flag is set
    if (state.skipDisplay) {
      return;
    }
    
    // Update position only if changed
    if (state.linePosition != state.lastDisplayedPosition) {
    //   u8x8.setCursor(0, 2);
    //   u8x8.print("     ");
      u8x8.setCursor(0, 2);
      u8x8.print(state.linePosition);
      state.lastDisplayedPosition = state.linePosition;
    }
    
    // Update state only if changed
    if (state.currentTurningState != state.lastDisplayedState || 
        state.linePosition != state.lastDisplayedPosition) {
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
      state.lastDisplayedState = state.currentTurningState;
    }
    
    // Update black count only if changed
    if (avgBlackCount != state.lastDisplayedBlackCount) {
    //   u8x8.setCursor(10, 2);
    //   u8x8.print("   ");
      u8x8.setCursor(10, 2);
      u8x8.print(avgBlackCount);
      state.lastDisplayedBlackCount = avgBlackCount;
    }
    
    // Update left turn count only if changed
    if (state.leftTurnCount != state.lastDisplayedLeftTurnCount) {
    //   u8x8.setCursor(3, 6);
    //   u8x8.print("   ");
      u8x8.setCursor(3, 6);
      u8x8.print(state.leftTurnCount);
      state.lastDisplayedLeftTurnCount = state.leftTurnCount;
    }
    
    // Update right turn count only if changed
    if (state.rightTurnCount != state.lastDisplayedRightTurnCount) {
    //   u8x8.setCursor(8, 6);
    //   u8x8.print("   ");
      u8x8.setCursor(8, 6);
      u8x8.print(state.rightTurnCount);
      state.lastDisplayedRightTurnCount = state.rightTurnCount;
    }
    
    // Update forward count only if changed
    if (state.forwardCount != state.lastDisplayedForwardCount) {
    //   u8x8.setCursor(13, 6);
    //   u8x8.print("  ");
      u8x8.setCursor(13, 6);
      u8x8.print(state.forwardCount);
      state.lastDisplayedForwardCount = state.forwardCount;
    }
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
    float kp = (float)55 / settings.pidScale;
    float ki = (float)10   / settings.pidScale;
    float kd = (float)1 / settings.pidScale;
    
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
    lineSensor.setThresholdRatio(ThresholdRatio::RATIO_15_16);
    // state.loopDelay = 10;
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
    
    // Initialize dynamic speed control
    state.currentBaseSpeed = 255;
    state.hasEnteredAggressiveTurn = false;
    state.lastStraightSpeedIncrease = 0;

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
      determineTurningState(blackCount, leftMotor, rightMotor);

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
          if (executeAggressiveTurnUntilLineFound(TURN_LEFT)) {
            goto exitLineFollowing; // Exit if button pressed during turn
          }
          isTurningLeft = true;
          skipNormalControl = true;
          break;
          
        case TURN_RIGHT:
          if (executeAggressiveTurnUntilLineFound(TURN_RIGHT)) {
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
