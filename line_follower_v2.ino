/**
 * Line Follower Robot - Main Menu Implementation
 * Features:
 * - Main menu with potentiometer scrolling
 * - Button selection
 * - Clean U8x8 text-based display
 */

// Debug configuration - set to 0 to disable serial debug output
#define DEBUG_SERIAL 0

#if DEBUG_SERIAL
  #define DEBUG_BEGIN(baud) Serial.begin(baud)
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_BEGIN(baud)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#include <U8x8lib.h>
#include "menu_system.h"
#include "settings_menu.h"
#include "pid_edit.h"
#include "line_sensor.h"
#include "motor.h"
#include "Settings.h"
#include "eeprom_manager.h"
#include "hal.h"
#include "timer.h"

// Display Pins (Software I2C for SSD1306 OLED)
#define SOFT_SDA 2
#define SOFT_SCL 3

// Input Pins
#define POT_PIN A7
#define BUTTON_PIN 5
#define POT_ENABLE_PIN 9

// Create display object - U8x8 is text-only mode (16x8 characters)
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(SOFT_SCL, SOFT_SDA, U8X8_PIN_NONE);

// Settings object
Settings settings;

// Sensor object
LineSensor lineSensor;

// Motor objects (will be initialized in setup)
Motor* leftMotor = nullptr;
Motor* rightMotor = nullptr;

// Timer for deltaTime calculation
Timer deltaTimer;

// UI State
enum UIState {
  MAIN_MENU,
  SETTINGS_MENU,
  PID_EDIT
};
UIState currentState = MAIN_MENU;
bool settingsMenuInitialized = false;
bool pidEditInitialized = false;

// Menu items
const char* menuItems[] = {
  "CFG",      // Settings/Configuration
  "CALIBRT",  // Calibration
  "RUN"       // Start running
};
const uint8_t MENU_ITEM_COUNT = 3;

// Create menu system
MenuSystem mainMenu(&u8x8, menuItems, MENU_ITEM_COUNT, "MENU", 3, POT_PIN, BUTTON_PIN, POT_ENABLE_PIN);

// Create settings menu
SettingsMenu settingsMenu(&u8x8, POT_PIN, BUTTON_PIN, POT_ENABLE_PIN);

// Create PID edit screen (will be initialized when needed)
PIDEdit* pidEdit = nullptr;

void setup() {
  DEBUG_BEGIN(9600);
  delay(100);
  DEBUG_PRINTLN("Line Follower - Main Menu");
  
  // Load settings from EEPROM
  settings.load();
  
  // Initialize display
  u8x8.begin();
  u8x8.setPowerSave(0);
  
  // Initialize motors using HAL pin definitions
  leftMotor = new Motor(
    HAL::MotorPins::Left::IN1, 
    HAL::MotorPins::Left::IN2, 
    HAL::MotorPins::Left::PWM, 
    HAL::MotorPins::STBY, 
    settings.alignmentOffset, 
    &settings
  );
  rightMotor = new Motor(
    HAL::MotorPins::Right::IN1, 
    HAL::MotorPins::Right::IN2, 
    HAL::MotorPins::Right::PWM, 
    HAL::MotorPins::STBY, 
    0, 
    &settings
  );
  
  // Initialize only the main menu (settings menu will be initialized when needed)
  mainMenu.begin();
  
  DEBUG_PRINTLN("System ready");
}

void loop() {
  switch (currentState) {
    case MAIN_MENU:
      handleMainMenu();
      break;
    case SETTINGS_MENU:
      handleSettingsMenu();
      break;
    case PID_EDIT:
      handlePIDEdit();
      break;
  }
  
  delay(50);  // Small delay for stability
}

void handleMainMenu() {
  // Update menu system (handles input and display)
  mainMenu.update();
  
  // Check if selection changed (optional - for debugging/logging)
  if (mainMenu.hasNewSelection()) {
    DEBUG_PRINT("Main Menu - Selection: ");
    DEBUG_PRINTLN(mainMenu.getSelectedItem());
  }
  
  // Handle button press
  if (mainMenu.isButtonJustPressed()) {
    uint8_t selection = mainMenu.getSelection();
    DEBUG_PRINT("Main Menu Button pressed! Selected: ");
    DEBUG_PRINTLN(mainMenu.getSelectedItem());
    
    switch (selection) {
      case 0:  // CFG
        DEBUG_PRINTLN("-> Entering Settings Menu");
        currentState = SETTINGS_MENU;
        if (!settingsMenuInitialized) {
          settingsMenu.begin();
          settingsMenuInitialized = true;
        } else {
          settingsMenu.requestRedraw();
        }
        // Clear any pending button state after menu transition
        settingsMenu.update();
        break;
      case 1:  // CALIBRT
        DEBUG_PRINTLN("-> Starting Calibration");
        performCalibration();
        break;
      case 2:  // RUN
        DEBUG_PRINTLN("-> Starting Run Mode");
        // TODO: Implement run mode
        break;
    }
  }
}

void handleSettingsMenu() {
  // Update settings menu
  settingsMenu.update();
  
  // Check if selection changed
  if (settingsMenu.hasNewSelection()) {
    DEBUG_PRINT("Settings Menu - Selection: ");
    DEBUG_PRINTLN(settingsMenu.getSelectedItem());
  }
  
  // Handle button press
  if (settingsMenu.isButtonJustPressed()) {
    uint8_t selection = settingsMenu.getSelection();
    DEBUG_PRINT("Settings Button pressed! Selected: ");
    DEBUG_PRINTLN(settingsMenu.getSelectedItem());
    
    switch (selection) {
      case 0:  // PID
        DEBUG_PRINTLN("-> Navigate to PID Edit screen");
        currentState = PID_EDIT;
        pidEditInitialized = false;  // Force re-initialization
        delay(100);  // Short delay to ensure button is released
        break;
      case 1:  // Test
        DEBUG_PRINTLN("-> Navigate to Test menu");
        // TODO: Implement test menu
        break;
      case 2:  // Thresh
        DEBUG_PRINTLN("-> Navigate to Threshold Select");
        // TODO: Implement threshold select
        break;
      case 3:  // Profil
        DEBUG_PRINTLN("-> Navigate to PID Profiles");
        // TODO: Implement PID profiles
        break;
      case 4:  // Mode
        DEBUG_PRINTLN("-> Navigate to PID Mode Select");
        // TODO: Implement mode select
        break;
      case 5:  // Sensor
        DEBUG_PRINTLN("-> Navigate to Sensor Mode Select");
        // TODO: Implement sensor mode select
        break;
      case 6:  // DspAut
        DEBUG_PRINTLN("-> Toggle Display Auto-off");
        // TODO: Implement display auto-off toggle
        break;
      case 7:  // VwData
        DEBUG_PRINTLN("-> Navigate to View Data");
        // TODO: Implement view data
        break;
      case 8:  // FacRst
        DEBUG_PRINTLN("-> Navigate to Factory Reset");
        // TODO: Implement factory reset
        break;
      case 9:  // Back
        DEBUG_PRINTLN("-> Returning to Main Menu");
        currentState = MAIN_MENU;
        mainMenu.requestRedraw();
        u8x8.clear();
        // Clear any pending button state after menu transition
        delay(100);  // Short delay to ensure button is released
        mainMenu.update();
        break;
      default:
        DEBUG_PRINTLN("-> Invalid selection");
        break;
    }
  }
}

void handlePIDEdit() {
  // Initialize on first entry
  if (!pidEditInitialized) {
    if (pidEdit == nullptr) {
      pidEdit = new PIDEdit(&u8x8, POT_PIN, BUTTON_PIN, POT_ENABLE_PIN);
    }
    pidEdit->begin();
    pidEditInitialized = true;
    DEBUG_PRINTLN("PID Edit initialized");
  }
  
  // Update and check for exit
  uint8_t result = pidEdit->update();
  if (result == 1) {
    // User exited (SAVE or BACK)
    DEBUG_PRINTLN("PID Edit exited");
    currentState = SETTINGS_MENU;
    settingsMenu.requestRedraw();
    delay(100);  // Short delay to ensure button is released
    settingsMenu.update();  // Clear any pending button state
  }
}

void performCalibration() {
  DEBUG_PRINTLN("Starting sensor calibration...");
  
  // Clear display and show calibration message
  u8x8.clear();
  u8x8.setCursor(0, 0);
  u8x8.print("CALIBRATING");
  u8x8.setCursor(0, 2);
  u8x8.print("Sweeping...");
  
  // Initialize calibration
  lineSensor.beginCalibration();
  
  // Calibration sweep - oscillate left and right
  const int sweepSpeed = 150;  // Medium speed for calibration
  const int sweepTime = 200;   // Time per sweep direction
  const int totalSweeps = 10;  // Number of complete sweeps
  
  Timer sweepTimer;
  
  for (int i = 0; i < totalSweeps; i++) {
    // Sweep right
    leftMotor->setState(MotorState::FORWARD);
    leftMotor->setSpeed(sweepSpeed);
    rightMotor->setState(MotorState::BACKWARD);
    rightMotor->setSpeed(sweepSpeed);
    
    sweepTimer.start();
    deltaTimer.start();
    while (sweepTimer.elapsed() < sweepTime) {
      unsigned long deltaTime = deltaTimer.elapsed();
      deltaTimer.start();
      
      leftMotor->update(deltaTime);
      rightMotor->update(deltaTime);
      lineSensor.updateCalibration();
      delay(10);
    }
    
    // Sweep left
    leftMotor->setState(MotorState::BACKWARD);
    leftMotor->setSpeed(sweepSpeed);
    rightMotor->setState(MotorState::FORWARD);
    rightMotor->setSpeed(sweepSpeed);
    
    sweepTimer.start();
    deltaTimer.start();
    while (sweepTimer.elapsed() < sweepTime) {
      unsigned long deltaTime = deltaTimer.elapsed();
      deltaTimer.start();
      
      leftMotor->update(deltaTime);
      rightMotor->update(deltaTime);
      lineSensor.updateCalibration();
      delay(10);
    }
    
    // Update progress on display
    u8x8.setCursor(0, 4);
    u8x8.print("Progress: ");
    u8x8.print((i + 1) * 10);
    u8x8.print("%  ");
  }
  
  // Stop motors
  leftMotor->brake();
  rightMotor->brake();
  
  // Finalize calibration
  lineSensor.endCalibration();
  
  // Show completion message
  u8x8.setCursor(0, 6);
  u8x8.print("COMPLETE!");
  delay(1500);
  
  // Return to main menu
  u8x8.clear();
  mainMenu.requestRedraw();
  
  DEBUG_PRINTLN("Calibration complete");
}
