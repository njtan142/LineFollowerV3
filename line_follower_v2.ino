/**
 * Line Follower Robot - Main Application
 * 
 * A menu-driven line following robot with advanced features:
 * - Interactive UI with potentiometer navigation and button selection
 * - Sensor calibration routine with visual feedback
 * - Configurable PID control parameters
 * - Settings persistence via EEPROM
 * - Clean text-based OLED display (16x8 characters)
 * 
 * Hardware:
 * - Arduino-compatible microcontroller
 * - SSD1306 OLED display (128x64)
 * - 8-channel line sensor array (analog)
 * - TB6612FNG dual motor driver
 * - Potentiometer and button for UI input
 */

/**
 * Debug Configuration
 * 
 * Conditional compilation for serial debugging. Set DEBUG_SERIAL to 1
 * to enable debug output, or 0 to disable and save program memory.
 * When disabled, all DEBUG_* macros compile to nothing (zero overhead).
 */
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

// Include required libraries and modules
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

/**
 * Pin Definitions
 */
// Display Pins - Software I2C for SSD1306 OLED
#define SOFT_SDA 2
#define SOFT_SCL 3

// Input Pins - User interface controls
#define POT_PIN A7          // Potentiometer for menu scrolling
#define BUTTON_PIN 5        // Button for selection/confirmation
#define POT_ENABLE_PIN 9    // Enable pin to power potentiometer (reduces noise)

/**
 * Global Objects
 */
// Display - U8x8 provides text-only interface (efficient, 16x8 characters)
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(SOFT_SCL, SOFT_SDA, U8X8_PIN_NONE);

// Robot configuration (loaded from EEPROM, persists across power cycles)
Settings settings;

// Line sensor array (8 channels with calibration)
LineSensor lineSensor;

// Motor controllers (allocated dynamically to use settings values)
Motor* leftMotor = nullptr;
Motor* rightMotor = nullptr;

// Timer for calculating time between updates (deltaTime)
Timer deltaTimer;

/**
 * UI State Machine
 * 
 * Tracks which screen is currently active. Each state has its own
 * handler function that manages display updates and user input.
 */
enum UIState {
  MAIN_MENU,      // Top-level menu: CFG, CALIBRT, RUN
  SETTINGS_MENU,  // Configuration submenu
  PID_EDIT        // PID parameter editing screen
};
UIState currentState = MAIN_MENU;
bool settingsMenuInitialized = false;
bool pidEditInitialized = false;

/**
 * Main Menu Items
 */
const char* menuItems[] = {
  "CFG",      // Enter settings/configuration
  "CALIBRT",  // Calibrate line sensors
  "RUN"       // Start line following mode
};
const uint8_t MENU_ITEM_COUNT = 3;

// Menu system instances
MenuSystem mainMenu(&u8x8, menuItems, MENU_ITEM_COUNT, "MENU", 3, POT_PIN, BUTTON_PIN, POT_ENABLE_PIN);
SettingsMenu settingsMenu(&u8x8, POT_PIN, BUTTON_PIN, POT_ENABLE_PIN);
PIDEdit* pidEdit = nullptr;  // Created dynamically when needed

/**
 * Setup - Initialization routine (runs once at power-on)
 * 
 * Initializes all hardware peripherals, loads saved settings, and
 * prepares the system for operation. This includes:
 * - Serial debug output (if enabled)
 * - Loading configuration from EEPROM
 * - Initializing OLED display
 * - Creating motor controller objects
 * - Setting up the main menu UI
 */
void setup() {
  // Initialize debug serial if enabled
  DEBUG_BEGIN(9600);
  delay(100);
  DEBUG_PRINTLN("Line Follower - Main Menu");
  
  // Load robot settings from EEPROM (PID values, speeds, etc.)
  settings.load();
  
  // Initialize OLED display
  u8x8.begin();
  u8x8.setPowerSave(0);  // Turn on display
  
  // Create motor controller objects with loaded settings
  // Left motor has alignment offset to compensate for mechanical differences
  leftMotor = new Motor(
    HAL::MotorPins::Left::IN1, 
    HAL::MotorPins::Left::IN2, 
    HAL::MotorPins::Left::PWM, 
    HAL::MotorPins::STBY, 
    settings.alignmentOffset,  // Compensation for left motor
    &settings
  );
  
  // Right motor uses zero offset (left compensates to match right)
  rightMotor = new Motor(
    HAL::MotorPins::Right::IN1, 
    HAL::MotorPins::Right::IN2, 
    HAL::MotorPins::Right::PWM, 
    HAL::MotorPins::STBY, 
    0,  // No offset for right motor
    &settings
  );
  
  // Initialize main menu (other menus created lazily when accessed)
  mainMenu.begin();
  
  DEBUG_PRINTLN("System ready");
}

/**
 * Main Loop - Runs continuously after setup()
 * 
 * Dispatches to appropriate handler based on current UI state.
 * Each handler manages its own screen updates and user input.
 * Loop runs as fast as possible with 50ms delay for stability.
 */
void loop() {
  // Route to current screen handler
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
  
  // Small delay for system stability and debouncing
  delay(50);
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

/**
 * Settings Menu Handler
 * 
 * Manages the configuration submenu with various robot settings.
 * Provides access to PID tuning, sensor configuration, and other parameters.
 * 
 * Menu options:
 * - PID: Edit PID controller parameters
 * - TEST: Motor and sensor testing (TODO)
 * - THRESH: Adjust sensor threshold ratio (TODO)
 * - PROFIL: Load/save PID profiles (TODO)
 * - MODE: Select PID mode (TODO)
 * - SENSOR: Select sensor mode (TODO)
 * - DSPAUT: Display auto-off setting (TODO)
 * - VWDATA: View live sensor data (TODO)
 * - FACRST: Factory reset (TODO)
 * - BACK: Return to main menu
 */
void handleSettingsMenu() {
  // Update menu UI
  settingsMenu.update();
  
  // Log selection changes for debugging
  if (settingsMenu.hasNewSelection()) {
    DEBUG_PRINT("Settings Menu - Selection: ");
    DEBUG_PRINTLN(settingsMenu.getSelectedItem());
  }
  
  // Process button press
  if (settingsMenu.isButtonJustPressed()) {
    uint8_t selection = settingsMenu.getSelection();
    DEBUG_PRINT("Settings Button pressed! Selected: ");
    DEBUG_PRINTLN(settingsMenu.getSelectedItem());
    
    // Execute action based on selected menu item
    switch (selection) {
      // PID - Navigate to PID editing screen
      case 0:
        DEBUG_PRINTLN("-> Navigate to PID Edit screen");
        currentState = PID_EDIT;
        pidEditInitialized = false;  // Force re-initialization to load fresh values
        delay(100);  // Brief delay to ensure button is released
        break;
      
      // TEST - Motor and sensor testing interface
      case 1:
        DEBUG_PRINTLN("-> Navigate to Test menu");
        // TODO: Implement test menu
        break;
      
      // THRESH - Adjust sensor threshold ratio
      case 2:
        DEBUG_PRINTLN("-> Navigate to Threshold Select");
        // TODO: Implement threshold select
        break;
      
      // PROFIL - PID profile management
      case 3:
        DEBUG_PRINTLN("-> Navigate to PID Profiles");
        // TODO: Implement PID profiles
        break;
      
      // MODE - PID mode selection
      case 4:
        DEBUG_PRINTLN("-> Navigate to PID Mode Select");
        // TODO: Implement mode select
        break;
      
      // SENSOR - Sensor mode configuration
      case 5:
        DEBUG_PRINTLN("-> Navigate to Sensor Mode Select");
        // TODO: Implement sensor mode select
        break;
      
      // DSPAUT - Display auto-off toggle
      case 6:
        DEBUG_PRINTLN("-> Toggle Display Auto-off");
        // TODO: Implement display auto-off toggle
        break;
      
      // VWDATA - View live sensor data
      case 7:
        DEBUG_PRINTLN("-> Navigate to View Data");
        // TODO: Implement view data
        break;
      
      // FACRST - Factory reset confirmation
      case 8:
        DEBUG_PRINTLN("-> Navigate to Factory Reset");
        // TODO: Implement factory reset
        break;
      
      // BACK - Return to main menu
      case 9:
        DEBUG_PRINTLN("-> Returning to Main Menu");
        currentState = MAIN_MENU;
        mainMenu.requestRedraw();
        u8x8.clear();
        delay(100);  // Brief delay to ensure button is released
        mainMenu.update();  // Clear any pending button state
        break;
      
      default:
        DEBUG_PRINTLN("-> Invalid selection");
        break;
    }
  }
}

/**
 * PID Edit Screen Handler
 * 
 * Manages the PID parameter editing interface. Allows user to adjust
 * Kp, Ki, Kd, and Scale values using the potentiometer with fine control.
 * Changes can be saved to EEPROM or discarded.
 * 
 * The PID Edit screen is created lazily on first access and manages
 * its own state internally. Returns 1 when user exits (SAVE or BACK).
 */
void handlePIDEdit() {
  // Initialize PID edit screen on first entry
  if (!pidEditInitialized) {
    // Create PID edit object if doesn't exist
    if (pidEdit == nullptr) {
      pidEdit = new PIDEdit(&u8x8, POT_PIN, BUTTON_PIN, POT_ENABLE_PIN);
    }
    
    // Initialize the PID edit screen
    pidEdit->begin();
    pidEditInitialized = true;
    DEBUG_PRINTLN("PID Edit initialized");
  }
  
  // Update PID edit screen and check for exit
  uint8_t result = pidEdit->update();
  
  // User exited (either SAVE or BACK)
  if (result == 1) {
    DEBUG_PRINTLN("PID Edit exited");
    
    // Return to settings menu
    currentState = SETTINGS_MENU;
    settingsMenu.requestRedraw();
    delay(100);  // Brief delay to ensure button is released
    settingsMenu.update();  // Clear any pending button state
  }
}

/**
 * Sensor Calibration Routine
 * 
 * Performs automatic line sensor calibration by sweeping the robot
 * left and right over the line. This allows sensors to learn the
 * difference between the line (dark) and background (light).
 * 
 * Process:
 * 1. Initialize calibration (reset min/max values)
 * 2. Sweep robot left and right repeatedly (10 complete sweeps)
 * 3. Continuously read sensors during movement to capture min/max values
 * 4. Calculate and save threshold values to EEPROM
 * 
 * Visual feedback is provided on the display showing sweep progress.
 * Motors use smooth acceleration for gentler calibration movement.
 */
void performCalibration() {
  DEBUG_PRINTLN("Starting sensor calibration...");
  
  // Display calibration start message
  u8x8.clear();
  u8x8.setCursor(0, 0);
  u8x8.print("CALIBRATING");
  u8x8.setCursor(0, 2);
  u8x8.print("Sweeping...");
  
  // Initialize calibration (reset sensor min/max tracking)
  lineSensor.beginCalibration();
  
  // Calibration parameters
  const int sweepSpeed = 150;  // Medium speed for smooth, controlled sweeps
  const int sweepTime = 200;   // Time to sweep in each direction (milliseconds)
  const int totalSweeps = 10;  // Number of complete left-right-left sweeps
  
  Timer sweepTimer;
  
  // Perform calibration sweeps
  for (int i = 0; i < totalSweeps; i++) {
    // SWEEP RIGHT: Turn right by spinning motors in opposite directions
    leftMotor->setState(MotorState::FORWARD);
    leftMotor->setSpeed(sweepSpeed);
    rightMotor->setState(MotorState::BACKWARD);
    rightMotor->setSpeed(sweepSpeed);
    
    // Execute right sweep for specified time
    sweepTimer.start();
    deltaTimer.start();
    while (sweepTimer.elapsed() < sweepTime) {
      // Calculate time since last update for smooth motor ramping
      unsigned long deltaTime = deltaTimer.elapsed();
      deltaTimer.start();
      
      // Update motor speeds (handles acceleration ramping)
      leftMotor->update(deltaTime);
      rightMotor->update(deltaTime);
      
      // Read and record sensor values
      lineSensor.updateCalibration();
      
      delay(10);  // Small delay between readings
    }
    
    // SWEEP LEFT: Turn left by reversing motor directions
    leftMotor->setState(MotorState::BACKWARD);
    leftMotor->setSpeed(sweepSpeed);
    rightMotor->setState(MotorState::FORWARD);
    rightMotor->setSpeed(sweepSpeed);
    
    // Execute left sweep for specified time
    sweepTimer.start();
    deltaTimer.start();
    while (sweepTimer.elapsed() < sweepTime) {
      // Calculate time since last update
      unsigned long deltaTime = deltaTimer.elapsed();
      deltaTimer.start();
      
      // Update motors and sensors
      leftMotor->update(deltaTime);
      rightMotor->update(deltaTime);
      lineSensor.updateCalibration();
      
      delay(10);
    }
    
    // Update progress indicator on display (10%, 20%, 30%, etc.)
    u8x8.setCursor(0, 4);
    u8x8.print("Progress: ");
    u8x8.print((i + 1) * 10);
    u8x8.print("%  ");
  }
  
  // Stop motors immediately (use brake, not coast)
  leftMotor->brake();
  rightMotor->brake();
  
  // Finalize calibration (calculate thresholds and save to EEPROM)
  lineSensor.endCalibration();
  
  // Show completion message
  u8x8.setCursor(0, 6);
  u8x8.print("COMPLETE!");
  delay(1500);  // Display completion message for 1.5 seconds
  
  // Return to main menu
  u8x8.clear();
  mainMenu.requestRedraw();
  
  DEBUG_PRINTLN("Calibration complete");
}
