/**
 * Test Menu - Hardware testing interface
 * Uses MenuSystem class for motor test menu UI logic
 * Custom UI for line sensor test (non-scrolling display)
 */

#ifndef TEST_MENU_H
#define TEST_MENU_H

#include <Arduino.h>
#include "menu_system.h"
#include "motor.h"
#include "line_sensor.h"

// Test menu items
const char* const testMenuItems[] = {
  "MOTOR",
  "SENSOR",
  "BACK"
};
const uint8_t TEST_ITEM_COUNT = 3;

class TestMenu {
private:
  MenuSystem* testMenuSystem;
  U8X8_SSD1306_128X64_NONAME_SW_I2C* display;
  uint8_t potPin;
  uint8_t buttonPin;
  uint8_t potEnablePin;
  
public:
  /**
   * Constructor
   * @param u8x8Display Pointer to U8x8 display object
   * @param pot Potentiometer analog pin
   * @param button Button digital pin
   * @param potEnable Potentiometer enable pin
   */
  TestMenu(U8X8_SSD1306_128X64_NONAME_SW_I2C* u8x8Display,
           uint8_t pot = A7,
           uint8_t button = 5,
           uint8_t potEnable = 9) {
    display = u8x8Display;
    potPin = pot;
    buttonPin = button;
    potEnablePin = potEnable;
    
    // Create main test menu with 3 items
    testMenuSystem = new MenuSystem(u8x8Display, testMenuItems, TEST_ITEM_COUNT, 
                                     "TEST", 3, pot, button, potEnable);
  }
  
  /**
   * Destructor - free allocated memory
   */
  ~TestMenu() {
    delete testMenuSystem;
  }
  
  /**
   * Initialize the test menu
   */
  void begin() {
    testMenuSystem->begin();
  }
  
  /**
   * Update test menu state based on inputs
   */
  void update() {
    testMenuSystem->update();
  }
  
  /**
   * Get current selected index
   */
  uint8_t getSelection() const {
    return testMenuSystem->getSelection();
  }
  
  /**
   * Get current selected menu item text
   */
  const char* getSelectedItem() const {
    return testMenuSystem->getSelectedItem();
  }
  
  /**
   * Check if button was just pressed
   */
  bool isButtonJustPressed() {
    return testMenuSystem->isButtonJustPressed();
  }
  
  /**
   * Force a full redraw on next update
   */
  void requestRedraw() {
    testMenuSystem->requestRedraw();
  }
  
  /**
   * Check if selection has changed
   */
  bool hasNewSelection() {
    return testMenuSystem->hasNewSelection();
  }
  
  /**
   * Line Sensor Test Display
   * Shows raw sensor readings with pot to change sensor number
   * Non-scrolling display
   * 
   * @param lineSensor Pointer to LineSensor object
   * @param selectedSensor Currently selected sensor (0-7)
   * @return true if user wants to exit, false to continue
   */
  bool displaySensorTest(LineSensor* lineSensor, uint8_t& selectedSensor) {
    // Read sensors
    lineSensor->readSensorsRaw();
    
    // Read potentiometer to select sensor (0-7)
    digitalWrite(potEnablePin, HIGH);
    delayMicroseconds(100);
    int potValue = analogRead(potPin);
    selectedSensor = map(potValue, 0, 1023, 0, 7);
    selectedSensor = constrain(selectedSensor, 0, 7);
    
    // Get raw value for selected sensor
    const int* sensorValues = lineSensor->getSensorValues();
    int rawValue = sensorValues[selectedSensor];
    
    // Display layout:
    // Row 0-1: Title "SENSOR TEST"
    // Row 2-3: Blank
    // Row 4-5: "S[N]: [VALUE]" where N is sensor number (0-7)
    // Row 6-7: "POT:CHG BTN:EXIT"
    
    display->setFont(u8x8_font_8x13_1x2_f);
    
    // Title (only needs update on first call or redraw)
    static bool needsFullRedraw = true;
    if (needsFullRedraw) {
      display->clear();
      display->setCursor(0, 0);
      display->print("SENSOR TEST");
      needsFullRedraw = false;
    }
    
    // Clear middle section and display sensor reading
    display->setCursor(0, 4);
    display->print("                "); // Clear line
    display->setCursor(0, 4);
    display->print("S");
    display->print(selectedSensor);
    display->print(": ");
    display->print(rawValue);
    
    // Bottom instruction line
    static bool instructionsDrawn = false;
    if (!instructionsDrawn) {
      display->setCursor(0, 6);
      display->print("POT:CHG BTN:EXIT");
      instructionsDrawn = true;
    }
    
    // Check for button press to exit
    static unsigned long lastDebounceTime = 0;
    static bool lastButtonReading = false;
    static bool buttonState = false;
    static bool lastButtonState = false;
    
    bool reading = (digitalRead(buttonPin) == HIGH);
    
    if (reading != lastButtonReading) {
      lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > 50) {
      if (reading != buttonState) {
        buttonState = reading;
      }
    }
    
    lastButtonReading = reading;
    
    // Detect rising edge (button press)
    bool buttonPressed = false;
    if (buttonState && !lastButtonState) {
      buttonPressed = true;
    }
    lastButtonState = buttonState;
    
    // Reset static flags on exit
    if (buttonPressed) {
      needsFullRedraw = true;
      instructionsDrawn = false;
    }
    
    return buttonPressed; // true = exit requested
  }
};

#endif // TEST_MENU_H
