/**
 * Menu System - Handles UI scrolling, selection, and input
 * Provides a clean interface for menu navigation with potentiometer and button
 */

#ifndef MENU_SYSTEM_H
#define MENU_SYSTEM_H

#include <Arduino.h>
#include <U8x8lib.h>

class MenuSystem {
private:
  U8X8_SSD1306_128X64_NONAME_SW_I2C* display;
  
  // Input pins
  uint8_t potPin;
  uint8_t buttonPin;
  uint8_t potEnablePin;
  
  // Menu data
  const char** menuItems;
  uint8_t itemCount;
  
  // State variables
  uint8_t currentSelection;
  uint8_t lastSelection;
  int lastPotValue;
  bool lastButtonState;
  bool needsRedraw;
  
  // Debounce timing
  unsigned long lastDebounceTime;
  bool lastButtonReading;
  bool buttonState;
  
  /**
   * Read potentiometer with 80% deadzone applied
   * Returns: 0-1023 mapped value
   */
  int readPotClamped() {
    digitalWrite(potEnablePin, HIGH);
    delayMicroseconds(100);
    int raw = analogRead(potPin);
    
    // Use middle 80% of the range (102 to 921 maps to 0 to 1023)
    const int DEADZONE = (1024 * 10) / 100;  // 10% on each side
    const int MIN_VAL = DEADZONE;            // 102
    const int MAX_VAL = 1024 - DEADZONE;     // 922
    
    // Clamp and remap
    if (raw < MIN_VAL) raw = MIN_VAL;
    if (raw > MAX_VAL) raw = MAX_VAL;
    
    return map(raw, MIN_VAL, MAX_VAL, 0, 1023);
  }
  
  /**
   * Read button with debouncing (50ms)
   * Returns: Current debounced button state
   */
  bool readButtonDebounced() {
    bool reading = (digitalRead(buttonPin) == HIGH);
    
    if (reading != lastButtonReading) {
      lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > 50) {  // 50ms debounce
      if (reading != buttonState) {
        buttonState = reading;
      }
    }
    
    lastButtonReading = reading;
    return buttonState;
  }
  
  /**
   * Draw full menu (initial or forced redraw)
   */
  void drawFullMenu() {
    display->clear();
    display->setFont(u8x8_font_8x13_1x2_f);
    
    // Title
    display->setCursor(0, 0);
    display->print("MENU");
    
    // Menu items at rows 2, 4, 6
    for (uint8_t i = 0; i < itemCount; i++) {
      display->setCursor(2, 2 + (i * 2));  // Rows: 2, 4, 6
      
      // Show selection indicator
      if (i == currentSelection) {
        display->print(">");
      } else {
        display->print(" ");
      }
      display->print(menuItems[i]);
    }
    
    needsRedraw = false;
  }
  
  /**
   * Update only the selection indicators (optimized partial redraw)
   */
  void updateSelectionIndicators(uint8_t oldSelection, uint8_t newSelection) {
    display->setFont(u8x8_font_8x13_1x2_f);
    
    // Clear old selection indicator
    display->setCursor(2, 2 + (oldSelection * 2));
    display->print(" ");
    
    // Draw new selection indicator
    display->setCursor(2, 2 + (newSelection * 2));
    display->print(">");
  }

public:
  /**
   * Constructor
   * @param u8x8Display Pointer to U8x8 display object
   * @param items Array of menu item strings
   * @param count Number of menu items
   * @param pot Potentiometer analog pin
   * @param button Button digital pin
   * @param potEnable Potentiometer enable pin
   */
  MenuSystem(U8X8_SSD1306_128X64_NONAME_SW_I2C* u8x8Display,
             const char** items, 
             uint8_t count,
             uint8_t pot = A7,
             uint8_t button = 5,
             uint8_t potEnable = 9)
    : display(u8x8Display),
      menuItems(items),
      itemCount(count),
      potPin(pot),
      buttonPin(button),
      potEnablePin(potEnable),
      currentSelection(0),
      lastSelection(255),
      lastPotValue(-1),
      lastButtonState(false),
      needsRedraw(true),
      lastDebounceTime(0),
      lastButtonReading(false),
      buttonState(false) {
  }
  
  /**
   * Initialize the menu system
   * Sets up pins and draws initial menu
   */
  void begin() {
    pinMode(potPin, INPUT);
    pinMode(buttonPin, INPUT);
    pinMode(potEnablePin, OUTPUT);
    digitalWrite(potEnablePin, HIGH);
    
    // Draw initial menu
    drawFullMenu();
  }
  
  /**
   * Update menu state based on inputs
   * Call this in loop() to handle input and update display
   */
  void update() {
    // Check if full redraw is needed
    if (needsRedraw) {
      drawFullMenu();
      lastSelection = currentSelection;
      return;
    }
    
    // Map potentiometer directly to menu selection
    int potValue = readPotClamped();
    
    // Map pot range (0-1023) to menu items (0 to itemCount-1)
    // Invert so that high pot value = first item (index 0)
    uint8_t newSelection = map(potValue, 0, 1023, itemCount - 1, 0);
    newSelection = constrain(newSelection, 0, itemCount - 1);
    
    // Update display if selection changed
    if (newSelection != currentSelection) {
      uint8_t oldSelection = currentSelection;
      currentSelection = newSelection;
      updateSelectionIndicators(oldSelection, currentSelection);
      lastSelection = currentSelection;
    }
    
    // Update button state for next cycle
    bool buttonPressed = readButtonDebounced();
    lastButtonState = buttonPressed;
  }
  
  /**
   * Check if selection has changed since last check
   * Returns: true if selection changed
   */
  bool hasNewSelection() {
    return currentSelection != lastSelection;
  }
  
  /**
   * Get current selected index
   * Returns: Index of currently selected menu item
   */
  uint8_t getSelection() const {
    return currentSelection;
  }
  
  /**
   * Get current selected menu item text
   * Returns: Pointer to menu item string
   */
  const char* getSelectedItem() const {
    return menuItems[currentSelection];
  }
  
  /**
   * Check if button was just pressed (rising edge)
   * Returns: true if button was just pressed
   */
  bool isButtonJustPressed() {
    bool buttonPressed = readButtonDebounced();
    bool justPressed = buttonPressed && !lastButtonState;
    lastButtonState = buttonPressed;
    return justPressed;
  }
  
  /**
   * Force a full redraw on next update
   */
  void requestRedraw() {
    needsRedraw = true;
  }
  
  /**
   * Get current potentiometer reading (for debugging)
   * Returns: 0-1023 value
   */
  int getPotValue() {
    return readPotClamped();
  }
};

#endif // MENU_SYSTEM_H
