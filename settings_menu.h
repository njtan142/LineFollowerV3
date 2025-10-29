/**
 * Settings Menu - Configuration menu wrapper
 * Uses MenuSystem class for all UI logic
 */

#ifndef SETTINGS_MENU_H
#define SETTINGS_MENU_H

#include <Arduino.h>
#include "menu_system.h"

// Settings menu items
const char* const settingsMenuItems[] = {
  "PID",
  "TEST",
  "THRESH",
  "PROFIL",
  "MODE",
  "SENSOR",
  "DSPAUT",
  "VWDATA",
  "FACRST",
  "BACK"
};

const uint8_t SETTINGS_ITEM_COUNT = 10;

class SettingsMenu {
private:
  MenuSystem* menuSystem;
  
public:
  /**
   * Constructor
   * @param u8x8Display Pointer to U8x8 display object
   * @param pot Potentiometer analog pin
   * @param button Button digital pin
   * @param potEnable Potentiometer enable pin
   */
  SettingsMenu(U8X8_SSD1306_128X64_NONAME_SW_I2C* u8x8Display,
               uint8_t pot = A7,
               uint8_t button = 5,
               uint8_t potEnable = 9) {
    // Create MenuSystem with 10 items, showing 3 at a time
    menuSystem = new MenuSystem(u8x8Display, settingsMenuItems, SETTINGS_ITEM_COUNT, 
                                 "CONFIG", 3, pot, button, potEnable);
  }
  
  /**
   * Destructor - free allocated memory
   */
  ~SettingsMenu() {
    delete menuSystem;
  }
  
  /**
   * Initialize the settings menu
   */
  void begin() {
    menuSystem->begin();
  }
  
  /**
   * Update menu state based on inputs
   */
  void update() {
    menuSystem->update();
  }
  
  /**
   * Get current selected index (0-9)
   */
  uint8_t getSelection() const {
    return menuSystem->getSelection();
  }
  
  /**
   * Get current selected menu item text
   */
  const char* getSelectedItem() const {
    return menuSystem->getSelectedItem();
  }
  
  /**
   * Check if button was just pressed
   */
  bool isButtonJustPressed() {
    return menuSystem->isButtonJustPressed();
  }
  
  /**
   * Force a full redraw on next update
   */
  void requestRedraw() {
    menuSystem->requestRedraw();
  }
  
  /**
   * Get current potentiometer reading (for debugging)
   */
  int getPotValue() {
    return menuSystem->getPotValue();
  }
  
  /**
   * Check if selection has changed
   */
  bool hasNewSelection() {
    return menuSystem->hasNewSelection();
  }
};

#endif // SETTINGS_MENU_H

