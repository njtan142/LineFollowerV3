/**
 * Settings Menu - Configuration menu wrapper
 * Uses MenuSystem class for all UI logic
 */

#ifndef SETTINGS_MENU_H
#define SETTINGS_MENU_H

#include <Arduino.h>
#include "menu_system.h"

// Settings menu items stored in PROGMEM
const char settingsItem0[] PROGMEM = "PID";
const char settingsItem1[] PROGMEM = "TEST";
const char settingsItem2[] PROGMEM = "THRESH";
const char settingsItem3[] PROGMEM = "PROFIL";
const char settingsItem4[] PROGMEM = "MODE";
const char settingsItem5[] PROGMEM = "SENSOR";
const char settingsItem6[] PROGMEM = "DSPAUT";
const char settingsItem7[] PROGMEM = "VWDATA";
const char settingsItem8[] PROGMEM = "FACRST";
const char settingsItem9[] PROGMEM = "BACK";

const char* const settingsMenuItems[] PROGMEM = {
  settingsItem0,
  settingsItem1,
  settingsItem2,
  settingsItem3,
  settingsItem4,
  settingsItem5,
  settingsItem6,
  settingsItem7,
  settingsItem8,
  settingsItem9
};

const uint8_t SETTINGS_ITEM_COUNT = 10;

class SettingsMenu {
private:
  MenuSystem* menuSystem;
  char itemBuffer[16];  // Buffer for loading strings from PROGMEM
  const char* displayItems[10];  // Pointers to loaded strings
  
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
    // Load menu items from PROGMEM into RAM
    for (uint8_t i = 0; i < SETTINGS_ITEM_COUNT; i++) {
      char* itemPtr = (char*)malloc(16);  // Allocate memory for each item
      strcpy_P(itemPtr, (char*)pgm_read_word(&(settingsMenuItems[i])));
      displayItems[i] = itemPtr;
    }
    
    // Create MenuSystem with 10 items, showing 3 at a time
    menuSystem = new MenuSystem(u8x8Display, displayItems, SETTINGS_ITEM_COUNT, 
                                 "CONFIG", 3, pot, button, potEnable);
  }
  
  /**
   * Destructor - free allocated memory
   */
  ~SettingsMenu() {
    for (uint8_t i = 0; i < SETTINGS_ITEM_COUNT; i++) {
      free((void*)displayItems[i]);
    }
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

