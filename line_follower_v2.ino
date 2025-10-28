/**
 * Line Follower Robot - Main Menu Implementation
 * Features:
 * - Main menu with potentiometer scrolling
 * - Button selection
 * - Clean U8x8 text-based display
 */

#include <U8x8lib.h>
#include "menu_system.h"

// Display Pins (Software I2C for SSD1306 OLED)
#define SOFT_SDA 2
#define SOFT_SCL 3

// Input Pins
#define POT_PIN A7
#define BUTTON_PIN 5
#define POT_ENABLE_PIN 9

// Create display object - U8x8 is text-only mode (16x8 characters)
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(SOFT_SCL, SOFT_SDA, U8X8_PIN_NONE);

// Menu items
const char* menuItems[] = {
  "CFG",      // Settings/Configuration
  "CALIBRT",  // Calibration
  "RUN"       // Start running
};
const uint8_t MENU_ITEM_COUNT = 3;

// Create menu system
MenuSystem menu(&u8x8, menuItems, MENU_ITEM_COUNT, POT_PIN, BUTTON_PIN, POT_ENABLE_PIN);

void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.println("Line Follower - Main Menu");
  
  // Initialize display
  u8x8.begin();
  u8x8.setPowerSave(0);
  
  // Initialize menu system
  menu.begin();
  
  Serial.println("System ready");
}

void loop() {
  // Update menu system (handles input and display)
  menu.update();
  
  // Check if selection changed (optional - for debugging/logging)
  if (menu.hasNewSelection()) {
    Serial.print("Potentiometer reading: ");
    Serial.print(menu.getPotValue());
    Serial.print(" -> Selection: ");
    Serial.println(menu.getSelection());
    Serial.print("Selected: ");
    Serial.println(menu.getSelectedItem());
  }
  
  // Handle button press
  if (menu.isButtonJustPressed()) {
    Serial.print("Button pressed! Selected: ");
    Serial.println(menu.getSelectedItem());
    
    // TODO: Navigate to selected menu item
    // For now, just request a redraw
    menu.requestRedraw();
  }
  
  delay(50);  // Small delay for stability
}
