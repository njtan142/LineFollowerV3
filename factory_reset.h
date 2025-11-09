/**
 * Factory Reset Screen - Reset settings to defaults
 * Provides confirmation before resetting PID, motor, and sensor calibration
 */

#ifndef FACTORY_RESET_H
#define FACTORY_RESET_H

#include <Arduino.h>
#include <U8x8lib.h>
#include "Settings.h"
#include "line_sensor.h"

class FactoryReset {
private:
  U8X8_SSD1306_128X64_NONAME_SW_I2C* display;
  uint8_t potPin;
  uint8_t buttonPin;
  uint8_t potEnablePin;
  
  enum ResetState {
    CONFIRM,    // Ask for confirmation
    RESETTING,  // Performing reset
    DONE        // Reset complete
  };
  
  ResetState state;
  bool confirmed;
  bool needsRedraw;
  
  // Button debouncing
  static unsigned long lastDebounceTime;
  static bool lastButtonReading;
  static bool buttonState;
  static bool lastButtonState;
  
  /**
   * Read potentiometer
   */
  int readPot() {
    digitalWrite(potEnablePin, HIGH);
    delayMicroseconds(100);
    int value = analogRead(potPin);
    return value;
  }
  
  /**
   * Read button with debouncing
   */
  bool readButtonDebounced() {
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
    return buttonState;
  }
  
  /**
   * Check for button press (rising edge)
   */
  bool isButtonPressed() {
    readButtonDebounced();
    bool pressed = buttonState && !lastButtonState;
    lastButtonState = buttonState;
    return pressed;
  }
  
  /**
   * Draw confirmation screen
   */
  void drawConfirm() {
    display->setFont(u8x8_font_8x13_1x2_f);
    display->clear();
    display->setCursor(0, 0);
    display->print("FACTORY RESET");
    display->setCursor(0, 2);
    display->print("RESET ALL?");
    display->setCursor(0, 4);
    if (confirmed) {
      display->print(">YES");
    } else {
      display->print(" YES");
    }
    display->setCursor(0, 6);
    if (!confirmed) {
      display->print(">NO");
    } else {
      display->print(" NO");
    }
  }
  
  /**
   * Draw resetting screen
   */
  void drawResetting() {
    display->clear();
    display->setCursor(0, 0);
    display->print("RESETTING...");
    display->setCursor(0, 2);
    display->print("PLEASE WAIT");
  }
  
  /**
   * Draw done screen
   */
  void drawDone() {
    display->clear();
    display->setCursor(0, 0);
    display->print("RESET");
    display->setCursor(0, 2);
    display->print("COMPLETE!");
    display->setCursor(0, 6);
    display->print("BTN:CONTINUE");
  }
  
public:
  FactoryReset(U8X8_SSD1306_128X64_NONAME_SW_I2C* u8x8Display,
               uint8_t pot = A7,
               uint8_t button = 5,
               uint8_t potEnable = 9) {
    display = u8x8Display;
    potPin = pot;
    buttonPin = button;
    potEnablePin = potEnable;
    state = CONFIRM;
    confirmed = false;
    needsRedraw = true;
  }
  
  /**
   * Initialize factory reset screen
   */
  void begin() {
    state = CONFIRM;
    confirmed = false;
    needsRedraw = true;
    lastButtonState = false;
  }
  
  /**
   * Update factory reset screen
   * Returns 0 = continue, 1 = cancelled, 2 = reset complete
   */
  uint8_t update(Settings* settings, LineSensor* sensor) {
    switch (state) {
      case CONFIRM:
        // Read potentiometer to toggle YES/NO
        int potValue = readPot();
        bool newConfirmed = (potValue > 512);
        
        // Redraw if selection changed
        if (newConfirmed != confirmed || needsRedraw) {
          confirmed = newConfirmed;
          drawConfirm();
          needsRedraw = false;
        }
        
        // Check for button press
        if (isButtonPressed()) {
          if (confirmed) {
            // User confirmed - proceed to reset
            state = RESETTING;
            needsRedraw = true;
            return 0; // Continue to next state
          } else {
            // User cancelled
            return 1; // Exit without reset
          }
        }
        break;
        
      case RESETTING:
        if (needsRedraw) {
          drawResetting();
          needsRedraw = false;
        }
        
        // Perform reset operations
        delay(500); // Brief pause for user feedback
        
        // Reset settings to defaults
        Settings defaultSettings;
        *settings = defaultSettings;
        settings->save();
        
        // Reset sensor calibration
        sensor->resetCalibration();
        sensor->saveCalibration();
        
        // Move to done state
        state = DONE;
        needsRedraw = true;
        delay(500);
        break;
        
      case DONE:
        if (needsRedraw) {
          drawDone();
          needsRedraw = false;
        }
        
        // Wait for button press to exit
        if (isButtonPressed()) {
          return 2; // Reset complete, exit
        }
        break;
    }
    
    return 0; // Continue showing screen
  }
  
  /**
   * Request full redraw
   */
  void requestRedraw() {
    needsRedraw = true;
  }
};

// Initialize static members
unsigned long FactoryReset::lastDebounceTime = 0;
bool FactoryReset::lastButtonReading = false;
bool FactoryReset::buttonState = false;
bool FactoryReset::lastButtonState = false;

#endif // FACTORY_RESET_H
