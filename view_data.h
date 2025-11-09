/**
 * View Data Screen - Display all configuration data
 * Shows system settings, PID values, sensor info, and memory status
 */

#ifndef VIEW_DATA_H
#define VIEW_DATA_H

#include <Arduino.h>
#include <U8x8lib.h>
#include "Settings.h"
#include "line_sensor.h"

class ViewData {
private:
  U8X8_SSD1306_128X64_NONAME_SW_I2C* display;
  uint8_t potPin;
  uint8_t buttonPin;
  uint8_t potEnablePin;
  
  uint8_t currentPage;
  uint8_t lastPage;
  const uint8_t totalPages = 5; // Number of data pages
  
  bool needsRedraw;
  
  // Button debouncing
  static unsigned long lastDebounceTime;
  static bool lastButtonReading;
  static bool buttonState;
  static bool lastButtonState;
  
  /**
   * Read potentiometer to select page
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
   * Draw a specific page of data
   */
  void drawPage(uint8_t page, Settings* settings, LineSensor* sensor) {
    display->setFont(u8x8_font_8x13_1x2_f);
    display->clear();
    
    switch (page) {
      case 0: // PID Values
        display->setCursor(0, 0);
        display->print("PID VALUES");
        display->setCursor(0, 2);
        display->print("KP:");
        display->print(settings->kp);
        display->setCursor(0, 4);
        display->print("KI:");
        display->print(settings->ki);
        display->setCursor(0, 6);
        display->print("KD:");
        display->print(settings->kd);
        break;
        
      case 1: // Motor Settings
        display->setCursor(0, 0);
        display->print("MOTOR");
        display->setCursor(0, 2);
        display->print("SPEED:");
        display->print(settings->baseSpeed);
        display->setCursor(0, 4);
        display->print("ALIGN:");
        display->print(settings->alignmentOffset);
        display->setCursor(0, 6);
        display->print("SCALE:");
        display->print(settings->pidScale);
        break;
        
      case 2: // Sensor Mode
        display->setCursor(0, 0);
        display->print("SENSOR");
        display->setCursor(0, 2);
        if (settings->sensorMode == SensorMode::NORMAL_8_CHANNEL) {
          display->print("MODE:8CH");
        } else {
          display->print("MODE:6CH MOD");
        }
        display->setCursor(0, 4);
        display->print("THRESH:");
        display->print(static_cast<int>(sensor->getThresholdRatio()));
        display->setCursor(0, 6);
        display->print("CALIB:OK");
        break;
        
      case 3: // PID Mode & Display
        display->setCursor(0, 0);
        display->print("CONTROL");
        display->setCursor(0, 2);
        if (settings->pidControlMode == PIDControlMode::NORMAL) {
          display->print("PID:NORMAL");
        } else {
          display->print("PID:OVERRIDE");
        }
        display->setCursor(0, 4);
        display->print("DISP:");
        display->print(settings->displayAutoOff ? "AUTOOFF" : "ALWAYSON");
        display->setCursor(0, 6);
        display->print("VER:");
        display->print(settings->version);
        break;
        
      case 4: // System Info
        display->setCursor(0, 0);
        display->print("SYSTEM");
        display->setCursor(0, 2);
        display->print("RAM:");
        display->print(freeRam());
        display->print("B");
        display->setCursor(0, 4);
        display->print("SETTINGS:OK");
        display->setCursor(0, 6);
        display->print("PG:");
        display->print(currentPage + 1);
        display->print("/");
        display->print(totalPages);
        break;
    }
  }
  
  /**
   * Calculate free RAM
   */
  int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
  }
  
public:
  ViewData(U8X8_SSD1306_128X64_NONAME_SW_I2C* u8x8Display,
           uint8_t pot = A7,
           uint8_t button = 5,
           uint8_t potEnable = 9) {
    display = u8x8Display;
    potPin = pot;
    buttonPin = button;
    potEnablePin = potEnable;
    currentPage = 0;
    lastPage = 255;
    needsRedraw = true;
  }
  
  /**
   * Initialize view data screen
   */
  void begin() {
    currentPage = 0;
    lastPage = 255;
    needsRedraw = true;
    lastButtonState = false;
  }
  
  /**
   * Update view data screen
   * Returns true if user wants to exit
   */
  bool update(Settings* settings, LineSensor* sensor) {
    // Read potentiometer to select page
    int potValue = readPot();
    currentPage = map(potValue, 0, 1023, 0, totalPages - 1);
    currentPage = constrain(currentPage, 0, totalPages - 1);
    
    // Redraw if page changed
    if (currentPage != lastPage || needsRedraw) {
      drawPage(currentPage, settings, sensor);
      lastPage = currentPage;
      needsRedraw = false;
    }
    
    // Check for exit button press
    return isButtonPressed();
  }
  
  /**
   * Request full redraw
   */
  void requestRedraw() {
    needsRedraw = true;
    lastPage = 255;
  }
};

// Initialize static members
unsigned long ViewData::lastDebounceTime = 0;
bool ViewData::lastButtonReading = false;
bool ViewData::buttonState = false;
bool ViewData::lastButtonState = false;

#endif // VIEW_DATA_H
