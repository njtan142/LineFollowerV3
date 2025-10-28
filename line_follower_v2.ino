/**
 * Line Follower Robot - Main Menu Implementation
 * Features:
 * - Main menu with potentiometer scrolling
 * - Button selection
 * - Clean U8x8 text-based display
 */

#include <U8x8lib.h>

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

// State variables
uint8_t currentSelection = 0;
uint8_t lastSelection = 255;  // Force initial draw
int lastPotValue = -1;
bool lastButtonState = false;
bool needsRedraw = true;

// Potentiometer helper - converts pot value to menu steps
uint8_t getMenuSelection(int potValue, uint8_t itemCount) {
  // Map potentiometer value (0-1023) to menu items (0 to itemCount-1)
  // Add small hysteresis to prevent jitter
  int steps = 1000 / itemCount;
  return constrain(potValue / steps, 0, itemCount - 1);
}

// Button debouncing
bool readButtonDebounced() {
  static unsigned long lastDebounceTime = 0;
  static bool lastReading = false;
  static bool buttonState = false;
  
  bool reading = (digitalRead(BUTTON_PIN) == HIGH);
  
  if (reading != lastReading) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > 50) {  // 50ms debounce
    if (reading != buttonState) {
      buttonState = reading;
    }
  }
  
  lastReading = reading;
  return buttonState;
}

// Draw the main menu
void drawMainMenu() {
  u8x8.clear();
  u8x8.setFont(u8x8_font_8x13_1x2_f);
  
  // Title
  u8x8.setCursor(0, 0);
  u8x8.print("MENU");
  
  // Menu items at rows 2, 4, 6 (3 items total)
  for (uint8_t i = 0; i < MENU_ITEM_COUNT; i++) {
    u8x8.setCursor(2, 2 + (i * 2));  // Rows: 2, 4, 6
    
    // Show selection indicator
    if (i == currentSelection) {
      u8x8.print(">");
    } else {
      u8x8.print(" ");
    }
    u8x8.print(menuItems[i]);
  }
  
  needsRedraw = false;
}

void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.println("Line Follower - Main Menu");
  
  // Initialize input pins
  pinMode(POT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(POT_ENABLE_PIN, OUTPUT);
  digitalWrite(POT_ENABLE_PIN, HIGH);  // Enable potentiometer
  
  // Initialize display
  u8x8.begin();
  u8x8.setPowerSave(0);
  
  Serial.println("System ready");
  
  // Draw initial menu
  drawMainMenu();
}

void loop() {
  // Enable potentiometer and read value
  digitalWrite(POT_ENABLE_PIN, HIGH);
  delayMicroseconds(100);
  int potValue = analogRead(POT_PIN);
  
  // Get current selection from potentiometer
  currentSelection = getMenuSelection(potValue, MENU_ITEM_COUNT);
  
  // Read button state
  bool buttonPressed = readButtonDebounced();
  bool buttonJustPressed = buttonPressed && !lastButtonState;
  
  // Check if selection changed
  if (currentSelection != lastSelection) {
    needsRedraw = true;
    lastSelection = currentSelection;
    
    Serial.print("Selected: ");
    Serial.println(menuItems[currentSelection]);
  }
  
  // Redraw if needed
  if (needsRedraw) {
    drawMainMenu();
  }
  
  // Handle button press
  if (buttonJustPressed) {
    Serial.print("Button pressed! Selected: ");
    Serial.println(menuItems[currentSelection]);
    
    // TODO: Navigate to selected menu item
    // For now, just flash the selection briefly
    needsRedraw = true;
  }
  
  lastButtonState = buttonPressed;
  
  delay(50);  // Small delay for stability
}
