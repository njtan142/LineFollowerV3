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

// Potentiometer reading with deadzone (80% of range)
int readPotClamped() {
  digitalWrite(POT_ENABLE_PIN, HIGH);
  delayMicroseconds(100);
  int raw = analogRead(POT_PIN);
  
  // Use middle 80% of the range (102 to 921 maps to 0 to 1023)
  const int DEADZONE = (1024 * 10) / 100;  // 10% on each side
  const int MIN_VAL = DEADZONE;            // 102
  const int MAX_VAL = 1024 - DEADZONE;     // 922
  
  // Clamp and remap
  if (raw < MIN_VAL) raw = MIN_VAL;
  if (raw > MAX_VAL) raw = MAX_VAL;
  
  return map(raw, MIN_VAL, MAX_VAL, 0, 1023);
}

// Get steps from potentiometer movement
// divisor: larger values = more sensitive (e.g., 1000/itemCount for menu navigation)
int getSteps(int divisor) {
  int potValue = readPotClamped();
  
  if (lastPotValue == -1) {
    lastPotValue = potValue;
    return 0;
  }
  
  int delta = lastPotValue - potValue;
  int steps = 0;
  
  if (abs(delta) >= divisor) {
    steps = delta / divisor;
    lastPotValue = potValue;
  }
  
  return steps;
}

// Constrain selection within valid range
uint8_t constrainSelection(int selection, uint8_t maxItems) {
  if (selection < 0) return 0;
  if (selection >= maxItems) return maxItems - 1;
  return selection;
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

// Draw the main menu (initial full draw)
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

// Update only the selection indicators (optimized partial redraw)
void updateSelection(uint8_t oldSelection, uint8_t newSelection) {
  u8x8.setFont(u8x8_font_8x13_1x2_f);
  
  // Clear old selection indicator
  u8x8.setCursor(2, 2 + (oldSelection * 2));
  u8x8.print(" ");
  
  // Draw new selection indicator
  u8x8.setCursor(2, 2 + (newSelection * 2));
  u8x8.print(">");
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
  // Map potentiometer directly to menu selection
  int potValue = readPotClamped();
  
  // Map pot range (0-1023) to menu items (0 to MENU_ITEM_COUNT-1)
  // Invert so that high pot value = first item (index 0)
  uint8_t newSelection = map(potValue, 0, 1023, MENU_ITEM_COUNT - 1, 0);
  newSelection = constrain(newSelection, 0, MENU_ITEM_COUNT - 1);
  
  Serial.print("Potentiometer reading: ");
  Serial.print(potValue);
  Serial.print(" -> Selection: ");
  Serial.println(newSelection);
  
  // Update display if selection changed
  if (newSelection != currentSelection) {
    uint8_t oldSelection = currentSelection;
    currentSelection = newSelection;
    updateSelection(oldSelection, currentSelection);  // Partial redraw
    lastSelection = currentSelection;
    Serial.print("Selected: ");
    Serial.println(menuItems[currentSelection]);
  }
  
  // Read button state
  bool buttonPressed = readButtonDebounced();
  bool buttonJustPressed = buttonPressed && !lastButtonState;
  
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
