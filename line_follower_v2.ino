/**
 * Potentiometer and Button Test
 * Tests the potentiometer reading on pin A7 and button on pin 5
 * Displays values on OLED and Serial Monitor
 */

#include <U8x8lib.h>

// Display Pins (Software I2C for SSD1306 OLED)
#define SOFT_SDA 2
#define SOFT_SCL 3

// Input Pins
#define POT_PIN A7
#define BUTTON_PIN 5
#define POT_ENABLE_PIN 9  // Enable pin for potentiometer multiplexing

// Create display object - U8x8 is text-only mode (16x8 characters)
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(SOFT_SCL, SOFT_SDA, U8X8_PIN_NONE);

void setup() {
  Serial.begin(9600);  // Changed to 9600 for stability
  delay(1000);  // Wait for serial to stabilize
  Serial.println("Potentiometer & Button Test");
  
  // Initialize input pins
  pinMode(POT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);  // Button reads HIGH when pressed
  pinMode(POT_ENABLE_PIN, OUTPUT);
  digitalWrite(POT_ENABLE_PIN, HIGH);  // Enable potentiometer
  
  // Initialize display
  u8x8.begin();
  u8x8.setFont(u8x8_font_8x13_1x2_f);  // Larger, clearer font
  
  Serial.println("Display initialized");
  Serial.println("Reading potentiometer on A7 and button on pin 5...");
}

void loop() {
  // Ensure potentiometer is enabled
  digitalWrite(POT_ENABLE_PIN, HIGH);
  delayMicroseconds(100);  // Give it time to settle
  
  // Read potentiometer value (0-1023)
  int potValue = analogRead(POT_PIN);
  
  // Read button state (HIGH when pressed)
  bool buttonPressed = (digitalRead(BUTTON_PIN) == HIGH);
  
  // Print to Serial Monitor
  Serial.print("POT: ");
  Serial.print(potValue);
  Serial.print(" | Button: ");
  Serial.println(buttonPressed ? "PRESSED" : "Released");
  
  // Display on OLED
  u8x8.setCursor(0, 0);
  u8x8.print("INPUT TEST");
  
  u8x8.setCursor(0, 2);
  u8x8.print("POT: ");
  u8x8.print(potValue);
  u8x8.print("    ");  // Extra spaces to clear old digits
  
  u8x8.setCursor(0, 4);
  u8x8.print("BTN: ");
  u8x8.print(buttonPressed ? "PRESSED " : "Released");
  
  u8x8.setCursor(0, 6);
  u8x8.print("T:");
  u8x8.print(millis() / 1000.0, 1);
  u8x8.print("s   ");
  
  delay(100);
}
