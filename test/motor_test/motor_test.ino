/*
 * Motor Test Sketch with OLED Display
 * 
 * This sketch tests both left and right motors with TB6612FNG motor driver.
 * Tests forward, backward, brake, and speed control functions.
 * Output is shown on OLED display (128x32, 4 lines).
 * 
 * Hardware Connections (TB6612FNG):
 * - Pin 7: STBY (Standby - enables driver)
 * Left Motor:
 *   - Pin 8: IN1
 *   - Pin 12: IN2
 *   - Pin 6: PWM (speed control)
 * Right Motor:
 *   - Pin 11: IN1
 *   - Pin 4: IN2
 *   - Pin 10: PWM (speed control)
 * OLED Display (SSD1306 128x32):
 *   - Pin 2: SDA (Software I2C)
 *   - Pin 3: SCL (Software I2C)
 *   - 4 lines at rows: 0, 2, 4, 6
 * 
 * Test Sequence:
 * 1. Both motors forward
 * 2. Both motors backward
 * 3. Left motor only
 * 4. Right motor only
 * 5. Rotate clockwise
 * 6. Rotate counter-clockwise
 * 7. Speed ramp test
 * 8. Brake test
 */

/**
 * Configuration
 * Set to 1 to enable, or 0 to disable.
 */
#define DEBUG_SERIAL 0
#define USE_OLED 0

#if DEBUG_SERIAL
  #define DEBUG_BEGIN(baud) Serial.begin(baud)
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_BEGIN(baud)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#include <Arduino.h>
#if USE_OLED
  #include <U8x8lib.h>
#endif

// Display pins (Software I2C)
#define SOFT_SDA 2
#define SOFT_SCL 3

// Motor Driver Pin Definitions
#define STBY_PIN 7

// Left Motor Pins
#define LEFT_IN1 8
#define LEFT_IN2 12
#define LEFT_PWM 6

// Right Motor Pins
#define RIGHT_IN1 11  // Changed from 13 to avoid built-in LED
#define RIGHT_IN2 4
#define RIGHT_PWM 10

#if USE_OLED
  // Create display object (128x32 = 4 lines at rows 0, 2, 4, 6)
  U8X8_SSD1306_128X32_UNIVISION_SW_I2C u8x8(SOFT_SCL, SOFT_SDA, U8X8_PIN_NONE);
#endif

// Motor states
enum MotorState {
  FORWARD,
  BACKWARD,
  BRAKE,
  COAST
};

// String constants in PROGMEM to save RAM
// Common words
const char STR_TEST[] PROGMEM = "Test";
const char STR_SPEED[] PROGMEM = "Speed";
const char STR_MOTOR[] PROGMEM = "Motor";
const char STR_MOTORS[] PROGMEM = "Motors";
const char STR_STOPPED[] PROGMEM = "Stopped";
const char STR_RUNNING[] PROGMEM = "Running...";
const char STR_COMPLETE[] PROGMEM = "Complete!";
const char STR_ROTATING[] PROGMEM = "Rotating...";
const char STR_FORWARD[] PROGMEM = "Forward";
const char STR_BACKWARD[] PROGMEM = "Backward";

// Display strings
const char STR_DIVIDER[] PROGMEM = "=================================";
const char STR_HEADER[] PROGMEM = "  Motor Test - TB6612FNG Driver  ";
const char STR_MOTOR_TEST[] PROGMEM = "MOTOR TEST";
const char STR_TB6612FNG[] PROGMEM = "TB6612FNG";
const char STR_PINS_OK[] PROGMEM = "Pins OK";
const char STR_STARTING[] PROGMEM = "Starting...";
const char STR_BOTH_FWD[] PROGMEM = "Both Forward";
const char STR_BOTH_BWD[] PROGMEM = "Both Backward";
const char STR_LEFT_MOTOR[] PROGMEM = "Left Motor";
const char STR_RIGHT_MOTOR[] PROGMEM = "Right Motor";
const char STR_ROTATE_CW[] PROGMEM = "Rotate CW";
const char STR_ROTATE_CCW[] PROGMEM = "Rotate CCW";
const char STR_SPEED_RAMP[] PROGMEM = "Speed Ramp";
const char STR_BRAKE_TEST[] PROGMEM = "Brake Test";
const char STR_ALL_TESTS[] PROGMEM = "All Tests";
const char STR_RESTARTING[] PROGMEM = "Restarting...";
const char STR_RAMP_UP[] PROGMEM = "Ramping UP";
const char STR_RAMP_DOWN[] PROGMEM = "Ramping DOWN";
const char STR_FULL_SPEED[] PROGMEM = "Full Speed";
const char STR_BRAKING[] PROGMEM = "BRAKING!";

// Function prototypes
void setMotorState(bool isLeft, MotorState state, uint8_t speed);
void enableDriver();
void disableDriver();
void printToDisplay(uint8_t row, const char* text);
void printToDisplayP(uint8_t row, const char* textP);
void testBothForward();
void testBothBackward();
void testLeftOnly();
void testRightOnly();
void testRotateClockwise();
void testRotateCounterClockwise();
void testBrake();
void testSpeedRamp();

void setup() {
  // Initialize serial communication (if debug enabled)
  DEBUG_BEGIN(115200);
  
#if USE_OLED
  // Initialize display
  u8x8.begin();
  u8x8.setPowerSave(0);  // Turn on display
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clear();
#endif
  
  DEBUG_PRINTLN((__FlashStringHelper*)STR_DIVIDER);
  DEBUG_PRINTLN((__FlashStringHelper*)STR_HEADER);
  DEBUG_PRINTLN((__FlashStringHelper*)STR_DIVIDER);
  DEBUG_PRINTLN();
  
  printToDisplayP(0, STR_MOTOR_TEST);
  printToDisplayP(2, STR_TB6612FNG);
  
  // Configure all motor pins
  pinMode(STBY_PIN, OUTPUT);
  
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  
  // Initially disable driver
  disableDriver();
  
  DEBUG_PRINTLN("Motor pins configured");
  DEBUG_PRINTLN("Starting test sequence in 2 seconds...");
  DEBUG_PRINTLN();
  
  printToDisplayP(4, STR_PINS_OK);
  printToDisplayP(6, STR_STARTING);
  
  delay(2000);
}

void loop() {
  // Build test number strings dynamically
  char testNum[8];
  
  // Wait a bit before starting
  delay(500);
  
  DEBUG_PRINTLN("\n>>> Test 1: Both Motors Forward");
  strcpy_P(testNum, STR_TEST); strcat(testNum, " 1");
  printToDisplay(0, testNum);
  printToDisplayP(2, STR_BOTH_FWD);
  printToDisplay(4, "                "); // Clear line 4
  printToDisplay(6, "                "); // Clear line 6
  testBothForward();
  delay(1000);
  
  DEBUG_PRINTLN("\n>>> Test 2: Both Motors Backward");
  strcpy_P(testNum, STR_TEST); strcat(testNum, " 2");
  printToDisplay(0, testNum);
  printToDisplayP(2, STR_BOTH_BWD);
  printToDisplay(4, "                ");
  printToDisplay(6, "                ");
  testBothBackward();
  delay(1000);
  
  DEBUG_PRINTLN("\n>>> Test 3: Left Motor Only");
  strcpy_P(testNum, STR_TEST); strcat(testNum, " 3");
  printToDisplay(0, testNum);
  printToDisplayP(2, STR_LEFT_MOTOR);
  printToDisplay(4, "                ");
  printToDisplay(6, "                ");
  testLeftOnly();
  delay(1000);
  
  DEBUG_PRINTLN("\n>>> Test 4: Right Motor Only");
  strcpy_P(testNum, STR_TEST); strcat(testNum, " 4");
  printToDisplay(0, testNum);
  printToDisplayP(2, STR_RIGHT_MOTOR);
  printToDisplay(4, "                ");
  printToDisplay(6, "                ");
  testRightOnly();
  delay(1000);
  
  DEBUG_PRINTLN("\n>>> Test 5: Rotate Clockwise (Right Forward, Left Backward)");
  strcpy_P(testNum, STR_TEST); strcat(testNum, " 5");
  printToDisplay(0, testNum);
  printToDisplayP(2, STR_ROTATE_CW);
  printToDisplay(4, "                ");
  printToDisplay(6, "                ");
  testRotateClockwise();
  delay(1000);
  
  DEBUG_PRINTLN("\n>>> Test 6: Rotate Counter-Clockwise (Left Forward, Right Backward)");
  strcpy_P(testNum, STR_TEST); strcat(testNum, " 6");
  printToDisplay(0, testNum);
  printToDisplayP(2, STR_ROTATE_CCW);
  printToDisplay(4, "                ");
  printToDisplay(6, "                ");
  testRotateCounterClockwise();
  delay(1000);
  
  DEBUG_PRINTLN("\n>>> Test 7: Speed Ramp Test");
  strcpy_P(testNum, STR_TEST); strcat(testNum, " 7");
  printToDisplay(0, testNum);
  printToDisplayP(2, STR_SPEED_RAMP);
  printToDisplay(4, "                ");
  printToDisplay(6, "                ");
  testSpeedRamp();
  delay(1000);
  
  DEBUG_PRINTLN("\n>>> Test 8: Brake Test");
  strcpy_P(testNum, STR_TEST); strcat(testNum, " 8");
  printToDisplay(0, testNum);
  printToDisplayP(2, STR_BRAKE_TEST);
  printToDisplay(4, "                ");
  printToDisplay(6, "                ");
  testBrake();
  delay(1000);
  
  DEBUG_PRINTLN();
  DEBUG_PRINTLN((__FlashStringHelper*)STR_DIVIDER);
  DEBUG_PRINTLN("  Test sequence complete!");
  DEBUG_PRINTLN("  Restarting in 3 seconds...");
  DEBUG_PRINTLN((__FlashStringHelper*)STR_DIVIDER);
  
  printToDisplayP(0, STR_ALL_TESTS);
  printToDisplayP(2, STR_COMPLETE);
  printToDisplay(4, "                ");
  printToDisplayP(6, STR_RESTARTING);
  delay(3000);
}

// Motor control functions
void setMotorState(bool isLeft, MotorState state, uint8_t speed) {
  uint8_t in1Pin = isLeft ? LEFT_IN1 : RIGHT_IN1;
  uint8_t in2Pin = isLeft ? LEFT_IN2 : RIGHT_IN2;
  uint8_t pwmPin = isLeft ? LEFT_PWM : RIGHT_PWM;
  
  switch (state) {
    case FORWARD:
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
      analogWrite(pwmPin, speed);
      break;
      
    case BACKWARD:
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
      analogWrite(pwmPin, speed);
      break;
      
    case BRAKE:
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, HIGH);
      analogWrite(pwmPin, 255);
      break;
      
    case COAST:
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, LOW);
      analogWrite(pwmPin, 0);
      break;
  }
}

void enableDriver() {
  digitalWrite(STBY_PIN, HIGH);
}

void disableDriver() {
  digitalWrite(STBY_PIN, LOW);
}

// Display helper functions
void printToDisplay(uint8_t row, const char* text) {
#if USE_OLED
  noInterrupts(); // Disable interrupts during I2C
  u8x8.setCursor(0, row);
  u8x8.print(text);
  // Clear rest of line (16 chars max)
  uint8_t len = strlen(text);
  for (uint8_t i = len; i < 16; i++) {
    u8x8.print(" ");
  }
  interrupts(); // Re-enable interrupts
  delay(10);
#endif
}

void printToDisplayP(uint8_t row, const char* textP) {
#if USE_OLED
  char buf[17];
  strncpy_P(buf, textP, 16);
  buf[16] = '\0';
  noInterrupts(); // Disable interrupts during I2C
  u8x8.setCursor(0, row);
  u8x8.print(buf);
  // Clear rest of line
  uint8_t len = strlen(buf);
  for (uint8_t i = len; i < 16; i++) {
    u8x8.print(" ");
  }
  interrupts(); // Re-enable interrupts
  delay(10);
#endif
}

// Test sequences
void testBothForward() {
  char buf[17];
  enableDriver();
  DEBUG_PRINTLN("Running both motors forward at speed 200 for 2 seconds");
  strcpy_P(buf, STR_SPEED); strcat(buf, ": 200");
  printToDisplay(4, buf);
  printToDisplayP(6, STR_RUNNING);
  setMotorState(true, FORWARD, 200);   // Left motor
  setMotorState(false, FORWARD, 200);  // Right motor
  delay(2000);
  
  setMotorState(true, COAST, 0);
  setMotorState(false, COAST, 0);
  disableDriver();
  DEBUG_PRINT((__FlashStringHelper*)STR_MOTORS); DEBUG_PRINTLN(" stopped");
  printToDisplayP(6, STR_STOPPED);
}

void testBothBackward() {
  char buf[17];
  enableDriver();
  DEBUG_PRINTLN("Running both motors backward at speed 200 for 2 seconds");
  strcpy_P(buf, STR_SPEED); strcat(buf, ": 200");
  printToDisplay(4, buf);
  printToDisplayP(6, STR_RUNNING);
  setMotorState(true, BACKWARD, 200);   // Left motor
  setMotorState(false, BACKWARD, 200);  // Right motor
  delay(2000);
  
  setMotorState(true, COAST, 0);
  setMotorState(false, COAST, 0);
  disableDriver();
  DEBUG_PRINT((__FlashStringHelper*)STR_MOTORS); DEBUG_PRINTLN(" stopped");
  printToDisplayP(6, STR_STOPPED);
}

void testLeftOnly() {
  char buf[17];
  enableDriver();
  DEBUG_PRINTLN("Running LEFT motor only - Forward at speed 180 for 1.5 seconds");
  strcpy(buf, "FWD "); strcat_P(buf, STR_SPEED); strcat(buf, ": 180");
  printToDisplay(4, buf);
  printToDisplayP(6, STR_RUNNING);
  setMotorState(true, FORWARD, 180);
  delay(1500);
  
  DEBUG_PRINTLN("Running LEFT motor only - Backward at speed 180 for 1.5 seconds");
  strcpy(buf, "BWD "); strcat_P(buf, STR_SPEED); strcat(buf, ": 180");
  printToDisplay(4, buf);
  setMotorState(true, BACKWARD, 180);
  delay(1500);
  
  setMotorState(true, COAST, 0);
  disableDriver();
  DEBUG_PRINT("Left "); DEBUG_PRINT((__FlashStringHelper*)STR_MOTOR); DEBUG_PRINTLN(" stopped");
  printToDisplayP(6, STR_STOPPED);
}

void testRightOnly() {
  char buf[17];
  enableDriver();
  DEBUG_PRINTLN("Running RIGHT motor only - Forward at speed 180 for 1.5 seconds");
  strcpy(buf, "FWD "); strcat_P(buf, STR_SPEED); strcat(buf, ": 180");
  printToDisplay(4, buf);
  printToDisplayP(6, STR_RUNNING);
  setMotorState(false, FORWARD, 180);
  delay(1500);
  
  DEBUG_PRINTLN("Running RIGHT motor only - Backward at speed 180 for 1.5 seconds");
  strcpy(buf, "BWD "); strcat_P(buf, STR_SPEED); strcat(buf, ": 180");
  printToDisplay(4, buf);
  setMotorState(false, BACKWARD, 180);
  delay(1500);
  
  setMotorState(false, COAST, 0);
  disableDriver();
  DEBUG_PRINT("Right "); DEBUG_PRINT((__FlashStringHelper*)STR_MOTOR); DEBUG_PRINTLN(" stopped");
  printToDisplayP(6, STR_STOPPED);
}

void testRotateClockwise() {
  char buf[17];
  enableDriver();
  DEBUG_PRINTLN("Rotating clockwise: Left=Backward, Right=Forward at speed 150 for 2 seconds");
  strcpy_P(buf, STR_SPEED); strcat(buf, ": 150");
  printToDisplay(4, buf);
  printToDisplayP(6, STR_ROTATING);
  setMotorState(true, BACKWARD, 150);   // Left backward
  setMotorState(false, FORWARD, 150);   // Right forward
  delay(2000);
  
  setMotorState(true, COAST, 0);
  setMotorState(false, COAST, 0);
  disableDriver();
  DEBUG_PRINTLN("Rotation stopped");
  printToDisplayP(6, STR_STOPPED);
}

void testRotateCounterClockwise() {
  char buf[17];
  enableDriver();
  DEBUG_PRINTLN("Rotating counter-clockwise: Left=Forward, Right=Backward at speed 150 for 2 seconds");
  strcpy_P(buf, STR_SPEED); strcat(buf, ": 150");
  printToDisplay(4, buf);
  printToDisplayP(6, STR_ROTATING);
  setMotorState(true, FORWARD, 150);    // Left forward
  setMotorState(false, BACKWARD, 150);  // Right backward
  delay(2000);
  
  setMotorState(true, COAST, 0);
  setMotorState(false, COAST, 0);
  disableDriver();
  DEBUG_PRINTLN("Rotation stopped");
  printToDisplayP(6, STR_STOPPED);
}

void testSpeedRamp() {
  char buf[17];
  enableDriver();
  DEBUG_PRINT((__FlashStringHelper*)STR_SPEED); DEBUG_PRINTLN(" ramp test: 0 -> 255 -> 0 (both motors forward)");
  printToDisplayP(4, STR_RAMP_UP);
  
  // Ramp up
  DEBUG_PRINTLN("Ramping UP...");
  for (int speed = 0; speed <= 255; speed += 5) {
    setMotorState(true, FORWARD, (uint8_t)speed);
    setMotorState(false, FORWARD, (uint8_t)speed);
    DEBUG_PRINT((__FlashStringHelper*)STR_SPEED); DEBUG_PRINT(": ");
    DEBUG_PRINTLN(speed);
    
    // Update display every 50 speed units
    if (speed % 50 == 0) {
      snprintf(buf, sizeof(buf), "Spd: %d", speed);
      printToDisplay(6, buf);
    }
    delay(100);
  }
  
  // Ramp down
  DEBUG_PRINTLN("Ramping DOWN...");
  printToDisplayP(4, STR_RAMP_DOWN);
  for (int speed = 255; speed >= 0; speed -= 5) {
    setMotorState(true, FORWARD, (uint8_t)speed);
    setMotorState(false, FORWARD, (uint8_t)speed);
    DEBUG_PRINT((__FlashStringHelper*)STR_SPEED); DEBUG_PRINT(": ");
    DEBUG_PRINTLN(speed);
    
    // Update display every 50 speed units
    if (speed % 50 == 0) {
      snprintf(buf, sizeof(buf), "Spd: %d", speed);
      printToDisplay(6, buf);
    }
    delay(100);
  }
  
  setMotorState(true, COAST, 0);
  setMotorState(false, COAST, 0);
  disableDriver();
  DEBUG_PRINT((__FlashStringHelper*)STR_SPEED); DEBUG_PRINTLN(" ramp complete");
  printToDisplayP(6, STR_COMPLETE);
}

void testBrake() {
  enableDriver();
  DEBUG_PRINT("Running "); DEBUG_PRINT((__FlashStringHelper*)STR_MOTORS); DEBUG_PRINTLN(" at speed 255 for 1 second, then BRAKE");
  printToDisplayP(4, STR_FULL_SPEED);
  printToDisplayP(6, STR_RUNNING);
  setMotorState(true, FORWARD, 255);
  setMotorState(false, FORWARD, 255);
  delay(1000);
  
  DEBUG_PRINTLN("BRAKING NOW!");
  printToDisplayP(6, STR_BRAKING);
  setMotorState(true, BRAKE, 255);
  setMotorState(false, BRAKE, 255);
  delay(500);
  
  setMotorState(true, COAST, 0);
  setMotorState(false, COAST, 0);
  disableDriver();
  DEBUG_PRINTLN("Brake test complete");
  printToDisplayP(6, STR_COMPLETE);
}
