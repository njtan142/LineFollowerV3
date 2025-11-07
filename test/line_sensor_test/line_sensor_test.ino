/*
 * Line Sensor Test Sketch
 * 
 * This sketch reads raw analog values from 8 line sensors (A0-A7)
 * and displays them on the Serial Monitor.
 * LED on pin 11 is used to enable the sensor array.
 * 
 * Hardware Connections:
 * - A0 to A7: Line sensor inputs
 * - Pin 11: LED enable for sensor array
 * 
 * Baud Rate: 115200
 */

#include <Arduino.h>

// Pin definitions
#define LED_ENABLE_PIN 11

// Sensor pins
const uint8_t sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Variables to store sensor readings
uint16_t sensorValues[8];

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for some boards)
  }
  
  // Configure LED enable pin
  pinMode(LED_ENABLE_PIN, OUTPUT);
  digitalWrite(LED_ENABLE_PIN, HIGH); // Turn on sensor array LEDs
  
  // Analog pins don't need pinMode configuration for analogRead
  // Note: A6 and A7 on Arduino Nano are analog-only and don't support INPUT_PULLUP
  
  Serial.println("Line Sensor Test");
  Serial.println("================");
  Serial.println("Reading raw analog values from A0 to A7");
  Serial.println();
  
  delay(1000); // Allow sensors to stabilize
}

void loop() {
  // Read all sensor values with a small delay for ADC settling
  for (uint8_t i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    delayMicroseconds(100); // Allow ADC to settle between readings
  }
  
  // Display sensor values
  Serial.print("Sensors: ");
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print("A");
    Serial.print(i);
    Serial.print(":");
    Serial.print(sensorValues[i]);
    
    if (i < 7) {
      Serial.print(" | ");
    }
  }
  Serial.println();
  
  // Optional: Display in bar graph format
  Serial.print("Graph:   ");
  for (uint8_t i = 0; i < 8; i++) {
    // Scale value to 0-10 range for visualization
    uint8_t barLength = map(sensorValues[i], 0, 1023, 0, 10);
    Serial.print("[");
    for (uint8_t j = 0; j < 10; j++) {
      if (j < barLength) {
        Serial.print("#");
      } else {
        Serial.print(" ");
      }
    }
    Serial.print("] ");
  }
  Serial.println();
  Serial.println();
  
  delay(500); // Update rate: 5 times per second
}
