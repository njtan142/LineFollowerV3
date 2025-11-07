/**
 * EEPROM Reset Test
 * 
 * This test sketch is used to completely clear and reset the EEPROM.
 * It performs the following operations:
 * 
 * 1. Reads and displays ALL current values from EEPROM:
 *    - Settings (magic number, version, motor, PID, display settings)
 *    - Line Sensor Calibration (magic number, min, max, threshold values)
 * 
 * 2. Clears ALL EEPROM data (writes zeros to every byte)
 * 
 * 3. Reads and displays values again to confirm clearing
 * 
 * WARNING: This will erase ALL saved data including:
 *          - All settings (motor speeds, PID values, etc.)
 *          - Line sensor calibration data
 *          - Everything needs to be reconfigured from scratch
 * 
 * Usage:
 * 1. Upload this sketch to the Arduino
 * 2. Open Serial Monitor at 115200 baud
 * 3. Review the current values
 * 4. Send any character to proceed with clearing
 * 5. Wait for confirmation that clearing is complete
 */

#include <EEPROM.h>

// Copy EEPROM memory map constants (matching eeprom_manager.h)
namespace EEPROMAddresses {
    // Settings block (0-99)
    const uint16_t SETTINGS_BASE = 0;
    const uint16_t SETTINGS_SIZE = 100;
    
    // Line Sensor Calibration block (100-151)
    const uint16_t SENSOR_MAGIC = 100;
    const uint16_t SENSOR_MIN = 102;
    const uint16_t SENSOR_MAX = 118;
    const uint16_t SENSOR_THRESHOLD = 134;
    const uint16_t THRESHOLD_RATIO = 150;
    const uint16_t SENSOR_BLOCK_END = 152;
    
    // Reserved for future expansion
    const uint16_t RESERVED_START = 152;
}

// Magic numbers for validation
const uint16_t SETTINGS_MAGIC = 0xABCD;
const uint16_t SENSOR_MAGIC_NUMBER = 0x5E50;

// Number of sensors
const uint8_t SENSORCOUNT = 8;

void setup() {
    Serial.begin(115200);
    
    // Wait for serial connection
    while (!Serial) {
        delay(100);
    }
    
    delay(1000);  // Give user time to open serial monitor
    
    Serial.println(F("============================================"));
    Serial.println(F("      EEPROM RESET TEST - FULL CLEAR"));
    Serial.println(F("============================================"));
    Serial.println();
    Serial.println(F("WARNING: This will erase ALL EEPROM data!"));
    Serial.println();
    
    // Step 1: Display current EEPROM values
    Serial.println(F("--- STEP 1: READING CURRENT EEPROM VALUES ---"));
    Serial.println();
    displayAllEEPROMData();
    
    // Wait for user confirmation
    Serial.println();
    Serial.println(F("============================================"));
    Serial.println(F("Send any character to CLEAR all EEPROM data"));
    Serial.println(F("(This will erase everything!)"));
    Serial.println(F("============================================"));
    
    // Wait for input
    while (!Serial.available()) {
        delay(100);
    }
    
    // Clear the serial buffer
    while (Serial.available()) {
        Serial.read();
    }
    
    // Step 2: Clear all EEPROM
    Serial.println();
    Serial.println(F("--- STEP 2: CLEARING ALL EEPROM DATA ---"));
    Serial.println();
    clearAllEEPROM();
    
    // Step 3: Display values after clearing
    Serial.println();
    Serial.println(F("--- STEP 3: READING EEPROM AFTER CLEARING ---"));
    Serial.println();
    displayAllEEPROMData();
    
    // Final message
    Serial.println();
    Serial.println(F("============================================"));
    Serial.println(F("EEPROM RESET COMPLETE!"));
    Serial.println(F("============================================"));
    Serial.println();
    Serial.println(F("All data has been cleared. Next steps:"));
    Serial.println(F("1. Upload the main line_follower_v2 sketch"));
    Serial.println(F("2. Reconfigure all settings"));
    Serial.println(F("3. Perform sensor calibration"));
    Serial.println();
}

void loop() {
    // Nothing to do in loop
    delay(1000);
}

/**
 * Display all EEPROM data in a structured format
 */
void displayAllEEPROMData() {
    Serial.println(F("========== SETTINGS BLOCK (0-99) =========="));
    displaySettingsData();
    
    Serial.println();
    Serial.println(F("======= LINE SENSOR CALIBRATION (100-151) ======="));
    displaySensorCalibrationData();
    
    Serial.println();
    Serial.println(F("========== RAW EEPROM DUMP =========="));
    displayRawEEPROMDump();
}

/**
 * Display Settings block data
 */
void displaySettingsData() {
    // Read magic number and version
    uint16_t magicNumber;
    EEPROM.get(EEPROMAddresses::SETTINGS_BASE, magicNumber);
    
    uint8_t version;
    EEPROM.get(EEPROMAddresses::SETTINGS_BASE + sizeof(uint16_t), version);
    
    Serial.print(F("Magic Number: 0x"));
    Serial.print(magicNumber, HEX);
    if (magicNumber == SETTINGS_MAGIC) {
        Serial.println(F(" (VALID)"));
    } else {
        Serial.println(F(" (INVALID)"));
    }
    
    Serial.print(F("Version: "));
    Serial.println(version);
    
    // Display first 20 bytes of settings as hex
    Serial.print(F("First 20 bytes (hex): "));
    for (int i = 0; i < 20; i++) {
        uint8_t val = EEPROM.read(EEPROMAddresses::SETTINGS_BASE + i);
        if (val < 16) Serial.print('0');
        Serial.print(val, HEX);
        Serial.print(' ');
    }
    Serial.println();
    
    // Note: Full settings structure would require including Settings.h
    // For this test, we just show the key indicators
    Serial.println(F("(Note: Full settings details require main sketch)"));
}

/**
 * Display Line Sensor Calibration data
 */
void displaySensorCalibrationData() {
    // Read sensor magic number
    uint16_t sensorMagic;
    EEPROM.get(EEPROMAddresses::SENSOR_MAGIC, sensorMagic);
    
    Serial.print(F("Sensor Magic: 0x"));
    Serial.print(sensorMagic, HEX);
    if (sensorMagic == SENSOR_MAGIC_NUMBER) {
        Serial.println(F(" (VALID)"));
    } else {
        Serial.println(F(" (INVALID)"));
    }
    
    Serial.println();
    
    // Display sensor min values
    Serial.println(F("Sensor Min Values (white):"));
    for (int i = 0; i < SENSORCOUNT; i++) {
        int minVal;
        EEPROM.get(EEPROMAddresses::SENSOR_MIN + i * 2, minVal);
        Serial.print(F("  Sensor "));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.println(minVal);
    }
    
    Serial.println();
    
    // Display sensor max values
    Serial.println(F("Sensor Max Values (black):"));
    for (int i = 0; i < SENSORCOUNT; i++) {
        int maxVal;
        EEPROM.get(EEPROMAddresses::SENSOR_MAX + i * 2, maxVal);
        Serial.print(F("  Sensor "));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.println(maxVal);
    }
    
    Serial.println();
    
    // Display sensor threshold values
    Serial.println(F("Sensor Threshold Values:"));
    for (int i = 0; i < SENSORCOUNT; i++) {
        int threshold;
        EEPROM.get(EEPROMAddresses::SENSOR_THRESHOLD + i * 2, threshold);
        Serial.print(F("  Sensor "));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.println(threshold);
    }
    
    Serial.println();
    
    // Display threshold ratio
    uint8_t thresholdRatio;
    EEPROM.get(EEPROMAddresses::THRESHOLD_RATIO, thresholdRatio);
    Serial.print(F("Threshold Ratio: "));
    Serial.print(thresholdRatio);
    Serial.print(F(" ("));
    switch (thresholdRatio) {
        case 0: Serial.print(F("15/16")); break;
        case 1: Serial.print(F("7/8")); break;
        case 2: Serial.print(F("3/4")); break;
        case 3: Serial.print(F("2/3")); break;
        case 4: Serial.print(F("1/2")); break;
        default: Serial.print(F("UNKNOWN")); break;
    }
    Serial.println(F(")"));
}

/**
 * Display raw EEPROM dump (first 152 bytes)
 */
void displayRawEEPROMDump() {
    Serial.println(F("Raw EEPROM (addresses 0-151):"));
    Serial.println(F("Addr: +0 +1 +2 +3 +4 +5 +6 +7 +8 +9 +A +B +C +D +E +F"));
    Serial.println(F("----------------------------------------------------"));
    
    for (int addr = 0; addr < 152; addr += 16) {
        // Print address
        if (addr < 100) Serial.print(' ');
        if (addr < 10) Serial.print(' ');
        Serial.print(addr);
        Serial.print(F(": "));
        
        // Print 16 bytes in hex
        for (int i = 0; i < 16 && (addr + i) < 152; i++) {
            uint8_t val = EEPROM.read(addr + i);
            if (val < 16) Serial.print('0');
            Serial.print(val, HEX);
            Serial.print(' ');
        }
        Serial.println();
    }
}

/**
 * Clear ALL EEPROM data by writing zeros
 */
void clearAllEEPROM() {
    Serial.println(F("Clearing EEPROM..."));
    Serial.print(F("EEPROM size: "));
    Serial.print(EEPROM.length());
    Serial.println(F(" bytes"));
    
    int totalBytes = EEPROM.length();
    int progressStep = totalBytes / 20;  // 20 progress markers
    
    Serial.print(F("Progress: ["));
    
    for (int i = 0; i < totalBytes; i++) {
        EEPROM.write(i, 0);
        
        // Show progress
        if (i % progressStep == 0) {
            Serial.print('#');
        }
    }
    
    Serial.println(F("] DONE"));
    Serial.println(F("All EEPROM bytes have been set to 0x00"));
}
