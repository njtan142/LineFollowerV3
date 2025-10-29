#ifndef EEPROM_MANAGER_H
#define EEPROM_MANAGER_H

#include <Arduino.h>
#include <EEPROM.h>

/**
 * @file eeprom_manager.h
 * @brief Centralized EEPROM management to prevent address conflicts
 * 
 * EEPROM Memory Layout:
 * - Address 0-99:   Settings (struct Settings)
 *   - 0-1:   Magic number (0xABCD)
 *   - 2:     Version byte
 *   - 3-99:  Settings data
 * 
 * - Address 100-151: Line Sensor Calibration
 *   - 100-101: Magic number (0x5E50)
 *   - 102-117: Sensor min values (8 sensors x 2 bytes)
 *   - 118-133: Sensor max values (8 sensors x 2 bytes)
 *   - 134-149: Sensor thresholds (8 sensors x 2 bytes)
 *   - 150-151: Threshold ratio enum value
 * 
 * - Address 152+: Reserved for future use
 * 
 * @warning Always use EEPROMManager for EEPROM operations to maintain layout consistency
 */
class EEPROMManager {
public:
    /**
     * @struct Addresses
     * @brief EEPROM address map for all stored data
     * 
     * Defines the memory layout of EEPROM to prevent overlaps.
     * When adding new data, use RESERVED_START as the base address.
     */
    struct Addresses {
        // Settings block (0-99)
        static const uint16_t SETTINGS_BASE = 0;    ///< Base address for Settings struct
        static const uint16_t SETTINGS_SIZE = 100;  ///< Size reserved for Settings
        
        // Line Sensor Calibration block (100-151)
        static const uint16_t SENSOR_MAGIC = 100;      ///< Sensor calibration magic number
        static const uint16_t SENSOR_MIN = 102;        ///< Min values (8 sensors x 2 bytes = 16 bytes)
        static const uint16_t SENSOR_MAX = 118;        ///< Max values (8 sensors x 2 bytes = 16 bytes)
        static const uint16_t SENSOR_THRESHOLD = 134;  ///< Thresholds (8 sensors x 2 bytes = 16 bytes)
        static const uint16_t THRESHOLD_RATIO = 150;   ///< Threshold ratio (2 bytes enum)
        static const uint16_t SENSOR_BLOCK_END = 152;  ///< End of sensor block
        
        // Reserved for future expansion
        static const uint16_t RESERVED_START = 152;    ///< Start of available space
    };
    
    /**
     * @brief Magic numbers for data validation
     * 
     * Each data block uses a unique magic number to verify integrity.
     * If magic number doesn't match, data is considered invalid.
     */
    enum : uint16_t {
        SETTINGS_MAGIC = 0xABCD,        ///< Magic number for Settings struct
        SENSOR_MAGIC_NUMBER = 0x5E50    ///< Magic number for sensor calibration ("SENS")
    };
    
    /**
     * @brief Read any type from EEPROM
     * @tparam T Type to read
     * @param address EEPROM address to read from
     * @return Data read from EEPROM
     */
    template<typename T>
    static T read(uint16_t address) {
        T data;
        EEPROM.get(address, data);
        return data;
    }
    
    /**
     * @brief Write any type to EEPROM
     * @tparam T Type to write
     * @param address EEPROM address to write to
     * @param data Data to write
     */
    template<typename T>
    static void write(uint16_t address, const T& data) {
        EEPROM.put(address, data);
    }
    
    /**
     * @brief Check if settings are valid (magic number matches)
     * @return true if valid, false otherwise
     */
    static bool isSettingsValid() {
        uint16_t magic = read<uint16_t>(Addresses::SETTINGS_BASE);
        return (magic == SETTINGS_MAGIC);
    }
    
    /**
     * @brief Check if sensor calibration is valid (magic number matches)
     * @return true if valid, false otherwise
     */
    static bool isSensorCalibrationValid() {
        uint16_t magic = read<uint16_t>(Addresses::SENSOR_MAGIC);
        return (magic == SENSOR_MAGIC_NUMBER);
    }
    
    /**
     * @brief Clear all EEPROM (write zeros)
     * @warning This erases ALL saved data
     */
    static void clearAll() {
        for (uint16_t i = 0; i < EEPROM.length(); i++) {
            EEPROM.write(i, 0);
        }
    }
    
    /**
     * @brief Clear only settings block
     */
    static void clearSettings() {
        for (uint16_t i = Addresses::SETTINGS_BASE; 
             i < Addresses::SETTINGS_BASE + Addresses::SETTINGS_SIZE; 
             i++) {
            EEPROM.write(i, 0);
        }
    }
    
    /**
     * @brief Clear only sensor calibration block
     */
    static void clearSensorCalibration() {
        for (uint16_t i = Addresses::SENSOR_MAGIC; 
             i < Addresses::SENSOR_BLOCK_END; 
             i++) {
            EEPROM.write(i, 0);
        }
    }
};

#endif
