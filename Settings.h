#ifndef SETTINGS_H
#define SETTINGS_H

#include "enums.h"
#include "eeprom_manager.h"
#include "constants.h"

/**
 * @file Settings.h
 * @brief Settings management with EEPROM persistence and version migration
 * 
 * This file defines the Settings structure that holds all configurable parameters
 * for the robot. Settings are saved to EEPROM and include version management for
 * backward compatibility.
 */

/**
 * @brief Current version of the Settings structure
 * 
 * Increment this value whenever the Settings struct layout changes.
 * Migration logic should be added to handle upgrades from older versions.
 */
#define SETTINGS_VERSION 1

/**
 * @struct Settings
 * @brief Holds all configurable robot parameters
 * 
 * This struct can be saved/loaded to EEPROM in one operation.
 * When modifying this structure:
 * 1. Increment SETTINGS_VERSION
 * 2. Add migration logic in migrateSettings()
 * 3. Update the default constructor
 */
struct Settings {
    // Version control (must be first after magic number)
    uint16_t magicNumber;  ///< Magic number for validation
    uint8_t version;       ///< Settings structure version
    
    // Motor settings
    int baseSpeed;           ///< Base speed for line following
    int alignmentOffset;     ///< Motor alignment correction offset
    int acceleration;        ///< Acceleration rate
    int deceleration;        ///< Deceleration rate
    int accelerationTime;    ///< Time to reach full speed (ms)
    int decelerationTime;    ///< Time to stop from full speed (ms)
    
    // PID settings
    int kp;                  ///< Proportional gain
    int ki;                  ///< Integral gain
    int kd;                  ///< Derivative gain
    int pidScale;            ///< PID scale factor for fixed-point math
    
    // Loop/control settings
    int timestep;            ///< Control loop timestep (ms)
    int movetime;            ///< Movement duration per step (ms)
    int breakTimestep;       ///< Brake timestep (ms)
    bool continuous;         ///< Continuous movement mode flag
    MovementType movementType;     ///< Movement profile type
    PIDControlMode pidControlMode; ///< PID control mode
    
    // Display settings
    bool displayAutoOff;     ///< Auto-off display after timeout
    
    // Sensor settings
    SensorMode sensorMode;   ///< Sensor array configuration mode
    
    /**
     * @brief Default constructor with factory defaults
     * 
     * Initializes all settings to their default values.
     * These values are used for new installations or after factory reset.
     */
    Settings() 
        : magicNumber(EEPROMManager::SETTINGS_MAGIC)
        , version(SETTINGS_VERSION)
        , baseSpeed(Constants::Motor::DEFAULT_BASE_SPEED)
        , alignmentOffset(0)
        , acceleration(Constants::Movement::DEFAULT_ACCELERATION)
        , deceleration(Constants::Movement::DEFAULT_DECELERATION)
        , accelerationTime(Constants::Movement::DEFAULT_ACCELERATION_TIME_MS)
        , decelerationTime(Constants::Movement::DEFAULT_DECELERATION_TIME_MS)
        , kp(Constants::PID::DEFAULT_KP)
        , ki(Constants::PID::DEFAULT_KI)
        , kd(Constants::PID::DEFAULT_KD)
        , pidScale(Constants::PID::DEFAULT_SCALE)
        , timestep(Constants::Movement::DEFAULT_TIMESTEP_MS)
        , movetime(Constants::Movement::DEFAULT_MOVETIME_MS)
        , breakTimestep(Constants::Movement::DEFAULT_BREAK_TIMESTEP_MS)
        , continuous(true)
        , movementType(MovementType::SMOOTH)
        , pidControlMode(PIDControlMode::NORMAL)
        , displayAutoOff(false)
        , sensorMode(SensorMode::MODIFIED_6_CHANNEL)  // Default to 6-channel mode
    {}
    
    /**
     * @brief Migrate settings from an older version
     * @param oldVersion The version to migrate from
     * 
     * This method handles backward compatibility when the Settings structure changes.
     * Add migration logic here for each version increment.
     * 
     * Example migration pattern:
     * @code
     * if (oldVersion < 2) {
     *     // Add new field with default value
     *     newField = defaultValue;
     * }
     * if (oldVersion < 3) {
     *     // Migrate field format
     *     newFormat = convertOldFormat(oldFormat);
     * }
     * @endcode
     */
    void migrateSettings(uint8_t oldVersion) {
        // Version 1 is the initial version - no migration needed yet
        // Future migrations will be added here as the structure evolves
        
        // Always update to current version after migration
        version = SETTINGS_VERSION;
        magicNumber = EEPROMManager::SETTINGS_MAGIC;
    }
    
    /**
     * @brief Load settings from EEPROM
     * 
     * Reads settings from EEPROM. If valid settings exist, loads them.
     * Otherwise, keeps default values.
     */
    void load() {
        if (EEPROMManager::isSettingsValid()) {
            Settings temp = EEPROMManager::read<Settings>(EEPROMManager::Addresses::SETTINGS_BASE);
            
            // Check version and migrate if needed
            if (temp.version != SETTINGS_VERSION) {
                temp.migrateSettings(temp.version);
            }
            
            // Copy loaded settings to this instance
            *this = temp;
        }
        // If not valid, keep current (default) values
    }
    
    /**
     * @brief Save settings to EEPROM
     * 
     * Updates version and magic number, then writes to EEPROM.
     */
    void save() {
        version = SETTINGS_VERSION;
        magicNumber = EEPROMManager::SETTINGS_MAGIC;
        EEPROMManager::write(EEPROMManager::Addresses::SETTINGS_BASE, *this);
    }
};

/**
 * @class SettingsManager
 * @brief Manages loading/saving settings to EEPROM with version control
 * 
 * Handles:
 * - Loading settings from EEPROM
 * - Saving settings to EEPROM
 * - Version checking and migration
 * - Factory reset to defaults
 */
class SettingsManager {
public:
    SettingsManager() {}
    
    /**
     * @brief Initialize EEPROM - load settings or create defaults
     * 
     * This method:
     * 1. Checks if valid settings exist in EEPROM
     * 2. If valid, loads them and checks version
     * 3. If version mismatch, performs migration
     * 4. If invalid or missing, creates defaults and saves
     */
    void begin() {
        if (EEPROMManager::isSettingsValid()) {
            // Load existing settings
            load();
            
            // Check if migration is needed
            if (settings.version != SETTINGS_VERSION) {
                uint8_t oldVersion = settings.version;
                settings.migrateSettings(oldVersion);
                save(); // Save migrated settings
            }
        } else {
            // No valid settings found - use defaults and save them
            settings = Settings();  // Use default constructor
            save();
        }
    }
    
    /**
     * @brief Check if EEPROM contains valid settings
     * @return true if magic number matches
     */
    bool isValid() const {
        return EEPROMManager::isSettingsValid();
    }
    
    /**
     * @brief Get the version of settings stored in EEPROM
     * @return Version number, or 0 if invalid
     */
    uint8_t getStoredVersion() const {
        if (!EEPROMManager::isSettingsValid()) {
            return 0;
        }
        // Read just the version byte
        return EEPROMManager::read<uint8_t>(EEPROMManager::Addresses::SETTINGS_BASE + sizeof(uint16_t));
    }
    
    /**
     * @brief Save settings to EEPROM
     * 
     * Writes the entire Settings struct to EEPROM.
     * Always updates version to current before saving.
     */
    void save() {
        settings.version = SETTINGS_VERSION;
        settings.magicNumber = EEPROMManager::SETTINGS_MAGIC;
        EEPROMManager::write(EEPROMManager::Addresses::SETTINGS_BASE, settings);
    }
    
    /**
     * @brief Load settings from EEPROM
     * 
     * Reads the entire Settings struct from EEPROM.
     * Should only be called after verifying settings are valid.
     */
    void load() {
        settings = EEPROMManager::read<Settings>(EEPROMManager::Addresses::SETTINGS_BASE);
    }
    
    /**
     * @brief Reset to factory defaults
     * 
     * Creates a new Settings object with default values and saves to EEPROM.
     */
    void resetToDefaults() {
        settings = Settings();  // Use default constructor
        save();
    }
    
    /**
     * @brief Get reference to current settings (for reading/modification)
     * @return Mutable reference to settings
     */
    Settings& getSettings() { return settings; }
    
    /**
     * @brief Get const reference to current settings (for reading only)
     * @return Const reference to settings
     */
    const Settings& getSettings() const { return settings; }
    
private:
    Settings settings;  ///< Current settings instance
};

#endif
