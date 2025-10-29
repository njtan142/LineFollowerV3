/**
 * PID Edit Screen - Edit PID parameters
 * Allows editing KP, KI, KD, Scale with fine control
 * Features:
 * - Navigation mode: scroll through parameters using MenuSystem
 * - Edit mode: adjust values with potentiometer
 * - Save to EEPROM or discard changes
 */

#ifndef PID_EDIT_H
#define PID_EDIT_H

#include <Arduino.h>
#include <U8x8lib.h>
#include "Settings.h"
#include "menu_system.h"

class PIDEdit {
private:
    // Display constants
    static const uint8_t TOTAL_ITEMS = 6;    // KP, KI, KD, Scale, SAVE, BACK
    
    // Item indices
    enum Item {
        ITEM_KP = 0,
        ITEM_KI = 1,
        ITEM_KD = 2,
        ITEM_SCALE = 3,
        ITEM_SAVE = 4,
        ITEM_BACK = 5
    };
    
    // UI state
    U8X8_SSD1306_128X64_NONAME_SW_I2C* display;
    uint8_t potPin;
    uint8_t buttonPin;
    uint8_t potEnablePin;
    
    // Menu system for navigation
    MenuSystem* menu;
    
    // Edit mode state
    bool isEditMode;
    
    // Value state
    Settings workingSettings;  // Working copy of settings
    int* currentValue;         // Pointer to currently editing value
    
    // Input state for edit mode only
    int lastPotValue;
    bool lastButtonStateEdit;  // Separate button state for edit mode
    unsigned long lastDebounceTimeEdit;
    bool lastButtonReadingEdit;
    bool buttonStateEdit;
    unsigned long lastIncrementTime;  // For incremental updates
    int initialValue;  // Store initial value when entering edit mode
    
    // Labels
    static const char* const labels[];
    
    // Menu items with values
    static const uint8_t MENU_ITEM_BUFFER_SIZE = 17; // 16 chars + null terminator
    char menuItemsBuffer[6][MENU_ITEM_BUFFER_SIZE];  // 6 items x MENU_ITEM_BUFFER_SIZE chars
    const char* menuItems[6];
    
    /**
     * Read button with debouncing (for edit mode only)
     * Separate from MenuSystem to avoid conflicts
     */
    bool readButtonDebouncedEdit() {
        bool reading = (digitalRead(buttonPin) == HIGH);
        
        if (reading != lastButtonReadingEdit) {
            lastDebounceTimeEdit = millis();
        }
        
        if ((millis() - lastDebounceTimeEdit) > 50) {
            if (reading != buttonStateEdit) {
                buttonStateEdit = reading;
            }
        }
        
        lastButtonReadingEdit = reading;
        return buttonStateEdit;
    }
    
    /**
     * Get pointer to value for given item
     */
    int* getValuePointer(uint8_t item) {
        switch (item) {
            case ITEM_KP: return &workingSettings.kp;
            case ITEM_KI: return &workingSettings.ki;
            case ITEM_KD: return &workingSettings.kd;
            case ITEM_SCALE: return &workingSettings.pidScale;
            default: return nullptr;
        }
    }
    
    /**
     * Update menu items with current values
     */
    void updateMenuItems() {
        // Format numeric items with values
        snprintf(menuItemsBuffer[0], MENU_ITEM_BUFFER_SIZE, "KP:%d", workingSettings.kp);
        snprintf(menuItemsBuffer[1], MENU_ITEM_BUFFER_SIZE, "KI:%d", workingSettings.ki);
        snprintf(menuItemsBuffer[2], MENU_ITEM_BUFFER_SIZE, "KD:%d", workingSettings.kd);
        snprintf(menuItemsBuffer[3], MENU_ITEM_BUFFER_SIZE, "Scl:%d", workingSettings.pidScale);
        snprintf(menuItemsBuffer[4], MENU_ITEM_BUFFER_SIZE, "SAVE");
        snprintf(menuItemsBuffer[5], MENU_ITEM_BUFFER_SIZE, "BACK");
        
        // Point to buffers
        for (uint8_t i = 0; i < 6; i++) {
            menuItems[i] = menuItemsBuffer[i];
        }
    }
    
    /**
     * Draw edit mode overlay
     */
    void drawEditOverlay() {
        display->setFont(u8x8_font_8x13_1x2_f);
        
        // Clear bottom area for edit display
        display->setCursor(0, 6);
        display->print("                ");
        display->setCursor(0, 6);
        display->print(">");
        display->print(labels[menu->getSelection()]);
        display->print(":");
        if (currentValue) {
            display->print(*currentValue);
        }
    }
    
    /**
     * Update edit mode value display
     */
    void updateEditValue() {
        display->setFont(u8x8_font_8x13_1x2_f);
        uint8_t labelLen = strlen(labels[menu->getSelection()]);
        display->setCursor(2 + labelLen, 6);
        display->print("      ");  // Clear old value
        display->setCursor(2 + labelLen, 6);
        if (currentValue) {
            display->print(*currentValue);
        }
    }
    
public:
    /**
     * Constructor
     */
    PIDEdit(U8X8_SSD1306_128X64_NONAME_SW_I2C* u8x8Display,
            uint8_t pot = A7,
            uint8_t button = 5,
            uint8_t potEnable = 9)
        : display(u8x8Display)
        , potPin(pot)
        , buttonPin(button)
        , potEnablePin(potEnable)
        , menu(nullptr)
        , isEditMode(false)
        , currentValue(nullptr)
        , lastPotValue(-1)
        , lastButtonStateEdit(false)
        , lastDebounceTimeEdit(0)
        , lastButtonReadingEdit(false)
        , buttonStateEdit(false)
        , lastIncrementTime(0)
        , initialValue(0)
    {
        // Load current settings
        workingSettings.load();
        
        // Initialize menu items
        updateMenuItems();
        
        // Create menu system
        menu = new MenuSystem(display, menuItems, TOTAL_ITEMS, "PID", 3, pot, button, potEnable);
    }
    
    /**
     * Destructor
     */
    ~PIDEdit() {
        if (menu != nullptr) {
            delete menu;
        }
    }
    
    /**
     * Initialize
     */
    void begin() {
        pinMode(potPin, INPUT);
        pinMode(buttonPin, INPUT);
        pinMode(potEnablePin, OUTPUT);
        digitalWrite(potEnablePin, HIGH);
        
        // Initialize menu
        menu->begin();
    }
    
    /**
     * Update state and handle input
     * Returns: 0 = stay in PID edit, 1 = exit to settings menu
     */
    uint8_t update() {
        if (isEditMode) {
            // EDIT MODE: Adjust value incrementally using potentiometer
            // Read button state for edit mode
            bool buttonPressed = readButtonDebouncedEdit();
            bool buttonJustPressed = (buttonPressed && !lastButtonStateEdit);
            lastButtonStateEdit = buttonPressed;
            
            // Get pot value from MenuSystem
            int potValue = menu->getPotValue();
            
            // Determine if editing Scale (different range and increment)
            uint8_t selection = menu->getSelection();
            bool isScale = (selection == ITEM_SCALE);
            
            // Use delta-based movement like MenuSystem
            // Calculate movement threshold based on increment size
            int movementThreshold;
            int incrementSize;
            int minValue, maxValue;
            // Constants for potentiometer scaling
            const int POT_FULL_RANGE = 1024;      // 10-bit ADC range
            const int SCALE_POT_FACTOR = 7;       // Empirical factor for scale sensitivity
            const int SCALE_POT_INCREMENTS = 30;  // Target number of increments for scale

            const int PID_POT_INCREMENTS = 100;   // Target number of increments for KP/KI/KD

            if (isScale) {
                // Scale: increment by 100, range 100-3000
                incrementSize = 100;
                minValue = 100;
                maxValue = 3000;
                // Calculate movement threshold so that turning the pot across its range gives about SCALE_POT_INCREMENTS steps
                movementThreshold = max(5, (POT_FULL_RANGE * SCALE_POT_FACTOR) / SCALE_POT_INCREMENTS);  // ~30 increments across full range
            } else {
                // KP, KI, KD: increment by 1, range 0-100
                incrementSize = 1;
                minValue = 0;
                maxValue = 100;
                // Calculate movement threshold so that turning the pot across its range gives about 43 increments across full range (1024 * 7 / 300 ≈ 23.9)
                movementThreshold = max(5, (POT_FULL_RANGE * SCALE_POT_FACTOR) / PID_POT_INCREMENTS);  // ~43 increments across full range
            }
            
            if (currentValue) {
                int potDelta = potValue - lastPotValue;
                
                // Loop to handle multiple increments for fast pot movements
                while (potDelta > movementThreshold) {
                    // Pot moved up - decrease value
                    if (*currentValue > minValue) {
                        int newValue = *currentValue - incrementSize;
                        if (newValue < minValue) newValue = minValue;
                        *currentValue = newValue;
                        
                        // Update display
                        updateMenuItems();
                        updateEditValue();
                    }
                    potDelta -= movementThreshold;
                    lastPotValue += movementThreshold;
                }
                
                while (potDelta < -movementThreshold) {
                    // Pot moved down - increase value
                    if (*currentValue < maxValue) {
                        int newValue = *currentValue + incrementSize;
                        if (newValue > maxValue) newValue = maxValue;
                        *currentValue = newValue;
                        
                        // Update display
                        updateMenuItems();
                        updateEditValue();
                    }
                    potDelta += movementThreshold;
                    lastPotValue -= movementThreshold;
                }
            }
            
            // Button exits edit mode
            if (buttonJustPressed) {
                isEditMode = false;
                
                // Update menu items and redraw
                updateMenuItems();
                menu->requestRedraw();
            }
            
        } else {
            // NAVIGATION MODE: Use MenuSystem
            menu->update();
            
            // Handle button press
            if (menu->isButtonJustPressed()) {
                uint8_t selection = menu->getSelection();
                
                if (selection == ITEM_SAVE) {
                    // Save to EEPROM
                    workingSettings.save();
                    return 1;  // Exit to settings menu
                    
                } else if (selection == ITEM_BACK) {
                    // Discard changes and exit
                    return 1;  // Exit to settings menu
                    
                } else {
                    // Enter edit mode for numeric value
                    currentValue = getValuePointer(selection);
                    if (currentValue) {
                        isEditMode = true;
                        initialValue = *currentValue;  // Store initial value
                        lastPotValue = menu->getPotValue();  // Initialize lastPotValue
                        drawEditOverlay();
                    }
                }
            }
        }
        
        return 0;  // Stay in PID edit
    }
    
    /**
     * Request redraw
     */
    void requestRedraw() {
        updateMenuItems();
        menu->requestRedraw();
    }
};

// Static member definitions
const char* const PIDEdit::labels[] = {
    "KP",
    "KI",
    "KD",
    "Scale",
    "SAVE",
    "BACK"
};

#endif // PID_EDIT_H
