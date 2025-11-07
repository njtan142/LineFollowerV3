/**
 * PID Edit Module - Header
 * 
 * Interactive interface for editing PID controller parameters with real-time
 * value adjustment using potentiometer input. Provides two-mode interaction:
 * navigation mode (scrolling through parameters) and edit mode (adjusting values).
 * 
 * Key Features:
 * - Two-mode operation (navigation/edit)
 * - Real-time value adjustment with potentiometer
 * - Delta-based incremental changes (no absolute positioning)
 * - Adaptive sensitivity based on parameter type
 * - Working copy prevents accidental changes
 * - EEPROM persistence on save
 * - Visual feedback with edit overlay
 * 
 * Editable Parameters:
 * - KP: Proportional gain (0-100, increment by 1)
 * - KI: Integral gain (0-100, increment by 1)
 * - KD: Derivative gain (0-100, increment by 1)
 * - Scale: PID output scaling (100-3000, increment by 100)
 * 
 * Operation Modes:
 * 1. Navigation Mode:
 *    - MenuSystem handles potentiometer scrolling and button presses
 *    - Select parameter with button press
 *    - Choose SAVE to persist or BACK to discard
 * 
 * 2. Edit Mode:
 *    - Direct potentiometer control of parameter value
 *    - Real-time display update
 *    - Button press exits back to navigation mode
 *    - Delta-based movement with adaptive thresholds
 * 
 * Usage Pattern:
 * 1. Create PIDEdit with display and pin configuration
 * 2. Call begin() to initialize
 * 3. Call update() in main loop
 * 4. Check return value: 0 = continue editing, 1 = exit to parent menu
 * 5. Handle exit by switching to appropriate menu
 */

#ifndef PID_EDIT_H
#define PID_EDIT_H

#include <Arduino.h>
#include <U8x8lib.h>
#include "Settings.h"
#include "menu_system.h"

/**
 * PIDEdit Class
 * 
 * Provides full-featured interface for editing PID parameters.
 * Manages two interaction modes and handles all input/display logic.
 */
class PIDEdit {
private:
    // === Configuration Constants ===
    static const uint8_t TOTAL_ITEMS = 6;    ///< Total menu items (4 params + SAVE + BACK)
    
    /**
     * Menu Item Indices
     * Defines the order and position of each parameter in the menu
     */
    enum Item {
        ITEM_KP = 0,      ///< Proportional gain parameter
        ITEM_KI = 1,      ///< Integral gain parameter
        ITEM_KD = 2,      ///< Derivative gain parameter
        ITEM_SCALE = 3,   ///< PID output scaling factor
        ITEM_SAVE = 4,    ///< Save changes to EEPROM
        ITEM_BACK = 5     ///< Discard changes and exit
    };
    
    // === Display Hardware ===
    U8X8_SSD1306_128X64_NONAME_SW_I2C* display;  ///< OLED display driver
    
    // === Input Hardware Configuration ===
    uint8_t potPin;         ///< Analog input pin for potentiometer
    uint8_t buttonPin;      ///< Digital input pin for button
    uint8_t potEnablePin;   ///< Digital output pin to power potentiometer
    
    // === Navigation System ===
    MenuSystem* menu;  ///< Menu system for parameter selection (navigation mode)
    
    // === Mode State ===
    bool isEditMode;  ///< true = editing value, false = navigating menu
    
    // === Value State ===
    Settings workingSettings;  ///< Working copy of settings (prevents accidental changes)
    int* currentValue;         ///< Pointer to currently editing parameter value
    
    // === Edit Mode Input State ===
    // Separate from MenuSystem to avoid conflicts during edit mode
    int lastPotValue;                      ///< Previous pot reading for delta calculation
    bool lastButtonStateEdit;              ///< Previous debounced button state
    unsigned long lastDebounceTimeEdit;    ///< Timestamp of last button state change
    bool lastButtonReadingEdit;            ///< Raw button reading from previous frame
    bool buttonStateEdit;                  ///< Debounced button state (stable)
    unsigned long lastIncrementTime;       ///< Timestamp of last value change (unused)
    int initialValue;                      ///< Value when entering edit mode (for restore)
    
    // === Display Formatting ===
    static const char* const labels[];  ///< Short parameter labels for display
    
    // Menu item strings with current values
    static const uint8_t MENU_ITEM_BUFFER_SIZE = 17;  ///< 16 display chars + null terminator
    char menuItemsBuffer[6][MENU_ITEM_BUFFER_SIZE];   ///< Formatted menu item strings
    const char* menuItems[6];                          ///< Pointers to menu item strings
    
    /**
     * Reads button state with debouncing (edit mode only)
     * 
     * Separate debouncing implementation from MenuSystem to prevent
     * conflicts between navigation and edit modes. Uses same 50ms
     * threshold as MenuSystem for consistency.
     * 
     * @return Debounced button state (true = pressed, false = released)
     */
    bool readButtonDebouncedEdit() {
        // Read current raw button state
        bool reading = (digitalRead(buttonPin) == HIGH);
        
        // If reading changed, reset debounce timer
        if (reading != lastButtonReadingEdit) {
            lastDebounceTimeEdit = millis();
        }
        
        // Check if reading has been stable for debounce period
        if ((millis() - lastDebounceTimeEdit) > 50) {  // 50ms threshold
            // Reading stable - update button state if changed
            if (reading != buttonStateEdit) {
                buttonStateEdit = reading;
            }
        }
        
        // Store reading for next iteration
        lastButtonReadingEdit = reading;
        return buttonStateEdit;
    }
    
    /**
     * Gets pointer to parameter value by menu item index
     * 
     * Provides safe access to working settings values. Returns nullptr
     * for non-editable items (SAVE, BACK).
     * 
     * @param item - Menu item index
     * @return Pointer to parameter value, or nullptr if not editable
     */
    int* getValuePointer(uint8_t item) {
        // Map menu item to settings field
        switch (item) {
            case ITEM_KP: return &workingSettings.kp;
            case ITEM_KI: return &workingSettings.ki;
            case ITEM_KD: return &workingSettings.kd;
            case ITEM_SCALE: return &workingSettings.pidScale;
            default: return nullptr;  // SAVE and BACK are not editable
        }
    }
    
    /**
     * Updates menu item strings with current parameter values
     * 
     * Formats each menu item to show parameter name and current value.
     * Called after any value change to keep display synchronized.
     * 
     * Format Examples:
     * - "KP:42"
     * - "Scale:1500"
     * - "SAVE"
     * - "BACK"
     */
    void updateMenuItems() {
        // Format parameter items with current values
        snprintf(menuItemsBuffer[0], MENU_ITEM_BUFFER_SIZE, "KP:%d", workingSettings.kp);
        snprintf(menuItemsBuffer[1], MENU_ITEM_BUFFER_SIZE, "KI:%d", workingSettings.ki);
        snprintf(menuItemsBuffer[2], MENU_ITEM_BUFFER_SIZE, "KD:%d", workingSettings.kd);
        snprintf(menuItemsBuffer[3], MENU_ITEM_BUFFER_SIZE, "Scl:%d", workingSettings.pidScale);
        
        // Action items (no values)
        snprintf(menuItemsBuffer[4], MENU_ITEM_BUFFER_SIZE, "SAVE");
        snprintf(menuItemsBuffer[5], MENU_ITEM_BUFFER_SIZE, "BACK");
        
        // Update pointers array
        for (uint8_t i = 0; i < 6; i++) {
            menuItems[i] = menuItemsBuffer[i];
        }
    }
    
    /**
     * Draws edit mode overlay at bottom of screen
     * 
     * Shows currently editing parameter with value in larger format.
     * Displayed at row 6 to avoid conflict with menu items (rows 0-4).
     * Prefix ">" indicates edit mode is active.
     * 
     * Format: ">KP:42"
     */
    void drawEditOverlay() {
        display->setFont(u8x8_font_8x13_1x2_f);
        
        // Clear bottom row
        display->setCursor(0, 6);
        display->print("                ");  // 16 spaces
        
        // Draw edit indicator and parameter
        display->setCursor(0, 6);
        display->print(">");
        display->print(labels[menu->getSelection()]);
        display->print(":");
        
        // Draw current value
        if (currentValue) {
            display->print(*currentValue);
        }
    }
    
    /**
     * Updates only the value portion of edit overlay
     * 
     * Partial display update for efficiency. Clears and redraws only
     * the numeric value, leaving label unchanged. Reduces flicker
     * during rapid value changes.
     */
    void updateEditValue() {
        display->setFont(u8x8_font_8x13_1x2_f);
        
        // Calculate cursor position (after ">Label:")
        uint8_t labelLen = strlen(labels[menu->getSelection()]);
        display->setCursor(2 + labelLen, 6);
        
        // Clear old value area
        display->print("      ");  // 6 spaces (enough for max value)
        
        // Redraw new value
        display->setCursor(2 + labelLen, 6);
        if (currentValue) {
            display->print(*currentValue);
        }
    }
    
public:
    /**
     * Constructor - Initializes PID editor
     * 
     * Creates editor with specified hardware configuration. Loads current
     * settings from EEPROM into working copy, formats menu items, and
     * creates menu system for navigation.
     * 
     * @param u8x8Display - Pointer to U8x8 display object (must be initialized)
     * @param pot - Analog pin for potentiometer (default: A7)
     * @param button - Digital pin for button input (default: 5)
     * @param potEnable - Digital pin to enable potentiometer power (default: 9)
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
        // Load current settings from EEPROM into working copy
        workingSettings.load();
        
        // Format menu items with loaded values
        updateMenuItems();
        
        // Create menu system for navigation mode
        menu = new MenuSystem(display, menuItems, TOTAL_ITEMS, "PID", 3, pot, button, potEnable);
    }
    
    /**
     * Destructor - Cleans up allocated resources
     * 
     * Deletes dynamically allocated MenuSystem to prevent memory leak.
     */
    ~PIDEdit() {
        if (menu != nullptr) {
            delete menu;
        }
    }
    
    /**
     * Initializes hardware and displays menu
     * 
     * Sets up GPIO pins and initializes menu system.
     * Must be called once before using update().
     */
    void begin() {
        // Configure GPIO pins
        pinMode(potPin, INPUT);
        pinMode(buttonPin, INPUT);
        pinMode(potEnablePin, OUTPUT);
        digitalWrite(potEnablePin, HIGH);  // Enable pot power
        
        // Initialize menu display
        menu->begin();
    }
    
    /**
     * Updates state and handles input (call in main loop)
     * 
     * Main update function with dual-mode operation:
     * - Navigation Mode: MenuSystem handles scrolling and selection
     * - Edit Mode: Direct potentiometer control of parameter values
     * 
     * Navigation Mode Flow:
     * 1. Update MenuSystem
     * 2. Check for button press
     * 3. Handle selection:
     *    - SAVE: Persist to EEPROM and exit
     *    - BACK: Discard changes and exit
     *    - Parameter: Enter edit mode
     * 
     * Edit Mode Flow:
     * 1. Read potentiometer delta
     * 2. Calculate adaptive threshold based on parameter type
     * 3. Apply incremental changes:
     *    - KP/KI/KD: ±1 per threshold crossing
     *    - Scale: ±100 per threshold crossing
     * 4. Update display on value change
     * 5. Button press exits to navigation mode
     * 
     * Adaptive Sensitivity:
     * - PID params: ~100 increments across full pot range (fine control)
     * - Scale: ~30 increments across full pot range (coarse control)
     * 
     * This provides appropriate sensitivity for each parameter's
     * typical adjustment range and step size.
     * 
     * @return Exit status: 0 = continue editing, 1 = exit to parent menu
     */
    uint8_t update() {
        // Check current mode
        if (isEditMode) {
            // === EDIT MODE: Adjust selected parameter value ===
            
            // Read button state with separate debouncing
            bool buttonPressed = readButtonDebouncedEdit();
            bool buttonJustPressed = (buttonPressed && !lastButtonStateEdit);
            lastButtonStateEdit = buttonPressed;
            
            // Get current potentiometer position from MenuSystem
            int potValue = menu->getPotValue();
            
            // Determine parameter type and configure sensitivity
            uint8_t selection = menu->getSelection();
            bool isScale = (selection == ITEM_SCALE);
            
            // Declare parameter range and sensitivity variables
            int movementThreshold;  // ADC units per increment
            int incrementSize;      // Value change per step
            int minValue, maxValue; // Parameter bounds
            
            // Potentiometer sensitivity constants
            const int POT_FULL_RANGE = 1024;      // 10-bit ADC range
            const int SCALE_POT_FACTOR = 7;       // Tuning factor for sensitivity
            const int SCALE_POT_INCREMENTS = 30;  // Target steps for scale (coarse)
            const int PID_POT_INCREMENTS = 100;   // Target steps for PID (fine)

            // Configure based on parameter type
            if (isScale) {
                // Scale parameter: coarse control (100 per step)
                incrementSize = 100;
                minValue = 100;
                maxValue = 3000;
                // Calculate threshold for ~30 total steps across pot range
                movementThreshold = max(5, (POT_FULL_RANGE * SCALE_POT_FACTOR) / SCALE_POT_INCREMENTS);
            } else {
                // PID parameters: fine control (1 per step)
                incrementSize = 1;
                minValue = 0;
                maxValue = 100;
                // Calculate threshold for ~100 total steps across pot range
                movementThreshold = max(5, (POT_FULL_RANGE * SCALE_POT_FACTOR) / PID_POT_INCREMENTS);
            }
            
            // Apply potentiometer changes
            if (currentValue) {
                // Calculate movement since last frame
                int potDelta = potValue - lastPotValue;
                
                // Handle upward pot movement (decreases value)
                while (potDelta > movementThreshold) {
                    // Check if value can decrease
                    if (*currentValue > minValue) {
                        // Apply decrement with bounds checking
                        int newValue = *currentValue - incrementSize;
                        if (newValue < minValue) newValue = minValue;
                        *currentValue = newValue;
                        
                        // Update display to show new value
                        updateMenuItems();
                        updateEditValue();
                    }
                    // Consume threshold amount from delta
                    potDelta -= movementThreshold;
                    lastPotValue += movementThreshold;
                }
                
                // Handle downward pot movement (increases value)
                while (potDelta < -movementThreshold) {
                    // Check if value can increase
                    if (*currentValue < maxValue) {
                        // Apply increment with bounds checking
                        int newValue = *currentValue + incrementSize;
                        if (newValue > maxValue) newValue = maxValue;
                        *currentValue = newValue;
                        
                        // Update display to show new value
                        updateMenuItems();
                        updateEditValue();
                    }
                    // Consume threshold amount from delta
                    potDelta += movementThreshold;
                    lastPotValue -= movementThreshold;
                }
            }
            
            // Check for button press (exits edit mode)
            if (buttonJustPressed) {
                // Return to navigation mode
                isEditMode = false;
                
                // Update menu items and trigger full redraw
                updateMenuItems();
                menu->requestRedraw();
            }
            
        } else {
            // === NAVIGATION MODE: Scroll through parameters ===
            
            // Update menu system (handles scrolling and input)
            menu->update();
            
            // Check for button press (selection)
            if (menu->isButtonJustPressed()) {
                uint8_t selection = menu->getSelection();
                
                // Handle action based on selected item
                if (selection == ITEM_SAVE) {
                    // Persist working settings to EEPROM
                    workingSettings.save();
                    return 1;  // Exit to parent menu
                    
                } else if (selection == ITEM_BACK) {
                    // Discard changes (don't save working copy)
                    return 1;  // Exit to parent menu
                    
                } else {
                    // Enter edit mode for selected parameter
                    currentValue = getValuePointer(selection);
                    
                    // Verify parameter is editable
                    if (currentValue) {
                        isEditMode = true;
                        initialValue = *currentValue;  // Store initial value (for potential restore)
                        lastPotValue = menu->getPotValue();  // Initialize for delta calculation
                        drawEditOverlay();  // Show edit mode display
                    }
                }
            }
        }
        
        return 0;  // Continue in PID editor
    }
    
    /**
     * Requests full menu redraw on next update
     * 
     * Call when returning to PID editor from parent menu or after
     * external display modifications. Updates menu items with current
     * values before requesting redraw.
     */
    void requestRedraw() {
        updateMenuItems();
        menu->requestRedraw();
    }
};

// === Static Member Definitions ===

/**
 * Short parameter labels for display
 * Used in edit mode overlay to show which parameter is being adjusted
 */
const char* const PIDEdit::labels[] = {
    "KP",     // Proportional gain label
    "KI",     // Integral gain label
    "KD",     // Derivative gain label
    "Scale",  // PID scale label
    "SAVE",   // Save action label
    "BACK"    // Back action label
};

#endif // PID_EDIT_H
