/**
 * Menu System Module - Header
 * 
 * Provides a complete scrollable menu interface for OLED displays.
 * Features potentiometer-based navigation with circular selection,
 * smooth scrolling, optimized partial screen updates, and debounced
 * button input.
 * 
 * Key Features:
 * - Potentiometer navigation with relative movement (no absolute positioning)
 * - Circular selection (wraps around at ends)
 * - Dynamic scroll centering (selected item stays in middle when possible)
 * - Smart partial redraws (updates only changed elements)
 * - Hardware button debouncing (50ms)
 * - Rising edge detection for button presses
 * - Configurable visible items (default 3)
 * - Adaptive movement threshold based on menu size
 * 
 * Hardware Requirements:
 * - SSD1306 OLED display (128x64)
 * - Potentiometer with enable pin (power saving)
 * - Push button (active HIGH)
 * 
 * Usage Pattern:
 * 1. Create MenuSystem with menu items array
 * 2. Call begin() to initialize
 * 3. Call update() in main loop
 * 4. Check isButtonJustPressed() for selection confirmation
 * 5. Get selected item with getSelection() or getSelectedItem()
 * 6. Call requestRedraw() when switching between menus
 */

#ifndef MENU_SYSTEM_H
#define MENU_SYSTEM_H

#include <Arduino.h>
#include <U8x8lib.h>

/**
 * MenuSystem Class
 * 
 * Manages interactive menu display with scrolling and input handling.
 * Optimized for responsiveness with minimal display updates.
 */
class MenuSystem {
private:
  // === Display Hardware ===
  U8X8_SSD1306_128X64_NONAME_SW_I2C* display;  ///< OLED display driver
  
  // === Input Hardware Configuration ===
  uint8_t potPin;         ///< Analog input pin for potentiometer
  uint8_t buttonPin;      ///< Digital input pin for button
  uint8_t potEnablePin;   ///< Digital output pin to power potentiometer (saves power)
  
  // === Menu Content ===
  const char** menuItems;  ///< Array of menu item strings
  uint8_t itemCount;       ///< Total number of menu items
  uint8_t visibleItems;    ///< Number of items displayed at once (typically 3)
  const char* title;       ///< Menu title displayed at top
  
  // === Selection State ===
  uint8_t currentSelection;   ///< Currently selected item index (0-based)
  uint8_t lastSelection;      ///< Previous selection for change detection (255 = invalid)
  uint8_t scrollOffset;       ///< Index of topmost visible item
  uint8_t lastScrollOffset;   ///< Previous scroll position (255 = invalid)
  int lastPotValue;           ///< Previous potentiometer reading for delta calculation
  
  // === Shared State Flags (static across all MenuSystem instances) ===
  static bool buttonJustPressed;  ///< Rising edge detected this frame
  static bool potInitialized;     ///< Potentiometer calibrated for relative movement
  
  // === Display Update Flags ===
  bool needsRedraw;  ///< Full screen redraw required on next update
  
  // === Button Debounce State (static - shared for hardware stability) ===
  static unsigned long lastDebounceTime;  ///< Timestamp of last button state change
  static bool lastButtonReading;          ///< Raw button reading from previous frame
  static bool buttonState;                ///< Debounced button state (stable)
  static bool lastButtonState;            ///< Previous stable state for edge detection
  
  /**
   * Reads potentiometer value with hardware power control
   * 
   * Enables potentiometer power briefly to take reading, saving power
   * when not actively being read. Adds small delay for voltage stabilization.
   * 
   * @return Raw ADC value (0-1023)
   */
  int readPotClamped() {
    // Enable potentiometer power
    digitalWrite(potEnablePin, HIGH);
    delayMicroseconds(100);  // Wait for voltage to stabilize
    
    // Read analog value
    int raw = analogRead(potPin);
    return raw;
  }
  
  /**
   * Reads button state with debouncing algorithm
   * 
   * Implements time-based debouncing to filter mechanical switch noise.
   * Button state only changes if it remains stable for 50ms, preventing
   * false triggers from contact bounce.
   * 
   * Algorithm:
   * 1. Read raw button state
   * 2. If state changed, reset debounce timer
   * 3. If stable for 50ms, accept new state
   * 
   * @return Debounced button state (true = pressed, false = released)
   */
  bool readButtonDebounced() {
    // Read current raw button state
    bool reading = (digitalRead(buttonPin) == HIGH);
    
    // If reading changed, reset debounce timer
    if (reading != lastButtonReading) {
      lastDebounceTime = millis();
    }
    
    // Check if reading has been stable for debounce period
    if ((millis() - lastDebounceTime) > 50) {  // 50ms debounce threshold
      // Reading stable - update button state if it changed
      if (reading != buttonState) {
        buttonState = reading;
      }
    }
    
    // Store reading for next iteration
    lastButtonReading = reading;
    return buttonState;
  }
  
  /**
   * Calculates scroll offset to keep selected item visible and centered
   * 
   * Implements smart scrolling logic:
   * - No scrolling if all items fit on screen
   * - Selected item at top when selecting first item
   * - Selected item at bottom when selecting last item
   * - Selected item centered (position 1 of 3) for middle items
   * 
   * This creates a natural scrolling experience where the selected
   * item stays in view and centered when possible.
   */
  void updateScrollOffset() {
    // Check if scrolling is needed
    if (itemCount <= visibleItems) {
      // All items fit on screen - no scrolling
      scrollOffset = 0;
      return;
    }
    
    // Calculate scroll position based on selection
    if (currentSelection == 0) {
      // At top - show from beginning
      scrollOffset = 0;
    } else if (currentSelection >= itemCount - 1) {
      // At bottom - align last item with bottom of screen
      scrollOffset = itemCount - visibleItems;
    } else {
      // Middle items - center selection (position 1 of 3 visible)
      scrollOffset = currentSelection - 1;
      
      // Clamp to valid range (prevent showing past end)
      if (scrollOffset > itemCount - visibleItems) {
        scrollOffset = itemCount - visibleItems;
      }
    }
  }
  
  /**
   * Renders complete menu to display
   * 
   * Performs full screen clear and redraw. Used for initial display
   * and when major layout changes occur (e.g., menu switching).
   * 
   * Layout:
   * - Row 0: Menu title
   * - Rows 2, 4, 6: Menu items (3 visible items)
   * - Selection indicator: ">" prefix on selected item
   */
  void drawFullMenu() {
    // Clear screen for fresh display
    display->clear();
    display->setFont(u8x8_font_8x13_1x2_f);
    
    // Draw title at top
    display->setCursor(0, 0);
    display->print(title);
    
    // Draw visible menu items (rows 2, 4, 6)
    uint8_t itemsToShow = min(visibleItems, itemCount);
    for (uint8_t i = 0; i < itemsToShow; i++) {
      uint8_t itemIndex = scrollOffset + i;
      
      // Check if item exists
      if (itemIndex < itemCount) {
        display->setCursor(0, 2 + (i * 2));  // Rows: 2, 4, 6 (double-height font)
        
        // Draw selection indicator
        if (itemIndex == currentSelection) {
          display->print(">");  // Selected
        } else {
          display->print(" ");  // Not selected
        }
        display->print(menuItems[itemIndex]);
      }
    }
    
    // Update state tracking
    needsRedraw = false;
    lastScrollOffset = scrollOffset;
  }
  
  /**
   * Updates only selection indicators (optimized partial redraw)
   * 
   * When selection changes without scrolling, only the ">" indicators
   * need to be updated. This is much faster than full redraw.
   * 
   * Process:
   * 1. Remove ">" from old selection (if visible)
   * 2. Add ">" to new selection (if visible)
   * 
   * @param oldSelection - Previous selected item index
   * @param newSelection - New selected item index
   */
  void updateSelectionIndicators(uint8_t oldSelection, uint8_t newSelection) {
    display->setFont(u8x8_font_8x13_1x2_f);
    
    // Update old selection - remove indicator (only if visible)
    if (oldSelection >= scrollOffset && oldSelection < scrollOffset + visibleItems) {
      uint8_t oldRow = 2 + ((oldSelection - scrollOffset) * 2);
      display->setCursor(0, oldRow);
      display->drawString(0, oldRow, " ");  // Replace with space
    }
    
    // Update new selection - add indicator (only if visible)
    if (newSelection >= scrollOffset && newSelection < scrollOffset + visibleItems) {
      uint8_t newRow = 2 + ((newSelection - scrollOffset) * 2);
      display->drawString(0, newRow, ">");  // Draw selection arrow
    }
  }
  
  /**
   * Updates menu items during scrolling (optimized partial redraw)
   * 
   * When scroll position changes, all visible items need updating but
   * screen clearing causes flicker. This method redraws only the item
   * rows without clearing.
   * 
   * Optimization Strategy:
   * - Draw center item first (most important)
   * - Draw edge items based on scroll direction
   * - Build complete line buffers to minimize display calls
   * - Use drawString() instead of print() (faster)
   * 
   * This creates smooth scrolling without flicker.
   */
  void updateMenuItemsOnScroll() {
    display->setFont(u8x8_font_8x13_1x2_f);
    
    uint8_t itemsToShow = min(visibleItems, itemCount);
    
    // Helper lambda to draw complete menu item line
    auto drawMenuItem = [&](uint8_t visiblePos) {
      uint8_t itemIndex = scrollOffset + visiblePos;
      
      // Check if item exists
      if (itemIndex < itemCount) {
        uint8_t row = 2 + (visiblePos * 2);  // Calculate display row
        
        // Build complete line in buffer (faster than multiple display calls)
        char lineBuffer[17];  // 16 chars + null terminator
        memset(lineBuffer, ' ', 16);  // Fill with spaces
        lineBuffer[16] = '\0';
        
        // Add selection indicator
        lineBuffer[0] = (itemIndex == currentSelection) ? '>' : ' ';
        
        // Copy menu text
        const char* itemText = menuItems[itemIndex];
        uint8_t textLen = strlen(itemText);
        if (textLen > 15) textLen = 15;  // Limit to 15 chars (1 reserved for indicator)
        memcpy(lineBuffer + 1, itemText, textLen);
        
        // Draw complete line at once (single display call)
        display->drawString(0, row, lineBuffer);
      }
    };
    
    // Determine scroll direction
    bool scrollingDown = (currentSelection > lastSelection);
    
    // Draw center item first (most visible, highest priority)
    if (itemsToShow >= 2) {
      drawMenuItem(1);  // Middle position (index 1 of 0,1,2)
    }
    
    // Draw edge items based on scroll direction
    // (minimizes perceived update delay)
    if (scrollingDown) {
      // Scrolling down: bottom edge changes first
      if (itemsToShow >= 3) {
        drawMenuItem(2);  // Bottom position
      }
      drawMenuItem(0);    // Top position
    } else {
      // Scrolling up: top edge changes first
      drawMenuItem(0);    // Top position
      if (itemsToShow >= 3) {
        drawMenuItem(2);  // Bottom position
      }
    }
  }

public:
  /**
   * Constructor - Initializes menu system
   * 
   * Creates menu with specified configuration. Does not initialize hardware
   * or draw display - call begin() for that.
   * 
   * @param u8x8Display - Pointer to U8x8 display object (must be initialized)
   * @param items - Array of C-string pointers for menu items
   * @param count - Number of menu items in array
   * @param menuTitle - Title string displayed at top (default: "MENU")
   * @param visible - Number of items visible at once (default: 3)
   * @param pot - Analog pin for potentiometer (default: A7)
   * @param button - Digital pin for button input (default: 5)
   * @param potEnable - Digital pin to enable potentiometer power (default: 9)
   */
  MenuSystem(U8X8_SSD1306_128X64_NONAME_SW_I2C* u8x8Display,
             const char** items, 
             uint8_t count,
             const char* menuTitle = "MENU",
             uint8_t visible = 3,
             uint8_t pot = A7,
             uint8_t button = 5,
             uint8_t potEnable = 9)
    : display(u8x8Display),
      menuItems(items),
      itemCount(count),
      visibleItems(visible),
      title(menuTitle),
      potPin(pot),
      buttonPin(button),
      potEnablePin(potEnable),
      currentSelection(0),
      lastSelection(255),      // Invalid value = not initialized
      scrollOffset(0),
      lastScrollOffset(255),   // Invalid value = not initialized
      needsRedraw(true),
      lastPotValue(-1) {       // Invalid value = not initialized
  }
  
  /**
   * Initializes menu hardware and displays initial menu
   * 
   * Sets up GPIO pins, initializes button state to prevent false
   * triggers on startup, resets potentiometer for relative movement,
   * and draws the initial menu screen.
   * 
   * Must be called once during setup before using the menu.
   */
  void begin() {
    // Configure GPIO pins
    pinMode(potPin, INPUT);
    pinMode(buttonPin, INPUT);
    pinMode(potEnablePin, OUTPUT);
    digitalWrite(potEnablePin, HIGH);  // Enable pot power
    
    // Initialize button state (prevents false trigger on first read)
    lastButtonState = readButtonDebounced();
    buttonJustPressed = false;
    
    // Reset potentiometer calibration for relative movement mode
    potInitialized = false;
    
    // Draw initial menu display
    drawFullMenu();
  }
  
  /**
   * Updates menu state based on input (call in main loop)
   * 
   * Main update function - handles input reading, selection changes,
   * scroll calculations, and display updates. Uses intelligent partial
   * redraws to minimize flicker.
   * 
   * Update Flow:
   * 1. Check if full redraw requested → draw and return
   * 2. Read potentiometer value
   * 3. Initialize pot on first read (relative mode)
   * 4. Calculate movement delta
   * 5. Update selection if threshold exceeded
   * 6. Calculate new scroll position
   * 7. Partial display update (indicators only or scroll update)
   * 8. Read button for edge detection
   * 
   * Movement Algorithm:
   * - Dynamic threshold based on menu size (allows full range traversal)
   * - Loop handles fast pot movements (multiple steps per frame)
   * - Circular selection (wraps at ends)
   * - Relative movement (no absolute positioning jitter)
   */
  void update() {
    // Handle full redraw request
    if (needsRedraw) {
      updateScrollOffset();
      drawFullMenu();
      lastSelection = currentSelection;
      // Re-initialize button state to prevent false triggers after redraw
      lastButtonState = readButtonDebounced();
      buttonJustPressed = false;
      return;
    }
    
    // Read current potentiometer position
    int potValue = readPotClamped();
    
    // Initialize pot value on first read (enables relative movement)
    if (!potInitialized) {
      lastPotValue = potValue;
      potInitialized = true;
    }
    
    // Calculate movement since last frame
    int potDelta = potValue - lastPotValue;
    
    // Calculate dynamic threshold based on menu size
    // Formula: (1024 ADC range * 70%) / (number of items)
    // This ensures entire menu is reachable across full pot range
    const int MOVEMENT_THRESHOLD = max(5, (1024 * 7) / (itemCount * 10));
    
    DEBUG_PRINTLN("MOVEMENT_THRESHOLD: " + String(MOVEMENT_THRESHOLD) + ", Pot Delta: " + String(potDelta) + ", Last Pot: " + String(lastPotValue));
    
    // Track new selection (may change multiple times for fast movements)
    uint8_t newSelection = currentSelection;
    
    // Handle upward pot movement (decreases selection index)
    while (potDelta > MOVEMENT_THRESHOLD) {
      // Move selection up
      if (newSelection > 0) {
        newSelection--;
      } else {
        // Wrap to bottom (circular selection)
        newSelection = itemCount - 1;
      }
      // Consume threshold amount from delta
      potDelta -= MOVEMENT_THRESHOLD;
      lastPotValue += MOVEMENT_THRESHOLD;
    }
    
    // Handle downward pot movement (increases selection index)
    while (potDelta < -MOVEMENT_THRESHOLD) {
      // Move selection down
      if (newSelection < itemCount - 1) {
        newSelection++;
      } else {
        // Wrap to top (circular selection)
        newSelection = 0;
      }
      // Consume threshold amount from delta
      potDelta += MOVEMENT_THRESHOLD;
      lastPotValue -= MOVEMENT_THRESHOLD;
    }
    
    // Apply selection change if different
    if (newSelection != currentSelection) {
      uint8_t oldSelection = currentSelection;
      uint8_t oldScrollOffset = scrollOffset;
      
      // Update selection and calculate new scroll position
      currentSelection = newSelection;
      updateScrollOffset();
      
      // Determine update type based on scroll change
      if (scrollOffset != oldScrollOffset) {
        // Scroll position changed - update all visible items
        updateMenuItemsOnScroll();
      } else {
        // Same scroll - just update selection indicators
        updateSelectionIndicators(oldSelection, currentSelection);
      }
      
      lastSelection = currentSelection;
    }
    
    // Update button state with edge detection
    bool currentButtonState = readButtonDebounced();
    buttonJustPressed = (currentButtonState && !lastButtonState);  // Rising edge
    lastButtonState = currentButtonState;
  }
  
  /**
   * Checks if selection changed since last check
   * 
   * Useful for detecting when user changed selection without pressing button.
   * Note: State is not automatically cleared - compare with getSelection().
   * 
   * @return true if selection different from last recorded position
   */
  bool hasNewSelection() {
    return currentSelection != lastSelection;
  }
  
  /**
   * Gets currently selected item index
   * 
   * @return Zero-based index of selected item (0 to itemCount-1)
   */
  uint8_t getSelection() const {
    return currentSelection;
  }
  
  /**
   * Gets text of currently selected menu item
   * 
   * @return Pointer to menu item string (from original items array)
   */
  const char* getSelectedItem() const {
    return menuItems[currentSelection];
  }
  
  /**
   * Checks if button was just pressed (rising edge detection)
   * 
   * Returns true only on the frame where button transitions from
   * released to pressed. Automatically cleared on next update().
   * Use this to detect button press events, not button state.
   * 
   * @return true if button was just pressed this frame
   */
  bool isButtonJustPressed() {
    return buttonJustPressed;
  }
  
  /**
   * Requests full menu redraw on next update()
   * 
   * Use when switching between menus, changing menu content, or
   * after external display modifications. Resets button state to
   * prevent false triggers and reinitializes potentiometer for
   * relative movement.
   * 
   * @param resetSelection - If true, resets selection to first item.
   *                         Default false to maintain user position.
   *                         Use with caution as it may be unexpected.
   */
  void requestRedraw(bool resetSelection = false) {
    needsRedraw = true;
    
    // Reset button state (prevents immediate re-trigger)
    buttonJustPressed = false;
    lastButtonState = readButtonDebounced();
    
    // Reset potentiometer for relative movement from new position
    potInitialized = false;
    
    // Optionally reset selection to top
    if (resetSelection) {
      currentSelection = 0;
      lastSelection = 0;
      scrollOffset = 0;
    }
  }
  
  /**
   * Gets current raw potentiometer reading
   * 
   * Useful for debugging or calibration. Briefly enables pot power.
   * 
   * @return ADC value (0-1023)
   */
  int getPotValue() {
    return readPotClamped();
  }
};

// === Static Member Definitions ===
// These must be defined outside the class (C++ requirement for static members)

bool MenuSystem::buttonJustPressed = false;     ///< Shared button press flag
bool MenuSystem::potInitialized = false;        ///< Shared pot calibration flag
unsigned long MenuSystem::lastDebounceTime = 0; ///< Shared debounce timer
bool MenuSystem::lastButtonReading = false;     ///< Shared raw button reading
bool MenuSystem::buttonState = false;           ///< Shared stable button state
bool MenuSystem::lastButtonState = false;       ///< Shared previous button state

#endif // MENU_SYSTEM_H
