/**
 * Menu System - Handles UI scrolling, selection, and input
 * Provides a clean interface for menu navigation with potentiometer and button
 */

#ifndef MENU_SYSTEM_H
#define MENU_SYSTEM_H

#include <Arduino.h>
#include <U8x8lib.h>

class MenuSystem {
private:
  U8X8_SSD1306_128X64_NONAME_SW_I2C* display;
  
  // Input pins
  uint8_t potPin;
  uint8_t buttonPin;
  uint8_t potEnablePin;
  
  // Menu data
  const char** menuItems;
  uint8_t itemCount;
  uint8_t visibleItems;  // Number of items visible at once (3 for scrolling menus)
  const char* title;     // Menu title
  
  // State variables
  uint8_t currentSelection;
  uint8_t lastSelection;
  uint8_t scrollOffset;
  uint8_t lastScrollOffset;
  int lastPotValue;
  bool lastButtonState;
  bool buttonJustPressed;
  bool needsRedraw;
  
  // Debounce timing
  unsigned long lastDebounceTime;
  bool lastButtonReading;
  bool buttonState;
  
  /**
   * Read potentiometer with 80% deadzone applied
   * Returns: 0-1023 mapped value
   */
  int readPotClamped() {
    digitalWrite(potEnablePin, HIGH);
    delayMicroseconds(100);
    int raw = analogRead(potPin);
    
    // Use middle 80% of the range (102 to 921 maps to 0 to 1023)
    const int DEADZONE = (1024 * 10) / 100;  // 10% on each side
    const int MIN_VAL = DEADZONE;            // 102
    const int MAX_VAL = 1024 - DEADZONE;     // 922
    
    // Clamp and remap
    if (raw < MIN_VAL) raw = MIN_VAL;
    if (raw > MAX_VAL) raw = MAX_VAL;
    
    return map(raw, MIN_VAL, MAX_VAL, 0, 1023);
  }
  
  /**
   * Read button with debouncing (50ms)
   * Returns: Current debounced button state
   */
  bool readButtonDebounced() {
    bool reading = (digitalRead(buttonPin) == HIGH);
    
    if (reading != lastButtonReading) {
      lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > 50) {  // 50ms debounce
      if (reading != buttonState) {
        buttonState = reading;
      }
    }
    
    lastButtonReading = reading;
    return buttonState;
  }
  
  /**
   * Calculate scroll offset to keep selection visible and centered when possible
   */
  void updateScrollOffset() {
    if (itemCount <= visibleItems) {
      // All items fit on screen, no scrolling needed
      scrollOffset = 0;
      return;
    }
    
    // Try to center the selection
    if (currentSelection == 0) {
      scrollOffset = 0;  // At top
    } else if (currentSelection >= itemCount - 1) {
      scrollOffset = itemCount - visibleItems;  // At bottom
    } else {
      // Center the selection (put it at position 1 of 3 visible items)
      scrollOffset = currentSelection - 1;
      
      // Clamp to valid range
      if (scrollOffset > itemCount - visibleItems) {
        scrollOffset = itemCount - visibleItems;
      }
    }
  }
  
  /**
   * Draw full menu (initial or forced redraw)
   */
  void drawFullMenu() {
    display->clear();
    display->setFont(u8x8_font_8x13_1x2_f);
    
    // Title
    display->setCursor(0, 0);
    display->print(title);
    
    // Menu items at rows 2, 4, 6
    uint8_t itemsToShow = min(visibleItems, itemCount);
    for (uint8_t i = 0; i < itemsToShow; i++) {
      uint8_t itemIndex = scrollOffset + i;
      
      if (itemIndex < itemCount) {
        display->setCursor(0, 2 + (i * 2));  // Rows: 2, 4, 6
        
        // Show selection indicator
        if (itemIndex == currentSelection) {
          display->print(">");
        } else {
          display->print(" ");
        }
        display->print(menuItems[itemIndex]);
      }
    }
    
    needsRedraw = false;
    lastScrollOffset = scrollOffset;
  }
  
  /**
   * Update only the selection indicators (optimized partial redraw)
   */
  void updateSelectionIndicators(uint8_t oldSelection, uint8_t newSelection) {
    display->setFont(u8x8_font_8x13_1x2_f);
    
    // Update old selection (remove indicator) - only if visible
    if (oldSelection >= scrollOffset && oldSelection < scrollOffset + visibleItems) {
      uint8_t oldRow = 2 + ((oldSelection - scrollOffset) * 2);
      display->setCursor(0, oldRow);
      display->drawString(0, oldRow, " ");  // Single char update
    }
    
    // Update new selection (add indicator) - only if visible
    if (newSelection >= scrollOffset && newSelection < scrollOffset + visibleItems) {
      uint8_t newRow = 2 + ((newSelection - scrollOffset) * 2);
      display->drawString(0, newRow, ">");  // Single char update
    }
  }
  
  /**
   * Update menu items when scrolling (partial redraw without clearing screen)
   * Draws center item first, then outer items based on scroll direction
   */
  void updateMenuItemsOnScroll() {
    display->setFont(u8x8_font_8x13_1x2_f);
    
    uint8_t itemsToShow = min(visibleItems, itemCount);
    
    // Helper lambda to draw a menu item at a given visible position
    auto drawMenuItem = [&](uint8_t visiblePos) {
      uint8_t itemIndex = scrollOffset + visiblePos;
      
      if (itemIndex < itemCount) {
        uint8_t row = 2 + (visiblePos * 2);  // Rows: 2, 4, 6
        
        // Build the entire line in a buffer first
        char lineBuffer[17];  // 16 chars + null terminator
        memset(lineBuffer, ' ', 16);
        lineBuffer[16] = '\0';
        
        // Add selection indicator
        lineBuffer[0] = (itemIndex == currentSelection) ? '>' : ' ';
        
        // Add menu text
        const char* itemText = menuItems[itemIndex];
        uint8_t textLen = strlen(itemText);
        if (textLen > 15) textLen = 15;  // Max 15 chars (1 reserved for indicator)
        memcpy(lineBuffer + 1, itemText, textLen);
        
        // Draw entire line at once using drawString (faster than print)
        display->drawString(0, row, lineBuffer);
      }
    };
    
    // Determine scroll direction based on selection position
    bool scrollingDown = (currentSelection > lastSelection);
    
    // Draw center item first (position 1 of 0,1,2 for 3 visible items)
    if (itemsToShow >= 2) {
      drawMenuItem(1);  // Middle item
    }
    
    // Draw outer items based on scroll direction
    if (scrollingDown) {
      // Scrolling down: draw bottom first, then top
      if (itemsToShow >= 3) {
        drawMenuItem(2);  // Bottom item
      }
      drawMenuItem(0);    // Top item
    } else {
      // Scrolling up: draw top first, then bottom
      drawMenuItem(0);    // Top item
      if (itemsToShow >= 3) {
        drawMenuItem(2);  // Bottom item
      }
    }
  }

public:
  /**
   * Constructor
   * @param u8x8Display Pointer to U8x8 display object
   * @param items Array of menu item strings
   * @param count Number of menu items
   * @param menuTitle Menu title string
   * @param visible Number of items to show at once (default 3)
   * @param pot Potentiometer analog pin
   * @param button Button digital pin
   * @param potEnable Potentiometer enable pin
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
      lastSelection(255),
      scrollOffset(0),
      lastScrollOffset(255),
      lastPotValue(-1),
      lastButtonState(false),
      buttonJustPressed(false),
      needsRedraw(true),
      lastDebounceTime(0),
      lastButtonReading(false),
      buttonState(false) {
  }
  
  /**
   * Initialize the menu system
   * Sets up pins and draws initial menu
   */
  void begin() {
    pinMode(potPin, INPUT);
    pinMode(buttonPin, INPUT);
    pinMode(potEnablePin, OUTPUT);
    digitalWrite(potEnablePin, HIGH);
    
    
    // Draw initial menu
    drawFullMenu();
  }
  
  /**
   * Update menu state based on inputs
   * Call this in loop() to handle input and update display
   */
  void update() {
    // Check if full redraw is needed
    if (needsRedraw) {
      updateScrollOffset();
      drawFullMenu();
      lastSelection = currentSelection;
      return;
    }
    
    // Map potentiometer directly to menu selection
    int potValue = readPotClamped();
    
    // Map pot range (0-1023) to menu items (0 to itemCount-1)
    // Invert so that high pot value = first item (index 0)
    uint8_t newSelection = map(potValue, 0, 1023, itemCount - 1, 0);
    newSelection = constrain(newSelection, 0, itemCount - 1);
    
    // Update display if selection changed
    if (newSelection != currentSelection) {
      uint8_t oldSelection = currentSelection;
      uint8_t oldScrollOffset = scrollOffset;
      
      currentSelection = newSelection;
      updateScrollOffset();
      
      // Check if we need scroll update or just indicator update
      if (scrollOffset != oldScrollOffset) {
        // Scroll position changed - redraw menu items without clearing screen
        updateMenuItemsOnScroll();
      } else {
        // Same scroll position - just update selection indicator
        updateSelectionIndicators(oldSelection, currentSelection);
      }
      
      lastSelection = currentSelection;
    }
    
    // Update button state
    buttonJustPressed = readButtonDebounced();
  }
  
  /**
   * Check if selection has changed since last check
   * Returns: true if selection changed
   */
  bool hasNewSelection() {
    return currentSelection != lastSelection;
  }
  
  /**
   * Get current selected index
   * Returns: Index of currently selected menu item
   */
  uint8_t getSelection() const {
    return currentSelection;
  }
  
  /**
   * Get current selected menu item text
   * Returns: Pointer to menu item string
   */
  const char* getSelectedItem() const {
    return menuItems[currentSelection];
  }
  
  /**
   * Check if button was just pressed (rising edge)
   * Returns: true if button was just pressed
   */
  bool isButtonJustPressed() {
    return buttonJustPressed;
  }
  
  /**
   * Force a full redraw on next update
   */
  void requestRedraw() {
    needsRedraw = true;
  }
  
  /**
   * Get current potentiometer reading (for debugging)
   * Returns: 0-1023 value
   */
  int getPotValue() {
    return readPotClamped();
  }
};

#endif // MENU_SYSTEM_H
