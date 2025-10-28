#ifndef DISPLAY_H
#define DISPLAY_H

#include <U8g2lib.h>
#include "hal.h"

/**
 * Display Module
 * Handles OLED display operations
 * Using page buffer mode to save RAM (uses ~128 bytes instead of ~1024 bytes)
 */
class Display {
private:
    // Changed from _F_ (full buffer) to _1_ (page buffer) to save RAM
    static U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2;
    
public:
    static void begin() {
        u8g2.begin();
        u8g2.setFont(u8g2_font_6x10_tr);
    }
    
    static void showText(const char* text, uint8_t x = 0, uint8_t y = 10) {
        u8g2.firstPage();
        do {
            u8g2.setCursor(x, y);
            u8g2.print(text);
        } while (u8g2.nextPage());
    }
    
    static void showStatus(const char* mode, int16_t position, float pidValue) {
        u8g2.firstPage();
        do {
            u8g2.setCursor(0, 10);
            u8g2.print("Mode: ");
            u8g2.print(mode);
            
            u8g2.setCursor(0, 25);
            u8g2.print("Pos: ");
            u8g2.print(position);
            
            u8g2.setCursor(0, 40);
            u8g2.print("PID: ");
            u8g2.print(pidValue, 1);
        } while (u8g2.nextPage());
    }
    
    static void showCalibration(uint8_t progress) {
        u8g2.firstPage();
        do {
            u8g2.setCursor(0, 10);
            u8g2.print("Calibrating...");
            
            // Progress bar
            u8g2.drawFrame(0, 20, 128, 10);
            u8g2.drawBox(2, 22, progress * 124 / 100, 6);
            
            u8g2.setCursor(0, 45);
            u8g2.print(progress);
            u8g2.print("%");
        } while (u8g2.nextPage());
    }
    
    static void showSensorValues(uint16_t* values, uint8_t count) {
        u8g2.firstPage();
        do {
            u8g2.setCursor(0, 10);
            u8g2.print("Sensors:");
            
            for (uint8_t i = 0; i < count && i < 8; i++) {
                u8g2.setCursor(i * 16, 25);
                u8g2.print(values[i]);
            }
            
            // Visual bar graph
            for (uint8_t i = 0; i < count && i < 8; i++) {
                uint8_t barHeight = map(values[i], 0, 1000, 0, 30);
                u8g2.drawBox(i * 16, 64 - barHeight, 12, barHeight);
            }
        } while (u8g2.nextPage());
    }
};

// Initialize static member - using page buffer mode (_1_) instead of full buffer (_F_)
U8G2_SSD1306_128X64_NONAME_F_SW_I2C Display::u8g2(
    U8G2_R0,
    HAL::DisplayPins::SOFT_SCL,
    HAL::DisplayPins::SOFT_SDA,
    U8X8_PIN_NONE
);

#endif
