# Line Follower Robot - UI Documentation

## Overview
The line follower robot uses a state-based UI system with a U8X8 OLED display (128x64 pixels, character-based mode). Navigation is controlled using a **potentiometer** (for scrolling/adjusting values) and a **button** (for selecting/confirming).


## Navigation Controls
- **Potentiometer**: Scroll through menu items or adjust values
- **Button**: Select menu item, confirm changes, or exit screens
- **Auto Display Off**: Optional feature to turn off display during run mode (saves power)

---

## Menu Structure

### 1. Main Menu
**Title**: "Menu"

**Menu Items**:
- **Cfg** - Enter Settings/Configuration menu
- **Calibrt** - Start sensor calibration process
- **Run** - Start the line following run

**Controls**:
- Potentiometer: Scroll through options
- Button: Select highlighted option

---

### 2. Settings Menu (Config)
**Title**: "Config"

**Menu Items**:
1. **PID** - Edit PID parameters
2. **Test** - Access motor and sensor test modes
3. **Thresh** - Select sensor threshold ratio
4. **Profil** - Select PID profile presets
5. **Mode** - Select PID control mode
6. **Sensor** - Select sensor mode (8-channel or 6-channel modified)
7. **DspAut** - Toggle display auto-off (ON/OFF)
8. **VwData** - View all configuration data and system info
9. **FacRst** - Factory reset options
10. **Back** - Return to main menu

**Display**: Shows 3 items at a time, scrolls when selection moves beyond visible range

**Controls**:
- Potentiometer: Scroll through options
- Button: Select option (some toggle immediately, others enter sub-menus)

---

## Sub-Menus and States

### 3. PID Edit Screen
**Title**: "PID"

**Editable Parameters**:
- **KP**: Proportional gain (0-10000)
- **KI**: Integral gain (0-10000)
- **KD**: Derivative gain (0-10000)
- **Scale**: PID scale factor (0-10000)
- **SAVE** - Save changes to EEPROM
- **BACK** - Return without saving

**Display**: Shows 2 parameters at a time centered around current selection

**Controls**:
- Potentiometer (navigation mode): Scroll through parameters
- Button (navigation mode): Enter edit mode for selected parameter
- Potentiometer (edit mode): Adjust value with fine control (20 steps per full rotation)
- Button (edit mode): Confirm value and return to navigation mode

**Indicators**:
- `*` - Selected parameter (navigation mode)
- `>` - Editing this parameter (edit mode)

---

### 4. PID Profiles
**Title**: "Profiles"

**Profile Options**:
1. **Aggrsiv** - Fast & Responsive
   - KP: 150, KI: 0, KD: 50, Scale: 100
   - Description: "Fast & Resp"

2. **Balance** - Default Settings
   - KP: 100, KI: 0, KD: 30, Scale: 100
   - Description: "Default Set"

3. **Smooth** - Slow & Stable
   - KP: 60, KI: 0, KD: 80, Scale: 100
   - Description: "Slow & Stable"

4. **Custom** - User-Defined
   - Loads current EEPROM values
   - Description: "From EEPROM"

**Display**: Shows selected profile name and description

**Controls**:
- Potentiometer: Scroll through profiles
- Button: Apply selected profile and save to EEPROM

**Footer**: "Btn:Apply"

---

### 5. PID Mode Select
**Title**: "Mode"

**Mode Options**:
1. **Normal** (Standard PID)
   - Description: "Std PID"
   - Standard PID control throughout

2. **OvridCtr** (Override on Center)
   - Description: "Rst on Ctr"
   - Resets PID when line is centered

**Display**: Shows selected mode name and description

**Controls**:
- Potentiometer: Toggle between modes
- Button: Apply selected mode and return to settings

**Footer**: "Btn:Apply"

---

### 6. Sensor Mode Select
**Title**: "Sensor"

**Mode Options**:
1. **Normal 8** - Standard 8-channel
   - Description: "A0-A7 direct"
   - Uses all 8 sensor channels (A0-A7)

2. **Mod 6CH** - Modified 6-channel
   - Description: "A1-A6+weight"
   - Uses center 6 channels (A1-A6) with weighted positioning

**Display**: Shows selected mode name and description

**Controls**:
- Potentiometer: Toggle between modes
- Button: Apply selected mode, save to EEPROM, and return to settings

**Footer**: "Btn:Apply"

---

### 7. Threshold Select
**Title**: "Thresh"

**Threshold Options**:
1. **15/16 (6%)** - Very sensitive, close to white
2. **7/8 (12%)** - Moderately sensitive
3. **3/4 (25%)** - Balanced sensitivity
4. **1/2 (50%)** - Low sensitivity, darker threshold

**Display**: Shows up to 3 options at a time with selection indicator (`>`)

**Controls**:
- Potentiometer: Scroll through threshold options
- Button: Apply selected threshold and return to settings

**Info**: Threshold determines the cutoff point between black and white readings based on calibrated min/max values.

---

### 8. Motor Tests
**Title**: "MotorTst"

**Test Modes**:
1. **BothFwd** - Both motors forward
2. **BothBak** - Both motors backward
3. **LeftOn** - Left motor only (forward)
4. **RightOn** - Right motor only (forward)
5. **RotateL** - Rotate left (left backward, right forward)
6. **RotateR** - Rotate right (left forward, right backward)

**Display**:
- Line 1: Current test mode name
- Line 2: Current motor speed value
- Footer: "Pot:Mod Btn:X"

**Controls**:
- Potentiometer: Switch between test modes (motors execute immediately)
- Button: Exit to settings menu

**Behavior**: Motors execute selected test in real-time. Display updates at 10 FPS (100ms intervals).

---

### 9. Sensor Tests
**Title**: "Sensor"

**Display**:
- Line 1: "Pos: [position_value]"
  - Position ranges from -3500 to +3500 (line position)
- Line 2: Empty
- Footer: "Prs exit"

**Controls**:
- Button: Exit to settings menu

**Behavior**: Real-time display of line sensor position, updates at 10 FPS (100ms intervals).

---

### 10. View Data Screen
**Title**: "VwData"

**Data Pages** (12 screens, scroll through with potentiometer):

**Page 0: PID - KP**
- KP: [value]
- Scale: [value]

**Page 1: PID - KI**
- KI: [value]
- Scale: [value]

**Page 2: PID - KD**
- KD: [value]
- BaseSpd: [value]

**Page 3: Motor Settings 1**
- BaseSpd: [value]
- Align: [value]

**Page 4: Motor Settings 2**
- Accel: [value]
- Decel: [value]

**Page 5: Sensor Info**
- Displays sensor readings or position

**Page 6: Threshold**
- Thresh: [ratio] (15/16, 7/8, 3/4, or 1/2)

**Page 7: Modes**
- Mode: [Norm/Ovrd] (PID mode)
- [Additional mode info]

**Page 8: Sensor Mode**
- Displays current sensor mode

**Page 9: Calibration Data**
- Shows sensor calibration status

**Page 10: System Info**
- Free RAM: [bytes] bytes
- Settings: Valid

**Page 11: Sensor Details**
- Sensors: [detailed info]

**Controls**:
- Potentiometer: Scroll through data pages (0-11)
- Button: Exit to settings menu

**Footer**: "Btn:Exit"

**Purpose**: View all configuration data, calibration status, and system diagnostics without editing.

---

### 11. Factory Reset Selection
**Title**: "Reset"

**Reset Options** (checkboxes):
- **[X]PID** / **[ ]PID** - Toggle PID settings reset
- **[X]Motr** / **[ ]Motr** - Toggle Motor settings reset
- **[X]Snsr** / **[ ]Snsr** - Toggle Sensor calibration reset
- **>>RESET** - Proceed to confirmation (if at least one selected)
- **Back** - Return to settings menu

**Display**: Shows 3 items at a time, scrolls when selection moves

**Controls**:
- Potentiometer: Scroll through options
- Button: Toggle checkbox (for settings) or confirm action (for RESET/BACK)

**Indicators**:
- `>` - Currently selected item
- `[X]` - Item checked for reset
- `[ ]` - Item not checked for reset

**Note**: At least one option must be checked to proceed to reset confirmation.

---

### 12. Factory Reset Confirmation
**Title**: "Reset: [items]"

**Confirmation Options**:
- **> No** - Cancel reset, return to settings
- **Yes** - Perform selected resets

**Display**:
- Title shows which items will be reset (e.g., "Reset: PID,Mot")
- Two options: No (default) and Yes

**Controls**:
- Potentiometer: Toggle between No/Yes
- Button: Confirm selection

**Reset Process** (if Yes is selected):
1. Shows "Reset.." title
2. Displays what's being reset:
   - "PID.." - Resetting PID settings
   - "Mot.." - Resetting motor settings
   - "Sns.." - Resetting sensor calibration
3. Performs reset operations:
   - PID: Restores default KP, KI, KD, scale, mode
   - Motor: Restores default speed, alignment, acceleration, deceleration
   - Sensor: Clears calibration data
4. Shows "Done" / "Reset!" confirmation
5. Delays 2 seconds
6. Returns to settings menu

---

### 13. Calibration Screen
**Title**: "Calibrt"

**Calibration Phases**:

**Phase 1: Waiting**
- Display: "Wait..."
- Brief pause before starting calibration

**Phase 2: Calibrating**
- Display: "Wait..."
- Robot performs sensor calibration sweep
- Blocking operation (user should move robot over black and white surfaces)

**Phase 3: Validating**
- Analyzes calibration data quality
- Calculates quality score (0-100)

**Phase 4: Results**
- Display format:
  - Line 1: "Done! [QUALITY]"
    - QUALITY: "GOOD" (75-100%), "POOR" (50-74%), or "FAIL" (<50%)
  - Line 2: "Q: [score]" (quality score percentage)
  - Line 3: "Btn:Retry"
  - Line 4: "Pot:Accept"

**Controls**:
- Button: Retry calibration (restart from phase 1)
- Potentiometer (>512): Accept calibration and return to main menu

**Quality Indicators**:
- **GOOD**: Quality ≥75% - Excellent calibration
- **POOR**: Quality 50-74% - Acceptable but could be better
- **FAIL**: Quality <50% or invalid - Should retry

**Note**: Calibration data is automatically saved to EEPROM when accepted.

---

### 14. Run Screen
**Title**: "DONE!" (only shown when finished)

**Run Phases**:

**Phase 1: Running**
- Display is OFF (if "DspAut" is enabled) to save power and reduce distractions
- Robot follows the line using PID control
- Collects statistics: min speed, max speed, average speed
- Statistics sampled every 100ms

**Finish Detection**:
- Triggered when 7 or 8 sensors detect black (finish line)
- Requires minimum 3 seconds runtime (to avoid false detection at start line)
- Motors brake when finish is detected
- Display turns ON to show results

**Phase 2: Finished (Results Screen)**
- **Line 1**: "T: [time]s" - Total run time in seconds
- **Line 2**: "L: [count]" - Line lost count (if tracked)
- **Line 3**: "Avg: [speed]" - Average speed during run
- **Line 4**: Speed range: "[min] - [max]"
- **Footer**: "Btn:Menu"

**Controls**:
- Button (anytime): Stop run and return to main menu

**Data Tracked**:
- **Run Time**: From start to finish or manual stop
- **Min Speed**: Lowest motor speed during run
- **Max Speed**: Highest motor speed during run
- **Average Speed**: Mean of all speed samples
- **Sample Count**: Number of speed measurements taken

**Note**: If display auto-off is disabled, display remains on during run but may show stale information since updates are paused for performance.

---

## UI State Flow Diagram

```
Main Menu (MAIN_MENU)
├─> Settings Menu (SETTINGS_MENU)
│   ├─> PID Edit (PID_EDIT)
│   ├─> Motor Tests (MOTOR_TESTS)
│   ├─> Sensor Tests (SENSOR_TESTS)
│   ├─> Threshold Select (THRESHOLD_SELECT)
│   ├─> PID Profiles (PID_PROFILES)
│   ├─> PID Mode Select (PID_MODE_SELECT)
│   ├─> Sensor Mode Select (SENSOR_MODE_SELECT)
│   ├─> View Data (VIEW_DATA)
│   ├─> Factory Reset Select (FACTORY_RESET_SELECT)
│   │   └─> Factory Reset Confirm (FACTORY_RESET_CONFIRM)
│   └─> Back to Main Menu
├─> Calibration (CALIBRATION)
│   └─> Back to Main Menu (after completion)
└─> Run (RUN)
    └─> Back to Main Menu (button press or finish)
```

---

## Common UI Patterns

### Menu Selection Indicator
- `*` - Selected item in navigation mode
- `>` - Selected item in list or edit mode
- Inverted (black background) - Selected item in some menus

### Value Display Format
- **Label:Value** - Standard format (e.g., "KP: 100")
- **Right-aligned values** - Some screens align values at x=80 pixels
- **Checkboxes** - `[X]` checked, `[ ]` unchecked

### Scrolling Behavior
- Most menus show 2-3 items at a time
- Selection stays centered when possible
- Scroll offset calculated based on current selection

### Display Update Strategy
- **On-demand updates**: Only redraws when `needsRedraw` flag is set
- **Throttled updates**: Test screens update at max 10 FPS (100ms intervals)
- **Blocking operations**: Calibration runs without display updates during sweep
- **Power saving**: Display can turn off during run mode

### Potentiometer Scaling
- **Menu navigation**: `getSteps(1000 / itemCount)` - proportional to menu size
- **Value editing**: `getSteps(20)` - fine control for precise adjustments
- **Tests**: `getSteps(1000 / testModeCount)` - quick mode switching



---

## Display Library: U8X8

The UI uses the U8X8 library in character mode:
- **Advantage**: Lower memory usage compared to U8G2 (no frame buffer)
- **Limitation**: Text-only display, no graphics or arbitrary positioning
- **Performance**: Faster updates, better for resource-constrained Arduino

---

## Future Enhancements (Potential)

- Add graph visualization for sensor readings
- Implement data logging to EEPROM
- Add more granular PID tuning options
- Create race timer with lap tracking
- Add diagnostic error messages
- Implement menu timeout to auto-return to main menu

---

## Developer Notes

### Adding New Menu Items
1. Add enum value to `ui_state.h`
2. Create new state class (header and implementation)
3. Register state in `UIManager` state array
4. Add menu item to parent state's update/draw methods
5. Update string tables in PROGMEM

### State Pattern Implementation
Each UI screen is implemented as a separate state class with:
- `enter()` - Initialize state when entered
- `update()` - Handle input and logic, return next UIMode
- `draw()` - Render screen contents (only called when needed)

### Display Optimization Tips
- Use `needsRedraw` flag to avoid unnecessary updates
- Throttle real-time displays to 10 FPS or slower
- Turn off display during CPU-intensive operations
- Clear only when necessary (full screen operations)
- Use character-based cursor positioning when possible

---

## Quick Reference: Button & Potentiometer Functions

| Context | Potentiometer | Button |
|---------|---------------|--------|
| **Menu Navigation** | Scroll items | Select item |
| **Value Editing** | Adjust value | Confirm value |
| **Test Modes** | Change mode | Exit tests |
| **Calibration** | Accept (>512) | Retry |
| **Run Mode** | N/A | Stop & exit |
| **Confirmation** | Toggle Yes/No | Confirm choice |
| **View Data** | Scroll pages | Exit to menu |

---

**Document Version**: 1.0  
**Last Updated**: October 28, 2025  
**Hardware**: Arduino-based Line Follower Robot  
**Display**: 128x64 OLED (U8X8 mode)
