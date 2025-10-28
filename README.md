# Line Follower Robot v2

A modular, PID-controlled line following robot for Arduino.

## Features

- **Modular Architecture**: Clean separation of concerns with dedicated modules
- **PID Control**: Smooth and accurate line following
- **Auto Calibration**: Automatic sensor calibration with visual feedback
- **OLED Display**: Real-time status monitoring
- **Easy Configuration**: Centralized configuration file for tuning

## Hardware Requirements

- Arduino Nano/Uno
- TB6612FNG Dual Motor Driver
- 8-channel IR Line Sensor Array
- SSD1306 OLED Display (128x64, I2C)
- 2x DC Motors
- Push Button
- Potentiometer (optional, for runtime tuning)

## Project Structure

```
line_follower_v2/
├── line_follower_v2.ino    # Main program with state machine
├── hal.h                    # Hardware Abstraction Layer
├── config.h                 # Configuration parameters
├── motor_control.h          # Motor control module
├── sensor.h                 # Line sensor module
├── pid_controller.h         # PID controller implementation
├── display.h                # OLED display module
└── README.md               # This file
```

## Module Overview

### `hal.h` - Hardware Abstraction Layer
Centralizes all pin definitions and low-level hardware operations. Decouples hardware from application logic.

### `config.h` - Configuration
All tunable parameters in one place:
- PID gains (Kp, Ki, Kd)
- Motor speeds
- Timing parameters
- System states

### `motor_control.h` - Motor Control
High-level motor control interface:
- Individual motor control
- Differential drive
- Movement primitives (forward, backward, turn)

### `sensor.h` - Line Sensor
Line sensor array management:
- Raw sensor reading
- Calibration
- Line position calculation
- Weighted average algorithm

### `pid_controller.h` - PID Controller
Generic PID controller implementation:
- Proportional-Integral-Derivative control
- Anti-windup protection
- Configurable gains and limits

### `display.h` - Display Module
OLED display management:
- Status display
- Calibration progress
- Sensor visualization
- Menu system

## Operation

### Startup
1. Power on the robot
2. Display shows "Ready! Press to Start"

### Calibration
1. Press button to start calibration
2. Robot oscillates left/right for 5 seconds
3. Place robot on the line during this time
4. Calibration completes automatically

### Running
1. After calibration, robot starts following the line
2. Display shows:
   - Current mode
   - Line position
   - PID output
3. Press button to stop

### States
- **IDLE**: Robot stopped, waiting for button press
- **CALIBRATING**: Auto-calibration in progress
- **RUNNING**: Following the line
- **DEBUG**: Sensor visualization mode

## Tuning Guide

### Basic Tuning (in `config.h`)

1. **Base Speed**: Start with 100-150, increase for faster following
2. **PID_KP**: Start with 0.15, increase if turns are too gentle
3. **PID_KD**: Start with 0.5, increase to reduce oscillation
4. **PID_KI**: Usually keep at 0.0, only add if steady-state error exists

### Advanced Tuning

**If robot oscillates:**
- Decrease `PID_KP`
- Increase `PID_KD`
- Reduce `BASE_SPEED`

**If robot loses line on turns:**
- Increase `PID_KP`
- Increase `BASE_SPEED`
- Check sensor calibration

**If robot is sluggish:**
- Increase `BASE_SPEED`
- Increase `PID_KP`
- Decrease `PID_KD`

## Pin Connections

### Motor Driver (TB6612FNG)
- STBY: Pin 7
- Left Motor: IN1=8, IN2=12, PWM=6
- Right Motor: IN1=13, IN2=4, PWM=10

### Line Sensors
- 8 sensors on A0-A7 (analog pins)

### Display (I2C OLED)
- SDA: Pin 2 (software I2C)
- SCL: Pin 3 (software I2C)

### User Interface
- Button: Pin 5 (with internal pullup)
- Potentiometer: A7
- Pot Enable: Pin 9

## Serial Debugging

Connect to serial monitor at 115200 baud to see:
- Initialization messages
- Mode changes
- Line position and PID output (when running)
- Error messages

## Extending the Project

### Add a new feature:
1. Create a new `.h` file for your module
2. Include it in `line_follower_v2.ino`
3. Initialize in `setup()`
4. Use in appropriate mode

### Modify behavior:
1. Check `config.h` first for parameters
2. Modify mode functions in main `.ino` file
3. Test incrementally

## License

Open source - feel free to modify and share!

## Version History

- v2.0: Complete modular rewrite with PID control
- v1.0: Initial basic implementation
