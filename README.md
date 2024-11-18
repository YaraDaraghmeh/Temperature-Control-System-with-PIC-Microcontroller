# Temperature Control System with PIC Microcontroller

## Project Overview
This is an advanced temperature control system implemented on a PIC microcontroller, featuring multiple operational modes and sophisticated temperature management capabilities.

## Features
- Multiple Operating Modes:
  - OFF Mode
  - COOL Mode
  - HEAT Mode
  - AUTO Mode (Automatic Heating/Cooling)

- Advanced Temperature Control:
  - Adjustable setpoint temperature
  - Outside temperature monitoring
  - Room temperature tracking
  - Percentage-based heating and cooling control

- User Interface:
  - LCD Display
  - Serial Communication Interface
  - Button-based Mode Selection
  - Interrupt-driven Controls

## Hardware Requirements
- PIC Microcontroller
- LCD Display
- Temperature Sensors (Analog Inputs)
- Heating Element
- Cooling Element
- Serial Communication Module

## Software Configuration
- Configured with multiple configuration pragmas
- Uses high-priority interrupts
- Supports real-time mode switching
- Implements PWM for precise temperature control

## Communication Protocol
- UART-based serial communication
- Commands:
  - 'S': System OFF
  - 'M': Status Report
  - 'm': Mode Information
  - 'c': Change Parameters

## Modes of Operation
1. **OFF Mode**: 
   - All heating/cooling elements disabled
   - Minimal system activity

2. **COOL Mode**:
   - Enables cooling system
   - Adjustable cooling percentage (0-100%)
   - Displays outside temperature

3. **HEAT Mode**:
   - Enables heating system
   - Adjustable heating percentage (0-100%)
   - Displays outside temperature

4. **AUTO Mode**:
   - Automatic switching between heating and cooling
   - Based on outside temperature and setpoint
   - Hysteresis control with adjustable range

## Key Functions
- `RX_isr()`: Serial communication handler
- `autoModeControl()`: Automatic mode temperature management
- `CoolMode()`: Cooling system control
- `heatMode()`: Heating system control
- `printingOnScreen()`: LCD display update

## Development Environment
- Microcontroller: PIC18 Series
- Compiler: XC8
- Development Tools: MPLAB X IDE

## Installation and Setup
1. Clone the repository
2. Open project in MPLAB X
3. Configure hardware connections
4. Compile and upload to microcontroller

## Customization
- Modify temperature thresholds in `#define` statements
- Adjust PWM and interrupt configurations as needed

## License


## Contributors
[Yara Daraghmeh]
