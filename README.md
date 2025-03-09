# Front Gate Control System

ESP8266-based automated gate control system with safety features and keypad access.

## Features

- **Secure Access Control**
  - Single-button keypad input (1-second press required)
  - Automatic state toggling (open/close)
  - Authorization reset after each operation

- **Safety Features**
  - Safe position initialization after power loss
  - Position bounds checking and validation
  - Calibration status verification
  - Limit switch monitoring
  - Emergency stops for critical conditions
  - Physical inward stop protection

- **Hardware Requirements**
  - ESP8266 microcontroller
  - Dual gate actuators
  - Limit switches for position detection
  - Single-button keypad input
  - WiFi connectivity for monitoring

## Installation

1. Install required Arduino libraries:
   - ESP8266WiFi
   - WiFiManager
   - ArduinoUnit (for tests)

2. Configure pin assignments in `front-fence-gpt.ino`
3. Upload code to ESP8266
4. First boot will initialize WiFi configuration

## Operation

1. **Initial Setup**
   - System auto-calibrates on first run
   - Gates will safely find zero position
   - Calibration establishes maximum safe limits

2. **Normal Operation**
   - Press keypad for 1 second to activate
   - Gates automatically open/close based on current state
   - System prevents operation while gates in motion

3. **Safety Protocols**
   - Gates can safely close past 0 position (physical stop)
   - System prevents exceeding calibrated maximum limits
   - Position tracking with detailed logging
   - Automatic stop on limit switch activation

## Testing

Run `front-fence-gpt-tests.ino` to verify:
- Position initialization
- Keypad access control
- Gate movement sequences
- Safety feature operation
- Limit switch responses
- Position bounds checking

## License

MIT License - See LICENSE file for details

Last Updated: March 9, 2025
