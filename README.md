# Front Gate Control System

AVR-based automated gate control system with safety features and keypad access.

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
  - Arduino AVR board (e.g., Arduino Mega)
  - Dual gate actuators
  - Limit switches for position detection
  - Single-button keypad input
  - WiFi connectivity for monitoring (using WiFiNINA)

## Installation

1. Install required Arduino libraries:
   - WiFiNINA

2. Configure WiFi settings:
   - Copy `config.h.template` to `config.h`
   - Update WIFI_SSID and WIFI_PASSWORD in `config.h`

3. Configure pin assignments in `front-fence-gpt.ino`
4. Upload code to Arduino
5. Monitor Serial output for connection status

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

To run the sketch in test mode:

1. Create a file `platform.local.txt` in your sketch folder with:
   ```
   compiler.cpp.extra_flags=-DARDUINO_UNIT_MAIN
   ```

2. Upload the sketch to your Arduino board

3. Open Serial Monitor to view test results:
   - Hardware initialization tests
   - WiFi connection tests
   - Keypad access control tests
   - Gate movement sequence tests
   - Safety feature verification
   - Position bounds checking
   - Limit switch response tests

The test suite uses mock hardware functions to safely verify all functionality without requiring actual motor movement.

## License

MIT License - See LICENSE file for details

Last Updated: March 10, 2025
