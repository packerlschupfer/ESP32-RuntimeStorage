# RuntimeStorage PlatformIO Example

This example demonstrates how to integrate the RuntimeStorage library into a complete ESP32 boiler controller project using PlatformIO.

## Features

- **Full PlatformIO project structure** ready for development
- **FreeRTOS multitasking** with separate tasks for sensors, control, and storage
- **Thread-safe I2C access** using mutex protection
- **Simulated boiler controller** with temperature sensors and PID control
- **Persistent storage** of system state, PID parameters, and operational data
- **Error logging** with critical error tracking
- **Serial command interface** for testing and monitoring

## Project Structure

```
platformio_project/
├── platformio.ini          # PlatformIO configuration
├── src/
│   ├── main.cpp           # Main application
│   └── BoilerController.cpp # Controller implementation
├── include/
│   ├── BoilerController.h  # Controller interface
│   └── config/
│       └── SystemConfig.h  # System configuration
├── lib/                    # Local libraries
└── test/                   # Unit tests
```

## Hardware Requirements

- ESP32 development board
- MB85RC256V FRAM module (32KB)
- I2C connections:
  - SDA: GPIO 33
  - SCL: GPIO 32
  - VCC: 3.3V
  - GND: GND

## Building and Uploading

1. **Install PlatformIO**:
   ```bash
   pip install platformio
   ```

2. **Navigate to project directory**:
   ```bash
   cd examples/platformio_project
   ```

3. **Build the project**:
   ```bash
   pio run
   ```

4. **Upload to ESP32**:
   ```bash
   pio run -t upload
   ```

5. **Monitor serial output**:
   ```bash
   pio device monitor
   ```

## Serial Commands

Once running, use these commands via serial terminal (115200 baud):

- `0` - Set mode OFF
- `1` - Set mode HEATING
- `2` - Set mode HOT WATER
- `3` - Set mode HEATING + HOT WATER
- `s` - Show system status
- `e` - Show error log
- `t` - Show temperature history
- `c` - Clear errors
- `r` - Reset FRAM (requires restart)
- `v` - Verify FRAM integrity
- `h` or `?` - Show help

## Configuration

Edit `include/config/SystemConfig.h` to customize:

- I2C pins and addresses
- Temperature limits and setpoints
- PID parameters
- Timing intervals
- Error codes
- Task priorities and stack sizes

## Code Overview

### Main Application (`src/main.cpp`)

- Initializes hardware and RuntimeStorage
- Creates FreeRTOS tasks for:
  - **Sensor Task**: Reads temperature sensors at 1Hz
  - **Control Task**: Runs control logic at 10Hz
  - **Storage Task**: Saves persistent data every 5 seconds
- Handles serial commands and status display

### Boiler Controller (`src/BoilerController.cpp`)

- Simulates a complete boiler control system
- Implements:
  - Temperature sensor simulation
  - PID control loops
  - Safety interlocks
  - Mode transitions
  - Error detection

### System Configuration (`include/config/SystemConfig.h`)

- Central configuration for all system parameters
- Pin definitions avoiding Ethernet PHY conflicts
- Operating modes and temperature limits
- Task and timing configuration

## Memory Usage

The project is optimized for ESP32 with:
- ~40KB program storage
- ~8KB static RAM
- ~20KB heap usage
- 32KB FRAM for persistent storage

## Integration with Real Hardware

To use with actual hardware:

1. **Replace simulated sensors** in `BoilerController::updateSensorReadings()`
2. **Add relay control** in `BoilerController::updateOutputs()`
3. **Implement actual I2C sensor drivers**
4. **Add MQTT/WiFi** for remote monitoring (see platformio.ini for libraries)

## Testing

Run unit tests:
```bash
pio test
```

## Troubleshooting

1. **FRAM not detected**:
   - Check I2C connections
   - Verify pull-up resistors (4.7kΩ)
   - Try I2C scanner example

2. **Task stack overflow**:
   - Increase stack sizes in SystemConfig.h
   - Monitor with `uxTaskGetStackHighWaterMark()`

3. **I2C conflicts**:
   - Ensure mutex is used for all I2C access
   - Check for proper mutex release

## Advanced Features

The platformio.ini includes configurations for:
- **OTA updates** (`esp32dev_ota` environment)
- **JTAG debugging** (`esp32dev_debug` environment)
- **Memory analysis** tools
- **Exception decoding**

## License

This example is part of the RuntimeStorage library and is provided under the GPL-3 License.