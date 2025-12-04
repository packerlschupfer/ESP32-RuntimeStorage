# RuntimeStorage Library Requirements

## Overview
A wrapper library for MB85RC256V FRAM to provide non-volatile runtime storage for the ESPlan Boiler Controller project. This library abstracts FRAM operations and provides type-safe storage for frequently changing runtime data, eliminating flash wear concerns.

## Hardware Specifications
- **Chip**: Fujitsu MB85RC256V
- **Capacity**: 32KB (256Kbit)
- **Interface**: I2C (400kHz/1MHz)
- **Address**: 0x50-0x57 (configurable via A0-A2 pins)
- **Voltage**: 3.3V
- **Endurance**: 10^13 write cycles
- **Data Retention**: 10 years at 85°C

## Functional Requirements

### 1. Core Functionality
- Initialize FRAM with I2C interface
- Verify FRAM presence and size
- Provide wear-leveling for critical areas (optional but good practice)
- Implement data integrity checks (CRC32)
- Support power-loss recovery
- Handle version migration for data structures

### 2. PID State Management
- Store PID controller states for multiple controllers
- Save integral terms, last error, output values
- Support at least 4 PID controllers
- Atomic updates to prevent corruption

### 3. Operational Counters
- Burner start count (uint32_t)
- Runtime hours for heating, water, total (float)
- Pump runtime counters (uint32_t)
- Error occurrence counts by type
- Persistent across power cycles

### 4. Event Logging
- Circular buffer for last N events (configurable, default 64)
- Event structure: timestamp, type, data
- Support event types: ERROR, WARNING, STATE_CHANGE, USER_ACTION
- Ability to retrieve events by type or time range

### 5. Temperature History
- Ring buffer for temperature readings
- Store last 64 readings per sensor
- Support up to 8 sensors
- Include timestamp with each reading

### 6. System State Preservation
- Current operating mode
- Active alarms and errors
- Last shutdown reason
- Configuration checksums
- Last known good state

## API Design

```cpp
class RuntimeStorage {
public:
    // Initialization
    bool begin(TwoWire& wire = Wire, uint8_t address = 0x50);
    bool isConnected();
    uint32_t getSize();
    
    // Format/Reset
    bool format();  // Erase all data
    bool reset();   // Reset to defaults
    
    // Version Management
    uint16_t getVersion();
    bool migrate(uint16_t fromVersion);
    
    // PID State Management
    bool savePIDState(uint8_t controllerId, const PIDState& state);
    bool loadPIDState(uint8_t controllerId, PIDState& state);
    
    // Counters
    bool incrementCounter(CounterType type);
    bool setCounter(CounterType type, uint32_t value);
    uint32_t getCounter(CounterType type);
    
    bool updateRuntimeHours(RuntimeType type, float hours);
    float getRuntimeHours(RuntimeType type);
    
    // Event Logging
    bool logEvent(const Event& event);
    bool logEvent(EventType type, uint32_t data = 0);
    size_t getEventCount();
    size_t getEvents(Event* buffer, size_t maxEvents, EventFilter filter = ALL_EVENTS);
    bool clearEvents();
    
    // Temperature History
    bool recordTemperature(uint8_t sensorId, Temperature_t temp);
    size_t getTemperatureHistory(uint8_t sensorId, TempReading* buffer, size_t maxReadings);
    bool getLatestTemperature(uint8_t sensorId, TempReading& reading);
    
    // System State
    bool saveSystemState(const SystemState& state);
    bool loadSystemState(SystemState& state);
    
    // Raw Access (for debugging/testing)
    bool writeBytes(uint16_t address, const uint8_t* data, size_t length);
    bool readBytes(uint16_t address, uint8_t* data, size_t length);
    
    // Diagnostics
    bool verifyIntegrity();
    void printMemoryMap();
    size_t getFreeSpace();
    
private:
    // Internal methods
    uint32_t calculateCRC(const uint8_t* data, size_t length);
    bool writeWithCRC(uint16_t address, const void* data, size_t length);
    bool readWithCRC(uint16_t address, void* data, size_t length);
};
```

## Data Structures

```cpp
// Temperature type from main project
typedef int16_t Temperature_t;  // Fixed-point, 1/10th degree

struct PIDState {
    float integral;
    float lastError;
    float output;
    uint32_t lastUpdateTime;
    uint32_t crc;
};

enum CounterType : uint8_t {
    COUNTER_BURNER_STARTS,
    COUNTER_HEATING_PUMP_STARTS,
    COUNTER_WATER_PUMP_STARTS,
    COUNTER_ERROR_COUNT,
    COUNTER_MAX
};

enum RuntimeType : uint8_t {
    RUNTIME_TOTAL,
    RUNTIME_HEATING,
    RUNTIME_WATER,
    RUNTIME_BURNER,
    RUNTIME_MAX
};

enum EventType : uint8_t {
    EVENT_ERROR = 0x01,
    EVENT_WARNING = 0x02,
    EVENT_STATE_CHANGE = 0x04,
    EVENT_USER_ACTION = 0x08,
    EVENT_SYSTEM = 0x10
};

struct Event {
    uint32_t timestamp;     // millis() or RTC time
    EventType type;
    uint8_t subtype;        // Error code, state, etc.
    uint16_t data;          // Event-specific data
};

struct TempReading {
    uint32_t timestamp;
    Temperature_t temperature;
    bool valid;
};

struct SystemState {
    uint8_t operatingMode;
    uint16_t activeErrors;
    uint8_t lastShutdownReason;
    uint32_t uptimeAtShutdown;
    uint32_t crc;
};

// Memory map header
struct MemoryHeader {
    uint32_t magic;         // 0x4652414D ('FRAM')
    uint16_t version;       // Data structure version
    uint16_t size;          // Total FRAM size
    uint32_t formatTime;    // When FRAM was formatted
    uint32_t crc;           // Header CRC
};
```

## Memory Map

```
Address Range    Size    Purpose
-------------    ----    -------
0x0000-0x001F    32B     Memory header and version info
0x0020-0x00FF    224B    System state and configuration
0x0100-0x01FF    256B    PID states (4 controllers × 64B)
0x0200-0x02FF    256B    Operational counters
0x0300-0x03FF    256B    Runtime hours (floats)
0x0400-0x0BFF    2KB     Event log circular buffer
0x0C00-0x2BFF    8KB     Temperature history buffers
0x2C00-0x7FFF    21KB    Reserved for future use

Total: 32KB
```

## Error Handling

1. **I2C Communication Errors**
   - Retry failed operations up to 3 times
   - Return false on persistent failure
   - Log errors if logger available

2. **Data Integrity Errors**
   - Each critical structure includes CRC32
   - Corrupted data returns default/safe values
   - Log corruption events

3. **Version Mismatch**
   - Attempt automatic migration
   - Fall back to format if migration fails
   - Preserve counters if possible

## Integration Requirements

1. **Dependencies**
   - Wire.h (I2C)
   - Adafruit_FRAM_I2C or equivalent
   - CRC32 library (or implement internally)

2. **ESP32 Compatibility**
   - Support both I2C ports
   - Thread-safe operations (mutex protection)
   - Work with Arduino framework

3. **Boiler Controller Integration**
   - Initialize after I2C in SystemInitializer
   - Add to SystemResourceProvider
   - Call from PersistentStorageTask for periodic operations
   - Update PID controllers to save state

## Testing Requirements

1. **Unit Tests**
   - Test all API methods
   - Verify CRC protection
   - Test power-loss recovery
   - Verify circular buffers

2. **Integration Tests**
   - Test with real MB85RC256V
   - Verify I2C bus sharing with DS3231
   - Test concurrent access
   - Measure performance

3. **Stress Tests**
   - Continuous write/read cycles
   - Power cycling during operations
   - Full memory usage
   - Concurrent access from multiple tasks

## Example Usage

```cpp
#include <RuntimeStorage.h>

RuntimeStorage fram;

void setup() {
    Wire.begin();
    
    if (!fram.begin(Wire, 0x50)) {
        Serial.println("FRAM not found!");
        return;
    }
    
    // Load saved PID state
    PIDState pidState;
    if (fram.loadPIDState(0, pidState)) {
        // Restore PID controller state
        pidController.setIntegral(pidState.integral);
    }
    
    // Log startup event
    fram.logEvent(EVENT_SYSTEM, 0x01);  // System startup
    
    // Get last temperature
    TempReading lastTemp;
    if (fram.getLatestTemperature(SENSOR_BOILER_OUTPUT, lastTemp)) {
        Serial.printf("Last temp: %.1f at %lu\n", 
                      lastTemp.temperature / 10.0, lastTemp.timestamp);
    }
}

void loop() {
    // Save PID state periodically
    static uint32_t lastSave = 0;
    if (millis() - lastSave > 1000) {  // Every second
        PIDState state = {
            .integral = pidController.getIntegral(),
            .lastError = pidController.getLastError(),
            .output = pidController.getOutput(),
            .lastUpdateTime = millis()
        };
        fram.savePIDState(0, state);
        lastSave = millis();
    }
    
    // Record temperature
    Temperature_t temp = readTemperature();
    fram.recordTemperature(SENSOR_BOILER_OUTPUT, temp);
}
```

## Performance Requirements

- Write speed: < 1ms for single structure
- Read speed: < 1ms for single structure
- Initialization: < 100ms
- CRC calculation: < 1ms for 256 bytes

## Future Enhancements

1. **Redundancy Support**
   - Dual FRAM chips for critical data
   - Automatic failover
   - Data mirroring

2. **Compression**
   - Optional compression for event logs
   - Run-length encoding for temperature history

3. **Remote Access**
   - MQTT commands to read FRAM data
   - Diagnostic dumps via network

4. **Wear Leveling**
   - Even though FRAM doesn't need it
   - Good practice for portability

## Notes for Implementers

1. Start with basic read/write functionality
2. Add CRC protection early
3. Implement PID state storage first (highest priority)
4. Use templates for type safety where appropriate
5. Consider making event log size configurable at compile time
6. Document thread safety requirements
7. Add debug output controlled by compile flag

## Delivery

Create library in standard Arduino library format:
- Place in `~/.platformio/lib/ESP32-RuntimeStorage/`
- Include examples for boiler controller use case
- Add keywords.txt for syntax highlighting
- Create library.properties file
- Include this requirements doc as README.md