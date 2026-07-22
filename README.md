# ESP32-RuntimeStorage

FRAM-backed non-volatile storage for ESP32, built around the Fujitsu **MB85RC256V** (32 KB, I²C).

Use it for data that changes constantly — PID integrator state, runtime counters, event and error logs, temperature history. FRAM has ~10^13 write endurance and needs no erase cycle, so unlike NVS/flash there is no wear budget to ration and no write-amplification to design around. Every write is byte-addressable and takes effect immediately.

## Why FRAM instead of NVS

| | FRAM (this library) | ESP32 NVS / flash |
|---|---|---|
| Write endurance | ~10^13 cycles | ~10^5 cycles per sector |
| Write granularity | 1 byte | page/sector erase-rewrite |
| Write latency | I²C transfer only | erase + program, can block |
| Power-loss safety | Write completes or doesn't | Partial sector rewrite possible |
| Good for | Per-second state, hot counters, logs | Config, credentials, calibration |

For settings that change rarely, use [ESP32-PersistentStorage](https://github.com/packerlschupfer/ESP32-PersistentStorage) instead. The two are complementary.

## Hardware

- **Chip**: Fujitsu MB85RC256V, 32 KB (256 Kbit)
- **Interface**: I²C, 400 kHz default
- **Address**: `0x50`–`0x57`, selected by the A0–A2 pins (default `0x50`)
- **Voltage**: 3.3 V
- **Retention**: 10 years at 85 °C

Wiring: `SDA`/`SCL` to your chosen I²C pins, `VCC` to 3.3 V, `GND` to `GND`. The examples use GPIO 33 (SDA) and GPIO 32 (SCL) to stay clear of the LAN8720 Ethernet PHY pins.

The chip shares the bus happily with other I²C devices such as a DS3231 RTC.

## Installation

```ini
lib_deps =
    https://github.com/packerlschupfer/ESP32-RuntimeStorage.git
    https://github.com/packerlschupfer/ESP32-MutexGuard.git   ; for thread-safe builds
```

## Quick start

```cpp
#include <RuntimeStorage.h>
#include <Wire.h>

RuntimeStorage fram;

void setup() {
    Serial.begin(115200);
    Wire.begin(33, 32);                 // SDA, SCL

    if (!fram.begin(Wire, 0x50)) {
        Serial.println("FRAM not found");
        return;
    }

    Serial.printf("Size: %lu bytes, layout version %u\n",
                  fram.getSize(), fram.getVersion());

    // Counters survive power cycles
    if (!fram.incrementCounter(rtstorage::COUNTER_SYSTEM_RESTARTS)) {
        Serial.println("counter write failed");
    }

    if (!fram.logEvent(rtstorage::EVENT_SYSTEM, 0x01)) {
        Serial.println("event write failed");
    }
}
```

### Every fallible call is `[[nodiscard]]`

All fallible methods are marked `[[nodiscard]]`, so ignoring a result is a compiler warning. This is deliberate: an I²C write can fail, and silently losing a burner-start count or a PID integrator is worse than a noisy build. Always branch on the return value:

```cpp
if (!fram.savePIDState(0, state)) {
    LOG_ERROR(TAG, "PID state not persisted");
}
```

## API

Everything lives in namespace `rtstorage`; the class itself is also pulled into the global namespace for convenience, so `RuntimeStorage` works unqualified while enum values need the `rtstorage::` prefix.

### Lifecycle

```cpp
bool     begin(TwoWire& wire = Wire, uint8_t address = 0x50);
bool     isConnected() noexcept;
uint32_t getSize() const noexcept;
bool     format();                  // erase everything, rewrite header
bool     reset();                   // restore defaults
uint16_t getVersion();
bool     migrate(uint16_t fromVersion);
```

### PID state

Up to 4 controllers (`MAX_PID_CONTROLLERS`), CRC-protected so a torn write is detected rather than restored as garbage.

```cpp
bool savePIDState(uint8_t controllerId, const PIDState& state);
bool loadPIDState(uint8_t controllerId, PIDState& state);
```

### Counters and runtime hours

```cpp
bool     incrementCounter(CounterType type);
bool     setCounter(CounterType type, uint32_t value);
uint32_t getCounter(CounterType type);

bool  updateRuntimeHours(RuntimeType type, float hours);
float getRuntimeHours(RuntimeType type);
```

`CounterType`: `COUNTER_BURNER_STARTS`, `COUNTER_HEATING_PUMP_STARTS`, `COUNTER_WATER_PUMP_STARTS`, `COUNTER_ERROR_COUNT`, `COUNTER_SYSTEM_RESTARTS`.

`RuntimeType`: `RUNTIME_TOTAL`, `RUNTIME_HEATING`, `RUNTIME_WATER`, `RUNTIME_BURNER`.

### Event log

Circular buffer of 64 events (`MAX_EVENTS`). `EventType` values are bit flags, so `getEvents()` can filter by mask.

```cpp
bool   logEvent(const Event& event);
bool   logEvent(EventType type, uint32_t data = 0);
size_t getEventCount() const noexcept;
size_t getEvents(Event* buffer, size_t maxEvents, EventFilter filter = ALL_EVENTS);
bool   clearEvents();
```

```cpp
Event buf[16];
size_t n = fram.getEvents(buf, 16, rtstorage::EVENT_ERROR | rtstorage::EVENT_WARNING);
```

`EventType`: `EVENT_ERROR` `0x01`, `EVENT_WARNING` `0x02`, `EVENT_STATE_CHANGE` `0x04`, `EVENT_USER_ACTION` `0x08`, `EVENT_SYSTEM` `0x10`, `EVENT_ALL` `0xFF`.

### Error log

Richer than the event log — carries a message and context string per entry. Keeps 50 errors plus a separate ring of 5 critical errors that ordinary errors cannot evict.

```cpp
bool   logError(uint32_t errorCode, const char* message = nullptr, const char* context = nullptr);
bool   logCriticalError(uint32_t errorCode, const char* message = nullptr, const char* context = nullptr);
bool   getError(size_t index, ErrorEntry& entry);
bool   getErrorStats(ErrorStats& stats);
size_t getErrorCount() const noexcept;
size_t getCriticalErrors(ErrorEntry* buffer, size_t maxCount);
bool   clearErrors();
bool   clearOldErrors(uint32_t daysOld);
```

Repeated identical error codes increment `ErrorEntry::count` rather than filling the buffer, and `ErrorStats::uniqueErrors` tracks distinct codes via an internal 256-bit bitmap. `message` holds up to 63 characters, `context` up to 31.

### Temperature history

Ring buffer of 64 readings for each of 8 sensors. `Temperature_t` is `int16_t` in tenths of a degree.

```cpp
bool   recordTemperature(uint8_t sensorId, Temperature_t temp);
size_t getTemperatureHistory(uint8_t sensorId, TempReading* buffer, size_t maxReadings);
bool   getLatestTemperature(uint8_t sensorId, TempReading& reading);
```

### System state, raw access, diagnostics

```cpp
bool saveSystemState(const SystemState& state);
bool loadSystemState(SystemState& state);

bool writeBytes(uint16_t address, const uint8_t* data, size_t length);
bool readBytes(uint16_t address, uint8_t* data, size_t length);

bool   verifyIntegrity();
void   printMemoryMap();
size_t getFreeSpace() const noexcept;
```

## Memory map

| Address | Size | Purpose |
|---|---|---|
| `0x0000`–`0x001F` | 32 B | Header: magic `0x4652414D`, version, CRC |
| `0x0020`–`0x00FF` | 224 B | System state |
| `0x0100`–`0x01FF` | 256 B | PID states (4 × 64 B) |
| `0x0200`–`0x02FF` | 256 B | Counters |
| `0x0300`–`0x03FF` | 256 B | Runtime hours |
| `0x0400`–`0x0BFF` | 2 KB | Event log |
| `0x0C00`–`0x2BFF` | 8 KB | Temperature history |
| `0x2C00`–`0x4BFF` | 8 KB | Error log |
| `0x4C00`–`0x4C1F` | 32 B | Error statistics |
| `0x4C20`–`0x7FFF` | ~13 KB | Reserved |

Offsets are compile-time constants in `RuntimeStorageConfig.h`.

## Thread safety

On ESP32, `USE_MUTEX_PROTECTION` is enabled automatically and every public method serializes on an internal mutex, so concurrent access from multiple FreeRTOS tasks is safe.

If another driver talks to the **same physical chip** outside this library — for example raw `Wire` transactions from your own code — it must serialize against that same mutex. FRAM reads are two-phase (set address latch, then read), and an interleaved transaction moves the latch mid-read, returning data from the wrong offset:

```cpp
SemaphoreHandle_t bus = fram.getBusMutex();     // nullptr if compiled out
if (bus && xSemaphoreTake(bus, pdMS_TO_TICKS(100)) == pdTRUE) {
    // raw Wire access to the FRAM here
    xSemaphoreGive(bus);
}
```

## Configuration

Compile-time constants in `RuntimeStorageConfig.h`:

| Constant | Default | Meaning |
|---|---|---|
| `DEFAULT_I2C_ADDRESS` | `0x50` | Chip address |
| `FRAM_SIZE` | 32768 | Total bytes |
| `I2C_SPEED` | 400000 | Bus speed |
| `MAX_RETRIES` | 3 | Retries per failed transfer |
| `RETRY_DELAY_MS` | 5 | Delay between retries |
| `MAX_PID_CONTROLLERS` | 4 | PID slots |
| `MAX_TEMP_SENSORS` | 8 | Sensor history channels |
| `MAX_EVENTS` | 64 | Event ring size |
| `TEMP_HISTORY_SIZE` | 64 | Readings per sensor |
| `MAX_ERRORS` | 50 | Error ring size |
| `MAX_CRITICAL_ERRORS` | 5 | Protected critical slots |

Build flags:

```ini
build_flags =
    -DRUNTIME_STORAGE_DEBUG=1     ; verbose logging (default 0)
```

## Data integrity

Every critical structure is written with a trailing CRC32 (polynomial `0xEDB88320`) and verified on read. A failed CRC returns `false` rather than handing back corrupt state, so callers can fall back to defaults. `verifyIntegrity()` walks the whole map on demand. Failed I²C transfers are retried up to `MAX_RETRIES` times before being reported.

On startup `begin()` checks the header magic and version; a version mismatch routes through `migrate()`, which falls back to `format()` if migration is not possible.

## Examples

| Example | Shows |
|---|---|
| `examples/basic` | Init, counters, events, temperature history |
| `examples/error_logging` | Error and critical-error logging, statistics |
| `examples/boiler_controller` | PID state persistence and runtime counters |
| `examples/i2c_scanner` | Locating the chip on the bus |
| `examples/stress_test` | Sustained write/read cycling |
| `examples/platformio_project` | Full PlatformIO project layout |

## License

This library is released under the **GNU General Public License v3.0**. See the [LICENSE](LICENSE) file for the full text.

Copyright (C) 2025-2026 packerlschupfer

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version. It is distributed WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
