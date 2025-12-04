# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2025-12-04

### Added
- Initial public release
- FRAM-based storage for MB85RC256V chip (32KB) via I2C
- PID state persistence for controller recovery
- Operational counters (burner starts, pump cycles, runtime hours)
- Event logging with circular buffer and filtering
- Error logging with severity levels
- Temperature history ring buffer with timestamps
- Schedule storage for timer scheduler
- CRC32 data integrity protection on all structures
- Thread-safe operations with FreeRTOS mutex
- Power loss recovery with atomic writes
- taskYIELD() for non-blocking I2C operations
- Memory map visualization utilities

Platform: ESP32 (Arduino/ESP-IDF)
Hardware: MB85RC256V FRAM (I2C address 0x50)
License: MIT
Dependencies: Wire (I2C)

### Notes
- Production-tested for runtime data persistence in industrial controller
- 10^13 write cycle endurance (virtually unlimited for embedded use)
- Previous internal versions (v0.1.x, v0.2.x) not publicly released
- Reset to v0.1.0 for clean public release start
