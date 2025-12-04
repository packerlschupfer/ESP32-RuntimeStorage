# CLAUDE.md - RuntimeStorage Library

## Overview
RuntimeStorage is a thread-safe FRAM library for ESP32 that provides non-volatile storage for runtime data in the ESPlan Boiler Controller project. It uses the Fujitsu MB85RC256V FRAM chip (32KB) via I2C interface.

## Key Features
- **PID State Persistence**: Save/restore PID controller states across power cycles
- **Operational Counters**: Track burner starts, pump cycles, runtime hours
- **Event Logging**: Circular buffer for system events with filtering
- **Temperature History**: Ring buffer for sensor readings with timestamps
- **Data Integrity**: CRC32 protection on all critical structures
- **Thread Safety**: FreeRTOS mutex protection for concurrent access
- **Power Loss Recovery**: Atomic writes and data verification

## Architecture

### Memory Map
```
0x0000-0x001F    32B     Header with version and CRC
0x0020-0x00FF    224B    System state and configuration
0x0100-0x01FF    256B    PID states (4 controllers × 64B)
0x0200-0x02FF    256B    Operational counters
0x0300-0x03FF    256B    Runtime hours (floats)
0x0400-0x0BFF    2KB     Event log circular buffer
0x0C00-0x2BFF    8KB     Temperature history buffers
0x2C00-0x7FFF    21KB    Reserved for future use
```

### Design Patterns
1. **Namespace Encapsulation**: All types and classes in `RuntimeStorage` namespace
2. **RAII Mutex Guard**: Thread-safe operations with automatic unlock
3. **Circular Buffers**: Efficient use of limited FRAM space
4. **CRC Protection**: Data integrity on power loss
5. **Retry Logic**: Robust I2C communication

## Development Commands

```bash
# Build the library
cd /home/mrnice/Documents/PlatformIO/libs/workspace_Class-RuntimeStorage
pio run

# Run examples
cd examples/basic && pio run -t upload
cd examples/boiler_controller && pio run -t upload
cd examples/stress_test && pio run -t upload

# Monitor output
pio device monitor -b 115200
```

## Integration with Boiler Controller

1. **Initialization**: After I2C in SystemInitializer
2. **Resource Provider**: Add to SystemResourceProvider
3. **Periodic Tasks**: Call from PersistentStorageTask
4. **PID Integration**: Update controllers to save/restore state

## Testing Strategy

### Unit Tests
- All API methods with edge cases
- CRC verification
- Power loss simulation
- Buffer overflow handling

### Integration Tests  
- Real MB85RC256V hardware
- I2C bus sharing with DS3231 RTC
- Concurrent access patterns
- Performance benchmarks

### Stress Tests
- Continuous read/write cycles
- Power cycling during operations
- Full memory utilization
- Multi-task access patterns

## Performance Characteristics
- Write speed: < 1ms per structure
- Read speed: < 1ms per structure  
- I2C speed: 400kHz default, 1MHz max
- Endurance: 10^13 write cycles
- Data retention: 10 years at 85°C

## Error Handling
1. **I2C Errors**: 3 retries with 5ms delay
2. **CRC Failures**: Return safe defaults
3. **Version Mismatch**: Automatic migration
4. **Buffer Overflow**: Circular overwrite

## Future Enhancements
1. **Redundancy**: Dual FRAM support
2. **Compression**: Event log compression
3. **Remote Access**: MQTT data retrieval
4. **Wear Leveling**: Even though not needed
5. **Encryption**: For sensitive data

## Known Limitations
- Fixed 32KB size (hardware limit)
- No built-in wear leveling (not needed for FRAM)
- Single I2C address (0x50-0x57 range)
- Thread safety adds ~100μs overhead

## Debugging Tips
- Enable `RUNTIME_STORAGE_DEBUG` for verbose logging
- Use `printMemoryMap()` to visualize layout
- Call `verifyIntegrity()` after issues
- Monitor I2C bus with logic analyzer
- Check CRC errors in production logs

## Development Notes
- After changes commit smartly