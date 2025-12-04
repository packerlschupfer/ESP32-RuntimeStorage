// RuntimeStorage.h
// FRAM-based runtime storage for ESP32 Boiler Controller
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "RuntimeStorageTypes.h"
#include "RuntimeStorageConfig.h"

#if USE_MUTEX_PROTECTION
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "MutexGuard.h"
#endif

namespace rtstorage {

class RuntimeStorage {
public:
    RuntimeStorage();
    ~RuntimeStorage();

    // Initialization
    [[nodiscard]] bool begin(TwoWire& wire = Wire, uint8_t address = DEFAULT_I2C_ADDRESS);
    [[nodiscard]] bool isConnected() noexcept;
    [[nodiscard]] uint32_t getSize() const noexcept;

    // Format/Reset
    [[nodiscard]] bool format();
    [[nodiscard]] bool reset();

    // Version Management
    [[nodiscard]] uint16_t getVersion();
    [[nodiscard]] bool migrate(uint16_t fromVersion);

    // PID State Management
    [[nodiscard]] bool savePIDState(uint8_t controllerId, const PIDState& state);
    [[nodiscard]] bool loadPIDState(uint8_t controllerId, PIDState& state);

    // Counters
    [[nodiscard]] bool incrementCounter(CounterType type);
    [[nodiscard]] bool setCounter(CounterType type, uint32_t value);
    [[nodiscard]] uint32_t getCounter(CounterType type);

    [[nodiscard]] bool updateRuntimeHours(RuntimeType type, float hours);
    [[nodiscard]] float getRuntimeHours(RuntimeType type);

    // Event Logging
    [[nodiscard]] bool logEvent(const Event& event);
    [[nodiscard]] bool logEvent(EventType type, uint32_t data = 0);
    [[nodiscard]] size_t getEventCount() const noexcept;
    [[nodiscard]] size_t getEvents(Event* buffer, size_t maxEvents, EventFilter filter = ALL_EVENTS);
    [[nodiscard]] bool clearEvents();

    // Temperature History
    [[nodiscard]] bool recordTemperature(uint8_t sensorId, Temperature_t temp);
    [[nodiscard]] size_t getTemperatureHistory(uint8_t sensorId, TempReading* buffer, size_t maxReadings);
    [[nodiscard]] bool getLatestTemperature(uint8_t sensorId, TempReading& reading);

    // System State
    [[nodiscard]] bool saveSystemState(const SystemState& state);
    [[nodiscard]] bool loadSystemState(SystemState& state);

    // Error Logging
    [[nodiscard]] bool logError(uint32_t errorCode, const char* message = nullptr, const char* context = nullptr);
    [[nodiscard]] bool logCriticalError(uint32_t errorCode, const char* message = nullptr, const char* context = nullptr);
    [[nodiscard]] bool getError(size_t index, ErrorEntry& entry);
    [[nodiscard]] bool getErrorStats(ErrorStats& stats);
    [[nodiscard]] size_t getErrorCount() const noexcept;
    [[nodiscard]] size_t getCriticalErrors(ErrorEntry* buffer, size_t maxCount);
    [[nodiscard]] bool clearErrors();
    [[nodiscard]] bool clearOldErrors(uint32_t daysOld);

    // Raw Access (for debugging/testing)
    [[nodiscard]] bool writeBytes(uint16_t address, const uint8_t* data, size_t length);
    [[nodiscard]] bool readBytes(uint16_t address, uint8_t* data, size_t length);

    // Diagnostics
    [[nodiscard]] bool verifyIntegrity();
    void printMemoryMap();
    [[nodiscard]] size_t getFreeSpace() const noexcept;

private:
    // I2C communication
    bool i2cWrite(uint16_t address, const uint8_t* data, size_t length);
    bool i2cRead(uint16_t address, uint8_t* data, size_t length);
    
    // CRC calculation
    uint32_t calculateCRC(const uint8_t* data, size_t length);
    bool writeWithCRC(uint16_t address, const void* data, size_t dataSize, size_t totalSize);
    bool readWithCRC(uint16_t address, void* data, size_t dataSize, size_t totalSize);
    
    // Memory management
    bool initializeMemory();
    bool verifyHeader();
    bool writeHeader();
    
    // Event log management
    uint16_t getEventWriteIndex();
    bool incrementEventWriteIndex();
    
    // Temperature history management
    uint16_t getTempWriteIndex(uint8_t sensorId);
    bool incrementTempWriteIndex(uint8_t sensorId);
    
    // Error log management
    uint16_t getErrorWriteIndex();
    bool incrementErrorWriteIndex();
    uint16_t getCriticalErrorWriteIndex();
    bool incrementCriticalErrorWriteIndex();
    bool updateErrorStats(uint32_t errorCode, uint32_t timestamp);
    bool saveErrorStats();
    bool isErrorCodeSeen(uint32_t errorCode) const;
    void markErrorCodeSeen(uint32_t errorCode);
    
    // Member variables
    TwoWire* _wire;
    uint8_t _i2cAddress;
    bool _initialized;
    
    // Cached values
    uint16_t _eventWriteIndex;
    uint16_t _eventCount;
    uint16_t _tempWriteIndex[MAX_TEMP_SENSORS];
    uint16_t _errorWriteIndex;
    uint16_t _errorCount;
    uint16_t _criticalErrorWriteIndex;
    uint16_t _criticalErrorCount;
    ErrorStats _errorStats;
    uint32_t _seenErrorBitmap[8];  // 256-bit bitmap for tracking unique error codes
    
#if USE_MUTEX_PROTECTION
    SemaphoreHandle_t _mutex;
#endif
};

} // namespace rtstorage

// Make the class available in global namespace for convenience
using rtstorage::RuntimeStorage;