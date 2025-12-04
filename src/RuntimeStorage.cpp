// RuntimeStorage.cpp
// FRAM-based runtime storage implementation for ESP32 Boiler Controller
#include "RuntimeStorage.h"
#include "RuntimeStorageLogging.h"
#include <cstring>

namespace rtstorage {

// Constructor
RuntimeStorage::RuntimeStorage() 
    : _wire(nullptr)
    , _i2cAddress(0)
    , _initialized(false)
    , _eventWriteIndex(0)
    , _eventCount(0)
    , _errorWriteIndex(0)
    , _errorCount(0)
    , _criticalErrorWriteIndex(0)
    , _criticalErrorCount(0) {
    
    memset(_tempWriteIndex, 0, sizeof(_tempWriteIndex));
    memset(&_errorStats, 0, sizeof(_errorStats));
    memset(_seenErrorBitmap, 0, sizeof(_seenErrorBitmap));
    
#if USE_MUTEX_PROTECTION
    _mutex = xSemaphoreCreateMutex();
#endif
}

// Destructor
RuntimeStorage::~RuntimeStorage() {
#if USE_MUTEX_PROTECTION
    if (_mutex) {
        vSemaphoreDelete(_mutex);
    }
#endif
}

// Initialize FRAM
bool RuntimeStorage::begin(TwoWire& wire, uint8_t address) {
    _wire = &wire;
    _i2cAddress = address;
    
    // Test I2C connection
    _wire->beginTransmission(_i2cAddress);
    if (_wire->endTransmission() != 0) {
        RTSTOR_LOG_E("FRAM not found at address 0x%02X", _i2cAddress);
        return false;
    }
    
    // Verify or initialize header
    MemoryHeader header;
    if (!readBytes(ADDR_HEADER, (uint8_t*)&header, sizeof(header))) {
        RTSTOR_LOG_E("Failed to read header");
        return false;
    }
    
    // Check if FRAM needs initialization
    if (header.magic != FRAM_MAGIC || header.version != CURRENT_VERSION) {
        RTSTOR_LOG_I("Initializing FRAM");
        if (!initializeMemory()) {
            return false;
        }
    }
    
    // Load cached indices
    (void)readBytes(ADDR_EVENT_LOG, (uint8_t*)&_eventWriteIndex, sizeof(_eventWriteIndex));
    (void)readBytes(ADDR_EVENT_LOG + 2, (uint8_t*)&_eventCount, sizeof(_eventCount));

    // Load error indices and stats
    (void)readBytes(ADDR_ERROR_LOG, (uint8_t*)&_errorWriteIndex, sizeof(_errorWriteIndex));
    (void)readBytes(ADDR_ERROR_LOG + 2, (uint8_t*)&_errorCount, sizeof(_errorCount));
    (void)readBytes(ADDR_ERROR_LOG + 4, (uint8_t*)&_criticalErrorWriteIndex, sizeof(_criticalErrorWriteIndex));
    (void)readBytes(ADDR_ERROR_LOG + 6, (uint8_t*)&_criticalErrorCount, sizeof(_criticalErrorCount));
    
    // Load error stats
    readWithCRC(ADDR_ERROR_STATS, &_errorStats, sizeof(ErrorStats) - sizeof(uint32_t), sizeof(ErrorStats));
    
    _initialized = true;
    RTSTOR_LOG_I("FRAM initialized successfully");
    return true;
}

// Check if connected
bool RuntimeStorage::isConnected() {
    if (!_initialized || !_wire) return false;
    
    _wire->beginTransmission(_i2cAddress);
    return (_wire->endTransmission() == 0);
}

// Get FRAM size
uint32_t RuntimeStorage::getSize() const noexcept {
    return FRAM_SIZE;
}

// Format FRAM
bool RuntimeStorage::format() {
    RTSTOR_LOG_I("Formatting FRAM");
    
    // Clear all data
    uint8_t buffer[256];
    memset(buffer, 0, sizeof(buffer));
    
    for (uint32_t addr = 0; addr < FRAM_SIZE; addr += sizeof(buffer)) {
        uint32_t chunkSize = min((uint32_t)sizeof(buffer), FRAM_SIZE - addr);
        if (!writeBytes(addr, buffer, chunkSize)) {
            return false;
        }
    }
    
    // Initialize with defaults
    return initializeMemory();
}

// Reset to defaults
bool RuntimeStorage::reset() {
    return format();
}

// Initialize memory with default values
bool RuntimeStorage::initializeMemory() {
    // Write header
    if (!writeHeader()) {
        return false;
    }
    
    // Initialize indices
    uint16_t zero = 0;
    (void)writeBytes(ADDR_EVENT_LOG, (uint8_t*)&zero, sizeof(zero));
    (void)writeBytes(ADDR_EVENT_LOG + 2, (uint8_t*)&zero, sizeof(zero));

    // Initialize error indices
    (void)writeBytes(ADDR_ERROR_LOG, (uint8_t*)&zero, sizeof(zero));
    (void)writeBytes(ADDR_ERROR_LOG + 2, (uint8_t*)&zero, sizeof(zero));
    (void)writeBytes(ADDR_ERROR_LOG + 4, (uint8_t*)&zero, sizeof(zero));
    (void)writeBytes(ADDR_ERROR_LOG + 6, (uint8_t*)&zero, sizeof(zero));
    
    // Initialize error stats and bitmap
    memset(&_errorStats, 0, sizeof(_errorStats));
    memset(_seenErrorBitmap, 0, sizeof(_seenErrorBitmap));
    saveErrorStats();

    return true;
}

// Write header
bool RuntimeStorage::writeHeader() {
    MemoryHeader header;
    header.magic = FRAM_MAGIC;
    header.version = CURRENT_VERSION;
    header.size = FRAM_SIZE;
    header.formatTime = millis();
    
    return writeWithCRC(ADDR_HEADER, &header, sizeof(header) - sizeof(uint32_t), sizeof(header));
}

// I2C write
bool RuntimeStorage::i2cWrite(uint16_t address, const uint8_t* data, size_t length) {
    if (!_wire) return false;
    
    for (uint8_t retry = 0; retry < MAX_RETRIES; retry++) {
        _wire->beginTransmission(_i2cAddress);
        _wire->write((uint8_t)(address >> 8));
        _wire->write((uint8_t)(address & 0xFF));
        
        size_t written = 0;
        while (written < length) {
            size_t chunkSize = min(length - written, (size_t)30);
            _wire->write(data + written, chunkSize);
            written += chunkSize;
        }
        
        if (_wire->endTransmission() == 0) {
            return true;
        }
        
        delay(RETRY_DELAY_MS);
    }
    
    return false;
}

// I2C read
bool RuntimeStorage::i2cRead(uint16_t address, uint8_t* data, size_t length) {
    if (!_wire) return false;
    
    for (uint8_t retry = 0; retry < MAX_RETRIES; retry++) {
        _wire->beginTransmission(_i2cAddress);
        _wire->write((uint8_t)(address >> 8));
        _wire->write((uint8_t)(address & 0xFF));
        
        if (_wire->endTransmission() == 0) {
            size_t read = 0;
            while (read < length) {
                size_t chunkSize = min(length - read, (size_t)32);
                _wire->requestFrom(_i2cAddress, (uint8_t)chunkSize);
                
                size_t available = _wire->available();
                for (size_t i = 0; i < available && read < length; i++) {
                    data[read++] = _wire->read();
                }
            }
            
            if (read == length) {
                return true;
            }
        }
        
        delay(RETRY_DELAY_MS);
    }
    
    return false;
}

// Calculate CRC32
uint32_t RuntimeStorage::calculateCRC(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ ((crc & 1) ? CRC_POLYNOMIAL : 0);
        }
    }
    
    return ~crc;
}

// Write with CRC
bool RuntimeStorage::writeWithCRC(uint16_t address, const void* data, size_t dataSize, size_t totalSize) {
    uint32_t crc = calculateCRC((const uint8_t*)data, dataSize);
    
    if (!writeBytes(address, (const uint8_t*)data, dataSize)) {
        return false;
    }
    
    return writeBytes(address + dataSize, (const uint8_t*)&crc, sizeof(crc));
}

// Read with CRC
bool RuntimeStorage::readWithCRC(uint16_t address, void* data, size_t dataSize, size_t totalSize) {
    if (!readBytes(address, (uint8_t*)data, dataSize)) {
        return false;
    }
    
    uint32_t storedCrc;
    if (!readBytes(address + dataSize, (uint8_t*)&storedCrc, sizeof(storedCrc))) {
        return false;
    }
    
    uint32_t calculatedCrc = calculateCRC((const uint8_t*)data, dataSize);
    return (calculatedCrc == storedCrc);
}

// Raw write
bool RuntimeStorage::writeBytes(uint16_t address, const uint8_t* data, size_t length) {
    if (address + length > FRAM_SIZE) {
        RTSTOR_LOG_E("Write beyond FRAM size");
        return false;
    }
    
#if USE_MUTEX_PROTECTION
    MutexGuard guard(_mutex);
    if (!guard) {
        return false;
    }
#endif

    return i2cWrite(address, data, length);
}

// Raw read
bool RuntimeStorage::readBytes(uint16_t address, uint8_t* data, size_t length) {
    if (address + length > FRAM_SIZE) {
        RTSTOR_LOG_E("Read beyond FRAM size");
        return false;
    }
    
#if USE_MUTEX_PROTECTION
    MutexGuard guard(_mutex);
    if (!guard) {
        return false;
    }
#endif

    return i2cRead(address, data, length);
}

// Save PID state
bool RuntimeStorage::savePIDState(uint8_t controllerId, const PIDState& state) {
    if (controllerId >= MAX_PID_CONTROLLERS) {
        return false;
    }
    
    uint16_t address = ADDR_PID_STATES + (controllerId * sizeof(PIDState));
    return writeWithCRC(address, &state, sizeof(state) - sizeof(uint32_t), sizeof(state));
}

// Load PID state
bool RuntimeStorage::loadPIDState(uint8_t controllerId, PIDState& state) {
    if (controllerId >= MAX_PID_CONTROLLERS) {
        return false;
    }
    
    uint16_t address = ADDR_PID_STATES + (controllerId * sizeof(PIDState));
    return readWithCRC(address, &state, sizeof(state) - sizeof(uint32_t), sizeof(state));
}

// Increment counter
bool RuntimeStorage::incrementCounter(CounterType type) {
    if (type >= COUNTER_MAX) {
        return false;
    }
    
    uint32_t value = getCounter(type);
    value++;
    return setCounter(type, value);
}

// Set counter
bool RuntimeStorage::setCounter(CounterType type, uint32_t value) {
    if (type >= COUNTER_MAX) {
        return false;
    }
    
    uint16_t address = ADDR_COUNTERS + (type * sizeof(uint32_t));
    return writeBytes(address, (const uint8_t*)&value, sizeof(value));
}

// Get counter
uint32_t RuntimeStorage::getCounter(CounterType type) {
    if (type >= COUNTER_MAX) {
        return 0;
    }
    
    uint32_t value = 0;
    uint16_t address = ADDR_COUNTERS + (type * sizeof(uint32_t));
    (void)readBytes(address, (uint8_t*)&value, sizeof(value));
    return value;
}

// Update runtime hours
bool RuntimeStorage::updateRuntimeHours(RuntimeType type, float hours) {
    if (type >= RUNTIME_MAX) {
        return false;
    }
    
    uint16_t address = ADDR_RUNTIME_HOURS + (type * sizeof(float));
    return writeBytes(address, (const uint8_t*)&hours, sizeof(hours));
}

// Get runtime hours
float RuntimeStorage::getRuntimeHours(RuntimeType type) {
    if (type >= RUNTIME_MAX) {
        return 0.0f;
    }
    
    float hours = 0.0f;
    uint16_t address = ADDR_RUNTIME_HOURS + (type * sizeof(float));
    (void)readBytes(address, (uint8_t*)&hours, sizeof(hours));
    return hours;
}

// Log event
bool RuntimeStorage::logEvent(const Event& event) {
    uint16_t index = getEventWriteIndex();
    uint16_t address = ADDR_EVENT_LOG + 8 + (index * sizeof(Event));
    
    if (!writeBytes(address, (const uint8_t*)&event, sizeof(event))) {
        return false;
    }
    
    return incrementEventWriteIndex();
}

// Log event (simplified)
bool RuntimeStorage::logEvent(EventType type, uint32_t data) {
    Event event;
    event.timestamp = millis();
    event.type = type;
    event.subtype = 0;
    event.data = data;
    
    return logEvent(event);
}

// Get event count
size_t RuntimeStorage::getEventCount() const noexcept {
    return min((size_t)_eventCount, (size_t)MAX_EVENTS);
}

// Get events
size_t RuntimeStorage::getEvents(Event* buffer, size_t maxEvents, EventFilter filter) {
    size_t count = getEventCount();
    size_t retrieved = 0;
    
    for (size_t i = 0; i < count && retrieved < maxEvents; i++) {
        uint16_t index = (_eventWriteIndex - 1 - i + MAX_EVENTS) % MAX_EVENTS;
        uint16_t address = ADDR_EVENT_LOG + 8 + (index * sizeof(Event));
        
        Event event;
        if (readBytes(address, (uint8_t*)&event, sizeof(event))) {
            if ((event.type & filter) != 0) {
                buffer[retrieved++] = event;
            }
        }
    }
    
    return retrieved;
}

// Clear events
bool RuntimeStorage::clearEvents() {
    _eventWriteIndex = 0;
    _eventCount = 0;
    
    if (!writeBytes(ADDR_EVENT_LOG, (uint8_t*)&_eventWriteIndex, sizeof(_eventWriteIndex))) {
        return false;
    }
    
    return writeBytes(ADDR_EVENT_LOG + 2, (uint8_t*)&_eventCount, sizeof(_eventCount));
}

// Get event write index
uint16_t RuntimeStorage::getEventWriteIndex() {
    return _eventWriteIndex % MAX_EVENTS;
}

// Increment event write index
bool RuntimeStorage::incrementEventWriteIndex() {
    _eventWriteIndex = (_eventWriteIndex + 1) % MAX_EVENTS;
    _eventCount++;
    
    if (!writeBytes(ADDR_EVENT_LOG, (uint8_t*)&_eventWriteIndex, sizeof(_eventWriteIndex))) {
        return false;
    }
    
    return writeBytes(ADDR_EVENT_LOG + 2, (uint8_t*)&_eventCount, sizeof(_eventCount));
}

// Save system state
bool RuntimeStorage::saveSystemState(const SystemState& state) {
    return writeWithCRC(ADDR_SYSTEM_STATE, &state, sizeof(state) - sizeof(uint32_t), sizeof(state));
}

// Load system state
bool RuntimeStorage::loadSystemState(SystemState& state) {
    return readWithCRC(ADDR_SYSTEM_STATE, &state, sizeof(state) - sizeof(uint32_t), sizeof(state));
}

// Error logging implementation
bool RuntimeStorage::logError(uint32_t errorCode, const char* message, const char* context) {
    uint16_t index = getErrorWriteIndex();
    uint16_t address = ADDR_ERROR_LOG + 8 + (index * sizeof(ErrorEntry));
    
    ErrorEntry entry;
    entry.timestamp = millis();
    entry.errorCode = errorCode;
    entry.count = 1;
    
    if (message) {
        strncpy(entry.message, message, sizeof(entry.message) - 1);
        entry.message[sizeof(entry.message) - 1] = '\0';
    }
    
    if (context) {
        strncpy(entry.context, context, sizeof(entry.context) - 1);
        entry.context[sizeof(entry.context) - 1] = '\0';
    }
    
    if (!writeBytes(address, (const uint8_t*)&entry, sizeof(entry))) {
        return false;
    }
    
    // Update stats
    updateErrorStats(errorCode, entry.timestamp);
    
    // Increment counter
    (void)incrementCounter(COUNTER_ERROR_COUNT);

    return incrementErrorWriteIndex();
}

bool RuntimeStorage::logCriticalError(uint32_t errorCode, const char* message, const char* context) {
    // Log to regular error log first
    (void)logError(errorCode, message, context);
    
    // Then log to critical error buffer
    uint16_t index = getCriticalErrorWriteIndex();
    uint16_t criticalBase = ADDR_ERROR_LOG + 8 + (MAX_ERRORS * sizeof(ErrorEntry));
    uint16_t address = criticalBase + (index * sizeof(ErrorEntry));
    
    ErrorEntry entry;
    entry.timestamp = millis();
    entry.errorCode = errorCode;
    entry.count = 1;
    
    if (message) {
        strncpy(entry.message, message, sizeof(entry.message) - 1);
        entry.message[sizeof(entry.message) - 1] = '\0';
    }
    
    if (context) {
        strncpy(entry.context, context, sizeof(entry.context) - 1);
        entry.context[sizeof(entry.context) - 1] = '\0';
    }
    
    if (!writeBytes(address, (const uint8_t*)&entry, sizeof(entry))) {
        return false;
    }
    
    // Update critical error count in stats
    _errorStats.criticalErrors++;
    saveErrorStats();
    
    return incrementCriticalErrorWriteIndex();
}

bool RuntimeStorage::getError(size_t index, ErrorEntry& entry) {
    if (index >= getErrorCount()) {
        return false;
    }
    
    // Calculate actual index in circular buffer
    uint16_t actualIndex = (_errorWriteIndex - 1 - index + MAX_ERRORS) % MAX_ERRORS;
    uint16_t address = ADDR_ERROR_LOG + 8 + (actualIndex * sizeof(ErrorEntry));
    
    return readBytes(address, (uint8_t*)&entry, sizeof(entry));
}

bool RuntimeStorage::getErrorStats(ErrorStats& stats) {
    stats = _errorStats;
    return true;
}

size_t RuntimeStorage::getErrorCount() const noexcept {
    return min((size_t)_errorCount, (size_t)MAX_ERRORS);
}

size_t RuntimeStorage::getCriticalErrors(ErrorEntry* buffer, size_t maxCount) {
    size_t count = min((size_t)_criticalErrorCount, (size_t)MAX_CRITICAL_ERRORS);
    size_t retrieved = 0;
    
    uint16_t criticalBase = ADDR_ERROR_LOG + 8 + (MAX_ERRORS * sizeof(ErrorEntry));
    
    for (size_t i = 0; i < count && retrieved < maxCount; i++) {
        uint16_t index = (_criticalErrorWriteIndex - 1 - i + MAX_CRITICAL_ERRORS) % MAX_CRITICAL_ERRORS;
        uint16_t address = criticalBase + (index * sizeof(ErrorEntry));
        
        if (readBytes(address, (uint8_t*)&buffer[retrieved], sizeof(ErrorEntry))) {
            retrieved++;
        }
    }
    
    return retrieved;
}

bool RuntimeStorage::clearErrors() {
    _errorWriteIndex = 0;
    _errorCount = 0;
    _criticalErrorWriteIndex = 0;
    _criticalErrorCount = 0;
    
    // Clear indices
    if (!writeBytes(ADDR_ERROR_LOG, (uint8_t*)&_errorWriteIndex, sizeof(_errorWriteIndex))) {
        return false;
    }
    if (!writeBytes(ADDR_ERROR_LOG + 2, (uint8_t*)&_errorCount, sizeof(_errorCount))) {
        return false;
    }
    if (!writeBytes(ADDR_ERROR_LOG + 4, (uint8_t*)&_criticalErrorWriteIndex, sizeof(_criticalErrorWriteIndex))) {
        return false;
    }
    if (!writeBytes(ADDR_ERROR_LOG + 6, (uint8_t*)&_criticalErrorCount, sizeof(_criticalErrorCount))) {
        return false;
    }
    
    // Clear stats
    memset(&_errorStats, 0, sizeof(_errorStats));
    return saveErrorStats();
}

bool RuntimeStorage::clearOldErrors(uint32_t daysOld) {
    // This would require reorganizing the circular buffer
    // For now, just return false
    (void)daysOld; // Suppress unused parameter warning
    return false;
}

// Get error write index
uint16_t RuntimeStorage::getErrorWriteIndex() {
    return _errorWriteIndex % MAX_ERRORS;
}

// Increment error write index
bool RuntimeStorage::incrementErrorWriteIndex() {
    _errorWriteIndex = (_errorWriteIndex + 1) % MAX_ERRORS;
    _errorCount++;
    
    if (!writeBytes(ADDR_ERROR_LOG, (uint8_t*)&_errorWriteIndex, sizeof(_errorWriteIndex))) {
        return false;
    }
    
    return writeBytes(ADDR_ERROR_LOG + 2, (uint8_t*)&_errorCount, sizeof(_errorCount));
}

// Get critical error write index
uint16_t RuntimeStorage::getCriticalErrorWriteIndex() {
    return _criticalErrorWriteIndex % MAX_CRITICAL_ERRORS;
}

// Increment critical error write index
bool RuntimeStorage::incrementCriticalErrorWriteIndex() {
    _criticalErrorWriteIndex = (_criticalErrorWriteIndex + 1) % MAX_CRITICAL_ERRORS;
    _criticalErrorCount++;
    
    if (!writeBytes(ADDR_ERROR_LOG + 4, (uint8_t*)&_criticalErrorWriteIndex, sizeof(_criticalErrorWriteIndex))) {
        return false;
    }
    
    return writeBytes(ADDR_ERROR_LOG + 6, (uint8_t*)&_criticalErrorCount, sizeof(_criticalErrorCount));
}

// Update error stats
bool RuntimeStorage::updateErrorStats(uint32_t errorCode, uint32_t timestamp) {
    _errorStats.totalErrors++;
    _errorStats.lastErrorTime = timestamp;

    if (_errorStats.oldestErrorTime == 0 || timestamp < _errorStats.oldestErrorTime) {
        _errorStats.oldestErrorTime = timestamp;
    }

    // Track unique errors using bitmap
    if (!isErrorCodeSeen(errorCode)) {
        markErrorCodeSeen(errorCode);
        _errorStats.uniqueErrors++;
    }

    return saveErrorStats();
}

// Check if error code has been seen before (using low byte as hash)
bool RuntimeStorage::isErrorCodeSeen(uint32_t errorCode) const {
    uint8_t hashIdx = errorCode & 0xFF;  // Use low byte as hash
    uint8_t wordIdx = hashIdx >> 5;       // Divide by 32 for word index
    uint8_t bitIdx = hashIdx & 0x1F;      // Mod 32 for bit index
    return (_seenErrorBitmap[wordIdx] & (1UL << bitIdx)) != 0;
}

// Mark error code as seen
void RuntimeStorage::markErrorCodeSeen(uint32_t errorCode) {
    uint8_t hashIdx = errorCode & 0xFF;  // Use low byte as hash
    uint8_t wordIdx = hashIdx >> 5;       // Divide by 32 for word index
    uint8_t bitIdx = hashIdx & 0x1F;      // Mod 32 for bit index
    _seenErrorBitmap[wordIdx] |= (1UL << bitIdx);
}

// Save error stats
bool RuntimeStorage::saveErrorStats() {
    return writeWithCRC(ADDR_ERROR_STATS, &_errorStats, sizeof(ErrorStats) - sizeof(uint32_t), sizeof(ErrorStats));
}

// Verify integrity
bool RuntimeStorage::verifyIntegrity() {
    MemoryHeader header;
    if (!readBytes(ADDR_HEADER, (uint8_t*)&header, sizeof(header))) {
        return false;
    }
    
    if (header.magic != FRAM_MAGIC || header.version != CURRENT_VERSION) {
        return false;
    }
    
    uint32_t calculatedCrc = calculateCRC((uint8_t*)&header, sizeof(header) - sizeof(uint32_t));
    return (calculatedCrc == header.crc);
}

// Print memory map
void RuntimeStorage::printMemoryMap() {
    RTSTOR_LOG_I("=== FRAM Memory Map ===");
    RTSTOR_LOG_I("Header:        0x%04X - 0x%04X (%d bytes)", 
        ADDR_HEADER, ADDR_HEADER + SIZE_HEADER - 1, SIZE_HEADER);
    RTSTOR_LOG_I("System State:  0x%04X - 0x%04X (%d bytes)", 
        ADDR_SYSTEM_STATE, ADDR_SYSTEM_STATE + SIZE_SYSTEM_STATE - 1, SIZE_SYSTEM_STATE);
    RTSTOR_LOG_I("PID States:    0x%04X - 0x%04X (%d bytes)", 
        ADDR_PID_STATES, ADDR_PID_STATES + SIZE_PID_STATES - 1, SIZE_PID_STATES);
    RTSTOR_LOG_I("Counters:      0x%04X - 0x%04X (%d bytes)", 
        ADDR_COUNTERS, ADDR_COUNTERS + SIZE_COUNTERS - 1, SIZE_COUNTERS);
    RTSTOR_LOG_I("Runtime Hours: 0x%04X - 0x%04X (%d bytes)", 
        ADDR_RUNTIME_HOURS, ADDR_RUNTIME_HOURS + SIZE_RUNTIME_HOURS - 1, SIZE_RUNTIME_HOURS);
    RTSTOR_LOG_I("Event Log:     0x%04X - 0x%04X (%d bytes)", 
        ADDR_EVENT_LOG, ADDR_EVENT_LOG + SIZE_EVENT_LOG - 1, SIZE_EVENT_LOG);
    RTSTOR_LOG_I("Temp History:  0x%04X - 0x%04X (%d bytes)", 
        ADDR_TEMP_HISTORY, ADDR_TEMP_HISTORY + SIZE_TEMP_HISTORY - 1, SIZE_TEMP_HISTORY);
    RTSTOR_LOG_I("Error Log:     0x%04X - 0x%04X (%d bytes)", 
        ADDR_ERROR_LOG, ADDR_ERROR_LOG + SIZE_ERROR_LOG - 1, SIZE_ERROR_LOG);
    RTSTOR_LOG_I("Error Stats:   0x%04X - 0x%04X (%d bytes)", 
        ADDR_ERROR_STATS, ADDR_ERROR_STATS + SIZE_ERROR_STATS - 1, SIZE_ERROR_STATS);
    RTSTOR_LOG_I("Reserved:      0x%04X - 0x%04X (%d bytes)", 
        ADDR_RESERVED, FRAM_SIZE - 1, FRAM_SIZE - ADDR_RESERVED);
    RTSTOR_LOG_I("Total:         %d bytes", FRAM_SIZE);
}

// Get free space
size_t RuntimeStorage::getFreeSpace() const noexcept {
    return FRAM_SIZE - ADDR_RESERVED;
}

// Record temperature
bool RuntimeStorage::recordTemperature(uint8_t sensorId, Temperature_t temp) {
    if (sensorId >= MAX_TEMP_SENSORS) {
        return false;
    }
    
    TempReading reading;
    reading.timestamp = millis();
    reading.temperature = temp;
    reading.valid = true;
    
    uint16_t index = getTempWriteIndex(sensorId);
    uint16_t baseAddr = ADDR_TEMP_HISTORY + (sensorId * TEMP_HISTORY_SIZE * sizeof(TempReading));
    uint16_t address = baseAddr + (index * sizeof(TempReading));
    
    if (!writeBytes(address, (const uint8_t*)&reading, sizeof(reading))) {
        return false;
    }
    
    return incrementTempWriteIndex(sensorId);
}

// Get temperature history
size_t RuntimeStorage::getTemperatureHistory(uint8_t sensorId, TempReading* buffer, size_t maxReadings) {
    if (sensorId >= MAX_TEMP_SENSORS) {
        return 0;
    }
    
    size_t count = min(maxReadings, (size_t)TEMP_HISTORY_SIZE);
    uint16_t currentIndex = _tempWriteIndex[sensorId];
    uint16_t baseAddr = ADDR_TEMP_HISTORY + (sensorId * TEMP_HISTORY_SIZE * sizeof(TempReading));
    
    for (size_t i = 0; i < count; i++) {
        uint16_t index = (currentIndex - 1 - i + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
        uint16_t address = baseAddr + (index * sizeof(TempReading));
        
        if (!readBytes(address, (uint8_t*)&buffer[i], sizeof(TempReading))) {
            return i;
        }
        
        if (buffer[i].timestamp == 0) {
            return i;
        }
    }
    
    return count;
}

// Get latest temperature
bool RuntimeStorage::getLatestTemperature(uint8_t sensorId, TempReading& reading) {
    if (sensorId >= MAX_TEMP_SENSORS) {
        return false;
    }
    
    uint16_t currentIndex = _tempWriteIndex[sensorId];
    if (currentIndex == 0) {
        return false;
    }
    
    uint16_t index = (currentIndex - 1) % TEMP_HISTORY_SIZE;
    uint16_t baseAddr = ADDR_TEMP_HISTORY + (sensorId * TEMP_HISTORY_SIZE * sizeof(TempReading));
    uint16_t address = baseAddr + (index * sizeof(TempReading));
    
    return readBytes(address, (uint8_t*)&reading, sizeof(reading));
}

// Get temp write index
uint16_t RuntimeStorage::getTempWriteIndex(uint8_t sensorId) {
    if (sensorId >= MAX_TEMP_SENSORS) {
        return 0;
    }
    return _tempWriteIndex[sensorId] % TEMP_HISTORY_SIZE;
}

// Increment temp write index
bool RuntimeStorage::incrementTempWriteIndex(uint8_t sensorId) {
    if (sensorId >= MAX_TEMP_SENSORS) {
        return false;
    }
    
    _tempWriteIndex[sensorId] = (_tempWriteIndex[sensorId] + 1) % TEMP_HISTORY_SIZE;
    
    // Store index in FRAM (after temperature history data)
    uint16_t indexAddr = ADDR_TEMP_HISTORY + SIZE_TEMP_HISTORY + (sensorId * sizeof(uint16_t));
    return writeBytes(indexAddr, (const uint8_t*)&_tempWriteIndex[sensorId], sizeof(uint16_t));
}

// Get version
uint16_t RuntimeStorage::getVersion() {
    MemoryHeader header;
    if (readBytes(ADDR_HEADER, (uint8_t*)&header, sizeof(header))) {
        return header.version;
    }
    return 0;
}

// Migrate data
bool RuntimeStorage::migrate(uint16_t fromVersion) {
    // Implement migration logic when needed
    return true;
}

} // namespace rtstorage