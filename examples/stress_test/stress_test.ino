// RuntimeStorage Stress Test Example
// Tests FRAM reliability and performance

#include <RuntimeStorage.h>
#include <Wire.h>

// I2C pins configured to avoid Ethernet PHY conflicts
#define I2C_SDA_PIN 33
#define I2C_SCL_PIN 32

RuntimeStorage fram;

// Test statistics
struct TestStats {
    uint32_t totalWrites = 0;
    uint32_t totalReads = 0;
    uint32_t writeErrors = 0;
    uint32_t readErrors = 0;
    uint32_t crcErrors = 0;
    uint32_t startTime = 0;
    uint32_t testDuration = 0;
};

TestStats stats;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println(F("\n=== RuntimeStorage Stress Test ==="));
    Serial.println(F("WARNING: This test will wear the FRAM!"));
    Serial.println(F("Press 'y' to continue, any other key to abort"));
    
    while (!Serial.available()) delay(10);
    char confirm = Serial.read();
    while (Serial.available()) Serial.read();  // Clear buffer
    
    if (confirm != 'y' && confirm != 'Y') {
        Serial.println(F("Test aborted"));
        while (1) delay(1000);
    }
    
    // Initialize I2C at maximum speed
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(1000000);  // 1MHz for MB85RC256V
    
    // Initialize FRAM
    if (!fram.begin(Wire, 0x50)) {
        Serial.println(F("FRAM initialization failed!"));
        while (1) delay(1000);
    }
    
    Serial.println(F("FRAM initialized successfully"));
    Serial.printf("Size: %lu bytes\n", fram.getSize());
    
    // Format for clean test
    Serial.println(F("\nFormatting FRAM for test..."));
    if (!fram.format()) {
        Serial.println(F("Format failed!"));
        while (1) delay(1000);
    }
    
    Serial.println(F("\nStarting stress tests..."));
    Serial.println(F("Press any key to stop\n"));
    
    stats.startTime = millis();
}

void loop() {
    // Run different stress tests
    if (!testRawAccess()) return;
    if (!testPIDStates()) return;
    if (!testCounters()) return;
    if (!testEventLogging()) return;
    if (!testTemperatureHistory()) return;
    if (!testConcurrentAccess()) return;
    if (!testPowerLossRecovery()) return;
    
    // Print statistics
    printStatistics();
    
    // Check for stop command
    if (Serial.available()) {
        Serial.println(F("\n\nTest stopped by user"));
        printFinalReport();
        while (1) delay(1000);
    }
}

bool testRawAccess() {
    Serial.print(F("Testing raw access... "));
    
    const size_t testSize = 256;
    uint8_t writeData[testSize];
    uint8_t readData[testSize];
    
    // Fill with random data
    for (size_t i = 0; i < testSize; i++) {
        writeData[i] = random(256);
    }
    
    // Write to multiple locations
    for (uint16_t addr = 0x3000; addr < 0x4000; addr += testSize) {
        if (!fram.writeBytes(addr, writeData, testSize)) {
            stats.writeErrors++;
            Serial.println(F("WRITE ERROR"));
            return false;
        }
        stats.totalWrites++;
        
        // Read back and verify
        if (!fram.readBytes(addr, readData, testSize)) {
            stats.readErrors++;
            Serial.println(F("READ ERROR"));
            return false;
        }
        stats.totalReads++;
        
        // Verify data
        if (memcmp(writeData, readData, testSize) != 0) {
            stats.crcErrors++;
            Serial.println(F("DATA MISMATCH"));
            return false;
        }
    }
    
    Serial.println(F("OK"));
    return true;
}

bool testPIDStates() {
    Serial.print(F("Testing PID states... "));
    
    for (uint8_t i = 0; i < RuntimeStorage::MAX_PID_CONTROLLERS; i++) {
        RuntimeStorage::PIDState writeState = {
            .integral = random(1000) / 10.0f,
            .lastError = random(200) / 10.0f - 10.0f,
            .output = random(101),
            .lastUpdateTime = millis()
        };
        
        if (!fram.savePIDState(i, writeState)) {
            stats.writeErrors++;
            Serial.println(F("WRITE ERROR"));
            return false;
        }
        stats.totalWrites++;
        
        RuntimeStorage::PIDState readState;
        if (!fram.loadPIDState(i, readState)) {
            stats.readErrors++;
            Serial.println(F("READ ERROR"));
            return false;
        }
        stats.totalReads++;
        
        // Verify (excluding CRC field)
        if (memcmp(&writeState, &readState, sizeof(RuntimeStorage::PIDState) - sizeof(uint32_t)) != 0) {
            stats.crcErrors++;
            Serial.println(F("DATA MISMATCH"));
            return false;
        }
    }
    
    Serial.println(F("OK"));
    return true;
}

bool testCounters() {
    Serial.print(F("Testing counters... "));
    
    // Test rapid increments
    for (int i = 0; i < 100; i++) {
        uint32_t before = fram.getCounter(RuntimeStorage::COUNTER_BURNER_STARTS);
        stats.totalReads++;
        
        if (!fram.incrementCounter(RuntimeStorage::COUNTER_BURNER_STARTS)) {
            stats.writeErrors++;
            Serial.println(F("INCREMENT ERROR"));
            return false;
        }
        stats.totalWrites++;
        
        uint32_t after = fram.getCounter(RuntimeStorage::COUNTER_BURNER_STARTS);
        stats.totalReads++;
        
        if (after != before + 1) {
            stats.crcErrors++;
            Serial.printf("COUNTER ERROR: %lu -> %lu\n", before, after);
            return false;
        }
    }
    
    Serial.println(F("OK"));
    return true;
}

bool testEventLogging() {
    Serial.print(F("Testing event logging... "));
    
    // Fill event log
    for (int i = 0; i < RuntimeStorage::MAX_EVENTS * 2; i++) {  // Overflow test
        RuntimeStorage::Event event = {
            .timestamp = millis(),
            .type = (RuntimeStorage::EventType)(1 << (i % 5)),
            .subtype = (uint8_t)(i & 0xFF),
            .data = (uint16_t)i
        };
        
        if (!fram.logEvent(event)) {
            stats.writeErrors++;
            Serial.println(F("LOG ERROR"));
            return false;
        }
        stats.totalWrites++;
    }
    
    // Verify circular buffer behavior
    size_t count = fram.getEventCount();
    if (count != RuntimeStorage::MAX_EVENTS) {
        Serial.printf("COUNT ERROR: %d != %d\n", count, RuntimeStorage::MAX_EVENTS);
        return false;
    }
    
    // Read back events
    RuntimeStorage::Event events[10];
    size_t read = fram.getEvents(events, 10);
    stats.totalReads += read;
    
    if (read != 10) {
        Serial.printf("READ ERROR: %d events\n", read);
        return false;
    }
    
    Serial.println(F("OK"));
    return true;
}

bool testTemperatureHistory() {
    Serial.print(F("Testing temperature history... "));
    
    // Write temperature data for all sensors
    for (uint8_t sensor = 0; sensor < RuntimeStorage::MAX_TEMP_SENSORS; sensor++) {
        for (int i = 0; i < RuntimeStorage::TEMP_HISTORY_SIZE + 10; i++) {  // Overflow test
            RuntimeStorage::Temperature_t temp = 200 + random(600);  // 20-80°C
            
            if (!fram.recordTemperature(sensor, temp)) {
                stats.writeErrors++;
                Serial.println(F("RECORD ERROR"));
                return false;
            }
            stats.totalWrites++;
        }
    }
    
    // Verify latest readings
    for (uint8_t sensor = 0; sensor < RuntimeStorage::MAX_TEMP_SENSORS; sensor++) {
        RuntimeStorage::TempReading reading;
        if (!fram.getLatestTemperature(sensor, reading)) {
            stats.readErrors++;
            Serial.println(F("READ ERROR"));
            return false;
        }
        stats.totalReads++;
        
        if (!reading.valid) {
            Serial.println(F("INVALID READING"));
            return false;
        }
    }
    
    Serial.println(F("OK"));
    return true;
}

bool testConcurrentAccess() {
    Serial.print(F("Testing concurrent access patterns... "));
    
    // Simulate multiple systems accessing FRAM
    for (int i = 0; i < 50; i++) {
        // System 1: Update counters
        fram.incrementCounter(RuntimeStorage::COUNTER_HEATING_PUMP_STARTS);
        stats.totalWrites++;
        
        // System 2: Log temperature
        fram.recordTemperature(random(8), 250 + random(100));
        stats.totalWrites++;
        
        // System 3: Save PID state
        RuntimeStorage::PIDState pid = {
            .integral = i * 1.5f,
            .lastError = i * 0.1f,
            .output = i % 100,
            .lastUpdateTime = millis()
        };
        fram.savePIDState(i % 4, pid);
        stats.totalWrites++;
        
        // System 4: Log event
        fram.logEvent(RuntimeStorage::EVENT_STATE_CHANGE, i);
        stats.totalWrites++;
        
        // Verify a random operation
        if (i % 10 == 0) {
            uint32_t counter = fram.getCounter(RuntimeStorage::COUNTER_HEATING_PUMP_STARTS);
            stats.totalReads++;
            if (counter < i / 2) {  // Rough check
                Serial.println(F("CONSISTENCY ERROR"));
                return false;
            }
        }
    }
    
    Serial.println(F("OK"));
    return true;
}

bool testPowerLossRecovery() {
    Serial.print(F("Testing power loss recovery... "));
    
    // Save known state
    RuntimeStorage::SystemState state = {
        .operatingMode = 0xAA,
        .activeErrors = 0x5555,
        .lastShutdownReason = 0xBB,
        .uptimeAtShutdown = 0x12345678
    };
    
    if (!fram.saveSystemState(state)) {
        stats.writeErrors++;
        Serial.println(F("SAVE ERROR"));
        return false;
    }
    stats.totalWrites++;
    
    // Save some counters
    fram.setCounter(RuntimeStorage::COUNTER_ERROR_COUNT, 0xDEADBEEF);
    stats.totalWrites++;
    
    // Simulate power loss (just verify data persists)
    delay(10);
    
    // Read back and verify
    RuntimeStorage::SystemState readState;
    if (!fram.loadSystemState(readState)) {
        stats.readErrors++;
        Serial.println(F("LOAD ERROR"));
        return false;
    }
    stats.totalReads++;
    
    if (memcmp(&state, &readState, sizeof(RuntimeStorage::SystemState) - sizeof(uint32_t)) != 0) {
        Serial.println(F("STATE MISMATCH"));
        return false;
    }
    
    uint32_t counter = fram.getCounter(RuntimeStorage::COUNTER_ERROR_COUNT);
    stats.totalReads++;
    if (counter != 0xDEADBEEF) {
        Serial.printf("COUNTER MISMATCH: 0x%08X\n", counter);
        return false;
    }
    
    Serial.println(F("OK"));
    return true;
}

void printStatistics() {
    static uint32_t lastPrint = 0;
    
    if (millis() - lastPrint > 5000) {  // Every 5 seconds
        lastPrint = millis();
        
        uint32_t elapsed = millis() - stats.startTime;
        float hours = elapsed / 3600000.0;
        
        Serial.println(F("\n--- Statistics ---"));
        Serial.printf("Runtime: %.2f hours\n", hours);
        Serial.printf("Total writes: %lu\n", stats.totalWrites);
        Serial.printf("Total reads: %lu\n", stats.totalReads);
        Serial.printf("Write rate: %.1f/sec\n", stats.totalWrites / (elapsed / 1000.0));
        Serial.printf("Read rate: %.1f/sec\n", stats.totalReads / (elapsed / 1000.0));
        Serial.printf("Errors - Write: %lu, Read: %lu, CRC: %lu\n",
                     stats.writeErrors, stats.readErrors, stats.crcErrors);
        
        // Calculate estimated lifetime
        uint64_t totalPossibleWrites = 1e13;  // MB85RC256V spec
        uint64_t writesPerHour = (uint64_t)(stats.totalWrites / hours);
        uint64_t hoursToFailure = totalPossibleWrites / writesPerHour;
        float yearsToFailure = hoursToFailure / (24.0 * 365.0);
        
        Serial.printf("Estimated FRAM lifetime at this rate: %.1f years\n", yearsToFailure);
    }
}

void printFinalReport() {
    stats.testDuration = millis() - stats.startTime;
    
    Serial.println(F("\n\n=== FINAL STRESS TEST REPORT ==="));
    Serial.printf("Test duration: %lu ms (%.2f hours)\n", 
                 stats.testDuration, stats.testDuration / 3600000.0);
    Serial.printf("Total operations: %lu\n", stats.totalWrites + stats.totalReads);
    Serial.printf("Total writes: %lu\n", stats.totalWrites);
    Serial.printf("Total reads: %lu\n", stats.totalReads);
    Serial.printf("Write errors: %lu (%.3f%%)\n", 
                 stats.writeErrors, 100.0 * stats.writeErrors / stats.totalWrites);
    Serial.printf("Read errors: %lu (%.3f%%)\n",
                 stats.readErrors, 100.0 * stats.readErrors / stats.totalReads);
    Serial.printf("CRC/Data errors: %lu\n", stats.crcErrors);
    
    uint32_t totalErrors = stats.writeErrors + stats.readErrors + stats.crcErrors;
    if (totalErrors == 0) {
        Serial.println(F("\nRESULT: ALL TESTS PASSED! ✓"));
    } else {
        Serial.println(F("\nRESULT: TESTS FAILED! ✗"));
    }
    
    // Verify final integrity
    Serial.println(F("\nFinal integrity check..."));
    if (fram.verifyIntegrity()) {
        Serial.println(F("FRAM integrity: OK"));
    } else {
        Serial.println(F("FRAM integrity: FAILED"));
    }
}