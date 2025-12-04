// RuntimeStorage Error Logging Example
// Demonstrates error logging and retrieval functionality

#include <Arduino.h>
#include <Wire.h>
#include <RuntimeStorage.h>

// I2C pins configured to avoid Ethernet PHY conflicts
#define I2C_SDA_PIN 33
#define I2C_SCL_PIN 32

// Create storage instance
RuntimeStorage::RuntimeStorage storage;

// Error codes for demo
enum ErrorCode {
    ERR_SENSOR_TIMEOUT = 0x1001,
    ERR_TEMP_OUT_OF_RANGE = 0x1002,
    ERR_COMM_FAILURE = 0x1003,
    ERR_WATCHDOG_RESET = 0x2001,  // Critical
    ERR_OVERHEAT = 0x2002,         // Critical
    ERR_POWER_LOSS = 0x2003        // Critical
};

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    
    Serial.println(F("\n=== RuntimeStorage Error Logging Example ==="));
    
    // Initialize I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
    
    // Initialize storage
    if (!storage.begin(Wire)) {
        Serial.println(F("ERROR: Failed to initialize RuntimeStorage!"));
        while (1) { delay(1000); }
    }
    
    Serial.println(F("RuntimeStorage initialized successfully"));
    
    // Show current error stats
    showErrorStats();
    
    // Demonstrate error logging
    demonstrateErrorLogging();
    
    // Show updated stats
    showErrorStats();
    
    // Demonstrate error retrieval
    demonstrateErrorRetrieval();
    
    // Demonstrate critical error handling
    demonstrateCriticalErrors();
    
    // Test error clearing
    testErrorClearing();
}

void loop() {
    // Simulate periodic errors
    static uint32_t lastErrorTime = 0;
    static uint32_t errorCount = 0;
    
    if (millis() - lastErrorTime > 5000) {
        lastErrorTime = millis();
        errorCount++;
        
        // Log a random error
        switch (errorCount % 4) {
            case 0:
                storage.logError(ERR_SENSOR_TIMEOUT, "DS18B20 timeout", "Sensor 1");
                Serial.println(F("Logged sensor timeout error"));
                break;
                
            case 1:
                storage.logError(ERR_TEMP_OUT_OF_RANGE, "Temp: 125.5C", "Boiler");
                Serial.println(F("Logged temperature range error"));
                break;
                
            case 2:
                storage.logError(ERR_COMM_FAILURE, "ModBus CRC fail", "VFD");
                Serial.println(F("Logged communication error"));
                break;
                
            case 3:
                // Every 4th error is critical
                storage.logCriticalError(ERR_OVERHEAT, "Shutdown at 95C", "Safety");
                Serial.println(F("Logged CRITICAL overheat error!"));
                break;
        }
        
        // Show count
        Serial.print(F("Total errors logged: "));
        Serial.println(storage.getErrorCount());
    }
}

void showErrorStats() {
    Serial.println(F("\n--- Current Error Statistics ---"));
    
    RuntimeStorage::ErrorStats stats;
    if (storage.getErrorStats(stats)) {
        Serial.print(F("Total errors: "));
        Serial.println(stats.totalErrors);
        
        Serial.print(F("Critical errors: "));
        Serial.println(stats.criticalErrors);
        
        Serial.print(F("Unique error codes: "));
        Serial.println(stats.uniqueErrors);
        
        if (stats.lastErrorTime > 0) {
            Serial.print(F("Last error: "));
            Serial.print((millis() - stats.lastErrorTime) / 1000);
            Serial.println(F(" seconds ago"));
        }
        
        if (stats.oldestErrorTime > 0) {
            Serial.print(F("Oldest error: "));
            Serial.print((millis() - stats.oldestErrorTime) / 1000);
            Serial.println(F(" seconds ago"));
        }
    } else {
        Serial.println(F("Failed to read error stats"));
    }
}

void demonstrateErrorLogging() {
    Serial.println(F("\n--- Demonstrating Error Logging ---"));
    
    // Log various errors
    storage.logError(ERR_SENSOR_TIMEOUT, "No response from sensor");
    Serial.println(F("Logged: Sensor timeout"));
    delay(100);
    
    storage.logError(ERR_TEMP_OUT_OF_RANGE, "Temp reading: -127.0");
    Serial.println(F("Logged: Temperature out of range"));
    delay(100);
    
    storage.logCriticalError(ERR_WATCHDOG_RESET, "WDT reset detected", "System");
    Serial.println(F("Logged: CRITICAL - Watchdog reset"));
    delay(100);
    
    storage.logError(ERR_COMM_FAILURE, "I2C bus stuck");
    Serial.println(F("Logged: Communication failure"));
    delay(100);
    
    storage.logCriticalError(ERR_POWER_LOSS, "Brownout detected", "Power");
    Serial.println(F("Logged: CRITICAL - Power loss"));
    
    Serial.print(F("\nTotal errors logged: "));
    Serial.println(storage.getErrorCount());
}

void demonstrateErrorRetrieval() {
    Serial.println(F("\n--- Recent Errors (newest first) ---"));
    
    size_t errorCount = storage.getErrorCount();
    size_t displayCount = min(errorCount, (size_t)10);
    
    for (size_t i = 0; i < displayCount; i++) {
        RuntimeStorage::ErrorEntry entry;
        if (storage.getError(i, entry)) {
            Serial.print(F("\n["));
            Serial.print(i);
            Serial.print(F("] Code: 0x"));
            Serial.print(entry.errorCode, HEX);
            
            Serial.print(F(" | Time: "));
            Serial.print((millis() - entry.timestamp) / 1000);
            Serial.print(F("s ago"));
            
            if (entry.count > 1) {
                Serial.print(F(" | Count: "));
                Serial.print(entry.count);
            }
            
            if (strlen(entry.message) > 0) {
                Serial.print(F("\n    Message: "));
                Serial.print(entry.message);
            }
            
            if (strlen(entry.context) > 0) {
                Serial.print(F(" | Context: "));
                Serial.print(entry.context);
            }
        }
    }
    Serial.println();
}

void demonstrateCriticalErrors() {
    Serial.println(F("\n--- Critical Errors Only ---"));
    
    RuntimeStorage::ErrorEntry criticalErrors[5];
    size_t count = storage.getCriticalErrors(criticalErrors, 5);
    
    if (count == 0) {
        Serial.println(F("No critical errors found"));
    } else {
        Serial.print(F("Found "));
        Serial.print(count);
        Serial.println(F(" critical errors:"));
        
        for (size_t i = 0; i < count; i++) {
            Serial.print(F("\n["));
            Serial.print(i);
            Serial.print(F("] Code: 0x"));
            Serial.print(criticalErrors[i].errorCode, HEX);
            
            Serial.print(F(" | Time: "));
            Serial.print((millis() - criticalErrors[i].timestamp) / 1000);
            Serial.print(F("s ago"));
            
            if (strlen(criticalErrors[i].message) > 0) {
                Serial.print(F("\n    Message: "));
                Serial.print(criticalErrors[i].message);
            }
        }
        Serial.println();
    }
}

void testErrorClearing() {
    Serial.println(F("\n--- Testing Error Clear ---"));
    
    Serial.print(F("Errors before clear: "));
    Serial.println(storage.getErrorCount());
    
    // Clear all errors
    if (storage.clearErrors()) {
        Serial.println(F("All errors cleared successfully"));
    } else {
        Serial.println(F("Failed to clear errors"));
    }
    
    Serial.print(F("Errors after clear: "));
    Serial.println(storage.getErrorCount());
    
    // Check stats after clear
    RuntimeStorage::ErrorStats stats;
    if (storage.getErrorStats(stats)) {
        Serial.print(F("Stats - Total: "));
        Serial.print(stats.totalErrors);
        Serial.print(F(", Critical: "));
        Serial.println(stats.criticalErrors);
    }
}