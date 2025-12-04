/**
 * RuntimeStorage PlatformIO Example
 * ESP32 Boiler Controller Integration
 * 
 * This example demonstrates how to integrate RuntimeStorage
 * with a boiler controller system using PlatformIO
 */

#include <Arduino.h>
#include <Wire.h>
#include <RuntimeStorage.h>
#include "config/SystemConfig.h"
#include "BoilerController.h"

// Create global instances
RuntimeStorage::RuntimeStorage storage;
BoilerController boilerController;

// Task handles
TaskHandle_t sensorTaskHandle = nullptr;
TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t storageTaskHandle = nullptr;

// Mutex for thread-safe access
SemaphoreHandle_t i2cMutex = nullptr;

// Forward declarations
void sensorTask(void* parameter);
void controlTask(void* parameter);
void storageTask(void* parameter);
void handleSerialCommands();
void printSystemStatus();
void showErrors();
void showTemperatureHistory();
void printHelp();

void setup() {
    // Initialize serial
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        delay(10);
    }
    
    Serial.println(F("\n=== ESP32 Boiler Controller with RuntimeStorage ==="));
    Serial.printf("Firmware Version: %s\n", FIRMWARE_VERSION);
    Serial.printf("Build Date: %s %s\n", __DATE__, __TIME__);
    
    // Create mutex for I2C access
    i2cMutex = xSemaphoreCreateMutex();
    if (!i2cMutex) {
        Serial.println(F("ERROR: Failed to create I2C mutex!"));
        esp_restart();
    }
    
    // Initialize I2C with mutex protection
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY);
        xSemaphoreGive(i2cMutex);
    }
    
    // Initialize RuntimeStorage
    Serial.println(F("Initializing RuntimeStorage..."));
    if (!storage.begin(Wire, FRAM_I2C_ADDRESS)) {
        Serial.println(F("ERROR: Failed to initialize RuntimeStorage!"));
        Serial.println(F("Check connections:"));
        Serial.printf("  SDA -> GPIO %d\n", I2C_SDA_PIN);
        Serial.printf("  SCL -> GPIO %d\n", I2C_SCL_PIN);
        Serial.println(F("  VCC -> 3.3V"));
        Serial.println(F("  GND -> GND"));
        
        // Log critical error before restart
        storage.logCriticalError(0xFFFF, "FRAM init failed", "Setup");
        delay(5000);
        esp_restart();
    }
    
    Serial.println(F("RuntimeStorage initialized successfully"));
    Serial.printf("FRAM Size: %lu bytes\n", storage.getSize());
    Serial.printf("Free Space: %lu bytes\n", storage.getFreeSpace());
    
    // Show memory map
    storage.printMemoryMap();
    
    // Log system startup
    storage.logEvent(RuntimeStorage::EVENT_SYSTEM, 0x01);  // System start
    storage.incrementCounter(RuntimeStorage::COUNTER_SYSTEM_RESTARTS);
    
    // Initialize boiler controller
    Serial.println(F("\nInitializing Boiler Controller..."));
    if (!boilerController.begin(&storage, &i2cMutex)) {
        Serial.println(F("ERROR: Failed to initialize Boiler Controller!"));
        storage.logCriticalError(0xFFFE, "Controller init failed", "Setup");
        delay(5000);
        esp_restart();
    }
    
    // Restore system state
    RuntimeStorage::SystemState sysState;
    if (storage.loadSystemState(sysState)) {
        Serial.println(F("\nRestored system state:"));
        Serial.printf("  Operating Mode: %d\n", sysState.operatingMode);
        Serial.printf("  Active Errors: 0x%04X\n", sysState.activeErrors);
        Serial.printf("  Last Shutdown: %d\n", sysState.lastShutdownReason);
        Serial.printf("  Uptime at shutdown: %lu ms\n", sysState.uptimeAtShutdown);
        
        boilerController.setOperatingMode(sysState.operatingMode);
    }
    
    // Show statistics
    Serial.println(F("\nSystem Statistics:"));
    Serial.printf("  System Restarts: %lu\n", 
        storage.getCounter(RuntimeStorage::COUNTER_SYSTEM_RESTARTS));
    Serial.printf("  Total Runtime: %.2f hours\n", 
        storage.getRuntimeHours(RuntimeStorage::RUNTIME_TOTAL));
    Serial.printf("  Error Count: %lu\n", 
        storage.getCounter(RuntimeStorage::COUNTER_ERROR_COUNT));
    
    // Check for errors
    RuntimeStorage::ErrorStats errorStats;
    if (storage.getErrorStats(errorStats)) {
        if (errorStats.criticalErrors > 0) {
            Serial.printf("\nWARNING: %lu critical errors detected!\n", 
                errorStats.criticalErrors);
            
            // Show last few critical errors
            RuntimeStorage::ErrorEntry errors[3];
            size_t count = storage.getCriticalErrors(errors, 3);
            for (size_t i = 0; i < count; i++) {
                Serial.printf("  [%lu] Code: 0x%04X - %s\n", 
                    errors[i].timestamp, errors[i].errorCode, errors[i].message);
            }
        }
    }
    
    // Create FreeRTOS tasks
    Serial.println(F("\nCreating tasks..."));
    
    xTaskCreatePinnedToCore(
        sensorTask,           // Task function
        "SensorTask",         // Name
        4096,                 // Stack size
        nullptr,              // Parameters
        2,                    // Priority
        &sensorTaskHandle,    // Handle
        1                     // Core
    );
    
    xTaskCreatePinnedToCore(
        controlTask,
        "ControlTask",
        4096,
        nullptr,
        3,
        &controlTaskHandle,
        1
    );
    
    xTaskCreatePinnedToCore(
        storageTask,
        "StorageTask",
        3072,
        nullptr,
        1,
        &storageTaskHandle,
        0
    );
    
    Serial.println(F("\nSystem initialization complete!"));
    Serial.println(F("Type 'h' for help\n"));
}

void loop() {
    // Main loop handles serial commands and periodic status
    static uint32_t lastStatusPrint = 0;
    
    // Handle serial commands
    handleSerialCommands();
    
    // Print status every 30 seconds
    if (millis() - lastStatusPrint > 30000) {
        lastStatusPrint = millis();
        printSystemStatus();
    }
    
    // Small delay to prevent watchdog
    delay(10);
}

// Sensor reading task
void sensorTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);  // 1 Hz
    
    while (true) {
        // Take I2C mutex
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100))) {
            // Read all temperature sensors
            boilerController.updateSensorReadings();
            
            // Release mutex
            xSemaphoreGive(i2cMutex);
            
            // Log temperatures to FRAM periodically
            static uint32_t lastTempLog = 0;
            if (millis() - lastTempLog > 60000) {  // Every minute
                lastTempLog = millis();
                
                // Get temperatures from controller
                float flowTemp = boilerController.getFlowTemperature();
                float returnTemp = boilerController.getReturnTemperature();
                float outsideTemp = boilerController.getOutsideTemperature();
                float roomTemp = boilerController.getRoomTemperature();
                
                // Convert to fixed-point and store
                storage.recordTemperature(0, (int16_t)(flowTemp * 10));
                storage.recordTemperature(1, (int16_t)(returnTemp * 10));
                storage.recordTemperature(2, (int16_t)(outsideTemp * 10));
                storage.recordTemperature(3, (int16_t)(roomTemp * 10));
            }
        }
        
        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Control logic task
void controlTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10 Hz
    
    while (true) {
        // Run control logic
        boilerController.update();
        
        // Check for safety conditions
        if (boilerController.hasError()) {
            uint32_t errorCode = boilerController.getErrorCode();
            const char* errorMsg = boilerController.getErrorMessage();
            
            // Log to FRAM
            if (boilerController.isCriticalError()) {
                storage.logCriticalError(errorCode, errorMsg, "Control");
            } else {
                storage.logError(errorCode, errorMsg, "Control");
            }
        }
        
        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Persistent storage task
void storageTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5000);  // 0.2 Hz
    
    while (true) {
        // Save PID states
        for (uint8_t i = 0; i < 4; i++) {
            RuntimeStorage::PIDState pidState;
            if (boilerController.getPIDState(i, pidState)) {
                storage.savePIDState(i, pidState);
            }
        }
        
        // Update runtime hours
        float currentHours = storage.getRuntimeHours(RuntimeStorage::RUNTIME_TOTAL);
        storage.updateRuntimeHours(RuntimeStorage::RUNTIME_TOTAL, 
            currentHours + (5.0 / 3600.0));  // Add 5 seconds
        
        // Update mode-specific runtime
        uint8_t mode = boilerController.getOperatingMode();
        if (mode == 1 || mode == 3) {  // Heating or Both
            currentHours = storage.getRuntimeHours(RuntimeStorage::RUNTIME_HEATING);
            storage.updateRuntimeHours(RuntimeStorage::RUNTIME_HEATING, 
                currentHours + (5.0 / 3600.0));
        }
        if (mode == 2 || mode == 3) {  // Water or Both
            currentHours = storage.getRuntimeHours(RuntimeStorage::RUNTIME_WATER);
            storage.updateRuntimeHours(RuntimeStorage::RUNTIME_WATER, 
                currentHours + (5.0 / 3600.0));
        }
        
        // Save system state
        RuntimeStorage::SystemState sysState = {
            .operatingMode = mode,
            .activeErrors = boilerController.getActiveErrors(),
            .lastShutdownReason = 0,  // Normal operation
            .uptimeAtShutdown = millis()
        };
        storage.saveSystemState(sysState);
        
        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void handleSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case '0':  // Off
                boilerController.setOperatingMode(0);
                storage.logEvent(RuntimeStorage::EVENT_USER_ACTION, 0);
                Serial.println(F("Mode: OFF"));
                break;
                
            case '1':  // Heating
                boilerController.setOperatingMode(1);
                storage.logEvent(RuntimeStorage::EVENT_USER_ACTION, 1);
                storage.incrementCounter(RuntimeStorage::COUNTER_HEATING_PUMP_STARTS);
                Serial.println(F("Mode: HEATING"));
                break;
                
            case '2':  // Hot Water
                boilerController.setOperatingMode(2);
                storage.logEvent(RuntimeStorage::EVENT_USER_ACTION, 2);
                storage.incrementCounter(RuntimeStorage::COUNTER_WATER_PUMP_STARTS);
                Serial.println(F("Mode: HOT WATER"));
                break;
                
            case '3':  // Both
                boilerController.setOperatingMode(3);
                storage.logEvent(RuntimeStorage::EVENT_USER_ACTION, 3);
                storage.incrementCounter(RuntimeStorage::COUNTER_HEATING_PUMP_STARTS);
                storage.incrementCounter(RuntimeStorage::COUNTER_WATER_PUMP_STARTS);
                Serial.println(F("Mode: HEATING + HOT WATER"));
                break;
                
            case 's':  // Status
                printSystemStatus();
                break;
                
            case 'e':  // Show errors
                showErrors();
                break;
                
            case 't':  // Show temperature history
                showTemperatureHistory();
                break;
                
            case 'c':  // Clear errors
                storage.clearErrors();
                Serial.println(F("Errors cleared"));
                break;
                
            case 'r':  // Reset FRAM
                Serial.println(F("Resetting FRAM..."));
                storage.reset();
                Serial.println(F("FRAM reset complete"));
                esp_restart();
                break;
                
            case 'v':  // Verify integrity
                if (storage.verifyIntegrity()) {
                    Serial.println(F("FRAM integrity: OK"));
                } else {
                    Serial.println(F("FRAM integrity: FAILED"));
                }
                break;
                
            case 'h':  // Help
            case '?':
                printHelp();
                break;
        }
        
        // Clear buffer
        while (Serial.available()) Serial.read();
    }
}

void printSystemStatus() {
    Serial.println(F("\n=== System Status ==="));
    Serial.printf("Uptime: %lu seconds\n", millis() / 1000);
    Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("Mode: %s\n", boilerController.getModeName());
    
    // Temperatures
    Serial.println(F("\nTemperatures:"));
    Serial.printf("  Flow: %.1f°C\n", boilerController.getFlowTemperature());
    Serial.printf("  Return: %.1f°C\n", boilerController.getReturnTemperature());
    Serial.printf("  Outside: %.1f°C\n", boilerController.getOutsideTemperature());
    Serial.printf("  Room: %.1f°C\n", boilerController.getRoomTemperature());
    
    // Runtime
    Serial.println(F("\nRuntime Hours:"));
    Serial.printf("  Total: %.2f\n", storage.getRuntimeHours(RuntimeStorage::RUNTIME_TOTAL));
    Serial.printf("  Heating: %.2f\n", storage.getRuntimeHours(RuntimeStorage::RUNTIME_HEATING));
    Serial.printf("  Water: %.2f\n", storage.getRuntimeHours(RuntimeStorage::RUNTIME_WATER));
    Serial.printf("  Burner: %.2f\n", storage.getRuntimeHours(RuntimeStorage::RUNTIME_BURNER));
    
    // Counters
    Serial.println(F("\nCounters:"));
    Serial.printf("  Burner Starts: %lu\n", 
        storage.getCounter(RuntimeStorage::COUNTER_BURNER_STARTS));
    Serial.printf("  System Restarts: %lu\n", 
        storage.getCounter(RuntimeStorage::COUNTER_SYSTEM_RESTARTS));
    Serial.printf("  Error Count: %lu\n", 
        storage.getCounter(RuntimeStorage::COUNTER_ERROR_COUNT));
    
    // Task status
    Serial.println(F("\nTask Status:"));
    Serial.printf("  Sensor: %s\n", 
        eTaskGetState(sensorTaskHandle) == eRunning ? "Running" : "Not Running");
    Serial.printf("  Control: %s\n", 
        eTaskGetState(controlTaskHandle) == eRunning ? "Running" : "Not Running");
    Serial.printf("  Storage: %s\n", 
        eTaskGetState(storageTaskHandle) == eRunning ? "Running" : "Not Running");
}

void showErrors() {
    Serial.println(F("\n=== Error Log ==="));
    
    // Show statistics
    RuntimeStorage::ErrorStats stats;
    if (storage.getErrorStats(stats)) {
        Serial.printf("Total Errors: %lu\n", stats.totalErrors);
        Serial.printf("Critical Errors: %lu\n", stats.criticalErrors);
        
        if (stats.lastErrorTime > 0) {
            Serial.printf("Last Error: %lu seconds ago\n", 
                (millis() - stats.lastErrorTime) / 1000);
        }
    }
    
    // Show recent errors
    Serial.println(F("\nRecent Errors:"));
    size_t errorCount = storage.getErrorCount();
    size_t showCount = min(errorCount, (size_t)10);
    
    for (size_t i = 0; i < showCount; i++) {
        RuntimeStorage::ErrorEntry entry;
        if (storage.getError(i, entry)) {
            Serial.printf("[%lu] Code: 0x%04X", 
                (millis() - entry.timestamp) / 1000, entry.errorCode);
            
            if (strlen(entry.message) > 0) {
                Serial.printf(" - %s", entry.message);
            }
            if (strlen(entry.context) > 0) {
                Serial.printf(" (%s)", entry.context);
            }
            Serial.println();
        }
    }
    
    // Show critical errors
    if (stats.criticalErrors > 0) {
        Serial.println(F("\nCritical Errors:"));
        RuntimeStorage::ErrorEntry criticalErrors[5];
        size_t count = storage.getCriticalErrors(criticalErrors, 5);
        
        for (size_t i = 0; i < count; i++) {
            Serial.printf("[%lu] Code: 0x%04X - %s\n", 
                (millis() - criticalErrors[i].timestamp) / 1000,
                criticalErrors[i].errorCode,
                criticalErrors[i].message);
        }
    }
}

void showTemperatureHistory() {
    Serial.println(F("\n=== Temperature History ==="));
    
    const char* sensorNames[] = {"Flow", "Return", "Outside", "Room"};
    
    for (uint8_t sensor = 0; sensor < 4; sensor++) {
        Serial.printf("\n%s Temperature:\n", sensorNames[sensor]);
        
        RuntimeStorage::TempReading readings[10];
        size_t count = storage.getTemperatureHistory(sensor, readings, 10);
        
        for (size_t i = 0; i < count; i++) {
            Serial.printf("  [%lu sec ago] %.1f°C\n", 
                (millis() - readings[i].timestamp) / 1000,
                readings[i].temperature / 10.0);
        }
    }
}

void printHelp() {
    Serial.println(F("\n=== Commands ==="));
    Serial.println(F("0 - Set mode OFF"));
    Serial.println(F("1 - Set mode HEATING"));
    Serial.println(F("2 - Set mode HOT WATER"));
    Serial.println(F("3 - Set mode HEATING + HOT WATER"));
    Serial.println(F("s - Show system status"));
    Serial.println(F("e - Show error log"));
    Serial.println(F("t - Show temperature history"));
    Serial.println(F("c - Clear errors"));
    Serial.println(F("r - Reset FRAM (requires restart)"));
    Serial.println(F("v - Verify FRAM integrity"));
    Serial.println(F("h/? - Show this help"));
}