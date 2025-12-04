// RuntimeStorage Basic Example
// Demonstrates basic FRAM operations

#include <RuntimeStorage.h>
#include <Wire.h>

// I2C pins configured to avoid Ethernet PHY conflicts
#define I2C_SDA_PIN 33
#define I2C_SCL_PIN 32

RuntimeStorage fram;

// Example sensor IDs
const uint8_t SENSOR_BOILER_OUTPUT = 0;
const uint8_t SENSOR_BOILER_RETURN = 1;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println(F("\n=== RuntimeStorage Basic Example ==="));
    
    // Initialize I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    
    // Initialize FRAM
    if (!fram.begin(Wire, 0x50)) {
        Serial.println(F("Failed to initialize FRAM!"));
        Serial.println(F("Check wiring:"));
        Serial.println(F("  SDA -> GPIO 33"));
        Serial.println(F("  SCL -> GPIO 32"));
        Serial.println(F("  VCC -> 3.3V"));
        Serial.println(F("  GND -> GND"));
        while (1) delay(1000);
    }
    
    Serial.printf("FRAM initialized successfully!\n");
    Serial.printf("Size: %lu bytes\n", fram.getSize());
    Serial.printf("Version: %d\n", fram.getVersion());
    
    // Print memory map
    fram.printMemoryMap();
    
    // Log startup event
    Serial.println(F("\nLogging startup event..."));
    fram.logEvent(RuntimeStorage::EVENT_SYSTEM, 0x01);
    
    // Save some test data
    Serial.println(F("\nSaving test data..."));
    
    // Save a counter
    uint32_t startCount = fram.getCounter(RuntimeStorage::COUNTER_BURNER_STARTS);
    Serial.printf("Current burner starts: %lu\n", startCount);
    fram.incrementCounter(RuntimeStorage::COUNTER_BURNER_STARTS);
    Serial.printf("After increment: %lu\n", fram.getCounter(RuntimeStorage::COUNTER_BURNER_STARTS));
    
    // Save runtime hours
    float hours = fram.getRuntimeHours(RuntimeStorage::RUNTIME_TOTAL);
    Serial.printf("Total runtime: %.2f hours\n", hours);
    fram.updateRuntimeHours(RuntimeStorage::RUNTIME_TOTAL, hours + 0.1);
    
    // Save a temperature reading
    RuntimeStorage::Temperature_t temp = 652;  // 65.2°C
    fram.recordTemperature(SENSOR_BOILER_OUTPUT, temp);
    Serial.printf("Recorded temperature: %.1f°C\n", temp / 10.0);
    
    // Save PID state
    RuntimeStorage::PIDState pidState = {
        .integral = 123.45f,
        .lastError = -2.5f,
        .output = 75.0f,
        .lastUpdateTime = millis()
    };
    fram.savePIDState(0, pidState);
    Serial.println(F("PID state saved"));
    
    // Save system state
    RuntimeStorage::SystemState sysState = {
        .operatingMode = 1,      // Heating mode
        .activeErrors = 0,
        .lastShutdownReason = 0,
        .uptimeAtShutdown = millis()
    };
    fram.saveSystemState(sysState);
    Serial.println(F("System state saved"));
}

void loop() {
    static uint32_t lastUpdate = 0;
    static uint32_t lastTempLog = 0;
    
    // Every 5 seconds, show some status
    if (millis() - lastUpdate > 5000) {
        lastUpdate = millis();
        
        Serial.println(F("\n--- Status Update ---"));
        
        // Show event count
        Serial.printf("Events logged: %d\n", fram.getEventCount());
        
        // Show latest temperature
        RuntimeStorage::TempReading lastTemp;
        if (fram.getLatestTemperature(SENSOR_BOILER_OUTPUT, lastTemp)) {
            Serial.printf("Last temperature: %.1f°C at %lu ms\n", 
                         lastTemp.temperature / 10.0, lastTemp.timestamp);
        }
        
        // Show runtime
        float totalHours = fram.getRuntimeHours(RuntimeStorage::RUNTIME_TOTAL);
        Serial.printf("Total runtime: %.2f hours\n", totalHours);
        
        // Load and verify PID state
        RuntimeStorage::PIDState loadedPid;
        if (fram.loadPIDState(0, loadedPid)) {
            Serial.printf("PID integral: %.2f\n", loadedPid.integral);
        }
    }
    
    // Every 10 seconds, log a temperature
    if (millis() - lastTempLog > 10000) {
        lastTempLog = millis();
        
        // Simulate temperature reading
        RuntimeStorage::Temperature_t temp = 600 + random(-50, 50);  // 60°C ± 5°C
        fram.recordTemperature(SENSOR_BOILER_OUTPUT, temp);
        Serial.printf("\nLogged temperature: %.1f°C\n", temp / 10.0);
        
        // Occasionally log an event
        if (random(0, 3) == 0) {
            RuntimeStorage::EventType types[] = {
                RuntimeStorage::EVENT_WARNING,
                RuntimeStorage::EVENT_STATE_CHANGE,
                RuntimeStorage::EVENT_USER_ACTION
            };
            RuntimeStorage::EventType type = types[random(0, 3)];
            fram.logEvent(type, random(0, 100));
            Serial.printf("Logged event type 0x%02X\n", type);
        }
    }
}