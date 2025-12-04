// RuntimeStorage Boiler Controller Example
// Demonstrates integration with a boiler control system

#include <RuntimeStorage.h>
#include <Wire.h>

// I2C pins configured to avoid Ethernet PHY conflicts
#define I2C_SDA_PIN 33
#define I2C_SCL_PIN 32

RuntimeStorage fram;

// Boiler system constants
const uint8_t SENSOR_BOILER_FLOW = 0;
const uint8_t SENSOR_BOILER_RETURN = 1;
const uint8_t SENSOR_OUTSIDE_TEMP = 2;
const uint8_t SENSOR_ROOM_TEMP = 3;

// PID controller IDs
const uint8_t PID_HEATING = 0;
const uint8_t PID_WATER = 1;

// Operating modes
enum OperatingMode {
    MODE_OFF = 0,
    MODE_HEATING = 1,
    MODE_WATER = 2,
    MODE_BOTH = 3,
    MODE_SUMMER = 4
};

// Simulated PID controller
class SimplePID {
public:
    float integral = 0;
    float lastError = 0;
    float output = 0;
    
    float update(float setpoint, float input, float kp, float ki, float kd) {
        float error = setpoint - input;
        integral += error;
        float derivative = error - lastError;
        
        output = kp * error + ki * integral + kd * derivative;
        output = constrain(output, 0, 100);
        
        lastError = error;
        return output;
    }
};

SimplePID heatingPID;
SimplePID waterPID;
OperatingMode currentMode = MODE_OFF;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println(F("\n=== Boiler Controller FRAM Example ==="));
    
    // Initialize I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    
    // Initialize FRAM
    if (!fram.begin(Wire, 0x50)) {
        Serial.println(F("FRAM initialization failed!"));
        while (1) delay(1000);
    }
    
    Serial.println(F("FRAM initialized successfully"));
    
    // Log system startup
    fram.logEvent(RuntimeStorage::EVENT_SYSTEM, 0x01);  // System start
    fram.incrementCounter(RuntimeStorage::COUNTER_BURNER_STARTS);
    
    // Load saved PID states
    RuntimeStorage::PIDState pidState;
    if (fram.loadPIDState(PID_HEATING, pidState)) {
        heatingPID.integral = pidState.integral;
        heatingPID.lastError = pidState.lastError;
        heatingPID.output = pidState.output;
        Serial.println(F("Restored heating PID state"));
    }
    
    if (fram.loadPIDState(PID_WATER, pidState)) {
        waterPID.integral = pidState.integral;
        waterPID.lastError = pidState.lastError;
        waterPID.output = pidState.output;
        Serial.println(F("Restored water PID state"));
    }
    
    // Load system state
    RuntimeStorage::SystemState sysState;
    if (fram.loadSystemState(sysState)) {
        currentMode = (OperatingMode)sysState.operatingMode;
        Serial.printf("Restored operating mode: %d\n", currentMode);
        Serial.printf("Last shutdown reason: %d\n", sysState.lastShutdownReason);
        Serial.printf("Uptime at shutdown: %lu ms\n", sysState.uptimeAtShutdown);
    }
    
    // Show statistics
    showStatistics();
}

void loop() {
    static uint32_t lastPidUpdate = 0;
    static uint32_t lastPidSave = 0;
    static uint32_t lastTempLog = 0;
    static uint32_t lastStatusPrint = 0;
    
    // Simulate temperature readings
    RuntimeStorage::Temperature_t flowTemp = 650 + random(-20, 20);      // 65°C ± 2°C
    RuntimeStorage::Temperature_t returnTemp = 450 + random(-20, 20);    // 45°C ± 2°C
    RuntimeStorage::Temperature_t outsideTemp = 100 + random(-10, 10);   // 10°C ± 1°C
    RuntimeStorage::Temperature_t roomTemp = 200 + random(-5, 5);        // 20°C ± 0.5°C
    
    // Update PID controllers (100ms interval)
    if (millis() - lastPidUpdate > 100) {
        lastPidUpdate = millis();
        
        if (currentMode == MODE_HEATING || currentMode == MODE_BOTH) {
            // Heating PID: maintain flow temperature based on outside temp
            float targetFlow = 70.0 - (outsideTemp / 10.0 * 0.5);  // Simple curve
            heatingPID.update(targetFlow, flowTemp / 10.0, 2.0, 0.1, 0.5);
        }
        
        if (currentMode == MODE_WATER || currentMode == MODE_BOTH) {
            // Water PID: maintain higher temperature for DHW
            waterPID.update(80.0, flowTemp / 10.0, 3.0, 0.2, 0.3);
        }
    }
    
    // Save PID states periodically (every 5 seconds)
    if (millis() - lastPidSave > 5000) {
        lastPidSave = millis();
        
        RuntimeStorage::PIDState pidState;
        
        // Save heating PID
        pidState.integral = heatingPID.integral;
        pidState.lastError = heatingPID.lastError;
        pidState.output = heatingPID.output;
        pidState.lastUpdateTime = millis();
        fram.savePIDState(PID_HEATING, pidState);
        
        // Save water PID
        pidState.integral = waterPID.integral;
        pidState.lastError = waterPID.lastError;
        pidState.output = waterPID.output;
        pidState.lastUpdateTime = millis();
        fram.savePIDState(PID_WATER, pidState);
        
        // Update runtime hours
        float currentHours = fram.getRuntimeHours(RuntimeStorage::RUNTIME_TOTAL);
        fram.updateRuntimeHours(RuntimeStorage::RUNTIME_TOTAL, currentHours + (5.0 / 3600.0));
        
        if (currentMode == MODE_HEATING || currentMode == MODE_BOTH) {
            currentHours = fram.getRuntimeHours(RuntimeStorage::RUNTIME_HEATING);
            fram.updateRuntimeHours(RuntimeStorage::RUNTIME_HEATING, currentHours + (5.0 / 3600.0));
        }
    }
    
    // Log temperatures periodically (every 30 seconds)
    if (millis() - lastTempLog > 30000) {
        lastTempLog = millis();
        
        fram.recordTemperature(SENSOR_BOILER_FLOW, flowTemp);
        fram.recordTemperature(SENSOR_BOILER_RETURN, returnTemp);
        fram.recordTemperature(SENSOR_OUTSIDE_TEMP, outsideTemp);
        fram.recordTemperature(SENSOR_ROOM_TEMP, roomTemp);
        
        Serial.println(F("\nTemperatures logged"));
    }
    
    // Print status (every 10 seconds)
    if (millis() - lastStatusPrint > 10000) {
        lastStatusPrint = millis();
        printStatus(flowTemp, returnTemp, outsideTemp, roomTemp);
    }
    
    // Handle serial commands
    handleSerialCommands();
}

void printStatus(RuntimeStorage::Temperature_t flow, RuntimeStorage::Temperature_t ret,
                RuntimeStorage::Temperature_t outside, RuntimeStorage::Temperature_t room) {
    Serial.println(F("\n=== System Status ==="));
    Serial.printf("Mode: %s\n", getModeName(currentMode));
    Serial.printf("Flow: %.1f°C, Return: %.1f°C\n", flow / 10.0, ret / 10.0);
    Serial.printf("Outside: %.1f°C, Room: %.1f°C\n", outside / 10.0, room / 10.0);
    Serial.printf("Heating output: %.1f%%, Water output: %.1f%%\n", 
                  heatingPID.output, waterPID.output);
    
    float totalHours = fram.getRuntimeHours(RuntimeStorage::RUNTIME_TOTAL);
    float heatingHours = fram.getRuntimeHours(RuntimeStorage::RUNTIME_HEATING);
    float waterHours = fram.getRuntimeHours(RuntimeStorage::RUNTIME_WATER);
    
    Serial.printf("Runtime - Total: %.2fh, Heating: %.2fh, Water: %.2fh\n",
                  totalHours, heatingHours, waterHours);
}

void showStatistics() {
    Serial.println(F("\n=== System Statistics ==="));
    
    // Show counters
    Serial.printf("Burner starts: %lu\n", fram.getCounter(RuntimeStorage::COUNTER_BURNER_STARTS));
    Serial.printf("Heating pump starts: %lu\n", fram.getCounter(RuntimeStorage::COUNTER_HEATING_PUMP_STARTS));
    Serial.printf("Water pump starts: %lu\n", fram.getCounter(RuntimeStorage::COUNTER_WATER_PUMP_STARTS));
    Serial.printf("Error count: %lu\n", fram.getCounter(RuntimeStorage::COUNTER_ERROR_COUNT));
    
    // Show runtime hours
    Serial.printf("Total runtime: %.2f hours\n", fram.getRuntimeHours(RuntimeStorage::RUNTIME_TOTAL));
    Serial.printf("Heating runtime: %.2f hours\n", fram.getRuntimeHours(RuntimeStorage::RUNTIME_HEATING));
    Serial.printf("Water runtime: %.2f hours\n", fram.getRuntimeHours(RuntimeStorage::RUNTIME_WATER));
    Serial.printf("Burner runtime: %.2f hours\n", fram.getRuntimeHours(RuntimeStorage::RUNTIME_BURNER));
    
    // Show events
    Serial.printf("Events logged: %d\n", fram.getEventCount());
    
    // Show recent events
    RuntimeStorage::Event events[10];
    size_t eventCount = fram.getEvents(events, 10);
    if (eventCount > 0) {
        Serial.println(F("\nRecent events:"));
        for (size_t i = 0; i < eventCount; i++) {
            Serial.printf("  [%lu] Type: 0x%02X, Subtype: %d, Data: %d\n",
                         events[i].timestamp, events[i].type, 
                         events[i].subtype, events[i].data);
        }
    }
}

void handleSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case '0':
                setMode(MODE_OFF);
                break;
            case '1':
                setMode(MODE_HEATING);
                break;
            case '2':
                setMode(MODE_WATER);
                break;
            case '3':
                setMode(MODE_BOTH);
                break;
            case 's':
                showStatistics();
                break;
            case 'e':
                showEventLog();
                break;
            case 't':
                showTemperatureHistory();
                break;
            case 'r':
                Serial.println(F("\nResetting FRAM..."));
                fram.reset();
                Serial.println(F("FRAM reset complete"));
                break;
            case 'v':
                Serial.println(F("\nVerifying FRAM integrity..."));
                if (fram.verifyIntegrity()) {
                    Serial.println(F("FRAM integrity OK"));
                } else {
                    Serial.println(F("FRAM integrity check failed!"));
                }
                break;
            case 'h':
            case '?':
                printHelp();
                break;
        }
        
        // Clear any remaining characters
        while (Serial.available()) Serial.read();
    }
}

void setMode(OperatingMode mode) {
    if (mode != currentMode) {
        // Log mode change
        fram.logEvent(RuntimeStorage::EVENT_STATE_CHANGE, 
                     (currentMode << 8) | mode);
        
        currentMode = mode;
        Serial.printf("\nMode changed to: %s\n", getModeName(mode));
        
        // Save system state
        RuntimeStorage::SystemState state = {
            .operatingMode = (uint8_t)mode,
            .activeErrors = 0,
            .lastShutdownReason = 0,
            .uptimeAtShutdown = millis()
        };
        fram.saveSystemState(state);
        
        // Update pump counters
        if (mode == MODE_HEATING || mode == MODE_BOTH) {
            fram.incrementCounter(RuntimeStorage::COUNTER_HEATING_PUMP_STARTS);
        }
        if (mode == MODE_WATER || mode == MODE_BOTH) {
            fram.incrementCounter(RuntimeStorage::COUNTER_WATER_PUMP_STARTS);
        }
    }
}

void showEventLog() {
    Serial.println(F("\n=== Event Log ==="));
    
    RuntimeStorage::Event events[20];
    size_t count = fram.getEvents(events, 20);
    
    for (size_t i = 0; i < count; i++) {
        Serial.printf("[%lu] ", events[i].timestamp);
        
        switch (events[i].type) {
            case RuntimeStorage::EVENT_ERROR:
                Serial.print(F("ERROR"));
                break;
            case RuntimeStorage::EVENT_WARNING:
                Serial.print(F("WARNING"));
                break;
            case RuntimeStorage::EVENT_STATE_CHANGE:
                Serial.print(F("STATE"));
                break;
            case RuntimeStorage::EVENT_USER_ACTION:
                Serial.print(F("USER"));
                break;
            case RuntimeStorage::EVENT_SYSTEM:
                Serial.print(F("SYSTEM"));
                break;
            default:
                Serial.printf("0x%02X", events[i].type);
        }
        
        Serial.printf(" sub:%d data:%d\n", events[i].subtype, events[i].data);
    }
}

void showTemperatureHistory() {
    Serial.println(F("\n=== Temperature History ==="));
    
    const char* sensorNames[] = {"Flow", "Return", "Outside", "Room"};
    
    for (uint8_t sensor = 0; sensor < 4; sensor++) {
        Serial.printf("\n%s temperature:\n", sensorNames[sensor]);
        
        RuntimeStorage::TempReading readings[10];
        size_t count = fram.getTemperatureHistory(sensor, readings, 10);
        
        for (size_t i = 0; i < count; i++) {
            Serial.printf("  [%lu] %.1f°C\n", 
                         readings[i].timestamp, readings[i].temperature / 10.0);
        }
    }
}

const char* getModeName(OperatingMode mode) {
    switch (mode) {
        case MODE_OFF: return "OFF";
        case MODE_HEATING: return "HEATING";
        case MODE_WATER: return "WATER";
        case MODE_BOTH: return "HEATING+WATER";
        case MODE_SUMMER: return "SUMMER";
        default: return "UNKNOWN";
    }
}

void printHelp() {
    Serial.println(F("\n=== Commands ==="));
    Serial.println(F("0 - Set mode OFF"));
    Serial.println(F("1 - Set mode HEATING"));
    Serial.println(F("2 - Set mode WATER"));
    Serial.println(F("3 - Set mode BOTH"));
    Serial.println(F("s - Show statistics"));
    Serial.println(F("e - Show event log"));
    Serial.println(F("t - Show temperature history"));
    Serial.println(F("r - Reset FRAM (preserves counters)"));
    Serial.println(F("v - Verify FRAM integrity"));
    Serial.println(F("h/? - Show this help"));
}