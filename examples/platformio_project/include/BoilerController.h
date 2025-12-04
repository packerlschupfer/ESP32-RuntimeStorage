/**
 * BoilerController.h
 * Simplified boiler controller interface for RuntimeStorage example
 */

#ifndef BOILER_CONTROLLER_H
#define BOILER_CONTROLLER_H

#include <Arduino.h>
#include <RuntimeStorage.h>
#include "config/SystemConfig.h"

class BoilerController {
public:
    BoilerController();
    
    // Initialization
    bool begin(RuntimeStorage::RuntimeStorage* storage, SemaphoreHandle_t* i2cMutex);
    
    // Main update function
    void update();
    
    // Sensor functions
    void updateSensorReadings();
    float getFlowTemperature() const { return _flowTemp; }
    float getReturnTemperature() const { return _returnTemp; }
    float getOutsideTemperature() const { return _outsideTemp; }
    float getRoomTemperature() const { return _roomTemp; }
    
    // Control functions
    void setOperatingMode(uint8_t mode);
    uint8_t getOperatingMode() const { return _operatingMode; }
    const char* getModeName() const;
    
    // PID functions
    bool getPIDState(uint8_t controllerId, RuntimeStorage::PIDState& state) const;
    
    // Error handling
    bool hasError() const { return _activeErrors != 0; }
    bool isCriticalError() const { return (_activeErrors & CRITICAL_ERROR_MASK) != 0; }
    uint16_t getActiveErrors() const { return _activeErrors; }
    uint32_t getErrorCode() const;
    const char* getErrorMessage() const;
    
private:
    // References
    RuntimeStorage::RuntimeStorage* _storage;
    SemaphoreHandle_t* _i2cMutex;
    
    // State variables
    uint8_t _operatingMode;
    uint16_t _activeErrors;
    uint32_t _lastErrorCode;
    
    // Temperatures (°C)
    float _flowTemp;
    float _returnTemp;
    float _outsideTemp;
    float _roomTemp;
    
    // PID controllers (simplified)
    struct SimplePID {
        float integral;
        float lastError;
        float output;
        uint32_t lastUpdateTime;
        
        void reset() {
            integral = 0;
            lastError = 0;
            output = 0;
            lastUpdateTime = 0;
        }
        
        float update(float setpoint, float input, float kp, float ki, float kd, uint32_t dt);
    };
    
    SimplePID _pidControllers[4];
    
    // Timing
    uint32_t _lastSensorRead;
    uint32_t _lastControlUpdate;
    uint32_t _burnerStartTime;
    uint32_t _burnerStopTime;
    
    // States
    bool _burnerOn;
    bool _heatingPumpOn;
    bool _waterPumpOn;
    
    // Private methods
    void runControlLogic();
    void updatePIDControllers();
    void checkSafetyConditions();
    void updateOutputs();
    float simulateTemperature(float current, float target, float rate);
    void logStateChange(uint8_t oldMode, uint8_t newMode);
};

// Inline implementation of PID update
inline float BoilerController::SimplePID::update(float setpoint, float input, 
                                                 float kp, float ki, float kd, 
                                                 uint32_t dt) {
    float error = setpoint - input;
    
    // Proportional term
    float P = kp * error;
    
    // Integral term with anti-windup
    integral += error * (dt / 1000.0f);
    integral = constrain(integral, -100.0f / ki, 100.0f / ki);
    float I = ki * integral;
    
    // Derivative term
    float derivative = 0;
    if (dt > 0) {
        derivative = (error - lastError) / (dt / 1000.0f);
    }
    float D = kd * derivative;
    
    // Calculate output
    output = P + I + D;
    output = constrain(output, 0, 100);
    
    // Store for next iteration
    lastError = error;
    lastUpdateTime = millis();
    
    return output;
}

#endif // BOILER_CONTROLLER_H