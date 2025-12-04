/**
 * BoilerController.cpp
 * Implementation of simplified boiler controller for RuntimeStorage example
 */

#include "BoilerController.h"

BoilerController::BoilerController() 
    : _storage(nullptr)
    , _i2cMutex(nullptr)
    , _operatingMode(MODE_OFF)
    , _activeErrors(0)
    , _lastErrorCode(0)
    , _flowTemp(20.0f)
    , _returnTemp(20.0f)
    , _outsideTemp(10.0f)
    , _roomTemp(20.0f)
    , _lastSensorRead(0)
    , _lastControlUpdate(0)
    , _burnerStartTime(0)
    , _burnerStopTime(0)
    , _burnerOn(false)
    , _heatingPumpOn(false)
    , _waterPumpOn(false) {
    
    // Initialize PID controllers
    for (int i = 0; i < 4; i++) {
        _pidControllers[i].reset();
    }
}

bool BoilerController::begin(RuntimeStorage::RuntimeStorage* storage, 
                           SemaphoreHandle_t* i2cMutex) {
    if (!storage || !i2cMutex) {
        return false;
    }
    
    _storage = storage;
    _i2cMutex = i2cMutex;
    
    // Load PID states from FRAM
    for (uint8_t i = 0; i < 4; i++) {
        RuntimeStorage::PIDState state;
        if (_storage->loadPIDState(i, state)) {
            _pidControllers[i].integral = state.integral;
            _pidControllers[i].lastError = state.lastError;
            _pidControllers[i].output = state.output;
            _pidControllers[i].lastUpdateTime = state.lastUpdateTime;
        }
    }
    
    // Initialize timing
    _lastSensorRead = millis();
    _lastControlUpdate = millis();
    _burnerStopTime = millis();  // Ensure minimum off time at start
    
    return true;
}

void BoilerController::update() {
    uint32_t now = millis();
    
    // Check safety conditions first
    checkSafetyConditions();
    
    // Run control logic at specified interval
    if (now - _lastControlUpdate >= CONTROL_LOOP_INTERVAL) {
        _lastControlUpdate = now;
        
        if (_operatingMode != MODE_OFF) {
            runControlLogic();
            updatePIDControllers();
        }
        
        updateOutputs();
    }
}

void BoilerController::updateSensorReadings() {
    // In a real implementation, this would read from actual sensors
    // For this example, we simulate temperature changes
    
    float targetFlow = 20.0f;  // Default off temperature
    
    if (_burnerOn) {
        // Burner is on, temperatures rise
        targetFlow = (_operatingMode == MODE_HOT_WATER) ? 80.0f : 70.0f;
        _flowTemp = simulateTemperature(_flowTemp, targetFlow, 0.5f);  // Heat up
        _returnTemp = simulateTemperature(_returnTemp, _flowTemp - 20.0f, 0.3f);
    } else {
        // Burner is off, temperatures fall
        _flowTemp = simulateTemperature(_flowTemp, _roomTemp, 0.1f);  // Cool down
        _returnTemp = simulateTemperature(_returnTemp, _roomTemp, 0.1f);
    }
    
    // Simulate outdoor temperature (slow changes)
    _outsideTemp = simulateTemperature(_outsideTemp, 
        10.0f + 5.0f * sin(millis() / 3600000.0f), 0.01f);
    
    // Simulate room temperature
    if (_heatingPumpOn && _flowTemp > 40.0f) {
        _roomTemp = simulateTemperature(_roomTemp, 22.0f, 0.05f);  // Heat room
    } else {
        _roomTemp = simulateTemperature(_roomTemp, _outsideTemp + 10.0f, 0.02f);  // Cool down
    }
    
    _lastSensorRead = millis();
}

void BoilerController::runControlLogic() {
    uint32_t now = millis();
    
    switch (_operatingMode) {
        case MODE_HEATING:
        case MODE_BOTH: {
            // Heating control logic
            float targetFlow = 70.0f - (_outsideTemp * 0.5f);  // Simple weather compensation
            targetFlow = constrain(targetFlow, TEMP_MIN_FLOW, TEMP_MAX_FLOW);
            
            // Hysteresis control for burner
            if (!_burnerOn && _flowTemp < targetFlow - 5.0f) {
                // Check minimum off time
                if (now - _burnerStopTime >= BURNER_MIN_OFF_TIME) {
                    _burnerOn = true;
                    _burnerStartTime = now;
                    _storage->incrementCounter(RuntimeStorage::COUNTER_BURNER_STARTS);
                    _storage->logEvent(RuntimeStorage::EVENT_STATE_CHANGE, 0x0100);  // Burner on
                }
            } else if (_burnerOn && _flowTemp > targetFlow + 5.0f) {
                // Check minimum on time
                if (now - _burnerStartTime >= BURNER_MIN_ON_TIME) {
                    _burnerOn = false;
                    _burnerStopTime = now;
                    _storage->logEvent(RuntimeStorage::EVENT_STATE_CHANGE, 0x0101);  // Burner off
                }
            }
            
            // Heating pump control
            _heatingPumpOn = (_operatingMode == MODE_HEATING || _operatingMode == MODE_BOTH);
            break;
        }
        
        case MODE_HOT_WATER: {
            // Hot water control logic
            float targetFlow = 80.0f;  // Fixed target for DHW
            
            if (!_burnerOn && _flowTemp < targetFlow - 5.0f) {
                if (now - _burnerStopTime >= BURNER_MIN_OFF_TIME) {
                    _burnerOn = true;
                    _burnerStartTime = now;
                    _storage->incrementCounter(RuntimeStorage::COUNTER_BURNER_STARTS);
                }
            } else if (_burnerOn && _flowTemp > targetFlow + 5.0f) {
                if (now - _burnerStartTime >= BURNER_MIN_ON_TIME) {
                    _burnerOn = false;
                    _burnerStopTime = now;
                }
            }
            
            // Water pump control
            _waterPumpOn = true;
            _heatingPumpOn = false;
            break;
        }
        
        case MODE_OFF:
        default:
            _burnerOn = false;
            _heatingPumpOn = false;
            _waterPumpOn = false;
            break;
    }
}

void BoilerController::updatePIDControllers() {
    uint32_t now = millis();
    
    // Update heating flow PID
    if (_operatingMode == MODE_HEATING || _operatingMode == MODE_BOTH) {
        float targetFlow = 70.0f - (_outsideTemp * 0.5f);
        uint32_t dt = now - _pidControllers[PID_HEATING_FLOW].lastUpdateTime;
        
        if (dt > 0) {
            _pidControllers[PID_HEATING_FLOW].update(
                targetFlow, _flowTemp, 
                PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD, dt
            );
        }
    }
    
    // Update room temperature PID
    if (_heatingPumpOn) {
        float targetRoom = 21.0f;  // Fixed setpoint for example
        uint32_t dt = now - _pidControllers[PID_HEATING_ROOM].lastUpdateTime;
        
        if (dt > 0) {
            _pidControllers[PID_HEATING_ROOM].update(
                targetRoom, _roomTemp,
                1.0f, 0.05f, 0.2f, dt  // Slower PID for room control
            );
        }
    }
}

void BoilerController::checkSafetyConditions() {
    uint16_t newErrors = 0;
    
    // Check overheat
    if (_flowTemp > TEMP_OVERHEAT_THRESHOLD) {
        newErrors |= ERROR_OVERHEAT;
        _burnerOn = false;  // Emergency shutdown
        
        if (!(_activeErrors & ERROR_OVERHEAT)) {
            _storage->logCriticalError(ERROR_OVERHEAT, "Flow overheat", "Safety");
        }
    }
    
    // Check freeze risk
    if (_outsideTemp < TEMP_FREEZE_THRESHOLD && _operatingMode == MODE_OFF) {
        newErrors |= ERROR_FREEZE_RISK;
        
        if (!(_activeErrors & ERROR_FREEZE_RISK)) {
            _storage->logCriticalError(ERROR_FREEZE_RISK, "Freeze risk", "Safety");
        }
    }
    
    // Check sensor failures (simplified - just check for unrealistic values)
    if (_flowTemp < -50.0f || _flowTemp > 150.0f) {
        newErrors |= ERROR_SENSOR_FAILURE;
        
        if (!(_activeErrors & ERROR_SENSOR_FAILURE)) {
            _storage->logError(ERROR_SENSOR_FAILURE, "Flow sensor fault", "Sensor");
        }
    }
    
    // Update active errors
    _activeErrors = newErrors;
    
    // Update error counter if new errors
    if (newErrors != 0 && _lastErrorCode != newErrors) {
        _lastErrorCode = newErrors;
        _storage->incrementCounter(RuntimeStorage::COUNTER_ERROR_COUNT);
    }
}

void BoilerController::updateOutputs() {
    // In a real implementation, this would control actual relays
    // For this example, we just track the states
    
    // Post-circulation logic
    static uint32_t pumpStopTime = 0;
    
    if (!_heatingPumpOn && !_waterPumpOn) {
        if (pumpStopTime == 0) {
            pumpStopTime = millis() + PUMP_POST_CIRCULATION;
        }
    } else {
        pumpStopTime = 0;
    }
    
    // Update runtime hours
    static uint32_t lastRuntimeUpdate = 0;
    if (millis() - lastRuntimeUpdate > 1000) {  // Every second
        lastRuntimeUpdate = millis();
        
        if (_burnerOn) {
            float hours = _storage->getRuntimeHours(RuntimeStorage::RUNTIME_BURNER);
            _storage->updateRuntimeHours(RuntimeStorage::RUNTIME_BURNER, hours + (1.0f / 3600.0f));
        }
    }
}

void BoilerController::setOperatingMode(uint8_t mode) {
    if (mode != _operatingMode && mode <= MODE_MAINTENANCE) {
        uint8_t oldMode = _operatingMode;
        _operatingMode = mode;
        
        // Log mode change
        logStateChange(oldMode, mode);
        
        // Reset PID controllers on mode change
        if (mode == MODE_OFF) {
            for (int i = 0; i < 4; i++) {
                _pidControllers[i].reset();
            }
            _burnerOn = false;
            _heatingPumpOn = false;
            _waterPumpOn = false;
        }
    }
}

const char* BoilerController::getModeName() const {
    switch (_operatingMode) {
        case MODE_OFF: return "OFF";
        case MODE_HEATING: return "HEATING";
        case MODE_HOT_WATER: return "HOT WATER";
        case MODE_BOTH: return "HEATING + HOT WATER";
        case MODE_SUMMER: return "SUMMER";
        case MODE_MAINTENANCE: return "MAINTENANCE";
        default: return "UNKNOWN";
    }
}

bool BoilerController::getPIDState(uint8_t controllerId, RuntimeStorage::PIDState& state) const {
    if (controllerId >= 4) {
        return false;
    }
    
    state.integral = _pidControllers[controllerId].integral;
    state.lastError = _pidControllers[controllerId].lastError;
    state.output = _pidControllers[controllerId].output;
    state.lastUpdateTime = _pidControllers[controllerId].lastUpdateTime;
    state.crc = 0;  // Will be calculated by storage
    
    return true;
}

uint32_t BoilerController::getErrorCode() const {
    // Return the highest priority error
    if (_activeErrors & ERROR_OVERHEAT) return ERROR_OVERHEAT;
    if (_activeErrors & ERROR_FREEZE_RISK) return ERROR_FREEZE_RISK;
    if (_activeErrors & ERROR_SENSOR_FAILURE) return ERROR_SENSOR_FAILURE;
    if (_activeErrors & ERROR_PUMP_FAILURE) return ERROR_PUMP_FAILURE;
    if (_activeErrors & ERROR_BURNER_FAILURE) return ERROR_BURNER_FAILURE;
    return _activeErrors;
}

const char* BoilerController::getErrorMessage() const {
    if (_activeErrors & ERROR_OVERHEAT) return "System overheat";
    if (_activeErrors & ERROR_FREEZE_RISK) return "Freeze protection active";
    if (_activeErrors & ERROR_SENSOR_FAILURE) return "Sensor failure";
    if (_activeErrors & ERROR_PUMP_FAILURE) return "Pump failure";
    if (_activeErrors & ERROR_BURNER_FAILURE) return "Burner failure";
    if (_activeErrors != 0) return "Multiple errors";
    return "No errors";
}

float BoilerController::simulateTemperature(float current, float target, float rate) {
    // Simple exponential approach simulation
    float diff = target - current;
    return current + (diff * rate * (CONTROL_LOOP_INTERVAL / 1000.0f));
}

void BoilerController::logStateChange(uint8_t oldMode, uint8_t newMode) {
    _storage->logEvent(RuntimeStorage::EVENT_STATE_CHANGE, (oldMode << 8) | newMode);
    
    // Update pump start counters
    if (newMode == MODE_HEATING || newMode == MODE_BOTH) {
        if (oldMode != MODE_HEATING && oldMode != MODE_BOTH) {
            _storage->incrementCounter(RuntimeStorage::COUNTER_HEATING_PUMP_STARTS);
        }
    }
    
    if (newMode == MODE_HOT_WATER || newMode == MODE_BOTH) {
        if (oldMode != MODE_HOT_WATER && oldMode != MODE_BOTH) {
            _storage->incrementCounter(RuntimeStorage::COUNTER_WATER_PUMP_STARTS);
        }
    }
}