/**
 * @file test_runtime_storage.cpp
 * @brief Unit tests for RuntimeStorage
 *
 * NOTE: These tests require actual MB85RC256V FRAM hardware connected via I2C.
 * For simulation testing, use the mock test file instead.
 */

#ifdef UNIT_TEST

#include <Arduino.h>
#include <unity.h>
#include <Wire.h>
#include <RuntimeStorage.h>

using namespace rtstorage;

static RuntimeStorage* storage = nullptr;
static bool framAvailable = false;

void setUp() {
    // Check if storage is initialized
    if (storage == nullptr) {
        storage = new RuntimeStorage();
        Wire.begin();
        framAvailable = storage->begin(Wire);
        if (!framAvailable) {
            TEST_MESSAGE("WARNING: FRAM not available - some tests will be skipped");
        }
    }
}

void tearDown() {
    // Keep storage instance for all tests
}

// ============= Basic Connectivity Tests =============

void test_storage_initialization() {
    if (!framAvailable) {
        TEST_IGNORE_MESSAGE("FRAM not available");
    }

    TEST_ASSERT_TRUE(storage->isConnected());
    TEST_ASSERT_GREATER_THAN(0, storage->getSize());
}

void test_storage_integrity() {
    if (!framAvailable) {
        TEST_IGNORE_MESSAGE("FRAM not available");
    }

    bool result = storage->verifyIntegrity();
    TEST_ASSERT_TRUE(result);
}

// ============= PID State Tests =============

void test_pid_state_save_load() {
    if (!framAvailable) {
        TEST_IGNORE_MESSAGE("FRAM not available");
    }

    PIDState saveState;
    saveState.integral = 123.45f;
    saveState.previousError = -67.89f;
    saveState.output = 50.0f;
    saveState.timestamp = millis();
    saveState.valid = true;

    // Save
    bool saveResult = storage->savePIDState(0, saveState);
    TEST_ASSERT_TRUE(saveResult);

    // Load
    PIDState loadState;
    bool loadResult = storage->loadPIDState(0, loadState);
    TEST_ASSERT_TRUE(loadResult);

    // Verify
    TEST_ASSERT_FLOAT_WITHIN(0.01f, saveState.integral, loadState.integral);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, saveState.previousError, loadState.previousError);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, saveState.output, loadState.output);
    TEST_ASSERT_TRUE(loadState.valid);
}

void test_pid_state_multiple_controllers() {
    if (!framAvailable) {
        TEST_IGNORE_MESSAGE("FRAM not available");
    }

    // Save different states for multiple controllers
    for (uint8_t i = 0; i < 4; i++) {
        PIDState state;
        state.integral = 100.0f * i;
        state.previousError = 10.0f * i;
        state.output = 25.0f * i;
        state.timestamp = millis();
        state.valid = true;

        bool result = storage->savePIDState(i, state);
        TEST_ASSERT_TRUE(result);
    }

    // Verify each controller's state
    for (uint8_t i = 0; i < 4; i++) {
        PIDState state;
        bool result = storage->loadPIDState(i, state);
        TEST_ASSERT_TRUE(result);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f * i, state.integral);
    }
}

// ============= Counter Tests =============

void test_counter_operations() {
    if (!framAvailable) {
        TEST_IGNORE_MESSAGE("FRAM not available");
    }

    // Set counter to known value
    bool setResult = storage->setCounter(CounterType::BURNER_STARTS, 100);
    TEST_ASSERT_TRUE(setResult);

    // Verify value
    uint32_t value = storage->getCounter(CounterType::BURNER_STARTS);
    TEST_ASSERT_EQUAL(100, value);

    // Increment
    bool incResult = storage->incrementCounter(CounterType::BURNER_STARTS);
    TEST_ASSERT_TRUE(incResult);

    // Verify incremented value
    value = storage->getCounter(CounterType::BURNER_STARTS);
    TEST_ASSERT_EQUAL(101, value);
}

void test_runtime_hours() {
    if (!framAvailable) {
        TEST_IGNORE_MESSAGE("FRAM not available");
    }

    // Update runtime hours
    bool result = storage->updateRuntimeHours(RuntimeType::BURNER, 123.5f);
    TEST_ASSERT_TRUE(result);

    // Read back
    float hours = storage->getRuntimeHours(RuntimeType::BURNER);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 123.5f, hours);
}

// ============= Event Logging Tests =============

void test_event_logging() {
    if (!framAvailable) {
        TEST_IGNORE_MESSAGE("FRAM not available");
    }

    // Clear existing events
    storage->clearEvents();

    // Log some events
    bool result1 = storage->logEvent(EventType::SYSTEM_START, 0);
    bool result2 = storage->logEvent(EventType::TEMP_ALARM, 850);  // 85.0°C
    bool result3 = storage->logEvent(EventType::ERROR, 123);

    TEST_ASSERT_TRUE(result1);
    TEST_ASSERT_TRUE(result2);
    TEST_ASSERT_TRUE(result3);

    // Verify count
    size_t count = storage->getEventCount();
    TEST_ASSERT_EQUAL(3, count);
}

void test_event_retrieval() {
    if (!framAvailable) {
        TEST_IGNORE_MESSAGE("FRAM not available");
    }

    // Clear and add known events
    storage->clearEvents();
    storage->logEvent(EventType::SYSTEM_START, 1);
    storage->logEvent(EventType::SYSTEM_START, 2);
    storage->logEvent(EventType::SYSTEM_START, 3);

    // Retrieve events
    Event buffer[10];
    size_t retrieved = storage->getEvents(buffer, 10);

    TEST_ASSERT_EQUAL(3, retrieved);
    TEST_ASSERT_EQUAL(EventType::SYSTEM_START, buffer[0].type);
}

// ============= System State Tests =============

void test_system_state() {
    if (!framAvailable) {
        TEST_IGNORE_MESSAGE("FRAM not available");
    }

    SystemState saveState;
    saveState.mode = 2;
    saveState.targetTemp = 650;  // 65.0°C
    saveState.timestamp = millis();

    // Save
    bool saveResult = storage->saveSystemState(saveState);
    TEST_ASSERT_TRUE(saveResult);

    // Load
    SystemState loadState;
    bool loadResult = storage->loadSystemState(loadState);
    TEST_ASSERT_TRUE(loadResult);

    // Verify
    TEST_ASSERT_EQUAL(saveState.mode, loadState.mode);
    TEST_ASSERT_EQUAL(saveState.targetTemp, loadState.targetTemp);
}

// ============= Error Logging Tests =============

void test_error_logging() {
    if (!framAvailable) {
        TEST_IGNORE_MESSAGE("FRAM not available");
    }

    storage->clearErrors();

    bool result = storage->logError(100, "Test error", "Unit test");
    TEST_ASSERT_TRUE(result);

    size_t count = storage->getErrorCount();
    TEST_ASSERT_EQUAL(1, count);
}

void test_critical_error_logging() {
    if (!framAvailable) {
        TEST_IGNORE_MESSAGE("FRAM not available");
    }

    storage->clearErrors();

    bool result = storage->logCriticalError(500, "Critical test", "Emergency");
    TEST_ASSERT_TRUE(result);

    ErrorEntry buffer[5];
    size_t count = storage->getCriticalErrors(buffer, 5);
    TEST_ASSERT_GREATER_THAN(0, count);
}

// ============= Raw Access Tests =============

void test_raw_read_write() {
    if (!framAvailable) {
        TEST_IGNORE_MESSAGE("FRAM not available");
    }

    // Write test pattern
    uint8_t writeData[8] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
    bool writeResult = storage->writeBytes(0x7FF0, writeData, 8);  // Near end of FRAM
    TEST_ASSERT_TRUE(writeResult);

    // Read back
    uint8_t readData[8] = {0};
    bool readResult = storage->readBytes(0x7FF0, readData, 8);
    TEST_ASSERT_TRUE(readResult);

    // Verify
    TEST_ASSERT_EQUAL_UINT8_ARRAY(writeData, readData, 8);
}

// ============= Nodiscard Warning Test =============

void test_nodiscard_usage() {
    if (!framAvailable) {
        TEST_IGNORE_MESSAGE("FRAM not available");
    }

    // These should compile without warnings because we use the return values
    [[maybe_unused]] bool b1 = storage->verifyIntegrity();
    [[maybe_unused]] bool b2 = storage->savePIDState(0, PIDState{});
    [[maybe_unused]] size_t s1 = storage->getEventCount();
    [[maybe_unused]] size_t s2 = storage->getFreeSpace();

    TEST_PASS();
}

// Test runner
void runRuntimeStorageTests() {
    UNITY_BEGIN();

    RUN_TEST(test_storage_initialization);
    RUN_TEST(test_storage_integrity);
    RUN_TEST(test_pid_state_save_load);
    RUN_TEST(test_pid_state_multiple_controllers);
    RUN_TEST(test_counter_operations);
    RUN_TEST(test_runtime_hours);
    RUN_TEST(test_event_logging);
    RUN_TEST(test_event_retrieval);
    RUN_TEST(test_system_state);
    RUN_TEST(test_error_logging);
    RUN_TEST(test_critical_error_logging);
    RUN_TEST(test_raw_read_write);
    RUN_TEST(test_nodiscard_usage);

    UNITY_END();

    // Cleanup
    if (storage) {
        delete storage;
        storage = nullptr;
    }
}

void setup() {
    delay(2000);
    Serial.begin(115200);
    Serial.println("\n=== RuntimeStorage Unit Tests ===\n");
    Serial.println("NOTE: Requires MB85RC256V FRAM connected via I2C");
    Serial.println("Default address: 0x50\n");
    runRuntimeStorageTests();
}

void loop() {}

#endif // UNIT_TEST
