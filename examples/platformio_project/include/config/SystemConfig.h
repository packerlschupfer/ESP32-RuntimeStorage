/**
 * System Configuration
 * Central configuration for the boiler controller system
 */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// Firmware information
#define FIRMWARE_VERSION "1.0.0"
#define HARDWARE_VERSION "ESP32-BOILER-V1"

// I2C Configuration (avoiding Ethernet PHY conflicts)
#ifndef I2C_SDA_PIN
    #define I2C_SDA_PIN 33
#endif

#ifndef I2C_SCL_PIN
    #define I2C_SCL_PIN 32
#endif

#define I2C_FREQUENCY 400000  // 400kHz

// FRAM Configuration
#define FRAM_I2C_ADDRESS 0x50  // MB85RC256V default address

// Temperature sensor addresses
#define TEMP_SENSOR_FLOW_ADDR     0x48  // Example I2C addresses
#define TEMP_SENSOR_RETURN_ADDR   0x49
#define TEMP_SENSOR_OUTSIDE_ADDR  0x4A
#define TEMP_SENSOR_ROOM_ADDR     0x4B

// Operating modes
enum OperatingMode {
    MODE_OFF = 0,
    MODE_HEATING = 1,
    MODE_HOT_WATER = 2,
    MODE_BOTH = 3,
    MODE_SUMMER = 4,
    MODE_MAINTENANCE = 5
};

// Temperature limits (in °C)
#define TEMP_MIN_FLOW           20.0f
#define TEMP_MAX_FLOW           90.0f
#define TEMP_MAX_RETURN         85.0f
#define TEMP_MIN_ROOM           5.0f
#define TEMP_MAX_ROOM           35.0f
#define TEMP_OVERHEAT_THRESHOLD 95.0f
#define TEMP_FREEZE_THRESHOLD   3.0f

// PID Controller IDs
#define PID_HEATING_FLOW    0
#define PID_HEATING_ROOM    1
#define PID_WATER_FLOW      2
#define PID_OUTDOOR_COMP    3

// PID Default values
#define PID_DEFAULT_KP      2.0f
#define PID_DEFAULT_KI      0.1f
#define PID_DEFAULT_KD      0.5f

// Timing constants (ms)
#define SENSOR_READ_INTERVAL    1000    // 1 second
#define CONTROL_LOOP_INTERVAL   100     // 100ms
#define STORAGE_SAVE_INTERVAL   5000    // 5 seconds
#define TEMP_LOG_INTERVAL       60000   // 1 minute
#define STATUS_PRINT_INTERVAL   30000   // 30 seconds

// Safety parameters
#define BURNER_MIN_OFF_TIME     180000  // 3 minutes
#define BURNER_MIN_ON_TIME      60000   // 1 minute
#define PUMP_POST_CIRCULATION   300000  // 5 minutes
#define WATCHDOG_TIMEOUT        30000   // 30 seconds

// Error codes
#define ERROR_SENSOR_FAILURE    0x0001
#define ERROR_OVERHEAT          0x0002
#define ERROR_FREEZE_RISK       0x0004
#define ERROR_PUMP_FAILURE      0x0008
#define ERROR_BURNER_FAILURE    0x0010
#define ERROR_FRAM_FAILURE      0x0020
#define ERROR_WATCHDOG_RESET    0x0040
#define ERROR_COMM_FAILURE      0x0080

// Critical error mask
#define CRITICAL_ERROR_MASK     (ERROR_OVERHEAT | ERROR_FREEZE_RISK | \
                                ERROR_WATCHDOG_RESET | ERROR_FRAM_FAILURE)

// Task stack sizes
#define SENSOR_TASK_STACK       4096
#define CONTROL_TASK_STACK      4096
#define STORAGE_TASK_STACK      3072
#define MQTT_TASK_STACK         8192
#define OTA_TASK_STACK          8192

// Task priorities (higher number = higher priority)
#define SENSOR_TASK_PRIORITY    2
#define CONTROL_TASK_PRIORITY   3
#define STORAGE_TASK_PRIORITY   1
#define MQTT_TASK_PRIORITY      1
#define OTA_TASK_PRIORITY       1

// Queue sizes
#define SENSOR_QUEUE_SIZE       10
#define CONTROL_QUEUE_SIZE      20
#define EVENT_QUEUE_SIZE        50

// Network configuration (if using WiFi/Ethernet)
#define WIFI_SSID               "YourSSID"
#define WIFI_PASSWORD           "YourPassword"
#define MQTT_SERVER             "192.168.1.100"
#define MQTT_PORT               1883
#define MQTT_CLIENT_ID          "ESP32BoilerController"
#define MQTT_TOPIC_PREFIX       "boiler/"

// Debug settings
#ifndef RUNTIME_STORAGE_DEBUG
    #define RUNTIME_STORAGE_DEBUG 1
#endif

#define DEBUG_SERIAL_BAUD       115200
#define ENABLE_HEAP_MONITORING  1
#define ENABLE_TASK_MONITORING  1

#endif // SYSTEM_CONFIG_H