// RuntimeStorageConfig.h
// Configuration and memory map for RuntimeStorage library
#pragma once

#include <Arduino.h>

namespace rtstorage {

// FRAM Configuration
const uint8_t DEFAULT_I2C_ADDRESS = 0x50;
const uint32_t FRAM_SIZE = 32768;  // 32KB (256Kbit)
const uint32_t I2C_SPEED = 400000; // 400kHz

// Retry configuration
const uint8_t MAX_RETRIES = 3;
const uint32_t RETRY_DELAY_MS = 5;

// Memory Map Addresses
const uint16_t ADDR_HEADER          = 0x0000;  // 32 bytes
const uint16_t ADDR_SYSTEM_STATE    = 0x0020;  // 224 bytes
const uint16_t ADDR_PID_STATES      = 0x0100;  // 256 bytes (4 controllers × 64B)
const uint16_t ADDR_COUNTERS        = 0x0200;  // 256 bytes
const uint16_t ADDR_RUNTIME_HOURS   = 0x0300;  // 256 bytes
const uint16_t ADDR_EVENT_LOG       = 0x0400;  // 2KB
const uint16_t ADDR_TEMP_HISTORY    = 0x0C00;  // 8KB
const uint16_t ADDR_ERROR_LOG       = 0x2C00;  // 8KB (50 errors × ~150B each)
const uint16_t ADDR_ERROR_STATS     = 0x4C00;  // 32 bytes
const uint16_t ADDR_RESERVED        = 0x4C20;  // 13KB for future use

// Sizes
const uint16_t SIZE_HEADER          = 32;
const uint16_t SIZE_SYSTEM_STATE    = 224;
const uint16_t SIZE_PID_STATES      = 256;
const uint16_t SIZE_COUNTERS        = 256;
const uint16_t SIZE_RUNTIME_HOURS   = 256;
const uint16_t SIZE_EVENT_LOG       = 2048;
const uint16_t SIZE_TEMP_HISTORY    = 8192;
const uint16_t SIZE_ERROR_LOG       = 8192;
const uint16_t SIZE_ERROR_STATS     = 32;

// Feature configuration
const uint8_t MAX_PID_CONTROLLERS   = 4;
const uint8_t MAX_TEMP_SENSORS      = 8;
const uint16_t MAX_EVENTS           = 64;  // Number of events in circular buffer
const uint16_t TEMP_HISTORY_SIZE    = 64;  // Readings per sensor
const uint16_t MAX_ERRORS           = 50;  // Number of errors in circular buffer
const uint16_t MAX_CRITICAL_ERRORS  = 5;   // Number of critical errors to keep

// CRC configuration
const uint32_t CRC_POLYNOMIAL = 0xEDB88320;  // Standard CRC32 polynomial

// Thread safety
#ifdef ESP32
    #define USE_MUTEX_PROTECTION 1
#else
    #define USE_MUTEX_PROTECTION 0
#endif

// Debug configuration
#ifndef RUNTIME_STORAGE_DEBUG
    #define RUNTIME_STORAGE_DEBUG 0
#endif

} // namespace rtstorage