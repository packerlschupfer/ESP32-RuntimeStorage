/*
 * RuntimeStorageTypes.h - part of the ESP32-RuntimeStorage library
 *
 * Copyright (C) 2025-2026 packerlschupfer
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// RuntimeStorageTypes.h
// Type definitions for RuntimeStorage library
#pragma once

#include <Arduino.h>

namespace rtstorage {

// Temperature type from main project
typedef int16_t Temperature_t;  // Fixed-point, 1/10th degree

// PID Controller state
struct PIDState {
    float integral;
    float lastError;
    float output;
    uint32_t lastUpdateTime;
    uint32_t crc;
};

// Counter types
enum CounterType : uint8_t {
    COUNTER_BURNER_STARTS,
    COUNTER_HEATING_PUMP_STARTS,
    COUNTER_WATER_PUMP_STARTS,
    COUNTER_ERROR_COUNT,
    COUNTER_SYSTEM_RESTARTS,
    COUNTER_MAX
};

// Runtime hour types
enum RuntimeType : uint8_t {
    RUNTIME_TOTAL,
    RUNTIME_HEATING,
    RUNTIME_WATER,
    RUNTIME_BURNER,
    RUNTIME_MAX
};

// Event types with bit flags for filtering
enum EventType : uint8_t {
    EVENT_ERROR        = 0x01,
    EVENT_WARNING      = 0x02,
    EVENT_STATE_CHANGE = 0x04,
    EVENT_USER_ACTION  = 0x08,
    EVENT_SYSTEM       = 0x10,
    EVENT_ALL          = 0xFF
};

// Error log entry structure
struct ErrorEntry {
    uint32_t timestamp;
    uint32_t errorCode;
    uint16_t count;
    char message[64];
    char context[32];
    
    ErrorEntry() : timestamp(0), errorCode(0), count(0) {
        message[0] = '\0';
        context[0] = '\0';
    }
} __attribute__((packed));

// Error statistics
struct ErrorStats {
    uint32_t totalErrors;
    uint32_t criticalErrors;
    uint32_t lastErrorTime;
    uint32_t oldestErrorTime;
    uint16_t uniqueErrors;
    uint32_t crc;
};

// Event filter type
typedef uint8_t EventFilter;
const EventFilter ALL_EVENTS = EVENT_ALL;

// Event structure
struct Event {
    uint32_t timestamp;     // millis() or RTC time
    EventType type;
    uint8_t subtype;        // Error code, state, etc.
    uint16_t data;          // Event-specific data
};

// Temperature reading with timestamp
struct TempReading {
    uint32_t timestamp;
    Temperature_t temperature;
    bool valid;
};

// System state structure
struct SystemState {
    uint8_t operatingMode;
    uint16_t activeErrors;
    uint8_t lastShutdownReason;
    uint32_t uptimeAtShutdown;
    uint32_t crc;
};

// Memory map header
struct MemoryHeader {
    uint32_t magic;         // 0x4652414D ('FRAM')
    uint16_t version;       // Data structure version
    uint16_t size;          // Total FRAM size
    uint32_t formatTime;    // When FRAM was formatted
    uint32_t crc;           // Header CRC
};

// Constants
const uint32_t FRAM_MAGIC = 0x4652414D;  // 'FRAM' in hex
const uint16_t CURRENT_VERSION = 1;

} // namespace rtstorage