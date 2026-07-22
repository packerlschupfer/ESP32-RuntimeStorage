/*
 * RuntimeStorageLogging.h - part of the ESP32-RuntimeStorage library
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

// RuntimeStorageLogging.h
// Logging configuration for RuntimeStorage library
#pragma once

#include <esp_log.h>  // Required for ESP_LOG_* constants

#define RTSTOR_LOG_TAG "RTSTOR"

// Define log levels based on debug flag
#ifdef RUNTIME_STORAGE_DEBUG
    // Debug mode: Show all levels
    #define RTSTOR_LOG_LEVEL_E ESP_LOG_ERROR
    #define RTSTOR_LOG_LEVEL_W ESP_LOG_WARN
    #define RTSTOR_LOG_LEVEL_I ESP_LOG_INFO
    #define RTSTOR_LOG_LEVEL_D ESP_LOG_DEBUG
    #define RTSTOR_LOG_LEVEL_V ESP_LOG_VERBOSE
#else
    // Release mode: Only Error, Warn, Info
    #define RTSTOR_LOG_LEVEL_E ESP_LOG_ERROR
    #define RTSTOR_LOG_LEVEL_W ESP_LOG_WARN
    #define RTSTOR_LOG_LEVEL_I ESP_LOG_INFO
    #define RTSTOR_LOG_LEVEL_D ESP_LOG_NONE  // Suppress
    #define RTSTOR_LOG_LEVEL_V ESP_LOG_NONE  // Suppress
#endif

// Route to custom logger or ESP-IDF
#ifdef USE_CUSTOM_LOGGER
    #include <LogInterface.h>
    #define RTSTOR_LOG_E(...) LOG_WRITE(RTSTOR_LOG_LEVEL_E, RTSTOR_LOG_TAG, __VA_ARGS__)
    #define RTSTOR_LOG_W(...) LOG_WRITE(RTSTOR_LOG_LEVEL_W, RTSTOR_LOG_TAG, __VA_ARGS__)
    #define RTSTOR_LOG_I(...) LOG_WRITE(RTSTOR_LOG_LEVEL_I, RTSTOR_LOG_TAG, __VA_ARGS__)
    #define RTSTOR_LOG_D(...) LOG_WRITE(RTSTOR_LOG_LEVEL_D, RTSTOR_LOG_TAG, __VA_ARGS__)
    #define RTSTOR_LOG_V(...) LOG_WRITE(RTSTOR_LOG_LEVEL_V, RTSTOR_LOG_TAG, __VA_ARGS__)
#else
    // ESP-IDF logging with compile-time suppression
    #define RTSTOR_LOG_E(...) ESP_LOGE(RTSTOR_LOG_TAG, __VA_ARGS__)
    #define RTSTOR_LOG_W(...) ESP_LOGW(RTSTOR_LOG_TAG, __VA_ARGS__)
    #define RTSTOR_LOG_I(...) ESP_LOGI(RTSTOR_LOG_TAG, __VA_ARGS__)
    #ifdef RUNTIME_STORAGE_DEBUG
        #define RTSTOR_LOG_D(...) ESP_LOGD(RTSTOR_LOG_TAG, __VA_ARGS__)
        #define RTSTOR_LOG_V(...) ESP_LOGV(RTSTOR_LOG_TAG, __VA_ARGS__)
    #else
        #define RTSTOR_LOG_D(...) ((void)0)
        #define RTSTOR_LOG_V(...) ((void)0)
    #endif
#endif

// Feature-specific debug helpers
#ifdef RUNTIME_STORAGE_DEBUG
    // Timing macros for performance debugging (use FreeRTOS for thread safety)
    #define RTSTOR_TIME_START() TickType_t _rtstor_start = xTaskGetTickCount()
    #define RTSTOR_TIME_END(msg) RTSTOR_LOG_D("Timing: %s took %lu ms", msg, pdTICKS_TO_MS(xTaskGetTickCount() - _rtstor_start))

    // Buffer dump helper
    #define RTSTOR_DUMP_BUFFER(buffer, length, description) \
        do { \
            RTSTOR_LOG_D("%s (%d bytes):", description, (int)(length)); \
            for (size_t _i = 0; _i < (length); _i++) { \
                RTSTOR_LOG_D("  [%04X] = 0x%02X", (unsigned int)_i, ((uint8_t*)(buffer))[_i]); \
            } \
        } while(0)
#else
    #define RTSTOR_TIME_START() ((void)0)
    #define RTSTOR_TIME_END(msg) ((void)0)
    #define RTSTOR_DUMP_BUFFER(buffer, length, description) ((void)0)
#endif
