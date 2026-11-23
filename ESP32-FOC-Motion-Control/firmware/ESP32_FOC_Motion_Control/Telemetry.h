#pragma once
#include "Config.h"
/**
 * @file Telemetry.h
 * @brief Bounded nonblocking serial output and 20Hz 21-channel CSV.
 *
 * `tx_printf` writes to a finite ring; `tx_service` drains <=32 bytes per loop.
 * CSV field i15 reports cached boot VIN, not a live bus voltage measurement.
 * The queue is NOT interrupt-safe or multi-thread-safe.
 */
uint16_t tx_room();
void tx_clear();
void tx_printf(const char* fmt, ...);
void tx_service();
void telemetry_update_csv();
