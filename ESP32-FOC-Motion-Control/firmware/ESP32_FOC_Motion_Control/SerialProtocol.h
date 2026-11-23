#pragma once
#include "Config.h"
/**
 * @file SerialProtocol.h
 * @brief Nonblocking command receive/parsing and human-readable reports.
 *
 * Numeric command strings may finish after a 120ms idle receive period.
 * RX and application commands share single-threaded main-loop context.
 */
void ack(char op);
void reject(char op,uint8_t why);
void status_brief();
void status_detailed();
void command_help();
void command_letter(char op);
void command_numeric(char op,const char* value);
void poll_rx();
