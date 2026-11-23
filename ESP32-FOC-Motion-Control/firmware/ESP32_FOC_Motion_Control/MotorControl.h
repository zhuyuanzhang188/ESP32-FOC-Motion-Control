#pragma once
#include "Config.h"
/**
 * @file MotorControl.h
 * @brief Single motor hardware setup and fast control-loop execution.
 *
 * Only motor_setup() performs board initialization and startup VIN sampling.
 * motor_update() retains V8.1 loopFOC -> reference -> move -> monitor ordering.
 * SimpleFOC provides the underlying FOC algorithm; this module configures it.
 */
void motor_setup();
void motor_update();
float bus_voltage(); // setup only! Never use in main loop/CSV.
