#pragma once
// V8.1 original hardware and motion constants; do not change during a structural refactor.
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char* FW_VERSION = "BC_P_N_ENGINEERING_V8_1";
static const float MIN_VIN = 11.1f;
static const float POSITION_KP = 20.0f;
static const float INNER_VEL_LIMIT = 20.0f;
// Initial software TEST travel limit. Change ONLY after checking mechanical safety.
static const float MAX_ABS_ANGLE = 100.0f;
// Fixed P stop jerk bound; selected after the user's J=60..200 study.
// Does not modify the J in the active C trajectory.
static const float STOP_JERK_MAX = 140.0f;
// V6 N uses the same V/A envelope as B/C. This is NOT a hardware guarantee.
static const float ONLINE_MAX_TEST_V = INNER_VEL_LIMIT;
static const float ONLINE_MAX_TEST_A = 30.0f;
static const float ONLINE_DEFAULT_J = 140.0f;
static const float ONLINE_TAKEOVER_MAX_ERR_RAD = 0.20f;
// Optional protective J ceiling for mathematically infeasible ordinary stops.
// J600 has been exercised in user's ordinary S-curves, but NOT validated for P takeover.
// It is deliberately NOT an E-stop, and MAY STILL exceed the velocity limit.
static const float STOP_PROTECT_JERK_MAX = 600.0f;
static const float STOP_BOUND_EPS = 0.01f;
// Monitoring only: never disable torque abruptly based on one noisy sensor sample.
static const float STOP_TRACK_WARN_RAD = 0.30f;
static const uint32_t STOP_TRACK_WARN_MS = 100;
static const uint32_t CSV_PERIOD_MS = 50;
static const uint32_t HOLD_BEFORE_MOVE_MS = 600;
static const float HOLD_ERROR_LIMIT = 0.25f;
static const float SOFT_TRAVEL_GUARD_MARGIN = 2.0f; // model allowance for FF/position tracking
static const float IQ_DIAGNOSTIC_WARN_A=0.45f;
static const uint32_t IQ_WARN_HOLD_MS=50;
static const uint32_t ONLINE_SOLVE_WARN_US=10000; // diagnostic, not hard real-time bound
static const uint32_t LOOP_GAP_OBSERVE_US = 5000UL; // observational only; tune from baseline
static const float TRACK_WARN_RAD=0.15f;
static const uint32_t TRACK_WARN_HOLD_MS=150;
static const float TRACK_WARN_RECOVER_RAD=0.10f;
static const float TRACK_SEVERE_RAD=0.30f;
static const uint32_t TRACK_SEVERE_HOLD_MS=200;
static const float TRACK_SEVERE_RECOVER_RAD=0.20f;
static const uint32_t RUN_METRIC_SAMPLE_MS=10;
static const uint32_t RUN_METRIC_MAX_DT_MS=100;
static const float RUN_SETTLE_ERR_RAD=0.05f;
static const uint32_t RUN_SETTLE_HOLD_MS=200;
static const uint32_t RUN_SETTLE_TIMEOUT_MS=2000;
static const uint32_t AUTO_SETTLE_TIMEOUT_MS = 8000;
static const uint8_t AUTO_MAX_ROUNDTRIPS = 10;
