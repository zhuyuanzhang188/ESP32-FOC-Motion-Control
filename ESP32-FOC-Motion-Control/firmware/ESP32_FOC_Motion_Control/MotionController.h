#pragma once
#include "TrajectoryPlanner.h"
/**
 * @file MotionController.h
 * @brief Single-axis motion state machine and online trajectory takeover.
 *
 * All angular positions are in rad; velocity in rad/s and acceleration in
 * rad/s^2. Online N uses an ABSOLUTE goal and starts from the current planned
 * state (x,v,a), not noisy instantaneous AS5600 speed readings.
 *
 * On an infeasible N candidate, the original planned trajectory is preserved.
 * No command here provides a certified emergency stop.
 */
enum ProfileMode : uint8_t {
  PROFILE_TRAPEZOID=0, PROFILE_SCURVE=1, PROFILE_SOFT_STOP=2, PROFILE_ONLINE=3
};
const char* stop_mode_name();
char profile_letter();
bool profile_active();
uint8_t profile_phase(uint32_t now);
/// Evaluate the active preplanned trajectory (no allocation or path solving).
void profile_eval(uint32_t now,float &theta,float &omega);
float profile_accel(uint32_t now);
void profile_hold(float angle);
float motion_target_angle();
const char* motion_state_name();
bool start_move(float to,char op);
void command_auto(const char* value);
void command_scurve_auto(const char* value);
void command_soft_stop();
/// N command: preflight a new absolute goal; atomic logical takeover on success.
void command_online_target(const char* value);
void auto_update();
