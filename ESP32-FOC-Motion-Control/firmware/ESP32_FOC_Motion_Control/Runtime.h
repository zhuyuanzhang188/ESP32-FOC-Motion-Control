#pragma once
#include "Config.h"
#include "MotorControl.h"
#include "TrajectoryPlanner.h"
#include "MotionController.h"
#include "SerialProtocol.h"
#include "Diagnostics.h"
#include "Telemetry.h"

// Compatibility state declarations. Each value has ONE definition in its owner .cpp.
// This explicit extern bridge prevents Arduino auto-prototype and multiple definition bugs.
extern BLDCMotor motor1;
extern BLDCDriver3PWM driver1;
extern MagneticSensorI2C sensor1;
extern TwoWire bus1;
extern InlineCurrentSense sense1;
extern float ff_gain, traj_vmax, traj_amax;
extern bool foc_ok, csv_on;
extern uint32_t last_csv_ms;
extern float csv_boot_vin;
extern uint32_t tx_drops;
extern Trapezoid trajectory;
extern SCurve scurve;
extern SoftStop soft_stop;
extern OnlineFullProfile online_profile;
extern ProfileMode profile_mode;
extern float scurve_jmax;
extern float online_last_j, online_last_target, online_last_v0, online_last_a0;
extern float online_last_brake_distance, online_last_peak_v;
extern bool online_cancelled_auto, stop_soft_limit_warning;
extern uint8_t stop_guard_mode;
extern float stop_plan_jerk, stop_comfort_peak_v;
extern float stop_predicted_peak_v, stop_predicted_min_x, stop_predicted_max_x;
extern bool stop_original_speed_conflict, stop_original_range_conflict;
extern bool stop_remaining_speed_conflict;
extern float stop_takeover_error, stop_track_max_error;
extern float stop_filtered_actual_v, stop_track_max_abs_v;
extern uint32_t stop_track_warn_since_ms, stop_track_over_samples;
extern bool stop_track_warning;
extern uint32_t stop_track_last_us;
extern float iq_abs_peak, iq_ref_abs_peak, online_max_error;
extern uint32_t online_takeovers, iq_warn_since, iq_warn_samples;
extern uint32_t online_solve_last_us, online_solve_max_us, online_solve_slow_count;
extern bool iq_warning;
extern float theta_d, omega_d, theta_cmd;
extern LoopTimingStats loop_timing;
extern char current_loop_tag;
extern uint32_t run_id, run_started_us, settle_candidate_ms;
extern TrackingStats tracking_stats;
extern RunSummary last_run_summary, last_interrupt_summary;
extern RunQualityStats run_quality;
extern uint32_t online_command_last_us, online_command_max_us;
extern uint32_t online_command_success_count;
extern bool diag_reset_pending;
extern uint32_t accepted, rejected;
extern uint8_t last_reject;
extern char last_op;
extern bool auto_running, auto_cancel_requested;
extern char auto_kind;
extern float auto_origin, auto_far;
extern uint8_t auto_legs_started, auto_legs_total;
extern uint32_t auto_wait_begin_ms;
