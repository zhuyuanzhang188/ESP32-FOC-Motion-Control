#pragma once
#include "Config.h"
/**
 * @file Diagnostics.h
 * @brief Motion accuracy, persistent error, q-axis current and latency stats.
 *
 * RMSE is accumulated from planned-minus-measured angle while moving with
 * a time-weighted ~10ms sampling cadence. Reporting is diagnostic only;
 * it must not be interpreted as a hardware overcurrent or position E-stop.
 */

struct LoopTimingStats {
  uint32_t loop_begin_us=0, prev_begin_us=0;
  bool prev_valid=false;
  uint32_t gap_max_us=0, foc_max_us=0, busy_max_us=0;
  uint32_t gap_last_us=0, foc_last_us=0, busy_last_us=0;
  uint32_t gap_count=0, foc_count=0, busy_count=0;
  uint32_t slow_gap_count=0;
  char last_loop_tag='-', worst_gap_tag='-', worst_busy_tag='-';
  uint64_t gap_sum_us=0, foc_sum_us=0, busy_sum_us=0;
};

struct TrackingStats {
  uint32_t run=0;
  char kind='-';
  bool running=false;
  bool warned_this_run=false, severe_this_run=false;
  bool warn_streak=false, severe_streak=false;
  bool warn_latched=false, severe_latched=false;
  uint32_t warn_since_ms=0, severe_since_ms=0;
  uint32_t max_warn_streak_ms=0, max_severe_streak_ms=0;
  uint32_t warn_events=0, severe_events=0, invalid_samples=0;
  float last_err=0, max_err_this_run=0, max_err_all=0;
};

struct RunSummary {
  bool valid=false;
  uint32_t run=0, elapsed_ms=0, sample_count=0, weighted_ms=0;
  uint32_t solve_us=0, command_us=0;
  char kind='-';
  const char* result="NONE";
  bool warn=false, severe=false, replaced=false;
  // final_err: actual distance to final goal ONLY for completed/timeout runs.
  // Replaced runs have no final-goal arrival; keep separate interruption metrics.
  float target=0, max_err=0, rmse=0, peak_v=0, final_err=NAN;
  float old_target_remaining=NAN, takeover_track_err=NAN;
};

struct RunQualityStats {
  bool active=false, plan_finished=false, settling=false;
  uint32_t run=0, begin_us=0, last_sample_ms=0;
  uint32_t plan_end_ms=0, settled_from_ms=0;
  uint32_t weighted_ms=0, sample_count=0, invalid=0;
  uint32_t solve_us=0, command_us=0;
  char kind='-';
  float target=0, max_err=0, peak_v=0;
  double sum_e2_dt=0;

  void finish(const char* result, float previous_plan_x=NAN);
  void start(uint32_t id,char mode,float to,float planned_peak,
             uint32_t solve_cost_us,uint32_t command_cost_us,
             float previous_plan_x=NAN);
  void update();
};

void timing_loop_begin(uint32_t now_us);
void timing_foc_done(uint32_t us);
void timing_loop_done(uint32_t now_us);
void monitor_runtime_diagnostics();
void monitor_stop_tracking(uint32_t now_us);
void monitor_np_tracking(uint32_t now_ms);
void clear_v8_diagnostics();
