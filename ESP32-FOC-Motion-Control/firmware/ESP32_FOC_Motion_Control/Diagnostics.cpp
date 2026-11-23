// Module extracted from the user's V8.1 single-axis firmware.
// Refactor only: keep original motion formulas, control parameters and sequence.
// SimpleFOC is an external dependency; this repository does not vendor it.
#include "Runtime.h"

float iq_abs_peak=0,iq_ref_abs_peak=0,online_max_error=0;
uint32_t online_takeovers=0,iq_warn_since=0,iq_warn_samples=0;
uint32_t online_solve_last_us=0,online_solve_max_us=0,online_solve_slow_count=0;
bool iq_warning=false;
TrackingStats tracking_stats;


LoopTimingStats loop_timing;
// A best-effort label: the command processed during a long loop, not proof of cause.
char current_loop_tag='-';

void timing_loop_begin(uint32_t now_us) {
  loop_timing.loop_begin_us=now_us;
  current_loop_tag='-';
  if(loop_timing.prev_valid) {
    const uint32_t gap=(uint32_t)(now_us-loop_timing.prev_begin_us);
    loop_timing.gap_last_us=gap;
    if(gap>loop_timing.gap_max_us){
      loop_timing.gap_max_us=gap;
      loop_timing.worst_gap_tag=loop_timing.last_loop_tag;
    }
    loop_timing.gap_sum_us+=gap;
    ++loop_timing.gap_count;
    if(gap>LOOP_GAP_OBSERVE_US)++loop_timing.slow_gap_count;
  }
  loop_timing.prev_begin_us=now_us;
  loop_timing.prev_valid=true;
}
void timing_foc_done(uint32_t us) {
  loop_timing.foc_last_us=us;
  if(us>loop_timing.foc_max_us)loop_timing.foc_max_us=us;
  loop_timing.foc_sum_us+=us;
  ++loop_timing.foc_count;
}
void timing_loop_done(uint32_t now_us) {
  const uint32_t busy=(uint32_t)(now_us-loop_timing.loop_begin_us);
  loop_timing.busy_last_us=busy;
  const char tag=current_loop_tag!='-'?current_loop_tag:
    (profile_active()?profile_letter():'-');
  if(busy>loop_timing.busy_max_us){
    loop_timing.busy_max_us=busy;
    loop_timing.worst_busy_tag=tag;
  }
  loop_timing.last_loop_tag=tag;
  loop_timing.busy_sum_us+=busy;
  ++loop_timing.busy_count;
}
void monitor_runtime_diagnostics() {
  const float iq=motor1.current.q, iq_ref=motor1.current_sp;
  if(isfinite(iq))iq_abs_peak=fmaxf(iq_abs_peak,fabsf(iq));
  if(isfinite(iq_ref))iq_ref_abs_peak=fmaxf(iq_ref_abs_peak,fabsf(iq_ref));
  if(isfinite(iq)&&fabsf(iq)>IQ_DIAGNOSTIC_WARN_A){
    ++iq_warn_samples;
    if(iq_warn_since==0)iq_warn_since=millis();
    if((uint32_t)(millis()-iq_warn_since)>=IQ_WARN_HOLD_MS)iq_warning=true;
  } else iq_warn_since=0;
  if(profile_mode==PROFILE_ONLINE && online_profile.ever &&
     isfinite(theta_d) && isfinite(motor1.shaft_angle)){
    online_max_error=fmaxf(online_max_error,fabsf(theta_d-motor1.shaft_angle));
  }
}


void monitor_stop_tracking(uint32_t now_us) {
  // Lightweight low-pass velocity estimate for DIAGNOSTICS ONLY.
  // AS5600 raw speed spikes must not directly change braking references.
  if(stop_track_last_us==0)stop_track_last_us=now_us;
  const float dt=fminf((uint32_t)(now_us-stop_track_last_us)*1e-6f,0.05f);
  stop_track_last_us=now_us;
  const float measured_v=motor1.shaft_velocity;
  if(isfinite(measured_v) && fabsf(measured_v)<250.0f) {
    const float alpha=dt/(0.10f+dt);
    stop_filtered_actual_v+=alpha*(measured_v-stop_filtered_actual_v);
  }
  if(profile_mode!=PROFILE_SOFT_STOP || !soft_stop.ever)return;
  const float since=(uint32_t)(now_us-soft_stop.begin_us)*1e-6f;
  if(since>soft_stop.total+2.0f)return;
  const float err=fabsf(theta_d-motor1.shaft_angle);
  if(!isfinite(err))return;
  stop_track_max_error=fmaxf(stop_track_max_error,err);
  stop_track_max_abs_v=fmaxf(stop_track_max_abs_v,fabsf(stop_filtered_actual_v));
  if(err>STOP_TRACK_WARN_RAD) {
    ++stop_track_over_samples;
    if(stop_track_warn_since_ms==0)stop_track_warn_since_ms=millis();
    if(!stop_track_warning && (uint32_t)(millis()-stop_track_warn_since_ms)>=STOP_TRACK_WARN_MS) {
      stop_track_warning=true;
      if(!csv_on)tx_printf("#WARN P_TRACK_ERR abs=%.3f threshold=%.2f\n",err,STOP_TRACK_WARN_RAD);
    }
  } else stop_track_warn_since_ms=0;
}



void monitor_np_tracking(uint32_t now_ms) {
  const bool on_np=(profile_mode==PROFILE_ONLINE || profile_mode==PROFILE_SOFT_STOP);
  if(!on_np || !profile_active()) {
    // Keep completed run's metrics visible in S; only clear active timers.
    tracking_stats.running=false;
    tracking_stats.warn_streak=false;
    tracking_stats.severe_streak=false;
    tracking_stats.warn_latched=false;
    tracking_stats.severe_latched=false;
    return;
  }
  const char kind=(profile_mode==PROFILE_ONLINE)?'N':'P';
  if(tracking_stats.run!=run_id || tracking_stats.kind!=kind) {
    tracking_stats.run=run_id;
    tracking_stats.kind=kind;
    tracking_stats.warned_this_run=false;
    tracking_stats.severe_this_run=false;
    tracking_stats.warn_streak=false;
    tracking_stats.severe_streak=false;
    tracking_stats.warn_latched=false;
    tracking_stats.severe_latched=false;
    tracking_stats.max_warn_streak_ms=0;
    tracking_stats.max_severe_streak_ms=0;
    tracking_stats.max_err_this_run=0;
    tracking_stats.last_err=0;
  }
  tracking_stats.running=true;
  if(!isfinite(theta_d) || !isfinite(motor1.shaft_angle)) {
    ++tracking_stats.invalid_samples;
    tracking_stats.warn_streak=false;
    tracking_stats.severe_streak=false;
    return;
  }
  const float err=fabsf(theta_d-motor1.shaft_angle);
  tracking_stats.last_err=err;
  if(err>tracking_stats.max_err_this_run)tracking_stats.max_err_this_run=err;
  if(err>tracking_stats.max_err_all)tracking_stats.max_err_all=err;

  // A warning is fired only after STRICT consecutive time above its threshold.
  // Recovery hysteresis affects the displayed active flag, NOT streak duration.
  if(err>TRACK_WARN_RAD) {
    if(!tracking_stats.warn_streak) {
      tracking_stats.warn_streak=true;
      tracking_stats.warn_since_ms=now_ms;
    }
    const uint32_t held=(uint32_t)(now_ms-tracking_stats.warn_since_ms);
    if(held>tracking_stats.max_warn_streak_ms)tracking_stats.max_warn_streak_ms=held;
    if(held>=TRACK_WARN_HOLD_MS && !tracking_stats.warn_latched) {
      tracking_stats.warn_latched=true;
      tracking_stats.warned_this_run=true;
      ++tracking_stats.warn_events;
    }
  } else tracking_stats.warn_streak=false;
  if(err<=TRACK_WARN_RECOVER_RAD)tracking_stats.warn_latched=false;

  if(err>TRACK_SEVERE_RAD) {
    if(!tracking_stats.severe_streak) {
      tracking_stats.severe_streak=true;
      tracking_stats.severe_since_ms=now_ms;
    }
    const uint32_t held=(uint32_t)(now_ms-tracking_stats.severe_since_ms);
    if(held>tracking_stats.max_severe_streak_ms)tracking_stats.max_severe_streak_ms=held;
    if(held>=TRACK_SEVERE_HOLD_MS && !tracking_stats.severe_latched) {
      tracking_stats.severe_latched=true;
      tracking_stats.severe_this_run=true;
      ++tracking_stats.severe_events;
    }
  } else tracking_stats.severe_streak=false;
  if(err<=TRACK_SEVERE_RECOVER_RAD)tracking_stats.severe_latched=false;
}

// V8 per-run accuracy. Sample at ~100 Hz and weight by observed elapsed
// milliseconds; the RMSE covers ACTIVE motion only (not post-motion settling).
// A finished profile is not considered positioned until the measured error
// stays <=0.05 rad for 200 ms; otherwise report NOT_SETTLED after 2 s.
// This diagnostic never changes the actuator's target or drive enable state.

RunSummary last_run_summary;
// Keep the latest interruption separately: after N settles, last_run_summary
// points to N, but I should still reveal why the previous run was cut short.
RunSummary last_interrupt_summary;


RunQualityStats run_quality;

uint32_t online_command_last_us=0,online_command_max_us=0;
uint32_t online_command_success_count=0;

// Reset AFTER the current loop completes, so the D-command parsing overhead
// is not incorrectly included in a new diagnostic baseline.
bool diag_reset_pending=false;

void RunQualityStats::finish(const char* result, float previous_plan_x) {
    if(!active)return;
    RunSummary summary;
    summary.valid=true;
    summary.run=run;summary.kind=kind;summary.result=result;
    summary.replaced=(strcmp(result,"REPLACED")==0);
    summary.target=target;summary.peak_v=peak_v;
    summary.elapsed_ms=(uint32_t)(micros()-begin_us)/1000UL;
    summary.sample_count=sample_count;summary.weighted_ms=weighted_ms;
    summary.rmse=weighted_ms?sqrtf((float)(sum_e2_dt/(double)weighted_ms)):NAN;
    summary.max_err=max_err;
    summary.solve_us=solve_us;summary.command_us=command_us;
    if((kind=='N'||kind=='P') && tracking_stats.run==run && tracking_stats.kind==kind){
      summary.warn=tracking_stats.warned_this_run;
      summary.severe=tracking_stats.severe_this_run;
    }
    // Old V8 reported fabs(old_target - current_position) as final_err for
    // REPLACED. That is remaining travel, NOT tracking error and NOT final
    // positioning precision. Keep physically distinct metrics instead.
    const float measured=motor1.shaft_angle;
    if(summary.replaced) {
      summary.final_err=NAN;
      if(isfinite(measured)) {
        summary.old_target_remaining=fabsf(target-measured);
        if(isfinite(previous_plan_x))
          summary.takeover_track_err=fabsf(previous_plan_x-measured);
      }
    } else {
      summary.final_err=isfinite(measured)?fabsf(target-measured):NAN;
    }
    last_run_summary=summary;
    if(summary.replaced)last_interrupt_summary=summary;
    active=false;
    if(!csv_on) {
      tx_printf("#RUN_DONE id=%lu type=%c result=%s target=%.3f elapsed_ms=%lu\n",
        (unsigned long)summary.run,summary.kind,summary.result,summary.target,
        (unsigned long)summary.elapsed_ms);
      if(summary.replaced) {
        tx_printf("#RUN_METRIC rmse=%.5f max_err=%.5f final_err=NA samples=%lu motion_ms=%lu plan_peak_v=%.3f warn=%u severe=%u\n",
          summary.rmse,summary.max_err,
          (unsigned long)summary.sample_count,(unsigned long)summary.weighted_ms,
          summary.peak_v,(unsigned)summary.warn,(unsigned)summary.severe);
        tx_printf("#INTERRUPT old_target_remaining=%.5f takeover_track_err=%.5f\n",
          summary.old_target_remaining,summary.takeover_track_err);
      } else {
        tx_printf("#RUN_METRIC rmse=%.5f max_err=%.5f final_err=%.5f samples=%lu motion_ms=%lu plan_peak_v=%.3f warn=%u severe=%u\n",
          summary.rmse,summary.max_err,summary.final_err,
          (unsigned long)summary.sample_count,(unsigned long)summary.weighted_ms,
          summary.peak_v,(unsigned)summary.warn,(unsigned)summary.severe);
      }
    }
  }

void RunQualityStats::start(uint32_t id,char mode,float to,float planned_peak,
             uint32_t solve_cost_us,uint32_t command_cost_us,
             float previous_plan_x) {
    // On an N/P takeover the caller passes the ORIGINAL planned position,
    // captured before changing profile_mode. Do not evaluate the new profile
    // to characterize the old run's tracking error.
    if(active)finish(plan_finished?"NOT_SETTLED":"REPLACED",previous_plan_x);
    active=true;plan_finished=false;settling=false;
    run=id;kind=mode;target=to;peak_v=planned_peak;
    begin_us=micros();last_sample_ms=millis();
    plan_end_ms=settled_from_ms=0;
    weighted_ms=sample_count=invalid=0;max_err=0;
    sum_e2_dt=0;
    solve_us=solve_cost_us;command_us=command_cost_us;
  }

void RunQualityStats::update() {
    if(!active)return;
    const uint32_t now_ms=millis();
    const bool moving=profile_active();
    if(moving && !plan_finished) {
      const uint32_t dt=(uint32_t)(now_ms-last_sample_ms);
      if(dt<RUN_METRIC_SAMPLE_MS)return;
      last_sample_ms=now_ms;
      if(dt>RUN_METRIC_MAX_DT_MS){++invalid;return;}
      if(!isfinite(theta_d) || !isfinite(motor1.shaft_angle)) {
        ++invalid;return;
      }
      const float e=theta_d-motor1.shaft_angle;
      const float ae=fabsf(e);
      if(ae>max_err)max_err=ae;
      sum_e2_dt+=(double)e*(double)e*(double)dt;
      weighted_ms+=dt;
      ++sample_count;
      return;
    }
    if(!plan_finished){
      plan_finished=true;
      plan_end_ms=now_ms;
      settling=false;
    }
    if(!isfinite(motor1.shaft_angle)){
      settling=false;
    }else if(fabsf(target-motor1.shaft_angle)<=RUN_SETTLE_ERR_RAD){
      if(!settling){settling=true;settled_from_ms=now_ms;}
      if((uint32_t)(now_ms-settled_from_ms)>=RUN_SETTLE_HOLD_MS){
        finish("SETTLED");return;
      }
    } else settling=false;
    if((uint32_t)(now_ms-plan_end_ms)>=RUN_SETTLE_TIMEOUT_MS)
      finish("NOT_SETTLED");
  }

void clear_v8_diagnostics() {
  loop_timing=LoopTimingStats();
  tracking_stats=TrackingStats();
  last_run_summary=RunSummary();
  last_interrupt_summary=RunSummary();
  run_quality=RunQualityStats();  // D allowed only when no run is pending.
  iq_abs_peak=0; iq_ref_abs_peak=0; iq_warn_since=0;
  iq_warn_samples=0; iq_warning=false;
  online_max_error=0; online_takeovers=0;
  online_solve_last_us=0; online_solve_max_us=0; online_solve_slow_count=0;
  online_command_last_us=0; online_command_max_us=0;
  online_command_success_count=0;
  // Lifetime accepted/rejected and TX-drop counters are intentionally retained.
}
