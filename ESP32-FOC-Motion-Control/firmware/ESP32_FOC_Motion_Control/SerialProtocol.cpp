// Module extracted from the user's V8.1 single-axis firmware.
// Refactor only: keep original motion formulas, control parameters and sequence.
// SimpleFOC is an external dependency; this repository does not vendor it.
#include "Runtime.h"

uint32_t accepted=0,rejected=0;
uint8_t last_reject=0;
char last_op=0;

void ack(char op) { ++accepted;last_op=op;last_reject=0; }
// Reject codes: 1=FOC-not-ready 2=angle-limit 3=trajectory-active
// 4=not-settled 5=already-there 6=invalid-parameters 7=auto-busy
// 8=auto-settle-timeout 9=serial-invalid-character 10=serial-overflow
void reject(char op,uint8_t why) {
  ++rejected;last_op=op;last_reject=why;
  if(!csv_on) tx_printf("#REJECT op=%c reason=%u\n",op,(unsigned)why);
}

void status_brief() {
  csv_on=false;tx_clear();
  const uint32_t gap_avg=loop_timing.gap_count?
    (uint32_t)(loop_timing.gap_sum_us/loop_timing.gap_count):0;
  const uint32_t foc_avg=loop_timing.foc_count?
    (uint32_t)(loop_timing.foc_sum_us/loop_timing.foc_count):0;
  tx_printf("#VERSION %s\n",FW_VERSION);
  tx_printf("#MOTOR ready=%u enabled=%u theta=%.4f\n",
    (unsigned)foc_ok,(unsigned)motor1.enabled,motor1.shaft_angle);
  tx_printf("#MOTION mode=%s run=%lu target=%.4f active=%u\n",
    motion_state_name(),(unsigned long)run_id,motion_target_angle(),
    (unsigned)profile_active());
  tx_printf("#PERF loop_avg_us=%lu loop_max_us=%lu foc_avg_us=%lu foc_max_us=%lu\n",
    (unsigned long)gap_avg,(unsigned long)loop_timing.gap_max_us,
    (unsigned long)foc_avg,(unsigned long)loop_timing.foc_max_us);
  tx_printf("#N_PERF solve_last_us=%lu solve_max_us=%lu cmd_last_us=%lu cmd_max_us=%lu\n",
    (unsigned long)online_solve_last_us,(unsigned long)online_solve_max_us,
    (unsigned long)online_command_last_us,(unsigned long)online_command_max_us);
  if(last_run_summary.valid)
    tx_printf("#TRACK last_rmse=%.5f last_max=%.5f warn=%lu severe=%lu\n",
      last_run_summary.rmse,last_run_summary.max_err,
      (unsigned long)tracking_stats.warn_events,
      (unsigned long)tracking_stats.severe_events);
  else tx_printf("#TRACK last_rmse=NA last_max=NA warn=%lu severe=%lu\n",
      (unsigned long)tracking_stats.warn_events,
      (unsigned long)tracking_stats.severe_events);
  tx_printf("#HEALTH iq_peak=%.4f iq_warn=%u rejects=%lu drops=%lu\n",
    iq_abs_peak,(unsigned)iq_warning,(unsigned long)rejected,(unsigned long)tx_drops);
  if(last_run_summary.valid) {
    tx_printf("#RUN_LAST id=%lu type=%c result=%s target=%.3f rmse=%.5f\n",
      (unsigned long)last_run_summary.run,last_run_summary.kind,
      last_run_summary.result,last_run_summary.target,last_run_summary.rmse);
  } else tx_printf("#RUN_LAST none\n");
}

// I: extended diagnostic view; leaves B/C/P/N control and 21-channel CSV alone.
void status_detailed() {
  status_brief();
  tx_printf("#PARAM V=%.3f A=%.3f FF=%.3f accepted=%lu last_op=%c why=%u\n",
    traj_vmax,traj_amax,ff_gain,(unsigned long)accepted,
    last_op?last_op:'-',(unsigned)last_reject);
  tx_printf("#AUTO kind=%c running=%u cancel=%u legs=%u/%u origin=%.3f far=%.3f J=%.1f\n",
    auto_kind,(unsigned)auto_running,(unsigned)auto_cancel_requested,
    (unsigned)auto_legs_started,(unsigned)auto_legs_total,auto_origin,auto_far,scurve_jmax);
  tx_printf("#PROFILE kind=%c theta_plan=%.4f omega_plan=%.4f theta_cmd=%.4f phase=%u\n",
    profile_letter(),theta_d,omega_d,theta_cmd,(unsigned)profile_phase(micros()));
  tx_printf("#STOP active=%u mode=%s target=%.3f duration=%.3f J=%.1f\n",
    (unsigned)soft_stop.active,stop_mode_name(),soft_stop.to,soft_stop.total,stop_plan_jerk);
  tx_printf("#STOP_PRED peak_normal=%.3f peak_used=%.3f limit=%.1f conflict=%u residual=%u\n",
    stop_comfort_peak_v,stop_predicted_peak_v,INNER_VEL_LIMIT,
    (unsigned)stop_original_speed_conflict,(unsigned)stop_remaining_speed_conflict);
  tx_printf("#STOP_RANGE min_x=%.3f max_x=%.3f limit=%.1f original_risk=%u remain_risk=%u\n",
    stop_predicted_min_x,stop_predicted_max_x,MAX_ABS_ANGLE,
    (unsigned)stop_original_range_conflict,(unsigned)stop_soft_limit_warning);
  tx_printf("#STOP_TRACK takeover=%.3f max=%.3f warned=%u samples_over=%lu vel_filt=%.3f\n",
    stop_takeover_error,stop_track_max_error,(unsigned)stop_track_warning,
    (unsigned long)stop_track_over_samples,stop_filtered_actual_v);
  tx_printf("#REPLAN active=%u mode=%s target=%.3f duration=%.3f peak=%.3f J=%.1f v0=%.3f a0=%.3f\n",
    (unsigned)online_profile.active,online_profile.mode_name(),online_last_target,
    online_profile.total,online_last_peak_v,online_last_j,online_last_v0,online_last_a0);
  tx_printf("#N_PATH stop_at=%.3f brake_s=%.3f min_x=%.3f max_x=%.3f takeovers=%lu\n",
    online_profile.braking.to,online_profile.brake_time,
    online_profile.min_x,online_profile.max_x,(unsigned long)online_takeovers);
  tx_printf("#N_PERF_DETAIL solve_slow=%lu budget_us=%lu cmd_success=%lu\n",
    (unsigned long)online_solve_slow_count,(unsigned long)ONLINE_SOLVE_WARN_US,
    (unsigned long)online_command_success_count);
  tx_printf("#HEALTH_DETAIL iq_ref_peak=%.4f iq_warn_samples=%lu\n",
    iq_ref_abs_peak,(unsigned long)iq_warn_samples);
  const uint32_t busy_avg=loop_timing.busy_count?
    (uint32_t)(loop_timing.busy_sum_us/loop_timing.busy_count):0;
  tx_printf("#LOOP_DETAIL busy_avg_us=%lu busy_max_us=%lu gap_slow=%lu budget_us=%lu gap_ctx=%c busy_ctx=%c\n",
    (unsigned long)busy_avg,(unsigned long)loop_timing.busy_max_us,
    (unsigned long)loop_timing.slow_gap_count,(unsigned long)LOOP_GAP_OBSERVE_US,
    loop_timing.worst_gap_tag,loop_timing.worst_busy_tag);
  tx_printf("#LOOP_SAMPLES gap=%lu foc=%lu busy=%lu\n",
    (unsigned long)loop_timing.gap_count,(unsigned long)loop_timing.foc_count,
    (unsigned long)loop_timing.busy_count);
  tx_printf("#TRACK_DETAIL kind=%c run=%lu running=%u max_all_np=%.4f invalid=%lu\n",
    tracking_stats.kind,(unsigned long)tracking_stats.run,
    (unsigned)tracking_stats.running,tracking_stats.max_err_all,
    (unsigned long)tracking_stats.invalid_samples);
  tx_printf("#TRACK_LIMIT warn=%.2f/%lums severe=%.2f/%lums action=LOG_ONLY\n",
    TRACK_WARN_RAD,(unsigned long)TRACK_WARN_HOLD_MS,
    TRACK_SEVERE_RAD,(unsigned long)TRACK_SEVERE_HOLD_MS);
  if(run_quality.active) {
    tx_printf("#RUN_ACTIVE id=%lu type=%c samples=%lu motion_ms=%lu max=%.4f plan_end=%u\n",
      (unsigned long)run_quality.run,run_quality.kind,
      (unsigned long)run_quality.sample_count,(unsigned long)run_quality.weighted_ms,
      run_quality.max_err,(unsigned)run_quality.plan_finished);
  }
  if(last_run_summary.valid) {
    if(last_run_summary.replaced) {
      tx_printf("#RUN_LAST_DETAIL id=%lu result=REPLACED motion_ms=%lu samples=%lu max=%.5f final=NA peak=%.3f remaining_old=%.5f takeover_track_err=%.5f solve_us=%lu cmd_us=%lu\n",
        (unsigned long)last_run_summary.run,
        (unsigned long)last_run_summary.weighted_ms,
        (unsigned long)last_run_summary.sample_count,
        last_run_summary.max_err,last_run_summary.peak_v,
        last_run_summary.old_target_remaining,last_run_summary.takeover_track_err,
        (unsigned long)last_run_summary.solve_us,
        (unsigned long)last_run_summary.command_us);
    } else {
      tx_printf("#RUN_LAST_DETAIL id=%lu result=%s motion_ms=%lu samples=%lu max=%.5f final=%.5f peak=%.3f solve_us=%lu cmd_us=%lu\n",
        (unsigned long)last_run_summary.run,last_run_summary.result,
        (unsigned long)last_run_summary.weighted_ms,
        (unsigned long)last_run_summary.sample_count,
        last_run_summary.max_err,last_run_summary.final_err,last_run_summary.peak_v,
        (unsigned long)last_run_summary.solve_us,
        (unsigned long)last_run_summary.command_us);
    }
  }
  if(last_interrupt_summary.valid) {
    tx_printf("#LAST_INTERRUPT id=%lu type=%c old_target=%.3f remaining_old=%.5f takeover_track_err=%.5f rmse=%.5f\n",
      (unsigned long)last_interrupt_summary.run,last_interrupt_summary.kind,
      last_interrupt_summary.target,last_interrupt_summary.old_target_remaining,
      last_interrupt_summary.takeover_track_err,last_interrupt_summary.rmse);
  }
}
void command_help() {
  csv_on=false;tx_clear();
  tx_printf("#HELP V8.1 Serial 115200; commands end with LF or 120ms idle\n");
  tx_printf("#HELP S=brief I=details ?=help D=reset diagnostics(IDLE only) O=toggle CSV21\n");
  tx_printf("#HELP C<V>,<A>,<J>,<distance>,<roundtrips> ex C2,4,60,8,3\n");
  tx_printf("#HELP B<V>,<A>,<cruise_s>,<roundtrips> ex B6,8,1.5,3\n");
  tx_printf("#HELP N<absolute_rad> retarget active B/C/P/N ex N12 or N-2\n");
  tx_printf("#HELP P=planned soft stop (NOT E-stop); B0/C0=cancel future legs\n");
  tx_printf("#HELP T<rad> direct absolute rest-to-rest; G=3.14 R=0 H=hold(stationary)\n");
  tx_printf("#HELP V<rad/s> A<rad/s2> F<ff> ex V4 A8 F1; motor2 disconnected\n");
  tx_printf("#HELP CSV i0..i20 unchanged from V7; i15=cached boot VIN\n");
}

void command_letter(char op) {
  // S/O count as accepted, but preserve the last movement command's diagnostic.
  // Previously ack('S') cleared the C reject reason before users could read it!
  current_loop_tag=op;
  if(op=='S') {++accepted;status_brief();return;}
  if(op=='I') {++accepted;status_detailed();return;}
  if(op=='?') {++accepted;command_help();return;}
  if(op=='O') {++accepted;csv_on=!csv_on;tx_clear();last_csv_ms=millis();return;}
  if(op=='P') {command_soft_stop();return;}
  if(op=='D') {
    if(profile_active() || auto_running || run_quality.active) {reject('D',3);return;}
    ++accepted;diag_reset_pending=true;
    if(!csv_on)tx_printf("#DIAG_RESET V8 experiment stats after this loop; lifetime rejects/drops kept\n");
    return;
  }
  if(!foc_ok) {reject(op,1);return;}
  if(auto_running && op=='H') {reject(op,7);return;}
  if(op=='G') start_move(3.14f,op);
  else if(op=='R') start_move(0.0f,op);
  else if(op=='H') {
    // Instant hold is unsafe during a moving N; P is the controlled-stop command.
    if(profile_active()) {reject('H',3);return;}
    profile_hold(motor1.shaft_angle);theta_d=motor1.shaft_angle;omega_d=0;
    settle_candidate_ms=millis();ack(op);
  }
}
void command_numeric(char op,const char* value) {
  current_loop_tag=op;
  if(op=='B') {command_auto(value);return;}
  if(op=='C') {command_scurve_auto(value);return;}
  if(op=='N') {command_online_target(value);return;}
  if(auto_running && op!='T') {reject(op,7);return;}
  // Never change V/A/FF during an active online plan: it invalidates bounds.
  if(profile_active() && (op=='V'||op=='A'||op=='F')) {reject(op,3);return;}
  char *end=nullptr;
  float x=strtof(value,&end);
  if(!*value || end==value || *end!='\0' || !isfinite(x)) {reject(op,6);return;}
  if(op=='T') {start_move(x,op);return;}
  if(op=='F' && x>=0 && x<=2) ff_gain=x;
  else if(op=='V' && x>0 && x<=INNER_VEL_LIMIT) traj_vmax=x;
  else if(op=='A' && x>0 && x<=30) traj_amax=x;
  else {reject(op,6);return;}
  ack(op);
  if(!csv_on) tx_printf("#SET %c=%.4f\n",op,x);
}
char rx_digits[56];
uint8_t rx_len=0;
char rx_op=0;
uint32_t rx_last_ms=0;
void finish_number() {
  if(!rx_op)return;
  rx_digits[rx_len]='\0';
  char op=rx_op;rx_op=0;rx_len=0;
  command_numeric(op,rx_digits);
}
void poll_rx() {
  for(uint8_t k=0;k<64 && Serial.available()>0;k++) {
    char c=(char)Serial.read();
    if(c=='\n'||c=='\r') {finish_number();continue;}
    if(c>='a'&&c<='z') c-=32;
    // Preserve V7's C? help alias: '?' belongs to the in-progress C command.
    if(c=='?' && rx_op=='C') {
      if(rx_len<sizeof(rx_digits)-1) {rx_digits[rx_len++]=c;rx_last_ms=millis();}
      else {rx_op=0;rx_len=0;reject('C',10);}
      continue;
    }
    bool one=(c=='S'||c=='I'||c=='?'||c=='O'||c=='G'||c=='R'||c=='H'||c=='P'||c=='D');
    bool number=(c=='F'||c=='V'||c=='A'||c=='T'||c=='B'||c=='C'||c=='N');
    if(one||number) {
      finish_number();
      if(one)command_letter(c);
      else {rx_op=c;rx_len=0;rx_last_ms=millis();}
      continue;
    }
    if(rx_op) {
      if((c>='0'&&c<='9')||c=='.'||c=='-'||c=='+'||c=='E'||
         ((rx_op=='B'||rx_op=='C')&&c==',')||
         (rx_op=='C'&&c=='?')) {
        if(rx_len<sizeof(rx_digits)-1) {rx_digits[rx_len++]=c;rx_last_ms=millis();}
        else {char bad=rx_op;rx_op=0;rx_len=0;reject(bad,10);}
      } else if(c==' '||c=='\t') {
        // VOFA sometimes appends trailing spaces; do not discard a complete C.
        rx_last_ms=millis();
      } else {
        char bad=rx_op;rx_op=0;rx_len=0;reject(bad,9);
      }
    }
  }
  if(rx_op && (uint32_t)(millis()-rx_last_ms)>=120) finish_number();
}
