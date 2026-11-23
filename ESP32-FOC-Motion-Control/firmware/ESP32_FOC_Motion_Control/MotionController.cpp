// SimpleFOC is an external dependency; this repository does not vendor it.
#include "Runtime.h"

float ff_gain = 1.0f;
float traj_vmax = 2.0f, traj_amax = 4.0f;

bool auto_running = false;
char auto_kind = 'B';  // 'B'=trapezoid, 'C'=S-curve
bool auto_cancel_requested = false;
float auto_origin = 0.0f, auto_far = 0.0f;
uint8_t auto_legs_started = 0, auto_legs_total = 0;
uint32_t auto_wait_begin_ms = 0;

uint32_t run_id=0,run_started_us=0,settle_candidate_ms=0;

ProfileMode profile_mode=PROFILE_TRAPEZOID;
float scurve_jmax=120.0f; // active C trajectory; fixed P jerk is STOP_JERK_MAX
float online_last_j=0,online_last_target=0,online_last_v0=0,online_last_a0=0;
float online_last_brake_distance=0,online_last_peak_v=0;
bool online_cancelled_auto=false;
bool stop_soft_limit_warning=false;
// P guard diagnostic state; persistent after P ends until the next P/move.
// mode: 0=NONE,1=NORMAL,2=PROTECT,3=CONFLICT (protective J still insufficient).
uint8_t stop_guard_mode=0;
float stop_plan_jerk=STOP_JERK_MAX, stop_comfort_peak_v=0;
float stop_predicted_peak_v=0,stop_predicted_min_x=0,stop_predicted_max_x=0;
bool stop_original_speed_conflict=false,stop_original_range_conflict=false;
bool stop_remaining_speed_conflict=false;
float stop_takeover_error=0, stop_track_max_error=0;
float stop_filtered_actual_v=0, stop_track_max_abs_v=0;
uint32_t stop_track_warn_since_ms=0,stop_track_over_samples=0;
bool stop_track_warning=false;
uint32_t stop_track_last_us=0;
// Observability only. These are diagnostic thresholds, NOT validated hardware
// safety thresholds; actual protection requires hardware and thermal testing.

const char* stop_mode_name() {
  if(stop_guard_mode==1)return "NORMAL";
  if(stop_guard_mode==2)return "PROTECT";
  if(stop_guard_mode==3)return "CONFLICT";
  return "NONE";
}
char profile_letter() {
  if(profile_mode==PROFILE_SCURVE)return 'C';
  if(profile_mode==PROFILE_ONLINE)return 'N';
  if(profile_mode==PROFILE_SOFT_STOP)return 'P';
  return 'B';
}
bool profile_active() {
  if(profile_mode==PROFILE_SCURVE)return scurve.active;
  if(profile_mode==PROFILE_ONLINE)return online_profile.active;
  if(profile_mode==PROFILE_SOFT_STOP)return soft_stop.active;
  return trajectory.active;
}
uint8_t profile_phase(uint32_t now) {
  if(profile_mode==PROFILE_SCURVE)return scurve.phase(now);
  if(profile_mode==PROFILE_ONLINE)return online_profile.phase(now);
  if(profile_mode==PROFILE_SOFT_STOP)return soft_stop.phase(now);
  return trajectory.phase(now);
}
void profile_eval(uint32_t now,float &theta,float &omega) {
  if(profile_mode==PROFILE_SCURVE)scurve.eval(now,theta,omega);
  else if(profile_mode==PROFILE_ONLINE)online_profile.eval(now,theta,omega);
  else if(profile_mode==PROFILE_SOFT_STOP)soft_stop.eval(now,theta,omega);
  else trajectory.eval(now,theta,omega);
}
float profile_accel(uint32_t now) {
  if(profile_mode==PROFILE_ONLINE)return online_profile.accel(now);
  if(profile_mode==PROFILE_SOFT_STOP) {
    if(!soft_stop.active)return 0;
    const float t=(uint32_t)(now-soft_stop.begin_us)*1e-6f;
    if(t>=soft_stop.total)return 0;
    uint8_t i=0;
    while(i+1<soft_stop.nseg && t>=soft_stop.start_t[i+1])++i;
    return soft_stop.start_a[i]+soft_stop.j[i]*(t-soft_stop.start_t[i]);
  }
  if(profile_mode==PROFILE_SCURVE) {
    if(!scurve.active)return 0;
    const float t=(uint32_t)(now-scurve.begin_us)*1e-6f;
    if(t>=scurve.total)return 0;
    int i=0;
    while(i<6 && t>=scurve.t_start[i+1])++i;
    return scurve.sign*(scurve.a_start[i]+scurve.jerks[i]*(t-scurve.t_start[i]));
  }
  if(!trajectory.active)return 0;
  const float t=(uint32_t)(now-trajectory.begin_us)*1e-6f;
  if(t<trajectory.ta)return trajectory.sign*trajectory.a;
  if(t<trajectory.ta+trajectory.tc)return 0;
  if(t<trajectory.tt)return -trajectory.sign*trajectory.a;
  return 0;
}
void profile_hold(float angle) {
  trajectory.hold(angle);scurve.hold(angle);soft_stop.hold(angle);online_profile.hold(angle);
  profile_mode=PROFILE_TRAPEZOID;
  stop_soft_limit_warning=false;
}

float theta_d=0,omega_d=0,theta_cmd=0;

float motion_target_angle() {
  if(profile_mode==PROFILE_SCURVE)return scurve.to;
  if(profile_mode==PROFILE_ONLINE)return online_last_target;
  if(profile_mode==PROFILE_SOFT_STOP)return soft_stop.to;
  return trajectory.to;
}
const char* motion_state_name() {
  if(profile_active()) {
    if(profile_mode==PROFILE_SCURVE)return "C";
    if(profile_mode==PROFILE_ONLINE)return "N";
    if(profile_mode==PROFILE_SOFT_STOP)return "P";
    return "B";
  }
  if(auto_running)return "AUTO_WAIT";
  return "IDLE";
}

// S: just the fields used in ordinary real-hardware tests; no repeated help.

// Avoid single-sample velocity threshold: AS5600 estimates are sometimes spiky.
// A new move requires the previous *planned* motion to end, position to settle,
// and a holding interval. No trajectory is restarted at nonzero planned speed.
bool start_move(float to,char op) {
  if(auto_running && op!=auto_kind) {reject(op,7);return false;}
  if(!foc_ok) {reject(op,1);return false;}
  if(!isfinite(to)||fabsf(to)>MAX_ABS_ANGLE) {reject(op,2);return false;}
  if(profile_active()) {reject(op,3);return false;}
  if((uint32_t)(millis()-settle_candidate_ms)<HOLD_BEFORE_MOVE_MS ||
      fabsf(theta_d-motor1.shaft_angle)>HOLD_ERROR_LIMIT) {
    reject(op,4); return false;
  }
  float from=motor1.shaft_angle;
  if(fabsf(to-from)<0.005f) {reject(op,5);return false;}
  // Stop-envelope preflight occurs at move BEGIN, never inside loopFOC.
  if(!preflight_move_envelope(from,to,op=='C'?'C':'B',traj_vmax,traj_amax,scurve_jmax)) {
    if(!csv_on)tx_printf("#LIMIT envelope preflight rejected %c to=%.3f (limit %.1f)\n",op,to,MAX_ABS_ANGLE);
    reject(op,2);return false;
  }
  uint32_t t=micros();
  if(op=='C') {
    if(!scurve.start(from,to,traj_vmax,traj_amax,scurve_jmax,t)) {
      reject(op,6);return false;
    }
    profile_mode=PROFILE_SCURVE;
  } else {
    trajectory.start(from,to,traj_vmax,traj_amax,t);
    profile_mode=PROFILE_TRAPEZOID;
  }
  stop_soft_limit_warning=false;
  stop_guard_mode=0;
  run_started_us=t; ++run_id; ack(op); settle_candidate_ms=millis();
  run_quality.start(run_id,profile_letter(),to,
    profile_mode==PROFILE_SCURVE?scurve.peak_v:trajectory.vpk,0,0);
  if(!csv_on) tx_printf("#MOVE id=%lu type=%c to=%.3f V=%.3f A=%.3f J=%.2f\n",
    (unsigned long)run_id,profile_mode==PROFILE_SCURVE?'C':'B',
    to,traj_vmax,traj_amax,profile_mode==PROFILE_SCURVE?scurve_jmax:0.0f);
  return true;
}
// B0 is intentionally a two-character command (not an instant emergency stop).
// If a leg is moving, allow that already-planned leg to decelerate normally;
// do not start any additional leg.
void cancel_auto(char op='B') {
  if(!auto_running) {ack(op); if(!csv_on) tx_printf("#AUTO IDLE\n"); return;}
  auto_cancel_requested = true;
  ack(op);
  if(!csv_on) tx_printf("#AUTO CANCEL requested; current leg will finish\n");
}

// Strict comma-delimited format: B<speed>,<acceleration>,<cruise_seconds>,<roundtrips>
// Example: B6,8,1.5,3 gives D=6*6/8+6*1.5=13.5 rad; 6 legs total.
void command_auto(const char* value) {
  if(strcmp(value,"0")==0) {cancel_auto();return;}
  if(auto_running) {reject('B',7);return;}
  if(!foc_ok) {reject('B',1);return;}

  char* p=nullptr;
  float v=strtof(value,&p);
  if(p==value || *p!=',') {reject('B',6);return;}
  const char* a_start=p+1;
  float a=strtof(a_start,&p);
  if(p==a_start || *p!=',') {reject('B',6);return;}
  const char* cruise_start=p+1;
  float cruise=strtof(cruise_start,&p);
  if(p==cruise_start || *p!=',') {reject('B',6);return;}
  const char* n_start=p+1;
  long n=strtol(n_start,&p,10);
  if(p==n_start || *p!='\0' || !isfinite(v) || !isfinite(a) || !isfinite(cruise) ||
     v<=0.0f || v>INNER_VEL_LIMIT || a<=0.0f || a>30.0f ||
     cruise<0.25f || cruise>5.0f || n<1 || n>AUTO_MAX_ROUNDTRIPS) {
    reject('B',6);return;
  }
  // Valid trapezoid with an intentionally nonzero constant-speed segment.
  float distance=v*v/a+v*cruise;
  float origin=motor1.shaft_angle;
  float far=origin+distance;
  if(!isfinite(distance) || !isfinite(far) || distance<0.01f ||
     fabsf(origin)>MAX_ABS_ANGLE || fabsf(far)>MAX_ABS_ANGLE) {
    reject('B',2);return;
  }
  // Check the original inter-move guard before changing any settings.
  if(profile_active()) {reject('B',3);return;}
  if((uint32_t)(millis()-settle_candidate_ms)<HOLD_BEFORE_MOVE_MS ||
     fabsf(theta_d-motor1.shaft_angle)>HOLD_ERROR_LIMIT) {
    reject('B',4);return;
  }

  traj_vmax=v;
  traj_amax=a;
  auto_origin=origin;
  auto_far=far;
  auto_legs_started=0;
  auto_legs_total=(uint8_t)(2*n);
  auto_wait_begin_ms=0;
  auto_cancel_requested=false;
  auto_kind='B';
  auto_running=true;
  if(!start_move(auto_far,'B')) {
    auto_running=false;
    return;
  }
  auto_legs_started=1;
  if(!csv_on) tx_printf("#AUTO START V=%.2f A=%.2f cruise=%.2fs D=%.3frad N=%ld legs=%u\n",
    v,a,cruise,distance,n,(unsigned)auto_legs_total);
}

// C<Vmax>,<Amax>,<Jmax>,<distance_rad>,<roundtrips>
// The D field is an explicit travel DISTANCE, not a cruise time.
// Use the same distance as B to make trapezoid-vs-S-curve comparisons fair.
// C0 is an alias of B0 and cancels future legs only.
void command_scurve_auto(const char* value) {
  // Print only in TEXT mode; numeric CSV must remain exactly 21 columns.
  // Distinguishes firmware not recognizing C from C being rejected by a guard.
  if(!csv_on) tx_printf("#C_RX raw=%s\n",value);
  if(strcmp(value,"0")==0) {cancel_auto('C');return;}
  if(strcmp(value,"?")==0) {
    if(!csv_on) tx_printf("#C_HELP C2,4,20,3,1 ; C10,8,80,27.5,3\n");
    return;
  }
  if(auto_running) {reject('C',7);return;}
  if(!foc_ok) {reject('C',1);return;}

  char* p=nullptr;
  const float v=strtof(value,&p);
  if(p==value || *p!=',') {reject('C',6);return;}
  const char* a_begin=p+1;
  const float a=strtof(a_begin,&p);
  if(p==a_begin || *p!=',') {reject('C',6);return;}
  const char* j_begin=p+1;
  const float j=strtof(j_begin,&p);
  if(p==j_begin || *p!=',') {reject('C',6);return;}
  const char* d_begin=p+1;
  const float d=strtof(d_begin,&p);
  if(p==d_begin || *p!=',') {reject('C',6);return;}
  const char* n_begin=p+1;
  const long n=strtol(n_begin,&p,10);
  if(p==n_begin || *p!='\0' || !isfinite(v) || !isfinite(a) ||
     !isfinite(j) || !isfinite(d) ||
     v<=0.0f || v>INNER_VEL_LIMIT || a<=0.0f || a>30.0f ||
     j<1.0f || j>2000.0f || d<0.01f ||
     n<1 || n>AUTO_MAX_ROUNDTRIPS) {
    reject('C',6);return;
  }
  const float origin=motor1.shaft_angle;
  const float far=origin+d;
  if(!isfinite(far)||fabsf(origin)>MAX_ABS_ANGLE||fabsf(far)>MAX_ABS_ANGLE) {
    if(!csv_on) tx_printf("#C_RANGE origin=%.3f far=%.3f limit=%.2f\n",origin,far,MAX_ABS_ANGLE);
    reject('C',2);return;
  }
  if(profile_active()) {reject('C',3);return;}
  if((uint32_t)(millis()-settle_candidate_ms)<HOLD_BEFORE_MOVE_MS ||
     fabsf(theta_d-motor1.shaft_angle)>HOLD_ERROR_LIMIT) {
    if(!csv_on) tx_printf("#C_WAIT hold_ms=%lu poserr=%.4f (need >=600ms and <=0.25rad)\n",
      (unsigned long)(millis()-settle_candidate_ms),fabsf(theta_d-motor1.shaft_angle));
    reject('C',4);return;
  }

  // Parameter change is committed only after every guard has passed.
  traj_vmax=v;
  traj_amax=a;
  scurve_jmax=j;
  auto_origin=origin;
  auto_far=far;
  auto_legs_started=0;
  auto_legs_total=(uint8_t)(2*n);
  auto_wait_begin_ms=0;
  auto_cancel_requested=false;
  auto_kind='C';
  auto_running=true;
  if(!start_move(auto_far,'C')) {
    auto_running=false;
    return;
  }
  auto_legs_started=1;
  if(!csv_on) tx_printf("#AUTO C START V=%.3f A=%.3f J=%.3f D=%.3f N=%ld legs=%u peak_v=%.3f total=%.3fs\n",
    v,a,j,d,n,(unsigned)auto_legs_total,scurve.peak_v,scurve.total);
}

// Called once per main loop, AFTER loopFOC/move and serial receive.
// Only starts the next leg after prior trajectory ends and position has been
// continuously within 0.25 rad for at least 600 ms.
void auto_update() {
  if(!auto_running) return;
  if(profile_active()) {auto_wait_begin_ms=0;return;}
  const uint32_t now=millis();
  if(auto_wait_begin_ms==0) auto_wait_begin_ms=now;
  // Cancelled: never start another leg; currently planned leg has finished.
  if(auto_cancel_requested) {
    auto_running=false;
    auto_cancel_requested=false;
    if(!csv_on) tx_printf("#AUTO CANCELLED after leg %u/%u\n",
      (unsigned)auto_legs_started,(unsigned)auto_legs_total);
    return;
  }
  const bool within_error=fabsf(theta_d-motor1.shaft_angle)<=HOLD_ERROR_LIMIT;
  const bool settled=within_error &&
    (uint32_t)(now-settle_candidate_ms)>=HOLD_BEFORE_MOVE_MS;
  if(!settled) {
    if((uint32_t)(now-auto_wait_begin_ms)>=AUTO_SETTLE_TIMEOUT_MS) {
      auto_running=false;
      last_reject=8;last_op=auto_kind;
      if(!csv_on) tx_printf("#AUTO ABORT settle timeout after leg %u/%u\n",
        (unsigned)auto_legs_started,(unsigned)auto_legs_total);
    }
    return;
  }
  if(auto_legs_started>=auto_legs_total) {
    auto_running=false;
    if(!csv_on) tx_printf("#AUTO DONE legs=%u run=%lu\n",
      (unsigned)auto_legs_started,(unsigned long)run_id);
    return;
  }
  // Leg 0: outward (started in B or C command); leg 1: return;
  // leg 2: outward; leg 3: return; ...
  const float next=(auto_legs_started%2==1)?auto_origin:auto_far;
  auto_wait_begin_ms=0;
  if(!start_move(next,auto_kind)) {
    auto_running=false;
    if(!csv_on) tx_printf("#AUTO ABORT next leg rejected\n");
    return;
  }
  ++auto_legs_started;
}

// P: interrupt the current planned movement with bounded-A/J deceleration.
// The new profile starts at the previous planned x,v,a (NOT spiky measured
// AS5600 velocity), so theta_cmd remains continuous across the takeover.
// Automatic B/C future legs are cancelled on successful P.
// This cannot prevent travel-limit overshoot if the stopping distance is too
// large; an external hardware cut-off is needed for dangerous conditions.
void command_soft_stop() {
  if(!foc_ok) {reject('P',1);return;}
  if(profile_mode==PROFILE_SOFT_STOP && soft_stop.active) {
    reject('P',3);return;
  }
  const uint32_t now_us=micros();
  float x=0,v=0;
  profile_eval(now_us,x,v);
  const float a=profile_accel(now_us);
  if(!isfinite(x)||!isfinite(v)||!isfinite(a)) {
    reject('P',6);return;
  }
  if(!profile_active() && fabsf(v)<1e-4f && fabsf(a)<1e-4f) {
    auto_running=false;auto_cancel_requested=false;auto_wait_begin_ms=0;
    ack('P');
    if(!csv_on)tx_printf("#P IDLE auto cancelled; target=%.3f\n",theta_d);
    return;
  }

  // Compute the NORMAL J140 stop first without mutating the live trajectory.
  SoftStop normal;
  if(!normal.start(x,v,a,traj_amax,STOP_JERK_MAX,now_us)) {
    reject('P',6);return;
  }
  const bool speed_risk=normal.predicted_peak_v>INNER_VEL_LIMIT+STOP_BOUND_EPS;
  const bool range_risk=normal.predicted_min_x<-MAX_ABS_ANGLE-STOP_BOUND_EPS ||
                         normal.predicted_max_x>MAX_ABS_ANGLE+STOP_BOUND_EPS;
  SoftStop chosen=normal;
  float chosen_jerk=STOP_JERK_MAX;
  uint8_t chosen_mode=1;
  // With outward acceleration at max speed, no finite jerk can strictly
  // guarantee no overshoot AND keep a(t) continuous. This higher-J profile
  // REDUCES predicted excursion, but is NOT guaranteed to eliminate it.
  if(speed_risk || range_risk) {
    SoftStop protective;
    if(protective.start(x,v,a,traj_amax,STOP_PROTECT_JERK_MAX,now_us)) {
      // Do not trade a NEW travel-limit violation for a modest speed benefit.
      const float normal_range_over=fmaxf(0.0f,-MAX_ABS_ANGLE-normal.predicted_min_x)+
                                    fmaxf(0.0f,normal.predicted_max_x-MAX_ABS_ANGLE);
      const float protect_range_over=fmaxf(0.0f,-MAX_ABS_ANGLE-protective.predicted_min_x)+
                                     fmaxf(0.0f,protective.predicted_max_x-MAX_ABS_ANGLE);
      const bool improves_travel=protect_range_over<normal_range_over-0.001f;
      const bool improves_speed=protective.predicted_peak_v<normal.predicted_peak_v-0.001f;
      if(improves_travel ||
         (protect_range_over<=normal_range_over+0.001f && improves_speed)) {
        chosen=protective;
        chosen_jerk=STOP_PROTECT_JERK_MAX;
        chosen_mode=2;
      }
    }
  }
  const bool residual_speed=chosen.predicted_peak_v>INNER_VEL_LIMIT+STOP_BOUND_EPS;
  const bool residual_range=chosen.predicted_min_x<-MAX_ABS_ANGLE-STOP_BOUND_EPS ||
                            chosen.predicted_max_x>MAX_ABS_ANGLE+STOP_BOUND_EPS;
  if(residual_speed||residual_range) chosen_mode=3;

  // A guard cannot reject P simply because a stop is mathematically infeasible:
  // that would leave the previous high-speed movement running. Use the shortest
  // available profile and report any unresolved physical constraint conflict.
  soft_stop=chosen;
  stop_original_speed_conflict=speed_risk;
  stop_original_range_conflict=range_risk;
  stop_remaining_speed_conflict=residual_speed;
  stop_soft_limit_warning=residual_range;
  stop_guard_mode=chosen_mode;
  stop_plan_jerk=chosen_jerk;
  stop_comfort_peak_v=normal.predicted_peak_v;
  stop_predicted_peak_v=soft_stop.predicted_peak_v;
  stop_predicted_min_x=soft_stop.predicted_min_x;
  stop_predicted_max_x=soft_stop.predicted_max_x;
  stop_takeover_error=fabsf(x-motor1.shaft_angle);
  stop_track_max_error=stop_takeover_error;
  stop_track_max_abs_v=fabsf(stop_filtered_actual_v);
  stop_track_warn_since_ms=0;stop_track_over_samples=0;stop_track_warning=false;
  profile_mode=PROFILE_SOFT_STOP;
  auto_running=false;auto_cancel_requested=false;auto_wait_begin_ms=0;
  run_started_us=now_us;++run_id;ack('P');
  settle_candidate_ms=millis();
  run_quality.start(run_id,'P',soft_stop.to,soft_stop.predicted_peak_v,0,0,x);
  if(!csv_on) {
    tx_printf("#P START id=%lu mode=%s from=%.3f v=%.3f a=%.3f to=%.3f duration=%.3fs J=%.0f\n",
      (unsigned long)run_id,stop_mode_name(),x,v,a,soft_stop.to,soft_stop.total,chosen_jerk);
    tx_printf("#P_GUARD peak_normal=%.3f peak_used=%.3f v_limit=%.1f min_x=%.3f max_x=%.3f\n",
      normal.predicted_peak_v,soft_stop.predicted_peak_v,INNER_VEL_LIMIT,
      soft_stop.predicted_min_x,soft_stop.predicted_max_x);
    if(chosen_mode==3)
      tx_printf("#WARN P_CONSTRAINT_CONFLICT speed=%u travel=%u; P IS NOT AN E-STOP\n",
        (unsigned)residual_speed,(unsigned)residual_range);
    else if(chosen_mode==2)
      tx_printf("#WARN P_PROTECT uses J=%.0f instead of %.0f; confirm hardware capability\n",
        chosen_jerk,STOP_JERK_MAX);
    if(stop_takeover_error>STOP_TRACK_WARN_RAD)
      tx_printf("#WARN P_TAKEOVER_TRACK_ERR abs=%.3f\n",stop_takeover_error);
  }
}

// V6 N: B/C/P/N can be retargeted repeatedly while moving.
// A freshly solved candidate is validated BEFORE the active profile changes.
// In particular, a close or behind goal can now brake, overshoot and return.
// A physically infeasible V/A/J/soft-limit request is still rejected safely.
void command_online_target(const char* value) {
  const uint32_t command_begin_us=micros();
  if(!foc_ok){reject('N',1);return;}
  char* end=nullptr;
  const float target=strtof(value,&end);
  if(end==value||*end!='\0'||!isfinite(target)){reject('N',6);return;}
  if(fabsf(target)>MAX_ABS_ANGLE-SOFT_TRAVEL_GUARD_MARGIN){reject('N',2);return;}
  if(!profile_active()) {reject('N',3);return;}
  if(traj_vmax>ONLINE_MAX_TEST_V+0.001f || traj_amax>ONLINE_MAX_TEST_A+0.001f) {
    reject('N',6);return;
  }
  const uint32_t now=micros();
  float x,v;profile_eval(now,x,v);
  const float a=profile_accel(now);
  if(!isfinite(x)||!isfinite(v)||!isfinite(a)){reject('N',6);return;}
  const float err=fabsf(x-motor1.shaft_angle);
  if(err>ONLINE_TAKEOVER_MAX_ERR_RAD) {
    if(!csv_on)tx_printf("#N_TRACK_ERR abs=%.3f limit=%.3f; old profile unchanged\n",
      err,ONLINE_TAKEOVER_MAX_ERR_RAD);
    reject('N',12);return;
  }
  // Keep C's lower jerk if commanded; use the existing N jerk on repeated N.
  float jerk=ONLINE_DEFAULT_J;
  if(profile_mode==PROFILE_SCURVE)jerk=fminf(scurve_jmax,ONLINE_DEFAULT_J);
  else if(profile_mode==PROFILE_ONLINE)jerk=online_last_j;
  // P with a J600 protective takeover is not a validated source for N yet.
  else if(profile_mode==PROFILE_SOFT_STOP && stop_plan_jerk>ONLINE_DEFAULT_J+0.1f){
    if(!csv_on)tx_printf("#N_PROTECT_STOP_ACTIVE J=%.1f; wait or keep P\n",stop_plan_jerk);
    reject('N',14);return;
  }
  OnlineFullProfile candidate;
  const uint32_t solve_begin_us=micros();
  const bool solve_ok=candidate.start(x,v,a,target,traj_vmax,traj_amax,jerk,now);
  online_solve_last_us=(uint32_t)(micros()-solve_begin_us);
  online_solve_max_us= (online_solve_last_us>online_solve_max_us)?
                       online_solve_last_us:online_solve_max_us;
  if(online_solve_last_us>ONLINE_SOLVE_WARN_US)++online_solve_slow_count;
  if(!solve_ok) {
    if(!csv_on)tx_printf("#N_REJECT x=%.3f v=%.3f a=%.3f target=%.3f why=%u predicted_peak=%.3f; old profile kept\n",
      x,v,a,target,(unsigned)candidate.failure,candidate.predicted_illegal_peak);
    reject('N',candidate.failure==OnlineFullProfile::SPEED_CONFLICT?14:11);
    return;
  }
  online_profile=candidate;
  online_last_j=jerk;online_last_target=target;
  online_last_v0=v;online_last_a0=a;
  online_last_brake_distance=candidate.brake_distance;
  online_last_peak_v=candidate.peak_speed;
  online_cancelled_auto=auto_running;
  auto_running=false;auto_cancel_requested=false;auto_wait_begin_ms=0;
  profile_mode=PROFILE_ONLINE;
  run_started_us=now;++run_id;ack('N');++online_takeovers;
  online_max_error=err;
  online_command_last_us=(uint32_t)(micros()-command_begin_us);
  if(online_command_last_us>online_command_max_us)
    online_command_max_us=online_command_last_us;
  ++online_command_success_count;
  run_quality.start(run_id,'N',target,candidate.peak_speed,
                    online_solve_last_us,online_command_last_us,x);
  settle_candidate_ms=millis();
  if(!csv_on) {
    tx_printf("#N START id=%lu mode=%s from=%.3f v=%.3f a=%.3f target=%.3f peak=%.3f duration=%.3f J=%.1f\n",
      (unsigned long)run_id,candidate.mode_name(),x,v,a,target,candidate.peak_speed,candidate.total,jerk);
    tx_printf("#N_PATH min_x=%.3f max_x=%.3f stop_at=%.3f stop_s=%.3f; auto cancelled\n",
      candidate.min_x,candidate.max_x,candidate.braking.to,candidate.brake_time);
    if(online_solve_last_us>ONLINE_SOLVE_WARN_US)
      tx_printf("#WARN N_SOLVE_SLOW us=%lu; evaluate control-loop timing\n",
        (unsigned long)online_solve_last_us);
  }
}

