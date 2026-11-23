#pragma once
#include "Config.h"
// Pure trajectory polynomial types. Member method bodies are unchanged from V8.1.
// A later separately tested change may move member definitions into .cpp.

struct Trapezoid {
  float from=0,to=0,travel=0,sign=1, a=0,vpk=0,ta=0,tc=0,tt=0;
  uint32_t begin_us=0;
  bool active=false,ever=false;
  void hold(float angle);
  void start(float x0,float x1,float vmax,float amax,uint32_t now);
  uint8_t phase(uint32_t now) const;
  void eval(uint32_t now,float &theta,float &omega);
};
// 7-piece symmetric, jerk-limited rest-to-rest profile.
// Assumptions: start/end velocity AND acceleration are zero; same limits in both directions.
// Compute all durations and segment-start states ONCE when a move starts.
// eval() uses closed-form polynomials; no numerical integration, I/O or allocations.
struct SCurve {
  float from=0, to=0, travel=0, sign=1;
  float vmax=0, amax=0, jmax=0, peak_v=0, peak_a=0;
  float dt[7]={0}, t_start[8]={0};
  float x_start[7]={0}, v_start[7]={0}, a_start[7]={0}, jerks[7]={0};
  float total=0;
  uint32_t begin_us=0;
  bool active=false, ever=false;

  void hold(float angle);

  static void accel_shape(float v,float amax,float jmax,
                          float &tj,float &ta,float &actual_a);
  static float no_cruise_distance(float v,float amax,float jmax);

  bool start(float x0,float x1,float maxv,float maxa,float maxj,uint32_t now);
  uint8_t phase(uint32_t now) const;
  void eval(uint32_t now,float &theta,float &omega);
};

// Jerk-bounded controlled stop from an arbitrary planned x,v,a state.
// It is NOT an emergency stop: measured speed can differ from planned speed.
// Uses exact constant-jerk polynomials. Acceleration is continuous at takeover.
// Usually 2/3 phases: ramp toward braking acceleration, optional -A hold,
// ramp acceleration back to zero. For a rare low-speed/strong-deceleration
// corner, release the existing acceleration first and brake any residual
// reverse velocity (max 4 segments). No delay(), I/O, malloc, or integration.
struct SoftStop {
  static const uint8_t MAX_SEGMENTS=5;
  float from=0,to=0,first_v=0,first_a=0;
  float start_t[6]={0}, start_x[5]={0}, start_v[5]={0};
  float start_a[5]={0}, j[5]={0}, dt[5]={0};
  float total=0, final_v=0, final_a=0;
  uint32_t begin_us=0;
  uint8_t nseg=0;
  bool active=false,ever=false;
  float predicted_peak_v=0, predicted_min_x=0, predicted_max_x=0;

  void hold(float x);
  float cur_x=0,cur_v=0,cur_a=0;
  bool add(float jerk,float seconds);
  // From current state, stop signed velocity u with bounded jerk and accel.
  // Uses a direct -J, optional -A plateau, +J profile when feasible.
  bool brake(float max_a,float max_j);
  // Exact segment-wise extrema: velocity peaks where a(t)=0;
  // position extremes occur where v(t)=0. All checked ONCE on P receipt.
  void update_extrema();
  bool start(float x,float v,float a,float max_a,float max_j,uint32_t now);
  uint8_t phase(uint32_t now) const;
  void eval(uint32_t now,float &theta,float &omega);
};

// Model-based preflight for future stops along B/C/T movement.
// Samples stages near both sides of each acceleration change.
// This assumes ideal tracking and cannot replace physical limit switches.
// Preflight checks: defined in TrajectoryPlanner.cpp.
bool stop_envelope_safe(float x,float v,float a,float max_a);
bool preflight_move_envelope(float x0,float x1,char mode,float v_max,float a_max,float jerk_max);

struct OnlineReplan {
  static const uint8_t MAX_SEGMENTS=9;
  float from=0,to=0,v0=0,a0=0,jmax=0,vmax=0,amax=0;
  float start_t[10]={0},dt[9]={0},jerks[9]={0};
  float start_x[9]={0},start_v[9]={0},start_a[9]={0};
  uint8_t stage_kind[9]={0}; // 1=accel/transition,2=cruise,3=decel
  float total=0,cur_x=0,cur_v=0,cur_a=0;
  float peak_speed=0,braking_distance=0,prefix_distance=0;
  float min_x=0,max_x=0;
  uint32_t begin_us=0;
  uint8_t nseg=0;
  bool active=false,ever=false;

  void hold(float x);
  bool add(float j,float seconds,uint8_t kind);
  static float shape_time(float dv,float a,float j);
  // Distance for accelerate from nonzero speed u to peak w, then brake w -> 0.
  static float distance_no_cruise(float u,float w,float a,float j);
  bool transition(float dir,float delta_v,float a,float j,uint8_t kind);
  // All velocity extrema occur at a=0, position extrema at v=0.
  // For the deliberately same-direction V5 planner, position is monotone.
  void extrema();
  // Returns false if goal too close, opposite to velocity, or velocity bound
  // cannot be met while continuously releasing the original acceleration.
  bool start(float x,float v,float a,float target,float maxv,float maxa,float maxj,uint32_t now);
  void eval_at(float t,float &x,float &v,float &a,uint8_t* phase_out=nullptr) const;
  void eval(uint32_t now,float &x,float &v);
  float accel(uint32_t now) const;
  uint8_t phase(uint32_t now) const;
  // V5: check the candidate trajectory and its potential P stopping envelope.
  // Keep this INSIDE OnlineReplan: Arduino auto-prototypes must not reference
  // OnlineReplan before its type definition. Rejection leaves B/C untouched.
  bool preflight() const;
};


// V6: online N goal planning. Direct branch reuses the V5 nonzero-state
// monotonic solution. For impossible-to-stop-at-goal / reverse targets,
// chain a bounded-jerk stop from the CURRENT x,v,a into a rest-to-rest S curve.
// This path may OVERSHOOT and reverse, and is continuous in x, v and a.
// It is feasible, but NOT globally time-optimal. Never clip v after planning.
// All solvers run only upon a N command, not in loopFOC().
struct OnlineFullProfile {
  enum : uint8_t { NONE=0, DIRECT=1, BRAKE_RETURN=2 };
  enum : uint8_t { OK=0, BAD_INPUT=1, SPEED_CONFLICT=2,
                   SOFT_LIMIT=3, SOLVER_FAIL=4, OVERLONG=5 };
  OnlineReplan direct;
  SoftStop braking;
  SCurve rest;
  uint8_t mode=NONE, failure=OK;
  uint32_t begin_us=0;
  float from=0,to=0,requested_to=0,v0=0,a0=0,total=0;
  float maxv=0,maxa=0,maxj=0,peak_speed=0,min_x=0,max_x=0;
  float brake_distance=0,brake_time=0, predicted_illegal_peak=0;
  bool rest_needed=false,active=false,ever=false;

  void hold(float x);
  // Pure polynomial sampling of the existing P braking profile.
  void braking_at(float t,float &x,float &v,float &a) const;
  // Pure polynomial sampling of the original rest-to-rest 7-segment profile.
  void rest_at(float t,float &x,float &v,float &a,uint8_t *phase_id=nullptr) const;
  bool start(float x,float v,float a,float target,float vmax,float amax,float jmax,uint32_t now);
  void eval_at(float t,float &x,float &v,float &a,uint8_t *phase_id=nullptr) const;
  void eval(uint32_t now,float &x,float &v);
  float accel(uint32_t now) const;
  uint8_t phase(uint32_t now) const;
  const char* mode_name() const;
};

