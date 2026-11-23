// SimpleFOC is an external dependency; this repository does not vendor it.
#include "TrajectoryPlanner.h"

Trapezoid trajectory;
SCurve scurve;
SoftStop soft_stop;
OnlineFullProfile online_profile;

bool stop_envelope_safe(float x,float v,float a,float max_a) {
  SoftStop trial;
  if(!trial.start(x,v,a,max_a,STOP_PROTECT_JERK_MAX,0))return false;
  return trial.predicted_min_x>=-MAX_ABS_ANGLE+SOFT_TRAVEL_GUARD_MARGIN &&
         trial.predicted_max_x<=MAX_ABS_ANGLE-SOFT_TRAVEL_GUARD_MARGIN;
}

// V5: same-direction online position replanner from ANY planned x/v/a.
// Unlike SCurve::start(), this constructor does NOT assume v0=a0=0.
// It continuously releases initial acceleration to zero, then creates
// jerk-limited acceleration/cruise/deceleration to a new REST target.
// No stop-and-restart discontinuity and no numerical integration in loop().
// First release may carry speed beyond its limit: the command is REJECTED

bool preflight_move_envelope(float x0,float x1,char mode,float v_max,float a_max,float jerk_max) {
  const uint32_t origin_time=1000000UL;
  SCurve sc;
  Trapezoid tr;
  float total=0;
  if(mode=='C') {
    if(!sc.start(x0,x1,v_max,a_max,jerk_max,origin_time))return false;
    total=sc.total;
  } else {
    tr.start(x0,x1,v_max,a_max,origin_time);
    total=tr.tt;
  }
  if(!isfinite(total)||total<=0)return false;
  float boundaries[8]={0};int count=0;
  if(mode=='C') {
    for(int i=0;i<8;i++)boundaries[count++]=sc.t_start[i];
  } else {
    boundaries[count++]=0;
    boundaries[count++]=tr.ta;
    boundaries[count++]=tr.ta+tr.tc;
    boundaries[count++]=tr.tt;
  }
  for(int pass=0;pass<2;pass++) {
    const int n=(pass==0)?64:3*count;
    for(int k=0;k<n;k++) {
      float t;
      if(pass==0)t=total*(float)k/63.0f;
      else {
        int b=k/3;
        float offset=(float)((k%3)-1)*0.001f;
        t=fmaxf(0.0f,fminf(total,boundaries[b]+offset));
      }
      if(t>=total-1e-6f)continue;
      const uint32_t now=origin_time+(uint32_t)(t*1000000.0f);
      float x=0,v=0,a=0;
      if(mode=='C') {
        sc.eval(now,x,v);
        int stage=0;
        while(stage<6 && t>=sc.t_start[stage+1])++stage;
        a=sc.sign*(sc.a_start[stage]+sc.jerks[stage]*(t-sc.t_start[stage]));
      } else {
        tr.eval(now,x,v);
        if(t<tr.ta)a=tr.sign*tr.a;
        else if(t<tr.ta+tr.tc)a=0;
        else a=-tr.sign*tr.a;
      }
      if(!stop_envelope_safe(x,v,a,a_max))return false;
    }
  }
  return true;
}

// Out-of-line methods: original V8.1 polynomial equations, unchanged.

void Trapezoid::hold(float angle) { from=to=angle; active=ever=false; }

void Trapezoid::start(float x0,float x1,float vmax,float amax,uint32_t now) {
    hold(x0); to=x1; travel=fabsf(x1-x0);
    if(travel<0.001f) { hold(x1); return; }
    sign=(x1>=x0)?1.0f:-1.0f; a=amax;
    float critical=vmax*vmax/amax;
    if(travel>critical) {
      vpk=vmax; ta=vmax/amax; tc=(travel-critical)/vmax;
    } else {
      vpk=sqrtf(travel*amax); ta=vpk/amax; tc=0.0f;
    }
    tt=2.0f*ta+tc; begin_us=now; active=ever=true;
  }

uint8_t Trapezoid::phase(uint32_t now) const {
    if(!ever) return 0;
    float t=(uint32_t)(now-begin_us)*1e-6f;
    if(t<ta) return 1;
    if(tc>0 && t<ta+tc) return 2;
    if(t<tt) return 3;
    if(t<tt+2.0f) return 4;
    return 0;
  }

void Trapezoid::eval(uint32_t now,float &theta,float &omega) {
    if(!active) { theta=to;omega=0;return; }
    float t=(uint32_t)(now-begin_us)*1e-6f;
    float s,v;
    if(t<ta) {s=0.5f*a*t*t;v=a*t;}
    else if(t<ta+tc) {float dt=t-ta; s=0.5f*a*ta*ta+vpk*dt;v=vpk;}
    else if(t<tt) {
      float dt=t-ta-tc;
      s=0.5f*a*ta*ta+vpk*tc+vpk*dt-0.5f*a*dt*dt;
      v=vpk-a*dt;
    } else {active=false;theta=to;omega=0;return;}
    theta=from+sign*s;omega=sign*v;
  }

void SCurve::hold(float angle) {
    from=to=angle; active=false; ever=false; total=0;
  }

void SCurve::accel_shape(float v,float amax,float jmax,
                          float &tj,float &ta,float &actual_a) {
    // If v < amax^2/jmax, acceleration cannot reach amax: tj=sqrt(v/J), ta=0.
    const float v_at_amax=amax*amax/jmax;
    if(v <= v_at_amax) {
      tj=sqrtf(v/jmax);
      ta=0.0f;
      actual_a=jmax*tj;
    } else {
      tj=amax/jmax;
      ta=v/amax-tj;
      actual_a=amax;
    }
  }

float SCurve::no_cruise_distance(float v,float amax,float jmax) {
    if(v<=0) return 0.0f;
    float tj,ta,actual_a;
    accel_shape(v,amax,jmax,tj,ta,actual_a);
    // Acceleration and deceleration each have mean velocity v/2.
    return v*(2.0f*tj+ta);
  }

bool SCurve::start(float x0,float x1,float maxv,float maxa,float maxj,uint32_t now) {
    if(!isfinite(x0)||!isfinite(x1)||!isfinite(maxv)||!isfinite(maxa)||
       !isfinite(maxj)||maxv<=0||maxa<=0||maxj<=0) return false;
    hold(x0);
    from=x0; to=x1; travel=fabsf(x1-x0);
    if(travel<0.001f) {hold(x1);return false;}
    sign=(x1>=x0)?1.0f:-1.0f;
    vmax=maxv;amax=maxa;jmax=maxj;

    // Find maximum reachable v; if distance is short, eliminate cruise and
    // (possibly) constant-acceleration segments automatically.
    peak_v=vmax;
    const float d_full=no_cruise_distance(vmax,amax,jmax);
    float cruise=0.0f;
    if(travel>d_full) {
      cruise=(travel-d_full)/vmax;
    } else {
      float low=0.0f,high=vmax;
      for(int i=0;i<28;i++) {
        const float mid=0.5f*(low+high);
        if(no_cruise_distance(mid,amax,jmax)>travel) high=mid;
        else low=mid;
      }
      peak_v=0.5f*(low+high);
    }
    float tj,ta;
    accel_shape(peak_v,amax,jmax,tj,ta,peak_a);
    const float dts[7]={tj,ta,tj,cruise,tj,ta,tj};
    const float js[7]={jmax,0.0f,-jmax,0.0f,-jmax,0.0f,jmax};
    float xx=0, vv=0, aa=0;
    t_start[0]=0;
    for(int i=0;i<7;i++) {
      dt[i]=dts[i];jerks[i]=js[i];
      x_start[i]=xx;v_start[i]=vv;a_start[i]=aa;
      const float h=dt[i];
      xx+=vv*h+0.5f*aa*h*h+(js[i]*h*h*h)/6.0f;
      vv+=aa*h+0.5f*js[i]*h*h;
      aa+=js[i]*h;
      t_start[i+1]=t_start[i]+h;
    }
    total=t_start[7];
    if(!isfinite(total)||total<=0||!isfinite(xx)) {hold(x0);return false;}
    begin_us=now;active=ever=true;
    return true;
  }

uint8_t SCurve::phase(uint32_t now) const {
    if(!ever)return 0;
    const float t=(uint32_t)(now-begin_us)*1e-6f;
    if(t<t_start[3])return 1;
    if(t<t_start[4])return 2;
    if(t<total)return 3;
    if(t<total+2.0f)return 4;
    return 0;
  }

void SCurve::eval(uint32_t now,float &theta,float &omega) {
    if(!active) {theta=to;omega=0;return;}
    const float t=(uint32_t)(now-begin_us)*1e-6f;
    if(t>=total) {active=false;theta=to;omega=0;return;}
    int stage=0;
    while(stage<6 && t>=t_start[stage+1]) ++stage;
    const float h=t-t_start[stage];
    const float j=jerks[stage];
    const float xx=x_start[stage]+v_start[stage]*h+
      0.5f*a_start[stage]*h*h+j*h*h*h/6.0f;
    const float vv=v_start[stage]+a_start[stage]*h+0.5f*j*h*h;
    theta=from+sign*xx;
    omega=sign*vv;
  }

void SoftStop::hold(float x) {
    from=to=x;first_v=first_a=total=final_v=final_a=0;
    nseg=0;active=ever=false;predicted_peak_v=0;predicted_min_x=predicted_max_x=x;
  }

bool SoftStop::add(float jerk,float seconds) {
    if(seconds<=1e-8f)return true;
    if(nseg>=MAX_SEGMENTS || !isfinite(seconds) || !isfinite(jerk))return false;
    const uint8_t i=nseg++;
    dt[i]=seconds;j[i]=jerk;
    start_t[i]=total;
    start_x[i]=cur_x;start_v[i]=cur_v;start_a[i]=cur_a;
    cur_x+=cur_v*seconds+0.5f*cur_a*seconds*seconds+
           jerk*seconds*seconds*seconds/6.0f;
    cur_v+=cur_a*seconds+0.5f*jerk*seconds*seconds;
    cur_a+=jerk*seconds;
    total+=seconds;
    start_t[nseg]=total;
    return isfinite(cur_x)&&isfinite(cur_v)&&isfinite(cur_a)&&isfinite(total);
  }

bool SoftStop::brake(float max_a,float max_j) {
    float u=fabsf(cur_v);
    if(u<1e-5f && fabsf(cur_a)<1e-5f)return true;
    float sign=cur_v>=0?1.0f:-1.0f;
    if(u<1e-5f)sign=cur_a>=0?1.0f:-1.0f;
    float b=sign*cur_a;
    // If acceleration already points against travel and its release alone
    // would cross zero speed, release first (continuous a), then brake the
    // resulting small reverse velocity. This deliberately permits a small
    // reversal rather than demanding an impossible monotone full stop.
    if(b<0.0f && u < b*b/(2.0f*max_j)-1e-6f) {
      if(!add(sign*max_j,-b/max_j))return false;
      u=fabsf(cur_v);
      if(u<1e-5f)return true;
      sign=cur_v>=0?1.0f:-1.0f;
      b=0.0f;
    }
    float peak=sqrtf(fmaxf(0.0f,max_j*u+0.5f*b*b));
    float magnitude=fminf(peak,max_a);
    float ramp_in=(b+magnitude)/max_j;
    if(ramp_in<-1e-5f)return false;
    if(ramp_in<0)ramp_in=0;
    float flat=0;
    if(peak>max_a) {
      flat=(u+(b*b-2.0f*max_a*max_a)/(2.0f*max_j))/max_a;
      if(flat<0)flat=0;
    }
    if(!add(-sign*max_j,ramp_in))return false;
    if(!add(0,flat))return false;
    if(!add(sign*max_j,magnitude/max_j))return false;
    return true;
  }

void SoftStop::update_extrema() {
    predicted_peak_v=fabsf(first_v);
    predicted_min_x=predicted_max_x=from;
    for(uint8_t i=0;i<nseg;i++) {
      const float h=dt[i], v0=start_v[i], a0=start_a[i], jj=j[i], x0=start_x[i];
      const float ve=v0+a0*h+0.5f*jj*h*h;
      const float xe=x0+v0*h+0.5f*a0*h*h+jj*h*h*h/6.0f;
      predicted_peak_v=fmaxf(predicted_peak_v,fmaxf(fabsf(v0),fabsf(ve)));
      predicted_min_x=fminf(predicted_min_x,fminf(x0,xe));
      predicted_max_x=fmaxf(predicted_max_x,fmaxf(x0,xe));
      if(fabsf(jj)>1e-7f) {
        // Acceleration zero means an extremum of signed velocity.
        const float z=-a0/jj;
        if(z>0 && z<h) {
          const float vz=v0+a0*z+0.5f*jj*z*z;
          predicted_peak_v=fmaxf(predicted_peak_v,fabsf(vz));
        }
        const float disc=a0*a0-2.0f*jj*v0;
        if(disc>=0) {
          const float root=sqrtf(disc);
          for(int k=-1;k<=1;k+=2) {
            const float t=(-a0+(float)k*root)/jj;
            if(t>0 && t<h) {
              const float xpos=x0+v0*t+0.5f*a0*t*t+jj*t*t*t/6.0f;
              predicted_min_x=fminf(predicted_min_x,xpos);
              predicted_max_x=fmaxf(predicted_max_x,xpos);
            }
          }
        }
      } else if(fabsf(a0)>1e-7f) {
        const float t=-v0/a0;
        if(t>0 && t<h) {
          const float xpos=x0+v0*t+0.5f*a0*t*t;
          predicted_min_x=fminf(predicted_min_x,xpos);
          predicted_max_x=fmaxf(predicted_max_x,xpos);
        }
      }
    }
  }

bool SoftStop::start(float x,float v,float a,float max_a,float max_j,uint32_t now) {
    if(!isfinite(x)||!isfinite(v)||!isfinite(a)||!isfinite(max_a)||
       !isfinite(max_j)||max_a<=0||max_j<=0||fabsf(a)>max_a+0.01f)
      return false;
    hold(x);
    from=cur_x=x;cur_v=first_v=v;cur_a=first_a=a;
    if(!brake(max_a,max_j)) {hold(x);return false;}
    if(!isfinite(cur_x)||!isfinite(cur_v)||!isfinite(cur_a)||
       fabsf(cur_v)>0.03f||fabsf(cur_a)>0.03f) {hold(x);return false;}
    to=cur_x;final_v=cur_v;final_a=cur_a;
    update_extrema();
    if(!isfinite(predicted_peak_v)||!isfinite(predicted_min_x)||
       !isfinite(predicted_max_x)) {hold(x);return false;}
    begin_us=now;ever=active=(nseg>0);
    return true;
  }

uint8_t SoftStop::phase(uint32_t now) const {
    if(!ever)return 0;
    const float t=(uint32_t)(now-begin_us)*1e-6f;
    if(t<total)return 3; // controlled deceleration (CSV i3)
    if(t<total+2.0f)return 4;
    return 0;
  }

void SoftStop::eval(uint32_t now,float &theta,float &omega) {
    if(!active) {theta=to;omega=0;return;}
    const float t=(uint32_t)(now-begin_us)*1e-6f;
    if(t>=total) {active=false;theta=to;omega=0;return;}
    uint8_t i=0;
    while(i+1<nseg && t>=start_t[i+1])++i;
    const float h=t-start_t[i];
    theta=start_x[i]+start_v[i]*h+0.5f*start_a[i]*h*h+
          j[i]*h*h*h/6.0f;
    omega=start_v[i]+start_a[i]*h+0.5f*j[i]*h*h;
  }

void OnlineReplan::hold(float x) {
    from=to=cur_x=x;v0=a0=cur_v=cur_a=0;
    total=peak_speed=braking_distance=prefix_distance=0;
    min_x=max_x=x;nseg=0;active=ever=false;
  }

bool OnlineReplan::add(float j,float seconds,uint8_t kind) {
    if(!isfinite(seconds)||!isfinite(j)||seconds<-1e-6f)return false;
    if(seconds<=1e-8f)return true;
    if(nseg>=MAX_SEGMENTS)return false;
    const int i=nseg++;
    dt[i]=seconds;jerks[i]=j;stage_kind[i]=kind;
    start_t[i]=total;start_x[i]=cur_x;start_v[i]=cur_v;start_a[i]=cur_a;
    const float h=seconds;
    cur_x+=cur_v*h+0.5f*cur_a*h*h+j*h*h*h/6.0f;
    cur_v+=cur_a*h+0.5f*j*h*h;
    cur_a+=j*h;
    total+=h;start_t[nseg]=total;
    return isfinite(cur_x)&&isfinite(cur_v)&&isfinite(cur_a)&&isfinite(total);
  }

float OnlineReplan::shape_time(float dv,float a,float j) {
    if(dv<=1e-8f)return 0;
    float tj,ta,aa;SCurve::accel_shape(dv,a,j,tj,ta,aa);
    return 2.0f*tj+ta;
  }

float OnlineReplan::distance_no_cruise(float u,float w,float a,float j) {
    return 0.5f*(u+w)*shape_time(w-u,a,j)+
           0.5f*w*shape_time(w,a,j);
  }

bool OnlineReplan::transition(float dir,float delta_v,float a,float j,uint8_t kind) {
    if(delta_v<=1e-6f)return true;
    float tj,ta,aa;SCurve::accel_shape(delta_v,a,j,tj,ta,aa);
    return add(dir*j,tj,kind)&&add(0.0f,ta,kind)&&add(-dir*j,tj,kind);
  }

void OnlineReplan::extrema() {
    min_x=max_x=from;peak_speed=fabsf(v0);
    for(int i=0;i<nseg;i++) {
      const float h=dt[i],j=jerks[i],v=start_v[i],a=start_a[i],x=start_x[i];
      const float xe=x+v*h+0.5f*a*h*h+j*h*h*h/6.0f;
      const float ve=v+a*h+0.5f*j*h*h;
      min_x=fminf(min_x,fminf(x,xe));max_x=fmaxf(max_x,fmaxf(x,xe));
      peak_speed=fmaxf(peak_speed,fmaxf(fabsf(v),fabsf(ve)));
      if(fabsf(j)>1e-8f) {
        const float zero_a=-a/j;
        if(zero_a>0 && zero_a<h) {
          const float vv=v+a*zero_a+0.5f*j*zero_a*zero_a;
          peak_speed=fmaxf(peak_speed,fabsf(vv));
        }
      }
    }
  }

bool OnlineReplan::start(float x,float v,float a,float target,float maxv,float maxa,float maxj,uint32_t now) {
    if(!isfinite(x)||!isfinite(v)||!isfinite(a)||!isfinite(target)||
       !isfinite(maxv)||!isfinite(maxa)||!isfinite(maxj)||
       maxv<=0||maxa<=0||maxj<=0||fabsf(a)>maxa+0.04f)return false;
    hold(x);from=x;to=target;v0=v;a0=a;vmax=maxv;amax=maxa;jmax=maxj;
    const float delta=target-x;
    if(fabsf(delta)<0.02f)return false;
    const float dir=(delta>0)?1.0f:-1.0f;
    if(dir*v<-0.001f)return false; // no moving-away / reverse replan in V5
    cur_x=x;cur_v=v;cur_a=a;
    // Step 1: bring nonzero acceleration to zero without any acceleration step.
    if(fabsf(cur_a)>1e-6f) {
      const float j_release=-copysignf(maxj,cur_a);
      if(!add(j_release,fabsf(cur_a)/maxj,1))return false;
    }
    if(fabsf(cur_a)>0.003f)return false;
    prefix_distance=dir*(cur_x-x);
    const float u=dir*cur_v;
    if(u<-1e-4f || u>maxv+0.003f)return false;
    if(prefix_distance<-0.0001f || dir*(target-cur_x)<-0.0001f)return false;
    // Minimum distance required to brake with zero acceleration start.
    braking_distance=0.5f*fmaxf(0.0f,u)*shape_time(fmaxf(0.0f,u),maxa,maxj);
    const float remaining=dir*(target-cur_x);
    if(remaining<braking_distance+0.0005f)return false;
    float peak=maxv;
    const float distance_at_vmax=distance_no_cruise(u,maxv,maxa,maxj);
    float cruise=0;
    if(remaining>distance_at_vmax) cruise=(remaining-distance_at_vmax)/maxv;
    else {
      float low=fmaxf(0.0f,u), high=maxv;
      for(int k=0;k<31;k++) {
        const float mid=0.5f*(low+high);
        if(distance_no_cruise(u,mid,maxa,maxj)>remaining)high=mid;
        else low=mid;
      }
      peak=0.5f*(low+high);
    }
    // Step 2: accelerate smoothly from u to peak; Step 3: cruise; Step 4: brake.
    if(!transition(dir,peak-u,maxa,maxj,1))return false;
    if(!add(0,cruise,2))return false;
    if(!transition(-dir,peak,maxa,maxj,3))return false;
    if(!isfinite(cur_x)||fabsf(cur_x-target)>0.002f||
       fabsf(cur_v)>0.002f||fabsf(cur_a)>0.002f||!isfinite(total)||total<=0)return false;
    // Correct only accumulated numerical round-off in the final hold target;
    // do not alter the polynomial during motion.
    extrema();
    if(peak_speed>maxv+0.004f||min_x<-MAX_ABS_ANGLE||max_x>MAX_ABS_ANGLE)return false;
    begin_us=now;ever=active=true;
    return true;
  }

void OnlineReplan::eval_at(float t,float &x,float &v,float &a,uint8_t* phase_out) const {
    if(nseg==0||t>=total){x=to;v=0;a=0;if(phase_out)*phase_out=4;return;}
    if(t<=0){x=from;v=v0;a=a0;if(phase_out)*phase_out=1;return;}
    uint8_t i=0;
    while(i+1<nseg&&t>=start_t[i+1])++i;
    const float h=t-start_t[i], j=jerks[i];
    x=start_x[i]+start_v[i]*h+0.5f*start_a[i]*h*h+j*h*h*h/6.0f;
    v=start_v[i]+start_a[i]*h+0.5f*j*h*h;
    a=start_a[i]+j*h;
    if(phase_out)*phase_out=stage_kind[i];
  }

void OnlineReplan::eval(uint32_t now,float &x,float &v) {
    if(!active){x=to;v=0;return;}
    const float t=(uint32_t)(now-begin_us)*1e-6f;
    if(t>=total){active=false;x=to;v=0;return;}
    float a=0;eval_at(t,x,v,a);
  }

float OnlineReplan::accel(uint32_t now) const {
    if(!active)return 0;
    const float t=(uint32_t)(now-begin_us)*1e-6f;
    float x,v,a;eval_at(t,x,v,a);return a;
  }

uint8_t OnlineReplan::phase(uint32_t now) const {
    if(!ever)return 0;
    const float t=(uint32_t)(now-begin_us)*1e-6f;
    if(t>=total+2.0f)return 0;
    if(t>=total)return 4;
    float x,v,a;uint8_t phase_id=0;
    eval_at(t,x,v,a,&phase_id);return phase_id;
  }

bool OnlineReplan::preflight() const {
  if(!isfinite(total)||total<=0||total>60.0f)return false;
  // This uses the same conservative J600 stop envelope as V4's B/C preflight.
  // In a sample guard, we only allocate SoftStop on the stack; no heap.
  for(int k=0;k<=64;k++) {
    const float t=total*(float)k/64.0f;
    float x,v,a;eval_at(t,x,v,a);
    if(!isfinite(x)||!isfinite(v)||!isfinite(a)||
       fabsf(v)>vmax+0.005f||fabsf(a)>amax+0.05f||
       fabsf(x)>MAX_ABS_ANGLE-SOFT_TRAVEL_GUARD_MARGIN)return false;
    if(!stop_envelope_safe(x,v,a,amax))return false;
  }
  for(int i=0;i<=nseg;i++){
    const float t=start_t[i];
    float x,v,a;eval_at(t,x,v,a);
    if(!isfinite(x)||!isfinite(v)||!isfinite(a)||
       fabsf(x)>MAX_ABS_ANGLE-SOFT_TRAVEL_GUARD_MARGIN||
       fabsf(v)>vmax+0.005f)return false;
  }
  return true;
  }

void OnlineFullProfile::hold(float x) {
    direct.hold(x);braking.hold(x);rest.hold(x);
    from=to=requested_to=x;v0=a0=total=0;
    peak_speed=brake_distance=brake_time=predicted_illegal_peak=0;
    min_x=max_x=x;mode=NONE;failure=OK;
    rest_needed=active=ever=false;
  }

void OnlineFullProfile::braking_at(float t,float &x,float &v,float &a) const {
    if(t<=0){x=braking.from;v=braking.first_v;a=braking.first_a;return;}
    if(t>=braking.total){x=braking.to;v=braking.final_v;a=braking.final_a;return;}
    uint8_t i=0;
    while(i+1<braking.nseg && t>=braking.start_t[i+1])++i;
    const float h=t-braking.start_t[i],j=braking.j[i];
    x=braking.start_x[i]+braking.start_v[i]*h+0.5f*braking.start_a[i]*h*h+j*h*h*h/6.0f;
    v=braking.start_v[i]+braking.start_a[i]*h+0.5f*j*h*h;
    a=braking.start_a[i]+j*h;
  }

void OnlineFullProfile::rest_at(float t,float &x,float &v,float &a,uint8_t *phase_id) const {
    if(!rest_needed || t>=rest.total){x=rest.to;v=a=0;
      if(phase_id)*phase_id=4;
      return;}
    if(t<=0){x=rest.from;v=a=0;
      if(phase_id)*phase_id=1;
      return;}
    uint8_t i=0;
    while(i<6 && t>=rest.t_start[i+1])++i;
    const float h=t-rest.t_start[i], j=rest.jerks[i];
    x=rest.from+rest.sign*(rest.x_start[i]+rest.v_start[i]*h+
      0.5f*rest.a_start[i]*h*h+j*h*h*h/6.0f);
    v=rest.sign*(rest.v_start[i]+rest.a_start[i]*h+0.5f*j*h*h);
    a=rest.sign*(rest.a_start[i]+j*h);
    if(phase_id)*phase_id=(i<3)?1:((i==3)?2:3);
  }

bool OnlineFullProfile::start(float x,float v,float a,float target,float vmax,float amax,float jmax,uint32_t now) {
    hold(x);from=x;requested_to=target;to=target;v0=v;a0=a;
    maxv=vmax;maxa=amax;maxj=jmax;begin_us=now;
    if(!isfinite(x)||!isfinite(v)||!isfinite(a)||!isfinite(target)||
       !isfinite(vmax)||!isfinite(amax)||!isfinite(jmax)||
       vmax<=0||amax<=0||jmax<=0||fabsf(a)>amax+0.05f||
       fabsf(v)>vmax+0.01f){failure=BAD_INPUT;return false;}
    if(fabsf(target)>MAX_ABS_ANGLE-SOFT_TRAVEL_GUARD_MARGIN){failure=SOFT_LIMIT;return false;}

    // Preferred solution: accelerate/hold/brake toward a reachable target
    // WITHOUT intentionally stopping in between (efficient for distant goals).
    if(direct.start(x,v,a,target,vmax,amax,jmax,now) && direct.preflight()) {
      mode=DIRECT;total=direct.total;peak_speed=direct.peak_speed;
      min_x=direct.min_x;max_x=direct.max_x;
      brake_distance=direct.braking_distance;to=target;
      active=ever=true;return true;
    }

    // Fallback for close, behind and reversing targets: stop continuously,
    // then re-accelerate from v=a=0 to the requested absolute target.
    if(!braking.start(x,v,a,amax,jmax,now)) {
      failure=SOLVER_FAIL;return false;
    }
    predicted_illegal_peak=braking.predicted_peak_v;
    if(braking.predicted_peak_v>vmax+0.008f) {
      // May be physically impossible to keep current v/a, respect J and
      // avoid a velocity excursion. Do not claim bounded motion or clamp v.
      failure=SPEED_CONFLICT;return false;
    }
    if(braking.predicted_min_x<-MAX_ABS_ANGLE+SOFT_TRAVEL_GUARD_MARGIN ||
       braking.predicted_max_x> MAX_ABS_ANGLE-SOFT_TRAVEL_GUARD_MARGIN) {
      failure=SOFT_LIMIT;return false;
    }
    if(fabsf(braking.final_v)>0.001f || fabsf(braking.final_a)>0.001f) {
      failure=SOLVER_FAIL;return false;
    }
    brake_time=braking.total;
    brake_distance=braking.to-x;
    rest_needed=fabsf(target-braking.to)>=0.001f;
    if(rest_needed && !rest.start(braking.to,target,vmax,amax,jmax,0)) {
      failure=SOLVER_FAIL;return false;
    }
    if(!rest_needed)rest.hold(braking.to);
    total=brake_time+(rest_needed?rest.total:0);
    if(!isfinite(total)||total<=0 || total>120.0f){failure=OVERLONG;return false;}
    min_x=fminf(braking.predicted_min_x,fminf(braking.to,target));
    max_x=fmaxf(braking.predicted_max_x,fmaxf(braking.to,target));
    peak_speed=fmaxf(braking.predicted_peak_v,rest_needed?rest.peak_v:0);
    if(min_x<-MAX_ABS_ANGLE+SOFT_TRAVEL_GUARD_MARGIN ||
       max_x>MAX_ABS_ANGLE-SOFT_TRAVEL_GUARD_MARGIN ||
       peak_speed>vmax+0.008f){failure=SOFT_LIMIT;return false;}
    // Check future P travel envelope throughout the new return profile.
    // Computed only at command receipt; not in the inner FOC loop.
    if(rest_needed) {
      for(int k=0;k<=64;k++) {
        const float t=rest.total*(float)k/64.0f;
        float xx,vv,aa;rest_at(t,xx,vv,aa);
        if(!isfinite(xx)||!isfinite(vv)||!isfinite(aa)||
           fabsf(vv)>vmax+0.008f||fabsf(aa)>amax+0.05f||
           !stop_envelope_safe(xx,vv,aa,amax)) {
          failure=SOFT_LIMIT;return false;
        }
      }
    }
    mode=BRAKE_RETURN;to=rest_needed?target:braking.to;
    active=ever=true;return true;
  }

void OnlineFullProfile::eval_at(float t,float &x,float &v,float &a,uint8_t *phase_id) const {
    if(mode==DIRECT){direct.eval_at(t,x,v,a,phase_id);return;}
    if(mode!=BRAKE_RETURN){x=to;v=a=0;if(phase_id)*phase_id=0;return;}
    if(t>=total){x=to;v=a=0;if(phase_id)*phase_id=4;
      return;}
    if(t<=brake_time){braking_at(t,x,v,a);if(phase_id)*phase_id=3;return;}
    rest_at(t-brake_time,x,v,a,phase_id);
  }

void OnlineFullProfile::eval(uint32_t now,float &x,float &v) {
    if(!active){x=to;v=0;return;}
    const float t=(uint32_t)(now-begin_us)*1e-6f;
    if(t>=total){active=false;x=to;v=0;return;}
    float a;eval_at(t,x,v,a);
  }

float OnlineFullProfile::accel(uint32_t now) const {
    if(!active)return 0;
    const float t=(uint32_t)(now-begin_us)*1e-6f;
    float x,v,a;eval_at(t,x,v,a);return a;
  }

uint8_t OnlineFullProfile::phase(uint32_t now) const {
    if(!ever)return 0;
    const float t=(uint32_t)(now-begin_us)*1e-6f;
    if(t>=total+2)return 0;
    if(t>=total)return 4;
    float x,v,a;uint8_t ph=0;eval_at(t,x,v,a,&ph);return ph;
  }

const char* OnlineFullProfile::mode_name() const {
    if(mode==DIRECT)return "DIRECT";
    if(mode!=BRAKE_RETURN)return "NONE";
    if(v0*(requested_to-from)<-0.001f)return "REVERSE";
    if(v0*(requested_to-braking.to)<-0.001f)return "OVERSHOOT_RETURN";
    return "BRAKE_RESTART";
  }
