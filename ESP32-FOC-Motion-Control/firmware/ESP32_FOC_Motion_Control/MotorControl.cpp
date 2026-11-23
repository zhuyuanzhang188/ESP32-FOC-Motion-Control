// Module extracted from the user's V8.1 single-axis firmware.
// Refactor only: keep original motion formulas, control parameters and sequence.
// SimpleFOC is an external dependency; this repository does not vendor it.
#include "Runtime.h"

BLDCMotor motor1 = BLDCMotor(7);
bool foc_ok = false;
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32, 33, 25, 12);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire bus1 = TwoWire(0);
InlineCurrentSense sense1 = InlineCurrentSense(0.01f, 50.0f, 39, 36);

float bus_voltage() {return analogReadMilliVolts(13)*8.5f/1000.0f;}

void motor_setup() {
  Serial.begin(115200);
  // The unpopulated/unused second motor must NOT receive PWM commands.
  // Disconnect motor2 physically when evaluating the single-motor controller.
  pinMode(26,INPUT_PULLDOWN);pinMode(27,INPUT_PULLDOWN);pinMode(14,INPUT_PULLDOWN);
  pinMode(32,INPUT_PULLUP);pinMode(33,INPUT_PULLUP);pinMode(25,INPUT_PULLUP);
  analogReadResolution(12);
  float vin=bus_voltage();
  while(vin<=MIN_VIN) {
    vin=bus_voltage();delay(100);
    Serial.printf("Waiting supply VIN=%.2f\n",vin);
  }
  csv_boot_vin = vin; // Once at startup only; not updated during motor movement.
  Serial.printf("Calibrating motor1 VIN=%.2f\n",vin);
  bus1.begin(19,18,400000UL);
  sensor1.init(&bus1);motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply=bus_voltage();driver1.init();motor1.linkDriver(&driver1);
  motor1.current_limit=1.0f;
  motor1.voltage_limit=bus_voltage();
  sense1.init();motor1.linkCurrentSense(&sense1);
  motor1.torque_controller=TorqueControlType::foc_current;
  motor1.controller=MotionControlType::angle;
  motor1.voltage_sensor_align=5;
  motor1.PID_current_q.P=5;motor1.PID_current_q.I=1000;
  motor1.PID_current_d.P=5;motor1.PID_current_d.I=1000;
  motor1.LPF_current_q.Tf=0.002;motor1.LPF_current_d.Tf=0.002;
  motor1.PID_velocity.P=0.021;motor1.PID_velocity.I=0.12;motor1.PID_velocity.D=0;
  motor1.P_angle.P=POSITION_KP;
  motor1.velocity_limit=INNER_VEL_LIMIT;
  motor1.useMonitoring(Serial);motor1.monitor_downsample=0;
  motor1.monitor_variables=_MON_TARGET|_MON_VEL|_MON_ANGLE|_MON_CURR_Q;
  motor1.init();
  int ok=motor1.initFOC();
  foc_ok=ok!=0;
  if(!foc_ok) {motor1.disable();Serial.println("#FAULT calibration failed");return;}
  motor1.target=0;trajectory.hold(0);
  settle_candidate_ms=millis();
  Serial.printf("#VERSION %s\n",FW_VERSION);
  Serial.println("#READY MOTOR1: V8.1 B/C/P/N; S brief I details ? help D reset O CSV21");
  _delay(1000);
}

// V8.1 original main-loop FOC work, extracted without reordering.
void motor_update() {
  if(foc_ok) {
    const uint32_t foc_start_us=micros();
    motor1.loopFOC();
    timing_foc_done((uint32_t)(micros()-foc_start_us));
    uint32_t now=micros();
    profile_eval(now,theta_d,omega_d);
    theta_cmd=theta_d+(ff_gain/POSITION_KP)*omega_d;
    motor1.target=theta_cmd;
    motor1.move();
    monitor_stop_tracking(now);
    monitor_runtime_diagnostics();
    monitor_np_tracking(millis());
    run_quality.update();
    // Require continuous position settling, not a noisy instantaneous velocity sample.
    if(profile_active()||fabsf(theta_d-motor1.shaft_angle)>HOLD_ERROR_LIMIT)
      settle_candidate_ms=millis();
  }
}
