/*
 * Hardware: DengFOC-V4 / ESP32 / SimpleFOC 2.2.1 / AS5600.
 * P is a software controlled stop, NOT a hardware E-stop.
 */
#include "Runtime.h"

void setup() {
  motor_setup();
}

void loop() {
  timing_loop_begin(micros());
  motor_update();
  poll_rx();
  auto_update();
  telemetry_update_csv();
  tx_service();
  timing_loop_done(micros());
  if(diag_reset_pending) {
    diag_reset_pending=false;
    clear_v8_diagnostics();
  }
}
