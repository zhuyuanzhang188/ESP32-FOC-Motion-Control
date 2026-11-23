// SimpleFOC is an external dependency; this repository does not vendor it.
#include "Runtime.h"

bool csv_on = false;
uint32_t last_csv_ms = 0;
// Diagnostic: cached boot voltage for CSV i15; no ADC call in telemetry.
float csv_boot_vin = 0.0f;

static const uint16_t TX_CAP = 4096;
char tx_buf[TX_CAP];
uint16_t tx_w = 0, tx_r = 0;
uint32_t tx_drops = 0;
uint16_t tx_room() {
  return tx_w >= tx_r ? TX_CAP-1-(tx_w-tx_r) : tx_r-tx_w-1;
}
void tx_clear() { tx_w = tx_r = 0; }
void tx_printf(const char* fmt, ...) {
  char line[290];
  va_list ap; va_start(ap,fmt);
  int n = vsnprintf(line,sizeof(line),fmt,ap);
  va_end(ap);
  if(n<=0 || n>=(int)sizeof(line) || n>tx_room()) { ++tx_drops; return; }
  for(int i=0;i<n;i++) { tx_buf[tx_w]=line[i]; tx_w=(tx_w+1)%TX_CAP; }
}
void tx_service() {
  int room=Serial.availableForWrite();
  if(room>32) room=32;
  while(room-->0 && tx_r!=tx_w) {
    Serial.write((uint8_t)tx_buf[tx_r]);
    tx_r=(tx_r+1)%TX_CAP;
  }
}

void telemetry_update_csv() {
  uint32_t now_ms=millis();
  if(foc_ok&&csv_on&&(uint32_t)(now_ms-last_csv_ms)>=CSV_PERIOD_MS) {
    last_csv_ms=now_ms;
    uint32_t now_us=micros();
    uint32_t elapsed=run_id?(uint32_t)(now_us-run_started_us)/1000UL:0;
    tx_printf("%lu,%lu,%lu,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%lu,%lu,%d,%u,%lu\n",
      (unsigned long)now_ms,(unsigned long)run_id,(unsigned long)elapsed,
      (unsigned)profile_phase(now_us),theta_d,motor1.shaft_angle,
      omega_d,motor1.shaft_velocity,motor1.current_sp,motor1.current.q,
      motor1.voltage.d,motor1.voltage.q,
      traj_vmax,traj_amax,ff_gain,csv_boot_vin,
      (unsigned long)accepted,(unsigned long)rejected,(int)last_op,
      (unsigned)last_reject,(unsigned long)tx_drops);
  }
}
