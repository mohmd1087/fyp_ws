/*
 * micro-ROS ESP32 Firmware for 4WD Skid-Steer Robot
 * =================================================
 * NAV2-ready firmware (front encoders only): publishes wheel odometry + IMU for EKF fusion.
 *
 * Publishes:
 *   /wheel/odometry  (nav_msgs/Odometry)   @ 50 Hz  — from FRONT wheel encoders ONLY (FL + FR)
 *   /imu/data        (sensor_msgs/Imu)     @ 100 Hz — from MPU6050 (6-axis)
 *
 * Subscribes:
 *   /cmd_vel         (geometry_msgs/Twist) — motor velocity commands
 *
 * Motor wiring (MATCHES your teleop sketch):
 *   LEFT:  M1 (Front) RPWM=23 LPWM=22,  M3 (Rear) RPWM=14 LPWM=13
 *   RIGHT: M2 (Front) RPWM=26 LPWM=25,  M4 (Rear) RPWM=18 LPWM=19
 *
 * Encoder wiring (FRONT encoders only — no external pullups needed on these pins):
 *   FL_A=32 FL_B=33
 *   FR_A=16 FR_B=17
 *
 * IMU (MPU6050) I2C wiring:
 *   SDA=21, SCL=27   (SCL moved off 22 because GPIO22 is a motor pin)
 *
 * IMPORTANT (ENCODERS POWERED AT 5V):
 * - ESP32 GPIOs are 3.3V logic. If your encoder outputs are 5V, DO NOT connect A/B directly.
 * - Either:
 *    (1) power encoder at 3.3V (if it works), OR
 *    (2) level-shift A/B to 3.3V (recommended).
 *
 * Transport:
 *   Serial (recommended first): set_microros_transports()
 *   WiFi UDP (optional): set_microros_wifi_transports(...)
 *
 * Tray link:
 *   The second (tray-controller) ESP32 is reached over ESP-NOW peer-to-peer
 *   radio — no GPIO wires. On /waiter/tray_cmd we send a TrayPacket{OPEN_CMD},
 *   and on its return TrayPacket{CLOSED_ACK} we publish /waiter/tray_status.
 *   Set TRAY_ESP_MAC below before flashing.
 */

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>

// ===================== TRANSPORT SELECT =====================
#define USE_WIFI 0  // 0 = Serial USB, 1 = WiFi UDP

#if USE_WIFI
  // *** CHANGE THESE TO YOUR WIFI NETWORK ***
  char WIFI_SSID[] = "Munim's Pixel";       // <-- your WiFi network name
  char WIFI_PASS[] = "gotohelll";    // <-- your WiFi password
  char AGENT_IP[]  = "10.50.232.81";   // <-- your laptop/PC IP on Faculty WiFi
  const uint16_t AGENT_PORT = 8888;
  // IMPORTANT: when USE_WIFI=1, ESP-NOW must use the AP's channel. Update
  // ESPNOW_CHANNEL below to match your AP and make sure the tray ESP matches.
#endif

// ===================== ESP-NOW LINK TO TRAY ESP32 =====================
// Replaces the old GPIO wires between the two boards (tray1/tray2/done). The
// two ESPs talk peer-to-peer over the WiFi radio — no router, no IP stack.
//
// TODO: paste the tray ESP's MAC address here (run the MAC-print sketch once).
// Format: 6 hex bytes, e.g. { 0xA4, 0xCF, 0x12, 0x34, 0x56, 0x78 }.
static uint8_t TRAY_ESP_MAC[6] = { 0xB0, 0xB2, 0x1C, 0xA8, 0x52, 0x50 };

#define ESPNOW_CHANNEL       1    // must match the tray ESP's channel
#define ESPNOW_MAX_RETRIES   3
#define ESPNOW_RETRY_GAP_MS  50

#define MSG_TRAY_OPEN_CMD    1
#define MSG_TRAY_CLOSED_ACK  2

typedef struct __attribute__((packed)) {
  uint8_t  msg_type;   // MSG_TRAY_OPEN_CMD or MSG_TRAY_CLOSED_ACK
  uint8_t  tray_id;    // 1 or 2
  uint32_t seq;        // monotonic per-sender counter
} TrayPacket;

// ===================== ROBOT GEOMETRY =====================
#define WHEEL_SEPARATION  0.435f   // meters (front wheel center-to-center, measured)
#define WHEEL_RADIUS      0.0675f  // meters
#define ENCODER_TICKS_PER_REV  1136 // SET THIS for your encoders (1x decoding on A rising)

// ===================== MOTOR PINS (FROM YOUR TELEOP CODE) =====================
// LEFT: M1 + M3
#define M1_RPWM 23
#define M1_LPWM 22
#define M3_RPWM 14
#define M3_LPWM 13
// RIGHT: M2 + M4
#define M2_RPWM 26
#define M2_LPWM 25
#define M4_RPWM 18
#define M4_LPWM 19

// ===================== ENCODER PINS (FRONT ONLY) =====================
// These pins support INPUT_PULLUP (no external pullups required).
#define FL_ENC_A 16
#define FL_ENC_B 17
#define FR_ENC_A 32
#define FR_ENC_B 33

// Tray commands & closed-acks now travel over ESP-NOW (see TRAY_ESP_MAC above),
// so no GPIO pins are used for the inter-ESP link anymore.

// ===================== IMU (MPU6050) =====================
#define IMU_I2C_ADDR  0x68
#define IMU_SDA       21
#define IMU_SCL       27
#define IMU_CALIBRATION_SAMPLES 500

// ===================== TIMING =====================
#define ODOM_PUBLISH_MS    100   // 10 Hz (serial bandwidth limited)
#define IMU_PUBLISH_MS     50    // 20 Hz (prioritize IMU for EKF yaw)
#define CMD_VEL_TIMEOUT_MS 250   // stop fast if no cmd_vel
#define TIME_SYNC_INTERVAL_MS 30000

// ===================== PWM (MATCH TELEOP STYLE) =====================
#define PWM_FREQ       1000
#define PWM_RESOLUTION 8
#define MAX_PWM        255
#define MIN_PWM        90      // stiction breakaway (lower = smoother low-speed motion, less PWM step at velocity onset)
#define MAX_CMD_SPEED_MPS 0.5f // scaling for cmd_vel -> PWM (tune)

// ===================== BATTERY VOLTAGE SENSING =====================
// GPIO36 reads scaled battery via 1/5 voltage divider on the PCB.
// Same formula as the standalone test sketch:
//   v_bat = (raw/4095) * V_REF * V_DIV * V_CAL
#define VOLT_PIN      36
#define V_REF         3.3f
#define V_DIV         5.0f
#define V_CAL         1.3044f
// V_NOMINAL = "normal" battery (11 V) — PWM is scaled by V_NOMINAL / V_measured
// so the robot behaves the same at 11 V and at full charge (~13 V).
#define V_NOMINAL     11.0f
#define V_COMP_MIN    0.6f      // don't scale below 60% (low-batt safeguard)
#define V_COMP_MAX    1.0f      // never boost above 100% (full PWM at/below nominal)
#define VBAT_EMA_A    0.05f     // low-pass filter alpha on ADC reading

// ===================== VELOCITY RAMPING (trapezoidal, simple & stable) =====================
// Fixed accel/decel rates — velocity ramps linearly toward target. No overshoot.
#define RAMP_TICK_MS        20     // 50 Hz ramp update
#define MAX_LINEAR_ACCEL    0.15f  // m/s^2 — reaches 0.15 m/s in ~1s
#define MAX_LINEAR_DECEL    0.25f  // m/s^2 — stops from 0.15 m/s in ~0.6s
#define MAX_ANGULAR_ACCEL   0.35f  // rad/s^2 — reaches 0.3 rad/s in ~0.85s
#define MAX_ANGULAR_DECEL   0.50f  // rad/s^2 — stops rotation in ~0.6s
// Jerk limits retained as macros (unused by trapezoidal ramp) in case we re-enable S-curve later
#define MAX_LINEAR_JERK     0.20f
#define MAX_ANGULAR_JERK    0.60f

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// ===================== HELPERS =====================
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static unsigned long _t = 0; \
  if (_t == 0) _t = millis(); \
  if (millis() - _t > (MS)) { X; _t = millis(); } \
} while(0)

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// ===================== micro-ROS state machine =====================
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agent_state = WAITING_AGENT;

// ===================== micro-ROS objects =====================
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t tray_status_publisher;
rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t tray_cmd_subscriber;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Int32 tray_cmd_msg;
std_msgs__msg__Int32 tray_status_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t odom_timer;
rcl_timer_t imu_timer;

// Frame IDs (writable buffers)
static char odom_frame_id[]       = "odom";
static char odom_child_frame_id[] = "base_footprint";  // recommended for Nav2/EKF
static char imu_frame_id[]        = "imu_link";

// ===================== encoder globals (FRONT ONLY) =====================
volatile long fl_ticks = 0, fr_ticks = 0;
long prev_fl_ticks = 0, prev_fr_ticks = 0;

double odom_x = 0.0, odom_y = 0.0, odom_theta = 0.0;
unsigned long last_odom_time = 0;

unsigned long last_cmd_vel_time = 0;
float target_linear_x = 0.0f;
float target_angular_z = 0.0f;

// Ramping state (current = what's actually applied to motors)
float current_linear_x  = 0.0f;
float current_angular_z = 0.0f;
unsigned long last_ramp_time = 0;

const double METERS_PER_TICK = (2.0 * M_PI * WHEEL_RADIUS) / (double)ENCODER_TICKS_PER_REV;

// ===================== Tray link state (ESP-NOW) =====================
// Last tray commanded via /waiter/tray_cmd (1 or 2). Used to validate CLOSED_ACK.
volatile int last_commanded_tray = 0;

// Monotonic sequence numbers for the two directions.
volatile uint32_t tx_seq = 0;
volatile uint32_t last_rx_ack_seq = 0;   // for dedup on the receive side

// Inter-task flag: set from ESP-NOW receive callback, consumed by main loop
// (avoids calling rcl_publish from the WiFi task).
volatile int pending_tray_status_publish = 0;

// TX status from the most recent esp_now_send call (set by OnDataSent callback).
volatile bool espnow_last_send_ok = false;
volatile bool espnow_send_done    = false;

// ===================== IMU bias =====================
float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

// ===================== time sync =====================
bool time_synced = false;
unsigned long last_time_sync = 0;

// ===================== PWM helpers (RPWM/LPWM) =====================
void pwmSetupPin(int pin) {
  ledcAttach(pin, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(pin, 0);
}

void motorStop(int rpwm, int lpwm) {
  ledcWrite(rpwm, 0);
  ledcWrite(lpwm, 0);
}

void motorForward(int rpwm, int lpwm, int spd) {
  ledcWrite(lpwm, 0);
  ledcWrite(rpwm, spd);
}

void motorBackward(int rpwm, int lpwm, int spd) {
  ledcWrite(rpwm, 0);
  ledcWrite(lpwm, spd);
}

void setup_motor_pins() {
  pwmSetupPin(M1_RPWM); pwmSetupPin(M1_LPWM);
  pwmSetupPin(M2_RPWM); pwmSetupPin(M2_LPWM);
  pwmSetupPin(M3_RPWM); pwmSetupPin(M3_LPWM);
  pwmSetupPin(M4_RPWM); pwmSetupPin(M4_LPWM);
}

// Battery voltage (filtered) and PWM compensation scale.
float g_vbat = V_NOMINAL;
float g_pwm_scale = 1.0f;

void setup_vbat() {
  pinMode(VOLT_PIN, INPUT);
  analogSetAttenuation(ADC_11db);
  // Prime filter with an initial sample.
  int raw = analogRead(VOLT_PIN);
  g_vbat = (raw / 4095.0f) * V_REF * V_DIV * V_CAL;
}

void update_vbat() {
  int raw = analogRead(VOLT_PIN);
  float v = (raw / 4095.0f) * V_REF * V_DIV * V_CAL;
  g_vbat = (1.0f - VBAT_EMA_A) * g_vbat + VBAT_EMA_A * v;
  float s = (g_vbat > 0.1f) ? (V_NOMINAL / g_vbat) : 1.0f;
  if (s < V_COMP_MIN) s = V_COMP_MIN;
  if (s > V_COMP_MAX) s = V_COMP_MAX;
  g_pwm_scale = s;
}

void stop_all_motors() {
  motorStop(M1_RPWM, M1_LPWM);
  motorStop(M2_RPWM, M2_LPWM);
  motorStop(M3_RPWM, M3_LPWM);
  motorStop(M4_RPWM, M4_LPWM);
}

int speed_to_pwm(float norm) {
  int pwm = (int)(fabs(norm) * (float)MAX_PWM * g_pwm_scale);
  if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
  if (pwm > MAX_PWM) pwm = MAX_PWM;
  return pwm;
}

void apply_cmd_vel(float linear_x, float angular_z) {
  // skid/diff mix
  float left_vel  = linear_x - (angular_z * WHEEL_SEPARATION * 0.5f);
  float right_vel = linear_x + (angular_z * WHEEL_SEPARATION * 0.5f);

  float left_n  = constrain(left_vel  / MAX_CMD_SPEED_MPS, -1.0f, 1.0f);
  float right_n = constrain(right_vel / MAX_CMD_SPEED_MPS, -1.0f, 1.0f);

  int lpwm = speed_to_pwm(left_n);
  int rpwm = speed_to_pwm(right_n);

  // LEFT side: M1 + M3
  if (left_n > 0.01f) {
    motorForward(M2_RPWM, M2_LPWM, lpwm);
    motorForward(M4_RPWM, M4_LPWM, lpwm);
  } else if (left_n < -0.01f) {
    motorBackward(M2_RPWM, M2_LPWM, lpwm);
    motorBackward(M4_RPWM, M4_LPWM, lpwm);
  } else {
    motorStop(M2_RPWM, M2_LPWM);
    motorStop(M4_RPWM, M4_LPWM);
  }

  // RIGHT side: M2 + M4
  if (right_n > 0.01f) {
    motorForward(M1_RPWM, M1_LPWM, rpwm);
    motorForward(M3_RPWM, M3_LPWM, rpwm);
  } else if (right_n < -0.01f) {
    motorBackward(M1_RPWM, M1_LPWM, rpwm);
    motorBackward(M3_RPWM, M3_LPWM, rpwm);
  } else {
    motorStop(M1_RPWM, M1_LPWM);
    motorStop(M3_RPWM, M3_LPWM);
  }
}

// ===================== Velocity ramping (two-stage, jerk-bounded) =====================
// Stage 1: low-pass filter on the *target*. Smooths step-changes from DWB so the
//   velocity setpoint moves continuously rather than jumping each control tick.
//   Critically: this smoothed setpoint can NEVER produce overshoot because it's
//   pure exponential approach (asymptotic, can't cross target).
// Stage 2: accel-limited tracking of the smoothed setpoint. Bounded slew rate.
//
// Why this avoids the previous oscillation bug: there's no projection logic, no
// sign-flipping accel commands, no asymmetric peak switching — just two well-known
// stable filters in series. Output velocity profile is smooth and monotonic toward
// any new target.
//
// Tuning: TARGET_FILTER_TAU_S sets how soft the velocity onset/release feels.
//   Larger = smoother but more lag. 0.4s is gentle; 0.2s is more responsive.
static inline float step_axis(float target, float current_vel,
                              float &smoothed_target, float tau_s,
                              float peak_accel, float peak_decel,
                              float dt) {
  // Stage 1: first-order low-pass on target.
  // smoothed += (target - smoothed) * (dt / (tau + dt))
  float alpha = dt / (tau_s + dt);
  smoothed_target += (target - smoothed_target) * alpha;

  // Stage 2: slew-rate-limited tracking of smoothed target.
  float err = smoothed_target - current_vel;
  bool decelerating = (fabsf(smoothed_target) < fabsf(current_vel));
  float rate = decelerating ? peak_decel : peak_accel;
  float step = rate * dt;

  float new_vel;
  if (err >  step)       new_vel = current_vel + step;
  else if (err < -step)  new_vel = current_vel - step;
  else                   new_vel = smoothed_target;

  return new_vel;
}

// Filter time constants — bigger τ → smoother (and slower to respond)
#define TARGET_FILTER_TAU_LIN_S  0.35f
#define TARGET_FILTER_TAU_ANG_S  0.30f

// Smoothed setpoints (Stage 1 outputs); persist across calls.
static float smoothed_target_linear  = 0.0f;
static float smoothed_target_angular = 0.0f;

void ramp_velocity_tick() {
  unsigned long now = millis();
  float dt = (now - last_ramp_time) / 1000.0f;
  if (dt < (RAMP_TICK_MS / 1000.0f)) return;
  last_ramp_time = now;

  current_linear_x  = step_axis(target_linear_x,  current_linear_x,
                                smoothed_target_linear,  TARGET_FILTER_TAU_LIN_S,
                                MAX_LINEAR_ACCEL,  MAX_LINEAR_DECEL,  dt);
  current_angular_z = step_axis(target_angular_z, current_angular_z,
                                smoothed_target_angular, TARGET_FILTER_TAU_ANG_S,
                                MAX_ANGULAR_ACCEL, MAX_ANGULAR_DECEL, dt);

  apply_cmd_vel(current_linear_x, current_angular_z);
}

// ===================== Encoder ISRs (1x decode: A rising, B for direction) =====================
void IRAM_ATTR fl_encoder_isr() { fl_ticks += digitalRead(FL_ENC_B) ? -1 : 1; }
// Right side inverted (keep if your FR counts go opposite direction vs FL)
void IRAM_ATTR fr_encoder_isr() { fr_ticks += digitalRead(FR_ENC_B) ? 1 : -1; }

void setup_encoders() {
  pinMode(FL_ENC_A, INPUT_PULLUP);
  pinMode(FL_ENC_B, INPUT_PULLUP);

  pinMode(FR_ENC_A, INPUT_PULLUP);
  pinMode(FR_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), fl_encoder_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), fr_encoder_isr, RISING);
}

// ===================== IMU (MPU6050) =====================
void read_imu_raw(float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)IMU_I2C_ADDR, (uint8_t)14, (uint8_t)true);

  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temp
  int16_t raw_gx = (Wire.read() << 8) | Wire.read();
  int16_t raw_gy = (Wire.read() << 8) | Wire.read();
  int16_t raw_gz = (Wire.read() << 8) | Wire.read();

  // accel ±4g => 8192 LSB/g, gyro ±500 dps => 65.5 LSB/(deg/s)
  *ax = (float)raw_ax / 8192.0f * 9.81f;
  *ay = (float)raw_ay / 8192.0f * 9.81f;
  *az = (float)raw_az / 8192.0f * 9.81f;

  *gx = (float)raw_gx / 65.5f * (M_PI / 180.0f);
  *gy = (float)raw_gy / 65.5f * (M_PI / 180.0f);
  *gz = (float)raw_gz / 65.5f * (M_PI / 180.0f);
}

void read_imu(float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {
  read_imu_raw(ax, ay, az, gx, gy, gz);
  *gx -= gyro_bias_x;
  *gy -= gyro_bias_y;
  *gz -= gyro_bias_z;
}

void setup_imu() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);

  // wake
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // gyro ±500
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission(true);

  // accel ±4g
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission(true);

  // DLPF 44Hz
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);

  delay(100);

  // gyro bias calibration (robot must be still)
  float sumx=0, sumy=0, sumz=0;
  float ax,ay,az,gx,gy,gz;
  for (int i=0; i<IMU_CALIBRATION_SAMPLES; i++) {
    read_imu_raw(&ax,&ay,&az,&gx,&gy,&gz);
    sumx += gx; sumy += gy; sumz += gz;
    delay(2);
  }
  gyro_bias_x = sumx / IMU_CALIBRATION_SAMPLES;
  gyro_bias_y = sumy / IMU_CALIBRATION_SAMPLES;
  gyro_bias_z = sumz / IMU_CALIBRATION_SAMPLES;
}

// ===================== Time sync helpers =====================
void get_timestamp(builtin_interfaces__msg__Time* stamp) {
  if (time_synced) {
    int64_t nanos = rmw_uros_epoch_nanos();
    if (nanos != 0) {
      stamp->sec = (int32_t)(nanos / 1000000000LL);
      stamp->nanosec = (uint32_t)(nanos % 1000000000LL);
      return;
    }
    time_synced = false;
  }
  unsigned long ms = millis();
  stamp->sec = (int32_t)(ms / 1000UL);
  stamp->nanosec = (uint32_t)((ms % 1000UL) * 1000000UL);
}

void sync_time() {
  rmw_uros_sync_session(1000);
  time_synced = (rmw_uros_epoch_millis() != 0);
  last_time_sync = millis();
}

// ===================== ESP-NOW link =====================
void espnow_on_send(const esp_now_send_info_t* tx_info, esp_now_send_status_t status) {
  (void)tx_info;
  espnow_last_send_ok = (status == ESP_NOW_SEND_SUCCESS);
  espnow_send_done    = true;
}

void espnow_on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  (void)info;
  if (len != (int)sizeof(TrayPacket)) return;
  TrayPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.msg_type != MSG_TRAY_CLOSED_ACK) return;
  if (pkt.tray_id != 1 && pkt.tray_id != 2) return;

  // Dedup: ignore repeats with the same-or-older seq.
  if (pkt.seq != 0 && pkt.seq == last_rx_ack_seq) return;
  last_rx_ack_seq = pkt.seq;

  if ((int)pkt.tray_id != last_commanded_tray) return;

  // Stash the tray_id in the message and ask the main loop to publish. Doing
  // rcl_publish from the WiFi task is unsafe.
  tray_status_msg.data = pkt.tray_id;
  pending_tray_status_publish = 1;
}

bool espnow_send_with_retry(const TrayPacket& pkt) {
  for (int attempt = 0; attempt < ESPNOW_MAX_RETRIES; attempt++) {
    espnow_send_done    = false;
    espnow_last_send_ok = false;
    esp_err_t rc = esp_now_send(TRAY_ESP_MAC, (const uint8_t*)&pkt, sizeof(pkt));
    if (rc == ESP_OK) {
      // Wait briefly for the TX-status callback.
      unsigned long t0 = millis();
      while (!espnow_send_done && (millis() - t0) < 50) { delay(1); }
      if (espnow_last_send_ok) return true;
    }
    delay(ESPNOW_RETRY_GAP_MS);
  }
  return false;
}

void setup_espnow() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_send_cb(espnow_on_send);
  esp_now_register_recv_cb(espnow_on_recv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, TRAY_ESP_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  if (!esp_now_is_peer_exist(TRAY_ESP_MAC)) {
    esp_now_add_peer(&peer);
  }
}

// ===================== Callbacks =====================
void tray_cmd_callback(const void* msgin) {
  const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
  if (msg->data != 1 && msg->data != 2) return;

  last_commanded_tray = (int)msg->data;

  TrayPacket pkt;
  pkt.msg_type = MSG_TRAY_OPEN_CMD;
  pkt.tray_id  = (uint8_t)msg->data;
  pkt.seq      = ++tx_seq;
  (void)espnow_send_with_retry(pkt);  // log/retry handled inside; orchestrator
                                      // timeout catches total delivery failure.
}

void cmd_vel_callback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
  target_linear_x = msg->linear.x;
  target_angular_z = msg->angular.z;
  last_cmd_vel_time = millis();

  // Normal zero commands (e.g. teleop key release, Nav2 goal end) are handled
  // by the S-curve ramp in ramp_velocity_tick() — it decelerates smoothly and
  // stops cleanly. The previous "emergency-stop bypass" that zero'd current_vel
  // and slammed the motor driver was causing mechanical kickback: chassis
  // momentum + abrupt H-bridge brake produced a visible reverse lurch, and
  // teleop's repeating forward command relaunched it → forward-reverse loop.
  //
  // True emergency stop on cmd_vel TIMEOUT is still enforced in the main loop
  // (CMD_VEL_TIMEOUT_MS), which smoothly ramps target to zero.
  // Motor output handled by ramp_velocity_tick() for smooth acceleration
}

// ===================== Timers =====================
void odom_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;
  if (!timer) return;

  unsigned long now = millis();
  double dt = (now - last_odom_time) / 1000.0;
  if (dt <= 0.0 || dt > 1.0) {
    last_odom_time = now;
    return;
  }
  last_odom_time = now;

  noInterrupts();
  long fl = fl_ticks, fr = fr_ticks;
  interrupts();

  long d_fl = fl - prev_fl_ticks;
  long d_fr = fr - prev_fr_ticks;

  prev_fl_ticks = fl;
  prev_fr_ticks = fr;

  // FRONT encoders only (no averaging with rear)
  double d_left  = (double)d_fl * METERS_PER_TICK;
  double d_right = (double)d_fr * METERS_PER_TICK;

  double d_center = (d_left + d_right) / 2.0;
  double d_theta  = (d_right - d_left) / WHEEL_SEPARATION;

  // Skid-steer slip compensation:
  // During a pure turn (wheels spinning opposite directions), d_center should be
  // exactly 0 but encoder noise makes it drift. If the turn rate is large relative
  // to forward motion, suppress the translational component to prevent ghost movement.
  if (fabs(d_theta) > 0.02 && fabs(d_center) < fabs(d_theta) * WHEEL_SEPARATION * 0.3) {
    d_center = 0.0;
  }

  double theta_mid = odom_theta + d_theta * 0.5;
  odom_x += d_center * cos(theta_mid);
  odom_y += d_center * sin(theta_mid);
  odom_theta += d_theta;

  while (odom_theta > M_PI)  odom_theta -= 2.0 * M_PI;
  while (odom_theta < -M_PI) odom_theta += 2.0 * M_PI;

  double vx = d_center / dt;
  double vtheta = d_theta / dt;

  get_timestamp(&odom_msg.header.stamp);

  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(odom_theta * 0.5);
  odom_msg.pose.pose.orientation.w = cos(odom_theta * 0.5);

  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = vtheta;

  rcl_publish(&odom_publisher, &odom_msg, NULL);
}

void imu_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;
  if (!timer) return;

  float ax,ay,az,gx,gy,gz;
  read_imu(&ax,&ay,&az,&gx,&gy,&gz);

  get_timestamp(&imu_msg.header.stamp);

  imu_msg.angular_velocity.x = gx;
  imu_msg.angular_velocity.y = gy;
  imu_msg.angular_velocity.z = gz;

  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;

  rcl_publish(&imu_publisher, &imu_msg, NULL);
}

// ===================== Message init =====================
void init_messages() {
  memset(odom_msg.pose.covariance, 0, sizeof(odom_msg.pose.covariance));
  memset(odom_msg.twist.covariance, 0, sizeof(odom_msg.twist.covariance));
  memset(imu_msg.orientation_covariance, 0, sizeof(imu_msg.orientation_covariance));
  memset(imu_msg.angular_velocity_covariance, 0, sizeof(imu_msg.angular_velocity_covariance));
  memset(imu_msg.linear_acceleration_covariance, 0, sizeof(imu_msg.linear_acceleration_covariance));

  odom_msg.header.frame_id.data = odom_frame_id;
  odom_msg.header.frame_id.size = strlen(odom_frame_id);
  odom_msg.header.frame_id.capacity = sizeof(odom_frame_id);

  odom_msg.child_frame_id.data = odom_child_frame_id;
  odom_msg.child_frame_id.size = strlen(odom_child_frame_id);
  odom_msg.child_frame_id.capacity = sizeof(odom_child_frame_id);

  imu_msg.header.frame_id.data = imu_frame_id;
  imu_msg.header.frame_id.size = strlen(imu_frame_id);
  imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);

  // Covariances (tune later)
  odom_msg.pose.covariance[0]  = 0.02;  // x
  odom_msg.pose.covariance[7]  = 0.02;  // y
  odom_msg.pose.covariance[35] = 0.08;  // yaw

  odom_msg.twist.covariance[0]  = 0.02; // vx
  odom_msg.twist.covariance[35] = 0.08; // vyaw

  // IMU: no orientation estimate => orientation_covariance[0] = -1
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;
  imu_msg.orientation_covariance[0] = -1.0;

  imu_msg.angular_velocity_covariance[0] = 0.002;
  imu_msg.angular_velocity_covariance[4] = 0.002;
  imu_msg.angular_velocity_covariance[8] = 0.002;

  imu_msg.linear_acceleration_covariance[0] = 0.02;
  imu_msg.linear_acceleration_covariance[4] = 0.02;
  imu_msg.linear_acceleration_covariance[8] = 0.02;
}

// ===================== Create/destroy entities =====================
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  sync_time();

  RCCHECK(rclc_node_init_default(&node, "esp32_driver", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &odom_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/wheel/odometry"));

  RCCHECK(rclc_publisher_init_default(
    &imu_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data"));

  RCCHECK(rclc_publisher_init_default(
    &tray_status_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/waiter/tray_status"));

  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
    &tray_cmd_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/waiter/tray_cmd"));

  RCCHECK(rclc_timer_init_default(
    &odom_timer, &support, RCL_MS_TO_NS(ODOM_PUBLISH_MS),
    odom_timer_callback));

  RCCHECK(rclc_timer_init_default(
    &imu_timer, &support, RCL_MS_TO_NS(IMU_PUBLISH_MS),
    imu_timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg,
                                        &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &tray_cmd_subscriber, &tray_cmd_msg,
                                        &tray_cmd_callback, ON_NEW_DATA));

  init_messages();

  last_odom_time = millis();
  last_cmd_vel_time = millis();
  return true;
}

void destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&odom_publisher, &node);
  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&tray_status_publisher, &node);
  rcl_subscription_fini(&cmd_vel_subscriber, &node);
  rcl_subscription_fini(&tray_cmd_subscriber, &node);
  rcl_timer_fini(&odom_timer);
  rcl_timer_fini(&imu_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ===================== SETUP =====================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  setup_motor_pins();
  setup_vbat();
  setup_encoders();
  setup_imu();
  stop_all_motors();

  // ESP-NOW link to tray ESP32 (replaces the old tray GPIO wires).
  // IMPORTANT: must be set up BEFORE micro-ROS WiFi transport (if USE_WIFI=1),
  // because micro-ROS WiFi init also configures the WiFi radio.
  setup_espnow();

#if USE_WIFI
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
  WiFi.setSleep(false);
#else
  set_microros_transports();
#endif

  agent_state = WAITING_AGENT;
}

// ===================== LOOP =====================
void loop() {
  switch (agent_state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
          ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      EXECUTE_EVERY_N_MS(250, digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)););
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        agent_state = AGENT_CONNECTED;
        digitalWrite(LED_BUILTIN, LOW);
      } else {
        destroy_entities();
        agent_state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
        agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
          ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );

      if (agent_state == AGENT_CONNECTED) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

        // Soft stop on cmd_vel timeout — ramp decelerates smoothly
        if (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS) {
          target_linear_x = 0.0f;
          target_angular_z = 0.0f;
        }

        // Velocity ramping: smoothly move current velocity toward target
        ramp_velocity_tick();

        // Battery voltage tracking @ 10 Hz — drives PWM compensation in speed_to_pwm()
        EXECUTE_EVERY_N_MS(100, update_vbat(););

        // Tray-closed ack from the tray ESP arrived via ESP-NOW. The WiFi-task
        // callback stashed the tray_id and set this flag; publish here where
        // it's safe to call into rclc.
        if (pending_tray_status_publish) {
          pending_tray_status_publish = 0;
          rcl_publish(&tray_status_publisher, &tray_status_msg, NULL);
          last_commanded_tray = 0;
        }

        if (millis() - last_time_sync > TIME_SYNC_INTERVAL_MS) {
          sync_time();
        }
      }
      break;

    case AGENT_DISCONNECTED:
      stop_all_motors();  // emergency hard-stop (safety)
      target_linear_x = 0.0f;
      target_angular_z = 0.0f;
      current_linear_x = 0.0f;
      current_angular_z = 0.0f;
      smoothed_target_linear  = 0.0f;
      smoothed_target_angular = 0.0f;
      time_synced = false;
      last_commanded_tray = 0;
      pending_tray_status_publish = 0;

      destroy_entities();
      agent_state = WAITING_AGENT;
      break;
  }
}
