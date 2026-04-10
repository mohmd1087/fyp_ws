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
 */

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <Wire.h>

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
  #include <WiFi.h>
  // *** CHANGE THESE TO YOUR WIFI NETWORK ***
  char WIFI_SSID[] = "Munim's Pixel";       // <-- your WiFi network name
  char WIFI_PASS[] = "gotohelll";    // <-- your WiFi password
  char AGENT_IP[]  = "10.50.232.81";   // <-- your laptop/PC IP on Faculty WiFi
  const uint16_t AGENT_PORT = 8888;
#endif

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

// ===================== TRAY GPIO PINS =====================
// These pins signal the second (motor-controller) ESP32 to open a tray.
// Tray 1 → GPIO 4, Tray 2 → GPIO 5. HIGH = open tray.
#define TRAY1_PIN 4
#define TRAY2_PIN 5
#define TRAY_SIGNAL_DURATION_MS 5000  // keep HIGH for 5 seconds

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
#define MIN_PWM        120     // overcome stiction (tune)
#define MAX_CMD_SPEED_MPS 0.5f // scaling for cmd_vel -> PWM (tune)

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
rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t tray_cmd_subscriber;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Int32 tray_cmd_msg;

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

const double METERS_PER_TICK = (2.0 * M_PI * WHEEL_RADIUS) / (double)ENCODER_TICKS_PER_REV;

// ===================== Tray signal state =====================
unsigned long tray_signal_start = 0;
bool tray_signal_active = false;

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

void stop_all_motors() {
  motorStop(M1_RPWM, M1_LPWM);
  motorStop(M2_RPWM, M2_LPWM);
  motorStop(M3_RPWM, M3_LPWM);
  motorStop(M4_RPWM, M4_LPWM);
}

int speed_to_pwm(float norm) {
  int pwm = (int)(fabs(norm) * (float)MAX_PWM);
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

// ===================== Callbacks =====================
void tray_cmd_callback(const void* msgin) {
  const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
  // Reset both pins first
  digitalWrite(TRAY1_PIN, LOW);
  digitalWrite(TRAY2_PIN, LOW);

  if (msg->data == 1) {
    digitalWrite(TRAY1_PIN, HIGH);
    tray_signal_active = true;
    tray_signal_start = millis();
  } else if (msg->data == 2) {
    digitalWrite(TRAY2_PIN, HIGH);
    tray_signal_active = true;
    tray_signal_start = millis();
  } else {
    tray_signal_active = false;
  }
}

void cmd_vel_callback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
  target_linear_x = msg->linear.x;
  target_angular_z = msg->angular.z;
  last_cmd_vel_time = millis();
  apply_cmd_vel(target_linear_x, target_angular_z);
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
  setup_encoders();
  setup_imu();
  stop_all_motors();

  // Tray signal pins (output to second ESP32)
  pinMode(TRAY1_PIN, OUTPUT);
  pinMode(TRAY2_PIN, OUTPUT);
  digitalWrite(TRAY1_PIN, LOW);
  digitalWrite(TRAY2_PIN, LOW);

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

        if (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS) {
          stop_all_motors();
          target_linear_x = 0.0f;
          target_angular_z = 0.0f;
        }

        // Auto-off tray signal after duration
        if (tray_signal_active && (millis() - tray_signal_start > TRAY_SIGNAL_DURATION_MS)) {
          digitalWrite(TRAY1_PIN, LOW);
          digitalWrite(TRAY2_PIN, LOW);
          tray_signal_active = false;
        }

        if (millis() - last_time_sync > TIME_SYNC_INTERVAL_MS) {
          sync_time();
        }
      }
      break;

    case AGENT_DISCONNECTED:
      stop_all_motors();
      target_linear_x = 0.0f;
      target_angular_z = 0.0f;
      time_synced = false;
      digitalWrite(TRAY1_PIN, LOW);
      digitalWrite(TRAY2_PIN, LOW);
      tray_signal_active = false;

      destroy_entities();
      agent_state = WAITING_AGENT;
      break;
  }
}
