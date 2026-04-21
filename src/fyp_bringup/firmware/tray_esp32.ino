/*
 * Tray ESP32 Firmware
 * ===================
 * Drives the two tray stepper motors. Talks to the micro-ROS ESP32 over
 * ESP-NOW peer-to-peer radio (no wires between the boards).
 *
 * Link protocol (6-byte TrayPacket, same both directions):
 *   micro-ROS ESP  --> this ESP:   { msg_type=OPEN_CMD,    tray_id=1|2, seq }
 *   this ESP       --> micro-ROS:  { msg_type=CLOSED_ACK,  tray_id=1|2, seq }
 *
 * Sequence on receiving an OPEN_CMD while IDLE:
 *   1. Run the selected motor FORWARD  (FORWARD_STEPS)  -> tray opens
 *   2. Dwell TRAY_OPEN_DWELL_MS (30 s)                  -> customer interacts
 *   3. Run the same motor BACKWARD (BACKWARD_STEPS)     -> tray closes
 *   4. Send CLOSED_ACK back over ESP-NOW (3x retry)     -> micro-ROS ESP
 *                                                          publishes tray_status
 *   5. Return to IDLE
 *
 * Only one tray can be active at a time. OPEN_CMDs arriving while busy are
 * ignored (logged over serial).
 *
 * Before flashing:
 *   - Flash the MAC-print sketch on the micro-ROS ESP, paste its MAC into
 *     MICRO_ROS_ESP_MAC below.
 *   - Make sure ESPNOW_CHANNEL matches the other firmware (both default 1).
 */

#include <Arduino.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =========================
// Motor 1 pins
// =========================
#define M1_STEP_PIN 33
#define M1_DIR_PIN  32
#define M1_M0_PIN   27
#define M1_M1_PIN   26
#define M1_M2_PIN   25

// =========================
// Motor 2 pins
// =========================
#define M2_STEP_PIN 19
#define M2_DIR_PIN  18
#define M2_M0_PIN   23
#define M2_M1_PIN   22
#define M2_M2_PIN   21

// =========================
// ESP-NOW link
// =========================
// TODO: paste the micro-ROS ESP's MAC address here.
static uint8_t MICRO_ROS_ESP_MAC[6] = { 0xF4, 0x2D, 0xC9, 0x70, 0x9F, 0x00 };

#define ESPNOW_CHANNEL       1   // must match the micro-ROS ESP's channel
#define ESPNOW_MAX_RETRIES   3
#define ESPNOW_RETRY_GAP_MS  50

#define MSG_TRAY_OPEN_CMD    1
#define MSG_TRAY_CLOSED_ACK  2

typedef struct __attribute__((packed)) {
  uint8_t  msg_type;
  uint8_t  tray_id;
  uint32_t seq;
} TrayPacket;

// =========================
// Motion profile
// =========================
const long FORWARD_STEPS  = 10000;
const long BACKWARD_STEPS = 9000;
const float STEPPER_MAX_SPEED    = 2000.0f;
const float STEPPER_ACCELERATION = 800.0f;

// =========================
// Timing
// =========================
const unsigned long TRAY_OPEN_DWELL_MS = 30000;  // hold tray open for 30 s

// =========================
// Stepper objects
// =========================
AccelStepper stepper1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);

// =========================
// State machine
// =========================
enum TrayState {
  IDLE,
  OPENING,
  OPEN_DWELL,
  CLOSING
};

TrayState state = IDLE;
int active_tray = 0;              // 1 or 2 while busy, 0 when idle
unsigned long state_entered_ms = 0;

// Pending command handed over from the ESP-NOW receive callback to the loop.
volatile int pending_tray_cmd = 0;  // 0 = none, else 1 or 2
volatile uint32_t last_rx_cmd_seq = 0;

// TX bookkeeping.
volatile bool espnow_last_send_ok = false;
volatile bool espnow_send_done    = false;
uint32_t tx_seq = 0;

// =========================
// Helpers
// =========================
void set_microstep_half(int m0, int m1, int m2) {
  digitalWrite(m0, HIGH);
  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW);
}

AccelStepper& motor_for(int tray) {
  return (tray == 1) ? stepper1 : stepper2;
}

void enter_state(TrayState s) {
  state = s;
  state_entered_ms = millis();
}

// =========================
// ESP-NOW
// =========================
void espnow_on_send(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
  (void)tx_info;
  espnow_last_send_ok = (status == ESP_NOW_SEND_SUCCESS);
  espnow_send_done    = true;
}

void espnow_on_recv(const esp_now_recv_info* info, const uint8_t* data, int len) {
  (void)info;
  if (len != (int)sizeof(TrayPacket)) return;
  TrayPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.msg_type != MSG_TRAY_OPEN_CMD) return;
  if (pkt.tray_id != 1 && pkt.tray_id != 2) return;

  // Dedup: ignore a packet we've already processed.
  if (pkt.seq != 0 && pkt.seq == last_rx_cmd_seq) return;
  last_rx_cmd_seq = pkt.seq;

  // Latch; the main loop picks this up only when state == IDLE so commands
  // arriving mid-cycle are dropped on the floor (intentional).
  pending_tray_cmd = pkt.tray_id;
}

bool espnow_send_with_retry(const TrayPacket& pkt) {
  for (int attempt = 0; attempt < ESPNOW_MAX_RETRIES; attempt++) {
    espnow_send_done    = false;
    espnow_last_send_ok = false;
    esp_err_t rc = esp_now_send(MICRO_ROS_ESP_MAC, (const uint8_t*)&pkt, sizeof(pkt));
    if (rc == ESP_OK) {
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

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(espnow_on_send);
  esp_now_register_recv_cb(espnow_on_recv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, MICRO_ROS_ESP_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  if (!esp_now_is_peer_exist(MICRO_ROS_ESP_MAC)) {
    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.println("ESP-NOW add_peer failed");
      return;
    }
  }
  Serial.println("ESP-NOW initialized, peer added.");
}

// =========================
// Setup / loop
// =========================
void setup() {
  Serial.begin(115200);
  delay(100);

  // Microstepping pins
  pinMode(M1_M0_PIN, OUTPUT);
  pinMode(M1_M1_PIN, OUTPUT);
  pinMode(M1_M2_PIN, OUTPUT);
  pinMode(M2_M0_PIN, OUTPUT);
  pinMode(M2_M1_PIN, OUTPUT);
  pinMode(M2_M2_PIN, OUTPUT);
  set_microstep_half(M1_M0_PIN, M1_M1_PIN, M1_M2_PIN);
  set_microstep_half(M2_M0_PIN, M2_M1_PIN, M2_M2_PIN);

  // Motor tuning
  stepper1.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper1.setAcceleration(STEPPER_ACCELERATION);
  stepper2.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper2.setAcceleration(STEPPER_ACCELERATION);

  setup_espnow();

  enter_state(IDLE);
  Serial.println("Tray ESP32 ready. Waiting for ESP-NOW tray command...");
}

void loop() {
  switch (state) {

    case IDLE: {
      int cmd = pending_tray_cmd;
      if (cmd == 1 || cmd == 2) {
        pending_tray_cmd = 0;
        active_tray = cmd;
        Serial.print("OPEN_CMD received: opening tray "); Serial.println(active_tray);
        AccelStepper& m = motor_for(active_tray);
        m.setCurrentPosition(0);
        m.move(FORWARD_STEPS);
        enter_state(OPENING);
      }
      break;
    }

    case OPENING: {
      AccelStepper& m = motor_for(active_tray);
      m.run();
      if (m.distanceToGo() == 0) {
        Serial.print("Tray "); Serial.print(active_tray);
        Serial.println(" open. Dwelling...");
        enter_state(OPEN_DWELL);
      }
      break;
    }

    case OPEN_DWELL: {
      if (millis() - state_entered_ms >= TRAY_OPEN_DWELL_MS) {
        AccelStepper& m = motor_for(active_tray);
        m.move(-BACKWARD_STEPS);
        Serial.print("Dwell complete. Closing tray "); Serial.println(active_tray);
        enter_state(CLOSING);
      }
      break;
    }

    case CLOSING: {
      AccelStepper& m = motor_for(active_tray);
      m.run();
      if (m.distanceToGo() == 0) {
        Serial.print("Tray "); Serial.print(active_tray);
        Serial.println(" closed. Sending CLOSED_ACK...");

        TrayPacket pkt;
        pkt.msg_type = MSG_TRAY_CLOSED_ACK;
        pkt.tray_id  = (uint8_t)active_tray;
        pkt.seq      = ++tx_seq;
        bool ok = espnow_send_with_retry(pkt);
        Serial.println(ok ? "CLOSED_ACK delivered." : "CLOSED_ACK send failed after retries.");

        active_tray = 0;
        enter_state(IDLE);
      }
      break;
    }
  }
}
