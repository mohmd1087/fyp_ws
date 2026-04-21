/*
 * Tray oscillation test
 * =====================
 * Drives tray 1 forward, then reverse, then forward, forever.
 * No ESP-NOW, no ROS — just the motor, for mechanical debugging.
 *
 * To test tray 2 instead, change the pin #defines to the M2_* values
 * from tray_esp32.ino (STEP=19, DIR=18, M0=23, M1=22, M2=21).
 */

#include <Arduino.h>
#include <AccelStepper.h>

// Motor 1 pins (tray 1). Swap to M2 pins for tray 2.
#define STEP_PIN 33
#define DIR_PIN  32
#define M0_PIN   27
#define M1_PIN   26
#define M2_PIN   25

const long  FORWARD_STEPS  = 10000;
const long  BACKWARD_STEPS = 9000;    // matches tray_esp32.ino
const float STEPPER_MAX_SPEED    = 2000.0f;
const float STEPPER_ACCELERATION = 800.0f;
const unsigned long PAUSE_MS     = 300;   // brief rest between direction flips

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

enum Phase { FORWARD, PAUSE_AFTER_FWD, BACKWARD, PAUSE_AFTER_BWD };
Phase phase = FORWARD;
unsigned long pause_started_ms = 0;

void set_microstep_half() {
  digitalWrite(M0_PIN, HIGH);
  digitalWrite(M1_PIN, LOW);
  digitalWrite(M2_PIN, LOW);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  set_microstep_half();

  stepper.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper.setAcceleration(STEPPER_ACCELERATION);
  stepper.setCurrentPosition(0);
  stepper.move(FORWARD_STEPS);

  Serial.println("Oscillation test started. Forward first.");
}

void loop() {
  switch (phase) {
    case FORWARD:
      stepper.run();
      if (stepper.distanceToGo() == 0) {
        Serial.println("Forward done. Pausing...");
        pause_started_ms = millis();
        phase = PAUSE_AFTER_FWD;
      }
      break;

    case PAUSE_AFTER_FWD:
      if (millis() - pause_started_ms >= PAUSE_MS) {
        stepper.move(-BACKWARD_STEPS);
        Serial.println("Reversing...");
        phase = BACKWARD;
      }
      break;

    case BACKWARD:
      stepper.run();
      if (stepper.distanceToGo() == 0) {
        Serial.println("Reverse done. Pausing...");
        pause_started_ms = millis();
        phase = PAUSE_AFTER_BWD;
      }
      break;

    case PAUSE_AFTER_BWD:
      if (millis() - pause_started_ms >= PAUSE_MS) {
        stepper.move(FORWARD_STEPS);
        Serial.println("Forward...");
        phase = FORWARD;
      }
      break;
  }
}
