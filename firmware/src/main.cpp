#include <Arduino.h>
#include "drive/MotionController.h"
#include "config/robot_config.h"

// -----------------------------------------------------------------------------
// Mecanum platform — v1 firmware skeleton (open-loop). See PLAN.md §5.
//
// Serial command interface (115200 baud, native USB-CDC), newline-terminated:
//   G              enable drivers
//   S              soft stop (target 0,0,0 — stays enabled)
//   E              emergency stop (disable drivers)
//   V vx vy w      set target body velocity: forward m/s, left m/s, CCW rad/s
//                  e.g.  "V 0.3 0 0" forward, "V 0 0.3 0" strafe, "V 0 0 1" spin
// -----------------------------------------------------------------------------

MotionController robot;

static const uint32_t CONTROL_PERIOD_MS = 1000 / CONTROL_HZ;   // 10 ms @ 100 Hz
static const uint32_t SENSOR_PERIOD_MS  = 60;                  // ~16 Hz
static const uint32_t TELEM_PERIOD_MS   = 500;

static uint32_t tCtrl = 0, tSens = 0, tTelem = 0;
static uint32_t lastCtrlUs = 0;
static String   line;

static void handleCommand(const String& s) {
  if (s.length() == 0) return;
  switch (s[0]) {
    case 'G': 
      robot.enable(true);
      Serial.println("[ok] enabled");
      break;
    case 'E': 
      robot.emergencyStop();
      Serial.println("[ok] ESTOP");
      break;
    case 'S': 
      robot.setTarget(0, 0, 0);
      Serial.println("[ok] stop");
      break;
    case 'V': {
      float vx = 0, vy = 0, w = 0;
      sscanf(s.c_str() + 1, "%f %f %f", &vx, &vy, &w);
      robot.setTarget(vx, vy, w);
      Serial.printf("[ok] target vx=%.2f vy=%.2f w=%.2f\n", vx, vy, w);
      break;
    }
    default: 
      Serial.println("[?] G|S|E|'V vx vy w'");
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nMecanum platform booting...");
  if (!robot.begin()) {
    Serial.println("[warn] INA226 not found -> current limiting DISABLED");
  }
  Serial.println("Commands: G=enable  S=stop  E=estop  'V vx vy w'=target");
  lastCtrlUs = micros();
}

void loop() {
  const uint32_t now = millis();

  // Non-blocking serial command intake.
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') { 
      handleCommand(line); line = ""; 
    }
    else if (line.length() < 48)  { 
      line += ch; 
    }
  }

  if (now - tSens >= SENSOR_PERIOD_MS) {
    tSens = now;
    robot.updateSensors();
  }

  if (now - tCtrl >= CONTROL_PERIOD_MS) {
    tCtrl = now;
    const uint32_t us = micros();
    const float dt = (us - lastCtrlUs) * 1e-6f;
    lastCtrlUs = us;
    robot.update(dt);
  }

  if (now - tTelem >= TELEM_PERIOD_MS) {
    tTelem = now;
    Serial.printf("V=%.1fV I=%.2fA obst=%.2fm en=%d\n",
                  robot.busVoltage(), robot.busCurrent(),
                  robot.obstacle(), robot.enabled());
  }
}
