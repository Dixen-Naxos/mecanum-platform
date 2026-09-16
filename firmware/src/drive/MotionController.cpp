#include "drive/MotionController.h"
#include "drive/Mecanum.h"
#include "config/pins.h"
#include "config/robot_config.h"
#include <Arduino.h>

bool MotionController::begin() {
  // Keep drivers disabled until enable() is called.
  pinMode(PIN_DRV_EN, OUTPUT);
  digitalWrite(PIN_DRV_EN, LOW);

  fl_.begin(PIN_FL_RPWM, PIN_FL_LPWM, INV_FL);
  fr_.begin(PIN_FR_RPWM, PIN_FR_LPWM, INV_FR);
  rl_.begin(PIN_RL_RPWM, PIN_RL_LPWM, INV_RL);
  rr_.begin(PIN_RR_RPWM, PIN_RR_LPWM, INV_RR);

  us_.begin();
  return power_.begin();   // false if INA226 not found (current limiting disabled)
}

void MotionController::setTarget(float vx, float vy, float w) {
  tVx_ = constrain(vx, -MAX_LINEAR_MPS,  MAX_LINEAR_MPS);
  tVy_ = constrain(vy, -MAX_LINEAR_MPS,  MAX_LINEAR_MPS);
  tW_  = constrain(w,  -MAX_ANGULAR_RPS, MAX_ANGULAR_RPS);
}

void MotionController::enable(bool on) {
  enabled_ = on;
  digitalWrite(PIN_DRV_EN, on ? HIGH : LOW);
  if (!on) { 
    fl_.stop(); 
    fr_.stop(); 
    rl_.stop(); 
    rr_.stop(); 
  }
}

void MotionController::emergencyStop() {
  tVx_ = tVy_ = tW_ = 0;
  vx_  = vy_  = w_  = 0;
  fl_.stop(); 
  fr_.stop(); 
  rl_.stop(); 
  rr_.stop();
  enabled_ = false;
  digitalWrite(PIN_DRV_EN, LOW);
}

void MotionController::updateSensors() {
  obstacle_m_ = us_.readMeters();
  power_.update();
}

float MotionController::slew(float cur, float tgt, float maxStep) {
  const float d = constrain(tgt - cur, -maxStep, maxStep);
  return cur + d;
}

void MotionController::update(float dt) {
  if (!enabled_) { 
    fl_.stop(); 
    fr_.stop(); 
    rl_.stop(); 
    rr_.stop(); 
    return; 
  }

  // Battery under-voltage cutoff (ignore the 0 V "no sensor" reading).
  if (power_.ok() && power_.voltage() > 1.0f && power_.voltage() < BUS_VOLTAGE_MIN_V) {
    emergencyStop();
    return;
  }

  // Obstacle guard: block forward motion when something is too close ahead.
  float txv = tVx_, tyv = tVy_, tw = tW_;
  if (obstacle_m_ > 0.0f && obstacle_m_ < OBSTACLE_STOP_M && txv > 0.0f) {
    txv = 0.0f;
  }

  // Acceleration ramp toward target (protects the battery from current spikes, §4).
  vx_ = slew(vx_, txv, ACCEL_LIMIT_MPS2     * dt);
  vy_ = slew(vy_, tyv, ACCEL_LIMIT_MPS2     * dt);
  w_  = slew(w_,  tw,  ANG_ACCEL_LIMIT_RPS2 * dt);

  // Kinematics -> normalized wheel commands.
  WheelCmd c = Mecanum::inverse(vx_, vy_, w_);

  // Total-current limiter: scale all commands by the throttle factor.
  const float k = power_.throttle();
  c.fl *= k; 
  c.fr *= k; 
  c.rl *= k; 
  c.rr *= k;

  fl_.set(c.fl); 
  fr_.set(c.fr); 
  rl_.set(c.rl); 
  rr_.set(c.rr);
}
