#pragma once
#include <Arduino.h>
#include "config/robot_config.h"

// One BTS7960 module driving one JGY-370, sign-magnitude control.
//   forward (cmd > 0): PWM on RPWM, LPWM = 0
//   reverse (cmd < 0): PWM on LPWM, RPWM = 0
// Never drives both inputs high (that would shoot-through the H-bridge logic).
//
// Uses the arduino-esp32 3.x LEDC API (attach/write by *pin*, channels auto-assigned).
class MotorDriver {
public:
  void begin(int rpwmPin, int lpwmPin, bool invert = false) {
    rpwm_   = rpwmPin;
    lpwm_   = lpwmPin;
    invert_ = invert;
    ledcAttach(rpwm_, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(lpwm_, PWM_FREQ_HZ, PWM_RES_BITS);
    stop();
  }

  // cmd in [-1, 1]
  void set(float cmd) {
    if (invert_) cmd = -cmd;
    cmd = constrain(cmd, -1.0f, 1.0f);
    const uint16_t duty = (uint16_t)(fabsf(cmd) * PWM_MAX);
    if (cmd >= 0.0f) {
      ledcWrite(lpwm_, 0);
      ledcWrite(rpwm_, duty);
    } else {
      ledcWrite(rpwm_, 0);
      ledcWrite(lpwm_, duty);
    }
  }

  void stop() {
    ledcWrite(rpwm_, 0);
    ledcWrite(lpwm_, 0);
  }

private:
  int  rpwm_   = -1;
  int  lpwm_   = -1;
  bool invert_ = false;
};
