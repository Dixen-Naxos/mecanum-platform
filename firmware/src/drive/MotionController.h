#pragma once
#include "drive/MotorDriver.h"
#include "power/CurrentMonitor.h"
#include "sensors/Ultrasonic.h"

// Owns the four motors, the battery monitor and the ultrasonic sensor, and turns
// a target body velocity (vx, vy, w) into safe wheel commands each control tick:
//   target clamp -> obstacle guard -> acceleration ramp -> kinematics
//   -> current-limit throttle -> motor output.
//
// The BTS7960s are gated by DRV_EN (external pull-down), so they are physically
// disabled until enable(true) is called after boot — and by emergencyStop().
class MotionController {
public:
  bool begin();                              // returns false if INA226 missing
  void setTarget(float vx, float vy, float w);
  void updateSensors();                      // call at ~15-20 Hz (US + power)
  void update(float dt);                     // call at CONTROL_HZ
  void enable(bool on);
  void emergencyStop();

  float busVoltage() const { 
    return power_.voltage(); 
  }
  float busCurrent() const { 
    return power_.current(); 
  }
  float obstacle()   const { 
    return obstacle_m_; 
  }
  bool  enabled()    const { 
    return enabled_; 
  }

private:
  static float slew(float cur, float tgt, float maxStep);

  MotorDriver    fl_, fr_, rl_, rr_;
  CurrentMonitor power_;
  Ultrasonic     us_;

  float tVx_ = 0, tVy_ = 0, tW_ = 0;   // target body velocity
  float vx_  = 0, vy_  = 0, w_  = 0;   // ramped (actual command) body velocity
  float obstacle_m_ = -1.0f;
  bool  enabled_    = false;
};
