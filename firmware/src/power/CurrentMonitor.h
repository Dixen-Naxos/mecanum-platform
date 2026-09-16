#pragma once
#include <Wire.h>
#include <INA226.h>
#include "config/pins.h"
#include "config/robot_config.h"

// High-side battery monitor (INA226 over I2C). Provides bus voltage, bus current,
// and a throttle factor used by MotionController to keep total draw under the
// battery's continuous limit (PLAN.md §4).
class CurrentMonitor {
public:
  bool begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    if (!ina_.begin()) return false;
    // Configures the internal calibration for the given shunt.
    ina_.setMaxCurrentShunt(SHUNT_MAX_A, SHUNT_OHMS);
    ok_ = true;
    return true;
  }

  void update() {
    if (!ok_) return;
    busV_ = ina_.getBusVoltage();   // volts
    curA_ = ina_.getCurrent();      // amps
  }

  float voltage() const { 
    return busV_; 
  }
  float current() const { 
    return curA_; 
  }
  bool  ok()      const { 
    return ok_; 
  }

  // 1.0 = full command allowed; <1.0 = scale commands down to hold the limit.
  float throttle() const {
    if (!ok_ || curA_ <= BUS_CURRENT_LIMIT_A) return 1.0f;
    return constrain(BUS_CURRENT_LIMIT_A / curA_, 0.2f, 1.0f);
  }

private:
  INA226 ina_{0x40};   // default I2C address
  float  busV_ = 0.0f;
  float  curA_ = 0.0f;
  bool   ok_   = false;
};
