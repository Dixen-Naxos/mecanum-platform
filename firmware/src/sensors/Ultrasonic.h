#pragma once
#include <Arduino.h>
#include "config/pins.h"
#include "config/robot_config.h"

// HC-SR04 driver. readMeters() is blocking via pulseIn (up to the timeout below).
//
// TODO(v2): make this fully non-blocking (rmt/interrupt on echo) so the 100 Hz
// control loop never hitches. At ~2 m timeout the worst-case stall is ~12 ms,
// which only happens when nothing is in range — acceptable for the skeleton if
// sampled at a modest rate (see main.cpp SENSOR_PERIOD_MS).
class Ultrasonic {
public:
  void begin() {
    pinMode(PIN_US_TRIG, OUTPUT);
    pinMode(PIN_US_ECHO, INPUT);
    digitalWrite(PIN_US_TRIG, LOW);
  }

  // Returns distance in meters, or -1 on timeout / no echo.
  float readMeters() {
    digitalWrite(PIN_US_TRIG, LOW);  
    delayMicroseconds(3);
    digitalWrite(PIN_US_TRIG, HIGH); 
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);

    const unsigned long us = pulseIn(PIN_US_ECHO, HIGH, TIMEOUT_US);
    if (us == 0) return -1.0f;
    return us * 0.0001715f;   // (343 m/s / 1e6) / 2  -> meters per microsecond, round trip
  }

private:
  static constexpr unsigned long TIMEOUT_US = 12000;  // ~2 m round trip
};
