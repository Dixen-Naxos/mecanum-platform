#pragma once
#include <math.h>
#include "config/robot_config.h"

struct WheelCmd { float fl, fr, rl, rr; };

// Mecanum inverse kinematics.
// Body frame: x = forward, y = left, w = CCW yaw (rad/s). Roller chirality per §6.
// Returns normalized wheel commands in [-1, 1] (1.0 == MOTOR_MAX_RADPS).
// If any wheel would saturate, the whole vector is scaled down so the commanded
// heading is preserved (only the speed drops).
//
// NOTE: the +/- pattern below assumes the standard X-configuration. Verify by
// commanding pure vx, then pure vy, then pure w and watching each wheel (§6);
// flip individual wheels via INV_* in robot_config.h if a direction is wrong.
class Mecanum {
public:
  static WheelCmd inverse(float vx, float vy, float w) {
    const float r = WHEEL_RADIUS_M;
    WheelCmd c;
    c.fl = (vx - vy - LXY * w) / r;   // rad/s
    c.fr = (vx + vy + LXY * w) / r;
    c.rl = (vx + vy - LXY * w) / r;
    c.rr = (vx - vy + LXY * w) / r;

    // rad/s -> normalized command
    const float k = 1.0f / MOTOR_MAX_RADPS;
    c.fl *= k; c.fr *= k; c.rl *= k; c.rr *= k;

    // scale down uniformly if over the limit on any wheel
    const float m = fmaxf(fmaxf(fabsf(c.fl), fabsf(c.fr)),
                          fmaxf(fabsf(c.rl), fabsf(c.rr)));
    if (m > 1.0f) {
      const float s = 1.0f / m;
      c.fl *= s; c.fr *= s; c.rl *= s; c.rr *= s;
    }
    return c;
  }
};
