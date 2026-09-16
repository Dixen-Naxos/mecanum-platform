#pragma once
#include <math.h>
#include <stdint.h>

// ------------------------------------------------------------------
// Tunable robot parameters. Values marked (MEASURE) must be set from
// the real chassis; values marked (TUNE) are safe starting points.
// ------------------------------------------------------------------

// ---- PWM (LEDC) ----
static constexpr uint32_t PWM_FREQ_HZ  = 20000;              // ~20 kHz, above audible
static constexpr uint8_t  PWM_RES_BITS = 10;                 // duty 0..1023
static constexpr uint16_t PWM_MAX      = (1u << PWM_RES_BITS) - 1;

// ---- Chassis geometry, METERS (MEASURE — see PLAN.md §6) ----
static constexpr float WHEEL_RADIUS_M = 0.040f;             // 80 mm wheel
static constexpr float WHEELBASE_L_M  = 0.20f;              // front<->rear axle
static constexpr float TRACK_W_M      = 0.18f;              // left<->right wheel
static constexpr float LXY            = (WHEELBASE_L_M + TRACK_W_M) * 0.5f; // (lx+ly)

// ---- Motor capability ----
// 150 rpm output shaft -> max wheel angular velocity (rad/s)
static constexpr float MOTOR_MAX_RADPS = 150.0f * 2.0f * (float)M_PI / 60.0f; // ~15.7

// ---- Motion limits (TUNE) ----
static constexpr float MAX_LINEAR_MPS       = 0.60f;       // clamp on vx, vy
static constexpr float MAX_ANGULAR_RPS      = 3.0f;        // clamp on w
static constexpr float ACCEL_LIMIT_MPS2     = 0.8f;        // linear ramp rate
static constexpr float ANG_ACCEL_LIMIT_RPS2 = 6.0f;        // angular ramp rate

// ---- Per-wheel direction: flip after the empirical sign check (§6) ----
static constexpr bool INV_FL = false;
static constexpr bool INV_FR = false;
static constexpr bool INV_RL = false;
static constexpr bool INV_RR = false;

// ---- Power / INA226 on the battery bus (§4) ----
static constexpr float SHUNT_OHMS          = 0.002f;       // MATCH your INA226 board
static constexpr float SHUNT_MAX_A         = 20.0f;
static constexpr float BUS_CURRENT_LIMIT_A = 6.5f;         // throttle above this
static constexpr float BUS_VOLTAGE_MIN_V   = 19.8f;        // 3.3 V/cell -> stop

// ---- Ultrasonic obstacle guard ----
static constexpr float OBSTACLE_STOP_M = 0.20f;            // block forward motion within this

// ---- Control loop ----
static constexpr uint32_t CONTROL_HZ = 100;
