#pragma once
// ESP32-S3-WROOM-1 pin map — see PLAN.md §5.1
// Avoids flash/PSRAM (GPIO 26-37), strapping (0/45/46/3), USB (19/20),
// UART0 (43/44), and the onboard RGB LED (38/48).

// ---- Motor PWM (RPWM, LPWM) per wheel; each uses one LEDC channel ----
static constexpr int PIN_FL_RPWM = 4;
static constexpr int PIN_FL_LPWM = 5;
static constexpr int PIN_FR_RPWM = 6;
static constexpr int PIN_FR_LPWM = 7;
static constexpr int PIN_RL_RPWM = 15;
static constexpr int PIN_RL_LPWM = 16;
static constexpr int PIN_RR_RPWM = 17;
static constexpr int PIN_RR_LPWM = 18;

// ---- Global BTS7960 enable (all R_EN/L_EN tied together) ----
// MUST have an external 10 kOhm pull-down so drivers stay OFF during boot.
static constexpr int PIN_DRV_EN = 2;

// ---- HC-SR04 ultrasonic ----
static constexpr int PIN_US_TRIG = 42;   // 3.3 V trigger, accepted directly
static constexpr int PIN_US_ECHO = 41;   // via 5 V -> 3.3 V divider (mandatory)

// ---- I2C (INA226 now; IMU can share the bus in v2) ----
static constexpr int PIN_I2C_SDA = 8;
static constexpr int PIN_I2C_SCL = 9;
