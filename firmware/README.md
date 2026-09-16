# Mecanum Platform — Firmware (v1, open-loop)

PlatformIO project for the ESP32-S3-DevKitC-1. Implements the drive stack from
[`../PLAN.md`](../PLAN.md): BTS7960 sign-magnitude HAL → mecanum kinematics →
motion controller (acceleration ramp + total-current limit + obstacle guard +
e-stop). No encoders — open-loop by design for v1.

## Layout

```
src/
  config/pins.h            ESP32-S3 pin map (PLAN.md §5.1)
  config/robot_config.h    geometry, limits, PWM & power constants (tune here)
  drive/MotorDriver.h      one BTS7960: sign-magnitude PWM via LEDC
  drive/Mecanum.h          inverse kinematics (vx,vy,w -> 4 wheel commands)
  drive/MotionController.*  glue: ramp, current limit, obstacle guard, e-stop
  power/CurrentMonitor.h   INA226 bus voltage/current + throttle factor
  sensors/Ultrasonic.h     HC-SR04 distance
  main.cpp                 loop scheduler + serial command interface
```

## Build / flash

```bash
pio run                 # build
pio run -t upload       # flash (use the DevKitC-1 port labeled "USB", not "UART")
pio device monitor      # 115200 baud
```

## Drive it (serial, newline-terminated)

| Cmd | Effect |
|-----|--------|
| `G` | enable drivers |
| `V 0.3 0 0` | forward 0.3 m/s |
| `V 0 0.3 0` | strafe left 0.3 m/s |
| `V 0 0 1` | spin CCW 1 rad/s |
| `S` | soft stop (stays enabled) |
| `E` | emergency stop (disables drivers) |

## Before first power-on (safety)

1. Wheels **off the ground** on a stand.
2. Confirm the **external 10 kΩ pull-down on DRV_EN (GPIO 2)** — this keeps all four
   BTS7960s disabled through boot. Drivers only energize after you send `G`.
3. Confirm the **HC-SR04 echo divider** (5 V → 3.3 V) on GPIO 41.

## First-run calibration (do in order)

1. **Set geometry** in `robot_config.h`: `WHEEL_RADIUS_M`, `WHEELBASE_L_M`,
   `TRACK_W_M` (measure the printed chassis).
2. **Set the shunt**: `SHUNT_OHMS` / `SHUNT_MAX_A` to match your INA226 board, or
   current readings/limiting will be wrong.
3. **Verify wheel signs (PLAN.md §6):** with wheels off the ground, command pure
   `V 0.2 0 0`, then `V 0 0.2 0`, then `V 0 0 0.5`. Each wheel should turn the way
   the kinematics expects. Flip any wrong wheel with `INV_FL/FR/RL/RR` in
   `robot_config.h`.
4. Tune `ACCEL_LIMIT_MPS2` and `BUS_CURRENT_LIMIT_A` against real current draw.

## Known limitations (v1)

- **Open-loop:** commanded velocity ≈ PWM duty; no wheel feedback. Fine given the
  slow, self-locking worm gears. Add encoders + PID in v2 (reserved GPIO, §5.1).
- **Blocking ultrasonic:** `pulseIn` can stall up to ~12 ms when nothing is in
  range. Sampled at ~16 Hz to bound jitter; make it interrupt/RMT-based in v2.
