# Mecanum Platform — Build Plan

A 4-wheel mecanum-drive mobile platform, piloted by an ESP32, with a dual-voltage
architecture (24 V motor bus + low-voltage logic) and a custom carrier PCB tying
everything together.

## 1. Bill of Materials (provided)

| Qty | Part | Notes |
|-----|------|-------|
| 1 | HC-SR04 | Ultrasonic range sensor, 5 V, echo pin is 5 V |
| 4 | JGY-370 24 V worm-gear motor | Plain (no encoder), **150 RPM** output. Self-locking, high torque → **v1 open-loop** |
| 4 | Mecanum wheels 60–80 mm | 6 mm D-shaft to match JGY-370. Must be a **correct left/right roller set** (2 of each chirality) |
| 4 | BTS7960 43 A H-bridge module | Massively over-rated for these motors, but fine as modules (heatsink included) |
| 1 | 6S Li-ion/LiPo 22.2 V, 3 Ah, 5–8 A continuous | **BMS-protected. This pack is the main design constraint (see §4).** |

Chassis: 3D-printed PETG.

## 2. Parts to add (not yet in BOM)

- **DC-DC buck 24 V → 5 V**, ≥3 A (logic, ESP32, HC-SR04). e.g. mini360 is too weak — use an LM2596HV/XL4015-class or a 5 V/5 A module. Must tolerate 25.2 V input.
- **3.3 V rail**: rely on the ESP32 module's onboard regulator if using a dev board; add a dedicated 3.3 V LDO/buck if using a bare ESP32-WROOM module.
- **Battery current/voltage monitor**: INA226 or INA219 (I²C, high-side shunt) — see §4/§6 for why this replaces per-motor analog current sensing.
- **Main power switch + fuse** (7.5–10 A) + **XT60** battery connector.
- **Bulk capacitor** on the 24 V motor bus (e.g. 470–1000 µF, 35 V) near the drivers.
- **Reverse-polarity protection** (P-FET) on battery input.
- Logic-level **voltage divider** for HC-SR04 echo (5 V → 3.3 V).
- Connectors: JST/screw terminals for motors, encoders, sensor; pin headers for the BTS7960 modules if used as daughterboards.
- Emergency-stop / kill switch on the 24 V bus (optional but recommended).

## 3. System Architecture

```
        6S Battery (22.2 V nom, BMS)
              │  XT60 + fuse + switch + reverse-protect
              ├──────────────► 24 V motor bus ──► 4× BTS7960 ──► 4× JGY-370
              │                                         ▲
              │                                         │ 8× PWM + EN
              └──► Buck 24→5 V ──► 5 V logic rail       │
                        │                               │
                        ├──► ESP32 (Vin) ──► 3.3 V ─────┘ (GPIO control)
                        ├──► HC-SR04 (5 V, echo via divider)
                        └──► INA226 (I²C battery monitor)
```

Two ground domains (power GND, logic GND) joined at a single star point near the
buck input.

## 4. Power Budget — the critical constraint

The **battery (5–8 A continuous) is the bottleneck**, not the drivers (43 A each).

- 6S range: **25.2 V** full → **~19.8 V** (3.3 V/cell) practical cutoff.
- 4 × JGY-370 no-load ≈ 0.3–0.5 A each; stall can hit 2–3 A each.
- Total system must stay under **~7 A continuous** to avoid BMS cutout / sag.

Design implications:
- **Budget ~1.5 A/motor cruising.** Straight-line all-4-driving is fine; hard
  acceleration, mecanum scrub, and stalls are the risk.
- Firmware must implement **acceleration ramping** and a **total-current limiter**
  (read INA226, throttle PWM if bus current approaches ~7 A).
- Consider a higher-continuous-current pack later if the platform must climb or
  push loads. Note this as a known limitation for v1.
- Worm gears are **self-locking** → the platform holds position with motors off
  (no holding current needed) but will **not coast** — braking is abrupt.

## 5. ESP32 Pin Budget

**Board (locked): ESP32-S3-DevKitC-1** (ESP32-S3-WROOM-1). Chosen for headroom:
nearly all S3 GPIO support input + output + internal pull-ups, so the v1 map *and*
a full v2 (4 encoders + IMU) fit on one board. Native USB-JTAG for debugging.

**Decision: v1 is open-loop (plain motors, no encoders).** This frees the 8 encoder
pins and removes the ADC2/WiFi conflict entirely. WiFi stays available.

| Function | Pins | Notes |
|----------|------|-------|
| Motor PWM (RPWM/LPWM ×4) | 8 | LEDC has 8 channels; any output GPIO |
| BTS7960 enable (R_EN/L_EN) | 1 | Tie all together, drive from 1 GPIO |
| HC-SR04 trig / echo | 2 | Echo needs divider; single-pin mode can save a GPIO |
| INA226 (I²C) | 2 | SDA/SCL, shared bus |
| **Total** | **~13** | Comfortable on the S3; ~25 usable GPIO remain for v2 |

**Key decision — current sensing:** do NOT wire the 8 BTS7960 IS pins to the ESP32
ADC. Use **one INA226 on the battery input** for total current instead — simpler,
frees pins, and matches the whole-system current-limit goal in §4.

**v2 upgrade path:** encoders/PID and an IMU can be added later using the reserved
GPIO (PCNT quadrature) without re-spinning the core design.

### 5.1 Concrete Pin Map (ESP32-S3-WROOM-1)

Wheel labels match §6 (FL/FR/RL/RR). Each motor uses two LEDC PWM channels
(sign-magnitude: PWM one input, hold the other at 0 — **never both high**). The S3's
LEDC has **exactly 8 channels** — all consumed by the motors, which is fine.
This map avoids the flash/PSRAM and strapping pins, so it works on **every**
S3-WROOM-1 variant (N8R2, N16R8, …).

| Signal | GPIO | LEDC ch | Dir | Notes |
|--------|------|---------|-----|-------|
| M_FL_RPWM | 4 | 0 | out | Front-left forward |
| M_FL_LPWM | 5 | 1 | out | Front-left reverse |
| M_FR_RPWM | 6 | 2 | out | Front-right forward |
| M_FR_LPWM | 7 | 3 | out | Front-right reverse |
| M_RL_RPWM | 15 | 4 | out | Rear-left forward |
| M_RL_LPWM | 16 | 5 | out | Rear-left reverse |
| M_RR_RPWM | 17 | 6 | out | Rear-right forward |
| M_RR_LPWM | 18 | 7 | out | Rear-right reverse |
| DRV_EN (global) | 2 | — | out | Drives all 8 R_EN/L_EN. **External 10 kΩ pull-down** → drivers OFF at boot |
| HC-SR04 TRIG | 42 | — | out | 3.3 V trigger, accepted directly |
| HC-SR04 ECHO | 41 | — | in | **5 V → 3.3 V divider required** (e.g. 1 kΩ series + 2 kΩ to GND) |
| I²C SDA (INA226) | 8 | — | i/o | Shared bus (IMU can join in v2) |
| I²C SCL (INA226) | 9 | — | i/o | Shared bus |

**Total: 13 pins.**

**Reserved for v2** (encoders + IMU): GPIO 1, 10, 11, 12, 13, 14, 21, 40, 47 — all
support input **and** internal pull-ups, ideal for 4× quadrature encoders (8 pins) via
PCNT. The IMU shares the I²C bus (GPIO 8/9). **v2 fits entirely on this board** — the
whole reason for the S3 over a WROOM-32.

**Pins deliberately avoided (do not use):**
- **GPIO 26–37** — SPI flash + (octal) PSRAM on WROOM-1. Using them bricks boot.
- **GPIO 0, 45, 46** — strapping (boot mode, VDD_SPI voltage, boot-log). Leave free.
- **GPIO 19 / 20** — native USB D−/D+; keep free for USB-JTAG flashing & debug.
- **GPIO 43 / 44** — UART0 TX/RX (serial console / UART-bridge flashing).
- **GPIO 3** — JTAG-select strap; leave free.
- **GPIO 38 & 48** — onboard RGB LED (which one depends on board revision); avoid both.

> Note: GPIO 22–25 do not exist on the S3. Unlike the classic ESP32, the S3 has **no
> input-only pins** — GPIO 41 works fine as the echo input (divider still required
> because the HC-SR04 drives it to 5 V).

### 5.2 Wiring / firmware rules baked into this map

1. **Drivers off during boot.** Motor PWM GPIOs are high-Z/glitchy at reset, so the
   global `DRV_EN` (GPIO 2) has an **external pull-down** — the H-bridges stay disabled
   until firmware raises it after init. This same pin is the software **e-stop**.
2. **LEDC config:** 8 channels, **~20 kHz** (above audible, within BTS7960 range),
   10-bit resolution (0–1023 duty). Match the frequency you validated on the Pico 2.
3. **Echo divider is mandatory** — the HC-SR04 echo line idles/pulses at 5 V and will
   stress the 3.3 V pad without it.
4. **Common ground** between the 5 V logic rail and the BTS7960 GND (§3 star point).
5. **Power the DevKit from the buck's 5 V** onto the board's `5V` pin; flash/debug via
   the DevKitC-1's **native USB-C** port (the one labeled `USB`, not `UART`).

## 6. Motion Spec & Mecanum Kinematics (firmware reference)

**Top speed (150 RPM = 2.5 rev/s):** `v_max = 2.5 × π × D`
- 60 mm wheels → **~0.47 m/s**
- 80 mm wheels → **~0.63 m/s**

Forward is fastest; pure strafe and diagonal moves lose ~15–30 % to mecanum roller
slip. This is a precise, deliberate platform, not a fast one — well matched to the
worm-gear torque and the tight current budget. Wheel choice trades top speed (80 mm)
against torque/clearance (60 mm); **80 mm recommended** unless clearance is a problem.

Wheel layout (top view), roller chirality alternates diagonally:

```
   FL(\)      FR(/)
   RL(/)      RR(\)
```

Inverse kinematics (wheel angular velocities from body velocity `vx, vy, ω`):

```
w_FL = (vx − vy − (lx+ly)·ω) / r
w_FR = (vx + vy + (lx+ly)·ω) / r
w_RL = (vx + vy − (lx+ly)·ω) / r
w_RR = (vx − vy + (lx+ly)·ω) / r
```

where `r` = wheel radius, `lx`/`ly` = half wheelbase/track. Signs depend on the
final wheel/roller mounting — **verify empirically** by commanding pure vx, vy, ω
one at a time.

## 7. Phased Plan

### Phase 0 — Requirements & architecture (this doc)
- Confirm motor variant (encoder vs plain), wheel chirality set, gear ratio/RPM.
- Freeze voltage tree and pin map. Order the §2 add-on parts.

### Phase 1 — Electrical validation on the bench (before any PCB)
- Wire **one** BTS7960 + one JGY-370 from the buck/battery on a breadboard.
- ~~Verify BTS7960 logic thresholds against 3.3 V~~ — **done**: confirmed from 3.3 V
  GPIO (Pi Pico 2). No level shifter needed; carry this over to the ESP32.
- Confirm HC-SR04 divider, buck thermals under motor load, INA226 readings.
- Drive one motor forward/reverse via ESP32 PWM. **Gate: single-axis drive works.**

### Phase 2 — Full drivetrain prototype (protoboard)
- All 4 motors + 4 drivers off the real battery.
- Implement kinematics + ramping + current limiter; validate vx/vy/ω moves.
- Measure real current draw per maneuver against the §4 budget.
- **Gate: platform drives holonomically on a stand, current within budget.**

### Phase 3 — Mechanical / chassis (parallel with 1–2)
- CAD the PETG chassis: motor mounts (JGY-370 bracket + 6 mm D-shaft coupling),
  wheel spacing (`lx`, `ly`), battery bay, PCB mount, HC-SR04 front mount.
- Print, dry-fit motors + wheels, check ground clearance and roller contact.
- PETG notes: 240–250 °C, enclosure helps warping, design ≥4 walls for motor mounts.

### Phase 4 — Custom PCB
- **Topology decision (locked): carrier board.** ESP32 + buck + INA226 + protection
  + connectors on the PCB, BTS7960 kept as pluggable modules (heatsinks, proven).
  Integrated BTS7960B chips deferred to a possible v2.
- Schematic (KiCad): power tree, star ground, decoupling, bulk cap, divider,
  headers for the 4 driver modules, motor/encoder/sensor connectors, USB/prog header.
- Layout: **wide 24 V traces / pour** (size for ≥8 A + margin), keep motor-bus
  switching noise away from ESP32/analog, mounting holes matching the CAD.
- Fab (JLCPCB/PCBWay), assemble, bring-up against Phase 2 firmware.
- **Gate: PCB replaces the protoboard with identical behavior.**

### Phase 5 — Firmware (developed across 1–4)
- **Skeleton scaffolded** in [`firmware/`](firmware/) (PlatformIO, ESP32-S3, open-loop):
  BTS7960 HAL, mecanum kinematics, motion controller (ramp + INA226 current limit +
  obstacle guard + e-stop), serial command interface. See `firmware/README.md`.
- Structure: HAL (PWM/encoder/ADC) → kinematics → motion controller (ramp + current
  limit) → command interface (WiFi/BLE/serial from a controller app or gamepad).
- Add HC-SR04 obstacle stop. **v1 is open-loop** (PWM ∝ commanded velocity, no PID);
  a velocity loop is a v2 item once encoders are added.

### Phase 6 — Integration, tuning, test
- Full assembly, on-floor driving. Tune ramp rates, current limit, deadband.
- Characterize battery runtime, thermals, max safe payload.
- Document wiring, pin map, and a repeatable bring-up checklist.

## 8. Key Risks & Decisions

1. **Battery current headroom (High):** 5–8 A for 4 motors is tight → mandatory
   ramping + current limiting; possibly a bigger pack for loaded use.
2. **Open-loop v1 (Decided):** plain motors, no PID. Acceptable given slow,
   self-locking worm gears; wheel/roller signs verified empirically (§6). GPIO
   reserved for v2 encoders + IMU.
3. **BTS7960 logic level (Resolved):** confirmed working from **3.3 V** GPIO (tested
   with a Pi Pico 2 / RP2350, same 3.3 V level as ESP32). **No level shifter needed.**
4. **Wheel chirality (Easy to get wrong):** ensure a proper mecanum set, mount per §6.
5. **PCB carrier vs integrated drivers (Decided):** carrier board for v1.
6. **Ground clearance / roller contact (Mechanical):** validate in CAD + dry-fit.

## 9. Immediate Next Steps
- [x] Confirm JGY-370 gear ratio / rated RPM — **150 RPM** (top speed ~0.47–0.63 m/s, §6).
- [ ] Order §2 add-on parts (buck, INA226, fuse, XT60, connectors, caps).
- [ ] Start Phase 1 bench test with a single motor/driver (verify 3.3 V drives BTS7960).
- [ ] Begin chassis CAD in parallel (Phase 3).
