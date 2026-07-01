# launchxl_3phganinv — LAUNCHXL-F280049C + BOOSTXL-3PhGaNInv Porting Checklist

**On hold / historical validation platform.** This board was used after the DRV8305EVM path was
set aside, but it is no longer the active product path. The active product target is
`esc6288_revA`; keep this directory for historical bench evidence and regression builds, but do not
start new feature work here unless it fixes shared code needed by esc6288.

GaN bring-up platform: TI BOOSTXL-3PhGaNInv (SENS007, Rev A) on the F280049C LaunchPad. The control
core (FOC/FAST/motor ID/loop tuning) is board-agnostic and reused as-is; this directory is only the
board layer.

Cloned from `boards/launchxl_drv8305evm/` (same LaunchPad Site 1; identical PWM + current-sense
pins). Key deltas applied: no SPI gate driver, active-LOW PWM enable, remapped voltage channels,
GaN scaling, GaN dead-time, and a digital over-temp trip. See board.h / hal.h / hal.c / user.h.

## Power stage
Three LMG5200 GaN half-bridges (80 V / 10 A) behind an SN74AVC8T245 PWM buffer (active-low OE).
INA240A1 (gain 20) on 5 mΩ in-line shunts, 1.65 V bias (REF3333). TMP302 digital over-temp.
On-board hardware OCP (~12 A, 1 A hyst) trips the buffer independently as a backstop.

## Pin Mapping (Site 1 / BoosterPack1 J1-J4) — locked; cross-referenced from schematics
| Function | Header | GPIO/ADC | Peripheral / notes |
|------|:--:|:--:|------|
| PWM A H/L | 40/39 | GPIO10/11 | EPWM6A/6B |
| PWM B H/L | 38/37 | GPIO8/9   | EPWM5A/5B |
| PWM C H/L | 36/35 | GPIO4/5   | EPWM3A/3B |
| IA/IB/IC | 27/28/29 | ADCIN B2/C0/A9 | CMPSS 3/1/6, direct-pin mux 0/1/3; PGA disabled |
| VA/VB/VC | 24/25/26 | ADCIN B0/C2/B1 | divider 100k/4.22k |
| VDC (bus) | 23 | ADCIN A5 | divider 100k/4.22k |
| VREF (diag, unused) | 30 | ADCIN A1 | not sampled |
| nEn_uC (PWM enable) | 13 | GPIO39 (out) | **ACTIVE-LOW**; safe-off = HIGH; ext pull-up R30 |
| OT (over-temp) | 34 | GPIO58 (in)  | **active-low**; routed to ePWM trip via HAL_PM_nFAULT_GPIO |

No SPI, no gate-driver fault pin, no WAKE. `BOARD_HAS_GATE_FAULT_INPUT = 0`.

## Scaling Constants (user.h)
- `USER_ADC_FULL_SCALE_CURRENT_A = 33.0`  (5 mΩ × INA240 gain 20 = 0.1 V/A → ±16.5 A)
- `USER_ADC_FULL_SCALE_VOLTAGE_V = 81.5`  ← ⚠️ confirm exact divider resistors on the bench
- `IA/IB/IC_OFFSET_A = +16.5`  (POSITIVE: read functions use a positive current_sf on this board)
- Current `current_sf` is POSITIVE in HAL_readADCData* (GaN INA240 inverted vs DRV8301/8305).
  ⚠️ **UNCONFIRMED** — the current loop is stable with +sf, but FAST never locked at low speed
  (see bench results); the sf sign / current-phase mapping vs PWM order is still to be commissioned.

## Bench-verify items
- [x] **Gate enable polarity**: GPIO39 idles HIGH (buffer disabled) at the is01 dead-wait; only the
      `*_3phganinv.js` helpers drive it LOW. Confirmed (check_3phganinv_is01.js).
- [x] **Dead-time** (`HAL_PWM_DBRED_CNT`/`DBFED_CNT = 20` ≈ 200 ns): scoped at 0 V bus via
      scope_deadtime_3phganinv.js — high/low non-overlap ~200 ns, as expected. No shoot-through at
      24 V either (is06/is07 ran fault-free). NOTE: LMG5200 "8 ns" in the datasheet is the t_MON/t_MOFF
      channel delay-MATCHING max, NOT an internal dead-time — the MCU dead-band is the only protection.
- [~] **Current-sense sign / phase order**: NOT resolved — see "FOC commissioning TODO" below. is02
      offsets are clean/symmetric; current loop stable; but FAST does not lock (speed sign inverted vs
      command, magnitude wrong). Needs methodical commissioning, not the simple flip noted earlier.
- [x] **Over-temp**: GPIO58 reads HIGH in normal operation (TMP302 deasserted). Trip not exercised.
- [x] **Voltage full-scale**: VdcBus reads ~24.0 V at a metered 24 V bus → 81.5 V FS is about right.
      (Confirm exact divider at higher bus before relying on it.)

## Bench results (2026-06-24, AM-4116-KV450 on 24 V)
Hardware bring-up of the new board **succeeded**. Validated over the XDS110/DSS helpers:
- **Power architecture (important):** the BoosterPack makes its own 5 V/3.3 V/VREF **from VBUS**
  (LM5017 buck → LP38691 LDO → REF3333; schematic sheets 1–2). **USB-only does NOT properly power the
  analog/gate stage** — with the bus off the board only weak-back-feeds 3.3 V through the **J5** jumper
  (VREF read 1.29 V, INA240 bias ~1208 cts). With 24 V applied: D5/D6 lit, VREF 2.99 V, VdcBus 24.0 V.
- **J5 jumper:** with the bus on, the BoosterPack's 3.3 V contends with the LaunchPad USB 3.3 V through
  J5 → this **dropped the XDS110** once. Fix: remove J5 for bus-powered + USB-debug (board self-powers
  analog; LaunchPad on USB). Re-applied bus afterward with no debugger drop.
- **is01 / is02:** GPIO39 safe-off confirmed; offset cal clean (3-phase symmetric, 2–3 LSB noise),
  VdcBus 24 V, voltage offsets valid with the bus on.
- **Dead-band:** scoped non-overlap ~200 ns at 0 V bus.
- **is06 @ 24 V:** power stage switches with NO shoot-through (fault-free), current loop closes (Iq
  tracks command), flux ≈ 0.0128 ≈ KV450 → correct motor profile. Iq ≤ 0.5 A would not break the rotor
  loose from standstill (torque mode, no rotating reference).
- **is07 speed control (run_is07_3phganinv.js, Iq capped 1.5 A):** the motor **spins** and follows the
  commanded direction (+ref → CW, −ref → CCW), but roughly, and **FAST never locks** — estimated speed
  is inverted vs the command and wrong magnitude, and is independent of physical direction (swapping
  two motor leads flipped rotation but not FAST's reading) → FAST is not tracking the rotor.

⚠️ VREF reads **2.99 V** (REF3333 nominal 3.3 V) → INA240 bias ~1.0 V not 1.65 V → asymmetric current
range (+22.9 / −10.1 A). Fine for small-Iq bring-up; investigate before high-current work.

## FOC commissioning TODO (next session — not a one-line fix)
The board is proven; what remains is sensorless commissioning. Do it methodically, NOT by blind
sweeping (PWM_PHASE_ORDER=2 drew ~10 A at 24 V in a wrong frame). Suggested order:
1. Drop the bus to ~12 V (LM5017 min) to bound wrong-frame current; keep the speed-loop Iq capped.
2. With a **controlled open-loop rotating vector** (angle commanded by us, estimator out of the loop),
   confirm PWM phase order × current-sense phase mapping are self-consistent and which direction is +.
3. Then settle the **current_sf sign** (only change from the proven DRV8305 config) against a known
   current; fix offset sign to match.
4. Verify motor params (the 4116 Rs/Ls/flux came from a legacy esc_drv8300 ID) — re-ID if needed for
   FAST to lock; FAST is weak at the ~20 Hz we tried, so also test at higher speed.
5. Restore `USER_MOTOR_MAX_CURRENT_A` (reverted to 8.0) / speed-loop gains and re-tune.
Helpers added this session: `tools/flash/3phganinv/{check_3phganinv_is01,cal_is02_3phganinv,scope_deadtime_3phganinv,run_is06_3phganinv,run_is07_3phganinv}.js`.

## Historical bench bring-up procedure (AM-4116 on this board)

**Do NOT run is01→is13 in sequence.** This board did run on hardware (see the 2026-06-24 bench
results above), but FAST never locked cleanly and the platform is paused. The LMG5200 has no
internal dead-time, and the AM-4116 (~40 mΩ) is unsafe on two stock labs (below). If this board is
ever resumed, treat the list below as the historical safe-order baseline, not as the current product
bring-up path.

Two hardware facts confirmed against the code (drive the order):
- **Stock SDK labs do NOT enable GPIO39 on this board.** `HAL_enableDRV()` is only called under
  `#ifdef DRV8320_SPI` (e.g. `is01_intro_hal.c:423`, `is06_torque_control.c:416`); this board defines
  no SPI macro (`build.sh:127`). So "it compiles" ≠ "PWM reaches the LMG5200". The gate buffer must be
  enabled explicitly — the `*_3phganinv.js` helpers below drive **GPIO39 LOW (active-low nEn_uC)**.
- **AM-4116 unsafe on is03 V/f and is05 ID here.** is03 scalar V/f puts `VOLT_MIN_V` across ~40 mΩ ≈
  instant over-current (`am_4116_kv450.h:46`); is05 FAST-ID startup transient exceeds this board's
  ±16.5 A sense ceiling (tighter than the DRV8305 EVM). See `product/BENCH.md`, `motors/README.md`.

GaN gate polarity (opposite of DRV8305): enable = `GPBCLEAR 0x7F0C` bit7 (GPIO39 LOW, readback 0);
disable = `GPBSET 0x7F0A` bit7 (GPIO39 HIGH, readback 1).

Order (USB powers the LaunchPad; keep the 24 V bus OFF until step 4):
1. `tools/flash/3phganinv/check_3phganinv_is01.js` — safety check only; confirms parked at dead-wait, GPIO39
   idles HIGH (buffer disabled), GPIO58/OT deasserted, zero-current counts ~2048. **Never enables.**
2. `tools/flash/3phganinv/cal_is02_3phganinv.js` — one-shot zero-current offset cal with the buffer **still
   disabled** (external INA240 samples without it); reports offsets (~+16.5 A) and noise.
3. `tools/flash/3phganinv/scope_deadtime_3phganinv.js` — **bus must be 0 V** (hard-aborts ≥2 V, and kills PWM
   if the bus rises during the hold). Loads `is02_offset_gain_cal.out` and uses its re-armed
   offset-cal 50% zero-vector (`Vabc_pu` hard-set 0 → 50% duty with NO `1/dcBus` term, unlike is03's
   V/f which goes Inf/NaN at 0 V), enables the buffer, and holds so you scope the high/low-side
   non-overlap (~200 ns / 20 counts). Raise `HAL_PWM_DBRED_CNT/DBFED_CNT` and rebuild if any overlap,
   BEFORE any bus voltage.
4. Apply current-limited **24 V**, then `tools/flash/3phganinv/run_is06_3phganinv.js [iq_A]` — default Iq 0.1 A,
   hard max 0.5 A. Confirm current/torque direction and rotation; fix `current_sf` sign (hal.h, both
   read fns) + offset sign (user.h), or `PWM_PHASE_ORDER`, if reversed.
5. Only after the above: small-speed is07; then is08/is10. is09/is12/is13 are later features.

Skipped on this board for the 4116: energized is03 V/f, is05 re-ID. Full-power 4116 → esc6288.

## Deferred (only if this board is resumed)
- CAN bridge (`can_bridge.c`, CANA GPIO32/33 — free on Site 1) → enables `CAN_CHECK`/`PRODUCT`.
- `product/product_main.c` board cases (limits, digital-OT path instead of analog NTC, init,
  offset-cal exception). Note: product reads `BOARD_GATE_FAULT_GPIO` when `BOARD_HAS_GATE_FAULT_INPUT`
  — this board sets it 0, so map over-temp via `BOARD_OVERTEMP_GPIO`, do not reuse the fault macro.

## Build / verify
```bash
# primary gate (compiles + links the board HAL with a lab):
BOARD=launchxl_3phganinv MOTOR=am_4116_kv450 LAB=is01_intro_hal bash build.sh
# regression: every single-motor lab:
BOARD=launchxl_3phganinv LAB=all bash build.sh
# secondary (src/ modules only; does NOT exercise board headers):
BOARD=launchxl_3phganinv MOTOR=am_4116_kv450 SRC_CHECK=1 bash build.sh
```
