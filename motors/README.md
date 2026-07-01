# motors/ — Motor Parameter Variants (Axis of Change 2)
One header file per motor, containing all FOC parameters for that motor: pole pairs, Rs, Ls(d/q), flux/Ke,
rated current/voltage, maximum speed, inertia, etc. Swapping a motor means swapping one header here — no changes to app/ or boards/.

## Selection Mechanism (integrated into the build)
- `MOTOR=<model> bash build.sh` → injects `-DBUILD_MOTOR_ID` → `motor_select.h` includes the corresponding profile.
- board user.h wraps the SDK example motor chain in `#if (BUILD_MOTOR_ID==TEMPLATE)`:
  - Default `motor_template`: uses the SDK example motor (Teknic), consistent with historical behavior.
  - Real motor selected: example chain is skipped; `motors/<model>.h` provides all `USER_MOTOR_*` values.
- IDs are registered in `config/build_config.h` + the `MOTOR` case in `build.sh`; build gates verify
  the selected profile is wired into the target board/user.h path.

## Supported Motors (custom NovaX/AM series, 4 models)
4 profiles are created and selectable. **Geometric pole pairs are filled** (4116=7, 62xx=14).
The **4116 KV450 profile now carries esc6288/current-scale-corrected values** (is05 completed on esc6288, 2026-07-01): Rs 0.0213 Ω (phase-neutral; bench line-line 42–43 mΩ / 2), Ls 30.0 µH (re-IDed after current-scale correction), flux ~0.012 V/Hz (consistent with the KV450 wind/profile; KV450 is a nameplate term, not a directly-measured rpm/V). The **62xx Rs/Ls/flux are still bench seeds** — run is05 and back-fill measured values:
- [x] `am_4116_kv450.h` (KV450 wind, 7 pp) — Rs/Ls/flux from esc6288 is05, 2026-07-01 (current-scale-corrected: Rs 0.0213 Ω phase-neutral, Ls 30.0 µH, flux ~0.012 V/Hz)
- [x] `am_4116_kvb.h` (KV470 wind, 7 pp) — Rs/Ls/flux from legacy FAST re-ID (esc_drv8300, 2026-06-11)
- [ ] `am_6212.h` (14 pole pairs)
- [ ] `am_6215.h` (14 pole pairs)

> **AM-4116 note:** is05 on the launchxl is NOT the back-fill path for the 4116 — FAST ID there hits
> the BOOSTXL-DRV8305 over-current protection (the ~40 mOhm line-line motor draws an instant
> ID-startup current transient past the launchxl CMPSS trip / its +-23.57 A sense ceiling). The
> esc6288 shunt/CMPSS are sized for this motor, and is05 now COMPLETES on esc6288 (continuous
> execution — halt-sampling desyncs the Ls phase). Runner: `tools/flash/esc6288_revA/run_is05_esc6288.js`.
> The 2026-07-01 esc6288 is05 result (with current-scale corrected to 254 A) is the authoritative
> back-fill: Rs 0.0213 Ω (phase-neutral), Ls 30.0 µH, flux ~0.012 V/Hz. On the launchxl, the 4116
> is limited to small-Iq low-load sanity (is06); full-power ID and running belong to esc6288. The
> 62xx still follow the is05 flow.

> Macro names aligned with SDK 6.0 `USER_MOTOR_*`. Operating range fields (FREQ/VOLT/rated) and inertia are placeholders; adjust per motor and power supply.
> Full decoupling (board carries only hardware-scaling macros, completely free of motor examples) can be cleaned up later; the current "wrap-and-skip" approach is sufficient.

> The legacy project (`../esc_drv8300_foc`) contains earlier parameters for these motors. The **4116
> legacy values are superseded**: the legacy Rs (0.0403 Ω) was a line-line value from the MotorWare
> pu→Ω recipe, not phase-neutral (correct phase-neutral = 0.0213 Ω); the legacy/old-scale Ls was
> identified under the uncorrected 330 A current scale and is superseded by the 2026-07-01 esc6288
> correction (30.0 µH). Use the values in `am_4116_kv450.h` (esc6288 is05, 2026-07-01). The
> **6212/6215 legacy profiles still have known cross-contamination** — for those, re-identify on
> hardware with is05 in this project rather than reusing the legacy numbers.

## Standard Workflow: Select Motor → is05 Identification → Back-fill → Tune Loops
The product target is `esc6288_revA`. Run motor identification only after the esc6288 staged
bring-up has passed far enough to make switching safe (rails/clock, OST, ADC, protection,
dead-band/short-pulse). The old launchxl workflows are historical validation paths and are not the
default place to identify new motor profiles.

Example using `am_6215` on esc6288 after the bench safety checks pass (same procedure for other
motors, change `MOTOR=`):

0. **Pre-power safety check**: pole pairs correct (geometry), `MAX_CURRENT_A`/`RES_EST`/`IND_EST` within power supply current limit,
   CMPSS overcurrent wired (before high-current operation; see board PORT_TODO), power supply set to low voltage (12–24 V) first.

1. **Identification**:
   ```bash
   BOARD=esc6288_revA MOTOR=am_6215 LAB=is05_motor_id bash build.sh
   ```
   Flash and run; read FAST-estimated `Rs / Ls_d / Ls_q / flux (Flux)` (via watch variables / datalog).

2. **Back-fill**: write measured values back into `motors/am_6215.h`, **overwriting** the bench-seed lines (`Rs_Ohm / Ls_d_H /
   Ls_q_H / RATED_FLUX_VpHz`), and update the comment from "seed, is05 to overwrite" to "is05 measured yyyy-mm-dd".
   Also fill in `MOTOR_KV_RPM_PER_V`. Check off the `[ ]` item in the list above.

3. **Verify current loop**:
   ```bash
   BOARD=esc6288_revA MOTOR=am_6215 LAB=is06_torque_control bash build.sh
   ```
   Run with small Iq on a free shaft; confirm current loop is stable and phase order is correct (no reverse rotation / no step-out).

4. **Speed loop + tuning**:
   ```bash
   BOARD=esc6288_revA MOTOR=am_6215 LAB=is07_speed_control bash build.sh
   ```
   Tune `INERTIA_Kgm2` and speed-loop Kp/Ki; converge `FREQ_MAX_HZ`/`RATED_*`/`VOLT_*` to measured operating range.

5. After completing each motor, `git commit` the profile (note is05 date + power supply / temperature conditions).

> Regression: after any change, `BOARD=<board> LAB=all bash build.sh` should still report 12/12, 0 warnings (default template, not affected by profiles).
