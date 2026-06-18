# motors/ — Motor Parameter Variants (Axis of Change 2)
One header file per motor, containing all FOC parameters for that motor: pole pairs, Rs, Ls(d/q), flux/Ke,
rated current/voltage, maximum speed, inertia, etc. Swapping a motor means swapping one header here — no changes to app/ or boards/.

## Selection Mechanism (integrated into the build)
- `MOTOR=<model> bash build.sh` → injects `-DBUILD_MOTOR_ID` → `motor_select.h` includes the corresponding profile.
- board user.h wraps the SDK example motor chain in `#if (BUILD_MOTOR_ID==TEMPLATE)`:
  - Default `motor_template`: uses the SDK example motor (Teknic), consistent with historical behavior.
  - Real motor selected: example chain is skipped; `motors/<model>.h` provides all `USER_MOTOR_*` values.
- IDs are registered in `config/build_config.h` + the `MOTOR` case in `build.sh`. Verified on both boards (20/20 pass).

## Supported Motors (custom NovaX/AM series, 4 models)
4 profiles are created and selectable. **Geometric pole pairs are filled** (4116=7, 62xx=14);
**Rs/Ls/flux are currently bench seeds** (compile and run is05); update with measured values after power-on identification:
- [ ] `am_4116_kva.h` (KV-A, 7 pole pairs) — KV to be filled; is05 to back-fill Rs/Ls/flux
- [ ] `am_4116_kvb.h` (KV-B, different KV from A, 7 pole pairs)
- [ ] `am_6212.h` (14 pole pairs)
- [ ] `am_6215.h` (14 pole pairs)

> Macro names aligned with SDK 6.0 `USER_MOTOR_*`. Operating range fields (FREQ/VOLT/rated) and inertia are placeholders; adjust per motor and power supply.
> Full decoupling (board carries only hardware-scaling macros, completely free of motor examples) can be cleaned up later; the current "wrap-and-skip" approach is sufficient.

> The legacy project (`../esc_drv8300_foc`) contains older parameters for these motors, but the 6212/6215 profiles have known cross-contamination and the 4116 flux has a known offset — **do not reuse directly; re-identify everything on hardware with is05 in this project**.

## Standard Workflow: Select Motor → is05 Identification → Back-fill → Tune Loops
Example using `am_6215` on the validation board (same procedure for other motors, change `MOTOR=`):

0. **Pre-power safety check**: pole pairs correct (geometry), `MAX_CURRENT_A`/`RES_EST`/`IND_EST` within power supply current limit,
   CMPSS overcurrent wired (before high-current operation; see board PORT_TODO), power supply set to low voltage (12–24 V) first.

1. **Identification**:
   ```bash
   BOARD=launchxl_drv8305evm MOTOR=am_6215 LAB=is05_motor_id bash build.sh
   ```
   Flash and run; read FAST-estimated `Rs / Ls_d / Ls_q / flux (Flux)` (via watch variables / datalog).

2. **Back-fill**: write measured values back into `motors/am_6215.h`, **overwriting** the bench-seed lines (`Rs_Ohm / Ls_d_H /
   Ls_q_H / RATED_FLUX_VpHz`), and update the comment from "seed, is05 to overwrite" to "is05 measured yyyy-mm-dd".
   Also fill in `MOTOR_KV_RPM_PER_V`. Check off the `[ ]` item in the list above.

3. **Verify current loop**:
   ```bash
   BOARD=launchxl_drv8305evm MOTOR=am_6215 LAB=is06_torque_control bash build.sh
   ```
   Run with small Iq on a free shaft; confirm current loop is stable and phase order is correct (no reverse rotation / no step-out).

4. **Speed loop + tuning**:
   ```bash
   BOARD=launchxl_drv8305evm MOTOR=am_6215 LAB=is07_speed_control bash build.sh
   ```
   Tune `INERTIA_Kgm2` and speed-loop Kp/Ki; converge `FREQ_MAX_HZ`/`RATED_*`/`VOLT_*` to measured operating range.

5. After completing each motor, `git commit` the profile (note is05 date + power supply / temperature conditions).

> Regression: after any change, `BOARD=<board> LAB=all bash build.sh` should still report 12/12, 0 warnings (default template, not affected by profiles).
