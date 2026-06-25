# Repository Guidelines

## Project Structure & Module Organization
This is F280049C FOC firmware on C2000Ware MotorControl SDK 6.0, driverlib, and EABI. Labs come from `solutions/common/sensorless_foc/source/<LAB>.c`; this repo supplies boards, motors, pure product modules, and `build.sh`.

- `boards/<board>/`: board HAL, gate-driver code, linker files, and `PORT_TODO.md`; boards are `esc6288_revA`, `launchxl_drv8305evm`, and `launchxl_3phganinv` (BOOSTXL-3PhGaNInv GaN BoosterPack).
- `motors/`: selectable motor profiles plus `motor_select.h`.
- `config/build_config.h`: board and motor ID definitions used by `build.sh`.
- `src/{app,comms,encoder,common}/`: board/motor-agnostic product logic: ESC state control, prop parking, MT6701 angle processing, DroneCAN helpers, and shared DTOs.
- `docs/`: datasheets, errata, TRM, and lab-guide references.

Runtime code follows three axes: hardware in `boards/`, motor parameters in `motors/`, and board-agnostic logic in `src/`. Adding a board or motor also requires registration in `build.sh`, `config/build_config.h`, and the relevant selector.

## Build, Test, and Development Commands
Use `build.sh`; it selects by `BOARD`, `MOTOR`, and `LAB`, then writes to `build/<BOARD>/<MOTOR>/<LAB>/`.

```bash
bash build.sh
BOARD=launchxl_drv8305evm LAB=all bash build.sh
BOARD=launchxl_drv8305evm MOTOR=am_6215 LAB=is05_motor_id bash build.sh
CGT=/path/to/ti-cgt-c2000 MCSDK_ROOT=/path/to/C2000Ware_MotorControl_SDK_6_00_00_00 bash build.sh
```

`LAB=all` is the main regression check. It smoke-builds single-motor labs and excludes `is11_dual_motor`. `CGT` and `MCSDK_ROOT` are environment-overridable; always link `_eabi` SDK libraries.

## Coding Style & Naming Conventions
Follow nearby TI driverlib/C2000Ware C style. Keep board-specific logic under `boards/<board>/`, declarations in headers, and implementation in C files. Use uppercase macros for build-time IDs and SDK parameters (`BUILD_BOARD_ID_*`, `USER_MOTOR_*`), and lowercase selector names such as `launchxl_drv8305evm`.

## Testing Guidelines
Host tests cover pure `src/` logic. Use the narrowest gate for the change, then `LAB=all` for regression:

```bash
bash tools/test/run.sh
BOARD=esc6288_revA MOTOR=am_4116_kva SRC_CHECK=1 bash build.sh
BOARD=launchxl_drv8305evm MOTOR=am_4116_kva SRC_CHECK=1 bash build.sh
BOARD=launchxl_drv8305evm CAN_CHECK=1 bash build.sh
BOARD=<board> LAB=all bash build.sh
```

`SRC_CHECK=1` compiles product modules without linking labs. `CAN_CHECK=1` compiles a board's CAN bridge (`launchxl_drv8305evm` and `esc6288_revA` are wired); boards without a `can_bridge.c` (currently `launchxl_3phganinv`) friendly-skip. For motor-profile work, also build the target lab.

## Safety & Configuration Notes
The local SDK tree and `build/` outputs are gitignored. Do not edit SDK vendor sources unless documenting a fork. Keep board-agnostic `src/` modules free of driverlib, board HAL, and SDK-lab dependencies. Every board brings up its gate stage in the *disabled* state during GPIO setup: `esc6288_revA` has no gate-enable pin (JSM6288T, also no nFAULT) and relies on the ePWM trip-zone (OST) for safe-off (`HAL_enableDRV()` is a no-op); `launchxl_drv8305evm` drives its active-high EN_GATE low; the active-low `launchxl_3phganinv` (nEn_uC buffer OE) drives its pin high; power-on work must confirm `HAL_enableDRV()` and current-limit/overcurrent state first. The `launchxl_drv8305evm` and `esc6288_revA` CAN bridges are compile-checked only (CANA, 1 Mbit; not yet hardware-validated); `launchxl_3phganinv` CAN pins remain TODO. Note: `launchxl_3phganinv` uses LMG5200 GaN half-bridges with NO internal dead-time — the MCU dead-band is the only shoot-through protection.

## Commit & Pull Request Guidelines
Use concise scoped commit subjects; keep them specific.

Pull requests should state affected board/motor/lab, exact build commands, hardware validation or safety limits, and relevant TODOs or bench notes.
