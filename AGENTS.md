# Repository Guidelines

## Project Structure & Module Organization
This is F280049C FOC firmware built on C2000Ware MotorControl SDK 6.0 with driverlib and EABI. During bring-up, lab control code comes from `solutions/common/sensorless_foc/source/<LAB>.c`; this repo supplies board layers, motor profiles, and `build.sh`.

- `boards/<board>/`: board HAL, gate-driver code, linker command files, and `PORT_TODO.md`; current boards are `esc6288_revA` and `launchxl_drv8305evm`.
- `motors/`: one selectable motor profile per header plus `motor_select.h`.
- `config/build_config.h`: board and motor ID definitions used by `build.sh`.
- `src/{app,comms,encoder,common}/`: planned board/motor-agnostic product code; currently placeholder directories.
- `docs/`: datasheets, errata, TRM, and lab-guide references.

Runtime code is organized around three axes: hardware in `boards/`, motor parameters in `motors/`, and future core logic in `src/`. Adding a board or motor still requires registration in `build.sh`, `config/build_config.h`, and the relevant selection header.

## Build, Test, and Development Commands
Use `build.sh`; it selects by `BOARD`, `MOTOR`, and `LAB`, then writes to `build/<BOARD>/<MOTOR>/<LAB>/`.

```bash
bash build.sh
BOARD=esc6288_revA LAB=is01_intro_hal bash build.sh
BOARD=launchxl_drv8305evm LAB=all bash build.sh
BOARD=launchxl_drv8305evm MOTOR=am_6215 LAB=is05_motor_id bash build.sh
CGT=/path/to/ti-cgt-c2000 MCSDK_ROOT=/path/to/C2000Ware_MotorControl_SDK_6_00_00_00 bash build.sh
```

`LAB=all` is the main regression check. It smoke-builds single-motor labs and excludes `is11_dual_motor`. Toolchain paths are environment-overridable; `CGT` is auto-detected from common TI install locations (override or pin a version with `CGT=...`), and `MCSDK_ROOT` defaults to the in-repo SDK. Always link `_eabi` SDK libraries.

## Coding Style & Naming Conventions
Follow the surrounding TI driverlib/C2000Ware C style. Keep board-specific logic under `boards/<board>/`, declarations in headers, and implementation in C files. Use uppercase macros for build-time IDs and SDK parameters (`BUILD_BOARD_ID_*`, `USER_MOTOR_*`). Use lowercase selector names such as `launchxl_drv8305evm` and `am_6215`.

## Testing Guidelines
There is no separate unit-test framework yet. Treat clean C2000 builds as baseline verification. After build-script, board-HAL, motor-selection, or SDK-lab changes, run:

```bash
BOARD=<board> LAB=all bash build.sh
```

For motor-profile work, also build the target lab, for example `MOTOR=am_6215 LAB=is05_motor_id`.

## Safety & Configuration Notes
The local SDK tree and `build/` outputs are gitignored. Do not edit SDK vendor sources unless documenting a fork. Both boards hold gate-driver enable low during GPIO setup; power-on work must confirm the `HAL_enableDRV()` path and current-limit/overcurrent state first.

## Commit & Pull Request Guidelines
Recent commits use concise scoped subjects such as `build.sh: nest output by motor too` and `docs(motors): add the select->is05->backfill->tune workflow memo`. Keep subjects specific.

Pull requests should state the affected board, motor, and lab; list exact build commands run; note hardware validation status or safety limits; and link relevant `PORT_TODO.md` items or bench notes.
