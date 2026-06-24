# f280049c_foc — New ESC Project (TMS320F280049C)

Modern FOC architecture based on **C2000Ware MotorControl SDK 6.00.00.00** (driverlib + EABI).
Replaces the legacy `../esc_drv8300_foc` (F28027F / MotorWare, end-of-life) and the interim `../motorware_clean` (F28062F, abandoned).

## Target Platform
- MCU: **TMS320F280049C** (F28004x, 100 MHz, FPU + **TMU**, 256 KB Flash / 100 KB RAM)
- Software stack: **C2000Ware MotorControl SDK 6.0** + driverlib (**not MotorWare**)
- Hardware boards: production board `esc6288_revA` uses a JSM6288T (6-input, no enable/SPI) gate driver; validation boards `launchxl_drv8305evm` (DRV8305) and `launchxl_3phganinv` (BOOSTXL-3PhGaNInv, LMG5200 GaN) (see Boards table)
- Control: InstaSPIN **FAST sensorless** (F280049C has FAST ROM) + future **MT6701 sensored** integration

## Selected Reference Base
SDK reference base `solutions/boostxl_drv8320rs/f28004x` (explicit target = F280049C); only the HAL/lab structure needed for porting is retained.
Incremental bring-up lab ladder (analogous to legacy MotorWare proj_lab01..10):
```
is01_intro_hal          HAL / clock / PWM bringup  ← current validation point
is02_offset_gain_cal    ADC offset/gain calibration
is03_hardware_test      hardware self-test
is04_signal_chain_test  signal chain test
is05_motor_id           motor identification
is06_torque_control     torque loop
is07_speed_control      speed loop
... is13_fwc_mtpa       field weakening / MTPA
```
> Single-motor labs (is01–is10, is12, is13) link cleanly on all boards with 0 warnings.
> **is11_dual_motor does not apply** (dual-motor; requires the removed `user_m1/m2/dm` + `hal_dm` scaffolding);
> `build.sh` exits with an error if it is selected.
>
> Control module sources needed by each lab (`vs_freq/vsf/fwc/mtpa`) are already included in `build.sh`; is12 automatically adds the `_VSF_EN_` macro.

Each lab has both EABI and COFF variants — **this project uses EABI**.

## Boards (three boards, orthogonal to the control core)
| Board (`BOARD=`) | Role | Gate driver | Current sensing | Status |
|------|------|---------|---------|------|
| `esc6288_revA` | Production ESC (custom, in fabrication) | JSM6288T / simple 6-input (no EN, no nFAULT, no SPI) | External shunt + op-amp | is01 builds; board-level HAL pending schematic migration into `board.h` |
| `launchxl_drv8305evm` | **Validation platform** (TI LaunchPad + BoosterPack) | DRV8305 (SPI-programmable, integrated CSA) | DRV8305 integrated CSA → direct ADC | **Phases 1–3 complete**: is01 pins correct and building, PGA/ADC front-end correct, DRV8305 SPI driver in place; Phase 4 power-on pending hardware |
| `launchxl_3phganinv` | **GaN validation platform** (TI LaunchPad + BOOSTXL-3PhGaNInv) | LMG5200 GaN half-bridges via SN74AVC8T245 buffer (nEn_uC active-low; no SPI) | INA240 (gain 20) on 5 mΩ in-line shunts → direct ADC | **Build complete** (LAB=all 12/12, SRC_CHECK clean); power-on + bench-verify pending (see `PORT_TODO.md`: dead-time, current sign, OT trip) |

> Use the LaunchPad validation boards to validate firmware and motor before the custom board returns (hardware-decoupled validation).
> All boards share the same FOC control core; only the board layer (HAL / gate driver / scaling) differs.
> See `boards/<board>/PORT_TODO.md` for details.

## Toolchain (verified on this machine)
- Compiler: `cl2000` (TI C2000 CGT). `build.sh` auto-detects a `ti-cgt-c2000_*` install (newest under `~/ti/ccs*`, `/opt/ti`, …); override or pin with `CGT=/path/to/ti-cgt-c2000_<ver>`. Validated with 22.6.0.LTS; EABI is forward-compatible.
- SDK: `./C2000Ware_MotorControl_SDK_6_00_00_00` (in-project, gitignored)
- driverlib (f28004x) prebuilt: `.../c2000ware/driverlib/f28004x/driverlib/ccs/Release/driverlib_eabi.lib`
- **ABI: EABI** (`--abi=eabi`) — modern native ABI; **no need** for the legacy `--abi=coffabi` hack used on F28062F
- ⚠️ Always use `_eabi`-suffix library variants when linking SDK libraries (e.g., `fluxHF_eabi.lib`, `f28004x_fast_rom_symbols_fpu32_eabi.lib`);
  linking COFF variants by mistake will cause unresolved `EST_*` symbols.

## Completed
- [x] SDK 6.0 installed and F28004x support verified
- [x] `is01_intro_hal` (RAM/EABI) command-line compile + link verified → valid c28xabi ELF image
- [x] `build.sh` (parameterized `BOARD × MOTOR × LAB`, injects board/motor IDs and extra sources, outputs to `build/<BOARD>/<MOTOR>/<LAB>/`)
- [x] **esc6288_revA**: gate driver converged from DRV8320 SPI template to `gate_driver.c/.h` + `board.h`; HAL GPIO safely cleaned up
- [x] **launchxl_drv8305evm validation platform (Phases 1–3)**:
  - Pin mapping (Site 1 / J1-J4) locked from SysConfig board file → is01 all pins correct, builds cleanly
  - Analog front-end corrected: current ADC B2/C0/A9 (phase order A, B, C); on-chip PGA disabled (DRV8305 has integrated CSA)
  - Scaling: 44.30 V / 47.14 A; CMPSS overcurrent mapping verified against datasheet (CMPSS3/1/6, pending hardware connection)
  - DRV8305 SPI register driver `drv8305.c/.h` (driverlib) compiled and integrated into `HAL_enableDRV`/`HAL_setupSPIA`
    (⚠️ stock is01 only enters the enable path under `DRV8320_SPI`; the **`DRV8305_configure()` runtime entry point must be hooked into the lab during bring-up**)

## To Do (bring-up sequence)
**Near-term (validation platform `launchxl_drv8305evm`, Phase 4 = power-on, requires hardware)**
1. Import project into CCS Theia; use SysConfig to generate CMPSS overcurrent configuration for DRV8305EVM (see `boards/launchxl_drv8305evm/PORT_TODO.md`)
2. DRV8305 SPI smoke test (read status/ID, confirm 6-PWM mode, oscilloscope timing verification)
3. ADC calibration (is02) → hardware self-test (is03, start with low voltage / low current) → motor identification (is05) → torque/speed loops (is06/07)

**Production board (`esc6288_revA`, once custom board returns)**
4. Board-level HAL porting: migrate GPIO/ADC/EPWM/CMPSS assignments from schematic into `boards/esc6288_revA/drivers/include/board.h`
5. Integrate **MT6701 encoder** (sensored FOC; reference SDK `absolute_encoder_boostxl_posmgr` / `servo_drive_with_can/sensored_foc`)
6. Integrate **CAN / DroneCAN** (reference `servo_drive_with_can`)

## Getting Started (first clone)
The vendor SDK and the TI compiler are **not** in the repo (gitignored / licensed). After cloning,
set up two external dependencies, then build to verify:

1. **TI C2000 compiler (CGT).** Install Code Composer Studio (which bundles `ti-cgt-c2000`) or the
   standalone CGT. `build.sh` auto-detects the newest `ti-cgt-c2000_*` under `~/ti/ccs*`, `~/ti`,
   `/opt/ti`. If it lives elsewhere, point to it:
   ```bash
   export CGT=/path/to/ti-cgt-c2000_<ver>      # e.g. .../ti-cgt-c2000_22.6.0.LTS to pin the validated version
   ```

2. **MotorControl SDK 6.00.00.00.** Download `C2000Ware_MotorControl_SDK_6_00_00_00` from TI and either
   place it in the project root (the default location, gitignored) or point to it:
   ```bash
   export MCSDK_ROOT=/path/to/C2000Ware_MotorControl_SDK_6_00_00_00
   ```

3. **Verify the build** (no hardware needed — command-line compile + link only):
   ```bash
   bash build.sh                                  # default board/motor/lab, should print ">>> DONE: ...is01_intro_hal.out"
   BOARD=launchxl_drv8305evm LAB=all bash build.sh # regression: all single-motor labs, expect "12 passed, 0 failed"
   ```

Host: Linux with `bash` (the build is a shell script; toolchain paths assume a Unix layout).
Outputs go to `build/<BOARD>/<MOTOR>/<LAB>/` (gitignored).

## Build
```bash
bash build.sh                                           # default: esc6288_revA / is01
BOARD=esc6288_revA        LAB=is01_intro_hal bash build.sh
BOARD=launchxl_drv8305evm LAB=is01_intro_hal bash build.sh   # validation platform (DRV8305_SPI auto-enabled)
BOARD=launchxl_drv8305evm LAB=all            bash build.sh   # smoke test: build all single-motor labs + summary (regression)
BOARD=launchxl_drv8305evm MOTOR=am_6215 LAB=is05_motor_id bash build.sh   # select motor profile
MCSDK_ROOT=/path/to/C2000Ware_MotorControl_SDK_6_00_00_00 bash build.sh
```
- `BOARD` → `build.sh` injects `-DBUILD_BOARD_ID`; each `board.h` self-checks to prevent board/build mismatch.
- `MOTOR` (default `motor_template`) → injects `-DBUILD_MOTOR_ID`, selects `motors/<model>.h`;
  when not template, board user.h skips the SDK example motor chain and uses the profile instead. Options:
  `motor_template / am_4116_kva / am_4116_kvb / am_6212 / am_6215`.
- `launchxl_drv8305evm` automatically appends `drv8305.c` and `--define=DRV8305_SPI`.

## Hardware Safety State
- All boards: `HAL_setupGPIOs()` brings up the gate stage in its *disabled* state by default; power-on testing requires explicit `HAL_enableDRV()` in the lab entry. The enable mechanism differs per board:
  - `esc6288_revA` (JSM6288T): **no gate-enable pin and no nFAULT pin** — safe-off is the ePWM trip-zone (OST), held until `HAL_enablePWM()`; `HAL_enableDRV()` is a no-op.
  - `launchxl_drv8305evm` (DRV8305): active-high EN_GATE on GPIO39, held **low** (disabled) until `HAL_enableDRV()`.
  - `launchxl_3phganinv` (LMG5200): active-low nEn_uC (buffer OE) on GPIO39, held **high** (disabled) until `HAL_enableDRV()` drives it low.
- **launchxl_3phganinv**: LMG5200 GaN half-bridges have **no internal dead-time** — the MCU dead-band (`HAL_PWM_DBRED_CNT`/`DBFED_CNT` in `hal.h`, ~200 ns) is the only shoot-through protection; verify on a scope before raising bus voltage.
- **esc6288_revA**: JSM6288T is a simple 6-input driver with no enable and no nFAULT — GPIO13 is not connected (input), `HAL_enableDRV()` is a no-op, and the gate stage is held off by the ePWM trip-zone (OST) until `HAL_enablePWM()`.
- **launchxl_drv8305evm**: EN_GATE=GPIO39 (low/off), nFAULT=GPIO13 (input); `HAL_enableDRV()` wakes and configures DRV8305.
  ⚠️ **Overcurrent protection (CMPSS) is not yet wired** (mapping verified; pending SysConfig generation) — must be connected before high-current operation (is05+); low-voltage low-current calibration/self-test can proceed first.

## Reference Blueprint (logic/parameters reference only — no code reuse)
- `../esc_drv8300_foc`: motor parameters, MT6701 driver logic, DroneCAN stack, control experience

## Directory Structure (layered by axis of change)
```
f280049c_foc/
├── C2000Ware_MotorControl_SDK_6_00_00_00/  # vendor SDK, referenced only, not modified (gitignore)
├── docs/                                    # local reference materials: TRM/datasheet/errata/SDK lab guide
├── boards/                                  # axis 1: hardware (MOSFETs / gate driver / shunt / layout)
│   ├── esc6288_revA/             # production ESC (JSM6288T, custom, in fabrication): board.h/hal.c/gate_driver.c/cmd/PORT_TODO.md
│   ├── launchxl_drv8305evm/      # validation platform (DRV8305EVM): same + drv8305.c/.h (SPI driver)
│   └── launchxl_3phganinv/       # GaN validation platform (BOOSTXL-3PhGaNInv, LMG5200): same, no SPI; active-low enable
├── config/build_config.h                    # board/motor ID selection (BUILD_BOARD_ID / BUILD_MOTOR_ID)
├── build.sh                                 # BOARD=.. MOTOR=.. LAB=.. bash build.sh
├── motors/                                  # axis 2 (integrated): one profile per motor + motor_select.h
│   # am_4116_kva/kvb, am_6212, am_6215, motor_template, motor_select.h
└── src/{app,comms,encoder,common}/         # axis 3 (planned): core logic, hardware/motor-agnostic (empty placeholders)
```
- Swap board/MOSFETs → add `boards/<new>`; swap motor → add `motors/<new>` + register ID in build_config.h/build.sh; `src/` untouched.
- **motors/ is integrated into the build**: `MOTOR=<model>` selects the profile; board user.h skips the SDK example motor chain in favor of it.
  All 4 profiles currently have bench-seed Rs/Ls/flux values (compile and run is05); update with measured values after identification (pole pairs filled from geometry).
- ⚠️ `src/` is still an empty placeholder — to be populated after is06/07 is running.
- Bring-up: `BOARD=esc6288_revA LAB=is01_intro_hal bash build.sh`
