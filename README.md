# f280049c_foc -- esc6288 FOC firmware (TMS320F280049C)

Modern FOC firmware based on **C2000Ware MotorControl SDK 6.00.00.00**
(driverlib + EABI). It replaces the legacy `../esc_drv8300_foc` F28027F /
MotorWare code and the interim `../motorware_clean` F28062F work.

## Project Focus

The product target is **`esc6288_revA`**. Current software work should serve that
board directly, or keep shared build/test gates healthy for it.

The two LaunchPad boards remain in the tree because they were useful validation
platforms and still catch portability regressions, but they are not current
bring-up targets:

- `launchxl_drv8305evm`: historical DRV8305 validation platform; on hold.
- `launchxl_3phganinv`: historical GaN validation platform; on hold.

Do not spend new feature work on those validation boards unless it fixes a
shared regression that affects `esc6288_revA`.

## Target Platform

- MCU: **TMS320F280049C** (F28004x, 100 MHz, FPU + **TMU**, 256 KB Flash /
  100 KB RAM)
- Product board: **`esc6288_revA`**, custom 12S ESC, currently at fab
- Gate driver: **JSM6288T**, six independent inputs, no EN pin, no nFAULT pin,
  internal anti-shoot-through interlock + dead time
- Safe-off model: ePWM trip-zone one-shot (**OST**) is the gate-disable
  mechanism; `HAL_enableDRV()` is a no-op on esc6288
- Control stack: SDK FAST sensorless core plus esc6288 product glue, DroneCAN,
  MT6701 absolute encoder path, NTC temperature protection, and host-tested pure
  control/support modules

## Current Status

- `esc6288_revA` is ported from the final schematic + netlist.
- Product application links for esc6288 (`PRODUCT=1`).
- SDK single-motor lab regression builds (`LAB=all`, 12/12) are green for
  esc6288.
- Pure `src/` modules are host-tested and cross-compiled as a zero-warning gate.
- CAN bridge/comms and product glue have dedicated cross-compile gates.
- Bring-up scripts for non-spinning stages are written under
  `tools/flash/esc6288_revA/` and syntax-checked. They have not run on hardware
  yet because the board has not returned from fab.

Implemented esc6288-facing software:

- MT6701 SSI frame decode + CRC6, board read adapter, and feedback plumbing
- NTC count -> degrees C conversion and live over-temperature latch
- Speed-mode setpoint skeleton, default gated off
- `nvparam` storage record for node-id + learned park reference, with CRC/range
  validation and host tests
- DroneCAN `uavcan.protocol.param.GetSet` access to those persisted fields
- CAN + RC-PWM dual-throttle arbiter (`src/app/esc_arbiter`) wired into the 1 ms
  tick; fuses the DroneCAN and RC-PWM throttle sources but ships inert
  (`ESC_ARB_EXPLICIT_CAN`, PWM ignored, behavior identical to CAN-only). Enable
  gate + bench prerequisites are in `boards/esc6288_revA/PORT_TODO.md`.
- esc6288 staged bring-up DSS scripts for rails/clock, idle/OST, ADC,
  protection, and peripherals

Hardware-dependent items intentionally deferred until the prototype is on the
bench:

- Stage 4 first-spin script and any real PWM output beyond explicit OST tests
- JSM6288T/MCU dead-time final tuning by oscilloscope
- MT6701 SSI polarity/timing and encoder zero/dir tuning
- Real Flash erase/program for `nvparam`
- Speed-loop ISR consumption/tuning, auto-park enable, active brake, and flight
  current/temperature thresholds

See `boards/esc6288_revA/PORT_TODO.md` for the authoritative bench checklist.

## Selected Reference Base

The retained SDK reference base is
`solutions/boostxl_drv8320rs/f28004x` (explicit target = F280049C). Only the
HAL/lab structure needed for porting is retained. The SDK lab ladder is still
useful as a compile regression and low-level bring-up reference:

```text
is01_intro_hal          HAL / clock / PWM bringup
is02_offset_gain_cal    ADC offset/gain calibration
is03_hardware_test      hardware self-test
is04_signal_chain_test  signal chain test
is05_motor_id           motor identification
is06_torque_control     torque loop
is07_speed_control      speed loop
... is13_fwc_mtpa       field weakening / MTPA
```

Single-motor labs (is01-is10, is12, is13) link cleanly; `is11_dual_motor` is
out of scope and `build.sh` rejects it.

## Boards

| Board (`BOARD=`) | Role now | Gate driver | Current sensing | Status |
|---|---|---|---|---|
| `esc6288_revA` | **Product target** | JSM6288T, 6-input, no EN/nFAULT, internal interlock + DT | 3x INA181A1 over 0.5 mohm shunts | Schematic/netlist port complete; product and lab build gates green; bench verification pending |
| `launchxl_drv8305evm` | On-hold validation/regression board | DRV8305 SPI gate driver / integrated CSA | DRV8305 CSA to ADC | Historical bring-up notes retained; no new feature work planned |
| `launchxl_3phganinv` | On-hold validation/regression board | LMG5200 GaN half-bridges via buffer | INA240 over 5 mohm in-line shunts | Buildable regression path; bench work paused |

All boards share the same FOC/control core. Board-specific HAL, gate driver,
pinout, scaling, and safety details live under `boards/<board>/`.

## Toolchain

- Compiler: `cl2000` from TI C2000 CGT. `build.sh` prefers the validated
  `ti-cgt-c2000_22.6.0.LTS` and can be pinned with `CGT=/path/to/ti-cgt-c2000_<ver>`.
- SDK: `C2000Ware_MotorControl_SDK_6_00_00_00`; either place it in the project
  root or set `MCSDK_ROOT=/path/to/C2000Ware_MotorControl_SDK_6_00_00_00`.
- ABI: **EABI** (`--abi=eabi`). Use `_eabi` SDK library variants such as
  `fluxHF_eabi.lib` and `f28004x_fast_rom_symbols_fpu32_eabi.lib`.

## Getting Started

Install the TI compiler and MotorControl SDK, then run the esc6288 gates:

```bash
bash tools/test/run.sh
BOARD=esc6288_revA MOTOR=am_4116_kva SRC_CHECK=1     bash build.sh
BOARD=esc6288_revA                   CAN_CHECK=1     bash build.sh
BOARD=esc6288_revA MOTOR=am_4116_kva PRODUCT_CHECK=1 bash build.sh
BOARD=esc6288_revA MOTOR=am_4116_kva PRODUCT=1       bash build.sh
BOARD=esc6288_revA LAB=all bash build.sh
```

The default build is still a quick SDK lab build:

```bash
bash build.sh   # BOARD=esc6288_revA LAB=is01_intro_hal MOTOR=motor_template
```

Outputs go to `build/<BOARD>/<MOTOR>/<LAB>/` or, for the product image, the
product subdirectory under `build/<BOARD>/<MOTOR>/`.

## Build Knobs

```bash
BOARD=esc6288_revA LAB=is01_intro_hal bash build.sh
BOARD=esc6288_revA MOTOR=am_4116_kva PRODUCT=1 bash build.sh
NODE_ID=25 ESC_INDEX=0 BOARD=esc6288_revA MOTOR=am_4116_kva PRODUCT=1 bash build.sh
EXTRA_DEFINES="--define=ESC6288_SPEED_MODE_DEFAULT=1" BOARD=esc6288_revA MOTOR=am_4116_kva PRODUCT=1 bash build.sh
MCSDK_ROOT=/path/to/C2000Ware_MotorControl_SDK_6_00_00_00 bash build.sh
```

- `BOARD` selects the board HAL and injects `BUILD_BOARD_ID`; each `board.h`
  self-checks the ID.
- `MOTOR` selects `motors/<model>.h` and injects `BUILD_MOTOR_ID`. Options:
  `motor_template`, `am_4116_kva`, `am_4116_kvb`, `am_6212`, `am_6215`.
- `PRODUCT=1` links `product/product_main.c` instead of an SDK lab.
- `NODE_ID=0` means DroneCAN dynamic node allocation; `1..127` is a static node id.
- `ESC_INDEX=0..19` selects the RawCommand array index.
- `PWM_PHASE_ORDER=auto|0..5` defaults to each board's configured order.
- `EXTRA_DEFINES` appends compiler defines verbatim for explicitly gated build
  switches.

## esc6288 Hardware Safety Model

On `esc6288_revA`, there is no physical gate-enable pin. The safe state is:

- EPWM1/2/3 trip-zone OST forced/set
- `flagRunIdentAndOnLine == 0` (not armed)
- no code path clears OST except the explicit arm path

`HAL_setupFaults()` leaves the outputs OST-forced. Offset calibration keeps the
gates tripped so ADC offsets sample with the bridge off. The stage scripts assert
this invariant before doing any read/check, and any script path that can un-trip
the EPWM requires an explicit argument and returns to safe-off on exit.

The JSM6288T has its own internal interlock and dead time. The MCU dead-band is
therefore extra margin, currently set to 20 counts (~200 ns) and left for bench
scope validation before any flight value is trusted.

## Bring-Up Scripts

esc6288 scripts live in `tools/flash/esc6288_revA/`:

```bash
dss.sh tools/flash/esc6288_revA/s1_rails_clock.js tools/flash/common/f280049c_xds110.ccxml <product.out>
dss.sh tools/flash/esc6288_revA/s2_idle_ost.js    tools/flash/common/f280049c_xds110.ccxml <product.out> [verify=offcal|verify=untrip]
dss.sh tools/flash/esc6288_revA/s3_adc_offsets.js tools/flash/common/f280049c_xds110.ccxml <product.out>
dss.sh tools/flash/esc6288_revA/s5_protection.js  tools/flash/common/f280049c_xds110.ccxml <product.out> [force=tz|inject=oc|inject=ov]
dss.sh tools/flash/esc6288_revA/s6_peripherals.js tools/flash/common/f280049c_xds110.ccxml <product.out>
```

Run them in order when the prototype returns. Stage 4 (first spin) is
deliberately not scripted yet; write it only after stages 1/2/3/5/6 have passed
on real hardware and the oscilloscope conditions are clear.

## Future Product Backlog

- Flash persistence: decode the stored `nvparam` record at boot and write it via
  driverlib Flash erase/program when node-id or park-ref changes.
- DroneCAN restart/parameter polish: `RestartNode` and any richer parameter UX
  needed after bench interop with ArduPilot/yakut.
- CAN DFU/OTA: bootloader plus DroneCAN file-transfer/update flow.
- Power-on startup chime: bench-tuned motor-winding tones before arming, if it
  remains desirable after the safety sequence is proven.

## Reference Blueprint

`../esc_drv8300_foc` remains a logic/parameter reference only: motor parameters,
MT6701 experience, DroneCAN behavior, and control lessons. Do not reuse legacy
MotorWare code directly in this project.

## Directory Structure

```text
f280049c_foc/
├── C2000Ware_MotorControl_SDK_6_00_00_00/  # vendor SDK, gitignored
├── docs/                                    # local datasheets / errata / SDK notes
├── boards/
│   ├── esc6288_revA/                        # product board HAL, drivers, linker, PORT_TODO
│   ├── launchxl_drv8305evm/                 # on-hold validation board
│   └── launchxl_3phganinv/                  # on-hold validation board
├── config/                                  # BUILD_BOARD_ID / BUILD_MOTOR_ID selection
├── motors/                                  # motor profiles
├── product/                                 # esc6288 product main
├── src/
│   ├── app/                                 # pure control glue, throttle arbiter, park/ref, nvparam
│   ├── comms/                               # pure DroneCAN protocol layers
│   ├── common/                              # shared DTOs, NTC conversion
│   └── encoder/                             # MT6701 pure decode/tracking
├── tools/flash/                             # DSS bench scripts and CAN host probes
├── tools/test/                              # host tests and generated goldens
└── build.sh                                 # BOARD=.. MOTOR=.. LAB=.. PRODUCT=..
```
