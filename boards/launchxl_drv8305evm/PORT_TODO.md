# launchxl_drv8305evm — LAUNCHXL-F280049C + BOOSTXL-DRV8305EVM Porting Checklist

**Hardware-decoupled validation platform** while the custom board `esc6288_revA` is being fabricated: use the TI official kit to validate firmware and motor first.
The control core (FOC/FAST/motor identification/loop tuning) is board-agnostic and 100% reusable; this directory only addresses the board layer.

## Reusable Assets (verified)
- Pin mapping (SysConfig): `c2000ware/boards/.meta/LAUNCHXL_F280049C.syscfg.json`
  + `.../boosterpack_json_files/BOOSTXL-DRV8305EVM.syscfg.json`
- DRV8305 SPI register map: `../../esc_drv8300_foc/motorware_1_01_00_18/sw/drivers/drvic/drv8305/src/32b/f28x/f2806x/drv8305.h`
- DRV8305EVM HAL reference: `../../esc_drv8300_foc/.../hal/boards/boostxldrv8305_revA` (legacy f2806x; reference timing/register sequence only)
- PWM half-bridge mostly ready: this board's three phases = EPWM6/5/3 = the existing boostxl_drv8320rs HAL "J1/J2 connection" path.

## Pin Mapping (Site 1 / BoosterPack1 J1-J4) — locked; see board.h
| Function | Header | GPIO/ADC | Peripheral |
|------|:--:|:--:|------|
| U H/L | 40/39 | GPIO10/11 | EPWM6A/6B |
| V H/L | 38/37 | GPIO8/9   | EPWM5A/5B |
| W H/L | 36/35 | GPIO4/5   | EPWM3A/3B |
| Ia/Ib/Ic | 27/28/29 | ADCIN B2/C0/A9 | CMPSS 3/1/6 (datasheet Table 6-13; implemented + hw-verified) |
| Va/Vb/Vc | 23/24/25 | ADCIN A5/B0/C2 | — |
| Vbus | 26 | ADCIN B1 | — |
| SPI CLK/STE/SIMO/SOMI | 7/19/15/14 | GPIO56/57/16/17 | SPIA |
| EN_GATE | 13 | GPIO39 (out) | active-high |
| nFAULT | 3 | GPIO13 (in) | active-low |
| WAKE | 12 | GPIO23 | — |
| PWRGD | 16 | XRSn ⚠️ | not usable as GPIO; read SPI status instead |

## Scaling Constants (BOOSTXL-DRV8305EVM, CSA gain default 10 V/V + 7 mΩ shunt)
- `USER_ADC_FULL_SCALE_CURRENT_A = 47.14`  ← TI official value, confirmed against legacy project
- `USER_ADC_FULL_SCALE_VOLTAGE_V = 44.30`  ← ⚠️ pending confirmation against EVM voltage divider resistors (VSENSE ~43.2 k/4.99 k)
- Offsets: current 0.5 × 47.14 ≈ 23.57 A; Vbus offset see user.h

## Phase Tasks
- [x] **Phase 1 Skeleton**: directory / board.h (pins) / cmd linker reuse / board ID added to build_config.h (=2)
- [x] **Phase 2 HAL Adaptation** (drivers/{include,source}): modified from boostxl_drv8320rs J1/J2 path
  - [x] PWM: EPWM6/5/3 — naturally matches J1/J2 path, no changes needed
  - [x] Current ADC: SOC0 channels changed to B2/C0/A9 (`HAL_setupADCs`); getCurrent read indices reordered for phase sequence A, B, C (`hal.h`)
  - [x] Voltage ADC: A5/B0/C2/B1 — naturally matches, no changes needed
  - [x] Gate driver GPIO: EN_GATE=GPIO39 (out, low), nFAULT=GPIO13 (in, pull-up); GPIO40 former nFAULT marked unused; GPIO23 marked as WAKE
  - [x] Scaling: 44.30 V / 47.14 A (`user.h`)
  - [x] `BOARD=launchxl_drv8305evm LAB=is01_intro_hal bash build.sh` compiles and links cleanly
  - [x] **PGA front-end**: base HAL enables PGA1/3/5 (gain 12) in `HAL_setupPGAs` — that is the DRV8320RS
    on-chip PGA front-end. The DRV8305EVM uses the DRV8305 integrated CSA directly connected to ADC pins, and **B2=PGA3_OF**: enabling PGA3 would drive B2 → incorrect current readings. Changed to **PGA_disable** (on-chip PGA not used on this board).
  - [x] **CMPSS overcurrent (fixed + hardware-verified)**: the inherited instance 5/3/1 + value=4 was wrong
    (value=4 selects `PGAx_OUT`, the on-chip PGA output, which is disabled on this board → a floating node that
    tripped spuriously whenever PWM was enabled). Authoritative mapping from F280049C datasheet **Table 6-13**
    ("Analog Pins and Internal Connections", direct-connect pins, on-chip PGA disabled):
    | Phase | Pin | CMPSS | HP/LP index (mux value) |
    |----|:--:|:--:|:--:|
    | Ia | B2 | CMPSS3 | 0 |
    | Ib | C0 | CMPSS1 | 1 |
    | Ic | A9 | CMPSS6 | 3 |
    Applied in `HAL_setupCMPSSs` (J1_J2 branch): `cmpssHandle[]`={CMPSS3,CMPSS1,CMPSS6};
    `ASysCtl_selectCMP{H,L}PMux`→ SELECT_3=0 / SELECT_1=1 / SELECT_6=3; ePWM X-BAR TRIP9→MUX10(CMPSS6),
    TRIP7→MUX00(CMPSS1), TRIP8→MUX04(CMPSS3). **Verified on hardware** (`tools/flash/diag_oc_latch.js`
    re-cal test): zero motor current + PWM on no longer asserts `moduleOverCurrent`. Remaining: the real
    over-current TRIP THRESHOLD (DAC high/low) still needs a load test before relying on it for protection.
  - Note: the WithoutOffsets read function is dead code for this lab (is01/is02 only call WithOffsets), left as-is.
    J5/J6 EPWM1/2/4 pins are still configured but the boosterpack is not connected; harmless.
- [x] **Phase 3 DRV8305 SPI Driver** (drivers/source/drv8305.c + include/drv8305.h)
  - [x] Register map/enums ported from MotorWare to driverlib style (16-bit frame, address enum, CSA gain)
  - [x] `DRV8305_writeSpi/readSpi` (driverlib `SPI_*BlockingFIFO`) + `DRV8305_configure`
        (clear faults + CSA gain 10 V/V → configured for 47.14 A; gate drive/dead-band left at EVM defaults)
  - [x] Integrated into HAL functions: `HAL_setupGate`→`HAL_setupSPIA` (SPI was previously uncalled; now connected);
        `HAL_enableDRV`→ EN_GATE wake + ~1 ms delay + `DRV8305_configure`
  - [x] `build.sh` adds `drv8305.c` + `--define=DRV8305_SPI` based on BOARD; both boards compile cleanly
  - ⚠️ **Runtime entry point not yet connected (pending bring-up)**: stock SDK lab `is01_intro_hal.c` only calls
        `HAL_enableDRV()` inside `#ifdef DRV8320_SPI`; we define `DRV8305_SPI` → the default is01 path **does not execute**
        `DRV8305_configure()` (EN stays low; safe until hardware is ready). In other words: `HAL_setupSPIA` runs with HAL initialization,
        but the DRV8305 register configuration/enable has no runtime entry point yet. During bring-up, add a DRV8305-specific lab hook or local
        lab wrapper that explicitly calls `HAL_enableDRV()` after confirming the timing sequence.
  - ⚠️ **Pending hardware confirmation**: SPI communication (read status/ID), 6-PWM mode (Control 7), VDS overcurrent threshold/gate drive current/dead-band to be tuned per motor from measurement; SPI polarity set per MotorWare (POL0PHA0), verify with oscilloscope on board.
- [ ] **Phase 4 Power-on Bring-up**: is02 calibration → is03 self-test (low voltage / low current first) → is05 motor identification → is06/07 loop tuning

## Information Needed (before Phase 4)
- Motor model and parameters: pole pairs, rated current/voltage, Rs/Ls (can be identified by is05)
- DC supply voltage (recommend starting at 12–24 V low voltage) and current limit
- Connector site confirmation (default Site 1 / J1-J4; if connected to J5-J8, pin mapping changes)
