# LaunchXL-F280049C + BOOSTXL-DRV8305EVM — AM-4116 bring-up bench notes

Bench: LaunchXL-F280049C docked to BOOSTXL-DRV8305EVM, AM-4116 outrunner (12N14P, 7 pole
pairs, KV450), clamped on the stator base, no prop, 24 V supply (≈5 A current limit).
All runs driven over DSS (`tools/flash/*.js`) since this CCS/DSS setup cannot call target
functions; EN_GATE is asserted at register level (GPBSET/GPBCLEAR bit7 = GPIO39).

## Hard conclusions (validated — the hardware chain is GOOD)

1. **DRV8305 config is correct (live SPI readback).** Raw-SPI read (`diag_drv8305_spi.js`,
   reads the SPIA peripheral directly, independent of firmware): `CTRL_7=0x216` →
   **PWM_MODE = 6-PWM**, **DEAD_TIME = 60 ns**, TBLANK 2 µs, TVDS 4 µs; `CTRL_A=0x000` →
   **CSA gain 10 V/V, DC_CAL off (normal measurement)**; `CTRL_5/6=0x344` default gate drive;
   `CTRL_C=0x2C8` (VDS_LEVEL 0x19); STATUS 1-4 = 0 (no faults). The "confirm 6-PWM" TODO is
   resolved: power-on default *is* 6-PWM, so even un-configured the mode is correct.
   *Note:* the DRV8305 dead time is a handshake (gate-state) mechanism, so the EPWM dead-band
   (DBRED/DBFED = 1 = 10 ns) does NOT set it — sweeping EPWM dead-band 10→800 ns changed
   nothing. The real dead-time lever is the gate-drive current (CTRL_5/6 IDRIVE/TDRIVE).

2. **Phase-order mapping is correct: `PWM_PHASE_ORDER=4` (CAB).** Confirmed by discrete
   forced-angle steps that the rotor follows the commanded direction.

3. **Current-sense scale is correct.** Shunt silkscreen = **R007 = 7 mΩ**; with CSA gain 10 V/V
   that is the firmware's assumed ±23.57 A full scale. `run_curcal.js` (fixed angle, Id steps)
   reads a clean `+Id / −Id/2 / −Id/2` split that tracks the command (0.97/2.01/3.02 A for
   1/2/3 A cmd). Firmware amps == real amps.

4. **Voltage feedback is correct.** `VdcBus = 24.1 V` (matches supply), voltage offsets ≈0.498
   (mid), `adcData.V_V` tracks the PWM node (≈12 V at 50% duty, ≈0.05 V under low drive).

5. **The "won't rotate" problem was UNDER-DRIVE, not hardware.** 1–1.5 A phase current only
   jitters/slips; **3 A drags the rotor smoothly and ramps open-loop to ~20 Hz elec (171 rpm)**.
   The legacy esc_drv8300 "0.4 A" figure is **DC input current**, not phase current — it misled
   us into commanding far too little phase current. Do NOT size startup current from DC input.

6. **Tooling: halt-polling chops the drive.** DSS scripts that halt every 0.5 s to read freeze
   the ISR → the field holds a DC vector → segmented motion + pulsing PSU current. EPWM is
   `EMULATION_FREE_RUN` so the carrier keeps running, but the control loop is frozen. **All
   "does it spin" judgements must use a single uninterrupted run (no mid-run halts).**

7. **is05 startup path was made equivalent to the working is04 drag.** With force-angle held
   through RampUp (`USER_FORCE_ANGLE_FREQ` 1→15 Hz, bench) and the RampUp d-axis current forced
   to 3 A, the captured angle freq = 20 Hz and Id/Iq = 3/0 A — identical to the is04 forced
   drag. So the is05 vs is04 difference is NOT the startup path anymore.

## Open cutoff: low-speed sensorless (FAST) lock

The remaining blocker is **sensorless FAST estimation at startup**, not hardware/mapping:

- At **5 Hz** the open-loop drag holds sync (Iq_in ≈ 0, phase current ≈ command, no slip), but
  FAST's speed estimate (`estOutputData.fm_lp_rps`) is **noise** (jumps 40–114 Hz, not 5 Hz).
  Back-EMF at 5 Hz ≈ flux·f = 0.012·5 = **0.06 V**, buried under the Rs drop + dead-time voltage
  (Vs ≈ 0.7 V) → poor SNR → FAST cannot extract it.
- At **10 Hz** mostly synced (Iq ≈ −1 A), FAST over-estimates ~1.5× (≈15 Hz).
- At **20 Hz** back-EMF is observable (0.24 V) **but the open-loop drag slips** (Iq → −6…−10 A,
  phase current ±8–10 A, FAST estimate 300–500 Hz garbage). Do not keep pushing 20 Hz open loop.

**There is no open-loop sweet spot** for this very-low-flux / very-low-Rs 4116 on this bench:
low enough to hold sync ⇒ back-EMF too small to observe; high enough to observe ⇒ drag slips.

## Next round (separate): I-f startup

Goal: **current-frequency (I-f) open-loop startup** — hold a commanded current vector and ramp
its frequency open-loop (closed *current* loop, open *position* loop) up into the
observable-back-EMF region, then **smoothly hand off to FAST** — instead of the standard is05
RampUp or a fixed forced-angle. Write the handoff criterion (speed/back-EMF threshold) explicitly.
This is the right path to spin the 4116 sensorless on LaunchXL and transfers to esc6288.

## Bench params used (NOT product defaults — see cleanup)
- `PWM_PHASE_ORDER=4` (correct, keep).
- `USER_MOTOR_RES_EST_CURRENT_A`: bench used 2 A (gentler than 4 A into 40 mΩ); Rs ID never
  completed, so not validated.
- `USER_FORCE_ANGLE_FREQ_Hz`: bench used 15 Hz (drag through RampUp). Stock is 1 Hz.
- bench-only firmware instrumentation (hal.c counters, drv8305.c regdump/sentinel, SDK is05.c
  RampUp override + angle capture) — debug only, reverted after this round.

## Reusable bench tools (tools/flash/, currently untracked)
- `diag_drv8305_spi.js` — raw-SPI DRV8305 register dump (no firmware support needed).
- `run_curcal.js` / `run_socal.js` — fixed-angle current-scale calibration.
- `run_draghi.js` / `run_5hz.js` — free-run forced-angle drag + FAST-estimate readout.
- `run_is05_rampup.js` — is05 RampUp-only controlled validation.
