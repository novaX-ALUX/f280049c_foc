# LaunchXL-F280049C + BOOSTXL-DRV8305EVM — AM-4116 bring-up bench notes

> **Status: CLOSED for LaunchXL+BOOSTXL + AM-4116 standard BEMF startup.**
> Do not keep retrying stock is05 / standard BEMF handoff on this bench — it is judged
> (see Stage 1 below): the open-loop drag pulls out ~15 Hz, short of the ~20 Hz FAST-observable
> region, and the ±23.57 A sense/protection margin is too narrow to brute-force it. This is a
> bench-transition limit, NOT a motor/FAST limit (legacy esc_drv8300 identified this motor and spun
> it to 8.16 krpm). Next work should move to **esc6288 bring-up** (MT6701 absolute angle bypasses
> low-speed sensorless) or a separate **HFI / low-speed-observer** project (new algorithm, not is05
> tuning, with explicit goals + exit criteria).

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
   jitters/slips; **3 A drags the rotor smoothly at low speed** (open-loop slip onset ≈ 15 Hz elec —
   quantified in the Stage 1 I-f section below; an earlier "~20 Hz / 171 rpm" note used the wrong
   `fm×7` conversion and an over-optimistic single run). The legacy esc_drv8300 "0.4 A" figure is
   **DC input current**, not phase current — it misled us into commanding far too little phase
   current. Do NOT size startup current from DC input.

6. **Tooling: halt-polling chops the drive.** DSS scripts that halt every 0.5 s to read freeze
   the ISR → the field holds a DC vector → segmented motion + pulsing PSU current. EPWM is
   `EMULATION_FREE_RUN` so the carrier keeps running, but the control loop is frozen. **All
   "does it spin" judgements must use a single uninterrupted run (no mid-run halts).**

7. **is05 startup path was made equivalent to the working is04 drag.** With force-angle held
   through RampUp (`USER_FORCE_ANGLE_FREQ` 1→15 Hz, bench) and the RampUp d-axis current forced
   to 3 A, the captured angle freq = 20 Hz and Id/Iq = 3/0 A — identical to the is04 forced
   drag. So the is05 vs is04 difference is NOT the startup path anymore.

## Unit correction (supersedes ALL earlier FAST-estimate Hz figures in this file)

Earlier drag notes converted `fm_lp_rps` to electrical Hz with `fm×7/(2π)` — **wrong, inflated 7×**.
The lab control path compares `estOutputData.fm_lp_rps × (1/2π)` **directly** to `speedRef_Hz`
(is05_motor_id.c:921, is07_speed_control.c:913); labs.h:962 divides `EST_getFm_lp_Hz()` by pole
pairs to get mechanical krpm. So the correct conversion is **`fm_lp_Hz(elec) = fm_lp_rps/(2π)`**
(directly comparable to the commanded electrical freq — do NOT multiply by pole pairs `p`); `p` is
used only for `mech_rpm = elec_Hz × 60 / p`. Any earlier "FAST estimate = NN Hz" in this file should
be read as ÷7 of the printed value.

## Stage 1 I-f characterization — RESULT: NO BEMF handoff window on LaunchXL+4116

Goal of the I-f round was open-loop drag into the observable-back-EMF region, then a smooth handoff
to FAST. Stage 1 (characterize first, gate before any handoff code) **fails the gate**: there is no
frequency band where the open-loop drag holds AND FAST tracks. Scripts: `run_if_char.js` (graded
plateaus, K complete short runs), `run_if_rec.js` + `run_if_rampB.js` (in-ISR recorder time-series).
All judgements use `fm_lp_Hz = fm_lp_rps/(2π)` and free-run only (no mid-run halts).

1. **Low-speed scan 5–13 Hz, 3 A, K=4–5 (run_if_char):** 0 sustained convergence; `fm_lp` scatters
   wildly run-to-run (incl. negative / wrong sign). Single halt-reads are unreliable here — K-sampling
   was essential to see it.
2. **Recorder @10 Hz time-series (run_if_rec):** `Id_in` holds ~3 A and `Iq_in ≈ 0` (drag IS synced;
   rotor goes forward — matches the visual), but `fm_lp` is **pure noise −3…+29 Hz, never settles**
   (plateau tail 20/103 within ±20%, mean ≈14 Hz ≈ 40 % high). So the failure is **FAST observability,
   not drag slip** — the two were disambiguated by the recorder.
3. **PWM-down test 40→20 kHz (hypothesis: a dead-time-dominated Vs floor):** **immediate hardware
   over-current** (`faultUse` bit4 = `MODULE_OVER_CURRENT_BITS` 0x10, labs.h:116) — the 34 µH motor's
   ripple ∝ 1/f_pwm doubles and the doubled control period doubles the current-loop `Ki`; 10 kHz is
   worse. Arithmetic also rules dead-time out as the main term: 60 ns × 40 kHz × 24 V ≈ **0.058 V** ≪
   the ~0.7 V Vs floor. So lowering PWM is not viable AND the floor is not the dead-time duty term.
4. **Continuous ramp 0→35 Hz @ 2 Hz/s (experiment B, run_if_rampB):** drag holds to ~13–14 Hz, then
   **slips hard at ~15 Hz** (Iq −7.6 A, ipk 8 A); 15–35 Hz is full slip (Iq ±18 A, **ipk to 23 A ≈
   sense full-scale**, `fm_lp` 90–130 Hz garbage). FAST never sustains a lock — longest contiguous
   "lock" was 0.22 s (a coincidence, far below the 1 s sustained threshold).

**Conclusion:** the band where the open-loop drag holds (≤ ~14 Hz) is exactly where FAST is noise; the
band where back-EMF would be observable (~25–30 Hz+) is past where the drag already slipped (~15 Hz).
**No overlap ⇒ no standard back-EMF I-f→FAST handoff window for the 4116 on LaunchXL+DRV8305.** Causes:
(a) back-EMF too small at low speed (0.06 V @5 Hz, 0.12 V @10 Hz) vs the ~0.7 V terminal/Rs/measurement
floor; (b) **the bottleneck is NOT the 60 ns dead-time duty term**, so lowering PWM (over-currents),
IDRIVE, or a GaN stage are **not the preferred fix for the proven cause** — GaN may still improve
switching/noise quality, but it is not the shortest path to the main cause shown here; (c) can't drag
higher to reach observability — slip onset ~15 Hz at 3 A, and more current isn't available (sense
saturates at ±23.57 A, plus heating).

> ⚠️ **SAFETY (read before any further open-loop work on LaunchXL).** Open-loop slip on this very-low-
> flux motor produces large circulating currents — experiment B reached **ipk ≈ 23 A (≈ full-scale)
> WITHOUT latching an over-current fault** (the CMPSS threshold sits high). **Do NOT bare-push past the
> slip point on LaunchXL.** Any future high-speed open-loop run must add a stricter software abort AND
> rely on hardware current limit / scope confirmation; here the only real-time protection was the bench
> PSU current limit (~5 A).

## Where this leaves I-f / next directions (separate round)

Standard BEMF handoff is out on this bench. Later-round options (not this round): (a) bridge the low-
speed gap with a non-FAST observer or high-frequency signal injection (HFI) before handing to FAST —
non-trivial for a 34 µH / 0.012 V·Hz⁻¹ motor; (b) evaluate `is09_flying_start` for catch/observer
behaviour; (c) defer low-speed sensorless startup to **esc6288** (different power stage, flux margin,
possibly an encoder). **Do NOT pursue PWM-down / IDRIVE / GaN as a low-speed-observability fix.**

## Bench params used (NOT product defaults)
- `PWM_PHASE_ORDER=4` (correct, keep); `USER_PWM_FREQ_kHz=40` (stock — 20 kHz over-currents this motor).
- `USER_MOTOR_RES_EST_CURRENT_A`: bench used 2 A (gentler than 4 A into 40 mΩ); Rs ID never completed.
- `USER_FORCE_ANGLE_FREQ_Hz`: an earlier round used 15 Hz; stock is 1 Hz. (Not used by the is04 I-f path.)
- All bench-only firmware instrumentation has been reverted; the SDK tree is back to stock and
  `LAB=all` is 12/12 on launchxl/am_4116.

## Reusable bench tools (tools/flash/) + recorder fork
- `run_if_char.js` — graded I-f plateaus, K complete short runs. **Runs on a clean is04 checkout.**
- `run_if_rec.js` / `run_if_rampB.js` — in-ISR recorder time-series readout. **Require the is04
  recorder fork:** apply `tools/flash/is04_if_recorder.patch` to the SDK is04 first, then rebuild.
  The SDK tree is gitignored and the fork is intentionally NOT committed there, so a clean checkout
  will NOT run these two until the patch is applied (`patch -p1 <…patch`; `patch -R -p1` reverts).
- (existing) `diag_drv8305_spi.js`, `run_curcal.js`/`run_socal.js`, `run_draghi.js`/`run_5hz.js`,
  `run_is05_rampup.js`. NOTE: `run_draghi.js`/`run_5hz.js` print FAST Hz with the old `fm×7/(2π)` —
  read those ÷7 (see Unit correction).
