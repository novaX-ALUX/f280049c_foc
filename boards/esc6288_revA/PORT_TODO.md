# esc6288_revA — bring-up checklist

Status: **ported from the final schematic + netlist; all build gates green** (SRC_CHECK,
CAN_CHECK, PRODUCT_CHECK, PRODUCT, `LAB=all` 12/12, host tests 11/11). **First prototype on the
bench 2026-06-30** — bring-up stages 1/2/3/4/5/6 passing, incl. **first spin** (sustained ~4 krpm,
sensorless FOC, zero faults). Encoder + CAN-to-flight-controller + RGB all verified; one board defect
found + bodged (see the 2026-06-30 log below). The hardware OC path was confirmed end-to-end; OV
injection and motor-coupled tuning remain — the items below are what to verify/tune. Anything marked
**[BENCH]** needs the hardware (scope/meter) to confirm.

Hardware: F280049CPMSR (64-pin PM), gate driver **U12 = JSM6288T** (6 independent inputs,
**no EN, no nFAULT**), 3× INA181A1 (gain 20) over 0.5 mΩ shunts (1.65 V mid-rail,
bidirectional), 30.1k+30.1k+2k voltage dividers, **10 MHz** resonator. Full pin map is the
header comment in `drivers/include/board.h`.

## Schematic-verified (implemented)
- **Clock**: 10 MHz resonator → `SYSCTL_IMULT(20)` → 100 MHz; both clock asserts use a
  board-local 10 MHz osc const (`hal.c`). *If SYSCLK were wrong everything downstream is 2× off.*
- **PWM**: phase A=EPWM1 (GPIO0/1), B=EPWM2 (GPIO2/3), C=EPWM3 (GPIO4/5); `PWM_PHASE_ORDER`
  default 0=ABC. Dead-band 20 cnt (~200 ns) — extra margin ON TOP of the JSM6288T's own
  built-in ~200 ns anti-shoot-through dead time + interlock (datasheet); see the bench item.
- **Current**: `USER_ADC_FULL_SCALE_CURRENT_A = 330` (±165 A), offsets −165 A; PGAs disabled
  (external INA path). IA=ADCINB15, IB=ADCINA1, IC=ADCINC2 (distinct cores).
- **Voltage**: `USER_ADC_FULL_SCALE_VOLTAGE_V = 102.63`; Udc=ADCINA6, UA/UB/UC=B6/B3/C6.
- **DAC outputs disabled** in `HAL_setupDACs` — DACA_OUT/DACB_OUT share the IA/IB sense pads
  and would corrupt current sensing if enabled.
- **Protection**: software OC on all 3 phases (1 ms product layer + a 20 kHz ISR backstop,
  `ESC6288_ISR_OC_A = 60 A`) + **CMPSS3** hardware OC on phase C + **CMPSS5** DC-bus OV
  (~56 V). No digital nFAULT trip (there is no fault pin).
- **Safe-off without an enable pin**: EPWM trip-zone (OST). `HAL_enableDRV` is a no-op;
  `HAL_setupFaults` leaves outputs OST-forced; **offset-cal does NOT clear the trip** (gates
  stay off during cal, ADC still samples); only the arm path (`HAL_enablePWM`) un-trips;
  `HAL_setPWMBrake` is stubbed (would defeat safe-off). Gate macros kept as `BOARD_GPIO_NONE`
  sentinels; `gate_driver.c` hard-skips the sentinel.
- **CAN**: CANA, GPIO37 TX / GPIO35 RX, 1 Mbit (`can_bridge.c`, mirrors launchxl).
- **Aux drivers** (board-side, compiled + init-hooked from product_init): RC-PWM throttle on
  eCAP1 (`rc_pwm.c`), MT6701 SSI on SPIA (`mt6701_ssi.c`), WS2812 RGB on GPIO12 (`rgb_led.c`).

## Bench bring-up — do in order, gate driver kept off (forced TZ) until each step passes

### Bench log — 2026-06-30 (first prototype; XDS110 over a CC1352 LaunchPad, USB + 12 V, no motor)
- **1 rails/clock — PASS.** SYSCLK 100 MHz confirmed (tick/wall ratio 1.005); OST 1/1/1 at boot, not armed.
- **2 idle/OST — PASS (observe).** OST 1/1/1 held through startup + offset-cal (cal does **not** un-trip); never self-armed.
- **3 ADC front-end — PASS, after a hardware fix.** Initially every channel read rail/garbage; root cause was the
  **VREFLO-not-grounded** board defect (see Rev-B note). A bodge wire VREFLO(pin 17)→AGND restored VREFHI to 3.3 V and
  s3 then read currents ~2040 (mid-rail) and `VdcBus_V=12.00`. **NTC still reads open (raw=0)** — separate, low-priority.
- **4 first spin — PASS.** `tools/flash/esc6288_revA/run_is06_esc6288.js` (is06 torque lab) at 12 V / 5 A, no
  prop. iq=0 scope mode first: gates un-tripped (OST 0/0/0), half-bridges switched cleanly, zero current, no
  fault, dead-band clean on the scope. Then iq=1.0 A with a continuous flick window: a hand-flick bootstrapped
  the sensorless FAST estimator and the rotor **held ~4 krpm (≈466 Hz elec) for 12 s, zero faults during the
  spin**. Validates the full chain: power stage → current sense (on the VREFLO bodge) → current loop → FAST →
  dead-band. NOTE the sensorless cold-start needs the flick (Iq alone ≤0.5 A did not break standstill), and a
  debugger gate-cut of the spinning motor latches `moduleOverCurrent` (faultUse=16) on the transient — BENIGN
  (the hardware OC path firing end-to-end, safe-off achieved). Motor-coupled `dir`/`zero_offset` tuning is next.
- **5 protection — PASS (comparator-level).** CMPSS3/CMPSS5 latch baseline clean; `force=tz` (software TZFRC OST)
  confirmed; `s5b_route.js` **PASS** — both high comparators trip when their DAC is lowered below the resting signal
  (bus-OV CMPSS5 @ ADCINA6 ~479; phase-C OC CMPSS3 @ ADCINC2 ~2034), DAC restored after. The CMPSS→X-BAR→OST
  end-to-end trip (`inject=oc/ov`, real OC/OV) is still **PENDING** — defer to first-spin.
- **6 peripherals — PARTIAL.** **Encoder MT6701 — PASS:** SPIA + HT0104 path confirmed (`g_enc.valid=1`,
  clean SSI decode, `position_rev` tracks the magnet — see the MT6701 item). **CAN — PASS (against an ArduPilot/AF-H7E flight
  controller).** Bit timing confirmed 1.000 Mbit (CAN_BTR); with the FC on the bus the esc6288 completed **DNA
  (`g_dn.node_id=124`)**, TX drains (`s_tx` head==tail, `in_flight=0`, `TEC=0`, no `LEC` error), and RX advances
  (receiving FC frames). NOTE: a bare USB-CAN adapter that is merely enumerated does **not** ACK (it gave
  `LEC=ACK error` / `TEC=128` / error-passive until a real ACKing node was attached) — use the FC (or bring the
  adapter on-bus in normal mode) when bench-testing CAN. **RGB — PASS:** GPIO12→U6→WS2812
  path verified via the `RGB_SELFTEST` boot sweep (R→G→B→white→off all correct at 6/6/1/12 timing).
  **RC-PWM:** no signal connected.
- **Bench caveat:** the hand-wired LaunchPad-XDS110 JTAG link was chronically marginal (intermittent `-2131`); see
  the JTAG note in `tools/flash/esc6288_revA/README.md`.
- **SDK lab walk-through (is01→is07), 2026-06-30:** is01 HAL (`check_is01_esc6288.js`) PASS — parked safe, raw ADC
  IA/IB/IC≈2050, Udc≈479. is02 offset cal (`cal_is02_esc6288.js`) PASS — current offsets matched to ~0.8 A across
  phases, Vbias≈0.49 V. **is03 SKIPPED** (V/f → ~100 A on the 40 mΩ 4116; see the BENCH note). is04 signal-chain
  (`run_is04_esc6288.js`) — current loop + Clarke/Park confirmed (Iq tracks, Id≈0, no fault); open-loop forced spin
  did not break standstill (see self-start note). **is05 FAST ID COMPLETED** (`run_is05_esc6288.js`, see below).
  is06 PASS (first spin, flick). is07 (`run_is07_esc6288.js`) speed-loop self-start does NOT cold-start — see below.

### is05 FAST motor ID — COMPLETE (2026-07-01)
Full FAST identification of the 4116 ran to `flagMotorIdentified=1` on esc6288 (5 completed runs, faultUse=0).
The lab's own ID sequence (RoverL→Rs→RampUp→Flux→Ls) needs **continuous execution** — halt-sampling over DSS
desyncs the fragile Ls phase. `run_is05_esc6288.js` runs the ID uninterrupted (~160 s, EST flux+Ls wait tables
need ~100 s) and halts once at the end. Repeatable results (median of 4): **Rs≈0.0164 Ω, Ls≈23.5 µH,
flux≈0.0128 V/Hz**.
- **flux 0.0128 ≈ profile 0.012** → KV450 confirmed on real hardware.
- **Rs mystery resolved (Codex, verdict B):** the legacy profile `0.0403` was the **LINE-LINE** value (the MotorWare
  pu→Ω recipe `Rs_pu/2^30 × Vfs/Ifs × 2^(30−qFmt)` = 0.0401 = bench line-line 42–43 mΩ), NOT the phase-to-neutral
  the FOC model wants. Both MotorWare and SDK6 FAST use phase-to-neutral; SDK6's direct getter is correct.
  `motors/am_4116_kv450.h` Rs corrected `0.0403 → 0.0213` (bench line-line/2).
- **Ls corrected `33.6 → 23.5 µH`** (esc6288 ID median; old 33.6 was the same suspect recipe).
- **RESOLVED (2026-07-01) — current sense over-reads 1.30×.** The current-scale ambiguity is settled: re-ran the
  is05 Rs-ID at **8 A** inject and Rs stayed **0.0164 Ω** (identical to the 4 A runs) — a low-signal artifact would
  have risen toward the meter value, so it is a fixed **gain error**: `k = R_meter(0.0213) / Rs_fw(0.0164) = 1.30`.
  Voltage scale is confirmed correct (flux matches), so the error is purely the current path. **Corrected
  `USER_ADC_FULL_SCALE_CURRENT_A` 330 → 254** (= 330/1.30) and the offset seeds -165 → -127. A fresh is05
  **validation** run then read **Rs=0.0223 Ω (≈ meter 0.0213), Ls=29.96 µH, flux=0.0126** — self-consistent, so
  profile Ls updated 23.5 → 30.0 µH (Rs kept at meter 0.0213). All product current thresholds are in amps and derived
  via FS, so they now trip at **TRUE amps** (the old 330 scale made them ~1.3× over-conservative by accident). HW root
  cause (real shunt/gain vs BOM 0.5 mΩ × 20 → measured ~0.013 V/A) is a **rev-B check item**. Before 15" prop:
  drop `oc_set_A` from 30 A to ~10–15 A (Codex bench limit).

### Sensorless cold-start (self-start from standstill) — [SOLVED 2026-07-01 via open-loop I/f]
The 4116 (≈0.012 V/Hz surface-PM outrunner) does **not** self-start on the FAST angle at standstill: is06 torque
needs a hand-flick; is07 speed-loop armed from standstill oscillates (FAST cannot lock with no observable BEMF).
Root cause is NOT `USER_FORCE_ANGLE_FREQ_Hz` (both this and legacy esc_drv8300 use 1.0 Hz) — the gap was a missing
deterministic rotor **align + open-loop I/f ramp** before FAST takes over.

**Solved:** `run_iftest_esc6288.js` proved the open-loop I/f rig on is04 (5/5 param sets: 4116 spins up from
standstill, FAST locks with 2–7° angle error, accel 10–40 Hz/s / id 2–3 A / handoff 30–50 Hz — wide no-load
envelope). The startup is now a **state machine in the product firmware** (`product/product_main.c`, `g_su` +
`startup_step()`): `SU_IDLE → SU_ALIGN (hold Id at angle 0) → SU_RAMP (open-loop angle ramps at accel, hold Id) →
SU_RUN (hand off to FAST angle + throttle Iq when freq≥handoff_Hz AND |FAST angle − open-loop angle|<thresh)`.
Live-tunable via DSS (`g_su.*`); `g_su.enable=0` = exact legacy behavior. **KEY FIX:** during ALIGN/RAMP the ISR
re-PARKs the measured `Iab` onto the **open-loop angle** (not `EST_getIdq_A`'s FAST angle, which is force-held near
0 while the open-loop angle ramps) — without this the current-PI feedback frame is mismatched with the applied
vector and the loop diverges into a >60 A spike (software ISR OC). Validated end-to-end
(`run_selfstart_esc6288.js`, product built `--define=ESC6288_BENCH_THROTTLE`): from standstill → SU_RUN via handoff
at 35 Hz, faultUse=0, spun 4292 rpm closed-loop under FAST, no flick. (Note: product FOC had never been armed on
esc6288 before this — the arm/align path itself is now proven too.)

**Hardening (Codex review 2026-07-01) — DONE + bench-validated:**
- ✅ `SU_FAULT` real safe-off — forces OST (`HAL_disablePWM`) + latches `moduleOverCurrent` so the main loop
  disarms (validated `run_slipguard_esc6288.js`: forced slip → OST + faultUse=16 + flagRun=0, no drive).
- ✅ Ramp slip/stall guard — past `slip_check_Hz`, require FAST speed ≥ `slip_frac`×open-loop freq and bounded angle
  error, else `SU_FAULT` (don't wait for timeout).
- ✅ Handoff dwell + blend — coherence (freq, angle, +speed sign) held `dwell_s` then `SU_BLEND` ramps Id→0 /
  Iq→throttle over `blend_s` (validated: handoff current went from a one-tick bump to clean Id≈0/Iq≈throttle).
- ✅ FAST fed the open-loop freq (not 0 Hz) during ALIGN/RAMP so its observer converges.
- ✅ Re-arm fault latch — `apply_setpoint` arms only when `faultUse.all==0` (a startup slip/OC latch stays sticky
  instead of re-pulsing the motor; validated: latched, no retry). Latch clears on power-cycle/reload.
- ✅ C99 dialect — `build.sh` now passes `--c99` (product + `LAB=all` compile clean); `bool` no longer relies on an
  unproven CGT extension.
- Atomic setpoint (Codex #1) — left; benign (apply_setpoint writes IdqSet before the run flag, startup ignores IdqSet).

**PROP-TEST GATES — must resolve/prove before the first 15" prop bring-up (Codex 2026-07-01):**
1. **PI integrator carryover at `SU_RAMP→SU_BLEND`** (top must-fix) — the Id/Iq current-PI integrators are not
   reset/preloaded across the open-loop→FAST frame switch; coherence keeps it small (validated CLEAN no-load) but it
   can spike on the first handoff under load. Resolve (flush/preload at BLEND entry) OR bench-prove under prop.
2. **Profile the ISR cycle margin** with the two `cosf/sinf` pairs (ALIGN/RAMP) at 20 kHz in CCS before prop.
3. **Set conservative prop params via DSS before arming** — defaults are no-load (`accel=25 Hz/s`, `err=0.35`); use
   `accel≈2 Hz/s`, `handoff≈35–45 Hz`, `err≈0.20`, `timeout≈20 s`, and a low `oc_set_A` (10–15 A).
4. Deferred: auto-recovery on throttle-drop (needs a separate startup-fault bit; must not clear a real ISR/HW OC).

1. **Rails + clock**: power 3V3/5V/12V only (no motor). Confirm **SYSCLK = 100 MHz** (toggle a
   GPIO at a known divide, scope it) — proves `IMULT(20)`. If it reads 50 MHz the resonator
   assumption is wrong.
2. **Idle outputs**: verify EPWM1/2/3 outputs idle LOW and a forced OST holds them low through
   offset-cal; confirm `HAL_enablePWM` is the only thing that un-trips.
3. **ADC offsets**: PWM idle → all 3 currents read ~1.65 V (~count 2048); Udc tracks the bus
   through the 31.1× divider (apply a known bench voltage, check the reported V). *If all channels
   read rail/garbage with the bus on, suspect the ADC reference first — see the VREFLO-not-grounded
   rev-A defect under "Rev-B hardware note".*
4. **[BENCH] Dead-band / short-pulse** at low bus before spinning; watch the half-bridge Vds for
   shoot-through; then close the current loop; then spin.
5. **Protection tests**: trip phase-C CMPSS3 OC, DC-bus CMPSS5 OV (~56 V), and the ISR software
   OC on phases A/B; confirm each forces all outputs low and latches `moduleOverCurrent`.
6. **CAN / encoder / RC-PWM / RGB**: DroneCAN node at 1 Mbit; MT6701 angle reads; RC-PWM raw
   capture (`RC_PWM_read`) + `esc_pwm_decode` valid window verified (runtime output stays CAN-only
   while `ESC_ARB_EXPLICIT_CAN` ships — see the enable gate below); RGB status colors.

## Dual-throttle arbiter (CAN + RC-PWM) — enable gate **[BENCH, flight-safety]**
The `src/app/esc_arbiter` module fuses the DroneCAN throttle with the RC-PWM throttle. It
**ships inert**: `product_build_arb_cfg()` sets `policy = ESC_ARB_EXPLICIT_CAN`, so PWM is
ignored at runtime and behavior is **timing-identical to the old CAN-only path** (the arbiter
holds the same `seq` between fresh CAN frames, so `esc_control`'s 0.5 s watchdog still ages
from the last real frame). Nothing below is required for the shipped image.

**Do NOT flip the policy to `ESC_ARB_CAN_PRIMARY` (the CAN-primary / PWM hot-standby fallback)
until ALL of these pass on hardware:**
- **HARD PREREQUISITE — receiver failsafe must NOT be "hold-last".** The firmware cannot
  detect a held/replayed stale pulse from a single PWM wire: a previously-armed-and-tracking
  PWM that keeps replaying its last valid 1–2 ms pulse after the RC link drops will be selected
  as the fallback when CAN is lost. This is an **inherent single-wire limitation, not a firmware
  bug** — the PWM low-dwell arm gate + "must have tracked CAN" lockout defeat a *cold/never-armed*
  or *divergent* stuck line, but NOT a hold-last replay of a value that was valid pre-loss.
  Mitigation is **receiver-side**: configure the RC RX failsafe to drop signal (no pulse /
  out-of-range) or go idle-low on link loss, and **verify it on the bench** (plan Appendix
  step 3). Only then is the fallback meaningful.
- The full bench gate in the implementation plan:
  `docs/superpowers/plans/2026-06-29-dual-throttle-arbitration.md` (Appendix) — esp. step 3
  (RX failsafe), 5–6 (tracking + conflict lockout), 7 (CAN-loss fallback only after tracking),
  8 (stuck-PWM rejection), 9 (both-dead coast). Run current-limited, no prop first, prop last.
- Observe arbiter state via the debugger globals `g_arb_active` / `g_arb_status_bits`
  (`product_main.c`); `ESC_ST_SRC_PWM` is set in telemetry but **not yet serialized** in the
  `esc.Status` frame, so it is not visible over CAN.

## [BENCH] Confirm / tune
- **JSM6288T dead time** — datasheet (`docs/JSM6288T.pdf`) confirms a per-phase HIN/LIN 6-input
  driver that DOES have built-in anti-shoot-through protection: an interlock (HIN=LIN=H → both
  outputs off, truth table p.8 / functional block "DeadTime & Control Logic") AND a ~200 ns
  internal dead time (DT min 100 / typ 200 / max 300 ns, p.5; waveform fig. 6-3). ton/toff
  ~120-250 ns. The MCU dead-band (`HAL_PWM_DBRED_CNT/DBFED_CNT`, `hal.h`) is therefore EXTRA
  margin, not the sole protection; it was lowered from 500 ns to **200 ns (20 counts)** since at
  40 kHz (25 µs period) 500 ns wasted ~2% of the period and the chip already covers shoot-through.
  [BENCH] scope the half-bridge Vds / gate waveforms at full bus + load + hot, then probe the MCU
  dead-band down 20 → 10 (200 → 100 ns) — but do NOT ship 10 as the flight value without that
  verification: the chip's DT floor is only ~100 ns worst-case and real FET non-overlap also
  depends on propagation/rise/fall, Qg/Coss and layout ringing, so trust the scope, not the
  additive number (and do not assume 0 is safe).
- **FET NVMFS5C612NL Vds rating** — this is the hard OV ceiling. Confirm the 12S OV
  (`HAL_BUS_OV_CMPSS_DACH` ≈ 56 V, and product `vbus_ov_set = 54`) sits safely below it.
- **OC thresholds** — product `oc_set_A = 30` and `ESC6288_ISR_OC_A = 60` are conservative
  bench values; raise toward the motor/shunt rating after validation.
- **Bus nominal / UV** — currently 48 V nominal, UV 18 V (bench-friendly); raise UV for flight.
- **MT6701 SSI** (`mt6701_ssi.c`) — **full 24-bit read + CRC; SSI protocol and the esc6288 SPIA + HT0104
  path both bench-confirmed (2026-06-30).** Per datasheet `docs/MT6701CT-STD.PDF` sec 6.8 (and the encoder
  daughterboard schematic `ef_encoder_mt6701.{NET,pdf}`): **POL1PHA0**; the 24-bit frame (14-bit
  angle + 4-bit Mg status + 6-bit CRC, poly X^6+X+1) is clocked as two 16-bit words inside one
  **manual-CSN** window (GPIO11 re-muxed from SPISTE to GPIO). **Bench finding (2026-06, LaunchXL
  SPIB rig, CLK=GPIO22/DO=GPIO31/CSN=GPIO34):** the MT6701 emits **one leading bit** before the
  frame, so it sits at bits [30:7] of the 32 clocked bits — the original `(w0<<8)|(w1>>8)` alignment
  was one bit early and gave **CRC 0/6** on real captures; the corrected `mt6701_ssi_frame()`
  (`>>7`, `src/encoder/mt6701.c`, host-tested) gives **6/6** and clean full-revolution tracking.
  Frames are then decoded + CRC6-validated by the pure host-tested `mt6701_decode_ssi()`.
  `MT6701_SSI_read()` returns *usable* = CRC ok & field normal & no loss-of-track; the product
  bridges it through `mt6701_update()` → `foc_raw_feedback_t.enc_*` → `esc_feedback_t`.
  **Confirmed:** SSI mode (part responds in SSI, not I²C), POL1PHA0, manual-CSN timing, and the
  1-bit frame alignment all read valid live angle. **(1) esc6288 SPIA + HT0104 path — CONFIRMED on the
  bench 2026-06-30:** s6 read `g_enc.valid=1 stale=0 glitch=0`, and a breakpoint in `MT6701_SSI_read`
  showed a clean decode (`crc_ok=1 field_ok=1 track_ok=1`, e.g. angle 7137); `position_rev` tracked a
  hand-rotated magnet (0.13 → 0.62 rev). **Remaining [BENCH]:** (2) tune `dir` / `zero_offset_counts`
  to the motor (needs the motor coupled).
- **`auto_park` status** — **disabled by default in code, on every board** (`auto_park_enable=false`
  in `product_build_esc_cfg`, `product/product_main.c`; the esc6288 `#if` block only sets protection
  thresholds and does NOT re-enable it). The encoder is now **bench-confirmed on the esc6288 SPIA + HT0104
  path** (see MT6701 above), so that precondition is met; remaining gates before flipping it on for esc6288:
  (a) tune `dir` / `zero_offset_counts` and confirm the learned park reference, (b) a powered closed-loop
  **prop-park** bench run (needs the motor coupled). Keep it
  `false` until all three pass.
- **RGB WS2812 timing** (`rgb_led.c`) — **bench-tuned on the LaunchXL GPIO0 rig (2026-06); esc6288
  GPIO12 → SN74LVC1T45 path CONFIRMED on the bench 2026-06-30.** The original `WS_*_LOOPS` (18/14/7/20)
  were ~3x too long — even T0H put the '0' high pulse past the WS2812 0→1 threshold, so every bit read
  '1' (`0xFFFFFF` = stuck white, never off). Retuned to **6/6/1/12**, which renders R/G/B/white/off
  correctly. T0H margin is tight (T0H=3 still white, T0H=1 correct). **Bench check:** the product does
  not drive status colors yet (deferred), so the path was verified with a one-shot color sweep — build
  the product with `EXTRA_DEFINES="--define=RGB_SELFTEST"` (the gated block after `RGB_init()` in
  `product_main.c`) and watch RGB1 cycle R→G→B→white→off at boot; the 6/6/1/12 timing rendered all five
  correctly on the esc6288 GPIO12→U6 path. **Remaining:** wire RGB status colors into the product loop.
- **NTC → °C** — **implemented; bench-pending calibration.** The NCP18XH103 (ADCINC3 → ADCC SOC2)
  is converted by the pure, host-tested `ntc_counts_to_celsius()` (`src/common/ntc.c`, beta model)
  using the board divider in `board.h` (3V3 — NTC — [ADC] — R14 10k — GND, so NTC **high-side**).
  `product_main` feeds the result into `raw.temp_C` → `esc_feedback_t` → `esc_control`'s over-temp
  latch (`temp_ot_set=100 / clr=85`), which is now **live**. A dead sensor (open/short → near-rail
  count) reads back `BOARD_NTC_OPEN_TEMP_C` (150 °C) and trips the fault (fail-safe hot).
  **Remaining [BENCH]:** confirm R14 value + the VREFHI/3V3 rail, and trim `r25`/`beta` against the
  measured curve (apply known bench temperatures, compare reported °C).

## Rev-B hardware note (not fixable in firmware on this rev)
The phase-A and phase-B current-sense op-amp outputs land on **ADCIN A0/B15/C15 and A1**, which
have **no CMPSS comparator** on F28004x — so only phase C and the DC bus get hardware
cycle-by-cycle trips. This rev relies on software OC for A/B by design (user-approved). For a
rev B, route the IA/IB sense onto CMPSS-capable pads (e.g. B2/A4) to restore 3-phase hardware OC.

### VREFLO not grounded — ADC reference dead (rev-A bodge applied; rev-B must fix in layout)
**Bench finding (2026-06-30, first prototype):** every ADC channel read garbage — currents pinned to
rail (4095/0, unstable run-to-run), `Udc_raw=0` and `VdcBus_V=0` with 12 V on the bus, `NTC=0`.
Root cause is a **board defect, not firmware**: the MCU **VREFLO (pin 17, VREFLOA/B/C) is not tied to
analog ground** — it floats at ~1.32 V and drags VREFHI (pin 16) to the same ~1.32 V, so the ADC
reference span (VREFHI−VREFLO) collapses to ≈0 and every conversion is meaningless. VDDA measured a
healthy 3.3 V; the front-end (INA181 outputs at 1.65 V, the U9 LMV321 1.65 V buffer, the Udc divider)
all metered correct, and the firmware VREF setup is the stock SDK internal-3.3 V idiom (verbatim from
`solutions/boostxl_drv8320rs/f28004x/.../hal.c`), so nothing is configurable away. Confirmed by the
fix: a **bodge wire from pin 17 (VREFLO) to AGND** restored VREFHI to 3.3 V and s3 immediately read
currents ~2040 (mid-rail) and `VdcBus_V=12.00`. **Rev-A:** keep the bodge solid — all bring-up depends
on it. **Rev-B:** tie VREFLOA/B/C directly to AGND in the schematic/layout — per the netlist
(`esc6288_revA.NET`) C16/C17 are VREFHI↔VREFLO decoupling caps (nets NetC16-2 / NetC16-1) and the
VREFLO net has **no** DC tie to AGND/VSSA, which is the missing connection. Diagnosis lives in the s3
stage (`tools/flash/esc6288_revA`).

## Parameter persistence (storage format done; Flash erase/program deferred)
The non-volatile record (DroneCAN node-id, learned park-ref valid flag + target angle) has a
pure, host-tested storage format: `src/app/nvparam.{h,c}` (magic + version + CRC16 + range
validation), tested by `tools/test/test_nvparam.c` (roundtrip, bad magic/version/CRC,
node-id + NaN/Inf bounds, mock-flash power-cycle). `product_main` already folds DNA-allocated
ids and learned park refs into an in-RAM `nvparam_t` mirror at the existing store-request sites.
**Deferred [target]:** the actual driverlib Flash read at boot (`nvparam_decode` of the
read-back words) and write (`nvparam_encode` → erase/program) — search `TODO(target)` in
`product_main.c`. Until then the mirror stays at defaults, so node_id falls back to
`BUILD_NODE_ID` and the park ref boots unlearned (behaviour unchanged).

**Remote access (DroneCAN `param.GetSet`, service 11):** `src/comms/dronecan_param.{h,c}` exposes
the three nvparam fields (`node_id`, `park_ref_valid`, `park_ref_target_rev`) for GetSet read by
index/name and write by name. Every write funnels through `nvparam_update_*` (so #4 stays the only
validation authority: illegal node-id → DNA, NaN/Inf park ref → invalidated), marks
`dronecan_param_dirty()`, and — like a DNA-allocated id — is folded into the RAM mirror with the
real Flash write deferred. Pure codec + transport are golden-tested against pydronecan 1.0.27
(`tools/test/dronecan_param_golden.inc`, `test_dronecan_param`). **[BENCH] interop caveat:** the
GetSet *response* serializes union tags byte-aligned (matching pydronecan/yakut); confirm GetSet
read/write against the actual flight controller (ArduPilot) on the bench. A node-id set takes
effect only after a restart (RestartNode is intentionally not implemented).

## Build / verify
```bash
BOARD=esc6288_revA MOTOR=am_4116_kv450 SRC_CHECK=1     bash build.sh   # pure src/ modules
BOARD=esc6288_revA                   CAN_CHECK=1     bash build.sh   # CAN bridge + comms
BOARD=esc6288_revA MOTOR=am_4116_kv450 PRODUCT_CHECK=1 bash build.sh   # product main + foc_bridge
BOARD=esc6288_revA MOTOR=am_4116_kv450 PRODUCT=1       bash build.sh   # full product link
BOARD=esc6288_revA LAB=all bash build.sh                              # 12-lab regression
bash tools/test/run.sh                                               # host tests (incl. src/ purity)
```
