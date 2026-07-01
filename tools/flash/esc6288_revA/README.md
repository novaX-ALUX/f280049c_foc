# esc6288_revA bring-up scripts

DSS stage scripts + bench probes for esc6288_revA bring-up, run **in order**, mirroring the
6-step bench order in `boards/esc6288_revA/PORT_TODO.md`. They are an *executable runbook*: each
prints explicit pass / stop conditions so bring-up is "follow the steps", not "improvise".

> **2026-06-30:** s1/s2/s3/s5/s5b/s6 have run on the first prototype — all PASS (one board defect found
> + bodged, see the VREFLO note in `PORT_TODO.md`). s6: encoder MT6701 (SPIA + HT0104, POL1PHA0 SSI,
> manual CSN, live 14-bit decode), CAN-to-flight-controller (ArduPilot/AF-H7E, DNA/TX/RX verified), and
> the RGB boot sweep all confirmed. **s4 (first spin) done** — sustained ~4 krpm, zero faults
> (`run_is06_esc6288.js`). The hardware OC path confirmed end-to-end (gate-cut transient); OV injection
> and motor-coupled tuning remain.
>
> **2026-07-01:** is05 FAST motor ID completed on esc6288 (runner `run_is05_esc6288.js`; continuous
> execution — halt-sampling desyncs the Ls phase). Current-scale corrected to 254 A
> (`USER_ADC_FULL_SCALE_CURRENT_A` 330→254, offsets −165→−127); Rs 0.0213 Ω (phase-neutral), Ls
> 30.0 µH back-filled into `am_4116_kv450.h`. Product open-loop I/f self-start validated at 12 V /
> no prop (`run_selfstart_esc6288.js`): SU_ALIGN → SU_RAMP → SU_BLEND → SU_RUN handoff to FAST with
> **no hand-flick**, zero faults. Safe-off verified via `run_slipguard_esc6288.js`.

## Two hard safety rules (enforced in every script)

1. **Default mode is observe/check** — it never un-trips the EPWM, never drives PWM, never spins.
2. Any action that releases the trip-zone or could output pulses is behind an **explicit arg**,
   and the script **unconditionally returns to safe-off on exit** (forces OST, de-arms
   `flagRunIdentAndOnLine=0`, sets `flagEnableSys=0`), including on every error/abort path.

esc6288 has **no gate-enable pin** (JSM6288T): safe-off *is* the EPWM trip-zone one-shot (OST).
The target is the **product image** (`product.out`), which self-sets `flagEnableSys=true` and runs
the offset cal automatically — so the safe invariant is **`TZFLG.OST` set on EPWM1/2/3 AND not
armed (`flagRunIdentAndOnLine==0`)**, not `flagEnableSys==0`. Run every stage at **zero bus
voltage** first.

## Stages

| # | script | PORT_TODO step | what it checks | writes? |
|---|--------|----------------|----------------|---------|
| 1 | `s1_rails_clock.js` | rails + clock | HAL up; SYSCLK ~100 MHz via `g_now_ms`/wall-clock ratio (~1.0) | none |
| 2 | `s2_idle_ost.js` | idle / OST | OST set on all phases; `verify=offcal` (cal holds trip); `verify=untrip` (un-trip + re-arm) | only with arg |
| 3 | `s3_adc_offsets.js` | ADC offsets | zero-current ~2048; Udc; NTC→°C sane | none |
| 5 | `s5_protection.js` | protection | CMPSS3/CMPSS5 latches + OST baseline; `force=tz`; `inject=oc`/`inject=ov` (manual inject, read-back) | only with arg |
| 5b| `s5b_route.js` | protection (no injection) | lowers each CMPSS high-DAC below the resting signal so the idle level trips the comparator — proves CMPSS3/CMPSS5 sense their input + DAC without real OC/OV. Restores DAC + clears latch; gates stay OST-tripped | DAC (restored) |
| 6 | `s6_peripherals.js` | CAN/enc/RC-PWM/RGB | CAN FIFO, MT6701 `g_enc`, RC-PWM eCAP, RGB note | none |

Invoke (shared ccxml + the product `.out`):

```bash
dss.sh tools/flash/esc6288_revA/s1_rails_clock.js tools/flash/common/f280049c_xds110.ccxml <product.out>
dss.sh tools/flash/esc6288_revA/s2_idle_ost.js    tools/flash/common/f280049c_xds110.ccxml <product.out> [verify=offcal|verify=untrip]
dss.sh tools/flash/esc6288_revA/s3_adc_offsets.js tools/flash/common/f280049c_xds110.ccxml <product.out>
dss.sh tools/flash/esc6288_revA/s5_protection.js  tools/flash/common/f280049c_xds110.ccxml <product.out> [force=tz|inject=oc|inject=ov]
dss.sh tools/flash/esc6288_revA/s5b_route.js      tools/flash/common/f280049c_xds110.ccxml <product.out>
dss.sh tools/flash/esc6288_revA/s6_peripherals.js tools/flash/common/f280049c_xds110.ccxml <product.out>
```

## Bench note — XDS110 JTAG link (2026-06-30)

First bring-up used a **CC1352R1 LaunchPad as a standalone XDS110** over hand-wired dupont leads.
That link was **chronically marginal**: connects went `Error -2131 @ 0x0 "Unable to access device
register"` intermittently and, after handling/soldering near the board, stopped connecting at all.
What was learned: `xds110reset` and a board power-cycle do **not** clear it — only **physically
reseating the JTAG leads** (and sometimes several connect retries) recovered it; a fully open lead
(or a disturbed **VTREF/3V3-sense** line — the XDS110 needs the target 3.3 V reference to connect)
gives a persistent -2131 regardless of software. Mitigations, in order: a **short, soldered/secured
JTAG harness** (not loose long dupont); set a **fixed low TCLK** (e.g. 500 kHz) via the CCS GUI target
config — hand-editing `USCIF.TCLK_PROGRAM` into the ccxml is fiddly and was rejected as `Error -300`,
so generate it from the GUI. If a stage aborts mid-run with -2131, the target is left in whatever state
it was; just reseat and re-run (all stages are restart-safe and force safe-off on exit).

## Bench helper probes (ad-hoc diagnostics, READ-ONLY)

Written during 2026-06-30 bring-up to dig past what the stage scripts expose. All read-only; gates stay
OST-tripped.

- `s5b_route.js <ccxml> <out>` — CMPSS comparator/route test with **no injection**: lowers each high-side
  DAC below the resting signal so the idle level trips the comparator (proves CMPSS3/CMPSS5 sense their
  input + DAC). Restores the DAC + clears the latch on exit.
- `enc_probe.js <ccxml> <out> [bpline]` — breakpoints `MT6701_SSI_read()` and dumps the decoded SSI frame
  (`crc_ok/field_ok/track_ok`, angle) to tell dead-SPI vs bad-CRC vs weak-field.
- `can_es.js <ccxml> <out>` — CANA error/status: `LEC` (ACK/bit/stuff/form), TEC/REC, EPASS/BOFF, and the
  `CAN_BTR` decoded to a bit rate. The fast way to see *why* CAN isn't passing (e.g. `LEC=ACK` = nothing on
  the bus ACKs).
- `can_probe.js <ccxml> <out> <&s_rx> <&s_tx> <&s_tx_in_flight>` — TX/RX FIFO movement + `g_dn.node_id`.
  The FIFOs are file-static, so pass their addresses (DSS can't resolve locals); resolve with:
  `nm <out> | awk '$3=="s_rx"{print "0x"$1}'` (likewise `s_tx`, `s_tx_in_flight`).

There is also a gated **RGB self-test**: build the product with `EXTRA_DEFINES="--define=RGB_SELFTEST"`
(the `#ifdef` block after `RGB_init()` in `product/product_main.c`) and watch RGB1 sweep R→G→B→white→off
at boot — verifies the GPIO12→SN74LVC1T45→WS2812 path. Off by default.

## Lab and product runners

SDK lab runners and product scripts (all in `tools/flash/esc6288_revA/`; no `run_ical` — the
current-scale cal was done through is05 at 8 A, not a separate runner):

| script | purpose |
|---|---|
| `check_is01_esc6288.js` | SAFETY-ONLY is01 check — HAL up, OST set, no switching |
| `cal_is02_esc6288.js` | is02 offset/gain cal, read back offsets (one-shot, gate stays OST) |
| `run_is04_esc6288.js` | is04 signal chain (open-loop I/f, no FAST) |
| `run_is05_esc6288.js` | is05 FAST motor ID to completion (Rs/Ls/flux); run continuously — halts desync Ls phase |
| `run_is06_esc6288.js` | is06 torque control first-spin (guarded; hand-flick bootstraps FAST) |
| `run_is07_esc6288.js` | is07 speed control |
| `run_iftest_esc6288.js` | is04 open-loop I/f rig (characterize I/f drag without FAST) |
| `run_selfstart_esc6288.js` | product open-loop I/f self-start (SU_ALIGN→SU_RAMP→SU_BLEND→SU_RUN; no hand-flick) |
| `run_slipguard_esc6288.js` | slip-guard / safe-off validator |
| `s5b_route.js` | CMPSS comparator/route test, no injection (read-only) |
| `enc_probe.js` | breakpoints MT6701_SSI_read(), dumps decoded SSI frame |
| `can_es.js` | CANA error/status (LEC, TEC/REC, EPASS/BOFF, BTR) |
| `can_probe.js` | TX/RX FIFO movement + `g_dn.node_id` (pass file-static addrs) |

## Stage 4 (first spin) — `run_is06_esc6288.js` (DONE 2026-06-30)

`run_is06_esc6288.js <ccxml> <is06_torque_control.out> [iq_A] [flick_s]` is the first-spin runner
(is06 torque lab; esc6288 OST safe-off, no EN_GATE). It un-trips the gates and is the only script that
drives switching, so it stays guarded: `iq_A=0` is SCOPE MODE (arm, half-bridges switch at ~0 A — scope
the dead-band before any torque); `iq_A>0` commands a small torque (≤1.0 A guard); `flick_s>0` runs the
FOC continuously so a hand-flick can bootstrap the sensorless FAST estimator from standstill. **Validated
2026-06-30:** scope mode clean, then `iq_A=1.0 flick_s=12` held ~4 krpm for 12 s with zero faults (see
the PORT_TODO bench log). Stop note: a debugger gate-cut of the spinning motor latches `faultUse=16`
(moduleOverCurrent) on the transient — benign (OC protection firing; safe-off achieved).

Manual stop conditions to honour at first spin:

- Zero/low bus first; current-limited supply; scope half-bridge Vds for shoot-through before raising bus.
- Confirm the MCU dead-band (200 ns, `HAL_PWM_DBxED_CNT`) + the JSM6288T's own ~200 ns DT give clean
  non-overlap on the scope before closing the loop (do not trust the additive number).
- Phase-A/B **software** OC (no CMPSS) only arms with PWM live — verify it there, not in stage 5.
- Abort to safe-off (force OST) on any fault latch.

## Product open-loop I/f self-start — `run_selfstart_esc6288.js` (DONE 2026-07-01)

The stock is06 path (above) requires a **hand-flick** to bootstrap the sensorless FAST estimator from
standstill. The product firmware (`product/product_main.c`, `g_su`) solves this via an open-loop I/f
startup state machine gated by `ESC6288_BENCH_THROTTLE`:

```
SU_ALIGN → SU_RAMP (open-loop I/f ramp) → SU_BLEND → SU_RUN (hand off to FAST)
```

`run_selfstart_esc6288.js <ccxml> <product.out> [iq_A]` arms the product image and drives it through
the I/f startup sequence. **Validated 2026-07-01 at 12 V / no prop:** self-starts with NO hand-flick,
zero faults, clean FAST handoff. Safe-off on exit is verified by `run_slipguard_esc6288.js`.

> The stock is06 path (hand-flick required) is still the first-spin reference for FAST baseline and
> scope verification. The I/f self-start is the product startup path; is06 is not replaced, only
> augmented.
