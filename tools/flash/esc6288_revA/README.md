# esc6288_revA bring-up scripts

DSS stage scripts to run **in order** the day the prototype returns from fab, mirroring the
6-step bench order in `boards/esc6288_revA/PORT_TODO.md`. They are an *executable runbook*: each
prints explicit pass / stop conditions so bring-up is "follow the steps", not "improvise".

> These run against real hardware over the XDS110 and have NOT been executed (no board yet);
> they are syntax-checked only. Expect to tune thresholds/timing at the bench.

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
| 6 | `s6_peripherals.js` | CAN/enc/RC-PWM/RGB | CAN FIFO, MT6701 `g_enc`, RC-PWM eCAP, RGB note | none |

Invoke (shared ccxml + the product `.out`):

```bash
dss.sh tools/flash/esc6288_revA/s1_rails_clock.js tools/flash/common/f280049c_xds110.ccxml <product.out>
dss.sh tools/flash/esc6288_revA/s2_idle_ost.js    tools/flash/common/f280049c_xds110.ccxml <product.out> [verify=offcal|verify=untrip]
dss.sh tools/flash/esc6288_revA/s3_adc_offsets.js tools/flash/common/f280049c_xds110.ccxml <product.out>
dss.sh tools/flash/esc6288_revA/s5_protection.js  tools/flash/common/f280049c_xds110.ccxml <product.out> [force=tz|inject=oc|inject=ov]
dss.sh tools/flash/esc6288_revA/s6_peripherals.js tools/flash/common/f280049c_xds110.ccxml <product.out>
```

## Stage 4 (first spin) — deliberately NOT scripted yet

The PORT_TODO step 4 (dead-band/short-pulse → close current loop → first spin) is the step with
the most live judgement and is the only one that actually un-trips the gates into switching. It is
**deferred to a separate first-spin script** written after stages 1/2/3/5/6 have run and the scope
conditions are known. Manual stop conditions to honour at first spin:

- Zero/low bus first; current-limited supply; scope half-bridge Vds for shoot-through before raising bus.
- Confirm the MCU dead-band (200 ns, `HAL_PWM_DBxED_CNT`) + the JSM6288T's own ~200 ns DT give clean
  non-overlap on the scope before closing the loop (do not trust the additive number).
- Phase-A/B **software** OC (no CMPSS) only arms with PWM live — verify it there, not in stage 5.
- Abort to safe-off (force OST) on any fault latch.
