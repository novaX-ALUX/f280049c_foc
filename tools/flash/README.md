# tools/flash — bench bring-up & diagnostic scripts

DSS (Debug Server Scripting) helpers driven over the XDS110 debugger, plus a host-side
DroneCAN probe. They load a `.out`, run it, poke variables, and read results back — used to
bring up each board step-by-step against its `boards/<board>/PORT_TODO.md`.

Organized **by board**, so the boards currently on hold stay quarantined and
`esc6288_revA` remains the active bring-up target.

## Invocation

```bash
# DSS scripts (.js): the SDK's dss.sh runs them; pass the shared ccxml + the lab/product .out
dss.sh tools/flash/<dir>/<script>.js  tools/flash/common/f280049c_xds110.ccxml  <build/.../foo.out>  [args...]

# host probe (.py): runs on the PC against the CAN-USB device
python3 tools/flash/common/dronecan_probe.py [device=/dev/ttyACM2] [target_node=25] [our_node=127]
```

Each script's header comment carries its exact usage line and pass/fail criteria.

## common/ — board-agnostic

| script | purpose |
|---|---|
| `f280049c_xds110.ccxml` | shared XDS110 target config (same F280049C MCU on every board) |
| `verify_stagea.js` | Stage-A liveness: connect / load / run / halt / read product vars |
| `diag_bus_sweep.js` | is the `adcData.dcBus_V` read path live or frozen? (raw count vs reported V) |
| `diag_precision.js` | DSS float-readback precision check (expression eval vs raw IEEE754) |
| `diag_oc_latch.js` | classify the product-main `moduleOverCurrent` latch after a forced OC |
| `diag_can_state.js` | CANA / DroneCAN peripheral + FIFO state (reads `s_*` by symbol -- board-agnostic) |
| `diag_rx_read.js` | RX FIFO state by ADDRESS; per-build addrs default to launchxl, override `txif=/ints=/rx=/tx=` (re-derive with `ofd2000 -g`) |
| `diag_tx_state.js` | TX/RX FIFO state by ADDRESS; per-build addrs default to launchxl, override `txif=/ints=/rx=/tx=` (re-derive with `ofd2000 -g`) |
| `dronecan_probe.py` | host-side serial DroneCAN node probe (enumerate / poll a node) |

## drv8305evm/ — LAUNCHXL-F280049C + BOOSTXL-DRV8305EVM

Original validation board. EN_GATE bring-up, DRV8305 SPI/CSA, 7 mΩ shunt, BOOSTXL J2 ISEN.

| script | purpose |
|---|---|
| `prepare_drv8305_gate.js` | enable the DRV8305 gate driver for any sensorless_foc SDK lab |
| `diag_drv8305_spi.js` | read/verify DRV8305 SPI registers |
| `diag_oc_classify.js` | classify a trip: DRV8305 nFAULT (GPIO13) + per-phase CMPSS latch |
| `cal_is02.js` | is02 offset/gain cal, read back offsets (runs cal once) |
| `run_socal.js` | hold Id = 0/1/2 A to probe ISEN_A/B/C, cross-check current scale |
| `run_curcal.js` | hold Id = 1/2/3 A for a DC clamp-meter current-scale check |
| `run_is03.js` | is03 scalar V/f open-loop first spin |
| `run_is05.js` | is05 FAST motor ID to completion (Rs/Ls/flux) |
| `run_is05_rampup.js` | is05 partial ID — stop after RampUp (non-destructive spin check) |
| `run_is06.js` | is06 small-Iq no-load torque + KV cross-check |
| `run_5hz.js` | low-speed drag-hold, watch FAST estimate vs true speed |
| `run_draghi.js` | strong-Id align + slow forced-angle ramp (smooth drag check) |
| `run_if_char.js` | I-f → FAST handoff: per-plateau convergence (clean is04) |
| `run_if_rec.js` | I-f time-series recorder (needs `is04_if_recorder.patch`) |
| `run_if_rampB.js` | high-frequency I-f ramp with in-ISR recorder |
| `is04_if_recorder.patch` | bench-only is04 fork adding the `g_rec_*` ring buffer (NOT committed into the SDK tree) |

## 3phganinv/ — LAUNCHXL-F280049C + BOOSTXL-3PhGaNInv  ⏸ on hold

LMG5200 GaN half-bridges, **no internal dead-time** (MCU dead-band is the only shoot-through
protection). Paused: gate polarity + ~200 ns dead-time confirmed, but FAST never locked at low
speed — see `boards/launchxl_3phganinv/PORT_TODO.md`. Historical safe-order scripts:

| order | script | purpose |
|---|---|---|
| 1 | `check_3phganinv_is01.js` | SAFETY-ONLY is01 check, gate buffer kept OFF |
| 2 | `cal_is02_3phganinv.js` | zero-current offsets + front-end health, buffer still OFF |
| 3 | `scope_deadtime_3phganinv.js` | scope half-bridge inputs at 0 V bus, verify GaN dead-time |
| 4 | `run_is06_3phganinv.js` | small-Iq torque sanity |
| 5 | `run_is07_3phganinv.js` | speed-control first spin + phase-order check |

## esc6288_revA/ — ESC6288 rev A (JSM6288T, no EN / no nFAULT)

Primary target. First prototype bench-run (2026-06-30/07-01) — stages passed as documented (protection
comparator/route + OC path proven; **OV injection still pending** — see PORT_TODO). See
`esc6288_revA/README.md` for the full runbook, pass/stop conditions, and the two hard safety rules
(default observe-only; any un-trip/PWM behind an explicit arg with unconditional safe-off on exit).
Safe-off here is the EPWM trip-zone (OST), since the JSM6288T has no gate-enable pin.

| script | purpose |
|---|---|
| `s1_rails_clock.js` | rails + clock (SYSCLK ~100 MHz via tick/wall ratio) |
| `s2_idle_ost.js` | idle PWM / OST safe-off (`verify=offcal`/`verify=untrip`) |
| `s3_adc_offsets.js` | ADC zero-current / Udc / NTC→°C |
| `s5_protection.js` | CMPSS3 OC / CMPSS5 OV / trip-zone (`force=tz`/`inject=oc`/`inject=ov`) |
| `s5b_route.js` | CMPSS comparator/route test, no injection |
| `s6_peripherals.js` | CAN / MT6701 encoder / RC-PWM / RGB (read-only) |
| `check_is01_esc6288.js` | SAFETY-ONLY is01 check, OST invariant |
| `cal_is02_esc6288.js` | is02 offset/gain cal (one-shot) |
| `run_is04_esc6288.js` | is04 signal chain / open-loop I/f |
| `run_is05_esc6288.js` | is05 FAST motor ID (continuous; halt desyncs Ls phase) |
| `run_is06_esc6288.js` | is06 first-spin torque (hand-flick bootstraps FAST) — DONE 2026-06-30 |
| `run_is07_esc6288.js` | is07 speed control |
| `run_iftest_esc6288.js` | is04 open-loop I/f characterization rig |
| `run_selfstart_esc6288.js` | product I/f self-start (no hand-flick) — DONE 2026-07-01 |
| `run_slipguard_esc6288.js` | slip-guard / safe-off validator |
| `enc_probe.js` / `can_probe.js` / `can_es.js` | ad-hoc read-only diagnostics |

See `esc6288_revA/README.md` for the complete runner inventory, pass conditions, and bench logs.
