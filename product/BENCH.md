# Product main — historical launchxl bench bring-up (③-c)

> Historical note: this runbook records the earlier `launchxl_drv8305evm` product-main bench
> validation. It is not the active bring-up path anymore. The current product target is
> `esc6288_revA`; use the root `README.md`, `boards/esc6288_revA/PORT_TODO.md`, and
> `tools/flash/esc6288_revA/README.md` for current work.

Bench runbook for `product/product_main.c` on **launchxl_drv8305evm + BOOSTXL-DRV8305EVM**.
Validates the launchxl torque chain: DroneCAN RawCommand → `esc_control` → FAST Iq, and
esc.Status / NodeStatus telemetry back.

> Safety first: current-limited supply, **no propeller**, start with a small `iq_max_A`.
> The launchxl CMPSS path is routed, digitally filtered, and ePWM-blanked for this bench,
> while keeping the inherited +/-11.8 A DAC threshold. Treat it as bench protection only:
> the real trip current still needs controlled-load validation, and esc6288 will use its
> own shunt/CMPSS thresholds.

## Build & load

```bash
# default: dynamic node id (DNA) — the H7E / ArduPilot path
BOARD=launchxl_drv8305evm MOTOR=am_4116_kv450 ESC_INDEX=0 PRODUCT=1 bash build.sh

# bench with a bare CAN tool (NO DNA allocator on the bus): pin a static node id
BOARD=launchxl_drv8305evm MOTOR=am_4116_kv450 ESC_INDEX=0 NODE_ID=25 PRODUCT=1 bash build.sh
```

Output nests by `ESC_INDEX` + `NODE_ID` so the DNA and static-id variants never overwrite each
other (avoids loading the wrong variant on the bench):
`build/launchxl_drv8305evm/am_4116_kv450/product/esc<ESC_INDEX>_node<NODE_ID>/product.out`
— e.g. the two commands above produce `.../product/esc0_node0/product.out` and
`.../product/esc0_node25/product.out`.

This is a **RAM build** (`_RAM`, `f28004x_ram_cpu_is_eabi.cmd`): load it over JTAG and run from
RAM. It does **not** persist across power-cycle (fine for the bench).

## Headless flash + verify (DSS, no CCS GUI)

Load and read back stage-A variables over the XDS110 straight from the shell. The target config
`tools/flash/common/f280049c_xds110.ccxml` binds the F280049C under the XDS110; its driver set **must**
include `cla2` (`icepick_c + c28x + cla2 + cs_child + ajsm`) — F280049C has a CLA that the
F280025C device file lacks, and the DebugServer rejects board-data generation ("invalid processor
ID") without the matching CLA driver.

```bash
DSLITE=~/ti/ccs/ccs_base/DebugServer/bin/DSLite
DSS=~/ti/ccs/ccs_base/scripting/bin/dss.sh
CCXML="$PWD/tools/flash/common/f280049c_xds110.ccxml"
OUT="$PWD/build/launchxl_drv8305evm/am_4116_kv450/product/esc0_node25/product.out"

# load to RAM + run (DSLite loads and exits, leaving the target running):
"$DSLITE" load -c "$CCXML" "$OUT"

# attach, run 3.5s, halt, print the stage-A readout, leave target running:
"$DSS" tools/flash/common/verify_stagea.js "$CCXML" "$OUT"
```

`verify_stagea.js <ccxml> <product.out>` prints `g_now_ms` twice (confirms the 1 ms tick
increments by ~1000), `flagEnableSys` / `flagEnableOffsetCalc` / `flagRunIdentAndOnLine`,
`faultUse.all`, `VdcBus_V`, and `g_dn.node_id` / `g_dn.armed`.

> DSLite drops a multi-MB trace named `true` in the cwd (its `--log` arg defaults to that string);
> it is gitignored. Pass `-g /tmp/dslite.log` to redirect it.

> **DSS float gotcha (verified on the bench):** `expression.evaluate("someFloat")` **truncates
> float32 to an integer** (e.g. `25.3406 -> 25`, `-23.87 -> -23`, `0.0125 -> 0`). Integer fields,
> pointers, and `&expr` addresses come through fine. So every script reads floats *exactly* by
> taking `&expr` (an int) and reconstructing the two 16-bit C28x words as IEEE754:
> `Float.intBitsToFloat(((hi&0xFFFF)<<16)|(lo&0xFFFF))` — see the `getf()` helper in
> `cal_is02.js` / `verify_stagea.js` / `prepare_drv8305_gate.js`. `tools/flash/common/diag_precision.js`
> demonstrates the bug (writes known floats, reads back via both paths). This matters: a naive
> `VdcBus_V > 5` gate "works" only because `25 > 5`, but any fine reading (offsets, currents, sub-volt
> trims) is garbage unless read the exact way.

- `ESC_INDEX=<0..19>` — our slot in the RawCommand array (validated in `build.sh`).
- `NODE_ID=0` — dynamic allocation (DNA). Needs an allocator on the bus (ArduPilot/H7E has one;
  pydronecan can run one). Until allocated, the node sends only DNA requests and — by design —
  ignores RawCommand (no arm, no output).
- `NODE_ID=1..127` — static node id, skips DNA. Use this when driving from a plain CAN tool.

## CCS watch variables

| Symbol | Meaning |
|---|---|
| `motorVars.flagEnableSys` | self-set true after init (no watch-window hand-off) |
| `motorVars.flagEnableOffsetCalc` | true during ADC offset cal; self-clears when done |
| `motorVars.flagRunIdentAndOnLine` | 1 only when armed + throttle + cal done |
| `motorVars.VdcBus_V` / `motorVars.speed_krpm` | bus voltage / estimated speed |
| `Idq_in_A.value[1]` / `IdqSet_A.value[1]` | measured Iq / commanded Iq |
| `g_now_ms` | 1 ms product-tick counter (liveness) |
| `g_dn.node_id` / `g_dn.armed` | effective node id (0 until DNA) / zero-frame handshake passed |
| `g_esc.state` / `g_esc.hard_fault_bits` / `g_esc.status_bits` | control state / latched faults / soft status |

LED2 blinks from `mainISR` — a steady blink means the control ISR is running.

## Stage A — no motor / current-limited smoke
1. Load + run `product.out`.
2. `flagEnableSys` becomes **true automatically** (confirms no watch-window dead-wait).
3. `g_now_ms` increments (1 ms tick alive); LED2 blinks (ISR alive).
4. Offset cal completes: `flagEnableOffsetCalc` → false (~50000 ISR cycles, ~2.5 s @ 20 kHz).
5. With nothing armed, `flagRunIdentAndOnLine` stays 0 and PWM is disabled (no output).
6. On a CAN analyzer: DNA Allocation requests at ~1 Hz (when `NODE_ID=0`).

## Stage B — CAN interop
- **DNA (`NODE_ID=0`)**: with an allocator present, `g_dn.node_id` becomes non-zero, then
  NodeStatus is sent at 1 Hz. Without an allocator it stays 0 — switch to a static `NODE_ID`.
- Send a RawCommand addressed to our `esc_index`:
  - 10 consecutive zero frames → `g_dn.armed = true`.
  - Then a throttle value → `g_esc` sees the throttle; esc.Status at 10 Hz carries our esc_index.
- Wrong `esc_index` → command not updated. RawCommand received **before** allocation → ignored
  (DNA gate; arming starts fresh after allocation).

## Stage C — low-voltage, current-limited, no prop, small Iq
- Arm + small throttle → `IdqSet_A.value[1] = throttle * iq_max_A`; FAST spins up,
  `Idq_in_A.value[1]` tracks the command.
- Drop the CAN link for ≥ `cmd_timeout_s` (0.5 s) → coast: `status_bits` shows
  `ESC_ST_FAILSAFE_COAST`, `IdqSet_A.value[1]` → 0, `flagRunIdentAndOnLine` → 0.
- `arm = false` (disarm) → immediate no output.

## Stage D — record parameters (→ commit 4)
Fill in stable bench values; these become the config-finalization commit.

| Parameter | Source | Bench default | Measured |
|---|---|---|---|
| `iq_max_A` | esc_control_cfg | 5.0 | _tbd_ |
| `iq_slew_A_s` | esc_control_cfg | 50.0 | _tbd_ |
| `cmd_timeout_s` | esc_control_cfg | 0.5 | _tbd_ |
| `vbus_ov_set / clr` | esc_control_cfg | 30 / 28 | _tbd_ |
| `vbus_uv_set / clr` | esc_control_cfg | 9 / 11 | _tbd_ |
| `oc_set_A / clr_A` | esc_control_cfg | 8 / 6 | _tbd_ |
| `iq_cmd_limit_A` | foc_bridge_cfg | 6.0 | _tbd_ |
| pole pairs | `USER_MOTOR_NUM_POLE_PAIRS` | per motor | _tbd_ |

## Power labs (is02+) — DRV8305 gate-enable prep

Every SDK sensorless lab that drives the power stage (is02 cal, is03 hardware test, is04 signal
chain, is05 motor ID, is06+ control) calls `HAL_enableDRV()` only under `#ifdef DRV8320_SPI`, but
launchxl builds with `DRV8305_SPI`, so the lab's own main never enables the DRV8305. DSS on this
setup can read symbols but cannot call target functions, so the prep script uses the only reliable
debugger path: run the lab to its dead-wait, assert EN_GATE by writing GPIO39, then release
`flagEnableSys`.

```bash
BOARD=launchxl_drv8305evm MOTOR=<motor> LAB=<lab> bash build.sh
"$DSS" tools/flash/drv8305evm/prepare_drv8305_gate.js "$CCXML" \
   build/launchxl_drv8305evm/<motor>/<lab>/<lab>.out
```

It runs the lab to its `while(flagEnableSys==false)` wait, asserts EN_GATE directly
(`GPBSET<-0x80` → GPIO39 high), sets `flagEnableSys=true` so offset cal runs, and **hard-fails**
(pulls EN_GATE low, leaves the target halted, exits 1) unless all checks pass: EN_GATE readback high,
offset cal done, `faultUse.all==0`, `VdcBus_V>5`. Positive proof the gate woke: `faultUse.all` goes
**16→0** (gate off → the DRV8305 CSAs float and the CMPSS trips; gate on → CSAs live, zero current
reads clean). On success it leaves the target running; drive the lab's own flow (offset/gain trim,
open-loop test, motor ID, …) from the CCS watch window.

Recommended order on this freshly-ported board: **is02 (offset+gain cal, confirm current/voltage
scaling) → is03 (open-loop spin, confirm phase order + current polarity/magnitude) → is04 (signal
chain) → is05 (motor ID)** — don't trust is05's Rs/Ls/flux until is02/is03 pass.

### EXCEPTION — AM-4116 (and any sub-~50 mOhm motor): skip is03 V/f, and is05 won't ID here

is03 is scalar **V/f open-loop**: it puts `USER_MOTOR_VOLT_MIN_V` straight across the winding at
standstill. For the 4116 (~40 mOhm) that is `4.0 V / 0.04 Ohm ~ 100 A` — an instant over-current; the
launchxl CMPSS (correctly) trips in microseconds (the charged 25 V bus cap supplies the spike before
the PSU foldback responds). **Do NOT run is03 V/f for the 4116** — it is not a meaningful step for a
low-Rs motor; confirm phase order under FAST/current control instead.

is05 (FAST ID) also will not complete on the launchxl for the 4116: the ID-startup current transient
exceeds the launchxl CMPSS trip and its **+-23.57 A current-sense ceiling** (7 mOhm shunt x CSA gain
10). The 4116 profiles are therefore **back-filled from the verified legacy esc_drv8300 FAST ID**
(see `motors/README.md`), not re-identified here. On the launchxl, exercise the 4116 only with small
Iq, low-load sanity (is06 / product torque path); full-power ID and running belong to esc6288, whose
shunt/CMPSS are sized for this motor. The 62xx (higher Rs) still follow the normal is02→is05 flow.

For the 4116 is06 sanity path, use the dedicated guarded runner:

```bash
BOARD=launchxl_drv8305evm MOTOR=am_4116_kv450 LAB=is06_torque_control bash build.sh
"$DSS" tools/flash/drv8305evm/run_is06.js "$CCXML" \
   build/launchxl_drv8305evm/am_4116_kv450/is06_torque_control/is06_torque_control.out 0.2
```

Hardware result after CMPSS route + blanking cleanup: at 24 V bus, `run_is06.js` completed the full
~9 s window at Iq=0.2 A and Iq=1.0 A with `faultUse.all=0`. FAST online flux averaged close to the
KVA/KV450 profile (`~0.012 V/Hz`). Treat that as a low-load sanity check only; full-power operation
and final protection thresholds are esc6288 work.

### is02 uses a dedicated script (`cal_is02.js`), not the generic gate-prep

is02 is the ONLY sensorless lab that re-arms offset calibration forever: after each 50000-ISR pass
it latches the offsets then immediately re-sets `flagEnableOffsetCalc=true` (guarded by the
lab-only `flagEnableOffsetCalibration`, default true). So `flagEnableOffsetCalc` is ~always 1 and
the generic script's "`flagEnableOffsetCalc==0` => cal done" readiness gate would **false-fail** on
is02. is03/is04/is05 self-clear once, so they keep using `prepare_drv8305_gate.js`.

```bash
BOARD=launchxl_drv8305evm MOTOR=am_4116_kv450 LAB=is02_offset_gain_cal bash build.sh
"$DSS" tools/flash/drv8305evm/cal_is02.js "$CCXML" \
   build/launchxl_drv8305evm/am_4116_kv450/is02_offset_gain_cal/is02_offset_gain_cal.out
```

`cal_is02.js` does the same register-level EN_GATE bring-up + safety gate, but first sets
`flagEnableOffsetCalibration=false` (one-shot cal so the offsets freeze and the flag settles to 0),
then reads back the **analog-front-end health at zero current** (no motor / current-limited / no
prop): `offsets_I_A` (3-phase symmetry), `offsets_V_V`, the post-offset-removal residual + noise
band on `adcData.I_A`, and `VdcBus_V` vs the metered bus. It is a HEALTH CHECK, not a final
calibration: the current gain (`USER_ADC_FULL_SCALE_CURRENT_A=47.14`) needs a known injected
current and is deferred to is03; the voltage gain can be sanity-checked now (meter VM/PVDD vs
`VdcBus_V`).

Per motor, capture the identified `USER_MOTOR_Rs_Ohm`, `Ls_d/Ls_q_H`, `RATED_FLUX_VpHz` and back
them into `motors/<motor>.h` (the select→is05→backfill→tune workflow).

## Historical scope boundary
At the time of this launchxl bench, esc6288 wiring, encoder/CMPSS/CAN integration, speed-mode
product plumbing, active brake, persistence, DroneCAN parameters, and on-bus interop were outside
this runbook. Several of those items are now implemented for esc6288; this file should not be used
as the current product status source.
