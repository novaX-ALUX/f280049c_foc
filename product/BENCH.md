# Product main — launchxl bench bring-up (③-c)

Bench runbook for `product/product_main.c` on **launchxl_drv8305evm + BOOSTXL-DRV8305EVM**.
Validates the launchxl torque chain: DroneCAN RawCommand → `esc_control` → FAST Iq, and
esc.Status / NodeStatus telemetry back. esc6288 is deferred (no schematic).

> Safety first: current-limited supply, **no propeller**, start with a small `iq_max_A`.
> The launchxl hardware over-current (CMPSS) path is still the inherited one
> (`boards/launchxl_drv8305evm/drivers/include/board.h` §6 TODO) — do NOT rely on it as the
> final protection. The software peak-current latch (`esc_control` on `i_motor_A`) is wired,
> but bench testing must still limit supply current.

## Build & load

```bash
# default: dynamic node id (DNA) — the H7E / ArduPilot path
BOARD=launchxl_drv8305evm MOTOR=am_4116_kva ESC_INDEX=0 PRODUCT=1 bash build.sh

# bench with a bare CAN tool (NO DNA allocator on the bus): pin a static node id
BOARD=launchxl_drv8305evm MOTOR=am_4116_kva ESC_INDEX=0 NODE_ID=25 PRODUCT=1 bash build.sh
```

Output nests by `ESC_INDEX` + `NODE_ID` so the DNA and static-id variants never overwrite each
other (avoids loading the wrong variant on the bench):
`build/launchxl_drv8305evm/am_4116_kva/product/esc<ESC_INDEX>_node<NODE_ID>/product.out`
— e.g. the two commands above produce `.../product/esc0_node0/product.out` and
`.../product/esc0_node25/product.out`.

This is a **RAM build** (`_RAM`, `f28004x_ram_cpu_is_eabi.cmd`): load it over JTAG with CCS /
UniFlash and run from RAM. It does **not** persist across power-cycle (fine for the bench).

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

## Out of scope here
esc6288 wiring (schematic / encoder / CMPSS / CAN pins), the is07 speed-PI ISR branch +
real prop-park, active short brake, Flash persistence of node-id / park-ref, DroneCAN GetSet,
and on-bus interop stress testing.
