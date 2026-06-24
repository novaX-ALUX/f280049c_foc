# launchxl_3phganinv — LAUNCHXL-F280049C + BOOSTXL-3PhGaNInv Porting Checklist

GaN bring-up platform: TI BOOSTXL-3PhGaNInv (SENS007, Rev A) on the F280049C LaunchPad,
after the BOOSTXL-DRV8305EVM bring-up was set aside. The control core (FOC/FAST/motor ID/loop
tuning) is board-agnostic and reused as-is; this directory is only the board layer.

Cloned from `boards/launchxl_drv8305evm/` (same LaunchPad Site 1; identical PWM + current-sense
pins). Key deltas applied: no SPI gate driver, active-LOW PWM enable, remapped voltage channels,
GaN scaling, GaN dead-time, and a digital over-temp trip. See board.h / hal.h / hal.c / user.h.

## Power stage
Three LMG5200 GaN half-bridges (80 V / 10 A) behind an SN74AVC8T245 PWM buffer (active-low OE).
INA240A1 (gain 20) on 5 mΩ in-line shunts, 1.65 V bias (REF3333). TMP302 digital over-temp.
On-board hardware OCP (~12 A, 1 A hyst) trips the buffer independently as a backstop.

## Pin Mapping (Site 1 / BoosterPack1 J1-J4) — locked; cross-referenced from schematics
| Function | Header | GPIO/ADC | Peripheral / notes |
|------|:--:|:--:|------|
| PWM A H/L | 40/39 | GPIO10/11 | EPWM6A/6B |
| PWM B H/L | 38/37 | GPIO8/9   | EPWM5A/5B |
| PWM C H/L | 36/35 | GPIO4/5   | EPWM3A/3B |
| IA/IB/IC | 27/28/29 | ADCIN B2/C0/A9 | CMPSS 3/1/6, direct-pin mux 0/1/3; PGA disabled |
| VA/VB/VC | 24/25/26 | ADCIN B0/C2/B1 | divider 100k/4.22k |
| VDC (bus) | 23 | ADCIN A5 | divider 100k/4.22k |
| VREF (diag, unused) | 30 | ADCIN A1 | not sampled |
| nEn_uC (PWM enable) | 13 | GPIO39 (out) | **ACTIVE-LOW**; safe-off = HIGH; ext pull-up R30 |
| OT (over-temp) | 34 | GPIO58 (in)  | **active-low**; routed to ePWM trip via HAL_PM_nFAULT_GPIO |

No SPI, no gate-driver fault pin, no WAKE. `BOARD_HAS_GATE_FAULT_INPUT = 0`.

## Scaling Constants (user.h)
- `USER_ADC_FULL_SCALE_CURRENT_A = 33.0`  (5 mΩ × INA240 gain 20 = 0.1 V/A → ±16.5 A)
- `USER_ADC_FULL_SCALE_VOLTAGE_V = 81.5`  ← ⚠️ confirm exact divider resistors on the bench
- `IA/IB/IC_OFFSET_A = +16.5`  (POSITIVE: read functions use a positive current_sf on this board)
- Current `current_sf` is POSITIVE in HAL_readADCData* (GaN INA240 inverted vs DRV8301/8305).

## Bench-verify items (do before applying bus voltage)
- [ ] **Gate enable polarity**: confirm GPIO39 idles HIGH (PWM disabled) at reset and after
      HAL_setupGPIOs; only HAL_enableDRV() drives it LOW. Verify no PWM at the LMG5200 inputs
      until enabled.
- [ ] **Dead-time** (`HAL_PWM_DBRED_CNT`/`DBFED_CNT = 20` ≈ 200 ns in hal.h): LMG5200 has NO
      internal shoot-through protection — scope both half-bridge inputs at low/zero bus, confirm
      non-overlap, then tune down. Do NOT raise bus voltage until verified.
- [ ] **Current-sense sign**: run is02 offset-cal, inject a known DC current, confirm Iq/torque
      direction. Flip `current_sf` sign in hal.h (both read functions) + offset sign in user.h if needed.
- [ ] **Phase order**: `BUILD_PWM_PHASE_ORDER=4` (build.sh) is the starting point — confirm
      rotation direction; swap two motor leads or change the order if reversed.
- [ ] **Over-temp trip**: confirm GPIO58 reads HIGH in normal operation (TMP302 deasserted, pull-up
      R48) and that asserting OT (active-low) trips PWM via the ePWM trip zone.
- [ ] **Voltage full-scale**: confirm 81.5 V against the actual divider; read VDC at a known bus voltage.

## Deferred (follow-up)
- CAN bridge (`can_bridge.c`, CANA GPIO32/33 — free on Site 1) → enables `CAN_CHECK`/`PRODUCT`.
- `product/product_main.c` board cases (limits, digital-OT path instead of analog NTC, init,
  offset-cal exception). Note: product reads `BOARD_GATE_FAULT_GPIO` when `BOARD_HAS_GATE_FAULT_INPUT`
  — this board sets it 0, so map over-temp via `BOARD_OVERTEMP_GPIO`, do not reuse the fault macro.

## Build / verify
```bash
# primary gate (compiles + links the board HAL with a lab):
BOARD=launchxl_3phganinv MOTOR=am_4116_kva LAB=is01_intro_hal bash build.sh
# regression: every single-motor lab:
BOARD=launchxl_3phganinv LAB=all bash build.sh
# secondary (src/ modules only; does NOT exercise board headers):
BOARD=launchxl_3phganinv MOTOR=am_4116_kva SRC_CHECK=1 bash build.sh
```
