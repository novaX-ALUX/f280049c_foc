# esc6288_revA -- FD6288/simple gate-driver Board Porting Checklist

This directory was copied from the SDK `boostxl_drv8320rs/f28004x` as a **compilable template** (ensuring is01 can build first).
The items below are the changes needed to adapt from DRV8320RS to the **FD6288/simple gate driver + your board**.

## 1. Gate Driver: DRV8320RS (SPI smart driver) → FD6288/simple gate driver
- [x] Build chain: `DRV8320_SPI` and `drv8320.c/h` removed.
- [x] `drivers/source/gate_driver.c` / `include/gate_driver.h` handle EN GPIO only.
- [x] `include/board.h` centralizes EN/FAULT GPIOs.
- [ ] Verify against schematic:
  - `BOARD_GATE_ENABLE_GPIO`
  - `BOARD_HAS_GATE_FAULT_INPUT`
  - `BOARD_GATE_FAULT_GPIO`

## 2. Current Sensing: DRV8320 integrated CSA → FD6288 external shunt + op-amp
- `HAL_NUM_CMPSS_CURRENT 3` (hal.h): confirm the number of shunt channels on your board (3/2/1-DC).
- **Key scaling macros** (currently in `drivers/include/user.h`, feeds back into the current loop):
  - `USER_ADC_FULL_SCALE_CURRENT_A` = full-scale current derived from (shunt resistance × op-amp gain × ADC reference)
  - Update current sensing ADC channels and CMPSS comparators (overcurrent protection threshold) to match your board
- Reference: the legacy project `../../esc_drv8300_foc` also uses a "simple driver + external shunt"; its current scaling approach can be borrowed directly.

## 3. Voltage Sensing
- `USER_ADC_FULL_SCALE_VOLTAGE_V`: update to match your board's DC bus voltage divider resistors.

## 4. PWM / Dead-band
- FD6288 has an **internal fixed dead-band** → the MCU-side dead-band can be set to a very small or minimum value (deadband setting in hal.c).
- Confirm FD6288 input mode (6-wire HIN/LIN or 3-wire PWM + enable), and update the EPWM channel mapping accordingly.

## 5. Pin Summary Table
- [x] `drivers/include/board.h` added.
- [ ] Continue migrating ADC/EPWM/CMPSS channel assignments from SDK template macros into `board.h`.

## Information Needed
PWM input mode / number of shunts · resistance · op-amp gain / presence of nFAULT / DC bus voltage divider ratio → provide these to complete the port precisely.
