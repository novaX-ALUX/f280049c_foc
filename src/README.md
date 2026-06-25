# src/ — Core Layer (hardware/motor-agnostic)

> Status: **active product logic**. This tree contains the pure, host-tested code that supports the
> esc6288 product image: control state, setpoint bridging, prop parking, MT6701 decode/tracking,
> NTC conversion, nvparam storage format, and DroneCAN protocol helpers. SDK labs still provide the
> FAST/HAL reference ladder, but the product application is no longer an empty placeholder.

This layer is the third axis of change: **core logic that does not vary with board or motor**.
Swap the board → touch only `boards/`; swap the motor → touch only `motors/`; the three axes are orthogonal and `src/` is untouched.

| Subdirectory | Responsibility | Source reference |
|--------|------|----------|
| `app/`     | ESC state control, FOC bridge, park-ref logic, nvparam storage record | custom |
| `comms/`   | DroneCAN frame/protocol helpers, FIFOs, `param.GetSet` codec/registry | legacy `../esc_drv8300_foc`, pydronecan golden checks, SDK `servo_drive_with_can` |
| `encoder/` | MT6701 SSI frame decode and angle/velocity tracking | MT6701 datasheet, SDK `absolute_encoder_boostxl_posmgr` |
| `common/`  | Shared DTOs and NTC beta-model conversion | custom |

The purity rule is strict: no driverlib, HAL, board headers, SDK lab headers, or target-only side
effects under `src/`. Hardware access belongs in `boards/<board>/drivers/`; product glue lives in
`product/product_main.c`. Run `bash tools/test/run.sh` for host coverage and
`BOARD=esc6288_revA MOTOR=am_4116_kva SRC_CHECK=1 bash build.sh` for the C28x compile gate.
