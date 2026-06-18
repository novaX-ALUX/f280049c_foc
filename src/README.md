# src/ — Core Layer (hardware/motor-agnostic)

> Status: **planned (empty directory placeholder)**. During the current bring-up phase, control code still comes from the SDK lab
> (`solutions/common/sensorless_foc/`); `src/` has not yet been populated. Once is05–is07 are running and
> a proprietary application takes shape, logic that is independent of hardware and motor will be migrated here incrementally.

This layer is the third axis of change: **core logic that does not vary with board or motor**.
Swap the board → touch only `boards/`; swap the motor → touch only `motors/`; the three axes are orthogonal and `src/` is untouched.

| Subdirectory | Responsibility | Source reference |
|--------|------|----------|
| `app/`     | Control state machine, protection logic, mode switching (decoupled from hardware and motor) | custom |
| `comms/`   | DroneCAN / CAN communication stack | legacy `../esc_drv8300_foc`, SDK `servo_drive_with_can` |
| `encoder/` | MT6701 sensored encoder interface (overlaid on FAST sensorless) | SDK `absolute_encoder_boostxl_posmgr` |
| `common/`  | Shared types and utilities | — |
