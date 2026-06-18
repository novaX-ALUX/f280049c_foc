# src/ —— 内核层(与硬件/电机无关)

> 状态:**规划中(空目录占位)**。当前 bring-up 阶段控制代码仍来自 SDK lab
> (`solutions/common/sensorless_foc/`),`src/` 尚未填充。待 is05~is07 跑通、
> 形成自有应用后,逐步把"与硬件/电机无关"的逻辑迁到这里。

这一层是第三条变化轴:**不随板子、不随电机变化**的核心逻辑。
换板只动 `boards/`,换电机只动 `motors/`,三者正交,`src/` 不动。

| 子目录 | 职责 | 来源参考 |
|--------|------|----------|
| `app/`     | 控制状态机、保护逻辑、模式切换(与硬件/电机解耦) | 自研 |
| `comms/`   | DroneCAN / CAN 通信栈 | 旧项目 `../esc_drv8300_foc`、SDK `servo_drive_with_can` |
| `encoder/` | MT6701 有感编码器接口(嫁接到 FAST 无感之上) | SDK `absolute_encoder_boostxl_posmgr` |
| `common/`  | 通用类型与工具 | — |
