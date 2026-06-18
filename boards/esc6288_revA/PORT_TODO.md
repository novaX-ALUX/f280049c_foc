# esc6288_revA -- FD6288/simple gate-driver 板级移植清单

本目录从 SDK `boostxl_drv8320rs/f28004x` 复制为**可编译模板**（保证 is01 先能跑），
下面是从 DRV8320RS 适配到 **FD6288/simple gate driver + 你的板** 需要改的点。

## 1. 栅极驱动：DRV8320RS(SPI 智能驱) -> FD6288/simple gate driver
- [x] 构建链路已移除 `DRV8320_SPI` 和 `drv8320.c/h`。
- [x] `drivers/source/gate_driver.c` / `include/gate_driver.h` 只负责 EN GPIO。
- [x] `include/board.h` 集中保存 EN/FAULT GPIO。
- [ ] 对照原理图确认：
  - `BOARD_GATE_ENABLE_GPIO`
  - `BOARD_HAS_GATE_FAULT_INPUT`
  - `BOARD_GATE_FAULT_GPIO`

## 2. 电流采样：DRV8320 内置CSA → FD6288 外部分流+运放
- `HAL_NUM_CMPSS_CURRENT 3`（hal.h）：确认你是几路分流（3/2/1-DC）。
- **关键标定宏**（当前在 `drivers/include/user.h`，会反馈到电流环）：
  - `USER_ADC_FULL_SCALE_CURRENT_A` = 你的(分流阻值 × 运放增益 × ADC基准)换算的满量程电流
  - 电流采样 ADC 通道、CMPSS 比较器（过流保护阈值）按你板子改
- 参考蓝本：旧项目 `../../esc_drv8300_foc` 也是"简单驱+外部分流"，电流标定思路可直接借鉴。

## 3. 电压采样
- `USER_ADC_FULL_SCALE_VOLTAGE_V` 按你的母线分压电阻改。

## 4. PWM / 死区
- FD6288 **内部固定死区** → MCU 侧死区可设很小或最小值（hal.c 的 deadband 设置）。
- 确认 FD6288 输入方式（6 路 HIN/LIN 还是 3 路 PWM+使能），对应 EPWM 通道映射。

## 5. 引脚总表
- [x] 已新增 `drivers/include/board.h`。
- [ ] 继续把 ADC/EPWM/CMPSS 通道从 SDK 模板宏迁入 `board.h`。

## 待你提供
PWM 输入方式 / 分流路数·阻值·运放增益 / 有无 nFAULT / 母线分压比 → 我即可精确改写。
