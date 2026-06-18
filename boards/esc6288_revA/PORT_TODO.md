# esc6288_revA —— FD6288 板级移植清单

本目录从 SDK `boostxl_drv8320rs/f28004x` 复制为**可编译模板**（保证 is01 先能跑），
下面是从 DRV8320RS 适配到 **FD6288 + 你的板** 需要改的点。

## 1. 栅极驱动：DRV8320RS(SPI智能驱) → FD6288(简单驱, 无SPI)
- `drivers/source/drv8320.c` / `include/drv8320.h`：**FD6288 无 SPI/寄存器**。
  - 删除/桩化所有 `DRV8320_writeData/readData/setupSpi` 调用。
  - 栅驱"配置"退化为：拉高 EN_GATE GPIO 使能即可。
  - 建议改名 `gate_driver.c/.h`，并把 hal.c 里对 DRV8320_* 的调用一并改掉。
- `hal.h` 引脚定义需改成你板子的实际 GPIO：
  - `HAL_DRV_SPI_CS_GPIO 57` → 删（无 SPI）
  - `HAL_DRV_EN_GATE_GPIO 13` → 你板上 FD6288 的使能脚
  - `HAL_PM_nFAULT_GPIO 40` / `HAL_PM_nOCTW_GPIO 40` → FD6288 若无故障输出则删；有则改脚号

## 2. 电流采样：DRV8320 内置CSA → FD6288 外部分流+运放
- `HAL_NUM_CMPSS_CURRENT 3`（hal.h）：确认你是几路分流（3/2/1-DC）。
- **关键标定宏**（在 `user_m1.h`，会反馈到电流环）：
  - `USER_M1_ADC_FULL_SCALE_CURRENT_A` = 你的(分流阻值 × 运放增益 × ADC基准)换算的满量程电流
  - 电流采样 ADC 通道、CMPSS 比较器（过流保护阈值）按你板子改
- 参考蓝本：旧项目 `../../esc_drv8300_foc` 也是"简单驱+外部分流"，电流标定思路可直接借鉴。

## 3. 电压采样
- `USER_M1_ADC_FULL_SCALE_VOLTAGE_V` 按你的母线分压电阻改。

## 4. PWM / 死区
- FD6288 **内部固定死区** → MCU 侧死区可设很小或最小值（hal.c 的 deadband 设置）。
- 确认 FD6288 输入方式（6 路 HIN/LIN 还是 3 路 PWM+使能），对应 EPWM 通道映射。

## 5. 引脚总表
- 把上面所有 GPIO/ADC/EPWM 通道集中到 `board.h`，作为这块板的唯一硬件事实来源。

## 待你提供
PWM 输入方式 / 分流路数·阻值·运放增益 / 有无 nFAULT / 母线分压比 → 我即可精确改写。
