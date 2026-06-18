# esc_f280049c_foc —— 全新 ESC 工程（TMS320F280049C）

基于 **C2000Ware MotorControl SDK 6.00.00.00** 的现代 FOC 架构（driverlib + EABI）。
取代旧的 `../esc_drv8300_foc`（F28027F / MotorWare，停更）和中途的 `../motorware_clean`（F28062F，已弃用）。

## 目标平台
- MCU: **TMS320F280049C**（F28004x，100 MHz，FPU + **TMU**，256KB Flash / 100KB RAM）
- 软件栈: **C2000Ware MotorControl SDK 6.0** + driverlib（**不再是 MotorWare**）
- 栅极驱动: DRV8300（参考 SDK 的 DRV8320RS 方案移植）
- 控制: InstaSPIN **FAST 无感**（F280049C 带 FAST ROM）+ 后续嫁接 **MT6701 有感**

## 选定底座
SDK 方案 `solutions/boostxl_drv8320rs/f28004x`（明确目标 = F280049C）。
递进式 bring-up 实验阶梯（类比旧 MotorWare 的 proj_lab01..10）：
```
is01_intro_hal      HAL/时钟/PWM 入门  ← 当前验证点
is02_offset_gain_cal  ADC 偏置/增益标定
is03_hardware_test    硬件自检
is04_signal_chain_test 信号链
is05_motor_id         电机辨识
is06_torque_control   转矩环
is07_speed_control    速度环
... is13_fwc_mtpa      弱磁/MTPA
```
每个实验有 EABI 和 COFF 两版 —— **本工程用 EABI**。

## 工具链（已在本机验证）
- 编译器: `cl2000` @ `~/ti/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS`（工程标 20.2.2，EABI 向前兼容）
- SDK: `~/ti/C2000Ware_MotorControl_SDK_6_00_00_00`
- driverlib (f28004x) 预编译: `.../c2000ware/driverlib/f28004x/driverlib/ccs/Release/driverlib_eabi.lib`
- **ABI: EABI**（`--abi=eabi`）—— 现代原生，**无需** F28062F 那套 `--abi=coffabi` 老 hack
- ⚠️ 链接 SDK 库务必用 `_eabi` 后缀变体（如 `fluxHF_eabi.lib`、`f28004x_fast_rom_symbols_fpu32_eabi.lib`），
  否则误链 COFF 版会出现 `EST_*` 未解析符号。

## 已完成
- [x] SDK 6.0 安装 + F28004x 支持核实
- [x] `is01_intro_hal`（RAM/EABI）命令行编译+链接验证通过 → 合法 c28xabi ELF 镜像
- [x] `build_is01.sh`（一键命令行构建，输出到 `build/is01/`）

## 待办（bring-up 顺序）
1. CCS Theia 导入 `is01..is03` 确认 GUI 侧 OK
2. **板级 HAL 移植**: 用 SDK 的 `drivers/source/hal.c` + `drv8320.c` 为模板，适配 esc_drv8300（DRV8300）板
3. ADC 标定 (is02) → 硬件自检 (is03) → 电机辨识 (is05) → 转矩/速度环
4. 嫁接 **MT6701 编码器**（有感 FOC，参考 SDK `absolute_encoder_boostxl_posmgr` / `servo_drive_with_can/sensored_foc`）
5. 嫁接 **CAN / DroneCAN**（参考 `servo_drive_with_can`）

## 构建
```bash
bash build_is01.sh   # 输出 build/is01/is01.out (RAM 镜像)
```

## 参考蓝本（不复用代码，仅参考逻辑/参数）
- `../esc_drv8300_foc`: 电机参数、MT6701 驱动逻辑、DroneCAN 栈、控制经验
