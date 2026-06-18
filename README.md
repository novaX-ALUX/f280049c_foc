# f280049c_foc —— 全新 ESC 工程（TMS320F280049C）

基于 **C2000Ware MotorControl SDK 6.00.00.00** 的现代 FOC 架构（driverlib + EABI）。
取代旧的 `../esc_drv8300_foc`（F28027F / MotorWare，停更）和中途的 `../motorware_clean`（F28062F，已弃用）。

## 目标平台
- MCU: **TMS320F280049C**（F28004x，100 MHz，FPU + **TMU**，256KB Flash / 100KB RAM）
- 软件栈: **C2000Ware MotorControl SDK 6.0** + driverlib（**不再是 MotorWare**）
- 栅极驱动: FD6288/simple gate driver（无 SPI 寄存器配置，板级 EN GPIO 控制）
- 控制: InstaSPIN **FAST 无感**（F280049C 带 FAST ROM）+ 后续嫁接 **MT6701 有感**

## 选定底座
SDK 参考底座 `solutions/boostxl_drv8320rs/f28004x`（明确目标 = F280049C），本工程只保留需要移植的 HAL/lab 结构。
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

## 板卡（两块并存，正交于控制核心）
| 板 (`BOARD=`) | 角色 | 栅极驱动 | 电流采样 | 状态 |
|------|------|---------|---------|------|
| `esc6288_revA` | 最终 ESC（自制，制作中） | FD6288 / simple（EN GPIO，无 SPI） | 外部分流 + 运放 | is01 编译通过；板级 HAL 待按原理图迁入 `board.h` |
| `launchxl_drv8305evm` | **验证平台**（TI 官方 LaunchPad + BoosterPack） | DRV8305（SPI 可编程，内置 CSA） | DRV8305 内置 CSA → 直连 ADC | **阶段 1–3 完成**：is01 引脚全对编译通过、PGA/ADC 前端正确、DRV8305 SPI 驱动就位；阶段 4 上电待硬件 |

> 自制板回来前,用 `launchxl_drv8305evm` 先把固件 + 电机跑通(硬件解耦验证)。
> 两块板共用同一套 FOC 控制核心,只有"板层"(HAL/栅驱/定标)不同。
> 详见各自的 `boards/<板>/PORT_TODO.md`。

## 工具链（已在本机验证）
- 编译器: `cl2000` @ `~/ti/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS`（工程标 20.2.2，EABI 向前兼容）
- SDK: `./C2000Ware_MotorControl_SDK_6_00_00_00`（工程内, 已 gitignore）
- driverlib (f28004x) 预编译: `.../c2000ware/driverlib/f28004x/driverlib/ccs/Release/driverlib_eabi.lib`
- **ABI: EABI**（`--abi=eabi`）—— 现代原生，**无需** F28062F 那套 `--abi=coffabi` 老 hack
- ⚠️ 链接 SDK 库务必用 `_eabi` 后缀变体（如 `fluxHF_eabi.lib`、`f28004x_fast_rom_symbols_fpu32_eabi.lib`），
  否则误链 COFF 版会出现 `EST_*` 未解析符号。

## 已完成
- [x] SDK 6.0 安装 + F28004x 支持核实
- [x] `is01_intro_hal`（RAM/EABI）命令行编译+链接验证通过 → 合法 c28xabi ELF 镜像
- [x] `build.sh`（参数化 `BOARD × LAB`，按 BOARD 注入板 ID / 附加源，输出到 `build/<BOARD>_<LAB>/`）
- [x] **esc6288_revA**: 栅驱从 DRV8320 SPI 模板收敛为 `gate_driver.c/.h` + `board.h`；HAL GPIO 安全清理
- [x] **launchxl_drv8305evm 验证平台（阶段 1–3）**:
  - 引脚映射(Site 1/J1-J4)从 SysConfig 板文件定死 → `is01` 引脚全对编译通过
  - 模拟前端校正:电流 ADC B2/C0/A9(相序 A,B,C)、禁用片内 PGA(DRV8305 自带 CSA 直连)
  - 定标 44.30V / 47.14A;CMPSS 过流映射用数据手册核对(CMPSS3/1/6,待硬件接好)
  - DRV8305 SPI 寄存器驱动 `drv8305.c/.h`(driverlib)接入 HAL

## 待办（bring-up 顺序）
**近期(验证平台 `launchxl_drv8305evm`,阶段 4 = 上电,需实物)**
1. CCS Theia 导入工程；用 SysConfig 对 DRV8305EVM 生成 CMPSS 过流配置(见 `boards/launchxl_drv8305evm/PORT_TODO.md`)
2. DRV8305 SPI 实测(读状态/ID、确认 6-PWM 模式、示波器复核时序)
3. ADC 标定 (is02) → 硬件自检 (is03，先低压小电流) → 电机辨识 (is05) → 转矩/速度环 (is06/07)

**最终板(`esc6288_revA`,等自制板回来)**
4. 板级 HAL 移植: 把 GPIO/ADC/EPWM/CMPSS 按原理图迁入 `boards/esc6288_revA/drivers/include/board.h`
5. 嫁接 **MT6701 编码器**（有感 FOC，参考 SDK `absolute_encoder_boostxl_posmgr` / `servo_drive_with_can/sensored_foc`）
6. 嫁接 **CAN / DroneCAN**（参考 `servo_drive_with_can`）

## 构建
```bash
bash build.sh                                           # 默认 esc6288_revA / is01
BOARD=esc6288_revA        LAB=is01_intro_hal bash build.sh
BOARD=launchxl_drv8305evm LAB=is01_intro_hal bash build.sh   # 验证平台(DRV8305_SPI 自动启用)
MCSDK_ROOT=/path/to/C2000Ware_MotorControl_SDK_6_00_00_00 bash build.sh
```
- `BOARD` → `build.sh` 注入 `-DBUILD_BOARD_ID`,各 `board.h` 自检防板/构建错配。
- `launchxl_drv8305evm` 会自动追加 `drv8305.c` 与 `--define=DRV8305_SPI`。

## 硬件安全状态
- 两板的 `HAL_setupGPIOs()` 都默认把栅驱 EN 拉低(active-high,失能);上电测试需在 lab 入口按确认时序显式 `HAL_enableDRV()`。
- **esc6288_revA**: 不定义 `DRV8320_SPI`,原始 DRV8320 SPI 段不执行;EN(GPIO13)用 STD 无内部上拉(靠外部下拉 fail-safe)。
- **launchxl_drv8305evm**: EN_GATE=GPIO39(低/关)、nFAULT=GPIO13(输入);`HAL_enableDRV()` 才唤醒并配置 DRV8305。
  ⚠️ **过流保护(CMPSS)尚未接好**(映射已核对、待 SysConfig 生成)——高电流(is05+)前务必先接好,低压小电流标定/自检可先行。

## 参考蓝本（不复用代码，仅参考逻辑/参数）
- `../esc_drv8300_foc`: 电机参数、MT6701 驱动逻辑、DroneCAN 栈、控制经验

## 目录结构（按变化轴分层）
```
f280049c_foc/
├── C2000Ware_MotorControl_SDK_6_00_00_00/  # 厂商SDK, 只引用不改 (gitignore)
├── boards/                                  # 变化轴1: 硬件(MOS/栅驱/分流/布局)
│   ├── esc6288_revA/             # 最终 ESC(FD6288, 自制中): board.h/hal.c/gate_driver.c/cmd/PORT_TODO.md
│   └── launchxl_drv8305evm/      # 验证平台(DRV8305EVM): 同上 + drv8305.c/.h(SPI 驱动)
├── config/build_config.h                    # 选板 ID(BUILD_BOARD_ID); build.sh 按 BOARD 注入 + board.h 自检
├── build.sh                                 # BOARD=.. LAB=.. bash build.sh
│   # ↓↓↓ 以下为规划中的目录,当前仅占位/模板,尚未接入构建 ↓↓↓
├── src/{app,comms,encoder,common}/         # 变化轴3(规划): 内核, 与硬件/电机无关 (空占位)
└── motors/                                  # 变化轴2(规划): 电机参数(每电机一个头, 模板)
```
- 换板/换MOS → 加 `boards/<新>`；换电机 → 加 `motors/<新>`；二者正交，`src/` 不动。
- ⚠️ **现状**:`src/` 为空占位,`motors/` 仅模板 —— 电机参数暂仍在 `boards/.../user.h`(SDK 单电机模板)。
  待 is05 辨识 + is06/07 跑通后,再把电机宏从 board 抽到 `motors/`,并填充 `src/`。
- bring-up: `BOARD=esc6288_revA LAB=is01_intro_hal bash build.sh`
