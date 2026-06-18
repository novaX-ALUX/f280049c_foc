# launchxl_drv8305evm —— LAUNCHXL-F280049C + BOOSTXL-DRV8305EVM 移植清单

自制板 `esc6288_revA` 制作期间的**解耦验证平台**:用 TI 官方套件先跑通固件 + 电机。
控制核心(FOC/FAST/电机辨识/调环)与板无关、100% 复用;本目录只解决"板层"。

## 现成可复用资产(已核实)
- 引脚映射(SysConfig): `c2000ware/boards/.meta/LAUNCHXL_F280049C.syscfg.json`
  + `.../boosterpack_json_files/BOOSTXL-DRV8305EVM.syscfg.json`
- DRV8305 SPI 寄存器图: `../../esc_drv8300_foc/motorware_1_01_00_18/sw/drivers/drvic/drv8305/src/32b/f28x/f2806x/drv8305.h`
- DRV8305EVM 的 HAL 参考: `../../esc_drv8300_foc/.../hal/boards/boostxldrv8305_revA`(老 f2806x, 仅参考时序/寄存器序列)
- PWM 半边几乎现成: 本板三相 = EPWM6/5/3 = 现有 boostxl_drv8320rs HAL 的 "J1/J2 connection" 路径。

## 引脚映射(Site 1 / BoosterPack1 J1-J4) —— 已定死, 见 board.h
| 功能 | 排针 | GPIO/ADC | 外设 |
|------|:--:|:--:|------|
| U H/L | 40/39 | GPIO10/11 | EPWM6A/6B |
| V H/L | 38/37 | GPIO8/9   | EPWM5A/5B |
| W H/L | 36/35 | GPIO4/5   | EPWM3A/3B |
| Ia/Ib/Ic | 27/28/29 | ADCIN B2/C0/A9 | CMPSS1/3/5 |
| Va/Vb/Vc | 23/24/25 | ADCIN A5/B0/C2 | — |
| Vbus | 26 | ADCIN B1 | — |
| SPI CLK/STE/SIMO/SOMI | 7/19/15/14 | GPIO56/57/16/17 | SPIA |
| EN_GATE | 13 | GPIO39 (out) | active-high |
| nFAULT | 3 | GPIO13 (in) | active-low |
| WAKE | 12 | GPIO23 | — |
| PWRGD | 16 | XRSn ⚠️ | 不可用 GPIO, 改读 SPI 状态 |

## 定标常量(BOOSTXL-DRV8305EVM, CSA 增益默认 10V/V + 7mΩ 分流)
- `USER_ADC_FULL_SCALE_CURRENT_A = 47.14`  ← TI 官方值, 旧项目核实
- `USER_ADC_FULL_SCALE_VOLTAGE_V = 44.30`  ← ⚠️ 待对 EVM 分压电阻确认(VSENSE 43.2k/4.99k 量级)
- 偏置: 电流 0.5*47.14 ≈ 23.57A; Vbus 偏置见 user.h

## 阶段任务
- [x] **阶段1 骨架**: 目录 / board.h(引脚) / cmd 链接器复用 / build_config.h 加板 ID(=2)
- [x] **阶段2 HAL 适配** (drivers/{include,source}): 基于 boostxl_drv8320rs J1/J2 路径改
  - [x] PWM: EPWM6/5/3 —— 天然吻合 J1/J2 路径, 无需改
  - [x] 电流 ADC: SOC0 通道改 B2/C0/A9(`HAL_setupADCs`); getCurrent 读取索引重排成相序 A,B,C(`hal.h`)
  - [x] 电压 ADC: A5/B0/C2/B1 —— 天然吻合, 无需改
  - [x] 栅驱 GPIO: EN_GATE=GPIO39(out,低), nFAULT=GPIO13(in,上拉); GPIO40 旧 nFAULT 标记为未用; GPIO23 标为 WAKE
  - [x] 定标: 44.30V / 47.14A(`user.h`)
  - [x] `BOARD=launchxl_drv8305evm LAB=is01_intro_hal bash build.sh` 编译链接通过
  - [x] **PGA 前端**: base HAL 在 `HAL_setupPGAs` 使能 PGA1/3/5(增益12)——那是 DRV8320RS 的
    "片内 PGA"前端。DRV8305EVM 用 DRV8305 自带 CSA 直连 ADC 引脚, 且 **B2=PGA3_OF**, PGA3 使能会
    抢驱 B2 → 电流读错。已改为 **PGA_disable**(本板不走片内 PGA)。
  - ⚠️ **CMPSS 过流(已用数据手册核对, 高电流 is05+ 前接好)**: 继承的实例 5/3/1 + value=4 对本板
    **完全错误**(那是 PGA 通路)。F28004x 数据手册 Table 5-1 权威映射(直连引脚):
    | 相 | 引脚 | CMPSS | HP 索引(mux value) |
    |----|:--:|:--:|:--:|
    | Ia | B2 | CMPSS3 | HP0 (value 0) |
    | Ib | C0 | CMPSS1 | HP1 (value 1) |
    | Ic | A9 | CMPSS6 | HP3 (value 3) |
    需同步改: `cmpssHandle[]`→{CMPSS3,CMPSS1,CMPSS6}; `ASysCtl_selectCMPHPMux`→对应 HP value;
    ePWM X-BAR TRIP 复用→ MUX04(CMPSS3)/MUX00(CMPSS1)/MUX10(CMPSS6)。
    **建议**: 这套 CMPSS/X-BAR 用 TI SysConfig(CCS 导入时, README TODO#1)对 DRV8305EVM 自动生成最稳;
    手工接易错且 is01/02/03 不依赖过流, 故暂留此精确清单, 不手工硬接。
  - 备注: WithoutOffsets 读函数对本 lab 是死代码(is01/is02 只调 WithOffsets), 未改。
    J5/J6 EPWM1/2/4 引脚仍被配置但 booster 不接, 无害。
- [ ] **阶段3 DRV8305 SPI 驱动** (drivers/source/drv8305.c + include/drv8305.h)
  - 从 MotorWare drv8305.h 移植寄存器/枚举到 driverlib 风格
  - 上电序列: WAKE→EN_GATE→写 CTRL 寄存器(CSA 增益 10V/V, 死区, OC 阈值)→读 nFAULT/状态
  - build.sh 增加 `--define=DRV8305_SPI` 与源文件
- [ ] **阶段4 上电 bring-up**: is02 标定 → is03 自检(低压/小电流先) → is05 辨识 → is06/07 调环

## 待你提供(阶段4 前)
- 验证电机型号/参数: 极对数、额定电流/电压、Rs/Ls(可由 is05 辨识)
- 直流电源电压(建议先 12–24V 低压试)与限流值
- 实际插接位确认(默认 Site 1 / J1-J4;若插 J5-J8 则引脚映射换一套)
