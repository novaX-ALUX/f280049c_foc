# motors/ —— 电机参数变体（变化轴 2）
每个电机一个头文件，集中放该电机的 FOC 参数：极对数、Rs、Ls(d/q)、磁链/Ke、
额定电流/电压、最大转速、惯量等。换电机=换这里一个头，不动 app/ 和 boards/。

## 选择机制（已接入构建）
- `MOTOR=<型号> bash build.sh` → 注入 `-DBUILD_MOTOR_ID` → `motor_select.h` 引入对应 profile。
- board user.h 在 SDK 示例电机链外包了 `#if (BUILD_MOTOR_ID==TEMPLATE)`:
  - 默认 `motor_template`:走 SDK 示例电机(Teknic),与历史行为一致。
  - 选真实电机:示例链跳过,由 `motors/<型号>.h` 提供全部 `USER_MOTOR_*`。
- ID 注册在 `config/build_config.h` + `build.sh` 的 `MOTOR` case。两板均验证(20/20 通过)。

## 待支持的电机（自研 NovaX/AM 系列，共 4 款）
4 个 profile 已建并可选。**几何极对数已填**(4116=7, 62xx=14);
**Rs/Ls/磁链现为 bench 种子**(能编译、能跑 is05), 上电辨识后回填覆盖:
- [ ] `am_4116_kva.h`（KV-A，7 极对）— KV 待填,is05 回填 Rs/Ls/磁链
- [ ] `am_4116_kvb.h`（KV-B，与 A 不同 KV，7 极对）
- [ ] `am_6212.h`（14 极对）
- [ ] `am_6215.h`（14 极对）

> 宏名对齐 SDK 6.0 `USER_MOTOR_*`。运行范围字段(FREQ/VOLT/额定)与惯量现为占位,按电机+电源调。
> 完全解耦(board 只留硬件标定宏、彻底不带电机示例)可后续再清, 现以"包裹跳过"实现已足够。

> 老工程（`../esc_drv8300_foc`）里有这些电机的旧参数，但 6212/6215 的 profile 有已知混淆、
> 4116 flux 也有偏差 —— **不直接复用,统一以 is05 在本工程实物上重新辨识为准**。

## 标准流程：选电机 → is05 辨识 → 回填 → 调环
以 `am_6215` 在验证板上为例(其余电机同理,换 `MOTOR=`):

0. **上电前确认**(安全):极对数对(几何)、`MAX_CURRENT_A`/`RES_EST`/`IND_EST` 在电源限流内、
   CMPSS 过流已接(高电流前,见板 PORT_TODO)、电源先低压(12–24V)。

1. **辨识**:
   ```bash
   BOARD=launchxl_drv8305evm MOTOR=am_6215 LAB=is05_motor_id bash build.sh
   ```
   烧录运行,读 FAST 估出的 `Rs / Ls_d / Ls_q / 磁链(Flux)`(看 watch 变量 / datalog)。

2. **回填**:把辨识值写回 `motors/am_6215.h`,**覆盖** bench 种子那几行(`Rs_Ohm / Ls_d_H /
   Ls_q_H / RATED_FLUX_VpHz`),并把该行注释从“种子, is05 覆盖”改成“is05 实测 yyyy-mm-dd”。
   顺手填 `MOTOR_KV_RPM_PER_V`。勾掉本文件上面清单里的 `[ ]`。

3. **验证电流环**:
   ```bash
   BOARD=launchxl_drv8305evm MOTOR=am_6215 LAB=is06_torque_control bash build.sh
   ```
   小 Iq 自由轴试转,确认电流环稳、相序对(不反转/不失步)。

4. **速度环 + 调参**:
   ```bash
   BOARD=launchxl_drv8305evm MOTOR=am_6215 LAB=is07_speed_control bash build.sh
   ```
   调 `INERTIA_Kgm2` 与速度环 Kp/Ki;按实测把 `FREQ_MAX_HZ`/`RATED_*`/`VOLT_*` 收敛到真实范围。

5. 每改完一个电机, `git commit` 该 profile(标注 is05 日期 + 电源/温度条件)。

> 回归:任何改动后 `BOARD=<板> LAB=all bash build.sh` 应仍 12/12、0 告警(默认 template,不受 profile 影响)。
