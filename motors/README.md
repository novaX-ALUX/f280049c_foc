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
