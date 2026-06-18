#!/usr/bin/env bash
# 参数化构建: 选 板(BOARD) × 实验/应用(LAB)
#   BOARD=esc6288_revA LAB=is01_intro_hal bash build.sh
#   BOARD=launchxl_drv8305evm LAB=all       bash build.sh   # 冒烟编全部单电机 lab + 汇总
# 板级 HAL/驱动/链接器来自 boards/$BOARD/; FOC 库 + lab 主程序来自 SDK.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

BOARD="${BOARD:-esc6288_revA}"
LAB="${LAB:-is01_intro_hal}"
MOTOR="${MOTOR:-motor_template}"     # 预留: 后续从 motors/ 注入电机参数

# 选板: BOARD 名 -> BUILD_BOARD_ID(与 config/build_config.h + 各 board.h 自检一致)
case "$BOARD" in
  esc6288_revA)        BOARD_ID=1 ;;
  launchxl_drv8305evm) BOARD_ID=2 ;;
  *) echo "未知板 BOARD=$BOARD (见 boards/ 与 config/build_config.h)"; exit 1 ;;
esac

CGT="${CGT:-/home/patrick/ti/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS}"
MCSDK="${MCSDK_ROOT:-$HERE/C2000Ware_MotorControl_SDK_6_00_00_00}"

# LAB=all: 冒烟编译该 BOARD 全部受支持单电机 lab(排除 is11 双电机), 汇总通过/失败
if [ "$LAB" = "all" ]; then
  [ -d "$MCSDK" ] || { echo "找不到 MCSDK: $MCSDK (设 MCSDK_ROOT)"; exit 1; }
  SELF="$HERE/$(basename "$0")"
  labs=$(ls "$MCSDK/solutions/common/sensorless_foc/source/"is*.c 2>/dev/null \
         | xargs -n1 basename | sed 's/\.c$//' | grep -vx 'is11_dual_motor' | sort || true)
  [ -n "$labs" ] || { echo "未发现 lab 源 (MCSDK=$MCSDK)"; exit 1; }
  pass=0; fail=0; failed=""
  echo ">>> 冒烟编译 BOARD=$BOARD 全部单电机 lab ..."
  for L in $labs; do
    log="/tmp/buildall_${BOARD}_${L}.log"
    if BOARD="$BOARD" LAB="$L" bash "$SELF" >"$log" 2>&1; then
      printf "  OK    %-26s warnings=%s\n" "$L" "$(grep -ci warning "$log" || true)"
      pass=$((pass+1))
    else
      printf "  FAIL  %-26s (日志 %s)\n" "$L" "$log"
      fail=$((fail+1)); failed="$failed $L"
    fi
  done
  echo ">>> 汇总 [$BOARD]: $pass 过, $fail 失败.${failed:+  失败:$failed}"
  if [ "$fail" -ne 0 ]; then exit 1; fi
  exit 0
fi

DEV="$MCSDK/c2000ware/device_support/f28004x"
DLIB="$MCSDK/c2000ware/driverlib/f28004x/driverlib"
BD="$HERE/boards/$BOARD"
CL="$CGT/bin/cl2000"
OUT="$HERE/build/${BOARD}_${LAB}"; rm -rf "$OUT"; mkdir -p "$OUT"; cd "$OUT"

[ -d "$BD" ] || { echo "未知板: $BOARD (见 boards/)"; exit 1; }
[ -d "$MCSDK" ] || { echo "找不到 MCSDK: $MCSDK (设置 MCSDK_ROOT 或放在工程内)"; exit 1; }
[ -x "$CL" ] || { echo "找不到 cl2000: $CL (设置 CGT)"; exit 1; }

CFLAGS="-v28 -ml -mt --float_support=fpu32 --tmu_support=tmu0 -O2 --fp_mode=relaxed --gen_func_subsections=on --abi=eabi --display_error_number --diag_warning=225 --diag_suppress=10063"
# 选板: build.sh 按 BOARD 注入 BUILD_BOARD_ID; 各 board.h 用它自检防止板/构建错配。
DEFINES="--define=_INLINE --define=_RAM --define=_F28004x --define=DATALOG_ENABLE --define=CPUTIME_ENABLE --define=BUILD_BOARD_ID=$BOARD_ID"
INC=( -I"$MCSDK" -I"$MCSDK/libraries/control/ctrl/include" -I"$MCSDK/libraries/control/pi/include"
  -I"$MCSDK/libraries/control/vsf/include" -I"$MCSDK/libraries/control/fwc/include" -I"$MCSDK/libraries/control/mtpa/include"
  -I"$MCSDK/libraries/control/vs_freq/include" -I"$MCSDK/libraries/filter/filter_fo/include" -I"$MCSDK/libraries/filter/filter_so/include"
  -I"$MCSDK/libraries/filter/offset/include" -I"$MCSDK/libraries/observers/est/include" -I"$MCSDK/libraries/observers/mpid/include"
  -I"$MCSDK/libraries/transforms/clarke/include" -I"$MCSDK/libraries/transforms/ipark/include" -I"$MCSDK/libraries/transforms/park/include"
  -I"$MCSDK/libraries/transforms/svgen/include" -I"$MCSDK/libraries/utilities/angle_gen/include" -I"$MCSDK/libraries/utilities/cpu_time/include"
  -I"$MCSDK/libraries/utilities/datalog/include" -I"$MCSDK/libraries/utilities/diagnostic/include" -I"$MCSDK/libraries/utilities/traj/include"
  -I"$MCSDK/libraries/utilities/types/include" -I"$MCSDK/solutions/common/sensorless_foc/include/"
  -I"$HERE/config" -I"$HERE/motors" -I"$BD/drivers/include" -I"$DLIB" -I"$DEV/common/include/" -I"$DEV/headers/include/" -I"$CGT/include" )

# FOC 库源(SDK) + 板级 HAL(boards/$BOARD) + lab 主程序(SDK common)
# 注:当前为 "lab-centric" —— 主程序固定取自 SDK 的 sensorless_foc/${LAB}.c(适配 is01~is13 bring-up)。
#     待自有应用成形后,改为可切换:SDK lab 用上面路径,产品 app 取自 src/app/${LAB}.c。
C_SRCS=(
  "$DEV/headers/source/f28004x_globalvariabledefs.c"
  "$MCSDK/libraries/observers/est/source/user.c"
  "$MCSDK/libraries/control/ctrl/source/ctrl.c"
  "$MCSDK/libraries/filter/filter_fo/source/filter_fo.c"
  "$MCSDK/libraries/control/pi/source/pi.c"
  "$MCSDK/libraries/control/vs_freq/source/vs_freq.c"
  "$MCSDK/libraries/control/vsf/source/vsf.c"
  "$MCSDK/libraries/control/fwc/source/fwc.c"
  "$MCSDK/libraries/control/mtpa/source/mtpa.c"
  "$MCSDK/libraries/transforms/clarke/source/clarke.c"
  "$MCSDK/libraries/transforms/park/source/park.c"
  "$MCSDK/libraries/transforms/ipark/source/ipark.c"
  "$MCSDK/libraries/transforms/svgen/source/svgen.c"
  "$MCSDK/libraries/transforms/svgen/source/svgen_current.c"
  "$MCSDK/libraries/utilities/angle_gen/source/angle_gen.c"
  "$MCSDK/libraries/utilities/traj/source/traj.c"
  "$MCSDK/libraries/utilities/datalog/source/datalog.c"
  "$MCSDK/libraries/utilities/cpu_time/source/cpu_time.c"
  "$BD/drivers/source/gate_driver.c"
  "$BD/drivers/source/hal.c"
  "$MCSDK/solutions/common/sensorless_foc/source/${LAB}.c"
)

# 板级附加源/宏: DRV8305EVM 需要 SPI 寄存器驱动
case "$BOARD" in
  launchxl_drv8305evm)
    C_SRCS+=( "$BD/drivers/source/drv8305.c" )
    DEFINES="$DEFINES --define=DRV8305_SPI"
    ;;
esac

# lab 专属宏: 某些 lab 用 #ifdef 开启可选控制特性
case "$LAB" in
  is12_variable_pwm_frequency) DEFINES="$DEFINES --define=_VSF_EN_" ;;   # 在线变开关频率
esac

# 不支持的 lab: is11 是双电机, 需要 user_m1/m2/dm + labs_dm/hal_dm 脚手架
# (单电机收敛时已移除)。两块板均为单电机目标。
case "$LAB" in
  is11_dual_motor)
    echo "LAB=$LAB 不支持: 双电机 lab 需要已移除的 user_m1/m2/dm + hal_dm 脚手架。"
    echo "  本工程两块板(esc6288_revA / launchxl_drv8305evm)均为单电机目标。"
    exit 2 ;;
esac
ASM_SRCS=( "$DEV/common/source/f28004x_codestartbranch.asm" )
LIBS=(
  "$DLIB/ccs/Release/driverlib_eabi.lib"
  "$MCSDK/libraries/observers/fast/lib/f28004x/f28004x_fast_rom_symbols_fpu32_eabi.lib"
  "$MCSDK/libraries/observers/mpid/lib/fluxHF_eabi.lib"
)
LNK=( "$BD/cmd/f28004x_ram_cpu_is_eabi.cmd" "$DEV/headers/cmd/f28004x_headers_nonbios.cmd" )

echo ">>> BOARD=$BOARD  MOTOR=$MOTOR  LAB=$LAB  MCSDK=$MCSDK"
echo ">>> CGT=$($CL --compiler_revision)"
for s in "${C_SRCS[@]}"; do echo "  CC $(basename "$s")"; "$CL" $CFLAGS "${INC[@]}" $DEFINES -c "$s"; done
for s in "${ASM_SRCS[@]}"; do echo "  AS $(basename "$s")"; "$CL" $CFLAGS "${INC[@]}" $DEFINES -c "$s"; done
echo ">>> Linking ${LAB}.out ..."
"$CL" --abi=eabi -z --reread_libs -m "${LAB}.map" --entry_point=code_start --stack_size=0x300 \
  -i"$DLIB/math/FPUfastRTS/c28/lib" -i"$CGT/lib" \
  *.obj "${LIBS[@]}" "${LNK[@]}" -llibc.a -w -o "${LAB}.out"
echo ">>> DONE: $OUT/${LAB}.out"; ls -la "${LAB}.out"
