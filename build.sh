#!/usr/bin/env bash
# 参数化构建: 选 板(BOARD) × 实验/应用(LAB)
#   BOARD=esc6288_revA LAB=is01_intro_hal bash build.sh
# 板级 HAL/驱动/链接器来自 boards/$BOARD/; FOC 库 + lab 主程序来自 SDK.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

BOARD="${BOARD:-esc6288_revA}"
LAB="${LAB:-is01_intro_hal}"
MOTOR="${MOTOR:-}"     # 预留: 后续从 motors/ 注入电机参数(当前电机参数仍在 board 的 user_m1.h)

CGT="${CGT:-/home/patrick/ti/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS}"
MCSDK="$HERE/C2000Ware_MotorControl_SDK_6_00_00_00"
DEV="$MCSDK/c2000ware/device_support/f28004x"
DLIB="$MCSDK/c2000ware/driverlib/f28004x/driverlib"
BD="$HERE/boards/$BOARD"
CL="$CGT/bin/cl2000"
OUT="$HERE/build/${BOARD}_${LAB}"; rm -rf "$OUT"; mkdir -p "$OUT"; cd "$OUT"

[ -d "$BD" ] || { echo "未知板: $BOARD (见 boards/)"; exit 1; }

CFLAGS="-v28 -ml -mt --float_support=fpu32 --tmu_support=tmu0 -O2 --fp_mode=relaxed --gen_func_subsections=on --abi=eabi --display_error_number --diag_warning=225 --diag_suppress=10063"
DEFINES="--define=_INLINE --define=_RAM --define=_F28004x --define=_BOOSTXL_8320RS_REVA_ --define=DRV8320_SPI --define=DATALOG_ENABLE --define=CPUTIME_ENABLE"
INC=( -I"$MCSDK" -I"$MCSDK/libraries/control/ctrl/include" -I"$MCSDK/libraries/control/pi/include"
  -I"$MCSDK/libraries/control/vsf/include" -I"$MCSDK/libraries/control/fwc/include" -I"$MCSDK/libraries/control/mtpa/include"
  -I"$MCSDK/libraries/control/vs_freq/include" -I"$MCSDK/libraries/filter/filter_fo/include" -I"$MCSDK/libraries/filter/filter_so/include"
  -I"$MCSDK/libraries/filter/offset/include" -I"$MCSDK/libraries/observers/est/include" -I"$MCSDK/libraries/observers/mpid/include"
  -I"$MCSDK/libraries/transforms/clarke/include" -I"$MCSDK/libraries/transforms/ipark/include" -I"$MCSDK/libraries/transforms/park/include"
  -I"$MCSDK/libraries/transforms/svgen/include" -I"$MCSDK/libraries/utilities/angle_gen/include" -I"$MCSDK/libraries/utilities/cpu_time/include"
  -I"$MCSDK/libraries/utilities/datalog/include" -I"$MCSDK/libraries/utilities/diagnostic/include" -I"$MCSDK/libraries/utilities/traj/include"
  -I"$MCSDK/libraries/utilities/types/include" -I"$MCSDK/solutions/common/sensorless_foc/include/"
  -I"$BD/drivers/include" -I"$DLIB" -I"$DEV/common/include/" -I"$DEV/headers/include/" -I"$CGT/include" )

# FOC 库源(SDK) + 板级 HAL(boards/$BOARD) + lab 主程序(SDK common)
C_SRCS=(
  "$DEV/headers/source/f28004x_globalvariabledefs.c"
  "$MCSDK/libraries/observers/est/source/user.c"
  "$MCSDK/libraries/control/ctrl/source/ctrl.c"
  "$MCSDK/libraries/filter/filter_fo/source/filter_fo.c"
  "$MCSDK/libraries/control/pi/source/pi.c"
  "$MCSDK/libraries/transforms/clarke/source/clarke.c"
  "$MCSDK/libraries/transforms/park/source/park.c"
  "$MCSDK/libraries/transforms/ipark/source/ipark.c"
  "$MCSDK/libraries/transforms/svgen/source/svgen.c"
  "$MCSDK/libraries/transforms/svgen/source/svgen_current.c"
  "$MCSDK/libraries/utilities/angle_gen/source/angle_gen.c"
  "$MCSDK/libraries/utilities/traj/source/traj.c"
  "$MCSDK/libraries/utilities/datalog/source/datalog.c"
  "$MCSDK/libraries/utilities/cpu_time/source/cpu_time.c"
  "$BD/drivers/source/drv8320.c"
  "$BD/drivers/source/hal.c"
  "$MCSDK/solutions/common/sensorless_foc/source/${LAB}.c"
)
ASM_SRCS=( "$DEV/common/source/f28004x_codestartbranch.asm" )
LIBS=(
  "$DLIB/ccs/Release/driverlib_eabi.lib"
  "$MCSDK/libraries/observers/fast/lib/f28004x/f28004x_fast_rom_symbols_fpu32_eabi.lib"
  "$MCSDK/libraries/observers/mpid/lib/fluxHF_eabi.lib"
)
LNK=( "$BD/cmd/f28004x_ram_cpu_is_eabi.cmd" "$DEV/headers/cmd/f28004x_headers_nonbios.cmd" )

echo ">>> BOARD=$BOARD  LAB=$LAB  CGT=$($CL --compiler_revision)"
for s in "${C_SRCS[@]}"; do echo "  CC $(basename "$s")"; "$CL" $CFLAGS "${INC[@]}" $DEFINES -c "$s"; done
for s in "${ASM_SRCS[@]}"; do echo "  AS $(basename "$s")"; "$CL" $CFLAGS "${INC[@]}" $DEFINES -c "$s"; done
echo ">>> Linking ${LAB}.out ..."
"$CL" --abi=eabi -z --reread_libs -m "${LAB}.map" --entry_point=code_start --stack_size=0x300 \
  -i"$DLIB/math/FPUfastRTS/c28/lib" -i"$CGT/lib" \
  *.obj "${LIBS[@]}" "${LNK[@]}" -llibc.a -w -o "${LAB}.out"
echo ">>> DONE: $OUT/${LAB}.out"; ls -la "${LAB}.out"
