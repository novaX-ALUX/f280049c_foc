#!/usr/bin/env bash
set -euo pipefail
CGT="/home/patrick/ti/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS"
MCSDK="/home/patrick/ti/C2000Ware_MotorControl_SDK_6_00_00_00"
DEV="$MCSDK/c2000ware/device_support/f28004x"
DLIB="$MCSDK/c2000ware/driverlib/f28004x/driverlib"
CL="$CGT/bin/cl2000"
OUT="$(dirname "$0")/build/is01"; rm -rf "$OUT"; mkdir -p "$OUT"; cd "$OUT"

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
  -I"$MCSDK/solutions/boostxl_drv8320rs/f28004x/drivers/include" -I"$DLIB" -I"$DEV/common/include/" -I"$DEV/headers/include/" -I"$CGT/include" )

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
  "$MCSDK/solutions/boostxl_drv8320rs/f28004x/drivers/source/drv8320.c"
  "$MCSDK/solutions/boostxl_drv8320rs/f28004x/drivers/source/hal.c"
  "$MCSDK/solutions/common/sensorless_foc/source/is01_intro_hal.c"
)
ASM_SRCS=( "$DEV/common/source/f28004x_codestartbranch.asm" )
LIBS=(
  "$DLIB/ccs/Release/driverlib_eabi.lib"
  "$MCSDK/libraries/observers/fast/lib/f28004x/f28004x_fast_rom_symbols_fpu32_eabi.lib"
  "$MCSDK/libraries/observers/mpid/lib/fluxHF_eabi.lib"
)
LNK=( "$MCSDK/solutions/boostxl_drv8320rs/f28004x/cmd/f28004x_ram_cpu_is_eabi.cmd"
      "$DEV/headers/cmd/f28004x_headers_nonbios.cmd" )

echo ">>> CGT: $($CL --compiler_revision)"
for s in "${C_SRCS[@]}"; do echo "  CC $(basename "$s")"; "$CL" $CFLAGS "${INC[@]}" $DEFINES -c "$s"; done
for s in "${ASM_SRCS[@]}"; do echo "  AS $(basename "$s")"; "$CL" $CFLAGS "${INC[@]}" $DEFINES -c "$s"; done
echo ">>> Linking is01.out ..."
"$CL" --abi=eabi -z --reread_libs -m is01.map --entry_point=code_start --stack_size=0x300 \
  -i"$DLIB/math/FPUfastRTS/c28/lib" -i"$CGT/lib" \
  *.obj "${LIBS[@]}" "${LNK[@]}" -llibc.a -w -o is01.out
echo ">>> DONE:"; ls -la is01.out
