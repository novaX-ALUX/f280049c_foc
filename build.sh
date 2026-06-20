#!/usr/bin/env bash
# Parameterized build: select board (BOARD) x motor (MOTOR) x lab/app (LAB)
#   BOARD=esc6288_revA LAB=is01_intro_hal bash build.sh
#   BOARD=launchxl_drv8305evm LAB=all       bash build.sh   # smoke-build every single-motor lab + summary
#   BOARD=esc6288_revA MOTOR=am_4116_kva SRC_CHECK=1 bash build.sh  # cross-compile src/ product modules only
# Board-level HAL/drivers/linker come from boards/$BOARD/; FOC libraries + lab main come from the SDK.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

BOARD="${BOARD:-esc6288_revA}"
LAB="${LAB:-is01_intro_hal}"
MOTOR="${MOTOR:-motor_template}"     # select motor profile (motors/); default is the SDK example motor
SRC_CHECK="${SRC_CHECK:-0}"          # 1 = cross-compile src/ product modules only (no link), then exit

# Motor selection: MOTOR name -> BUILD_MOTOR_ID (must match config/build_config.h + motors/motor_select.h)
case "$MOTOR" in
  motor_template) MOTOR_ID=1 ;;
  am_4116_kva)    MOTOR_ID=2 ;;
  am_4116_kvb)    MOTOR_ID=3 ;;
  am_6212)        MOTOR_ID=4 ;;
  am_6215)        MOTOR_ID=5 ;;
  *) echo "Unknown motor MOTOR=$MOTOR (see motors/ and config/build_config.h)"; exit 1 ;;
esac

# Board selection: BOARD name -> BUILD_BOARD_ID (must match config/build_config.h + each board.h self-check)
case "$BOARD" in
  esc6288_revA)        BOARD_ID=1 ;;
  launchxl_drv8305evm) BOARD_ID=2 ;;
  *) echo "Unknown board BOARD=$BOARD (see boards/ and config/build_config.h)"; exit 1 ;;
esac

# TI C2000 compiler (CGT). Priority: $CGT env > validated LTS > newest installed > legacy default.
# Flight/production firmware pins to the validated LTS toolchain for qualified, reproducible builds
# (newer CGT versions emit benign vendor-driverlib-header warnings and aren't requalified here).
# Auto-detect only falls back to the newest installed CGT when the validated one is absent.
CGT_PIN="ti-cgt-c2000_22.6.0.LTS"   # validated toolchain for this project
if [ -z "${CGT:-}" ]; then
  # 1) prefer the validated version wherever it is installed
  for c in "$HOME"/ti/ccs*/tools/compiler/"$CGT_PIN" \
           "$HOME"/ti/ccs*/ccs/tools/compiler/"$CGT_PIN" \
           "$HOME"/ti/"$CGT_PIN" \
           /opt/ti/ccs*/tools/compiler/"$CGT_PIN" /opt/ti/"$CGT_PIN"; do
    [ -x "$c/bin/cl2000" ] && CGT="$c" && break
  done
  # 2) else newest installed CGT (portability; may emit benign newer-compiler vendor warnings)
  if [ -z "${CGT:-}" ]; then
    for c in $(ls -d \
          "$HOME"/ti/ccs*/tools/compiler/ti-cgt-c2000_* \
          "$HOME"/ti/ccs*/ccs/tools/compiler/ti-cgt-c2000_* \
          "$HOME"/ti/ti-cgt-c2000_* \
          /opt/ti/ccs*/tools/compiler/ti-cgt-c2000_* \
          /opt/ti/ti-cgt-c2000_* 2>/dev/null | sort -Vr); do
      [ -x "$c/bin/cl2000" ] && CGT="$c" && break
    done
  fi
fi
CGT="${CGT:-/home/patrick/ti/ccs/tools/compiler/$CGT_PIN}"  # legacy fallback
MCSDK="${MCSDK_ROOT:-$HERE/C2000Ware_MotorControl_SDK_6_00_00_00}"

# --- common toolchain paths + compiler flags (shared by SRC_CHECK and the lab builds) ---
DEV="$MCSDK/c2000ware/device_support/f28004x"
DLIB="$MCSDK/c2000ware/driverlib/f28004x/driverlib"
BD="$HERE/boards/$BOARD"
CL="$CGT/bin/cl2000"

[ -d "$BD" ] || { echo "Unknown board: $BOARD (see boards/)"; exit 1; }
[ -d "$MCSDK" ] || { echo "MCSDK not found: $MCSDK (set MCSDK_ROOT or place it inside the project)"; exit 1; }
[ -x "$CL" ] || { echo "cl2000 not found: $CL"; echo "  Install the TI C2000 compiler (CCS or standalone CGT), or set CGT=/path/to/ti-cgt-c2000_<ver>"; exit 1; }

CFLAGS="-v28 -ml -mt --float_support=fpu32 --tmu_support=tmu0 -O2 --fp_mode=relaxed --gen_func_subsections=on --abi=eabi --display_error_number --diag_warning=225 --diag_suppress=10063"
# Board selection: build.sh injects BUILD_BOARD_ID per BOARD; each board.h uses it to self-check against board/build mismatch.
DEFINES="--define=_INLINE --define=_RAM --define=_F28004x --define=DATALOG_ENABLE --define=CPUTIME_ENABLE --define=BUILD_BOARD_ID=$BOARD_ID --define=BUILD_MOTOR_ID=$MOTOR_ID"
INC=( -I"$MCSDK" -I"$MCSDK/libraries/control/ctrl/include" -I"$MCSDK/libraries/control/pi/include"
  -I"$MCSDK/libraries/control/vsf/include" -I"$MCSDK/libraries/control/fwc/include" -I"$MCSDK/libraries/control/mtpa/include"
  -I"$MCSDK/libraries/control/vs_freq/include" -I"$MCSDK/libraries/filter/filter_fo/include" -I"$MCSDK/libraries/filter/filter_so/include"
  -I"$MCSDK/libraries/filter/offset/include" -I"$MCSDK/libraries/observers/est/include" -I"$MCSDK/libraries/observers/mpid/include"
  -I"$MCSDK/libraries/transforms/clarke/include" -I"$MCSDK/libraries/transforms/ipark/include" -I"$MCSDK/libraries/transforms/park/include"
  -I"$MCSDK/libraries/transforms/svgen/include" -I"$MCSDK/libraries/utilities/angle_gen/include" -I"$MCSDK/libraries/utilities/cpu_time/include"
  -I"$MCSDK/libraries/utilities/datalog/include" -I"$MCSDK/libraries/utilities/diagnostic/include" -I"$MCSDK/libraries/utilities/traj/include"
  -I"$MCSDK/libraries/utilities/types/include" -I"$MCSDK/solutions/common/sensorless_foc/include/"
  -I"$HERE/config" -I"$HERE/motors" -I"$BD/drivers/include" -I"$DLIB" -I"$DEV/common/include/" -I"$DEV/headers/include/" -I"$CGT/include"
  -I"$HERE/src/common" -I"$HERE/src/encoder" -I"$HERE/src/app" )

# --- SRC_CHECK=1: cross-compile ONLY the src/ product modules (no link), as a 0-warning gate. ---
# Deliberately placed AFTER CFLAGS/DEFINES/INC are built and BEFORE the LAB=all / single-lab paths,
# so "SRC_CHECK=1 LAB=all bash build.sh" runs the src check and does NOT fall into the LAB=all loop.
# cl2000 does not fail on warnings, so we capture each file's compile log and treat any real
# compiler-warning line (TI "warning #" / GCC-style "warning:") in the LOG as failure.
if [ "$SRC_CHECK" = "1" ]; then
  echo ">>> SRC_CHECK: cross-compiling src/ product modules (BOARD=$BOARD MOTOR=$MOTOR), no link ..."
  mapfile -t src_files < <(find "$HERE/src" -name '*.c' | sort)
  if [ "${#src_files[@]}" -eq 0 ]; then
    echo ">>> SRC_CHECK: no .c files under src/ yet -- nothing to cross-compile."; exit 0
  fi
  OUT="$HERE/build/_srccheck/${BOARD}/${MOTOR}"; rm -rf "$OUT"; mkdir -p "$OUT"; cd "$OUT"
  warns=0; fails=0
  for s in "${src_files[@]}"; do
    log="$OUT/$(basename "$s").log"
    if "$CL" $CFLAGS "${INC[@]}" $DEFINES -c "$s" >"$log" 2>&1; then
      n=$(grep -cE 'warning #|warning:' "$log" || true)
      printf "  CC %-22s warnings=%s\n" "$(basename "$s")" "$n"
      warns=$((warns + n))
      [ "$n" -ne 0 ] && cat "$log"
    else
      printf "  FAIL %-20s (see %s)\n" "$(basename "$s")" "$log"; cat "$log"
      fails=$((fails + 1))
    fi
  done
  if [ "$fails" -ne 0 ]; then echo ">>> SRC_CHECK FAILED [$BOARD/$MOTOR]: $fails file(s) did not compile."; exit 1; fi
  if [ "$warns" -ne 0 ]; then echo ">>> SRC_CHECK FAILED [$BOARD/$MOTOR]: $warns warning(s) (0-warning gate)."; exit 1; fi
  echo ">>> SRC_CHECK OK [$BOARD/$MOTOR]: ${#src_files[@]} module(s) cross-compile clean (0 warnings)."
  exit 0
fi

# --- LAB=all: smoke-build every supported single-motor lab for this BOARD (excluding the is11 dual-motor lab) ---
if [ "$LAB" = "all" ]; then
  SELF="$HERE/$(basename "$0")"
  labs=$(ls "$MCSDK/solutions/common/sensorless_foc/source/"is*.c 2>/dev/null \
         | xargs -n1 basename | sed 's/\.c$//' | grep -vx 'is11_dual_motor' | sort || true)
  [ -n "$labs" ] || { echo "No lab sources found (MCSDK=$MCSDK)"; exit 1; }
  pass=0; fail=0; failed=""
  echo ">>> Smoke-building BOARD=$BOARD MOTOR=$MOTOR, all single-motor labs ..."
  for L in $labs; do
    log="/tmp/buildall_${BOARD}_${MOTOR}_${L}.log"
    if BOARD="$BOARD" MOTOR="$MOTOR" LAB="$L" SRC_CHECK=0 bash "$SELF" >"$log" 2>&1; then
      printf "  OK    %-26s warnings=%s\n" "$L" "$(grep -ci warning "$log" || true)"
      pass=$((pass+1))
    else
      printf "  FAIL  %-26s (log %s)\n" "$L" "$log"
      fail=$((fail+1)); failed="$failed $L"
    fi
  done
  echo ">>> Summary [$BOARD/$MOTOR]: $pass passed, $fail failed.${failed:+  failed:$failed}"
  if [ "$fail" -ne 0 ]; then exit 1; fi
  exit 0
fi

# --- single-lab build ---
# Output is laid out per board/motor/lab: build/<BOARD>/<MOTOR>/<LAB>/
# (MOTOR is a real build dimension; leaving it out of the path lets different motors' .out/.map overwrite each other -> wrong firmware flashed)
OUT="$HERE/build/${BOARD}/${MOTOR}/${LAB}"; rm -rf "$OUT"; mkdir -p "$OUT"; cd "$OUT"

# FOC library sources (SDK) + board HAL (boards/$BOARD) + lab main (SDK common)
# Note: currently "lab-centric" -- the main always comes from the SDK's sensorless_foc/${LAB}.c (suited to is01~is13 bring-up).
#       Once a proprietary app takes shape, make this switchable: SDK labs use the path above, product app comes from src/app/${LAB}.c.
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

# Board-specific extra sources/defines: DRV8305EVM needs the SPI register driver
case "$BOARD" in
  launchxl_drv8305evm)
    C_SRCS+=( "$BD/drivers/source/drv8305.c" )
    DEFINES="$DEFINES --define=DRV8305_SPI"
    ;;
esac

# Lab-specific defines: some labs enable optional control features via #ifdef
case "$LAB" in
  is12_variable_pwm_frequency) DEFINES="$DEFINES --define=_VSF_EN_" ;;   # online variable switching frequency
esac

# Unsupported lab: is11 is dual-motor, needs the user_m1/m2/dm + labs_dm/hal_dm scaffolding
# (removed during the single-motor convergence). Both boards are single-motor targets.
case "$LAB" in
  is11_dual_motor)
    echo "LAB=$LAB unsupported: the dual-motor lab needs the removed user_m1/m2/dm + hal_dm scaffolding."
    echo "  Both boards in this project (esc6288_revA / launchxl_drv8305evm) are single-motor targets."
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
