#!/usr/bin/env bash
# Host unit tests for the src/ product layer. Pure gcc, no SDK, no driverlib.
# This is the primary correctness gate for src/encoder + src/app before the
# (later) on-target cross-compile (build.sh SRC_CHECK).
set -euo pipefail

HERE="$(cd "$(dirname "$0")/../.." && pwd)"   # repo root
SRC="$HERE/src"
TDIR="$HERE/tools/test"
CC="${CC:-gcc}"
CFLAGS="-std=c11 -Wall -Wextra -Werror -O2"
INCS=(-I"$SRC/common" -I"$SRC/encoder" -I"$SRC/app" -I"$SRC/comms" -I"$TDIR")

# --- purity check: src/ must stay free of SDK / board / driverlib headers ---
# (forbidden -> the host build would only pass because INC paths happen to be wide;
#  on target this would silently pull vendor semantics into the "pure" layer.)
echo ">>> purity check (src/ may not include SDK/board headers) ..."
if grep -REn '#[[:space:]]*include[[:space:]]*[<"](driverlib\.h|device\.h|hal\.h|user\.h)' "$SRC" \
   || grep -REn 'C2000Ware_MotorControl_SDK' "$SRC"; then
  echo "    PURITY FAIL: a forbidden SDK/board header is included under src/."
  exit 1
fi
echo "    ok (allowed: stdint.h / stdbool.h / math.h)"

SRCS=$(find "$SRC" -name '*.c' | sort)
WORK="${TMPDIR:-/tmp}/esc_host_test"
mkdir -p "$WORK"

pass=0; fail=0
shopt -s nullglob
for tsrc in "$TDIR"/test_*.c; do
  name="$(basename "$tsrc" .c)"
  bin="$WORK/$name"
  if ! "$CC" $CFLAGS "${INCS[@]}" "$tsrc" $SRCS -lm -o "$bin"; then
    echo "  BUILD FAIL  $name"; fail=$((fail+1)); continue
  fi
  if "$bin"; then
    echo "  PASS        $name"; pass=$((pass+1))
  else
    echo "  FAIL        $name"; fail=$((fail+1))
  fi
done

echo ">>> host tests: $pass passed, $fail failed"
[ "$fail" -eq 0 ]
