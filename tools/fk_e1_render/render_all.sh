#!/usr/bin/env bash
# FK-E1: コンテナの中で、書き出した全シーンを R-PY または R-GZ で描く。
# 使い方: bash tools/fk_e1_render/render_all.sh py|gz
set -u

MODE="${1:?usage: render_all.sh py|gz}"
ROOT=/work/sim_out/fk_e1
TOOLS=/work/tools/fk_e1_render
mkdir -p "$ROOT/logs"
LOG="$ROOT/logs/render_${MODE}.log"
: > "$LOG"

fail=0
for d in "$ROOT"/export/*/*; do
  [ -d "$d" ] || continue
  combo="$(basename "$(dirname "$d")")"
  stem="$(basename "$d")"
  out="$ROOT/render_${MODE}/${combo}/${stem}"
  echo "=== $MODE $combo/$stem ===" | tee -a "$LOG"
  if [ "$MODE" = "py" ]; then
    PYOPENGL_PLATFORM=osmesa python3 "$TOOLS/render_py.py" --export-dir "$d" --out "$out" 2>&1 | tee -a "$LOG"
  else
    xvfb-run -a -s "-screen 0 1280x720x24" python3 "$TOOLS/render_gz.py" --export-dir "$d" --out "$out" --frames 10 --mesh scene.obj 2>&1 | tee -a "$LOG"
  fi
  if [ "${PIPESTATUS[0]}" -ne 0 ]; then
    echo "FAIL $MODE $combo/$stem" | tee -a "$LOG"
    fail=$((fail + 1))
  fi
done
echo "failures: $fail" | tee -a "$LOG"
exit 0
