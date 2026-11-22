#!/usr/bin/env bash
# シミュレーション一式を止める（WP-SIM-04）。
#
# 止めるのは自分が起こしたものだけ:
#   - ss（sim_up.sh が起こしたコンテナ。`scripts/ss.sh stop`）
#   - compose の sim_th・sim_hve（project 名 fk-sim-stack）
# ss のコンテナが別の人のもので（バイルダーから動いているもの）あれば、
# sim_up.sh は起動を拒否するので、ここでは一切触らない。
set -euo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FK_REPO="$(cd "$SIM_DIR/../.." && pwd)"

STATE="$FK_REPO/.briefs/tmp/sim_stack.env"
if [ -f "$STATE" ]; then
  # shellcheck disable=SC1091
  . "$STATE"
fi

echo "== ss を止める（SS_REPO=$SS_REPO）=="
if [ -n "${SS_REPO:-}" ] && [ -f "$SS_REPO/scripts/ss.sh" ]; then
  bash "$SS_REPO/scripts/ss.sh" stop || echo "ss の stop は失敗しました（続ける）" >&2
else
  echo "SS_REPO の ss.sh が無いので、ss は止めません"
fi

echo "== compose の sim_th・sim_hve を止める =="
docker compose -p fk-sim-stack -f "$SIM_DIR/docker-compose.yml" down

rm -f "$FK_REPO/.briefs/tmp/sim_stack.env" "$FK_REPO/.briefs/tmp/robot_pose.env"

echo "残っていないか確認:"
docker ps -a --filter "name=sim_th" --filter "name=sim_hve" --filter "name=ss-system"
