#!/usr/bin/env bash
# シミュレーション一式を止める（WP-SIM-04）。
#
# 止めるのは自分が起こしたものだけ:
#   - ss（sim_up.sh が起こしたコンテナ。`scripts/ss.sh stop`）
#   - compose の sim_th・sim_hve・sim_fk（project 名 fk-sim-stack）
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
# compose の補間に要る環境変数を揃える（記録が無い場合は空でない既定）
: "${FK_REPO:=$FK_REPO}"
: "${HVE_REPO:=../hve-system}"
: "${SCENE_DIR:=$FK_REPO/sim_out/scenes/public_room}"
: "${SCENE_DIR_SRC:=$FK_REPO/sim_out/scenes/public_room}"
: "${WORLD_SRC:=$SCENE_DIR/world.world}"
: "${ROBOT_X:=6.0}"
: "${ROBOT_Y:=2.5}"
: "${ROBOT_YAW:=1.5708}"
: "${SS_REPO:=../ss-system}"
: "${FK_WS:=$FK_REPO/fk_ws}"
export FK_REPO FK_WS HVE_REPO SCENE_DIR SCENE_DIR_SRC WORLD_SRC ROBOT_X ROBOT_Y ROBOT_YAW SS_REPO

echo "== ss を止める（SS_REPO=${SS_REPO:-未設定}）=="
if [ -n "${SS_REPO:-}" ] && [ -f "$SS_REPO/scripts/ss.sh" ]; then
  bash "$SS_REPO/scripts/ss.sh" stop || echo "ss の stop は失敗しました（続ける）" >&2
else
  echo "SS_REPO の ss.sh が無いので、ss は止めません"
fi

echo "== compose の sim_th・sim_hve・sim_fk を止める =="
docker compose -p fk-sim-stack -f "$SIM_DIR/docker-compose.yml" down

rm -f "$FK_REPO/.briefs/tmp/sim_stack.env" "$FK_REPO/.briefs/tmp/robot_pose.env"

echo "残っていないか確認:"
docker ps -a --filter "name=sim_th" --filter "name=sim_hve" --filter "name=sim_fk" --filter "name=ss-system"
