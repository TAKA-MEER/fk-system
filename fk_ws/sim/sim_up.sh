#!/usr/bin/env bash
# シミュレーション一式を起こす（WP-SIM-04。DetailedDesign-sim.md §6・段 2）。
#
# 使い方:
#   bash fk_ws/sim/sim_up.sh [シーン名]     （既定 public_room）
#   シーンは fk_ws/src/fk_sim/scenes/<名前>.yaml から探す
#
# 起きるもの:
#   - compose の sim_th（th の Gazebo。像 fk-sim-stack/th:latest）
#   - compose の sim_hve（hve の偽物と描画サービス。像 fk-sim:latest）
#   - ss（ss-system の scripts/ss.sh start。コンテナ名 ss-system）
# 止めるのは `sim_down.sh`（自分が起こしたものだけ）。
#
# 環境変数（すべて任意）:
#   TH_REPO   th-system の置き場（既定 ../th-system。import_th_sim.sh へ渡す）
#   SS_REPO   ss-system の置き場（既定 ../ss-system）
#   HVE_REPO  hve-system の置き場（既定 ../hve-system）
#   BLENDER   シーン生成に使う Blender の実行ファイル（生成するのに要る）
#
# 前提: パイプされていないこと（docker が使えること）。ROS_DOMAIN_ID は 78 固定。
set -euo pipefail

SCENE_NAME="${1:-public_room}"

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FK_REPO="$(cd "$SIM_DIR/../.." && pwd)"
FK_WS="$FK_REPO/fk_ws"
SCENE_YAML="$FK_WS/src/fk_sim/scenes/$SCENE_NAME.yaml"

export ROS_DOMAIN_ID=78

TH_REPO="${TH_REPO:-$FK_REPO/../th-system}"
SS_REPO="${SS_REPO:-$FK_REPO/../ss-system}"
HVE_REPO="${HVE_REPO:-$FK_REPO/../hve-system}"

if [ ! -f "$SCENE_YAML" ]; then
  echo "ERROR: シーンがありません: $SCENE_YAML" >&2
  exit 1
fi

# --- 1. シーンの生成（world.world・scene.glb・scene.json） ----------------------
OUT_DIR="$FK_REPO/sim_out/scenes/$SCENE_NAME"
echo "== scene 生成（$SCENE_NAME）=="
BLENDER="${BLENDER:-}"
if [ -z "$BLENDER" ] && [ -f "C:/Program Files/Blender Foundation/Blender 4.2/blender.exe" ]; then
  BLENDER="C:/Program Files/Blender Foundation/Blender 4.2/blender.exe"
fi
if [ ! -f "$OUT_DIR/scene.json" ] || [ "$SCENE_YAML" -nt "$OUT_DIR/scene.json" ]; then
  if [ -z "$BLENDER" ]; then
    echo "ERROR: Blender がありません（環境変数 BLENDER を設定して）" >&2
    exit 1
  fi
  (cd "$FK_REPO" && PYTHONPATH=fk_ws/src/fk_sim BLENDER="$BLENDER" \
    python -m fk_sim.scene build "$SCENE_YAML")
else
  echo "生成物が残っています（$OUT_DIR。如しき色は再生成しない）"
fi

python - "$OUT_DIR" "$FK_REPO" <<'PY'
import json, os, sys
scene_dir, fk_repo = sys.argv[1], sys.argv[2]
scene_file = os.path.join(scene_dir, "scene.json").replace("\\", "/")
pose = json.load(open(scene_file, encoding="utf-8"))["robot"]["pose"]
open(os.path.join(fk_repo, ".briefs/tmp/robot_pose.env"), "w", encoding="utf-8").write("ROBOT_X=" + str(pose[0]) + "\nROBOT_Y=" + str(pose[1]) + "\nROBOT_YAW=" + str(pose[2]) + "\n")
print("robot pose (scene.json):", pose)
PY
# shellcheck disable=SC1091
. "$FK_REPO/.briefs/tmp/robot_pose.env"
export ROBOT_X ROBOT_Y ROBOT_YAW

# --- 2. th の取り出し（th_repo のコミットから。静的ファイル） --------------------
echo "== th の取り出し（$TH_REPO）=="
TH_REPO="$TH_REPO" bash "$SIM_DIR/import_th_sim.sh"

# --- 3. ss のコンテナが既存なら、その場で止めて報告する（止めない） -------------
if [ -n "$(docker ps -q --filter 'name=^ss-system$' 2>/dev/null)" ]; then
  echo "ERROR: コンテナ ss-system が既に動いています。ほかの人の作業かもしれないので止めていません。" >&2
  echo "終わってからこのスクリプトを回し直してください。" >&2
  exit 1
fi

# --- 4. compose で sim_th・sim_hve を起こす -------------------------------------
export WORLD_SRC="$OUT_DIR/world.world"
export SCENE_DIR_SRC="$OUT_DIR"
export FK_REPO HVE_REPO SS_REPO

echo "== compose で sim_th・sim_hve を起こす =="
echo "注意: th の像（fk-sim-stack/th:latest）のビルドは時間がかかります（初回）"
docker compose -p fk-sim-stack -f "$SIM_DIR/docker-compose.yml" up -d

# --- 5. th と hve の準備を待つ ----------------------------------------------------
echo "== Gazebo の待ち（/set_entity_state まで）=="
docker exec sim_th bash -c '
  source /opt/ros/humble/setup.bash && source /root/th_ws/install/setup.bash &&
  export ROS_DOMAIN_ID=78 &&
  for i in $(seq 1 60); do
    if ros2 service list 2>/dev/null | grep -q set_entity_state; then exit 0; fi
    sleep 1
  done
  echo "Gazebo の set_entity_state が 60 秒で出ませんでした" >&2; exit 1'

echo "== hve の偽物（18000・18080）と描画の絵（18100）の待ち =="
docker exec sim_hve bash -c '
  for port in 18000 18080 18100; do
    for i in $(seq 1 30); do
      if (exec 3<>/dev/tcp/127.0.0.1/$port) 2>/dev/null; then exec 3>&- 3<&-; break; fi
      sleep 1
    done
  done'

# --- 6. ss を起こす（ss の起動の作りを写さない） ---------------------------------
echo "== ss を起こす（$SS_REPO/scripts/ss.sh start）=="
bash "$SS_REPO/scripts/ss.sh" start --camera-host 127.0.0.1:18000 --lift-host 127.0.0.1:18080

# --- 7. 記録を残す（sim_down.sh が読む） ------------------------------------------
mkdir -p "$FK_REPO/.briefs/tmp"
cat > "$FK_REPO/.briefs/tmp/sim_stack.env" <<EOF
SCENE_NAME=$SCENE_NAME
OUT_DIR=$OUT_DIR
SS_REPO=$SS_REPO
EOF

echo
echo "一式が起きました。確認:"
echo "  docker ps --filter name=sim_th      # gzserver が動いていること"
echo "  docker exec sim_th ros2 service list | grep get_entity_state"
echo "  ss は、コンテナ名 $SS_REPO の ss.sh status で IDLE を見る"
echo "止める: bash fk_ws/sim/sim_down.sh"
