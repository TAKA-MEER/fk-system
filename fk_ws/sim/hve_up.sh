#!/usr/bin/env bash
# sim_hve の中で hve の偽物 3 つ（昇降部・カメラ・映像）と描画サービス
# `sim_render` を起こす（WP-SIM-03。DetailedDesign-names.md §8）。
# 止めるときは、自分が起こした子だけを止める（trap で子を拾う）。
#
# 前提（コンテナの中で）:
#   - hve-system は読み取り専用でマウントされ、HVE_REPO で指す
#     （既定 ../hve-system。ss の SS_REPO と同じ渡し方）
#   - hve の偽物の依存は /opt/hve-venv（Dockerfile）に入っている
#   - fk_ws のソースは /work/fk_ws にあり、PYTHONPATH に入っている
#   - シーンの生成物の出力（scene.glb・scene.json）は SCENE_DIR で指す
#     （既定 sim_out/scenes/public_room。リポジトリ直下からの相対）
set -uo pipefail

# sim_render は rclpy を読む（ROS の環境が要る。fk_ws/Dockerfile と同じ守り）
if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  . /opt/ros/humble/setup.bash
  set -u
fi

HVE_REPO="${HVE_REPO:-../hve-system}"
HVE_PYTHON="${HVE_PYTHON:-/opt/hve-venv/bin/python}"
SCENE_DIR="${SCENE_DIR:-sim_out/scenes/public_room}"
LIFT_PORT="${LIFT_PORT:-18080}"
CAMERA_PORT="${CAMERA_PORT:-18000}"
VIDEO_PORT="${VIDEO_PORT:-8080}"
FRAME_PORT="${FRAME_PORT:-18100}"
FIXED_X="${FIXED_X:-6.0}"
FIXED_Y="${FIXED_Y:-2.5}"
FIXED_YAW="${FIXED_YAW:-1.5708}"
POSE_SOURCE="${POSE_SOURCE:-fixed}"
ROBOT_ENTITY="${ROBOT_ENTITY:-th_robot}"

PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do
    [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait_port() {
  # wait_port HOST PORT SECONDS: 待ち受けが来るまで待つ（来なければ異常終了）
  local host="$1" port="$2" limit="$3" tries=0
  while [ "$tries" -lt "$((limit * 5))" ]; do
    if (exec 3<>"/dev/tcp/$host/$port") 2>/dev/null; then
      exec 3>&- 3<&-
      return 0
    fi
    tries=$((tries + 1))
    sleep 0.2
  done
  echo "hve_up: $host:$port が ${limit} s 開きませんでした" >&2
  return 1
}

echo "hve_up: 昇降部の偽物（$LIFT_PORT）"
"$HVE_PYTHON" "$HVE_REPO/tools/fake_lift_server.py" --port "$LIFT_PORT" &
PIDS+=($!)
wait_port 127.0.0.1 "$LIFT_PORT" 15

echo "hve_up: カメラの偽物（$CAMERA_PORT。昇降部は 127.0.0.1:$LIFT_PORT）"
(cd "$HVE_REPO/camera" && PYTHONPATH="$HVE_REPO/camera" exec "$HVE_PYTHON" -m hve_camera \
  --fake --lift "127.0.0.1:$LIFT_PORT" --port "$CAMERA_PORT") &
PIDS+=($!)
wait_port 127.0.0.1 "$CAMERA_PORT" 15

echo "hve_up: 映像の偽物（$VIDEO_PORT。絵は http://127.0.0.1:$FRAME_PORT/frame）"
(cd "$HVE_REPO/camera" && PYTHONPATH="$HVE_REPO/camera" exec "$HVE_PYTHON" -m hve_video \
  --fake --frame-url "http://127.0.0.1:$FRAME_PORT/frame" --port "$VIDEO_PORT") &
PIDS+=($!)
wait_port 127.0.0.1 "$VIDEO_PORT" 15

echo "hve_up: sim_render（絵 $FRAME_PORT。位置は $POSE_SOURCE）"
python3 -m fk_sim.sim_render --ros-args \
  -p "scene_dir:=$SCENE_DIR" \
  -p "frame_port:=$FRAME_PORT" \
  -p "camera_host:=127.0.0.1:$CAMERA_PORT" \
  -p "lift_host:=127.0.0.1:$LIFT_PORT" \
  -p "pose_source:=$POSE_SOURCE" \
  -p "robot_entity:=$ROBOT_ENTITY" \
  -p "fixed_x:=$FIXED_X" -p "fixed_y:=$FIXED_Y" -p "fixed_yaw:=$FIXED_YAW" &
PIDS+=($!)
wait_port 127.0.0.1 "$FRAME_PORT" 15

echo "hve_up: 4 つとも待ち受けています（止める時は Ctrl+C。子だけ止めます）"
wait "${PIDS[-1]}"
