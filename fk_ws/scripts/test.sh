#!/usr/bin/env bash
# コンテナの中で回す: cd fk_ws && docker compose run --rm -e ROS_DOMAIN_ID=78 fk_robot bash scripts/test.sh
set -euo pipefail
cd /root/fk_ws

# 1. 取り込んだ th の型が fk_ws/th_ref と同じコミットか
if [ ! -f th_import/th_ref ] || [ "$(tr -d '[:space:]' < th_import/th_ref)" != "$(tr -d '[:space:]' < th_ref)" ]; then
  echo "ERROR: th_import/th_ref が th_ref と違う（または無い）。ホストで fk_ws/scripts/import_th.sh を回して" >&2
  exit 1
fi

# 1b. 取り込んだ ss の型が fk_ws/ss_ref と同じコミットか（th と同じ形）
if [ ! -f ss_import/ss_ref ] || [ "$(tr -d '[:space:]' < ss_import/ss_ref)" != "$(tr -d '[:space:]' < ss_ref)" ]; then
  echo "ERROR: ss_import/ss_ref が ss_ref と違う（または無い）。ホストで fk_ws/scripts/import_ss.sh を回して" >&2
  exit 1
fi

# 2. 実機の ROS_DOMAIN_ID(10) では結合試験を回さない（実機と混線する）
if [ "${ROS_DOMAIN_ID:-10}" = "10" ]; then
  echo "ERROR: ROS_DOMAIN_ID=10 は実機と混線するので試験を回さない。-e ROS_DOMAIN_ID=78 を付けて" >&2
  exit 1
fi

# 3. ビルド
set +u   # ROS の setup.bash は未設定変数を参照する
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
set -u

# 4. 試験（パッケージごとに回す。test_scene は ROS 2 なしの生成器の試験）
rc=0
for d in src/*/test src/*/test_scene src/*/test_render src/*/test_sim_render; do
  [ -d "$d" ] || continue
  echo "== pytest $d"
  python3 -m pytest -p no:cacheprovider "$d" || rc=$?
done
exit $rc
