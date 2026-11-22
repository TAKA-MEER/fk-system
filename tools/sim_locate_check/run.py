#!/usr/bin/env python3
"""Gazebo の機体を置き直して ss に依頼し、結果を scene.json の正解と比べる。

使い方:

    python tools/sim_locate_check/run.py --x 5.0 --y 4.0 --yaw 1.5708 \\
        --scene sim_out/scenes/public_room --from-deg -180 --span-deg 360

前提:
  - `sim_up.sh` で一式が起きていること（`sim_th`・`sim_hve`・`ss-system`）
  - ss の `scripts/ss.sh`（環境変数 `SS_REPO`。既定は `../ss-system`）

流れ:
  1. `docker exec sim_th ros2 service call /set_entity_state ...` で機体を置き直す
  2. 2 秒待ってから ss に依頼する
  3. 結果（`base_link` 座標）と正解（ワールド座標）を `pose.py` で比べ、
     盤ごとに中心のずれ・向きのずれ・四隅のずれ・距離を表で出す

求める精度（ではない、今の観測）:
  `docs/plan/spec/Spec-open.md` §2.4 の「ss-system に求める精度」：
  2.6 m 以内は四隅 5 cm 以内、2.6〜5 m は中心 ±25 cm・向き ±10°。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from pose import panel_compare  # noqa: E402


def _run(cmd, env=None):
    full_env = dict(os.environ)
    if env:
        full_env.update(env)
    print("+ " + " ".join(cmd), file=sys.stderr)
    return subprocess.run(cmd, env=full_env, capture_output=True, text=True)


def _yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def set_robot_pose(container, x, y, yaw, name="th_robot"):
    """Gazebo の `/set_entity_state` で機体を置き直す（`gazebo_ros_state`）。"""
    qx, qy, qz, qw = _yaw_to_quaternion(yaw)
    request = (
        "{state: {name: '%s', pose: {position: {x: %.6f, y: %.6f, z: 0.0}, "
        "orientation: {x: %.6f, y: %.6f, z: %.6f, w: %.6f}}}}"
        % (name, x, y, qx, qy, qz, qw)
    )
    inner = (
        "source /opt/ros/humble/setup.bash && source /root/th_ws/install/setup.bash && "
        "timeout 20 ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \"%s\""
        % request.replace('"', '\\"')
    )
    result = _run(["docker", "exec", container, "bash", "-c", inner],
                  env={"ROS_DOMAIN_ID": "78"})
    if result.returncode != 0:
        raise RuntimeError(f"機体の置き直しに失敗した:\n{result.stdout}\n{result.stderr}")
    return result.stdout.strip()


def request_locate(ss_repo, ss_args, out_json):
    """ss の `scripts/ss.sh locate` で依頼し、結果の JSON を読む。"""
    script = os.path.join(ss_repo, "scripts", "ss.sh")
    cmd = ["bash", script, "locate", "--yes", "--json", out_json] + list(ss_args)
    result = _run(cmd, env={"ROS_DOMAIN_ID": "78"})
    if result.returncode != 0:
        raise RuntimeError(f"ss への依頼に失敗した:\n{result.stdout}\n{result.stderr}")
    # ss.sh はコンテナ内の相対パス（/ws 相対）に書く。ホスト側は ss_repo を起点として読む。
    host_path = os.path.join(ss_repo, out_json)
    if not os.path.exists(host_path):
        raise RuntimeError(f"ss の結果が書かれなかった: {host_path}")
    with open(host_path, encoding="utf-8") as f:
        return json.load(f)


def _load_scene(scene_dir):
    path = os.path.join(scene_dir, "scene.json")
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def compare(result, scene, robot_pose):
    """盤ごとに並べ替えた距離の近いものから真の盤と比べる。

    ss の結果に `id` が無いので、真の盤の中で中心の距離が最も近いものと比べる。
    """
    truths = scene["panels"]
    rows = []
    for ss_panel in result.get("panels", []):
        ss_center = (float(ss_panel["x_m"]), float(ss_panel["y_m"]))
        from pose import base_link_to_world
        world_center = base_link_to_world(ss_center, robot_pose)
        best_truth, best_dist = None, None
        for truth in truths:
            d = math.hypot(world_center[0] - truth["center"][0],
                           world_center[1] - truth["center"][1])
            if best_dist is None or d < best_dist:
                best_truth, best_dist = truth, d
        if best_truth is None:
            continue
        cmp_ = panel_compare(ss_panel, best_truth, robot_pose)
        rows.append({
            "truth_id": best_truth.get("id"),
            "label": best_truth.get("label"),
            "center_err_m": cmp_["center_err_m"],
            "yaw_err_rad": cmp_["yaw_err_rad"],
            "corner_errs_m": cmp_["corner_errs_m"],
            "distance_m": cmp_["distance_m"],
            "truth_center": best_truth["center"],
            "x_m": ss_panel["x_m"], "y_m": ss_panel["y_m"],
            "yaw_rad": ss_panel["yaw_rad"],
            "width_m": ss_panel["width_m"], "depth_m": ss_panel["depth_m"],
            "front_known": ss_panel.get("front_known"),
            "front_yaw_rad": ss_panel.get("front_yaw_rad"),
        })
    rows.sort(key=lambda r: (r["distance_m"], r["truth_id"]))
    return rows


def print_table(rows):
    def fmt(v):
        return "-" if v is None else f"{v:.3f}"

    print(f"{'真盤':<6} {'距離m':>7} {'中心のずれm':>11} {'向きのずれrad':>13} "
          f"{'四隅の最大m':>11} {'四隅の平均m':>11} {'pos_std':>8}")
    for r in rows:
        corner = r["corner_errs_m"]
        max_corner = max(corner) if corner else None
        mean_corner = (sum(corner) / len(corner)) if corner else None
        print(f"{str(r['truth_id']):<6} {r['distance_m']:7.2f} {r['center_err_m']:11.3f} "
              f"{fmt(r['yaw_err_rad']):>13} {fmt(max_corner):>11} {fmt(mean_corner):>11}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x", type=float, required=True, help="機体の置き直し先 x [m]")
    parser.add_argument("--y", type=float, required=True, help="機体の置き直し先 y [m]")
    parser.add_argument("--yaw", type=float, required=True, help="機体の向き [rad]")
    parser.add_argument("--scene", required=True, help="シーンの生成物のディレクトリ")
    parser.add_argument("--container", default="sim_th", help="th のコンテナ（既定 sim_th）")
    parser.add_argument("--out", default=None, help="ss の結果 JSON の書き出し先（省略時は自動）")
    parser.add_argument("--sleep", type=float, default=2.0, help="置き直しから依頼までの待ち [s]")
    parser.add_argument("--locate-args", nargs=argparse.REMAINDER,
                        help="ss へ渡す引数（--from-deg など）")
    args = parser.parse_args(argv)

    scene_dir = os.path.abspath(args.scene)
    scene = _load_scene(scene_dir)
    # ss の結果は ss.sh locate の保存先（コンテナ内の /ws 相対）に書く。
    out_json = (args.out or "out/sim_check.json").replace("\\", "/")
    os.makedirs(os.path.dirname(out_json) or ".", exist_ok=True)
    ss_repo = os.environ.get("SS_REPO") or os.path.join(
        os.path.dirname(_HERE), "..", "..", "ss-system")
    ss_repo = os.path.abspath(ss_repo).replace("\\", "/")
    # Windows の Python は C:\ 形式を返す。bash は C:/ 形式で読める。

    robot_pose = (args.x, args.y, args.yaw)
    out = set_robot_pose(args.container, args.x, args.y, args.yaw)
    print(f"機体を置き直した（{args.x}, {args.y}, yaw={args.yaw}）")
    if out:
        print(out[:500])
    print(f"{args.sleep:.1f} 秒待ってから ss に依頼します")
    time.sleep(args.sleep)

    # ss の依頼は ss.sh locate の引数をそのまま渡す（locate_args があれば）
    ss_args = args.locate_args or []
    t0 = time.time()
    result = request_locate(ss_repo, ss_args, out_json)
    took = time.time() - t0
    print(f"ss の応答が来た（{took:.1f} s。結果: {out_json}）")
    print(f"reason: {result.get('reason')}")
    rows = compare(result, scene, robot_pose)
    if not rows:
        print("見つかった盤はありません")
        return 0
    print_table(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
