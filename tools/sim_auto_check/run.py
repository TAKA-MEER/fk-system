#!/usr/bin/env python3
"""本物の th（Gazebo）と本システムで自律測定を最初から最後まで通す道具。

`sim_fk` の像の中で回す（ROS 2 と `fk_msgs`・`th_system_msgs` があるので）:

    docker exec sim_fk python3 /work/tools/sim_auto_check/run.py \\
        --scene /work/sim_out/scenes/public_room [--panels P001,P002] \\
        [--out /work/sim_out/auto_check/<時刻>]

手順（DetailedDesign-sim.md §7.1 の道具の行）:
  1. th の `/measure/status` が `IDLE` で本システムの生存確認（`link_ok`）が届くまで待つ
  2. th の `/system/trigger` に `ui.goto`（`{"kind":"AUTO_MEASURE"}`）を送る
     （断られたら理由を残して時間をおいて送り直す）
  3. 本システムに `auto.begin` → 配置図（`/fk/auto/layout`）を待つ
  4. 盤を選んで `auto.start`（既定は全部）
  5. 結果（`/fk/auto/results`）・`AutoState`・th の `/measure/status` を
     時刻つきで記録し続ける（出力先。既定は `sim_out/auto_check/<時刻>/`）
  6. 終わりで、盤ごとの結果と、配置図の盤と `scene.json` の正解の盤の
     対応（中心のずれ）を表で出す

指示・トピックの型は本システムの既存のもの（`fk_msgs`）と th の
取り込んだ型（`th_system_msgs`）に従う。名前は発明しない。
`ROS_DOMAIN_ID=78` で回すこと。
"""

from __future__ import annotations

import argparse
import json
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from check import (ACTIVE_SCREEN_PERIOD_S, active_screen_fields, classify_end,  # noqa: E402
                   count_results, expected_visible, format_missing, format_table,
                   match_layout_to_truth, missing_visible)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True, help="scene.json のあるディレクトリ")
    ap.add_argument("--panels", default="",
                    help="選ぶ盤（カンマ区切り。既定は配置図の全部）")
    ap.add_argument("--out", default="",
                    help="出力先（既定は sim_out/auto_check/<時刻>/）")
    ap.add_argument("--timeout", type=float, default=600.0, help="全体の上限 [s]")
    ap.add_argument("--goto-retry-s", type=float, default=5.0,
                    help="ui.goto が断られたときの送り直しの間隔 [s]")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from fk_msgs.msg import AutoState, Layout, PanelResults
    from fk_msgs.srv import AutoCommand
    from th_system_msgs.msg import ActiveScreen, MeasureStatus
    from th_system_msgs.srv import UiTrigger

    out_dir = args.out or os.path.join(
        "sim_out/auto_check", time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    log_path = os.path.join(out_dir, "trace.jsonl")
    logf = open(log_path, "a", encoding="utf-8")

    def record(kind, obj):
        line = {"t": time.time(), "kind": kind, **obj}
        logf.write(json.dumps(line, ensure_ascii=False) + "\n")
        logf.flush()
        print(f"[{time.strftime('%H:%M:%S')}] {kind}: {obj}", flush=True)

    rclpy.init()
    node = Node("sim_auto_check")
    latest = {"th": {}, "state": {}, "layout": {}, "results": {}}

    def on_th(msg):
        latest["th"] = {"mode": msg.mode, "state": msg.state,
                        "link_ok": msg.link_ok, "working": msg.working,
                        "reentry_required": msg.reentry_required}
        record("th_status", latest["th"])

    def on_state(msg):
        latest["state"] = {"phase": msg.phase, "panel_id": msg.panel_id,
                           "queue": list(msg.queue), "halt_reason": msg.halt_reason,
                           "gate_reason": msg.gate_reason,
                           "th_mode": msg.th_mode, "th_state": msg.th_state,
                           "th_reject_reason": msg.th_reject_reason}
        record("auto_state", latest["state"])

    def on_layout(msg):
        latest["layout"] = {
            "map_instance_id": msg.map_instance_id,
            "survey_reason": msg.survey_reason,
            "panels": [{"panel_id": p.panel_id, "x": p.x, "y": p.y,
                        "front_yaw": p.front_yaw, "width_m": p.width_m,
                        "depth_m": p.depth_m} for p in msg.panels]}
        record("layout", latest["layout"])

    def on_results(msg):
        latest["results"] = {"results": [
            {"panel_id": r.panel_id, "result": r.result,
             "reason": r.reason, "side": r.side} for r in msg.results]}
        record("results", latest["results"])

    # 測定機能の画面の在席（/ui/active_screen、2 Hz）。無いと th が
    # speed_limit stop で機体を動かさない（DetailedDesign-sim.md §7.1）。
    client_id = f"fk-sim-{random.randint(0, 0xFFFFFFFF):08x}"
    pub_screen = node.create_publisher(ActiveScreen, "/ui/active_screen", 10)

    def publish_screen():
        now = node.get_clock().now()
        f = active_screen_fields(client_id, now.nanoseconds / 1e9,
                                 now.nanoseconds % 1_000_000_000)
        msg = ActiveScreen()
        msg.screen_id = f["screen_id"]
        msg.client_id = f["client_id"]
        msg.interacting = f["interacting"]
        msg.last_input.sec = f["last_input_sec"]
        msg.last_input.nanosec = f["last_input_nanosec"]
        pub_screen.publish(msg)

    node.create_timer(ACTIVE_SCREEN_PERIOD_S, publish_screen)

    node.create_subscription(MeasureStatus, "/measure/status", on_th, 10)
    node.create_subscription(AutoState, "/fk/auto/state", on_state, 10)
    node.create_subscription(Layout, "/fk/auto/layout", on_layout, 10)
    node.create_subscription(PanelResults, "/fk/auto/results", on_results, 10)
    trigger = node.create_client(UiTrigger, "/system/trigger")
    auto_cmd = node.create_client(AutoCommand, "/fk/auto/command")

    def spin_until(cond, deadline):
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if cond():
                return True
        return False

    def call(client, req, deadline):
        fut = client.call_async(req)
        return spin_until(fut.done, deadline) and fut.result()

    end = time.time() + args.timeout
    try:
        # 1. th が IDLE で生存確認が届くまで待つ
        # th の MeasureStatus は mode=IDLE・state=NONE（state の IDLE は無い）。
        record("wait", {"what": "th mode IDLE + link_ok"})
        if not spin_until(lambda: latest["th"].get("mode") == "IDLE"
                          and latest["th"].get("link_ok"), end):
            record("end", {"reason": "th_not_ready", **latest})
            return 2
        # 2. ui.goto（断られたら理由を残して送り直す）
        while time.time() < end:
            req = UiTrigger.Request()
            req.trigger, req.arg_json, req.requester = \
                "ui.goto", '{"kind":"AUTO_MEASURE"}', "sim_auto_check"
            res = call(trigger, req, min(end, time.time() + 10))
            record("ui_goto", {"accepted": bool(res and res.accepted),
                               "reject_reason_key": getattr(res, "reject_reason_key", "")})
            if res and res.accepted:
                break
            time.sleep(args.goto_retry_s)
        else:
            record("end", {"reason": "ui_goto_rejected", **latest})
            return 2
        # 3. auto.begin（受け付けられるまで出し直す）
        while time.time() < end:
            req = AutoCommand.Request()
            req.command = "auto.begin"
            res = call(auto_cmd, req, min(end, time.time() + 10))
            record("auto_begin", {"accepted": bool(res and res.accepted),
                                  "reject_reason": getattr(res, "reject_reason", "")})
            if res and res.accepted:
                break
            time.sleep(1.0)
        else:
            record("end", {"reason": "auto_begin_rejected", **latest})
            return 2
        # 4. 配置図を待つ
        if not spin_until(lambda: bool(latest["layout"].get("panels")), end):
            record("end", {"reason": "no_layout", **latest})
            return 2
        selected = [p["panel_id"] for p in latest["layout"]["panels"]]
        if args.panels:
            selected = [s.strip() for s in args.panels.split(",") if s.strip()]
        record("selected", {"panels": selected})
        # 5. auto.start
        req = AutoCommand.Request()
        req.command, req.panel_ids = "auto.start", selected
        res = call(auto_cmd, req, min(end, time.time() + 10))
        record("auto_start", {"accepted": bool(res and res.accepted),
                              "reject_reason": getattr(res, "reject_reason", "")})
        if not (res and res.accepted):
            record("end", {"reason": "auto_start_rejected", **latest})
            return 2
        # 6. 終わりまで記録し続ける
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            results = latest["results"].get("results", [])
            cls = classify_end(latest["state"], results, selected)
            if cls in ("finished_waiting", "halted", "rejected"):
                break
        else:
            cls = "timeout"
        results = latest["results"].get("results", [])
        record("end", {"reason": cls, "counts": count_results(results), **latest})
        # 7. 正解との対応を表で出す
        with open(os.path.join(args.scene, "scene.json"), encoding="utf-8") as f:
            scene = json.load(f)
        layout_panels = latest["layout"].get("panels", [])
        expected = expected_visible(scene)
        truths = [p for p in scene["panels"] if p["id"] in expected]
        rows = match_layout_to_truth(layout_panels, truths)
        lines = format_table(rows, results)
        missing = missing_visible(layout_panels, scene)
        lines += format_missing(missing)
        for line in lines:
            print(line)
        record("missing_visible", {"missing": missing})
        summary = {"outcome": cls, "selected": selected,
                   "counts": count_results(results), "table": lines,
                   "missing_visible": missing,
                   "th": latest["th"], "state": latest["state"]}
        with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        print(f"記録: {out_dir}（trace.jsonl・summary.json）")
        return 0 if cls == "finished_waiting" else 1
    finally:
        logf.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
