"""配置図（map フレーム）と scene.json の正解（world フレーム）の対応、結果の数え方。

ROS 2・Gazebo には触れない（純関数のみ）。`run.py` から使う。
座標の約束: 配置図の盤の中心（`Panel.x`・`Panel.y`）は map フレーム、
`scene.json` の盤の中心（`center`）は Gazebo のワールド座標。段 3 の
`public_room` では th の map がワールドに揃う前提で比べる（ずれが出れば
そのまま表に出る）。
"""

from __future__ import annotations

import math


def nearest_truth(xy, truth_panels):
    """`truth_panels` の中で中心が `xy` にいちばん近いものと、その距離を返す。

    `truth_panels` は `scene.json` の `panels`（`id`・`center` を持つ）。
    いちばん近いものが同点のときは先に出たものを返す。
    """
    best, best_d = None, None
    for truth in truth_panels:
        d = math.hypot(float(truth["center"][0]) - float(xy[0]),
                       float(truth["center"][1]) - float(xy[1]))
        if best_d is None or d < best_d:
            best, best_d = truth, d
    return best, best_d


def match_layout_to_truth(layout_panels, truth_panels):
    """配置図の盤ごとに、いちばん近い正解の盤と中心のずれを返す。

    `layout_panels` は `Panel` 相当の辞書の列（`panel_id`・`x`・`y`）。
    戻り値は `[{layout_id, truth_id, center_err_m}]`。
    """
    rows = []
    for panel in layout_panels:
        truth, dist = nearest_truth((panel["x"], panel["y"]), truth_panels)
        if truth is None:
            continue
        rows.append({"layout_id": panel["panel_id"],
                     "truth_id": truth["id"],
                     "center_err_m": dist})
    return rows


def count_results(results):
    """結果の数え方。`DONE` と `SKIPPED` は別に数える（混ぜない）。"""
    counts = {"DONE": 0, "SKIPPED": 0, "other": 0}
    for r in results:
        if r["result"] in ("DONE", "SKIPPED"):
            counts[r["result"]] += 1
        else:
            counts["other"] += 1
    return counts


def is_pass(results, selected_ids):
    """選んだ盤がすべて「済み」か「飛ばした・`no_mechanism`」か。

    段 3 の合格（DetailedDesign-sim.md §7.1）。`DONE` 以外で合格になるのは
    `SKIPPED` かつ理由が `no_mechanism`（バイパス中）のときだけ。
    """
    by_id = {r["panel_id"]: r for r in results}
    for pid in selected_ids:
        r = by_id.get(pid)
        if r is None:
            return False
        if r["result"] == "DONE":
            continue
        if r["result"] == "SKIPPED" and r.get("reason") == "no_mechanism":
            continue
        return False
    return True


def classify_end(state, results, selected_ids):
    """終わり方の分類。`run.py` の記録と表に出す。

    戻り値: `finished_waiting`（最後の盤の前で指示待ち）・`halted`・
    `rejected`（受付拒否が残っている）・`timeout`・`running`。
    """
    phase = (state or {}).get("phase", "")
    halt = (state or {}).get("halt_reason", "")
    if halt:
        return "halted"
    if is_pass(results, selected_ids) and phase in ("SELECT", "FINISH"):
        return "finished_waiting"
    if (state or {}).get("gate_reason"):
        return "rejected"
    return "running"


def format_table(match_rows, results):
    """盤ごとの結果と正解との対応を表の行にする（文字列の列）。"""
    result_by_id = {r["panel_id"]: r for r in results}
    lines = []
    for row in match_rows:
        r = result_by_id.get(row["layout_id"], {})
        lines.append(
            f"{row['layout_id']} -> {row['truth_id']}: "
            f"中心のずれ {row['center_err_m']:.3f} m, "
            f"結果 {r.get('result', '-')}/{r.get('reason', '-')}")
    return lines
