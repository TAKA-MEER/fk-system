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


def expected_visible(scene):
    """置き場から見えるはずの盤（`scene.json` の `visible_panels`）。

    `robot.visible_panels` が無い古い `scene.json` では全部を期待なす。
    """
    robot = (scene or {}).get("robot") or {}
    ids = robot.get("visible_panels")
    if ids is not None:
        return list(ids)
    return [p["id"] for p in (scene or {}).get("panels", [])]


def missing_visible(layout_panels, scene):
    """見えるはずなのに配置図に無い盤を返す。

    戻り値は `[{truth_id, distance_m}]`。距離は置き場（`robot.pose`）から
    正解の盤の中心までの水平距離。
    """
    expected = expected_visible(scene)
    truths = (scene or {}).get("panels", [])
    present = {row["truth_id"] for row in match_layout_to_truth(layout_panels, truths)}
    pose = ((scene or {}).get("robot") or {}).get("pose") or [0.0, 0.0]
    rows = []
    for truth in truths:
        if truth["id"] in expected and truth["id"] not in present:
            dist = math.hypot(float(truth["center"][0]) - float(pose[0]),
                              float(truth["center"][1]) - float(pose[1]))
            rows.append({"truth_id": truth["id"], "distance_m": dist})
    return rows


def format_missing(rows):
    """見えるはずなのに配置図に無い盤の表の行にする（文字列の列）。"""
    return [f"欠け {row['truth_id']}: 置き場から {row['distance_m']:.3f} m" for row in rows]


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

    戻り値: `finished_waiting`（最後の盤の前で指示待ち）・
    `finished_with_failures`（`FINISH` だが合格でない）・`halted`・
    `rejected`（受付拒否が残っている）・`timeout`・`running`。

    本システムが `FINISH`（選んだ盤を回り終えた）になったら、合否を
    かかわらず時間切れを待たずに終える。合格でなければ
    `finished_with_failures`（WP-SIM-10）。飛ばした盤と理由は
    `format_failures` で表に出す。

    `rejected` には、選んだ盤がまだ残っているのに本システムが `SELECT`
    に戻り、`th_reject_reason`（例: th が発進を拒否した理由）が残っている
    場合も含む（WP-SIM-08）。この状態では本システムは指示待ちで何も進まない
    ため、時間切れまで待たずに終える。`finished_waiting`（全盤が済んだ）が
    優先する。
    """
    phase = (state or {}).get("phase", "")
    halt = (state or {}).get("halt_reason", "")
    if halt:
        return "halted"
    if phase == "FINISH":
        return "finished_waiting" if is_pass(results, selected_ids) \
            else "finished_with_failures"
    if is_pass(results, selected_ids) and phase in ("SELECT", "FINISH"):
        return "finished_waiting"
    if (state or {}).get("th_reject_reason") and phase == "SELECT" \
            and not is_pass(results, selected_ids):
        return "rejected"
    if (state or {}).get("gate_reason"):
        return "rejected"
    return "running"


ACTIVE_SCREEN_ID = "S-60"
ACTIVE_SCREEN_PERIOD_S = 0.5


def active_screen_fields(client_id, last_input_sec, last_input_nanosec=0):
    """`/ui/active_screen` の在席の欄（th の `ActiveScreen` 型の形）。

    `run.py` が 2 Hz で出す。`screen_id` は `S-60`（th の AUTO_MEASURE）で
    固定。不在だと th が `speed_limit stop` で `/cmd_vel` が 0 のままになる
    （th の知らせ 2026-11-24）。
    """
    return {"screen_id": ACTIVE_SCREEN_ID, "client_id": client_id,
            "interacting": True,
            "last_input_sec": int(last_input_sec),
            "last_input_nanosec": int(last_input_nanosec)}


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


def format_failures(results, selected_ids):
    """`finished_with_failures` のときに表へ出す、選んだ盤の飛ばし方（文字列の列）。

    `DONE` の盤は出さない。`SKIPPED` は理由をそのまま出す（バイパスの
    `no_mechanism` を含む。飛ばしであること自体を表で見えるようにするため）。
    それ以外の結果と、結果が無い盤も出す。
    """
    by_id = {r["panel_id"]: r for r in results}
    lines = []
    for pid in selected_ids:
        r = by_id.get(pid)
        if r is None:
            lines.append(f"{pid}: 結果なし")
        elif r["result"] == "SKIPPED":
            lines.append(f"{pid}: SKIPPED/{r.get('reason', '-')}")
        elif r["result"] != "DONE":
            lines.append(f"{pid}: {r['result']}/{r.get('reason', '-')}")
    return lines
