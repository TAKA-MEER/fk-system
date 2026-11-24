"""`check.py` の純関数の試験（ホストで回る。ROS 2 不要）。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from check import (classify_end, count_results, format_table,  # noqa: E402
                   is_pass, match_layout_to_truth, nearest_truth)

TRUTHS = [{"id": "P001", "center": [5.1, 7.52]},
          {"id": "P002", "center": [5.9, 7.52]},
          {"id": "P003", "center": [6.8, 7.52]},
          {"id": "P004", "center": [3.0, 3.0]}]


def test_nearest_truth_picks_closest_not_first():
    truth, dist = nearest_truth((6.81, 7.51), TRUTHS)
    assert truth["id"] == "P003"
    assert abs(dist - 0.01414) < 0.001


def test_match_layout_to_truth():
    layout = [{"panel_id": "L1", "x": 3.01, "y": 3.0},
              {"panel_id": "L2", "x": 5.09, "y": 7.52}]
    rows = match_layout_to_truth(layout, TRUTHS)
    assert rows[0]["truth_id"] == "P004"
    assert rows[1]["truth_id"] == "P001"
    assert abs(rows[0]["center_err_m"] - 0.01) < 1e-9


def test_count_results_keeps_skipped_separate():
    results = [{"panel_id": "A", "result": "DONE", "reason": ""},
               {"panel_id": "B", "result": "SKIPPED", "reason": "no_mechanism"},
               {"panel_id": "C", "result": "SKIPPED", "reason": "no_path"}]
    assert count_results(results) == {"DONE": 1, "SKIPPED": 2, "other": 0}


def test_is_pass_done_and_no_mechanism_only():
    done = [{"panel_id": "A", "result": "DONE", "reason": ""}]
    assert is_pass(done, ["A"]) is True
    skipped_ok = [{"panel_id": "A", "result": "SKIPPED", "reason": "no_mechanism"}]
    assert is_pass(skipped_ok, ["A"]) is True
    skipped_bad = [{"panel_id": "A", "result": "SKIPPED", "reason": "no_path"}]
    assert is_pass(skipped_bad, ["A"]) is False
    assert is_pass([], ["A"]) is False


def test_classify_end():
    state = {"phase": "FINISH", "halt_reason": "", "gate_reason": ""}
    results = [{"panel_id": "A", "result": "DONE", "reason": ""}]
    assert classify_end(state, results, ["A"]) == "finished_waiting"
    assert classify_end({"halt_reason": "lift_not_lowered"}, [], []) == "halted"
    assert classify_end({"phase": "GOAL", "halt_reason": "", "gate_reason": ""},
                        [], ["A"]) == "running"


def test_classify_end_select_with_th_reject_and_remaining_is_rejected():
    # WP-SIM-08: 選んだ盤が残っているのに SELECT に戻り th_reject_reason が
    # 入っていたら、時間切れまで待たず rejected で終える。
    state = {"phase": "SELECT", "halt_reason": "", "gate_reason": "",
             "th_reject_reason": "corner_obstacle"}
    results = [{"panel_id": "A", "result": "SKIPPED", "reason": "top_not_found"}]
    assert classify_end(state, results, ["A", "B"]) == "rejected"
    # 結果が何も無い（最初の盤の発進を拒否された）場合も rejected。
    assert classify_end(state, [], ["A"]) == "rejected"


def test_classify_end_finish_with_failures():
    # WP-SIM-10: FINISH でも合格でなければ finished_with_failures で終える
    # （時間切れまで待たない）。合格なら finished_waiting。
    state = {"phase": "FINISH", "halt_reason": "", "gate_reason": ""}
    ok = [{"panel_id": "A", "result": "DONE", "reason": ""},
          {"panel_id": "B", "result": "SKIPPED", "reason": "no_mechanism"}]
    assert classify_end(state, ok, ["A", "B"]) == "finished_waiting"
    bad = [{"panel_id": "A", "result": "DONE", "reason": ""},
           {"panel_id": "B", "result": "SKIPPED", "reason": "top_not_found"}]
    assert classify_end(state, bad, ["A", "B"]) == "finished_with_failures"
    # 結果がまだ届いていない盤は、FINISH でも running（結果の受け取りは
    # AutoState より遅れる場合がある。実測 2026-11-25）。
    assert classify_end(state, bad[:1], ["A", "B"]) == "running"
    assert classify_end(state, [], ["A"]) == "running"
    # halt があれば halted が優先する（FINISH でも）。
    halted = {"phase": "FINISH", "halt_reason": "ss_failed", "gate_reason": ""}
    assert classify_end(halted, bad, ["A", "B"]) == "halted"


def test_format_failures_lists_skipped_with_reason():
    from check import format_failures
    results = [{"panel_id": "A", "result": "DONE", "reason": ""},
               {"panel_id": "B", "result": "SKIPPED", "reason": "top_not_found"},
               {"panel_id": "C", "result": "SKIPPED", "reason": "no_mechanism"},
               {"panel_id": "D", "result": "SKIPPED", "reason": "no_path"}]
    lines = format_failures(results, ["A", "B", "C", "D", "E"])
    assert lines == ["B: SKIPPED/top_not_found", "C: SKIPPED/no_mechanism",
                     "D: SKIPPED/no_path", "E: 結果なし"]
    # 選ばなかった盤の結果は出さない。
    assert format_failures(results, ["B"]) == ["B: SKIPPED/top_not_found"]


def test_classify_end_select_th_reject_does_not_preempt_finished_or_running():
    # 全盤が済んで指示待ちなら、残った th_reject_reason より finished を優先。
    state = {"phase": "SELECT", "halt_reason": "", "gate_reason": "",
             "th_reject_reason": "corner_obstacle"}
    results = [{"panel_id": "A", "result": "DONE", "reason": ""}]
    assert classify_end(state, results, ["A"]) == "finished_waiting"
    # GOAL など SELECT 以外では、th_reject_reason があっても running のまま。
    goal_state = {"phase": "GOAL", "halt_reason": "", "gate_reason": "",
                  "th_reject_reason": "corner_obstacle"}
    assert classify_end(goal_state, [], ["A"]) == "running"
    # SELECT でも th_reject_reason が無ければ running のまま。
    plain = {"phase": "SELECT", "halt_reason": "", "gate_reason": "",
             "th_reject_reason": ""}
    assert classify_end(plain, [], ["A"]) == "running"


def test_format_table():
    rows = [{"layout_id": "L1", "truth_id": "P004", "center_err_m": 0.01}]
    results = [{"panel_id": "L1", "result": "DONE", "reason": ""}]
    lines = format_table(rows, results)
    assert len(lines) == 1 and "P004" in lines[0] and "DONE" in lines[0]


# --- WP-SIM-06: 正解を visible_panels に絞る・欠けを表に出す ---

SCENE = {"robot": {"pose": [5.0, 4.0, 0.0],
                   "visible_panels": ["P001", "P002"],
                   "hidden_panels": [{"id": "P003", "reason": "far"}]},
         "panels": TRUTHS + [{"id": "P005", "center": [9.0, 9.0]}]}


def test_expected_visible_filters_by_visible_panels():
    from check import expected_visible
    assert expected_visible(SCENE) == ["P001", "P002"]
    # 古い scene.json（robot.visible_panels が無い）は全部を期待なす。
    old = {"panels": TRUTHS}
    assert expected_visible(old) == ["P001", "P002", "P003", "P004"]


def test_missing_visible_lists_expected_panels_absent_from_layout():
    from check import format_missing, missing_visible
    layout = [{"panel_id": "L1", "x": 5.09, "y": 7.52}]  # P001 だけ
    rows = missing_visible(layout, SCENE)
    assert [r["truth_id"] for r in rows] == ["P002"]
    # 距離は置き場（5, 4）から中心（5.9, 7.52）までの水平距離（独立計算）。
    assert abs(rows[0]["distance_m"] - 3.633) < 0.01
    assert format_missing(rows) == ["欠け P002: 置き場から 3.633 m"]


def test_missing_visible_empty_when_all_expected_present():
    from check import missing_visible
    layout = [{"panel_id": "L1", "x": 5.09, "y": 7.52},
              {"panel_id": "L2", "x": 5.91, "y": 7.52},
              {"panel_id": "L3", "x": 6.79, "y": 7.52}]
    assert missing_visible(layout, SCENE) == []
