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


def test_format_table():
    rows = [{"layout_id": "L1", "truth_id": "P004", "center_err_m": 0.01}]
    results = [{"panel_id": "L1", "result": "DONE", "reason": ""}]
    lines = format_table(rows, results)
    assert len(lines) == 1 and "P004" in lines[0] and "DONE" in lines[0]
