"""WP-AUTO-10 状態機械（DetailedDesign-auto.md §2.6）。訪問（盤と側）で回す。"""
import pytest

from fk_auto.core import Params
from fk_auto.fakes import FakeArm
from test_runner_world import WPanel, World


def goals(w):
    return [c for c in w.th.sent if c.command == "meas.goal"]


# --- 回り込みの順 ---

def test_front_all_then_back_for_deep_panel_only():
    w = World([WPanel(3.0, 0.0, d=2.0), WPanel(3.0, 3.0, d=1.0)])
    w.to_select()
    w.start()
    w.until_phase("FINISH")
    # 結果は正面側を両方回ったあと、背面側（2 m の盤だけ）が続く
    assert w.results_side() == [
        ("P001", "SKIPPED", "no_mechanism", "front"),
        ("P002", "SKIPPED", "no_mechanism", "front"),
        ("P001", "SKIPPED", "no_mechanism", "back"),
    ]
    gs = goals(w)
    assert len(gs) == 3
    assert gs[0].x < 3.0 and gs[1].x < 3.0      # 正面側の立ち位置
    assert gs[2].x > 3.0                          # 背面側の立ち位置（背面側は最後）


def test_back_visit_uses_arm_done_and_side_is_passed_to_arm():
    arm = FakeArm(work_ms=200)
    w = World([WPanel(3.0, 0.0, d=2.0)], mechanism=True, arm=arm)
    w.to_select()
    w.start()
    w.until_phase("FINISH")
    assert w.results_side() == [
        ("P001", "DONE", "", "front"),
        ("P001", "DONE", "", "back"),
    ]
    assert arm.start_sides == ["front", "back"]


def test_back_is_queued_after_arm_space_blocked_too():
    arm = FakeArm(work_ms=200, outcome=("SKIPPED", "arm_space_blocked"))
    w = World([WPanel(3.0, 0.0, d=2.0)], mechanism=True, arm=arm)
    w.to_select()
    w.start()
    w.until_phase("FINISH")
    assert w.results_side() == [
        ("P001", "SKIPPED", "arm_space_blocked", "front"),
        ("P001", "SKIPPED", "arm_space_blocked", "back"),
    ]


def test_shallow_panel_has_no_back_visit():
    w = World([WPanel(3.0, 0.0, d=1.0)])
    w.to_select()
    w.start()
    w.until_phase("FINISH")
    assert w.results_side() == [("P001", "SKIPPED", "no_mechanism", "front")]
    assert len(goals(w)) == 1


# --- 正面側に届かなかった盤には背面側を足さない ---

def test_no_back_when_front_at_panel_not_reached():
    w = World([WPanel(3.0, 0.0, d=2.0)])
    w.to_select()
    w.hide.add(0)          # 盤前の認識し直しで見つからない
    w.start()
    w.until_phase("FINISH")
    assert w.results_side() == [("P001", "SKIPPED", "top_not_found", "front")]
    assert len(goals(w)) == 1


def test_back_skip_does_not_erase_front_result():
    w = World([WPanel(3.0, 0.0, d=2.0)])
    w.to_select()
    w.start()
    w.until(lambda: any(r[3] == "front" for r in w.results_side()))
    w.hide.add(0)          # 背面側の認識し直しでは見つからない
    w.until_phase("FINISH")
    assert w.results_side() == [
        ("P001", "SKIPPED", "no_mechanism", "front"),
        ("P001", "SKIPPED", "top_not_found", "back"),
    ]


# --- 奥行きが測れていない盤 ---

def test_unmeasured_panel_back_merges_depth_and_centre():
    w = World([WPanel(3.0, 0.0, d=1.86, unmeasured=True)])
    w.to_select()
    lay = w.runner.get_layout().panels[0]
    assert lay.depth_measured is False and lay.depth_m == pytest.approx(0.86)
    w.start()
    w.until_phase("FINISH")
    assert w.results_side() == [
        ("P001", "SKIPPED", "no_mechanism", "front"),
        ("P001", "SKIPPED", "no_mechanism", "back"),
    ]
    lay = w.runner.get_layout().panels[0]
    assert lay.depth_measured is True
    assert lay.depth_m == pytest.approx(1.86)
    assert lay.x == pytest.approx(3.0) and lay.y == pytest.approx(0.0)
    assert lay.front_yaw == pytest.approx(3.141592653589793)
    # 背面側の立ち位置は panel_depth_max_m を仮定（正面の面 2.07 から内向き 2.04 ＋ 0.389）
    gs = goals(w)
    assert len(gs) == 2
    assert gs[1].x == pytest.approx(2.07 + 2.04 + 0.389)


# --- 入り直しで背面側の待ちを捨てる ---

def test_reentry_discards_back_wait():
    w = World([WPanel(3.0, 0.0, d=2.0), WPanel(3.0, 3.0, d=1.0)])
    w.to_select()
    w.start()
    w.until(lambda: any(r[3] == "front" for r in w.results_side()))
    assert w.runner._back_wait == ["P001"] or w.runner._back_wait  # 正面側の途中では待ちがある
    w.th.model.lose_localization()
    w.until_phase("REENTRY")
    assert w.runner._back_wait == []
