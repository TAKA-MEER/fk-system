"""§2.1 の段の行ごとの試験（DetailedDesign-auto.md §2.1・§2.3）。試験名の接頭辞が段に対応する。

  test_idle_* / test_prepare_* / test_survey_* / test_select_* / test_lower_* / test_goal_* /
  test_recheck_* / test_at_panel_* / test_finish_*
"""
import math

import pytest

from fk_auto import core
from fk_auto.core import Params
from fk_auto.fakes import FakeArm, SsResponse
from test_runner_world import WPanel, World, body_rect

P = Params()
STAND_X = 3.0 - 0.3 - 0.389          # 盤（x=3・奥行き 0.6）の正面側に立つ位置


# --- IDLE -------------------------------------------------------------------

def test_idle_sends_nothing_and_begin_only_while_th_in_auto_measure():
    w = World([WPanel(3.0, 0.0)])
    w.tick(5)
    assert w.phase == "IDLE" and w.th_cmds() == [] and w.ss.requests == []
    assert w.cmd("auto.begin").reject_reason == "phase_not_accepting"   # th が AUTO_MEASURE でない
    w.th.model.human_enter_auto()
    w.tick(2)
    assert w.cmd("auto.begin").accepted
    assert w.phase == "PREPARE"


# --- PREPARE ----------------------------------------------------------------

def test_prepare_goes_to_survey_when_origin_is_taken_without_lowering():
    w = World([WPanel(3.0, 0.0)], lift_height_mm=500)   # 上にいるが原点は取れている
    w.begin()
    w.tick()
    assert w.phase == "SURVEY" and w.lift.lowers() == 0


def test_prepare_lowers_to_bottom_when_origin_missing():
    w = World([WPanel(3.0, 0.0)], lift_height_mm=250, lift_kw=dict(origin_ok=False))
    w.begin()
    w.until_phase("SURVEY")
    assert w.lift.height_mm == 0 and w.lift.origin_ok and w.lift.lowers() == 3


def test_prepare_halts_lift_not_lowered_when_lift_cannot_go_down():
    w = World([WPanel(3.0, 0.0)], lift_height_mm=250,
              lift_kw=dict(origin_ok=False, lower_step_mm=0.0), params=Params(lift_lower_timeout_s=3))
    w.begin()
    w.until_phase("HALTED")
    assert w.runner.get_state().halt_reason == "lift_not_lowered"


# --- SURVEY -----------------------------------------------------------------

def test_survey_requests_full_circle_and_builds_layout_in_map_coordinates():
    w = World([WPanel(3.0, 0.0), WPanel(3.0, 4.0)])
    w.to_select()
    req = w.ss.requests[0]
    assert req.azimuth_from_rad == pytest.approx(-math.pi)
    assert req.azimuth_span_rad == pytest.approx(2 * math.pi)
    assert req.max_distance_m == 0 and req.time_limit_s == P.ss_result_timeout_s
    lay = w.runner.get_layout()
    assert lay.map_instance_id == "map-1"
    assert [(p.panel_id, round(p.x, 3), round(p.y, 3), p.row_id) for p in lay.panels] == [
        ("P001", 3.0, 0.0, "R1"), ("P002", 3.0, 4.0, "R2")]
    assert all(p.confident and p.top_height_m == 2.35 for p in lay.panels)
    assert w.th_cmds() == []                          # 配置図を作るあいだ th に指示は出さない


def test_survey_uses_partial_result_and_shows_unseen_directions():
    """途中で終わった結果（時間切れ）も、返った盤と見た範囲を配置図に使う。"""
    w = World([WPanel(3.0, 0.0)])
    rect = body_rect(w.panels[0], w.pose())
    w.ss.responses.append(SsResponse("timeout", (rect,), (-0.5,), (1.0,)))
    w.to_select()
    lay = w.runner.get_layout()
    assert len(lay.panels) == 1
    assert len(lay.unseen_from_rad) == 1 and len(lay.unseen_to_rad) == 1   # 見ていない方向が残る
    assert lay.survey_reason == "timeout"                                  # ss の終わり方のまま


def test_survey_rechecks_when_result_is_unusable_and_gives_up_after_retries():
    """th が動いた（位置が許容差を超えて変わった）結果は捨てて依頼し直し、ss_retry_max 回まで。"""
    w = World([WPanel(3.0, 0.0)])
    base = w.respond

    def moving(req):
        w.th.model.x += 0.5         # 依頼のあいだに機体が動いた
        return base(req)
    w.ss.responder = moving
    w.to_select()
    assert len(w.ss.requests) == 1 + P.ss_retry_max
    assert w.runner.get_layout().panels == []
    assert w.runner.get_layout().survey_reason == "ss_failed"


def test_survey_retry_then_success_uses_second_result():
    w = World([WPanel(3.0, 0.0)])
    base = w.respond
    calls = []

    def once_moving(req):
        calls.append(1)
        if len(calls) == 1:
            w.th.model.x += 0.5
        return base(req)
    w.ss.responder = once_moving
    w.to_select()
    assert len(w.ss.requests) == 2 and len(w.runner.get_layout().panels) == 1


def test_survey_no_result_cancels_waits_cleanup_and_gives_empty_layout():
    w = World([WPanel(3.0, 0.0)], params=Params(ss_result_timeout_s=5), ss_kw=dict(
        result_delay_ms=0))
    w.ss.never_result = True
    w.begin()
    w.until_phase("SELECT")
    assert [c[0] for c in w.ss.cancels] == ["s1"]
    assert w.runner.get_layout().panels == [] and w.runner.get_layout().survey_reason == "ss_failed"
    assert w.ss.status().state == "IDLE"        # 後片付けの完了まで待ってから SELECT に来た


def test_survey_empty_result_gives_empty_layout_and_rescan_finds_panels():
    w = World([WPanel(3.0, 0.0)])
    w.hide = {0}
    w.to_select()
    assert w.runner.get_layout().panels == []
    assert w.runner.get_layout().survey_reason == "no_lugs"
    w.hide = set()
    assert w.cmd("auto.rescan").accepted
    assert w.phase == "SURVEY"
    w.until_phase("SELECT")
    assert len(w.runner.get_layout().panels) == 1


def test_survey_waits_for_cleanup_before_going_to_select_and_before_any_move():
    w = World([WPanel(3.0, 0.0)], ss_kw=dict(cleanup_delay_ms=1000))
    w.begin()
    w.until(lambda: len(w.ss.requests) == 1 and w.ss.status().state == "CLEANING")
    assert w.phase == "SURVEY"
    w.until_phase("SELECT")
    assert w.ss.status().state == "IDLE"


# --- SELECT -----------------------------------------------------------------

def test_select_start_validates_panels_and_order():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    assert w.cmd("auto.start", panel_ids=[]).reject_reason == "bad_argument"
    assert w.cmd("auto.start", panel_ids=["P999"]).reject_reason == "bad_argument"
    assert w.phase == "SELECT"
    assert w.cmd("auto.stop").reject_reason == "phase_not_accepting"
    assert w.cmd("auto.run").reject_reason == "phase_not_accepting"
    assert w.cmd("auto.start", panel_ids=["P001"]).accepted
    st = w.runner.get_state()
    assert st.phase == "LOWER" and st.panel_id == "P001" and st.queue == []


def test_select_rescan_keeps_panel_ids_increasing_never_reused():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    assert w.ids() == ["P001"]
    assert w.cmd("auto.rescan").accepted
    w.until_phase("SELECT")
    assert w.ids() == ["P002"]
    w.cmd("auto.rescan")
    w.until_phase("SELECT")
    assert w.ids() == ["P003"]


def test_select_rescan_rejected_when_arm_not_stowed():
    w = World([WPanel(3.0, 0.0)], mechanism=True, arm=FakeArm())
    w.to_select()
    w.lift.arm_stowed = False
    w.tick()
    assert w.cmd("auto.rescan").reject_reason == "arm_not_stowed"
    assert w.phase == "SELECT"


# --- LOWER ------------------------------------------------------------------

def test_lower_lowers_one_command_per_step_until_bottom_then_goal():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.lift.height_mm = 300
    w.start()
    n0 = w.lift.lowers()
    w.tick()
    assert w.phase == "LOWER" and w.lift.lowers() == n0 + 1   # step 1 回に命令 1 つ
    w.until_phase("GOAL")
    assert w.lift.height_mm == 0 and w.lift.lowers() == n0 + 3
    assert w.th_cmds() == []                                 # 下がりきる前は th に出さない


def test_lower_halts_when_not_lowered_within_timeout():
    w = World([WPanel(3.0, 0.0)], params=Params(lift_lower_timeout_s=2))
    w.to_select()
    w.lift.height_mm = 300
    w.lift.lower_step_mm = 0.0
    w.start()
    w.until_phase("HALTED")
    assert w.runner.get_state().halt_reason == "lift_not_lowered" and w.th_cmds() == []


# --- GOAL -------------------------------------------------------------------

def test_goal_sends_stand_pose_with_layout_map_id_then_recheck():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.start()
    w.until_phase("RECHECK")
    g = w.th.sent[0]
    assert g.command == "meas.goal" and g.map_instance_id == "map-1"
    assert (g.x, g.y, g.yaw) == pytest.approx((STAND_X, 0.0, 0.0))
    assert w.th.model.x == pytest.approx(STAND_X)


def test_goal_blocked_reroutes_after_wait_then_continues_when_path_opens():
    w = World([WPanel(3.0, 0.0)], th_kw=dict(nav_time_ms=1000))
    w.to_select()
    w.start()
    w.until(lambda: w.th.model.state == "NAV")
    w.th.model.set_blocked(True)
    w.tick(30)                                          # blocked_wait_s(5 s) には届かない
    assert "meas.reroute" not in w.th_cmds()
    w.until(lambda: "meas.reroute" in w.th_cmds(), 10_000)
    w.th.model.set_blocked(False)
    w.until_phase("RECHECK")
    assert w.th_cmds().count("meas.reroute") == 1


def test_goal_unsolved_after_reroute_max_aborts_then_skips_no_path():
    w = World([WPanel(3.0, 0.0), WPanel(3.0, 4.0)])
    w.to_select()
    w.start()
    w.until(lambda: w.th.model.state == "NAV")
    w.th.model.set_blocked(True)
    w.until(lambda: "meas.abort" in w.th_cmds(), 120_000)
    assert w.th_cmds().count("meas.reroute") == P.reroute_max
    w.until(lambda: ("P001", "SKIPPED", "no_path") in w.results())
    assert w.th.model.state in ("WAIT", "NAV", "ALIGN", "BLOCKED")   # 次の盤へ進んでいる
    assert w.runner.get_state().panel_id == "P002"


def test_goal_skips_stand_blocked_without_sending_goal():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    # 立ち位置を塞ぐ（地図は x:-10..10・解像度 0.1。立ち位置 (2.311, 0) の 1 マスを占有にする）
    g = w.th.model.grid
    data = list(g.data)
    col, row = int((STAND_X + 10.0) / 0.1), int((0.0 + 10.0) / 0.1)
    data[row * g.width + col] = 100
    w.th.model.grid = core.Grid(g.resolution, g.origin_x, g.origin_y, g.width, g.height, data)
    w.start()
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "stand_blocked")]
    assert w.th_cmds() == []                              # 出さずに飛ばす


# --- RECHECK ----------------------------------------------------------------

def test_recheck_requests_front_half_from_stand_and_goes_to_at_panel():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.start()
    w.until_phase("RECHECK")
    w.until(lambda: len(w.ss.requests) == 2)
    r = w.ss.requests[1]
    assert r.azimuth_from_rad == pytest.approx(-math.pi / 2)
    assert r.azimuth_span_rad == pytest.approx(P.recheck_span_rad)
    assert r.max_distance_m == 0
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "no_mechanism")]


def test_recheck_updates_layout_and_regoals_once_when_stand_moves_beyond_tolerance():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.recheck_shift = [0.2, 0.2]            # 認識し直すと盤が 0.2 m 奥にあった（出し直した先では合う）
    w.start()
    w.until_phase("FINISH")
    goals = [c for c in w.th.sent if c.command == "meas.goal"]
    assert len(goals) == 2 and goals[1].x == pytest.approx(STAND_X + 0.2)
    assert w.runner.get_layout().panels[0].x == pytest.approx(3.2)
    assert w.ids() == ["P001"]              # 位置を直しても同じ盤は同じ ID
    assert w.results() == [("P001", "SKIPPED", "no_mechanism")]


def test_recheck_stand_mismatch_twice_skips_stand_mismatch():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.recheck_shift = [0.2, 0.35]           # 出し直した先でも、盤がさらに奥に見えて、まだ許容差を超える
    w.start()
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "stand_mismatch")]
    assert len([c for c in w.th.sent if c.command == "meas.goal"]) == 2


def test_recheck_top_not_found_when_nothing_returned():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.start()
    w.until_phase("RECHECK")
    w.hide = {0}
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "top_not_found")]


def test_recheck_does_not_mistake_a_neighbor_panel_for_the_target():
    """目当ての盤が見えず、recheck_match_max_m より遠くに別の盤だけ見える → top_not_found。"""
    w = World([WPanel(3.0, 0.0), WPanel(3.0, 4.0)])
    w.to_select()
    w.start(["P001"])
    w.until_phase("RECHECK")
    w.hide = {0}
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "top_not_found")]
    assert [c.command for c in w.th.sent].count("meas.goal") == 1    # 隣の盤の位置へは行かない


def test_recheck_picks_nearest_rect_within_match_limit():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.recheck_shift = [0.25, 0.25]          # 0.3 m 以内のずれ: 同じ盤
    w.start()
    w.until_phase("FINISH")
    assert w.runner.get_layout().panels[0].x == pytest.approx(3.25)


def test_recheck_top_not_found_when_not_confident():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.panels[0].pos_std = 0.5               # 認識し直しで確からしさがしきい値を満たさない
    w.start()
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "top_not_found")]


def test_recheck_camera_too_low():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.start()
    w.until_phase("RECHECK")
    w.ss.responder = lambda req: SsResponse("camera_too_low")
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "camera_too_low")]


def test_recheck_ss_failed_when_no_result_comes_after_cancel_and_cleanup():
    w = World([WPanel(3.0, 0.0)], params=Params(ss_result_timeout_s=3))
    w.to_select()
    w.start()
    w.until_phase("RECHECK")
    w.ss.never_result = True
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "ss_failed")]
    assert w.ss.cancels and w.ss.status().state == "IDLE"


def test_recheck_ss_failed_when_results_unusable_after_retries():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.start()
    w.until_phase("RECHECK")
    base = w.respond

    def moving(req):
        w.th.model.x += 0.5
        return base(req)
    w.ss.responder = moving
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "ss_failed")]


# --- AT_PANEL / 測定機構なしの道 ------------------------------------------------

def test_at_panel_no_mechanism_when_lift_says_not_attached_does_not_hand_over_work():
    arm = FakeArm()
    w = World([WPanel(3.0, 0.0)], mechanism=False, arm=arm)
    w.to_select()
    w.start()
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "no_mechanism")] and arm.starts == []


def test_at_panel_no_mechanism_when_arm_port_refuses_work():
    w = World([WPanel(3.0, 0.0)], mechanism=True)        # NoMechanism は作業を受けない
    w.to_select()
    w.start()
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "no_mechanism")]


def test_at_panel_hands_work_to_arm_and_records_done():
    arm = FakeArm()
    w = World([WPanel(3.0, 0.0)], mechanism=True, arm=arm)
    w.to_select()
    w.start()
    w.until_phase("FINISH")
    assert arm.starts == ["P001"] and w.results() == [("P001", "DONE", "")]


def test_at_panel_arm_space_blocked_skips():
    arm = FakeArm(outcome=("SKIPPED", "arm_space_blocked"))
    w = World([WPanel(3.0, 0.0)], mechanism=True, arm=arm)
    w.to_select()
    w.start()
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "arm_space_blocked")]


def test_at_panel_arm_stuck_halts():
    arm = FakeArm(outcome=("HALT", "arm_stuck"))
    w = World([WPanel(3.0, 0.0)], mechanism=True, arm=arm)
    w.to_select()
    w.start()
    w.until_phase("HALTED")
    assert w.runner.get_state().halt_reason == "arm_stuck"


def test_at_panel_halts_when_arm_link_lost_while_working():
    arm = FakeArm(work_ms=100_000)
    w = World([WPanel(3.0, 0.0)], mechanism=True, arm=arm)
    w.to_select()
    w.start()
    w.until_phase("AT_PANEL")
    w.until(lambda: arm.starts)
    w.lift.arm_link_ok = False
    w.until_phase("HALTED")
    assert w.runner.get_state().halt_reason == "arm_link_lost"


# --- 複数盤・回る順・FINISH -----------------------------------------------------

def test_two_panels_in_one_row_go_right_first_then_next():
    w = World([WPanel(3.0, 1.1), WPanel(3.0, 0.0)])       # 同じ列（隙間 0.1 m）。右（y が小さい）から
    w.to_select()
    assert len({p.row_id for p in w.runner.get_layout().panels}) == 1
    w.start()
    w.until_phase("FINISH")
    ys = [round(c.y, 2) for c in w.th.sent if c.command == "meas.goal"]
    assert ys == [0.0, 1.1]
    assert [r[0] for r in w.results()] == ["P002", "P001"]      # 回った順（P002 は y=0 の盤）


def test_two_rows_nearest_row_first_and_queue_shown():
    w = World([WPanel(3.0, 4.0), WPanel(3.0, 0.0)])
    w.to_select()
    w.start()
    w.until(lambda: w.runner.get_state().phase == "GOAL")
    st = w.runner.get_state()
    assert st.panel_id == "P002" and st.queue == ["P001"]
    w.until_phase("FINISH")
    assert [r[0] for r in w.results()] == ["P002", "P001"]


def test_finish_lowers_lists_results_and_waits_then_start_again_goes_to_lower():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.start()
    w.until_phase("FINISH")
    w.lift.height_mm = 200              # 測定機構が持ち上げた
    w.until(lambda: w.lift.height_mm == 0)
    assert w.phase == "FINISH" and len(w.results()) == 1
    assert w.cmd("auto.start", panel_ids=["P001"]).accepted
    assert w.phase == "LOWER"
    w.until_phase("FINISH")
    assert len(w.results()) == 1            # 同じ盤の結果は置き換える
