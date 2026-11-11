"""§2.2 の割り込みの行ごとの試験（DetailedDesign-auto.md §2.2）と、止める理由・th の拒否。

  test_pause_*   : th が PAUSE（人のジョグ・通信途絶・見張りの停止）→ PAUSED、人の「走行」
  test_human_*   : 人が昇降機を操作した（ss の「人の操作で中断」を含む）
  test_reentry_* : CANCELLED・localization_lost／reentry_required → REENTRY、入り直し
  test_halt_*    : 全体を止める
  test_abort_*   : 人の「中止」
  test_finish_*  : 人の「終了」（腕の確認）
  test_reject_*  : th が動く指示を拒否した（暫定の行）
"""
import pytest

from fk_auto.core import Params
from fk_auto.fakes import FakeArm, SsResponse
from test_runner_world import WPanel, World


def to_nav(w, n=1):
    w.to_select()
    w.start()
    w.until(lambda: w.th.model.state in ("NAV", "ALIGN"))


# --- PAUSED -----------------------------------------------------------------

def test_pause_goal_jog_waits_for_run_and_never_starts_by_itself():
    w = World([WPanel(3.0, 0.0)])
    to_nav(w)
    w.th.model.human_jog()
    w.until_phase("PAUSED")
    w.tick(100)                                  # 十分待っても自分からは発進しない
    assert w.phase == "PAUSED" and "meas.start" not in w.th_cmds()
    r = w.cmd("auto.run")
    assert r.accepted
    w.until_phase("GOAL")
    assert w.th_cmds().count("meas.start") == 1 and w.th_cmds().count("meas.goal") == 1  # GOAL はそのまま
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "no_mechanism")]


def test_pause_comm_loss_pauses_then_run_resumes():
    w = World([WPanel(3.0, 0.0)], th_kw=dict(nav_time_ms=5000))
    to_nav(w)
    w.th.link_alive = False
    w.until_phase("PAUSED")
    w.th.link_alive = True
    w.tick(3)
    assert w.phase == "PAUSED"                   # 通信が戻っても走らない
    assert w.cmd("auto.run").accepted
    w.until_phase("FINISH")


def test_pause_run_rejected_outside_paused_and_when_gate_fails():
    w = World([WPanel(3.0, 0.0)])
    to_nav(w)
    assert w.cmd("auto.run").reject_reason == "phase_not_accepting"   # PAUSED でない
    w.th.model.human_jog()
    w.until_phase("PAUSED")
    w.ss.external_busy = True                    # 本システム以外の ss の依頼が動いている
    w.tick()
    assert w.cmd("auto.run").reject_reason == "ss_busy"
    w.ss.external_busy = False
    w.lift.arm_stowed = True
    w.lift.mechanism_attached = True
    w.lift.arm_stowed = False
    w.tick()
    assert w.cmd("auto.run").reject_reason == "arm_not_stowed"
    assert "meas.start" not in w.th_cmds()


def test_pause_run_lowers_first_when_lift_is_off_bottom_then_starts():
    w = World([WPanel(3.0, 0.0)])
    to_nav(w)
    w.th.model.human_jog()
    w.until_phase("PAUSED")
    w.lift.height_mm = 200
    w.tick()
    assert w.cmd("auto.run").accepted
    n_start = w.th_cmds().count("meas.start")
    w.tick()
    assert w.th_cmds().count("meas.start") == n_start     # 下がりきるまで発進しない
    w.until_phase("GOAL")
    assert w.lift.height_mm == 0 and w.th_cmds().count("meas.start") == 1


def test_pause_recheck_restarts_from_the_head():
    w = World([WPanel(3.0, 0.0)], ss_kw=dict(result_delay_ms=1000))
    w.to_select()
    w.start()
    w.until(lambda: w.phase == "RECHECK" and len(w.ss.requests) == 2)
    w.th.model.human_jog()
    w.until_phase("PAUSED")
    assert [c[0] for c in w.ss.cancels] == ["s2"]            # 出していた依頼は取り消す
    w.until(lambda: w.ss.status().state == "IDLE")
    assert w.cmd("auto.run").accepted
    w.until_phase("RECHECK")
    w.until(lambda: len(w.ss.requests) == 3)                 # 段の頭から（依頼し直す）
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "no_mechanism")]


def test_pause_at_panel_only_tells_the_arm():
    arm = FakeArm(work_ms=2000)
    w = World([WPanel(3.0, 0.0)], mechanism=True, arm=arm)
    w.to_select()
    w.start()
    w.until(lambda: arm.starts)
    w.th.model.human_jog()
    w.until_phase("PAUSED")
    assert arm.pauses == [True] and arm.cancels == 0
    assert w.cmd("auto.run").accepted
    w.until_phase("AT_PANEL")
    assert arm.pauses == [True, False]
    w.until_phase("FINISH")
    assert w.results() == [("P001", "DONE", "")]


def test_pause_lift_drift_stops_th_then_run_lowers_and_resumes():
    w = World([WPanel(3.0, 0.0)], th_kw=dict(nav_time_ms=3000))
    to_nav(w)
    w.lift.height_mm = 60                        # 下端から 50 mm 以上離れた
    w.until_phase("PAUSED")
    assert "meas.stop" in w.th_cmds()
    assert w.cmd("auto.run").accepted
    w.until_phase("GOAL")
    assert w.lift.height_mm == 0
    w.until_phase("FINISH")


# --- 人が昇降機を操作した --------------------------------------------------------

def test_human_operating_lift_while_lowering_stops_commands_and_waits_for_run():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.lift.height_mm = 400
    w.start()
    w.tick()
    n = w.lift.lowers()
    w.lift.human_operating = True
    w.until_phase("PAUSED")
    w.tick(10)
    assert w.lift.lowers() == n                  # 押し直さない
    assert w.cmd("auto.run").reject_reason == "phase_not_accepting"   # 人が操作している間は受けない
    w.lift.human_operating = False
    w.tick()
    assert w.cmd("auto.run").accepted
    w.until_phase("GOAL")                         # 今の段（LOWER）をやり直して下端へ
    assert w.lift.height_mm == 0


def test_human_override_reported_by_ss_pauses_survey_and_run_redoes_it():
    w = World([WPanel(3.0, 0.0)])
    w.ss.responses.append(SsResponse("human_override"))
    w.begin()
    w.until_phase("PAUSED")
    assert w.runner.get_layout().panels == []
    w.tick(5)
    assert w.phase == "PAUSED"
    assert w.cmd("auto.run").accepted
    w.until_phase("SELECT")
    assert len(w.runner.get_layout().panels) == 1 and len(w.ss.requests) == 2


# --- 入り直し -------------------------------------------------------------------

def test_reentry_on_localization_lost_keeps_results_and_rebuilds_layout():
    w = World([WPanel(3.0, 0.0), WPanel(3.0, 4.0)])
    w.to_select()
    w.start(["P001"])
    w.until_phase("FINISH")
    done = w.results()
    assert done == [("P001", "SKIPPED", "no_mechanism")]
    w.start(["P002"])
    w.until(lambda: w.th.model.state in ("NAV", "ALIGN"))
    w.th.model.lose_localization()
    w.until_phase("REENTRY")
    assert w.runner.get_state().reentry_required
    assert w.cmd("auto.run").reject_reason == "phase_not_accepting"   # 「走行」を受けない
    n = len(w.th.sent)
    w.tick(10)
    assert len(w.th.sent) == n                    # REENTRY の間は何も出さない
    w.th.model.human_exit()
    w.until_phase("IDLE")
    assert w.results() == done                    # 測り終えた盤の結果は残す
    assert w.th.model.human_enter_auto()
    w.tick(2)
    assert w.cmd("auto.begin").accepted
    w.until_phase("SELECT")
    assert w.ids() == ["P003", "P004"]            # 配置図は作り直す（ID は使い回さない）
    assert w.runner.get_layout().map_instance_id == "map-2"


def test_reentry_on_status_flag_alone_even_if_result_was_missed():
    w = World([WPanel(3.0, 0.0)])
    to_nav(w)
    w.th.model.reentry_required = True            # 取り消しの結果を取りこぼしても状態で分かる
    w.until_phase("REENTRY")


def test_th_leaving_auto_mode_returns_to_idle():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.th.model.human_exit()
    w.until_phase("IDLE")
    assert w.runner.get_layout().panels == []


# --- 全体を止める -----------------------------------------------------------------

def test_halt_ss_cleanup_timeout_and_nothing_new_is_sent():
    w = World([WPanel(3.0, 0.0)], params=Params(ss_cleanup_timeout_s=3), ss_kw=dict())
    w.ss.never_cleanup = True
    w.begin()
    w.until_phase("HALTED")
    assert w.runner.get_state().halt_reason == "ss_cleanup_timeout"
    n_ss, n_th, n_lift = len(w.ss.requests), len(w.th.sent), w.lift.lowers()
    w.tick(50)
    assert (len(w.ss.requests), len(w.th.sent), w.lift.lowers()) == (n_ss, n_th, n_lift)


def test_halt_ss_link_lost_while_waiting_for_result():
    w = World([WPanel(3.0, 0.0)], ss_kw=dict(result_delay_ms=5000))
    w.begin()
    w.until(lambda: w.ss.requests)
    w.ss.frozen = True                            # ss の状態が届かなくなる
    w.until_phase("HALTED")
    assert w.runner.get_state().halt_reason == "ss_link_lost"


def test_halt_state_unreadable_stops_th_when_lift_height_goes_stale_while_running():
    w = World([WPanel(3.0, 0.0)], th_kw=dict(nav_time_ms=5000))
    to_nav(w)
    w.lift.height_readable = False                # 高さが読めなくなる（lift_state_stale_ms 後）
    w.until_phase("HALTED")
    assert w.runner.get_state().halt_reason == "state_unreadable"
    w.tick(2)
    assert "meas.stop" in w.th_cmds() and w.th.model.state == "PAUSE"


def test_halt_is_left_only_by_human_finish():
    w = World([WPanel(3.0, 0.0)], params=Params(ss_cleanup_timeout_s=3))
    w.ss.never_cleanup = True
    w.begin()
    w.until_phase("HALTED")
    assert w.cmd("auto.start", panel_ids=["P001"]).reject_reason == "phase_not_accepting"
    assert w.cmd("auto.finish").accepted
    w.until_phase("IDLE")
    assert w.th.model.mode == "IDLE"


# --- 人の「中止」 -----------------------------------------------------------------

def test_abort_while_running_sends_abort_waits_for_its_result_then_select():
    """meas.abort を送ったあと、結果が届くまで次へ進まない（th の返事を遅らせて見る）。"""
    w = World([WPanel(3.0, 0.0), WPanel(3.0, 4.0)], th_kw=dict(nav_time_ms=5000,
                                                              command_latency_ms=300))
    w.to_select()
    w.start()
    w.until(lambda: w.th.model.state in ("NAV", "ALIGN"))
    assert w.cmd("auto.abort").accepted
    w.tick()
    assert w.th_cmds().count("meas.abort") == 1
    n = len(w.th.sent)
    w.tick(1)
    assert w.phase == "GOAL" and len(w.th.sent) == n      # 結果が届くまで待つ（次へ進まない）
    w.until_phase("SELECT")
    assert w.th.model.state == "WAIT" and w.runner.get_state().queue == []
    assert w.results() == []                                # 中止した盤の結果は残さない
    w.tick(5)
    assert w.th_cmds().count("meas.goal") == 1              # 中止のあと勝手に次の盤へ行かない


def test_abort_in_recheck_cancels_ss_and_waits_for_cleanup():
    w = World([WPanel(3.0, 0.0)], ss_kw=dict(result_delay_ms=2000, cleanup_delay_ms=500))
    w.to_select()
    w.start()
    w.until(lambda: w.phase == "RECHECK" and len(w.ss.requests) == 2)
    assert w.cmd("auto.abort").accepted
    w.until_phase("SELECT")
    assert [c[0] for c in w.ss.cancels] == ["s2"] and w.ss.status().state == "IDLE"


def test_abort_at_panel_hands_cancel_to_arm_and_waits_for_stowing():
    arm = FakeArm(work_ms=100_000, cancel_ms=1000)
    w = World([WPanel(3.0, 0.0)], mechanism=True, arm=arm)
    w.to_select()
    w.start()
    w.until(lambda: arm.starts)
    assert w.cmd("auto.abort").accepted
    w.tick()
    assert arm.cancels == 1 and w.phase == "AT_PANEL"        # 格納するまで SELECT へ行かない
    w.until_phase("SELECT")
    assert w.results() == []


def test_abort_rejected_outside_listed_phases():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    assert w.cmd("auto.abort").reject_reason == "phase_not_accepting"


def test_abort_from_paused_aborts_th_goal():
    w = World([WPanel(3.0, 0.0)])
    to_nav(w)
    w.th.model.human_jog()
    w.until_phase("PAUSED")
    assert w.cmd("auto.abort").accepted
    w.until_phase("SELECT")
    assert "meas.abort" in w.th_cmds() and w.th.model.state == "WAIT"


# --- 人の「停止」 -----------------------------------------------------------------

def test_stop_command_sends_stop_only_while_running():
    w = World([WPanel(3.0, 0.0)], th_kw=dict(nav_time_ms=5000))
    w.to_select()
    assert w.cmd("auto.stop").reject_reason == "phase_not_accepting"
    w.start()
    w.until(lambda: w.th.model.state == "NAV")
    assert w.cmd("auto.stop").accepted
    w.until_phase("PAUSED")
    assert "meas.stop" in w.th_cmds()


# --- 人の「終了」 -----------------------------------------------------------------

def test_finish_requires_arm_confirmation_unless_stowed_or_no_mechanism():
    w = World([WPanel(3.0, 0.0)], mechanism=True, arm=FakeArm())
    w.to_select()
    assert w.cmd("auto.finish").accepted                       # 格納位置にある: 確認は要らない
    w.until_phase("IDLE")
    w2 = World([WPanel(3.0, 0.0)], mechanism=True, arm=FakeArm())
    w2.to_select()
    w2.lift.arm_stowed = False
    w2.tick()
    assert w2.cmd("auto.finish").reject_reason == "arm_not_confirmed"
    assert w2.th_cmds() == []
    assert w2.cmd("auto.finish", arm_confirmed=True).accepted
    w2.until_phase("IDLE")
    assert w2.th_cmds() == ["meas.finish"] and w2.th.model.mode == "IDLE"


def test_finish_unreadable_arm_state_also_requires_confirmation():
    w = World([WPanel(3.0, 0.0)], mechanism=True, arm=FakeArm())
    w.to_select()
    w.lift.arm_readable = False
    w.tick(10)                                    # 腕の状態が古くなる
    assert w.cmd("auto.finish").reject_reason == "arm_not_confirmed"


def test_finish_with_no_mechanism_needs_no_confirmation():
    w = World([WPanel(3.0, 0.0)], mechanism=False)
    w.to_select()
    w.lift.arm_stowed = False                     # 腕の条件は満たしているとみなす
    w.tick()
    assert w.cmd("auto.finish").accepted
    w.until_phase("IDLE")


# --- th が動く指示を拒否した（暫定の行）--------------------------------------------

def test_reject_goal_corner_obstacle_returns_to_select_keeping_results():
    w = World([WPanel(3.0, 0.0), WPanel(3.0, 4.0)])
    w.to_select()
    w.start(["P001"])
    w.until_phase("FINISH")
    w.th.model.corner_obstacle = True
    w.start(["P002"])
    w.until_phase("SELECT")
    st = w.runner.get_state()
    assert st.th_reject_reason == "corner_obstacle" and st.th_state == "WAIT"
    assert w.results() == [("P001", "SKIPPED", "no_mechanism")]
    assert w.th_cmds().count("meas.goal") == 2            # 自分からは出し直さない
    w.tick(5)
    assert w.th_cmds().count("meas.goal") == 2


def test_reject_goal_map_mismatch_goes_to_reentry():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.th.model.map_instance_id = "map-other"      # 配置図の地図と th の地図が食い違う
    w.tick(2)
    w.start()
    w.until_phase("REENTRY")
    assert w.runner.get_state().th_reject_reason == "map_mismatch"


def test_reject_start_stays_paused():
    w = World([WPanel(3.0, 0.0)])
    to_nav(w)
    w.th.model.human_jog()
    w.until_phase("PAUSED")
    w.th.model.corner_obstacle = True
    assert w.cmd("auto.run").accepted
    w.tick(5)
    assert w.phase == "PAUSED" and w.runner.get_state().th_reject_reason == "corner_obstacle"
    assert w.th_cmds().count("meas.start") == 1           # 出し直さない
    w.th.model.corner_obstacle = False
    assert w.cmd("auto.run").accepted
    w.until_phase("FINISH")


def test_reject_start_when_th_requires_reentry_goes_to_reentry():  # 状態の旗でも拒否でも REENTRY へ（どちらの道かまでは分けない）
    w = World([WPanel(3.0, 0.0)])
    to_nav(w)
    w.th.model.human_jog()
    w.until_phase("PAUSED")
    w.th.model.reentry_required = False
    assert w.cmd("auto.run").accepted
    w.th.model.reentry_required = True           # 受け付けの間に th が入り直しを要求した
    w.until_phase("REENTRY")


def test_reject_reroute_counts_toward_reroute_max_then_abort_and_skip():
    w = World([WPanel(3.0, 0.0), WPanel(3.0, 4.0)])
    w.to_select()
    w.start()
    w.until(lambda: w.th.model.state == "NAV")
    w.th.model.set_blocked(True)
    # 再計算が拒否される状況を、th のモデルの拒否理由で作る
    orig = w.th.model._reject_by_state
    w.th.model._reject_by_state = lambda cmd, args: ("state_not_accepting"
                                                      if cmd == "meas.reroute" else orig(cmd, args))
    w.until(lambda: ("P001", "SKIPPED", "no_path") in w.results(), 120_000)
    assert w.th_cmds().count("meas.reroute") == Params().reroute_max
    assert "meas.abort" in w.th_cmds()


def test_reject_stop_side_when_th_is_already_stopped_does_not_wait_for_result():
    w = World([WPanel(3.0, 0.0)])
    to_nav(w)
    assert w.cmd("auto.abort").accepted
    w.th.model.state = "WAIT"                     # すでに止まっている。abort は拒否される
    w.th.model.pending = None
    w.until_phase("SELECT")


def test_halt_sender_refuses_moving_and_lowering_even_when_gate_would_pass_but_allows_stop_side():
    """HALTED の間は、関門が通る条件でも、動く指示・下降・ss への依頼を送り口が出さない。"""
    arm = FakeArm(outcome=("HALT", "arm_stuck"))
    w = World([WPanel(3.0, 0.0)], mechanism=True, arm=arm)
    w.to_select()
    w.start()
    w.until_phase("HALTED")
    w.lift.height_mm = 0
    w.tick(3)
    n_th, n_lift, n_ss = len(w.th.sent), len(w.lift.commands), len(w.ss.requests)
    assert w.runner._emit("meas.goal", map_instance_id="map-1", x=1, y=1, yaw=0) is None
    assert w.runner._emit("meas.start") is None
    assert w.runner._emit("meas.reroute") is None
    from fk_auto import core
    assert w.runner._emit(core.CMD_LIFT_LOWER) is None
    assert w.runner._emit(core.CMD_SS_REQUEST, azimuth_from_rad=0, azimuth_span_rad=1) is None
    assert (len(w.th.sent), len(w.lift.commands), len(w.ss.requests)) == (n_th, n_lift, n_ss)
    assert w.runner._emit("meas.stop") is not None           # 止める側は通す


def test_human_operating_lift_while_lowering_for_run_stops_commands_and_needs_new_run():
    """PAUSED で「走行」→ 先に下げる、の途中で人が昇降機を操作したら、命令をやめ押し直さない。"""
    w = World([WPanel(3.0, 0.0)])
    to_nav(w)
    w.th.model.human_jog()
    w.until_phase("PAUSED")
    w.lift.height_mm = 1000
    w.tick()
    assert w.cmd("auto.run").accepted
    w.tick()
    n = w.lift.lowers()
    assert n >= 1
    w.lift.human_operating = True
    w.tick(10)
    assert w.lift.lowers() == n and w.phase == "PAUSED"
    assert "meas.start" not in w.th_cmds()
    w.lift.human_operating = False
    w.tick(3)
    assert w.lift.lowers() == n                   # 自動では下げ直さない（新しい「走行」を待つ）
    assert w.cmd("auto.run").accepted
    w.until_phase("GOAL")
    assert w.lift.height_mm == 0
