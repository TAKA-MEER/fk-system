"""全体を縛る試験（ブリーフ §5）: 送り口 1 つ・関門・見張り・口が無いとき・時刻は引数だけ。

- 表に無い指示を出さない（すべての筋書きで FakeTh が受けた指示は 6 つだけ）
- 関門が通さない様子では、動く指示と下降の命令が 1 つも口に届かない
- 下げている途中で ss が IDLE でなくなったら、次の step から下降の命令が出ない
- 走っている間は drive_watch を毎回見て、止めるなら meas.stop
- 口が無い（None）とき何も送らない
"""
import sys
import time

import pytest

from fk_auto import core
from fk_auto.core import Params
from fk_auto.fakes import FakeArm, FakeLift, FakeSs, FakeTh, Sim, ThModel
from fk_auto.runner import Runner
from test_runner_world import WPanel, World

ALLOWED = {"meas.goal", "meas.start", "meas.reroute", "meas.stop", "meas.abort", "meas.finish"}
MOVING = {"meas.goal", "meas.start", "meas.reroute"}


# --- すべての筋書きで、表の指示だけ ---------------------------------------------------

def sc_normal(w):
    w.to_select()
    w.start()
    w.until_phase("FINISH")
    w.cmd("auto.finish")
    w.until_phase("IDLE")


def sc_two_rows_and_recheck_regoal(w):
    w.to_select()
    w.recheck_shift = [0.2, 0.2]
    w.start()
    w.until_phase("FINISH")


def sc_no_path(w):
    w.to_select()
    w.start()
    w.until(lambda: w.th.model.state == "NAV")
    w.th.model.set_blocked(True)
    w.until_phase("FINISH", 200_000)


def sc_jog_and_run(w):
    w.to_select()
    w.start()
    w.until(lambda: w.th.model.state == "NAV")
    w.th.model.human_jog()
    w.until_phase("PAUSED")
    w.cmd("auto.run")
    w.until_phase("FINISH")


def sc_abort(w):
    w.to_select()
    w.start()
    w.until(lambda: w.th.model.state == "NAV")
    w.cmd("auto.abort")
    w.until_phase("SELECT")


def sc_reentry(w):
    w.to_select()
    w.start()
    w.until(lambda: w.th.model.state == "NAV")
    w.th.model.lose_localization()
    w.until_phase("REENTRY")
    w.tick(5)


def sc_th_rejects(w):
    w.to_select()
    w.th.model.corner_obstacle = True
    w.start()
    w.until_phase("SELECT")


def sc_drift(w):
    w.to_select()
    w.start()
    w.until(lambda: w.th.model.state == "NAV")
    w.lift.height_mm = 80
    w.until_phase("PAUSED")
    w.cmd("auto.run")
    w.until_phase("FINISH")


def sc_halt(w):
    w.ss.never_cleanup = True
    w.begin()
    w.until_phase("HALTED", 200_000)
    w.cmd("auto.finish", arm_confirmed=True)


@pytest.mark.parametrize("scenario", [sc_normal, sc_two_rows_and_recheck_regoal, sc_no_path,
                                      sc_jog_and_run, sc_abort, sc_reentry, sc_th_rejects,
                                      sc_drift, sc_halt])
def test_only_listed_commands_reach_th_in_every_scenario(scenario):
    w = World([WPanel(3.0, 0.0), WPanel(3.0, 4.0)], params=Params(ss_cleanup_timeout_s=3))
    scenario(w)
    cmds = w.th_cmds()
    assert cmds and set(cmds) <= ALLOWED
    # 人の「走行」を受けていないのに meas.start を出さない
    assert cmds.count("meas.start") <= (1 if scenario in (sc_jog_and_run, sc_drift) else 0)


def test_the_single_sender_refuses_commands_not_in_the_table_and_sends_nothing():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    n_th, n_lift, n_ss = len(w.th.sent), len(w.lift.commands), len(w.ss.requests)
    for c in ("meas.turn", "meas.translate", "meas.set_limits", "meas.done", "ui.start",
              "lift.up"):
        assert w.runner._emit(c) is None
        assert w.runner.get_state().gate_reason == "not_listed"
    assert (len(w.th.sent), len(w.lift.commands), len(w.ss.requests)) == (n_th, n_lift, n_ss)


def test_sender_calls_gate_every_time(monkeypatch):
    """gate が呼ばれた回数（th・昇降機・ss への送りの数だけ）。"""
    calls = []
    real = core.gate

    def spy(command, snapshot, params=Params()):
        calls.append(command)
        return real(command, snapshot, params)
    monkeypatch.setattr(core, "gate", spy)
    w = World([WPanel(3.0, 0.0)], lift_height_mm=0.0)
    w.to_select()
    w.lift.height_mm = 300
    w.start()
    w.until_phase("FINISH")
    sent = (len(w.th.sent) + w.lift.lowers() + len(w.ss.requests))
    assert sent > 5
    assert sum(1 for c in calls if c.startswith("meas.") or c in (
        core.CMD_LIFT_LOWER, core.CMD_SS_REQUEST)) >= sent


# --- 関門が通さない様子では、動く指示と下降の命令が出ない --------------------------------

def at_goal_after_lower(w):
    """LOWER を終えて GOAL に入った直後（まだ meas.goal を出していない）で止める。"""
    w.to_select()
    w.lift.height_mm = 200
    w.start()
    w.until_phase("GOAL")
    assert w.th_cmds() == []


def moving_none(w):
    assert not (set(w.th_cmds()) & MOVING)


def test_gate_blocks_goal_when_lift_not_at_bottom():
    w = World([WPanel(3.0, 0.0)])
    at_goal_after_lower(w)
    w.lift.height_mm = 100
    w.tick(20)
    moving_none(w)
    assert w.runner.get_state().gate_reason == "lift_not_bottom"


def test_gate_blocks_goal_when_arm_out_and_mechanism_attached():
    w = World([WPanel(3.0, 0.0)], mechanism=True, arm=FakeArm())
    at_goal_after_lower(w)
    w.lift.arm_stowed = False
    w.tick(20)
    moving_none(w)
    assert w.runner.get_state().gate_reason == "arm_not_stowed"
    w.lift.arm_stowed = True                      # 条件がそろえば出る
    w.tick(3)
    assert "meas.goal" in w.th_cmds()


def test_gate_blocks_goal_when_ss_is_busy_with_another_request():
    w = World([WPanel(3.0, 0.0)])
    at_goal_after_lower(w)
    w.ss.external_busy = True
    w.tick(20)
    moving_none(w)
    assert w.runner.get_state().gate_reason == "ss_busy"


def test_gate_blocks_goal_when_th_status_is_stale_or_version_differs():
    w = World([WPanel(3.0, 0.0)])
    at_goal_after_lower(w)
    w.th.model.status_frozen = True
    w.th.model.status_ms -= 1000                   # 配信が止まって 1 s 経った（th_status_stale_ms 500 を超える）
    w.tick(20)
    moving_none(w)
    assert w.runner.get_state().gate_reason == "state_unreadable"
    w2 = World([WPanel(3.0, 0.0)])
    at_goal_after_lower(w2)
    w2.th.model.interface_version = "9"
    w2.th.heartbeat_version = "9"
    w2.tick(20)
    moving_none(w2)
    assert w2.runner.get_state().gate_reason == "version_mismatch"


def test_gate_blocks_lowering_when_arm_out_or_ss_busy_or_arm_link_lost():
    for kw, reason in ((dict(arm_stowed=False), "arm_not_stowed"),
                       (dict(arm_link_ok=False), "arm_link_lost")):
        w = World([WPanel(3.0, 0.0)], mechanism=True, arm=FakeArm(), params=Params(
            lift_lower_timeout_s=1000))
        w.to_select()
        w.lift.height_mm = 300
        for k, v in kw.items():
            setattr(w.lift, k, v)
        w.start()
        n = w.lift.lowers()
        w.tick(30)
        assert w.lift.lowers() == n and w.th_cmds() == []
        assert w.runner.get_state().gate_reason == reason
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.lift.height_mm = 300
    w.start()
    n = w.lift.lowers()
    w.ss.external_busy = True
    w.tick(30)
    assert w.lift.lowers() == n and w.runner.get_state().gate_reason == "ss_busy"


def test_lowering_stops_from_the_next_step_when_ss_leaves_idle_midway():
    """下げている途中で ss が IDLE でなくなったら、次の step から下降の命令を出さない。"""
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.lift.height_mm = 1000
    w.start()
    w.tick(3)
    n = w.lift.lowers()
    assert 0 < n < 10
    w.ss.external_busy = True
    w.tick(1)                                      # ss の状態が変わった step
    assert w.lift.lowers() == n                    # この step でもう出さない
    w.tick(20)
    assert w.lift.lowers() == n and w.phase == "LOWER"
    w.ss.external_busy = False                      # 戻れば下げ続ける
    w.until_phase("GOAL")
    assert w.lift.height_mm == 0


def test_lowering_stops_when_arm_controller_link_drops_midway():
    w = World([WPanel(3.0, 0.0)], mechanism=True, arm=FakeArm(),
              params=Params(lift_lower_timeout_s=1000))
    w.to_select()
    w.lift.height_mm = 1000
    w.start()
    w.tick(3)
    n = w.lift.lowers()
    w.lift.arm_link_ok = False
    w.tick(1)
    w.tick(20)
    assert w.lift.lowers() == n


def test_no_resume_without_gate_conditions_even_after_run():
    w = World([WPanel(3.0, 0.0)], th_kw=dict(nav_time_ms=5000))
    w.to_select()
    w.start()
    w.until(lambda: w.th.model.state == "NAV")
    w.th.model.human_jog()
    w.until_phase("PAUSED")
    assert w.cmd("auto.run").accepted
    w.ss.external_busy = True                      # 受け付けのあとで ss が動いた
    w.tick(20)
    assert "meas.start" not in w.th_cmds()


# --- 走っている間の見張り --------------------------------------------------------------

def test_drive_watch_drift_of_50mm_during_nav_sends_stop():
    w = World([WPanel(3.0, 0.0)], th_kw=dict(nav_time_ms=5000))
    w.to_select()
    w.start()
    w.until(lambda: w.th.model.state == "NAV")
    w.lift.height_mm = 49
    w.tick(5)
    assert "meas.stop" not in w.th_cmds()
    w.lift.height_mm = 50
    w.tick(3)
    assert w.th_cmds().count("meas.stop") == 1 and w.th.model.state == "PAUSE"
    assert w.runner.get_state().gate_reason in ("lift_drift", "")


def test_drive_watch_stale_height_during_nav_sends_stop():
    w = World([WPanel(3.0, 0.0)], th_kw=dict(nav_time_ms=5000))
    w.to_select()
    w.start()
    w.until(lambda: w.th.model.state == "NAV")
    w.lift.height_readable = False
    w.until(lambda: "meas.stop" in w.th_cmds(), 5000)


def test_drive_watch_stale_arm_state_with_mechanism_sends_stop():
    w = World([WPanel(3.0, 0.0)], mechanism=True, arm=FakeArm(), th_kw=dict(nav_time_ms=5000))
    w.to_select()
    w.start()
    w.until(lambda: w.th.model.state == "NAV")
    w.lift.arm_readable = False
    w.until(lambda: "meas.stop" in w.th_cmds(), 5000)


def test_drive_watch_keeps_watching_when_th_status_goes_stale():
    """th の状態が古くなっても、最後に読めた状態で見張る（drive_watch の決め）。"""
    w = World([WPanel(3.0, 0.0)], th_kw=dict(nav_time_ms=60_000))
    w.to_select()
    w.start()
    w.until(lambda: w.th.model.state == "NAV")
    w.th.model.status_frozen = True
    w.tick(10)
    w.lift.height_mm = 80
    w.tick(3)
    assert "meas.stop" in w.th_cmds()


def test_drive_watch_does_not_stop_when_th_is_waiting():
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.lift.height_mm = 300                         # WAIT の間の昇降機の高さでは止めない
    w.tick(5)
    assert "meas.stop" not in w.th_cmds()


# --- 口が無い（None）-------------------------------------------------------------------

@pytest.mark.parametrize("missing", ["lift", "ss", "th", "all"])
def test_missing_ports_send_nothing_and_do_not_crash(missing):
    th = FakeTh()
    lift, ss = FakeLift(height_mm=200), FakeSs()
    kw = dict(th=th, lift=lift, ss=ss)
    if missing == "all":
        kw = dict(th=None, lift=None, ss=None)
    else:
        kw[missing] = None
    r = Runner(kw["th"], kw["lift"], kw["ss"], None)
    sim = Sim(r, th=th, lift=lift, ss=ss)
    sim.tick(2)
    th.model.human_enter_auto()
    sim.tick(2)
    r.command("auto.begin")
    sim.tick(100)
    r.command("auto.start", panel_ids=["P001"])
    r.command("auto.finish", arm_confirmed=True)
    sim.tick(100)
    assert set(th.commands()) <= {"meas.finish"}
    assert not (set(th.commands()) & MOVING)
    assert lift.lowers() == 0 and ss.requests == []


def test_missing_lift_port_waits_in_prepare_with_state_unreadable():
    th, ss = FakeTh(), FakeSs()
    r = Runner(th, None, ss, None)
    sim = Sim(r, th=th, ss=ss)
    sim.tick(2)
    th.model.human_enter_auto()
    sim.tick(2)
    assert r.command("auto.begin").accepted
    sim.tick(50)
    assert r.phase == "PREPARE" and r.get_state().gate_reason == "state_unreadable"
    assert th.sent == [] and ss.requests == []


# --- 時刻は引数だけ・ROS に触れない ----------------------------------------------------

def test_runner_never_reads_a_clock(monkeypatch):
    def boom(*a, **k):
        raise AssertionError("時計を読んだ")
    for name in ("monotonic", "time", "monotonic_ns", "time_ns", "perf_counter", "sleep"):
        monkeypatch.setattr(time, name, boom)
    w = World([WPanel(3.0, 0.0)])
    sc_normal(w)
    sc_jog_and_run(World([WPanel(3.0, 0.0)]))


def test_no_ros_modules_are_imported():
    import fk_auto.fakes, fk_auto.ports, fk_auto.runner  # noqa: F401
    assert not [m for m in sys.modules if m.split(".")[0] in ("rclpy", "rcl_interfaces",
                                                              "std_msgs", "fk_msgs",
                                                              "th_system_msgs")]


def test_same_inputs_and_times_give_same_results():
    def run():
        w = World([WPanel(3.0, 0.0), WPanel(3.0, 4.0)])
        sc_two_rows_and_recheck_regoal(w)
        return ([(c.command, c.x, c.y, c.at_ms) for c in w.th.sent], w.results(), w.now)
    assert run() == run()
