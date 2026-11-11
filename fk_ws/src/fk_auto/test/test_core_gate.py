"""§4 の表: gate（§4.1）と drive_watch（§4.2）の試験。

試験名の接頭辞が表の行に対応する。
  test_gate_move_*   : meas.goal・meas.start・meas.reroute の行
  test_gate_halting_*: meas.stop・meas.abort・meas.finish の行
  test_gate_lower_*  : 昇降機を下げる の行
  test_gate_ss_*     : ss への依頼 の行
  test_gate_unlisted_*: 表に無い指示 の行
  test_watch_*       : drive_watch の行
"""
from dataclasses import replace

import pytest

from fk_auto.core import (CMD_LIFT_LOWER, CMD_SS_REQUEST, LiftStatus, Params, Snapshot, SsStatus,
                          ThStatus, drive_watch, gate)

P = Params()
NOW = 10_000.0


def good(**over):
    """全部そろった様子。th は WAIT・下端・腕は格納位置・測定機構あり・腕と通信できる・ss は IDLE。"""
    th = ThStatus(read_ms=NOW - 100, interface_version="1", mode="AUTO_MEASURE", state="WAIT",
                  pose_valid=True, map_instance_id="m", reentry_required=False)
    lift = LiftStatus(height_read_ms=NOW - 100, at_bottom=True, height_above_bottom_mm=0.0,
                      arm_read_ms=NOW - 100, arm_stowed=True, mechanism_attached=True,
                      arm_link_ok=True)
    ss = SsStatus(read_ms=NOW - 100, state="IDLE")
    snap = Snapshot(now_ms=NOW, th=th, lift=lift, ss=ss)
    return replace(snap, **over)


def with_th(snap, **kw):
    return replace(snap, th=replace(snap.th, **kw))


def with_lift(snap, **kw):
    return replace(snap, lift=replace(snap.lift, **kw))


def with_ss(snap, **kw):
    return replace(snap, ss=replace(snap.ss, **kw))


MOVING = ["meas.goal", "meas.start", "meas.reroute"]
HALTING = ["meas.stop", "meas.abort", "meas.finish"]


def reason(cmd, snap, p=P):
    r = gate(cmd, snap, p)
    assert not r.allowed
    return r.reason


# --- meas.goal / start / reroute --------------------------------------------

@pytest.mark.parametrize("cmd", MOVING)
def test_gate_move_all_ok_passes(cmd):
    r = gate(cmd, good(), P)
    assert r.allowed and r.reason is None


@pytest.mark.parametrize("cmd", MOVING)
def test_gate_move_th_missing(cmd):
    assert reason(cmd, replace(good(), th=None)) == "state_unreadable"


@pytest.mark.parametrize("cmd", MOVING)
def test_gate_move_th_never_read(cmd):
    assert reason(cmd, with_th(good(), read_ms=None)) == "state_unreadable"


@pytest.mark.parametrize("cmd", MOVING)
def test_gate_move_th_stale_boundary(cmd):
    assert gate(cmd, with_th(good(), read_ms=NOW - 500), P).allowed      # ちょうど 500 は古くない
    assert reason(cmd, with_th(good(), read_ms=NOW - 501)) == "state_unreadable"


@pytest.mark.parametrize("cmd", MOVING)
def test_gate_move_version_mismatch(cmd):
    assert reason(cmd, with_th(good(), interface_version="2")) == "version_mismatch"
    assert reason(cmd, with_th(good(), interface_version=None)) == "version_mismatch"


@pytest.mark.parametrize("cmd", MOVING)
def test_gate_move_reentry_required(cmd):
    assert reason(cmd, with_th(good(), reentry_required=True)) == "reentry_required"


@pytest.mark.parametrize("cmd", MOVING)
def test_gate_move_lift_missing_or_unreadable(cmd):
    assert reason(cmd, replace(good(), lift=None)) == "state_unreadable"
    assert reason(cmd, with_lift(good(), at_bottom=None)) == "state_unreadable"
    assert reason(cmd, with_lift(good(), height_read_ms=None)) == "state_unreadable"
    assert reason(cmd, with_lift(good(), height_read_ms=NOW - 501)) == "state_unreadable"
    assert gate(cmd, with_lift(good(), height_read_ms=NOW - 500), P).allowed


@pytest.mark.parametrize("cmd", MOVING)
def test_gate_move_lift_not_bottom(cmd):
    assert reason(cmd, with_lift(good(), at_bottom=False)) == "lift_not_bottom"


@pytest.mark.parametrize("cmd", MOVING)
def test_gate_move_arm_unreadable(cmd):
    assert reason(cmd, with_lift(good(), arm_stowed=None)) == "state_unreadable"
    assert reason(cmd, with_lift(good(), arm_read_ms=None)) == "state_unreadable"
    assert reason(cmd, with_lift(good(), arm_read_ms=NOW - 501)) == "state_unreadable"


@pytest.mark.parametrize("cmd", MOVING)
def test_gate_move_arm_not_stowed(cmd):
    assert reason(cmd, with_lift(good(), arm_stowed=False)) == "arm_not_stowed"


@pytest.mark.parametrize("cmd", MOVING)
def test_gate_move_no_mechanism_waives_arm(cmd):
    """測定機構が付いていないと読めたときだけ腕の条件を外す。"""
    assert gate(cmd, with_lift(good(), mechanism_attached=False, arm_stowed=False), P).allowed
    assert gate(cmd, with_lift(good(), mechanism_attached=False, arm_stowed=None,
                               arm_read_ms=None), P).allowed
    # 付いているか分からない（None）→ 付いているとみなす
    assert reason(cmd, with_lift(good(), mechanism_attached=None, arm_stowed=False)) == "arm_not_stowed"
    assert reason(cmd, with_lift(good(), mechanism_attached=None, arm_stowed=None)) == "state_unreadable"


@pytest.mark.parametrize("cmd", MOVING)
def test_gate_move_no_mechanism_does_not_waive_lift_bottom(cmd):
    assert reason(cmd, with_lift(good(), mechanism_attached=False, at_bottom=False)) == "lift_not_bottom"


@pytest.mark.parametrize("cmd", MOVING)
def test_gate_move_ss_unreadable(cmd):
    assert reason(cmd, replace(good(), ss=None)) == "state_unreadable"
    assert reason(cmd, with_ss(good(), state=None)) == "state_unreadable"
    assert reason(cmd, with_ss(good(), read_ms=None)) == "state_unreadable"
    assert reason(cmd, with_ss(good(), read_ms=NOW - 1001)) == "state_unreadable"
    assert gate(cmd, with_ss(good(), read_ms=NOW - 1000), P).allowed


@pytest.mark.parametrize("cmd", MOVING)
@pytest.mark.parametrize("state", ["BUSY", "RAISING", "SCANNING", "CLEANUP", "x"])
def test_gate_move_ss_busy(cmd, state):
    assert reason(cmd, with_ss(good(), state=state)) == "ss_busy"


def test_gate_move_reason_order_when_several_missing():
    """複数欠けたときは th 読めない → 版 → 入り直し → 昇降機 → 腕 → ss の順。"""
    snap = with_ss(with_lift(with_th(good(), reentry_required=True, interface_version="9"),
                             at_bottom=False, arm_stowed=False), state="BUSY")
    assert reason("meas.goal", snap) == "version_mismatch"
    snap = with_th(snap, interface_version="1")
    assert reason("meas.goal", snap) == "reentry_required"
    snap = with_th(snap, reentry_required=False)
    assert reason("meas.goal", snap) == "lift_not_bottom"
    snap = with_lift(snap, at_bottom=True)
    assert reason("meas.goal", snap) == "arm_not_stowed"
    snap = with_lift(snap, arm_stowed=True)
    assert reason("meas.goal", snap) == "ss_busy"
    assert reason("meas.goal", replace(snap, th=None)) == "state_unreadable"


# --- meas.stop / abort / finish ---------------------------------------------

@pytest.mark.parametrize("cmd", HALTING)
def test_gate_halting_always_pass_even_if_nothing_readable(cmd):
    assert gate(cmd, Snapshot(now_ms=NOW), P).allowed
    assert gate(cmd, Snapshot(now_ms=NOW, th=None, lift=None, ss=None), P).allowed


@pytest.mark.parametrize("cmd", HALTING)
def test_gate_halting_pass_with_every_condition_bad(cmd):
    snap = with_ss(with_lift(with_th(good(), reentry_required=True, interface_version="9",
                                     read_ms=NOW - 9999), at_bottom=False, arm_stowed=False,
                             arm_link_ok=False), state="BUSY")
    assert gate(cmd, snap, P).allowed


# --- 昇降機を下げる ----------------------------------------------------------

def test_gate_lower_all_ok_passes():
    assert gate(CMD_LIFT_LOWER, good(), P).allowed


def test_gate_lower_arm_unreadable_and_not_stowed():
    assert reason(CMD_LIFT_LOWER, with_lift(good(), arm_stowed=None)) == "state_unreadable"
    assert reason(CMD_LIFT_LOWER, with_lift(good(), arm_read_ms=NOW - 501)) == "state_unreadable"
    assert reason(CMD_LIFT_LOWER, replace(good(), lift=None)) == "arm_link_lost"
    assert reason(CMD_LIFT_LOWER, with_lift(good(), arm_stowed=False)) == "arm_not_stowed"


def test_gate_lower_ss_conditions():
    assert reason(CMD_LIFT_LOWER, replace(good(), ss=None)) == "state_unreadable"
    assert reason(CMD_LIFT_LOWER, with_ss(good(), read_ms=NOW - 1001)) == "state_unreadable"
    assert reason(CMD_LIFT_LOWER, with_ss(good(), state="BUSY")) == "ss_busy"


def test_gate_lower_arm_link_lost_including_unreadable():
    assert reason(CMD_LIFT_LOWER, with_lift(good(), arm_link_ok=False)) == "arm_link_lost"
    assert reason(CMD_LIFT_LOWER, with_lift(good(), arm_link_ok=None)) == "arm_link_lost"
    # 付いているか分からないときは付いているとみなす
    assert reason(CMD_LIFT_LOWER, with_lift(good(), mechanism_attached=None,
                                            arm_link_ok=False)) == "arm_link_lost"


def test_gate_lower_no_mechanism_waives_arm_and_link():
    snap = with_lift(good(), mechanism_attached=False, arm_stowed=False, arm_link_ok=False)
    assert gate(CMD_LIFT_LOWER, snap, P).allowed
    # 測定機構なしでも ss の条件は外さない
    assert reason(CMD_LIFT_LOWER, with_ss(snap, state="BUSY")) == "ss_busy"


def test_gate_lower_does_not_need_th_or_bottom():
    """下げる命令は th が読めなくても・下端でなくても通る（th 側の条件は表に無い）。"""
    assert gate(CMD_LIFT_LOWER, with_lift(replace(good(), th=None), at_bottom=False), P).allowed


# --- ss への依頼 --------------------------------------------------------------

def test_gate_ss_all_ok_passes():
    assert gate(CMD_SS_REQUEST, good(), P).allowed


def test_gate_ss_th_unreadable():
    assert reason(CMD_SS_REQUEST, replace(good(), th=None)) == "state_unreadable"
    assert reason(CMD_SS_REQUEST, with_th(good(), read_ms=NOW - 501)) == "state_unreadable"


@pytest.mark.parametrize("state", ["NAV", "PAUSE", "ALIGN", "BLOCKED", "ADJUST", "ESTOP", None])
def test_gate_ss_th_not_waiting(state):
    assert reason(CMD_SS_REQUEST, with_th(good(), state=state)) == "th_not_waiting"


def test_gate_ss_lift_unreadable():
    assert reason(CMD_SS_REQUEST, replace(good(), lift=None)) == "state_unreadable"
    assert reason(CMD_SS_REQUEST, with_lift(good(), at_bottom=None)) == "state_unreadable"
    assert reason(CMD_SS_REQUEST, with_lift(good(), height_read_ms=NOW - 501)) == "state_unreadable"


def test_gate_ss_arm_unreadable_and_not_stowed():
    assert reason(CMD_SS_REQUEST, with_lift(good(), arm_stowed=None)) == "state_unreadable"
    assert reason(CMD_SS_REQUEST, with_lift(good(), arm_stowed=False)) == "arm_not_stowed"


def test_gate_ss_no_mechanism_waives_arm():
    assert gate(CMD_SS_REQUEST, with_lift(good(), mechanism_attached=False, arm_stowed=None), P).allowed


def test_gate_ss_does_not_need_bottom():
    assert gate(CMD_SS_REQUEST, with_lift(good(), at_bottom=False), P).allowed


# --- 表に無い指示 --------------------------------------------------------------

@pytest.mark.parametrize("cmd", ["meas.turn", "meas.translate", "meas.set_limits", "meas.done",
                                 "meas.unknown", "", "foo", "MEAS.GOAL", "meas.goal ", "ui.estop"])
def test_gate_unlisted_not_passed_even_when_everything_ok(cmd):
    assert reason(cmd, good()) == "not_listed"


def test_gate_unlisted_not_passed_with_nothing_readable():
    assert reason("meas.turn", Snapshot(now_ms=NOW)) == "not_listed"


# --- drive_watch --------------------------------------------------------------

@pytest.mark.parametrize("state", ["NAV", "ALIGN", "BLOCKED"])
def test_watch_ok_when_at_bottom(state):
    assert not drive_watch(with_th(good(), state=state), P).stop


@pytest.mark.parametrize("state", ["NAV", "ALIGN", "BLOCKED"])
def test_watch_drift_stops(state):
    snap = with_lift(with_th(good(), state=state), height_above_bottom_mm=50.0, at_bottom=False)
    r = drive_watch(snap, P)
    assert r.stop and r.reason == "lift_drift"


def test_watch_drift_boundary_is_inclusive():
    snap = with_th(good(), state="NAV")
    assert not drive_watch(with_lift(snap, height_above_bottom_mm=49.9), P).stop
    assert drive_watch(with_lift(snap, height_above_bottom_mm=50.0), P).stop
    assert drive_watch(with_lift(snap, height_above_bottom_mm=500.0), P).stop


def test_watch_drift_threshold_is_the_parameter():
    snap = with_lift(with_th(good(), state="NAV"), height_above_bottom_mm=30.0)
    assert not drive_watch(snap, P).stop
    assert drive_watch(snap, replace(P, lift_drift_stop_mm=30)).stop


@pytest.mark.parametrize("state", ["NAV", "ALIGN", "BLOCKED"])
def test_watch_height_unreadable_stops(state):
    base = with_th(good(), state=state)
    for lift in (None,):
        r = drive_watch(replace(base, lift=lift), P)
        assert r.stop and r.reason == "state_unreadable"
    r = drive_watch(with_lift(base, height_above_bottom_mm=None), P)
    assert r.stop and r.reason == "state_unreadable"
    r = drive_watch(with_lift(base, height_read_ms=None), P)
    assert r.stop and r.reason == "state_unreadable"


def test_watch_height_stale_boundary():
    base = with_th(good(), state="NAV")
    assert not drive_watch(with_lift(base, height_read_ms=NOW - 500), P).stop
    r = drive_watch(with_lift(base, height_read_ms=NOW - 501), P)
    assert r.stop and r.reason == "state_unreadable"


def test_watch_arm_state_stale_stops():
    base = with_th(good(), state="NAV")
    assert not drive_watch(with_lift(base, arm_read_ms=NOW - 500), P).stop
    r = drive_watch(with_lift(base, arm_read_ms=NOW - 501), P)
    assert r.stop and r.reason == "state_unreadable"
    r = drive_watch(with_lift(base, arm_stowed=None), P)
    assert r.stop and r.reason == "state_unreadable"
    r = drive_watch(with_lift(base, arm_read_ms=None), P)
    assert r.stop and r.reason == "state_unreadable"


def test_watch_no_mechanism_ignores_arm_state():
    base = with_th(good(), state="NAV")
    assert not drive_watch(with_lift(base, mechanism_attached=False, arm_stowed=None,
                                     arm_read_ms=None), P).stop
    # 高さが読めなければ測定機構なしでも止める
    assert drive_watch(with_lift(base, mechanism_attached=False, height_read_ms=None), P).stop


def test_watch_adjust_is_not_stopped_by_height():
    """ADJUST（盤前の旋回）は高さで止める対象から外す。"""
    snap = with_lift(with_th(good(), state="ADJUST"), height_above_bottom_mm=1000.0,
                     at_bottom=False, height_read_ms=None, arm_read_ms=None)
    assert not drive_watch(snap, P).stop
    assert not drive_watch(replace(snap, lift=None), P).stop


@pytest.mark.parametrize("state", ["WAIT", "PAUSE", "ESTOP", "CARRY", "IDLE", None])
def test_watch_other_th_states_do_nothing(state):
    snap = with_lift(with_th(good(), state=state), height_above_bottom_mm=1000.0, height_read_ms=None)
    assert not drive_watch(snap, P).stop


def test_watch_th_never_read_does_nothing():
    snap = with_lift(replace(good(), th=None), height_above_bottom_mm=1000.0)
    assert not drive_watch(snap, P).stop


def test_watch_th_stale_nav_still_watches_drift():
    """th の状態が古くても、最後の状態が NAV なら昇降機の離れで止める。"""
    snap = with_lift(with_th(good(), state="NAV", read_ms=NOW - 5000),
                     height_above_bottom_mm=50.0, at_bottom=False)
    r = drive_watch(snap, P)
    assert r.stop and r.reason == "lift_drift"


def test_watch_th_stale_nav_height_stale_stops():
    snap = with_lift(with_th(good(), state="NAV", read_ms=NOW - 5000), height_read_ms=NOW - 501)
    r = drive_watch(snap, P)
    assert r.stop and r.reason == "state_unreadable"


def test_watch_th_stale_nav_ok_when_at_bottom():
    assert not drive_watch(with_th(good(), state="NAV", read_ms=NOW - 5000), P).stop


def test_watch_th_stale_last_state_adjust_does_nothing():
    snap = with_lift(with_th(good(), state="ADJUST", read_ms=NOW - 5000),
                     height_above_bottom_mm=1000.0)
    assert not drive_watch(snap, P).stop
