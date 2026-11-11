"""§3 の表: can_run・classify_skip・classify_halt の試験。"""
from dataclasses import replace

import pytest

from fk_auto.core import (HaltEvent, LiftStatus, Params, RunDecision, Snapshot, SkipEvent,
                          SsStatus, ThStatus, can_run, classify_halt, classify_skip)

P = Params()
NOW = 10_000.0


def paused(**over):
    th = ThStatus(read_ms=NOW - 100, interface_version="1", state="PAUSE", pose_valid=True)
    lift = LiftStatus(height_read_ms=NOW - 100, at_bottom=True, height_above_bottom_mm=0.0,
                      arm_read_ms=NOW - 100, arm_stowed=True, mechanism_attached=True,
                      arm_link_ok=True)
    return replace(Snapshot(now_ms=NOW, th=th, lift=lift, ss=SsStatus(NOW - 100, "IDLE")), **over)


def with_th(s, **kw):
    return replace(s, th=replace(s.th, **kw))


def with_lift(s, **kw):
    return replace(s, lift=replace(s.lift, **kw))


def rej(snap):
    r = can_run(snap, P)
    assert r.kind == "reject"
    return r.reason


def test_can_run_all_ok():
    assert can_run(paused(), P) == RunDecision("run")


def test_can_run_th_not_pause():
    for st in ("WAIT", "NAV", "ALIGN", "BLOCKED", "ADJUST", "ESTOP", None):
        assert rej(with_th(paused(), state=st)) == "phase_not_accepting"


def test_can_run_reentry_required():
    assert rej(with_th(paused(), reentry_required=True)) == "reentry_required"


def test_can_run_th_unreadable_and_version():
    assert rej(replace(paused(), th=None)) == "state_unreadable"
    assert rej(with_th(paused(), read_ms=NOW - 501)) == "state_unreadable"
    assert rej(with_th(paused(), interface_version="2")) == "version_mismatch"


def test_can_run_arm_out_or_unreadable():
    assert rej(with_lift(paused(), arm_stowed=False)) == "arm_not_stowed"
    assert rej(with_lift(paused(), arm_stowed=None)) == "state_unreadable"


def test_can_run_no_mechanism_ignores_arm():
    assert can_run(with_lift(paused(), mechanism_attached=False, arm_stowed=False), P).kind == "run"


def test_can_run_lift_not_bottom_means_lower_first_not_reject():
    r = can_run(with_lift(paused(), at_bottom=False), P)
    assert r.kind == "lower_first" and r.reason is None


def test_can_run_lift_unreadable_is_rejected():
    assert rej(with_lift(paused(), at_bottom=None)) == "state_unreadable"
    assert rej(replace(paused(), lift=None)) == "state_unreadable"


def test_can_run_ss_busy():
    assert rej(replace(paused(), ss=SsStatus(NOW - 100, "BUSY"))) == "ss_busy"
    assert rej(replace(paused(), ss=None)) == "state_unreadable"


def test_can_run_arm_out_beats_lower_first():
    assert rej(with_lift(paused(), at_bottom=False, arm_stowed=False)) == "arm_not_stowed"


def test_classify_skip_all_events():
    assert classify_skip(SkipEvent.PATH_NOT_FOUND) == "no_path"
    assert classify_skip(SkipEvent.STAND_AREA_OCCUPIED) == "stand_blocked"
    assert classify_skip(SkipEvent.STAND_MISMATCH_TWICE) == "stand_mismatch"
    assert classify_skip(SkipEvent.TOP_NOT_FOUND) == "top_not_found"
    assert classify_skip(SkipEvent.ARM_SPACE_BLOCKED) == "arm_space_blocked"
    assert classify_skip(SkipEvent.MECHANISM_ABSENT) == "no_mechanism"
    assert classify_skip(SkipEvent.SS_TIMEOUT) == "ss_failed"
    assert classify_skip(SkipEvent.SS_RESULT_UNUSABLE) == "ss_failed"
    assert classify_skip(SkipEvent.CAMERA_TOO_LOW) == "camera_too_low"
    assert {classify_skip(e) for e in SkipEvent} <= {
        "no_path", "stand_blocked", "stand_mismatch", "top_not_found", "arm_space_blocked",
        "no_mechanism", "ss_failed", "camera_too_low"}


def test_classify_halt_all_events():
    assert classify_halt(HaltEvent.ARM_STUCK) == "arm_stuck"
    assert classify_halt(HaltEvent.LIFT_CANNOT_LOWER) == "lift_not_lowered"
    assert classify_halt(HaltEvent.LIFT_OR_ARM_UNREADABLE) == "state_unreadable"
    assert classify_halt(HaltEvent.SS_CLEANUP_NOT_DONE) == "ss_cleanup_timeout"
    assert classify_halt(HaltEvent.SS_LINK_LOST) == "ss_link_lost"
    assert classify_halt(HaltEvent.ARM_LINK_LOST) == "arm_link_lost"


def test_classify_rejects_unknown_event():
    with pytest.raises(KeyError):
        classify_skip("no_path")
    with pytest.raises(KeyError):
        classify_halt(SkipEvent.PATH_NOT_FOUND)
