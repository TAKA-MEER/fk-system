"""ss の口の版 2 の扱い（DetailedDesign-auto.md §2.4。WP-AUTO-09）。

後片付けの待ち方（結果の cleanup_limit_s）と、理由 ceiling_unknown・camera_pose_unknown。
Runner を偽の口で回して、本番の経路（_ss_step・_do_recheck・_do_paused）を縛る。
"""
import dataclasses

import pytest

from fk_auto.core import Params, ss_cleanup_deadline_ms
from fk_auto.fakes import FakeSs, SsResponse
from test_runner_world import WPanel, World, body_rect


# --- 純関数・既定値・偽の ss -------------------------------------------------

def test_params_defaults_for_cleanup_wait():
    p = Params()
    assert p.ss_cleanup_timeout_s == 210 and p.ss_cleanup_margin_s == 10


def test_deadline_uses_result_time_plus_limit_plus_margin_when_limit_is_valid():
    assert ss_cleanup_deadline_ms(5_000, 100.0, 1_000, 10, 210) == 5_000 + 110_000
    assert ss_cleanup_deadline_ms(5_000, 0.0, 1_000, 10, 210) == 5_000 + 10_000


@pytest.mark.parametrize("res_ms,limit", [(None, 100.0), (5_000, None), (5_000, float("nan")),
                                           (5_000, float("inf")), (5_000, -1.0),
                                           (5_000, float("-inf"))])
def test_deadline_falls_back_to_fixed_timeout_from_wait_start(res_ms, limit):
    assert ss_cleanup_deadline_ms(res_ms, limit, 1_000, 10, 210) == 1_000 + 210_000


def test_fake_ss_puts_cleanup_limit_in_results():
    ss = FakeSs(cleanup_delay_ms=3000, default=SsResponse("completed"))
    ss.request(0, 1, 0, 10)
    ss.advance(1000)
    assert ss.poll_results()[0].cleanup_limit_s == pytest.approx(3.0)     # 既定は遅れ / 1000
    ss2 = FakeSs(default=SsResponse("completed", cleanup_limit_s=100.0))
    ss2.request(0, 1, 0, 10)
    ss2.advance(1000)
    assert ss2.poll_results()[0].cleanup_limit_s == 100.0
    ss3 = FakeSs(default=SsResponse("human_override", cleanup_limit_s=100.0))
    ss3.request(0, 1, 0, 10)
    ss3.advance(1000)
    assert ss3.poll_results()[0].cleanup_limit_s == 0.0
    ss4 = FakeSs(cleanup_delay_ms=2000, default=SsResponse("completed"))
    rid = ss4.request(0, 1, 0, 10)
    ss4.cancel(rid)
    ss4.advance(100)
    r = ss4.poll_results()[0]
    assert r.reason == "canceled" and r.cleanup_limit_s == pytest.approx(2.0)


# --- 後片付けの待ち方 --------------------------------------------------------

def to_recheck(w, **ss_attrs):
    """盤前の認識し直しの頭まで進め、ss の様子を差し替える（依頼は次の step で出る）。"""
    w.to_select()
    w.start()
    w.until_phase("RECHECK")
    for k, v in ss_attrs.items():
        setattr(w.ss, k, v)


def with_limit(w, limit):
    base = w.respond
    w.ss.responder = lambda req: dataclasses.replace(base(req), cleanup_limit_s=limit)


def result_time(w):
    """最後の依頼の結果を Runner が受ける tick（結果は依頼から result_delay_ms 後に出る）。"""
    return w.ss.requests[-1].at_ms + w.ss.result_delay_ms


def test_long_cleanup_limit_does_not_halt_even_beyond_the_old_fixed_60s():
    w = World([WPanel(3.0, 0.0)], params=Params(ss_cleanup_timeout_s=60))
    to_recheck(w, cleanup_delay_ms=100_000)
    with_limit(w, 100.0)
    w.until(lambda: len(w.ss.requests) == 2)
    rt = result_time(w)
    w.until(lambda: w.now >= rt + 99_000)
    assert w.phase == "RECHECK"                       # 古い固定 60 s なら止まっていた
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "no_mechanism")]
    assert w.runner.get_state().halt_reason == ""


def test_cleanup_limit_5s_halts_after_15s_from_result_and_not_at_14s():
    w = World([WPanel(3.0, 0.0)])
    to_recheck(w, never_cleanup=True)
    with_limit(w, 5.0)
    w.until(lambda: len(w.ss.requests) == 2)
    rt = result_time(w)
    w.until(lambda: w.now >= rt + 14_000)
    assert w.phase == "RECHECK"
    w.until_phase("HALTED")
    assert rt + 15_000 < w.now <= rt + 15_400
    assert w.runner.get_state().halt_reason == "ss_cleanup_timeout"


def test_no_result_after_cancel_and_no_cleanup_halts_after_210s_from_wait_start():
    w = World([WPanel(3.0, 0.0)], params=Params(ss_result_timeout_s=3))
    to_recheck(w, never_result=True, never_cleanup=True)
    w.ss.poll_results = lambda: []                    # 取り消したあとの結果も届かない
    w.until(lambda: len(w.ss.cancels) == 1)
    t0 = w.ss.cancels[0][1]
    w.until(lambda: w.now >= t0 + 209_000)
    assert w.phase == "RECHECK"
    w.until_phase("HALTED", 20_000)
    assert t0 + 210_000 < w.now <= t0 + 210_500
    assert w.runner.get_state().halt_reason == "ss_cleanup_timeout"


def test_result_arriving_while_waiting_after_cancel_restarts_the_count_from_its_arrival():
    w = World([WPanel(3.0, 0.0)], params=Params(ss_result_timeout_s=3))
    to_recheck(w, never_result=True, never_cleanup=True)
    real_poll = w.ss.poll_results
    w.ss.poll_results = lambda: ([] if w.now < 100_000 else real_poll())
    w.until(lambda: w.now >= 100_000)
    assert w.phase == "RECHECK"                       # 固定の待ちならまだ止まらない時刻
    w.until_phase("HALTED", 20_000)                   # 結果（上限 0.2 s ＋ 余裕 10 s）が届いた時刻から数える
    assert 110_000 < w.now <= 111_000


@pytest.mark.parametrize("limit", [float("nan"), -1.0, float("inf")])
def test_invalid_cleanup_limit_waits_for_the_fixed_timeout(limit):
    w = World([WPanel(3.0, 0.0)], params=Params(ss_cleanup_timeout_s=20))
    to_recheck(w, never_cleanup=True)
    with_limit(w, limit)
    w.until(lambda: len(w.ss.requests) == 2)
    rt = result_time(w)
    w.until(lambda: w.now >= rt + 19_000)
    assert w.phase == "RECHECK"
    w.until_phase("HALTED")
    assert rt + 20_000 < w.now <= rt + 20_400


# --- RECHECK の理由 2 つ -----------------------------------------------------

def with_reason(w, reason):
    base = w.respond
    w.ss.responder = lambda req: dataclasses.replace(base(req), reason=reason)
    return base


def test_recheck_ceiling_unknown_skips_and_ignores_returned_panels():
    w = World([WPanel(3.0, 0.0)])
    to_recheck(w)
    with_reason(w, "ceiling_unknown")                 # 盤は返るが使わない
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "ceiling_unknown")]


def test_recheck_camera_pose_unknown_pauses_and_resumes_only_on_run():
    w = World([WPanel(3.0, 0.0)])
    to_recheck(w)
    base = with_reason(w, "camera_pose_unknown")
    w.until_phase("PAUSED")
    assert w.runner.get_state().gate_reason == "camera_pose_unknown"
    assert w.results() == []                          # 飛ばさず、結果を残さない
    n = len(w.ss.requests)
    assert n == 2
    w.tick(300)                                       # 自分からは依頼し直さない
    assert w.phase == "PAUSED" and len(w.ss.requests) == n
    assert w.runner.get_state().gate_reason == "camera_pose_unknown"   # 出し続ける
    assert w.results() == []
    w.ss.responder = base                             # 向きが分かるようになった
    assert w.cmd("auto.run").accepted
    w.tick(2)
    assert len(w.ss.requests) == n + 1                # RECHECK の頭からやり直して依頼が 1 つ増える
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "no_mechanism")]   # AT_PANEL まで進んだ
    assert "meas.start" not in w.th_cmds()            # th は WAIT のまま（走らせない）
    assert len([c for c in w.th.sent if c.command == "meas.goal"]) == 1


def test_camera_pose_unknown_pause_does_not_use_the_returned_panel():
    w = World([WPanel(3.0, 0.0)])
    to_recheck(w)
    before = w.runner.get_layout().panels[0]
    with_reason(w, "camera_pose_unknown")
    w.recheck_shift = [0.25]                          # 返った盤は 0.25 m ずれている
    w.until_phase("PAUSED")
    assert w.runner.get_layout().panels[0] == before


def test_camera_pose_unknown_pause_waits_for_cleanup_to_complete():
    w = World([WPanel(3.0, 0.0)])
    to_recheck(w, cleanup_delay_ms=5000)
    with_reason(w, "camera_pose_unknown")
    w.until(lambda: len(w.ss.requests) == 2)
    rt = result_time(w)
    w.until(lambda: w.now >= rt + 4_000)
    assert w.phase == "RECHECK" and w.ss.status().state == "CLEANING"
    w.until_phase("PAUSED")
    assert w.now >= rt + 5_000 and w.ss.status().state == "IDLE"


def test_camera_pose_unknown_pause_still_reacts_to_human_leaving():
    w = World([WPanel(3.0, 0.0)])
    to_recheck(w)
    with_reason(w, "camera_pose_unknown")
    w.until_phase("PAUSED")
    w.th.model.human_exit()                           # 人が抜けた
    w.until_phase("IDLE")


# --- SURVEY は今の作りで通る -------------------------------------------------

@pytest.mark.parametrize("reason", ["ceiling_unknown", "camera_pose_unknown"])
def test_survey_uses_partial_panels_and_seen_range_for_new_reasons(reason):
    w = World([WPanel(3.0, 0.0)])
    rect = body_rect(w.panels[0], w.pose())
    w.ss.responses.append(SsResponse(reason, (rect,), (-0.5,), (1.0,)))
    w.to_select()
    lay = w.runner.get_layout()
    assert len(lay.panels) == 1 and lay.survey_reason == reason
    assert len(lay.unseen_from_rad) == 1
