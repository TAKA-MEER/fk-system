"""WP-AUTO-01 結合試験: auto_runner と fake_th を ROS 2 で通す。
待つ上限は conftest のパラメータから計算する（固定の長い sleep にしない）。"""
from conftest import FAKE_PARAMS, MARGIN_S, RUNNER_PARAMS, wait_until

STALL_S = RUNNER_PARAMS['runner_stall_ms'] / 1000.0
STALE_S = RUNNER_PARAMS['th_status_stale_ms'] / 1000.0
LINK_TIMEOUT_S = FAKE_PARAMS['measure_link_timeout_ms'] / 1000.0
STATE_S = RUNNER_PARAMS['state_period_ms'] / 1000.0


def test_a_heartbeat_reaches_fake_th_and_command_is_not_rejected_for_link(harness):
    """(a) 生存確認が届いて link_ok が true になり、指示の拒否理由が version_mismatch・link_lost ではない"""
    assert wait_until(harness.fake.link_ok, MARGIN_S + LINK_TIMEOUT_S)
    reply = harness.send_command('a1')
    assert not reply.accepted
    assert reply.reject_reason not in ('version_mismatch', 'link_lost')
    assert reply.reject_reason == 'state_not_accepting'


def test_a_empty_command_id_is_bad_argument(harness):
    assert wait_until(harness.fake.link_ok, MARGIN_S + LINK_TIMEOUT_S)
    assert harness.send_command('').reject_reason == 'bad_argument'


def test_b_version_mismatch(harness_v2):
    """(b) fake_th の版が "2" だと、link_ok が false・指示は version_mismatch・gate_reason が version_mismatch"""
    h = harness_v2
    assert wait_until(lambda: h.gate_reason == 'version_mismatch', MARGIN_S + STATE_S)
    # 生存確認が 1 つも届く前は link_lost になるので、届いてから拒否理由を見る
    assert wait_until(lambda: h.fake._last_hb_version is not None, MARGIN_S + LINK_TIMEOUT_S)
    assert h.send_command('b1').reject_reason == 'version_mismatch'
    assert not h.fake.link_ok()
    # 生存確認は出続けているのに（実行器は健全）、版が違うので数えられない
    assert not h.runner._hb_timer.is_canceled()


def test_c_state_machine_stall_stops_heartbeat(harness):
    """(c) 状態機械を止めると runner_stall_ms のうちに生存確認が止まり、link_ok が false・指示が link_lost。
    実行器（と生存確認のタイマー）は動いたまま。"""
    h = harness
    assert wait_until(h.fake.link_ok, MARGIN_S + LINK_TIMEOUT_S)
    h.runner.stop_state_machine()
    # 止めてから、生存確認が止まる（stall）＋ fake が途絶とみなす（timeout）＋余裕
    assert wait_until(lambda: not h.fake.link_ok(), STALL_S + LINK_TIMEOUT_S + MARGIN_S)
    assert h.send_command('c1').reject_reason == 'link_lost'
    assert not h.runner._hb_timer.is_canceled()          # 実行器は生きている
    # 状態機械を戻せば、生存確認も戻る（止まったのは判断が止まったからだと分かる）
    h.runner.resume_state_machine()
    assert wait_until(h.fake.link_ok, MARGIN_S + LINK_TIMEOUT_S)


def test_d_th_status_stale_is_state_unreadable(harness):
    """(d) fake_th を止める（状態が届かなくなる）と、th_status_stale_ms のうちに gate_reason が state_unreadable"""
    h = harness
    assert wait_until(lambda: h.gate_reason == '', MARGIN_S + STALE_S)
    h.fake.stop_status()
    assert wait_until(lambda: h.gate_reason == 'state_unreadable', STALE_S + STATE_S + MARGIN_S)


def test_c2_step_that_cannot_finish_stops_heartbeat(harness):
    """(c') step が最後まで回らない（例外で止まる）と、回った時刻は更新されず、生存確認が止まる。
    step を呼び終えてから「回った時刻」を記録することを縛る（呼ぶ前に記録すると生存確認が出続ける）。"""
    h = harness
    assert wait_until(h.fake.link_ok, MARGIN_S + LINK_TIMEOUT_S)
    h.break_step = True
    assert wait_until(lambda: not h.fake.link_ok(), STALL_S + LINK_TIMEOUT_S + MARGIN_S)
    assert h.errors                                       # step の失敗は握りつぶされず表に出ている
    assert not h.runner._hb_timer.is_canceled()
    h.break_step = False
    assert wait_until(h.fake.link_ok, MARGIN_S + LINK_TIMEOUT_S)
