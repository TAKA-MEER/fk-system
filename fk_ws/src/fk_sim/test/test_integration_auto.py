"""WP-AUTO-03 結合試験: auto_runner（FakeLift・FakeSs を渡す）と fake_th（FakeThNode）を ROS 2 で通し、
/fk/auto/command で操作して /fk/auto/* を見る。"""
from conftest import ALLOWED_CMDS, MARGIN_S, wait_until

LONG_S = 30.0


def ids_of(h):
    return [p.panel_id for p in h.layouts[-1].panels] if h.layouts else []


def result_rows(h):
    return [(r.panel_id, r.result, r.reason) for r in h.results[-1].results] if h.results else []


def to_select(h):
    h.enter_auto()
    assert h.command('auto.begin')[0]
    assert wait_until(lambda: len(ids_of(h)) == 2 and h.phase == 'SELECT', LONG_S)


def test_e_two_panels_are_visited_and_listed(make_harness):
    """(e) 回り切る: 配置図 → 選ぶ → 2 つとも SKIPPED/no_mechanism → FINISH。指示は表の 6 つだけ。"""
    h = make_harness(ports=True, fake_params=dict(nav_time_ms=300))
    to_select(h)
    assert h.layouts[-1].map_instance_id.startswith('map-')
    assert h.layouts[-1].survey_reason == ''
    assert h.command('auto.start', ids=ids_of(h))[0]
    assert wait_until(lambda: h.phase == 'FINISH', LONG_S), h.state
    assert sorted(result_rows(h)) == [('P001', 'SKIPPED', 'no_mechanism'),
                                      ('P002', 'SKIPPED', 'no_mechanism')]
    assert h.th_commands and set(h.th_commands) <= ALLOWED_CMDS
    assert h.th_commands.count('meas.goal') == 2
    assert not h.errors


def test_f_without_lift_and_ss_ports_nothing_moves(make_harness):
    """(f) 口が無ければ動かない: meas.goal は届かず、gate_reason は state_unreadable。"""
    h = make_harness(ports=False)
    h.enter_auto()
    assert h.command('auto.begin')[0]
    assert wait_until(lambda: h.phase == 'PREPARE' and h.gate_reason == 'state_unreadable', MARGIN_S)
    import time
    time.sleep(1.0)
    assert h.phase == 'PREPARE' and 'meas.goal' not in h.th_commands
    assert not (set(h.th_commands) - ALLOWED_CMDS)


def test_g_jog_pauses_and_run_continues_to_the_end(make_harness):
    """(g) 走っている間にジョグ → PAUSED → auto.run → 続きから回り切る。"""
    h = make_harness(ports=True, panels=((3.0, 0.0),), fake_params=dict(nav_time_ms=2000))
    h.enter_auto()
    assert h.command('auto.begin')[0]
    assert wait_until(lambda: h.phase == 'SELECT' and ids_of(h), LONG_S)
    assert h.command('auto.start', ids=ids_of(h))[0]
    assert wait_until(lambda: h.state.th_state == 'NAV', LONG_S)
    h.fake.do(h.fake.model.human_jog)
    assert wait_until(lambda: h.phase == 'PAUSED', LONG_S)
    import time
    time.sleep(0.5)
    assert h.phase == 'PAUSED' and 'meas.start' not in h.th_commands     # 自分からは発進しない
    assert h.command('auto.run')[0]
    assert wait_until(lambda: h.phase == 'FINISH', LONG_S), h.state
    assert result_rows(h) == [('P001', 'SKIPPED', 'no_mechanism')]
    assert h.th_commands.count('meas.goal') == 1 and h.th_commands.count('meas.start') == 1


def test_h_th_rejection_reaches_the_screen(make_harness):
    """(h) th が meas.goal を拒否 → th_reject_reason に出て SELECT に戻る。"""
    h = make_harness(ports=True, panels=((3.0, 0.0),))
    h.enter_auto()
    assert h.command('auto.begin')[0]
    assert wait_until(lambda: h.phase == 'SELECT' and ids_of(h), LONG_S)
    h.fake.do(setattr, h.fake.model, 'corner_obstacle', True)
    assert h.command('auto.start', ids=ids_of(h))[0]
    assert wait_until(lambda: h.phase == 'SELECT' and h.state.th_reject_reason == 'corner_obstacle',
                      LONG_S), h.state
    assert h.th_commands.count('meas.goal') == 1


def test_command_service_rejects_with_reason(make_harness):
    h = make_harness(ports=False)
    assert wait_until(lambda: h.state is not None, MARGIN_S)
    assert h.command('auto.start', ids=['P001']) == (False, 'phase_not_accepting')
    assert h.command('auto.bogus') == (False, 'bad_argument')
