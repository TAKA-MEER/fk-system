"""WP-AUTO-07: 測定機構なしの道の入り口は mechanism_bypass（DetailedDesign-auto.md §2.3）。

- True（既定）: mechanism_attached が True/None でも回り切り、盤は no_mechanism。
  昇降機の状態が読めないときは動く指示を出さない。
- False: arm_link_ok が True でない間は th・ss・昇降機へ何も出さない。
"""
from fk_auto.core import Params
from fk_auto.fakes import FakeArm
from test_runner_world import WPanel, World

MOVING = {"meas.goal", "meas.start", "meas.reroute"}


def test_bypass_true_with_attached_true_or_none_still_no_mechanism():
    for attached in (True, None):
        # attached の値によらず、バイパスなら AT_PANEL で作業を渡さず飛ばす
        w = World([WPanel(3.0, 0.0)], mechanism=False, lift_kw=dict(mechanism_attached=attached),
                  arm=FakeArm())
        w.to_select()
        w.start()
        w.until_phase("FINISH")
        assert w.results() == [("P001", "SKIPPED", "no_mechanism")]
        assert w.arm.starts == []          # FakeArm へ作業は渡さない
        assert set(w.th_cmds()) & MOVING


def test_bypass_true_lift_unreadable_sends_no_moving_command():
    w = World([WPanel(3.0, 0.0)], mechanism=False)
    w.to_select()
    w.lift.height_readable = False     # 高さの読めた時刻が止まる
    w.tick(30)
    w.start()
    w.tick(200)
    assert not (set(w.th_cmds()) & MOVING)
    assert w.lift.lowers() == 0


def test_bypass_true_lift_missing_sends_no_moving_command():
    w = World([WPanel(3.0, 0.0)], mechanism=False)
    w.to_select()
    w.lift.connected = False           # status() が None
    w.tick(30)
    # PREPARE/LOWER で待つだけ。動く指示も下降も出ない
    assert not (set(w.th_cmds()) & MOVING)
    assert w.lift.lowers() == 0


def test_no_bypass_arm_link_lost_sends_nothing():
    for link in (False, None):
        w = World([WPanel(3.0, 0.0)], mechanism=True, arm=FakeArm(),
                  params=Params(mechanism_bypass=False))
        w.to_select()
        w.lift.arm_link_ok = link
        w.lift.height_mm = 300
        n_ss = len(w.ss.requests)
        w.start()
        w.tick(200)
        assert not (set(w.th_cmds()) & MOVING), w.th_cmds()
        assert w.lift.lowers() == 0
        assert len(w.ss.requests) == n_ss or w.phase == "SURVEY"
        # SURVEY への ss 依頼も関門が止める（依頼は出ない）
        assert len(w.ss.requests) == n_ss


def test_no_bypass_arm_link_ok_normal_path_to_arm():
    w = World([WPanel(3.0, 0.0)], mechanism=True, arm=FakeArm(),
              params=Params(mechanism_bypass=False))
    w.to_select()
    w.start()
    w.until_phase("FINISH")
    assert w.results() == [("P001", "DONE", "")]
    assert w.arm.starts == ["P001"]


def test_bypass_state_view_reports_setting():
    w = World([WPanel(3.0, 0.0)], mechanism=False)
    assert w.runner.get_state().mechanism_bypass is True
    w2 = World([WPanel(3.0, 0.0)], mechanism=True, arm=FakeArm(),
               params=Params(mechanism_bypass=False))
    assert w2.runner.get_state().mechanism_bypass is False
