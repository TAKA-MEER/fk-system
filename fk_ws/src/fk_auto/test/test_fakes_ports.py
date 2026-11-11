"""FakeTh（記録）・FakeLift・FakeSs・FakeArm・NoMechanism の試験。"""
from fk_auto import core
from fk_auto.fakes import FakeArm, FakeLift, FakeSs, FakeTh, SsResponse
from fk_auto.ports import NoMechanism


def test_fakeTh_records_every_command_and_replies_on_next_advance():
    th = FakeTh()
    th.advance(100)
    th.model.human_enter_auto()
    th.send("c1", "meas.goal", th.model.map_instance_id, 1.0, 2.0, 0.5)
    assert th.commands() == ["meas.goal"] and th.sent[0].at_ms == 100
    assert th.poll_replies() == []             # 応答は非同期（次の advance で来る）
    th.advance(200)
    assert th.poll_replies()[0].accepted
    assert th.poll_replies() == []             # 取り出したら消える


def test_fakeTh_status_none_when_unreadable_and_link_loss_switch():
    th = FakeTh()
    assert th.status() is None                 # まだ 1 度も配信されていない
    th.advance(100)
    assert th.status().read_ms == 100
    th.unreadable = True
    assert th.status() is None
    th.unreadable = False
    th.link_alive = False
    th.advance(2000)
    assert not th.model.link_ok()


def test_fakeLift_lowers_step_by_step_stops_at_bottom_and_records():
    lift = FakeLift(height_mm=250, lower_step_mm=100, origin_ok=False)
    lift.advance(100)
    assert not lift.status().at_bottom
    for _ in range(3):
        lift.lower()
    assert lift.height_mm == 0 and lift.status().at_bottom and lift.origin_ok
    lift.lower()
    assert lift.height_mm == 0 and lift.lowers() == 4


def test_fakeLift_human_operating_does_not_move_but_records():
    lift = FakeLift(height_mm=200)
    lift.human_operating = True
    lift.lower()
    assert lift.height_mm == 200 and lift.lowers() == 1
    assert lift.status().operator_is_human


def test_fakeLift_unreadable_and_disconnected():
    lift = FakeLift()
    lift.advance(100)
    lift.height_readable = False
    lift.advance(900)
    assert lift.status().height_read_ms == 100   # 読めた時刻が止まる
    lift.connected = False
    assert lift.status() is None


def test_fakeSs_request_result_cleanup_sequence_and_busy():
    ss = FakeSs(result_delay_ms=200, cleanup_delay_ms=300)
    resp = SsResponse("completed", (core.SsRect(3, 0, 3.14, 1, 0.6),), (-3.14,), (6.28,))
    ss.responses.append(resp)
    ss.advance(0)
    rid = ss.request(-3.14, 6.28, 0, 300)
    assert rid == "s1" and ss.status().state == "RUNNING"
    assert ss.request(0, 1, 0, 1) is None                  # 1 件だけ
    ss.advance(100)
    assert ss.poll_results() == []
    ss.advance(200)
    r = ss.poll_results()[0]
    assert r.request_id == rid and r.reason == "completed" and len(r.panels) == 1
    assert ss.status().state == "CLEANING" and ss.poll_cleanups() == []
    ss.advance(500)
    c = ss.poll_cleanups()[0]
    assert c.request_id == rid and ss.status().state == "IDLE"


def test_fakeSs_cancel_gives_canceled_result_then_cleanup_and_never_flags():
    ss = FakeSs()
    ss.never_result = True
    ss.advance(0)
    rid = ss.request(0, 1, 0, 1)
    ss.advance(10_000)
    assert ss.poll_results() == []
    ss.cancel(rid)
    ss.advance(10_100)
    assert ss.poll_results()[0].reason == "canceled"
    ss.never_cleanup = True
    ss.advance(20_000)
    assert ss.poll_cleanups() == [] and ss.status().state == "CLEANING"


def test_fakeSs_human_override_does_not_restore_lift():
    ss = FakeSs(default=SsResponse("human_override"))
    ss.advance(0)
    ss.request(0, 1, 0, 1)
    ss.advance(1000)
    ss.advance(2000)
    assert ss.poll_cleanups()[0].lift_restored is False


def test_fakeSs_external_busy_and_frozen_status():
    ss = FakeSs()
    ss.advance(100)
    ss.external_busy = True
    assert ss.status().state == "RUNNING" and ss.request(0, 1, 0, 1) is None
    ss.frozen = True
    ss.advance(900)
    assert ss.status().read_ms == 100


def test_noMechanism_refuses_work_and_uses_core_stand_pose():
    arm = NoMechanism()
    p = core.Panel("P001", 3.0, 0.0, 3.14159, 0.3, 1.0)
    assert arm.start_work("P001") is False and arm.poll_results() == []
    assert arm.stand_pose(p) == core.stand_pose(p, 0.389, 0.0)


def test_fakeArm_work_and_cancel():
    arm = FakeArm(work_ms=500, cancel_ms=300)
    arm.advance(0)
    assert arm.start_work("P001")
    arm.advance(400)
    assert arm.poll_results() == []
    arm.advance(500)
    assert arm.poll_results()[0].result == "DONE"
    arm.start_work("P002")
    arm.cancel_work()
    arm.advance(1100)
    assert arm.poll_results()[0].result == "CANCELLED"
