"""WP-AUTO-10 結合試験: 奥行き 2 m の盤と 1 m の盤を選ぶと、正面側を両方回ったあと
2 m の盤の背面側を回り、結果に側が付く（偽の th・偽の ss を ROS 2 で通す）。"""
import pytest

from conftest import wait_until
from test_integration_ss import SsHarness, ids_of


@pytest.fixture
def make_ss_harness(ros_context):
    made = []

    def make(**kw):
        h = SsHarness(**kw)
        made.append(h)
        return h
    yield make
    for h in made:
        h.close()

LONG_S = 30.0


def result_rows(h):
    return [(r.panel_id, r.result, r.reason, r.side)
            for r in h.results[-1].results] if h.results else []


def test_deep_panel_back_after_all_front(make_ss_harness):
    h = make_ss_harness(panels=[(3.0, 0.0, 2.0), (3.0, 3.0, 1.0)],
                        fake_params=dict(nav_time_ms=300))
    h.enter_auto()
    assert h.command('auto.begin')[0]
    assert wait_until(lambda: len(ids_of(h)) == 2 and h.phase == 'SELECT', LONG_S), h.state
    assert h.command('auto.start', ids=ids_of(h))[0]
    assert wait_until(lambda: h.phase == 'FINISH', LONG_S), h.state
    rows = result_rows(h)
    assert rows == [
        ('P001', 'SKIPPED', 'no_mechanism', 'front'),
        ('P002', 'SKIPPED', 'no_mechanism', 'front'),
        ('P001', 'SKIPPED', 'no_mechanism', 'back'),
    ], rows
    # 状態の側も背面側の訪問で back になる（FINISH では空に戻るので、結果の側で見る）
    assert not h.errors
