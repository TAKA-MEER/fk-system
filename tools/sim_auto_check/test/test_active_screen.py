"""在席（/ui/active_screen）の欄を組む純関数の試験（ホストで回る）。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from check import ACTIVE_SCREEN_PERIOD_S, active_screen_fields  # noqa: E402


def test_active_screen_is_s60_and_interacting():
    f = active_screen_fields("fk-sim-0123abcd", 12.5, 500)
    assert f["screen_id"] == "S-60"
    assert f["client_id"] == "fk-sim-0123abcd"
    assert f["interacting"] is True
    assert f["last_input_sec"] == 12
    assert f["last_input_nanosec"] == 500


def test_active_screen_period_is_half_second():
    assert ACTIVE_SCREEN_PERIOD_S == 0.5  # 2 Hz
