"""fake_th 単体（auto_runner なし）: 版の違う生存確認は受信に数えない・拒否の順。"""
from conftest import MARGIN_S, wait_until


def test_heartbeat_with_other_version_is_not_counted(harness_v2):
    h = harness_v2
    assert wait_until(lambda: h.fake.model._last_hb_version == '1', MARGIN_S)
    assert h.fake.model._last_valid_hb_ms is None
    assert not h.fake.link_ok()
