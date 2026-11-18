"""奥行きを測れていない盤（ss の口の版 3。DetailedDesign-auto.md §2.5）の試験。

- to_map・make_panel が depth_measured を引き継ぐこと
- recheck_rect: 測れていない認識し直しは配置図の奥行きで直し、基準の辺を動かさず中心を奥へずらすこと
- Runner の配置図・盤前の認識し直しにこの扱いが入っていること（FakeSs の盤で指定できる）
"""
import math

import pytest

from fk_auto import core
from fk_auto.core import MapRect, Pose, SsRect
from fk_auto.fakes import SsResponse
from test_runner_world import TWO_PI, WPanel, World, body_rect, visible


def _map_rect(yaw, depth=0.86, measured=False, x=3.0, y=1.0):
    return MapRect(x=x, y=y, yaw_rad=yaw, width_m=1.0, depth_m=depth,
                   depth_measured=measured)


@pytest.mark.parametrize("yaw_deg", [0, 90, -170, 180, -45])
def test_recheck_rect_keeps_reference_edge_and_uses_survey_depth(yaw_deg):
    yaw = math.radians(yaw_deg)
    rect = _map_rect(yaw)
    survey = 1.4
    out = core.recheck_rect(rect, survey, True)
    assert out.depth_m == pytest.approx(survey)
    assert out.depth_measured is True
    # 基準の辺の位置は変わらない: ずらした中心からその辺までの距離が配置図の奥行きの半分
    nx, ny = math.cos(yaw), math.sin(yaw)
    edge_x, edge_y = rect.x + rect.depth_m / 2 * nx, rect.y + rect.depth_m / 2 * ny
    dist = (edge_x - out.x) * nx + (edge_y - out.y) * ny
    assert dist == pytest.approx(survey / 2)
    # 中心のずれは奥行きの差の半分だけ
    assert math.hypot(out.x - rect.x, out.y - rect.y) == pytest.approx((survey - rect.depth_m) / 2)


def test_recheck_rect_unchanged_when_recheck_measured():
    rect = _map_rect(0.3, measured=True)
    assert core.recheck_rect(rect, 1.4, True) is rect


def test_recheck_rect_unchanged_when_survey_not_measured():
    rect = _map_rect(0.3, measured=False)
    out = core.recheck_rect(rect, 1.4, False)
    assert out is rect and out.depth_measured is False


def test_to_map_and_make_panel_carry_depth_measured():
    ss = SsRect(x_m=2.0, y_m=0.0, yaw_rad=math.pi, width_m=1.0, depth_m=0.86,
                depth_measured=False)
    mp = core.to_map([ss], Pose(0.0, 0.0, 0.0))
    assert mp.panels[0].depth_measured is False
    edge = core.front_edge(mp.panels[0], (0.0, 0.0))
    panel = core.make_panel("P001", mp.panels[0], edge)
    assert panel.depth_measured is False


def _recheck_responder(w, depth_m, measured):
    orig = w.respond

    def responder(req):
        if req.azimuth_span_rad < TWO_PI - 1e-6:   # 盤前の認識し直し
            pose = w.pose()
            wp = w.panels[0]
            if visible(wp, pose, req.azimuth_from_rad, req.azimuth_span_rad,
                       req.max_distance_m):
                rect = body_rect(wp, pose, depth_m=depth_m, depth_measured=measured)
                return SsResponse("completed", (rect,), (req.azimuth_from_rad,),
                                  (req.azimuth_span_rad,))
            return SsResponse("no_lugs")
        return orig(req)

    return responder


def test_survey_layout_carries_depth_measured_from_fake_ss():
    """配置図では測れていない盤をそのまま使う（PanelView.depth_measured が false）。"""
    w = World([WPanel(3.0, 0.0, d=0.86)])
    orig = w.respond

    def responder(req):
        pose = w.pose()
        wp = w.panels[0]
        rect = body_rect(wp, pose, depth_m=0.86, depth_measured=False)
        return SsResponse("completed", (rect,), (req.azimuth_from_rad,),
                          (req.azimuth_span_rad,))

    w.ss.responder = responder
    w.to_select()
    panels = w.runner.get_layout().panels
    assert panels[0].depth_measured is False and panels[0].depth_m == pytest.approx(0.86)


def test_recheck_uses_survey_depth_and_shifts_center_toward_back():
    """配置図は測れていた (depth 1.0) ・認識し直しは測れていない (depth 0.86 default) →
    配置図の盤は奥行き 1.0 のまま、中心は §2.5 の式どおり奥へずらす。"""
    w = World([WPanel(3.0, 0.0, d=1.0)])
    w.ss.responder = _recheck_responder(w, depth_m=0.86, measured=False)
    w.to_select()
    assert w.runner.get_layout().panels[0].depth_measured is True
    w.start()
    w.until_phase("FINISH")
    panel = w.runner.get_layout().panels[0]
    assert panel.depth_m == pytest.approx(1.0)
    assert panel.depth_measured is True
    # ss の中心 (3,0) から −(1.0 − 0.86)/2 × 外向きの法線 (cos π, sin π) だけずらす
    expected_x = 3.0 - (1.0 - 0.86) / 2 * math.cos(math.pi)
    assert panel.x == pytest.approx(expected_x)
    assert panel.y == pytest.approx(0.0)


def test_recheck_measured_uses_recheck_depth_as_is():
    w = World([WPanel(3.0, 0.0, d=0.6)])
    w.ss.responder = _recheck_responder(w, depth_m=0.9, measured=True)
    w.to_select()
    w.start()
    w.until_phase("FINISH")
    panel = w.runner.get_layout().panels[0]
    assert panel.depth_m == pytest.approx(0.9) and panel.depth_measured is True


def test_survey_ss_rejected_appears_as_survey_reason():
    """ss が依頼を断った（reason ss_rejected）: SURVEY は今の作りのまま survey_reason に出る。"""
    w = World([WPanel(3.0, 0.0)])
    w.ss.responder = lambda req: SsResponse("ss_rejected")
    w.to_select()
    assert w.runner.get_layout().survey_reason == "ss_rejected"
    assert w.runner.get_layout().panels == []


def test_recheck_ss_rejected_skips_with_ss_failed():
    """RECHECK で ss_rejected は飛ばす（ss_failed）。"""
    w = World([WPanel(3.0, 0.0)])
    w.to_select()
    w.start()
    w.until_phase("RECHECK")
    w.ss.responder = lambda req: SsResponse("ss_rejected")
    w.until_phase("FINISH")
    assert w.results() == [("P001", "SKIPPED", "ss_failed")]
