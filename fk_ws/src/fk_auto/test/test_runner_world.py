"""Runner の試験の道具（偽の口を組んで、盤のある世界を作る）。試験そのものは test_runner_*.py。

世界の盤は地図の座標。盤の正面の向き yaw は盤から外向き（既定は π: 機体の側 −x に向く）。
ss の偽物は、依頼の時点の th の位置から見た機体基準の矩形を返す（方位角の範囲・距離で絞る）。
"""
import math
from dataclasses import dataclass
from typing import List, Optional

from fk_auto import core
from fk_auto.core import Params, SsRect
from fk_auto.fakes import FakeArm, FakeLift, FakeSs, FakeTh, Sim, SsResponse, ThModel, facing_ss_rect
from fk_auto.ports import NoMechanism
from fk_auto.runner import Runner

TWO_PI = 2 * math.pi


@dataclass
class WPanel:
    x: float
    y: float
    yaw: float = math.pi
    w: float = 1.0
    d: float = 0.6
    pos_std: float = 0.02
    yaw_std: float = 1.0
    unmeasured: bool = False   # 奥行きを測れていない盤（正面側では 0.86 m を返す。背面側から見れば測れる）


def body_rect(wp: WPanel, pose, **over) -> SsRect:
    """世界の盤を機体基準の矩形にする。ss は機体に向いていた辺を基準に返すので、
    背面側から見れば背面が基準の辺（向きが反対）になる奧行きが測れていない盤は、
    正面側では 0.86 m を返す（fakes.facing_ss_rect の置き方）。"""
    fx, fy = math.cos(wp.yaw), math.sin(wp.yaw)
    front_side = (pose.x - wp.x) * fx + (pose.y - wp.y) * fy > 0
    kw_over = dict(over)
    if "depth_m" in kw_over:   # 明示の上書き（既存の試験用）: 中心は世界のまま
        d = kw_over.pop("depth_m")
        meas = kw_over.pop("depth_measured", True)
        return facing_ss_rect(wp.x, wp.y, wp.yaw, wp.w, d, pose,
                              depth_measured=meas, pos_std_m=wp.pos_std,
                              yaw_std_deg=wp.yaw_std, **kw_over)
    kw_over.pop("depth_measured", None)
    if wp.unmeasured and front_side:
        return facing_ss_rect(wp.x, wp.y, wp.yaw, wp.w, wp.d, pose, assumed_depth_m=0.86,
                              depth_measured=False, pos_std_m=wp.pos_std,
                              yaw_std_deg=wp.yaw_std, **kw_over)
    return facing_ss_rect(wp.x, wp.y, wp.yaw, wp.w, wp.d, pose,
                          pos_std_m=wp.pos_std, yaw_std_deg=wp.yaw_std, **kw_over)


def visible(wp: WPanel, pose, frm: float, span: float, max_d: float) -> bool:
    rect = body_rect(wp, pose)
    if math.hypot(rect.x_m, rect.y_m) > (max_d or 5.0):
        return False
    rel = (math.atan2(rect.y_m, rect.x_m) - frm) % TWO_PI
    return span >= TWO_PI - 1e-9 or rel <= span


class World:
    """th・昇降機・ss・測定機構なしの口と Runner を組み、盤のある世界で回す。"""

    def __init__(self, panels: List[WPanel], mechanism: Optional[bool] = False,
                 lift_height_mm: float = 0.0, arm=None, params: Optional[Params] = None,
                 th_kw=None, lift_kw=None, ss_kw=None):
        self.panels = panels
        self.p = params or Params()
        self.th = FakeTh(ThModel(**(th_kw or {})))
        self.lift = FakeLift(height_mm=lift_height_mm, mechanism_attached=mechanism,
                             **(lift_kw or {}))
        self.ss = FakeSs(responder=self.respond, **(ss_kw or {}))
        self.arm = arm if arm is not None else NoMechanism(self.p)
        self.runner = Runner(self.th, self.lift, self.ss, self.arm, self.p)
        self.sim = Sim(self.runner, self.th, self.lift, self.ss,
                       self.arm if hasattr(self.arm, "advance") else None)
        self.hide: set = set()           # 見えなくする盤の番号（見つからない筋書き用）
        self.recheck_shift: List[float] = []   # 盤前の認識し直しで盤の x をずらす量（依頼ごとに 1 つ使う）

    # --- ss の中身 ---
    def pose(self):
        m = self.th.model
        return core.Pose(m.x, m.y, m.yaw)

    def respond(self, req):
        pose = self.pose()
        shift = 0.0
        if req.azimuth_span_rad < TWO_PI - 1e-6 and self.recheck_shift:   # 盤前の認識し直し
            shift = self.recheck_shift.pop(0)
        rects = tuple(body_rect(WPanel(wp.x + shift, wp.y, wp.yaw, wp.w, wp.d, wp.pos_std,
                                       wp.yaw_std, wp.unmeasured), pose)
                      for i, wp in enumerate(self.panels)
                      if i not in self.hide and visible(wp, pose, req.azimuth_from_rad,
                                                        req.azimuth_span_rad, req.max_distance_m))
        return SsResponse("completed" if rects else "no_lugs", rects, (req.azimuth_from_rad,),
                          (req.azimuth_span_rad,))

    # --- 進める ---
    def tick(self, n=1):
        self.sim.tick(n)

    def until(self, pred, max_ms=600_000):
        ok = self.sim.run_until(pred, max_ms)
        assert ok, f"時間内に満たされなかった: phase={self.phase} {self.runner.get_state()}"

    def until_phase(self, phase, max_ms=600_000):
        self.until(lambda: self.phase == phase, max_ms)

    @property
    def phase(self):
        return self.runner.phase

    @property
    def now(self):
        return self.sim.now

    def cmd(self, name, **kw):
        return self.runner.command(name, **kw)

    # --- 筋書きの入口 ---
    def begin(self):
        self.tick(2)                     # 生存確認が届き、状態が読める
        assert self.th.model.human_enter_auto()
        self.tick(2)
        r = self.cmd("auto.begin")
        assert r.accepted, r
        return r

    def to_select(self):
        self.begin()
        self.until_phase("SELECT")

    def ids(self) -> List[str]:
        return [p.panel_id for p in self.runner.get_layout().panels]

    def start(self, ids=None):
        r = self.cmd("auto.start", panel_ids=ids if ids is not None else self.ids())
        assert r.accepted, r
        return r

    def results(self):
        return [(r.panel_id, r.result, r.reason) for r in self.runner.get_results()]

    def results_side(self):
        return [(r.panel_id, r.result, r.reason, r.side) for r in self.runner.get_results()]

    def th_cmds(self):
        return self.th.commands()
