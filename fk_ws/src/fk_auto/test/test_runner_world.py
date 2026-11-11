"""Runner の試験の道具（偽の口を組んで、盤のある世界を作る）。試験そのものは test_runner_*.py。

世界の盤は地図の座標。盤の正面の向き yaw は盤から外向き（既定は π: 機体の側 −x に向く）。
ss の偽物は、依頼の時点の th の位置から見た機体基準の矩形を返す（方位角の範囲・距離で絞る）。
"""
import math
from dataclasses import dataclass
from typing import List, Optional

from fk_auto import core
from fk_auto.core import Params, SsRect
from fk_auto.fakes import FakeArm, FakeLift, FakeSs, FakeTh, Sim, SsResponse, ThModel
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


def body_rect(wp: WPanel, pose, **over) -> SsRect:
    dx, dy = wp.x - pose.x, wp.y - pose.y
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    kw = dict(x_m=c * dx + s * dy, y_m=-s * dx + c * dy, yaw_rad=core.wrap_angle(wp.yaw - pose.yaw),
              width_m=wp.w, depth_m=wp.d, pos_std_m=wp.pos_std, yaw_std_deg=wp.yaw_std, lugs=4,
              top_height_m=2.35)
    kw.update(over)
    return SsRect(**kw)


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
                                       wp.yaw_std), pose)
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

    def th_cmds(self):
        return self.th.commands()
