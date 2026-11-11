"""偽の口と th のモデル（ROS 2 なし。DetailedDesign-auto.md §6）。

どれも時刻を読まない。時刻は advance(now_ms) の引数だけで進む（ID はカウンタ。乱数・uuid・
time は使わない）。ThModel は段階 B で fake_th ノード（FakeThNode）も使うので、ROS にも時計にも
触れない。
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, Dict, List, Optional, Tuple

from fk_auto import core
from fk_auto.ports import (ArmPort, ArmResult, LiftPort, SsCleanup, SsPort, SsResult, ThPort,
                           ThReply, ThResult)

# ---------------------------------------------------------------------------
# ThModel（th の振る舞いの ROS なしのモデル）
# ---------------------------------------------------------------------------
# 正: th Spec-modes.md SM-3.1.2-120〜-142・th DetailedDesign-measure.md §3.1〜§3.3。
#
# 簡略化（段階 A。モデルが持たないもの）:
# - 状態は WAIT・NAV・ALIGN・BLOCKED・PAUSE だけ。ADJUST（旋回・並進）・AT_PANEL・ESTOP・CARRY は無い。
#   meas.turn・translate・set_limits・done は state_not_accepting で拒否する。
# - 自己位置を見失ったとき（-142）は ESTOP ではなく PAUSE に落とし、reentry_required を立てる
#   （目的地は CANCELLED + localization_lost で取り消す。解除して戻る先が PAUSE という点だけ合わせた）。
# - 手前の経路の確かめ・角の円の障害物は、試験用の旗（blocked・corner_obstacle）で作る。
#
# 指示の拒否の順は WP-AUTO-01 の fake_th.py と同じ（th §3.1 は順を決めていない）:
#   1. command_id が空 -> bad_argument
#   2. 最後に受けた生存確認の版が違う（受けたことがあるときだけ）-> version_mismatch
#   3. 生存確認が途絶えている（まだ 1 つも受けていないのを含む）-> link_lost
#   4. 状態に応じた拒否（このファイルの _reject_by_state）

KNOWN_UNSUPPORTED = ("meas.turn", "meas.translate", "meas.set_limits", "meas.done")
RUNNING_STATES = ("NAV", "ALIGN", "BLOCKED")


class ThModel:
    def __init__(self, interface_version: str = "1", measure_link_timeout_ms: float = 1000,
                 status_period_ms: float = 100, nav_time_ms: float = 1000,
                 command_latency_ms: float = 0, grid: Optional[core.Grid] = None):
        self.interface_version = interface_version
        self.measure_link_timeout_ms = measure_link_timeout_ms
        self.status_period_ms = status_period_ms
        self.nav_time_ms = nav_time_ms
        self.command_latency_ms = command_latency_ms
        self.grid = grid or core.Grid(0.1, -10.0, -10.0, 200, 200, [0] * (200 * 200))

        self.mode = "IDLE"
        self.state = "NONE"
        self.x = self.y = self.yaw = 0.0
        self.pose_valid = True
        self.map_instance_id = ""
        self.reentry_required = False
        self.blocked = False             # 試験用: 手前の経路が塞がっている
        self.corner_obstacle = False     # 試験用: 角の円の中に障害物がある（goal・start を拒否）
        self.pending: Optional[Tuple[str, str]] = None   # (command_id, command) 目的地を持っている間
        self.pending_command_id = ""
        self._goal: Optional[Tuple[float, float, float]] = None
        self._remaining_ms = 0.0
        self._prev_state: Optional[str] = None           # PAUSE に落ちる前の状態
        self._map_seq = 0
        self._now = 0.0
        self._last_adv: Optional[float] = None
        self._queue: Deque[Tuple[float, str, str, dict]] = deque()
        self._last_hb_version: Optional[str] = None
        self._last_valid_hb_ms: Optional[float] = None
        self.status_ms: Optional[float] = None
        self.status_frozen = False       # 試験用: 状態の配信を止める（th が落ちた想定）
        self.out_replies: List[ThReply] = []
        self.out_results: List[ThResult] = []

    # --- 生存確認 ---
    def on_heartbeat(self, version: str, now_ms: float) -> None:
        self._last_hb_version = version
        if version == self.interface_version:     # 版が違うものは受信に数えない（th §3.4）
            self._last_valid_hb_ms = now_ms

    def link_ok(self) -> bool:
        return (self._last_valid_hb_ms is not None
                and self._now - self._last_valid_hb_ms <= self.measure_link_timeout_ms)

    # --- 指示 ---
    def submit(self, command_id: str, command: str, now_ms: float, **args) -> None:
        """指示を受け取る。応答は次の advance（command_latency_ms 後）に out_replies へ出る。"""
        self._queue.append((now_ms + self.command_latency_ms, command_id, command, args))

    def advance(self, now_ms: float) -> None:
        dt = 0.0 if self._last_adv is None else max(0.0, now_ms - self._last_adv)
        self._last_adv = now_ms
        self._now = now_ms
        while self._queue and self._queue[0][0] <= now_ms:
            _, cid, cmd, args = self._queue.popleft()
            self._handle(cid, cmd, args)
        # 通信途絶: 走っている間は PAUSE（-133）。目的地は持ったまま。結果は出さない
        if self.mode == "AUTO_MEASURE" and self.state in RUNNING_STATES and not self.link_ok():
            self._pause()
        if self.mode == "AUTO_MEASURE" and self.state in ("NAV", "ALIGN"):
            self._remaining_ms -= dt
            if self._remaining_ms <= 0.0:
                self.x, self.y, self.yaw = self._goal
                self.state = "WAIT"
                self._finish_pending("ARRIVED")
            elif self._remaining_ms <= 0.2 * self.nav_time_ms:
                self.state = "ALIGN"
        if not self.status_frozen:
            self.status_ms = now_ms

    def status(self) -> core.ThStatus:
        return core.ThStatus(
            read_ms=self.status_ms, interface_version=self.interface_version, mode=self.mode,
            state=self.state, pose_valid=self.pose_valid, map_instance_id=self.map_instance_id,
            x=self.x, y=self.y, yaw=self.yaw, reentry_required=self.reentry_required)

    def drain_replies(self) -> List[ThReply]:
        out, self.out_replies = self.out_replies, []
        return out

    def drain_results(self) -> List[ThResult]:
        out, self.out_results = self.out_results, []
        return out

    # --- 試験用の操作（人・環境） ---
    def human_enter_auto(self) -> bool:
        """人が「自律測定」→「許可する」（-120）。測定機能と通信できなければ拒否（False）。
        入るたびに地図を作り直す（世代が変わる）。"""
        if not self.link_ok():
            return False
        self.mode = "AUTO_MEASURE"
        self.state = "WAIT"
        self._map_seq += 1
        self.map_instance_id = f"map-{self._map_seq}"
        self.reentry_required = False
        self.pending = None
        self.pending_command_id = ""
        self._prev_state = None
        return True

    def human_exit(self) -> None:
        """人が自律測定を抜ける（SM-3.1.1-18）。目的地は CANCELLED + mode_exit。"""
        self._finish_pending("CANCELLED", "mode_exit")
        self.mode = "IDLE"
        self.state = "NONE"
        self._prev_state = None

    def human_jog(self) -> None:
        """人のジョグ（PAUSE へ。目的地は持ったまま。結果は出さない）。"""
        if self.mode == "AUTO_MEASURE" and self.state in RUNNING_STATES + ("WAIT",):
            self._pause()

    def set_blocked(self, blocked: bool) -> None:
        """手前の経路が塞がる／空く（-125・-126）。"""
        self.blocked = blocked
        if self.mode == "AUTO_MEASURE":
            if blocked and self.state == "NAV":
                self.state = "BLOCKED"
            elif not blocked and self.state == "BLOCKED":
                self.state = "NAV"

    def lose_localization(self) -> None:
        """自己位置を見失う（-142）。目的地は CANCELLED + localization_lost。"""
        self._finish_pending("CANCELLED", "localization_lost")
        self.reentry_required = True
        self.state = "PAUSE"
        self._prev_state = None

    # --- 内部 ---
    def _pause(self) -> None:
        self._prev_state = self.state
        self.state = "PAUSE"

    def _finish_pending(self, result: str, reason: str = "") -> None:
        if self.pending is not None:
            cid, cmd = self.pending
            self.out_results.append(ThResult(cid, cmd, result, reason))
        self.pending = None
        self.pending_command_id = ""
        self._goal = None

    def _reply(self, cid: str, accepted: bool, reason: str = "") -> None:
        self.out_replies.append(ThReply(cid, accepted, reason, self.mode, self.state))

    def _reject_reason(self, cid: str, cmd: str, args: dict) -> Optional[str]:
        if cid == "":
            return "bad_argument"
        if self._last_hb_version is not None and self._last_hb_version != self.interface_version:
            return "version_mismatch"
        if not self.link_ok():
            return "link_lost"
        return self._reject_by_state(cmd, args)

    def _reject_by_state(self, cmd: str, args: dict) -> Optional[str]:
        if cmd in KNOWN_UNSUPPORTED or cmd.startswith("ui."):
            return "state_not_accepting"           # ui.* は拒否する（th §3.1）
        if cmd not in ("meas.goal", "meas.reroute", "meas.start", "meas.stop", "meas.abort",
                       "meas.finish"):
            return "bad_argument"
        if self.mode != "AUTO_MEASURE":
            return "state_not_accepting"
        st = self.state
        if cmd == "meas.goal":
            if self.reentry_required:
                return "localization_lost"
            if st != "WAIT":
                return "state_not_accepting"
            if args.get("map_instance_id", "") != self.map_instance_id:
                return "map_mismatch"
            if self.corner_obstacle:
                return "corner_obstacle"
        elif cmd == "meas.reroute":
            if st not in ("NAV", "BLOCKED"):
                return "state_not_accepting"
        elif cmd == "meas.start":
            if self.reentry_required:
                return "localization_lost"
            if st != "PAUSE":
                return "state_not_accepting"
            if self.corner_obstacle and self._prev_state in ("NAV", "ALIGN"):
                return "corner_obstacle"
        elif cmd == "meas.stop":
            if st not in RUNNING_STATES:
                return "state_not_accepting"
        elif cmd == "meas.abort":
            if st not in RUNNING_STATES + ("PAUSE",):
                return "state_not_accepting"
        return None   # meas.finish は AUTO_MEASURE の任意の状態

    def _handle(self, cid: str, cmd: str, args: dict) -> None:
        reason = self._reject_reason(cid, cmd, args)
        if reason:
            self._reply(cid, False, reason)
            return
        self._reply(cid, True)
        if cmd == "meas.goal":
            self.pending = (cid, cmd)
            self.pending_command_id = cid
            self._goal = (args["x"], args["y"], args["yaw"])
            self._remaining_ms = self.nav_time_ms
            self.state = "BLOCKED" if self.blocked else "NAV"
            return
        if cmd == "meas.reroute":
            self.state = "BLOCKED" if self.blocked else "NAV"   # 目的地はもとの指示のまま
        elif cmd == "meas.start":
            self.state = self._prev_state or "WAIT"
            self._prev_state = None
        elif cmd == "meas.stop":
            self._pause()
        elif cmd == "meas.abort":
            self._finish_pending("CANCELLED", "aborted")
            self.state = "WAIT"
            self._prev_state = None
        elif cmd == "meas.finish":
            self._finish_pending("CANCELLED", "finished")
            self.mode = "IDLE"
            self.state = "NONE"
            self._prev_state = None
        self.out_results.append(ThResult(cid, cmd, "DONE", ""))   # すぐ DONE（§3.1 の表）


# ---------------------------------------------------------------------------
# FakeTh（ThModel を使う th の口。同じプロセスの中）
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SentCommand:
    command_id: str
    command: str
    map_instance_id: str
    x: float
    y: float
    yaw: float
    at_ms: float


class FakeTh(ThPort):
    def __init__(self, model: Optional[ThModel] = None, heartbeat_version: Optional[str] = None):
        self.model = model or ThModel()
        self.sent: List[SentCommand] = []            # Runner が出した指示をすべて記録する
        self.link_alive = True                       # 試験用: False にすると生存確認が止まる
        self.heartbeat_version = heartbeat_version or self.model.interface_version
        self.unreadable = False                      # 試験用: True なら status() は None
        self._now = 0.0
        self._replies: List[ThReply] = []
        self._results: List[ThResult] = []

    def advance(self, now_ms: float) -> None:
        self._now = now_ms
        if self.link_alive:       # Runner は生存確認を出さない（ノードの仕事）ので、ここで模す
            self.model.on_heartbeat(self.heartbeat_version, now_ms)
        self.model.advance(now_ms)
        self._replies.extend(self.model.drain_replies())
        self._results.extend(self.model.drain_results())

    # --- ThPort ---
    def status(self) -> Optional[core.ThStatus]:
        if self.unreadable or self.model.status_ms is None:
            return None
        return self.model.status()

    def grid(self) -> Optional[core.Grid]:
        return self.model.grid

    def send(self, command_id, command, map_instance_id="", x=0.0, y=0.0, yaw=0.0) -> None:
        self.sent.append(SentCommand(command_id, command, map_instance_id, x, y, yaw, self._now))
        self.model.submit(command_id, command, self._now, map_instance_id=map_instance_id,
                          x=x, y=y, yaw=yaw)

    def poll_replies(self) -> List[ThReply]:
        out, self._replies = self._replies, []
        return out

    def poll_results(self) -> List[ThResult]:
        out, self._results = self._results, []
        return out

    # --- 試験用 ---
    def commands(self) -> List[str]:
        return [c.command for c in self.sent]


# ---------------------------------------------------------------------------
# FakeLift
# ---------------------------------------------------------------------------

class FakeLift(LiftPort):
    """昇降機。下降の命令を受けるたびに lower_step_mm だけ下がり、下端で止まる（原点が取れる）。"""

    def __init__(self, height_mm: float = 0.0, lower_step_mm: float = 100.0,
                 mechanism_attached: Optional[bool] = True, arm_stowed: Optional[bool] = True,
                 arm_link_ok: Optional[bool] = True, origin_ok: Optional[bool] = True):
        self.height_mm = height_mm
        self.lower_step_mm = lower_step_mm
        self.mechanism_attached = mechanism_attached
        self.arm_stowed = arm_stowed
        self.arm_link_ok = arm_link_ok
        self.origin_ok = origin_ok
        self.human_operating = False        # いま操作している元が人
        self.connected = True               # False なら status() は None（口が無いのと同じ）
        self.height_readable = True         # False なら高さの読めた時刻が止まる
        self.arm_readable = True            # False なら腕の読めた時刻が止まる
        self.commands: List[Tuple[str, float]] = []   # 受けた命令すべて（("lower"|"stop", 時刻）)
        self._now = 0.0
        self._height_ms = 0.0
        self._arm_ms = 0.0

    def advance(self, now_ms: float) -> None:
        self._now = now_ms
        if self.height_readable:
            self._height_ms = now_ms
        if self.arm_readable:
            self._arm_ms = now_ms

    def status(self) -> Optional[core.LiftStatus]:
        if not self.connected:
            return None
        return core.LiftStatus(
            height_read_ms=self._height_ms, at_bottom=self.height_mm <= 1e-9,
            height_above_bottom_mm=self.height_mm, arm_read_ms=self._arm_ms,
            arm_stowed=self.arm_stowed, mechanism_attached=self.mechanism_attached,
            arm_link_ok=self.arm_link_ok, origin_ok=self.origin_ok,
            operator_is_human=self.human_operating)

    def lower(self) -> None:
        self.commands.append(("lower", self._now))
        if not self.human_operating:   # 人が操作している間は動かない（命令は記録する）
            self.height_mm = max(0.0, self.height_mm - self.lower_step_mm)
            if self.height_mm <= 1e-9:
                self.origin_ok = True

    def stop(self) -> None:
        self.commands.append(("stop", self._now))

    def lowers(self) -> int:
        return sum(1 for c, _ in self.commands if c == "lower")


# ---------------------------------------------------------------------------
# FakeSs
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SsRequestRecord:
    request_id: str
    azimuth_from_rad: float
    azimuth_span_rad: float
    max_distance_m: float
    time_limit_s: float
    at_ms: float


@dataclass(frozen=True)
class SsResponse:
    """依頼ごとに返す中身（依頼した時点の機体基準）。"""
    reason: str = "completed"
    panels: Tuple[core.SsRect, ...] = ()
    seen_from_rad: Tuple[float, ...] = ()
    seen_span_rad: Tuple[float, ...] = ()


class FakeSs(SsPort):
    """ss の偽物。依頼 → RUNNING → result_delay_ms 後に結果 → CLEANING → cleanup_delay_ms 後に
    後片付けの完了 → IDLE（ss DetailedDesign-locate.md §0・§2・§3・§4）。

    返す中身: responses（依頼の順に 1 つずつ使う）→ なくなったら responder(依頼)→ なければ default。
    """

    def __init__(self, result_delay_ms: float = 200, cleanup_delay_ms: float = 200,
                 default: Optional[SsResponse] = None,
                 responder: Optional[Callable[[SsRequestRecord], SsResponse]] = None):
        self.result_delay_ms = result_delay_ms
        self.cleanup_delay_ms = cleanup_delay_ms
        self.default = default or SsResponse(reason="no_lugs")
        self.responder = responder
        self.responses: Deque[SsResponse] = deque()
        self.never_result = False        # True なら結果が来ない（取り消されるまで）
        self.never_cleanup = False       # True なら後片付けの完了が来ない
        self.reject_requests = False     # True なら依頼を拒否する
        self.frozen = False              # True なら状態が更新されない（通信できない）
        self.connected = True            # False なら status() は None
        self.requests: List[SsRequestRecord] = []
        self.cancels: List[Tuple[str, float]] = []
        self._state = "IDLE"
        self._status_ms = 0.0
        self._now = 0.0
        self._seq = 0
        self._active: Optional[dict] = None
        self._results: List[SsResult] = []
        self._cleanups: List[SsCleanup] = []
        self.external_busy = False       # 本システム以外の依頼が動いている

    def advance(self, now_ms: float) -> None:
        self._now = now_ms
        a = self._active
        if a is not None and a["stage"] == "running":
            if a["cancel"] or (not self.never_result and now_ms >= a["result_at"]):
                resp: SsResponse = SsResponse(reason="canceled") if a["cancel"] else a["resp"]
                self._results.append(SsResult(a["id"], resp.reason, resp.panels,
                                              resp.seen_from_rad, resp.seen_span_rad))
                a["stage"] = "cleaning"
                a["reason"] = resp.reason
                a["cleanup_at"] = now_ms + self.cleanup_delay_ms
                self._state = "CLEANING"
        if a is not None and a["stage"] == "cleaning":
            if not self.never_cleanup and now_ms >= a["cleanup_at"]:
                self._cleanups.append(SsCleanup(a["id"], a["reason"],
                                                lift_restored=a["reason"] != "human_override"))
                self._active = None
                self._state = "IDLE"
        if not self.frozen:
            self._status_ms = now_ms

    def status(self) -> Optional[core.SsStatus]:
        if not self.connected:
            return None
        state = "RUNNING" if (self.external_busy and self._state == "IDLE") else self._state
        return core.SsStatus(read_ms=self._status_ms, state=state)

    def request(self, azimuth_from_rad, azimuth_span_rad, max_distance_m, time_limit_s):
        if self.reject_requests or self._state != "IDLE" or self.external_busy:
            return None
        self._seq += 1
        rid = f"s{self._seq}"
        rec = SsRequestRecord(rid, azimuth_from_rad, azimuth_span_rad, max_distance_m,
                              time_limit_s, self._now)
        self.requests.append(rec)
        if self.responses:
            resp = self.responses.popleft()
        elif self.responder is not None:
            resp = self.responder(rec)
        else:
            resp = self.default
        self._active = {"id": rid, "stage": "running", "cancel": False, "resp": resp,
                        "result_at": self._now + self.result_delay_ms}
        self._state = "RUNNING"
        return rid

    def cancel(self, request_id: str) -> None:
        self.cancels.append((request_id, self._now))
        if self._active is not None and self._active["id"] == request_id:
            self._active["cancel"] = True

    def poll_results(self) -> List[SsResult]:
        out, self._results = self._results, []
        return out

    def poll_cleanups(self) -> List[SsCleanup]:
        out, self._cleanups = self._cleanups, []
        return out


# ---------------------------------------------------------------------------
# FakeArm（AT_PANEL の行を試験するための小さな測定機構。Python の中だけの名前）
# ---------------------------------------------------------------------------

class FakeArm(ArmPort):
    """作業を受け、work_ms 後に outcome を返す。中止を渡されると cancel_ms 後に CANCELLED。"""

    def __init__(self, params: Optional[core.Params] = None, work_ms: float = 500,
                 cancel_ms: float = 300, outcome: Tuple[str, str] = ("DONE", "")):
        self._p = params or core.Params()
        self.work_ms = work_ms
        self.cancel_ms = cancel_ms
        self.outcome = outcome
        self.stowed: Optional[bool] = True
        self.starts: List[str] = []
        self.cancels = 0
        self.pauses: List[bool] = []
        self._now = 0.0
        self._work: Optional[dict] = None
        self._results: List[ArmResult] = []

    def advance(self, now_ms: float) -> None:
        self._now = now_ms
        w = self._work
        if w is not None and now_ms >= w["due"]:
            self._results.append(w["result"])
            self._work = None

    def stand_pose(self, panel: core.Panel) -> core.Pose:
        return core.stand_pose(panel, self._p.stand_distance_min_m, self._p.stand_margin_m)

    def arm_stowed(self) -> Optional[bool]:
        return self.stowed

    def start_work(self, panel_id: str) -> bool:
        self.starts.append(panel_id)
        self._work = {"due": self._now + self.work_ms,
                      "result": ArmResult(panel_id, self.outcome[0], self.outcome[1])}
        return True

    def cancel_work(self) -> None:
        self.cancels += 1
        if self._work is not None:
            pid = self._work["result"].panel_id
            self._work = {"due": self._now + self.cancel_ms, "result": ArmResult(pid, "CANCELLED")}

    def set_paused(self, paused: bool) -> None:
        self.pauses.append(paused)

    def poll_results(self) -> List[ArmResult]:
        out, self._results = self._results, []
        return out


# ---------------------------------------------------------------------------
# Sim（時刻を進めて口を動かし、Runner を回す。試験・段階 B の結合で使う）
# ---------------------------------------------------------------------------

class Sim:
    """口の偽物と Runner をまとめて、時刻を引数だけで進める。None の口は動かさない。"""

    def __init__(self, runner, th=None, lift=None, ss=None, arm=None, dt_ms: float = 100):
        self.runner = runner
        self.th, self.lift, self.ss, self.arm = th, lift, ss, arm
        self.dt_ms = dt_ms
        self.now = 0.0

    def tick(self, n: int = 1) -> None:
        for _ in range(n):
            self.now += self.dt_ms
            for p in (self.th, self.lift, self.ss, self.arm):
                if p is not None and hasattr(p, "advance"):
                    p.advance(self.now)
            self.runner.step(self.now)

    def run_until(self, pred: Callable[[], bool], max_ms: float = 600_000) -> bool:
        limit = self.now + max_ms
        while self.now < limit:
            if pred():
                return True
            self.tick()
        return pred()
