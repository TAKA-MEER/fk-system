"""自律測定の移動の状態機械 Runner（DetailedDesign-auto.md §2。ROS なし）。

口（ports.py）から集めた「いまの様子」（core.Snapshot）を core の純関数に渡し、返った判断を口へ
流すだけ。判定は core にある。

安全の要（DetailedDesign-auto.md §0・§4。ブリーフ §4.1）:
- th への指示（meas.*）・昇降機への命令（下げる）・ss への依頼は、**_emit という 1 つのメソッドだけ**が
  口へ出す。_emit は送る直前に毎回 core.gate を通し、通らなければ送らず理由を gate_reason に出す。
  ほかの場所から口の送るメソッド（send・lower・request ほか）を呼んではいけない。
  gate の表に無い指示（meas.turn など）は gate が not_listed で止める。
- 昇降機を下げ続ける間も、step 1 回につき下降の命令を 1 つだけ出し、毎回 gate を通す。
- core.drive_watch を step のたびに見る（走っている間でなくても呼び、core が th の状態で絞る）。
- 時刻は step(now_ms) の引数だけ。time.monotonic() などは読まない。
- 口が None なら読めない扱い（何も送らない）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from fk_auto import core
from fk_auto.core import HaltEvent, Pose, SkipEvent
from fk_auto.ports import ArmPort, LiftPort, SsCleanup, SsPort, SsResult, ThPort, ThReply, ThResult

# ---------------------------------------------------------------------------
# 外へ見せるもの（段階 B でメッセージ AutoState・Layout・Panel・PanelResults にする。names.md §4.1）
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CommandReply:
    accepted: bool
    reject_reason: str = ""


@dataclass
class AutoStateView:
    """names.md §4.1 AutoState の欄（header は now_ms）。"""
    now_ms: float = 0.0
    phase: str = "IDLE"
    panel_id: str = ""
    queue: List[str] = field(default_factory=list)
    halt_reason: str = ""
    gate_reason: str = ""
    lift_at_bottom: bool = False
    lift_readable: bool = False
    arm_stowed: bool = False
    mechanism_attached: bool = False
    reentry_required: bool = False
    th_mode: str = ""
    th_state: str = ""
    th_reject_reason: str = ""


@dataclass(frozen=True)
class PanelView:
    """names.md §4.1 Panel の欄。"""
    panel_id: str
    x: float
    y: float
    front_yaw: float
    width_m: float
    depth_m: float
    top_height_m: float
    pos_std_m: float
    yaw_std_deg: float
    lugs: int
    confident: bool
    ss_front_known: bool
    ss_front_yaw: float
    front_doubt: bool
    row_id: str = ""


@dataclass
class LayoutView:
    """names.md §4.1 Layout の欄（header は now_ms）。"""
    now_ms: float = 0.0
    map_instance_id: str = ""
    panels: List[PanelView] = field(default_factory=list)
    unseen_from_rad: List[float] = field(default_factory=list)
    unseen_to_rad: List[float] = field(default_factory=list)


@dataclass(frozen=True)
class PanelResultView:
    """names.md §4.1 PanelResult の欄。result は DONE / SKIPPED。"""
    panel_id: str
    result: str
    reason: str = ""


# ---------------------------------------------------------------------------
# 内部
# ---------------------------------------------------------------------------

_STOP_SIDE_TH = ("meas.stop", "meas.abort", "meas.finish")
# gate の表の外（止める側・知らせる側）。HALTED でも通す
_INTERNAL_STOP = ("ss.cancel", "arm.cancel")
_INTERNAL = _INTERNAL_STOP + ("arm.start", "arm.pause")
_RUNNING = ("NAV", "ALIGN", "BLOCKED")
_LOWERING_PHASES = ("PREPARE", "LOWER", "FINISH")
_ACTIVE_PHASES = ("PREPARE", "SURVEY", "SELECT", "LOWER", "GOAL", "RECHECK", "AT_PANEL",
                  "FINISH")
_SS_UNUSABLE = (SkipEvent.SS_TIMEOUT, SkipEvent.SS_RESULT_UNUSABLE)


@dataclass
class _SsJob:
    kind: str                           # "survey" | "recheck"
    stage: str = "send"                 # send / await_result / await_cleanup
    request_id: Optional[str] = None
    sent_ms: float = 0.0
    cleanup_started_ms: float = 0.0
    th_req: Optional[core.ThStatus] = None
    waited: bool = True                 # 依頼のあいだ状態がずっと WAIT（ss_result_valid へ渡す）
    attempt: int = 0
    timed_out: bool = False
    result: Optional[SsResult] = None
    cleanup: Optional[SsCleanup] = None


@dataclass
class _SsDone:
    result: Optional[SsResult] = None
    th_req: Optional[core.ThStatus] = None
    failure: Optional[SkipEvent] = None


@dataclass
class _Goal:
    stage: str = "plan"                 # plan / send / await_reply / running
    stand: Optional[Pose] = None
    goal_id: Optional[str] = None
    arrived: bool = False
    blocked_since: Optional[float] = None
    reroutes: int = 0


@dataclass
class _Abort:
    kind: str                           # "human" | "no_path"
    stage: str = "th"                   # th / ss / arm
    th_id: Optional[str] = None
    th_done: bool = False
    ss_cancel_sent: bool = False
    ss_cancel_ms: float = 0.0
    arm_cancel_sent: bool = False


class Runner:
    def __init__(self, th: Optional[ThPort], lift: Optional[LiftPort], ss: Optional[SsPort],
                 arm: Optional[ArmPort], params: Optional[core.Params] = None):
        self.p = params or core.Params()
        self._th, self._lift, self._ss, self._arm = th, lift, ss, arm
        self._now = 0.0
        self._snap = core.Snapshot(now_ms=0.0)
        self._cmd_seq = 0
        self._panel_seq = 0                         # panel_id の通し番号。Runner が生きている間は戻さない
        self._sent: Dict[str, str] = {}             # command_id → 目的（goal・reroute・start ほか）

        self._phase = "IDLE"
        self._cur = ""
        self._queue: List[str] = []
        self._halt_reason = ""
        self._gate_reason = ""
        self._th_reject_reason = ""
        self.last_survey_failure = ""               # 空の配置図にした理由（画面に出す欄は未定）

        self._results: List[PanelResultView] = []   # 測り終えた盤の結果。入り直しでも消さない
        self._layout_map_id = ""
        self._layout_panels: List[PanelView] = []
        self._core_panels: Dict[str, core.Panel] = {}
        self._rows: List[List[str]] = []
        self._unseen: List[Tuple[float, float]] = []
        self._regoal: Dict[str, int] = {}

        self._reentry_seen = False
        self._stop_id: Optional[str] = None
        self._finishing: Optional[str] = None
        self._run_req = False
        self._paused_from = ""
        self._paused_cause = ""
        self._abort: Optional[_Abort] = None
        self._enter("IDLE")
        self._arm_results: List = []

    # ------------------------------------------------------------------ 公開

    @property
    def phase(self) -> str:
        return self._phase

    def get_state(self) -> AutoStateView:
        s, th, lift = self._snap, self._snap.th, self._snap.lift
        return AutoStateView(
            now_ms=self._now, phase=self._phase, panel_id=self._cur, queue=list(self._queue),
            halt_reason=self._halt_reason, gate_reason=self._gate_reason,
            lift_at_bottom=bool(lift and lift.at_bottom), lift_readable=self._height_readable(s),
            arm_stowed=bool(lift and lift.arm_stowed), mechanism_attached=bool(
                lift and lift.mechanism_attached),
            reentry_required=bool(th and th.reentry_required),
            th_mode=(th.mode or "") if th else "", th_state=(th.state or "") if th else "",
            th_reject_reason=self._th_reject_reason)

    def get_layout(self) -> LayoutView:
        return LayoutView(
            now_ms=self._now, map_instance_id=self._layout_map_id,
            panels=list(self._layout_panels),
            unseen_from_rad=[a for a, _ in self._unseen], unseen_to_rad=[b for _, b in self._unseen])

    def get_results(self) -> List[PanelResultView]:
        return list(self._results)

    def step(self, now_ms: float) -> None:
        """状態機械を 1 回回す。時刻はこの引数だけ。"""
        self._now = now_ms
        self._gate_reason = ""
        snap = self._read(now_ms)
        self._snap = snap
        self._drain(snap)
        self._watch(snap)
        if self._interrupts(snap):
            return
        if self._abort is not None:
            self._abort_step(snap)
            return
        getattr(self, "_do_" + self._phase.lower())(snap)

    # ------------------------------------------------------------------ 画面からの指示

    def command(self, name: str, panel_ids: Sequence[str] = (),
                arm_confirmed: bool = False) -> CommandReply:
        """names.md §4.2 の auto.*。受けるか、拒否と理由（reject_reason）を返す。"""
        fn = {"auto.begin": self._c_begin, "auto.start": self._c_start, "auto.stop": self._c_stop,
              "auto.run": self._c_run, "auto.abort": self._c_abort, "auto.rescan": self._c_rescan,
              "auto.finish": self._c_finish}.get(name)
        if fn is None:
            return CommandReply(False, "bad_argument")
        return fn(panel_ids=panel_ids, arm_confirmed=arm_confirmed)

    def _no(self, reason: str) -> CommandReply:
        return CommandReply(False, reason)

    def _th_fresh(self, snap: core.Snapshot) -> Optional[core.ThStatus]:
        th = snap.th
        if th is not None and core._fresh(th.read_ms, snap.now_ms, self.p.th_status_stale_ms):
            return th
        return None

    def _c_begin(self, **_):
        th = self._th_fresh(self._snap)
        if self._phase != "IDLE" or th is None or th.mode != "AUTO_MEASURE":
            return self._no("phase_not_accepting")
        self._th_reject_reason = ""
        self._reentry_seen = False
        self._enter("PREPARE")
        return CommandReply(True)

    def _c_start(self, panel_ids=(), **_):
        if self._phase not in ("SELECT", "FINISH"):
            return self._no("phase_not_accepting")
        ids = list(dict.fromkeys(panel_ids))
        if not ids or any(i not in self._core_panels for i in ids):
            return self._no("bad_argument")
        th = self._th_fresh(self._snap)
        if th is None:
            return self._no("state_unreadable")
        if th.interface_version != self.p.th_interface_version:
            return self._no("version_mismatch")
        if th.reentry_required:
            return self._no("reentry_required")
        order = core.order_panels(ids, self._rows, list(self._core_panels.values()),
                                  (th.x, th.y))
        self._th_reject_reason = ""
        self._regoal = {}
        self._cur, self._queue = order[0], order[1:]
        self._enter("LOWER")
        return CommandReply(True)

    def _c_stop(self, **_):
        th = self._th_fresh(self._snap)
        if th is None or th.state not in _RUNNING:
            return self._no("phase_not_accepting")
        if self._emit("meas.stop") is None:
            return self._no("state_unreadable")
        return CommandReply(True)

    def _c_run(self, **_):
        if self._phase != "PAUSED" or self._run_req:
            return self._no("phase_not_accepting")
        snap = self._snap
        th = self._th_fresh(snap)
        if self._paused_cause == "th":
            if th is not None and th.state != "PAUSE":
                return self._no("phase_not_accepting")
            d = core.can_run(snap, self.p)
            if d.kind == "reject":
                return self._no(d.reason)
        else:   # 人が昇降機を操作した割り込み。操作が続いている間は受けない
            if snap.lift is None or snap.lift.operator_is_human is not False:
                return self._no("phase_not_accepting")
        self._th_reject_reason = ""
        self._run_req = True
        self._lower_started = None
        return CommandReply(True)

    def _c_abort(self, **_):
        if self._phase not in ("GOAL", "RECHECK", "AT_PANEL", "PAUSED"):
            return self._no("phase_not_accepting")
        if self._abort is None:
            self._abort = _Abort("human")
        self._run_req = False
        return CommandReply(True)

    def _c_rescan(self, **_):
        if self._phase not in ("SELECT", "FINISH"):
            return self._no("phase_not_accepting")
        g = core.gate(core.CMD_SS_REQUEST, self._snap, self.p)
        if not g.allowed:
            return self._no(g.reason)
        self._cur, self._queue = "", []
        self._enter("SURVEY")
        return CommandReply(True)

    def _c_finish(self, arm_confirmed=False, **_):
        snap = self._snap
        absent = core._mechanism_absent(snap)
        stowed = core._arm_readable(snap, self.p) and bool(snap.lift.arm_stowed)
        if not absent and not stowed and not arm_confirmed:
            return self._no("arm_not_confirmed")
        cid = self._emit("meas.finish")
        if cid is None:
            return self._no("state_unreadable")
        self._finishing = cid
        return CommandReply(True)

    # ------------------------------------------------------------------ 送り口

    def _emit(self, command: str, snap: Optional[core.Snapshot] = None, **kw):
        """th・昇降機・ss・測定機構へ出す、**唯一の送り口**。通ったら口へ出して、th なら command_id・
        ss なら依頼の ID・それ以外は "ok" を返す。通らない・口が無いときは None。

        - gate の表にある指示（meas.*・昇降機を下げる・ss への依頼）は、毎回 core.gate を通す。
          通らなければ送らず、理由を gate_reason に出す。表に無い指示は gate が not_listed で止める。
        - ss の取り消し・測定機構への指示は gate の表の外（止める側・知らせる側）。
        - 全体を止めている間（HALTED）は、止める側のほかは出さない。
        """
        snap = snap or self._snap
        if self._phase == "HALTED" and command not in _STOP_SIDE_TH + _INTERNAL_STOP:
            return None
        if command not in _INTERNAL:
            g = core.gate(command, snap, self.p)
            if not g.allowed:
                self._gate_reason = g.reason
                return None
            if command == core.CMD_SS_REQUEST:
                r = core._ss_reason(snap, self.p)    # ss が IDLE で読めること（口が無ければ読めない）
                if r:
                    self._gate_reason = r
                    return None
        if command.startswith("meas."):
            if self._th is None:
                return None
            self._cmd_seq += 1
            cid = f"c{self._cmd_seq}"
            self._sent[cid] = command[len("meas."):]
            self._th.send(cid, command, kw.get("map_instance_id", ""), kw.get("x", 0.0),
                          kw.get("y", 0.0), kw.get("yaw", 0.0))
            return cid
        if command == core.CMD_LIFT_LOWER:
            if self._lift is None:
                return None
            self._lift.lower()
            return "ok"
        if command == core.CMD_SS_REQUEST:
            if self._ss is None:
                return None
            rid = self._ss.request(kw["azimuth_from_rad"], kw["azimuth_span_rad"],
                                   self.p.ss_max_distance_m, self.p.ss_result_timeout_s)
            if rid is None:
                self._gate_reason = "ss_busy"       # ss が受けなかった
            return rid
        if command == "ss.cancel":
            if self._ss is None:
                return None
            self._ss.cancel(kw["request_id"])
            return "ok"
        if self._arm is None:
            return None
        if command == "arm.start":
            return "ok" if self._arm.start_work(kw["panel_id"]) else None
        if command == "arm.cancel":
            self._arm.cancel_work()
            return "ok"
        if command == "arm.pause":
            self._arm.set_paused(kw["paused"])
            return "ok"
        return None

    # ------------------------------------------------------------------ 読む

    def _read(self, now_ms: float) -> core.Snapshot:
        return core.Snapshot(
            now_ms=now_ms, th=self._th.status() if self._th else None,
            lift=self._lift.status() if self._lift else None,
            ss=self._ss.status() if self._ss else None)

    def _height_readable(self, s: core.Snapshot) -> bool:
        return core._lift_height_readable(s, self.p)

    def _drain(self, snap: core.Snapshot) -> None:
        if self._th is not None:
            for r in self._th.poll_replies():
                self._on_reply(r)
            for r in self._th.poll_results():
                self._on_result(r)
        if self._ss is not None:
            job = self._job
            for r in self._ss.poll_results():
                if job is not None and job.request_id == r.request_id:
                    job.result = r
            for c in self._ss.poll_cleanups():
                if job is not None and job.request_id == c.request_id:
                    job.cleanup = c
        if self._arm is not None:
            self._arm_results.extend(self._arm.poll_results())

    # ------------------------------------------------------------------ th の返事と結果

    def _on_reply(self, r: ThReply) -> None:
        purpose = self._sent.get(r.command_id)
        if purpose is None:
            return
        if purpose == "stop":
            self._stop_id = None
        g = self._goal
        if r.accepted:
            if purpose == "goal" and self._phase == "GOAL" and g.goal_id == r.command_id \
                    and g.stage == "await_reply":
                g.stage = "running"
            elif purpose == "finish" and self._finishing == r.command_id:
                self._to_idle()
            return
        self._th_reject_reason = r.reject_reason
        if purpose == "finish" and self._finishing == r.command_id:
            self._finishing = None
        elif purpose == "abort" and self._abort is not None and self._abort.th_id == r.command_id:
            self._abort.th_done = True        # th の状態で止まっていると分かれば待たずに進む
        elif purpose == "goal" and self._phase == "GOAL" and g.goal_id == r.command_id:
            if r.reject_reason in ("localization_lost", "map_mismatch"):
                self._to_reentry()            # 地図か配置図が古い
            else:
                self._back_to_select()        # 動く指示を拒否: 出し直さず選び直しへ（th は WAIT のまま）
        elif purpose == "start":
            self._run_req = False             # PAUSED のまま（もう一度「走行」）
            if r.reject_reason in ("localization_lost", "map_mismatch"):
                self._to_reentry()
        # reroute: 再計算 1 回の失敗として数える（送ったときに数え済み）

    def _on_result(self, r: ThResult) -> None:
        if r.reason == "localization_lost":
            self._reentry_seen = True
        purpose = self._sent.get(r.command_id)
        g = self._goal
        if purpose == "goal" and r.result == "ARRIVED" and g.goal_id == r.command_id:
            g.arrived = True
        elif purpose == "abort" and r.result == "DONE" and self._abort is not None \
                and self._abort.th_id == r.command_id:
            self._abort.th_done = True

    # ------------------------------------------------------------------ 割り込み・見張り

    def _watch(self, snap: core.Snapshot) -> None:
        """走っている間は毎回 core.drive_watch。止めるなら送り口から meas.stop。"""
        w = core.drive_watch(snap, self.p)
        if not w.stop:
            return
        if w.reason == "lift_drift":
            self._gate_reason = "lift_drift"
        if self._stop_id is None:
            self._stop_id = self._emit("meas.stop", snap)
        if w.reason == "state_unreadable" and self._phase not in ("HALTED", "IDLE"):
            # 決め: 昇降機の高さか腕の状態が読めなくなったら、止めたうえで全体を止める（設計書 §2.1）
            self._halt(core.classify_halt(HaltEvent.LIFT_OR_ARM_UNREADABLE))

    def _interrupts(self, snap: core.Snapshot) -> bool:
        """どの段からでも効く割り込み。True なら、この step で段の処理はしない。"""
        if self._phase == "IDLE":
            return True
        th = self._th_fresh(snap)
        if th is not None and th.mode != "AUTO_MEASURE":
            self._to_idle()                   # 人が抜けた
            return True
        if self._phase in ("HALTED", "REENTRY"):
            return True
        if self._reentry_seen or (th is not None and th.reentry_required):
            self._to_reentry()
            return True
        if (self._phase in _LOWERING_PHASES and not self._lower_done and snap.lift is not None
                and snap.lift.operator_is_human):
            self._pause("human")              # 人が昇降機を操作した（人が優先）
            return True
        if th is not None and th.state == "PAUSE" and self._phase in _ACTIVE_PHASES:
            self._pause("th")
        return False

    # ------------------------------------------------------------------ 段の遷移

    def _enter(self, phase: str) -> None:
        self._phase = phase
        self._lower_started: Optional[float] = None
        self._lower_done = False
        self._job: Optional[_SsJob] = None
        self._work_started = False
        self._run_req = False
        self._goal = _Goal()

    def _pause(self, cause: str) -> None:
        if self._phase == "PAUSED":
            return
        self._paused_from, self._paused_cause = self._phase, cause
        job = self._job
        if job is not None and job.request_id and job.cleanup is None:
            self._emit("ss.cancel", request_id=job.request_id)
        self._job = None
        if self._phase == "AT_PANEL" and self._work_started:
            self._emit("arm.pause", paused=True)
        self._phase = "PAUSED"
        self._run_req = False

    def _resume(self) -> None:
        frm = self._paused_from
        self._run_req = False
        if frm == "AT_PANEL" and self._work_started:
            self._emit("arm.pause", paused=False)
        if frm == "GOAL":
            self._phase = "GOAL"              # 目的地はそのまま続く
        elif frm == "AT_PANEL":
            self._phase = "AT_PANEL"
        else:
            self._enter(frm)                  # 段の頭からやり直す（RECHECK は機体が動いたかもしれない）

    def _halt(self, reason: str) -> None:
        self._phase = "HALTED"
        self._halt_reason = reason
        self._abort = None
        self._job = None

    def _to_reentry(self) -> None:
        job = self._job
        if job is not None and job.request_id and job.cleanup is None:
            self._emit("ss.cancel", request_id=job.request_id)
        self._job = None
        self._abort = None
        self._phase = "REENTRY"

    def _to_idle(self) -> None:
        job = self._job
        if job is not None and job.request_id and job.cleanup is None:
            self._emit("ss.cancel", request_id=job.request_id)
        if self._phase == "AT_PANEL" and self._work_started:
            self._emit("arm.cancel")
        self._enter("IDLE")
        self._cur, self._queue = "", []
        self._layout_map_id, self._layout_panels, self._core_panels = "", [], {}
        self._rows, self._unseen, self._regoal = [], [], {}
        self._abort = None
        self._halt_reason = ""
        self._th_reject_reason = ""
        self._reentry_seen = False
        self._finishing = None
        self._stop_id = None

    def _back_to_select(self) -> None:
        self._cur, self._queue, self._abort = "", [], None
        self._enter("SELECT")

    def _record(self, panel_id: str, result: str, reason: str = "") -> None:
        rec = PanelResultView(panel_id, result, reason)
        for i, r in enumerate(self._results):
            if r.panel_id == panel_id:
                self._results[i] = rec
                return
        self._results.append(rec)

    def _skip(self, event: SkipEvent) -> None:
        self._record(self._cur, "SKIPPED", core.classify_skip(event))
        self._next_panel()

    def _next_panel(self) -> None:
        self._abort = None
        if self._queue:
            self._cur = self._queue.pop(0)
            self._enter("LOWER")
        else:
            self._cur = ""
            self._enter("FINISH")

    # ------------------------------------------------------------------ 昇降機を下げる

    def _lower_step(self, snap: core.Snapshot, need_origin: bool = False) -> str:
        """下降の命令を 1 つ出す（毎回 gate を通る）。'done' / 'wait' / 'halt' を返す。"""
        if self._lower_started is None:
            self._lower_started = snap.now_ms
        lift = snap.lift
        readable = self._height_readable(snap)
        if readable and lift.at_bottom and (not need_origin or lift.origin_ok is True):
            return "done"
        if snap.now_ms - self._lower_started > self.p.lift_lower_timeout_s * 1000.0:
            self._halt(core.classify_halt(HaltEvent.LIFT_CANNOT_LOWER))
            return "halt"
        if not readable:
            self._gate_reason = "state_unreadable"    # 高さが読めないまま下げない
            return "wait"
        self._emit(core.CMD_LIFT_LOWER, snap)         # 通らなければ出さない（次の step でまた見る）
        return "wait"

    # ------------------------------------------------------------------ 段の処理

    def _do_idle(self, snap):
        pass

    def _do_halted(self, snap):
        pass

    def _do_reentry(self, snap):
        pass

    def _do_prepare(self, snap):
        """腕と原点: 原点が取れていなければ下端まで下げる（腕・ss の条件は下降と依頼の関門が見る）。"""
        if snap.lift is None:
            self._gate_reason = "state_unreadable"
            return
        if snap.lift.origin_ok is True and self._height_readable(snap):
            self._enter("SURVEY")
            return
        self._lower_step(snap, need_origin=True)

    def _do_select(self, snap):
        pass

    def _do_lower(self, snap):
        if self._lower_step(snap) == "done":
            self._enter("GOAL")

    def _do_finish(self, snap):
        if not self._lower_done and self._lower_step(snap) == "done":
            self._lower_done = True

    def _do_paused(self, snap):
        th = self._th_fresh(snap)
        if self._paused_cause == "th":
            if th is not None and th.state != "PAUSE":
                self._resume()                # th が走り出した（人の「走行」の meas.start が通った）
                return
            if not self._run_req:
                return
            d = core.can_run(snap, self.p)
            if d.kind == "run":
                if self._emit("meas.start", snap) is not None:
                    self._run_req = False
            elif d.kind == "lower_first":
                self._lower_step(snap)
            else:
                self._run_req = False
                self._gate_reason = d.reason
        elif self._run_req and snap.lift is not None and snap.lift.operator_is_human is False:
            self._resume()                    # 今の段をやり直す

    # --- ss ---

    def _ss_step(self, snap: core.Snapshot, job: _SsJob) -> Optional[_SsDone]:
        """依頼・結果・後片付けの完了を見て、終わったら _SsDone。途中は None。"""
        p, now = self.p, snap.now_ms
        if job.stage == "send":
            if job.kind == "survey":
                frm, span = -math.pi, 2.0 * math.pi
            else:
                frm, span = -math.pi / 2.0, p.recheck_span_rad
            rid = self._emit(core.CMD_SS_REQUEST, snap, azimuth_from_rad=frm,
                             azimuth_span_rad=span)
            if rid is None:
                return None                   # 出せなかった（関門）。出せるまで待つ
            job.request_id, job.sent_ms, job.th_req = rid, now, snap.th
            job.waited, job.stage = True, "await_result"
            job.result = job.cleanup = None
            return None
        # 出したあと: 状態の通り道・th の状態を見張る
        if not core._ss_readable(snap, p):
            self._halt(core.classify_halt(HaltEvent.SS_LINK_LOST))
            return None
        th = self._th_fresh(snap)
        if th is not None and th.state != "WAIT":
            job.waited = False
        if job.stage == "await_result":
            if job.result is not None or job.cleanup is not None:
                job.stage, job.cleanup_started_ms = "await_cleanup", now
            elif now - job.sent_ms > p.ss_result_timeout_s * 1000.0 + p.ss_status_stale_ms:
                # ss 自身の時間切れ（部分の結果つき）が先に来るよう、ss の状態の途絶の許容分だけ待つ
                self._emit("ss.cancel", request_id=job.request_id)
                job.timed_out = True
                job.stage, job.cleanup_started_ms = "await_cleanup", now
            else:
                return None
        if job.cleanup is None:
            if now - job.cleanup_started_ms > p.ss_cleanup_timeout_s * 1000.0:
                self._halt(core.classify_halt(HaltEvent.SS_CLEANUP_NOT_DONE))
            return None
        if job.cleanup.reason == "human_override" or (
                job.result is not None and job.result.reason == "human_override"):
            self._pause("human")              # 人の操作で中断: PAUSED。人の「走行」で段の頭から
            return None
        if job.timed_out:
            return _SsDone(failure=SkipEvent.SS_TIMEOUT)
        if job.result is not None and core.ss_result_valid(
                job.th_req, th, job.waited, p.ss_pose_tol_m, p.ss_pose_tol_rad):
            return _SsDone(result=job.result, th_req=job.th_req)
        job.attempt += 1                      # 結果を捨てて依頼し直す
        if job.attempt > p.ss_retry_max:
            return _SsDone(failure=SkipEvent.SS_RESULT_UNUSABLE)
        job.stage, job.request_id = "send", None
        return None

    def _do_survey(self, snap):
        if self._job is None:
            self._job = _SsJob("survey")
        done = self._ss_step(snap, self._job)
        if done is None or self._phase != "SURVEY":
            return
        if done.failure is not None:          # 結果が来ない・使えない: 空の配置図（もう一度探せる）
            self.last_survey_failure = done.failure.name
            self._set_layout(snap, None, None)
        else:
            self.last_survey_failure = ""
            self._set_layout(snap, done.result, done.th_req)
        self._enter("SELECT")

    # --- 配置図 ---

    def _panel_view(self, pid: str, ss_rect: core.SsRect, rect: core.MapRect,
                    edge: core.FrontEdge, row_id: str) -> PanelView:
        p = self.p
        return PanelView(
            panel_id=pid, x=rect.x, y=rect.y, front_yaw=edge.front_yaw, width_m=rect.width_m,
            depth_m=rect.depth_m, top_height_m=ss_rect.top_height_m, pos_std_m=rect.pos_std_m,
            yaw_std_deg=rect.yaw_std_deg, lugs=rect.lugs,
            confident=core.is_confident(rect.pos_std_m, rect.yaw_std_deg, p.ss_pos_std_max_m,
                                        p.ss_yaw_std_max_deg),
            ss_front_known=rect.front_known,
            ss_front_yaw=edge.ss_front_yaw if edge.ss_front_yaw is not None else 0.0,
            front_doubt=edge.front_doubt, row_id=row_id)

    def _set_layout(self, snap, result: Optional[SsResult], th_req: Optional[core.ThStatus]) -> None:
        self._layout_panels, self._core_panels, self._rows, self._unseen = [], {}, [], []
        self._layout_map_id = th_req.map_instance_id if th_req is not None else (
            snap.th.map_instance_id if snap.th is not None else "")
        if result is None:
            return
        req = Pose(th_req.x, th_req.y, th_req.yaw)
        mp = core.to_map(result.panels, req, result.seen_from_rad, result.seen_span_rad)
        self._unseen = list(mp.unseen)
        views, cps = [], []
        for ss_rect, rect in zip(result.panels, mp.panels):
            edge = core.front_edge(rect, (req.x, req.y))
            self._panel_seq += 1
            pid = f"P{self._panel_seq:03d}"
            cps.append(core.make_panel(pid, rect, edge))
            views.append((pid, ss_rect, rect, edge))
        self._rows = core.group_rows(cps, self.p.row_yaw_tol_rad, self.p.row_gap_max_m)
        row_of = {pid: f"R{i + 1}" for i, r in enumerate(self._rows) for pid in r}
        for (pid, ss_rect, rect, edge), cp in zip(views, cps):
            self._layout_panels.append(self._panel_view(pid, ss_rect, rect, edge, row_of.get(pid, "")))
            self._core_panels[pid] = cp

    # --- GOAL ---

    def _do_goal(self, snap):
        g = self._goal
        if g.arrived:
            self._enter("RECHECK")
            return
        if g.stage == "plan":
            stand = (self._arm.stand_pose if self._arm else lambda c: core.stand_pose(
                c, self.p.stand_distance_min_m, self.p.stand_margin_m))(self._core_panels[self._cur])
            grid = self._th.grid() if self._th else None
            if grid is not None and core.stand_blocked(grid, stand, self.p.footprint_half_m,
                                                       self.p.map_occupied_min):
                self._skip(SkipEvent.STAND_AREA_OCCUPIED)    # 出さずに飛ばす
                return
            g.stand, g.stage = stand, "send"
        if g.stage == "send":
            cid = self._emit("meas.goal", snap, map_instance_id=self._layout_map_id,
                             x=g.stand.x, y=g.stand.y, yaw=g.stand.yaw)
            if cid is not None:
                g.goal_id, g.stage = cid, "await_reply"
            return
        if g.stage != "running":
            return
        th = self._th_fresh(snap)
        if th is None or th.state != "BLOCKED":
            g.blocked_since = None
            return
        if g.blocked_since is None:
            g.blocked_since = snap.now_ms
        elif snap.now_ms - g.blocked_since >= self.p.blocked_wait_s * 1000.0:
            if g.reroutes < self.p.reroute_max:
                if self._emit("meas.reroute", snap) is not None:
                    g.reroutes += 1
                    g.blocked_since = snap.now_ms
            else:
                self._abort = _Abort("no_path")       # 解けない: 止めて（meas.abort）飛ばす

    # --- RECHECK ---

    def _do_recheck(self, snap):
        if self._job is None:
            self._job = _SsJob("recheck")
        done = self._ss_step(snap, self._job)
        if done is None or self._phase != "RECHECK":
            return
        if done.failure is not None:
            self._skip(done.failure)                  # ss_failed
            return
        res = done.result
        if res.reason == "camera_too_low":
            self._skip(SkipEvent.CAMERA_TOO_LOW)
            return
        req = Pose(done.th_req.x, done.th_req.y, done.th_req.yaw)
        mp = core.to_map(res.panels, req, res.seen_from_rad, res.seen_span_rad)
        if not mp.panels:
            self._skip(SkipEvent.TOP_NOT_FOUND)
            return
        old = self._core_panels[self._cur]
        # 同じ盤の選び方（設計書に規則が無いので決めた）: 前の中心に最も近い矩形
        i = min(range(len(mp.panels)), key=lambda k: math.hypot(mp.panels[k].x - old.x,
                                                                mp.panels[k].y - old.y))
        rect, ss_rect = mp.panels[i], res.panels[i]
        if not core.is_confident(rect.pos_std_m, rect.yaw_std_deg, self.p.ss_pos_std_max_m,
                                 self.p.ss_yaw_std_max_deg):
            self._skip(SkipEvent.TOP_NOT_FOUND)
            return
        edge = core.front_edge(rect, (req.x, req.y))
        cp = core.make_panel(self._cur, rect, edge)
        self._core_panels[self._cur] = cp
        for k, v in enumerate(self._layout_panels):
            if v.panel_id == self._cur:
                self._layout_panels[k] = self._panel_view(self._cur, ss_rect, rect, edge, v.row_id)
        stand = (self._arm.stand_pose(cp) if self._arm else core.stand_pose(
            cp, self.p.stand_distance_min_m, self.p.stand_margin_m))
        now_pose = Pose(snap.th.x, snap.th.y, snap.th.yaw)
        if not core.needs_regoal(stand, now_pose, self.p.arrive_tol_m, self.p.arrive_tol_rad):
            self._enter("AT_PANEL")
        elif self._regoal.get(self._cur, 0) < self.p.regoal_max:
            self._regoal[self._cur] = self._regoal.get(self._cur, 0) + 1
            self._enter("GOAL")                       # 1 回だけ目的地を出し直す
        else:
            self._skip(SkipEvent.STAND_MISMATCH_TWICE)

    # --- AT_PANEL ---

    def _do_at_panel(self, snap):
        if not self._work_started:
            if core._mechanism_absent(snap) or self._emit("arm.start", snap,
                                                           panel_id=self._cur) is None:
                self._skip(SkipEvent.MECHANISM_ABSENT)   # 測定機構なしの道（設計書 §2.3）
                return
            self._work_started = True
            return
        lift = snap.lift
        if lift is None or lift.arm_link_ok is not True:
            self._halt(core.classify_halt(HaltEvent.ARM_LINK_LOST))   # 腕が出ている間に通信できない
            return
        for r in list(self._arm_results):
            if r.panel_id != self._cur:
                continue
            self._arm_results.remove(r)
            if r.result == "DONE":
                self._record(self._cur, "DONE", "")
                self._next_panel()
            elif r.result == "SKIPPED":
                self._skip(SkipEvent.ARM_SPACE_BLOCKED)
            elif r.result == "HALT":
                self._halt(core.classify_halt(HaltEvent.ARM_STUCK))
            return

    # --- 中止 ---

    def _abort_step(self, snap: core.Snapshot) -> None:
        """人の「中止」・経路が解けない（no_path）の流れ。meas.abort の結果を待ってから次へ進む。"""
        a = self._abort
        if a.stage == "th":
            if a.th_id is None and not a.th_done:
                th = self._th_fresh(snap)
                if th is not None and th.state in _RUNNING + ("PAUSE",):
                    a.th_id = self._emit("meas.abort", snap)
                    if a.th_id is None:
                        return
                else:
                    a.th_done = True          # すでに止まっている（目的地を持っていない）
            if not a.th_done:
                return
            if a.kind == "no_path":
                self._skip(SkipEvent.PATH_NOT_FOUND)
                return
            a.stage = "ss"
        if a.stage == "ss":
            job = self._job
            if job is not None and job.request_id and job.cleanup is None:
                if not a.ss_cancel_sent:
                    self._emit("ss.cancel", request_id=job.request_id)
                    a.ss_cancel_sent, a.ss_cancel_ms = True, snap.now_ms
                if snap.now_ms - a.ss_cancel_ms > self.p.ss_cleanup_timeout_s * 1000.0:
                    self._halt(core.classify_halt(HaltEvent.SS_CLEANUP_NOT_DONE))
                return
            a.stage = "arm"
        if a.stage == "arm":
            if self._work_started:
                if not a.arm_cancel_sent:
                    self._emit("arm.cancel")
                    a.arm_cancel_sent = True
                if not any(r.result == "CANCELLED" for r in self._arm_results):
                    return                    # 腕の格納を待つ
                self._arm_results = [r for r in self._arm_results if r.result != "CANCELLED"]
            self._back_to_select()
