"""口（DetailedDesign-auto.md §1.3）の抽象と NoMechanism。

ROS 2 にも通信にも触れない（rclpy・ROS のメッセージ型を import しない）。
本物（RosThPort ほか）と偽物（fakes.py）は同じ形で差し替える。時刻はどの口も読まない
（Runner が step(now_ms) で受けた時刻だけが動く。偽物は advance(now_ms) で進める）。

口とのやり取りの形（WP-AUTO-03 段階 A で決めた）:
- 読むもの: status() など。分からないものは None（Runner は「読めない」として扱う）。
- 出すもの: send()・lower()・request() など。**Runner の送り口（_emit）だけが呼ぶ**。
- 出来事（th の返事と結果・ss の結果と後片付けの完了・測定機構の結果）は、口に溜めておき、
  Runner が step のたびに poll_*() で取り出す（取り出したら口からは消える）。
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Tuple

from fk_auto import core


# ---------------------------------------------------------------------------
# 出来事の型
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ThReply:
    """/measure/command の応答（受けたか・拒否の理由キー）。"""
    command_id: str
    accepted: bool
    reject_reason: str = ""
    mode: str = ""
    state: str = ""


@dataclass(frozen=True)
class ThResult:
    """/measure/result（受けた指示の終わり。result は ARRIVED / DONE / CANCELLED）。"""
    command_id: str
    command: str
    result: str
    reason: str = ""


@dataclass(frozen=True)
class SsResult:
    """ss の結果（依頼した時点の機体基準）。reason は ss の名前のまま（completed・timeout など）。"""
    request_id: str
    reason: str
    panels: Tuple[core.SsRect, ...] = ()
    seen_from_rad: Tuple[float, ...] = ()
    seen_span_rad: Tuple[float, ...] = ()


@dataclass(frozen=True)
class SsCleanup:
    """ss の後片付けの完了。reason は結果の reason と同じ。"""
    request_id: str
    reason: str
    lift_restored: bool = True


@dataclass(frozen=True)
class ArmResult:
    """盤前の作業の結果。result は DONE / SKIPPED（reason は arm_space_blocked）/ HALT（reason は
    arm_stuck）/ CANCELLED（Runner が中止を渡し、腕を格納し終えた）。"""
    panel_id: str
    result: str
    reason: str = ""


# ---------------------------------------------------------------------------
# 口
# ---------------------------------------------------------------------------

class ThPort(ABC):
    """th の口。指示は非同期（返事は後で poll_replies に来る）。"""

    @abstractmethod
    def status(self) -> Optional[core.ThStatus]:
        """/measure/status の最後の値（read_ms は受けた時刻）。まだ読めていなければ None。"""

    @abstractmethod
    def grid(self) -> Optional[core.Grid]:
        """/map の最後の値。無ければ None。"""

    @abstractmethod
    def send(self, command_id: str, command: str, map_instance_id: str = "",
             x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        """meas.* を出す。Runner の送り口だけが呼ぶ。"""

    @abstractmethod
    def poll_replies(self) -> List[ThReply]: ...

    @abstractmethod
    def poll_results(self) -> List[ThResult]: ...


class LiftPort(ABC):
    @abstractmethod
    def status(self) -> Optional[core.LiftStatus]:
        """昇降機・腕の状態（DetailedDesign-auto.md §1.3）。口が無い・読めないなら None。"""

    @abstractmethod
    def lower(self) -> None:
        """下降の命令を 1 つ出す（下端に着くまで 1 step ごとに送り続ける）。"""

    @abstractmethod
    def stop(self) -> None:
        """昇降機を止める（Runner は今のところ使わない。口の抽象として持つ）。"""


class SsPort(ABC):
    @abstractmethod
    def status(self) -> Optional[core.SsStatus]:
        """ss の状態（state は IDLE / RUNNING / CLEANING。read_ms は受けた時刻）。"""

    @abstractmethod
    def request(self, azimuth_from_rad: float, azimuth_span_rad: float, max_distance_m: float,
                time_limit_s: float) -> Optional[str]:
        """依頼を出す。受け付けられなければ None（ss が拒否した）。受けたら依頼の ID。"""

    @abstractmethod
    def cancel(self, request_id: str) -> None: ...

    @abstractmethod
    def poll_results(self) -> List[SsResult]: ...

    @abstractmethod
    def poll_cleanups(self) -> List[SsCleanup]: ...


class ArmPort(ABC):
    """測定機構の口。"""

    @abstractmethod
    def stand_pose(self, panel: core.Panel) -> core.Pose:
        """盤ごとの立ち位置（最初の版は core.stand_pose を呼ぶ）。"""

    @abstractmethod
    def arm_stowed(self) -> Optional[bool]:
        """腕が格納位置か（None は読めない）。Runner は昇降機の状態（LiftStatus）から読むので、
        今のところ使わない。"""

    @abstractmethod
    def start_work(self, panel_id: str) -> bool:
        """盤前の作業を始める。受けなければ False（NoMechanism）。"""

    @abstractmethod
    def cancel_work(self) -> None:
        """中止（腕を格納してから止める）。格納し終えたら ArmResult(CANCELLED) を返す。"""

    @abstractmethod
    def set_paused(self, paused: bool) -> None:
        """一時停止・再開を知らせるだけ。どうするかは測定機構の側（DetailedDesign-auto.md §2.2）。"""

    @abstractmethod
    def poll_results(self) -> List[ArmResult]: ...


class NoMechanism(ArmPort):
    """最初の版の測定機構の口（測定機構なし）。作業を受けない。"""

    def __init__(self, params: Optional[core.Params] = None):
        self._p = params or core.Params()

    def stand_pose(self, panel: core.Panel) -> core.Pose:
        return core.stand_pose(panel, self._p.stand_distance_min_m, self._p.stand_margin_m)

    def arm_stowed(self) -> Optional[bool]:
        return True   # 腕は無い（腕の条件は満たしているとみなす。Spec-auto.md §7）

    def start_work(self, panel_id: str) -> bool:
        return False

    def cancel_work(self) -> None:
        pass

    def set_paused(self, paused: bool) -> None:
        pass

    def poll_results(self) -> List[ArmResult]:
        return []
