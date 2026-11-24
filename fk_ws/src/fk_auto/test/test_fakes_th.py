"""ThModel の約束の試験（th DetailedDesign-measure.md §3.1〜§3.3・Spec-modes SM-3.1.2-120〜-142）。

時刻は引数だけ。受けた指示には結果がちょうど 1 回・目的地を持ったまま PAUSE に落ちたら出さない。
"""
import pytest

from fk_auto.fakes import ThModel


class Th:
    """ThModel を時刻つきで動かす小さな道具。"""

    def __init__(self, **kw):
        self.m = ThModel(**kw)
        self.now = 0.0
        self.replies = []
        self.results = []
        self.n = 0
        self.tick()

    def tick(self, ms=100, hb="1"):
        self.now += ms
        if hb is not None:
            self.m.on_heartbeat(hb, self.now)
        self.m.advance(self.now)
        self.replies += self.m.drain_replies()
        self.results += self.m.drain_results()

    def send(self, cmd, cid=None, hb="1", **args):
        self.n += 1
        cid = f"c{self.n}" if cid is None else cid
        self.m.submit(cid, cmd, self.now, **args)
        self.tick(hb=hb)
        return cid

    def reply(self, cid):
        return [r for r in self.replies if r.command_id == cid][-1]

    def res(self, cid):
        return [r for r in self.results if r.command_id == cid]


def entered(**kw):
    t = Th(**kw)
    assert t.m.human_enter_auto()
    t.tick()
    return t


def goal(t, x=1.0, y=0.0, yaw=0.0, hb="1"):
    return t.send("meas.goal", hb=hb, map_instance_id=t.m.map_instance_id, x=x, y=y, yaw=yaw)


# --- 入口・地図の世代 -------------------------------------------------------------

def test_enter_requires_link_and_new_map_each_time():
    t = Th()
    t.m.status_frozen = False
    t2 = Th()
    t2.tick(hb=None)
    t2.tick(1500, hb=None)              # 生存確認が途絶えた
    assert not t2.m.human_enter_auto()  # -120: 通信できなければ拒否
    assert t.m.human_enter_auto()
    first = t.m.map_instance_id
    t.m.human_exit()
    t.m.human_enter_auto()
    assert t.m.map_instance_id != first  # 入るたびに地図を作り直す


# --- 拒否の順（fake_th.py と同じ）--------------------------------------------------

def test_reject_empty_command_id_first():
    t = entered()
    t.send("meas.goal", cid="", map_instance_id="x", x=0, y=0, yaw=0)
    assert t.replies[-1].reject_reason == "bad_argument"


def _never_valid():
    t = Th()
    t.m._ever_valid_hb = False           # できたてに版の合う受信が無い状態へ戻す
    t.m._last_valid_hb_ms = None
    return t


def test_reject_version_mismatch_only_before_any_valid_heartbeat():
    # 版の合う受信が一度も無いときだけ version_mismatch（DetailedDesign-auto.md §2.7）
    t = _never_valid()
    t.tick(hb="2")                       # 版違いだけ届いた
    cid = goal(t, hb="2")
    assert t.reply(cid).reject_reason == "version_mismatch"


def test_wrong_version_after_valid_heartbeat_is_link_lost_not_version_mismatch():
    t = entered()                        # 版の合う受信が一度ある
    t.tick(hb="2")                       # 版違いの生存確認（数えられない）
    t.tick(1500, hb="2")                 # 最後の有効な受信から途絶
    cid = goal(t, hb="2")
    assert t.reply(cid).reject_reason == "link_lost"


def test_duplicate_command_id_is_bad_argument_before_version_and_link():
    t = entered()
    cid = goal(t)                        # 進行中の id
    dup = t.send("meas.stop", cid=cid)
    assert t.reply(dup).reject_reason == "bad_argument"
    t.tick(3000)                         # 到着して終わっても覚えている
    dup2 = t.send("meas.stop", cid=cid)
    assert t.reply(dup2).reject_reason == "bad_argument"
    t2 = entered()
    t2.m._ever_valid_hb = False          # 版の合う受信が無い状態へ戻す
    t2.m._last_valid_hb_ms = None
    t2.tick(hb="2")
    cid2 = goal(t2, hb="2")              # 拒否された id は覚えない（受理したものだけ）
    assert t2.reply(cid2).reject_reason == "version_mismatch"
    t2.tick(hb="1")
    retry = t2.send("meas.goal", cid=cid2, map_instance_id=t2.m.map_instance_id,
                    x=1.0, y=0.0, yaw=0.0)
    assert t2.reply(retry).accepted


def test_reject_link_lost_after_timeout_and_before_any_heartbeat():
    t = entered()
    t.tick(1500, hb=None)
    cid = goal(t, hb=None)
    assert t.reply(cid).reject_reason == "link_lost"
    fresh = ThModel()
    fresh.submit("c", "meas.goal", 0.0)
    fresh.advance(0.0)
    assert fresh.drain_replies()[0].reject_reason == "link_lost"   # まだ 1 つも受けていない


def test_wrong_version_heartbeat_does_not_count_as_alive():
    m = ThModel()
    m.on_heartbeat("2", 0.0)
    m.advance(0.0)
    assert not m.link_ok()


# --- 状態に応じた拒否（§3.1 の表）---------------------------------------------------

def test_idle_mode_rejects_everything():
    t = Th()
    cid = t.send("meas.goal", map_instance_id="", x=0, y=0, yaw=0)
    assert t.reply(cid).reject_reason == "state_not_accepting"


def test_goal_only_in_wait_with_matching_map_and_no_corner_obstacle():
    t = entered()
    cid = t.send("meas.goal", map_instance_id="old", x=1, y=0, yaw=0)
    assert t.reply(cid).reject_reason == "map_mismatch"
    t.m.corner_obstacle = True
    cid = goal(t)
    assert t.reply(cid).reject_reason == "corner_obstacle"
    t.m.corner_obstacle = False
    cid = goal(t)
    assert t.reply(cid).accepted and t.m.state == "NAV"
    cid2 = goal(t)                        # NAV 中は受けない（-132）
    assert t.reply(cid2).reject_reason == "state_not_accepting"


def test_unsupported_and_ui_commands_rejected():
    t = entered()
    for cmd in ("meas.turn", "meas.translate"):      # ADJUST が無い簡略化
        cid = t.send(cmd)
        assert t.reply(cid).reject_reason == "state_not_accepting"
    for cmd in ("ui.start", "meas.bogus"):          # meas. 以外・未知の meas.* は bad_argument
        cid = t.send(cmd)
        assert t.reply(cid).reject_reason == "bad_argument"
    # bad_argument は版・途絶より先（版違いだけの th でも bad_argument）
    t2 = _never_valid()
    t2.tick(hb="2")
    cid = t2.send("meas.bogus", hb="2")
    assert t2.reply(cid).reject_reason == "bad_argument"


@pytest.mark.parametrize("cmd,state", [("meas.reroute", "WAIT"), ("meas.start", "WAIT"),
                                       ("meas.stop", "WAIT"), ("meas.abort", "WAIT")])
def test_commands_rejected_in_wrong_state(cmd, state):
    t = entered()
    cid = t.send(cmd)
    assert t.reply(cid).reject_reason == "state_not_accepting"
    assert t.res(cid) == []             # 拒否した指示に結果は出さない


# --- 結果は受けた指示に必ず 1 回（§3.2）---------------------------------------------

def test_goal_arrives_after_nav_time_with_single_arrived_and_pose_updated():
    t = entered(nav_time_ms=1000)
    cid = goal(t, x=2.0, y=1.0, yaw=0.5)
    states = set()
    for _ in range(15):
        t.tick()
        states.add(t.m.state)
    assert {"NAV", "ALIGN", "WAIT"} <= states
    assert [r.result for r in t.res(cid)] == ["ARRIVED"]       # ちょうど 1 回
    assert (t.m.x, t.m.y, t.m.yaw) == (2.0, 1.0, 0.5)


@pytest.mark.parametrize("cmd", ["meas.reroute", "meas.stop"])
def test_immediate_done_commands(cmd):
    t = entered()
    goal(t)
    cid = t.send(cmd)
    assert [r.result for r in t.res(cid)] == ["DONE"]


def test_stop_pauses_keeps_goal_and_emits_no_result_for_goal():
    t = entered()
    g = goal(t)
    t.send("meas.stop")
    assert t.m.state == "PAUSE"
    t.tick(3000)
    assert t.res(g) == []                # 目的地を持ったまま PAUSE: 結果は出さない
    s = t.send("meas.start")
    assert [r.result for r in t.res(s)] == ["DONE"]
    t.tick(3000)
    assert [r.result for r in t.res(g)] == ["ARRIVED"]   # もとの command_id のまま続く


def test_abort_cancels_pending_goal_exactly_once_and_done_for_abort():
    """§3.2 の表の CANCELLED の行: 中止は進行中の指示を CANCELLED(aborted)、abort 自身は DONE。"""
    t = entered()
    g = goal(t)
    a = t.send("meas.abort")
    assert [(r.result, r.reason) for r in t.res(g)] == [("CANCELLED", "aborted")]
    assert [r.result for r in t.res(a)] == ["DONE"]
    assert t.m.state == "WAIT"
    t.tick(3000)
    assert len(t.res(g)) == 1


def test_abort_from_pause_cancels_goal():
    t = entered()
    g = goal(t)
    t.m.human_jog()
    t.send("meas.abort")
    assert [(r.result, r.reason) for r in t.res(g)] == [("CANCELLED", "aborted")]


def test_finish_cancels_pending_and_leaves_auto_mode():
    t = entered()
    g = goal(t)
    f = t.send("meas.finish")
    assert [(r.result, r.reason) for r in t.res(g)] == [("CANCELLED", "finished")]
    assert [r.result for r in t.res(f)] == ["DONE"]
    assert t.m.mode == "IDLE"


def test_human_exit_cancels_pending_with_mode_exit():
    t = entered()
    g = goal(t)
    t.m.human_exit()
    t.tick()
    assert [(r.result, r.reason) for r in t.res(g)] == [("CANCELLED", "mode_exit")]


def test_jog_pauses_without_result_and_start_resumes_prev_state():
    t = entered()
    g = goal(t)
    t.m.human_jog()
    assert t.m.state == "PAUSE" and t.res(g) == []
    t.send("meas.start")
    assert t.m.state == "NAV"


def test_comm_loss_while_running_pauses_without_result():
    t = entered()
    g = goal(t)
    t.tick(1500, hb=None)               # -133: 通信途絶
    assert t.m.state == "PAUSE" and t.res(g) == []
    t.tick(100)                         # 通信が戻っても自分では走らない
    assert t.m.state == "PAUSE"


def test_localization_lost_cancels_goal_and_requires_reentry():
    t = entered()
    g = goal(t)
    t.m.lose_localization()
    t.tick()
    assert [(r.result, r.reason) for r in t.res(g)] == [("CANCELLED", "localization_lost")]
    assert t.m.status().reentry_required
    for cmd in ("meas.start",):
        cid = t.send(cmd)
        assert t.reply(cid).reject_reason == "localization_lost"
    t.m.human_exit()
    t.m.human_enter_auto()
    assert not t.m.status().reentry_required     # 入り直すと解ける


def test_goal_rejected_while_reentry_required():
    t = entered()
    t.m.lose_localization()
    cid = goal(t)
    assert t.reply(cid).reject_reason == "localization_lost"


# --- 非常停止・手押し（ESTOP・CARRY）-------------------------------------------

def test_estop_holds_goal_and_release_returns_to_pause_then_start_continues():
    t = entered()
    g = goal(t)
    t.m.human_estop()
    assert (t.m.mode, t.m.state) == ("ESTOP", "NONE")
    t.tick(3000)
    assert t.res(g) == []                # 目的地を持ったまま結果を出さない
    t.m.human_estop_release()
    assert (t.m.mode, t.m.state) == ("AUTO_MEASURE", "PAUSE")
    t.tick(3000)
    assert t.res(g) == []                # 解除後も自分では走らない
    t.send("meas.start")
    assert t.m.state == "NAV"
    t.tick(3000)
    assert [r.result for r in t.res(g)] == ["ARRIVED"]


def test_carry_holds_goal_like_estop():
    t = entered()
    g = goal(t)
    t.m.human_carry()
    assert (t.m.mode, t.m.state) == ("CARRY", "NONE")
    t.tick(3000)
    assert t.res(g) == []
    t.m.human_carry_release()
    assert (t.m.mode, t.m.state) == ("AUTO_MEASURE", "PAUSE")
    t.send("meas.start")
    t.tick(3000)
    assert [r.result for r in t.res(g)] == ["ARRIVED"]


def test_start_from_blocked_returns_to_nav_and_from_align_to_align():
    t = entered()
    goal(t)
    t.m.set_blocked(True)
    t.send("meas.stop")                 # BLOCKED から PAUSE
    assert t.m.state == "PAUSE"
    t.m.set_blocked(False)
    t.send("meas.start")
    assert t.m.state == "NAV"           # 止める前が BLOCKED なら NAV（BLOCKED ではない）
    t2 = entered()
    goal(t2)
    t2.m.state = "ALIGN"
    t2.send("meas.stop")
    t2.send("meas.start")
    assert t2.m.state == "ALIGN"


def test_enter_auto_only_from_idle():
    t = entered()
    assert not t.m.human_enter_auto()   # すでに入っている
    t.m.human_estop()
    assert not t.m.human_enter_auto()


def test_localization_lost_goes_to_estop_and_exit_clears_reentry():
    t = entered()
    g = goal(t)
    t.m.lose_localization()
    assert (t.m.mode, t.m.state) == ("ESTOP", "NONE")
    t.tick()
    assert [(r.result, r.reason) for r in t.res(g)] == [("CANCELLED", "localization_lost")]
    assert t.m.status().reentry_required
    t.m.human_exit()
    assert (t.m.mode, t.m.state) == ("IDLE", "NONE") and not t.m.status().reentry_required


# --- 状態 BLOCKED ---------------------------------------------------------------

def test_blocked_and_reroute():
    t = entered()
    g = goal(t)
    t.m.set_blocked(True)
    assert t.m.state == "BLOCKED"
    t.tick(5000)
    assert t.m.state == "BLOCKED" and t.res(g) == []   # BLOCKED の間は進まない
    r = t.send("meas.reroute")
    assert t.m.state == "BLOCKED"                       # まだ塞がっているなら BLOCKED のまま
    t.m.set_blocked(False)
    assert t.m.state == "NAV"
    t.tick(2000)
    assert [x.result for x in t.res(g)] == ["ARRIVED"]


# --- 時刻・遅延 ----------------------------------------------------------------

def test_command_latency_delays_reply_and_effect():
    t = entered(command_latency_ms=300)
    m = t.m
    m.submit("late", "meas.goal", t.now, map_instance_id=m.map_instance_id, x=1, y=0, yaw=0)
    t.tick(100)
    assert t.replies == [] and m.state == "WAIT"
    t.tick(300)
    assert t.reply("late").accepted and m.state == "NAV"


def test_status_freeze_stops_read_time():
    t = entered()
    before = t.m.status().read_ms
    t.m.status_frozen = True
    t.tick(1000)
    assert t.m.status().read_ms == before
