"""出力先の守り（DetailedDesign-sim.md §8）: site の記述を本リポジトリの中へ書かない。"""

import os
import subprocess


class GuardError(RuntimeError):
    """site の記述の出力先がリポジトリの中にある。"""


def repo_toplevel(start=None):
    """git リポジトリの直下（無ければ None）。"""
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            cwd=start or os.getcwd(), capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError):
        return None
    if out.returncode != 0:
        return None
    return os.path.realpath(out.stdout.strip())


def _tree_root_for(out):
    """出力先から上へたどり、本システムの作業ツリーの直下を見つける。

    目印は直下の fk_ws/src/fk_sim（worktree でも同じ）。cwd や git に頼らない
    （リポジトリの外から打つと素通りしていた穴を塞ぐ）。見つからなければ None。
    出力先がまだ無くても、上へたどれば直下は見つかる。
    """
    cur = out
    while True:
        if os.path.isdir(os.path.join(cur, "fk_ws", "src", "fk_sim")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            return None
        cur = parent


def check_output(scene, out_dir):
    """止める条件なら GuardError。書いてよい出力先の絶対パスを返す。"""
    try:
        out = os.path.realpath(out_dir)
    except (TypeError, ValueError) as e:
        if scene["visibility"] == "site":
            raise GuardError(f"site の出力先（{out_dir!r}）を解決できません。書かずに止めます: {e}") from e
        raise
    if scene["visibility"] == "site":
        top = _tree_root_for(out)
        # ルートまでたどって目印が無ければ作業ツリーの外なので書いてよい。
        # （パスが解決できないときは上の except で止める）
        if top is not None and (out == top or out.startswith(top + os.sep)):
            raise GuardError(
                f"visibility: site の記述の出力先が本リポジトリの中にあります（{out}）:"
                f" {top} の下。site の出力はリポジトリの外へ出してください。")
    return out
