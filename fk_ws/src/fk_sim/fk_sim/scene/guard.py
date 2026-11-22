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


def check_output(scene, out_dir, toplevel=None):
    """止める条件なら GuardError。書いてよい出力先の絶対パスを返す。"""
    toplevel = toplevel if toplevel is not None else repo_toplevel()
    out = os.path.realpath(out_dir)
    if scene["visibility"] == "site" and toplevel:
        top = os.path.realpath(toplevel)
        if out == top or out.startswith(top + os.sep):
            raise GuardError(
                f"visibility: site の記述の出力先が本リポジトリの中にあります（{out}）。"
                "site の出力はリポジトリの外へ出してください。")
    return out
