"""WP-SIM-11: 姿勢の組み立て・金具の振り分け・突き合わせの試験（ホストで回す）。"""

import ast
import inspect
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import appearance  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))


def test_stand_base_pose_computed():
    # stand_pose: 0.389 + 0.5 = 0.889 m を正面の辺（中心から 0.5 m）からへ
    pose = appearance.stand_base_pose()
    assert pose["x"] == pytest.approx(3.0)
    assert abs(pose["y"] - (3.0 + 0.5 + 0.389 + 0.5)) < 1e-9
    assert pose["yaw"] == pytest.approx(-math.pi / 2.0)
    # 記録の位置 (3.0, 4.66) と変換 0.27 m 食い違う（ブリーフどおり報告する差）
    assert abs(appearance.RECORDED_POSE["y"] - pose["y"]) > 0.1


def test_scan_pitches_include_38_2():
    pitches = appearance.scan_pitches_deg()
    assert len(pitches) == 3
    assert pitches[0] == pytest.approx(20.0, abs=0.5)
    assert pitches[1] == pytest.approx(38.2, abs=0.5)
    assert pitches[2] == pytest.approx(56.35, abs=0.5)


def test_build_poses_center_yaw_and_pitch_sign():
    poses = appearance.build_poses()
    center = [p for p in poses if p["kind"] == "center"]
    assert center, "center の姿勢が無い"
    for p in poses:
        assert p["cam_pitch_deg"] < 0.0  # 下向きは hve の符号で負
        assert p["height_m"] == pytest.approx(3.10 - 1.82)
    central = [p for p in center if p["cam_yaw_deg"] == 0.0]
    assert central, "中央ヨー 0 の姿勢が無い（P004 が画角の中央に入らない）"
    recorded = [p for p in poses if p["kind"] == "recorded"]
    assert recorded and recorded[0]["y"] == pytest.approx(4.66)


def test_classify_lug_near_far():
    robot = (3.0, 4.389)
    center = (3.0, 3.0)
    # 北側（正面側）の金具 2 個は近い、奥側（南側）の 2 個は遠い
    assert appearance.classify_lug((2.47, 3.57, 2.35), robot, center) == "near"
    assert appearance.classify_lug((3.53, 3.57, 2.35), robot, center) == "near"
    assert appearance.classify_lug((2.47, 2.43, 2.35), robot, center) == "far"
    assert appearance.classify_lug((3.53, 2.43, 2.35), robot, center) == "far"


def test_match_detections_tolerance():
    gt = [(100.0, 100.0), (200.0, 200.0)]
    dets = [(103.0, 104.0), (230.0, 200.0)]
    results, false = appearance.match_detections(gt, dets)
    assert results[0]["found"] and results[0]["err_px"] == pytest.approx(5.0)
    assert not results[1]["found"]  # 30 px は 15 px を超える
    assert false == 2 - 1


def test_match_detections_no_false_when_all_match():
    gt = [(100.0, 100.0)]
    results, false = appearance.match_detections(gt, [(100.0, 100.0)])
    assert results[0]["found"] and false == 0


def test_renderer_defaults_identical():
    sys.path.insert(0, os.path.join(REPO, "fk_ws", "src", "fk_sim"))
    from fk_sim.render import core  # noqa: E402
    sig = inspect.signature(core.Renderer.__init__)
    assert sig.parameters["metallic_factor"].default is None
    assert sig.parameters["roughness_factor"].default is None


def test_build_scene_defaults_identical():
    path = os.path.join(REPO, "fk_ws", "src", "fk_sim", "blender", "build_scene.py")
    tree = ast.parse(open(path, encoding="utf-8").read())
    defaults = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "attr", None) == "add_argument":
            name = node.args[0].value if node.args else ""
            for kw in node.keywords:
                if kw.arg == "default":
                    defaults[name] = ast.literal_eval(kw.value)
    assert defaults["--lug-rgb"] == "0.6,0.6,0.62"
    assert defaults["--lug-rough"] == 0.4
    assert defaults["--lug-metallic"] == 0.0
    assert defaults["--blend"] is None


def test_variants_base_empty_and_a_nonempty():
    assert appearance.VARIANTS["base"] == {"scene_overrides": {}, "renderer_overrides": {}}
    for name in ("a_bright", "a_dark", "a_gloss"):
        v = appearance.VARIANTS[name]
        assert v["scene_overrides"] or v["renderer_overrides"], name
