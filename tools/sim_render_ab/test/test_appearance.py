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
    # 期待値は stand_pose を通さずに数で書く（stand の計算を変えたらこの試験で気づく）:
    # 北の面は中心から width/2 = 0.6 m（y=3.6）。立ち位置は 3.0 + 0.6 + 0.389 + 0.5 = 4.489。
    pose = appearance.stand_base_pose()
    assert pose["x"] == pytest.approx(3.0)
    assert pose["y"] == pytest.approx(4.489)
    assert pose["yaw"] == pytest.approx(-math.pi / 2.0)
    # 記録の位置 (3.0, 4.66) との差は 0.17 m（ブリーフどおり報告する差）
    assert abs(appearance.RECORDED_POSE["y"] - pose["y"]) == pytest.approx(0.171)


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
    # ヨーの振り（下向き 38.2\u00b0 の段、\u221224〜+24\u00b0 の 6\u00b0 刻み）
    for kind in ("sweep_stand", "sweep_recorded"):
        sweep = [p for p in poses if p["kind"] == kind]
        assert len(sweep) == 9, kind
        assert [p["cam_yaw_deg"] for p in sweep] == [float(y) for y in range(-24, 25, 6)]
        for p in sweep:
            assert p["cam_pitch_deg"] == pytest.approx(-38.176, abs=0.5)


def test_classify_lug_near_far():
    robot = (3.0, 4.489)
    center = (3.0, 3.0)
    # 北側（正面側）の金具 2 個は近い、奥側（南側）の 2 個は遠い
    assert appearance.classify_lug((2.47, 3.57, 2.35), robot, center) == "near"
    assert appearance.classify_lug((3.53, 3.57, 2.35), robot, center) == "near"
    assert appearance.classify_lug((2.47, 2.43, 2.35), robot, center) == "far"
    assert appearance.classify_lug((3.53, 2.43, 2.35), robot, center) == "far"


def test_edge_class_boundary():
    # p01 の右の金具（1256, 427。縁から 24 px）は (a)、p02 の金具（958, 384。縁から 322 px）は (b)
    assert appearance.edge_class((1256.0, 427.0)) == "a"
    assert appearance.edge_class((958.0, 384.0)) == "b"
    assert appearance.edge_margin_px((1256.0, 427.0)) == pytest.approx(24.0)
    assert appearance.EDGE_RING_RADIUS_PX == 76.0


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
    # None のときに実際に材質へ入る値が今と同じ（0.0・0.7）であることを数で確かめる
    assert core.material_factors(None, None) == (0.0, 0.7)
    assert core.material_factors(0.0, 0.15) == (0.0, 0.15)
    assert core.material_factors(None, 0.15) == (0.0, 0.15)
    assert core.material_factors(0.5, None) == (0.5, 0.7)
    # 呼び出し側（__init__）がこの関数を通ることも縛る
    src = open(os.path.join(REPO, "fk_ws", "src", "fk_sim", "fk_sim", "render",
                            "core.py"), encoding="utf-8").read()
    assert "metal, rough = material_factors(metallic_factor, roughness_factor)" in src


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
    # 金具の既定色は 2026-11-26 に実物の写真に合わせて変えた（LUG_RGB と同じ値であること）
    lug_rgb = next(ast.literal_eval(n.value) for n in ast.walk(tree)
                   if isinstance(n, ast.Assign) and getattr(n.targets[0], "id", None) == "LUG_RGB")
    assert defaults["--lug-rgb"] == "0.72,0.72,0.66"
    assert tuple(float(v) for v in defaults["--lug-rgb"].split(",")) == lug_rgb
    assert defaults["--lug-rough"] == 0.4
    assert defaults["--lug-metallic"] == 0.0
    assert defaults["--blend"] is None


def test_variants_base_empty_and_a_nonempty():
    assert appearance.VARIANTS["base"] == {"scene_overrides": {}, "renderer_overrides": {}}
    for name in ("a_bright", "a_dark", "a_gloss"):
        v = appearance.VARIANTS[name]
        assert v["scene_overrides"] or v["renderer_overrides"], name
