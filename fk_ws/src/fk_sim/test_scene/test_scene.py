"""シーン生成器の試験（ホストで回す。ROS 2 なし）。

幾何の正しさは、実装側の関数を使わずに記述から直接計算した値と比べる
（回転の符号の誤りを捕まえるため）。
"""
import json
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

from fk_sim import scene
from fk_sim.scene import guard, model
from fk_sim.scene.answers import build_answer, compare_blender_objects
from fk_sim.scene.world import build_world

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENES_DIR = os.path.join(ROOT, "scenes")
PUBLIC_ROOM = os.path.join(SCENES_DIR, "public_room.yaml")
TOL = 1e-6


def write_scene(tmp_path, data, name="t.yaml"):
    path = os.path.join(str(tmp_path), name)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True)
    return path


def base_scene():
    """回転した盤を含む最小の記述（独立計算との比べ用）。"""
    return {
        "visibility": "public", "name": "t_room",
        "room": {"size_x": 10.0, "size_y": 8.0, "ceiling_h": 4.0},
        "panels": [
            {"id": "P001", "center": [4.0, 5.0], "front_yaw": 0.7,
             "width": 1.0, "depth": 0.6, "height": 2.35, "label": "No.9"},
            {"id": "P002", "center": [8.0, 2.0], "front_yaw": -1.57,
             "width": 0.8, "depth": 0.8, "height": 2.35, "label": "No.1"},
        ],
        "obstacles": [{"center": [2.0, 2.0], "size": [0.8, 0.5, 0.4], "yaw": 0.3}],
        "robot": {"pose": [5.0, 1.0, 0.0]},
        "lights": [{"type": "point", "position": [5.0, 4.0, 3.5], "energy": 1.0}],
    }


# --- ワールド（SDF）の箱が記述と一致すること ---

def _models(world_xml):
    root = ET.fromstring(world_xml)
    return {m.get("name"): m for m in root.find("world").findall("model")}


def _box_size_and_pose(m):
    link = m.find("link")
    size = [float(v) for v in link.find("visual/geometry/box/size").text.split()]
    pose = [float(v) for v in m.find("pose").text.split()]
    return size, pose


def test_world_boxes_match_description():
    scene_data = model.validate_scene(base_scene())
    models = _models(build_world(scene_data))
    # 壁 4 面・床・天井・盤 2 面・障害物 1 つ
    for name in ("floor", "wall_south", "wall_north", "wall_west", "wall_east",
                 "ceiling", "panel_P001", "panel_P002", "obstacle_0"):
        assert name in models, f"モデル {name} がワールドにありません"
    assert len(models) == 9
    # 壁は部屋の内法に合わせた大きさ
    size, pose = _box_size_and_pose(models["wall_south"])
    assert size == [10.0, 0.15, 4.0]
    assert pose[1] == pytest.approx(0.0)
    size, pose = _box_size_and_pose(models["wall_north"])
    assert size == [10.0, 0.15, 4.0]
    assert pose[1] == pytest.approx(8.0)
    size, pose = _box_size_and_pose(models["wall_west"])
    assert size == [0.15, 8.0, 4.0]
    assert pose[0] == pytest.approx(0.0)
    size, pose = _box_size_and_pose(models["wall_east"])
    assert size == [0.15, 8.0, 4.0]
    assert pose[0] == pytest.approx(10.0)
    # 盤は中心・大きさ・向きが記述どおり（向きは θ = front_yaw + π/2）
    for p in base_scene()["panels"]:
        size, pose = _box_size_and_pose(models[f"panel_{p['id']}"])
        assert size == [p["width"], p["depth"], p["height"]]
        assert pose[0] == pytest.approx(p["center"][0])
        assert pose[1] == pytest.approx(p["center"][1])
        assert pose[2] == pytest.approx(p["height"] / 2)
        assert pose[5] == pytest.approx(p["front_yaw"] + math.pi / 2, abs=1e-5)
    # 障害物
    size, pose = _box_size_and_pose(models["obstacle_0"])
    assert size == [0.8, 0.5, 0.4]
    assert pose[:2] == [2.0, 2.0]
    assert pose[5] == pytest.approx(0.3, abs=1e-5)
    # 天井は衝突なし・床は衝突あり（LiDAR に当たる形）
    assert models["ceiling"].find("link/collision") is None
    assert models["floor"].find("link/collision") is not None
    assert models["panel_P001"].find("link/collision") is not None


# --- scene.json の天面四隅と金具が独立計算と一致すること（回転あり） ---

def independent_panel(panel):
    """記述から直接計算する。front_yaw を φ、回転を θ = φ + π/2 とする。"""
    th = panel["front_yaw"] + math.pi / 2
    c, s = math.cos(th), math.sin(th)
    cx, cy = panel["center"]
    w, d, h = panel["width"], panel["depth"], panel["height"]

    def to_world(lx, ly):
        return [cx + c * lx - s * ly, cy + s * lx + c * ly, h]

    corners = {"FL": to_world(-w / 2, -d / 2), "FR": to_world(w / 2, -d / 2),
               "BL": to_world(-w / 2, d / 2), "BR": to_world(w / 2, d / 2)}
    lugs = {}
    for corner, (lx, ly) in {"FL": (-w / 2 + 0.07, -d / 2 + 0.07),
                             "FR": (w / 2 - 0.07, -d / 2 + 0.07),
                             "BL": (-w / 2 + 0.07, d / 2 - 0.07),
                             "BR": (w / 2 - 0.07, d / 2 - 0.07)}.items():
        x, y, _ = to_world(lx, ly)
        lugs[corner] = {"base": [x, y, h], "ring_center": [x, y, h + 0.0365]}
    return corners, lugs


def test_answer_corners_and_lugs_match_independent_calc():
    scene_data = model.validate_scene(base_scene())
    answer = build_answer(scene_data)
    assert [p["id"] for p in answer["panels"]] == ["P001", "P002"]
    for p, got in zip(base_scene()["panels"], answer["panels"]):
        corners, lugs = independent_panel(p)
        for corner in ("FL", "FR", "BL", "BR"):
            assert got["top_corners"][corner] == pytest.approx(corners[corner], abs=1e-9)
            assert got["lugs"][corner]["base"] == pytest.approx(lugs[corner]["base"], abs=1e-9)
            assert got["lugs"][corner]["ring_center"] == pytest.approx(
                lugs[corner]["ring_center"], abs=1e-9)
    # 回転が効いていること（front_yaw=0.7 でただの並進では無い）
    corners, _ = independent_panel(base_scene()["panels"][0])
    fl = corners["FL"]
    assert fl[0] != pytest.approx(4.0 - 0.5) or fl[1] != pytest.approx(5.0 - 0.3)
    # lugs=none は空
    scene_data = model.validate_scene({**base_scene(),
                                       "panels": [dict(base_scene()["panels"][0], lugs="none")]})
    assert build_answer(scene_data)["panels"][0]["lugs"] == {}


# --- 守り（site はリポジトリの中へ書けない） ---

def _make_fake_tree(tmp_path):
    """作業ツリーの形（fk_ws/src/fk_sim の目印）を tmp_path の下に作る。"""
    top = os.path.join(str(tmp_path), "repo")
    os.makedirs(os.path.join(top, "fk_ws", "src", "fk_sim"))
    return top


def test_site_inside_repo_is_refused_and_nothing_written(tmp_path):
    data = dict(base_scene(), visibility="site")
    path = write_scene(tmp_path, data)
    top = _make_fake_tree(tmp_path)
    inside = os.path.join(top, "sim_out", "x")
    with pytest.raises(guard.GuardError):
        scene.build_scene(path, out_dir=inside)
    assert not os.path.exists(inside), "止めたら何も書いてはいけない"


def test_site_refused_from_outside_cwd(tmp_path, monkeypatch):
    """cwd がリポジトリの外でも、出力先が中なら止まる（cwd に頼らない）。"""
    top = _make_fake_tree(tmp_path)
    data = dict(base_scene(), visibility="site")
    path = write_scene(tmp_path, data)
    monkeypatch.chdir(tmp_path)  # リポジトリの外の cwd
    inside = os.path.join(top, "sim_out", "scenes", "leak")
    with pytest.raises(guard.GuardError):
        scene.build_scene(path, out_dir=inside)
    assert not os.path.exists(inside)


def test_site_refused_deep_nonexistent_path(tmp_path):
    """出力先がまだ無い深いパスでも、直下が見つかれば止まる。"""
    top = _make_fake_tree(tmp_path)
    data = dict(base_scene(), visibility="site")
    path = write_scene(tmp_path, data)
    deep = os.path.join(top, "sim_out", "scenes", "deep", "deeper", "leak2")
    with pytest.raises(guard.GuardError):
        scene.build_scene(path, out_dir=deep)
    assert not os.path.exists(deep)


def test_site_outside_repo_is_written(tmp_path):
    data = dict(base_scene(), visibility="site")
    path = write_scene(tmp_path, data)
    out = os.path.join(str(tmp_path), "outside")
    a = scene.build_scene(path, out_dir=out)
    assert os.path.isfile(os.path.join(a[0], "world.world"))
    assert os.path.isfile(os.path.join(a[0], "scene.json"))


def test_guard_allows_public_inside_repo(tmp_path):
    top = _make_fake_tree(tmp_path)
    scene_data = model.validate_scene(base_scene())
    assert guard.check_output(scene_data, os.path.join(top, "sim_out"))


# --- リポジトリの中の記述はすべて public ---

def test_scene_files_are_all_public():
    files = []
    for dirpath, _, filenames in os.walk(SCENES_DIR):
        files.extend(os.path.join(dirpath, f) for f in filenames if f.endswith(".yaml"))
    assert files, "scenes/ に記述がありません"
    for f in files:
        data = yaml.safe_load(open(f, encoding="utf-8"))
        assert data["visibility"] == "public", f"{f} が public ではありません"


# --- 記述の誤りで分かる言葉の例外になること ---

def test_overlapping_panels_raise():
    data = base_scene()
    data["panels"].append({"center": [4.3, 5.0], "front_yaw": 0.7, "width": 1.0, "depth": 0.6})
    with pytest.raises(model.SceneError, match="重な"):
        model.validate_scene(data)


def test_panel_outside_walls_raises():
    data = base_scene()
    data["panels"][0]["center"] = [0.2, 5.0]
    with pytest.raises(model.SceneError, match="壁の外"):
        model.validate_scene(data)


def test_missing_field_raises():
    data = base_scene()
    del data["panels"][0]["width"]
    with pytest.raises(model.SceneError, match="width"):
        model.validate_scene(data)


def test_bad_visibility_raises():
    data = dict(base_scene(), visibility="secret")
    with pytest.raises(model.SceneError, match="visibility"):
        model.validate_scene(data)


def test_public_room_builds_without_blender(tmp_path, monkeypatch):
    monkeypatch.delenv("BLENDER", raising=False)
    out, blender_time = scene.build_scene(PUBLIC_ROOM, out_dir=os.path.join(str(tmp_path), "o"))
    assert os.path.isfile(os.path.join(out, "world.world"))
    assert os.path.isfile(os.path.join(out, "scene.json"))
    assert blender_time is None  # BLENDER が無いホストでは .glb は作らない
    answer = json.load(open(os.path.join(out, "scene.json"), encoding="utf-8"))
    assert len(answer["panels"]) == 4


# --- Blender の外接箱と scene.json の比べ（compare_blender_objects） ---

def test_compare_blender_objects_ok_and_mismatch():
    scene_data = model.validate_scene(base_scene())
    answer = build_answer(scene_data)
    blender_objects = {}
    for p in answer["panels"]:
        corners = list(p["top_corners"].values())
        xs, ys = [c[0] for c in corners], [c[1] for c in corners]
        blender_objects[f"panel_{p['id']}_body"] = {
            "min": [min(xs), min(ys), model.HB], "max": [max(xs), max(ys), p["height"] - model.ROOF_T]}
        blender_objects[f"panel_{p['id']}_roof"] = {
            "min": [min(xs), min(ys), p["height"] - model.ROOF_T], "max": [max(xs), max(ys), p["height"]]}
        for corner, lug in p["lugs"].items():
            x, y = lug["base"][0], lug["base"][1]
            blender_objects[f"lug_{p['id']}_{corner}"] = {
                "min": [x - 0.01, y - 0.01, lug["base"][2]],
                "max": [x + 0.01, y + 0.01, lug["base"][2] + 0.05]}
    ok, _ = compare_blender_objects(answer, blender_objects)
    assert ok
    # 金具を 1 cm 超ずらしてみる
    blender_objects["lug_P001_FL"]["min"][0] += 0.05
    blender_objects["lug_P001_FL"]["max"][0] += 0.05
    ok, problems = compare_blender_objects(answer, blender_objects)
    assert not ok and any("金具" in p for p in problems)


# --- Blender があるときだけ scene.glb が scene.json と合うこと ---

@pytest.mark.skipif(os.environ.get("BLENDER") is None
                    or not os.path.isfile(os.environ.get("BLENDER", "")),
                    reason="BLENDER が無いので .glb の比べは飛ばす")
def test_blender_glb_matches_answer(tmp_path):
    out = os.path.join(str(tmp_path), "blender_out")
    scene.build_scene(PUBLIC_ROOM, out_dir=out)
    assert os.path.isfile(os.path.join(out, "scene.glb"))
    with open(os.path.join(out, "scene.json"), encoding="utf-8") as f:
        answer = json.load(f)
    with open(os.path.join(out, "blender_objects.json"), encoding="utf-8") as f:
        blender_objects = json.load(f)
    ok, problems = compare_blender_objects(answer, blender_objects)
    assert ok, "\n".join(problems)


@pytest.mark.skipif(os.environ.get("BLENDER") is None
                    or not os.path.isfile(os.environ.get("BLENDER", "")),
                    reason="BLENDER が無いのでリングの向きの確かめは飛ばす")
def test_blender_lug_ring_faces_panel_center(tmp_path):
    """金具のリング面の水平方向（局所 x）は盤の中心を向くこと。

    ss は金具 2 個の組から盤が広がる側をリング面の水平の向き（盤の中心を向く）で
    決める（ss `assemble._pair_center`）。試作の `gen_scene.py` 675 行目（9265d2d）
    の `ring_yaw = atan2(-y, -x)` と同じ決まり。90° ずらすとssは手前の列を
    手前へ広げてしまい、盤に組めなくなる（WP-SIM-04 の実測）。
    """
    out = os.path.join(str(tmp_path), "blender_out")
    scene.build_scene(PUBLIC_ROOM, out_dir=out)
    with open(os.path.join(out, "scene.json"), encoding="utf-8") as f:
        answer = json.load(f)
    with open(os.path.join(out, "blender_objects.json"), encoding="utf-8") as f:
        blender_objects = json.load(f)
    for p in answer["panels"]:
        cx, cy = p["center"]
        for corner, lug in p["lugs"].items():
            x, y = lug["base"][0], lug["base"][1]
            obj = blender_objects[f"lug_{p['id']}_{corner}"]
            yaw = obj["rotation_euler"][2]
            ring_x = (math.cos(yaw), math.sin(yaw))
            to_center = (cx - x, cy - y)
            norm = math.hypot(*to_center)
            dot = (ring_x[0] * to_center[0] + ring_x[1] * to_center[1]) / norm
            assert dot > math.cos(math.radians(5.0)), (
                f"{p['id']} {corner}: リングの向き {math.degrees(yaw):.1f}° が"
                f"盤の中心への向き {math.degrees(math.atan2(cy - y, cx - x)):.1f}° と平行でない")


def test_cli_site_refused_from_outside_cwd(tmp_path):
    """リポジトリの外を cwd にして CLI で site を中の出力先へ書こうとすると止まる。"""
    top = _make_fake_tree(tmp_path)
    data = dict(base_scene(), visibility="site")
    path = write_scene(tmp_path, data)
    out = os.path.join(top, "sim_out", "scenes", "leak2")
    proc = subprocess.run(
        [sys.executable, "-m", "fk_sim.scene", "build", path, "--out", out],
        capture_output=True, text=True, cwd=str(tmp_path),
        env={**os.environ, "PYTHONPATH": ROOT + os.pathsep + os.environ.get("PYTHONPATH", ""),
             "BLENDER": ""})
    assert proc.returncode != 0, (proc.stdout, proc.stderr)
    assert not os.path.exists(out)


def test_cli_build_public_room(tmp_path):
    out = os.path.join(str(tmp_path), "cli_out")
    proc = subprocess.run(
        [sys.executable, "-m", "fk_sim.scene", "build", PUBLIC_ROOM, "--out", out],
        capture_output=True, text=True, env={**os.environ, "PYTHONPATH": ROOT + os.pathsep + os.environ.get("PYTHONPATH", "")})
    assert proc.returncode == 0, proc.stderr
    assert os.path.isfile(os.path.join(out, "scene.json"))
