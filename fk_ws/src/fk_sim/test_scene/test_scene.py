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
from fk_sim.scene import guard, model, start_pose
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


def fake_configs(tmp_path):
    """ss の locator.yaml と th の nav2 の代わり（数字だけ置いた最小のもの）。"""
    ss_repo = os.path.join(str(tmp_path), "ss")
    cfg = os.path.join(ss_repo, "ss_ws", "src", "ss_locator", "config")
    os.makedirs(cfg)
    with open(os.path.join(cfg, "locator.yaml"), "w", encoding="utf-8") as f:
        yaml.safe_dump({"locator_node": {"ros__parameters": {
            "max_distance_default_m": 5.0, "scan_camera_height_m": 3.1}}}, f)
    nav2 = os.path.join(str(tmp_path), "nav2.yaml")
    with open(nav2, "w", encoding="utf-8") as f:
        yaml.safe_dump({"bt_navigator": {"ros__parameters": {"robot_radius": 0.36}}}, f)
    return ss_repo, nav2


def test_site_outside_repo_is_written(tmp_path):
    data = dict(base_scene(), visibility="site")
    path = write_scene(tmp_path, data)
    ss_repo, nav2 = fake_configs(tmp_path)
    out = os.path.join(str(tmp_path), "outside")
    a = scene.build_scene(path, out_dir=out, ss_repo=ss_repo, nav2_params=nav2)
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
    ss_repo, nav2 = fake_configs(tmp_path)
    out, blender_time = scene.build_scene(PUBLIC_ROOM, out_dir=os.path.join(str(tmp_path), "o"),
                                          ss_repo=ss_repo, nav2_params=nav2)
    assert os.path.isfile(os.path.join(out, "world.world"))
    assert os.path.isfile(os.path.join(out, "scene.json"))
    assert blender_time is None  # BLENDER が無いホストでは .glb は作らない
    answer = json.load(open(os.path.join(out, "scene.json"), encoding="utf-8"))
    assert len(answer["panels"]) == 4
    # public_room には robot が無い（WP-SIM-06）。生成器が選んで書く。
    assert set(answer["robot"]) == {"pose", "pose_source", "visible_panels", "hidden_panels"}


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
    ss_repo, nav2 = fake_configs(tmp_path)
    out = os.path.join(str(tmp_path), "blender_out")
    scene.build_scene(PUBLIC_ROOM, out_dir=out, ss_repo=ss_repo, nav2_params=nav2)
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
    ss_repo, nav2 = fake_configs(tmp_path)
    out = os.path.join(str(tmp_path), "blender_out")
    scene.build_scene(PUBLIC_ROOM, out_dir=out, ss_repo=ss_repo, nav2_params=nav2)
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
    ss_repo, nav2 = fake_configs(tmp_path)
    out = os.path.join(str(tmp_path), "cli_out")
    proc = subprocess.run(
        [sys.executable, "-m", "fk_sim.scene", "build", PUBLIC_ROOM, "--out", out],
        capture_output=True, text=True, env={**os.environ, "PYTHONPATH": ROOT + os.pathsep + os.environ.get("PYTHONPATH", ""),
                                            "SS_REPO": ss_repo, "FK_SIM_NAV2_PARAMS": nav2})
    assert proc.returncode == 0, proc.stderr
    assert os.path.isfile(os.path.join(out, "scene.json"))


# --- WP-SIM-06: 機体の置き場（start_pose。期待値は試験の中で直接計算する） ---

def pose_scene(panels, obstacles=None, robot=None, size=(10.0, 8.0)):
    data = {"visibility": "public", "name": "pose_room",
            "room": {"size_x": size[0], "size_y": size[1], "ceiling_h": 4.0},
            "panels": panels,
            "obstacles": obstacles or [],
            "lights": [{"type": "point", "position": [5.0, 4.0, 3.5], "energy": 1.0}]}
    if robot is not None:
        data["robot"] = robot
    return model.validate_scene(data)


def axis_panel(pid, cx, cy, height=2.35):
    """向きの無い盤（front_yaw=-π/2 なら θ=0。金具は (±0.43, ±0.43) の世界座標）。"""
    return {"id": pid, "center": [cx, cy], "front_yaw": -1.5707963267948966,
            "width": 1.0, "depth": 1.0, "height": height, "label": ""}


def test_robot_is_optional_in_description():
    data = base_scene()
    del data["robot"]
    assert model.validate_scene(data)["robot"] is None


def test_distance_boundary_just_inside_and_outside():
    scene_data = pose_scene([axis_panel("P001", 5.0, 5.0)])
    rx, ry = 5.0, 1.0
    # 金具は (5±0.43, 5±0.43)。いちばん遠い金具までの距離（独立計算）。
    farthest = math.hypot(0.43, 5.43 - ry)
    # 探す距離 - 余裕(0.3) がちょうど farthest → 内側として見える。
    visible, hidden = start_pose.visible_panels_at(
        scene_data, rx, ry, farthest + 0.3, 3.1)
    assert visible == ["P001"] and hidden == []
    # 0.02 m 短ければ外側で far。
    visible, hidden = start_pose.visible_panels_at(
        scene_data, rx, ry, farthest + 0.3 - 0.02, 3.1)
    assert visible == [] and hidden == [{"id": "P001", "reason": "far"}]


def test_tall_obstacle_blocks_low_cart_does_not():
    panel = axis_panel("P001", 5.0, 5.0)
    # 線分（カメラ (5,1,3.1) → 金具 z≈2.386）が障害物 (5,3) の上を横切る高さは
    # 途中で約 2.55〜2.75 m。高さ 2.9 m の棚は遮り、0.5 m の台車は通る。
    for height, expected in ((2.9, []), (0.5, ["P001"])):
        scene_data = pose_scene(
            [panel], obstacles=[{"center": [5.0, 3.0], "size": [1.0, 0.5, height], "yaw": 0.0}])
        visible, hidden = start_pose.visible_panels_at(scene_data, 5.0, 1.0, 5.0, 3.1)
        assert visible == expected, height
        if not expected:
            assert hidden == [{"id": "P001", "reason": "blocked"}]


def test_other_panel_blocks_own_panel_does_not():
    # 背の高い盤（3.0 m）が前の盤。後ろの盤の金具への線分は前盤の箱を切る。
    # 自分の盤の箱が線分に当たっても（金具は自盤の上）見えると数える。
    scene_data = pose_scene([
        axis_panel("P001", 5.0, 5.0),
        axis_panel("P002", 5.0, 3.5, height=3.0),
    ])
    visible, hidden = start_pose.visible_panels_at(scene_data, 5.0, 1.0, 5.0, 3.1)
    assert visible == ["P002"], (visible, hidden)
    assert hidden == [{"id": "P001", "reason": "blocked"}]


def test_chooses_grid_point_clear_of_walls_and_panels_and_is_deterministic():
    scene_data = pose_scene([axis_panel("P001", 5.0, 5.0)])
    first = start_pose.choose_start_pose(scene_data, 5.0, 3.1, 0.36)
    second = start_pose.choose_start_pose(scene_data, 5.0, 3.1, 0.36)
    assert first == second
    assert first["pose_source"] == "auto"
    assert first["visible_panels"] == ["P001"]
    x, y, yaw = first["pose"]
    # 壁の内側の面（wall_t/2=0.075）から半径（0.36+0.2=0.56）離れていること。
    assert 0.075 + 0.56 <= x <= 10.0 - 0.075 - 0.56
    assert 0.075 + 0.56 <= y <= 8.0 - 0.075 - 0.56
    # 盤の箱（x 4.5〜5.5, y 4.5〜5.5）に半径分当たっていないこと（独立計算）。
    dx = max(abs(x - 5.0) - 0.5, 0.0)
    dy = max(abs(y - 5.0) - 0.5, 0.0)
    assert math.hypot(dx, dy) >= 0.56
    # 向きは盤の中心（5, 5）へ（独立計算）。
    assert abs(yaw - math.atan2(5.0 - y, 5.0 - x)) < 1e-9
    # 見える盤 1 枚のときの置き場は、盤までの距離が短い所が勝つ。
    # (5, 3.75) は盤の中心まで 1.25 m で当たらない（下に計算）。
    assert math.hypot(5.0 - x, 5.0 - y) <= math.hypot(5.0 - 5.0, 5.0 - 3.75) + 1e-9


def test_tie_break_picks_nearest_max_distance_point():
    # 左右対称の 2 枚。両方見えて、見える盤までのいちばん遠い距離が短い所は
    # 中心軸の直下 (5, 4)（中心までの距離 sqrt(1^2+1^2)=1.414...）。
    scene_data = pose_scene([axis_panel("P001", 4.0, 5.0), axis_panel("P002", 6.0, 5.0)])
    robot = start_pose.choose_start_pose(scene_data, 5.0, 3.1, 0.36)
    assert robot["visible_panels"] == ["P001", "P002"]
    assert robot["pose"][:2] == [5.0, 4.0], robot["pose"]


def test_given_pose_is_used_unchanged_with_reasons():
    scene_data = pose_scene([axis_panel("P001", 5.0, 5.0)], robot={"pose": [6.0, 1.0, 0.5]})
    robot = start_pose.resolve_robot  # 参照だけ（設定は下の試験で確かめる）
    del robot
    given = start_pose.evaluate_given_pose(scene_data, [6.0, 1.0, 0.5], 5.0, 3.1)
    assert given["pose"] == [6.0, 1.0, 0.5]
    assert given["pose_source"] == "given"
    assert given["visible_panels"] == ["P001"]


def test_config_files_must_be_readable(tmp_path):
    with pytest.raises(start_pose.StartPoseError):
        start_pose.load_locator_params(os.path.join(str(tmp_path), "no_ss"))
    with pytest.raises(start_pose.StartPoseError):
        start_pose.load_robot_radius(os.path.join(str(tmp_path), "no_nav2.yaml"))
    ss_repo, nav2 = fake_configs(tmp_path)
    assert start_pose.load_locator_params(ss_repo) == (5.0, 3.1)
    assert start_pose.load_robot_radius(nav2) == 0.36
    # robot_radius の無い nav2 は止める。
    empty = os.path.join(str(tmp_path), "empty.yaml")
    with open(empty, "w", encoding="utf-8") as f:
        yaml.safe_dump({"bt_navigator": {"ros__parameters": {}}}, f)
    with pytest.raises(start_pose.StartPoseError):
        start_pose.load_robot_radius(empty)


def test_build_scene_auto_and_given_writes_robot_keys(tmp_path, monkeypatch):
    monkeypatch.delenv("BLENDER", raising=False)
    ss_repo, nav2 = fake_configs(tmp_path)
    # robot 無し → auto
    data = base_scene()
    del data["robot"]
    path = write_scene(tmp_path, data)
    out, _ = scene.build_scene(path, out_dir=os.path.join(str(tmp_path), "auto_o"),
                               ss_repo=ss_repo, nav2_params=nav2)
    answer = json.load(open(os.path.join(out, "scene.json"), encoding="utf-8"))
    assert answer["robot"]["pose_source"] == "auto"
    # robot មាន → given で置き場は発表どおりのまま
    path = write_scene(tmp_path, base_scene(), "given.yaml")
    out, _ = scene.build_scene(path, out_dir=os.path.join(str(tmp_path), "given_o"),
                               ss_repo=ss_repo, nav2_params=nav2)
    answer = json.load(open(os.path.join(out, "scene.json"), encoding="utf-8"))
    assert answer["robot"]["pose"] == [5.0, 1.0, 0.0]
    assert answer["robot"]["pose_source"] == "given"


def test_chosen_pose_clear_of_obstacle_by_radius():
    # 盤にいちばん近い良い所（格子順で最初に当たる (3.75, 5.0)）を台車が占める。
    # 障害物を無視するとそこが選ばれるが、選ばれた置き場は半径（0.36+0.2）だけ離れる。
    scene_data = pose_scene(
        [axis_panel("P001", 5.0, 5.0)],
        obstacles=[{"center": [3.75, 5.0], "size": [1.0, 1.0, 0.4], "yaw": 0.0}])
    robot = start_pose.choose_start_pose(scene_data, 5.0, 3.1, 0.36)
    assert robot["visible_panels"] == ["P001"]
    x, y = robot["pose"][:2]
    # 障害物の矩形（x 3.25〜4.25, y 4.5〜5.5）までの距離（独立計算。回転なし）。
    dx = max(abs(x - 3.75) - 0.5, 0.0)
    dy = max(abs(y - 5.0) - 0.5, 0.0)
    assert math.hypot(dx, dy) >= 0.56 - 1e-9, robot["pose"]


def test_chosen_pose_clear_of_wall_by_radius():
    # 盤を西の壁際に置く。壁に寄るほど盤に近くなるが、壁の内側の面
    # （x=0.075）から半径（0.36+0.2）だけ離れた所までが置ける。
    scene_data = pose_scene([axis_panel("P001", 1.6, 4.0)])
    robot = start_pose.choose_start_pose(scene_data, 5.0, 3.1, 0.36)
    assert robot["visible_panels"] == ["P001"]
    x = robot["pose"][0]
    assert x - 0.075 >= 0.56 - 1e-9, robot["pose"]


def test_own_panel_box_does_not_block_when_camera_below_top():
    # カメラ（2.0 m）が天面（2.35 m）より低いと、遠い金具への線分は
    # 自分の盤の箱の中を切る（独立の確認: z=2.35 を切る点は盤の矩形の中）。
    # それでも自分の盤は遮られない（見えると数える）。
    # z=2.35 を切る t = (2.35-2.0)/(2.3865-2.0) ≈ 0.905。遠い金具 (5.43, 5.43) への
    # 線分のその点は (5.39, 5.20) で盤の矩形（x 4.5〜5.5, y 4.5〜5.5）の中。
    t = (2.35 - 2.0) / (2.3865 - 2.0)
    cx = 5.0 + t * (5.43 - 5.0)
    cy = 3.0 + t * (5.43 - 3.0)
    assert 4.5 < cx < 5.5 and 4.5 < cy < 5.5
    scene_data = pose_scene([axis_panel("P001", 5.0, 5.0)])
    visible, hidden = start_pose.visible_panels_at(scene_data, 5.0, 3.0, 5.0, 2.0)
    assert visible == ["P001"] and hidden == [], (visible, hidden)
