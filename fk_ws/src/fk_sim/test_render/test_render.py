"""描画の芯（`fk_sim.render`）の試験。コンテナ（`fk-sim`）の中で回す。ROS 2 なし。

既存の幾何の試験（光軸上・光軸外の物。ss の `geometry.project` と比べる）は、
描画と正解の両方が `world_to_camera_T` を通るため、`world_to_camera_T` 自体の
誤りを捕まえない。そこで「ワールド座標で独立に決めた期待値」を絵の中で直接
確かめる試験（機体の向き・高さ・ロール・呼び出し側の取り付けの値）を足した。
"""

import io
import math
import os
import sys

import numpy as np
import pytest

pyrender = pytest.importorskip("pyrender")
trimesh = pytest.importorskip("trimesh")
pytest.importorskip("PIL")
pytest.importorskip("yaml")

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fk_sim.render.core import Renderer, load_ss_camera, world_to_camera_T  # noqa: E402


def _geometry():
    try:
        from fk_sim.render import core
    except Exception:  # pragma: no cover
        pytest.skip("fk_sim.render を読めない")
    try:
        return core._load_ss_geometry()
    except Exception:
        pytest.skip("ss-system（SS_REPO）が読めない")


@pytest.fixture(scope="module")
def renderer_and_box(tmp_path_factory):
    """色の付いた小さな箱だけのシーンを作り、Renderer を 1 つだけ用意する。"""
    geometry = _geometry()
    d = str(tmp_path_factory.mktemp("scene"))
    box_size = 0.2
    # メリットの外: 既知の姿勢でカメラの光軸上に箱の中心を置く
    pose = dict(x=1.0, y=2.0, yaw=0.4, height_m=1.3, cam_yaw_deg=25.0, cam_pitch_deg=-15.0)
    T_wc = world_to_camera_T(pose["x"], pose["y"], pose["yaw"], pose["height_m"],
                             pose["cam_yaw_deg"], pose["cam_pitch_deg"])
    D = 2.0
    center_world = (np.linalg.inv(T_wc) @ np.array([0.0, 0.0, D, 1.0]))[:3]
    from trimesh.visual.material import PBRMaterial

    mesh = trimesh.creation.box(extents=[box_size] * 3)
    mesh.visual = trimesh.visual.TextureVisuals(
        uv=np.zeros((len(mesh.vertices), 2)),
        material=PBRMaterial(baseColorFactor=[1.0, 0.0, 0.0, 1.0]))
    mesh.apply_translation(center_world)
    mesh2 = trimesh.creation.icosphere(subdivisions=3, radius=0.08)
    mesh2.visual = trimesh.visual.TextureVisuals(
        uv=np.zeros((len(mesh2.vertices), 2)),
        material=PBRMaterial(baseColorFactor=[0.0, 0.0, 1.0, 1.0]))
    off_world = (np.linalg.inv(T_wc) @ np.array([0.62, 0.34, D, 1.0]))[:3]
    mesh2.apply_translation(off_world)
    scene = trimesh.Scene([mesh, mesh2])
    scene.export(os.path.join(d, "scene.glb"))
    with open(os.path.join(d, "scene.json"), "w", encoding="utf-8") as f:
        f.write("{}")
    r = Renderer(d)
    yield r, T_wc, center_world, pose, off_world
    r.close()


def _nonbg(jpg: bytes):
    from PIL import Image

    img = np.asarray(Image.open(io.BytesIO(jpg)).convert("RGB")).astype(np.int32)
    bg = np.array([0.8, 0.85, 0.9]) * 255.0
    return img, np.abs(img - bg).sum(axis=2) > 30


def _red_mask(jpg: bytes):
    img, nonbg = _nonbg(jpg)
    # 赤い箱の画素（赤が青より大きいもの）
    return nonbg & (img[..., 0] > img[..., 2])


def test_geometry_on_axis(renderer_and_box):
    r, T_wc, center_world, pose, _off = renderer_and_box
    geometry = _geometry()
    cam = r._camera
    jpg = r.render(pose["x"], pose["y"], pose["yaw"], pose["height_m"],
                   pose["cam_yaw_deg"], pose["cam_pitch_deg"], seed=3)
    uv, ok = geometry.project(center_world.reshape(1, 3), cam["K"], cam["dist"], T_wc, w=1280, h=720)
    assert bool(ok[0])
    mask = _red_mask(jpg)
    assert mask.sum() > 100
    ys, xs = np.nonzero(mask)
    centroid = np.array([xs.mean(), ys.mean()])
    err = float(np.abs(centroid - uv[0]).max())
    assert err < 4.0, f"像の中心 {centroid} と投影 {uv[0]} の誤差 {err:.1f} px"


def test_geometry_off_axis(renderer_and_box):
    """光軸外の箱（歪み・ロール・ヨーの符号の誤りを捕まえる）。"""
    from PIL import Image

    r, T_wc, _center, pose, off_world = renderer_and_box
    geometry = _geometry()
    cam = r._camera
    jpg = r.render(pose["x"], pose["y"], pose["yaw"], pose["height_m"],
                   pose["cam_yaw_deg"], pose["cam_pitch_deg"], seed=3)
    uv, ok = geometry.project(off_world.reshape(1, 3), cam["K"], cam["dist"], T_wc, w=1280, h=720)
    assert bool(ok[0])
    img, nonbg = _nonbg(jpg)
    mask = nonbg & (img[..., 2] > img[..., 0])
    assert mask.sum() > 100
    ys, xs = np.nonzero(mask)
    centroid = np.array([xs.mean(), ys.mean()])
    err = float(np.abs(centroid - uv[0]).max())
    # 球の陰の境界などマスクの非対称で数 px 出るため 12 px とする
    # （ロール・ヨー・ピッチの符号の誤りは数十 px ずれるので拾える）
    assert err < 12.0, f"像の中心 {centroid} と投影 {uv[0]} の誤差 {err:.1f} px"


def test_undistort_roundtrip(renderer_and_box):
    """歪みの写像が ss の `project` と往復する（歪みの向きを縛る）。"""
    import numpy as _np

    from fk_sim.render import core as _core

    r, _T, _c, _p, _off = renderer_and_box
    geometry = _geometry()
    cam = r._camera
    K, dist = cam["K"], cam["dist"]
    fold = geometry.fold_radius(dist)
    u0, v0, _ok = _core._undistort_map(K, dist, 1280, 720, fold)
    errs = []
    for v in range(0, 720, 45):
        for u in range(0, 1280, 45):
            # 歪みなしの光線をカメラ座標に直して ss の project で戻す
            xc = np.array([[ (u0[v, u] - K[0, 2]) / K[0, 0],
                             (v0[v, u] - K[1, 2]) / K[1, 1], 1.0]])
            uv, ok = geometry.project(xc, K, dist, _np.eye(4))
            if bool(ok[0]):
                errs.append(float(_np.abs(uv[0] - [u, v]).max()))
    assert max(errs) < 0.5, f"歪みの往復の誤差 {max(errs):.3f} px"


def test_jpeg_size(renderer_and_box):
    from PIL import Image

    r, _T, _c, pose, _off = renderer_and_box
    jpg = r.render(pose["x"], pose["y"], pose["yaw"], pose["height_m"],
                   pose["cam_yaw_deg"], pose["cam_pitch_deg"], seed=1)
    img = Image.open(io.BytesIO(jpg))
    assert img.size == (1280, 720)
    assert img.format == "JPEG"


def test_cache_same_input(renderer_and_box):
    r, _T, _c, pose, _off = renderer_and_box
    a = r.render(pose["x"], pose["y"], pose["yaw"], pose["height_m"],
                 pose["cam_yaw_deg"], pose["cam_pitch_deg"], seed=7)
    n = r.render_count
    b = r.render(pose["x"], pose["y"], pose["yaw"], pose["height_m"],
                 pose["cam_yaw_deg"], pose["cam_pitch_deg"], seed=7)
    assert a == b
    assert r.render_count == n


def _rss_kb():
    with open("/proc/self/status", encoding="ascii") as f:
        for line in f:
            if line.startswith("VmRSS:"):
                return int(line.split()[1])
    return 0


def test_rss_does_not_grow_per_render(renderer_and_box):
    """同じ Renderer で違う姿勢を数十枚描いても、RSS が 1 枚あたり線形に増えないこと。

    pyrender の OffscreenRenderer を消さずに使い回すと 1 枚あたり約 127 MiB を
    保持し続けてコンテナが OOM で落ちた（WP-SIM-04 の実測）。
    """
    r, _T, _c, pose, _off = renderer_and_box
    # 暖本体の分を先に描く（初回は文脈の確保で増える）
    for i in range(3):
        r.render(pose["x"], pose["y"], pose["yaw"], pose["height_m"],
                 pose["cam_yaw_deg"] + float(i), pose["cam_pitch_deg"], seed=20 + i)
    import gc
    gc.collect()
    base = _rss_kb()
    for i in range(30):
        r.render(pose["x"], pose["y"], pose["yaw"], pose["height_m"],
                 pose["cam_yaw_deg"] + 3.0 + float(i) * 2.0, pose["cam_pitch_deg"], seed=100 + i)
    gc.collect()
    growth_mib = (_rss_kb() - base) / 1024.0
    # この方法では 30 枚で 100 MiB 程度増える（実シーンでは数 GiB）。直っている場合は平坦。
    assert growth_mib < 50.0, f"30 枚で RSS が {growth_mib:.0f} MiB 増えた（線形の増え方）"


def test_different_pose_changes(renderer_and_box):
    r, _T, _c, pose, _off = renderer_and_box
    before = r.render(pose["x"], pose["y"], pose["yaw"], pose["height_m"],
                      pose["cam_yaw_deg"], pose["cam_pitch_deg"], seed=7)
    after = r.render(pose["x"], pose["y"], pose["yaw"], pose["height_m"],
                     pose["cam_yaw_deg"] + 12.0, pose["cam_pitch_deg"], seed=7)
    assert before != after
    assert r.render_count >= 2


def test_hve_conversion_signs():
    """hve の `pitch_deg`（見上げが正）→ ss の `pitch_down_rad`（見下ろしが正）の変換。"""
    from fk_sim.render.core import hve_to_ss_yaw_pitch

    yaw, pitch = hve_to_ss_yaw_pitch(30.0, 10.0)
    assert yaw == pytest.approx(math.radians(30.0))
    assert pitch == pytest.approx(-math.radians(10.0))


# ---------------------------------------------------------------------------
# world_to_camera_T を通さずに期待値を決める試験
# （物のワールド座標を独立に与え、絵の中の位置で判定する）

ROBOT = dict(x=1.0, y=4.0, yaw=0.7)
HEADING = np.array([math.cos(ROBOT["yaw"]), math.sin(ROBOT["yaw"])])
LEFT = np.array([-math.sin(ROBOT["yaw"]), math.cos(ROBOT["yaw"])])
RIGHT = -LEFT
DIST = 2.5
LIFT = 1.3


def _zero_camera():
    """取り付けのずれをすべて 0 にしたカメラ（K・dist は ss の locator.yaml の値）。"""
    cam = load_ss_camera()
    for k in ("mount_x_m", "mount_y_m", "forward_offset_m", "optical_offset_m",
              "height_offset_m", "pitch_offset_rad", "yaw_offset_rad", "roll_rad"):
        cam[k] = 0.0
    return cam


def _make_scene(tmp_path, objects):
    """objects: [(ワールド座標 (x,y,z), (r,g,b))] の小さな球だけのシーンを作る。"""
    from trimesh.visual.material import PBRMaterial

    d = str(tmp_path / "scene")
    os.makedirs(d, exist_ok=True)
    meshes = []
    for (x, y, z), color in objects:
        m = trimesh.creation.icosphere(subdivisions=2, radius=0.08)
        m.visual = trimesh.visual.TextureVisuals(
            uv=np.zeros((len(m.vertices), 2)),
            material=PBRMaterial(baseColorFactor=[c / 255.0 for c in color] + [1.0]))
        m.apply_translation([x, y, z])
        meshes.append(m)
    trimesh.Scene(meshes).export(os.path.join(d, "scene.glb"))
    with open(os.path.join(d, "scene.json"), "w", encoding="utf-8") as f:
        f.write("{}")
    return d


def _mask_centroid(jpg: bytes):
    """物の画素全体の重心 (u, v) と画素数を返す（背景は FK-E1 と同じ色）。"""
    from PIL import Image

    img = np.asarray(Image.open(io.BytesIO(jpg)).convert("RGB")).astype(np.int32)
    bg = np.array([0.8, 0.85, 0.9]) * 255.0
    mask = np.abs(img - bg).sum(axis=2) > 30
    if mask.sum() == 0:
        return None, 0
    ys, xs = np.nonzero(mask)
    return np.array([xs.mean(), ys.mean()]), int(mask.sum())


def _render(scene_dir, camera, **pose):
    r = Renderer(scene_dir, camera=camera)
    try:
        return r.render(pose["x"], pose["y"], pose["yaw"], pose["height_m"],
                        pose.get("cam_yaw_deg", 0.0), pose.get("cam_pitch_deg", 0.0),
                        seed=1)
    finally:
        r.close()


def test_robot_heading_front_object_is_centered(tmp_path):
    """機体の向きの合成: 真正面の物は中心に、真横の物は写らない。

    期待値はワールド座標で独立に決めた（`world_to_camera_T` を通さない）。
    機体の向きの合成（ワールド → base_link の回転）の符号の誤りは、正面の物が
    中心から大きく外れることで赤になる。
    """
    cam = _zero_camera()
    base = np.array([ROBOT["x"], ROBOT["y"]])
    front = base + DIST * HEADING
    side = base + DIST * LEFT
    d = _make_scene(tmp_path, [((front[0], front[1], LIFT), (220, 30, 30)),
                               ((side[0], side[1], LIFT), (30, 220, 30))])
    jpg = _render(d, cam, **ROBOT, height_m=LIFT)
    centroid, count = _mask_centroid(jpg)
    assert count > 100
    K = cam["K"]
    err = float(np.abs(centroid - [K[0, 2], K[1, 2]]).max())
    assert err < 6.0, f"正面の物の像の中心 {centroid} が画像中心から {err:.1f} px"

    # 真横（左）の物だけの絵では写らない
    d2 = _make_scene(tmp_path / "side", [((side[0], side[1], LIFT), (30, 220, 30))])
    jpg2 = _render(d2, cam, **ROBOT, height_m=LIFT)
    _, count2 = _mask_centroid(jpg2)
    assert count2 == 0, "真横の物が写っている（機体の向きの合成の誤り）"


def test_camera_height_object_position(tmp_path):
    """高さ: 物がカメラの高さなら縦の中心、0.3 m 上なら fy·0.3/D だけ上。

    期待値はワールド座標と K から独立に決めた。取り付けの高さの差
    （昇降機の高さへの `camera_height_offset_m` の加算）の誤りや、
    カメラの高さの誤りは、縦の位置のずれで赤になる。
    """
    cam = _zero_camera()
    base = np.array([ROBOT["x"], ROBOT["y"]])
    front = base + DIST * HEADING
    # ずれ 0 のカメラの高さ = 昇降機の高さなので、物の高さも LIFT に置く
    d = _make_scene(tmp_path, [((front[0], front[1], LIFT), (220, 30, 30))])
    K = cam["K"]
    cx, cy, fy = K[0, 2], K[1, 2], K[1, 1]

    centroid, _ = _mask_centroid(_render(d, cam, **ROBOT, height_m=LIFT))
    assert centroid is not None
    assert abs(centroid[1] - cy) < 6.0, f"カメラの高さの物の縦位置 {centroid[1]:.1f}"

    # 機体が 0.3 m 低い（カメラが 0.3 m 下）→ 物は fy·0.3/D だけ上に出る
    centroid2, _ = _mask_centroid(_render(d, cam, **ROBOT, height_m=LIFT - 0.3))
    assert centroid2 is not None
    expected = cy - fy * 0.3 / DIST
    assert abs(centroid2[1] - expected) < 8.0, \
        f"0.3 m 上の物の縦位置 {centroid2[1]:.1f}（期待 {expected:.1f}）"


def test_roll_tilt_direction(tmp_path):
    """ロール: 正のロールで右の物が上、左の物が下に写る。

    ss の `camera_T` の docstring（「正 = カメラの右が下へ回る」）から決めた:
    カメラの右の軸が下を向くので、同じ高さの左右 2 物は右が上で左が下になる
    （右の物の像が上）。ロールを 0 にする誤りや符号の誤りは、傾きの無さ・逆転で
    赤になる。
    """
    cam = _zero_camera()
    phi_deg = 5.0
    cam["roll_rad"] = math.radians(phi_deg)
    base = np.array([ROBOT["x"], ROBOT["y"]])
    mid = base + DIST * HEADING
    right_obj = mid + 0.5 * RIGHT
    left_obj = mid + 0.5 * LEFT
    d = _make_scene(tmp_path, [((right_obj[0], right_obj[1], LIFT), (220, 30, 30)),
                               ((left_obj[0], left_obj[1], LIFT), (30, 30, 220))])
    jpg = _render(d, cam, **ROBOT, height_m=LIFT)
    from PIL import Image

    img = np.asarray(Image.open(io.BytesIO(jpg)).convert("RGB")).astype(np.int32)
    bg = np.array([0.8, 0.85, 0.9]) * 255.0
    nonbg = np.abs(img - bg).sum(axis=2) > 30
    right_mask = nonbg & (img[..., 0] > img[..., 2])   # 赤 = 右の物
    left_mask = nonbg & (img[..., 2] > img[..., 0])    # 青 = 左の物
    assert right_mask.sum() > 100 and left_mask.sum() > 100
    ry, rx = np.nonzero(right_mask)
    ly, lx = np.nonzero(left_mask)
    # 右の物は左の物より上に写る（v が小さい）
    assert ry.mean() < ly.mean() - 20, \
        f"右の物の縦位置 {ry.mean():.1f} が左の物 {ly.mean():.1f} より下（ロール符号）"
    assert rx.mean() > lx.mean(), "左右の物が入れ替わっている"


def test_default_camera_offsets_reach_rendering(tmp_path):
    """呼び出し側: 既定のカメラの取り付けのずれが描画に届いている。

    `camera_yaw_offset_deg`（-1.0°。左が正なので右へ 1°）の分だけ左へずれる:
    ずれ 0 のカメラでは中心に写る物が、既定のカメラでは cx - fx·tan(1°) 付近に
    写る。また、物の高さを「昇降機の高さ + camera_height_offset_m」にすると
    縦も中心に写る（高さの差を足すことを縛る）。
    """
    zero = _zero_camera()
    default = load_ss_camera()
    base = np.array([ROBOT["x"], ROBOT["y"]])
    front = base + DIST * HEADING
    K = default["K"]
    cx, cy, fx = K[0, 2], K[1, 2], K[0, 0]

    # ずれ 0 のカメラ: 中心に写る（基準）
    d0 = _make_scene(tmp_path, [((front[0], front[1], LIFT), (220, 30, 30))])
    c0, _ = _mask_centroid(_render(d0, zero, **ROBOT, height_m=LIFT))
    assert c0 is not None and abs(c0[0] - cx) < 6.0

    # 既定のカメラ: 物は昇降機の高さ + camera_height_offset_m の高さに置く
    obj_z = LIFT + default["height_offset_m"]
    d1 = _make_scene(tmp_path / "default", [((front[0], front[1], obj_z), (220, 30, 30))])
    c1, _ = _mask_centroid(_render(d1, default, **ROBOT, height_m=LIFT))
    assert c1 is not None, "既定のカメラで物が写らない（高さの差の誤り）"
    # 横は cx - fx·tan(1.0°) 付近（yaw offset で右へ向くので物は左へ）
    expected_u = cx - fx * math.tan(math.radians(1.0))
    assert abs(c1[0] - expected_u) < 8.0, \
        f"既定カメラの横位置 {c1[0]:.1f}（期待 {expected_u:.1f}、yaw offset）"
    # 縦は中心付近（物の高さ = カメラの高さ。pitch offset 0.3° で数 px 動く）
    assert abs(c1[1] - cy) < 20.0, f"既定カメラの縦位置 {c1[1]:.1f}（高さの差の誤り）"
