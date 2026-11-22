"""描画の芯（`fk_sim.render`）の試験。コンテナ（`fk-sim`）の中で回す。ROS 2 なし。

幾何の試験は、小さな既知の箱を描き、その像の中心を ss の `geometry.project`
（同じ `K`・`dist`・`T`。`SS_REPO` から import する。写さない）と比べる。
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

from fk_sim.render.core import Renderer, world_to_camera_T  # noqa: E402


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
