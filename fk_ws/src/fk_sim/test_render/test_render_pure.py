"""描画の芯の純関数（光源の姿勢・ぼかし）の試験。pyrender なしでホストでも回る。"""
import numpy as np
import pytest

from fk_sim.render import core


@pytest.mark.parametrize("direction", [(0.15, 0.3, -1.0), (-0.5, 0.6, -0.4), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)])
def test_directional_light_pose_points_along_direction(direction):
    pose = core.directional_light_pose(direction)
    R = pose[:3, :3]
    # pyrender の方向光源は局所 -Z へ進む
    d = np.asarray(direction, float) / np.linalg.norm(direction)
    assert np.allclose(-R[:, 2], d)
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-9)
    assert np.linalg.det(R) == pytest.approx(1.0)
    assert np.allclose(pose[:3, 3], 0.0) and np.allclose(pose[3], [0, 0, 0, 1])


def test_key_light_comes_from_above():
    # 主光源は上から下へ（天面が明るく、正面が暗くなる向き）
    d = np.asarray(core.KEY_DIRECTION, float)
    d /= np.linalg.norm(d)
    assert d[2] < -0.9


def test_gaussian_blur_zero_sigma_is_identity():
    img = np.random.default_rng(0).uniform(0, 255, (20, 30, 3))
    assert core.gaussian_blur(img, 0.0) is img


def test_gaussian_blur_keeps_flat_image_and_spreads_impulse():
    flat = np.full((15, 17, 3), 123.0)
    assert np.allclose(core.gaussian_blur(flat, 0.8), 123.0)
    imp = np.zeros((31, 31, 1))
    imp[15, 15, 0] = 1000.0
    out = core.gaussian_blur(imp, 1.5)[..., 0]
    assert out.sum() == pytest.approx(1000.0, rel=1e-6)
    assert out[15, 15] < 1000.0 * 0.2           # 中心から広がる
    assert out[15, 16] == pytest.approx(out[16, 15])  # 縦横で同じ
    # 広がりの標準偏差が sigma に近い（行の周辺分布で見る）
    row = out.sum(axis=0)
    x = np.arange(31) - 15
    std = np.sqrt((row * x ** 2).sum() / row.sum())
    assert std == pytest.approx(1.5, rel=0.05)
