"""描画の芯（`fk_sim.render`）の中身。純粋な描画のみ（ROS 2・hve の口は読まない）。

入力は ss / hve の約束と同じ意味:
  - 昇降機の高さは hve の `height_mm` / 1000（m）
  - カメラの向きは hve の `yaw_deg`・`pitch_deg`。変換は ss の `hve_camera_core` と同じ
    （`pitch_down_rad = -pitch_deg`。`yaw_deg` はそのまま）

カメラの外部パラメータは ss の `geometry.camera_T` で作り、ワールド → カメラにする。
描画は FK-E1（`tools/fk_e1_render/render_py.py`）と同じ光源の既定で
pyrender + OSMesa（CPU）で歪みなしを少し広めに描いてから `camera_K`・`camera_dist`
で歪ませ、最後に FK-E1 の後処理（ノイズと JPEG）を足す。
"""

from __future__ import annotations

import io
import json
import math
import os

os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")

import numpy as np

WIDTH = 1280
HEIGHT = 720
# 歪みなしを描く幅（歪ませたときの写像先が収まるよう少し広げる）
_RENDER_SCALE = 1.3
# 材質の既定（従来の値。material_factors が None のときに実際に使う値）。
# WP-SIM-11 の切り分け A はこの下敷きを Renderer の引数で上書きする。
DEFAULT_METALLIC = 0.0
DEFAULT_ROUGHNESS = 0.7


def material_factors(metallic_factor, roughness_factor):
    """材質へ入れる (metallic, roughness)。None は従来の既定（0.0・0.7）。"""
    metal = DEFAULT_METALLIC if metallic_factor is None else float(metallic_factor)
    rough = DEFAULT_ROUGHNESS if roughness_factor is None else float(roughness_factor)
    return metal, rough


def _ss_repo() -> str:
    return os.environ.get("SS_REPO", "../ss-system")


def _load_ss_geometry():
    import sys

    repo = _ss_repo()
    locator_dir = os.path.join(repo, "ss_ws", "src", "ss_locator")
    if locator_dir not in sys.path:
        sys.path.insert(0, locator_dir)
    from ss_locator import geometry  # ss-system（読み取り専用。写さない）

    return geometry


def load_ss_camera():
    """ss の `locator.yaml` からカメラの取り付け値と `camera_K`・`camera_dist` を読む。"""
    import yaml

    yaml_path = os.path.join(
        _ss_repo(), "ss_ws", "src", "ss_locator", "config", "locator.yaml"
    )
    with open(yaml_path, encoding="utf-8") as f:
        params = yaml.safe_load(f)["locator_node"]["ros__parameters"]

    def get(name):
        return params[name]

    return {
        "K": np.asarray(get("camera_K"), dtype=np.float64).reshape(3, 3),
        "dist": np.asarray(get("camera_dist"), dtype=np.float64).reshape(-1),
        "mount_x_m": float(get("camera_mount_x_m")),
        "mount_y_m": float(get("camera_mount_y_m")),
        "forward_offset_m": float(get("camera_forward_offset_m")),
        "optical_offset_m": float(get("camera_optical_offset_m")),
        "height_offset_m": float(get("camera_height_offset_m")),
        "pitch_offset_rad": math.radians(float(get("camera_pitch_offset_deg"))),
        "yaw_offset_rad": math.radians(float(get("camera_yaw_offset_deg"))),
        "roll_rad": math.radians(float(get("camera_roll_deg"))),
    }


def hve_to_ss_yaw_pitch(yaw_deg: float, pitch_deg: float) -> tuple[float, float]:
    """hve の `yaw_deg`・`pitch_deg` → ss の `yaw_rad`・`pitch_down_rad`（`hve_camera_core` と同じ）。"""
    return math.radians(float(yaw_deg)), -math.radians(float(pitch_deg))


def world_to_camera_T(x: float, y: float, robot_yaw: float,
                      lift_height_m: float, cam_yaw_deg: float, cam_pitch_deg: float,
                      camera=None) -> np.ndarray:
    """ワールド点 → OpenCV カメラ の外部パラメータ (4×4)。

    `camera_T` は `base_link` → カメラなので、機体の床の点 (x, y, 向き robot_yaw)
    の `base_link` への変換と合成する。
    """
    geometry = _load_ss_geometry()
    cam = camera or load_ss_camera()
    yaw_rad, pitch_down_rad = hve_to_ss_yaw_pitch(cam_yaw_deg, cam_pitch_deg)
    T_bc = geometry.camera_T(
        float(lift_height_m) + cam["height_offset_m"],
        yaw_rad,
        pitch_down_rad,
        cam["mount_x_m"],
        cam["mount_y_m"],
        cam["forward_offset_m"],
        optical_offset_m=cam["optical_offset_m"],
        pitch_offset_rad=cam["pitch_offset_rad"],
        yaw_offset_rad=cam["yaw_offset_rad"],
        roll_rad=cam["roll_rad"],
    )
    c, s = math.cos(float(robot_yaw)), math.sin(float(robot_yaw))
    # ワールド → base_link: 回転 Rz(-robot_yaw)、並進 -Rz(-robot_yaw) @ (x, y, 0)
    R_wb = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    T_wb = np.eye(4)
    T_wb[:3, :3] = R_wb
    T_wb[:3, 3] = -R_wb @ np.array([float(x), float(y), 0.0])
    return T_bc @ T_wb


def _opencv_to_opengl_camera_pose(T_wc: np.ndarray) -> np.ndarray:
    """ワールド → OpenCV カメラの (4×4) から pyrender 用のカメラの pose（カメラ → ワールド）にする。

    pyrender のカメラは OpenGL の向き（X 右・Y 上・手前が -Z）で、OpenCV の
    カメラ向き（X 右・Y 下・前が +Z）と Y・Z の符号が違う。
    """
    S = np.diag([1.0, -1.0, -1.0, 1.0])
    return np.linalg.inv(T_wc) @ S


def _undistort_map(K: np.ndarray, dist: np.ndarray, w: int, h: int,
                   fold_r: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """出力（歪みあり）の各画素が当たる歪みなしの画素位置を返す。

    戻り値は (u0, v0, ok)。`ok` は歪みの折り返しより内側で、歪みなし絵の中にあるもの。
    """
    geometry = _load_ss_geometry()
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    us = np.arange(w, dtype=np.float64)
    vs = np.arange(h, dtype=np.float64)
    xd, yd = np.meshgrid((us - cx) / fx, (vs - cy) / fy)
    x, y = xd.copy(), yd.copy()
    k1, k2 = float(dist[0]), float(dist[1])
    for _ in range(20):
        r2 = x * x + y * y
        f = 1.0 + k1 * r2 + k2 * r2 * r2
        f = np.where(np.abs(f) < 1e-12, 1e-12, f)
        new_x, new_y = xd / f, yd / f
        if np.max(np.abs(new_x - x)) < 1e-12 and np.max(np.abs(new_y - y)) < 1e-12:
            x, y = new_x, new_y
            break
        x, y = new_x, new_y
    r = np.sqrt(x * x + y * y)
    ok = r < fold_r
    return fx * x + cx, fy * y + cy, ok


def _bilinear(img: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    h, w = img.shape[:2]
    u = np.clip(u, 0.0, w - 1.001)
    v = np.clip(v, 0.0, h - 1.001)
    u0 = np.floor(u).astype(np.int64)
    v0 = np.floor(v).astype(np.int64)
    du = (u - u0)[..., None]
    dv = (v - v0)[..., None]
    imgf = img.astype(np.float64)
    top = imgf[v0, u0] * (1 - du) + imgf[v0, u0 + 1] * du
    bottom = imgf[v0 + 1, u0] * (1 - du) + imgf[v0 + 1, u0 + 1] * du
    return top * (1 - dv) + bottom * dv


def _post_noise_jpeg(img_rgb: np.ndarray, seed: int) -> bytes:
    """FK-E1（`post_noise_jpeg.py`）と同じ後処理: ノイズと JPEG 圧縮。

    乱数の種は引数で与える（FK-E1 と同じノイズの組み立て: 画素ごとの
    `noise_sigma` と輝度ノイズ `0.7 * noise_sigma`、JPEG 品質 82〜92）。
    """
    from PIL import Image

    img = np.asarray(img_rgb, dtype=np.float32) / 255.0
    h, w = img.shape[:2]
    rng = np.random.default_rng(seed)
    sigma = float(rng.uniform(0.002, 0.012))
    quality = int(rng.integers(82, 93))
    out = img + rng.normal(0.0, sigma, img.shape).astype(np.float32)
    out += rng.normal(0.0, sigma * 0.7, (h, w, 1)).astype(np.float32)
    out = np.clip(out, 0.0, 1.0)
    buf = io.BytesIO()
    Image.fromarray(np.round(out * 255.0).astype(np.uint8)).save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


class Renderer:
    """シーンの `.glb` を一度だけ読み込み、姿勢ごとに JPEG を返す。"""

    def __init__(self, scene_dir: str, camera=None,
                 dir_intensity: float = 0.6, ambient: float = 0.25,
                 point_intensity: float = 6.0,
                 metallic_factor: float | None = None,
                 roughness_factor: float | None = None):
        import pyrender
        import trimesh

        self._scene_dir = str(scene_dir)
        self._camera = camera or load_ss_camera()
        self._geometry = _load_ss_geometry()

        glb = os.path.join(self._scene_dir, "scene.glb")
        tm = trimesh.load(glb, force="scene")
        scene = pyrender.Scene.from_trimesh_scene(tm, bg_color=(0.8, 0.85, 0.9))
        # 金属は環境マップ無しだと真っ黒になるので、拡散寄りに描く（FK-E1 と同じ）。
        # WP-SIM-11 の切り分け A で金具の艶だけ試すときに上書きする（既定は変えない）。
        metal, rough = material_factors(metallic_factor, roughness_factor)
        for m in scene.meshes:
            for prim in m.primitives:
                mat = prim.material
                if isinstance(mat, pyrender.MetallicRoughnessMaterial):
                    mat.metallicFactor = metal
                    mat.roughnessFactor = rough
        # 全体を照らす方向光源（斜め上から。FK-E1 と同じ）
        R = np.array([
            [0.866, -0.25, 0.433, 0.0],
            [0.0, 0.866, 0.5, 0.0],
            [-0.5, -0.433, 0.75, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        scene.add(pyrender.DirectionalLight(color=np.ones(3), intensity=dir_intensity), pose=R)
        scene.ambient_light = np.array([ambient, ambient, ambient])
        # scene.json に照明の記載があれば、FK-E1 と同じ扱いで点光源を足す
        json_path = os.path.join(self._scene_dir, "scene.json")
        if os.path.isfile(json_path):
            with open(json_path, encoding="utf-8") as f:
                info = json.load(f)
            lights = sorted(info.get("lights", []), key=lambda q: q.get("energy", 0.0), reverse=True)
            for q in [z for z in lights if z.get("type") in {"POINT", "SPOT", "AREA"}][:4]:
                pose = np.eye(4)
                pose[:3, 3] = np.array(q["location"], float)
                scene.add(pyrender.PointLight(color=np.array(q["color"], float),
                                              intensity=point_intensity), pose=pose)
        self._scene = scene

        K = self._camera["K"]
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        self._w2 = int(round(WIDTH * _RENDER_SCALE))
        self._h2 = int(round(HEIGHT * _RENDER_SCALE))
        self._ox = (self._w2 - WIDTH) / 2.0
        self._oy = (self._h2 - HEIGHT) / 2.0
        cam = pyrender.IntrinsicsCamera(fx, fy, cx + self._ox, cy + self._oy, 0.05, 100.0)
        self._cam_node = scene.add(cam, pose=np.eye(4))
        self._renderer = pyrender.OffscreenRenderer(self._w2, self._h2)
        self._fold_r = self._geometry.fold_radius(self._camera["dist"])
        u0, v0, ok = _undistort_map(K, self._camera["dist"], WIDTH, HEIGHT, self._fold_r)
        self._map_u = u0 + self._ox
        self._map_v = v0 + self._oy
        self._map_ok = ok & (self._map_u >= 0) & (self._map_u < self._w2 - 1) \
            & (self._map_v >= 0) & (self._map_v < self._h2 - 1)

        self._cache: dict = {}
        self.render_count = 0

    @property
    def scene_dir(self) -> str:
        return self._scene_dir

    def render(self, x: float, y: float, yaw: float, height_m: float,
               cam_yaw_deg: float, cam_pitch_deg: float, seed: int = 0) -> bytes:
        """1280×720 の JPEG のバイト列。前と同じ入力なら前の 1 枚をそのまま返す。"""
        key = (float(x), float(y), float(yaw), float(height_m),
               float(cam_yaw_deg), float(cam_pitch_deg), int(seed))
        if key in self._cache:
            return self._cache[key]
        T_wc = world_to_camera_T(x, y, yaw, height_m, cam_yaw_deg, cam_pitch_deg,
                                 camera=self._camera)
        self._cam_node.matrix = _opencv_to_opengl_camera_pose(T_wc)
        color, _depth = self._renderer.render(self._scene)
        # pyrender の OffscreenRenderer を使い回すと 1 枚あたり約 127 MiB の
        # メモリを保持し続ける（シーンの使い回しで OOM する）。描くたびに消して
        # 作り直す（同じ絵はキャッシュで先に返すので、描いた枚数だけコストが乗る）。
        self._renderer.delete()
        import pyrender
        self._renderer = pyrender.OffscreenRenderer(self._w2, self._h2)
        warped = _bilinear(color, self._map_u, self._map_v)
        warped = np.where(self._map_ok[..., None], warped, 0.0)
        out = _post_noise_jpeg(np.clip(warped, 0.0, 255.0).astype(np.uint8), seed)
        self._cache[key] = out
        self.render_count += 1
        return out

    def close(self):
        self._renderer.delete()
