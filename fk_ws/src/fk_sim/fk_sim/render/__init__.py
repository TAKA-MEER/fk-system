"""描画の芯（`fk_sim.render`）。詳細設計: `DetailedDesign-sim.md` §4。

ROS 2 も hve の口も読まない。シーンの出力（`scene.glb`・`scene.json`）と、
機体の位置・昇降機の高さ・カメラの向きを受けて 1280×720 の JPEG のバイト列を返す。

カメラの外部パラメータは ss の `geometry.camera_T` を import して使う
（`SS_REPO`、既定 `../ss-system`。写さない）。内部の取り付け値（`camera_*`）と
`camera_K`・`camera_dist` は ss の `locator.yaml` を読む。
"""

from .core import (
    Renderer,
    load_ss_camera,
    world_to_camera_T,
)

__all__ = ["Renderer", "load_ss_camera", "world_to_camera_T"]
