# 検出の確かめ（`sim_render_check`）

ホスト側（lifting-lug-pose の venv）で回す: 描画は fk-sim コンテナの中で行い、
ss の `Detector`（imgsz=1280・conf=0.25・CPU）で金具の検出を確かめる。

```
<venvpython> tools/sim_render_check/check.py --scene sim_out/scenes/public_room \
    --out .briefs/tmp/check_frames
```

**ここの「検出率」は描き方（露出・歪み・天面の見え方）の確かめであって、
位置の確かめではない。**正解の投影も描画も同じ `world_to_camera_T` を通るため、
`world_to_camera_T` 自体の誤りはここでは捕まらない。位置の確かめは
`fk_ws/src/fk_sim/test_render/` の、ワールド座標で独立に決めた期待値で判定する
試験（機体の向き・高さ・ロール・取り付けの値）が担う（WP-SIM-02 差し戻し対応）。
