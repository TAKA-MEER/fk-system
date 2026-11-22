# tools/fk_e1_render — FK-E1 の軽い描画の再現手順

GPU の無い環境で、Blender で作ったシーンを Gazebo Classic（R-GZ）と pyrender（R-PY）で描き、ss-system の吊り金具の検出器にかけるまでのスクリプト一式。結論は `docs/plan/sim-render.md` を参照。

## 前提

- Blender 4.2（`C:/Program Files/Blender Foundation/Blender 4.2/blender.exe`）
- `lifting-lug-pose` の `.venv`（ultralytics・torch・numpy・Pillow・scipy 入り。`run_all.py` の実行用）
- Docker Desktop（Linux コンテナ。GPU を渡さないこと）
- ss-system の `Detector` と重み `lug_v3_s.pt`（読むだけ。手を入れない）

## 手順（ホスト側から）

```bash
# lifting-lug-pose の .venv の Python で
python tools/fk_e1_render/run_all.py gen       # Cycles の基準絵 2 組（--no-post / 後処理あり）
python tools/fk_e1_render/run_all.py export    # .blend から .dae・.glb・カメラと照明の JSON
python tools/fk_e1_render/run_all.py docker    # 軽い描画用のコンテナをビルド
python tools/fk_e1_render/run_all.py render py # R-PY の描画（全 20 シーン）
python tools/fk_e1_render/run_all.py render gz # R-GZ の描画（全 20 シーン）
python tools/fk_e1_render/run_all.py variants  # ノイズ＋JPEG の変種
python tools/fk_e1_render/run_all.py detect    # 検出器にかけ、sim_out/fk_e1/detect/ に指標を出す
```

生成物（`.blend`・書き出し・絵・検出の結果）は `sim_out/fk_e1/` に出る（gitignore 済み）。

## 各スクリプト

| ファイル | 役割 |
| --- | --- |
| `run_all.py` | 上記を順に実行するホスト側ラッパー（`docker run` は引数のリスト渡し。PowerShell の引用符崩壊を避ける） |
| `export_scene.py` | Blender バッチ用（`blender -b <scene>.blend -P export_scene.py -- --out <dir>`）。`.dae`・`.glb`・`camera_lights.json` を出す |
| `Dockerfile` | `ubuntu:22.04`＋`gazebo`11＋pyrender（CPU・Mesa llvmpipe） |
| `render_all.sh` | コンテナ内で全シーンを描く（`render_all.sh py` / `gz`。`.sh` は LF・BOM なし） |
| `render_py.py` | R-PY。`.glb` を pyrender＋OSMesa で描く。起動と 1 枚の時間を分けて測る |
| `render_gz.py` | R-GZ。`scene.glb` を trimesh でジオメトリ毎のワールド座標 `.obj`（法線・個別 `.mtl`）に変換し、SDF の複数 `<visual>` として `gzserver`（xvfb＋llvmpipe）で描く |
| `world_sdf.py` | 書き出しから Gazebo の `.world` を作る（カメラは Blender の行列を Gazebo の前・左・上へ組み替え、照明は近いものを置く） |
| `post_noise_jpeg.py` | ノイズ（σ 0.002〜0.012）と JPEG（品質 82〜92）のみを足す（`gen_scene.py` の後処理に合わせた値） |
| `detect_eval.py` | ss-system の `Detector` にかけ、検出率・誤差・誤検出・確信度を出す（15 px 閾値） |

## 注意点

- **`.dae` を直接 Gazebo に読ませない**: Blender の COLLADA（Z-up）は OGRE の読み込みで軸が変換され、白飛びする。Blender の `.obj` 書き出しもオブジェクト変換を焼き込まない。R-GZ は変換が正しい `.glb` から作る（`render_gz.py` の既定動作）。
- **trimesh の obj 書き出しは法線を明示的に指定する**（`include_normals=True`）。既定では法線が落ちて壁が黒くなる。
- **PowerShell から `docker run ... bash -c` で引用符付きの文字列を渡さない**。ラッパーは `render_all.sh` のようなファイル（LF・BOM なし）で渡す。Git Bash から `docker run -w /work` を直接打つと MSYS のパス変換で壊れるので、`run_all.py` 経由で実行する。
- **Docker で root 所有のファイルが `sim_out/` に残る**ことがある。その場合はホスト側で持ち主を戻す（`docker run --rm -v <repo>:/work fk-e1-render chown -R $(id -u):$(id -g) /work/sim_out` など）。
- 診断用オプション（既定では使わない）: `render_gz.py --rpy-order xyz`（SDF の RPY の解釈順の試行）、`--scene-roll`（`.dae` の軸変換を戻す試行）、`--max-parts`（obj パーツ数の上限）。
