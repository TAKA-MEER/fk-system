# 軽い描画の比較実験（FK-E1）の詳細

本体は [sim-render.md](sim-render.md)。ここは手順・数値・失敗の記録である。

## 実験条件

- シーン: `gen_scene.py` で単盤・連盤 × 電気室・工場の 4 組合せ、各 2 視点で計 20 シーン（`single_er` 6・`single_factory` 4・`row_er` 4・`row_factory` 6）。カメラは `robot`、1280×720。Blender 4.2（Cycles・GPU あり）で生成。
- 基準の絵: Cycles `--no-post`（歪み・ノイズ・JPEG なし PNG）で 20 枚。後処理あり（学習データと同じ条件）も同シーンで 20 枚。カメラ行列と盤の姿勢が 20 シーンすべて一致することを確認した（正解の金具数が 134 と 135 で 1 個違うのは、可視判定が歪み込み画像で行われているため）。
- 書き出し: Blender バッチで `.blend` から `.dae`・`.glb`・カメラと照明の JSON（`tools/fk_e1_render/export_scene.py`）。文字はメッシュに変換済み。手続き的なマテリアル（ノイズ・汚れ・バンプ）は基本色に落とした（JSON の `lost` に記載。残った色は Principled BSDF の Base Color の既定値）。
- 軽い描画はすべて Docker（`tools/fk_e1_render/Dockerfile`: `ubuntu:22.04`＋`gazebo`11＋pyrender、GPU なし、Mesa llvmpipe）で、同じカメラ位置・画角・1280×720 で描いた。露出は Cycles の絵の平均明るさ（約 163/158/149）に合わせて調整した（R-PY の平均約 155/151/144、R-GZ は約 162/159/156 の試行で確定。Gazebo の方向光源・点光源・ambient の強さは `render_gz.py` の既定値、pyrender の方向光源・ambient・点光源は `render_py.py` の既定値）。
- 変種: 軽い描画の絵にノイズ（σ 0.002〜0.012）と JPEG（品質 82〜92）のみを足した版（`post_noise_jpeg.py`。歪みは入れていない）。
- 検出器: ss-system の `Detector`（`imgsz=1280`・`conf=0.25`・`device="cpu"`。重み `lug_v3_s.pt` の SHA-256 照合済み）。正解は注釈の `ring_center_px`（見えている金具のみ）。

## 手順

`tools/fk_e1_render/run_all.py` で通しで回せる（[再現手順](../../tools/fk_e1_render/README.md)）。生成物は `sim_out/fk_e1/`（gitignore 済み）に出る。

| 手順 | 実行 |
| --- | --- |
| シーン生成（Cycles 2 組） | `run_all.py gen` |
| 書き出し（`.dae`・`.glb`・JSON） | `run_all.py export` |
| コンテナのビルド | `run_all.py docker` |
| R-PY / R-GZ の描画 | `run_all.py render py` / `render gz` |
| ノイズ＋JPEG の変種 | `run_all.py variants` |
| 検出と指標 | `run_all.py detect`（結果は `sim_out/fk_e1/detect/summary.json` と組ごとの JSON） |

## 数値（組ごと）

[本体の表](sim-render.md) の内訳。1 枚の時間はシーンごとの `times.json` の中央値（R-PY は暖機 1 回後の 3 回の中央値、R-GZ は保存された 10 枚の間隔の中央値。起動は最初の 1 枚が出るまでの時間で、コンテナ起動と Python の起動は含まない）。

- R-PY の 1 枚時間はシーンの重さに強く依存した（単盤・電気室で 0.31〜0.51 s、工場・連盤で最大 3.1 s。描画対象のポリゴン数に比例する）。
- R-GZ の 1 枚時間はシーンによらず 0.016〜0.045 s（中央値 0.033 s）で安定した。起動は小さなシーンで 1.6 s、重いシーン（工場・連盤、`.glb` から 439 個の obj パーツへの変換を含む）で最大 9.8 s だった。
- 検出の外れ（R-PY で見つからなかった 2 個。`row_factory/000002_0` の 1 個と `single_factory/000000_1` の 1 個）は、Cycles でも同じシーンで見つかっているものではなく、軽い描画側の見え方の差による。ノイズ＋JPEG 版では R-PY が 133/134 まで回復した。

## 落ちたもの・失敗の記録

R-GZ を動く形にするまでに、以下でつまずいた（すべて最終形では回避済み）。

1. **`.dae` を Gazebo にそのまま読ませると白飛びする**: Blender の COLLADA（Z-up）は Gazebo/OGRE の読み込みで軸を変換され、カメラの前に壁が来て全面が白（255）になった。モデルを回転させて戻す（±90°）ことも試したが、盤は見えなかった。
2. **Blender の `.obj` 書き出しはオブジェクト変換を焼き込まない**: 盤が局所座標のまま出て、裏側が見える絵になった。親子関係を解いて焼き込む（`parent_clear`＋`transform_apply`）ことも試したが、座標軸まで変換されてしまい直らなかった。
3. **`.glb` を trimesh で 1 個に結合するとマテリアルが落ちる**: 全面が黒になった。ジオメトリ毎に分けて個別の `.mtl` を付ける形にした。
4. **trimesh の obj 書き出しは既定で法線を書かない**: 壁が黒くなった（`include_normals=True` で対処）。
5. **壁を入れたとたん画面が真っ黒になった**: 原因は法線の欠落（上記 4）で、影ではなかった。影は R-PY（影なし）に揃えるため切ってある。
6. **PowerShell から `docker run ... bash -c` で `xvfb-run -s '-screen 0 1280x720x24'` を渡すと引用符が崩れる**: ラッパーのシェルスクリプトをファイル（LF・BOM なし。`.gitattributes` で `* text=auto eol=lf`）にして `bash <file>` で実行することで回避した。Git Bash から `docker run -w /work` を直接打つと `/work` が MSYS のパス変換で壊れるので、`run_all.py`（引数のリスト渡し）経由で実行する。

残った形: R-GZ は `scene.glb` を trimesh でジオメトリ毎にワールド座標の `.obj`（法線・個別 `.mtl` あり）へ変換し、SDF の複数 `<visual>` として置く（`render_gz.py` の既定動作）。`.dae` は書き出しに残っているが、R-GZ では使っていない。

## 絵の例の場所

`sim_out/fk_e1/` の下（gitignore 済み。再生成できる）。

- `cycles_nopost/<組合せ>/images/<stem>.png` — 基準の絵
- `cycles_post/<組合せ>/images/<stem>.jpg` — 後処理ありの絵
- `render_py/<組合せ>/<stem>/frame.png` — R-PY
- `render_gz/<組合せ>/<stem>/frame.jpg` — R-GZ
- `variants/<py|gz>/<組合せ>/<stem>.jpg` — ノイズ＋JPEG の変種
- `detect/<組>.json` — 組ごとの 1 枚ごとの結果

代表例: `single_er/000000_0`（単盤・電気室）と `row_factory/000002_1`（重い連盤・工場）。
