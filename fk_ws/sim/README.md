# シミュレーション一式（WP-SIM-04）

th の Gazebo・hve の偽物と描画サービス・本物の ss の 3 つを起こして、
**ss で配置図が取れる**ところまで（段 2）。

DetailedDesign-sim.md §6（起動）・§7（段 2）の道具。名前の正本は
DetailedDesign-names.md §8（`fk_ws/sim/th_sim_ref`・`import_th_sim.sh`・
`sim_up.sh`・`sim_down.sh`・`tools/sim_locate_check/`）。

## 起こす

```bash
bash fk_ws/sim/sim_up.sh          # シーン public_room（既定）
bash fk_ws/sim/sim_up.sh <名前>   # 別のシーン
```

環境変数（絶対パスで渡す。既定は `../th-system` など）：

| 変数 | 意味 | 既定 |
| --- | --- | --- |
| `TH_REPO` | th-system の置き場 | `../th-system` |
| `SS_REPO` | ss-system の置き場 | `../ss-system` |
| `HVE_REPO` | hve-system の置き場 | `../hve-system` |
| `BLENDER` | シーン生成に使う Blender の実行ファイル | `C:/Program Files/Blender Foundation/Blender 4.2/blender.exe` が無い場合は失敗 |
| `ROS_DOMAIN_ID` | 78 固定 | — |

`ROS_DOMAIN_ID=78`・`network_mode: host`・FastDDS は UDP だけ（th の
プロファイル）。`th_robot` コンテナには触らない。GPU は渡さない。

### 何が起きるか

| サービス | 像 | 起動 |
| --- | --- | --- |
| `sim_th` | `fk-sim-stack/th:latest`（`fk_ws/sim/th_src/Dockerfile` から） | `ros2 launch th_bringup gazebo.launch.py sim:=true gui:=false rviz:=false world:=<生成したワールド> robot_x/y/yaw:=<記述の機体の置き場>`。`th_ws` は像の中で `colcon build` 済み（起動でビルドしない） |
| `sim_hve` | `fk-sim:latest`（ビルド済みの描画の像） | `bash fk_ws/sim/hve_up.sh`。`pose_source: gazebo`（Gazebo の機体の真の位置で描く） |
| ss | ss-system の `scripts/ss.sh start` | `start --camera-host 127.0.0.1:18000 --lift-host 127.0.0.1:18080`（`ROS_DOMAIN_ID=78` を渡す。ss の起動の作りを写さない） |

シーンの生成物（`world.world`・`scene.glb`・`scene.json`）は
`sim_out/scenes/<名前>/` に置き、`sim_th` と `sim_hve` へ読み取り専用で
マウントする。th の取り出し（`fk_ws/sim/th_src/`）は `.gitignore` 済み。

**始めに同じ名前のコンテナ `ss-system` が既に動いていたら、sim_up.sh は
実行せず止まる（止めない）。** 別の人の作業かもしれないから。

**th の像（`fk-sim-stack/th:latest`）のビルドは初回で時間がかかります。**
2 段で作る: th の `th_ws/Dockerfile`（apt 依存等だけビルド。tag `fk-sim-stack/th-base:latest`。
15 分ほど）→ `fk_ws/sim/th_Dockerfile`（上に th_ws のソースを入れて colcon build）。
まず `docker compose build` を回して（compose の `build` が `th_Dockerfile` を参照）
一式を起こす。1 回だけ `docker tag fk-sim-stack/th:latest fk-sim-stack/th-base:latest` を
事前に実行する。

```bash
docker build -t fk-sim-stack/th-base:latest -f fk_ws/sim/th_src/Dockerfile fk_ws/sim/th_src
docker tag fk-sim-stack/th-base:latest fk-sim-stack/th:latest   # 互換のためのタグ付け
```

**シーンは `fk_ws/src/fk_sim/scenes/public_room.yaml` を読んで生成する。**
Blender が無い環境では `BLENDER` を設定することか、生成物（`sim_out/scenes/<名前>/scene.json`）が既にあれば
それを使う（記述が更新されたら作り直す）。

## 起きているか見る

```bash
docker ps --filter name=sim_th              # gzserver が動いていること（gzclient は無い）
docker exec sim_th ros2 service list | grep get_entity_state   # Gazebo が見えている
docker exec sim_hve curl -s http://127.0.0.1:18100/frame -o /tmp/frame.jpg  # sim_render が絵を返した
bash "$SS_REPO/scripts/ss.sh" status        # ss の状態が IDLE（待ち）になる
```

## 機体を置き直して ss に依頼する（結果と正解を比べる）

`tools/sim_locate_check/`（機体を置き直して依頼し、結果の盤の矩形を
`scene.json` の正解と比較する道具）:

```bash
python tools/sim_locate_check/run.py --x <x> --y <y> --yaw <yaw> \
  --scene sim_out/scenes/public_room
```

3 か所以上（盤から 1.5〜2.6 m の所を少なくとも 1 か所、2.6〜5 m の所を
少なくとも 1 か所）に置いてそれぞれ依頼し出す。
盤ごとに、機械の中心のずれ・向きのずれ・四隅のずれ・機体からの距離が出る。
座標の直し（`base_link` への変換）は純関数（`pose.py`）で、ホストで
回る試験（`test/test_pose.py`）がある。

### 実測（2026-11-22、public_room シーン。数値はすべてこの環境で回した値）

機体の向きはすべて yaw 1.5708 rad。依頼は ss が受付から結果まで約 88〜90 秒
（走査に数十枚の描画。描き直しは `OffscreenRenderer` を都度作り直す対処）。

| 機体の置き場 | 盤までの距離 | ss の応答 | `reason` | 盤の数 |
| --- | --- | --- | --- | --- |
| (6.0, 2.5) | 約 5.1 m | 90.4 s | `lugs_ungrouped` | 0 |
| (6.0, 4.5) | 約 3.1 m | 87.2 s | `completed` | 5 |
| (3.0, 4.0) | 約 4.1 m | 89.7 s | `completed` | 3 |
| (6.0, 5.5) | 約 2.0 m | 87.9 s | `completed` | 6 |

盤に組めた配置の比較表（中心のずれ・向きのずれ・四隅のずれ・原点距離。`run.py` の表）：
向きは ss 側が `front_known=False` を返し続けたため常に `-`（得られず）。
正解の盤は 4 面あるが、ss は前後どちらから見ても盤として返すため
（盤の表裏を ss が区別できない）正しい位置に組めた分に加えて ±0.69 m ずれた
複製も出る（表に P001・P002 が重複して見える）ことを確認した。

- (6.0, 4.5)：正しい位置に組めた盤の中心のずれは 0.004 m（原点距離 3.02 m）。
- (3.0, 4.0)：同 0.302 m ほか。
- (6.0, 5.5)：同 0.081 m（原点距離 2.02 m）。
- 四隅のずれはどれも 0.8〜1.1 m（ss 側の幅の見積もりが正解より大きいため。
  1 例として (6.0, 4.5) の盤 P002 は ss の幅 1.076 m に対し正解 0.8 m）。
- 約 5 m では金具は見えたが盤に組めず（`lugs_ungrouped`、0 面）。

検出率='/ 'ではなく、このシーン・距離の盤面の可否は ss と描画の組み合わせで決まる
ことを表から読み取るための道具である（仕様の精度 5 cm を満たす値ではない）。

座標変換の約束：`world = R(yaw) * base_link_xy + (x, y)`。
`ss` の結果は盤ごとの天面の矩形を **ローカル座標（`base_link`）** で返す
（`locate_cli` の `--json` の形は ss の実装が決める）。

## 止める

```bash
bash fk_ws/sim/sim_down.sh
```

**自分が起としたコンテナだけを止める**
（`sim_th`・`sim_hve` は project 名 `fk-sim-stack` で指定。ss は
`scripts/ss.sh stop` で。`docker stop` の名前は使わない）。

## 既知のこと

- ss と描画のループの間、描画は `sim_render` が `GET /frame` で持つ。hve の映像の偽物（`hve_video`）は
  `--frame-url http://127.0.0.1:18100/frame` で取りに来る。
- ss が結果を返してから後片付けの完了までかかった時間は、ss の結果の形を見る（`cleanup_limit_s` など）。
- 名札の文字は出ない（日本語のフォントが無いため。範囲外）
