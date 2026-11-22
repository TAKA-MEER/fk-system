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
失敗する場合は `docker build -t fk-sim-stack/th:latest -f fk_ws/sim/th_src/Dockerfile fk_ws/sim/th_src`
を先に回してからこのスクリプトを回してください。

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
