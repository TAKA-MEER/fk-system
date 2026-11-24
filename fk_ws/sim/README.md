# シミュレーション一式（WP-SIM-04・WP-SIM-05）

th の Gazebo・hve の偽物と描画サービス・本物の ss を起こして、
**ss で配置図が取れる**ところまで（段 2）。段 3 からは本システム
（`sim_fk`）も起こして自律測定を通す（`tools/sim_auto_check/`）。

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
| `BLENDER` | シーン生成に使う Blender の実行ファイル | Windows のときだけ `C:/Program Files/Blender Foundation/Blender 4.2/blender.exe`。Ubuntu では `~/.local/opt/blender-4.2.2-linux-x64/blender` があればそれ（公式 Linux 版 4.2.2。https://download.blender.org/release/Blender4.2/）。どちらも無ければ失敗 |
| `ROS_DOMAIN_ID` | 78 固定 | — |

`ROS_DOMAIN_ID=78`・`network_mode: host`・FastDDS は UDP だけ（th の
プロファイル）。`th_robot` コンテナには触らない。GPU は渡さない。

### 何が起きるか

| サービス | 像 | 起動 |
| --- | --- | --- |
| `sim_th` | `fk-sim-stack/th:latest`（`fk_ws/sim/th_src/Dockerfile` から） | `ros2 launch th_bringup gazebo.launch.py sim:=true gui:=false rviz:=false world:=<生成したワールド> robot_x/y/yaw:=<記述の機体の置き場>`。`th_ws` は像の中で `colcon build` 済み（起動でビルドしない） |
| `sim_hve` | `fk-sim:latest`（ビルド済みの描画の像） | `bash fk_ws/sim/hve_up.sh`。`pose_source: gazebo`（Gazebo の機体の真の位置で描く） |
| `sim_fk` | `fk_ws-fk_robot:latest`（`fk_ws` の `docker compose build` でできる像） | コンテナ内で `colcon build` して `ros2 launch fk_bringup fk.launch.py`。パラメータは `lift_host: 127.0.0.1`・`lift_port: 18080`・`mechanism_bypass: true`（インストール先の `params.yaml` へ上書き。ホストの物は変えない）。起動前に `fk_ws/scripts/import_th.sh`・`import_ss.sh` で `th_import`・`ss_import` を取り込むこと |
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

## 自律測定を通す（段 3）

th を `IDLE` に入れるために開発モードの手設定（`dev_mode`・
`dev_ignore_link`）は要らない。th の直し（`103a6e42`。sim の `link_ok` が
物理非常停止の受信を要らなくなった）で `IDLE` に入る。確認:

```bash
docker exec sim_fk bash -c 'source /opt/ros/humble/setup.bash && source /root/fk_ws/install/setup.bash && export ROS_DOMAIN_ID=78 && ros2 topic echo /measure/status --once'  # mode: IDLE を確かめる
```

次に道具を回す。`sim_fk` の像の中で回す（ROS 2 と型があるので）。
出力は `sim_out/`（`sim_fk` へ bind mount 済み）に書かれる。道具は
root のまま回し（`docker exec -u` でホストのユーザーにすると、像の
`HOME` が `/` になって `rclpy.init()` が落ち、`/root` が読めず型も
読めない）、終わったあとに同じ exec の中で出力先をホストのユーザーへ
`chown` する（道具が失敗しても `trap` で chown される）:

```bash
docker exec sim_fk bash -c 'set +u; source /opt/ros/humble/setup.bash; source /root/fk_ws/install/setup.bash; export ROS_DOMAIN_ID=78; trap "chown -R '"$(id -u):$(id -g)"' /root/fk_ws/sim_out/auto_check" EXIT; python3 /work/tools/sim_auto_check/run.py --scene /work/sim_out/scenes/public_room'
```

手順・出力の既定は `tools/sim_auto_check/README.md`。

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

### 実測（2026-11-22（2 回目・金具の向き修正後）、public_room シーン。数値はすべてこの環境で回した値）

前の実測（下の表の 1 回目）は、シーンの吊り金具の向きの誤りによるものであった（リング面の水平方向が盤の中心を 90° 外れていた）。

| 機体の置き場 | 盤までの距離 | ss の応答 | `reason` | 盤の数 |
| --- | --- | --- | --- | --- |
| (6.0, 2.5) | 約 5.1 m | 87.3 s | `completed` | 1 |
| (6.0, 4.5) | 約 3.1 m | 89.2 s | `completed` | 4 |
| (3.0, 4.0) | 約 4.1 m | 88.3 s | `completed` | 2 |
| (6.0, 5.0) | 約 2.5 m | 88.7 s | `completed` | 4 |
| (6.0, 5.5) | 約 2.0 m | 89.9 s | `completed` | 4 |

盤ごとに並べた比較（`run.py` の表。向きのずれは ss 側が `front_known=False` を
返し続けたため得られず `-`。四隅の対応は 90°/180° 回転と逆向き経路の 8 通りの中から
最大誤差が最小の組を選び、その組み合わせを「対応」の列に示す。`mirror` は逆向きで組んだ意）：

機体の置き場 (6.0, 2.5)：

- P004：距離 3.03 m、中心のずれ 0.007 m、四隅最大 0.011 m（対応 mirror 180°）。
  この位置ではこの 1 面しか組めなかった（壁の P001〜P003 は 5 m 離れている）。

機体の置き場 (6.0, 4.5)：

- P002：距離 3.02 m、中心のずれ 0.022 m、四隅最大 0.047 m（対応 mirror 0°）。
- P003：距離 3.12 m、中心のずれ 0.045 m、四隅最大 0.055 m（対応 mirror 0°）。
- P001：距離 3.14 m、中心のずれ 0.007 m、四隅最大 0.012 m（対応 mirror 0°）。
- P004：距離 3.35 m、中心のずれ 0.005 m、四隅最大 0.006 m（対応 mirror 180°）。

機体の置き場 (3.0, 4.0)：

- P001：距離 4.10 m、中心のずれ 0.003 m、四隅最大 0.007 m（対応 mirror 0°）。
- P002：距離 4.56 m、中心のずれ 0.004 m、四隅最大 0.014 m（対応 mirror 0°）。

機体の置き場 (6.0, 5.0)：

- P002：距離 2.52 m、中心のずれ 0.007 m、四隅最大 0.010 m（対応 mirror 0°）。
- P003：距離 2.64 m、中心のずれ 0.006 m、四隅最大 0.009 m（対応 mirror 0°）。
- P001：距離 2.67 m、中心のずれ 0.005 m、四隅最大 0.007 m（対応 mirror 0°）。
- P004：距離 3.59 m、中心のずれ 0.013 m、四隅最大 0.018 m（対応 mirror 180°）。

機体の置き場 (6.0, 5.5)：

- P002：距離 2.02 m、中心のずれ 0.006 m、四隅最大 0.010 m（対応 mirror 0°）。
- P003：距離 2.17 m、中心のずれ 0.004 m、四隅最大 0.007 m（対応 mirror 0°）。
- P001：距離 2.21 m、中心のずれ 0.004 m、四隅最大 0.005 m（対応 mirror 0°）。
- P004：距離 3.90 m、中心のずれ 0.002 m、四隅最大 0.019 m（対応 mirror 180°）。

注記：
- 検出は距離に依存し、約 5 m では壁の盤が組めず（この配置では 1 面）、4 m 程度でも一部組めない。
- 向きのずれは ss が正面の向きを返さないため得られていない（`front_known=False`）。
- ss の前後（表裏）の区別は無いため、四隅の対応は `mirror 180°` を選ぶ場合がある。

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
