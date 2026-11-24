# `sim_auto_check`（WP-SIM-05 段 3 の道具）

本物の th（Gazebo）と本システムで自律測定を最初から最後まで通す。
DetailedDesign-sim.md §7.1 の道具の行。

## 回し方

`sim_fk` の像の中で回す（ROS 2 と型があるので）。出力は `sim_out/`
（`sim_fk` へ bind mount 済み）に書かれる。道具は root のまま回し、
終わったあとに同じ exec の中で出力先をホストのユーザーへ `chown`
する（`docker exec -u` は像の `HOME` が `/` になり `/root` も読めない
ので使わない。道具が失敗しても `trap` で chown される）:

```bash
docker exec sim_fk bash -c 'set +u; source /opt/ros/humble/setup.bash; source /root/fk_ws/install/setup.bash; export ROS_DOMAIN_ID=78; trap "chown -R '"$(id -u):$(id -g)"' /root/fk_ws/sim_out/auto_check" EXIT; python3 /work/tools/sim_auto_check/run.py --scene /work/sim_out/scenes/public_room'
```

th を `IDLE` に入れる手順（`dev_mode` の疎通の無視。なぜ要るか）は
`fk_ws/sim/README.md` の段 3 の節に書いてある。

`--panels P001,P002` で選ぶ盤を絞れる（既定は配置図の全部）。
`--out` で出力先を変えられる（既定は `sim_out/auto_check/<時刻>/`）。
`ROS_DOMAIN_ID=78` で回すこと。

## やること

1. th の `/measure/status` が `IDLE` で本システムの生存確認（`link_ok`）が届くまで待つ
2. th の `/system/trigger` に `ui.goto`（`{"kind":"AUTO_MEASURE"}`）を送る
   （断られたら理由を残して時間をおいて送り直す）
3. 本システムに `auto.begin` → 配置図（`/fk/auto/layout`）を待つ
4. 盤を選んで `auto.start`
5. 結果（`/fk/auto/results`）・`AutoState`・th の `/measure/status` を
   時刻つきで記録し続ける（`trace.jsonl`）
6. 終わりで、盤ごとの結果と、配置図の盤と `scene.json` の正解の盤の
   対応（中心のずれ）を表で出す（`summary.json` と標準出力）

## 純関数

`check.py`: 配置図と正解の対応（いちばん近い盤）・結果の数え方
（`DONE` と `SKIPPED` を混ぜない）・段 3 の合格の判定・終わり方の分類。
ホストで回る試験は `test/test_check.py`:

```bash
python3 -B -m pytest -p no:cacheprovider -p no:anyio tools/sim_auto_check/test
```
