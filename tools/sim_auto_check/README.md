# `sim_auto_check`（WP-SIM-05 段 3 の道具）

本物の th（Gazebo）と本システムで自律測定を最初から最後まで通す。
DetailedDesign-sim.md §7.1 の道具の行。

## 回し方

`sim_fk` の像の中で回す（ROS 2 と型があるので）:

```bash
docker exec sim_fk python3 /work/tools/sim_auto_check/run.py \
  --scene /work/sim_out/scenes/public_room
```

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
