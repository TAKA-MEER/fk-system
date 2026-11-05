# 名前辞書

[DetailedDesign.md](DetailedDesign.md) の詳細。**この文書に無い名前を実装で作ってはいけない。**

> **`DD-1`**: 名前を発明する余地をゼロにする。必要な名前が無いと分かったら、
> **実装する前にこのファイルへ行を足す。**

**2026-11-04: 自律測定の移動（[-auto.md](DetailedDesign-auto.md)）の名前を足した。**接頭辞・名前空間・ポート・コンテナ名（§0・§6）は **2026-11-05 ユーザー承認**。

---

## 0. 命名規則

**th-system と同じ構成で作る**（2026-11-04 ユーザー決定。[Spec-open.md](../spec/Spec-open.md) `FK-A2`）ので、th-system [DetailedDesign-names.md](../../../../th-system/docs/plan/detailed/DetailedDesign-names.md) §0 に揃える
（パッケージ接頭辞だけを本システム用に決める）。

| 対象 | 規則（2026-11-05 ユーザー承認） | 例 |
| --- | --- | --- |
| パッケージ | `fk_<領域>` | `fk_auto` |
| トピック・サービス | **`/fk/<領域>/<名前>`**（th と同じ ROS 2 のドメインに居るので、th の名前空間とぶつからないよう `/fk/` を前に付ける） | `/fk/auto/state` |
| 指示（画面 → 機能側） | `<領域>.<名前>`（th の `ui.*`・`meas.*` と同じ形） | `auto.start` |
| 段・理由 | 段は大文字スネーク、理由キーは小文字スネーク | `GOAL` ／ `no_path` |
| パラメータ | 小文字スネーク。単位を接尾辞に付ける | `blocked_wait_s` |
| 作業パケット | `WP-<領域>-<番号>` | `WP-AUTO-01` |

## 1. 関連システム側の名前（参照のみ・こちらでは発明しない）

`FK-A1` で結合すると決まったら、結合に関わる相手側の名前を出典つきで載せる。**正は相手の文書とコード。**
**th-system とのやり取り（自律測定と、お立会試験の停止命令の間）の名前と形は th-system が書き、本システムが確かめる**（2026-11-04 ユーザー決定）。

| 相手 | 正 | 状態 |
| --- | --- | --- |
| th-system | th [DetailedDesign-measure.md](../../../../th-system/docs/plan/detailed/DetailedDesign-measure.md)（通り道 §1・名前の一覧 §2・型 §3・画面 §4・入口の分離 §5）。型は th `th_ws/src/th_system_msgs` を取り込み、版 `interface_version` で照合する | **2026-11-04 本システムが確かめ、ユーザーが承認。**自己位置を見失ったときに目的地を取り消して返すことを th に頼み、th が反映した（th `6f1ca35`。[Spec-open.md](../spec/Spec-open.md) §1.2）。版は `"1"` のまま（th は実装前の案の直しでは上げない） |

| hve-system（昇降部） | hve [DetailedDesign-protocol-modules.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-protocol-modules.md)（**案**。今の実物は hve [DetailedDesign-protocol.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-protocol.md) §2）。本システムは名前 `fk`・`operator: auto`・`busy_label`「測定操作中」で繋ぐ | **2026-11-05 本システムが確かめた。**頼み（測定機構の欄は分からないとき `null`）は hve が反映（hve `6411a6f`）。ss の確認待ち。**hve の実装が入るまで、案の名前で実装しない**（hve の指示） |

| ss-system | ss の詳細設計（`docs/plan/detailed/`。**まだ書かれていない**）。ROS 2 のアクションの形（依頼・取り消し・結果）と、後片付けの完了を知らせる別の口 | 2026-11-05 ss のユーザー決定。ss が書いたら本システムが確かめる |

**名前は写さない**（写しは先方が直ったときに古いまま残る）。ss-system の名前は未定（[-open.md](DetailedDesign-open.md) §1）。決まるまで [-auto.md](DetailedDesign-auto.md) §1.3 の口は抽象のまま。

## 2. パッケージ・ノード・Python の名前

| パッケージ | 中身 |
| --- | --- |
| `fk_msgs` | 本システムの型（§4） |
| `fk_auto` | 自律測定の移動。モジュール `fk_auto.core`（純関数）・`fk_auto.ports`（口）・`fk_auto.runner`（状態機械）。ノード **`auto_runner`** |
| `fk_sim` | 試験用。ノード **`fake_th`**（th の約束どおりに振る舞う偽の th） |
| `fk_web` | 本システムの画面（静的ファイル） |
| `fk_bringup` | launch と設定（`params.yaml`） |

| Python の名前 | 意味 |
| --- | --- |
| `Snapshot` | 状態機械が core に渡す「いまの様子」（th・昇降機・ss・測定機構の口から集めたもの） |
| `ThPort`・`LiftPort`・`SsPort`・`ArmPort` | 口（[-auto.md](DetailedDesign-auto.md) §1.3） |
| `FakeTh`・`FakeLift`・`FakeSs`・`NoMechanism` | 偽の口（ROS 2 なしの試験用）。`NoMechanism` は最初の版の測定機構の口（測定機構なし） |
| `to_map`・`front_edge`・`is_confident`・`group_rows`・`order_panels`・`stand_pose`・`stand_blocked`・`needs_regoal`・`ss_result_valid`・`can_run`・`classify_skip`・`classify_halt`・`gate`・`drive_watch` | 純関数（[-auto.md](DetailedDesign-auto.md) §3・§4） |

## 3. トピック・サービス（本システム側）

| 名前 | 種別・型 | 向き | QoS・頻度 |
| --- | --- | --- | --- |
| `/fk/auto/command` | サービス `fk_msgs/AutoCommand` | 画面 → `auto_runner` | 操作のたび |
| `/fk/auto/state` | トピック `fk_msgs/AutoState` | `auto_runner` → 画面 | reliable・transient_local・depth 1。`state_period_ms` ごと＋変化時 |
| `/fk/auto/layout` | トピック `fk_msgs/Layout` | `auto_runner` → 画面 | reliable・transient_local・depth 1。変化時 |
| `/fk/auto/results` | トピック `fk_msgs/PanelResults` | `auto_runner` → 画面 | reliable・transient_local・depth 1。変化時 |

th 側の名前（`/measure/*`・`/map`・`/ui/active_screen`・`/safety/estop_ui` ほか）は §1 の th の文書のとおり。

## 4. 型・指示・段・理由（`fk_msgs`）

### 4.1 型

| 型 | 中身 |
| --- | --- |
| `AutoCommand.srv` | `string command` / `string[] panel_ids`（`auto.start` だけ）/ `bool arm_confirmed`（`auto.finish` だけ。画面で腕の確認が済んだ）→ `bool accepted` / `string reject_reason` |
| `AutoState.msg` | `std_msgs/Header header` / `string phase` / `string panel_id`（いま向かっている・作業中の盤）/ `string[] queue`（残りの順）/ `string halt_reason` / `string gate_reason`（関門で止めている理由。無ければ空）/ `bool lift_at_bottom` / `bool lift_readable` / `bool arm_stowed` / `bool mechanism_attached` / `bool reentry_required` / `string th_mode` / `string th_state` |
| `Panel.msg` | `string panel_id` / `float64 x` / `float64 y`（天面の中心。map フレーム [m]）/ `float64 front_yaw`（正面の向き。盤から外向き [rad]）/ `float32 width_m` / `float32 depth_m` / `float32 top_height_m` / `float32 pos_std_m` / `float32 yaw_std_deg` / `int32 hangers`（ss の確からしさ）/ `bool confident`（しきい値を満たすか）/ `string row_id` |
| `Layout.msg` | `std_msgs/Header header` / `string map_instance_id` / `Panel[] panels` / `float32[] unseen_from_rad` / `float32[] unseen_to_rad`（見ていない方向。map フレーム。ss が返す「実際に見た方位角の範囲」の残り） |
| `PanelResult.msg` | `string panel_id` / `string result`（`DONE` / `SKIPPED`）/ `string reason` |
| `PanelResults.msg` | `std_msgs/Header header` / `PanelResult[] results` |

### 4.2 指示（`auto.*`。画面 → `auto_runner`）

| 指示 | 意味 | 受ける段 |
| --- | --- | --- |
| `auto.begin` | 画面が開いた（配置図を作り始める） | `IDLE`（th が `AUTO_MEASURE` のとき） |
| `auto.start` | 選んだ盤で開始 | `SELECT`・`FINISH` |
| `auto.stop` | 停止（→ `meas.stop`） | 走っている間 |
| `auto.run` | 走行（→ `can_run` を通れば `meas.start`） | `PAUSED` |
| `auto.abort` | 中止 | `GOAL`・`RECHECK`・`AT_PANEL`・`PAUSED` |
| `auto.rescan` | もう一度探す | `SELECT`・`FINISH`（指示待ちで腕が格納位置） |
| `auto.finish` | 終了（→ `meas.finish`） | いつでも |

| `reject_reason` | 意味 |
| --- | --- |
| `phase_not_accepting` | いまの段では受けない |
| `lift_not_bottom`・`arm_not_stowed`・`ss_busy`・`state_unreadable`・`reentry_required`・`version_mismatch` | 関門が通さない（[-auto.md](DetailedDesign-auto.md) §4.1） |
| `arm_not_confirmed` | 腕が格納位置にないのに `arm_confirmed` が無い |
| `bad_argument` | 盤が空・知らない盤 |

### 4.3 段と理由

| 種類 | 名前 |
| --- | --- |
| 段（`phase`） | `IDLE` / `PREPARE` / `SURVEY` / `SELECT` / `LOWER` / `GOAL` / `RECHECK` / `AT_PANEL` / `FINISH` / `PAUSED` / `REENTRY` / `HALTED` |
| 飛ばす理由（`PanelResult.reason`） | `no_path`（経路の再計算をしても解けない）/ `stand_blocked`（正面側の床が塞がっていて立てない）/ `stand_mismatch`（認識し直した立ち位置が許容差を超える）/ `top_not_found`（天面が見つからない）/ `arm_space_blocked`（腕が振れる範囲が左右とも塞がっている。測定機構の口が返す）/ `no_mechanism`（測定機構が付いていない）/ `ss_failed`（ss の結果が時間内に来ない・使える結果が得られない） |
| 止める理由（`halt_reason`） | `arm_stuck`（腕が出たまま動けない）/ `lift_not_lowered`（昇降機を下げられない）/ `state_unreadable`（昇降機の高さか腕の状態が読めない）/ `ss_cleanup_timeout`（ss の後片付けの完了が来ない）/ `ss_link_lost`（ss と通信できない）/ `arm_link_lost`（腕が出ている間に腕の制御器と通信できない） |

## 5. パラメータ（`fk_bringup` の `params.yaml`。`auto_runner`）

| 名前 | 単位 | 値 | 出どころ |
| --- | --- | --- | --- |
| `heartbeat_period_ms` | ms | 200 | 先方（th の生存確認 5 Hz。th DetailedDesign-measure.md §3.4） |
| `runner_stall_ms` | ms | 500 | 暫定（状態機械がこれより長く回らなければ生存確認を止める） |
| `state_period_ms` | ms | 500 | 暫定 |
| `th_status_stale_ms` | ms | 500 | 暫定（th の状態は 10 Hz。これより古ければ th の様子が読めないとみなし、動く指示を出さない） |
| `lift_state_stale_ms` | ms | 500 | 暫定（昇降部の配信は 100 ms（hve の仮値）。その 5 回分。hve も足りると確かめた） |
| `lift_cmd_period_ms` | ms | 100 | 先方（hve の勧め。昇降部は 600 ms 命令が途切れると止まる） |
| `lift_host` | — | `hve-lift.local` | 先方（hve の mDNS の名前）。引けなければ IP を設定で与える（直書きしない） |
| `lift_drift_stop_mm` | mm | 50 | 暫定（[Spec-auto.md](../spec/Spec-auto.md) §4・`FK-V5`） |
| `no_lower_move_max_m` | m | 0 | 暫定（`FK-V5`。毎回下げる） |
| `lift_lower_timeout_s` | s | 60 | 暫定（下端まで下げきるのを待つ上限。超えたら `lift_not_lowered`） |
| `stand_distance_min_m` | m | 0.389 | 先方（th Spec-measure §4.4。盤面から機体の中心まで） |
| `stand_margin_m` | m | 0 | 暫定（`FK-V1`） |
| `arrive_tol_m` | m | 0.12 | 先方（th の到着の許容差） |
| `arrive_tol_rad` | rad | 0.12 | 先方（同上） |
| `regoal_max` | 回 | 1 | 仕様（[Spec-auto.md](../spec/Spec-auto.md) §6「目的地を 1 回出し直してもまだ超える」） |
| `reroute_max` | 回 | 3 | 暫定（仕様は「何回か」） |
| `blocked_wait_s` | s | 5 | 暫定（`BLOCKED` がこれだけ続いたら経路の再計算を出す） |
| `ss_result_timeout_s` | s | 300 | 暫定（ss へ制限時間として渡す。ss は過ぎたら「時間切れ」で後片付けして返す） |
| `ss_pos_std_max_m` | m | 0.15 | 暫定（盤を「確か」とみなす位置の標準偏差の上限。ss の配置図向けの精度の目標 ±15 cm に揃えた） |
| `ss_yaw_std_max_deg` | ° | 5 | 暫定（同じく向き。ss の目標 ±5° に揃えた） |
| `ss_cleanup_timeout_s` | s | 60 | 暫定（仕様は「一定時間」。結果か取り消しのあと、後片付けの完了を待つ上限。超えたら `ss_cleanup_timeout`。依頼から数えると、本システムが待つのは最長で `ss_result_timeout_s` ＋ これ） |
| `ss_retry_max` | 回 | 2 | 暫定（ss の結果が使えないとき依頼し直す上限） |
| `ss_pose_tol_m` | m | 0.02 | 暫定（ss の依頼の間に機体が動いていないとみなす差） |
| `ss_pose_tol_rad` | rad | 0.02 | 暫定（同上） |
| `row_yaw_tol_rad` | rad | 0.17 | 暫定（約 10°。同じ列とみなす正面の向きの差） |
| `row_gap_max_m` | m | 0.3 | 暫定（同じ列とみなす隣との隙間） |

腕が振れる範囲の大きさは測定機構の側の値（[DetailedDesign.md](DetailedDesign.md) §2）。

## 6. コンテナ・ポート

| 名前 | 値 | 出どころ |
| --- | --- | --- |
| コンテナ | `fk_robot` | 2026-11-05 ユーザー承認（th の `th_robot` に揃えた） |
| `fk_rosbridge_port` | 9091 | 2026-11-05 ユーザー承認（th の 9090 と分ける。WP-AUTO-01 で th が使うポートと重ならないことを確かめる） |
| `fk_web_port` | 8091 | 2026-11-05 ユーザー承認（同上） |
| ワークスペース | リポジトリの `fk_ws/`（`fk_ws/src/<パッケージ>`） | 2026-11-05 ユーザー承認（th の `th_ws/` に揃えた） |
| th の型と UDP プロファイル | `../th-system` の `th_ws/src/th_system_msgs` と `th_ws/scripts/vibe_measure/fastdds_udp.xml` を読み取り専用でコンテナに入れる（写さない） | th DetailedDesign-measure.md §1 |
