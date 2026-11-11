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

| hve-system（昇降部） | hve [DetailedDesign-names.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-names.md)（2026-11-05 に案から移された）と [DetailedDesign-protocol-modules.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-protocol-modules.md)。作業パケットは hve の段階 6（`WP-LIFT-13`・`14` ほか）。今の実物は hve [DetailedDesign-protocol.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-protocol.md) §2。`busy_label` は UTF-8 で 48 バイトまで（超えると無しの扱い）。本システムは名前 `fk`・`operator: auto`・`busy_label`「測定操作中」で繋ぐ | **2026-11-05 本システムが確かめた。**頼み（測定機構の欄は分からないとき `null`）は hve が反映（hve `6411a6f`）。ss も確かめ、hve が作業パケットに切った（hve `2f415d3`・`7a5fe5a`）。**hve が実機に入れたと知らせるまで、この名前で実装しない・今の昇降部で上昇させない**（hve の指示） |

| ss-system | ss [DetailedDesign-locate.md](../../../../ss-system/docs/plan/detailed/DetailedDesign-locate.md)（口の振る舞い）と ss [DetailedDesign-names.md](../../../../ss-system/docs/plan/detailed/DetailedDesign-names.md) §0・§3（名前）。アクション・後片付けの完了・状態の 3 つ。型は ss の `ss_ws/src/ss_msgs` を読み取り専用で取り込み、版は状態の `interface_version` で照合する | **2026-11-05 本システムが確かめ、ss が記録した**（ss `9cd9764`）。ss の後片付けの上限は暫定 30 s（結果を返してから数える）。**型 `ss_msgs` は ss の `main` に入った**（ss `cf23438`。2026-11-05。2026-11-11 に ss の名前辞書と欄まで一致することを確かめた） |

**名前は写さない**（写しは先方が直ったときに古いまま残る）。hve の名前で本物の口（WP-AUTO-05）を作るのは、hve が実機に入れたと知らせてから。それまで [-auto.md](DetailedDesign-auto.md) §1.3 の昇降機の口は抽象のまま（偽物で試験する）。

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
| `CMD_LIFT_LOWER`（`"lift.lower"`）・`CMD_SS_REQUEST`（`"ss.request"`） | `gate` に渡す「昇降機を下げる」「ss への依頼」（`fk_auto.core` の定数。外へは出さない。2026-11-11 WP-AUTO-02） |
| `TH_INTERFACE_VERSION` | 取り込んだ th の型の版（`"1"`。th [DetailedDesign-measure.md](../../../../th-system/docs/plan/detailed/DetailedDesign-measure.md) §1.2）。生存確認に載せ、`/measure/status` の版と比べる。**`fk_ws/th_ref`（§7）を進めるときに一緒に見直す**（2026-11-11） |

外に出る文字列（段・理由・指示・パラメータ・トピック・型の欄）はこの文書にあるものだけを使う。**Python の中だけの名前（関数の引数・dataclass の欄・内部の列挙・試験の名前）は実装者が決めてよい**（2026-11-11）。パラメータを dataclass で受けるときは、欄の名前を §5 の名前に揃える。

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
| `Panel.msg` | `string panel_id` / `float64 x` / `float64 y`（天面の中心。map フレーム [m]）/ `float64 front_yaw`（正面の向き。盤から外向き [rad]）/ `float32 width_m` / `float32 depth_m` / `float32 top_height_m` / `float32 pos_std_m` / `float32 yaw_std_deg` / `int32 lugs`（ss の確からしさ。使った吊り金具の数。ss の名前に揃えた）/ `bool confident`（しきい値を満たすか）/ `bool ss_front_known`（ss が「正面が分かっている」と返した）/ `float64 ss_front_yaw`（そのときの ss の正面の向き。参考）/ `bool front_doubt`（ss の正面と、機体に向いていた辺が違う）/ `string row_id` |
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
| 飛ばす理由（`PanelResult.reason`） | `no_path`（経路の再計算をしても解けない）/ `stand_blocked`（正面側の床が塞がっていて立てない）/ `stand_mismatch`（認識し直した立ち位置が許容差を超える）/ `top_not_found`（天面が見つからない）/ `arm_space_blocked`（腕が振れる範囲が左右とも塞がっている。測定機構の口が返す）/ `no_mechanism`（測定機構が付いていない）/ `ss_failed`（ss の結果が時間内に来ない・使える結果が得られない）/ `camera_too_low`（ss が撮る高さまでカメラを上げられない。天井が低い・天井の値が古いなど） |
| 関門と見張りの理由（`AutoState.gate_reason`。`AutoCommand` の `reject_reason` にも使う） | §4.2 の `lift_not_bottom`・`arm_not_stowed`・`ss_busy`・`state_unreadable`（th の様子・昇降機の高さ・腕の状態・ss の状態のどれかが読めない）・`reentry_required`・`version_mismatch` に加えて、**2026-11-11 に足した**: `th_not_waiting`（ss への依頼で th が `WAIT` でない）/ `arm_link_lost`（測定機構があるのに腕の制御器と通信できない。止める理由と同じキー）/ `not_listed`（関門の表に無い指示。[-auto.md](DetailedDesign-auto.md) §4.1）/ `lift_drift`（走っている間に昇降機が下端から `lift_drift_stop_mm` 以上離れた。見張り §4.2） |
| 止める理由（`halt_reason`） | `arm_stuck`（腕が出たまま動けない）/ `lift_not_lowered`（昇降機を下げられない）/ `state_unreadable`（昇降機の高さか腕の状態が読めない）/ `ss_cleanup_timeout`（ss の後片付けの完了が来ない）/ `ss_link_lost`（ss と通信できない）/ `arm_link_lost`（腕が出ている間に腕の制御器と通信できない） |

## 5. パラメータ（`fk_bringup` の `params.yaml`。`auto_runner`）

| 名前 | 単位 | 値 | 出どころ |
| --- | --- | --- | --- |
| `heartbeat_period_ms` | ms | 200 | 先方（th の生存確認 5 Hz。th DetailedDesign-measure.md §3.4） |
| `runner_stall_ms` | ms | 500 | 暫定（状態機械がこれより長く回らなければ生存確認を止める） |
| `runner_period_ms` | ms | 100 | 暫定（状態機械を回すタイマーの周期。`runner_stall_ms` より十分短く。2026-11-11 に足した） |
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
| `ss_status_stale_ms` | ms | 1000 | 先方（ss の勧め。ss の状態は 200 ms ごと。その 5 回分。届かなければ `ss_link_lost`） |
| `ss_pos_std_max_m` | m | 0.15 | 暫定（盤を「確か」とみなす位置の標準偏差の上限。ss の配置図向けの精度の目標 ±15 cm に揃えた） |
| `ss_yaw_std_max_deg` | ° | 5 | 暫定（同じく向き。ss の目標 ±5° に揃えた） |
| `ss_cleanup_timeout_s` | s | 60 | 暫定（仕様は「一定時間」。結果か取り消しのあと、後片付けの完了を待つ上限。超えたら `ss_cleanup_timeout`。依頼から数えると、本システムが待つのは最長で `ss_result_timeout_s` ＋ これ。**ss 自身の上限 `cleanup_timeout_s`（暫定 30 s。結果を返してから数える）より長くしておく**こと。ss は見直して 60 s に近づくときは先に知らせる） |
| `ss_retry_max` | 回 | 2 | 暫定（ss の結果が使えないとき依頼し直す上限） |
| `ss_pose_tol_m` | m | 0.02 | 暫定（ss の依頼の間に機体が動いていないとみなす差） |
| `ss_pose_tol_rad` | rad | 0.02 | 暫定（同上） |
| `row_yaw_tol_rad` | rad | 0.17 | 暫定（約 10°。同じ列とみなす正面の向きの差） |
| `row_gap_max_m` | m | 0.3 | 暫定（同じ列とみなす隣との隙間） |
| `footprint_half_m` | m | 0.275 | 先方（th の機体の外形 0.55 m 角の半分。th [DetailedDesign-measure.md](../../../../th-system/docs/plan/detailed/DetailedDesign-measure.md) §3.5）。`stand_blocked` が立ち位置に置く機体の形（正方形）。2026-11-11 に足した |
| `map_occupied_min` | — | 1 | 暫定（`/map` のマスの値がこれ以上なら塞がっているとみなす。slam_toolbox は 0・100・−1 しか出さないので、0 より大きければ塞がっている。−1（分からない）は塞がっていないとみなす（[-auto.md](DetailedDesign-auto.md) §3 `stand_blocked`）。2026-11-11 に足した） |

腕が振れる範囲の大きさは測定機構の側の値（[DetailedDesign.md](DetailedDesign.md) §2）。

### 5.1 偽の th（`fk_sim` の `fake_th`）

試験でだけ使う。th の約束（th [DetailedDesign-measure.md](../../../../th-system/docs/plan/detailed/DetailedDesign-measure.md)）どおりに振る舞うための値。2026-11-11 に足した。

| 名前 | 単位 | 既定 | 出どころ |
| --- | --- | --- | --- |
| `interface_version` | — | `"1"` | 先方（th の版）。試験で食い違いを作るときに変える |
| `measure_link_timeout_ms` | ms | 1000 | 暫定（th の名前に揃えた。th の値は実装時に決まる（th §3.4 の (b)）。決まったら揃える） |
| `status_period_ms` | ms | 100 | 先方（th の `/measure/status` は 10 Hz。th §2） |

## 6. コンテナ・ポート

| 名前 | 値 | 出どころ |
| --- | --- | --- |
| コンテナ | `fk_robot` | 2026-11-05 ユーザー承認（th の `th_robot` に揃えた） |
| `fk_rosbridge_port` | 9091 | 2026-11-05 ユーザー承認（th の 9090 と分ける。WP-AUTO-01 で th が使うポートと重ならないことを確かめる） |
| `fk_web_port` | 8091 | 2026-11-05 ユーザー承認（同上） |
| ワークスペース | リポジトリの `fk_ws/`（`fk_ws/src/<パッケージ>`） | 2026-11-05 ユーザー承認（th の `th_ws/` に揃えた） |
| th の型と UDP プロファイル | `../th-system` の `th_ws/src/th_system_msgs` と `th_ws/scripts/vibe_measure/fastdds_udp.xml` を、**`fk_ws/th_ref` に書いたコミットから**取り出して使う（§7。写してコミットしない。th の作業ツリーの今のブランチに頼らない。2026-11-11） | th DetailedDesign-measure.md §1 |
| `ROS_DOMAIN_ID`（試験） | 78 | 暫定（2026-11-11。実機の 10 と、th の試験が使う 77 と分ける。th `CLAUDE.md` 2026-09-23） |

## 7. ファイル・ディレクトリ

**th-system の `th_ws/` の作りに揃える**（2026-11-11）。パスはリポジトリ直下から。

| パス | 中身 | git |
| --- | --- | --- |
| `fk_ws/Dockerfile` | `ros:humble-ros-base` が元。th の `th_ws/Dockerfile` に揃え、要るものだけ入れる | 入れる |
| `fk_ws/docker-compose.yml` | サービス・コンテナ `fk_robot`（`network_mode: host`・`ROS_DOMAIN_ID=10`・FastDDS は UDP だけ） | 入れる |
| `fk_ws/th_ref` | 取り込む th のコミット（全桁 1 行。いまは th `feat/era3-measure` の `a33dd9a`） | 入れる |
| `fk_ws/scripts/import_th.sh` | ホストで回す。`fk_ws/th_ref` のコミットから th の型と UDP プロファイルを `git archive` で取り出す（th の場所の既定は `../th-system`） | 入れる |
| `fk_ws/src/th_system_msgs/` | 取り込み先（th の型） | **入れない**（`.gitignore`） |
| `fk_ws/th_import/` | 取り込み先: UDP プロファイル `fastdds_udp.xml` と、取り出したコミット `th_ref`（`fk_ws/th_ref` と合っているかを `test.sh` が見る） | **入れない**（`.gitignore`） |
| `fk_ws/scripts/test.sh` | コンテナの中で、取り込みが `th_ref` と合っているかの確認・`colcon build`・試験 | 入れる |
| `fk_ws/src/fk_msgs/` | §4.1 の型 | 入れる |
| `fk_ws/src/fk_auto/fk_auto/core.py` | 純関数と `Snapshot`（WP-AUTO-02） | 入れる |
| `fk_ws/src/fk_auto/fk_auto/auto_runner.py` | ノード `auto_runner` | 入れる |
| `fk_ws/src/fk_auto/test/` | 試験。`conftest.py` がホストの pytest から `fk_auto` を import できるようにする | 入れる |
| `fk_ws/src/fk_sim/fk_sim/fake_th.py` | ノード `fake_th` | 入れる |
| `fk_ws/src/fk_sim/test/` | `fake_th` を相手に ROS 2 で通す結合試験（コンテナの中で回す） | 入れる |
| `fk_ws/src/fk_bringup/config/params.yaml` | §5 の値（ノード名 `auto_runner` の下） | 入れる |
| `fk_ws/src/fk_bringup/launch/fk.launch.py` | 実機用の起動（`auto_runner`） | 入れる |
