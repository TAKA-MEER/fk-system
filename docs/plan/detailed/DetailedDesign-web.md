# 測定機能の画面（自律測定）

[DetailedDesign.md](DetailedDesign.md) の詳細。**`WP-AUTO-04`（画面）を実装できる形に落とす。**
入力は [Spec.md](../spec/Spec.md) §1.2（画面の入口）・[Spec-auto.md](../spec/Spec-auto.md) §2・§6〜§9・[-auto.md](DetailedDesign-auto.md)（機能側）と、
th-system [DetailedDesign-measure.md](../../../../th-system/docs/plan/detailed/DetailedDesign-measure.md) §4（画面の通り道）・th-system [DetailedDesign-webui.md](../../../../th-system/docs/plan/detailed/DetailedDesign-webui.md)（シェルの決まり）。
名前はすべて [-names.md](DetailedDesign-names.md)（`DD-1`）。

**見た目の正は [mockup/index.html](mockup/index.html)**（th のモックアップと同じ扱い。右下の操作台で段を切り替えられる）。
**状態: 案（2026-11-12）。**お立会試験の画面（停止命令の間。`S-21`）は測定機構の設計のあとで足す。

---

## 0. 結論

| 項目 | 決めたこと |
| --- | --- |
| 作り | **th の画面と同じ**（React 18・Vite 4・roslibjs。CDN を使わない）。パッケージ `fk_web` |
| th の部品 | **ヘッダー・CSS・ロゴ・W-1 の決め方（純関数と表）・文言の表・roslib・論理サイズを、`fk_ws/th_ref` のコミットから取り込む。写さない**（§1.2）。見た目と決まりが th とずれない |
| 本システムが書くもの | 画面の中身（配置図・操作カード・一覧）・本システムの文言・rosbridge 2 本のつなぎ・表示を決める純関数 |
| 配置 | **th の試験画面（S-20・S-21）と同じ 2 列。**左＝配置図、右＝次にやること・操作カード（停止／走行）・巡回・タブ（配電盤／結果／機器の状態）。手順バーは一番上に横一列（§3） |
| 通り道 | **本システムの rosbridge（9091）で機能側と、th の rosbridge（9090）で非常停止・在席・異常への応答・ヘッダーの中身・自己位置・地図**（§1.1） |
| 安全 | **非常停止は th の rosbridge へ直接送る**（機能側が落ちても届く）。**th の rosbridge が切れたら、この画面の非常停止が届かないことを帯で出す**（§5.2）。画面は `meas.*` を送らない |

---

## 1. 構成

### 1.1 通り道

| rosbridge | 名前 | 向き | 使い方 |
| --- | --- | --- | --- |
| **本システム**（`fk_rosbridge_port`） | `/fk/auto/state`・`/fk/auto/layout`・`/fk/auto/results` | 購読 | 操作カード・配置図・一覧（[-names.md](DetailedDesign-names.md) §3） |
| 〃 | `/fk/auto/command` | 呼ぶ | `auto.*`。返事が `accepted=false` なら理由をウィンドウで出す |
| **th**（`th_rosbridge_port`） | `/system/state`・`/safety/fault`・`/safety/active_faults` | 購読 | ヘッダー・W-1（th DetailedDesign-measure.md §4） |
| 〃 | `/measure/status` | 購読（`throttle_rate` 200 ms） | 配置図の機体の位置と向きだけ。**操作カードは機能側の `AutoState` で決める**（th の rosbridge が切れても機能側は th と ROS 2 でつながっているため） |
| 〃 | `/route/map_view` | 購読（`compression: 'cbor'`・`throttle_rate` 2000 ms） | 配置図の下の地図。**th の画面と同じ表示用の間引いた地図**（th `ros/routeMapTopicConfig.js`）。自律測定の間に配信されるかは th に確かめる（§9） |
| 〃 | `/ui/active_screen` | 出す（2 Hz） | 在席（§5.4） |
| 〃 | `/safety/estop_ui` | 出す | 非常停止（§5.4） |
| 〃 | `/system/trigger` | 呼ぶ | **W-1 への人の応答（`ui.resume_*`）だけ** |

- 接続先は `location.hostname` とポート（[-names.md](DetailedDesign-names.md) §6）から作る。IP を直書きしない。試験のときは URL の問い合わせでポートを替える（§7）。
- **画面は `meas.*` を送らない。**「走行」なども機能側を通す（[DetailedDesign.md](DetailedDesign.md) §2）。
- どちらの rosbridge も、切れたら 1 秒ごとにつなぎ直す。

### 1.2 th から取り込むもの

`fk_ws/scripts/import_th.sh` を広げ、**`fk_ws/th_ref` のコミットから `git archive` で `fk_ws/th_import/web/` に取り出す**（型と同じ。gitignore）。Vite の別名 `@th` で読む。th_ref を進めたら回し直す。

| th のファイル（`th_ws/web_ui/` の下） | 使い方 |
| --- | --- |
| `src/shell/theme.css` | **全体の CSS。**本システムの CSS はこの後に足す（後勝ち）。th のクラス（`.card`・`.opsgrid`・`.win`・`#release`・`.stop-banner` ほか）をそのまま使う |
| `src/shell/Header.jsx`・`logo.js` | **ヘッダーそのもの。**画面名「測定機能」・モード・ゾーン・異常の枠・非常停止 |
| `src/shell/limits.js` | `isW1Active`・`resumeChoices`・`showEstopRelease`・`headerTone`・`opButtonTone`（純関数） |
| `src/shell/stageMetrics.js` | 論理サイズ（横長 1280×720／縦長 960×1280）と倍率。`FixedStage.jsx` は th の設定画面に依存するので、同じ形を本システムで書く（向きは端末の縦横だけで決める） |
| `src/generated/attributes.json` | W-1 の選択肢（`resumeChoices`） |
| `src/i18n/modes.js`・`states.js`・`faults.js`・`reasons.js` | ヘッダー・W-1・解除バーの文言 |
| `public/roslib.min.js` | rosbridge のつなぎ（CBOR 対応の同梱版） |

**取り込んだファイルは書き換えない。**足りなければ th に頼む（§9）。

### 1.3 中身の作り

| 層 | 中身 | 試験 |
| --- | --- | --- |
| **表示を決める純関数**（`src/view/`） | `AutoState`・`Layout`・`PanelResults`・th の様子 → 段の表示・押せるボタン・青のボタン・帯・配置図の図形（§4・§5）。**React にも rosbridge にも触れない** | `node --test`（§4・§5 の表の行ごと） |
| 描画（`src/parts/`） | 純関数の結果を描くだけ | Playwright（ROS なし。試験用の様子を差し込む） |
| つなぎ（`src/ros/`） | rosbridge 2 本・非常停止の送り続け・在席 | 結合（コンテナ。§7） |

モックアップの `<script>` は、この純関数と同じ決め方で書いてある（`blueOf`・`bannerOf`・`paintOps` など）。

---

## 2. シェル（th と同じ）

| 決まり | 中身 | 出どころ |
| --- | --- | --- |
| 重なり順 | ヘッダー（非常停止）100 ＞ 非常停止の解除バー 70 ＞ ウィンドウ 50 ＞ 本文。**何を開いても非常停止と解除バーは押せる** | th `SD-3`・webui §2.1 |
| 非常停止 | 画面内のどのボタンよりも高い。押したら**この画面でラッチ**し、`true` を 2 Hz で送り続ける。解除は下端の解除バーで `false` を 1 回 | th Spec-safety.md §4.2・webui §2.2 |
| 論理サイズ | 横長 1280×720・縦長 960×1280 を等倍で拡大（余りは黒帯）。文字は表示領域の幅に追随 | th webui §2.3 |
| スクロール | **本文はスクロールさせない。**長い一覧は一覧の中だけスクロール | th `U-13` |
| 止まっている理由の帯 | th の `.stop-banner`（**下端に重ねる 1 行。本文を押し下げない**）。出すのは 1 本だけ（§5.2） | th `WS-9R` |
| ダイアログ | ブラウザ標準（`alert`・`confirm`）を使わない | th webui §6 |
| 表記 | 「盤」と略さず「配電盤」。説明文を置かない（状態・次にできる操作・拒否の理由だけ） | th webui §7.1・§7.2 |

---

## 3. 配置

| 場所 | 中身 |
| --- | --- |
| 上（2 列にまたがる） | **手順バー**: ① 配電盤を探す ② 測る配電盤を選ぶ ③ 回って測る ④ 結果（th の `UX-1`） |
| 左列の上 | 操作バー: **「終了」**（左）・段のピル（右）。**「終了」は操作カードに入れない**（th webui §4） |
| 左列 | **配置図**（§5.1）。列の残りの高さいっぱい。下に凡例 |
| 右列 ① | **次にやること**（th の `.next-action`）。押す操作が無い段は、同じ高さの状態表示にする |
| 右列 ② | **操作カード**: 停止（左上）・走行（右上）。th の格子のまま。下に状態（太字）と補足 |
| 右列 ③ | **巡回**: 選ぶ段は「選択 n 面 ／ 見つかった配電盤 n 面」と「すべて選ぶ」「選択を解除」「もう一度探す」。回る段は「測定済み・飛ばした・残り」と進み具合の帯と**「巡回を中止」**（2 回押し。1 回目で「もう一度押すと中止（10）」、10 秒で戻る。th の `ArmedButton`） |
| 右列 ④ | **タブ**: 配電盤（列ごとの一覧。選ぶ段は押して選ぶ）／結果（順・配電盤・結果と理由）／機器の状態（走行系・昇降機・測定機構の腕・画像認識・配置図・測定機能） |
| 縦長 | 配置図を上に全幅、その下に右列 ①〜③ と ④ を 2 列で並べる |

**①〜③ は段によって高さを変えない**（「停止」の位置を動かさない。th webui §4「置くボタンの位置は動かさない」）。

---

## 4. 段ごとの表示

| 段（`phase`） | 手順 | 次にやること | 巡回 | 配置図・一覧で選べる |
| --- | --- | --- | --- | --- |
| `IDLE` | — | 画面全体の代わりに「走行系の画面で『自律測定』を許可すると始まります」 | — | — |
| `PREPARE`・`SURVEY` | ① | 「周りの配電盤を探しています」 | 選ぶ形（押せない） | × |
| `SELECT` | ② | **「選んだ n 面で開始」**（`auto.start`）。配電盤が無ければ状態表示 | 選ぶ形 | ○ |
| `LOWER`・`GOAL`・`RECHECK`・`AT_PANEL` | ③ | 「自動で回っています」 | 回る形 | × |
| `PAUSED` | ③ | 「『走行』で続きから再開します」 | 回る形 | × |
| `FINISH` | ④ | 飛ばした盤があれば**「測れなかった n 面を選び直す」**（押すとその盤を選んで一覧へ）。選んでいれば「選んだ n 面で開始」 | 選ぶ形 | ○ |
| `REENTRY`・`HALTED` | 止まった手順を赤で | 「左上の『終了』で抜けてください」。「終了」を目立たせる | 回る形（押せない） | × |

| ボタン | 押せるとき（先読み。**権威は機能側**） | 押せないとき |
| --- | --- | --- |
| 停止（`auto.stop`） | `AutoState.th_state` が `NAV`・`ALIGN`・`BLOCKED`（機能側の `_RUNNING` と同じ） | th と同じく無効（破線） |
| 走行（`auto.run`） | 段が `PAUSED` | 無効。`REENTRY`・`HALTED`・測定機能と通信できないときは直下に理由バッジ（th の `UX-2-b`） |
| 巡回を中止（`auto.abort`） | 段が `GOAL`・`RECHECK`・`AT_PANEL`・`PAUSED` | 無効 |
| 開始・もう一度探す・選択 | 段が `SELECT`・`FINISH` | 無効 |
| 終了（`auto.finish`） | いつでも | — |

- **青（いまの状態）は 1 つだけ**: `th_state` が `NAV`・`ALIGN`・`BLOCKED` → 走行、`PAUSE` → 停止、ほかは無し。色と形は th の `opButtonTone()`（青＝いま／赤の丸太枠＝次に押す停止／白い太枠＝次に押す走行）。
- **測定機能と通信できない**（rosbridge が切れた・`/fk/auto/state` が `state_period_ms` の 3 倍届かない）ときは、段のピルを「不明」、状態を「測定機能と通信できません」、補足に最後に受け取った様子を出し、ボタンを無効にする。
- 機能側が `accepted=false` を返したら、`reject_reason` の文をウィンドウ（「この操作はできません」）で出す。

---

## 5. 配置図・帯・ウィンドウ

### 5.1 配置図

| 描くもの | 描き方 | 元 |
| --- | --- | --- |
| 地図 | 空き＝床色、塞がり＝明るい灰、分からない＝背景。1 m の目盛りと縮尺 | `/route/map_view` |
| 配電盤 | 天面の矩形（中心・正面の向き・幅・奥行き）。**正面の辺を白い太線**、中に `panel_id` | `Layout.panels` |
| 配電盤の状態 | 未選択＝灰、選択・順番待ち＝青、向かっている＝黄、測定済み＝緑、飛ばした＝赤の斜線。背面の右の角に印（✓ 選択／順番の数字／✓ 済み／✕ 飛ばした） | 画面の選択・`AutoState.queue`・`panel_id`・`PanelResults` |
| 確かでない | 破線・薄く。一覧に「位置が確かでない」 | `Panel.confident` |
| 正面が違うかも | 正面の外に「?」。一覧に「正面が違うかも」 | `Panel.front_doubt` |
| いま向かっている立ち位置 | 機体の外形（0.55 m 角）の破線の四角を点滅、機体から点線 | `AutoState.goal_*`（§8 で足す） |
| 機体 | 外形 0.55 m 角と向きの三角（th の地図と同じ） | `/measure/status` |
| 見ていない方向 | 全周を探した位置から、見ていない方位角の扇（半径は ss の最大距離 5 m）を斜線で。地図の範囲で切る | `Layout.unseen_*`・`survey_x`・`survey_y`（§8 で足す） |
| 地図の上の帯 | 探している間「周りの配電盤を探しています（全周）」／配置図が空・途中で終わった理由（`survey_reason`）＋「『もう一度探す』で探し直せます」／測定機能と通信できない間「最後に受け取った配置図です」 | `phase`・`Layout.survey_reason` |
| `REENTRY` | 地図を暗くして「この地図と配置図は使えません」 | `phase` |

- 向きは地図の座標のまま（x 右・y 上。回さない）。地図と配置図が収まるように合わせる。
- **選べる段では、配電盤を押すと選択が切り替わる**（一覧の行・「列を選ぶ」でも同じ）。選択は画面だけが持ち、「開始」で `panel_ids` として送る。

### 5.2 帯（1 本だけ。上が優先）

| # | 出るとき | 色 | 文 |
| --- | --- | --- | --- |
| 1 | **th の rosbridge が切れた** | 赤 | 走行系と通信できません。この画面の非常停止は届きません。機体の非常停止ボタンを使ってください。 |
| 2 | 測定機能と通信できない（§4） | 赤 | 測定機能と通信できません。この画面の操作は届きません（非常停止は届きます）。 |
| 3 | `REENTRY` | 赤 | 自己位置を見失いました。「終了」で抜けて、自律測定に入り直してください（測り終えた結果は残ります）。 |
| 4 | `HALTED` | 赤 | 止まっています：〈`halt_reason` の文〉。周りを確かめてから「終了」で抜けてください。 |
| 5 | `th_reject_reason` がある | 黄 | 走行系が指示を受けませんでした：〈理由の文〉。〈`PAUSED` なら「もう一度『走行』を押してください」、ほかは「配電盤を選び直して『開始』してください」〉。 |
| 6 | `gate_reason` がある | 黄 | 〈`gate_reason` の文〉。 |

### 5.3 ウィンドウ

| ウィンドウ | 出るとき | 中身と応答 |
| --- | --- | --- |
| **W-1**（異常・非常停止） | th の `isW1Active(mode, state, fault.active, pause_reason)`。th の AppShell と同じく、すぐ解消した異常でも出したままにする | 文言は th の表、選択肢は `resumeChoices(mode, attributes)`。押したら `/system/trigger` へ `ui.resume_*`。「非表示」でヘッダーに「再表示 ▲」。**本システムが自分で応答しない** |
| **W-4 終了** | 「終了」 | 腕が格納位置と読めている・測定機構なし → 「自律測定を終了しますか」（やめる／メインメニューへ）。**それ以外（読めないを含む）→ 「測定機構の腕が格納位置にあることを確かめましたか」**（th の「自律測定を終了」と同じ文言）。「メインメニューへ」で `auto.finish`（`arm_confirmed`）。受けられたら th の画面（`th_web_port`）へ移る |
| W-4 終了（測定機能と通信できない） | 同上 | 「走行系の画面の『自律測定を終了』で抜けてください」（やめる／走行系の画面へ） |
| この操作はできません | 機能側が拒否した | `reject_reason` の文（確認） |

W-2（手押し）・W-3（案内）・W-5（経路ブロック）・W-6（手動）は**出さない**（th の画面で応答する。§9 の問い 1・2）。手押しの間はヘッダーのモードが「手押し」になる。

### 5.4 在席・非常停止・開いたとき

| 項目 | 決まり |
| --- | --- |
| 在席 | `/ui/active_screen` を 2 Hz。`screen_id` は `S-60`（th のモードが `AUTO_MEASURE`）。`client_id` は `fk-` ＋ 乱数（タブを閉じるまで同じ）。`interacting` は画面が前面か、`last_input` は最後に触った時刻（th と同じ） |
| 非常停止の解除バー | th の `showEstopRelease(この画面で押した, mode, estop_ui)` で出す |
| 開いたとき | `AutoState.phase` が `IDLE` で th のモードが `AUTO_MEASURE` なら、`auto.begin` を 1 回送る（[Spec-auto.md](../spec/Spec-auto.md) §2「画面が開いたら自動で始める」） |
| 抜けたあと | th のモードが `AUTO_MEASURE` でなくなったら（th の画面から抜けた場合も）、§4 の `IDLE` の表示にする |

---

## 6. 文言

キーは [-names.md](DetailedDesign-names.md) §4・th DetailedDesign-measure.md §3.1・ss DetailedDesign-names.md §3 のまま。表示する文は `src/i18n/` に置く（**試験で全キーに文があることを確かめる**）。

| 種類 | キー → 文 |
| --- | --- |
| 段（状態／ピル） | `PREPARE` 準備しています／準備中・`SURVEY` 配電盤を探しています／探しています・`SELECT` 測る配電盤を選んでください／選んでください・`LOWER` 昇降機を下げています・`GOAL` 配電盤 〈ID〉 へ移動中／移動中・`RECHECK` 配電盤 〈ID〉 を確かめ直しています／確かめ直し中・`AT_PANEL` 配電盤 〈ID〉 の前で作業中／配電盤前・`PAUSED` 一時停止中・`FINISH` 選んだ配電盤を回り終えました／回り終えました・`REENTRY` 自己位置を見失いました／入り直しが要ります・`HALTED` 止まっています |
| 飛ばした理由 | `no_path` 経路が見つからない・`stand_blocked` 正面の床が塞がっていて立てない・`stand_mismatch` 立ち位置が合わない・`top_not_found` 天面が見つからない・`arm_space_blocked` 腕を振る場所が塞がっている・`no_mechanism` 測定機構が付いていない・`ss_failed` 画像認識の結果が得られない・`camera_too_low` カメラを必要な高さまで上げられない |
| 止めた理由 | `arm_stuck` 測定機構の腕が出たまま動けません・`lift_not_lowered` 昇降機を下げられません・`state_unreadable` 昇降機の高さか腕の状態が読めません・`ss_cleanup_timeout` 画像認識の後片付けが終わりません・`ss_link_lost` 画像認識と通信できません・`arm_link_lost` 測定機構の制御器と通信できません |
| 関門の理由 | `lift_not_bottom` 昇降機が下端に着くのを待っています・`arm_not_stowed` 測定機構の腕が格納位置に戻るのを待っています・`ss_busy` 画像認識が昇降機とカメラを使い終わるのを待っています・`state_unreadable` 走行系・昇降機・腕・画像認識のどれかの状態が読めないため、止めています・`reentry_required` 自律測定に入り直すまで動けません・`version_mismatch` 走行系とやり取りの版が合わないため、指示を出せません・`th_not_waiting` 走行系が指示待ちになるのを待っています・`arm_link_lost` 測定機構の制御器と通信できないため、止めています・`lift_drift` 昇降機が下端から離れたため止めました・`not_listed` 出せない指示を止めました（不具合です。記録してください） |
| th が拒否した理由 | `state_not_accepting` 走行系がいまは指示を受けられません・`turning` 旋回中です・`corner_obstacle` 機体の周りに障害物があります・`map_mismatch` 地図が作り直されています・`version_mismatch` やり取りの版が合いません・`link_lost` 測定機能の生存確認が届いていません・`localization_lost` 自己位置を見失っています・`bad_argument` 指示の中身が正しくありません・`th_unreachable` 走行系に指示が届きません |
| 配置図の理由（`survey_reason`） | `no_lugs` 吊り金具が見つかりませんでした・`lugs_ungrouped` 吊り金具を配電盤にまとめられませんでした・`no_image` カメラの画像が得られませんでした・`lift_refused` 昇降機を動かせませんでした・`camera_too_low` カメラを必要な高さまで上げられませんでした・`canceled` 途中で取り消しました・`timeout` 時間内に全周を見終わりませんでした・`human_override` 人が昇降機を操作したため中断しました・`ss_failed` 画像認識の結果が得られませんでした |
| 一時停止の理由（補足） | th の `pause_reason`: `jog` 手動操作が割り込みました・`fault` 異常で止まりました・`presence_lost` 操作する画面が離れたため止まりました。`gate_reason` が `lift_drift` なら「昇降機が下端から離れたため止めました」 |
| th の状態（機器の状態タブ） | `WAIT` 指示待ち・`NAV` 走行中・`ALIGN` 向きを合わせています・`BLOCKED` 塞がれて止まっています・`ADJUST` 配電盤の前で旋回中・`PAUSE` 一時停止 |
| 拒否（`reject_reason`） | `phase_not_accepting` いまはこの操作を受け付けません・`bad_argument` 配電盤を選んでください・`arm_not_confirmed` 測定機構の腕が格納位置にあることを確かめてください。関門の理由は上の表の文 |

---

## 7. 試験と受け入れ条件

| 層 | 確かめること（**どれもコマンドで判定できる形**） |
| --- | --- |
| 純関数（`node --test`） | §4 の表の行ごと（段 → 次にやること・押せるボタン・青）。§5.2 の優先順。§6 の全キーに文がある（[-names.md](DetailedDesign-names.md) §4.3 の一覧と突き合わせる） |
| 画面（Playwright。ROS なし） | 試験用の様子（`window.__fkTestState`）を差し込み、**両方の論理サイズ・全段で**: ① 本文がスクロールしない ② 非常停止が最も高いボタン ③ W-1 と W-4 を開いたまま非常停止と解除バーが押せる（`elementFromPoint`） ④ 「停止」の位置が段によって動かない ⑤ 操作カードの青が 1 つ以下 |
| 結合（コンテナ。`fake_th` 相手） | 試験用の起動で機能側（偽の昇降機・ss）・`fake_th`・rosbridge 2 本（試験用のポート）・画面の配信を立て、Playwright で: 画面を開く → 配置図が出る → 2 面選んで開始 → 結果に 2 行（`no_mechanism`）→ 終了 → `fake_th` が `meas.finish` を受ける。非常停止を押す → `fake_th` に `/safety/estop_ui` の `true` が 2 Hz で届き続ける → 解除で `false` |
| 変異（実装管理担当が回す） | 非常停止を 1 回だけ送るようにする → 結合が赤／解除バーの重なり順をウィンドウより下にする → 画面の ③ が赤／画面から `/measure/command` を呼ぶ道を足す → ソースの検査（`fk_web/src` に `/measure/command` が無い）が赤 |

---

## 8. 名前と型の追加（[-names.md](DetailedDesign-names.md) に入れた）

| 追加 | 理由 |
| --- | --- |
| `Layout.survey_x`・`survey_y` | 見ていない方向の扇の中心（全周を依頼した時点の機体の位置。map フレーム）。今の `Layout` には方位角しか無く、扇を描けない |
| `AutoState.goal_valid`・`goal_x`・`goal_y`・`goal_yaw` | いま向かっている立ち位置（`meas.goal` に出したもの）。周りの人に止まる場所を見せる |
| ポート `th_rosbridge_port`（9090）・`th_web_port`（5173） | th の rosbridge と th の画面（[-names.md](DetailedDesign-names.md) §6） |
| ファイル（`fk_web` 一式・`fk_ws/th_import/web/`・モックアップ・試験用の起動） | [-names.md](DetailedDesign-names.md) §7 |

`fake_th` が出すもの（`/system/state`・`/route/map_view`）と受けるもの（`/safety/estop_ui`・`/ui/active_screen`）の値は、WP-AUTO-04 のブリーフを書くときに [-names.md](DetailedDesign-names.md) §5.1 へ足す。

## 9. 未決と依存

| # | 問い・依存 | 案 | 止まるもの |
| --- | --- | --- | --- |
| 1 | **この画面に「手動」（ジョグ）を置くか。**th の S-60 には置いてある。仕様は「人はジョグで機体を動かせる」とだけ書き、どの画面からかは決めていない | **置かない**（操作カードの右下は空ける）。ジョグは th の画面で行う。th の通り道（th DetailedDesign-measure.md §4）にジョグが無く、置くなら th との相談が要る | — |
| 2 | **th の画面（S-60）へ移るボタンを置くか。**今は「終了」（抜ける）しか th へ戻る道が無い。手動・手押しへの応答は th の画面で行う | 操作バーの「終了」の右に小さく「走行系の画面へ」を置く（画面を移るだけで、抜けない） | — |
| 3 | **th の表に `AUTO_MEASURE` が無い**（th_ref `a33dd9a` の `modes.js`・`attributes.json`）。このままでは、ヘッダーのモードが「不明」になり、W-1 の選択肢も無い | th の 8-g で入る見込み（th Spec-webui.md §8.1 にはある）。入ったら th_ref を進める。**それまでは `fake_th` 相手の確認だけ**（申し送り。[Spec-open.md](../spec/Spec-open.md) §1.2） | 本物の th とつなぐ確認 |
| 4 | **自律測定の間に `/route/map_view`（表示用の間引いた地図）が配信されるか** | th に確かめる（申し送り）。配信されないなら `/map` を同じ設定（CBOR・間引き）で購読する | 地図の表示だけ |
