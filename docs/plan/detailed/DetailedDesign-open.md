# DetailedDesign-open — 未確定・申し送り・レビュー指摘

[DetailedDesign.md](DetailedDesign.md) の詳細。

---

## 1. 詳細設計側の未確定事項

[-auto.md](DetailedDesign-auto.md)（2026-11-04 初版）を書いて立った問い。**ユーザーの確認待ち。**

| # | 問い | 案 | 止まるもの |
| --- | --- | --- | --- |
| 1 | **昇降機の口の名前と形を、誰が書くか**。本システムが要るのは、読むもの（下端か・高さが読めるか・原点が取れているか・操作元・測定機構が付いているか・腕の制御器と通信できるか・腕が格納位置か）と、出すもの（下端まで下げる・止める）。昇降部の WS には、まだ腕の状態も目標の高さへの移動も無い | th と同じく **hve-system が書き、本システムが確かめる**（申し送り） | WP-AUTO-05 |
| 2 | **ss-system の通り道**（ss がどこで動き、何で通すか）。あわせて、**ss が実際に見た方向の範囲も返す**ことを ss の spec に足すか（[Spec-auto.md](../spec/Spec-auto.md) §2 の「見ていない方向」を出すのに要る。今の ss の spec は盤の一覧と理由だけ） | 通り道は決めてもらう。見た方向の範囲は ss の spec に足す（実装前なので直接書ける） | WP-AUTO-06 |
| 3 | **接頭辞・名前空間・ポート・コンテナ名**（[-names.md](DetailedDesign-names.md) §0・§6）: `fk_`・`/fk/<領域>/<名前>`・rosbridge 9091・画面 8091・`fk_robot` | 案のとおり | WP-AUTO-01 |

## 2. 仕様書からの申し送り

| 仕様書の ID | 詳細設計で要ること |
| --- | --- |
| `FK-A1`（移動）・`FK-A2`（PC 側） | **2026-11-04 決定**（th-system と同じ PC・同じ構成、コンテナとポートは別。移動の指示と状態は th-system Spec-measure.md §3 のとおり）。[DetailedDesign.md](DetailedDesign.md) §2 と [-names.md](DetailedDesign-names.md) §0 を書く。**th 側の名前は th-system が書き、2026-11-04 に確かめた**（[-names.md](DetailedDesign-names.md) §1）。腕が振れる範囲・画面の「走行」の通り道・ヘッダーの中身は [DetailedDesign.md](DetailedDesign.md) §2 に置いた |
| `FK-A1`（昇降機・画像認識）・`FK-A2`（アームの制御器のファーム） | 決まり次第、同上 |
| [Spec-auto.md](../spec/Spec-auto.md)（2026-11-04 新設） | **2026-11-04 に [-auto.md](DetailedDesign-auto.md) を書いた（案）。**元の申し送り: 自律測定の移動の詳細設計を書く。値は暫定（下げずに動ける長さ 0・止める閾値 50 mm・盤のピンの余裕 0・並進を使わない。出どころは [Spec-open.md](../spec/Spec-open.md) `FK-V1`・`FK-V5`・`FK-X7`。`DD-3` の「暫定」として書く）。仕様書が「何回か」「一定時間」とだけ書いたもの（経路の再計算の回数・ss-system の完了を待つ時間）は詳細設計で暫定値を置く |

## 3. レビュー指摘管理表

| # | 回 | 指摘 | 対応 |
| --- | --- | --- | --- |
| — | — | — | — |
