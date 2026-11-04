# DetailedDesign-open — 未確定・申し送り・レビュー指摘

[DetailedDesign.md](DetailedDesign.md) の詳細。

---

## 1. 詳細設計側の未確定事項

なし（仕様書の対象・構成が決まっていないため、まだ詳細設計の問いが立たない）。

## 2. 仕様書からの申し送り

| 仕様書の ID | 詳細設計で要ること |
| --- | --- |
| `FK-A1`（移動）・`FK-A2`（PC 側） | **2026-11-04 決定**（th-system と同じ PC・同じ構成、コンテナとポートは別。移動の指示と状態は th-system Spec-measure.md §3 のとおり）。[DetailedDesign.md](DetailedDesign.md) §2 と [-names.md](DetailedDesign-names.md) §0 を書く。**th 側の名前は th-system が書き、2026-11-04 に確かめた**（[-names.md](DetailedDesign-names.md) §1）。腕が振れる範囲・画面の「走行」の通り道・ヘッダーの中身は §2 に置いた |
| `FK-A1`（昇降機・画像認識）・`FK-A2`（アームの制御器のファーム） | 決まり次第、同上 |
| [Spec-auto.md](../spec/Spec-auto.md)（2026-11-04 新設） | **自律測定の移動の詳細設計を書く。**値は暫定（下げずに動ける長さ 0・止める閾値 50 mm・盤のピンの余裕 0・並進を使わない。出どころは [Spec-open.md](../spec/Spec-open.md) `FK-V1`・`FK-V5`・`FK-X7`。`DD-3` の「暫定」として書く）。仕様書が「何回か」「一定時間」とだけ書いたもの（経路の再計算の回数・ss-system の完了を待つ時間）は詳細設計で暫定値を置く |

## 3. レビュー指摘管理表

| # | 回 | 指摘 | 対応 |
| --- | --- | --- | --- |
| — | — | — | — |
