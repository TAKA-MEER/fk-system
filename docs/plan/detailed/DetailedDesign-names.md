# 名前辞書

[DetailedDesign.md](DetailedDesign.md) の詳細。**この文書に無い名前を実装で作ってはいけない。**

> **`DD-1`**: 名前を発明する余地をゼロにする。必要な名前が無いと分かったら、
> **実装する前にこのファイルへ行を足す。**

**2026-10-30 時点で空。**構成（[Spec-open.md](../spec/Spec-open.md) `FK-A1`・`FK-A2`）が決まったら §0 の命名規則から埋める。

---

## 0. 命名規則

**th-system と同じ構成で作る**（2026-11-04 ユーザー決定。[Spec-open.md](../spec/Spec-open.md) `FK-A2`）ので、th-system [DetailedDesign-names.md](../../../../th-system/docs/plan/detailed/DetailedDesign-names.md) §0 に揃える
（パッケージ接頭辞だけを本システム用に決める。未定）。

## 1. 関連システム側の名前（参照のみ・こちらでは発明しない）

`FK-A1` で結合すると決まったら、結合に関わる相手側の名前を出典つきで載せる。**正は相手の文書とコード。**
**th-system とのやり取り（自律測定と、お立会試験の停止命令の間）の名前と形は th-system が書き、本システムが確かめる**（2026-11-04 ユーザー決定）。

| 相手 | 正 | 状態 |
| --- | --- | --- |
| th-system | th [DetailedDesign-measure.md](../../../../th-system/docs/plan/detailed/DetailedDesign-measure.md)（通り道 §1・名前の一覧 §2・型 §3・画面 §4・入口の分離 §5）。型は th `th_ws/src/th_system_msgs` を取り込み、版 `interface_version` で照合する | **2026-11-04 本システムが確かめ、ユーザーが承認。**自己位置を見失ったときに目的地を取り消して返すことを th に頼み、th が反映した（th `6f1ca35`。[Spec-open.md](../spec/Spec-open.md) §1.2）。版は `"1"` のまま（th は実装前の案の直しでは上げない） |

**名前は写さない**（写しは先方が直ったときに古いまま残る）。本システムの名前（パッケージ接頭辞など）は §0 が決まってから足す。
