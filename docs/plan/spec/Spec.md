# 完全設計書 — fk-system

**実装（ノード名・トピック名・ピン・ファイル構成）は書かない。次工程の詳細設計に回す。**
読み方・位置づけは [README.md](README.md)。

> **2026-10-30 時点で骨格のみ。**出どころのある事実だけを書き、決まっていない所は
> `未定` とし [Spec-open.md](Spec-open.md) の ID を付けた。**推測で埋めない**
> （推測は次のセッションで別の推測になる）。

---

## 1. 対象

**未定**（[Spec-open.md](Spec-open.md) `FK-S1`）。

関連するシステムの担当範囲（出典の写し。**原文が正**）:

| システム | 範囲 | 出典 |
| --- | --- | --- |
| th-system | 配電盤上部を確認するロボットのうち、移動に関わるすべて（WebUI・始業点検・校正・異常時の扱いを含む）。範囲外はカメラ昇降・盤前の低速並進・ライン誘導走行のマップ編集画面 | th-system [Spec.md](../../../../th-system/docs/plan/spec/Spec.md) §1 |
| hve-system | th-system の走行部に載る昇降機と上部カメラ | hve-system [Spec.md](../../../../hve-system/docs/plan/spec/Spec.md) §1 |
| **本システム** | 未定（`FK-S1`） | — |

---

## 2. 用語

関連システムと同じ語は th-system [Spec.md](../../../../th-system/docs/plan/spec/Spec.md) §2 をそのまま使う（本システムが同じ現場で使われるなら）。
本システムで新たに要る語は、対象が決まってから足す。

---

## 3. 目標

**未定**（[Spec-open.md](Spec-open.md) `FK-G1`）。

## 4. 設計思想

**未定。**th-system の設計思想（SD-1〜SD-9）を引き継ぐかどうかから決める（`FK-G1`）。

## 5. 全体構成

**未定**（[Spec-open.md](Spec-open.md) §2.2）。

## 6. 目次

| 章 | ファイル | 状態 |
| --- | --- | --- |
| 未確定事項・関連システムとの関係 | [Spec-open.md](Spec-open.md) | 初版 |
| 領域ごとの詳細 | [README.md](README.md) §2 | 未作成 |
