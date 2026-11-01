# docs/plan/source — 原典・先行試作・関連リポジトリへの参照

spec に統合する前の材料。**仕様の正ではない。**spec に取り込んだら、取り込んだ先を §3 に書く。

## 1. 先行試作・原典

**まだ無い。**別リポジトリにあるものは**写さずに**場所だけ書く（複製すると、元が直ったときに古い写しが残る）。

| 材料 | 場所 | 中身 |
| --- | --- | --- |
| 測定機構のスケッチ（2026-10-30・ユーザー作） | [measure-arm-sketch-2026-10-30.jpg](measure-arm-sketch-2026-10-30.jpg)（原画は `~/ドキュメント/MIRS/fig/image.jpg`） | 跳ね橋のような腕で膜厚センサを天面へ近づける 3 コマ。説明は [arm.md](../arm.md) |
| 測定機構のスケッチ（2026-11-01・ユーザー作。仮） | [measure-arm-sketch-2026-11-01.png](measure-arm-sketch-2026-11-01.png) | 腕を起こし・伸ばし・大まかに下げ・精密に当てる 5 コマ。説明は [arm.md](../arm.md) |

## 2. 関連リポジトリ

| リポジトリ | 入口 | 中身 |
| --- | --- | --- |
| th-system | `../th-system/docs/plan/spec/README.md` | 移動システムの完全設計書（正本） |
| hve-system | `../hve-system/docs/plan/spec/README.md` | 昇降機・上部カメラの完全設計書（正本） |
| ss-system | `../ss-system/docs/plan/spec/README.md` | 画像認識の完全設計書（正本）。先行試作は `../../lifting-lug-pose`（git 管理外） |

パスはリポジトリ直下（`fk-system/`）基準。

## 3. spec への取り込み状況

| 材料 | 取り込み先 |
| --- | --- |
| 測定機構のスケッチ（2026-10-30・2026-11-01） | [arm.md](../arm.md)（検討メモ）。決まった部分は [Spec.md](../spec/Spec.md) §1.3（2026-10-30） |
