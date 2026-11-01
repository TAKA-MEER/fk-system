# fk-system

配電盤上部を確認するロボットに**測定機能（膜厚・板厚。同じ操作で測れる）**を加えるソフトウェア。
走行系（th-system）・昇降機とカメラ（hve-system）を使って自動での測定を目指し、各システムへの指示と情報の取得を行う。**走行の指示は本システムが出す。**
測定モードは**お立会試験中の測定**と**自律測定**の 2 つ。画面は走行系の画面を主とし、そこから入る。
開発体制は th-system・hve-system と同じにしてある。

**現在は設計段階。**コードはまだ無い。

| 知りたいこと | 見る文書 |
| --- | --- |
| 何を作るのか・どう振る舞うべきか | [docs/plan/spec/README.md](docs/plan/spec/README.md)（完成形の正本） |
| 何が決まっていないか | [docs/plan/spec/Spec-open.md](docs/plan/spec/Spec-open.md) |
| どう実装するか | [docs/plan/detailed/README.md](docs/plan/detailed/README.md) |
| 実装の順番・進み具合 | [docs/plan/ImplementationPlan.md](docs/plan/ImplementationPlan.md) |
| 作業のしかた（人・AI 共通） | [CLAUDE.md](CLAUDE.md)（codex は [AGENTS.md](AGENTS.md) から入る） |

関連リポジトリ（同じ `TM/` 配下）:

| リポジトリ | 中身 | 本システムとの関係 |
| --- | --- | --- |
| `../th-system` | 移動システム（走行部・WebUI・安全・教示再生・試験場内の移動） | 指示を出し情報を取る相手。**画面の主。**開発の手法はここが最も進んでいる |
| `../hve-system` | th-system の走行部に載る昇降機と上部カメラ | 指示を出し情報を取る相手（**カメラは ss-system が動かす。昇降機は本システムと ss-system の両方が動かす**）。走行を指示する権限は持たず、th-system とは直接やり取りしない（hve-system `HD-1`） |
| `../ss-system` | 画像認識（盤の配置・天面の位置の推定） | **欲しい情報を渡す相手。**情報を得るために hve-system のカメラを動かす。仕様は本システムの設計が決める（ss-system は汎用を意識する）。実装管理担当へ指示を出せる |
