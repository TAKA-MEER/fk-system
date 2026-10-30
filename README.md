# fk-system

**何を作るかは未定**（[Spec-open.md](docs/plan/spec/Spec-open.md) `FK-S1`）。
th-system・hve-system と関連するソフトウェアで、開発体制はその 2 つと同じにしてある。

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
| `../th-system` | 移動システム（走行部・WebUI・安全・教示再生・試験場内の移動） | 未定（`FK-A1`）。**開発の手法はここが最も進んでいる** |
| `../hve-system` | th-system の走行部に載る昇降機と上部カメラ | 未定（`FK-A1`） |
