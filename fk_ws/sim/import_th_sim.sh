#!/usr/bin/env bash
# th-system の th_ws を、fk_ws/sim/th_sim_ref のコミットから取り出す（ホストで）。
# th の作業ツリーの今のブランチには頼らない（git archive はコミットから読む）。
# 取り出し先は fk_ws/sim/th_src/（.gitignore 済み）。th の場所は環境変数 TH_REPO
# （既定はリポジトリ直下から ../th-system）。
set -euo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SIM_DIR/../.." && pwd)"
TH_REPO="${TH_REPO:-$REPO/../th-system}"
REF="$(tr -d '[:space:]' < "$SIM_DIR/th_sim_ref")"

if ! git -C "$TH_REPO" cat-file -e "${REF}^{commit}" 2>/dev/null; then
  echo "ERROR: th のリポジトリ ($TH_REPO) にコミット $REF が無い。git -C $TH_REPO fetch してから回して" >&2
  exit 1
fi

# git archive は th_ws/ の中身だけを取り出す（th_src へ直接入れる）
rm -rf "$SIM_DIR/th_src"
mkdir -p "$SIM_DIR/th_src"
git -C "$TH_REPO" archive "$REF" th_ws | tar -x -C "$SIM_DIR/th_src" --strip-components=1
# 移した後に git が無いのは普通（archive から取ったもの）。
echo "$REF" > "$SIM_DIR/th_src/th_sim_ref"
echo "取り込んだ: th $REF -> fk_ws/sim/th_src"
