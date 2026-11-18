#!/usr/bin/env bash
# ss-system の型（ss_msgs）を、fk_ws/ss_ref のコミットから取り出す。
# ホストで回す。ss の作業ツリーの今のブランチには頼らない（git archive はコミットから読む）。
# ss の場所: 環境変数 SS_REPO（既定はリポジトリ直下から ../ss-system）。
# th の import_th.sh と同じ作り（fk_ws/ss_import/ss_ref で取り込んだコミットを残す）。
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "$WS/.." && pwd)"
SS_REPO="${SS_REPO:-$REPO/../ss-system}"
REF="$(tr -d '[:space:]' < "$WS/ss_ref")"

if ! git -C "$SS_REPO" cat-file -e "${REF}^{commit}" 2>/dev/null; then
  echo "ERROR: ss のリポジトリ ($SS_REPO) にコミット $REF が無い。git -C $SS_REPO fetch してから回して" >&2
  exit 1
fi

rm -rf "$WS/src/ss_msgs" "$WS/ss_import"
mkdir -p "$WS/src" "$WS/ss_import"

# ss_ws/src/ss_msgs -> fk_ws/src/ss_msgs/（先頭 2 階層 ss_ws/src を落とす）
git -C "$SS_REPO" archive "$REF" ss_ws/src/ss_msgs | tar -x -C "$WS/src" --strip-components=2
echo "$REF" > "$WS/ss_import/ss_ref"
echo "取り込んだ: ss $REF -> fk_ws/src/ss_msgs"
