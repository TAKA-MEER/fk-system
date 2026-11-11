#!/usr/bin/env bash
# th-system の型（th_system_msgs）と UDP プロファイルを、fk_ws/th_ref のコミットから取り出す。
# ホストで回す。th の作業ツリーの今のブランチには頼らない（git archive はコミットから読む）。
# th の場所: 環境変数 TH_REPO（既定はリポジトリ直下から ../th-system）。
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "$WS/.." && pwd)"
TH_REPO="${TH_REPO:-$REPO/../th-system}"
REF="$(tr -d '[:space:]' < "$WS/th_ref")"

if ! git -C "$TH_REPO" cat-file -e "${REF}^{commit}" 2>/dev/null; then
  echo "ERROR: th のリポジトリ ($TH_REPO) にコミット $REF が無い。git -C $TH_REPO fetch してから回して" >&2
  exit 1
fi

rm -rf "$WS/src/th_system_msgs" "$WS/th_import"
mkdir -p "$WS/src" "$WS/th_import"

# th_ws/src/th_system_msgs -> fk_ws/src/th_system_msgs/（先頭 3 階層を落とす）
git -C "$TH_REPO" archive "$REF" th_ws/src/th_system_msgs | tar -x -C "$WS/src" --strip-components=2
# UDP プロファイル
git -C "$TH_REPO" archive "$REF" th_ws/scripts/vibe_measure/fastdds_udp.xml \
  | tar -x -C "$WS/th_import" --strip-components=3
echo "$REF" > "$WS/th_import/th_ref"
echo "取り込んだ: th $REF -> fk_ws/src/th_system_msgs, fk_ws/th_import/fastdds_udp.xml"
