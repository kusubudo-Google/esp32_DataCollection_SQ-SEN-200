#!/usr/bin/env bash
# 当 src/main.cpp 里的 FW_VERSION 与已提交版本不同时,自动 commit + push。
# 由 .claude/settings.json 的 Stop hook 调用。版本没变时什么都不做。
set -uo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root" || exit 0

file="src/main.cpp"
re='#define[[:space:]]+FW_VERSION[[:space:]]+"[^"]*"'

working="$(grep -oE "$re" "$file" 2>/dev/null | head -1)"
committed="$(git show HEAD:"$file" 2>/dev/null | grep -oE "$re" | head -1)"

[ -z "$working" ] && exit 0                 # 找不到版本行,放弃
[ "$working" = "$committed" ] && exit 0     # 版本没变,不动

ver="$(printf '%s' "$working" | sed -E 's/.*"([^"]*)".*/\1/')"

git add -A || exit 0
git commit -m "chore: release ${ver}" >/dev/null 2>&1 || exit 0

if git push >/dev/null 2>&1; then
  printf '{"systemMessage": "FW_VERSION %s: committed and pushed to git"}\n' "$ver"
else
  printf '{"systemMessage": "FW_VERSION %s: committed locally, git push failed"}\n' "$ver"
fi
exit 0
