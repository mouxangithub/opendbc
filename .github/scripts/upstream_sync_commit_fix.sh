#!/usr/bin/env bash
set -euo pipefail

OUTPUT_BRANCH="${OUTPUT_BRANCH:-master-c3-new}"

if grep -r --include='*' -E '^<<<<<<< |^=======$|^>>>>>>> ' . \
  --exclude-dir=.git --exclude-dir=.venv 2>/dev/null | head -5; then
  echo "::error::Unresolved conflict markers remain"
  exit 1
fi

git add -A
if git diff --cached --quiet; then
  echo "Nothing to commit after conflict resolution"
  exit 0
fi

git commit -m "sync: resolve merge conflicts (OpenCode)"
