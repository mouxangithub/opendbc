#!/usr/bin/env bash
# Verify merge conflicts resolved and complete the merge commit.
set -euo pipefail

unmerged="$(git diff --name-only --diff-filter=U 2>/dev/null || true)"
if [[ -n "$unmerged" ]]; then
  echo "::error::Unmerged files remain:"
  echo "$unmerged"
  exit 1
fi

# Git conflict markers: <<<<<<< and >>>>>>> (avoid bare ======= false positives)
markers="$(git grep -n -E '^(<{7}|>{7})' -- . 2>/dev/null | head -30 || true)"
if [[ -n "$markers" ]]; then
  echo "::error::Conflict markers remain:"
  echo "$markers"
  exit 1
fi

git add -A

if ! git diff --cached --quiet; then
  git commit -m "sync: resolve merge conflicts (OpenCode)"
  echo "Merge commit created"
elif [[ -f .git/MERGE_HEAD ]]; then
  echo "::error::Merge still in progress but nothing staged to commit"
  git status --short
  exit 1
else
  echo "No staged changes; merge may already be complete"
fi
