#!/usr/bin/env bash
# Merge sunnypilot upstream into fork branch, output master-c3-new for on-device testing.
set -euo pipefail

FORK_BRANCH="${FORK_BRANCH:-master-c3}"
OUTPUT_BRANCH="${OUTPUT_BRANCH:-master-c3-new}"
UPSTREAM_URL="${UPSTREAM_URL:?UPSTREAM_URL required}"
UPSTREAM_BRANCH="${UPSTREAM_BRANCH:-master}"
GITHUB_OUTPUT="${GITHUB_OUTPUT:-/dev/stdout}"

git config user.name "github-actions[bot]"
git config user.email "github-actions[bot]@users.noreply.github.com"

if git remote | grep -qx upstream; then
  git remote set-url upstream "$UPSTREAM_URL"
else
  git remote add upstream "$UPSTREAM_URL"
fi

git fetch origin "$FORK_BRANCH" --depth=1 || git fetch origin "$FORK_BRANCH"
git fetch upstream "$UPSTREAM_BRANCH" --depth=50

# Build sync result on OUTPUT_BRANCH only; FORK_BRANCH (master-c3) is read-only baseline.
git checkout -B "$OUTPUT_BRANCH" "origin/$FORK_BRANCH"

set +e
git merge "upstream/$UPSTREAM_BRANCH" --no-edit \
  -m "sync: merge ${UPSTREAM_URL##*/} ${UPSTREAM_BRANCH} into ${OUTPUT_BRANCH}"
MERGE_RC=$?
set -e

if [[ $MERGE_RC -ne 0 ]]; then
  echo "merge_conflicts=true" >> "$GITHUB_OUTPUT"
  echo "::warning::Merge conflicts — OpenCode step required"
  exit 0
fi

echo "merge_conflicts=false" >> "$GITHUB_OUTPUT"
echo "Merge clean"
