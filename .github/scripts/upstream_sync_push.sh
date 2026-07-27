#!/usr/bin/env bash
# Push only master-c3-new — never push master-c3 from automation.
set -euo pipefail

FORK_BRANCH="${FORK_BRANCH:-master-c3}"
OUTPUT_BRANCH="${OUTPUT_BRANCH:-master-c3-new}"

current="$(git branch --show-current)"
if [[ "$current" != "$OUTPUT_BRANCH" ]]; then
  echo "::error::Refusing to push: on branch '$current', expected '$OUTPUT_BRANCH'"
  exit 1
fi

echo "Pushing ${OUTPUT_BRANCH} only (base ${FORK_BRANCH} is not modified by CI)"
git push origin "${OUTPUT_BRANCH}" --force-with-lease
