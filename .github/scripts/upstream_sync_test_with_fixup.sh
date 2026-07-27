#!/usr/bin/env bash
# Run CI tests; on failure invoke OpenCode to fix and retry (bounded).
set -euo pipefail

TEST_SCRIPT="${1:?test command required}"
PROMPT_FILE="${2:?prompt file required}"
MAX_FIX_ATTEMPTS="${MAX_FIX_ATTEMPTS:-2}"
LOG="${RUNNER_TEMP:-/tmp}/upstream_test.log"

run_test() {
  set +e
  bash -c "$TEST_SCRIPT" 2>&1 | tee "$LOG"
  local rc=${PIPESTATUS[0]}
  set -e
  return "$rc"
}

if run_test; then
  echo "Tests passed on first run"
  exit 0
fi

for ((fix = 1; fix <= MAX_FIX_ATTEMPTS; fix++)); do
  echo "::warning::Tests failed; OpenCode fix attempt ${fix}/${MAX_FIX_ATTEMPTS}"
  bash .github/scripts/upstream_sync_opencode_test_fix.sh "$PROMPT_FILE" "$LOG"

  if ! git diff --quiet || ! git diff --cached --quiet; then
    git add -A
    git commit -m "sync: fix CI test failures (OpenCode attempt ${fix})"
  fi

  if run_test; then
    echo "Tests passed after OpenCode fix attempt ${fix}"
    exit 0
  fi
done

echo "::warning::Tests still failing after ${MAX_FIX_ATTEMPTS} OpenCode fix attempts — will push branch and open PR for human review"
tail -120 "$LOG" || true
exit 1
