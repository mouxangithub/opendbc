#!/usr/bin/env bash
# Open or update draft PR: master-c3-new → master-c3 (manual merge after device test).
set -euo pipefail

FORK_BRANCH="${FORK_BRANCH:-master-c3}"
OUTPUT_BRANCH="${OUTPUT_BRANCH:-master-c3-new}"
REPO_TITLE="${1:-sync: upstream → master-c3-new}"
SYNC_TEST_PASSED="${SYNC_TEST_PASSED:-true}"

if ! command -v gh >/dev/null 2>&1; then
  echo "::error::gh CLI required"
  exit 1
fi

PR_TITLE="$REPO_TITLE"
if [[ "$SYNC_TEST_PASSED" != "true" ]]; then
  PR_TITLE="[需人工修复] ${REPO_TITLE}"
fi

existing="$(gh pr list --head "${OUTPUT_BRANCH}" --base "${FORK_BRANCH}" --json number,state -q '.[0].number' 2>/dev/null || true)"

BODY="$(cat <<EOF
## 上游同步（仅 \`${OUTPUT_BRANCH}\`，不直接改 \`${FORK_BRANCH}\`）

- CI 已将 sunnypilot 上游合并结果推到分支 **\`${OUTPUT_BRANCH}\`**
- **\`${FORK_BRANCH}\` 不会被 workflow 直接 push 或 merge**
- 本 PR 为 **Draft**，测车通过前请勿合并

## 请你自测

1. \`git fetch origin && git checkout ${OUTPUT_BRANCH}\`
2. 编译 / replay / 上路验证
3. 将本 PR 标记为 **Ready for review**，审阅通过后 **手动 Merge** 到 \`${FORK_BRANCH}\`

OpenCode 仅做审阅建议，上游同步 PR **不会自动 merge**。
EOF
)"

if [[ "$SYNC_TEST_PASSED" != "true" ]]; then
  log_tail=""
  if [[ -n "${SYNC_TEST_LOG:-}" && -f "$SYNC_TEST_LOG" ]]; then
    log_tail="$(tail -80 "$SYNC_TEST_LOG")"
  fi
  BODY="${BODY}

## ⚠️ CI 测试未通过（需人工审核）

OpenCode 已尝试自动修复但仍未通过。请 checkout \`${OUTPUT_BRANCH}\` 本地修复后 push 到同一分支，或在本 PR 上继续提交。

${log_tail:+**失败日志（最近 80 行）：**
\`\`\`
${log_tail}
\`\`\`}"
fi

LABELS=(ai-auto-review upstream-sync)
if [[ "$SYNC_TEST_PASSED" != "true" ]]; then
  LABELS+=(needs-human-review)
fi
label_args=()
for label in "${LABELS[@]}"; do
  label_args+=(--label "$label")
done

if [[ -n "$existing" ]]; then
  echo "Updating existing PR #${existing}"
  gh pr edit "$existing" --title "$PR_TITLE" --body "$BODY" "${label_args[@]}" 2>/dev/null || true
else
  gh pr create --draft --base "${FORK_BRANCH}" --head "${OUTPUT_BRANCH}" \
    --title "$PR_TITLE" --body "$BODY" "${label_args[@]}" || true
fi

if [[ -n "${OPEN_PR_EXTRA_BODY:-}" ]]; then
  pr_num="$(gh pr list --head "${OUTPUT_BRANCH}" --base "${FORK_BRANCH}" --json number -q '.[0].number' 2>/dev/null || true)"
  if [[ -n "$pr_num" ]]; then
    gh pr edit "$pr_num" --body "${BODY}

${OPEN_PR_EXTRA_BODY}" || true
  fi
fi
