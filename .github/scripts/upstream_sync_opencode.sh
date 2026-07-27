#!/usr/bin/env bash
# Resolve merge conflicts in-place via OpenCode CLI (no dispatch branch / PR).
set -euo pipefail

PROMPT_FILE="${1:-}"
FORK_BRANCH="${FORK_BRANCH:-master-c3}"
MODEL="${OPENCODE_MODEL:-opencode/deepseek-v4-flash}"
PROMPT_PATH="${RUNNER_TEMP:-/tmp}/upstream_oc_prompt.txt"

if [[ -z "${OPENCODE_API_KEY:-}" ]]; then
  echo "::error::OPENCODE_API_KEY is required"
  exit 1
fi

git fetch origin "$FORK_BRANCH" --depth=1 2>/dev/null || git fetch origin "$FORK_BRANCH"

if [[ -n "$PROMPT_FILE" && -f "$PROMPT_FILE" ]]; then
  cp "$PROMPT_FILE" "$PROMPT_PATH"
elif git show "origin/${FORK_BRANCH}:.github/scripts/prompts/upstream_sync_panda.txt" >"$PROMPT_PATH" 2>/dev/null; then
  :
elif git show "origin/${FORK_BRANCH}:.github/scripts/prompts/upstream_sync_opendbc.txt" >"$PROMPT_PATH" 2>/dev/null; then
  :
elif git show "origin/${FORK_BRANCH}:.github/scripts/prompts/upstream_sync_openpilot.txt" >"$PROMPT_PATH" 2>/dev/null; then
  :
else
  echo "::error::OpenCode prompt file not found"
  exit 1
fi

CONFLICT_LIST="$({
  git diff --name-only --diff-filter=U 2>/dev/null || true
  git grep -l -E '^(<{7}|>{7})' -- . 2>/dev/null || true
} | sort -u | head -50)"

if [[ -n "$CONFLICT_LIST" ]]; then
  {
    echo ""
    echo "以下文件仍有冲突，请全部处理："
    echo "$CONFLICT_LIST"
  } >>"$PROMPT_PATH"
fi

cat >>"$PROMPT_PATH" <<'EOF'

---
硬性约束（必须遵守）：
- 只在当前 git 分支上修改文件，解决所有 <<<<<<< / ======= / >>>>>>> 冲突标记
- 不要创建新分支，不要 git push，不要开 Pull Request
- 完成后对修改过的文件执行 git add
- 不要删除 C3 / DOS / tici / sunnypilot 适配目录来回避冲突
EOF

if ! command -v opencode >/dev/null 2>&1; then
  curl -fsSL https://opencode.ai/install | bash
  export PATH="$HOME/.opencode/bin:$PATH"
fi

mkdir -p "$HOME/.config/opencode"
cat >"$HOME/.config/opencode/opencode.json" <<EOF
{
  "\$schema": "https://opencode.ai/config.json",
  "model": "${MODEL}",
  "provider": {
    "opencode": {
      "options": {
        "apiKey": "${OPENCODE_API_KEY}"
      }
    }
  }
}
EOF

echo "OpenCode resolving conflicts on branch: $(git branch --show-current)"
timeout 900 opencode run --model "$MODEL" --auto "$(cat "$PROMPT_PATH")"
