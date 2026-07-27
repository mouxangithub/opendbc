#!/usr/bin/env bash
# Ask OpenCode to fix CI test failures using the captured log.
set -euo pipefail

PROMPT_FILE="${1:?prompt file required}"
TEST_LOG="${2:?test log required}"
MODEL="${OPENCODE_MODEL:-opencode/deepseek-v4-flash}"
PROMPT_PATH="${RUNNER_TEMP:-/tmp}/upstream_oc_test_prompt.txt"

if [[ -z "${OPENCODE_API_KEY:-}" ]]; then
  echo "::error::OPENCODE_API_KEY is required for test auto-fix"
  exit 1
fi

if [[ ! -f "$PROMPT_FILE" ]]; then
  echo "::error::Prompt file not found: $PROMPT_FILE"
  exit 1
fi

cp "$PROMPT_FILE" "$PROMPT_PATH"
{
  echo ""
  echo "## CI 测试失败日志（最近 250 行）"
  tail -250 "$TEST_LOG" 2>/dev/null || cat "$TEST_LOG"
  echo ""
  echo "## 当前 git 状态"
  git branch --show-current
  git status --short | head -60
} >>"$PROMPT_PATH"

cat >>"$PROMPT_PATH" <<'EOF'

---
硬性约束（必须遵守）：
- 只修改当前仓库源码，使上述 CI 测试通过
- 不要创建新分支，不要 git push，不要开 Pull Request
- 保留 master-c3 / C3 / sunnypilot fork 适配，不要删目录来回避问题
- 修完后对修改过的文件执行 git add
- 若存在 <<<<<<< 冲突标记，一并清理
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

echo "OpenCode fixing test failures on branch: $(git branch --show-current)"
timeout 900 opencode run --model "$MODEL" --auto "$(cat "$PROMPT_PATH")"
