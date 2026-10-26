#!/bin/bash
# Reset all agent statuses to idle and clear task/result files
# Usage: ./reset.sh [agent]
#   agent: antigravity | kiro | all (default: all)

AGENT=${1:-all}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGENTS_DIR="$(dirname "$SCRIPT_DIR")"

reset_agent() {
  local name=$1
  local dir="$AGENTS_DIR/$name"

  if [ ! -d "$dir" ]; then
    echo "Warning: agent folder not found: $dir"
    return
  fi

  # Reset status
  sed -i.bak 's/\*\*Status:\*\* .*/\*\*Status:\*\* idle/' "$dir/status.md" 2>/dev/null || \
    echo "**Status:** idle" > "$dir/status.md"
  rm -f "$dir/status.md.bak"

  # Clear task
  echo "<!-- Cleared by reset.sh -->" > "$dir/task.md"

  # Clear result
  echo "<!-- Cleared by reset.sh -->" > "$dir/result.md"

  echo "Reset: $name"
}

if [ "$AGENT" = "all" ]; then
  reset_agent "antigravity"
  reset_agent "kiro"
else
  reset_agent "$AGENT"
fi

echo "Done."
