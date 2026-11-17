#!/bin/bash
# Archive completed task/result pairs to .agents/history/
# Usage: ./archive.sh <agent> <task-id>
#   agent:   antigravity | kiro
#   task-id: timestamp or unique id used in the task file

AGENT=${1}
TASK_ID=${2}

if [ -z "$AGENT" ] || [ -z "$TASK_ID" ]; then
  echo "Usage: ./archive.sh <agent> <task-id>"
  echo "  agent:   antigravity | kiro"
  echo "  task-id: timestamp or unique id"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGENTS_DIR="$(dirname "$SCRIPT_DIR")"
HISTORY_DIR="$AGENTS_DIR/history"
SOURCE_DIR="$AGENTS_DIR/$AGENT"

if [ ! -d "$SOURCE_DIR" ]; then
  echo "Error: agent folder not found: $SOURCE_DIR"
  exit 1
fi

mkdir -p "$HISTORY_DIR"

# Archive task
if [ -f "$SOURCE_DIR/task.md" ]; then
  cp "$SOURCE_DIR/task.md" "$HISTORY_DIR/${TASK_ID}_${AGENT}_task.md"
  echo "Archived: ${TASK_ID}_${AGENT}_task.md"
fi

# Archive result
if [ -f "$SOURCE_DIR/result.md" ]; then
  cp "$SOURCE_DIR/result.md" "$HISTORY_DIR/${TASK_ID}_${AGENT}_result.md"
  echo "Archived: ${TASK_ID}_${AGENT}_result.md"
fi

echo "Done."
