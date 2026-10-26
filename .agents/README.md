# Agent Collaboration

This folder is the communication channel between Kiro and Antigravity IDE.

## Structure

```
.agents/
  ├── antigravity/
  │   ├── AGENTS.md    # Instructions for Antigravity agent
  │   ├── task.md      # Kiro writes tasks → Antigravity reads
  │   ├── result.md    # Antigravity writes responses → Kiro reads
  │   └── status.md    # Antigravity's current status
  ├── kiro/
  │   ├── task.md      # Antigravity writes tasks → Kiro reads
  │   ├── result.md    # Kiro writes responses → Antigravity reads
  │   └── status.md    # Kiro's current status
  ├── context/
  │   ├── project.md        # Tech stack, architecture overview
  │   ├── conventions.md    # Coding style, naming rules
  │   ├── current-sprint.md # Active goals, in-progress features
  │   └── decisions.md      # Key decisions already made
  ├── history/         # Archived task/result pairs (gitignored)
  ├── scripts/
  │   ├── archive.sh / archive.cmd   # Archive completed exchanges
  │   ├── reset.sh / reset.cmd       # Reset statuses and clear files
  │   └── pre_invocation.js          # Antigravity pre-invocation hook
  └── hooks.json       # Antigravity hook configuration
```

## Status Values

| Status | Meaning |
|--------|---------|
| `idle` | Ready to receive a new task |
| `processing` | Currently working on a task |
| `done` | Task completed, result is ready to read |
| `error` | Task failed |

## Task Format

```markdown
## Task: <title>
**File:** <relative path to relevant file, if any>
**Context:** <brief context to share>
**Question/Action:** <what the other agent should do>
**Task-ID:** <timestamp, e.g. 2026-05-24T13-00-00>
```

## Result Format

```markdown
## Result: <task title>
**Status:** Completed / Partial / Failed
**Task-ID:** <same id from task>
**Response:**
<full response here>
```

## Workflow: Kiro → Antigravity

1. Check `.agents/antigravity/status.md` — skip if `processing`
2. Write task to `.agents/antigravity/task.md`
3. Kiro runs `agy` as a background process
4. Antigravity processes the task, writes result to `.agents/antigravity/result.md`, sets status to `done`
5. Kiro reads result on demand

## Workflow: Antigravity → Kiro

1. Check `.agents/kiro/status.md` — skip if `processing`
2. Antigravity writes task to `.agents/kiro/task.md`
3. Kiro hook (`fileEdited`) auto-triggers and processes the task
4. Kiro writes result to `.agents/kiro/result.md`, sets status to `done`
5. Antigravity reads result

## Scripts

```bash
# Archive a completed exchange (Linux/macOS)
./.agents/scripts/archive.sh antigravity 2026-05-24T13-00-00

# Archive a completed exchange (Windows)
.agents\scripts\archive.cmd antigravity 2026-05-24T13-00-00

# Reset all agents (Linux/macOS)
./.agents/scripts/reset.sh

# Reset all agents (Windows)
.agents\scripts\reset.cmd
```
