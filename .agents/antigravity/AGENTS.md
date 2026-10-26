# Antigravity Agent Instructions

You are operating as part of a two-way collaboration system with Kiro IDE. This folder (`.agents/`) is the communication channel between you and Kiro.

## Your Files

| File | Your Role |
|------|-----------|
| `.agents/antigravity/task.md` | Read tasks assigned to you by Kiro |
| `.agents/antigravity/result.md` | Write your responses for Kiro to read |
| `.agents/antigravity/status.md` | Update your current status |
| `.agents/kiro/task.md` | Write tasks you want Kiro to handle |
| `.agents/kiro/result.md` | Read Kiro's responses |
| `.agents/kiro/status.md` | Check if Kiro is busy before assigning a task |

## Shared Context

Before processing or writing a task, read relevant files from `.agents/context/`:
- `project.md` — tech stack and architecture
- `conventions.md` — coding style and patterns
- `current-sprint.md` — active goals and blockers
- `decisions.md` — key decisions already made

## Archiving to History

After completing a task, archive the exchange to `.agents/history/`:
```
.agents/history/<Task-ID>_antigravity_task.md
.agents/history/<Task-ID>_antigravity_result.md
```

## Workflow: Processing a Task from Kiro

1. Update `.agents/antigravity/status.md` → set status to `processing`
2. Read `.agents/antigravity/task.md`
3. Complete the task
4. Write your full response to `.agents/antigravity/result.md`
5. Update `.agents/antigravity/status.md` → set status to `done`

## Workflow: Delegating a Task to Kiro

1. Check `.agents/kiro/status.md` — wait if status is `processing`
2. Write the task to `.agents/kiro/task.md` using this format:

```markdown
## Task: <title>
**File:** <relative path to relevant file, if any>
**Context:** <brief context Kiro needs>
**Question/Action:** <exactly what Kiro should do>
**Task-ID:** <timestamp or unique id to avoid reprocessing>
```

3. Update `.agents/kiro/status.md` → set status to `idle` (Kiro's hook will trigger automatically)
4. Read `.agents/kiro/result.md` when Kiro sets its status to `done`

## Result Format

Always write results in this format:

```markdown
## Result: <task title>
**Status:** Completed / Partial / Failed
**Task-ID:** <same id from task>
**Response:**
<full response here>
```

## Important Rules

- Always use absolute paths when reading or writing files
- Do not overwrite `task.md` files — only write to your own `result.md`
- Check status before assigning tasks to avoid conflicts
