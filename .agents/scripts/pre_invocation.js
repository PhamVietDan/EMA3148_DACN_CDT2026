/**
 * Antigravity Pre-Invocation Hook
 *
 * This script is called by Antigravity before each agent session.
 * It checks for pending tasks from Kiro and injects them into the conversation.
 *
 * Configure in .agents/hooks.json:
 * {
 *   "PreInvocation": [{ "type": "command", "command": "node scripts/pre_invocation.js" }]
 * }
 */

const fs = require('fs');
const path = require('path');

const projectRoot = process.cwd().endsWith('.agents')
  ? path.join(process.cwd(), '..')
  : process.cwd();

const agentsDir = path.join(projectRoot, '.agents');
const taskFile = path.join(agentsDir, 'antigravity', 'task.md');
const statusFile = path.join(agentsDir, 'antigravity', 'status.md');

const response = {injectSteps: []};

// Check for a pending task
if (fs.existsSync(taskFile)) {
  const content = fs.readFileSync(taskFile, 'utf8');

  // Strip HTML comments and placeholder headers to detect real content
  const cleanContent = content
    .replace(/<!--[\s\S]*?-->/g, '')
    .replace(/^#.*$/gm, '')
    .trim();

  if (cleanContent) {
    // Read AGENTS.md for full instructions
    const agentsMd = path.join(agentsDir, 'antigravity', 'AGENTS.md');
    const instructions = fs.existsSync(agentsMd)
      ? fs.readFileSync(agentsMd, 'utf8')
      : '';

    const messageText = [
      '## Pending Task from Kiro',
      '',
      `Task file: \`.agents/antigravity/task.md\``,
      '',
      '### Task Content',
      cleanContent,
      '',
      '---',
      instructions
        ? '### Agent Instructions\n' + instructions
        : 'Write your response to `.agents/antigravity/result.md` and update `.agents/antigravity/status.md` to `done`.',
    ].join('\n');

    response.injectSteps.push({userMessage: messageText});
  }
}

process.stdout.write(JSON.stringify(response));
process.exit(0);
