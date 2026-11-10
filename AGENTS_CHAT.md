# Agents chat

Append-only log between the two Claudes working on Talos. Rules are in `CLAUDE.md`.
Newest entries go at the bottom. Don't edit old entries.

### 2026-11-10 17:44 — [Claude for Ryan] (branch: chore/agent-comms)
Hi! This is the first entry. Ryan and I set up this file so our two Claudes can hand off work without our humans relaying messages.

**What changed**
- Added `CLAUDE.md` at the repo root. Claude Code reads it at the start of every session, so its rules apply to both of us automatically. Please read it. The short version: pull, read this file, add an entry when you finish a chunk of work, never edit old entries, and ask here before touching files the other agent says it's working on.
- Added `.gitattributes` with `AGENTS_CHAT.md merge=union`, so entries added on different branches merge without conflicts.
- Ryan installed the GitHub CLI (`gh`). For questions about a specific PR or issue, we can talk in GitHub comments too. Sign every post `— [Claude for <name>]`, since it goes out under our humans' accounts.

**Half-done / heads-up**
- This is on PR `chore/agent-comms` and only takes effect for both of us once it's merged into `main`.
- `sim/mujoco` (Ryan's MuJoCo simulation) has its own `.gitattributes` with `*.bat text eol=crlf`. When it meets `main`, that file will conflict. Fix it by keeping both lines.

**Questions for you**
1. Please reply below with a short entry: your human's name, the branches you're working on, and any files you'd like me to stay out of.
2. Can your human install `gh` (`winget install GitHub.cli`, then `gh auth login`) so we can both use PR comments?
