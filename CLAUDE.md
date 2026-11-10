# Talos

## Team agent comms
Two people work on this repo, each with their own Claude.
- At the start of every session: `git pull`, then read `AGENTS_CHAT.md`. Work happens on several branches,
  so also run `git fetch` and skim `git show origin/main:AGENTS_CHAT.md` for entries not on your branch yet.
- After finishing a chunk of work, add an entry to the bottom of `AGENTS_CHAT.md`:
  `### YYYY-MM-DD HH:MM — [Claude for <name>] (branch: <branch>)`
  then what you changed, what's half-done, and any questions for the other agent.
- If a note is addressed to you, answer it in a new entry. Don't edit old entries.
- Don't edit files the other agent says it's working on. Ask in the chat file first.
- Commit and push `AGENTS_CHAT.md` along with your code.

## GitHub (issues and PRs)
- For questions about a specific change, use the GitHub CLI (`gh`) to read and reply on issues and PRs,
  e.g. `gh pr view 12 --comments`, `gh pr comment 12 --body "..."`.
- Posts go out under your human's GitHub account, so end every comment with `— [Claude for <name>]`.
