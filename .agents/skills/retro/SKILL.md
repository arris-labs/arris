---
name: retro
description: Read a plan's (or this) session's transcripts and propose improvements to the agent's environment, most severe first.
disable-model-invocation: true
argument-hint: <plan slug, session id, or nothing for this session>
---

# /retro — improve the environment, not the code

The question is what in this repository's environment (steering files,
checks, tools, docs) made the work slower, wronger or more dependent on the
human than it had to be, and the smallest change that fixes it for every
later session.

1. **Read the sessions.** `.agents/skills/retro/session-digest --slug
   <slug>` (or `--session <id>`; nothing given: this session, from your
   own context). It prints each session's human messages, skills, tools,
   failed calls and stalls; sessions from before the checkout moved need
   `--project-dir` with the old path's transcript directory. Open a
   transcript only where the digest points at something
   (`~/.claude*/projects/<this repo>/<id>.jsonl`). Read the plan's commits
   too (`git log --grep='plans/<slug>'`).
2. **Find the candidates**, each with its evidence (session id and a quote
   or command):
   - **The human stepping in**: a correction, a repeated instruction, a
     question about where things stand, an answer the agent could have
     looked up. Each is a missing rule, pointer or skill step.
   - **Navigation**: a fact that took many reads to find, or a doc the
     agent had to be pointed at. Fix with a sharper pointer where the agent
     already looks.
   - **Checks**: a mistake a lint, test or hook could have caught, or a
     check that exists but is not wired. A mechanical rule gets a check,
     not a sentence.
   - **Steering files**: in `AGENTS.md`, `.agents/rules/` and the skills, a
     sentence that changes nothing the agent would do anyway, restates what
     a command or config already says, phrases a goal as a prohibition, or
     has gone stale. Always-loaded lines cost every turn.
   - **Tool economy**: a long gate run repeated, a wait polled by hand, an
     oversized output read whole, a full gate run where the slice would do.
   - **Information access**: a log, oracle value, nightly artifact or
     machine fact the agent could not reach.
3. **Reply with the candidates, most severe first**: what happened (the
   evidence), the change (file and wording, or the check), and its cost.
   Change nothing yet.
4. Apply the ones the human accepts, one commit per change as `docs: …`
   (or `build`/`ci` for a check), the body citing the session it came from.

`$ARGUMENTS`
