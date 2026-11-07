#!/usr/bin/env bash
# A Claude Code PreToolUse hook on Bash (`.claude/settings.json`): refuses a
# command the rules forbid but agents still write, with the way past it.
#
#   - A foreground wait: a `while`/`until` loop around `sleep`, `tail --pid`,
#     or `tail -f` (`.agents/rules/git.md`, no foreground waits). Run the
#     command itself in the foreground with a `timeout` when its result is
#     needed now, or with `run_in_background` and wait for the notification.
#     A background command may loop: its end is the notification.
#   - `git stash`, any form but `list` and `show`: a subagent's `git stash; git stash pop` on a
#     clean tree popped an old stash of another plan and left a file
#     unmerged (/retro horn-torus). Steps commit; a patch file sets work
#     aside.
#
# Neither rule reads a heredoc's body (a commit message, a script written to
# a file), and the stash rule reads no quoted text either (`git commit -m
# "…stash…"`, `grep stash`); the loop rule does, since `sh -c '…'` is a
# loop's usual wrapper.
#
# Reads the hook's JSON on stdin; exit 2 refuses, its stderr goes to the agent.
#
#   tools/bash-guard-test.sh tests it.
set -uo pipefail
input=$(cat)
cmd=$(jq -r '.tool_input.command // ""' <<<"$input")
bg=$(jq -r '.tool_input.run_in_background // false' <<<"$input")

refuse() { echo "bash-guard: $*" >&2; exit 2; }

# The command without its heredoc bodies: after a line opening `<<WORD`,
# `<<'WORD'` or `<<-WORD`, every line up to the one that is WORD.
code=$(awk '
  end != "" { t = $0; if (dash) sub(/^\t+/, "", t); if (t == end) end = ""; next }
  { print; l = $0; gsub(/<<</, "", l)  # a herestring is not a heredoc
    if (match(l, /<<-?[[:space:]]*["\047]?[A-Za-z_][A-Za-z0-9_]*/)) {
      d = substr(l, RSTART, RLENGTH); dash = (d ~ /^<<-/)
      sub(/^<<-?[[:space:]]*["\047]?/, "", d); end = d } }' <<<"$cmd")
# …and without its quoted strings, for a rule about the command's own words.
bare=$(sed -E "s/'[^']*'//g; s/\"([^\"\\]|\\.)*\"//g" <<<"$code")

# A word starts the command, follows a separator, or opens a quoted `sh -c`.
w="(^|[;&|({'\"[:space:]])"
if [ "$bg" != true ]; then
  if grep -qE "${w}(while|until)[[:space:]].*[[:space:]]sleep[[:space:]]" <<<"$code" ||
    grep -qE "${w}tail[[:space:]]+([^|;&]*[[:space:]])?(--pid|-[a-zA-Z]*f[a-zA-Z]*([[:space:]]|\$)|--follow)" <<<"$code"; then
    refuse "a foreground wait loop (.agents/rules/git.md, no foreground waits). Run the command you are waiting on with run_in_background (or in the foreground under \`timeout\` if it ends within 3 minutes: the harness backgrounds anything longer) and end your turn; its completion notification resumes you. A commit's test half is not a step's to wait for (tools/verify.sh --status reads it, waiting on a run in flight)."
  fi
fi
# `list` and `show` only read; every other form (bare, push, pop, apply, drop) moves work.
if grep -E "${w}git[[:space:]]+([^|;&]*[[:space:]])?stash([[:space:]]|\$)" <<<"$bare" |
  grep -qvE "stash[[:space:]]+(list|show)([[:space:]]|\$)"; then
  refuse "git stash (.agents/rules/git.md): a pop or apply can bring back another session's work and leave the tree unmerged. Commit the work, or set it aside with \`git diff > <scratchpad>/x.patch\` and \`git checkout -- <paths>\`."
fi
exit 0
