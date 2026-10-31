#!/usr/bin/env bash
# A Claude Code PreToolUse hook on Bash (`.claude/settings.json`): refuses a
# command the rules forbid but agents still write, with the way past it.
#
#   - A foreground wait: a `while`/`until` loop around `sleep`, `tail --pid`,
#     or `tail -f` (`.agents/rules/git.md`, no foreground waits). Run the
#     command itself in the foreground with a `timeout` when its result is
#     needed now, or with `run_in_background` and wait for the notification.
#     A background command may loop: its end is the notification.
#   - `git stash`, in any form: a subagent's `git stash; git stash pop` on a
#     clean tree popped an old stash of another plan and left a file
#     unmerged (/retro horn-torus). Steps commit; a patch file sets work
#     aside. The match reads the whole command, quoted text included.
#
# Reads the hook's JSON on stdin; exit 2 refuses, its stderr goes to the agent.
#
#   tools/bash-guard-test.sh tests it.
set -uo pipefail
input=$(cat)
cmd=$(jq -r '.tool_input.command // ""' <<<"$input")
bg=$(jq -r '.tool_input.run_in_background // false' <<<"$input")

refuse() { echo "bash-guard: $*" >&2; exit 2; }

# A word starts the command, follows a separator, or opens a quoted `sh -c`.
w="(^|[;&|({'\"[:space:]])"
if [ "$bg" != true ]; then
  if grep -qE "${w}(while|until)[[:space:]].*[[:space:]]sleep[[:space:]]" <<<"$cmd" ||
    grep -qE "${w}tail[[:space:]]+([^|;&]*[[:space:]])?(--pid|-[a-zA-Z]*f[a-zA-Z]*([[:space:]]|\$)|--follow)" <<<"$cmd"; then
    refuse "a foreground wait loop (.agents/rules/git.md, no foreground waits). Run the command you are waiting on in the foreground with \`timeout\` if you need its result now, or with run_in_background and wait for its completion notification. A commit's test half is not a step's to wait for (tools/verify.sh --status reads it, waiting on a run in flight)."
  fi
fi
if grep -qE "${w}git[[:space:]]+([^|;&]*[[:space:]])?stash([[:space:]]|\$)" <<<"$cmd"; then
  refuse "git stash (.agents/rules/git.md): a pop or apply can bring back another session's work and leave the tree unmerged. Commit the work, or set it aside with \`git diff > <scratchpad>/x.patch\` and \`git checkout -- <paths>\`."
fi
exit 0
