#!/usr/bin/env bash
# A Claude Code PreToolUse hook on Bash (`.claude/settings.json`): refuses a
# command the rules forbid but agents still write, with the way past it.
#
#   - A foreground wait: a `while`/`until` loop around `sleep`, `tail --pid`,
#     or `tail -f` (`.agents/rules/git.md`, no foreground waits). Run the
#     command itself in the foreground with a `timeout` when its result is
#     needed now, or with `run_in_background` and wait for the notification.
#     A background command may loop: its end is the notification.
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
exit 0
