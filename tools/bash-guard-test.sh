#!/usr/bin/env bash
# Tests `tools/bash-guard.sh` on the commands agents wrote (the poll loops of
# the horn-torus run) and on ones it must let through.
#
#   tools/bash-guard-test.sh
set -uo pipefail
guard="$(git rev-parse --show-toplevel)/tools/bash-guard.sh"
fail=0
run() { # <expected exit> <run_in_background> <command>
  local got
  jq -n --arg c "$3" --argjson b "$2" '{tool_name:"Bash",tool_input:{command:$c,run_in_background:$b}}' |
    "$guard" 2>/dev/null
  got=$?
  [ "$got" = "$1" ] || { echo "FAIL (want $1, got $got, bg=$2): $3" >&2; fail=1; }
}

# refused in the foreground
run 2 false 'while kill -0 811433 2>/dev/null; do sleep 20; done; git log -1'
run 2 false "timeout 590 sh -c 'until grep -q \"^done\" /tmp/x.log; do sleep 5; done'"
run 2 false 'cd repo; tail --pid=877518 -f /dev/null; tail -12 /tmp/blend.log'
run 2 false 'timeout 170 tail -f /tmp/t.output | grep -m1 Summary'
run 2 false 'while ! grep -q "^exit" c1.log; do sleep 10; done'
# the same loop in the background is its own notification
run 0 true 'timeout 1800 sh -c "until grep -q done x; do sleep 5; done"'
# not waits
run 0 false 'tail -n 20 /tmp/gate.log'
run 0 false 'tail -3 a.log b.log'
run 0 false 'cargo nextest run -p arris-geom --profile fast'
run 0 false 'tools/verify.sh --status'
run 0 false 'git log --format=%s | grep -c "while"'
run 0 false 'grep -n "sleep" tools/verify.sh'
# git stash, foreground or not
run 2 false 'git stash; cargo test; git stash pop'
run 2 true 'cd x && git stash push -m wip'
run 2 false 'git -C repo stash list'
run 0 false 'git log --grep=stash'
run 0 false 'git status --short'

[ "$fail" = 0 ] && echo "bash-guard-test: ok" || exit 1
