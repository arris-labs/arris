#!/usr/bin/env bash
# Stops a stuck pre-commit gate or test run in this checkout: the hook, the
# `git commit` waiting on it, nextest, cargo and the test binaries — only
# processes whose working directory is inside this repository, so another
# checkout's build on the same machine is left alone.
#
#   tools/stop-gate.sh
#
# `pkill -f <pattern>` from an agent's shell matches that shell's own
# command line, which holds the pattern, and kills it (exit 144); this
# script carries the patterns in its body, never in its arguments, and
# skips itself and its ancestors besides.
set -euo pipefail
root=$(git rev-parse --show-toplevel)

# Ourselves and every ancestor: never a target.
keep=" $$ "
p=$$
while [ "$p" -gt 1 ]; do
  p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
  [ -n "$p" ] || break
  keep+="$p "
done

# The PIDs whose command line matches `$1` and whose cwd is under the root.
ours() {
  local pid cwd
  for pid in $(pgrep -f -- "$1" || true); do
    [[ "$keep" == *" $pid "* ]] && continue
    cwd=$(readlink "/proc/$pid/cwd" 2>/dev/null) || continue
    [[ "$cwd" == "$root" || "$cwd" == "$root"/* ]] && echo "$pid"
  done
}

# Outermost first, so nothing restarts what the next line stops.
patterns=(
  'git commit'
  '\.githooks/pre-commit'
  'cargo-nextest'
  'cargo (nextest|test|clippy|doc)'
  "$root/target/[^ ]*/deps/"
)

stopped=()
for pat in "${patterns[@]}"; do
  for pid in $(ours "$pat"); do
    stopped+=("$pid $(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null | cut -c1-100)")
    kill -TERM "$pid" 2>/dev/null || true
  done
done

if [ ${#stopped[@]} -eq 0 ]; then
  echo "stop-gate: nothing running in $root"
  exit 0
fi
printf 'stop-gate: TERM %s\n' "${stopped[@]}"

# Whatever ignored TERM for three seconds gets KILL.
sleep 3
for line in "${stopped[@]}"; do
  pid=${line%% *}
  if kill -0 "$pid" 2>/dev/null; then
    kill -KILL "$pid" 2>/dev/null || true
    echo "stop-gate: KILL $pid"
  fi
done

if [ -e "$(git rev-parse --git-dir)/index.lock" ]; then
  echo "stop-gate: index.lock remains; remove it only once no git runs here" >&2
fi
