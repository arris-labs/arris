#!/usr/bin/env bash
# Smoke test the detached launcher on a scratch repository.
set -euo pipefail
launcher="$(dirname "$(realpath "$0")")/launch-at-commit.sh"
scratch=$(mktemp -d)
trap 'rm -rf "$scratch"' EXIT
cd "$scratch"
git init -q -b main
git config user.email test@example.com
git config user.name test
printf 'pinned\n' > marker
git add marker
git commit -qm pinned
sha=$(git rev-parse HEAD)
printf 'dirty\n' > marker

export ARRIS_GATE_SCOPE=off
job=$("$launcher" HEAD 'acceptance job' python3 -c '
import os
import signal
from pathlib import Path

assert Path("marker").read_text() == "pinned\n"
try:
    os.kill(os.getpid(), signal.SIGINT)
except KeyboardInterrupt:
    print("received Ctrl-C")
    raise SystemExit(23)
raise AssertionError("SIGINT did not raise KeyboardInterrupt")
')
for ((attempt = 0; attempt < 200; attempt++)); do
  [ ! -f "$job/exit.status" ] || break
  sleep 0.05
done
if [ ! -f "$job/exit.status" ] || [ "$(cat "$job/exit.status")" != 23 ]; then
  cat "$job/job.log" >&2
  exit 1
fi
[ "$(cat "$job/sha")" = "$sha" ]
grep -qx 'received Ctrl-C' "$job/job.log"
[ "$(cat marker)" = dirty ]
if "$launcher" HEAD "$job" true 2>/dev/null; then
  echo "launch-at-commit-test: accepted an existing job directory" >&2
  exit 1
fi
[ "$(cat "$job/exit.status")" = 23 ]
echo "launch-at-commit-test: ok"
