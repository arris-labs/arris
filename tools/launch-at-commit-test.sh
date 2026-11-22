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
job="$scratch/acceptance job"
worker='
import os
import signal
from pathlib import Path
import sys
import time

assert Path("marker").read_text() == "pinned\n"
ready, release = map(Path, sys.argv[1:])
ready.touch()
deadline = time.monotonic() + 15
while not release.exists():
    assert time.monotonic() < deadline, "test did not release detached job"
    time.sleep(0.05)
try:
    os.kill(os.getpid(), signal.SIGINT)
except KeyboardInterrupt:
    print("received Ctrl-C")
    raise SystemExit(23)
raise AssertionError("SIGINT did not raise KeyboardInterrupt")
'
wait_for_file() {
  local attempt
  for ((attempt = 0; attempt < 200; attempt++)); do
    [ ! -f "$1" ] || return 0
    sleep 0.05
  done
  return 1
}

setsid bash -c '"$@" && exec sleep 20' launch \
  "$launcher" HEAD "$job" python3 -c "$worker" "$scratch/ready" "$scratch/release" >/dev/null &
origin=$!
if ! wait_for_file "$scratch/ready"; then
  cat "$job/job.log" >&2
  exit 1
fi
# Closing the launching session must leave the verdict writer alive.
kill -TERM -- "-$origin"
wait "$origin" 2>/dev/null || true
touch "$scratch/release"

if ! wait_for_file "$job/exit.status" || [ "$(cat "$job/exit.status")" != 23 ]; then
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
