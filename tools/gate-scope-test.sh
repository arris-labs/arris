#!/usr/bin/env bash
# `tools/gate-scope.sh` held: a command over the bound fails naming it and
# this shell survives; a command inside the bound passes; `ARRIS_GATE_SCOPE=off`
# runs unchanged. Skips the scope cases where `systemd-run --user` is absent.
set -uo pipefail
cd "$(git rev-parse --show-toplevel)"
fail=0
unset ARRIS_GATE_IN_SCOPE ARRIS_GATE_SCOPED  # a gate running this test is itself in a scope
out=$(ARRIS_GATE_SCOPE=off tools/gate-scope.sh sh -c 'echo "in=${ARRIS_GATE_IN_SCOPE:-none}"')
[ "$out" = "in=none" ] || { echo "FAIL off: ran in a scope ($out)" >&2; fail=1; }
if command -v systemd-run >/dev/null && systemd-run --user --scope --quiet true >/dev/null 2>&1; then
  out=$(tools/gate-scope.sh sh -c 'echo "in=${ARRIS_GATE_IN_SCOPE:-none}"')
  [ "$out" = "in=1" ] || { echo "FAIL scope: not run in a scope ($out)" >&2; fail=1; }
  err=$(ARRIS_GATE_MEMORY_MAX=64M python3 -c '
import subprocess, sys
r = subprocess.run(["tools/gate-scope.sh", "python3", "-c", "x = bytearray(400 * 1024 * 1024)"], capture_output=True, text=True)
sys.stdout.write(str(r.returncode) + " " + r.stderr)')
  case $err in
    "137 "*MemoryMax=64M*) ;;
    *) echo "FAIL bound: $err" >&2; fail=1 ;;
  esac
else
  echo "gate-scope-test: no systemd-run --user, scope cases skipped"
fi
[ "$fail" = 0 ] && echo "gate-scope-test: ok" || exit 1
