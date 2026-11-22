#!/usr/bin/env bash
# Launch pinned acceptance when the harness has no completion notification.
# The new job directory holds sha, job.log and, on completion, exit.status.
#   tools/launch-at-commit.sh <sha> <new-job-directory> <command> [args…]
set -euo pipefail

if [ "$#" -lt 3 ]; then
  echo "usage: tools/launch-at-commit.sh <sha> <new-job-directory> <command> [args…]" >&2
  exit 2
fi
sha=$(git rev-parse --verify "$1^{commit}")
job=$2
shift 2
at_commit="$(dirname "$(realpath "$0")")/at-commit.sh"

command -v setsid >/dev/null
env --default-signal=INT,QUIT true
umask 077
mkdir -- "$job"
job=$(realpath -- "$job")
printf '%s\n' "$sha" >"$job/sha"

# A background shell ignores INT and QUIT. Restore the defaults before the
# pinned command starts so Python can install its usual Ctrl-C handler.
# Keep the verdict writer in the detached session with the command.
setsid env --default-signal=INT,QUIT bash -c '
  job=$1
  shift
  status=0
  "$@" || status=$?
  printf "%s\n" "$status" >"$job/exit.status.tmp"
  mv -- "$job/exit.status.tmp" "$job/exit.status"
' launch-at-commit "$job" "$at_commit" "$sha" "$@" </dev/null >"$job/job.log" 2>&1 &
printf '%s\n' "$job"
