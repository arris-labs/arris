#!/usr/bin/env bash
# A full gate taken on the same code is reused (ADR-0085 §3).
#
#   tools/gate-reuse.sh                    the nearest recorded ancestor of HEAD
#                                          (HEAD included) whose diff to HEAD
#                                          reaches no code path: its sha; exit 1
#                                          when there is none
#   tools/gate-reuse.sh --record           record HEAD as gated; refused when a
#                                          code path is dirty
#   tools/gate-reuse.sh --reaches-code <a> <b>
#                                          exit 0 when the diff a..b touches a
#                                          code path: the one list, also
#                                          `/retire-plan`'s
#
# A record is a file `arris-verify/gated/<sha>` of the common git dir; a swept
# one costs one more gate, never a wrong reuse.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

# The code paths: what a test, the gate or the build reads.
is_code() {
  case $1 in
    crates/* | tests/* | fuzz/* | Cargo.* | clippy.toml | rust-toolchain.toml) return 0 ;;
    .config/* | .githooks/* | tools/*gate* | tools/oracle/* | tools/check-*) return 0 ;;
    tools/*.sh) return 1 ;;
    tools/*) return 0 ;;
  esac
  return 1
}

reaches_code() {
  local f
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    is_code "$f" && return 0
  done < <(git diff --name-only "$1" "$2" --)
  return 1
}

gated=$(realpath "$(git rev-parse --git-common-dir)")/arris-verify/gated

case ${1:-} in
  --reaches-code)
    [ $# -eq 3 ] || { echo "usage: tools/gate-reuse.sh --reaches-code <a> <b>" >&2; exit 2; }
    reaches_code "$2" "$3"
    ;;
  --record)
    while IFS= read -r line; do
      p=${line:3}
      for f in "${p%% -> *}" "${p##* -> }"; do
        f=${f#\"}; f=${f%\"}
        if is_code "$f"; then
          echo "gate-reuse: $f is dirty; no record" >&2
          exit 1
        fi
      done
    done < <(git status --porcelain --untracked-files=all)
    mkdir -p "$gated"
    git rev-parse HEAD >"$gated/$(git rev-parse HEAD)"
    ;;
  "")
    while IFS= read -r sha; do
      [ -f "$gated/$sha" ] || continue
      reaches_code "$sha" HEAD || { echo "$sha"; exit 0; }
    done < <(git rev-list --max-count=200 HEAD)
    exit 1
    ;;
  *)
    echo "usage: tools/gate-reuse.sh [--record | --reaches-code <a> <b>]" >&2
    exit 2
    ;;
esac
