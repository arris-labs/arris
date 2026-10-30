#!/usr/bin/env bash
# The rework measure: per tier, how many plan steps needed a later commit to
# repair them, so the grades (`.agents/tiers.md`) can be re-weighted on
# evidence. Reads the commits' `(plans/<slug> step N)` suffixes
# (`.agents/rules/git.md`, attribution).
#
#   tools/step-rework.sh [--since <date>]    # default: the whole history
#
# A step's tier is read from the plan file as it stood in the step's first
# commit: its grade `**[2]**`, or the tier a `**[2, tier 3]**` override
# names; a step with no readable grade counts under `none`. A step's own
# commits are those up to and including the first that is neither `partial`
# nor a lettered part (`step 5b`); every later commit naming the step is
# rework. `steps 1-2` names each step of the range. `(plans/<slug> review)`
# commits are review fixes: no step claims them, so they are counted per plan.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

since=()
if [ "${1:-}" = "--since" ]; then
  [ -n "${2:-}" ] || { echo "usage: $0 [--since <date>]" >&2; exit 2; }
  since=(--since "$2")
elif [ $# -gt 0 ]; then
  echo "usage: $0 [--since <date>]" >&2; exit 2
fi

declare -A tier_of closed rework review
re='plans/([a-z0-9-]+) (steps?|review)( ([0-9]+)(-([0-9]+))?([a-z]?))?'

# grade_of <commit> <slug> <n> — the tier, or "none".
grade_of() {
  local line
  line=$(git show "$1:docs/plans/$2.md" 2>/dev/null \
    | grep -E "^- \[[ x]\] Step $3 " | head -n 1 || true)
  if [[ "$line" =~ \*\*\[([0-9])(,\ tier\ ([0-9]))?[],] ]]; then
    echo "${BASH_REMATCH[3]:-${BASH_REMATCH[1]}}"
  else
    echo none
  fi
}

while IFS=$'\t' read -r sha subject; do
  [[ "$subject" =~ $re ]] || continue
  slug=${BASH_REMATCH[1]}; kind=${BASH_REMATCH[2]}
  if [ "$kind" = review ]; then
    review[$slug]=$(( ${review[$slug]:-0} + 1 )); continue
  fi
  lo=${BASH_REMATCH[4]:-}; hi=${BASH_REMATCH[6]:-$lo}; letter=${BASH_REMATCH[7]:-}
  [ -n "$lo" ] || continue
  partial=0; [[ "$subject" == *partial* ]] && partial=1
  for ((n = lo; n <= hi; n++)); do
    key="$slug:$n"
    if [ -n "${closed[$key]:-}" ]; then
      rework[$key]=$(( ${rework[$key]:-0} + 1 )); continue
    fi
    [ -n "${tier_of[$key]:-}" ] || tier_of[$key]=$(grade_of "$sha" "$slug" "$n")
    if [ $partial = 0 ] && [ -z "$letter" ]; then closed[$key]=1; fi
  done
done < <(git log --reverse "${since[@]}" --format='%H%x09%s')

set +u  # empty associative arrays are unbound on older bash
declare -A steps reworked commits
for key in "${!tier_of[@]}"; do
  t=${tier_of[$key]}
  steps[$t]=$(( ${steps[$t]:-0} + 1 ))
  r=${rework[$key]:-0}
  if [ "$r" -gt 0 ]; then
    reworked[$t]=$(( ${reworked[$t]:-0} + 1 ))
    commits[$t]=$(( ${commits[$t]:-0} + r ))
  fi
done

printf '%-5s %6s %9s %6s %7s\n' tier steps reworked share commits
for t in 1 2 3 none; do
  s=${steps[$t]:-0}; w=${reworked[$t]:-0}; c=${commits[$t]:-0}
  if [ "$s" -gt 0 ]; then share=$(( 100 * w / s ))%; else share=-; fi
  printf '%-5s %6d %9d %6s %7d\n' "$t" "$s" "$w" "$share" "$c"
done

if [ -n "${review[*]:-}" ]; then
  echo
  echo "review fixes per plan (claimed by no step):"
  for slug in $(printf '%s\n' "${!review[@]}" | sort); do
    printf '  %-32s %d\n' "$slug" "${review[$slug]}"
  done
fi
