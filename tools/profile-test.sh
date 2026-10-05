#!/usr/bin/env bash
# `fast` is `default` minus exactly the named slow set and the `real_*`
# corpus tests (ADR-0032). A renamed test leaves the set silently if nothing
# compares the two lists; this does, by name, in both directions.
#
#   tools/profile-test.sh
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

# The slow set, by test name. Keep in step with `.config/nextest.toml`.
slow='guard::coverage
boolean_fixtures_read_back
boolean_plate_10x10_holes_cut_many
part::tests::a_battery_stage_is_held_to_its_class
a_budget_stops_every_operation_at_the_same_step_and_leaves_the_model_as_it_was'

list() { cargo nextest list --workspace --cargo-quiet --message-format oneline ${1:+--profile "$1"} 2>/dev/null | sort; }
full=$(list)
fast=$(list fast)

# Every name in the slow set must exist in `default`, or the filter is stale.
expected=""
while read -r name; do
  [ -n "$name" ] || continue
  hits=$(grep -E " ${name//\//\\/}\$|::${name}\$| ${name}\$" <<<"$full" || true)
  [ -n "$hits" ] || { echo "profile-test: slow-set name '$name' matches no test" >&2; exit 1; }
  expected+="$hits"$'\n'
done <<<"$slow"
expected+=$(grep -E ' real_[^ ]*$' <<<"$full" || true)
expected=$(sort -u <<<"$expected" | sed '/^$/d')

actual=$(comm -23 <(echo "$full") <(echo "$fast"))
extra=$(comm -13 <(echo "$full") <(echo "$fast"))
[ -z "$extra" ] || { echo "profile-test: fast lists tests default does not:" >&2; echo "$extra" >&2; exit 1; }
if [ "$actual" != "$expected" ]; then
  echo "profile-test: the difference is not the named set. Expected out of fast:" >&2
  echo "$expected" >&2; echo "actual:" >&2; echo "$actual" >&2
  exit 1
fi
echo "profile-test: fast = default minus $(wc -l <<<"$actual") tests (the slow set and real_*)"
