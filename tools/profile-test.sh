#!/usr/bin/env bash
# `fast` is `default` minus exactly the named slow set and the `real_*`
# corpus tests (ADR-0032). A renamed test leaves the set silently if nothing
# compares the two lists; this does, by name, in both directions.
#
#   tools/profile-test.sh
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

# The slow set, by test name, as `tools/test-timings.sh --slow-set` printed
# it. Keep in step with `.config/nextest.toml`.
slow='a_budget_stops_every_operation_at_the_same_step_and_leaves_the_model_as_it_was
an_oracle_with_no_solid_holds_arris_to_additivity
arris_and_the_oracle_agree_on_multi_tool_booleans
blend_drilled_torus_fillet
blend_fixtures_read_back
blend_nist_ctc_04_chamfered_wall_fillet
blend_nist_ftc_10_cylinder_pair_bspline_fillet
boolean_ball_pole_drill_cut
boolean_ball_pole_slice_cut
boolean_fixtures_read_back
boolean_nist_ctc_04_face_arrangement_turn
boolean_nist_ctc_05_ap203_box_cut_nurbs_plane
boolean_nist_ctc_05_ap242_box_cut_nurbs_plane
boolean_pipe_elbow_fuse
boolean_pipe_elbow_fuse_band_ends
boolean_plate_10x10_holes_cut_many
boolean_revolve_extrude_fuse_cut_rounding_knots
boolean_ring_corner_common
boolean_ring_pin_cut
census::tests::ftc_06_run_is_a_dihedral_jump_of_the_nurbs_cycle
census::tests::no_split_rims_second_vertex_is_refused
guard::coverage
loft_loft_closed_torus_like
part::tests::a_battery_stage_is_held_to_its_class
provenance_consumer_rebuild
split_nurbs_cylinder_oblique
sweep_fixtures_read_back
sweep_sweep_spring_cut_split
the_band_at_the_fixtures_numbers
transform_mirror_hollow_ring'

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
