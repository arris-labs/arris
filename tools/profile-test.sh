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
a_chain_into_a_twinned_arc_meets_on_the_balls_arc
a_cylinders_twinned_rim_chamfers_as_the_closed_forms_cone
a_cylinders_twinned_rim_fillets_as_the_closed_forms_torus
an_elbows_cut_edge_holds_its_pcurve_gap_in_the_poses_that_found_it
an_oracle_with_no_solid_holds_arris_to_additivity
arris_and_the_oracle_agree_on_multi_tool_booleans
blend_bicubic_top_box_edge_fillet
blend_bicubic_top_fillet_common_spline_sweep
blend_bicubic_top_fillet_cut_by_spline_sweep
blend_bicubic_top_fillet_fuse_spline_sweep
blend_box_all_edges_fillet
blend_cusp_corner_crescent_chamfer_second_blend
blend_cusp_overhang_tip_chamfer
blend_cusp_overhang_tip_fillet
blend_cusp_pocket_fillet
blend_cylinder_rim_spindle_fillet
blend_drilled_torus_fillet
blend_elbow_posed_fit_span_limit
blend_fixtures_read_back
blend_lip_fillet_runs_into_a_hole
blend_nist_ctc_02_ap242_ellipse_chain_fillet
blend_nist_ctc_04_chamfered_wall_fillet
blend_nist_ctc_05_ap203_nurbs_torus_fillet
blend_nist_ctc_05_ap242_battery_edge_fillet
blend_nist_ctc_05_ap242_nurbs_torus_fillet
blend_nist_ftc_10_cylinder_pair_bspline_fillet
blend_nurbs_cap_cylinder_rim_chamfer
blend_nurbs_cap_cylinder_rim_fillet
blend_rounded_block_oblique_top_fillet
blend_rounded_box_steep_split_cap_rim_fillet
blend_rounded_column_split_cap_rim_fillet
blend_rounded_pocket_oblique_floor_fillet
blend_s_bend_oblique_top_fillet
blend_sphere_cylinder_fillet
blend_sphere_cylinder_oblique_seam_fillet
blend_sphere_cylinder_pin_seam_slant_fillet
blend_split_pin_foot_fillet
blend_standoff_root_fillet_runs_into_a_hole
blend_standoff_root_fillet_runs_off_at_a_shallow_angle
blend_tee_cylinders_fillet
boolean_ball_pole_drill_cut
boolean_ball_pole_slice_cut
boolean_fixtures_read_back
boolean_nist_ctc_04_face_arrangement_turn
boolean_nist_ctc_05_ap242_box_cut_nurbs_plane
boolean_pipe_elbow_fuse
boolean_pipe_elbow_fuse_band_ends
boolean_plate_10x10_holes_cut_many
boolean_revolve_cylinder_cut_box_fuse_builder_fault
boolean_revolve_extrude_fuse_cut_rounding_knots
boolean_ring_corner_common
boolean_ring_pin_cut
boolean_seam_beside_crossing_fuse
capability_nurbs
capability_nurbs_heavy
guard::coverage
loft_fixtures_read_back
loft_loft_closed_torus_like
loft_loft_common_loft_across_seams
loft_loft_duct
loft_loft_duct_cut_by_box
loft_loft_rect_circle_rect_ruled
loft_loft_smooth_fuse_spline_sweep
march::walk::tests::march_a_tube_circle_is_refused
offset_rounded_box_whole
part::tests::a_battery_stage_is_held_to_its_class
provenance_consumer_rebuild
query_section_sweep_spline_handle
split_nurbs_cylinder_oblique
sweep_fixtures_read_back
sweep_sweep_arc_tilted_profile
sweep_sweep_pipe_l_bend_cross_fuse
sweep_sweep_rect_s_bend_fillet
sweep_sweep_spline_cut_by_spline_sweep
sweep_sweep_spline_handle
sweep_sweep_spring_cut_split
sweep_sweep_thread
sweep_sweep_thread_chamfer
the_band_at_the_fixtures_numbers
the_horn_and_spindle_edges_of_the_fetched_parts_build_or_are_refused_by_name
the_parallel_cylinder_edges_of_the_fetched_parts_are_counted_by_outcome
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
