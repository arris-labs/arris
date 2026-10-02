//! The fixture corpus, one test per fixture and every variant of it
//! (`docs/ROADMAP.md` §Fixtures): the recipe built in Arris, the checker
//! at `Full`, counts and genus against the oracle, STEP read back by the
//! oracle, provenance accounting, the dump diffed against `dump.txt`.
//! Every fixture is live.

use arris_debug::corpus::{self, CorpusError};
use arris_debug::fixtures;

fn run(name: &str) {
    let dir = fixtures::corpus_root().join(name);
    let fixture = fixtures::load(&dir).unwrap();
    for variant in fixture.recipe.variant_names() {
        if let Err(e) = corpus::run(&dir, &variant) {
            panic!("{name} [{variant}]: {e}");
        }
    }
}

/// A part fixture (ADR-0026): every solid of its file read and held to
/// its recorded outcome.
fn run_part(name: &str) {
    let dir = fixtures::corpus_root().join(name);
    if let Err(e) = arris_debug::part::run(&dir) {
        panic!("{name}: {e}");
    }
}

/// One variant of a fixture, for a recipe whose variants are worth
/// failing apart.
fn run_variant(name: &str, variant: &str) {
    let dir = fixtures::corpus_root().join(name);
    if let Err(e) = corpus::run(&dir, variant) {
        panic!("{name} [{variant}]: {e}");
    }
}

#[test]
fn primitive_box() {
    run("primitive/box");
}

#[test]
fn primitive_cylinder() {
    run("primitive/cylinder");
}

/// A consumer's own topology through `ops::build` (ADR-0028): a
/// tetrahedron, straight and skewed.
#[test]
fn build_tetrahedron() {
    run("build/tetrahedron");
}

/// A concave edge.
#[test]
fn build_l_prism() {
    run("build/l-prism");
}

/// Faces with an inner loop, genus 1.
#[test]
fn build_frame() {
    run("build/frame");
}

#[test]
fn transform_posed_cylinder() {
    run("transform/posed-cylinder");
}

/// The hollow ring moved: two shells carried whole, the cavity still the
/// one lump's void.
#[test]
fn transform_moved_hollow_ring() {
    run("transform/moved-hollow-ring");
}

/// The mirror of a body in a plane beside it, through it and across it
/// (ADR-0031), each against Open CASCADE's mirror.
#[test]
fn transform_mirror_box() {
    run("transform/mirror-box");
}

#[test]
fn transform_mirror_posed_cylinder() {
    run("transform/mirror-posed-cylinder");
}

#[test]
fn transform_mirror_hollow_ring() {
    run("transform/mirror-hollow-ring");
}

#[test]
fn transform_mirror_ball_corner_cut() {
    run("transform/mirror-ball-corner-cut");
}

#[test]
fn transform_mirror_frustum() {
    run("transform/mirror-frustum");
}

#[test]
fn transform_mirror_elliptic_cylinder() {
    run("transform/mirror-elliptic-cylinder");
}

#[test]
fn transform_mirror_filleted_box() {
    run("transform/mirror-filleted-box");
}

/// A box with a cylinder boss, fused, mirrored: the mirror of a boolean's
/// result (ADR-0031).
#[test]
fn transform_mirror_fused_boss() {
    run("transform/mirror-fused-boss");
}

#[test]
fn boolean_through_hole() {
    run("boolean/through-hole");
}

#[test]
fn boolean_blind_hole() {
    run("boolean/blind-hole");
}

/// A solid read from a STEP file as an operand — Arris's own STEP of
/// `boolean/through-hole`'s result — cut by a box through its hole: the
/// `step` op read by both kernels' readers (ADR-0026).
#[test]
fn boolean_step_operand_cut() {
    run("boolean/step-operand-cut");
}

/// The consumer's through-hole probe in its own units: a 0.1 m plate less
/// a r 0.02 cylinder, in a model whose default tolerance is a micrometre
/// — `w²t − πr²t = 8.7434e-5`.
#[test]
fn boolean_probe_through_hole_m() {
    run("boolean/probe-through-hole-m");
}

/// The consumer's blind-hole probe in the same units: the floor kept,
/// `w²t − πr²d = 1.8743e-4`.
#[test]
fn boolean_probe_blind_hole_m() {
    run("boolean/probe-blind-hole-m");
}

/// The consumer's flush-union probe in the same units: two 1 m cubes
/// sharing a face, volume 2.0.
#[test]
fn boolean_probe_flush_union_m() {
    run("boolean/probe-flush-union-m");
}

#[test]
fn boolean_bolt_pattern_8() {
    run("boolean/bolt-pattern-8");
}

/// Two boxes sharing a face: the flush case. The shared face vanishes
/// and the four edges around it are held once, from the first operand.
#[test]
fn boolean_flush_union() {
    run("boolean/flush-union");
}

#[test]
fn boolean_corner_union() {
    run("boolean/corner-union");
}

#[test]
fn boolean_corner_common() {
    run("boolean/corner-common");
}

/// A body united with, minus and intersected with its own mirror image
/// (ADR-0031): a mirrored operand is an operand like any other.
#[test]
fn boolean_mirror_ball_corner_cut_fuse() {
    run("boolean/mirror-ball-corner-cut-fuse");
}

#[test]
fn boolean_mirror_box_fuse() {
    run("boolean/mirror-box-fuse");
}

#[test]
fn boolean_mirror_box_cut() {
    run("boolean/mirror-box-cut");
}

#[test]
fn boolean_mirror_box_common() {
    run("boolean/mirror-box-common");
}

#[test]
fn boolean_mirror_cylinders_fuse() {
    run("boolean/mirror-cylinders-fuse");
}

/// The oblique mirror of a cylinder, united: Open CASCADE cuts one more section arc
/// at an ellipse's parameter origin (`analytic.counts_differ`).
#[test]
fn boolean_mirror_cylinders_oblique_fuse() {
    run("boolean/mirror-cylinders-oblique-fuse");
}

/// The oblique mirror of a cylinder, intersected: Open CASCADE cuts one more section arc
/// at an ellipse's parameter origin (`analytic.counts_differ`).
#[test]
fn boolean_mirror_cylinders_oblique_common() {
    run("boolean/mirror-cylinders-oblique-common");
}

/// Touching images have no material in common: a typed refusal, as the
/// oracle's empty result.
#[test]
fn boolean_mirror_box_touching_common() {
    run("boolean/mirror-box-touching-common");
}

#[test]
fn boolean_mirror_cylinders_touching_common() {
    run("boolean/mirror-cylinders-touching-common");
}

#[test]
fn boolean_mirror_cylinders_cut() {
    run("boolean/mirror-cylinders-cut");
}

#[test]
fn boolean_mirror_cylinders_common() {
    run("boolean/mirror-cylinders-common");
}

#[test]
fn boolean_corner_cut() {
    run("boolean/corner-cut");
}

/// The two flush boxes' common is the shared face alone: nothing with
/// thickness, the runner's degenerate path.
#[test]
fn boolean_flush_common() {
    run("boolean/flush-common");
}

#[test]
fn boolean_disjoint_cut() {
    run("boolean/disjoint-cut");
}

#[test]
fn boolean_frame_cut() {
    run("boolean/frame-cut");
}

/// The target inside the tool: nothing survives, and the oracle records
/// no solid — the runner's degenerate path.
#[test]
fn boolean_swallow_cut() {
    run("boolean/swallow-cut");
}

/// A slab through the plate: two boxes, two lumps of one solid — Open
/// CASCADE's two solids (ADR-0006).
#[test]
fn boolean_split_cut() {
    run("boolean/split-cut");
}

/// The consumer's hollow box in its own units: one lump, its cavity a
/// void shell made of the tool's faces turned inward.
#[test]
fn boolean_enclosed_cavity() {
    run("boolean/enclosed-cavity");
}

/// Operands apart: two lumps, every face of both kept by id.
#[test]
fn boolean_disjoint_fuse() {
    run("boolean/disjoint-fuse");
}

/// A cylinder wholly inside a box: a void of three faces, the seam
/// carried with them.
#[test]
fn boolean_cavity_cylinder() {
    run("boolean/cavity-cylinder");
}

/// A box inside the cavity of a hollow box: three shells, two lumps, one
/// of them in the other's void.
#[test]
fn boolean_lump_in_cavity() {
    run("boolean/lump-in-cavity");
}

/// Two boxes touching along an edge: Open CASCADE builds two solids that
/// share it, Arris refuses with `Reason::NonManifold` — the runner's
/// expected-error path.
#[test]
fn boolean_edge_touching_fuse() {
    run("boolean/edge-touching-fuse");
}

#[test]
fn boolean_boss() {
    run("boolean/boss");
}

/// The boss's bottom cap coincident with the plate's top: the cap
/// vanishes, the plate's top is split by the cap's rim and keeps the
/// outside, and the rim is the wall's own edge.
#[test]
fn boolean_boss_flush() {
    run("boolean/boss-flush");
}

/// A rod filling a tube's bore: the coincident cylinder walls vanish,
/// the rod's discs sit beside the tube's annuli sharing the inner
/// circles — the cylinder–cylinder coincident arm, and the common
/// blocks of a periodic edge.
#[test]
fn boolean_coaxial_fuse() {
    run("boolean/coaxial-fuse");
}

/// A hole drilled at 30°: two ellipse sections, NURBS pcurves on the
/// wall, and a wall whose (u, v) region is a strip oblique to the
/// ruling — the case the tessellator flattens the ruled direction for
/// (ADR-0005).
#[test]
fn boolean_oblique_hole() {
    run("boolean/oblique-hole");
}

/// A cylinder touching the plate's side face from outside: the tangent
/// pair contributes no section edge and no split, and the result is
/// the plate with every id kept. Open CASCADE imprints the ruling; the
/// fixture states that convention in `analytic.counts_differ`.
#[test]
fn boolean_tangent_outside_cut() {
    run("boolean/tangent-outside-cut");
}

/// A blind hole whose wall touches a side face from inside along a
/// ruling interior to both: the slit no manifold `Solid` can carry,
/// `Reason::TangentContact` through the runner's expected-error path
/// (plan m4-booleans `⚠ OPEN` 1).
#[test]
fn boolean_tangent_hole() {
    run("boolean/tangent-hole");
}

/// The same solid as `through-hole` in another pose: both operands moved
/// by one rigid motion before the cut.
#[test]
fn boolean_posed_through_hole() {
    run("boolean/posed-through-hole");
}

/// Two coaxial cylinders: the pair has no section curve, and the bore is
/// the tool's wall reversed.
#[test]
fn boolean_coaxial_cut() {
    run("boolean/coaxial-cut");
}

/// Operands that share no material: the runner's degenerate path through
/// `common`.
#[test]
fn boolean_disjoint_common() {
    run("boolean/disjoint-common");
}

/// A sliver: the top and bottom faces are a D of one straight edge and
/// one arc under an eighth of a turn, which the checker's minimum
/// discretisation once flattened to a chord (plan step 9).
#[test]
fn boolean_sliver_common() {
    run("boolean/sliver-common");
}

/// Two equal cylinders crossing at 90°: the Steinmetz solid. The two
/// section ellipses cross each other at (0, ±R, 0), a section vertex no
/// operand edge made, and each wall's lens through the seam is two
/// faces. Open CASCADE also cuts an arc at the ellipse's parameter
/// origin; the fixture states that convention in `analytic.counts_differ`.
#[test]
fn boolean_cross_cylinders_common() {
    run("boolean/cross-cylinders-common");
}

/// The same cylinders fused: each wall keeps its two pieces outside the
/// other, meeting at the crossing vertices.
#[test]
fn boolean_cross_cylinders_fuse() {
    run("boolean/cross-cylinders-fuse");
}

/// The target minus the tool that is as wide as it: two lumps whose
/// closures touch at the crossing vertices, `Reason::NonManifold`
/// through the runner's expected-error path (ADR-0006).
#[test]
fn boolean_cross_cylinders_cut() {
    run("boolean/cross-cylinders-cut");
}

/// The common at ψ = 60°: ellipses of two different major radii still
/// crossing at (0, ±R, 0), `16R³/(3 sin ψ)`.
#[test]
fn boolean_oblique_cross_common() {
    run("boolean/oblique-cross-common");
}

/// The Steinmetz solid with the tool's seam through a crossing vertex:
/// the seam touches the target's wall there, and the touch landing on
/// the section vertex cuts the seam. Open CASCADE's extra arc vertex is
/// stated in `analytic.counts_differ`.
#[test]
fn boolean_seam_through_crossing_common() {
    run("boolean/seam-through-crossing-common");
}

/// The crossing-cylinder fuse with the second cylinder's seam turned to
/// just beside a crossing vertex rather than through it: the same solid
/// as at any turn, so the result is held to cross-cylinders-fuse's
/// closed forms — which Open CASCADE misses in this band, stated in
/// `analytic.measure_differs` (ADR-0015). The seam's touch is resolved
/// into its crossings (ADR-0016) and the sliver between the seam and the
/// two ellipses is decided by the transversal rule.
#[test]
fn boolean_seam_beside_crossing_fuse() {
    run("boolean/seam-beside-crossing-fuse");
}

/// The same fuse with the seam between one and two tolerances from the
/// crossing vertex: its two crossings and that vertex are three points
/// one vertex by closure, whose balls meet though no two lie within one
/// tolerance, and its point is the seam's. The section edges' pcurves
/// end on that vertex's own (u, v) on each face — the seam's pave on the
/// turned wall — and the body is the one at −90°.
#[test]
fn boolean_seam_a_tolerance_from_crossing_fuse() {
    run("boolean/seam-a-tolerance-from-crossing-fuse");
}

/// flush-union's boxes a quarter of a tolerance into and away from each other, and four apart, fused: the flush union within the tolerance, two lumps beyond (ADR-0022).
#[test]
fn boolean_flush_union_band_ends() {
    run("boolean/flush-union-band-ends");
}

/// boss-flush's boss a quarter of a tolerance into and off its plate, and four off, fused (ADR-0022).
#[test]
fn boolean_boss_flush_band_ends() {
    run("boolean/boss-flush-band-ends");
}

/// pin-in-bore-fuse's pin a quarter of a tolerance out through and in off the bore's wall, and four in, fused (ADR-0022).
#[test]
fn boolean_pin_in_bore_fuse_band_ends() {
    run("boolean/pin-in-bore-fuse-band-ends");
}

/// coaxial-fuse's rod a quarter of a tolerance off the tube's axis either way, fused: the flush union (ADR-0022).
#[test]
fn boolean_coaxial_fuse_band_ends() {
    run("boolean/coaxial-fuse-band-ends");
}

/// coaxial-cut's tool a quarter of a tolerance and four off the shared axis (ADR-0022).
#[test]
fn boolean_coaxial_cut_band_ends() {
    run("boolean/coaxial-cut-band-ends");
}

/// tangent-cylinders-cut's tool four tolerances into and away from the target (ADR-0022).
#[test]
fn boolean_tangent_cylinders_cut_band_ends() {
    run("boolean/tangent-cylinders-cut-band-ends");
}

/// tangent-hole's hole four tolerances out through and in off the side face: a clean cut either way (ADR-0022).
#[test]
fn boolean_tangent_hole_band_ends() {
    run("boolean/tangent-hole-band-ends");
}

/// tangent-hole's hole a quarter of a tolerance out through and in off the side face: the contact's own refusal, Reason::TangentContact (ADR-0022).
#[test]
fn boolean_tangent_hole_a_quarter_off() {
    run("boolean/tangent-hole-a-quarter-off");
}

/// tangent-outside-cut's tool four tolerances into and away from the plate, and turned half a tolerance about the contact's middle (ADR-0022).
#[test]
fn boolean_tangent_outside_cut_band_ends() {
    run("boolean/tangent-outside-cut-band-ends");
}

/// pipe-elbow-fuse's pipe a quarter of a tolerance into and away from the bend, a whole one in, and grown by half of one, fused (ADR-0022).
#[test]
fn boolean_pipe_elbow_fuse_band_ends() {
    run("boolean/pipe-elbow-fuse-band-ends");
}

/// edge-touching-fuse's boxes a quarter of a tolerance into each other and apart across their shared edge, and four apart, the first cut by the second (ADR-0022).
#[test]
fn boolean_edge_touching_band_ends() {
    run("boolean/edge-touching-band-ends");
}

/// flush-union's boxes four tolerances into each other, fused: one box with the 4e-7 strip where their side faces overlap kept as a face (ADR-0022).
#[test]
fn boolean_flush_union_four_into() {
    run("boolean/flush-union-four-into");
}

/// boss-flush's boss four tolerances into its plate, fused, held to its closed forms where Open CASCADE's own fuse is the far side (ADR-0022).
#[test]
fn boolean_boss_flush_four_into() {
    run("boolean/boss-flush-four-into");
}

/// pin-in-bore-fuse's pin four tolerances out through the bore's wall, fused: a lens 4e-7 proud of the wall (ADR-0022).
#[test]
fn boolean_pin_in_bore_four_out() {
    run("boolean/pin-in-bore-four-out");
}

/// coaxial-fuse's rod four tolerances off the tube's axis along +x, fused: a crescent hole 4e-7 wide, genus 1 (ADR-0022).
#[test]
fn boolean_coaxial_fuse_four_off() {
    run("boolean/coaxial-fuse-four-off");
}

/// coaxial-fuse's rod four tolerances off the tube's axis along −x, fused: the same crescent hole the other way (ADR-0022).
#[test]
fn boolean_coaxial_fuse_four_off_back() {
    run("boolean/coaxial-fuse-four-off-back");
}

/// tangent-cylinders-cut's tool a quarter of a tolerance into and away from the target: the target whole, as at the contact (ADR-0022).
#[test]
fn boolean_tangent_cylinders_a_quarter_off() {
    run("boolean/tangent-cylinders-a-quarter-off");
}

/// tangent-outside-cut's tool a quarter of a tolerance into and away from the plate: the plate whole, as at the contact (ADR-0022).
#[test]
fn boolean_tangent_outside_a_quarter_off() {
    run("boolean/tangent-outside-a-quarter-off");
}

/// edge-touching-fuse's boxes four tolerances into each other across their shared edge, the first cut by the second: a notch 4e-7 deep (ADR-0022).
#[test]
fn boolean_edge_touching_four_into() {
    run("boolean/edge-touching-four-into");
}

/// edge-touching-fuse's boxes, the second turned a tolerance about the shared edge's middle, cut: the tool's edge crosses the box's there, the section block along both edges at once is each face's boundary (ADR-0022).
#[test]
fn boolean_edge_touching_tilted_cut() {
    run("boolean/edge-touching-tilted-cut");
}

/// The common of the same cylinders with the seam just past two
/// tolerances from the crossing vertex: its two crossings are vertices
/// of their own, 3.1e-7 from that vertex, and the piece of the turned
/// wall between them and the two ellipses is a sliver face a few
/// tolerances across. The desired body is the Steinmetz solid at a
/// generic turn.
/// The ring torus mirrored in a plane beside it, through it and across it:
/// Arris's mirror is clean and matches the oracle in every stage but the
/// last, where its own reader refuses Open CASCADE's STEP of the mirrored
/// A ball united with itself turned half a turn: one sphere under two
/// frames, which the split leaves with a dangling face. A body mirrored
/// through its centre is this case.
#[test]
#[ignore = "kernel bug: the face arrangement is not a subdivision, a section edge of f0 ends at a node nothing else reaches: coincident spheres under frames a half turn apart are not one surface to the split (docs/BACKLOG.md, coincident quadrics under different frames)"]
fn regression_coincident_spheres_rotated_frame() {
    run("regression/coincident-spheres-rotated-frame");
}

/// ring.
#[test]
#[ignore = "L4 f0 no outer loop: the reader builds no outer loop for the torus face of Open CASCADE's STEP of a mirrored ring, whose torus frame is left-handed (docs/BACKLOG.md, a reader of left-handed surface frames)"]
fn regression_mirror_torus_ring() {
    run("regression/mirror-torus-ring");
}

#[test]
#[ignore = "L4, a loop of zero signed area: the sliver face between the seam's two crossings and the crossing vertex, three vertices a few tolerances apart, is kept and its (u, v) polygon does not resolve it (docs/BACKLOG.md, material a tolerance or two thick)"]
fn regression_seam_two_tolerances_from_crossing_common() {
    run("regression/seam-two-tolerances-from-crossing-common");
}

/// A drill touching the main wall from inside, at a singular point of the
/// traced section: the pave model makes the point one section vertex
/// that ends both branches, and the main wall is left two pieces meeting
/// only there — one shell touching itself at a point.
#[test]
#[ignore = "OpError::Degenerate, Reason::NonManifold naming the singular vertex: the main wall's two pieces meet only there, pinched between the drill's exits, a shell touching itself at a point that a manifold Solid does not hold; the desired body is a General one (docs/BACKLOG.md, a shell touching itself at a vertex)"]
fn regression_singular_bore_cut() {
    run("regression/singular-bore-cut");
}

/// A ball sliced by a face 3.7e-7 from its pole: not through the singular
/// point and nearer than the sphere's (u, v) polygons resolve, refused by
/// name rather than built.
#[test]
#[ignore = "Reason::BesideSingularity, a section passes the sphere's pole without running through it, nearer than the face's (u, v) resolves (docs/BACKLOG.md, polygons by span and chords in length)"]
fn regression_ball_beside_pole_slice_cut() {
    run("regression/ball-beside-pole-slice-cut");
}

/// flush-union's second box a tolerance off the shared face: moved away, into the first, or turned about the face's middle line.
#[test]
#[ignore = "Fault::Split EmptySubEdge, Fault::CommonBlock, Fault::Split Turn, and plane against plane Unsupported a hair off parallel (docs/BACKLOG.md, ADR-0022)"]
fn regression_flush_union_a_tolerance_off() {
    run("regression/flush-union-a-tolerance-off");
}

/// boss-flush's boss turned about a diameter of its cap by a quarter of a tolerance, or two.
#[test]
#[ignore = "plane against plane Unsupported a hair off parallel; beyond it Fault::Split, a section edge ends at a node nothing else reaches (docs/BACKLOG.md, ADR-0022)"]
fn regression_boss_flush_tilted() {
    run("regression/boss-flush-tilted");
}

/// A boss lifted a tolerance off its plate, posed: the union fails the checker.
#[test]
#[ignore = "plane against plane Unsupported a hair off parallel once posed; the survey built the same pair into a union that fails the checker (docs/BACKLOG.md, ADR-0022)"]
fn regression_boss_flush_posed_gap_fuse() {
    run("regression/boss-flush-posed-gap-fuse");
}

/// pin-in-bore-fuse's pin a tolerance or two out through the bore's wall, turned, or grown.
#[test]
#[ignore = "Fault::Split Dangling and NoInterior, and a union that fails the checker (docs/BACKLOG.md, ADR-0022)"]
fn regression_pin_in_bore_a_tolerance_off() {
    run("regression/pin-in-bore-a-tolerance-off");
}

/// coaxial-fuse's rod a tolerance or two off the tube's axis, turned about its middle, or grown.
#[test]
#[ignore = "Fault::CommonBlock, Fault::Split NoInterior and Dangling, Fault::Builder EdgeUses, circle against plane Unsupported (docs/BACKLOG.md, ADR-0022)"]
fn regression_coaxial_fuse_a_tolerance_off() {
    run("regression/coaxial-fuse-a-tolerance-off");
}

/// A rod filling a tube's bore, turned a quarter of a tolerance: a section edge crosses a seam.
#[test]
#[ignore = "Fault::Seam, a section edge crosses a seam without a pave there (docs/BACKLOG.md, ADR-0022)"]
fn regression_coaxial_fuse_tilted_seam() {
    run("regression/coaxial-fuse-tilted-seam");
}

/// A rod filling a tube's bore, turned a quarter of a tolerance: a cylinder is placed with no frame.
#[test]
#[ignore = "Fault::Geometry, a degenerate cylinder surface with a non-finite frame (docs/BACKLOG.md, ADR-0022)"]
fn regression_coaxial_fuse_tilted_frame() {
    run("regression/coaxial-fuse-tilted-frame");
}

/// tangent-cylinders-cut's tool a tolerance into the target, or turned two about the contact's middle.
#[test]
#[ignore = "Fault::Split, a section edge ends at a node nothing else reaches (docs/BACKLOG.md, ADR-0022)"]
fn regression_tangent_cylinders_a_tolerance_in() {
    run("regression/tangent-cylinders-a-tolerance-in");
}

/// tangent-cylinders-cut's operands two tolerances into each other, intersected: a sliver the oracle builds no solid of.
#[test]
#[ignore = "the common fails the checker, a panic in a debug build: L4, sliver loops of zero signed area, where the desired is a refusal (docs/BACKLOG.md, ADR-0022)"]
fn regression_tangent_cylinders_overlap_common() {
    run("regression/tangent-cylinders-overlap-common");
}

/// tangent-cylinders-fuse's tool turned a tolerance and a half about the contact's middle.
#[test]
#[ignore = "Fault::Lumps, two shells of the result meet, where the desired is TangentContact (docs/BACKLOG.md, ADR-0022)"]
fn regression_tangent_cylinders_tilted_fuse() {
    run("regression/tangent-cylinders-tilted-fuse");
}

/// tangent-hole's operands fused, at the contact and a tolerance out: the section circle is tangent to the top face's edge.
#[test]
#[ignore = "Fault::Builder EdgeUses, where the desired is TangentContact (docs/BACKLOG.md, ADR-0022)"]
fn regression_tangent_hole_fuse() {
    run("regression/tangent-hole-fuse");
}

/// tangent-hole's hole a tolerance and a half out through the side face, or turned about the contact's middle.
#[test]
#[ignore = "Fault::Split NoInterior and Dangling, plane against cylinder Unsupported, where the desired is TangentContact (docs/BACKLOG.md, ADR-0022)"]
fn regression_tangent_hole_a_tolerance_out() {
    run("regression/tangent-hole-a-tolerance-out");
}

/// A blind hole touching a side face, turned a tolerance, posed: a body the checker refuses.
#[test]
#[ignore = "a body where TangentContact is desired, which S5 refuses: two faces intersect away from their shared edges (docs/BACKLOG.md, ADR-0022)"]
fn regression_tangent_hole_tilted_posed_cut() {
    run("regression/tangent-hole-tilted-posed-cut");
}

/// tangent-outside-cut's tool a tolerance into the plate, or turned a quarter or a half about the contact's middle.
#[test]
#[ignore = "Fault::Split NoInterior and Dangling, and a section fit that still deviates at its most spans (docs/BACKLOG.md, ADR-0022)"]
fn regression_tangent_outside_a_tolerance_in() {
    run("regression/tangent-outside-a-tolerance-in");
}

/// A cylinder touching a box face from outside, grown a tolerance, intersected: a hole of the arrangement in no region.
#[test]
#[ignore = "Fault::Split Hole, where the desired is a refusal (docs/BACKLOG.md, ADR-0022)"]
fn regression_tangent_outside_grown_common() {
    run("regression/tangent-outside-grown-common");
}

/// A cylinder touching a box face from outside, turned four tolerances, posed, intersected: a sliver the checker refuses.
#[test]
#[ignore = "a body where a refusal is desired, which B1 refuses: no outer shell (docs/BACKLOG.md, ADR-0022)"]
fn regression_tangent_outside_tilted_posed_common() {
    run("regression/tangent-outside-tilted-posed-common");
}

/// A cylinder touching a box face from outside, turned a quarter of a tolerance, posed, fused.
#[test]
#[ignore = "Fault::Geometry, the section fit still deviates by 1.7e-7 (at step 1 the ellipse section off its surfaces by 1.2e-7), where the desired is TangentContact (docs/BACKLOG.md, ADR-0022)"]
fn regression_tangent_outside_tilted_posed_fuse() {
    run("regression/tangent-outside-tilted-posed-fuse");
}

/// pipe-elbow-fuse's pipe a tolerance or so into the bend, turned about its cap's middle, or grown.
#[test]
#[ignore = "Fault::CommonBlock, Fault::Split Turn, torus sections called degenerate (Fault::Geometry), plane against plane and circle against plane Unsupported (docs/BACKLOG.md, ADR-0022)"]
fn regression_pipe_elbow_a_tolerance_off() {
    run("regression/pipe-elbow-a-tolerance-off");
}

/// A pipe pushed a tolerance into its bend, posed: a fit's normal equations are singular.
#[test]
#[ignore = "Fault::Split, a cycle does not turn once; the survey built the same pair into a fit whose normal equations are singular (docs/BACKLOG.md, ADR-0022)"]
fn regression_pipe_elbow_posed_fuse() {
    run("regression/pipe-elbow-posed-fuse");
}

/// Two boxes touching along an edge, the second turned half a tolerance, posed, cut.
#[test]
#[ignore = "one vertex too many (10/14): the edges cross at a grazing angle and the crossing's hits, scattered along the edge by the rounding over it, are two vertices (docs/BACKLOG.md, a hair off parallel; ADR-0022)"]
fn regression_edge_touching_tilted_posed_cut() {
    run("regression/edge-touching-tilted-posed-cut");
}

/// A tilted cylinder less a slab whose rectangle crosses its section on
/// all four sides (prop::recipe's draw, shrunk).
#[test]
fn boolean_tilted_cylinder_slot_cut() {
    run("boolean/tilted-cylinder-slot-cut");
}

/// A disc in common with a pin crossing its wall, fused with a parallel
/// thinner disc (prop::recipe's draw, shrunk).
#[test]
fn boolean_pin_at_disc_rim_common_fuse() {
    run("boolean/pin-at-disc-rim-common-fuse");
}

/// A cylinder fused with a revolved profile, less a posed extrusion (the differential's draw, shrunk; ADR-0024 step
/// 5b).
#[test]
#[ignore = "the cut's result fails the checker's L5 (a loop intersects itself) (docs/BACKLOG.md, the differential's findings)"]
fn regression_revolve_fuse_extrude_cut_loop_crosses_itself() {
    run("regression/revolve-fuse-extrude-cut-loop-crosses-itself");
}

/// A revolved profile fused with a posed cylinder and then an extrusion (the differential's draw, shrunk; ADR-0024 step
/// 5b).
#[test]
#[ignore = "one shell and three faces fewer than Open CASCADE's: a cavity is missing (docs/BACKLOG.md, the differential's findings)"]
fn regression_revolve_cylinder_extrude_fuse_misses_a_shell() {
    run("regression/revolve-cylinder-extrude-fuse-misses-a-shell");
}

/// A revolved sketch fused with an extrusion and cut by a second one,
/// drawn by the differential on a nightly seed (`e4bc2ddf…`, case 757)
/// and shrunk: its section curves carry knots a rounding apart, which a
/// knot rule refusing them turned into the checker's S5 on a cone face
/// against a cylinder face.
#[test]
fn boolean_revolve_extrude_fuse_cut_rounding_knots() {
    run("boolean/revolve-extrude-fuse-cut-rounding-knots");
}

/// A full revolve less an elliptic extrusion (the differential's draw, shrunk; ADR-0024 step
/// 5b).
#[test]
#[ignore = "more faces, edges and vertices than Open CASCADE's (docs/BACKLOG.md, the differential's findings)"]
fn regression_revolve_cut_by_extrusion_extra_faces() {
    run("regression/revolve-cut-by-extrusion-extra-faces");
}

/// A revolved profile fused with a posed box (the differential's draw, shrunk; ADR-0024 step
/// 5b).
#[test]
#[ignore = "the tessellation at chord 1e-3 is not closed (docs/BACKLOG.md, the differential's findings)"]
fn regression_revolve_box_fuse_mesh_not_closed() {
    run("regression/revolve-box-fuse-mesh-not-closed");
}

/// Three posed cylinders fused in turn (the differential's draw, shrunk; ADR-0024 step
/// 5b).
#[test]
#[ignore = "the second fuse returns OpError::Internal(Split) (docs/BACKLOG.md, the differential's findings)"]
fn regression_three_cylinders_fuse_split_fault() {
    run("regression/three-cylinders-fuse-split-fault");
}

/// A mirrored revolve and a mirrored extrusion intersected with a posed cylinder in turn (the differential's draw, shrunk; CI run 36786571173).
#[test]
#[ignore = "the second common returns OpError::Internal(Seam) (docs/BACKLOG.md, the differential's findings)"]
fn regression_mirrored_revolve_common_seam_fault() {
    run("regression/mirrored-revolve-common-seam-fault");
}

/// A box, a revolved profile and a chamfered cylinder fused in turn (the differential's draw, shrunk; ADR-0024 step
/// 5b).
#[test]
#[ignore = "a fuse returns OpError::Internal(Geometry) (docs/BACKLOG.md, the differential's findings)"]
fn regression_box_revolve_cylinder_chamfer_fuse_geometry_fault() {
    run("regression/box-revolve-cylinder-chamfer-fuse-geometry-fault");
}

/// A box, a revolved profile and a cylinder fused in turn (the differential's draw, shrunk; ADR-0024 step
/// 5b).
#[test]
#[ignore = "a fuse returns OpError::Internal(Lumps) (docs/BACKLOG.md, the differential's findings)"]
fn regression_box_revolve_cylinder_fuse_lumps_fault() {
    run("regression/box-revolve-cylinder-fuse-lumps-fault");
}

/// A box in common with a revolved profile, fused with a cylinder (the differential's draw, shrunk; ADR-0024 step
/// 5b).
#[test]
#[ignore = "a boolean returns OpError::Internal(Builder) (docs/BACKLOG.md, the differential's findings)"]
fn regression_box_revolve_cylinder_common_fuse_builder_fault() {
    run("regression/box-revolve-cylinder-common-fuse-builder-fault");
}

/// A cylinder along x lying on a plate, its seam on the touch, cut from
/// the plate (boolean_prop's tangent pair at 5000 cases, shrunk;
/// ADR-0024).
#[test]
#[ignore = "Fault::Split, a section edge ending at a node nothing else reaches, where the desired cut is the plate (docs/BACKLOG.md, the seam on a touch)"]
fn regression_tangent_seam_on_face_cut() {
    run("regression/tangent-seam-on-face-cut");
}

/// A square less a quarter disc, extruded and cut by a box inside it:
/// the arc is tangent to both lines it meets, so each cap's loop has two
/// cusps, each a turn of +π round its spike (ADR-0026 §4, the battery's
/// CTC-04 and FTC-06).
#[test]
fn boolean_spandrel_cavity_cut() {
    run("boolean/spandrel-cavity-cut");
}

/// NIST's CTC-04 cut by a speck of a box: its plane f393, bounded by
/// B-spline edges that leave a line tangent to it, has two cusps, and
/// splits into one piece that turns once (ADR-0026 §4).
#[test]
fn boolean_nist_ctc_04_face_arrangement_turn() {
    run("boolean/nist-ctc-04-face-arrangement-turn");
}

/// NIST's FTC-06 cut by a speck of a box: its plane f0, whose loop runs
/// between small blends tangent to its lines, has ten cusps, and splits
/// into one piece that turns once; its counts are its own reading's,
/// Open CASCADE's healed (ADR-0026 §3, §4).
#[test]
fn boolean_nist_ftc_06_face_arrangement_turn() {
    run("boolean/nist-ftc-06-face-arrangement-turn");
}

/// An all-NURBS box cut by a box inside it: the tool's faces are
/// classified by a ray against NURBS faces, which has no closed form, and
/// the boolean refuses the pair by name, the NURBS cycle's count
/// (ADR-0026 §5) — the battery's every cut and drill of FTC-07 and
/// FTC-10.
#[test]
fn boolean_nurbs_box_cavity_cut() {
    run("boolean/nurbs-box-cavity-cut");
}

/// A tee of equal radii: the branch's rim circle touches the main wall
/// exactly at the two crossing vertices, and each touch cuts the rim
/// there.
#[test]
fn boolean_tee_fuse() {
    run("boolean/tee-fuse");
}

/// A tee of unequal radii: the branch's wall meets the main wall in one
/// traced loop, one periodic fitted curve paved once where the branch's
/// seam pierces the main wall (ADR-0018).
#[test]
fn boolean_tee_unequal_fuse() {
    run("boolean/tee-unequal-fuse");
}

/// The same operands' cut: a pocket opening through the main wall along
/// the loop.
#[test]
fn boolean_tee_unequal_cut() {
    run("boolean/tee-unequal-cut");
}

/// The same operands' common: the branch inside the main.
#[test]
fn boolean_tee_unequal_common() {
    run("boolean/tee-unequal-common");
}

/// A drill on a skew axis breaking out of the main cylinder's side: one
/// traced loop, a notch.
#[test]
fn boolean_skew_hole_cut() {
    run("boolean/skew-hole-cut");
}

/// A drill on a skew axis inside the main cylinder: two traced loops, a
/// bore, genus 1.
#[test]
fn boolean_skew_bore_cut() {
    run("boolean/skew-bore-cut");
}

/// A slot across the fused tee's junction: the box's planes cross the
/// tee's fitted edge (ADR-0018) in four section vertices, found by the
/// NURBS curve against a plane, and every edge stays at its faces'
/// tolerance.
#[test]
fn boolean_tee_unequal_slot_cut() {
    run("boolean/tee-unequal-slot-cut");
}

/// A drill through the fused tee's junction: its wall meets both walls in
/// traced loops that cross the tee's fitted edge, found by the NURBS
/// curve against a cylinder.
#[test]
fn boolean_tee_unequal_drill_cut() {
    run("boolean/tee-unequal-drill-cut");
}

/// The consumer's cylinder − cylinder transversal probe in its own units:
/// two parallel walls meeting in two rulings, `(πr² − lens)·h =
/// 2.2079e-5`.
#[test]
fn boolean_parallel_cylinders_cut() {
    run("boolean/parallel-cylinders-cut");
}

/// The same operands' common: the lens prism.
#[test]
fn boolean_parallel_cylinders_common() {
    run("boolean/parallel-cylinders-common");
}

/// The same operands fused, and flush: the rim circles crossing on the
/// coincident caps at the rulings' ends.
#[test]
fn boolean_parallel_cylinders_fuse() {
    run("boolean/parallel-cylinders-fuse");
}

/// A ruling on the target's seam: that block is the seam edge, not a
/// section edge.
#[test]
fn boolean_parallel_cylinders_seam() {
    run("boolean/parallel-cylinders-seam");
}

/// Two walls touching from outside along a ruling: the curvature rule puts
/// each outside the other, and the cut is the target with every id kept.
/// Open CASCADE imprints the ruling (`analytic.counts_differ`).
#[test]
fn boolean_tangent_cylinders_cut() {
    run("boolean/tangent-cylinders-cut");
}

/// The same operands fused: both walls survive through the contact,
/// `Reason::TangentContact` through the runner's expected-error path.
#[test]
fn boolean_tangent_cylinders_fuse() {
    run("boolean/tangent-cylinders-fuse");
}

/// A pin touching a bore's wall from inside: the pin's wall is inside the
/// bore, and the fuse is the bore with every id kept.
#[test]
fn boolean_pin_in_bore_fuse() {
    run("boolean/pin-in-bore-fuse");
}

/// The pin cut from the bore: both walls survive through the contact,
/// `Reason::TangentContact`.
#[test]
fn boolean_pin_in_bore_cut() {
    run("boolean/pin-in-bore-cut");
}

/// Two short cylinders crossing at 30°: each of the tool's cap planes cuts
/// the target's wall in an ellipse coplanar with the tool's rim circle, and
/// whether that rim lies along the ellipse is decided without the quartic.
#[test]
fn boolean_short_cross_cylinders_fuse() {
    run("boolean/short-cross-cylinders-fuse");
}

/// A box with an elliptic cylinder cut through it: an elliptic-cylinder
/// operand face (ADR-0014).
#[test]
fn boolean_elliptic_operand_cut() {
    run("boolean/elliptic-operand-cut");
}

/// Two elliptic prisms crossing at right angles, fused: an elliptic
/// cylinder on both operands, meeting in the two conics the unit
/// Steinmetz solid's planes pull back to.
#[test]
fn boolean_elliptic_cross_fuse() {
    run("boolean/elliptic-cross-fuse");
}

/// A bored frustum cut by a tilted half-space: a whole ellipse on the
/// cone, its pcurve fitted over the cone's own projection (ADR-0021).
#[test]
fn boolean_frustum_oblique_cut() {
    run("boolean/frustum-oblique-cut");
}

/// A slot cut past the same frustum: three planes parallel to the cone's
/// axis, each meeting it in an exact hyperbola (ADR-0018).
#[test]
fn boolean_frustum_slot_cut() {
    run("boolean/frustum-slot-cut");
}

/// A drill across the same frustum's wall: a cone and a cylinder on skew
/// axes, two traced and fitted quartic loops (ADR-0018, ADR-0019).
#[test]
fn boolean_frustum_cross_drill_cut() {
    run("boolean/frustum-cross-drill-cut");
}

/// A boss chamfered and then slotted: the cone Arris made taken back as
/// a boolean operand, which is the closure ADR-0020 is about.
#[test]
fn boolean_chamfered_boss_slot_cut() {
    run("boolean/chamfered-boss-slot-cut");
}

/// A ball with a box's corner cut out of it: three small circles on a
/// whole sphere face, each pcurve fitted over the sphere's own projection.
#[test]
fn boolean_ball_corner_cut() {
    run("boolean/ball-corner-cut");
}

/// A drill through a ball beside its axis: a sphere and a cylinder on
/// skew axes, two traced loops.
#[test]
fn boolean_ball_offset_drill_cut() {
    run("boolean/ball-offset-drill-cut");
}

/// A bar through a ball that swallows one of its poles: the section loop
/// winds once round `u`, from the seam back to the seam, and ends there
/// at a latitude where a tolerance is a wide step in `u`.
#[test]
fn boolean_ball_polar_drill_cut() {
    run("boolean/ball-polar-drill-cut");
}

/// Two balls revolved about different axes, in common: a lens between
/// two sphere faces meeting in one circle.
#[test]
fn boolean_ball_ball_common() {
    run("boolean/ball-ball-common");
}

/// A ring and a slab parallel to its axis, in common: two spiric ovals,
/// each across both of the torus's seams.
#[test]
fn boolean_ring_slab_common() {
    run("boolean/ring-slab-common");
}

/// A ring in common with a block at a general pose: closed toric
/// sections cut by the block's edges into blocks, one of which wraps
/// past the end of its periodic knots — on a plane, whose exact pcurve
/// carries those knots.
#[test]
fn boolean_ring_corner_common() {
    run("boolean/ring-corner-common");
}

/// A pin drilled through a ring's tube: a torus and a cylinder, two
/// traced loops in the torus's own parameter plane (ADR-0019).
#[test]
fn boolean_ring_pin_cut() {
    run("boolean/ring-pin-cut");
}

/// A boss filleted at its base and then drilled through the blend: the
/// torus Arris made taken back as a boolean operand (ADR-0020).
#[test]
fn boolean_filleted_boss_drill_cut() {
    run("boolean/filleted-boss-drill-cut");
}

/// A cube's corner filleted and then notched: a fillet's cylinders and
/// its corner sphere cut after they were made (ADR-0020).
#[test]
fn boolean_filleted_corner_notch_cut() {
    run("boolean/filleted-corner-notch-cut");
}

/// A cone sliced by a plane through its apex: two rulings ending at the
/// operand's singular vertex, which paves them (ADR-0021).
#[test]
fn boolean_cone_apex_slice_cut() {
    run("boolean/cone-apex-slice-cut");
}

/// A ball sliced by an oblique plane through a pole: a small circle
/// through the sphere's singular vertex, split there (ADR-0021).
#[test]
fn boolean_ball_pole_slice_cut() {
    run("boolean/ball-pole-slice-cut");
}

/// The same slice turned 2e-4 of a radian off the seam's meridian: the
/// circle crosses the seam again 1.8e-4 from the pole, found at the
/// model's smallest distance, and its block between the two crossings is
/// the seam's piece, not a section edge.
#[test]
fn boolean_pole_slice_beside_seam_cut() {
    run("boolean/pole-slice-beside-seam-cut");
}

/// A stub leaving a frustum through its cone wall, cut and then fused
/// back: the restoring fuse traces the cone and the stub's wall over
/// another region than the cut did, and the cut's section edge, on the
/// cone and the bore, is along the fuse's own section by the surfaces,
/// never by comparing two splines.
#[test]
fn boolean_frustum_stub_cut_then_fuse() {
    run("boolean/frustum-stub-cut-then-fuse");
}

/// A ball cut from a bar whose wall it meets at about 5° where the
/// section leaves the cap: the checker's own fit of that grazing section,
/// traced over the faces' boxes, and the edge's are each held to the one
/// exact branch, so they lie within half a tolerance of each other and
/// S5 finds every sample of its curve on the shared edge.
#[test]
fn boolean_grazing_ball_bar_cut() {
    run("boolean/grazing-ball-bar-cut");
}

/// A drill whose wall runs through both of a ball's poles: two traced
/// loops, each through a singular vertex of the sphere.
#[test]
fn boolean_ball_pole_drill_cut() {
    run("boolean/ball-pole-drill-cut");
}

/// A quarter bend fused with the straight pipe it runs into: the torus
/// and the wall touch along the tube circle and cross in a quartic
/// beside it, one `Meets` of both kinds.
#[test]
fn boolean_pipe_elbow_fuse() {
    run("boolean/pipe-elbow-fuse");
}

/// A ball cut from a bore of its radius: a contact along a closed curve
/// interior to both faces, refused as a tangent contact.
#[test]
fn boolean_ball_in_bore_cut() {
    run("boolean/ball-in-bore-cut");
}

/// A cone in common with a ball through its apex on its axis: a circle
/// and a point in one `Meets`.
#[test]
fn boolean_ball_on_apex_common() {
    run("boolean/ball-on-apex-common");
}

/// A rectangle with a circular hole extruded: `boolean/through-hole`'s
/// solid and numbers by the other path.
#[test]
fn sweep_extrude_plate_with_hole() {
    run("sweep/extrude-plate-with-hole");
}

/// A stadium with a hole: two half-cylinder faces with no seam beside the
/// seamed bore, their boxes apart so the checker decides every pair.
#[test]
fn sweep_extrude_slot() {
    run("sweep/extrude-slot");
}

/// The plate-with-hole profile on `z = 10` extruded down: the same solid
/// again, the profile face on top keeping its frame.
#[test]
fn sweep_extrude_downward() {
    run("sweep/extrude-downward");
}

/// A full ellipse with a turned major axis extruded: one seamed elliptic
/// cylinder between two planes, volume `π a b h` (ADR-0014).
#[test]
fn sweep_extrude_ellipse() {
    run("sweep/extrude-ellipse");
}

/// Two half-ellipses joined by lines: two elliptic-cylinder faces with no
/// seam, their full sections crossing off the faces.
#[test]
fn sweep_extrude_elliptic_slot() {
    run("sweep/extrude-elliptic-slot");
}

/// A rectangle with an elliptic hole: the seamed elliptic bore inside
/// four walls, `boolean/through-hole`'s counts.
#[test]
fn sweep_extrude_plate_elliptic_hole() {
    run("sweep/extrude-plate-elliptic-hole");
}

/// A rectangle revolved a full turn about z: two seamed walls and two
/// annuli of two closed rises each — `boolean/coaxial-cut`'s solid by the
/// other path.
#[test]
fn sweep_revolve_tube() {
    run("sweep/revolve-tube");
}

/// The same rectangle a quarter turn: two flat ends, every rise an arc.
#[test]
fn sweep_revolve_quarter() {
    run("sweep/revolve-quarter");
}

/// An L revolved 270°: the walls' (u, v) regions run past `π` with no
/// seam, two annular sectors of one notch face each other.
#[test]
fn sweep_revolve_l_profile() {
    run("sweep/revolve-l-profile");
}

/// A full turn of a rectangle with a rectangular hole: the hole closes
/// into a ring-shaped cavity, one lump of two shells.
#[test]
fn sweep_revolve_hollow_ring() {
    run("sweep/revolve-hollow-ring");
}

/// The consumer's rectangle with a side on the axis: a solid cylinder in
/// a full turn, and in a quarter and three-quarter turn a sector whose
/// flat ends share the edge on the axis — reflex there past half a turn.
#[test]
fn sweep_revolve_onto_axis() {
    run("sweep/revolve-onto-axis");
}

/// The same profile in the consumer's units: a full turn at a micrometre
/// default tolerance, volume 2π.
#[test]
fn sweep_probe_revolve_onto_axis_m() {
    run("sweep/probe-revolve-onto-axis-m");
}

/// A rectangle on the axis with a notch cut in from it: in a full turn
/// the notch closes into a void of the one lump, two shells from one
/// loop; a quarter turn opens it onto the flat ends.
#[test]
fn sweep_revolve_notch_to_axis() {
    run("sweep/revolve-notch-to-axis");
}

/// A trapezoid revolved about z: a frustum less its bore, its cone
/// narrowing along the axis and, in `widening`, widening; a quarter turn's
/// flat ends meet the cone along a ruling (ADR-0008).
#[test]
fn sweep_revolve_frustum() {
    run("sweep/revolve-frustum");
}

/// An arc about the origin revolved a full turn: a spherical zone less
/// its bore, S5 deciding the sphere against the annuli and the bore by the
/// meridian arm (ADR-0008).
#[test]
fn sweep_revolve_barrel() {
    run("sweep/revolve-barrel");
}

/// A circle revolved about z: `sample::torus` as a revolve builds it, one
/// face and one vertex; a quarter turn's discs cut the torus in its tube
/// circles (ADR-0008).
#[test]
fn sweep_revolve_ring() {
    run("sweep/revolve-ring");
}

/// An ellipse revolved about z: Open CASCADE builds the elliptic torus
/// and Arris refuses it, `Reason::EllipticRevolve` naming the segment,
/// since the surface it would sweep has no variant (ADR-0014).
#[test]
fn sweep_revolve_ellipse() {
    run("sweep/revolve-ellipse");
}

/// The consumer's 2-cube with one vertical edge filleted: a plane–plane
/// blend, a cylinder between two circle ends (ADR-0007), at `r = 0.2`
/// and at `r = 0.5`.
#[test]
fn blend_box_edge_fillet() {
    run("blend/box-edge-fillet");
}

/// The same solid rotated: a cap edge of the cube, the blend's ends on
/// two side faces.
#[test]
fn blend_box_cap_edge_fillet() {
    run("blend/box-cap-edge-fillet");
}

/// The cube rotated about (1, 1, 1) and moved before the fillet: the
/// same blend in an oblique pose, the edge and every probe named by the
/// rotation's closed form.
#[test]
fn blend_box_posed_edge_fillet() {
    run("blend/box-posed-edge-fillet");
}

/// An extruded parallelogram's slanted top edge: the end faces are
/// oblique to the edge, so each end trim is an ellipse arc with a fitted
/// pcurve on the blend (the oblique-section rule).
#[test]
fn blend_box_oblique_end() {
    run("blend/box-oblique-end");
}

/// An extruded L's inner vertical edge: a concave plane–plane blend that
/// adds material, its cylinder's axis in the notch and the blend face
/// reversed against it.
#[test]
fn blend_l_inner_edge() {
    run("blend/l-inner-edge");
}

/// The cube's four vertical edges in one call: four disjoint blends, each
/// cap edge cut at both its ends by two of them.
#[test]
fn blend_box_four_verticals() {
    run("blend/box-four-verticals");
}

/// The consumer's 2-cube with one vertical edge chamfered: a plane–plane
/// chamfer, a plane between two segments on the caps, at `d = 0.2` and
/// at `d = 0.5`.
#[test]
fn blend_box_edge_chamfer() {
    run("blend/box-edge-chamfer");
}

/// The cube's four vertical edges chamfered in one call.
#[test]
fn blend_box_four_vertical_chamfers() {
    run("blend/box-four-vertical-chamfers");
}

/// The miter's chamfer twin: a vertical and a cap edge at one corner, two
/// chamfer planes meeting in a line, every face pair checked at `Full`.
#[test]
fn blend_box_corner_chamfers() {
    run("blend/box-corner-chamfers");
}

/// A second fillet on a filleted body: the extruded square's side edge
/// 0–1, then 1–2 of that result, the two blends sharing the side face
/// the first modified.
#[test]
fn blend_second_fillet() {
    run("blend/second-fillet");
}

/// The consumer's second-blend probe in its own units: the same two
/// disjoint blends on a 0.1 m cube at a micrometre default tolerance.
#[test]
fn blend_probe_second_fillet_m() {
    run("blend/probe-second-fillet-m");
}

/// The consumer's probe of a vertical and a cap edge blended in one call,
/// in the same units: the two blends share the side face x = 0 without
/// meeting on it.
#[test]
fn blend_probe_cap_and_vertical_fillet_m() {
    run("blend/probe-cap-and-vertical-fillet-m");
}

/// A rise ending at a vertex of five edges, where a box stands on its
/// corner on another's top edge: `Reason::VertexBlend`.
#[test]
fn blend_five_edge_vertex() {
    run("blend/five-edge-vertex");
}

/// Three fillets at one box corner: the sphere corner, an octant about
/// the ball's centre tangent to the three cylinders, its pole a degenerate
/// edge (ADR-0007). S5 decides the sphere against each cylinder and each
/// plane by the meridian arm, so nothing is unchecked.
#[test]
fn blend_box_corner_three_fillets() {
    run("blend/box-corner-three-fillets");
}

/// Every edge of the cube filleted in one call: twelve cylinders and eight
/// sphere corners, no face keeping a vertex of the box.
#[test]
fn blend_box_all_edges_fillet() {
    run("blend/box-all-edges-fillet");
}

/// Three chamfers at one box corner, meeting in a triangle.
#[test]
fn blend_box_corner_three_chamfers() {
    run("blend/box-corner-three-chamfers");
}

/// The miter: the vertical and the cap edge at one corner blended in one
/// call, two cylinders meeting in the ellipse of their bisecting plane
/// (ADR-0007). S5 decides the two blend cylinders by the equal-radius
/// crossing arm, so the checker stage has nothing unchecked.
#[test]
fn blend_fillet_miter() {
    run("blend/fillet-miter");
}

/// A half disc's chord edge: a plane against a cylinder along a ruling,
/// convex, the blend cylinder tangent to the arc face along a ruling —
/// S5 decides the pair by the parallel-axis arm's inside tangency.
#[test]
fn blend_d_chord_edge() {
    run("blend/d-chord-edge");
}

/// A half-round rib's root on a plate: the ruling arm concave, the blend
/// adding material, tangent to the rib from outside.
#[test]
fn blend_rib_root_edge() {
    run("blend/rib-root-edge");
}

/// A plate with a D-shaped notch in one side, its top rim filleted: an
/// open half circle between the top plane and the notch's cylinder, the
/// blend half a torus trimmed at each end on its meridian in the plane
/// through the cylinder's axis (ADR-0035).
#[test]
fn blend_d_notch_rim_fillet() {
    run("blend/d-notch-rim-fillet");
}

/// The D-notch's open arc chamfered: half a cone coaxial with the notch,
/// trimmed on the plane through its axis (ADR-0035).
#[test]
fn blend_d_notch_rim_chamfer() {
    run("blend/d-notch-rim-chamfer");
}

/// The stadium's whole outline chamfered in one call: strips and half
/// cones meeting in straight chords at the tangent vertices (ADR-0035).
#[test]
fn blend_stadium_outline_chamfer() {
    run("blend/stadium-outline-chamfer");
}

/// A disc's rim split in two half circles, chamfered: one half cone per
/// edge (ADR-0035).
#[test]
fn blend_split_rim_disc_chamfer() {
    run("blend/split-rim-disc-chamfer");
}

/// A flat running tangentially into a quarter-round of a revolved bend, the
/// edge along the flat filleted: the chain runs on into the arc between the
/// end plane and the torus, a pair with no stripe, and Arris refuses it as
/// `Unsupported` where Open CASCADE builds the chain (ADR-0035 §6).
#[test]
fn blend_chain_through_torus_unsupported() {
    run("blend/chain-through-torus-unsupported");
}

/// The D-notch with its circle's centre off the side face's plane: the open
/// arc ends on a plane neither through the cylinder's axis nor square to
/// it, an end Arris refuses as `Unsupported` where Open CASCADE builds it
/// (ADR-0035 §4, §6).
#[test]
fn blend_oblique_end_unsupported() {
    run("blend/oblique-end-unsupported");
}

/// A bar's wall running tangentially into a quarter cylinder, its top edge
/// filleted: the fillet runs on from the line into the arc at the tangent
/// vertex, a cylinder stripe and a torus section meeting on the ball's
/// great circle (ADR-0035), an open chain trimmed at each end.
#[test]
fn blend_line_into_arc_fillet() {
    run("blend/line-into-arc-fillet");
}

/// The cap edge of a face a first blend trimmed, ending where that
/// blend's contact meets its arc: the fillet runs on through the arc's
/// torus into the next cap edge, an open chain of three Open CASCADE
/// builds as one spine.
#[test]
fn blend_tangent_chain_cap_edge() {
    run("blend/tangent-chain-cap-edge");
}

/// A hole's top rim: a plane against a cylinder along a circle, convex,
/// the blend a quarter of a torus coaxial with the hole with no ends
/// (ADR-0007), S5 deciding it against the top face and the wall by the
/// meridian arm (ADR-0008).
#[test]
fn blend_hole_rim_fillet() {
    run("blend/hole-rim-fillet");
}

/// The same rim chamfered: a 45° cone coaxial with the hole.
#[test]
fn blend_hole_rim_chamfer() {
    run("blend/hole-rim-chamfer");
}

/// A boss's base on a revolved disc: the circle arm concave, the torus
/// adding material, every face on the one axis.
#[test]
fn blend_boss_base_fillet() {
    run("blend/boss-base-fillet");
}

/// A consumer's polyhedron bitten and filleted, in three variants
/// (ADR-0028): `provenance.rs` holds its chain to consumer keys.
#[test]
fn provenance_consumer_rebuild() {
    run("provenance/consumer-rebuild");
}

/// The same recipe under three parameter sets, one test each so a
/// variant that drifts says which: the eight bolt holes' chain is
/// `crates/arris/tests/provenance.rs`'s subject, and these hold each
/// variant's solid to the oracle and to its own `dump.<variant>.txt`.
#[test]
fn provenance_bolt_pattern_rebuild() {
    run_variant("provenance/bolt-pattern-rebuild", "default");
}

#[test]
fn provenance_bolt_pattern_rebuild_thicker_wider() {
    run_variant("provenance/bolt-pattern-rebuild", "thicker-wider");
}

#[test]
fn provenance_bolt_pattern_rebuild_tighter() {
    run_variant("provenance/bolt-pattern-rebuild", "tighter");
}

/// The split-order fixtures (ADR-0009), one test per variant as above:
/// `provenance.rs` holds piece `k` of every split face to the same
/// neighbours in every variant; these hold each variant's solid.
#[test]
fn provenance_split_bar_cut() {
    run_variant("provenance/split-bar-cut", "default");
}

#[test]
fn provenance_split_bar_cut_left() {
    run_variant("provenance/split-bar-cut", "left");
}

#[test]
fn provenance_split_bar_cut_right() {
    run_variant("provenance/split-bar-cut", "right");
}

#[test]
fn provenance_split_bar_cut_narrow() {
    run_variant("provenance/split-bar-cut", "narrow");
}

#[test]
fn provenance_split_frame_cut() {
    run_variant("provenance/split-frame-cut", "default");
}

#[test]
fn provenance_split_frame_cut_left() {
    run_variant("provenance/split-frame-cut", "left");
}

#[test]
fn provenance_split_frame_cut_right() {
    run_variant("provenance/split-frame-cut", "right");
}

#[test]
fn provenance_split_frame_cut_narrow() {
    run_variant("provenance/split-frame-cut", "narrow");
}

#[test]
fn provenance_split_cylinder_seam() {
    run_variant("provenance/split-cylinder-seam", "default");
}

#[test]
fn provenance_split_cylinder_seam_turned_back() {
    run_variant("provenance/split-cylinder-seam", "turned-back");
}

#[test]
fn provenance_split_cylinder_seam_turned_on() {
    run_variant("provenance/split-cylinder-seam", "turned-on");
}

#[test]
fn provenance_split_cylinder_seam_narrow() {
    run_variant("provenance/split-cylinder-seam", "narrow");
}

#[test]
fn provenance_split_cross_common() {
    run_variant("provenance/split-cross-common", "default");
}

#[test]
fn provenance_split_cross_common_larger() {
    run_variant("provenance/split-cross-common", "larger");
}

#[test]
fn provenance_split_cross_common_longer() {
    run_variant("provenance/split-cross-common", "longer");
}

#[test]
fn provenance_split_cross_common_turned() {
    run_variant("provenance/split-cross-common", "turned");
}

/// The edge split-order fixture (ADR-0009, step 2): two notches into one
/// box edge, the second splitting the piece the first left.
#[test]
fn provenance_split_edge_notch() {
    run_variant("provenance/split-edge-notch", "default");
}

#[test]
fn provenance_split_edge_notch_slid() {
    run_variant("provenance/split-edge-notch", "slid");
}

#[test]
fn provenance_split_edge_notch_apart() {
    run_variant("provenance/split-edge-notch", "apart");
}

#[test]
fn provenance_split_edge_notch_narrow() {
    run_variant("provenance/split-edge-notch", "narrow");
}

/// A variant the recipe does not have fails naming it.
#[test]
fn an_unknown_variant_fails_with_its_name() {
    let dir = fixtures::corpus_root().join("primitive/box");
    let err = corpus::run(&dir, "nope").unwrap_err();
    assert!(
        matches!(&err, CorpusError::Variant { variant, .. } if variant == "nope"),
        "{err}"
    );
}

/// A committed dump that differs by one id fails with the diff, and a
/// missing one says how to write it.
#[test]
fn a_dump_that_differs_by_one_id_fails_with_the_diff() {
    let scratch = tempdir("dump-diff");
    let from = fixtures::corpus_root().join("primitive/cylinder");
    for file in ["fixture.json", "expected.json"] {
        std::fs::copy(from.join(file), scratch.join(file)).unwrap();
    }
    let err = corpus::run(&scratch, "default").unwrap_err();
    assert!(
        matches!(&err, CorpusError::Dump { what, .. } if what.contains("not committed")),
        "{err}"
    );
    let dump = std::fs::read_to_string(from.join("dump.txt")).unwrap();
    assert!(dump.contains("coedge +e1 "), "the seam walked up");
    std::fs::write(
        scratch.join("dump.txt"),
        dump.replacen("coedge +e1 ", "coedge +e7 ", 1),
    )
    .unwrap();
    let err = corpus::run(&scratch, "default").unwrap_err();
    match &err {
        CorpusError::Dump { what, .. } => {
            let committed = what
                .lines()
                .any(|l| l.starts_with('-') && l.contains("coedge +e7"));
            let built = what
                .lines()
                .any(|l| l.starts_with('+') && l.contains("coedge +e1"));
            assert!(committed && built, "{what}");
        }
        other => panic!("{other}"),
    }
    std::fs::write(scratch.join("dump.txt"), &dump).unwrap();
    corpus::run(&scratch, "default").unwrap();
}

fn tempdir(tag: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("arris-corpus-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// NIST's CTC-02 AP242 solid alone, from the fetched tier: mitred
/// cylinders of radius 10 that the checker at `Full` finds meeting away
/// from their shared edges.
#[test]
#[ignore = "S5: f366–f368, mitred cylinders of one radius, meet away from their shared edges on a read Open CASCADE holds valid (docs/BACKLOG.md, S5 on the fetched tier's reads)"]
fn regression_nist_ctc_02_ap242_mitred_pipes_s5() {
    run_part("regression/nist-ctc-02-ap242-mitred-pipes-s5");
}

/// NIST's FTC-06 AP242 solid alone, from the fetched tier: cylinders at
/// 45° that the checker at `Full` finds meeting away from their shared
/// edges.
#[test]
#[ignore = "S5: f110, f113 and f114, cylinders mitred at 45°, meet away from their shared edges on a read Open CASCADE holds valid (docs/BACKLOG.md, S5 on the fetched tier's reads)"]
fn regression_nist_ftc_06_ap242_mitred_pipes_s5() {
    run_part("regression/nist-ftc-06-ap242-mitred-pipes-s5");
}

/// NIST's CTC-05 AP203-with-PMI solid alone, from the fetched tier: a
/// torus elbow and the cylinders of its minor radius it joins, which the
/// checker at `Full` finds meeting away from their shared edges.
#[test]
#[ignore = "S5: the torus f116 and the cylinders f114 and f118 of its minor radius meet away from their shared edges on a read Open CASCADE holds valid (docs/BACKLOG.md, S5 on the fetched tier's reads)"]
fn regression_nist_ctc_05_ap203_torus_elbow_s5() {
    run_part("regression/nist-ctc-05-ap203-torus-elbow-s5");
}

/// NIST's STC-10 AP242 solid alone, from the fetched tier: three holes
/// into a bore, their axes crossing, which the checker at `Full` finds
/// meeting the bore away from their shared edges.
#[test]
#[ignore = "S5: the holes f64, f86 and f87 meet the bore f58 away from their shared edges, as f61–f63 meet f60, on a read Open CASCADE holds valid (docs/BACKLOG.md, S5 on the fetched tier's reads)"]
fn regression_nist_stc_10_ap242_holes_into_bore_s5() {
    run_part("regression/nist-stc-10-ap242-holes-into-bore-s5");
}

/// NIST's FTC-10 AP242 solid alone, from the fetched tier: a mesh point of
/// face f95 1.4e-4 from where its (u, v) evaluates.
#[test]
#[ignore = "mesh: a point of f95 stands 1.4e-4 from where its (u, v) evaluates, past the face's and its boundary's 4.6e-6 (docs/BACKLOG.md, a mesh point off its face on the fetched tier)"]
fn regression_nist_ftc_10_ap242_mesh_off_face() {
    run_part("regression/nist-ftc-10-ap242-mesh-off-face");
}

/// NIST's CTC-04 AP203-with-PMI solid alone, from the fetched tier: its
/// battery's drill along the second principal axis is a kernel fault.
#[test]
#[ignore = "drill_y: OpError::Internal(Fault::Geometry) where Open CASCADE builds the drilled part (docs/BACKLOG.md, a drill's geometry fault on the fetched tier)"]
fn regression_nist_ctc_04_ap203_drill_geometry_fault() {
    run_part("regression/nist-ctc-04-ap203-drill-geometry-fault");
}

/// The cap-edge chain with the first radius at twice the second: the
/// arc's torus is a horn torus, its major radius equal to its minor, which
/// the ring refuses. The desired body is Open CASCADE's.
#[test]
#[ignore = "Reason::BlendTooLarge: the arc's fillet is a horn torus, major radius equal to minor, and the ring holds a torus to a ring torus (docs/BACKLOG.md, a horn torus on a blend's arc)"]
fn regression_tangent_chain_horn_torus() {
    run("regression/tangent-chain-horn-torus");
}

/// A rib standing on a plate, its vertical corner edge filleted down to the
/// plate: a convex blend ending on a face that surrounds the rib, the edge's
/// corner with it concave, so the end arc lies in the rib's footprint and the
/// plate's top gains the corner.
#[test]
fn blend_rib_corner_fillet_to_plate() {
    run("blend/rib-corner-fillet-to-plate");
}

/// The same rib's corner chamfered: the chamfer's end a segment in the
/// footprint, the plate's top gaining the corner.
#[test]
fn blend_rib_corner_chamfer_to_plate() {
    run("blend/rib-corner-chamfer-to-plate");
}

/// A rib with a 45° sloped end, the sloped corner filleted: concave at the
/// plate below it, convex at the rib's top above.
#[test]
fn blend_rib_sloped_end_fillet() {
    run("blend/rib-sloped-end-fillet");
}

/// A pocket's vertical corner filleted: a concave blend adding material, at
/// a concave corner with the floor and a convex one with the block's top.
#[test]
fn blend_pocket_corner_fillet() {
    run("blend/pocket-corner-fillet");
}

/// A square boss's vertical corner filleted down to its plate.
#[test]
fn blend_boss_corner_fillet() {
    run("blend/boss-corner-fillet");
}

/// A disc whose outline is two half circles, both filleted in one call:
/// a closed chain of two open arcs at tangent vertices, the shape a rim has
/// in the real parts where the file splits it at its seam.
#[test]
fn blend_split_rim_disc_fillet() {
    run("blend/split-rim-disc-fillet");
}

/// A stadium's whole top outline filleted in one call: line, arc, line, arc,
/// a closed chain through four tangent vertices.
#[test]
fn blend_stadium_outline_fillet() {
    run("blend/stadium-outline-fillet");
}

/// A stadium turned in space, its outline filleted through one named edge:
/// the two half tori of one torus are a pair the checker's S5 leaves
/// undecided at `Full`, where the same body unturned has nothing unchecked.
#[test]
#[ignore = "S5 leaves two faces of one torus undecided in a turned pose: f19 (torus) against f20 (torus) is not decided, so Full has an unchecked pair (docs/BACKLOG.md, coincident tori in S5; found by the C6 outline property)"]
fn regression_turned_stadium_fillet_torus_pair() {
    run("regression/turned-stadium-fillet-torus-pair");
}

/// A cylinder whose circles are placed on the axis `(-1, -6.1e-17, 0)`
/// with no reference direction, as NIST's CTC-04 and FTC-08 write them:
/// ISO 10303-42's reference direction is world `Y` for an axis along `X`
/// to rounding (ADR-0026 §4).
#[test]
fn real_axis_placement_along_x_without_reference() {
    run_part("real/axis-placement-along-x-without-reference");
}

/// A cylinder whose side face is bounded by its two circles and no seam,
/// as NIST's CTC-03, FTC-10 and FTC-11 write it: the reader joins the two
/// loops by a seam along a ruling (ADR-0026's amendment of step 5).
#[test]
fn real_seamless_cylinder_band() {
    run_part("real/seamless-cylinder-band");
}

/// A cone bounded by its base circle alone, its apex implicit, as NIST's
/// FTC-10 writes its drill points: the reader adds the apex and its seam
/// (ADR-0026's amendment of step 5).
#[test]
fn real_cone_face_without_its_apex() {
    run_part("real/cone-face-without-its-apex");
}

/// A half ball whose sphere face is bounded by one meridian circle
/// through both poles, as NIST's FTC-06 writes face #351: the reader
/// splits the edge at the pole it runs through (ADR-0026's amendment of
/// step 5).
#[test]
fn real_edge_through_sphere_pole() {
    run_part("real/edge-through-sphere-pole");
}

/// A half ball shown in two AP242 saved views, draughting models that map
/// its shape — one from a camera, as NIST's CTC-01 AP242 edition writes
/// it, one displaced: a presentation places nothing, and the reader reads
/// one solid (ADR-0026's amendment of step 9).
#[test]
fn real_saved_view_draughting_model() {
    run_part("real/saved-view-draughting-model");
}

/// NIST's CTC-01 whole, whose edge #1864 lies 0.0065 off its cylinders:
/// its pcurve is fitted past the cap, and the cap judges the gap it
/// leaves (ADR-0026's amendment of step 5).
#[test]
fn real_pcurve_fit_reported_as_gap() {
    run_part("real/pcurve-fit-reported-as-gap");
}

/// NIST's CTC-05 whole, held to the time budget its read once broke:
/// fits that climbed to their tolerance one costly miss at a time
/// (ADR-0026's amendment of step 5).
#[test]
fn real_slow_gap_refusal() {
    run_part("real/slow-gap-refusal");
}

/// NIST's FTC-07 whole, whose torus is tangent to a plane on its seam:
/// the checker's S5 decides the pair as one touch (ADR-0026's amendment
/// of step 5).
#[test]
fn real_torus_plane_section_undecided() {
    run_part("real/torus-plane-section-undecided");
}

/// NIST's FTC-09, AP203 geometry only: one solid of 158 faces, read whole.
#[test]
fn real_nist_ftc_09() {
    run_part("real/nist-ftc-09");
}

/// `real/nist-ftc-09` with one plane wrapped in an `OFFSET_SURFACE` by
/// hand: the refusal path, held to the refusal it records.
#[test]
fn real_nist_ftc_09_offset() {
    run_part("real/nist-ftc-09-offset");
}

/// NIST's CTC-01: one solid read whole, an exporter's edge fitted past the cap.
#[test]
fn real_nist_ctc_01() {
    run_part("real/nist-ctc-01");
}

/// NIST's CTC-02: its solid refused for a real gap of 0.0112, its surface body as a surface.
#[test]
fn real_nist_ctc_02() {
    run_part("real/nist-ctc-02");
}

/// NIST's CTC-03: one solid read whole, its seamless bands joined.
#[test]
fn real_nist_ctc_03() {
    run_part("real/nist-ctc-03");
}

/// NIST's CTC-04: one solid read whole, its three surface bodies refused as surfaces.
#[test]
fn real_nist_ctc_04() {
    run_part("real/nist-ctc-04");
}

/// NIST's CTC-05: its solid refused for a real gap of 0.0182, its surface body as a surface.
#[test]
fn real_nist_ctc_05() {
    run_part("real/nist-ctc-05");
}

/// NIST's FTC-06: one solid read whole, its meridian edge split at the pole it passes.
#[test]
fn real_nist_ftc_06() {
    run_part("real/nist-ftc-06");
}

/// NIST's FTC-07: one solid read whole.
#[test]
fn real_nist_ftc_07() {
    run_part("real/nist-ftc-07");
}

/// NIST's FTC-08: one solid read whole.
#[test]
fn real_nist_ftc_08() {
    run_part("real/nist-ftc-08");
}

/// NIST's FTC-10: one solid read whole, its bands joined and its cones given their apex.
#[test]
fn real_nist_ftc_10() {
    run_part("real/nist-ftc-10");
}

/// NIST's FTC-11: one solid read whole, its seamless bands joined.
#[test]
fn real_nist_ftc_11() {
    run_part("real/nist-ftc-11");
}
