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

/// The corpus as a table: one row per test, `name => how "area/dir"`.
/// `run` runs every variant of the recipe, `variant` one of them, `part`
/// a part fixture. Doc comments and `#[ignore = …]` on a row carry to its
/// test, whose name stays the row's (`tools/gate.sh` and the nextest
/// profiles select by name prefix). `TABLE` lists every row's directory
/// for `every_fixture_directory_has_a_corpus_test`.
macro_rules! corpus_tests {
    ($($(#[$meta:meta])* $name:ident => $how:ident $dir:literal $(, $variant:literal)?;)*) => {
        $(
            #[test]
            $(#[$meta])*
            fn $name() {
                corpus_tests!(@call $how $dir $(, $variant)?);
            }
        )*

        /// The directory of every row, in table order.
        const TABLE: &[&str] = &[$($dir),*];
    };
    (@call run $dir:literal) => { run($dir) };
    (@call part $dir:literal) => { run_part($dir) };
    (@call variant $dir:literal, $variant:literal) => { run_variant($dir, $variant) };
}

corpus_tests! {
    primitive_box => run "primitive/box";
    primitive_cylinder => run "primitive/cylinder";
    /// A consumer's own topology through `ops::build` (ADR-0028): a
    /// tetrahedron, straight and skewed.
    build_tetrahedron => run "build/tetrahedron";
    /// A concave edge.
    build_l_prism => run "build/l-prism";
    /// Faces with an inner loop, genus 1.
    build_frame => run "build/frame";
    transform_posed_cylinder => run "transform/posed-cylinder";
    /// The hollow ring moved: two shells carried whole, the cavity still the
    /// one lump's void.
    transform_moved_hollow_ring => run "transform/moved-hollow-ring";
    /// The mirror of a body in a plane beside it, through it and across it
    /// (ADR-0031), each against Open CASCADE's mirror.
    transform_mirror_box => run "transform/mirror-box";
    transform_mirror_posed_cylinder => run "transform/mirror-posed-cylinder";
    transform_mirror_hollow_ring => run "transform/mirror-hollow-ring";
    transform_mirror_ball_corner_cut => run "transform/mirror-ball-corner-cut";
    transform_mirror_frustum => run "transform/mirror-frustum";
    transform_mirror_elliptic_cylinder => run "transform/mirror-elliptic-cylinder";
    transform_mirror_filleted_box => run "transform/mirror-filleted-box";
    /// A box with a cylinder boss, fused, mirrored: the mirror of a boolean's
    /// result (ADR-0031).
    transform_mirror_fused_boss => run "transform/mirror-fused-boss";
    boolean_through_hole => run "boolean/through-hole";
    boolean_blind_hole => run "boolean/blind-hole";
    /// A solid read from a STEP file as an operand — Arris's own STEP of
    /// `boolean/through-hole`'s result — cut by a box through its hole: the
    /// `step` op read by both kernels' readers (ADR-0026).
    boolean_step_operand_cut => run "boolean/step-operand-cut";
    /// The consumer's through-hole probe in its own units: a 0.1 m plate less
    /// a r 0.02 cylinder, in a model whose default tolerance is a micrometre
    /// — `w²t − πr²t = 8.7434e-5`.
    boolean_probe_through_hole_m => run "boolean/probe-through-hole-m";
    /// The consumer's blind-hole probe in the same units: the floor kept,
    /// `w²t − πr²d = 1.8743e-4`.
    boolean_probe_blind_hole_m => run "boolean/probe-blind-hole-m";
    /// The consumer's flush-union probe in the same units: two 1 m cubes
    /// sharing a face, volume 2.0.
    boolean_probe_flush_union_m => run "boolean/probe-flush-union-m";
    boolean_bolt_pattern_8 => run "boolean/bolt-pattern-8";
    /// Two boxes sharing a face: the flush case. The shared face vanishes
    /// and the four edges around it are held once, from the first operand.
    boolean_flush_union => run "boolean/flush-union";
    boolean_corner_union => run "boolean/corner-union";
    boolean_corner_common => run "boolean/corner-common";
    /// A body united with, minus and intersected with its own mirror image
    /// (ADR-0031): a mirrored operand is an operand like any other.
    boolean_mirror_ball_corner_cut_fuse => run "boolean/mirror-ball-corner-cut-fuse";
    boolean_mirror_box_fuse => run "boolean/mirror-box-fuse";
    boolean_mirror_box_cut => run "boolean/mirror-box-cut";
    boolean_mirror_box_common => run "boolean/mirror-box-common";
    boolean_mirror_cylinders_fuse => run "boolean/mirror-cylinders-fuse";
    /// The oblique mirror of a cylinder, united: Open CASCADE cuts one more section arc
    /// at an ellipse's parameter origin (`analytic.counts_differ`).
    boolean_mirror_cylinders_oblique_fuse => run "boolean/mirror-cylinders-oblique-fuse";
    /// The oblique mirror of a cylinder, intersected: Open CASCADE cuts one more section arc
    /// at an ellipse's parameter origin (`analytic.counts_differ`).
    boolean_mirror_cylinders_oblique_common => run "boolean/mirror-cylinders-oblique-common";
    /// Touching images have no material in common: a typed refusal, as the
    /// oracle's empty result.
    boolean_mirror_box_touching_common => run "boolean/mirror-box-touching-common";
    boolean_mirror_cylinders_touching_common => run "boolean/mirror-cylinders-touching-common";
    boolean_mirror_cylinders_cut => run "boolean/mirror-cylinders-cut";
    boolean_mirror_cylinders_common => run "boolean/mirror-cylinders-common";
    boolean_corner_cut => run "boolean/corner-cut";
    /// The two flush boxes' common is the shared face alone: nothing with
    /// thickness, the runner's degenerate path.
    boolean_flush_common => run "boolean/flush-common";
    boolean_disjoint_cut => run "boolean/disjoint-cut";
    boolean_frame_cut => run "boolean/frame-cut";
    /// The target inside the tool: nothing survives, and the oracle records
    /// no solid — the runner's degenerate path.
    boolean_swallow_cut => run "boolean/swallow-cut";
    /// A slab through the plate: two boxes, two lumps of one solid — Open
    /// CASCADE's two solids (ADR-0006).
    boolean_split_cut => run "boolean/split-cut";
    /// The consumer's hollow box in its own units: one lump, its cavity a
    /// void shell made of the tool's faces turned inward.
    boolean_enclosed_cavity => run "boolean/enclosed-cavity";
    /// Operands apart: two lumps, every face of both kept by id.
    boolean_disjoint_fuse => run "boolean/disjoint-fuse";
    /// A cylinder wholly inside a box: a void of three faces, the seam
    /// carried with them.
    boolean_cavity_cylinder => run "boolean/cavity-cylinder";
    /// A box inside the cavity of a hollow box: three shells, two lumps, one
    /// of them in the other's void.
    boolean_lump_in_cavity => run "boolean/lump-in-cavity";
    /// Two boxes touching along an edge: Open CASCADE builds two solids that
    /// share it, Arris refuses with `InputReason::NonManifold` — the runner's
    /// expected-error path.
    boolean_edge_touching_fuse => run "boolean/edge-touching-fuse";
    boolean_boss => run "boolean/boss";
    /// The boss's bottom cap coincident with the plate's top: the cap
    /// vanishes, the plate's top is split by the cap's rim and keeps the
    /// outside, and the rim is the wall's own edge.
    boolean_boss_flush => run "boolean/boss-flush";
    /// A rod filling a tube's bore: the coincident cylinder walls vanish,
    /// the rod's discs sit beside the tube's annuli sharing the inner
    /// circles — the cylinder–cylinder coincident arm, and the common
    /// blocks of a periodic edge.
    boolean_coaxial_fuse => run "boolean/coaxial-fuse";
    /// A hole drilled at 30°: two ellipse sections, NURBS pcurves on the
    /// wall, and a wall whose (u, v) region is a strip oblique to the
    /// ruling — the case the tessellator flattens the ruled direction for
    /// (ADR-0005).
    boolean_oblique_hole => run "boolean/oblique-hole";
    /// A cylinder touching the plate's side face from outside: the tangent
    /// pair contributes no section edge and no split, and the result is
    /// the plate with every id kept. Open CASCADE imprints the ruling; the
    /// fixture states that convention in `analytic.counts_differ`.
    boolean_tangent_outside_cut => run "boolean/tangent-outside-cut";
    /// A blind hole whose wall touches a side face from inside along a
    /// ruling interior to both: the slit no manifold `Solid` can carry,
    /// `BooleanReason::TangentContact` through the runner's expected-error path
    /// (plan m4-booleans `⚠ OPEN` 1).
    boolean_tangent_hole => run "boolean/tangent-hole";
    /// The same solid as `through-hole` in another pose: both operands moved
    /// by one rigid motion before the cut.
    boolean_posed_through_hole => run "boolean/posed-through-hole";
    /// Two coaxial cylinders: the pair has no section curve, and the bore is
    /// the tool's wall reversed.
    boolean_coaxial_cut => run "boolean/coaxial-cut";
    /// Operands that share no material: the runner's degenerate path through
    /// `common`.
    boolean_disjoint_common => run "boolean/disjoint-common";
    /// A sliver: the top and bottom faces are a D of one straight edge and
    /// one arc under an eighth of a turn, which the checker's minimum
    /// discretisation once flattened to a chord (plan step 9).
    boolean_sliver_common => run "boolean/sliver-common";
    /// Two equal cylinders crossing at 90°: the Steinmetz solid. The two
    /// section ellipses cross each other at (0, ±R, 0), a section vertex no
    /// operand edge made, and each wall's lens through the seam is two
    /// faces. Open CASCADE also cuts an arc at the ellipse's parameter
    /// origin; the fixture states that convention in `analytic.counts_differ`.
    boolean_cross_cylinders_common => run "boolean/cross-cylinders-common";
    /// The same cylinders fused: each wall keeps its two pieces outside the
    /// other, meeting at the crossing vertices.
    boolean_cross_cylinders_fuse => run "boolean/cross-cylinders-fuse";
    /// The target minus the tool that is as wide as it: two lumps whose
    /// closures touch at the crossing vertices, `InputReason::NonManifold`
    /// through the runner's expected-error path (ADR-0006).
    boolean_cross_cylinders_cut => run "boolean/cross-cylinders-cut";
    /// The common at ψ = 60°: ellipses of two different major radii still
    /// crossing at (0, ±R, 0), `16R³/(3 sin ψ)`.
    boolean_oblique_cross_common => run "boolean/oblique-cross-common";
    /// The Steinmetz solid with the tool's seam through a crossing vertex:
    /// the seam touches the target's wall there, and the touch landing on
    /// the section vertex cuts the seam. Open CASCADE's extra arc vertex is
    /// stated in `analytic.counts_differ`.
    boolean_seam_through_crossing_common => run "boolean/seam-through-crossing-common";
    /// The crossing-cylinder fuse with the second cylinder's seam turned to
    /// just beside a crossing vertex rather than through it: the same solid
    /// as at any turn, so the result is held to cross-cylinders-fuse's
    /// closed forms — which Open CASCADE misses in this band, stated in
    /// `analytic.measure_differs` (ADR-0015). The seam's touch is resolved
    /// into its crossings (ADR-0016) and the sliver between the seam and the
    /// two ellipses is decided by the transversal rule.
    boolean_seam_beside_crossing_fuse => run "boolean/seam-beside-crossing-fuse";
    /// The same fuse with the seam between one and two tolerances from the
    /// crossing vertex: its two crossings and that vertex are three points
    /// one vertex by closure, whose balls meet though no two lie within one
    /// tolerance, and its point is the seam's. The section edges' pcurves
    /// end on that vertex's own (u, v) on each face — the seam's pave on the
    /// turned wall — and the body is the one at −90°.
    boolean_seam_a_tolerance_from_crossing_fuse => run "boolean/seam-a-tolerance-from-crossing-fuse";
    /// flush-union's boxes a quarter of a tolerance into and away from each other, and four apart, fused: the flush union within the tolerance, two lumps beyond (ADR-0022).
    boolean_flush_union_band_ends => run "boolean/flush-union-band-ends";
    /// boss-flush's boss a quarter of a tolerance into and off its plate, and four off, fused (ADR-0022).
    boolean_boss_flush_band_ends => run "boolean/boss-flush-band-ends";
    /// pin-in-bore-fuse's pin a quarter of a tolerance out through and in off the bore's wall, and four in, fused (ADR-0022).
    boolean_pin_in_bore_fuse_band_ends => run "boolean/pin-in-bore-fuse-band-ends";
    /// coaxial-fuse's rod a quarter of a tolerance off the tube's axis either way, fused: the flush union (ADR-0022).
    boolean_coaxial_fuse_band_ends => run "boolean/coaxial-fuse-band-ends";
    /// coaxial-cut's tool a quarter of a tolerance and four off the shared axis (ADR-0022).
    boolean_coaxial_cut_band_ends => run "boolean/coaxial-cut-band-ends";
    /// tangent-cylinders-cut's tool four tolerances into and away from the target (ADR-0022).
    boolean_tangent_cylinders_cut_band_ends => run "boolean/tangent-cylinders-cut-band-ends";
    /// tangent-hole's hole four tolerances out through and in off the side face: a clean cut either way (ADR-0022).
    boolean_tangent_hole_band_ends => run "boolean/tangent-hole-band-ends";
    /// tangent-hole's hole a quarter of a tolerance out through and in off the side face: the contact's own refusal, BooleanReason::TangentContact (ADR-0022).
    boolean_tangent_hole_a_quarter_off => run "boolean/tangent-hole-a-quarter-off";
    /// tangent-outside-cut's tool four tolerances into and away from the plate, and turned half a tolerance about the contact's middle (ADR-0022).
    boolean_tangent_outside_cut_band_ends => run "boolean/tangent-outside-cut-band-ends";
    /// pipe-elbow-fuse's pipe a quarter of a tolerance into and away from the bend, a whole one in, and grown by half of one, fused (ADR-0022).
    boolean_pipe_elbow_fuse_band_ends => run "boolean/pipe-elbow-fuse-band-ends";
    /// edge-touching-fuse's boxes a quarter of a tolerance into each other and apart across their shared edge, and four apart, the first cut by the second (ADR-0022).
    boolean_edge_touching_band_ends => run "boolean/edge-touching-band-ends";
    /// flush-union's boxes four tolerances into each other, fused: one box with the 4e-7 strip where their side faces overlap kept as a face (ADR-0022).
    boolean_flush_union_four_into => run "boolean/flush-union-four-into";
    /// boss-flush's boss four tolerances into its plate, fused, held to its closed forms where Open CASCADE's own fuse is the far side (ADR-0022).
    boolean_boss_flush_four_into => run "boolean/boss-flush-four-into";
    /// pin-in-bore-fuse's pin four tolerances out through the bore's wall, fused: a lens 4e-7 proud of the wall (ADR-0022).
    boolean_pin_in_bore_four_out => run "boolean/pin-in-bore-four-out";
    /// coaxial-fuse's rod four tolerances off the tube's axis along +x, fused: a crescent hole 4e-7 wide, genus 1 (ADR-0022).
    boolean_coaxial_fuse_four_off => run "boolean/coaxial-fuse-four-off";
    /// coaxial-fuse's rod four tolerances off the tube's axis along −x, fused: the same crescent hole the other way (ADR-0022).
    boolean_coaxial_fuse_four_off_back => run "boolean/coaxial-fuse-four-off-back";
    /// tangent-cylinders-cut's tool a quarter of a tolerance into and away from the target: the target whole, as at the contact (ADR-0022).
    boolean_tangent_cylinders_a_quarter_off => run "boolean/tangent-cylinders-a-quarter-off";
    /// tangent-outside-cut's tool a quarter of a tolerance into and away from the plate: the plate whole, as at the contact (ADR-0022).
    boolean_tangent_outside_a_quarter_off => run "boolean/tangent-outside-a-quarter-off";
    /// edge-touching-fuse's boxes four tolerances into each other across their shared edge, the first cut by the second: a notch 4e-7 deep (ADR-0022).
    boolean_edge_touching_four_into => run "boolean/edge-touching-four-into";
    /// edge-touching-fuse's boxes, the second turned a tolerance about the shared edge's middle, cut: the tool's edge crosses the box's there, the section block along both edges at once is each face's boundary (ADR-0022).
    boolean_edge_touching_tilted_cut => run "boolean/edge-touching-tilted-cut";
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
    #[ignore = "kernel bug: the face arrangement is not a subdivision, a section edge of f0 ends at a node nothing else reaches: coincident spheres under frames a half turn apart are not one surface to the split (docs/BACKLOG.md, coincident quadrics under different frames)"]
    regression_coincident_spheres_rotated_frame => run "regression/coincident-spheres-rotated-frame";
    /// ring.
    #[ignore = "L4 f0 no outer loop: the reader builds no outer loop for the torus face of Open CASCADE's STEP of a mirrored ring, whose torus frame is left-handed (docs/BACKLOG.md, a reader of left-handed surface frames)"]
    regression_mirror_torus_ring => run "regression/mirror-torus-ring";
    #[ignore = "L4, a loop of zero signed area: the sliver face between the seam's two crossings and the crossing vertex, three vertices a few tolerances apart, is kept and its (u, v) polygon does not resolve it (docs/BACKLOG.md, material a tolerance or two thick)"]
    regression_seam_two_tolerances_from_crossing_common => run "regression/seam-two-tolerances-from-crossing-common";
    /// A drill touching the main wall from inside, at a singular point of the
    /// traced section: the pave model makes the point one section vertex
    /// that ends both branches, and the main wall is left two pieces meeting
    /// only there — one shell touching itself at a point.
    #[ignore = "OpError::Degenerate, Reason::NonManifold naming the singular vertex: the main wall's two pieces meet only there, pinched between the drill's exits, a shell touching itself at a point that a manifold Solid does not hold; the desired body is a General one (docs/BACKLOG.md, a shell touching itself at a vertex)"]
    regression_singular_bore_cut => run "regression/singular-bore-cut";
    /// Case 216 of the differential's turned draw: a cylinder under a
    /// toroidal dome, revolved a full turn, its three circular edges filleted
    /// in one call. Arris builds one exact face per fillet (6/11/7, measures
    /// matching); Open CASCADE walks the dome's rim with fitted B-spline strips
    /// and a corner patch (12/23/13), held as `counts_differ`. What fails is the
    /// read-back of Open CASCADE's own STEP of it.
    #[ignore = "read-back: #773 on face #769: the pcurve fit still deviates by 2.4 with 3673 spans, the most it may use, on a strip of Open CASCADE's STEP of the blend (the seam edge of #139 reads since the 2026-10-04 reader fix; the differential's counts disagreement on this recipe is Open CASCADE's convention, not Arris's)"]
    regression_turned_dome_three_rim_fillet => run "regression/turned-dome-three-rim-fillet";
    /// A ball sliced by a face 3.7e-7 from its pole: not through the singular
    /// point and nearer than the sphere's (u, v) polygons resolve, refused by
    /// name rather than built.
    #[ignore = "Reason::BesideSingularity, a section passes the sphere's pole without running through it, nearer than the face's (u, v) resolves (docs/BACKLOG.md, polygons by span and chords in length)"]
    regression_ball_beside_pole_slice_cut => run "regression/ball-beside-pole-slice-cut";
    /// flush-union's second box a tolerance off the shared face: moved away, into the first, or turned about the face's middle line.
    #[ignore = "Fault::Split EmptySubEdge, Fault::CommonBlock, Fault::Split Turn, and plane against plane Unsupported a hair off parallel (docs/BACKLOG.md, ADR-0022)"]
    regression_flush_union_a_tolerance_off => run "regression/flush-union-a-tolerance-off";
    /// boss-flush's boss turned about a diameter of its cap by a quarter of a tolerance, or two.
    #[ignore = "plane against plane Unsupported a hair off parallel; beyond it Fault::Split, a section edge ends at a node nothing else reaches (docs/BACKLOG.md, ADR-0022)"]
    regression_boss_flush_tilted => run "regression/boss-flush-tilted";
    /// A boss lifted a tolerance off its plate, posed: the union fails the checker.
    #[ignore = "plane against plane Unsupported a hair off parallel once posed; the survey built the same pair into a union that fails the checker (docs/BACKLOG.md, ADR-0022)"]
    regression_boss_flush_posed_gap_fuse => run "regression/boss-flush-posed-gap-fuse";
    /// pin-in-bore-fuse's pin a tolerance or two out through the bore's wall, turned, or grown.
    #[ignore = "Fault::Split Dangling and NoInterior, and a union that fails the checker (docs/BACKLOG.md, ADR-0022)"]
    regression_pin_in_bore_a_tolerance_off => run "regression/pin-in-bore-a-tolerance-off";
    /// coaxial-fuse's rod a tolerance or two off the tube's axis, turned about its middle, or grown.
    #[ignore = "Fault::CommonBlock, Fault::Split NoInterior and Dangling, Fault::Builder EdgeUses, circle against plane Unsupported (docs/BACKLOG.md, ADR-0022)"]
    regression_coaxial_fuse_a_tolerance_off => run "regression/coaxial-fuse-a-tolerance-off";
    /// A rod filling a tube's bore, turned a quarter of a tolerance: a section edge crosses a seam.
    #[ignore = "Fault::Seam, a section edge crosses a seam without a pave there (docs/BACKLOG.md, ADR-0022)"]
    regression_coaxial_fuse_tilted_seam => run "regression/coaxial-fuse-tilted-seam";
    /// A rod filling a tube's bore, turned a quarter of a tolerance: a cylinder is placed with no frame.
    #[ignore = "Fault::Geometry, a degenerate cylinder surface with a non-finite frame (docs/BACKLOG.md, ADR-0022)"]
    regression_coaxial_fuse_tilted_frame => run "regression/coaxial-fuse-tilted-frame";
    /// tangent-cylinders-cut's tool a tolerance into the target, or turned two about the contact's middle.
    #[ignore = "Fault::Split, a section edge ends at a node nothing else reaches (docs/BACKLOG.md, ADR-0022)"]
    regression_tangent_cylinders_a_tolerance_in => run "regression/tangent-cylinders-a-tolerance-in";
    /// tangent-cylinders-cut's operands two tolerances into each other, intersected: a sliver the oracle builds no solid of.
    #[ignore = "the common fails the checker, a panic in a debug build: L4, sliver loops of zero signed area, where the desired is a refusal (docs/BACKLOG.md, ADR-0022)"]
    regression_tangent_cylinders_overlap_common => run "regression/tangent-cylinders-overlap-common";
    /// tangent-cylinders-fuse's tool turned a tolerance and a half about the contact's middle.
    #[ignore = "Fault::Lumps, two shells of the result meet, where the desired is TangentContact (docs/BACKLOG.md, ADR-0022)"]
    regression_tangent_cylinders_tilted_fuse => run "regression/tangent-cylinders-tilted-fuse";
    /// tangent-hole's operands fused, at the contact and a tolerance out: the section circle is tangent to the top face's edge.
    #[ignore = "Fault::Builder EdgeUses, where the desired is TangentContact (docs/BACKLOG.md, ADR-0022)"]
    regression_tangent_hole_fuse => run "regression/tangent-hole-fuse";
    /// tangent-hole's hole a tolerance and a half out through the side face, or turned about the contact's middle.
    #[ignore = "Fault::Split NoInterior and Dangling, plane against cylinder Unsupported, where the desired is TangentContact (docs/BACKLOG.md, ADR-0022)"]
    regression_tangent_hole_a_tolerance_out => run "regression/tangent-hole-a-tolerance-out";
    /// A blind hole touching a side face, turned a tolerance, posed: a body the checker refuses.
    #[ignore = "a body where TangentContact is desired, which S5 refuses: two faces intersect away from their shared edges (docs/BACKLOG.md, ADR-0022)"]
    regression_tangent_hole_tilted_posed_cut => run "regression/tangent-hole-tilted-posed-cut";
    /// tangent-outside-cut's tool a tolerance into the plate, or turned a quarter or a half about the contact's middle.
    #[ignore = "Fault::Split NoInterior and Dangling, and a section fit that still deviates at its most spans (docs/BACKLOG.md, ADR-0022)"]
    regression_tangent_outside_a_tolerance_in => run "regression/tangent-outside-a-tolerance-in";
    /// A cylinder touching a box face from outside, grown a tolerance, intersected: a hole of the arrangement in no region.
    #[ignore = "Fault::Split Hole, where the desired is a refusal (docs/BACKLOG.md, ADR-0022)"]
    regression_tangent_outside_grown_common => run "regression/tangent-outside-grown-common";
    /// A cylinder touching a box face from outside, turned four tolerances, posed, intersected: a sliver the checker refuses.
    #[ignore = "a body where a refusal is desired, which B1 refuses: no outer shell (docs/BACKLOG.md, ADR-0022)"]
    regression_tangent_outside_tilted_posed_common => run "regression/tangent-outside-tilted-posed-common";
    /// A cylinder touching a box face from outside, turned a quarter of a tolerance, posed, fused.
    #[ignore = "Fault::Geometry, the section fit still deviates by 1.7e-7 (at step 1 the ellipse section off its surfaces by 1.2e-7), where the desired is TangentContact (docs/BACKLOG.md, ADR-0022)"]
    regression_tangent_outside_tilted_posed_fuse => run "regression/tangent-outside-tilted-posed-fuse";
    /// pipe-elbow-fuse's pipe a tolerance or so into the bend, turned about its cap's middle, or grown.
    #[ignore = "Fault::CommonBlock, Fault::Split Turn, torus sections called degenerate (Fault::Geometry), plane against plane and circle against plane Unsupported (docs/BACKLOG.md, ADR-0022)"]
    regression_pipe_elbow_a_tolerance_off => run "regression/pipe-elbow-a-tolerance-off";
    /// A pipe pushed a tolerance into its bend, posed: a fit's normal equations are singular.
    #[ignore = "Fault::Split, a cycle does not turn once; the survey built the same pair into a fit whose normal equations are singular (docs/BACKLOG.md, ADR-0022)"]
    regression_pipe_elbow_posed_fuse => run "regression/pipe-elbow-posed-fuse";
    /// Two boxes touching along an edge, the second turned half a tolerance, posed, cut.
    #[ignore = "one vertex too many (10/14): the edges cross at a grazing angle and the crossing's hits, scattered along the edge by the rounding over it, are two vertices (docs/BACKLOG.md, a hair off parallel; ADR-0022)"]
    regression_edge_touching_tilted_posed_cut => run "regression/edge-touching-tilted-posed-cut";
    /// A tilted cylinder less a slab whose rectangle crosses its section on
    /// all four sides (prop::recipe's draw, shrunk).
    boolean_tilted_cylinder_slot_cut => run "boolean/tilted-cylinder-slot-cut";
    /// A disc in common with a pin crossing its wall, fused with a parallel
    /// thinner disc (prop::recipe's draw, shrunk).
    boolean_pin_at_disc_rim_common_fuse => run "boolean/pin-at-disc-rim-common-fuse";
    /// A cylinder fused with a revolved profile, less a posed extrusion (the differential's draw, shrunk; ADR-0024 step
    /// 5b).
    #[ignore = "the cut's result fails the checker's L5 (a loop intersects itself) (docs/BACKLOG.md, the differential's findings)"]
    regression_revolve_fuse_extrude_cut_loop_crosses_itself => run "regression/revolve-fuse-extrude-cut-loop-crosses-itself";
    /// A revolved profile fused with a posed cylinder and then an extrusion (the differential's draw, shrunk; ADR-0024 step
    /// 5b).
    #[ignore = "one shell and three faces fewer than Open CASCADE's: a cavity is missing (docs/BACKLOG.md, the differential's findings)"]
    regression_revolve_cylinder_extrude_fuse_misses_a_shell => run "regression/revolve-cylinder-extrude-fuse-misses-a-shell";
    /// A revolved sketch fused with an extrusion and cut by a second one,
    /// drawn by the differential on a nightly seed (`e4bc2ddf…`, case 757)
    /// and shrunk: its section curves carry knots a rounding apart, which a
    /// knot rule refusing them turned into the checker's S5 on a cone face
    /// against a cylinder face.
    boolean_revolve_extrude_fuse_cut_rounding_knots => run "boolean/revolve-extrude-fuse-cut-rounding-knots";
    /// A full revolve less an elliptic extrusion (the differential's draw, shrunk; ADR-0024 step
    /// 5b).
    #[ignore = "more faces, edges and vertices than Open CASCADE's (docs/BACKLOG.md, the differential's findings)"]
    regression_revolve_cut_by_extrusion_extra_faces => run "regression/revolve-cut-by-extrusion-extra-faces";
    /// A revolved profile fused with a posed box (the differential's draw, shrunk; ADR-0024 step
    /// 5b).
    #[ignore = "the tessellation at chord 1e-3 is not closed (docs/BACKLOG.md, the differential's findings)"]
    regression_revolve_box_fuse_mesh_not_closed => run "regression/revolve-box-fuse-mesh-not-closed";
    /// Three posed cylinders fused in turn (the differential's draw, shrunk; ADR-0024 step
    /// 5b).
    #[ignore = "the second fuse returns OpError::Internal(Split) (docs/BACKLOG.md, the differential's findings)"]
    regression_three_cylinders_fuse_split_fault => run "regression/three-cylinders-fuse-split-fault";
    /// A mirrored revolve and a mirrored extrusion intersected with a posed cylinder in turn (the differential's draw, shrunk; CI run 36786571173).
    #[ignore = "the second common returns OpError::Internal(Seam) (docs/BACKLOG.md, the differential's findings)"]
    regression_mirrored_revolve_common_seam_fault => run "regression/mirrored-revolve-common-seam-fault";
    /// A box, a revolved profile and a chamfered cylinder fused in turn (the differential's draw, shrunk; ADR-0024 step
    /// 5b).
    #[ignore = "a fuse returns OpError::Internal(Geometry) (docs/BACKLOG.md, the differential's findings)"]
    regression_box_revolve_cylinder_chamfer_fuse_geometry_fault => run "regression/box-revolve-cylinder-chamfer-fuse-geometry-fault";
    /// A box, a revolved profile and a cylinder fused in turn (the differential's draw, shrunk; ADR-0024 step
    /// 5b).
    #[ignore = "a fuse returns OpError::Internal(Lumps) (docs/BACKLOG.md, the differential's findings)"]
    regression_box_revolve_cylinder_fuse_lumps_fault => run "regression/box-revolve-cylinder-fuse-lumps-fault";
    /// A box in common with a revolved profile, fused with a cylinder (the differential's draw, shrunk; ADR-0024 step
    /// 5b).
    #[ignore = "a boolean returns OpError::Internal(Builder) (docs/BACKLOG.md, the differential's findings)"]
    regression_box_revolve_cylinder_common_fuse_builder_fault => run "regression/box-revolve-cylinder-common-fuse-builder-fault";
    /// A cylinder along x lying on a plate, its seam on the touch, cut from
    /// the plate (boolean_prop's tangent pair at 5000 cases, shrunk;
    /// ADR-0024).
    #[ignore = "Fault::Split, a section edge ending at a node nothing else reaches, where the desired cut is the plate (docs/BACKLOG.md, the seam on a touch)"]
    regression_tangent_seam_on_face_cut => run "regression/tangent-seam-on-face-cut";
    /// A square less a quarter disc, extruded and cut by a box inside it:
    /// the arc is tangent to both lines it meets, so each cap's loop has two
    /// cusps, each a turn of +π round its spike (ADR-0026 §4, the battery's
    /// CTC-04 and FTC-06).
    boolean_spandrel_cavity_cut => run "boolean/spandrel-cavity-cut";
    /// NIST's CTC-04 cut by a speck of a box: its plane f393, bounded by
    /// B-spline edges that leave a line tangent to it, has two cusps, and
    /// splits into one piece that turns once (ADR-0026 §4).
    boolean_nist_ctc_04_face_arrangement_turn => run "boolean/nist-ctc-04-face-arrangement-turn";
    /// NIST's FTC-06 cut by a speck of a box: its plane f0, whose loop runs
    /// between small blends tangent to its lines, has ten cusps, and splits
    /// into one piece that turns once; its counts are its own reading's,
    /// Open CASCADE's healed (ADR-0026 §3, §4).
    boolean_nist_ftc_06_face_arrangement_turn => run "boolean/nist-ftc-06-face-arrangement-turn";
    /// An all-NURBS box cut by a box inside it: the tool's faces are
    /// classified by a ray against NURBS faces, which has no closed form, and
    /// the boolean refuses the pair by name, the NURBS cycle's count
    /// (ADR-0026 §5) — the battery's every cut and drill of FTC-07 and
    /// FTC-10.
    boolean_nurbs_box_cavity_cut => run "boolean/nurbs-box-cavity-cut";
    /// A tee of equal radii: the branch's rim circle touches the main wall
    /// exactly at the two crossing vertices, and each touch cuts the rim
    /// there.
    boolean_tee_fuse => run "boolean/tee-fuse";
    /// A tee of unequal radii: the branch's wall meets the main wall in one
    /// traced loop, one periodic fitted curve paved once where the branch's
    /// seam pierces the main wall (ADR-0018).
    boolean_tee_unequal_fuse => run "boolean/tee-unequal-fuse";
    /// The same operands' cut: a pocket opening through the main wall along
    /// the loop.
    boolean_tee_unequal_cut => run "boolean/tee-unequal-cut";
    /// The same operands' common: the branch inside the main.
    boolean_tee_unequal_common => run "boolean/tee-unequal-common";
    /// A drill on a skew axis breaking out of the main cylinder's side: one
    /// traced loop, a notch.
    boolean_skew_hole_cut => run "boolean/skew-hole-cut";
    /// A drill on a skew axis inside the main cylinder: two traced loops, a
    /// bore, genus 1.
    boolean_skew_bore_cut => run "boolean/skew-bore-cut";
    /// A slot across the fused tee's junction: the box's planes cross the
    /// tee's fitted edge (ADR-0018) in four section vertices, found by the
    /// NURBS curve against a plane, and every edge stays at its faces'
    /// tolerance.
    boolean_tee_unequal_slot_cut => run "boolean/tee-unequal-slot-cut";
    /// A drill through the fused tee's junction: its wall meets both walls in
    /// traced loops that cross the tee's fitted edge, found by the NURBS
    /// curve against a cylinder.
    boolean_tee_unequal_drill_cut => run "boolean/tee-unequal-drill-cut";
    /// The consumer's cylinder − cylinder transversal probe in its own units:
    /// two parallel walls meeting in two rulings, `(πr² − lens)·h =
    /// 2.2079e-5`.
    boolean_parallel_cylinders_cut => run "boolean/parallel-cylinders-cut";
    /// The same operands' common: the lens prism.
    boolean_parallel_cylinders_common => run "boolean/parallel-cylinders-common";
    /// The same operands fused, and flush: the rim circles crossing on the
    /// coincident caps at the rulings' ends.
    boolean_parallel_cylinders_fuse => run "boolean/parallel-cylinders-fuse";
    /// A ruling on the target's seam: that block is the seam edge, not a
    /// section edge.
    boolean_parallel_cylinders_seam => run "boolean/parallel-cylinders-seam";
    /// Two walls touching from outside along a ruling: the curvature rule puts
    /// each outside the other, and the cut is the target with every id kept.
    /// Open CASCADE imprints the ruling (`analytic.counts_differ`).
    boolean_tangent_cylinders_cut => run "boolean/tangent-cylinders-cut";
    /// The same operands fused: both walls survive through the contact,
    /// `BooleanReason::TangentContact` through the runner's expected-error path.
    boolean_tangent_cylinders_fuse => run "boolean/tangent-cylinders-fuse";
    /// A pin touching a bore's wall from inside: the pin's wall is inside the
    /// bore, and the fuse is the bore with every id kept.
    boolean_pin_in_bore_fuse => run "boolean/pin-in-bore-fuse";
    /// The pin cut from the bore: both walls survive through the contact,
    /// `BooleanReason::TangentContact`.
    boolean_pin_in_bore_cut => run "boolean/pin-in-bore-cut";
    /// Two short cylinders crossing at 30°: each of the tool's cap planes cuts
    /// the target's wall in an ellipse coplanar with the tool's rim circle, and
    /// whether that rim lies along the ellipse is decided without the quartic.
    boolean_short_cross_cylinders_fuse => run "boolean/short-cross-cylinders-fuse";
    /// A box with an elliptic cylinder cut through it: an elliptic-cylinder
    /// operand face (ADR-0014).
    boolean_elliptic_operand_cut => run "boolean/elliptic-operand-cut";
    /// Two elliptic prisms crossing at right angles, fused: an elliptic
    /// cylinder on both operands, meeting in the two conics the unit
    /// Steinmetz solid's planes pull back to.
    boolean_elliptic_cross_fuse => run "boolean/elliptic-cross-fuse";
    /// A bored frustum cut by a tilted half-space: a whole ellipse on the
    /// cone, its pcurve fitted over the cone's own projection (ADR-0021).
    boolean_frustum_oblique_cut => run "boolean/frustum-oblique-cut";
    /// A slot cut past the same frustum: three planes parallel to the cone's
    /// axis, each meeting it in an exact hyperbola (ADR-0018).
    boolean_frustum_slot_cut => run "boolean/frustum-slot-cut";
    /// A drill across the same frustum's wall: a cone and a cylinder on skew
    /// axes, two traced and fitted quartic loops (ADR-0018, ADR-0019).
    boolean_frustum_cross_drill_cut => run "boolean/frustum-cross-drill-cut";
    /// A boss chamfered and then slotted: the cone Arris made taken back as
    /// a boolean operand, which is the closure ADR-0020 is about.
    boolean_chamfered_boss_slot_cut => run "boolean/chamfered-boss-slot-cut";
    /// A ball with a box's corner cut out of it: three small circles on a
    /// whole sphere face, each pcurve fitted over the sphere's own projection.
    boolean_ball_corner_cut => run "boolean/ball-corner-cut";
    /// A drill through a ball beside its axis: a sphere and a cylinder on
    /// skew axes, two traced loops.
    boolean_ball_offset_drill_cut => run "boolean/ball-offset-drill-cut";
    /// A bar through a ball that swallows one of its poles: the section loop
    /// winds once round `u`, from the seam back to the seam, and ends there
    /// at a latitude where a tolerance is a wide step in `u`.
    boolean_ball_polar_drill_cut => run "boolean/ball-polar-drill-cut";
    /// Two balls revolved about different axes, in common: a lens between
    /// two sphere faces meeting in one circle.
    boolean_ball_ball_common => run "boolean/ball-ball-common";
    /// A ring and a slab parallel to its axis, in common: two spiric ovals,
    /// each across both of the torus's seams.
    boolean_ring_slab_common => run "boolean/ring-slab-common";
    /// A ring in common with a block at a general pose: closed toric
    /// sections cut by the block's edges into blocks, one of which wraps
    /// past the end of its periodic knots — on a plane, whose exact pcurve
    /// carries those knots.
    boolean_ring_corner_common => run "boolean/ring-corner-common";
    /// A pin drilled through a ring's tube: a torus and a cylinder, two
    /// traced loops in the torus's own parameter plane (ADR-0019).
    boolean_ring_pin_cut => run "boolean/ring-pin-cut";
    /// A boss filleted at its base and then drilled through the blend: the
    /// torus Arris made taken back as a boolean operand (ADR-0020).
    boolean_filleted_boss_drill_cut => run "boolean/filleted-boss-drill-cut";
    /// A cube's corner filleted and then notched: a fillet's cylinders and
    /// its corner sphere cut after they were made (ADR-0020).
    boolean_filleted_corner_notch_cut => run "boolean/filleted-corner-notch-cut";
    /// A cone sliced by a plane through its apex: two rulings ending at the
    /// operand's singular vertex, which paves them (ADR-0021).
    boolean_cone_apex_slice_cut => run "boolean/cone-apex-slice-cut";
    /// A ball sliced by an oblique plane through a pole: a small circle
    /// through the sphere's singular vertex, split there (ADR-0021).
    boolean_ball_pole_slice_cut => run "boolean/ball-pole-slice-cut";
    /// The same slice turned 2e-4 of a radian off the seam's meridian: the
    /// circle crosses the seam again 1.8e-4 from the pole, found at the
    /// model's smallest distance, and its block between the two crossings is
    /// the seam's piece, not a section edge.
    boolean_pole_slice_beside_seam_cut => run "boolean/pole-slice-beside-seam-cut";
    /// A stub leaving a frustum through its cone wall, cut and then fused
    /// back: the restoring fuse traces the cone and the stub's wall over
    /// another region than the cut did, and the cut's section edge, on the
    /// cone and the bore, is along the fuse's own section by the surfaces,
    /// never by comparing two splines.
    boolean_frustum_stub_cut_then_fuse => run "boolean/frustum-stub-cut-then-fuse";
    /// A ball cut from a bar whose wall it meets at about 5° where the
    /// section leaves the cap: the checker's own fit of that grazing section,
    /// traced over the faces' boxes, and the edge's are each held to the one
    /// exact branch, so they lie within half a tolerance of each other and
    /// S5 finds every sample of its curve on the shared edge.
    boolean_grazing_ball_bar_cut => run "boolean/grazing-ball-bar-cut";
    /// A drill whose wall runs through both of a ball's poles: two traced
    /// loops, each through a singular vertex of the sphere.
    boolean_ball_pole_drill_cut => run "boolean/ball-pole-drill-cut";
    /// A quarter bend fused with the straight pipe it runs into: the torus
    /// and the wall touch along the tube circle and cross in a quartic
    /// beside it, one `Meets` of both kinds.
    boolean_pipe_elbow_fuse => run "boolean/pipe-elbow-fuse";
    /// A ball cut from a bore of its radius: a contact along a closed curve
    /// interior to both faces, refused as a tangent contact.
    boolean_ball_in_bore_cut => run "boolean/ball-in-bore-cut";
    /// A cone in common with a ball through its apex on its axis: a circle
    /// and a point in one `Meets`.
    boolean_ball_on_apex_common => run "boolean/ball-on-apex-common";
    /// A rectangle with a circular hole extruded: `boolean/through-hole`'s
    /// solid and numbers by the other path.
    sweep_extrude_plate_with_hole => run "sweep/extrude-plate-with-hole";
    /// A stadium with a hole: two half-cylinder faces with no seam beside the
    /// seamed bore, their boxes apart so the checker decides every pair.
    sweep_extrude_slot => run "sweep/extrude-slot";
    /// The plate-with-hole profile on `z = 10` extruded down: the same solid
    /// again, the profile face on top keeping its frame.
    sweep_extrude_downward => run "sweep/extrude-downward";
    /// A full ellipse with a turned major axis extruded: one seamed elliptic
    /// cylinder between two planes, volume `π a b h` (ADR-0014).
    sweep_extrude_ellipse => run "sweep/extrude-ellipse";
    /// Two half-ellipses joined by lines: two elliptic-cylinder faces with no
    /// seam, their full sections crossing off the faces.
    sweep_extrude_elliptic_slot => run "sweep/extrude-elliptic-slot";
    /// A rectangle with an elliptic hole: the seamed elliptic bore inside
    /// four walls, `boolean/through-hole`'s counts.
    sweep_extrude_plate_elliptic_hole => run "sweep/extrude-plate-elliptic-hole";
    /// A rectangle revolved a full turn about z: two seamed walls and two
    /// annuli of two closed rises each — `boolean/coaxial-cut`'s solid by the
    /// other path.
    sweep_revolve_tube => run "sweep/revolve-tube";
    /// The same rectangle a quarter turn: two flat ends, every rise an arc.
    sweep_revolve_quarter => run "sweep/revolve-quarter";
    /// An L revolved 270°: the walls' (u, v) regions run past `π` with no
    /// seam, two annular sectors of one notch face each other.
    sweep_revolve_l_profile => run "sweep/revolve-l-profile";
    /// A full turn of a rectangle with a rectangular hole: the hole closes
    /// into a ring-shaped cavity, one lump of two shells.
    sweep_revolve_hollow_ring => run "sweep/revolve-hollow-ring";
    /// The consumer's rectangle with a side on the axis: a solid cylinder in
    /// a full turn, and in a quarter and three-quarter turn a sector whose
    /// flat ends share the edge on the axis — reflex there past half a turn.
    sweep_revolve_onto_axis => run "sweep/revolve-onto-axis";
    /// The same profile in the consumer's units: a full turn at a micrometre
    /// default tolerance, volume 2π.
    sweep_probe_revolve_onto_axis_m => run "sweep/probe-revolve-onto-axis-m";
    /// A rectangle on the axis with a notch cut in from it: in a full turn
    /// the notch closes into a void of the one lump, two shells from one
    /// loop; a quarter turn opens it onto the flat ends.
    sweep_revolve_notch_to_axis => run "sweep/revolve-notch-to-axis";
    /// A trapezoid revolved about z: a frustum less its bore, its cone
    /// narrowing along the axis and, in `widening`, widening; a quarter turn's
    /// flat ends meet the cone along a ruling (ADR-0008).
    sweep_revolve_frustum => run "sweep/revolve-frustum";
    /// An arc about the origin revolved a full turn: a spherical zone less
    /// its bore, S5 deciding the sphere against the annuli and the bore by the
    /// meridian arm (ADR-0008).
    sweep_revolve_barrel => run "sweep/revolve-barrel";
    /// A circle revolved about z: `sample::torus` as a revolve builds it, one
    /// face and one vertex; a quarter turn's discs cut the torus in its tube
    /// circles (ADR-0008).
    sweep_revolve_ring => run "sweep/revolve-ring";
    /// An ellipse revolved about z: Open CASCADE builds the elliptic torus
    /// and Arris refuses it, `SweepReason::EllipticRevolve` naming the segment,
    /// since the surface it would sweep has no variant (ADR-0014).
    sweep_revolve_ellipse => run "sweep/revolve-ellipse";
    /// The consumer's 2-cube with one vertical edge filleted: a plane–plane
    /// blend, a cylinder between two circle ends (ADR-0007), at `r = 0.2`
    /// and at `r = 0.5`.
    blend_box_edge_fillet => run "blend/box-edge-fillet";
    /// The same solid rotated: a cap edge of the cube, the blend's ends on
    /// two side faces.
    blend_box_cap_edge_fillet => run "blend/box-cap-edge-fillet";
    /// The cube rotated about (1, 1, 1) and moved before the fillet: the
    /// same blend in an oblique pose, the edge and every probe named by the
    /// rotation's closed form.
    blend_box_posed_edge_fillet => run "blend/box-posed-edge-fillet";
    /// An extruded parallelogram's slanted top edge: the end faces are
    /// oblique to the edge, so each end trim is an ellipse arc with a fitted
    /// pcurve on the blend (the oblique-section rule).
    blend_box_oblique_end => run "blend/box-oblique-end";
    /// An extruded L's inner vertical edge: a concave plane–plane blend that
    /// adds material, its cylinder's axis in the notch and the blend face
    /// reversed against it.
    blend_l_inner_edge => run "blend/l-inner-edge";
    /// The cube's four vertical edges in one call: four disjoint blends, each
    /// cap edge cut at both its ends by two of them.
    blend_box_four_verticals => run "blend/box-four-verticals";
    /// The consumer's 2-cube with one vertical edge chamfered: a plane–plane
    /// chamfer, a plane between two segments on the caps, at `d = 0.2` and
    /// at `d = 0.5`.
    blend_box_edge_chamfer => run "blend/box-edge-chamfer";
    /// The cube's four vertical edges chamfered in one call.
    blend_box_four_vertical_chamfers => run "blend/box-four-vertical-chamfers";
    /// The miter's chamfer twin: a vertical and a cap edge at one corner, two
    /// chamfer planes meeting in a line, every face pair checked at `Full`.
    blend_box_corner_chamfers => run "blend/box-corner-chamfers";
    /// A second fillet on a filleted body: the extruded square's side edge
    /// 0–1, then 1–2 of that result, the two blends sharing the side face
    /// the first modified.
    blend_second_fillet => run "blend/second-fillet";
    /// The consumer's second-blend probe in its own units: the same two
    /// disjoint blends on a 0.1 m cube at a micrometre default tolerance.
    blend_probe_second_fillet_m => run "blend/probe-second-fillet-m";
    /// The consumer's probe of a vertical and a cap edge blended in one call,
    /// in the same units: the two blends share the side face x = 0 without
    /// meeting on it.
    blend_probe_cap_and_vertical_fillet_m => run "blend/probe-cap-and-vertical-fillet-m";
    /// A rise ending at a vertex of five edges, where a box stands on its
    /// corner on another's top edge: `BlendReason::VertexBlend`.
    blend_five_edge_vertex => run "blend/five-edge-vertex";
    /// Three fillets at one box corner: the sphere corner, an octant about
    /// the ball's centre tangent to the three cylinders, its pole a degenerate
    /// edge (ADR-0007). S5 decides the sphere against each cylinder and each
    /// plane by the meridian arm, so nothing is unchecked.
    blend_box_corner_three_fillets => run "blend/box-corner-three-fillets";
    /// Every edge of the cube filleted in one call: twelve cylinders and eight
    /// sphere corners, no face keeping a vertex of the box.
    blend_box_all_edges_fillet => run "blend/box-all-edges-fillet";
    /// Three chamfers at one box corner, meeting in a triangle.
    blend_box_corner_three_chamfers => run "blend/box-corner-three-chamfers";
    /// The miter: the vertical and the cap edge at one corner blended in one
    /// call, two cylinders meeting in the ellipse of their bisecting plane
    /// (ADR-0007). S5 decides the two blend cylinders by the equal-radius
    /// crossing arm, so the checker stage has nothing unchecked.
    blend_fillet_miter => run "blend/fillet-miter";
    /// Two fillets of a slanted prism's corner, the vertical edge at the
    /// parallelogram's acute vertex and its cap edge, whose dihedrals differ
    /// (ADR-0044): the ellipse of the two cylinders up to the cap's far
    /// contact, then the vertical's cylinder trimmed by the top face in a
    /// circle the top face takes, the third edge cut at its end — 12
    /// vertices, 18 edges and 8 faces, Open CASCADE's counts.
    blend_miter_unequal_dihedrals_fillet => run "blend/miter-unequal-dihedrals-fillet";
    /// The chamfer twin: the edges make unequal angles with the third edge, so
    /// the chamfers' line stops at `m` on the vertical's far contact and the
    /// cap chamfer, the wider, runs on in a chord across the slanted face, which
    /// takes it; the third edge is cut at its end (ADR-0044) — Open CASCADE's 12
    /// vertices, 18 edges and 8 faces.
    blend_miter_unequal_dihedrals_chamfer => run "blend/miter-unequal-dihedrals-chamfer";
    /// A half disc's chord edge: a plane against a cylinder along a ruling,
    /// convex, the blend cylinder tangent to the arc face along a ruling —
    /// S5 decides the pair by the parallel-axis arm's inside tangency.
    blend_d_chord_edge => run "blend/d-chord-edge";
    /// A half-round rib's root on a plate: the ruling arm concave, the blend
    /// adding material, tangent to the rib from outside.
    blend_rib_root_edge => run "blend/rib-root-edge";
    /// A plate with a D-shaped notch in one side, its top rim filleted: an
    /// open half circle between the top plane and the notch's cylinder, the
    /// blend half a torus trimmed at each end on its meridian in the plane
    /// through the cylinder's axis (ADR-0035).
    blend_d_notch_rim_fillet => run "blend/d-notch-rim-fillet";
    /// The D-notch's open arc chamfered: half a cone coaxial with the notch,
    /// trimmed on the plane through its axis (ADR-0035).
    blend_d_notch_rim_chamfer => run "blend/d-notch-rim-chamfer";
    /// The stadium's whole outline chamfered in one call: strips and half
    /// cones meeting in straight chords at the tangent vertices (ADR-0035).
    blend_stadium_outline_chamfer => run "blend/stadium-outline-chamfer";
    /// A disc's rim split in two half circles, chamfered: one half cone per
    /// edge (ADR-0035).
    blend_split_rim_disc_chamfer => run "blend/split-rim-disc-chamfer";
    /// A flat running tangentially into a quarter-round of a revolved bend, the
    /// edge along the flat filleted: the chain runs on into the arc between the
    /// end plane and the torus, a pair with no stripe, and Arris refuses it as
    /// `Unsupported` where Open CASCADE builds the chain (ADR-0035 §6).
    blend_chain_through_torus_unsupported => run "blend/chain-through-torus-unsupported";
    /// A cylinder's top rim filleted past its radius: the ball's centre circle
    /// is narrower than its tube, a spindle torus, which Arris refuses as
    /// `BlendTooLarge` where Open CASCADE builds it (ADR-0036 §3).
    blend_cylinder_rim_spindle_fillet => run "blend/cylinder-rim-spindle-fillet";
    /// A cone cut by an oblique plane, the ellipse between them filleted: a
    /// centre locus that is no circle, `Unsupported` where Open CASCADE builds
    /// it (ADR-0036 §4).
    blend_oblique_cut_cone_fillet => run "blend/oblique-cut-cone-fillet";
    /// A half cone's ruling on the end plane through its apex, filleted:
    /// `Unsupported`, the pair outside the table, where Open CASCADE builds it.
    blend_half_cone_ruling_fillet => run "blend/half-cone-ruling-fillet";
    /// The branch of a tee of two cylinders with crossing axes filleted along
    /// the quartic they meet in: `Unsupported`, where Open CASCADE builds it.
    blend_tee_cylinders_fillet => run "blend/tee-cylinders-fillet";
    /// A torus drilled parallel to its axis and off it, the bore's edge
    /// filleted: a torus against a cylinder off its axis, `Unsupported`, where
    /// Open CASCADE builds it.
    blend_drilled_torus_fillet => run "blend/drilled-torus-fillet";
    /// The D-notch with its circle's centre off the side face's plane: the open
    /// arc ends on a plane parallel to the cylinder's axis and off it, which
    /// cuts the torus in a spiric section, traced and fitted between the two
    /// contacts' trim points, each contact trimmed at its own angle (ADR-0037).
    blend_d_notch_off_axis_rim_fillet => run "blend/d-notch-off-axis-rim-fillet";
    /// Two overlapping bosses on a plate, the first's foot filleted: an open
    /// arc whose ends lie on the second boss's wall, a cylinder about a
    /// parallel axis off the arc's, which each contact meets at its own angle
    /// and the torus in a section traced and fitted (ADR-0037).
    blend_twin_boss_foot_fillet => run "blend/twin-boss-foot-fillet";
    /// The same foot chamfered: the cone's end on the second boss's wall,
    /// traced along its rulings and fitted (ADR-0037).
    blend_twin_boss_foot_chamfer => run "blend/twin-boss-foot-chamfer";
    /// A rib running into a round boss, its top edge filleted: the ruling
    /// stripe's cylinder ends on the boss's cylinder square across it in a
    /// quartic, traced along the rulings and fitted between where each
    /// contact line pierces the boss (ADR-0037).
    blend_rib_into_boss_fillet => run "blend/rib-into-boss-fillet";
    /// The same edge chamfered: the chamfer's plane ends on the boss in an
    /// ellipse, exact.
    blend_rib_into_boss_chamfer => run "blend/rib-into-boss-chamfer";
    /// The rib running into a conical boss, filleted: the stripe's cylinder
    /// ends on the cone in a quartic, traced and fitted (ADR-0037).
    blend_rib_into_cone_fillet => run "blend/rib-into-cone-fillet";
    /// The rib into the conical boss chamfered: the chamfer's plane ends on
    /// the cone in an ellipse the intersector writes exactly (`plane_cone`).
    blend_rib_into_cone_chamfer => run "blend/rib-into-cone-chamfer";
    /// A bar's wall running tangentially into a quarter cylinder, its top edge
    /// filleted: the fillet runs on from the line into the arc at the tangent
    /// vertex, a cylinder stripe and a torus section meeting on the ball's
    /// great circle (ADR-0035), an open chain trimmed at each end.
    blend_line_into_arc_fillet => run "blend/line-into-arc-fillet";
    /// The cap edge of a face a first blend trimmed, ending where that
    /// blend's contact meets its arc: the fillet runs on through the arc's
    /// torus into the next cap edge, an open chain of three Open CASCADE
    /// builds as one spine.
    blend_tangent_chain_cap_edge => run "blend/tangent-chain-cap-edge";
    /// A hole's top rim: a plane against a cylinder along a circle, convex,
    /// the blend a quarter of a torus coaxial with the hole with no ends
    /// (ADR-0007), S5 deciding it against the top face and the wall by the
    /// meridian arm (ADR-0008).
    blend_hole_rim_fillet => run "blend/hole-rim-fillet";
    /// The same rim chamfered: a 45° cone coaxial with the hole.
    blend_hole_rim_chamfer => run "blend/hole-rim-chamfer";
    /// A hole's rim read from NIST FTC-06, one half of its split circle
    /// filleted: the chain runs on through the seam's vertex and through the
    /// vertex of only the two arcs, where they continue one another (ADR-0041),
    /// and closes as a ring of two half tori. Open CASCADE's two faces each
    /// span a whole turn, so the fixture is held to its own counts and closed
    /// forms.
    blend_split_rim_two_edge_vertex_fillet => run "blend/split-rim-two-edge-vertex-fillet";
    /// The same rim chamfered: two half cones meeting on the chord at each
    /// vertex.
    blend_split_rim_two_edge_vertex_chamfer => run "blend/split-rim-two-edge-vertex-chamfer";
    /// A boss's base split in two half circles, the wall's seam at one vertex
    /// and nothing but the arcs at the other, one half filleted: concave, a
    /// ring of two half tori (ADR-0041).
    blend_split_boss_base_fillet => run "blend/split-boss-base-fillet";
    /// The same base chamfered.
    blend_split_boss_base_chamfer => run "blend/split-boss-base-chamfer";
    /// A conical hole's rim split in two half circles, one half filleted: a
    /// plane against a cone, convex, the ring of two half tori (ADR-0041).
    blend_split_cone_hole_rim_fillet => run "blend/split-cone-hole-rim-fillet";
    /// The same rim chamfered: two half cones meeting on the chord.
    blend_split_cone_hole_rim_chamfer => run "blend/split-cone-hole-rim-chamfer";
    /// A spherical dimple's rim split in two, one half filleted: a plane
    /// against a sphere, convex; Open CASCADE builds the ring as one face.
    blend_split_dimple_rim_fillet => run "blend/split-dimple-rim-fillet";
    /// A ring torus on a plate, its outer circle split in two, one half
    /// filleted: a plane against a torus, concave.
    blend_split_torus_ring_fillet => run "blend/split-torus-ring-fillet";
    /// A pin's shoulder split in two, one half filleted: a cylinder against a
    /// cone, both faces carrying their seam at the one vertex, so a vertex of
    /// four edges with two seams, each cut at its own contact (ADR-0041).
    blend_split_pin_shoulder_fillet => run "blend/split-pin-shoulder-fillet";
    /// A cylinder with a torus collar, the circle where they meet split in
    /// two, one half filleted: a cylinder against a torus, concave, two seams.
    blend_split_torus_collar_fillet => run "blend/split-torus-collar-fillet";
    /// A hole's rim of three arcs, two of its vertices carrying nothing but
    /// two arcs, one arc filleted: the chain runs on through all three and
    /// closes as a ring of three pieces of one torus (ADR-0041).
    blend_three_arc_hole_rim_fillet => run "blend/three-arc-hole-rim-fillet";
    /// A boss's base on a revolved disc: the circle arm concave, the torus
    /// adding material, every face on the one axis.
    blend_boss_base_fillet => run "blend/boss-base-fillet";
    /// A consumer's polyhedron bitten and filleted, in three variants
    /// (ADR-0028): `provenance.rs` holds its chain to consumer keys.
    provenance_consumer_rebuild => run "provenance/consumer-rebuild";
    /// The same recipe under three parameter sets, one test each so a
    /// variant that drifts says which: the eight bolt holes' chain is
    /// `crates/arris/tests/provenance.rs`'s subject, and these hold each
    /// variant's solid to the oracle and to its own `dump.<variant>.txt`.
    provenance_bolt_pattern_rebuild => variant "provenance/bolt-pattern-rebuild", "default";
    provenance_bolt_pattern_rebuild_thicker_wider => variant "provenance/bolt-pattern-rebuild", "thicker-wider";
    provenance_bolt_pattern_rebuild_tighter => variant "provenance/bolt-pattern-rebuild", "tighter";
    /// The split-order fixtures (ADR-0009), one test per variant as above:
    /// `provenance.rs` holds piece `k` of every split face to the same
    /// neighbours in every variant; these hold each variant's solid.
    provenance_split_bar_cut => variant "provenance/split-bar-cut", "default";
    provenance_split_bar_cut_left => variant "provenance/split-bar-cut", "left";
    provenance_split_bar_cut_right => variant "provenance/split-bar-cut", "right";
    provenance_split_bar_cut_narrow => variant "provenance/split-bar-cut", "narrow";
    provenance_split_frame_cut => variant "provenance/split-frame-cut", "default";
    provenance_split_frame_cut_left => variant "provenance/split-frame-cut", "left";
    provenance_split_frame_cut_right => variant "provenance/split-frame-cut", "right";
    provenance_split_frame_cut_narrow => variant "provenance/split-frame-cut", "narrow";
    provenance_split_cylinder_seam => variant "provenance/split-cylinder-seam", "default";
    provenance_split_cylinder_seam_turned_back => variant "provenance/split-cylinder-seam", "turned-back";
    provenance_split_cylinder_seam_turned_on => variant "provenance/split-cylinder-seam", "turned-on";
    provenance_split_cylinder_seam_narrow => variant "provenance/split-cylinder-seam", "narrow";
    provenance_split_cross_common => variant "provenance/split-cross-common", "default";
    provenance_split_cross_common_larger => variant "provenance/split-cross-common", "larger";
    provenance_split_cross_common_longer => variant "provenance/split-cross-common", "longer";
    provenance_split_cross_common_turned => variant "provenance/split-cross-common", "turned";
    /// The edge split-order fixture (ADR-0009, step 2): two notches into one
    /// box edge, the second splitting the piece the first left.
    provenance_split_edge_notch => variant "provenance/split-edge-notch", "default";
    provenance_split_edge_notch_slid => variant "provenance/split-edge-notch", "slid";
    provenance_split_edge_notch_apart => variant "provenance/split-edge-notch", "apart";
    provenance_split_edge_notch_narrow => variant "provenance/split-edge-notch", "narrow";
    /// NIST's CTC-02 AP242 solid alone, from the fetched tier: mitred
    /// cylinders of radius 10 that the checker at `Full` finds meeting away
    /// from their shared edges.
    #[ignore = "S5: f366–f368, mitred cylinders of one radius, meet away from their shared edges on a read Open CASCADE holds valid (docs/BACKLOG.md, S5 on the fetched tier's reads)"]
    regression_nist_ctc_02_ap242_mitred_pipes_s5 => part "regression/nist-ctc-02-ap242-mitred-pipes-s5";
    /// NIST's FTC-06 AP242 solid alone, from the fetched tier: cylinders at
    /// 45° that the checker at `Full` finds meeting away from their shared
    /// edges.
    #[ignore = "S5: f110, f113 and f114, cylinders mitred at 45°, meet away from their shared edges on a read Open CASCADE holds valid (docs/BACKLOG.md, S5 on the fetched tier's reads)"]
    regression_nist_ftc_06_ap242_mitred_pipes_s5 => part "regression/nist-ftc-06-ap242-mitred-pipes-s5";
    /// NIST's CTC-05 AP203-with-PMI solid alone, from the fetched tier: a
    /// torus elbow and the cylinders of its minor radius it joins, which the
    /// checker at `Full` finds meeting away from their shared edges.
    #[ignore = "S5: the torus f116 and the cylinders f114 and f118 of its minor radius meet away from their shared edges on a read Open CASCADE holds valid (docs/BACKLOG.md, S5 on the fetched tier's reads)"]
    regression_nist_ctc_05_ap203_torus_elbow_s5 => part "regression/nist-ctc-05-ap203-torus-elbow-s5";
    /// NIST's STC-10 AP242 solid alone, from the fetched tier: three holes
    /// into a bore, their axes crossing, which the checker at `Full` finds
    /// meeting the bore away from their shared edges.
    #[ignore = "S5: the holes f64, f86 and f87 meet the bore f58 away from their shared edges, as f61–f63 meet f60, on a read Open CASCADE holds valid (docs/BACKLOG.md, S5 on the fetched tier's reads)"]
    regression_nist_stc_10_ap242_holes_into_bore_s5 => part "regression/nist-stc-10-ap242-holes-into-bore-s5";
    /// NIST's FTC-10 AP242 solid alone, from the fetched tier: a mesh point of
    /// face f95 1.4e-4 from where its (u, v) evaluates.
    #[ignore = "mesh: a point of f95 stands 1.4e-4 from where its (u, v) evaluates, past the face's and its boundary's 4.6e-6 (docs/BACKLOG.md, a mesh point off its face on the fetched tier)"]
    regression_nist_ftc_10_ap242_mesh_off_face => part "regression/nist-ftc-10-ap242-mesh-off-face";
    /// NIST's CTC-04 AP203-with-PMI solid alone, from the fetched tier: its
    /// battery's drill along the second principal axis is a kernel fault.
    #[ignore = "drill_y: OpError::Internal(Fault::Geometry) where Open CASCADE builds the drilled part (docs/BACKLOG.md, a drill's geometry fault on the fetched tier)"]
    regression_nist_ctc_04_ap203_drill_geometry_fault => part "regression/nist-ctc-04-ap203-drill-geometry-fault";
    /// The cap-edge chain with the first radius at twice the second: the
    /// arc's torus is a horn torus, its major radius equal to its minor, which
    /// the ring refuses. The desired body is Open CASCADE's.
    #[ignore = "Reason::BlendTooLarge: the arc's fillet is a horn torus, major radius equal to minor, and the ring holds a torus to a ring torus (docs/BACKLOG.md, a horn torus on a blend's arc)"]
    regression_tangent_chain_horn_torus => run "regression/tangent-chain-horn-torus";
    /// A plate with a crescent hole tessellated at a coarse chord: the hole's
    /// two tangent arcs' polygons cross at the cusp, and the mesher refuses a
    /// valid face. The desired mesh is closed and within the inscribed bound.
    #[ignore = "MeshError::Face: at the fewest segments per turn the polygons of the hole's two tangent arcs cross near the cusp (docs/BACKLOG.md, a loop with a cusp at a coarse chord)"]
    regression_crescent_hole_coarse_mesh => run "regression/crescent-hole-coarse-mesh";
    /// A crescent profile whose small radius is not half the big one: its
    /// minimal polygon's chords from the cusp cross, and the profile is refused
    /// as self-intersecting. The desired body is Open CASCADE's prism.
    #[ignore = "ProfileError::SelfIntersecting: at the fewest segments per turn the chords of the loop's two tangent arcs cross at the cusp (docs/BACKLOG.md, a loop with a cusp at a coarse chord)"]
    regression_cusp_profile_off_half => run "regression/cusp-profile-off-half";
    /// A crescent's small arc filleted at 0.74 of its height in one pose: the
    /// trace's branch ends at the node `Q` a hair past `tol.linear` from the
    /// closed-form `Q`, and the cut is refused. The desired body is Open
    /// CASCADE's, as it is at rest.
    #[ignore = "OpError::Unsupported: the torus's branch ends at the node Q a hair past tol.linear from the closed-form Q, so no stretch holds both trim points (docs/BACKLOG.md, a cusp's node Q matched within tol.linear)"]
    regression_cusp_small_arc_wide_fillet => run "regression/cusp-small-arc-wide-fillet";
    /// A rib standing on a plate, its vertical corner edge filleted down to the
    /// plate: a convex blend ending on a face that surrounds the rib, the edge's
    /// corner with it concave, so the end arc lies in the rib's footprint and the
    /// plate's top gains the corner.
    blend_rib_corner_fillet_to_plate => run "blend/rib-corner-fillet-to-plate";
    /// The same rib's corner chamfered: the chamfer's end a segment in the
    /// footprint, the plate's top gaining the corner.
    blend_rib_corner_chamfer_to_plate => run "blend/rib-corner-chamfer-to-plate";
    /// A rib with a 45° sloped end, the sloped corner filleted: concave at the
    /// plate below it, convex at the rib's top above.
    blend_rib_sloped_end_fillet => run "blend/rib-sloped-end-fillet";
    /// A crescent prism's big arc filleted to the cusp where it is tangent to
    /// the small arc, both leaving the vertex the same way and both convex: the
    /// stripe is cut by the small wall, its traced cut ending at the node where
    /// the torus touches it on the spine (ADR-0042).
    blend_cusp_crescent_fillet => run "blend/cusp-crescent-fillet";
    /// The crescent's big arc chamfered to the cusp: the cone cut by the small
    /// wall, transverse at the spine (ADR-0042).
    blend_cusp_crescent_chamfer => run "blend/cusp-crescent-chamfer";
    /// The crescent's small arc filleted to the cusp: the stripe about the
    /// small axis cut by the big wall (ADR-0042).
    blend_cusp_crescent_small_arc_fillet => run "blend/cusp-crescent-small-arc-fillet";
    /// The crescent cut as a pocket, its big arc's floor edge filleted to the
    /// cusp: a concave stripe adding material, cut by the small wall (ADR-0042).
    blend_cusp_pocket_fillet => run "blend/cusp-pocket-fillet";
    /// A spandrel prism's bottom edge, a line between two planes, filleted to
    /// the cusp where the arc leaves it tangent: the stripe's cylinder cut by
    /// the arc's, ending at the node on the spine (ADR-0042).
    blend_cusp_spandrel_fillet => run "blend/cusp-spandrel-fillet";
    /// Both of the crescent's top arcs filleted: the stripes meet at the cusp in
    /// a corner patch, refused as `TangentChain` (ADR-0042 §6).
    blend_cusp_crescent_both_arcs_fillet => run "blend/cusp-crescent-both-arcs-fillet";
    /// A lip's underside arc filleted to an overhang tip, the cusp of FTC-06,
    /// its edges of opposite senses: Open CASCADE caps it with B-spline faces,
    /// and Arris refuses it as `TangentChain` (ADR-0042 §6).
    blend_cusp_overhang_tip_fillet => run "blend/cusp-overhang-tip-fillet";
    /// A pocket's vertical corner filleted: a concave blend adding material, at
    /// a concave corner with the floor and a convex one with the block's top.
    blend_pocket_corner_fillet => run "blend/pocket-corner-fillet";
    /// A square boss's vertical corner filleted down to its plate.
    blend_boss_corner_fillet => run "blend/boss-corner-fillet";
    /// A disc whose outline is two half circles, both filleted in one call:
    /// a closed chain of two open arcs at tangent vertices, the shape a rim has
    /// in the real parts where the file splits it at its seam.
    blend_split_rim_disc_fillet => run "blend/split-rim-disc-fillet";
    /// The shrunk CTC-04 corner: a chamfered stadium's chamfer foot filleted,
    /// the chain running on at a vertex of four edges where both its faces
    /// turn tangentially, each junction the ball's great circle between the two
    /// tangent edges, both cut (ADR-0039). Open CASCADE's cylinder-against-cone
    /// blend is walked, so the measures are held to the closed forms' band.
    blend_chamfered_stadium_foot_fillet => run "blend/chamfered-stadium-foot-fillet";
    /// The chamfer twin of `blend_chamfered_stadium_foot_fillet`: each junction
    /// a chord between the two tangent edges, both cut (ADR-0039).
    blend_chamfered_stadium_foot_chamfer => run "blend/chamfered-stadium-foot-chamfer";
    /// The concave twin: a stadium pocket's floor outline chamfered, the
    /// chamfer's root against the wall filleted, adding material round the
    /// whole outline through four vertices of four edges (ADR-0039).
    blend_stadium_pocket_chamfer_root_fillet => run "blend/stadium-pocket-chamfer-root-fillet";
    /// A hexagonal prism with its top edges chamfered, the foot of one facet
    /// filleted: each end reaches a vertex of four edges whose faces across are
    /// two, the next side face and the next facet, a fan. The end is an arc on
    /// each, and the extra edge between them is cut where the blend crosses it
    /// (ADR-0043; CTC-01's 12 plane × plane edges).
    blend_hex_chamfer_foot_fan_fillet => run "blend/hex-chamfer-foot-fan-fillet";
    /// The chamfer twin of `blend_hex_chamfer_foot_fan_fillet`: a chord on
    /// each piece across (ADR-0043).
    blend_hex_chamfer_foot_fan_chamfer => run "blend/hex-chamfer-foot-fan-chamfer";
    /// The shrunk CTC-04 and FTC-09 pin: a split disc's rim chamfered into two
    /// half cones, one half of the chamfer's foot filleted, the chain running on
    /// into the other half where both its faces turn at the split. Two rings
    /// meet at both ends, each junction the ball's meridian between the two
    /// split rulings, both cut (ADR-0039 §2).
    blend_split_pin_foot_fillet => run "blend/split-pin-foot-fillet";
    /// CTC-04's chamfered wall, read from STEP, its foot filleted: the walls
    /// turn round their rounded corners at faces tangent only to 9e-10 as
    /// read, which the blend's own tolerance takes as tangent (ADR-0040).
    blend_nist_ctc_04_chamfered_wall_fillet => run "blend/nist-ctc-04-chamfered-wall-fillet";
    /// A stadium's whole top outline filleted in one call: line, arc, line, arc,
    /// a closed chain through four tangent vertices.
    blend_stadium_outline_fillet => run "blend/stadium-outline-fillet";
    /// A stadium turned in space, its outline filleted through one named edge:
    /// the two half tori of one torus are a pair the checker's S5 leaves
    /// undecided at `Full`, where the same body unturned has nothing unchecked.
    #[ignore = "S5 leaves two faces of one torus undecided in a turned pose: f19 (torus) against f20 (torus) is not decided, so Full has an unchecked pair (docs/BACKLOG.md, coincident tori in S5; found by the C6 outline property)"]
    regression_turned_stadium_fillet_torus_pair => run "regression/turned-stadium-fillet-torus-pair";
    /// A frustum's top rim filleted: a plane against a cone along a coaxial
    /// circle, convex, the blend a ring torus (ADR-0036).
    blend_frustum_rim_fillet => run "blend/frustum-rim-fillet";
    /// The same rim chamfered: a cone coaxial with the frustum.
    blend_frustum_rim_chamfer => run "blend/frustum-rim-chamfer";
    /// A conical boss's base on its disc filleted: the cone row concave, the
    /// torus adding material.
    blend_cone_boss_base_fillet => run "blend/cone-boss-base-fillet";
    /// The same base chamfered, concave.
    blend_cone_boss_base_chamfer => run "blend/cone-boss-base-chamfer";
    /// A frustum revolved a half turn, its rim's half circle filleted: the cone
    /// row's open arc, ended on the two planes through the axis.
    blend_half_frustum_rim_fillet => run "blend/half-frustum-rim-fillet";
    /// The same half frustum's rim arc chamfered: a cone over the arc's range.
    blend_half_frustum_rim_chamfer => run "blend/half-frustum-rim-chamfer";
    /// A turned part's shoulder, a cylinder into a coaxial cone, chamfered: a
    /// cone through the two contacts, both faces' seams shortened.
    blend_turned_shoulder_chamfer => run "blend/turned-shoulder-chamfer";
    /// A turned part's shoulder, a cylinder into a coaxial cone, filleted: the
    /// blend a ring torus coaxial with both. Open CASCADE's blend is a B-spline
    /// off the torus, so the fixture's closed forms come with the pair.
    blend_turned_shoulder_fillet => run "blend/turned-shoulder-fillet";
    /// A cylinder capped by a coaxial spherical dome, its rim filleted: a
    /// circle meridian against a line, the blend a ring torus coaxial with
    /// both, both faces' seams shortened.
    blend_dome_rim_fillet => run "blend/dome-rim-fillet";
    /// The dome's rim chamfered: the cone through the cylinder's contact and
    /// the sphere's at the chord.
    blend_dome_rim_chamfer => run "blend/dome-rim-chamfer";
    /// A torus ring cut square to its axis, both rims filleted: a plane against
    /// a coaxial torus along two parallels, convex.
    blend_cut_torus_rims_fillet => run "blend/cut-torus-rims-fillet";
    /// The cut ring's rims chamfered: a cone through the floor's contact and
    /// the tube's at the chord.
    blend_cut_torus_rims_chamfer => run "blend/cut-torus-rims-chamfer";
    /// A toroidal bead on a disc, both roots filleted: a plane against a
    /// coaxial torus along two parallels, concave, the blend adding material.
    blend_bead_root_fillet => run "blend/bead-root-fillet";
    /// The bead's roots chamfered, concave.
    blend_bead_root_chamfer => run "blend/bead-root-chamfer";
    /// A convex fillet running into a step, as NIST CTC-03's and FTC-08's
    /// sampled edges do: the end is a mixed corner, whose convex corner edge
    /// the trim lengthens past the vertex and whose face across takes the
    /// spandrel (ADR-0038).
    blend_fillet_into_a_step => run "blend/fillet-into-a-step";
    /// The chamfer twin: the step's face takes the triangle between the chord
    /// and the corner.
    blend_chamfer_into_a_step => run "blend/chamfer-into-a-step";
    /// The step's face leaning back and overhanging: an ellipse across, the
    /// slanted corner edge lengthened along its own line.
    blend_fillet_into_a_leaning_step => run "blend/fillet-into-a-leaning-step";
    /// A concave fillet at a mixed corner at both ends, the mirror image of
    /// a convex one into a step: the concave corner edges lengthen.
    blend_concave_fillet_into_a_step => run "blend/concave-fillet-into-a-step";
    /// A fillet on a half-turned part's end face running into its riser, a
    /// cone (and a cylinder in the variant): the mixed corner on a curved
    /// face across, the ruling lengthened down the end face and the end traced
    /// on the riser (ADR-0037, ADR-0038).
    blend_fillet_into_a_turned_step => run "blend/fillet-into-a-turned-step";
    /// The chamfer twin: the chamfer's plane meets the riser in a conic,
    /// exact, and the riser takes the end from outside.
    blend_chamfer_into_a_turned_step => run "blend/chamfer-into-a-turned-step";
    /// A ring's open arc running into a step at both ends, the planes through
    /// its axis: each vertical edge lengthened down the cylinder, the step
    /// taking the torus's section (ADR-0038).
    blend_rim_fillet_into_a_step => run "blend/rim-fillet-into-a-step";
    /// The chamfer twin: a quarter cone, the steps taking its triangle.
    blend_rim_chamfer_into_a_step => run "blend/rim-chamfer-into-a-step";
    /// The step moved off the axis at one end: a plane parallel to it, which
    /// takes the torus's section traced and fitted (ADR-0037, ADR-0038).
    blend_rim_fillet_into_an_off_axis_step => run "blend/rim-fillet-into-an-off-axis-step";
    /// A plate with two round bosses nearly on each other, the first's foot
    /// chamfered, the body turned 45° about y: the chamfer's cone against the
    /// second wall's cylinder (parallel axes, 0.3 apart) is refused in this
    /// pose and built at rest (found by `blend_prop`'s end families).
    #[ignore = "Unsupported, a cone against a cylinder: the quartic's trace does not decide in this pose (found by the end families' property in `blend_prop`)"]
    regression_twin_boss_foot_turned_chamfer_cone_cylinder => run "regression/twin-boss-foot-turned-chamfer-cone-cylinder";
    /// An ogive bar end, two arcs meeting at a corner, its ruling filleted: two
    /// cylinders with parallel axes, convex, the blend a cylinder.
    #[ignore = "Unsupported, a cylinder against a cylinder: the parallel-cylinder row is decided and not built (ADR-0036 §5; docs/BACKLOG.md, the parallel-cylinder row of the blend table)"]
    regression_ogive_bar_ruling_fillet => run "regression/ogive-bar-ruling-fillet";
    /// A figure eight's waist filleted: two cylinders with parallel axes,
    /// concave, the blend a cylinder adding material.
    #[ignore = "Unsupported, a cylinder against a cylinder: the parallel-cylinder row is decided and not built (ADR-0036 §5; docs/BACKLOG.md, the parallel-cylinder row of the blend table)"]
    regression_figure_eight_waist_fillet => run "regression/figure-eight-waist-fillet";
    /// A cube whose filleted edge lies 1.3e-7 off one of its faces in the file,
    /// as two of NIST FTC-06's edges do: within the edge's tolerance, outside
    /// the face's.
    #[ignore = "kernel bug: Internal(Geometry(NotOnSurface)), a blend curve 1.3e-7 off the plane it is drawn on, held to the faces' tolerance where the edge's covers it (docs/BACKLOG.md, a blend of an edge off its face)"]
    regression_edge_off_its_plane_fillet => run "regression/edge-off-its-plane-fillet";
    /// NIST CTC-03's solid #101, one of two plane-against-plane edges filleted
    /// at the battery's radius: the blend's output fails the checker at L2 (a
    /// loop's pcurves jump 4.6e-7 in (u, v)) where Open CASCADE builds the
    /// body. A release build does not check, so the battery's census counted the
    /// edge as built.
    #[ignore = "kernel bug: the blend's output fails the checker at L2, pcurves 4.6e-7 apart where the file's faces are 1e-7 tolerant (docs/BACKLOG.md, a blend's output at a loosely written corner)"]
    regression_nist_ctc_03_two_plane_edges_fillet_checker_fault => run "regression/nist-ctc-03-two-plane-edges-fillet-checker-fault";
    /// A box cut to a barrel and a plane beside the corner, the vertical edge
    /// at the origin blended: its end reaches a vertex of four edges whose faces
    /// across are a plane and a cylinder, a fan of two with a curved piece,
    /// where Open CASCADE cuts the end on each and puts a vertex on the ridge
    /// (ADR-0043).
    blend_barrel_ridge_fan_fillet => run "blend/barrel-ridge-fan-fillet";
    /// The chamfer twin of `blend_barrel_ridge_fan_fillet`.
    blend_barrel_ridge_fan_chamfer => run "blend/barrel-ridge-fan-chamfer";
    /// A block under a convex roof of three planes, the rise at the origin
    /// blended: its end reaches a vertex of five edges whose faces across are a
    /// fan of three, one arc on each and a vertex on each of the two ridges, as
    /// Open CASCADE builds it (ADR-0043).
    blend_roof_fan_three_fillet => run "blend/roof-fan-three-fillet";
    /// The chamfer twin of `blend_roof_fan_three_fillet`.
    blend_roof_fan_three_chamfer => run "blend/roof-fan-three-chamfer";
    /// A 10-cube's top face pushed out by 2: a 10 × 10 × 12 block.
    offset_box_top_pushed => run "offset/box-top-pushed";
    /// A 10-cube's top face pulled in by 2: a 10 × 10 × 8 block.
    offset_box_top_pulled => run "offset/box-top-pulled";
    /// A 10-cube with every face pushed out by 1: a 12-cube, sharp joins.
    offset_box_whole_out => run "offset/box-whole-out";
    /// A 10-cube with every face pulled in by 1: an 8-cube.
    offset_box_whole_in => run "offset/box-whole-in";
    /// An extruded L's inner face pushed and pulled: the short arm's top
    /// trimmed or extended to meet it, two concave edges among the four
    /// recomputed.
    offset_l_inner_face => run "offset/l-inner-face";
    /// A blind pocket's floor pushed (shallower) and pulled (deeper): the
    /// four walls trimmed or extended. Open CASCADE returns the bare closed
    /// shell, made the solid it bounds (ADR-0048 §4).
    offset_pocket_floor => run "offset/pocket-floor";
    /// A trapezoid prism's top pushed past where its sloped sides meet: the
    /// top's edges would end before they start, refused as `Vanishes`
    /// where Open CASCADE drops the face.
    offset_wedge_top_past_ridge => run "offset/wedge-top-past-ridge";
    /// A square pyramid with one side pushed: the four sides no longer meet
    /// at the apex, refused as `VertexSplits` where Open CASCADE splits it
    /// into an edge.
    offset_pyramid_side_pushed => run "offset/pyramid-side-pushed";
    /// A cylinder's top cap pushed out by 3 and pulled in by 3: the wall
    /// extended along its rulings or trimmed, the cap's circle its section.
    offset_cylinder_top_pushed => run "offset/cylinder-top-pushed";
    /// A plate's through-hole wall pushed (the hole narrows to radius 2)
    /// and pulled (it widens to 4): the wall a coaxial cylinder, its seam
    /// kept in the seam's plane.
    offset_hole_wall_pushed => run "offset/hole-wall-pushed";
    /// A cylinder with all three faces moved out and in: sharp rims where
    /// the offset cap meets the offset wall.
    offset_cylinder_whole => run "offset/cylinder-whole";
    /// A frustum's cone side pushed and pulled: the coaxial cone d·√2
    /// wider, the rims its sections at the fixed planes.
    offset_frustum_side => run "offset/frustum-side";
    /// A boss capped by a hemisphere with the dome moved: the concentric
    /// sphere, the pole kept on the axis.
    offset_boss_dome => run "offset/boss-dome";
    /// A ring torus offset whole, out and in: the same major radius, the
    /// tube radius moved.
    offset_torus_ring => run "offset/torus-ring";
    /// A hole's wall moved past its own radius: the cylinder collapses,
    /// refused as `SurfaceCollapses` where Open CASCADE builds an
    /// inverted one.
    offset_hole_wall_collapses => run "offset/hole-wall-collapses";
    /// An elliptic cylinder's wall moved: the parallel curve of an ellipse
    /// is no ellipse, refused as `NoExactOffset`.
    offset_ellipse_wall_pushed => run "offset/ellipse-wall-pushed";
    /// A rounded cube's top face moved out and in: every face tangent to
    /// the next, the whole body dragged — planes moved, blend cylinders and
    /// corner spheres re-radiused, each tangent edge carried.
    offset_rounded_box_whole => run "offset/rounded-box-whole";
    /// The rounded cube moved in past its blends' radius: the dragged
    /// cylinders and spheres collapse, refused as `SurfaceCollapses`.
    offset_rounded_box_in_past_radius => run "offset/rounded-box-in-past-radius";
    /// A block's top face pushed and pulled with its filleted edge: the
    /// blend and the front face beyond it dragged along the tangent chain.
    offset_fillet_dragged => run "offset/fillet-dragged";
    /// A pocket's wall pushed and pulled with its floor fillet: the
    /// concave blend and the floor beyond it dragged, the blend tightened
    /// or widened about its own axis.
    offset_pocket_wall_dragged => run "offset/pocket-wall-dragged";
    /// A fillet sliced by an oblique flat, the top pulled: the dragged
    /// strip of blend shrinks clear of the fixed flat, refused as `Gap`.
    offset_fillet_cut_gap => run "offset/fillet-cut-gap";
    /// The same sliced block's top pushed: the dragged strip meets the
    /// fixed flat further up it, their section a new line.
    offset_fillet_cut_pushed => run "offset/fillet-cut-pushed";
    /// A pocket's floor pulled below the block's bottom face: the walls
    /// run through a face no moved face touches, refused as
    /// `SelfIntersects` from the `Full` report.
    offset_pocket_floor_through => run "offset/pocket-floor-through";
    /// A boss's side pushed past the plate's own side: the boss hangs
    /// through the plate's edge, refused as `SelfIntersects`.
    offset_boss_through_side => run "offset/boss-through-side";
    /// A 10-cube hollowed inward to a wall of 2, open on top.
    shell_box_top_in => run "shell/box-top-in";
    /// A 10-cube hollowed outward to a wall of 2, open on top: the skin runs up to the opening's plane.
    shell_box_top_out => run "shell/box-top-out";
    /// A 10-cube hollowed inward with no opening: a closed void, the second shell reversed.
    shell_box_closed_in => run "shell/box-closed-in";
    /// A 10-cube hollowed outward with no opening: the cube's faces the void inside a 14-cube.
    shell_box_closed_out => run "shell/box-closed-out";
    /// A 10-cube hollowed inward, open on top and bottom: a square tube, genus 1.
    shell_box_tube => run "shell/box-tube";
    /// An extruded L hollowed inward to a wall of 1, open at its long arm's end.
    shell_l_bracket => run "shell/l-bracket";
    /// A 10-cube hollowed inward, open on top and front, which share an edge: each rim face one loop.
    shell_box_top_side_in => run "shell/box-top-side-in";
    /// A 10-cube hollowed outward, open on top and front, which share an edge.
    shell_box_top_side_out => run "shell/box-top-side-out";
    /// A 10-cube hollowed inward, open on three faces meeting at a corner: the corner in no face.
    shell_box_three_in => run "shell/box-three-in";
    /// A 10-cube hollowed outward, open on three faces meeting at a corner.
    shell_box_three_out => run "shell/box-three-out";
    /// A wedge hollowed inward, open on both slanted faces, which share the ridge.
    shell_wedge_in => run "shell/wedge-in";
    /// A wedge hollowed outward, open on both slanted faces: the skin runs to the ridge extended.
    shell_wedge_out => run "shell/wedge-out";
    /// A cylinder hollowed inward, open at both ends: a tube, genus 1.
    shell_tube_in => run "shell/tube-in";
    /// A cylinder hollowed outward, open at both ends: the side the bore.
    shell_tube_out => run "shell/tube-out";
    /// A revolved cup (floor, torus fillet, cone) hollowed inward, open at the top.
    shell_cup_in => run "shell/cup-in";
    /// A revolved cup (floor, torus fillet, cone) hollowed outward, open at the top.
    shell_cup_out => run "shell/cup-out";
    /// A hemisphere hollowed inward, open on its flat: the skin through the pole.
    shell_bowl_in => run "shell/bowl-in";
    /// A hemisphere hollowed outward, open on its flat.
    shell_bowl_out => run "shell/bowl-out";
    /// A ball hollowed inward with no opening: a spherical void.
    shell_sphere_closed_in => run "shell/sphere-closed-in";
    /// A ball hollowed outward with no opening: the ball the void.
    shell_sphere_closed_out => run "shell/sphere-closed-out";
    /// A torus ring hollowed inward with no opening: a toroidal void.
    shell_torus_closed_in => run "shell/torus-closed-in";
    /// A torus ring hollowed outward with no opening: the ring the void.
    shell_torus_closed_out => run "shell/torus-closed-out";
    /// A pin hollowed inward past its radius: `SurfaceCollapses`, refused.
    shell_pin_collapses => run "shell/pin-collapses";
    /// An elliptic prism hollowed inward: `NoExactOffset`, refused.
    shell_ellipse_wall => run "shell/ellipse-wall";
    /// A 10-cube with its vertical edges filleted, hollowed inward open on
    /// top: each blend moves with its tangent sides, re-radiused to r - t.
    shell_rounded_box_in => run "shell/rounded-box-in";
    /// The same rounded cube shelled outward: each blend re-radiused to r + t.
    shell_rounded_box_out => run "shell/rounded-box-out";
    /// A block filleted along its top-front edge, its top opened: the blend
    /// would drag the opening, refused as `OpeningDragged`.
    shell_fillet_neighbour_open => run "shell/fillet-neighbour-open";
    /// A closed plate hollowed past its half-thickness: refused.
    shell_plate_past_half => run "shell/plate-past-half";
    /// A block with a boss hollowed past the boss's radius: refused.
    shell_boss_past_radius => run "shell/boss-past-radius";
    /// A pyramid on an oblong base hollowed: its apex splits, `VertexSplits`.
    shell_pyramid_apex => run "shell/pyramid-apex";
    /// A pocket whose floor is thinner than two walls, hollowed: the floor's
    /// skin runs through the bottom's, `SelfIntersects` from the `Full` report.
    shell_pocket_floor_thin => run "shell/pocket-floor-thin";
    /// NIST FTC-08's edge at (59.69, 30.48, 44.831) filleted alone at r 0.2721,
    /// a chain of three: the output fails the checker at L2 where Open CASCADE
    /// builds it, as it did before the blend's tangency took the faces'
    /// tolerance. A release build does not check, so the census counted it as
    /// built.
    #[ignore = "kernel bug: the blend's output fails the checker at L2, two loops' pcurves jump 3.1e-7 and 2.4e-6 in (u, v) (docs/BACKLOG.md, a blend's output at a loosely written corner)"]
    regression_nist_ftc_08_fillet_pcurve_jump_checker_fault => run "regression/nist-ftc-08-fillet-pcurve-jump-checker-fault";
    /// Open CASCADE's own STEP of `blend/bead-root-fillet`, read as a part: its
    /// walked blend of degree 2 by 14, on which the reader fits a contact's
    /// pcurve for minutes and then refuses it.
    #[ignore = "reader: no pcurve, #21 on face #149, the fit's normal equations singular on Open CASCADE's walked blend surface after minutes of fitting (docs/BACKLOG.md, a walked blend's file)"]
    regression_bead_root_fillet_occt_step => run "regression/bead-root-fillet-occt-step";
    /// Open CASCADE's STEP of a cylinder capped by a spherical dome, converted
    /// to B-splines: the cap's pole a vertex the reader finds no singular row
    /// for.
    #[ignore = "reader: unsupported entity, a face of one loop wrapping a period with no singular point on its side, the converted sphere cap's pole (docs/BACKLOG.md, a converted sphere cap)"]
    regression_dome_cap_nurbs_read_back => run "regression/dome-cap-nurbs-read-back";
    /// A cylinder whose circles are placed on the axis `(-1, -6.1e-17, 0)`
    /// with no reference direction, as NIST's CTC-04 and FTC-08 write them:
    /// ISO 10303-42's reference direction is world `Y` for an axis along `X`
    /// to rounding (ADR-0026 §4).
    real_axis_placement_along_x_without_reference => part "real/axis-placement-along-x-without-reference";
    /// A cylinder whose side face is bounded by its two circles and no seam,
    /// as NIST's CTC-03, FTC-10 and FTC-11 write it: the reader joins the two
    /// loops by a seam along a ruling (ADR-0026's amendment of step 5).
    real_seamless_cylinder_band => part "real/seamless-cylinder-band";
    /// A cone bounded by its base circle alone, its apex implicit, as NIST's
    /// FTC-10 writes its drill points: the reader adds the apex and its seam
    /// (ADR-0026's amendment of step 5).
    real_cone_face_without_its_apex => part "real/cone-face-without-its-apex";
    /// A half ball whose sphere face is bounded by one meridian circle
    /// through both poles, as NIST's FTC-06 writes face #351: the reader
    /// splits the edge at the pole it runs through (ADR-0026's amendment of
    /// step 5).
    real_edge_through_sphere_pole => part "real/edge-through-sphere-pole";
    /// A half ball shown in two AP242 saved views, draughting models that map
    /// its shape — one from a camera, as NIST's CTC-01 AP242 edition writes
    /// it, one displaced: a presentation places nothing, and the reader reads
    /// one solid (ADR-0026's amendment of step 9).
    real_saved_view_draughting_model => part "real/saved-view-draughting-model";
    /// NIST's CTC-01 whole, whose edge #1864 lies 0.0065 off its cylinders:
    /// its pcurve is fitted past the cap, and the cap judges the gap it
    /// leaves (ADR-0026's amendment of step 5).
    real_pcurve_fit_reported_as_gap => part "real/pcurve-fit-reported-as-gap";
    /// NIST's CTC-05 whole, held to the time budget its read once broke:
    /// fits that climbed to their tolerance one costly miss at a time
    /// (ADR-0026's amendment of step 5).
    real_slow_gap_refusal => part "real/slow-gap-refusal";
    /// NIST's FTC-07 whole, whose torus is tangent to a plane on its seam:
    /// the checker's S5 decides the pair as one touch (ADR-0026's amendment
    /// of step 5).
    real_torus_plane_section_undecided => part "real/torus-plane-section-undecided";
    /// NIST's FTC-09, AP203 geometry only: one solid of 158 faces, read whole.
    real_nist_ftc_09 => part "real/nist-ftc-09";
    /// `real/nist-ftc-09` with one plane wrapped in an `OFFSET_SURFACE` by
    /// hand: the refusal path, held to the refusal it records.
    real_nist_ftc_09_offset => part "real/nist-ftc-09-offset";
    /// NIST's CTC-01: one solid read whole, an exporter's edge fitted past the cap.
    real_nist_ctc_01 => part "real/nist-ctc-01";
    /// NIST's CTC-02: its solid refused for a real gap of 0.0112, its surface body as a surface.
    real_nist_ctc_02 => part "real/nist-ctc-02";
    /// NIST's CTC-03: one solid read whole, its seamless bands joined.
    real_nist_ctc_03 => part "real/nist-ctc-03";
    /// NIST's CTC-04: one solid read whole, its three surface bodies refused as surfaces.
    real_nist_ctc_04 => part "real/nist-ctc-04";
    /// NIST's CTC-05: its solid refused for a real gap of 0.0182, its surface body as a surface.
    real_nist_ctc_05 => part "real/nist-ctc-05";
    /// NIST's FTC-06: one solid read whole, its meridian edge split at the pole it passes.
    real_nist_ftc_06 => part "real/nist-ftc-06";
    /// NIST's FTC-07: one solid read whole.
    real_nist_ftc_07 => part "real/nist-ftc-07";
    /// NIST's FTC-08: one solid read whole.
    real_nist_ftc_08 => part "real/nist-ftc-08";
    /// NIST's FTC-10: one solid read whole, its bands joined and its cones given their apex.
    real_nist_ftc_10 => part "real/nist-ftc-10";
    /// NIST's FTC-11: one solid read whole, its seamless bands joined.
    real_nist_ftc_11 => part "real/nist-ftc-11";
    /// Found by the differential at 256 cases while the turned parts changed
    /// its draw: no blend in it.
    #[ignore = "checker: S5 cannot decide a cylinder against a cone in a twice-fused result, two pairs unchecked at Full (docs/BACKLOG.md, S5 of a cone against a cylinder)"]
    regression_cone_cylinder_fuse_s5_undecided => run "regression/cone-cylinder-fuse-s5-undecided";
    /// Found by the differential at 1000 cases of nightly 2026-09-28's seed: a
    /// revolve cut by an extrusion, the same S5 family as the fuse above.
    #[ignore = "checker: S5 cannot decide a cone against a cylinder in a cut result (docs/BACKLOG.md, S5 of a cone against a cylinder)"]
    regression_revolve_cut_extrusion_s5_undecided => run "regression/revolve-cut-extrusion-s5-undecided";
    /// A hexagon with an off-centre hole revolved about one of its edges,
    /// posed, cut by a cylinder and fused with a prism, drawn by the body
    /// bytes' property (nightly 2026-10-01): the result had a hole loop that
    /// met its outer loop (L5), because a closed section edge's box ignored the
    /// part of its range past the curve's domain; fixed, and what it waited for
    /// was the reader (a closed B-spline edge ending at its seam).
    boolean_body_bytes_revolved_hole_loops_intersect => run "boolean/body-bytes-revolved-hole-loops-intersect";
    /// A cylinder cut by a turned part, fused with a box blended at one edge
    /// (nightly 2026-09-30, case 862, seed `9f2abc6e…`): the fuse refused as
    /// `Internal(Lumps)`, an undecided nesting of two lumps. Cleared by the
    /// work since (the fillet's tolerance and the checker's rounding); kept as
    /// the case it was.
    boolean_cylinder_revolve_common_fillet_box_fuse => run "boolean/cylinder-revolve-common-fillet-box-fuse";
    /// A turned part cut by a cylinder and fused with a thin box (nightly
    /// 2026-10-01, case 476, seed `e016178c…`): `Internal(Builder)` then; Arris
    /// now builds it and agrees with Open CASCADE in the differential, and the
    /// corpus waits on the reader only.
    boolean_revolve_cylinder_cut_box_fuse_builder_fault => run "boolean/revolve-cylinder-cut-box-fuse-builder-fault";
    /// A cylinder and a posed prism in common, a chamfered box cut from it
    /// (nightly 2026-10-01, case 991): the checker's L5 then, since fixed by
    /// the bounds of a periodic NURBS; the
    /// corpus waits on the reader only.
    boolean_cylinder_prism_common_chamfer_cut_loops_intersect => run "boolean/cylinder-prism-common-chamfer-cut-loops-intersect";
    /// An elliptic prism, mirrored, fused with a posed revolve (nightly
    /// 2026-10-04, case 328, seed `3549c46a…`): a planar cap bounded by the
    /// ellipse and two section edges, whose polygon at chord 0.001 crosses
    /// itself, so the mesh is refused. The desired mesh is closed and valid.
    #[ignore = "MeshError::Face: the polygon of a cap's loop of an ellipse and two fitted section edges crosses itself at chord 0.001 (docs/BACKLOG.md, findings of the differential)"]
    regression_prism_mirror_revolve_fuse_mesh_crossing => run "regression/prism-mirror-revolve-fuse-mesh-crossing";
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

/// Every fixture directory of the corpus has a row in the table, and every
/// row names a fixture directory: a fixture nothing runs is a fixture that
/// cannot fail. Geometry fixtures are not run here (the runner skips them,
/// `corpus_lint.rs` and the geom crate hold them).
#[test]
fn every_fixture_directory_has_a_corpus_test() {
    let root = fixtures::corpus_root();
    let mut problems = Vec::new();
    let mut on_disk = std::collections::BTreeSet::new();
    for dir in fixtures::corpus() {
        if fixtures::kind_of(&dir).unwrap() == fixtures::Kind::Geometry {
            continue;
        }
        on_disk.insert(fixtures::name_of(&dir));
    }
    let rows: std::collections::BTreeSet<&str> = TABLE.iter().copied().collect();
    for name in &on_disk {
        if !rows.contains(name.as_str()) {
            problems.push(format!("{name}: a fixture directory with no corpus test"));
        }
    }
    for row in &rows {
        if !on_disk.contains(*row) || !root.join(row).join("fixture.json").is_file() {
            problems.push(format!("{row}: a corpus test with no fixture directory"));
        }
    }
    assert!(problems.is_empty(), "{}", problems.join("\n"));
}
