//! `ops::fillet` and `ops::chamfer` at random (ADR-0007): a box and an
//! extruded L of random proportions, a random subset of their plane–plane
//! edges — no edge meeting a concave one at a vertex — and a size below
//! the bound the faces allow. Each result is clean at `Full`, its record
//! audits, its volume is the input's less the closed form per edge
//! corrected by each miter's and each corner's, blending then moving
//! it equals moving then blending by volume and counts, and two runs dump
//! identically. A failure prints the case and the seed, and becomes a
//! fixture under `tests/fixtures/regression/` (`tests/fixtures/README.md`
//! §Property-test failures).
//!
//! The end families of ADR-0037 follow: a rib running into a round or
//! conical boss with its top edge blended, and a plate's boss whose foot
//! is blended where a second boss crosses it. Their blends end on a fitted
//! curve, so the volume is the unblended body's less or plus a section
//! integrated in closed form over the face across, to the model's
//! tolerance, and each result also round-trips through STEP.
//!
//! A rim split into two to four arcs (ADR-0041) follows: a hole's edge or a
//! boss's foot on a plate, one arc blended and the chain closing round the
//! ring, at the torus-swept closed form.
//!
//! A blend ending at a cusp (ADR-0042) follows: a crescent prism, its
//! pocket or a spandrel, one edge running to the cusp and its stripe cut by
//! the next wall, at the corner section integrated over the gap.
//!
//! A blend ending across a fan of faces (ADR-0043) closes the file: a
//! rise whose top is a roof of two or three planes meeting along sharp
//! ridges from the rise's top, at the roof's height integrated over the
//! blend's cross-section.
//!
//! Two cylinders with parallel axes meeting along a ruling (ADR-0036 §5)
//! follow: an ogive bar's tip and two united discs' waist, filleted or
//! chamfered, at the section's closed form.
//!
//! A concave blend ending on a face a fuse left in flush pieces (ADR-0082)
//! follows: an L of two or three fused boxes and a step of two or three,
//! the inner edge filleted or chamfered, at a lone concave edge's closed
//! form.
//!
//! A miter of unequal dihedrals (ADR-0044) is last: a prism over a random
//! parallelogram, the rise at one vertex and a top edge there blended, at
//! the two blends' regions in closed form less their overlap.

mod cusp;
mod cusp_corner;
mod four_edge_vertex;
mod horn;
mod miter;
mod parallel_cylinders;
mod plane_pairs;
mod real_part_kinks;
mod rim_arcs;
mod run_over_edge;
mod seam_vertex;
mod step_end;
mod support;
mod turned;
