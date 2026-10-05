//! **Split order at random poses** (ADR-0009, `docs/DATA-MODEL.md`
//! §Provenance): a box cut by one or two bars, and the same recipe built
//! again with every bar moved and resized within the bound the guarantee
//! is stated under — which entities bound which piece never changes. The
//! order of an origin's pieces has to be the same in both builds, and to
//! survive composition, a `Model::retain` that frees the first build's
//! slots for the second, and a `Provenance::mapped` through
//! `Model::import`.
//!
//! Neither build's ids may reach the comparison, so every entity is named
//! by the **label** of the primitive role it came from: which body of the
//! recipe, and which part of it. A piece is then described by its
//! neighbours' labelled origins — a face's by the faces it shares an edge
//! with, an edge's by its two end vertices — the same description
//! `crates/arris/tests/provenance.rs` gives the `provenance/split-*`
//! fixtures, in random poses instead of committed ones.
//!
//! A failure prints the shrunk case and the seed, and becomes a fixture
//! under `tests/fixtures/provenance/` (`tests/fixtures/README.md`
//! §Property-test failures).

use arris_debug::unmetered::{primitive_box, transform};
use std::collections::{BTreeMap, BTreeSet};

use arris_debug::prop::body::BarCut;
use arris_debug::testing::fail;
use arris_debug::unmetered::cut;
use arris_debug::{prop, prop_shards};
use arris_ops::OpError;
use arris_topo::provenance::{Origin, Relation, Role, audit};
use arris_topo::{Body, EntityId, Model, Orientation, Provenance, Shape};
use proptest::prelude::*;

/// What an entity of an input body is called, whichever build it belongs
/// to and whatever id it was given: the index of the primitive in the
/// recipe — `0` the box, `1` the first bar, `2` the second — and the role
/// of that primitive it was generated from.
type Label = (usize, Role);

/// A piece described without a single id: for every neighbour of it in
/// the result, that neighbour's labelled origins. A face's neighbours are
/// the faces it shares an edge with, an edge's its two end vertices.
type Signature = BTreeSet<Vec<(Relation, Label)>>;

/// Which kind of output an order is read over. The guarantee is among
/// outputs of **one** kind: an origin also lists the vertices and, for a
/// face, the section edges it generated, and those reach the record in
/// the operation's own recording order (ADR-0009).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Kind {
    Face,
    Edge,
}

impl Kind {
    fn holds(self, s: Shape) -> bool {
        matches!(
            (self, s.id),
            (Kind::Face, EntityId::Face(_)) | (Kind::Edge, EntityId::Edge(_))
        )
    }
}

/// The order one build gives every origin it lists two or more outputs of
/// one kind for, as signatures: what two builds of the same recipe have
/// to agree on.
type Order = BTreeMap<(Relation, Label, Kind), Vec<Signature>>;

/// One build of the recipe: the result, the cuts composed, and the label
/// of every entity the cuts name as an origin.
struct Built {
    body: Body,
    /// The first cut's record, and the second's when there is one.
    steps: Vec<Provenance>,
    /// The cuts composed left to right.
    record: Provenance,
    labels: BTreeMap<Shape, Label>,
}

fn forward(id: impl Into<EntityId>) -> Shape {
    Shape::new(id, Orientation::Forward)
}

/// One primitive of the recipe in `m`: the box built and moved into the
/// case's pose, with the two records composed, so every entity of it is
/// `Generated` from exactly one role.
fn primitive(
    m: &mut Model,
    boxed: &arris_debug::prop::body::Boxed,
) -> Result<(Body, Provenance), OpError> {
    let (body, made) = primitive_box(m, boxed.min, boxed.max)?;
    let (moved, motion) = transform(m, body, &boxed.pose)?;
    Ok((moved, made.then(&motion)))
}

/// The recipe of build `which` in `m`: the box, then a cut with each bar
/// in turn.
fn build(m: &mut Model, case: &BarCut, which: usize) -> Result<Built, TestCaseError> {
    let mut labels = BTreeMap::new();
    let mut label_body = |p: &Provenance, index: usize| {
        for origin in p.origins_recorded() {
            let Origin::Role(role) = origin else {
                continue;
            };
            for &output in p.generated_from(origin) {
                labels.insert(output, (index, role));
            }
        }
    };
    let (mut body, p) = primitive(m, &case.cuboid_of(which)).map_err(fail)?;
    label_body(&p, 0);
    let mut steps = Vec::new();
    for (k, bar) in case.slabs(which).iter().enumerate() {
        let (tool, p) = primitive(m, bar).map_err(fail)?;
        label_body(&p, k + 1);
        let (next, cut_record) = cut(m, body, tool).map_err(|e| fail(format!("cut {k}: {e}")))?;
        audit(m, &[body, tool], next, &cut_record)
            .map_err(|e| fail(format!("cut {k}: provenance: {e}")))?;
        steps.push(cut_record);
        body = next;
    }
    let record = steps
        .iter()
        .skip(1)
        .fold(steps[0].clone(), |acc, next| acc.then(next));
    Ok(Built {
        body,
        steps,
        record,
        labels,
    })
}

/// An output's origins as labels. An origin the labels do not hold is a
/// programming error in the test, not a kernel one: the cuts' origins are
/// entities of the primitives, and every entity of a primitive has a
/// role.
fn labelled(built: &Built, output: Shape) -> Result<Vec<(Relation, Label)>, TestCaseError> {
    built
        .record
        .origins(output)
        .into_iter()
        .map(|(relation, origin)| match origin {
            Origin::Entity(s) => built
                .labels
                .get(&s)
                .map(|&label| (relation, label))
                .ok_or_else(|| fail(format!("{s} has no role"))),
            Origin::Role(r) => Err(fail(format!("a cut named the role {r}"))),
        })
        .collect()
}

/// Every face of the result using each edge, from the faces' loops.
fn faces_by_edge(m: &Model, body: Body) -> Result<BTreeMap<EntityId, Vec<Shape>>, TestCaseError> {
    let mut out: BTreeMap<EntityId, Vec<Shape>> = BTreeMap::new();
    for f in m.faces(body).map_err(fail)? {
        for l in m.face(f.id).map_err(fail)?.loops() {
            for c in l.coedges() {
                let users = out.entry(EntityId::Edge(c.edge())).or_default();
                if !users.contains(&forward(f.id)) {
                    users.push(forward(f.id));
                }
            }
        }
    }
    Ok(out)
}

/// The neighbours of an output: for a face the faces it shares an edge
/// with, for an edge its two end vertices, each as its labelled origins.
fn signature(
    m: &Model,
    built: &Built,
    by_edge: &BTreeMap<EntityId, Vec<Shape>>,
    output: Shape,
) -> Result<Signature, TestCaseError> {
    let mut out = Signature::new();
    match output.id {
        EntityId::Face(f) => {
            for l in m.face(f).map_err(fail)?.loops() {
                for c in l.coedges() {
                    for &g in by_edge.get(&EntityId::Edge(c.edge())).into_iter().flatten() {
                        if g != output {
                            out.insert(labelled(built, g)?);
                        }
                    }
                }
            }
        }
        EntityId::Edge(e) => {
            let edge = m.edge(e).map_err(fail)?;
            for v in [edge.start(), edge.end()] {
                out.insert(labelled(built, forward(v))?);
            }
        }
        _ => {}
    }
    Ok(out)
}

/// The order this build gives every origin it lists two or more faces —
/// or two or more edges — of one kind for.
fn order(m: &Model, built: &Built) -> Result<Order, TestCaseError> {
    let by_edge = faces_by_edge(m, built.body)?;
    let mut out = Order::new();
    for origin in built.record.origins_recorded() {
        let Origin::Entity(s) = origin else {
            continue;
        };
        let Some(&label) = built.labels.get(&s) else {
            continue;
        };
        for (relation, list) in [
            (Relation::Modified, built.record.modified_from(origin)),
            (Relation::Generated, built.record.generated_from(origin)),
        ] {
            for kind in [Kind::Face, Kind::Edge] {
                let outputs: Vec<Shape> = list.iter().copied().filter(|s| kind.holds(*s)).collect();
                if outputs.len() < 2 {
                    continue;
                }
                let mut signatures = Vec::with_capacity(outputs.len());
                for o in outputs {
                    signatures.push(signature(m, built, &by_edge, o)?);
                }
                out.insert((relation, label, kind), signatures);
            }
        }
    }
    Ok(out)
}

/// Both builds of `case`, each in its own model, and the order each
/// gives: the first in `m`, which the caller may then reuse.
fn orders(m: &mut Model, case: &BarCut) -> Result<(Built, Order, Order), TestCaseError> {
    let first = build(m, case, 0)?;
    let base = order(m, &first)?;
    let mut other = Model::default();
    let second = build(&mut other, case, 1)?;
    let moved = order(&other, &second)?;
    Ok((first, base, moved))
}

prop_shards! {
    /// Moving and resizing every bar leaves the order of every origin's
    /// pieces alone: the two builds, in models of their own, describe
    /// piece `k` of every split origin the same way. Then the same
    /// perturbed recipe built again in the *first* model, after a
    /// `Model::retain` that frees every slot it used, agrees with both —
    /// an id reused by another entity never reaches the order.
    the_order_survives_moving_the_bars_and_reusing_the_ids
        [shard_0 shard_1 shard_2 shard_3]
        (case) = prop::body::bar_cut() => {
            let mut m = Model::default();
            let (_, base, moved) = orders(&mut m, &case)?;
            prop_assert!(!base.is_empty(), "the bars split nothing");
            prop_assert_eq!(&base, &moved, "the bars moved");
            m.retain(&[]).map_err(fail)?;
            let again = build(&mut m, &case, 1)?;
            let refilled = order(&m, &again)?;
            prop_assert_eq!(&refilled, &moved, "the same recipe over freed slots");
            Ok(())
        }
}

prop_shards! {
    /// [`Provenance::then`] **nests** the order: the outputs standing for
    /// piece `i` of an origin come before those of piece `i + 1`. Read
    /// off the two cuts — the first's pieces of an origin in order, each
    /// one's own images under the second — the composed list is those
    /// blocks end to end, and nothing else.
    then_nests_the_pieces_of_the_first_cut_inside_the_composed_list
        [shard_0 shard_1]
        (case) = prop::body::bar_cut() => {
            let mut m = Model::default();
            let built = build(&mut m, &case, 0)?;
            if built.steps.len() < 2 {
                return Ok(());
            }
            let (first, second) = (&built.steps[0], &built.steps[1]);
            for origin in first.origins_recorded() {
                let composed = built.record.modified_from(origin);
                let mut blocks: Vec<usize> = Vec::new();
                for &piece in first.modified_from(origin) {
                    // What the second cut made of that piece: its own
                    // pieces, or the piece itself where it was untouched.
                    let images: Vec<Shape> = if second.modified_from(piece).is_empty() {
                        vec![piece]
                    } else {
                        second.modified_from(piece).to_vec()
                    };
                    blocks.extend(
                        images
                            .iter()
                            .filter_map(|image| composed.iter().position(|o| o == image)),
                    );
                }
                prop_assert!(
                    blocks.windows(2).all(|w| w[0] < w[1]),
                    "{origin}: the composed list {composed:?} does not nest {blocks:?}"
                );
                prop_assert_eq!(
                    blocks.len(),
                    composed.len(),
                    "{}: the composed list holds an output no piece of the first cut stands for",
                    origin
                );
            }
            Ok(())
        }
}

prop_shards! {
    /// [`Provenance::mapped`] through a `Model::import` keeps the order:
    /// the result imported into a fresh model and the record translated
    /// through the `IdMap` describes every origin's pieces exactly as the
    /// record it came from does. The origins are entities of the input
    /// bodies, which the import does not carry, so they stay as they are
    /// and the labels still name them.
    mapped_through_an_import_keeps_the_order
        [shard_0 shard_1]
        (case) = prop::body::bar_cut() => {
            let mut m = Model::default();
            let built = build(&mut m, &case, 0)?;
            let base = order(&m, &built)?;
            let mut fresh = Model::default();
            let (body, map) = fresh.import(&m, built.body).map_err(fail)?;
            let imported = Built {
                body,
                steps: Vec::new(),
                record: built.record.mapped(&map),
                labels: built.labels.clone(),
            };
            prop_assert_eq!(order(&fresh, &imported)?, base, "mapped through an import");
            Ok(())
        }
}

/// The `build/` fixtures a consumer's body is drawn from: the recipe's
/// directory and variant, each built through `ops::build`.
const CONSUMER_BODIES: [(&str, &str); 4] = [
    ("build/tetrahedron", "default"),
    ("build/tetrahedron", "skewed"),
    ("build/l-prism", "default"),
    ("build/frame", "default"),
];

/// A consumer's body in a random rigid pose, and a random axis-aligned
/// box about a point of it: which body, the pose, where the box's centre
/// is in the body's own bounding box (fractions of it), and the box's
/// half-extents (fractions of the body's largest extent).
fn consumer_cut() -> impl Strategy<Value = (usize, arris_math::Isometry, [f64; 3], [f64; 3])> {
    (
        0..CONSUMER_BODIES.len(),
        prop::pose_in(50.0),
        [
            prop::finite_f64(0.0..=1.0),
            prop::finite_f64(0.0..=1.0),
            prop::finite_f64(0.0..=1.0),
        ],
        [
            prop::finite_f64(0.1..=0.6),
            prop::finite_f64(0.1..=0.6),
            prop::finite_f64(0.1..=0.6),
        ],
    )
}

prop_shards! {
    /// **The chain of a consumer's body ends in its own words**
    /// (ADR-0028): a `build/` fixture's body,
    /// built through `ops::build`, moved to a random pose and cut by a
    /// random box, has a composed record — build, then transform, then
    /// the box, then the cut — that `audit` holds with no inputs, and
    /// every origin of every output is a role: the consumer's key or the
    /// box's part, never an entity and never nowhere.
    a_consumer_bodys_chain_ends_at_its_keys_or_the_tools_roles
        [shard_0 shard_1 shard_2 shard_3]
        ((which, pose, at, half)) = consumer_cut() => {
            let (fixture, variant) = CONSUMER_BODIES[which];
            let dir = arris_debug::fixtures::corpus_root().join(fixture);
            let mut chain = arris_debug::corpus::chain(&dir, variant).map_err(fail)?;
            let built = &chain.steps[&chain.result];
            let (body, record) = (built.body, built.provenance.clone());
            let m = &mut chain.model;
            let closure = m.closure(body).map_err(fail)?;
            let points: Vec<_> = closure
                .vertices
                .iter()
                .map(|&v| m.vertex(v).map(|x| x.point()))
                .collect::<Result<_, _>>()
                .map_err(fail)?;
            let lo = points.iter().fold([f64::INFINITY; 3], |a, p| [a[0].min(p.x), a[1].min(p.y), a[2].min(p.z)]);
            let hi = points.iter().fold([f64::NEG_INFINITY; 3], |a, p| [a[0].max(p.x), a[1].max(p.y), a[2].max(p.z)]);
            let size = (0..3).map(|i| hi[i] - lo[i]).fold(0.0, f64::max);
            let (moved, motion) = transform(m, body, &pose).map_err(fail)?;
            let local = arris_math::Point3::new(
                lo[0] + at[0] * (hi[0] - lo[0]),
                lo[1] + at[1] * (hi[1] - lo[1]),
                lo[2] + at[2] * (hi[2] - lo[2]),
            );
            let centre = pose.apply(local);
            let h = half.map(|f| f * size);
            let (tool, boxed) = primitive_box(
                m,
                [centre.x - h[0], centre.y - h[1], centre.z - h[2]],
                [centre.x + h[0], centre.y + h[1], centre.z + h[2]],
            )
            .map_err(fail)?;
            let (out, cutting) = match cut(m, moved, tool) {
                Ok(done) => done,
                // A box that misses the body or swallows it has nothing
                // to name, and neither does a contact the boolean refuses.
                Err(OpError::Degenerate { .. }) => return Ok(()),
                Err(e) => return Err(fail(e)),
            };
            let whole = record.then(&motion).then(&boxed).then(&cutting);
            audit(m, &[], out, &whole).map_err(fail)?;
            for origin in whole.origins_recorded() {
                prop_assert!(
                    matches!(origin, Origin::Role(Role::Consumer(_) | Role::Box(_))),
                    "{origin} is an origin of the chain"
                );
            }
            prop_assert!(
                whole
                    .origins_recorded()
                    .any(|o| matches!(o, Origin::Role(Role::Consumer(_)))),
                "nothing of the consumer's body survives"
            );
            Ok(())
        }
}
