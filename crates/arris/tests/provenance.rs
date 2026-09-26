//! The provenance chain of `provenance/bolt-pattern-rebuild`
//! (`docs/ROADMAP.md` §M4, `docs/DATA-MODEL.md` §Provenance): a plate and
//! eight bolt-hole tools built, the eight cuts composed with
//! [`Provenance::then`], and every entity of the result followed back to
//! the role it came from. The fixture is the same recipe under three
//! parameter sets, and the chain has to be the same in all three — a
//! consumer's persistent name is a function of the chain, so it must not
//! move when a dimension does. Eight of eight hole walls named through
//! the tool they were cut with is the milestone's number.
//!
//! The corpus test in `corpus.rs` holds the same fixture to the oracle's
//! numbers in each variant; this one reads only the records.
//!
//! The split-order fixtures (`provenance/split-*`, ADR-0009) are the same
//! kind of proof for the *order* of an origin's pieces: a recipe whose
//! variants move, resize and turn a split without changing which
//! entities bound which piece, and piece `k` of every split face has to
//! keep its neighbours in every variant. `split-edge-notch` does it for
//! **edges** — two notches cut one after the other into one box edge, so
//! the cuts composed list three pieces of it — and the edge order is read
//! geometrically too: edges of one origin on one curve ascend by their
//! range on it, which is the rule itself rather than a consequence of it.
//!
//! `provenance/consumer-rebuild` makes the same claim on a body a
//! consumer built through `ops::build` (ADR-0028): its chains end at the
//! consumer's own keys, the same in every variant.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use arris::topo::entity::EdgeGeometry;
use arris::topo::provenance::{
    BoxPart, ConsumerKey, Coord, CylinderPart, Origin, Provenance, Relation, Role, Side,
};
use arris::topo::{EdgeId, EntityId, FaceId, Orientation, Shape};
use arris_debug::corpus::Chain;
use arris_debug::polyhedron::edge_key;
use arris_debug::{corpus, fixtures};

/// The fixture every test here reads.
const FIXTURE: &str = "provenance/bolt-pattern-rebuild";

/// Its three parameter sets, `default` first.
const VARIANTS: [&str; 3] = ["default", "thicker-wider", "tighter"];

/// The steps that make a body from nothing: the plate, then the eight
/// tools in the order the cuts take them.
const PRIMITIVES: [&str; 9] = ["plate", "h0", "h1", "h2", "h3", "h4", "h5", "h6", "h7"];

/// The eight cuts, `result` being the last.
const CUTS: [&str; 8] = ["c0", "c1", "c2", "c3", "c4", "c5", "c6", "result"];

fn chain_of(variant: &str) -> Chain {
    let dir = fixtures::corpus_root().join(FIXTURE);
    corpus::chain(&dir, variant).unwrap_or_else(|e| panic!("{variant}: {e}"))
}

/// The records of `names` composed left to right.
fn composed(chain: &Chain, names: &[&str]) -> Provenance {
    names
        .iter()
        .skip(1)
        .fold(chain.steps[names[0]].provenance.clone(), |p, name| {
            p.then(&chain.steps[*name].provenance)
        })
}

/// The whole recipe: the nine bodies built, then the eight cuts.
fn whole(chain: &Chain) -> Provenance {
    let names: Vec<&str> = PRIMITIVES.iter().chain(CUTS.iter()).copied().collect();
    composed(chain, &names)
}

/// The faces among `shapes`, in order.
fn faces(shapes: &[Shape]) -> Vec<FaceId> {
    shapes
        .iter()
        .filter_map(|s| match s.id {
            EntityId::Face(f) => Some(f),
            _ => None,
        })
        .collect()
}

/// The forward handle to a face, which is how a face reaches a record.
fn forward(face: FaceId) -> Shape {
    Shape::new(face, Orientation::Forward)
}

/// The one face a step generated from `role`.
fn face_from(p: &Provenance, role: Role) -> FaceId {
    let found = faces(p.generated_from(role));
    assert_eq!(found.len(), 1, "{role} generated {found:?}");
    found[0]
}

/// Each of the eight hole walls is generated from the wall of the tool
/// that cut it, and from no other face: the chain of eight cuts names
/// which bolt hole a wall belongs to, which is what a rebuild needs.
#[test]
fn eight_of_eight_hole_walls_name_the_tool_they_came_from() {
    for variant in VARIANTS {
        let chain = chain_of(variant);
        let cuts = composed(&chain, &CUTS);
        let body = chain.result().unwrap();
        let closure = chain.model.closure(body).unwrap();
        let mut walls = Vec::new();
        for tool in &PRIMITIVES[1..] {
            let wall = face_from(
                &chain.steps[*tool].provenance,
                Role::Cylinder(CylinderPart::Wall),
            );
            let tool_wall = forward(wall);
            // The tool's face is gone and its image is the hole's wall
            // (`docs/DATA-MODEL.md` §Provenance, the tool rule).
            assert!(cuts.is_deleted(tool_wall), "{variant}: {tool}'s wall kept");
            let image = faces(cuts.generated_from(tool_wall));
            assert_eq!(image.len(), 1, "{variant}: {tool}'s wall made {image:?}");
            let hole = image[0];
            assert!(
                closure.faces.contains(&hole),
                "{variant}: {tool}'s image {hole:?} is not in the result"
            );
            assert!(
                matches!(
                    chain
                        .model
                        .surface(chain.model.face(hole).unwrap().surface()),
                    Ok(arris::geom::Surface::Cylinder { .. })
                ),
                "{variant}: {tool}'s image is not a cylindrical face"
            );
            let origins = cuts.origins(forward(hole));
            assert_eq!(
                origins,
                [(Relation::Generated, Origin::Entity(tool_wall))],
                "{variant}: {tool}'s image comes from more than its tool"
            );
            walls.push(hole);
        }
        let distinct: BTreeSet<FaceId> = walls.iter().copied().collect();
        assert_eq!(
            distinct.len(),
            8,
            "{variant}: {walls:?} are not eight walls"
        );
    }
}

/// Followed through the primitives too, every hole wall's origins end at
/// one role — `Role::Cylinder(CylinderPart::Wall)` — and the relation is
/// `Generated`, since a chain that passes through a role is generation
/// however the pieces were cut ([`Relation::then`]).
#[test]
fn every_hole_wall_ends_at_the_cylinder_wall_role() {
    let role = Role::Cylinder(CylinderPart::Wall);
    for variant in VARIANTS {
        let chain = chain_of(variant);
        let cuts = composed(&chain, &CUTS);
        let all = whole(&chain);
        let from_role: BTreeSet<FaceId> = faces(all.generated_from(role)).into_iter().collect();
        assert_eq!(from_role.len(), 8, "{variant}: {from_role:?}");
        for &wall in &from_role {
            assert_eq!(
                all.origins(forward(wall)),
                [(Relation::Generated, Origin::Role(role))],
                "{variant}: {wall:?} comes from more than the wall role"
            );
        }
        // The same eight faces the eight cuts name through their tools.
        let images: BTreeSet<FaceId> = PRIMITIVES[1..]
            .iter()
            .flat_map(|tool| {
                let wall = forward(face_from(&chain.steps[*tool].provenance, role));
                faces(cuts.generated_from(wall))
            })
            .collect();
        assert_eq!(from_role, images, "{variant}");
    }
}

/// The plate's top face is modified by the chain of cuts into one face,
/// and that face carries nine loops: its outer boundary and the eight
/// bolt holes. Through the primitives as well it is `Generated` from
/// `Role::Box(Face(Z, Max))`, not `Modified` — the relation of a chain
/// that starts at a role is generation.
#[test]
fn the_plates_top_face_is_one_face_of_nine_loops() {
    let role = Role::Box(BoxPart::Face(Coord::Z, Side::Max));
    for variant in VARIANTS {
        let chain = chain_of(variant);
        let top = forward(face_from(&chain.steps["plate"].provenance, role));
        let cuts = composed(&chain, &CUTS);
        let pieces = faces(cuts.modified_from(top));
        assert_eq!(pieces.len(), 1, "{variant}: the top became {pieces:?}");
        let drilled = pieces[0];
        assert_eq!(
            chain.model.face(drilled).unwrap().loops().len(),
            9,
            "{variant}: one outer loop and eight holes"
        );
        assert!(
            cuts.origins(forward(drilled))
                .contains(&(Relation::Modified, Origin::Entity(top))),
            "{variant}: the drilled top is not a piece of the plate's top"
        );
        let all = whole(&chain);
        assert_eq!(
            faces(all.generated_from(role)),
            [drilled],
            "{variant}: the chain from the role does not reach the drilled top"
        );
        assert!(all.modified_from(role).is_empty(), "{variant}");
        // The four sides the tools never touched are kept, id and all.
        let body = chain.result().unwrap();
        for side in [
            BoxPart::Face(Coord::X, Side::Min),
            BoxPart::Face(Coord::X, Side::Max),
            BoxPart::Face(Coord::Y, Side::Min),
            BoxPart::Face(Coord::Y, Side::Max),
        ] {
            let face = forward(face_from(&chain.steps["plate"].provenance, Role::Box(side)));
            assert!(
                cuts.is_kept(face, &chain.model, body),
                "{variant}: {side:?} is not kept"
            );
        }
    }
}

/// One recipe under three parameter sets is one chain: the composed
/// record, entity ids and all, is the same in every variant.
#[test]
fn the_chain_is_the_same_in_every_variant() {
    let chains: Vec<Chain> = VARIANTS.iter().map(|v| chain_of(v)).collect();
    let first = whole(&chains[0]);
    let first_cuts = composed(&chains[0], &CUTS);
    for (variant, chain) in VARIANTS.iter().zip(&chains).skip(1) {
        assert_eq!(whole(chain), first, "{variant} chains differently");
        assert_eq!(
            composed(chain, &CUTS),
            first_cuts,
            "{variant}'s cuts chain differently"
        );
    }
}

/// [`Provenance::then`] is associative over the chain, so a consumer may
/// compose the seventeen records however it brackets them and get the
/// same answer: left fold, right fold, and the primitives composed apart
/// from the cuts all agree.
#[test]
fn the_chain_is_associative_whatever_the_bracketing() {
    for variant in VARIANTS {
        let chain = chain_of(variant);
        let names: Vec<&str> = PRIMITIVES.iter().chain(CUTS.iter()).copied().collect();
        let left = composed(&chain, &names);
        let right = names[..names.len() - 1].iter().rev().fold(
            chain.steps[*names.last().unwrap()].provenance.clone(),
            |p, name| chain.steps[*name].provenance.then(&p),
        );
        assert_eq!(right, left, "{variant}: right fold");
        let split = composed(&chain, &PRIMITIVES).then(&composed(&chain, &CUTS));
        assert_eq!(split, left, "{variant}: the bodies apart from the cuts");
    }
}

/// The chain accounts for the result: every entity of the body is an
/// output of the composed record or an untouched input kept through it,
/// and the counts are the oracle's for that variant.
#[test]
fn the_chain_accounts_for_every_entity_of_the_result() {
    let fixture = fixtures::load(&fixtures::corpus_root().join(FIXTURE)).unwrap();
    for variant in VARIANTS {
        let chain = chain_of(variant);
        let body = chain.result().unwrap();
        let closure = chain.model.closure(body).unwrap();
        let counts = fixture.expected.results[variant].counts;
        assert_eq!(closure.vertices.len(), counts.vertices, "{variant}");
        assert_eq!(closure.edges.len(), counts.edges, "{variant}");
        assert_eq!(closure.faces.len(), counts.faces, "{variant}");
        let all = whole(&chain);
        let outputs: BTreeSet<EntityId> = all.outputs().iter().map(|s| s.id).collect();
        let cuts = composed(&chain, &CUTS);
        let entities = closure
            .vertices
            .iter()
            .map(|v| EntityId::Vertex(*v))
            .chain(closure.edges.iter().map(|e| EntityId::Edge(*e)))
            .chain(closure.faces.iter().map(|f| EntityId::Face(*f)));
        for id in entities {
            let shape = Shape::new(id, Orientation::Forward);
            assert!(
                outputs.contains(&id) || cuts.is_kept(shape, &chain.model, body),
                "{variant}: {shape} is neither an output of the chain nor kept"
            );
        }
    }
}

/// The split-order fixtures and their variants, `default` first: a bar
/// through a box, the same bar through a frame (a tool face surviving
/// in two `Generated` pieces), a slab through a rod along its axis, and
/// the Steinmetz solid — the two closed walls each with a tie only the
/// (u, v) tiebreak orders.
const SPLIT_FIXTURES: [(&str, &[&str]); 4] = [
    (
        "provenance/split-bar-cut",
        &["default", "left", "right", "narrow"],
    ),
    (
        "provenance/split-frame-cut",
        &["default", "left", "right", "narrow"],
    ),
    (
        "provenance/split-cylinder-seam",
        &["default", "turned-back", "turned-on", "narrow"],
    ),
    (
        "provenance/split-cross-common",
        &["default", "larger", "longer", "turned"],
    ),
];

/// The step names of `fixture`'s recipe in recipe order, profile steps
/// left out, which is the order the records compose in.
fn step_names(fixture: &str) -> Vec<String> {
    let loaded = fixtures::load(&fixtures::corpus_root().join(fixture)).unwrap();
    loaded
        .recipe
        .steps
        .iter()
        .map(|s| s.name().to_string())
        .collect()
}

/// A piece's signature: for every face it shares an edge with in the
/// result, that face's origins through the whole recipe — roles, which
/// a parameter edit leaves alone. Two pieces of one origin with the same
/// signature are a tie, and the fixtures that hold one are posed so that
/// the tied pieces lie on either side of a coordinate plane through the
/// world origin: [`side_of`] tells them apart.
type Signature = BTreeSet<Vec<(Relation, Origin)>>;

/// A signature as text: each origin list joined by `&`, the lists by `|`.
fn describe(s: &Signature) -> String {
    let parts: Vec<String> = s
        .iter()
        .map(|o| {
            o.iter()
                .map(|(r, o)| format!("{r} {o}"))
                .collect::<Vec<_>>()
                .join(" & ")
        })
        .collect();
    format!("[{}]", parts.join(" | "))
}

/// Every face of `body` using each edge, from the faces' loops.
fn faces_by_edge(chain: &Chain) -> BTreeMap<EntityId, Vec<FaceId>> {
    let body = chain.result().unwrap();
    let mut out: BTreeMap<EntityId, Vec<FaceId>> = BTreeMap::new();
    for f in chain.model.faces(body).unwrap() {
        for l in chain.model.face(f.id).unwrap().loops() {
            for c in l.coedges() {
                let users = out.entry(EntityId::Edge(c.edge())).or_default();
                if !users.contains(&f.id) {
                    users.push(f.id);
                }
            }
        }
    }
    out
}

fn signature_of(
    chain: &Chain,
    whole: &Provenance,
    by_edge: &BTreeMap<EntityId, Vec<FaceId>>,
    face: FaceId,
) -> Signature {
    let mut out = Signature::new();
    for l in chain.model.face(face).unwrap().loops() {
        for c in l.coedges() {
            for &g in &by_edge[&EntityId::Edge(c.edge())] {
                if g != face {
                    out.insert(whole.origins(forward(g)));
                }
            }
        }
    }
    out
}

/// Which side of each coordinate plane of the origin face's own surface
/// frame the piece lies on — the frame a transform carries with the
/// face, so the answer is the pose's — read from the mean of the piece's
/// edge midpoints: `Less`, `Equal` (within the model's default
/// tolerance) or `Greater` per axis.
fn side_of(chain: &Chain, origin: FaceId, piece: FaceId) -> [Ordering; 3] {
    let m = &chain.model;
    let frame = *m
        .surface(m.face(origin).unwrap().surface())
        .unwrap()
        .frame()
        .expect("an analytic origin face");
    let mut sum = [0.0; 3];
    let mut n = 0.0;
    for l in m.face(piece).unwrap().loops() {
        for c in l.coedges() {
            let EdgeGeometry::Curve { curve, range } = m.edge(c.edge()).unwrap().geometry() else {
                continue;
            };
            let p = frame.to_local(m.curve(curve).unwrap().point(range.midpoint()));
            sum = [sum[0] + p.x, sum[1] + p.y, sum[2] + p.z];
            n += 1.0;
        }
    }
    let tol = m.precision().default_tolerance;
    sum.map(|s| {
        let mean = s / n;
        if mean.abs() <= tol {
            Ordering::Equal
        } else {
            mean.partial_cmp(&0.0).unwrap()
        }
    })
}

/// The ordered pieces of every origin the result step split into two or
/// more faces, `Modified` and `Generated` alike, each piece as its
/// signature and, for a piece that ties with another of the same origin,
/// its side.
fn split_order(fixture: &str, variant: &str) -> BTreeMap<(Relation, Origin), Vec<String>> {
    let chain = chain_of_fixture(fixture, variant);
    let names = step_names(fixture);
    let names: Vec<&str> = names.iter().map(String::as_str).collect();
    let whole = composed(&chain, &names);
    let p = &chain.steps[*names.last().unwrap()].provenance;
    let by_edge = faces_by_edge(&chain);
    let mut out = BTreeMap::new();
    for origin in p.origins_recorded() {
        for (relation, list) in [
            (Relation::Modified, p.modified_from(origin)),
            (Relation::Generated, p.generated_from(origin)),
        ] {
            let pieces = faces(list);
            if pieces.len() < 2 {
                continue;
            }
            let signatures: Vec<Signature> = pieces
                .iter()
                .map(|&f| signature_of(&chain, &whole, &by_edge, f))
                .collect();
            let described: Vec<String> = pieces
                .iter()
                .zip(&signatures)
                .map(|(&f, s)| {
                    let tied = signatures.iter().filter(|t| *t == s).count() > 1;
                    let sides = if tied {
                        let Origin::Entity(Shape {
                            id: EntityId::Face(origin_face),
                            ..
                        }) = origin
                        else {
                            panic!("{fixture}: a tie of pieces of {origin}, which is no face");
                        };
                        format!(" side {:?}", side_of(&chain, origin_face, f))
                    } else {
                        String::new()
                    };
                    format!("{}{sides}", describe(s))
                })
                .collect();
            out.insert((relation, origin), described);
        }
    }
    out
}

fn chain_of_fixture(fixture: &str, variant: &str) -> Chain {
    let dir = fixtures::corpus_root().join(fixture);
    corpus::chain(&dir, variant).unwrap_or_else(|e| panic!("{fixture} [{variant}]: {e}"))
}

/// Every split-order fixture splits something: at least one origin into
/// two or more faces. The bar cuts have no tie; the two fixtures on a
/// closed wall each hold one, the pieces on either side of a seam. The
/// frame cut is the one with a `Generated` list of two pieces.
#[test]
fn the_split_fixtures_split_faces_and_the_closed_walls_tie() {
    for (fixture, variants) in SPLIT_FIXTURES {
        let order = split_order(fixture, variants[0]);
        assert!(!order.is_empty(), "{fixture} splits nothing");
        let tied = order
            .values()
            .flatten()
            .any(|piece| piece.contains(" side "));
        let cylindrical = fixture.contains("cylinder") || fixture.contains("cross");
        assert_eq!(tied, cylindrical, "{fixture}: {order:#?}");
        let generated = order.keys().any(|(r, _)| *r == Relation::Generated);
        assert_eq!(
            generated,
            fixture == "provenance/split-frame-cut",
            "{fixture}: {order:#?}"
        );
    }
}

/// Piece `k` of every split origin has the same signature — the same
/// neighbours by role, and for a tie the same side — in every variant:
/// the order a consumer's `Split(k)` name relies on (ADR-0009).
#[test]
fn piece_k_of_every_split_origin_is_the_same_in_every_variant() {
    let mut failures = Vec::new();
    for (fixture, variants) in SPLIT_FIXTURES {
        let first = split_order(fixture, variants[0]);
        for variant in &variants[1..] {
            let order = split_order(fixture, variant);
            assert_eq!(
                order.keys().collect::<Vec<_>>(),
                first.keys().collect::<Vec<_>>(),
                "{fixture} [{variant}] splits different origins"
            );
            for ((relation, origin), pieces) in &order {
                let expected = &first[&(*relation, *origin)];
                assert_eq!(
                    pieces.len(),
                    expected.len(),
                    "{fixture} [{variant}]: {origin}"
                );
                for (k, (got, want)) in pieces.iter().zip(expected).enumerate() {
                    if got != want {
                        failures.push(format!(
                            "{fixture} [{variant}]: piece {k} of {origin} ({relation}) is {got}, the default's is {want}"
                        ));
                    }
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// The edge split-order fixture (ADR-0009, the edge rule): two notches
/// cut one after the other into the top-front edge of a box, the first
/// splitting that edge in two and the second splitting one of the halves,
/// so the two cuts composed list three pieces of one edge — a list no
/// single step makes, and the nesting [`Provenance::then`] guarantees.
const NOTCH: &str = "provenance/split-edge-notch";

/// Its parameter sets, `default` first: the notches slid along the edge
/// without swapping, and narrowed.
const NOTCH_VARIANTS: [&str; 4] = ["default", "slid", "apart", "narrow"];

/// The notch recipe's two cuts, in order.
const NOTCH_CUTS: [&str; 2] = ["cut0", "result"];

/// Every fixture the edge order is read on: the notch recipe and the four
/// face fixtures, whose tool faces generate section edges as well as
/// pieces — a face pair that meets in a curve the other operand's edges
/// cut generates several, and the frame cut has one.
fn edge_fixtures() -> Vec<(&'static str, &'static [&'static str])> {
    std::iter::once((NOTCH, &NOTCH_VARIANTS[..]))
        .chain(SPLIT_FIXTURES)
        .collect()
}

/// The edges among `shapes`, in order.
fn edge_ids(shapes: &[Shape]) -> Vec<EdgeId> {
    shapes
        .iter()
        .filter_map(|s| match s.id {
            EntityId::Edge(e) => Some(e),
            _ => None,
        })
        .collect()
}

/// An edge's signature: the origins, through the whole recipe, of its two
/// end vertices — roles, which a parameter edit leaves alone — as a set,
/// so an edge reads the same whichever way round it was built. A closed
/// edge's two ends are one vertex, and its signature holds one entry.
fn edge_signature(chain: &Chain, whole: &Provenance, edge: EdgeId) -> Signature {
    let e = chain.model.edge(edge).unwrap();
    [e.start(), e.end()]
        .into_iter()
        .map(|v| whole.origins(Shape::new(v, Orientation::Forward)))
        .collect()
}

/// The ordered edge outputs of every origin `p` records two or more of,
/// each as its signature: the order a consumer's `Split(k)` over edges
/// relies on (ADR-0009). An origin with one edge output has nothing to
/// order and is left out.
fn edge_order(
    chain: &Chain,
    whole: &Provenance,
    p: &Provenance,
) -> BTreeMap<(Relation, Origin), Vec<String>> {
    let mut out = BTreeMap::new();
    for origin in p.origins_recorded() {
        for (relation, list) in [
            (Relation::Modified, p.modified_from(origin)),
            (Relation::Generated, p.generated_from(origin)),
        ] {
            let edges = edge_ids(list);
            if edges.len() < 2 {
                continue;
            }
            let described: Vec<String> = edges
                .iter()
                .map(|&e| describe(&edge_signature(chain, whole, e)))
                .collect();
            out.insert((relation, origin), described);
        }
    }
    out
}

/// The two readings of one variant of an edge fixture: the result step's
/// own record, whose origins are the entities the last operation took,
/// and — for the notch recipe alone — its two cuts composed, whose
/// origins are the block's own edges.
fn edge_orders(fixture: &str, variant: &str) -> Vec<BTreeMap<(Relation, Origin), Vec<String>>> {
    let chain = chain_of_fixture(fixture, variant);
    let names = step_names(fixture);
    let names: Vec<&str> = names.iter().map(String::as_str).collect();
    let whole = composed(&chain, &names);
    let mut out = vec![edge_order(
        &chain,
        &whole,
        &chain.steps[*names.last().unwrap()].provenance,
    )];
    if fixture == NOTCH {
        out.push(edge_order(&chain, &whole, &composed(&chain, &NOTCH_CUTS)));
    }
    out
}

/// The two notches make one origin edge into three pieces, and no two of
/// them read alike — so the order the record lists them in is a claim
/// with content. The three come in split order, which here is not the id
/// order: the piece the first cut left untouched was built before the
/// two the second cut made from the other half, and it comes last
/// because it lies last along the edge's curve.
#[test]
fn the_two_notches_make_one_edge_into_three_pieces_the_ids_do_not_order() {
    let chain = chain_of_fixture(NOTCH, NOTCH_VARIANTS[0]);
    let cuts = composed(&chain, &NOTCH_CUTS);
    let split: Vec<(Origin, Vec<EdgeId>)> = cuts
        .origins_recorded()
        .map(|o| (o, edge_ids(cuts.modified_from(o))))
        .filter(|(_, pieces)| pieces.len() > 2)
        .collect();
    assert_eq!(split.len(), 1, "one origin edge in three pieces: {split:?}");
    let (origin, pieces) = &split[0];
    assert_eq!(pieces.len(), 3, "{origin}: {pieces:?}");
    let names = step_names(NOTCH);
    let names: Vec<&str> = names.iter().map(String::as_str).collect();
    let whole = composed(&chain, &names);
    let signatures: BTreeSet<Signature> = pieces
        .iter()
        .map(|&e| edge_signature(&chain, &whole, e))
        .collect();
    assert_eq!(signatures.len(), 3, "{origin}: two pieces read alike");
    let mut by_id = pieces.clone();
    by_id.sort();
    assert_ne!(
        &by_id, pieces,
        "the fixture no longer proves the split order is not the id order; \
         find a recipe that does before trusting it"
    );
}

/// Edge outputs of one origin that lie on the same curve ascend by their
/// range on it (ADR-0009): an operand edge's pieces along the edge's own
/// curve, a closed edge's from its range's start, and the section edges
/// one face pair generates along the section curve. Outputs on different
/// curves are not compared — a face origin pairs with several faces of
/// the other operand, and the pairs' own order is what separates them.
#[test]
fn an_origins_edges_ascend_along_each_curve_they_lie_on() {
    let mut failures = Vec::new();
    for (fixture, variants) in edge_fixtures() {
        for variant in variants {
            let chain = chain_of_fixture(fixture, variant);
            let names = step_names(fixture);
            let names: Vec<&str> = names.iter().map(String::as_str).collect();
            let mut records = vec![chain.steps[*names.last().unwrap()].provenance.clone()];
            if fixture == NOTCH {
                records.push(composed(&chain, &NOTCH_CUTS));
            }
            for p in &records {
                for origin in p.origins_recorded() {
                    for (relation, list) in [
                        (Relation::Modified, p.modified_from(origin)),
                        (Relation::Generated, p.generated_from(origin)),
                    ] {
                        let mut last: BTreeMap<_, f64> = BTreeMap::new();
                        for e in edge_ids(list) {
                            let EdgeGeometry::Curve { curve, range } =
                                chain.model.edge(e).unwrap().geometry()
                            else {
                                continue;
                            };
                            if let Some(&before) = last.get(&curve)
                                && range.lo() <= before
                            {
                                failures.push(format!(
                                    "{fixture} [{variant}]: {relation} {origin} lists {e:?} \
                                     at {} on {curve:?} after a piece at {before}",
                                    range.lo()
                                ));
                            }
                            last.insert(curve, range.lo());
                        }
                    }
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Piece `k` of every origin an operation records two or more edges for
/// has the same signature — the same end vertices by role — in every
/// variant: the order a consumer's `Split(k)` over edges relies on
/// (ADR-0009), through one step and through the two cuts composed.
#[test]
fn edge_k_of_every_origin_is_the_same_in_every_variant() {
    let mut failures = Vec::new();
    for (fixture, variants) in edge_fixtures() {
        let first = edge_orders(fixture, variants[0]);
        assert!(
            first.iter().any(|reading| !reading.is_empty()),
            "{fixture} records no origin with two edges"
        );
        for variant in &variants[1..] {
            let orders = edge_orders(fixture, variant);
            for (reading, expected) in orders.iter().zip(&first) {
                assert_eq!(
                    reading.keys().collect::<Vec<_>>(),
                    expected.keys().collect::<Vec<_>>(),
                    "{fixture} [{variant}] records different origins"
                );
                for ((relation, origin), edges) in reading {
                    let want = &expected[&(*relation, *origin)];
                    assert_eq!(edges.len(), want.len(), "{fixture} [{variant}]: {origin}");
                    for (k, (got, want)) in edges.iter().zip(want).enumerate() {
                        if got != want {
                            failures.push(format!(
                                "{fixture} [{variant}]: edge {k} of {origin} ({relation}) is \
                                 {got}, the default's is {want}"
                            ));
                        }
                    }
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// The consumer's body of `provenance/consumer-rebuild` (ADR-0028): the
/// L-prism the consumer built and keyed in namespace 2, bitten at its
/// concave corner and filleted along its far edge.
const CONSUMER_FIXTURE: &str = "provenance/consumer-rebuild";

/// Its variants, `default` first.
const CONSUMER_VARIANTS: [&str; 3] = ["default", "longer", "thicker"];

/// The consumer's key `key`, in the fixture's namespace.
fn consumer(key: u64) -> Role {
    Role::Consumer(ConsumerKey { namespace: 2, key })
}

/// The whole recipe of the consumer's body composed: the polyhedron, the
/// cylinder, the cut, the fillet.
fn consumer_chain(variant: &str) -> (Chain, Provenance) {
    let chain = chain_of_fixture(CONSUMER_FIXTURE, variant);
    let names = step_names(CONSUMER_FIXTURE);
    let names: Vec<&str> = names.iter().map(String::as_str).collect();
    let whole = composed(&chain, &names);
    (chain, whole)
}

/// The stability claim on a consumer's body: every chain ends at a role
/// — the consumer's key or the bite's part, never an entity — and the
/// composed record, every key's outputs in their split order and every
/// id, is the same in all three variants, so a name the consumer builds
/// from its own keys does not move when a dimension does.
#[test]
fn a_consumer_bodys_chain_ends_at_its_keys_in_every_variant() {
    let (_, first) = consumer_chain(CONSUMER_VARIANTS[0]);
    for origin in first.origins_recorded() {
        assert!(
            matches!(
                origin,
                Origin::Role(Role::Consumer(ConsumerKey { namespace: 2, .. }) | Role::Cylinder(_))
            ),
            "{origin} is an origin of the chain"
        );
    }
    for variant in &CONSUMER_VARIANTS[1..] {
        let (_, whole) = consumer_chain(variant);
        assert_eq!(whole, first, "{variant} chains differently");
    }
}

/// What the chain says, by the consumer's own keys: the concave edge up
/// the inner corner (points 3 and 9) is gone into the bite, both inner
/// faces (4 and 5) are still one face each, and the fillet's face is
/// generated from the far edge (points 1 and 7) it replaced.
#[test]
fn the_consumer_keys_name_what_the_bite_and_the_fillet_did() {
    for variant in CONSUMER_VARIANTS {
        let (chain, whole) = consumer_chain(variant);
        let body = chain.result().unwrap();
        let live = chain.model.faces(body).unwrap();
        assert!(
            whole.generated_from(consumer(edge_key(3, 9))).is_empty(),
            "{variant}: the concave edge survives"
        );
        for inner in [4, 5] {
            let found = faces(whole.generated_from(consumer(inner)));
            assert_eq!(found.len(), 1, "{variant}: inner face {inner}");
            assert!(live.iter().any(|f| f.id == found[0]), "{variant}");
        }
        let blend = faces(whole.generated_from(consumer(edge_key(1, 7))));
        assert_eq!(blend.len(), 1, "{variant}: the fillet's face");
        assert!(live.iter().any(|f| f.id == blend[0]), "{variant}");
    }
}
