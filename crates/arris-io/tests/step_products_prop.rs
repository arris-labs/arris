//! The product tree round trip as a property (ADR-0033): a random tree
//! over random quadric solids — depth at most three, fan-out at most three,
//! random rigid placements, names from an alphabet with the quote, the
//! backslash and a non-ASCII letter, random colours, a product placed
//! twice where a node is copied — written by Arris reads back to the tree
//! it was written from, and per instance to the body's volume and centroid
//! composed with its path.

use arris_debug::prop::body::{QuadricSolid, quadric_solid};
use arris_debug::testing::fail;
use arris_debug::unmetered::{mass_properties, step_read};
use arris_debug::{prop, prop_shards};
use arris_io::arris_check::arris_topo::arris_math::{Control, Isometry};
use arris_io::arris_check::arris_topo::{Body, Model};
use arris_io::step::{self, Occurrence, ProductTree, Read, ReadOptions, Rgb};
use proptest::prelude::*;

/// What one node of the random tree is, before it is a product.
#[derive(Debug, Clone)]
struct Node {
    name: String,
    pose: Isometry,
    /// The second placement of a node that is placed twice.
    again: Option<Isometry>,
    colour: Option<[f64; 3]>,
    solid: Option<QuadricSolid>,
    children: Vec<Node>,
}

fn name() -> impl Strategy<Value = String> {
    proptest::collection::vec(
        prop_oneof![
            Just('a'),
            Just('Z'),
            Just('7'),
            Just(' '),
            Just('\''),
            Just('\\'),
            Just('é'),
            Just(';'),
            Just('('),
        ],
        0..6,
    )
    .prop_map(|cs| cs.into_iter().collect())
}

fn colour() -> impl Strategy<Value = Option<[f64; 3]>> {
    proptest::option::of([0.0f64..=1.0, 0.0f64..=1.0, 0.0f64..=1.0])
}

fn node() -> impl Strategy<Value = Node> {
    let pose = || prop::pose_in(30.0);
    let leaf = (
        name(),
        pose(),
        proptest::option::of(pose()),
        colour(),
        quadric_solid(),
    )
        .prop_map(|(name, pose, again, colour, solid)| Node {
            name,
            pose,
            again,
            colour,
            solid: Some(solid),
            children: vec![],
        });
    leaf.prop_recursive(3, 12, 3, move |inner| {
        (
            name(),
            pose(),
            proptest::option::of(pose()),
            colour(),
            proptest::option::of(quadric_solid()),
            proptest::collection::vec(inner, 1..=3),
        )
            .prop_map(|(name, pose, again, colour, solid, children)| Node {
                name,
                pose,
                again,
                colour,
                solid,
                children,
            })
    })
}

/// The occurrence of `node`, its bodies pushed on `bodies`; a node with a
/// second placement is placed a second time as the same product while the
/// parent has room (three children at most).
fn occurrence(
    node: &Node,
    key: &mut u64,
    m: &mut Model,
    bodies: &mut Vec<Body>,
) -> Result<Occurrence, TestCaseError> {
    *key += 1;
    let mine = *key;
    let mut solids = vec![];
    if let Some(solid) = node.solid {
        solids.push(bodies.len());
        bodies.push(solid.build(m).map_err(fail)?);
    }
    let mut children = Vec::new();
    for child in &node.children {
        let made = occurrence(child, key, m, bodies)?;
        children.push(made.clone());
        if let Some(pose) = child
            .again
            .filter(|_| node.children.len() + children.len() - 1 < 3)
        {
            children.push(Occurrence {
                placement: Ok(pose),
                ..made
            });
        }
    }
    Ok(Occurrence {
        product: Some(mine),
        name: node.name.clone(),
        placement: Ok(node.pose),
        // A colour is its own solids': none, none.
        colour: node.colour.filter(|_| !solids.is_empty()).map(Rgb),
        solids,
        children,
    })
}

/// `read` is `written`, placements to rounding, colours exactly, and each
/// solid's body the prototype's at the composition of the path.
fn same(
    read: &Occurrence,
    written: &Occurrence,
    above: &Isometry,
    at: &Path,
    back: (&Model, &Read),
) -> Result<(), TestCaseError> {
    prop_assert_eq!(&read.name, &written.name);
    prop_assert_eq!(read.colour, written.colour, "{}", read.name);
    let composed = written.placement.as_ref().unwrap().then(above);
    prop_assert_eq!(read.solids.len(), written.solids.len(), "{}", read.name);
    for (&i, &w) in read.solids.iter().zip(&written.solids) {
        let body = back.1.solids[i].result.as_ref().map_err(fail)?.body;
        let mass = mass_properties(back.0, body).map_err(fail)?;
        let (volume, centroid) = at.prototypes[w];
        prop_assert!(
            (mass.volume - volume).abs() <= 1e-9 * volume,
            "{}: {} vs {volume}",
            read.name,
            mass.volume
        );
        let want = composed.apply(centroid);
        prop_assert!(
            (mass.centroid - want).norm() <= 1e-6,
            "{}: {} vs {want}",
            read.name,
            mass.centroid
        );
    }
    prop_assert_eq!(read.children.len(), written.children.len(), "{}", read.name);
    for (r, w) in read.children.iter().zip(&written.children) {
        let (g, want) = (r.placement.as_ref().unwrap(), w.placement.as_ref().unwrap());
        let (a, b) = (
            g.translation() - want.translation(),
            g.rotation().angle_to(&want.rotation()),
        );
        prop_assert!(
            a.norm() <= 1e-9 && b <= 1e-9,
            "{}: {g:?} vs {want:?}",
            r.name
        );
        same(r, w, &composed, at, back)?;
    }
    Ok(())
}

/// The prototype bodies' volumes and centroids.
struct Path {
    prototypes: Vec<(f64, arris_io::arris_check::arris_topo::arris_math::Point3)>,
}

fn round_trip(root: &Node) -> Result<(), TestCaseError> {
    let mut m = Model::default();
    let mut bodies = Vec::new();
    let mut key = 0;
    let mut written = occurrence(root, &mut key, &mut m, &mut bodies)?;
    written.placement = Ok(Isometry::identity());
    let tree = ProductTree {
        roots: vec![written.clone()],
        faces: vec![],
    };
    let text = step::write_products(&m, &bodies, &tree, &Control::NONE).map_err(fail)?;
    prop_assert_eq!(
        &text,
        &step::write_products(&m, &bodies, &tree, &Control::NONE).map_err(fail)?,
        "deterministic"
    );
    let mut prototypes = Vec::new();
    for &b in &bodies {
        let mass = mass_properties(&m, b).map_err(fail)?;
        prototypes.push((mass.volume, mass.centroid));
    }
    let mut back_model = Model::default();
    let back = step_read(&mut back_model, &text, &ReadOptions::default()).map_err(fail)?;
    prop_assert_eq!(back.products.roots.len(), 1);
    same(
        &back.products.roots[0],
        &written,
        &Isometry::identity(),
        &Path { prototypes },
        (&back_model, &back),
    )
}

prop_shards! {
    /// A written product tree reads back to itself, instance by instance.
    a_written_tree_reads_back_to_itself [shard_0 shard_1 shard_2 shard_3] (root) = node() => {
        round_trip(&root)
    }
}
