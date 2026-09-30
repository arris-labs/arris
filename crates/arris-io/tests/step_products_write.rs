//! The product tree written (ADR-0033): Arris's own assembly reads back to
//! the tree it was written from — names, nesting, placements, colours, and
//! per instance the volume and centroid of the body composed with its
//! path — and Open CASCADE's XCAF reader reads it to the same names,
//! placements, colours and volumes.

use arris_debug::oracle::{self, OracleOccurrence};
use arris_debug::sample;
use arris_debug::unmetered::{mass_properties, step_read};
use arris_io::arris_check::arris_topo::Model;
use arris_io::arris_check::arris_topo::arris_math::nalgebra::UnitQuaternion;
use arris_io::arris_check::arris_topo::arris_math::{Control, Isometry, Point3, UnitVec3, Vec3};
use arris_io::step::{self, Occurrence, ProductTree, ReadOptions, Rgb, StepError, TreeError};

fn turn(axis: [f64; 3], degrees: f64, by: [f64; 3]) -> Isometry {
    let axis = UnitVec3::new_normalize(Vec3::new(axis[0], axis[1], axis[2]));
    Isometry::new(
        UnitQuaternion::from_axis_angle(&axis, degrees.to_radians()),
        Vec3::new(by[0], by[1], by[2]),
    )
}

fn occurrence(
    product: Option<u64>,
    name: &str,
    placement: Isometry,
    colour: Option<[f64; 3]>,
    solids: &[usize],
    children: Vec<Occurrence>,
) -> Occurrence {
    Occurrence {
        product,
        name: name.to_string(),
        placement: Ok(placement),
        colour: colour.map(Rgb),
        solids: solids.to_vec(),
        children,
    }
}

/// An assembly of a pin placed twice — once directly, once in a
/// sub-assembly, as one product — and a block in the sub-assembly.
fn tree() -> ProductTree {
    let pin = |placement| {
        occurrence(
            Some(1),
            "pin",
            placement,
            Some([0.25, 0.5, 0.75]),
            &[0],
            vec![],
        )
    };
    let sub = occurrence(
        Some(2),
        "sub",
        turn([0.0, 1.0, 0.0], 20.0, [0.0, 0.0, 30.0]),
        None,
        &[],
        vec![
            pin(turn([1.0, 0.0, 0.0], 90.0, [0.0, 20.0, 0.0])),
            occurrence(
                None,
                "block 'b' \\ é",
                turn([0.0, 1.0, 1.0], -45.0, [-7.0, -20.0, 5.0]),
                None,
                &[1],
                vec![],
            ),
        ],
    );
    let root = occurrence(
        Some(3),
        "assembly",
        Isometry::identity(),
        None,
        &[],
        vec![pin(turn([0.0, 0.0, 1.0], 30.0, [10.0, 0.0, 0.0])), sub],
    );
    ProductTree {
        roots: vec![root],
        faces: vec![],
    }
}

fn bodies(m: &mut Model) -> Vec<arris_io::arris_check::arris_topo::Body> {
    vec![
        sample::cylinder(m, 4.0, 12.0).unwrap(),
        sample::cuboid(m, Point3::new(1.0, 2.0, 3.0), Point3::new(6.0, 9.0, 13.0)).unwrap(),
    ]
}

fn matrix(p: &Isometry) -> [f64; 12] {
    let r = p.rotation().to_rotation_matrix();
    let t = p.translation();
    let mut out = [0.0; 12];
    for row in 0..3 {
        for col in 0..3 {
            out[4 * row + col] = r[(row, col)];
        }
        out[4 * row + 3] = t[row];
    }
    out
}

/// `read` is `written`, placements to `within`, and each solid of `read`
/// the written body's volume and centroid at the path's composition.
fn same(
    read: &Occurrence,
    written: &Occurrence,
    above: &Isometry,
    m: &Model,
    back: &step::Read,
    prototypes: &[(f64, Point3)],
    within: f64,
) {
    assert_eq!(read.name, written.name);
    assert_eq!(read.colour, written.colour, "{}", read.name);
    let composed = written.placement.as_ref().unwrap().then(above);
    assert_eq!(read.solids.len(), written.solids.len(), "{}", read.name);
    for (&i, &w) in read.solids.iter().zip(&written.solids) {
        let body = back.solids[i].result.as_ref().unwrap().body;
        let mass = mass_properties(m, body).unwrap();
        let (volume, centroid) = prototypes[w];
        assert!(
            (mass.volume - volume).abs() <= 1e-9 * volume,
            "{}",
            read.name
        );
        let want = composed.apply(centroid);
        assert!(
            (mass.centroid - want).norm() <= within,
            "{}: {} vs {want}",
            read.name,
            mass.centroid
        );
    }
    assert_eq!(read.children.len(), written.children.len(), "{}", read.name);
    for (r, w) in read.children.iter().zip(&written.children) {
        let g = r.placement.as_ref().unwrap();
        let want = w.placement.as_ref().unwrap();
        for (a, b) in matrix(g).iter().zip(matrix(want)) {
            assert!((a - b).abs() <= within, "{}: {g:?} vs {want:?}", r.name);
        }
        same(r, w, &composed, m, back, prototypes, within);
    }
}

#[test]
fn an_assembly_written_reads_back_to_its_tree() {
    let mut m = Model::default();
    let bodies = bodies(&mut m);
    let tree = tree();
    let text = step::write_products(&m, &bodies, &tree, &Control::NONE).unwrap();
    assert_eq!(text.matches("PRODUCT('pin'").count(), 1, "a product once");
    assert_eq!(text.matches("NEXT_ASSEMBLY_USAGE_OCCURRENCE(").count(), 4);
    let prototypes: Vec<(f64, Point3)> = bodies
        .iter()
        .map(|&b| {
            let mass = mass_properties(&m, b).unwrap();
            (mass.volume, mass.centroid)
        })
        .collect();
    let mut back_model = Model::default();
    let back = step_read(&mut back_model, &text, &ReadOptions::default()).unwrap();
    assert_eq!(back.solids.len(), 3);
    assert_eq!(back.products.roots.len(), 1);
    same(
        &back.products.roots[0],
        &tree.roots[0],
        &Isometry::identity(),
        &back_model,
        &back,
        &prototypes,
        1e-9,
    );
    assert_eq!(
        text,
        step::write_products(&m, &bodies, &tree, &Control::NONE).unwrap()
    );
}

/// `found` (Open CASCADE's) is `written`: names, nesting, placements,
/// part colours.
fn same_as_occt(found: &OracleOccurrence, written: &Occurrence, root: bool) {
    assert_eq!(found.name, written.name);
    match (&found.colour, written.colour) {
        (None, None) => {}
        (Some(g), Some(Rgb(w))) => {
            for (a, b) in g.iter().zip(w) {
                assert!((a - b).abs() < 1e-7, "{}: {g:?} vs {w:?}", found.name);
            }
        }
        (g, w) => panic!("{}: colour {g:?} vs {w:?}", found.name),
    }
    if !root {
        let want = matrix(written.placement.as_ref().unwrap());
        let got = found.placement.unwrap();
        for (a, b) in got.iter().zip(want) {
            assert!((a - b).abs() < 1e-9, "{}: {got:?} vs {want:?}", found.name);
        }
    }
    assert_eq!(
        found.children.len(),
        written.children.len(),
        "{}",
        found.name
    );
    for (f, w) in found.children.iter().zip(&written.children) {
        same_as_occt(f, w, false);
    }
}

#[test]
fn open_cascade_reads_the_written_assembly_to_the_same_structure() {
    let mut m = Model::default();
    let bodies = bodies(&mut m);
    let tree = tree();
    let text = step::write_products(&m, &bodies, &tree, &Control::NONE).unwrap();
    let occt = oracle::occt_read_assembly(&text, "arris-products-write").unwrap();
    assert_eq!(occt.roots.len(), 1, "{:#?}", occt.roots);
    same_as_occt(&occt.roots[0], &tree.roots[0], true);
    // Every instance Arris reads is one Open CASCADE measures: the same
    // volume and centroid, in some order.
    let mut back_model = Model::default();
    let back = step_read(&mut back_model, &text, &ReadOptions::default()).unwrap();
    assert_eq!(back.solids.len(), occt.instances.len());
    let mut unmatched = occt.instances.clone();
    for solid in &back.solids {
        let mass = mass_properties(&back_model, solid.result.as_ref().unwrap().body).unwrap();
        let at = unmatched
            .iter()
            .position(|o| {
                (o.volume - mass.volume).abs() <= 1e-9 * o.volume
                    && (mass.centroid - Point3::new(o.centroid[0], o.centroid[1], o.centroid[2]))
                        .norm()
                        <= 1e-7
            })
            .unwrap_or_else(|| panic!("no instance at {} in {unmatched:?}", mass.centroid));
        unmatched.remove(at);
    }
}

#[test]
fn a_tree_that_cannot_be_written_is_a_typed_error() {
    let mut m = Model::default();
    let bodies = bodies(&mut m);
    let write = |tree: &ProductTree| step::write_products(&m, &bodies, tree, &Control::NONE);
    assert_eq!(write(&ProductTree::default()), Err(StepError::NoBodies));
    let part = |solids: &[usize], product| {
        occurrence(product, "p", Isometry::identity(), None, solids, vec![])
    };
    let root = |children| occurrence(None, "r", Isometry::identity(), None, &[], children);
    let tree = |children| ProductTree {
        roots: vec![root(children)],
        faces: vec![],
    };
    assert!(matches!(
        write(&tree(vec![part(&[5], None)])),
        Err(StepError::Tree(TreeError::SolidOutOfRange {
            index: 5,
            bodies: 2,
            ..
        }))
    ));
    assert_eq!(
        write(&tree(vec![part(&[0], None), part(&[0], None)])),
        Err(StepError::Tree(TreeError::SolidShared { index: 0 }))
    );
    assert_eq!(
        write(&tree(vec![part(&[0], Some(7)), part(&[1], Some(7))])),
        Err(StepError::Tree(TreeError::ProductMismatch { product: 7 }))
    );
    let mut bad = part(&[0], None);
    bad.placement = Ok(Isometry::from_translation(Vec3::new(f64::NAN, 0.0, 0.0)));
    assert!(matches!(
        write(&tree(vec![bad])),
        Err(StepError::Tree(TreeError::Placement { .. }))
    ));
    let mut refused = part(&[0], None);
    refused.placement = Err(step::Refusal::NoLengthUnit { context: 1 });
    assert!(matches!(
        write(&tree(vec![refused])),
        Err(StepError::Tree(TreeError::Placement { .. }))
    ));
    let mut pale = part(&[0], None);
    pale.colour = Some(Rgb([2.0, 0.0, 0.0]));
    assert!(matches!(
        write(&tree(vec![pale])),
        Err(StepError::Tree(TreeError::Colour { .. }))
    ));
    let inner = occurrence(
        Some(9),
        "a",
        Isometry::identity(),
        None,
        &[],
        vec![part(&[0], Some(9))],
    );
    assert_eq!(
        write(&tree(vec![inner])),
        Err(StepError::Tree(TreeError::Cycle { product: 9 }))
    );
}

/// A face the tree colours is a `STYLED_ITEM` of its `ADVANCED_FACE`, and
/// reads back as the same colour on that face of each instance of the
/// solid.
#[test]
fn a_coloured_face_reads_back_on_each_instance() {
    let mut m = Model::default();
    let bodies = bodies(&mut m);
    let face = m.faces(bodies[0]).unwrap()[0].id;
    let mut tree = tree();
    tree.faces.push(step::FaceColour {
        solid: 0,
        face,
        colour: Rgb([1.0, 0.0, 0.5]),
    });
    let text = step::write_products(&m, &bodies, &tree, &Control::NONE).unwrap();
    let mut back_model = Model::default();
    let back = step_read(&mut back_model, &text, &ReadOptions::default()).unwrap();
    let faces = &back.products.faces;
    assert_eq!(faces.len(), 2, "{faces:?}: the pin's two instances");
    for f in faces {
        assert_eq!(f.colour, Rgb([1.0, 0.0, 0.5]));
        assert!(back.solids[f.solid].result.is_ok());
    }
    let mut stranger = tree.clone();
    stranger.faces[0].solid = 1;
    assert!(matches!(
        step::write_products(&m, &bodies, &stranger, &Control::NONE),
        Err(StepError::Tree(TreeError::Face { solid: 1, .. }))
    ));
}
