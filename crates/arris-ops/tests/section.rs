//! `ops::section` in named poses with closed forms (ADR-0056): a box, a
//! cylinder across and along its axis, a tube, a sphere, a U-bracket, and
//! the flush cases that settle what a section is — the plane's points with
//! material on both sides, so a face of the body lying on the plane is no
//! section.

use core::f64::consts::PI;

use arris_check::{Level, check};
use arris_debug::dump_text;
use arris_debug::sample;
use arris_debug::unmetered::{cut, fuse, primitive_box, primitive_cylinder};
use arris_math::{Axis, Control, Frame, Point3, Vec3};
use arris_ops::{Section, section};
use arris_topo::entity::BodyKind;
use arris_topo::provenance::audit;
use arris_topo::{Body, EntityId, Model, Orientation, Origin, Shape};

const EPS: f64 = 1e-9;

fn plane(origin: Point3, z: Vec3) -> Frame {
    Frame::from_z(origin, z).unwrap()
}

fn block(m: &mut Model, min: [f64; 3], max: [f64; 3]) -> Body {
    primitive_box(
        m,
        Point3::new(min[0], min[1], min[2]),
        Point3::new(max[0], max[1], max[2]),
    )
    .unwrap()
    .0
}

/// The section, held to what every section owes: a `Sheet` clean at
/// `Full`, one shell per face, every face facing the frame's `z`, the body
/// as it was, and a record that audits with every entity of the sheet
/// given an origin on the body.
fn cut_by(m: &mut Model, body: Body, frame: &Frame) -> Option<Section> {
    let before = dump_text(m, body).unwrap();
    let s = section(m, body, frame, &Control::NONE).unwrap();
    assert_eq!(dump_text(m, body).unwrap(), before, "the body is untouched");
    let s = s?;
    assert_eq!(m.body(s.sheet.id).unwrap().kind(), BodyKind::Sheet);
    let report = check(m, s.sheet, Level::Full);
    assert!(report.is_ok(), "{report}");
    let faces = m.faces(s.sheet).unwrap();
    assert_eq!(m.shells(s.sheet).unwrap().len(), faces.len());
    for f in &faces {
        let face = m.face(f.id).unwrap();
        let surface = m.surface(face.surface()).unwrap();
        let e = surface.eval(0.0, 0.0);
        let n = e.du.cross(&e.dv).normalize() * f.orientation.sign();
        assert!((n - frame.z().into_inner()).norm() < EPS, "{n}");
        assert!(frame.to_local(e.point).z.abs() < EPS, "on the plane");
    }
    audit(m, &[], s.sheet, &s.provenance).unwrap();
    let c = m.closure(s.sheet).unwrap();
    for e in &c.edges {
        let y = Shape::new(*e, Orientation::Forward);
        let on: Vec<_> = s.provenance.origins(y);
        assert!(!on.is_empty(), "{y} has an origin");
        for (_, o) in on {
            let Origin::Entity(o) = o else {
                panic!("{y} from a role")
            };
            assert!(matches!(o.id, EntityId::Face(_)), "{y} from {o}");
        }
    }
    Some(s)
}

/// Faces, loops and edges of the sheet.
fn counts(m: &Model, sheet: Body) -> (usize, usize, usize) {
    let faces = m.faces(sheet).unwrap();
    let loops = faces
        .iter()
        .map(|f| m.face(f.id).unwrap().loops().len())
        .sum();
    (faces.len(), loops, m.edges(sheet).unwrap().len())
}

fn holds(s: &Section, area: f64, centroid: Point3) {
    assert!(
        (s.area - area).abs() < EPS * area.max(1.0),
        "area {} against {area}",
        s.area
    );
    assert!(
        (s.centroid - centroid).norm() < EPS * area.sqrt().max(1.0),
        "centroid {} against {centroid}",
        s.centroid
    );
}

#[test]
fn a_box_mid_height_is_a_rectangle_whose_edges_lie_on_the_walls() {
    let mut m = Model::default();
    let b = block(&mut m, [0.0, 0.0, 0.0], [10.0, 6.0, 4.0]);
    let s = cut_by(&mut m, b, &plane(Point3::new(0.0, 0.0, 1.5), Vec3::z())).unwrap();
    holds(&s, 60.0, Point3::new(5.0, 3.0, 1.5));
    assert_eq!(counts(&m, s.sheet), (1, 1, 4));
    // Each section edge on one wall, each section vertex on one upright.
    let c = m.closure(s.sheet).unwrap();
    for e in &c.edges {
        assert_eq!(
            s.provenance
                .origins(Shape::new(*e, Orientation::Forward))
                .len(),
            1
        );
    }
    for v in &c.vertices {
        let on = s.provenance.origins(Shape::new(*v, Orientation::Forward));
        assert_eq!(on.len(), 1, "{v}");
        assert!(matches!(on[0].1, Origin::Entity(o) if matches!(o.id, EntityId::Edge(_))));
    }
}

#[test]
fn a_box_across_a_corner_is_a_triangle() {
    let mut m = Model::default();
    let b = block(&mut m, [0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let s = cut_by(
        &mut m,
        b,
        &plane(Point3::new(1.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 1.0)),
    )
    .unwrap();
    // The equilateral triangle through (3, 0, 0), (0, 3, 0), (0, 0, 3).
    holds(&s, 4.5 * 3f64.sqrt(), Point3::new(1.0, 1.0, 1.0));
    assert_eq!(counts(&m, s.sheet), (1, 1, 3));
}

#[test]
fn a_cylinder_across_is_a_disc_and_along_its_axis_a_rectangle() {
    let mut m = Model::default();
    let (c, _) = primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 3.0, 10.0).unwrap();
    let s = cut_by(&mut m, c, &plane(Point3::new(0.0, 0.0, 4.0), Vec3::z())).unwrap();
    holds(&s, 9.0 * PI, Point3::new(0.0, 0.0, 4.0));
    assert_eq!(counts(&m, s.sheet).0, 1);
    // Through the axis and the seam (at the frame's `x`): two rulings and
    // two diameters of the caps.
    let s = cut_by(&mut m, c, &plane(Point3::origin(), Vec3::y())).unwrap();
    holds(&s, 60.0, Point3::new(0.0, 0.0, 5.0));
    assert_eq!(counts(&m, s.sheet), (1, 1, 4));
}

#[test]
fn a_tube_across_is_one_ring_of_two_loops() {
    let mut m = Model::default();
    let axis = Axis::z_at(Point3::origin());
    let (outer, _) = primitive_cylinder(&mut m, axis, 3.0, 10.0).unwrap();
    let (bore, _) =
        primitive_cylinder(&mut m, Axis::z_at(Point3::new(0.0, 0.0, -1.0)), 1.0, 12.0).unwrap();
    let (tube, _) = cut(&mut m, outer, bore).unwrap();
    let s = cut_by(&mut m, tube, &plane(Point3::new(0.0, 0.0, 4.0), Vec3::z())).unwrap();
    holds(&s, 8.0 * PI, Point3::new(0.0, 0.0, 4.0));
    let (faces, loops, _) = counts(&m, s.sheet);
    assert_eq!((faces, loops), (1, 2));
}

#[test]
fn a_sphere_off_centre_is_a_disc_of_the_chord() {
    let mut m = Model::default();
    let ball = sample::sphere(&mut m, Point3::new(1.0, -2.0, 0.5), 5.0).unwrap();
    let s = cut_by(&mut m, ball, &plane(Point3::new(0.0, 0.0, 3.5), Vec3::z())).unwrap();
    holds(&s, PI * (25.0 - 9.0), Point3::new(1.0, -2.0, 3.5));
    assert_eq!(counts(&m, s.sheet).0, 1);
}

#[test]
fn a_u_bracket_across_both_arms_is_two_faces() {
    let mut m = Model::default();
    let base = block(&mut m, [0.0, 0.0, 0.0], [10.0, 4.0, 10.0]);
    let slot = block(&mut m, [3.0, -1.0, 3.0], [7.0, 5.0, 11.0]);
    let (u, _) = cut(&mut m, base, slot).unwrap();
    let s = cut_by(&mut m, u, &plane(Point3::new(0.0, 0.0, 6.0), Vec3::z())).unwrap();
    holds(&s, 24.0, Point3::new(5.0, 2.0, 6.0));
    assert_eq!(counts(&m, s.sheet), (2, 2, 8));
}

#[test]
fn a_plane_that_misses_or_touches_is_none() {
    let mut m = Model::default();
    let b = block(&mut m, [0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    for frame in [
        plane(Point3::new(0.0, 0.0, 20.0), Vec3::z()),
        // Flush with the top, from either side.
        plane(Point3::new(0.0, 0.0, 10.0), Vec3::z()),
        plane(Point3::new(0.0, 0.0, 10.0), -Vec3::z()),
        // Through a corner, from either side. Along an edge only is the
        // booleans' fault `regression/box-edge-touching-split`.
        plane(Point3::new(10.0, 10.0, 10.0), Vec3::new(1.0, 1.0, 1.0)),
        plane(Point3::new(10.0, 10.0, 10.0), Vec3::new(-1.0, -1.0, -1.0)),
    ] {
        assert!(cut_by(&mut m, b, &frame).is_none(), "{frame:?}");
    }
    let (c, _) = primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 3.0, 10.0).unwrap();
    let corner = plane(Point3::new(2.9, 2.9, 0.0), Vec3::new(1.0, 1.0, 0.0));
    assert!(cut_by(&mut m, c, &corner).is_none());
}

/// A stepped block cut at its step: the step's face lies on the plane with
/// material below it only, so the section is the narrow part's footprint
/// alone — from either side of the plane.
#[test]
fn a_stepped_block_at_its_step_is_the_narrow_footprint() {
    let mut m = Model::default();
    let wide = block(&mut m, [0.0, 0.0, 0.0], [10.0, 10.0, 5.0]);
    let narrow = block(&mut m, [0.0, 0.0, 5.0], [4.0, 10.0, 10.0]);
    let (stepped, _) = fuse(&mut m, wide, narrow).unwrap();
    for z in [Vec3::z(), -Vec3::z()] {
        let s = cut_by(&mut m, stepped, &plane(Point3::new(0.0, 0.0, 5.0), z)).unwrap();
        holds(&s, 40.0, Point3::new(2.0, 5.0, 5.0));
        assert_eq!(counts(&m, s.sheet), (1, 1, 4), "{z}");
    }
}

/// A column under a slab, cut at the slab's underside: the underside's
/// overhangs lie on the plane with material above them only, so the
/// section is the column's footprint alone — from either side.
#[test]
fn an_overhang_on_the_plane_is_no_section() {
    let mut m = Model::default();
    let column = block(&mut m, [4.0, 0.0, 0.0], [6.0, 10.0, 6.0]);
    let slab = block(&mut m, [0.0, 0.0, 5.0], [10.0, 10.0, 10.0]);
    let (tee, _) = fuse(&mut m, column, slab).unwrap();
    for z in [Vec3::z(), -Vec3::z()] {
        let s = cut_by(&mut m, tee, &plane(Point3::new(0.0, 0.0, 5.0), z)).unwrap();
        holds(&s, 20.0, Point3::new(5.0, 5.0, 5.0));
        assert_eq!(counts(&m, s.sheet), (1, 1, 4), "{z}");
    }
}

/// A sheet is no solid to cut.
#[test]
fn a_sheet_is_refused_as_not_solid() {
    let mut m = Model::default();
    let b = block(&mut m, [0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let at = plane(Point3::new(0.0, 0.0, 5.0), Vec3::z());
    let s = cut_by(&mut m, b, &at).unwrap();
    match section(&mut m, s.sheet, &at, &Control::NONE) {
        Err(arris_ops::OpError::Degenerate { entities, reason }) => {
            assert_eq!(
                reason,
                arris_ops::Reason::Input(arris_ops::InputReason::NotSolid)
            );
            assert_eq!(entities, [Shape::new(s.sheet.id, s.sheet.orientation)]);
        }
        other => panic!("{other:?}"),
    }
}

/// The same input gives the same sheet, ids and record included, and the
/// model holds nothing of the half-space afterwards: the next body made is
/// the one a model that never cut would make after importing the sheet.
#[test]
fn two_runs_give_the_same_ids() {
    let run = || {
        let mut m = Model::default();
        let base = block(&mut m, [0.0, 0.0, 0.0], [10.0, 4.0, 10.0]);
        let slot = block(&mut m, [3.0, -1.0, 3.0], [7.0, 5.0, 11.0]);
        let (u, _) = cut(&mut m, base, slot).unwrap();
        let at = plane(Point3::new(0.5, 0.0, 6.0), Vec3::new(0.1, 0.2, 1.0));
        let s = cut_by(&mut m, u, &at).unwrap();
        let next = block(&mut m, [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        (dump_text(&m, s.sheet).unwrap(), s, next)
    };
    let (a, sa, na) = run();
    let (b, sb, nb) = run();
    assert_eq!(a, b);
    assert_eq!(sa, sb);
    assert_eq!(na, nb);
    // No body of the scratch run reached the model: the sheet is the body
    // after the bracket, the next box the one after the sheet.
    assert_eq!(sa.sheet.id.index(), 3);
    assert_eq!(na.id.index(), 4);
}
