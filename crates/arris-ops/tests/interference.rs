//! `query::interference` on named poses (ADR-0055): the verdict, the
//! shared volume and the containment flags against closed forms. The
//! oracle's comparison is the `query/interference-*` corpus.

use arris_debug::sample;
use arris_debug::unmetered::{primitive_box, primitive_cylinder};
use arris_math::{Axis, Control, Point3};
use arris_ops::OpError;
use arris_ops::query::{Contact, Interference, interference, interference_many};
use arris_topo::{Body, Model};
use core::f64::consts::PI;

const EXACT: f64 = 1e-9;

fn cube(m: &mut Model, lo: [f64; 3], hi: [f64; 3]) -> Body {
    primitive_box(
        m,
        Point3::new(lo[0], lo[1], lo[2]),
        Point3::new(hi[0], hi[1], hi[2]),
    )
    .unwrap()
    .0
}

fn of(m: &Model, a: Body, b: Body) -> Interference {
    interference(m, a, b, &Control::NONE).unwrap()
}

fn overlap(i: Interference) -> (f64, bool, bool) {
    let Interference::Overlapping {
        volume,
        a_in_b,
        b_in_a,
    } = i
    else {
        panic!("expected overlapping, got {i:?}")
    };
    (volume, a_in_b, b_in_a)
}

#[test]
fn boxes_apart_report_the_distance() {
    let mut m = Model::default();
    let a = cube(&mut m, [0.0; 3], [1.0; 3]);
    let b = cube(&mut m, [3.0, 0.0, 0.0], [4.0, 1.0, 1.0]);
    for (x, y) in [(a, b), (b, a)] {
        let Interference::Apart(d) = of(&m, x, y) else {
            panic!("apart")
        };
        assert!((d.distance - 2.0).abs() < EXACT);
    }
}

#[test]
fn boxes_sharing_a_face_touch() {
    let mut m = Model::default();
    let a = cube(&mut m, [0.0; 3], [1.0; 3]);
    let b = cube(&mut m, [1.0, 0.0, 0.0], [2.0, 1.0, 1.0]);
    for (x, y) in [(a, b), (b, a)] {
        let Interference::Touching { at, .. } = of(&m, x, y) else {
            panic!("touching")
        };
        assert!((at.x - 1.0).abs() < EXACT);
    }
}

#[test]
fn boxes_on_an_edge_or_a_corner_touch_without_volume() {
    let mut m = Model::default();
    let a = cube(&mut m, [0.0; 3], [1.0; 3]);
    let edge = cube(&mut m, [1.0, 1.0, 0.0], [2.0, 2.0, 1.0]);
    let corner = cube(&mut m, [1.0; 3], [2.0; 3]);
    for other in [edge, corner] {
        for (x, y) in [(a, other), (other, a)] {
            let Interference::Touching { at, .. } = of(&m, x, y) else {
                panic!("touching")
            };
            assert!((at.x - 1.0).abs() < EXACT && (at.y - 1.0).abs() < EXACT);
        }
    }
}

#[test]
fn crossing_boxes_overlap_by_their_common() {
    let mut m = Model::default();
    let a = cube(&mut m, [0.0; 3], [2.0; 3]);
    let b = cube(&mut m, [1.0; 3], [3.0; 3]);
    for (x, y) in [(a, b), (b, a)] {
        let (volume, a_in_b, b_in_a) = overlap(of(&m, x, y));
        assert!((volume - 1.0).abs() < EXACT && !a_in_b && !b_in_a);
    }
}

#[test]
fn a_solid_inside_another_is_contained_one_way() {
    let mut m = Model::default();
    let outer = cube(&mut m, [0.0; 3], [4.0; 3]);
    let inner = cube(&mut m, [1.0; 3], [2.0; 3]);
    let (volume, a_in_b, b_in_a) = overlap(of(&m, outer, inner));
    assert!((volume - 1.0).abs() < EXACT && !a_in_b && b_in_a);
    let (volume, a_in_b, b_in_a) = overlap(of(&m, inner, outer));
    assert!((volume - 1.0).abs() < EXACT && a_in_b && !b_in_a);
    // Flush with a corner of the outer one: still inside it.
    let corner = cube(&mut m, [0.0; 3], [1.0; 3]);
    let (_, a_in_b, b_in_a) = overlap(of(&m, outer, corner));
    assert!(!a_in_b && b_in_a);
}

#[test]
fn equal_solids_are_each_inside_the_other() {
    let mut m = Model::default();
    let a = cube(&mut m, [0.0; 3], [2.0; 3]);
    let b = cube(&mut m, [0.0; 3], [2.0; 3]);
    let (volume, a_in_b, b_in_a) = overlap(of(&m, a, b));
    assert!((volume - 8.0).abs() < EXACT && a_in_b && b_in_a);
}

#[test]
fn a_ball_inside_a_block_has_no_vertex_to_say_so() {
    let mut m = Model::default();
    let block = cube(&mut m, [-5.0; 3], [5.0; 3]);
    let ball = sample::sphere(&mut m, Point3::origin(), 2.0).unwrap();
    let (volume, a_in_b, b_in_a) = overlap(of(&m, block, ball));
    assert!((volume - 4.0 / 3.0 * PI * 8.0).abs() < EXACT * 100.0);
    assert!(!a_in_b && b_in_a);
    let (_, a_in_b, b_in_a) = overlap(of(&m, ball, block));
    assert!(a_in_b && !b_in_a);
}

#[test]
fn a_pin_in_a_hole_is_apart_with_clearance_and_overlapping_pressed() {
    let mut m = Model::default();
    let plate = cube(&mut m, [0.0, 0.0, 0.0], [40.0, 30.0, 10.0]);
    let hole = primitive_cylinder(&mut m, Axis::z_at(Point3::new(20.0, 15.0, -1.0)), 4.0, 12.0)
        .unwrap()
        .0;
    let (drilled, _) = arris_debug::unmetered::cut(&mut m, plate, hole).unwrap();
    let loose = primitive_cylinder(&mut m, Axis::z_at(Point3::new(20.0, 15.0, 0.0)), 3.5, 10.0)
        .unwrap()
        .0;
    let Interference::Apart(d) = of(&m, drilled, loose) else {
        panic!("apart")
    };
    assert!((d.distance - 0.5).abs() < EXACT);
    let pressed = primitive_cylinder(&mut m, Axis::z_at(Point3::new(20.0, 15.0, 0.0)), 4.5, 10.0)
        .unwrap()
        .0;
    let (volume, a_in_b, b_in_a) = overlap(of(&m, drilled, pressed));
    // The ring between radii 4 and 4.5, through the plate's 10.
    assert!((volume - PI * (4.5 * 4.5 - 16.0) * 10.0).abs() < EXACT * 10.0);
    assert!(!a_in_b && !b_in_a);
}

fn contact(i: Interference) -> Contact {
    let Interference::Touching { contact, .. } = i else {
        panic!("expected touching, got {i:?}")
    };
    contact
}

fn both_ways(m: &Model, a: Body, b: Body) -> Contact {
    let c = contact(of(m, a, b));
    assert_eq!(c, contact(of(m, b, a)));
    c
}

#[test]
fn boxes_meeting_on_a_face_an_edge_or_a_corner_name_the_dimension() {
    let mut m = Model::default();
    let a = cube(&mut m, [0.0; 3], [1.0; 3]);
    let face = cube(&mut m, [1.0, 0.0, 0.0], [2.0, 1.0, 1.0]);
    let part_face = cube(&mut m, [1.0, 0.5, 0.5], [2.0, 2.0, 2.0]);
    let edge = cube(&mut m, [1.0, 1.0, 0.0], [2.0, 2.0, 1.0]);
    let corner = cube(&mut m, [1.0; 3], [2.0; 3]);
    assert_eq!(both_ways(&m, a, face), Contact::Face);
    assert_eq!(both_ways(&m, a, part_face), Contact::Face);
    assert_eq!(both_ways(&m, a, edge), Contact::Curve);
    assert_eq!(both_ways(&m, a, corner), Contact::Point);
}

#[test]
fn two_balls_meet_at_a_point() {
    let mut m = Model::default();
    let one = sample::sphere(&mut m, Point3::origin(), 1.0).unwrap();
    let two = sample::sphere(&mut m, Point3::new(0.0, 2.0, 0.0), 1.0).unwrap();
    assert_eq!(both_ways(&m, one, two), Contact::Point);
}

#[test]
fn cylinders_side_by_side_and_a_box_on_a_ruling_meet_along_a_curve() {
    let mut m = Model::default();
    let one = primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 1.0, 4.0)
        .unwrap()
        .0;
    let two = primitive_cylinder(&mut m, Axis::z_at(Point3::new(2.0, 0.0, 0.0)), 1.0, 4.0)
        .unwrap()
        .0;
    assert_eq!(both_ways(&m, one, two), Contact::Curve);
    let slab = cube(&mut m, [1.0, -3.0, 0.0], [3.0, 3.0, 4.0]);
    assert_eq!(both_ways(&m, one, slab), Contact::Curve);
}

#[test]
fn a_body_against_itself_is_inside_itself() {
    let mut m = Model::default();
    let a = cube(&mut m, [0.0; 3], [2.0; 3]);
    let (volume, a_in_b, b_in_a) = overlap(of(&m, a, a));
    assert!((volume - 8.0).abs() < EXACT && a_in_b && b_in_a);
}

#[test]
fn an_assembly_reports_each_pair_that_is_not_apart_in_pair_order() {
    let mut m = Model::default();
    let base = cube(&mut m, [0.0; 3], [10.0; 3]);
    let far = cube(&mut m, [40.0, 0.0, 0.0], [50.0, 10.0, 10.0]);
    let crossing = cube(&mut m, [5.0; 3], [15.0; 3]);
    let flush = cube(&mut m, [10.0, 0.0, 0.0], [20.0, 10.0, 10.0]);
    let bodies = [crossing, far, base, flush];
    let clashes = interference_many(&m, &bodies, &Control::NONE).unwrap();
    let pairs: Vec<[usize; 2]> = clashes.iter().map(|c| c.operands).collect();
    // crossing/base overlap, crossing/flush overlap (5..15 meets 10..20),
    // base/flush touch; `far` meets nobody.
    assert_eq!(pairs, [[0, 2], [0, 3], [2, 3]]);
    for c in &clashes {
        let [i, j] = c.operands;
        assert_eq!(c.verdict, of(&m, bodies[i], bodies[j]));
    }
    assert!(matches!(
        clashes[2].verdict,
        Interference::Touching {
            contact: Contact::Face,
            ..
        }
    ));
}

#[test]
fn an_assembly_with_nothing_in_it_has_no_clash() {
    let m = Model::default();
    assert!(
        interference_many(&m, &[], &Control::NONE)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn a_poll_stops_the_screen() {
    let mut m = Model::default();
    let a = cube(&mut m, [0.0; 3], [2.0; 3]);
    let b = cube(&mut m, [1.0; 3], [3.0; 3]);
    let stop = || true;
    let r = interference_many(&m, &[a, b], &Control::poll(&stop));
    assert!(matches!(r, Err(OpError::Interrupted(_))), "{r:?}");
    // A budget of no steps stops at the first candidate pair too, and
    // a pair of boxes that are apart is not a candidate.
    let far = cube(&mut m, [40.0; 3], [41.0; 3]);
    let r = interference_many(&m, &[a, far], &Control::NONE.with_budget(0));
    assert!(r.unwrap().is_empty());
}
