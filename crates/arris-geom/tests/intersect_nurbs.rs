//! `intersect_surfaces` over a NURBS surface and an analytic one, in both
//! orders (ADR-0061): the exact twin of a cylinder met by each of the six
//! kinds, every fitted point on both surfaces within the tolerance and the
//! argument order not reaching the result, a NURBS twin of a pole met at
//! the pole as a point, a meridian met beside its pole, a saddle a hair
//! past the domain's edge, and two NURBS surfaces still `Unsupported`.
//! Walked blends from the corpus's recipes against the faces beside them
//! (plans/walked-saddle-cell): a saddle at a corner of the walked skin, and
//! a section across the skin's arc knot.
//! `tests/oracle.rs` holds the same section to Open CASCADE's.

use arris_debug::testing::tol;
use arris_debug::{corpus, fixtures};
use core::f64::consts::{FRAC_PI_2, PI};

use arris_geom::trace_section;
use arris_geom::{
    Curve, GeomError, MeetKind, NurbsSurface, Surface, SurfaceIntersection, SurfaceKind,
    intersect_surfaces,
};
use arris_math::{Aabb, Frame, Interval, Meter, Point3, Tolerance, Vec3};

fn frame(origin: [f64; 3], z: [f64; 3]) -> Frame {
    Frame::from_z(Point3::from(origin), Vec3::from(z)).unwrap()
}

fn within() -> Aabb {
    Aabb {
        min: [-10.0; 3],
        max: [10.0; 3],
    }
}

fn meet(a: &Surface, b: &Surface) -> Result<SurfaceIntersection, GeomError> {
    intersect_surfaces(a, b, &within(), tol(), &mut Meter::default())
}

fn wall() -> Surface {
    let pipe = Surface::Cylinder {
        frame: Frame::world(),
        radius: 1.0,
    };
    let nurbs = pipe
        .to_nurbs([Interval::TURN, Interval::new(-1.0, 1.0).unwrap()])
        .unwrap();
    Surface::Nurbs(nurbs)
}

fn distance(surface: &Surface, p: Point3) -> f64 {
    surface.project(p).map_or(f64::INFINITY, |on| on.distance)
}

#[test]
fn a_cylinder_twin_meets_each_analytic_kind_in_both_orders() {
    let wall = wall();
    let sideways = frame([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let others = [
        (
            "plane",
            Surface::Plane {
                frame: frame([0.0, 0.0, 0.2], [0.1, 0.2, 1.0]),
            },
        ),
        (
            "cylinder",
            Surface::Cylinder {
                frame: sideways,
                radius: 0.5,
            },
        ),
        (
            "elliptic cylinder",
            Surface::EllipticCylinder {
                frame: sideways,
                major_radius: 0.6,
                minor_radius: 0.4,
            },
        ),
        (
            "cone",
            Surface::Cone {
                frame: sideways,
                radius: 0.3,
                half_angle: 0.17,
            },
        ),
        (
            "sphere",
            Surface::Sphere {
                frame: frame([0.9, 0.0, 0.0], [0.0, 0.0, 1.0]),
                radius: 0.5,
            },
        ),
        (
            "torus",
            Surface::Torus {
                frame: sideways,
                major_radius: 0.8,
                minor_radius: 0.5,
            },
        ),
    ];
    for (name, other) in &others {
        let hit = meet(&wall, other).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert!(!hit.curves().is_empty(), "{name}: {hit:?}");
        for m in hit.curves() {
            let Curve::Nurbs(c) = &m.curve else {
                panic!("{name}: {m:?}")
            };
            let d = c.domain();
            for k in 0..=40 {
                let p = c.eval(d.lerp(k as f64 / 40.0)).point;
                assert!(distance(&wall, p) <= tol().linear, "{name}: off the wall");
                assert!(distance(other, p) <= tol().linear, "{name}: off the other");
            }
        }
        let back = meet(other, &wall).unwrap_or_else(|e| panic!("{name} swapped: {e}"));
        assert_eq!(hit, back, "{name}: the order reaches the result");
    }
}

#[test]
fn a_hemisphere_twin_meets_the_plane_tangent_at_its_pole_in_the_pole() {
    let globe = Surface::Sphere {
        frame: Frame::world(),
        radius: 2.0,
    };
    let half = [Interval::TURN, Interval::new(0.0, FRAC_PI_2).unwrap()];
    let dome = Surface::Nurbs(globe.to_nurbs(half).unwrap());
    let cap = Surface::Plane {
        frame: frame([0.0, 0.0, 2.0], [0.0, 0.0, 1.0]),
    };
    for (a, b) in [(&dome, &cap), (&cap, &dome)] {
        let hit = meet(a, b).unwrap();
        assert!(hit.curves().is_empty(), "{hit:?}");
        let [point] = hit.points() else {
            panic!("{hit:?}")
        };
        assert_eq!(point.kind, MeetKind::Touch);
        assert!((point.point - Point3::new(0.0, 0.0, 2.0)).norm() <= tol().linear);
    }
}

/// The half dome's twin against the plane through its pole and a
/// meridian: the section is the meridian, an isoline of `u`, and the pole,
/// a collapsed row of `v` the plane crosses, so isolines run both ways.
/// The pole is divided out beside the meridian and comes back as a point
/// (ADR-0061 §3, amended at ADR-0062; it was
/// refused as tube circles both ways), as a walked blend's pole meets the
/// faces it narrows between.
#[test]
fn a_half_dome_twin_meets_the_plane_through_its_pole_in_a_meridian_and_the_pole() {
    let globe = Surface::Sphere {
        frame: Frame::world(),
        radius: 2.0,
    };
    let half = [
        Interval::new(0.0, PI).unwrap(),
        Interval::new(0.0, FRAC_PI_2).unwrap(),
    ];
    let dome = Surface::Nurbs(globe.to_nurbs(half).unwrap());
    let upright = Surface::Plane {
        frame: frame([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
    };
    for (a, b) in [(&dome, &upright), (&upright, &dome)] {
        let hit = meet(a, b).unwrap();
        let [meridian] = hit.curves() else {
            panic!("{hit:?}")
        };
        assert_eq!(meridian.kind, MeetKind::Crossing);
        let d = meridian.curve.domain();
        for k in 0..=40 {
            let p = meridian.curve.eval(d.lerp(k as f64 / 40.0)).point;
            assert!(p.x.abs() <= tol().linear, "off the plane: {p:?}");
            assert!(
                (p.coords.norm() - 2.0).abs() <= tol().linear,
                "off the dome: {p:?}"
            );
        }
        for point in hit.points() {
            assert!((point.point - Point3::new(0.0, 0.0, 2.0)).norm() <= tol().linear);
        }
    }
}

/// `z = (u − v)(u + 2v) + c` over `u ∈ [ε, 1]`, `v ∈ [−0.8, 1.3]`, a
/// quadratic Bézier patch: its saddle at the origin is `ε` past the edge
/// `u = ε`, and the plane `z = 0` misses it by `c`, so the section's near
/// miss turns inside the domain beside it, and leaves it across `u = 1`
/// clear of the corners. Within the tolerance the edge does not
/// tell the saddle from one on it, and the section is traced with the
/// saddle put on the edge (ADR-0061 §3, amended at ADR-0062, where a walked blend's contact edge met the face across a
/// junction so: its turning points were refused as not told apart).
#[test]
fn a_saddle_a_hair_past_the_domains_edge_is_the_sections_singular_point_on_it() {
    let (epsilon, c) = (1e-8, -5e-10);
    let (u, v) = ([epsilon, 1.0], [-0.8, 1.3]);
    // The polar forms of `u`, `v` and `z`, at the ends each control point
    // takes of `[u₀, u₁]` and `[v₀, v₁]`.
    let ends = |x: [f64; 2], k: usize| [[x[0], x[0]], [x[0], x[1]], [x[1], x[1]]][k];
    let mut points = Vec::with_capacity(9);
    for i in 0..3 {
        for j in 0..3 {
            let ([u1, u2], [v1, v2]) = (ends(u, i), ends(v, j));
            let (um, vm) = (0.5 * (u1 + u2), 0.5 * (v1 + v2));
            points.push(Point3::new(um, vm, u1 * u2 + um * vm - 2.0 * v1 * v2 + c));
        }
    }
    let knots = |x: [f64; 2]| vec![x[0], x[0], x[0], x[1], x[1], x[1]];
    let patch = NurbsSurface::new([2, 2], [knots(u), knots(v)], points, vec![1.0; 9]).unwrap();
    let saddle = Surface::Nurbs(patch);
    let flat = Surface::Plane {
        frame: Frame::world(),
    };
    for (a, b) in [(&saddle, &flat), (&flat, &saddle)] {
        let hit = meet(a, b).unwrap();
        assert!(!hit.curves().is_empty(), "{hit:?}");
        let mut nearest = f64::INFINITY;
        for m in hit.curves() {
            let d = m.curve.domain();
            for k in 0..=400 {
                let p = m.curve.eval(d.lerp(k as f64 / 400.0)).point;
                assert!(p.z.abs() <= tol().linear, "off the plane: {p:?}");
                assert!(distance(&saddle, p) <= tol().linear, "off the patch: {p:?}");
                assert!(p.x >= epsilon - tol().linear, "past the edge: {p:?}");
                nearest = nearest.min(p.coords.xy().norm());
            }
        }
        // The arm round `+u` turns `√(8|c|/9)` from the saddle, which is
        // the section's singular point on the edge, within the tolerance
        // of both surfaces there.
        assert!(nearest <= 1e-3, "{nearest}");
        let [on_edge] = hit.points() else {
            panic!("{hit:?}")
        };
        assert!((on_edge.point - Point3::new(epsilon, 0.0, 0.0)).norm() <= tol().linear);
    }
}

/// `z = 4v² − u²` over `[0, 1]²`, a quadratic Bézier patch, against the
/// plane `z = 0`: the section's saddle is the patch's corner at the origin,
/// on an edge of `u` and of `v` at once, and of its four arms the one
/// inside the quadrant, `v = u/2`, runs out across `u = 1`. The saddle's
/// cell is cut at both edges, in either order of the surfaces (ADR-0062, where a walked blend's patch met cylinders
/// at its corners).
#[test]
fn a_saddle_at_a_corner_of_the_domain_keeps_the_one_arm_inside_it() {
    let (u, v) = ([0.0, 1.0], [0.0, 1.0]);
    let ends = |x: [f64; 2], k: usize| [[x[0], x[0]], [x[0], x[1]], [x[1], x[1]]][k];
    let mut points = Vec::with_capacity(9);
    for i in 0..3 {
        for j in 0..3 {
            let ([u1, u2], [v1, v2]) = (ends(u, i), ends(v, j));
            let (um, vm) = (0.5 * (u1 + u2), 0.5 * (v1 + v2));
            points.push(Point3::new(um, vm, 4.0 * v1 * v2 - u1 * u2));
        }
    }
    let knots = |x: [f64; 2]| vec![x[0], x[0], x[0], x[1], x[1], x[1]];
    let patch = NurbsSurface::new([2, 2], [knots(u), knots(v)], points, vec![1.0; 9]).unwrap();
    let saddle = Surface::Nurbs(patch);
    let flat = Surface::Plane {
        frame: Frame::world(),
    };
    for (a, b) in [(&saddle, &flat), (&flat, &saddle)] {
        let hit = meet(a, b).unwrap();
        let [arm] = hit.curves() else {
            panic!("{hit:?}")
        };
        let d = arm.curve.domain();
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for k in 0..=200 {
            let p = arm.curve.eval(d.lerp(k as f64 / 200.0)).point;
            assert!(p.z.abs() <= tol().linear, "off the plane: {p:?}");
            assert!(
                (p.y - 0.5 * p.x).abs() <= tol().linear,
                "off the arm: {p:?}"
            );
            lo = f64::min(lo, p.x);
            hi = f64::max(hi, p.x);
        }
        assert!(lo <= tol().linear && hi >= 1.0 - tol().linear, "{lo} {hi}");
    }
}

#[test]
fn a_nurbs_surface_against_itself_is_flush_and_unsupported() {
    let a = wall();
    assert!(matches!(meet(&a, &a), Err(GeomError::Unsupported { .. })));
}

/// Every pair of a walked face of `chain`'s result (a NURBS face no
/// earlier step's body has, so not one a read operand brought) and an
/// analytic face of `kind`, with the pair's tolerance: the larger of the
/// two faces' own, as S5 decides it.
fn walked_against(chain: &corpus::Chain, kind: SurfaceKind) -> Vec<(Surface, Surface, Tolerance)> {
    let m = &chain.model;
    let body = chain.result().unwrap();
    let surfaces = |body| {
        (m.faces(body).unwrap().into_iter())
            .map(|f| m.face(f.id).unwrap().surface())
            .collect::<Vec<_>>()
    };
    let earlier: Vec<_> = (chain.steps.iter())
        .filter(|(name, _)| **name != chain.result)
        .flat_map(|(_, made)| surfaces(made.body))
        .collect();
    let faces: Vec<_> = (m.faces(body).unwrap().into_iter())
        .map(|f| {
            let face = m.face(f.id).unwrap();
            let id = face.surface();
            (id, m.surface(id).unwrap().clone(), face.tolerance())
        })
        .collect();
    let mut pairs = Vec::new();
    let walked =
        |(id, s, _): &&(_, Surface, _)| s.kind() == SurfaceKind::Nurbs && !earlier.contains(id);
    for (_, walked, tw) in faces.iter().filter(walked) {
        for (_, other, to) in faces.iter().filter(|(_, s, _)| s.kind() == kind) {
            let at = Tolerance::new(tw.max(*to), tol().angular);
            pairs.push((walked.clone(), other.clone(), at));
        }
    }
    pairs
}

/// Each pair traces in both orders, every branch's samples within the
/// pair's tolerance of both surfaces.
fn traces(pairs: &[(Surface, Surface, Tolerance)]) {
    assert!(!pairs.is_empty());
    for (walked, other, at) in pairs {
        for (a, b) in [(walked, other), (other, walked)] {
            let trace = trace_section(a, b, &within(), *at, &mut Meter::default())
                .unwrap_or_else(|e| panic!("{:?} against {:?}: {e}", a.kind(), b.kind()));
            for branch in trace.branches() {
                let d = branch.domain();
                for k in 0..=64 {
                    let p = branch.point(d.lerp(k as f64 / 64.0));
                    for s in [a, b] {
                        assert!(
                            distance(s, p) <= 2.0 * at.linear,
                            "{p:?} off {:?}",
                            s.kind()
                        );
                    }
                }
            }
        }
    }
}

/// `blend/s-bend-oblique-top-fillet`: the walked stripe of the first
/// arc ends at the leaning junction on the ball off the vertex (ADR-0072),
/// its corner on the ruling where the second arc's cylinder is tangent to
/// the first. Against that cylinder the section has a saddle at the
/// corner: on the skin's edge of `u`, `3.5e-7` of the walk's `v` inside
/// the other edge, its value `2e-14` and its reach `4.2e-7`. No cell cut
/// at one edge fits it; the tracer refused the pair as
/// `CrowdedSingularity` until a saddle within its reach of an edge was
/// put on it (ADR-0061, amended 2026-10-28).
#[test]
fn a_saddle_at_a_walked_skins_corner_within_its_reach_of_both_edges_is_on_the_corner() {
    let dir = fixtures::corpus_root().join("blend/s-bend-oblique-top-fillet");
    let chain = corpus::chain(&dir, "default").unwrap();
    traces(&walked_against(&chain, SurfaceKind::Cylinder));
}

/// `blend/nist-ctc-02-ap242-ellipse-chain-fillet`: the same corner
/// against the planes across the excerpt's rulings, where the saddle is
/// off both edges, `1.6e-8` of the walk's `u` and `4.2e-7` of its `v`
/// inside, each within its reach (`9e-6` and `4e-6`). The S-bend's is on
/// one edge to rounding; this one needs the rule on both.
#[test]
fn a_saddle_within_its_reach_of_both_edges_of_a_walked_skin_is_on_the_corner() {
    let dir = fixtures::corpus_root().join("blend/nist-ctc-02-ap242-ellipse-chain-fillet");
    let chain = corpus::chain(&dir, "default").unwrap();
    traces(&walked_against(&chain, SurfaceKind::Plane));
}

/// `blend/rounded-pocket-oblique-floor-fillet` with its floor at 30°: the
/// walked corner skins are a rational quadratic arc of two spans across
/// the stripe, joined at a double knot, where the surface is tangent
/// continuous but not parametrically (its normals agree to `3e-8`). Against
/// the table stripe's cylinder, which it meets on the junction's arc, the
/// section has roots on that knot line with `∂f/∂v` of opposite signs
/// either side (`±5e-9`, the floors `1e-11`), which the tracer took for a
/// turn back at a kink and refused as `UnresolvedTurning`.
#[test]
fn a_section_across_a_walked_skins_arc_knot_is_no_turn_at_a_kink() {
    let dir = fixtures::corpus_root().join("blend/rounded-pocket-oblique-floor-fillet");
    let mut recipe = fixtures::load(&dir).unwrap().recipe;
    recipe.params.insert("a".into(), 30.0);
    let chain = corpus::build("blend/rounded-pocket-oblique-floor-fillet at 30°", &recipe).unwrap();
    traces(&walked_against(&chain, SurfaceKind::Cylinder));
}

/// `blend/rounded-block-oblique-top-fillet` reshaped to the block
/// `blend_prop.rs`'s oblique property found at 1000 cases: `length` 3 and
/// `width` 1.5 for the recipe's 4 and 2, the tilt's axis across the middle,
/// and the cut part turned 180° about `y` and dropped 89.66 before its
/// fillet, as the property posed it.
fn posed_block(corner: f64, tilt: f64, r: f64, drop: f64) -> corpus::Chain {
    use arris_debug::fixtures::{Loop, Num, Rotate, Segment, Step};
    let dir = fixtures::corpus_root().join("blend/rounded-block-oblique-top-fillet");
    let mut recipe = fixtures::load(&dir).unwrap().recipe;
    for (k, x) in [
        ("R", corner),
        ("a", tilt),
        ("r", r),
        ("L", 3.0),
        ("W", 1.5),
        ("Z", drop),
    ] {
        recipe.params.insert(k.into(), x);
    }
    let sized = |n: &mut Num| match n {
        Num::Literal(x) if *x == 4.0 => *n = Num::Expr("L".into()),
        Num::Literal(x) if *x == 2.0 => *n = Num::Expr("W".into()),
        Num::Expr(t) if t.starts_with("4 ") => *t = format!("L{}", &t[1..]),
        Num::Expr(t) if t.starts_with("2 ") => *t = format!("W{}", &t[1..]),
        _ => {}
    };
    for step in &mut recipe.steps {
        match step {
            Step::Profile {
                outer: Loop::Path { start, segments },
                ..
            } => {
                start.iter_mut().for_each(sized);
                for segment in segments {
                    match segment {
                        Segment::Line { line_to } => line_to.iter_mut().for_each(sized),
                        Segment::Arc { arc_to, via } => {
                            arc_to.iter_mut().chain(via.iter_mut()).for_each(sized);
                        }
                        Segment::Ellipse { .. } => panic!("the block's profile has no ellipse"),
                    }
                }
            }
            Step::Transform {
                rotate:
                    Some(Rotate {
                        origin: Some(origin),
                        ..
                    }),
                ..
            } => origin[1] = Num::Expr("W / 2".into()),
            _ => {}
        }
    }
    let Some(Step::Fillet { name, radius, .. }) = recipe.steps.pop() else {
        panic!("the recipe ends in its fillet");
    };
    let l = Num::Literal;
    recipe.steps.push(Step::Transform {
        name: "posed".into(),
        of: "cut_top".into(),
        translate: Some([l(0.0), l(0.0), Num::Expr("Z".into())]),
        rotate: Some(Rotate {
            axis: [l(0.0), l(1.0), l(0.0)],
            origin: None,
            angle_deg: l(180.0),
        }),
    });
    recipe.steps.push(Step::Fillet {
        name,
        of: "posed".into(),
        edges: vec![[
            Num::Expr("-L / 2".into()),
            l(0.0),
            Num::Expr("(W / 2) * tan(radians(a)) - 1.5 + Z".into()),
        ]],
        radius,
    });
    corpus::build("blend/rounded-block-oblique-top-fillet, posed", &recipe).unwrap()
}

/// A rounded block under a top tilted 31°, posed as the oblique property
/// found it: its walked corner skins are tangent to the table stripes'
/// cylinders along the junctions' arcs, `F` along the isoline `3e-13`,
/// but the tangency's own curve wanders `6e-7` of the walk's `u` either
/// side of it, so its slope across reaches `5.6e-8` against a bound of
/// `4.5e-8`. Measured as if dropped over the whole domain the line was a
/// crossing, the quotient kept the tangency's other half beside the edge,
/// and the tracer refused the pair as `UnresolvedTurning`; the slope is
/// dropped over the patches the isoline is divided out of alone, a tenth
/// of that, and the line is a tangency (ADR-0061, amended 2026-10-29).
#[test]
fn a_walked_skin_tangent_to_its_table_cylinder_along_the_arc_is_divided_at_order_two() {
    let chain = posed_block(
        0.4591144096367237,
        31.015754421798317,
        0.22468281425389497,
        -89.66487392943412,
    );
    traces(&walked_against(&chain, SurfaceKind::Cylinder));
}
