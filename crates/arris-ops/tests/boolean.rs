//! `ops::boolean::interferences` (ADR-0004):
//! the pave model on the corpus's boolean fixtures — the section curves,
//! their paves and the hits that made them — at random poses of a box
//! and a cylinder, and identical over two runs.

use arris_debug::unmetered::{
    common, cut, extrude, fuse, fuse_many, interferences, interferences_many, mass_properties,
    primitive_box, primitive_cylinder, revolve, transform,
};
use arris_debug::{corpus, fixtures, prop, sample};
use arris_geom::{
    Curve, Curve2, GeomKind, MeetKind, Profile, ProfileLoop, ProfileSegment, Surface,
    SurfaceIntersection, SurfaceKind,
};
use arris_math::real;
use arris_ops::OpError;

use arris_math::nalgebra::UnitQuaternion;
use arris_math::{Axis, Frame, Interval, Isometry, Point3, Vec3};

use arris_ops::boolean::{Interferences, Landing, VertexSource};
use arris_topo::{Body, EdgeId, Model};
use core::f64::consts::TAU;
use proptest::prelude::*;

/// The operands of a boolean fixture, built by its recipe.
fn inputs(name: &str) -> (Model, Body, Body) {
    let dir = fixtures::corpus_root().join(name);
    let inputs = corpus::inputs(&dir, "default").unwrap();
    let (a, b) = inputs.operands().unwrap();
    (inputs.model, a, b)
}

fn interferences_of(name: &str) -> (Model, Body, Body, Interferences) {
    let (m, a, b) = inputs(name);
    let i = interferences(&m, a, b).unwrap();
    (m, a, b, i)
}

/// The edges of `body` whose curve is a line, in iteration order.
fn line_edges(m: &Model, body: Body) -> Vec<EdgeId> {
    m.edges(body)
        .unwrap()
        .into_iter()
        .filter(|e| {
            let edge = m.edge(e.id).unwrap();
            edge.curve()
                .is_some_and(|(c, _)| matches!(m.curve(c).unwrap(), Curve::Line { .. }))
        })
        .map(|e| e.id)
        .collect()
}

/// `n` points over `range`, both ends included (`Interval::samples`, for
/// `n >= 2`; a test with a count of one would divide by zero here).
fn samples(range: Interval, n: usize) -> impl Iterator<Item = f64> {
    (0..n).map(move |i| range.lerp(i as f64 / (n - 1) as f64))
}

/// The surfaces of a section edge's two faces.
fn surfaces_of<'m>(m: &'m Model, i: &Interferences, section: usize) -> [&'m Surface; 2] {
    let pair = &i.pairs[i.curves[i.sections[section].curve].pair];
    [pair.a, pair.b].map(|f| m.surface(m.face(f).unwrap().surface()).unwrap())
}

/// Every section edge lies on both surfaces, its pcurves are
/// same-parameter with it within its tolerance, its ends are its
/// vertices, and its vertices are at least as tolerant as it.
fn assert_sections_consistent(m: &Model, i: &Interferences) -> Result<(), TestCaseError> {
    for (k, s) in i.sections.iter().enumerate() {
        let curve = &i.curves[s.curve].curve;
        let surfaces = surfaces_of(m, i, k);
        for t in samples(s.range, 23) {
            let p = curve.point(t);
            for (which, surface) in surfaces.iter().enumerate() {
                let d = surface
                    .project(p)
                    .map_err(|e| TestCaseError::fail(e.to_string()))?
                    .distance;
                prop_assert!(d <= s.tolerance, "s{k} off surface {which} by {d}\n{i}");
                let q = s.pcurves[which].point(t);
                let gap = (surface.point(q.x, q.y) - p).norm();
                prop_assert!(
                    gap <= s.tolerance,
                    "s{k} pcurve {which} not same-parameter: {gap} at t {t}\n{i}"
                );
            }
        }
        for (end, t) in [(s.start, s.range.lo()), (s.end, s.range.hi())] {
            let v = &i.vertices[end];
            let gap = (curve.point(t) - v.point).norm();
            prop_assert!(gap <= v.tolerance, "s{k} end v{end} off by {gap}\n{i}");
            prop_assert!(v.tolerance >= s.tolerance, "v{end} below s{k}\n{i}");
        }
    }
    for (edge, paves) in &i.paves {
        let e = m.edge(*edge).unwrap();
        // A degenerate edge has no curve: its paves are the arrivals at a
        // singular vertex, read in its own (u, v).
        let Some((c, range)) = e.curve() else {
            continue;
        };
        let curve = m.curve(c).unwrap();
        for p in paves {
            prop_assert!(
                range.contains(p.t),
                "{edge} pave t {} outside {range:?}",
                p.t
            );
            let v = &i.vertices[p.vertex];
            let gap = (curve.point(p.t) - v.point).norm();
            prop_assert!(
                gap <= v.tolerance,
                "{edge} pave v{} off by {gap}\n{i}",
                p.vertex
            );
        }
    }
    Ok(())
}

/// A `Meets` of curves only, every one meeting as `kind`: a crossing or
/// a touching pair.
fn meets_only(r: &SurfaceIntersection, kind: MeetKind) -> bool {
    r.points().is_empty() && !r.curves().is_empty() && r.curves().iter().all(|c| c.kind == kind)
}

#[test]
fn through_hole_has_two_section_circles_each_paved_once_at_the_seam() {
    let (m, _, hole, i) = interferences_of("boolean/through-hole");
    let seam = line_edges(&m, hole);
    assert_eq!(seam.len(), 1);
    assert_eq!(i.hits.len(), 2, "{i}");
    for h in &i.hits {
        assert_eq!(h.edge, seam[0], "{i}");
        assert_eq!(h.landing, Landing::Interior);
        assert!(!h.tangent && h.at_vertex.is_none() && h.vertex.is_some());
    }
    assert_eq!(i.vertices.len(), 2, "{i}");
    assert_eq!(i.paves.len(), 1, "only the seam is paved\n{i}");
    assert_eq!(i.paves[&seam[0]].len(), 2);
    assert_eq!(i.curves.len(), 2, "{i}");
    assert_eq!(i.sections.len(), 2, "{i}");
    for s in &i.sections {
        let c = &i.curves[s.curve];
        assert!(
            matches!(c.curve, Curve::Circle { radius, .. } if (radius - 4.0).abs() < 1e-12),
            "{i}"
        );
        assert_eq!(c.paves.len(), 1);
        assert_eq!(c.paves[0].t, 0.0, "the seam is the circle's parameter zero");
        assert_eq!(s.start, s.end);
        assert_eq!(s.range, Interval::TURN);
        assert!(
            matches!(s.pcurves[0], Curve2::Circle { .. }),
            "on the cap plane"
        );
        assert!(matches!(s.pcurves[1], Curve2::Line { .. }), "on the wall");
        assert!((s.tolerance - m.precision().default_tolerance).abs() < 1e-15);
    }
    assert!(i.coincident.is_empty());
    assert_sections_consistent(&m, &i).unwrap();
}

#[test]
fn blind_hole_has_one_section_circle() {
    let (m, _, _, i) = interferences_of("boolean/blind-hole");
    assert_eq!(i.hits.len(), 1, "{i}");
    assert_eq!(i.sections.len(), 1, "{i}");
    assert!(matches!(
        i.curves[i.sections[0].curve].curve,
        Curve::Circle { .. }
    ));
    assert_eq!(i.curves[i.sections[0].curve].paves.len(), 1);
    assert_sections_consistent(&m, &i).unwrap();
}

#[test]
fn frame_cut_has_eight_segments_paved_by_the_windows_vertical_edges() {
    let (m, _, window, i) = interferences_of("boolean/frame-cut");
    let vertical: Vec<EdgeId> = line_edges(&m, window)
        .into_iter()
        .filter(|&e| {
            let (c, _) = m.edge(e).unwrap().curve().unwrap();
            matches!(m.curve(c).unwrap(), Curve::Line { direction, .. } if direction.z.abs() > 0.5)
        })
        .collect();
    assert_eq!(vertical.len(), 4);
    assert_eq!(i.hits.len(), 8, "{i}");
    assert!(i.hits.iter().all(|h| vertical.contains(&h.edge)), "{i}");
    assert_eq!(i.vertices.len(), 8, "{i}");
    assert_eq!(i.sections.len(), 8, "{i}");
    let mut lengths: Vec<f64> = i.sections.iter().map(|s| s.range.length()).collect();
    lengths.sort_by(f64::total_cmp);
    for (found, wanted) in lengths
        .iter()
        .zip([10.0, 10.0, 10.0, 10.0, 20.0, 20.0, 20.0, 20.0])
    {
        assert!((found - wanted).abs() < 1e-9, "{lengths:?}\n{i}");
    }
    for s in &i.sections {
        assert!(matches!(i.curves[s.curve].curve, Curve::Line { .. }));
        assert_ne!(s.start, s.end);
        assert_eq!(i.curves[s.curve].paves.len(), 2);
    }
    assert_sections_consistent(&m, &i).unwrap();
}

#[test]
fn corner_union_has_six_unit_segments() {
    let (m, _, _, i) = interferences_of("boolean/corner-union");
    assert_eq!(i.hits.len(), 6, "{i}");
    assert_eq!(i.vertices.len(), 6, "{i}");
    assert_eq!(i.sections.len(), 6, "{i}");
    for s in &i.sections {
        assert!((s.range.length() - 1.0).abs() < 1e-12, "{i}");
    }
    assert_sections_consistent(&m, &i).unwrap();
}

#[test]
fn a_disjoint_pair_and_a_tangent_touch_have_no_section() {
    let (_, _, _, i) = interferences_of("boolean/disjoint-cut");
    assert!(
        i.pairs.is_empty() && i.hits.is_empty() && i.sections.is_empty(),
        "{i}"
    );

    let (_, _, _, i) = interferences_of("boolean/tangent-outside-cut");
    assert!(
        i.pairs
            .iter()
            .any(|p| meets_only(&p.intersection, MeetKind::Touch)),
        "{i}"
    );
    assert!(
        i.hits.iter().all(|h| h.tangent && h.vertex.is_none()),
        "{i}"
    );
    assert!(i.vertices.is_empty(), "{i}");
    assert!(i.sections.is_empty(), "{i}");
    // The touches pave the ruling: the plate's rim edges touch the wall
    // at z 0 and z 10, and the segment between is the contact, inside
    // both faces, at (40, 15, 5).
    assert_eq!(i.contacts.len(), 1, "{i}");
    let c = &i.contacts[0];
    assert!(meets_only(&i.pairs[c.pair].intersection, MeetKind::Touch));
    assert!((c.range.length() - 10.0).abs() < 1e-9, "{i}");
    assert!(
        (c.point - Point3::new(40.0, 15.0, 5.0)).norm() < 1e-9,
        "{i}"
    );
}

/// A 30 x 5 x 1 bar and a radius-3 boss of its height whose rim is
/// tangent to the bar's end face `x = 15 * end` from inside, its centre at
/// `x = 12 * end`, or from outside at `x = 18 * end`.
fn bar_and_end_boss(end: f64, outside: bool) -> (Model, Body, Body) {
    let mut m = Model::default();
    let (bar, _) = primitive_box(&mut m, [-15.0, -2.5, -0.5], [15.0, 2.5, 0.5]).unwrap();
    let centre = if outside { 18.0 } else { 12.0 } * end;
    let axis = Axis::new(Point3::new(centre, 0.0, -0.5), Vec3::z()).unwrap();
    let (boss, _) = primitive_cylinder(&mut m, axis, 3.0, 1.0).unwrap();
    (m, bar, boss)
}

/// The boss's rim touches the bar's end edge at `(−15, 0, ±0.5)`, inside
/// both edges, on the top and the bottom the two share. Where the rim runs
/// inside the bar's face beside the touch, the touch is a vertex of both
/// edges, which paves the end edge and the rim, and the end face's touch of
/// the wall joins it: the bar's face keeps a corner on each side of it. A
/// rim touching the end edge from outside runs inside neither face and
/// splits nothing.
#[test]
fn a_rim_touching_an_edge_from_inside_its_face_is_a_vertex_of_both() {
    let (m, bar, boss) = bar_and_end_boss(-1.0, false);
    let i = interferences(&m, bar, boss).unwrap();
    let touches: Vec<_> = i
        .crossings
        .iter()
        .filter(|x| (x.point - Point3::new(-15.0, 0.0, x.point.z)).norm() < 1e-9)
        .collect();
    assert_eq!(touches.len(), 2, "{i}");
    for x in touches {
        let vertex = x.vertex.unwrap_or_else(|| panic!("{i}"));
        assert!(!x.tangent, "{i}");
        for edge in [x.a, x.b] {
            assert!(i.paves[&edge].iter().any(|p| p.vertex == vertex), "{i}");
        }
        assert!(
            i.hits
                .iter()
                .filter(|h| h.tangent)
                .any(|h| h.vertex == Some(vertex)),
            "{i}"
        );
    }

    // At x = 15 from outside, so the rim's seam, at (21, 0), is not there.
    let (m, bar, boss) = bar_and_end_boss(1.0, true);
    let i = interferences(&m, bar, boss).unwrap();
    assert!(!i.crossings.is_empty(), "{i}");
    assert!(
        i.crossings.iter().all(|x| x.tangent && x.vertex.is_none()),
        "{i}"
    );
    assert!(i.vertices.is_empty(), "{i}");
}

/// The bar with a boss tangent to each end (`boolean/fuse-bar-end-bosses-tangent`):
/// one boss then the other, in either order, either operand first, and
/// both in one `fuse_many`, give the same body, clean at `Full`, with the
/// closed form's volume. At `x = 15` the touch is the boss's seam vertex,
/// at `x = −15` it lies inside both edges.
#[test]
fn bosses_tangent_to_both_ends_fuse_alike_in_any_order() {
    let (d, r) = (2.5_f64, 3.0_f64);
    let half = (r * r - d * d).sqrt();
    let segment = r * r * real::atan2(half, d) - d * half;
    let exact = 150.0 + 4.0 * segment;
    let build = |order: usize| {
        let mut m = Model::default();
        let (bar, _) = primitive_box(&mut m, [-15.0, -2.5, -0.5], [15.0, 2.5, 0.5]).unwrap();
        let [b1, b2] = [12.0, -12.0].map(|x| {
            let axis = Axis::new(Point3::new(x, 0.0, -0.5), Vec3::z()).unwrap();
            primitive_cylinder(&mut m, axis, r, 1.0).unwrap().0
        });
        let body = match order {
            0 => {
                let (one, _) = fuse(&mut m, bar, b1).unwrap();
                fuse(&mut m, one, b2).unwrap().0
            }
            1 => {
                let (one, _) = fuse(&mut m, bar, b2).unwrap();
                fuse(&mut m, one, b1).unwrap().0
            }
            2 => {
                let (one, _) = fuse(&mut m, b2, bar).unwrap();
                fuse(&mut m, b1, one).unwrap().0
            }
            _ => fuse_many(&mut m, &[bar, b1, b2]).unwrap().0,
        };
        (m, body)
    };
    for order in 0..4 {
        let (m, body) = build(order);
        let report = check(&m, body, Level::Full);
        assert!(
            report.is_ok() && report.unchecked().is_empty(),
            "order {order}: {report}"
        );
        assert_eq!(
            arris_debug::dump::euler_line(&m, body).unwrap(),
            "euler 30/62/34/34/1 g0 = 0",
            "order {order}"
        );
        let volume = mass_properties(&m, body).unwrap().volume;
        assert!(
            (volume - exact).abs() < 1e-9 * exact,
            "order {order}: {volume} against {exact}"
        );
    }
}

/// A 4 x 6 x 8 box bored along z by a cylinder of radius `r`, then a 1 x 1
/// column through the bore cut away (`boolean/cut-bore-tangent-column` at
/// `r = 0.5`). At 0.5 the bore touches each column wall along a ruling from
/// inside it: the box's material lies on both sides of the ruling on the
/// wall and the column's on both sides on the bore, so the ruling is no
/// section on either face and each wall stays whole. At 0.45 the bore is
/// inside the column; at 0.55 it reaches past each wall by a circular
/// segment. Each is clean at `Full`, genus 1, with the closed form's volume.
#[test]
fn a_column_through_a_bore_it_touches_cuts_to_the_closed_form() {
    for r in [0.45_f64, 0.5, 0.55] {
        let mut m = Model::default();
        let (bx, _) = primitive_box(&mut m, [-2.0, -3.0, -4.0], [2.0, 3.0, 4.0]).unwrap();
        let axis = Axis::new(Point3::new(0.0, 0.0, 4.0), -Vec3::z()).unwrap();
        let (bore, _) = primitive_cylinder(&mut m, axis, r, 20.0).unwrap();
        let (bored, _) = cut(&mut m, bx, bore).unwrap();
        let (column, _) = primitive_box(&mut m, [-0.5, -0.5, -10.0], [0.5, 0.5, 10.0]).unwrap();
        let (body, _) = cut(&mut m, bored, column).unwrap();
        let report = check(&m, body, Level::Full);
        assert!(
            report.is_ok() && report.unchecked().is_empty(),
            "r = {r}: {report}"
        );
        assert_eq!(
            report.euler().map(|e| e.genus),
            Some(1),
            "r = {r}: {report}"
        );
        // The hole's section is the unit square and, past each of its
        // sides, the bore's segment beyond it.
        let d = 0.5_f64;
        let segment = if r > d {
            r * r * real::acos(d / r) - d * (r * r - d * d).sqrt()
        } else {
            0.0
        };
        let exact = 4.0 * 6.0 * 8.0 - 8.0 * (1.0 + 4.0 * segment);
        let volume = mass_properties(&m, body).unwrap().volume;
        assert!(
            (volume - exact).abs() < 1e-9 * exact,
            "r = {r}: {volume} against {exact}"
        );
    }
}

/// A vertical unit cylinder, z 0..3 on x = 5, fused with a horizontal
/// one of length `len` along x from −6 at z = 3
/// (`boolean/fuse-end-disc-tangent-to-wall` at `len = 12`). At 12 its end
/// disc, in x = 6, touches the vertical wall along a ruling, and the
/// section ellipse through (6, 0, 2) touches the horizontal wall's end
/// rim there: the order at that node is decided by the ellipse's own
/// tangent, which its fitted pcurve misses by about a tenth of a
/// micro-radian. At 12.5 and 14 the end disc stands clear. The vertical
/// cylinder's part inside the horizontal one is 8/3 for every length
/// from 12, and each wall loses 4 of its area to the other: V = 3π + lenπ
/// − 8/3, A = 9π + 2π len − 8, clean at `Full` and genus 0.
#[test]
fn a_cylinder_whose_end_disc_touches_a_wall_fuses_to_the_closed_form() {
    for len in [12.0_f64, 12.5, 14.0] {
        let mut m = Model::default();
        let up = Axis::new(Point3::new(5.0, 0.0, 0.0), Vec3::z()).unwrap();
        let (vertical, _) = primitive_cylinder(&mut m, up, 1.0, 3.0).unwrap();
        let along = Axis::new(Point3::new(-6.0, 0.0, 3.0), Vec3::x()).unwrap();
        let (horizontal, _) = primitive_cylinder(&mut m, along, 1.0, len).unwrap();
        let (body, _) = fuse(&mut m, vertical, horizontal).unwrap();
        let report = check(&m, body, Level::Full);
        assert!(
            report.is_ok() && report.unchecked().is_empty(),
            "len = {len}: {report}"
        );
        assert_eq!(
            report.euler().map(|e| e.genus),
            Some(0),
            "len = {len}: {report}"
        );
        let pi = core::f64::consts::PI;
        let measured = mass_properties(&m, body).unwrap();
        let volume = 3.0 * pi + len * pi - 8.0 / 3.0;
        let area = 9.0 * pi + 2.0 * pi * len - 8.0;
        assert!(
            (measured.volume - volume).abs() < 1e-9 * volume,
            "len = {len}: {} against {volume}",
            measured.volume
        );
        assert!(
            (measured.area - area).abs() < 1e-9 * area,
            "len = {len}: {} against {area}",
            measured.area
        );
    }
}

#[test]
fn a_boss_has_one_section_circle_and_swallows_its_bottom_cap() {
    let (m, _, _, i) = interferences_of("boolean/boss");
    assert_eq!(i.sections.len(), 1, "{i}");
    assert!(matches!(
        i.curves[i.sections[0].curve].curve,
        Curve::Circle { .. }
    ));
    assert_eq!(i.hits.len(), 1, "{i}");
    assert_sections_consistent(&m, &i).unwrap();
}

#[test]
fn an_oblique_hole_has_two_ellipses_with_nurbs_pcurves_on_the_wall() {
    let (m, _, _, i) = interferences_of("boolean/oblique-hole");
    assert_eq!(i.sections.len(), 2, "{i}");
    for (k, s) in i.sections.iter().enumerate() {
        assert!(
            matches!(i.curves[s.curve].curve, Curve::Ellipse { .. }),
            "{i}"
        );
        assert!(
            matches!(s.pcurves[0], Curve2::Ellipse { .. }),
            "on the plate"
        );
        assert!(matches!(s.pcurves[1], Curve2::Nurbs(_)), "on the wall");
        assert_eq!(s.start, s.end);
        assert!((s.range.length() - core::f64::consts::TAU).abs() < 1e-12);
        // E4 on the wall: the surface along the pcurve is the curve at
        // the same parameter within the edge's tolerance, at the
        // checker's sample count.
        let wall = surfaces_of(&m, &i, k)[1];
        for t in samples(s.range, m.precision().check_samples) {
            let q = s.pcurves[1].point(t);
            let gap = (wall.point(q.x, q.y) - i.curves[s.curve].curve.point(t)).norm();
            assert!(gap <= s.tolerance, "{gap} at t {t}\n{i}");
        }
        // The pcurve lies in the wall's own copy of the domain.
        let Curve2::Nurbs(n) = &s.pcurves[1] else {
            unreachable!()
        };
        for p in n.control_points() {
            assert!(
                p.x >= -1e-6 && p.x <= core::f64::consts::TAU + 1e-6,
                "{p}\n{i}"
            );
        }
    }
    assert_sections_consistent(&m, &i).unwrap();
}

/// Two equal cylinders crossing at 90°: one wall pair, two ellipses,
/// each seam piercing the other wall twice, and the two ellipses crossing
/// each other at (0, ±R, 0) where no edge is — two section crossings,
/// each a vertex of its own that paves both ellipses, so every ellipse
/// has four paves and four section edges.
#[test]
fn crossing_cylinders_have_two_section_crossings_paving_both_ellipses() {
    let (m, _, _, i) = interferences_of("boolean/cross-cylinders-common");
    let transversal: Vec<usize> = (0..i.pairs.len())
        .filter(|&p| meets_only(&i.pairs[p].intersection, MeetKind::Crossing))
        .collect();
    assert_eq!(transversal.len(), 1, "{i}");
    assert_eq!(i.hits.len(), 4, "{i}");
    assert_eq!(i.section_crossings.len(), 2, "{i}");
    let mut crossing_vertices = Vec::new();
    for (k, x) in i.section_crossings.iter().enumerate() {
        assert_eq!(x.pair, transversal[0]);
        assert_eq!(x.curves, [0, 1]);
        assert!(!x.tangent, "{i}");
        assert!(x.point.x.abs() < 1e-9 && x.point.z.abs() < 1e-9, "{i}");
        assert!((x.point.y.abs() - 1.0).abs() < 1e-9, "{i}");
        let v = x
            .vertex
            .expect("a crossing that is not a touch has a vertex");
        assert_eq!(i.vertices[v].source, VertexSource::SectionCrossing, "{i}");
        assert_eq!(i.vertices[v].section_crossings, vec![k], "{i}");
        assert!(i.vertices[v].hits.is_empty(), "{i}");
        crossing_vertices.push(v);
    }
    assert_eq!(i.vertices.len(), 6, "{i}");
    assert_eq!(i.curves.len(), 2, "{i}");
    for c in &i.curves {
        assert!(matches!(c.curve, Curve::Ellipse { .. }), "{i}");
        assert_eq!(c.paves.len(), 4, "{i}");
        assert_eq!(c.edges.len(), 4, "{i}");
        for &v in &crossing_vertices {
            assert!(c.paves.iter().any(|p| p.vertex == v), "{i}");
        }
    }
    assert_eq!(i.sections.len(), 8, "{i}");
    assert_sections_consistent(&m, &i).unwrap();
}

/// The touches that land on a crossing vertex: the tool's seam through
/// (0, R, 0) in `seam-through-crossing-common`, and the branch's rim
/// circle through both crossing vertices in `tee-fuse`. Each touch joins
/// the section vertex the two ellipses made and paves its edge there; a
/// touch landing nowhere still joins nothing.
#[test]
fn a_touch_on_a_crossing_vertex_joins_it_and_paves_its_edge() {
    for (name, touches) in [
        ("boolean/seam-through-crossing-common", 1),
        ("boolean/tee-fuse", 2),
    ] {
        let (m, _, _, i) = interferences_of(name);
        assert_eq!(i.section_crossings.len(), 2, "{name}\n{i}");
        let joined: Vec<usize> = (0..i.hits.len())
            .filter(|&h| i.hits[h].tangent && i.hits[h].vertex.is_some())
            .collect();
        assert_eq!(joined.len(), touches, "{name}\n{i}");
        for h in joined {
            let hit = &i.hits[h];
            let v = hit.vertex.unwrap();
            assert_eq!(
                i.vertices[v].source,
                VertexSource::SectionCrossing,
                "{name}\n{i}"
            );
            assert!(i.vertices[v].hits.contains(&h), "{name}\n{i}");
            assert!((hit.point.y.abs() - 1.0).abs() < 1e-9, "{name}\n{i}");
            let paves = i.paves.get(&hit.edge).expect("the touched edge is paved");
            assert!(paves.iter().any(|p| p.vertex == v), "{name}\n{i}");
        }
        assert_sections_consistent(&m, &i).unwrap();
    }
}

/// `boolean/cross-cylinders-fuse`'s two cylinders, `R = 1` and `L = 6`
/// on the `z` and `x` axes, the second turned about its own axis by
/// `turn` degrees — which moves its seam and nothing else.
fn turned_crossing(turn: f64) -> (Model, Body, Body) {
    let mut m = Model::default();
    let along = |origin: Point3, direction: Vec3| Axis::new(origin, direction).unwrap();
    let (a, _) = primitive_cylinder(
        &mut m,
        along(Point3::new(0.0, 0.0, -3.0), Vec3::z()),
        1.0,
        6.0,
    )
    .unwrap();
    let (b, _) = primitive_cylinder(
        &mut m,
        along(Point3::new(-3.0, 0.0, 0.0), Vec3::x()),
        1.0,
        6.0,
    )
    .unwrap();
    let about = UnitQuaternion::from_axis_angle(&Vec3::x_axis(), turn.to_radians());
    let (b, _) = transform(&mut m, b, &Isometry::from_rotation(about)).unwrap();
    (m, a, b)
}

/// A touch off every vertex stands for the crossings the section curves
/// make of its edge (ADR-0016). Crossing cylinders with the second turned
/// about its own axis to just beside ±90°: its seam runs `R sin δ` beside
/// a crossing vertex, a chord `R (1 − cos δ)` deep in the first wall —
/// under the tolerance up to 0.0256°, so the intersector's one touch,
/// landing on nothing — and both ellipses cross it. The two crossings are
/// hits on that wall to rounding, each with its vertex paving the seam and
/// one ellipse, and the pave is the one of a generic turn: eight section
/// edges, not a section edge over a seam (`Fault::Seam`).
#[test]
fn a_touch_beside_a_crossing_vertex_is_resolved_through_the_section_curves() {
    for turn in [
        -90.02_f64, -90.01, -90.001, -90.0001, -89.9999, -89.98, 89.98, 90.02,
    ] {
        let (m, a, b) = turned_crossing(turn);
        let i = interferences(&m, a, b).unwrap_or_else(|e| panic!("turn {turn}: {e}"));

        let touches: Vec<&_> = i.hits.iter().filter(|h| h.tangent).collect();
        assert_eq!(touches.len(), 1, "turn {turn}\n{i}");
        let touch = touches[0];
        assert_eq!(touch.vertex, None, "turn {turn}\n{i}");
        let resolved: Vec<&_> = i
            .hits
            .iter()
            .filter(|h| !h.tangent && h.edge == touch.edge && h.face == touch.face)
            .collect();
        assert_eq!(resolved.len(), 2, "turn {turn}\n{i}");
        let lateral = real::sin((turn.abs() - 90.0).to_radians()).abs();
        for hit in &resolved {
            assert!(
                ((hit.t - touch.t).abs() - lateral).abs() < 1e-12,
                "turn {turn}\n{i}"
            );
            assert!(
                (real::hypot(hit.point.x, hit.point.y) - 1.0).abs() < 1e-14,
                "turn {turn}\n{i}"
            );
            let v = hit.vertex.expect("a crossing has a vertex");
            assert_eq!(i.vertices[v].source, VertexSource::Hits, "turn {turn}\n{i}");
            let paves = i.paves.get(&hit.edge).expect("the seam is paved");
            assert!(paves.iter().any(|p| p.vertex == v), "turn {turn}\n{i}");
            let on = i
                .curves
                .iter()
                .filter(|c| c.paves.iter().any(|p| p.vertex == v))
                .count();
            assert_eq!(on, 1, "turn {turn}\n{i}");
        }
        assert!(
            i.hits
                .windows(2)
                .all(|w| (w[0].edge, w[0].t) <= (w[1].edge, w[1].t)),
            "turn {turn}\n{i}"
        );
        assert_eq!(i.hits.len(), 5, "turn {turn}\n{i}");
        assert_eq!(i.vertices.len(), 6, "turn {turn}\n{i}");
        for c in &i.curves {
            assert_eq!((c.paves.len(), c.edges.len()), (4, 4), "turn {turn}\n{i}");
        }
        assert_eq!(i.sections.len(), 8, "turn {turn}\n{i}");
        assert_sections_consistent(&m, &i).unwrap();
    }
}

/// Two parallel walls, one ruling of the pair on the target's seam: the
/// seam lies in the tool's wall, the block of that ruling is the seam and
/// no section edge, and the seam's one piece inside the tool's wall is
/// its image there — no face of the target is coincident with the wall to
/// place it through.
#[test]
fn a_seam_on_a_ruling_is_an_image_on_the_other_wall() {
    let (m, a, _, i) = interferences_of("boolean/parallel-cylinders-seam");
    let seam = line_edges(&m, a);
    assert_eq!(seam.len(), 1);
    assert_eq!(i.coincident.len(), 1, "{i}");
    let (edge, wall) = i.coincident[0];
    assert_eq!(edge, seam[0], "{i}");
    let on_seam: Vec<_> = i
        .curves
        .iter()
        .filter(|c| matches!(c.curve, Curve::Line { .. }))
        .collect();
    assert_eq!(on_seam.len(), 2, "{i}");
    assert_eq!(
        on_seam.iter().filter(|c| c.edges.is_empty()).count(),
        1,
        "the ruling on the seam is no section edge\n{i}"
    );
    assert_eq!(i.images.len(), 1, "{i}");
    let image = &i.images[0];
    assert_eq!(
        (image.edge, image.operand, image.index),
        (edge, 0, 0),
        "{i}"
    );
    assert_eq!(i.pairs[image.pair].b, wall, "{i}");
    assert!(matches!(image.pcurve, Curve2::Line { .. }), "{i}");
    assert_eq!(i.sections.len(), 3, "{i}");
    assert_sections_consistent(&m, &i).unwrap();
}

#[test]
fn two_runs_are_identical() {
    let (m, a, b, i) = interferences_of("boolean/through-hole");
    let again = interferences(&m, a, b).unwrap();
    assert_eq!(i, again);
    assert_eq!(i.to_string(), again.to_string());
    assert!(i.to_string().starts_with("interferences "));
}

/// A NURBS patch flush with another over an area is the one surface pair
/// the intersector refuses (BACKLOG §76): the refusal names the pair, the
/// first operand's face first. A plane against a NURBS patch is an
/// ordinary pair (ADR-0061), and so are two crossing NURBS patches
/// (ADR-0068).
#[test]
fn a_flush_nurbs_pair_is_unsupported_and_names_the_faces() {
    let mut m = Model::default();
    let patchwork =
        sample::cuboid_nurbs(&mut m, Point3::origin(), Point3::new(2.0, 2.0, 2.0)).unwrap();
    let shifted =
        sample::cuboid_nurbs(&mut m, Point3::origin(), Point3::new(2.0, 2.0, 2.0)).unwrap();
    let err = interferences(&m, patchwork, shifted).unwrap_err();
    match err {
        OpError::Unsupported { a, b } => {
            assert_eq!(a.0, GeomKind::Surface(SurfaceKind::Nurbs));
            assert_eq!(b.0, GeomKind::Surface(SurfaceKind::Nurbs));
            assert!(
                m.faces(patchwork)
                    .unwrap()
                    .iter()
                    .any(|f| f.shape().id == a.1.id)
            );
            assert!(
                m.faces(shifted)
                    .unwrap()
                    .iter()
                    .any(|f| f.shape().id == b.1.id)
            );
        }
        other => panic!("{other}"),
    }
}

/// Two NURBS cuboids with no plane in common cross: the intersector marches the
/// section of their patches instead of refusing the pair.
#[test]
fn two_offset_nurbs_cuboids_are_intersected() {
    let mut m = Model::default();
    let patchwork =
        sample::cuboid_nurbs(&mut m, Point3::origin(), Point3::new(2.0, 2.0, 2.0)).unwrap();
    let shifted = sample::cuboid_nurbs(
        &mut m,
        Point3::new(1.0, 0.5, 0.5),
        Point3::new(3.0, 1.5, 3.0),
    )
    .unwrap();
    match interferences(&m, patchwork, shifted) {
        Ok(_) => {}
        Err(OpError::Unsupported { .. }) => {
            panic!("a crossing NURBS pair should not be unsupported")
        }
        Err(other) => panic!("{other}"),
    }
}

/// No guard stands before the intersector (ADR-0008, ADR-0020): the
/// revolve with a cone face against a box through it is an operand like
/// any other — the box's two cap planes cut the cone in circles and its
/// four sides in hyperbolas — and so are a whole sphere face, closed on
/// its two degenerate edges, and a whole torus face, periodic both ways
/// between its two seams. Each cut is a body the checker passes at
/// `Full` with nothing undecided.
#[test]
fn a_cone_face_is_an_operand_like_any_other() {
    let mut m = Model::default();
    let p = |u, v| Point2::new(u, v);
    // The frustum of `sweep/revolve-frustum`: x ∈ [1, 4] at z = −1
    // narrowing to x ∈ [1, 2] at z = 1, about z.
    let profile = Profile {
        plane: Frame::new(Point3::origin(), -Vec3::y(), Vec3::x()).unwrap(),
        outer: ProfileLoop::Path {
            start: p(1.0, -1.0),
            segments: vec![
                ProfileSegment::LineTo(p(4.0, -1.0)),
                ProfileSegment::LineTo(p(2.0, 1.0)),
                ProfileSegment::LineTo(p(1.0, 1.0)),
                ProfileSegment::LineTo(p(1.0, -1.0)),
            ],
        },
        holes: Vec::new(),
    };
    let (frustum, _) = revolve(&mut m, &profile, Axis::z_at(Point3::origin()), TAU).unwrap();
    assert!(
        m.faces(frustum).unwrap().iter().any(|f| m
            .surface(m.face(f.id).unwrap().surface())
            .unwrap()
            .kind()
            == SurfaceKind::Cone)
    );
    let (cube, _) = primitive_box(
        &mut m,
        Point3::new(-5.0, -5.0, -0.5),
        Point3::new(5.0, 5.0, 0.5),
    )
    .unwrap();
    let i = interferences(&m, cube, frustum).unwrap();
    assert!(!i.sections.is_empty(), "{i}");
    assert_sections_consistent(&m, &i).unwrap();
    let (kept, _) = cut(&mut m, cube, frustum).unwrap();
    let report = check(&m, kept, Level::Full);
    assert!(report.is_empty(), "{report}");
    assert!(report.unchecked().is_empty(), "{report}");

    for quadric in [
        sample::sphere(&mut m, Point3::origin(), 1.5).unwrap(),
        sample::torus(&mut m, Point3::origin(), 3.0, 0.8).unwrap(),
    ] {
        let i = interferences(&m, cube, quadric).unwrap();
        assert!(!i.sections.is_empty(), "{i}");
        assert_sections_consistent(&m, &i).unwrap();
        for kept in [
            cut(&mut m, cube, quadric).unwrap().0,
            cut(&mut m, quadric, cube).unwrap().0,
        ] {
            let report = check(&m, kept, Level::Full);
            assert!(report.is_empty(), "{report}");
            assert!(report.unchecked().is_empty(), "{report}");
        }
    }
}

#[test]
fn random_overlapping_pairs_pave_consistently() {
    prop::check(prop::body::overlapping_pair(), |pair| {
        let mut m = Model::default();
        let (a, b) = pair
            .build(&mut m)
            .map_err(|e| TestCaseError::fail(e.to_string()))?;
        let i = interferences(&m, a, b).map_err(|e| TestCaseError::fail(e.to_string()))?;
        prop_assert!(
            !i.sections.is_empty(),
            "the axis passes through the box\n{i}"
        );
        assert_sections_consistent(&m, &i)?;
        let again = interferences(&m, a, b).map_err(|e| TestCaseError::fail(e.to_string()))?;
        prop_assert_eq!(i.to_string(), again.to_string());
        Ok(())
    });
}

/// A cap circle of `body`'s cylinder, whose edge is a periodic curve
/// (`arris_math::Interval::TURN`, [`Curve::Circle`]) — the shape a whole
/// turn's rounding can land a hit at its own start.
fn circle_edges(m: &Model, body: Body) -> Vec<EdgeId> {
    m.edges(body)
        .unwrap()
        .into_iter()
        .filter(|e| {
            let edge = m.edge(e.id).unwrap();
            edge.curve()
                .is_some_and(|(c, _)| matches!(m.curve(c).unwrap(), Curve::Circle { .. }))
        })
        .map(|e| e.id)
        .collect()
}

/// A pin offset from a bore's axis by `big − small`, tangent to the
/// bore's wall from inside along the ruling its cap circles cross at
/// their own `t = 0` (both built on `Axis::z_at`, whose `X` is the same
/// canonical perpendicular for either radius) — then the whole pair
/// turned and carried a hundred units from the origin, so the touch
/// lands where `conic2::trig2_roots` finds it a rounding short of a
/// whole turn without the snap (ADR-0022; found
/// by search over `intersect_curve_surface` directly). `land`'s
/// `vertex_at` already reads the point, not `t`, so the pave model
/// never blocked on the wrapped root either way; the property held here
/// is `intersect_curve_surface`'s own — every edge's paves cut it into
/// blocks no shorter than its tolerance — through a case built to hit
/// this exact boundary.
#[test]
fn a_pin_tangent_at_its_cap_circles_own_start_paves_no_short_block() {
    let (big, small) = (0.11106208302349421, 0.03334964848347397);
    let delta = big - small;
    let bore_height = 4.0 * big;
    let pin_height = 1.5 * big;
    let pin_z = 0.5 * (bore_height - pin_height);

    let mut m = Model::default();
    let (bore, _) =
        primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), big, bore_height).unwrap();
    let (pin, _) = primitive_cylinder(
        &mut m,
        Axis::z_at(Point3::new(delta, 0.0, pin_z)),
        small,
        pin_height,
    )
    .unwrap();

    let motion = Isometry::new(
        UnitQuaternion::from_euler_angles(
            0.1473530425457661,
            -2.5207683519930253,
            2.7501568180617575,
        ),
        Vec3::new(38.50274331577241, 23.32493683456147, -11.174177282923182),
    );
    let (bore, _) = transform(&mut m, bore, &motion).unwrap();
    let (pin, _) = transform(&mut m, pin, &motion).unwrap();

    let i = interferences(&m, bore, pin).unwrap();
    for &edge in circle_edges(&m, bore).iter().chain(&circle_edges(&m, pin)) {
        let e = m.edge(edge).unwrap();
        let (curve_id, range) = e.curve().unwrap();
        let Curve::Circle { radius, .. } = m.curve(curve_id).unwrap() else {
            unreachable!()
        };
        let mut params = vec![range.lo()];
        if let Some(paves) = i.paves.get(&edge) {
            params.extend(paves.iter().map(|p| p.t));
        }
        params.push(range.hi());
        params.sort_by(f64::total_cmp);
        for w in params.windows(2) {
            let length = (w[1] - w[0]) * radius;
            assert!(
                length >= e.tolerance(),
                "edge {edge:?} paves a block {length} long, shorter than its tolerance {}: {params:?}",
                e.tolerance()
            );
        }
    }
}

// ---- `ops::cut` (plan step 7): split, classify, assemble ----

use arris_math::Point2;
use arris_ops::{BooleanReason, InputReason, Reason};
use arris_topo::{AnyId, EntityId, Face as FaceHandle, Orientation, Origin, Provenance, Shape};

use arris_check::{Level, check, lumps};
/// A piece within the tolerance of the other operand throughout is
/// decided by the transversal rule (`docs/ARCHITECTURE.md` §Operations).
/// With the tool's seam `R sin δ` beside a crossing vertex, the piece of
/// its wall between the seam and the two ellipses is a sliver that far
/// across and `R (1 − cos δ)` from the first wall everywhere: its interior
/// point classifies `On` a face its own is transversal to, and was
/// `Unsupported` across the band out to 0.046°. At every turn — the band
/// on both sides of ±90°, its two ends, and the one-sided band where Open
/// CASCADE builds a sliver of its own — `fuse` and `common` are the
/// Steinmetz union and solid, clean at `Full`, with a generic turn's
/// counts, and the volume within 1e-9 of the closed form: the fitted
/// ellipses' error, 2.4e-10 at every turn alike. Within two tolerances
/// of the crossing vertex, 5.8e-6° to 1.1e-5° past ±90° and nearer, the
/// seam's touch joins that vertex and its crossings with it: the body is
/// the one at ±90°, the seam through the vertex, with its counts
/// (`boolean/seam-a-tolerance-from-crossing-fuse`).
#[test]
fn a_sliver_within_the_tolerance_of_the_other_wall_is_decided_at_its_section_edges() {
    let counts_at = |op: Boolean, turn: f64| {
        let (mut m, a, b) = turned_crossing(turn);
        let (body, _) = op(&mut m, a, b).unwrap();
        arris_debug::dump::euler_line(&m, body).unwrap()
    };
    let exact_common = 16.0 / 3.0;
    let exact_fuse = TAU * 6.0 - exact_common;
    let beside = [
        -90.03_f64, -90.04, -90.045, -90.046, -90.02, -90.001, -90.0001, -89.9999, -89.99, -89.97,
        89.97, 90.03,
    ];
    let within_two = [
        -90.000003_f64,
        -90.000006,
        -90.000007,
        -90.00001,
        -89.999993,
        89.999994,
        90.000011,
    ];
    for (name, op, exact) in [
        ("fuse", fuse as Boolean, exact_fuse),
        ("common", common as Boolean, exact_common),
    ] {
        let (generic, through) = (counts_at(op, 30.0), counts_at(op, -90.0));
        let turns = beside
            .iter()
            .map(|&t| (t, &generic))
            .chain(within_two.iter().map(|&t| (t, &through)));
        for (turn, counts) in turns {
            let (mut m, a, b) = turned_crossing(turn);
            let (body, _) = op(&mut m, a, b).unwrap_or_else(|e| panic!("{name} at {turn}: {e}"));
            let report = check(&m, body, Level::Full);
            assert!(
                report.is_ok() && report.unchecked().is_empty(),
                "{name} at {turn}: {report}"
            );
            assert_eq!(
                &arris_debug::dump::euler_line(&m, body).unwrap(),
                counts,
                "{name} at {turn}"
            );
            let volume = mass_properties(&m, body).unwrap().volume;
            assert!(
                (volume - exact).abs() < 1e-9 * exact,
                "{name} at {turn}: {volume} against {exact}"
            );
        }
    }
}

/// A cylinder of radius 1 standing on `z = 0`, and a half-space below a
/// face turned by `tilt` about a line `inside` the rim at `x = −1`: the
/// face's ellipse on the wall crosses the rim twice, `2√(2·inside)`
/// apart, and between the two lies at most `tilt · inside` from it.
fn rim_beside_ellipse(inside: f64, tilt: f64) -> (Model, Body, Body) {
    let mut m = Model::default();
    let (a, _) = primitive_cylinder(
        &mut m,
        Axis::new(Point3::origin(), Vec3::z()).unwrap(),
        1.0,
        2.0,
    )
    .unwrap();
    let (b, _) = primitive_box(&mut m, [-5.0, -5.0, -5.0], [5.0, 5.0, 0.0]).unwrap();
    let q = UnitQuaternion::from_axis_angle(&Vec3::y_axis(), tilt);
    let about = Point3::new(-1.0 + inside, 0.0, 0.0);
    let motion = Isometry::new(q, about.coords - (q * about).coords);
    let (b, _) = transform(&mut m, b, &motion).unwrap();
    (m, a, b)
}

/// A block of a section curve within the tolerance of an operand edge
/// between the same two vertices is that edge's piece, not a section edge
/// of its own: the piece's image on the other face stands where the
/// section edge would have been, and no face has a sliver between the
/// two. At a ball's pole, the small circle 2e-4 of a radian off the seam
/// crosses it again 1.8e-4 on — two curves the intersector calls tangent,
/// asked again at the model's smallest distance — and its block from
/// there to the pole is the seam's last piece, on the cut face
/// (`boolean/pole-slice-beside-seam-cut`). On a cylinder, the ellipse a
/// face turned by 1e-3 cuts from the wall runs a quarter, a half and
/// three quarters of a tolerance beside the bottom rim over 0.02 of it:
/// that block of the ellipse is the rim's piece, the tool's face is
/// bounded by it and by its line across the cap, and the cut and the fuse
/// are clean at `Full` with the rim cut twice, their volumes the
/// cylinder's less and plus a wedge of `(8√2 / 15) tilt · inside^(5/2)`.
/// The common is that wedge alone, under a tolerance thick, a sliver
/// solid of the band's (`docs/BACKLOG.md`), not asked here.
#[test]
fn a_section_block_along_an_operand_edge_is_that_edges_piece() {
    let (m, a, _, i) = interferences_of("boolean/pole-slice-beside-seam-cut");
    let seam = circle_edges(&m, a);
    assert_eq!(seam.len(), 1, "{i}");
    assert_eq!(
        i.paves[&seam[0]].len(),
        1,
        "the seam paved at the far crossing\n{i}"
    );
    assert_eq!(i.sections.len(), 1, "{i}");
    let [image] = &i.images[..] else {
        panic!("{i}");
    };
    assert_eq!(
        (image.edge, image.operand, image.index),
        (seam[0], 0, 1),
        "{i}"
    );
    assert!(
        (image.range.hi() - core::f64::consts::PI).abs() < 1e-15,
        "{i}"
    );
    assert_sections_consistent(&m, &i).unwrap();

    let inside = 5e-5;
    for fraction in [0.25, 0.5, 0.75] {
        let tilt = fraction * 1e-7 / inside;
        let (m, a, b) = rim_beside_ellipse(inside, tilt);
        let i = interferences(&m, a, b).unwrap_or_else(|e| panic!("{fraction}: {e}"));
        let ellipse = i
            .curves
            .iter()
            .find(|c| matches!(c.curve, Curve::Ellipse { .. }))
            .expect("the wall's section");
        assert_eq!(ellipse.paves.len(), 2, "{fraction}\n{i}");
        assert!(ellipse.edges.is_empty(), "{fraction}\n{i}");
        let [image] = &i.images[..] else {
            panic!("{fraction}\n{i}");
        };
        let rim = i.paves.get(&image.edge).expect("the rim is paved");
        assert_eq!(rim.len(), 2, "{fraction}\n{i}");
        assert_eq!(image.index, 1, "{fraction}\n{i}");
        assert_eq!(i.sections.len(), 1, "the line across the cap\n{i}");
        assert_sections_consistent(&m, &i).unwrap();

        let wedge = 8.0 * 2f64.sqrt() / 15.0 * tilt * real::powf(inside, 2.5);
        for (name, op, exact, counts) in [
            ("cut", cut as Boolean, TAU - wedge, "euler 4/6/4/4/1 g0 = 0"),
            (
                "fuse",
                fuse as Boolean,
                500.0 + TAU - wedge,
                "euler 12/18/9/10/1 g0 = 0",
            ),
        ] {
            let (mut m, a, b) = rim_beside_ellipse(inside, tilt);
            let (body, _) =
                op(&mut m, a, b).unwrap_or_else(|e| panic!("{name} at {fraction}: {e}"));
            let report = check(&m, body, Level::Full);
            assert!(
                report.is_ok() && report.unchecked().is_empty(),
                "{name} at {fraction}: {report}"
            );
            assert_eq!(
                arris_debug::dump::euler_line(&m, body).unwrap(),
                counts,
                "{name} at {fraction}"
            );
            let volume = mass_properties(&m, body).unwrap().volume;
            assert!(
                (volume - exact).abs() < 1e-9 * exact,
                "{name} at {fraction}: {volume} against {exact}"
            );
        }
    }
}

/// The traced sections of step 8's fixtures (ADR-0018): each pair of
/// walls meets in closed loops, periodic fitted curves paved where the
/// other operand's seam pierces them, and every section edge on a loop —
/// in the pave model and in the result the boolean builds — is at its
/// faces' tolerance: the fit's quarter of it leaves the pcurves room
/// under the other half, so nothing grows.
#[test]
fn a_traced_section_edge_is_at_its_faces_tolerance() {
    for (name, op, loops) in [
        ("boolean/tee-unequal-fuse", fuse as Boolean, 1),
        ("boolean/tee-unequal-cut", cut as Boolean, 1),
        ("boolean/tee-unequal-common", common as Boolean, 1),
        ("boolean/skew-hole-cut", cut as Boolean, 1),
        ("boolean/skew-bore-cut", cut as Boolean, 2),
    ] {
        let (mut m, a, b) = inputs(name);
        let i = interferences(&m, a, b).unwrap();
        assert_sections_consistent(&m, &i).unwrap();
        let traced: Vec<_> = i
            .curves
            .iter()
            .filter(|c| !c.edges.is_empty())
            .filter(|c| matches!(&c.curve, Curve::Nurbs(n) if n.period().is_some()))
            .collect();
        assert_eq!(traced.len(), loops, "{name}\n{i}");
        for s in &i.sections {
            let pair = &i.pairs[i.curves[s.curve].pair];
            let faces = [pair.a, pair.b].map(|f| m.face(f).unwrap().tolerance());
            assert_eq!(s.tolerance, faces[0].max(faces[1]), "{name}\n{i}");
        }
        let default = m.precision().default_tolerance;
        let (body, _) = op(&mut m, a, b).unwrap_or_else(|e| panic!("{name}: {e}"));
        for e in m.edges(body).unwrap() {
            let edge = m.edge(e.id).unwrap();
            assert_eq!(edge.tolerance(), default, "{name}: {} grew", e.id);
        }
    }
}

/// A boolean through a fitted edge (ADR-0018), the tee's loop in its three
/// pieces between the seams: the box's
/// planes cross it in four points and the drill's wall in two, each a
/// crossing of the NURBS curve against the other operand's surface that
/// paves the edge, and every edge of the result — the fitted edge's
/// pieces, the traced loops the drill's wall adds, the lines and circles
/// the box's planes add — is at its faces' tolerance: nothing grew
/// through the chain, the idea's first tripwire kept as an assertion.
#[test]
fn a_boolean_through_a_fitted_edge_grows_no_tolerance() {
    for (name, crossings) in [
        ("boolean/tee-unequal-slot-cut", 4),
        ("boolean/tee-unequal-drill-cut", 2),
    ] {
        let (mut m, a, b) = inputs(name);
        let fitted: Vec<EdgeId> = m
            .edges(a)
            .unwrap()
            .into_iter()
            .filter(|e| {
                let edge = m.edge(e.id).unwrap();
                edge.curve()
                    .is_some_and(|(c, _)| matches!(m.curve(c).unwrap(), Curve::Nurbs(_)))
            })
            .map(|e| e.id)
            .collect();
        // The loop, cut where the two seams pierce it.
        assert_eq!(fitted.len(), 3, "{name}: the tee's loop");
        let i = interferences(&m, a, b).unwrap();
        assert_sections_consistent(&m, &i).unwrap();
        let hits: Vec<_> = i
            .hits
            .iter()
            .filter(|h| fitted.contains(&h.edge) && !h.tangent)
            .collect();
        assert_eq!(hits.len(), crossings, "{name}\n{i}");
        for h in hits {
            let surface = m.surface(m.face(h.face).unwrap().surface()).unwrap();
            let distance = surface.project(h.point).unwrap().distance;
            assert!(distance < 1e-12, "{name}: {h:?} is {distance} off");
        }
        let default = m.precision().default_tolerance;
        let (body, _) = cut(&mut m, a, b).unwrap_or_else(|e| panic!("{name}: {e}"));
        for e in m.edges(body).unwrap() {
            let edge = m.edge(e.id).unwrap();
            assert_eq!(edge.tolerance(), default, "{name}: {} grew", e.id);
        }
    }
}

/// Two cylinders whose axes pass `R − r` apart, a drill touching the
/// main wall from inside at `(0, R, 0)`: a singular point of the traced
/// section (ADR-0018), each of the two branches leaving it round one exit
/// of the drill and coming back. The point is one section vertex, made
/// from the pair's `Meets` point as its section crossings, and it paves
/// both ends of both branches, so the blocks between it and the seams'
/// hits cover each branch whole.
#[test]
fn a_singular_point_is_the_vertex_that_ends_its_branches() {
    let mut m = Model::default();
    let along = |origin: Point3, direction: Vec3| Axis::new(origin, direction).unwrap();
    let (main, _) = primitive_cylinder(
        &mut m,
        along(Point3::new(0.0, 0.0, -3.0), Vec3::z()),
        1.0,
        6.0,
    )
    .unwrap();
    let (drill, _) = primitive_cylinder(
        &mut m,
        along(Point3::new(-3.0, 0.4, 0.0), Vec3::x()),
        0.6,
        6.0,
    )
    .unwrap();
    let i = interferences(&m, main, drill).unwrap();
    let singular = Point3::new(0.0, 1.0, 0.0);
    let v: Vec<usize> = (0..i.vertices.len())
        .filter(|&k| (i.vertices[k].point - singular).norm() < 1e-12)
        .collect();
    assert_eq!(v.len(), 1, "{i}");
    let v = v[0];
    assert_eq!(i.vertices[v].source, VertexSource::SectionCrossing, "{i}");
    assert!(i.vertices[v].hits.is_empty(), "{i}");
    let branches: Vec<_> = i
        .curves
        .iter()
        .filter(|c| matches!(&c.curve, Curve::Nurbs(n) if n.period().is_none()))
        .collect();
    assert_eq!(branches.len(), 2, "{i}");
    for c in branches {
        let domain = c.curve.domain();
        let (first, last) = (c.paves.first().unwrap(), c.paves.last().unwrap());
        assert_eq!((first.t, first.vertex), (domain.lo(), v), "{i}");
        assert_eq!((last.t, last.vertex), (domain.hi(), v), "{i}");
        let covered: f64 = c.edges.iter().map(|&s| i.sections[s].range.length()).sum();
        assert!((covered - domain.length()).abs() < 1e-12, "{i}");
    }
    assert_sections_consistent(&m, &i).unwrap();
}

/// The fixture's operands cut, the model with them.
fn cut_of(name: &str) -> (Model, Body, Body, Body, Provenance) {
    let (mut m, a, b) = inputs(name);
    let (body, p) = cut(&mut m, a, b).unwrap();
    (m, a, b, body, p)
}

fn shape(id: impl Into<EntityId>) -> Shape {
    Shape::new(id, Orientation::Forward)
}

/// The through-hole's provenance table, exactly: the plate's top and
/// bottom faces `Modified` into one piece each with a hole loop, the four
/// sides kept and unrecorded, the tool's wall `Deleted` and its piece
/// `Generated` from it, the tool's caps, rims and vertices `Deleted`, the
/// seam `Deleted` and its middle piece `Generated`, two section vertices
/// `Generated` from the seam and each cap plane, two section edges the
/// `generated_pair` of the wall and each cap.
#[test]
fn through_hole_provenance_is_the_designed_table() {
    let (m, plate, hole, body, p) = cut_of("boolean/through-hole");
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    assert_eq!(m.faces(body).unwrap().len(), 7);

    // The plate: which faces are the caps (z = 0 and z = 10).
    let plate_faces = m.faces(plate).unwrap();
    let is_cap = |f: &FaceHandle| {
        let surface = m.surface(m.face(f.id).unwrap().surface()).unwrap();
        matches!(surface, Surface::Plane { frame } if frame.z().z.abs() > 0.5)
    };
    let (caps, sides): (Vec<&FaceHandle>, Vec<&FaceHandle>) =
        plate_faces.iter().partition(|f| is_cap(f));
    assert_eq!((caps.len(), sides.len()), (2, 4));
    for f in &caps {
        let pieces = p.modified_from(shape(f.id));
        assert_eq!(pieces.len(), 1, "{p}");
        let piece: FaceHandle = pieces[0].try_into().unwrap();
        assert_eq!(m.face(piece.id).unwrap().loops().len(), 2, "a hole loop");
        assert!(!p.is_deleted(shape(f.id)));
    }
    let out = m.closure(body).unwrap();
    for f in &sides {
        assert!(p.is_kept(shape(f.id), &m, body), "{p}");
        assert!(out.faces.contains(&f.id));
    }
    for e in m.edges(plate).unwrap() {
        assert!(p.is_kept(shape(e.id), &m, body), "{e} {p}");
    }
    for v in m.vertices(plate).unwrap() {
        assert!(p.is_kept(shape(v.id), &m, body), "{v} {p}");
    }

    // The tool: everything deleted; the wall and the seam with a piece
    // generated from each, the rest with none.
    let tool_faces = m.faces(hole).unwrap();
    let wall = tool_faces
        .iter()
        .find(|f| {
            matches!(
                m.surface(m.face(f.id).unwrap().surface()).unwrap(),
                Surface::Cylinder { .. }
            )
        })
        .unwrap();
    for f in &tool_faces {
        assert!(p.is_deleted(shape(f.id)), "{p}");
        let generated = p.generated_from(shape(f.id));
        if f.id == wall.id {
            let faces: Vec<_> = generated
                .iter()
                .filter(|s| matches!(s.id, EntityId::Face(_)))
                .collect();
            assert_eq!(faces.len(), 1, "the wall's piece\n{p}");
        } else {
            assert!(generated.is_empty(), "a cap generates nothing\n{p}");
        }
    }
    let seam = line_edges(&m, hole);
    assert_eq!(seam.len(), 1);
    for e in m.edges(hole).unwrap() {
        assert!(p.is_deleted(shape(e.id)), "{p}");
        let generated = p.generated_from(shape(e.id));
        if e.id == seam[0] {
            let edges: Vec<_> = generated
                .iter()
                .filter(|s| matches!(s.id, EntityId::Edge(_)))
                .collect();
            assert_eq!(edges.len(), 1, "the seam's middle piece\n{p}");
        } else {
            assert!(generated.is_empty(), "a rim generates nothing\n{p}");
        }
    }
    for v in m.vertices(hole).unwrap() {
        assert!(p.is_deleted(shape(v.id)) && p.generated_from(shape(v.id)).is_empty());
    }

    // Section vertices and edges.
    let section_vertices: Vec<Shape> = p
        .generated_from(shape(seam[0]))
        .iter()
        .copied()
        .filter(|s| matches!(s.id, EntityId::Vertex(_)))
        .collect();
    assert_eq!(section_vertices.len(), 2, "{p}");
    for cap in &caps {
        let from_cap = p.generated_from(shape(cap.id));
        let vertices: Vec<_> = from_cap
            .iter()
            .filter(|s| section_vertices.contains(s))
            .collect();
        assert_eq!(vertices.len(), 1, "one section vertex per cap\n{p}");
        let pair = p.generated_pair(shape(wall.id), shape(cap.id));
        let edges: Vec<_> = pair
            .iter()
            .filter(|s| matches!(s.id, EntityId::Edge(_)))
            .collect();
        assert_eq!(edges.len(), 1, "one section edge per cap\n{p}");
    }

    // The shell and the body: modified from the plate's, the tool's
    // deleted.
    assert_eq!(p.modified_from(Origin::Entity(shape(plate.id))).len(), 1);
    assert!(p.is_deleted(Shape::from(hole)));

    // Nothing else: the record has exactly these outputs.
    assert_eq!(p.outputs().len(), 10, "{p}");
}

/// `frame-cut` is `sample::frame` built the other way: the same counts,
/// the same mass properties to 1e-12.
#[test]
fn frame_cut_is_the_hand_built_frame() {
    let (m, _, _, body, _) = cut_of("boolean/frame-cut");
    let mut twin = Model::default();
    let frame = sample::frame(
        &mut twin,
        Point3::origin(),
        Point3::new(40.0, 30.0, 10.0),
        Point2::new(10.0, 10.0),
        Point2::new(30.0, 20.0),
    )
    .unwrap();
    let (a, b) = (
        check(&m, body, Level::Full).euler().unwrap(),
        check(&twin, frame, Level::Full).euler().unwrap(),
    );
    assert_eq!(a.to_string(), b.to_string());
    let (x, y) = (
        mass_properties(&m, body).unwrap(),
        mass_properties(&twin, frame).unwrap(),
    );
    assert!((x.volume - y.volume).abs() <= 1e-12 * y.volume);
    assert!((x.area - y.area).abs() <= 1e-12 * y.area);
    assert!((x.centroid - y.centroid).norm() <= 1e-12 * 40.0);
    for i in 0..3 {
        for j in 0..3 {
            assert!((x.inertia[(i, j)] - y.inertia[(i, j)]).abs() <= 1e-12 * y.inertia[(2, 2)]);
        }
    }
}

/// A tool clear of the target: the result is a new shell over the
/// target's own faces, and the tool is entirely `Deleted`.
#[test]
fn a_disjoint_cut_keeps_every_face_of_the_target() {
    let (m, plate, tool, body, p) = cut_of("boolean/disjoint-cut");
    assert_eq!(m.faces(body).unwrap(), m.faces(plate).unwrap());
    for f in m.faces(tool).unwrap() {
        assert!(p.is_deleted(shape(f.id)) && p.generated_from(shape(f.id)).is_empty());
    }
    assert_eq!(p.outputs().len(), 2, "the shell and the body\n{p}");
}

/// The target inside the tool selects nothing: the typed refusal, and the
/// model is as it was.
#[test]
fn a_swallowed_target_is_a_typed_refusal() {
    let (mut m, a, b) = inputs("boolean/swallow-cut");
    let before = arris_debug::dump_text(&m, a).unwrap();
    let faces = m.faces(a).unwrap().len();
    match cut(&mut m, a, b).unwrap_err() {
        OpError::Degenerate {
            reason: Reason::Boolean(BooleanReason::Empty),
            entities,
        } => assert_eq!(entities.as_slice(), [Shape::from(a), Shape::from(b)]),
        other => panic!("{other:?}"),
    }
    assert_eq!(arris_debug::dump_text(&m, a).unwrap(), before);
    assert_eq!(m.faces(a).unwrap().len(), faces);
}

/// A tool that does not resolve in the target's model — the wrong model,
/// `NotFound`'s own words — is named by its own id, not by the target's:
/// `OpError::NotFound` never substitutes an unrelated body for the one
/// that failed to resolve.
#[test]
fn a_boolean_over_a_body_that_does_not_resolve_still_names_that_body() {
    let mut m = Model::default();
    let (a, _) = primitive_box(&mut m, Point3::origin(), Point3::new(1.0, 1.0, 1.0)).unwrap();
    let mut other = Model::default();
    // A first body in `other` so `b`'s id is one `m` never assigned, not
    // one that happens to alias `a`'s.
    primitive_box(&mut other, Point3::origin(), Point3::new(1.0, 1.0, 1.0)).unwrap();
    let (b, _) = primitive_box(&mut other, Point3::origin(), Point3::new(1.0, 1.0, 1.0)).unwrap();
    match cut(&mut m, a, b).unwrap_err() {
        OpError::NotFound(id) => assert_eq!(id, AnyId::from(EntityId::from(b.id))),
        other => panic!("{other:?}"),
    }
}

/// Two boxes meeting along an edge fuse as two lumps with separate
/// copies of the edge and both endpoints (ADR-0090).
#[test]
fn boxes_touching_along_an_edge_give_two_lumps_with_provenance() {
    let (mut m, a, b) = inputs("boolean/edge-touching-fuse");
    let on_contact = |v| {
        let point = m.vertex(v).unwrap().point();
        point.x == 10.0 && point.y == 10.0
    };
    let input_edges = [a, b].map(|body| {
        m.edges(body)
            .unwrap()
            .into_iter()
            .find(|edge| {
                let edge = m.edge(edge.id).unwrap();
                on_contact(edge.start()) && on_contact(edge.end())
            })
            .unwrap()
            .id
    });
    let input_vertices = input_edges.map(|id| {
        let edge = m.edge(id).unwrap();
        [edge.start(), edge.end()]
    });

    let (body, p) = fuse(&mut m, a, b).unwrap();
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    assert_eq!(arris_check::lumps(&m, body).unwrap().len(), 2);
    arris_topo::provenance::audit(&m, &[a, b], body, &p).unwrap();
    let copies = p.modified_from(shape(input_edges[0]));
    assert_eq!(copies.len(), 2);
    assert_eq!(copies, p.modified_from(shape(input_edges[1])));

    let edge_vertices: Vec<_> = copies
        .iter()
        .map(|copy| {
            let EntityId::Edge(edge) = copy.id else {
                panic!("expected a contact edge, got {copy:?}");
            };
            let edge = m.edge(edge).unwrap();
            [edge.start(), edge.end()]
        })
        .collect();
    for input in input_vertices.into_iter().flatten() {
        let point = m.vertex(input).unwrap().point();
        let images = p.modified_from(shape(input));
        assert_eq!(images.len(), 2);
        for ends in &edge_vertices {
            assert_eq!(
                ends.iter()
                    .filter(|&&v| m.vertex(v).unwrap().point() == point)
                    .filter(|&&v| images.contains(&shape(v)))
                    .count(),
                1
            );
        }
    }
}

fn polygon_prism(m: &mut Model, xy: &[[f64; 2]], base: f64, length: f64) -> Body {
    let profile = Profile {
        plane: Frame::new(Point3::new(0.0, 0.0, base), Vec3::z(), Vec3::x()).unwrap(),
        outer: ProfileLoop::Path {
            start: Point2::from(xy[0]),
            segments: xy[1..]
                .iter()
                .chain(&xy[..1])
                .map(|&p| ProfileSegment::LineTo(Point2::from(p)))
                .collect(),
        },
        holes: Vec::new(),
    };
    extrude(m, &profile, Vec3::z(), length).unwrap().0
}

/// Three wedges meet on the z axis. The second wedge's edge represents
/// the third's in a common block and has its own block with the first.
/// Open CASCADE (cadquery-ocp 8.0.1.0.0) returns three solids sharing
/// the axis edge, with total volume 70.
#[test]
fn a_common_block_can_use_another_blocks_source_as_representative() {
    let mut m = Model::default();
    let wedges = [
        [[0.0, 0.0], [4.0, 0.0], [4.0, 2.0]],
        [[0.0, 0.0], [-4.0, 1.0], [-4.0, 0.0]],
        [[0.0, 0.0], [4.0, -1.0], [4.0, -0.5]],
    ]
    .map(|xy| polygon_prism(&mut m, &xy, 0.0, 10.0));
    for order in [wedges, [wedges[2], wedges[1], wedges[0]]] {
        let pave = interferences_many(&m, &order).unwrap();
        assert_eq!(pave.blocks.len(), 2, "{pave}");
        assert_eq!(pave.blocks[0].b, pave.blocks[1].a, "{pave}");
    }
}

/// A triangular void reaches the reentrant edge of an L-shaped prism.
/// Open CASCADE (cadquery-ocp 8.0.1.0.0) returns one solid of volume 2976
/// with two shells and one shared edge, (0, 0, 2) to (0, 0, 8), used by
/// four faces. ADR-0090 refuses this contact within one lump.
#[test]
fn a_void_touching_the_outer_shell_at_a_shared_edge_names_the_edge() {
    let mut m = Model::default();
    let target = polygon_prism(
        &mut m,
        &[
            [-10.0, -10.0],
            [10.0, -10.0],
            [10.0, 0.0],
            [0.0, 0.0],
            [0.0, 10.0],
            [-10.0, 10.0],
        ],
        0.0,
        10.0,
    );
    let tool = polygon_prism(&mut m, &[[0.0, 0.0], [-4.0, 0.0], [-2.0, -2.0]], 2.0, 6.0);

    let pave = interferences(&m, target, tool).unwrap();
    assert_eq!(pave.blocks.len(), 1, "{pave}");
    let contact = pave.blocks[0].a.0;
    let edge = m.edge(contact).unwrap();
    let [a, b] = [edge.start(), edge.end()].map(|v| m.vertex(v).unwrap().point());
    assert_eq!([a.x, a.y, b.x, b.y], [0.0; 4]);
    assert_eq!([a.z.min(b.z), a.z.max(b.z)], [0.0, 10.0]);
    match cut(&mut m, target, tool).unwrap_err() {
        OpError::Degenerate {
            reason: Reason::Input(InputReason::NonManifold),
            entities,
        } => assert_eq!(entities, [shape(contact)]),
        other => panic!("{other:?}"),
    }
}

/// A bridge that only covers part of the contact names both copied
/// operand edges, including when the result splits them.
#[test]
fn a_partial_bridge_names_the_two_inherited_contact_edges() {
    let (mut m, touching, bridge) = inputs("boolean/edge-touching-exposed-join");
    let mut copies: Vec<_> = touching_box_edge_copies(&m, touching)
        .into_iter()
        .map(shape)
        .collect();
    copies.sort_unstable();
    assert_eq!(copies.len(), 2);
    match fuse(&mut m, touching, bridge).unwrap_err() {
        OpError::Degenerate {
            reason: Reason::Input(InputReason::NonManifold),
            entities,
        } => assert_eq!(entities, copies),
        other => panic!("{other:?}"),
    }
}

/// Joining two shells remotely must refuse their unjoined corners within
/// vertex tolerance. Assemble the input from already separate shells so
/// their corners keep their distinct positions and ids.
#[test]
fn a_remote_bridge_refuses_inherited_corners_within_vertex_tolerance() {
    let mut m = Model::default();
    let corner = Point3::new(10.0, 10.0, 10.0);
    let separated = corner + Vec3::repeat(m.precision().default_tolerance / 2.0);
    let (a, _) = primitive_box(&mut m, Point3::origin(), corner).unwrap();
    let (b, _) = primitive_box(&mut m, separated, Point3::new(20.0, 20.0, 20.0)).unwrap();
    let mut corners = [(a, corner), (b, separated)].map(|(body, point)| {
        m.vertices(body)
            .unwrap()
            .into_iter()
            .find(|v| m.vertex(v.id).unwrap().point() == point)
            .map(|v| shape(v.id))
            .unwrap()
    });
    corners.sort_unstable();
    let shells = [a, b]
        .into_iter()
        .flat_map(|body| m.shells(body).unwrap())
        .collect();
    let body = Body::forward(m.raw().add_body(arris_topo::entity::Body::solid(shells)));
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    assert_eq!(lumps(&m, body).unwrap().len(), 2);
    let before = arris_debug::dump_text(&m, body).unwrap();

    let (first, _) = primitive_box(&mut m, [4.0, 4.0, 9.0], [6.0, 6.0, 16.0]).unwrap();
    let (second, _) = primitive_box(&mut m, [5.0, 5.0, 14.0], [12.0, 12.0, 16.0]).unwrap();
    match fuse_many(&mut m, &[body, first, second]).unwrap_err() {
        OpError::Degenerate {
            reason: Reason::Input(InputReason::NonManifold),
            entities,
        } => assert_eq!(entities, corners),
        other => panic!("{other:?}"),
    }
    assert_eq!(arris_debug::dump_text(&m, body).unwrap(), before);
}

/// A V4 pair already present in an operand shell can survive a disjoint
/// fuse with its points, tolerances and vertex ids intact.
#[test]
fn a_disjoint_fuse_preserves_a_preexisting_pair_within_vertex_tolerance() {
    use arris_topo::builder::{Assembly, Builder, KeepGeometry, VertexSpec};
    use arris_topo::entity::BodyKind;

    let mut m = Model::default();
    let precision = m.precision();
    let width = precision.max_tolerance * 0.4;
    let (prism, _) = primitive_box(&mut m, [0.0; 3], [width, width, 10.0]).unwrap();
    let (mut assembly, _) = Assembly::of_body(&mut m, prism, &mut KeepGeometry).unwrap();
    for vertex in &mut assembly.vertices {
        if let VertexSpec::New { point, tolerance } = vertex
            && *point == Point3::origin()
        {
            *tolerance = precision.max_tolerance;
        }
    }
    let (builder, _) = Builder::assemble(&m, precision.default_tolerance, assembly).unwrap();
    let a = builder.finish(&mut m, BodyKind::Solid).unwrap().body;
    let report = check(&m, a, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    let pairs = arris_check::coincident_vertices(&m, a).unwrap();
    assert_eq!(pairs.len(), 1);

    let (b, _) = primitive_box(&mut m, [100.0; 3], [110.0; 3]).unwrap();
    let (joined, provenance) = fuse(&mut m, a, b).unwrap();
    assert_eq!(lumps(&m, joined).unwrap().len(), 2);
    assert_eq!(arris_check::coincident_vertices(&m, joined).unwrap(), pairs);
    for vertex in pairs.into_iter().flatten() {
        assert!(provenance.is_kept(shape(vertex), &m, joined));
    }
    let report = check(&m, joined, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
}

/// The full bridge drops both copies of the buried edge and its endpoints.
#[test]
fn a_full_bridge_drops_the_inherited_contact_entities() {
    let (mut m, touching, bridge) = inputs("boolean/edge-touching-covered-join");
    let copies = touching_box_edge_copies(&m, touching);
    assert_eq!(copies.len(), 2);
    let (joined, provenance) = fuse(&mut m, touching, bridge).unwrap();
    assert_eq!(lumps(&m, joined).unwrap().len(), 1);
    for id in copies {
        let edge = m.edge(id).unwrap();
        assert!(provenance.is_deleted(shape(id)));
        assert!(provenance.is_deleted(shape(edge.start())));
        assert!(provenance.is_deleted(shape(edge.end())));
    }
}

/// The two original edges of the touching box recipes, at x = y = 10.
fn touching_box_edge_copies(m: &Model, body: Body) -> Vec<EdgeId> {
    m.edges(body)
        .unwrap()
        .into_iter()
        .filter(|edge| {
            let edge = m.edge(edge.id).unwrap();
            [edge.start(), edge.end()].into_iter().all(|id| {
                let point = m.vertex(id).unwrap().point();
                point.x == 10.0 && point.y == 10.0
            })
        })
        .map(|edge| edge.id)
        .collect()
}

/// The two cubes retain their own copies of the touching corner.
#[test]
fn boxes_touching_at_a_corner_give_two_lumps_with_provenance() {
    let corner = Point3::new(10.0, 10.0, 10.0);
    let mut m = Model::default();
    let (a, _) = primitive_box(&mut m, Point3::origin(), corner).unwrap();
    let (b, _) = primitive_box(&mut m, corner, Point3::new(20.0, 20.0, 20.0)).unwrap();
    let source_vertices = [a, b].map(|body| {
        m.vertices(body)
            .unwrap()
            .into_iter()
            .find(|v| m.vertex(v.id).unwrap().point() == corner)
            .unwrap()
            .id
    });
    let before = arris_debug::dump_text(&m, a).unwrap();
    let (body, p) = fuse(&mut m, a, b).unwrap();
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    assert_eq!(lumps(&m, body).unwrap().len(), 2);
    let volume = mass_properties(&m, body).unwrap().volume;
    assert!((volume - 2000.0).abs() < 1e-9 * 2000.0, "{volume}");
    arris_topo::provenance::audit(&m, &[a, b], body, &p).unwrap();
    let copies: Vec<_> = m
        .vertices(body)
        .unwrap()
        .into_iter()
        .filter(|v| m.vertex(v.id).unwrap().point() == corner)
        .map(|v| shape(v.id))
        .collect();
    assert_eq!(copies.len(), 2);
    for source in source_vertices {
        let images = p.modified_from(shape(source));
        assert_eq!(images.len(), 2);
        assert!(copies.iter().all(|copy| images.contains(copy)));
    }
    assert_eq!(arris_debug::dump_text(&m, a).unwrap(), before);
}

/// A tool that splits its target leaves two lumps of one solid
/// (ADR-0006): two shells, clean at `Full`, the plate's volume less the
/// slab's, the plate's shell `Modified` into both result shells and the
/// slab's `Deleted` with nothing generated from it.
#[test]
fn a_split_target_is_two_lumps_of_one_solid() {
    let (m, plate, slab, body, p) = cut_of("boolean/split-cut");
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    let shells = m.shells(body).unwrap();
    assert_eq!(shells.len(), 2);
    assert_eq!(lumps(&m, body).unwrap().len(), 2);
    let plate_shell = shape(m.shells(plate).unwrap()[0].id);
    let result_shells: Vec<Shape> = shells.iter().map(|s| shape(s.id)).collect();
    assert_eq!(p.modified_from(plate_shell), result_shells.as_slice());
    let slab_shell = shape(m.shells(slab).unwrap()[0].id);
    assert!(p.is_deleted(slab_shell) && p.generated_from(slab_shell).is_empty());
    let volume = mass_properties(&m, body).unwrap().volume;
    assert!((volume - 10800.0).abs() < 1e-9 * 10800.0, "{volume}");
}

/// Two runs of every cut fixture give the same dump.
#[test]
fn two_cuts_are_identical() {
    for name in [
        "boolean/through-hole",
        "boolean/blind-hole",
        "boolean/frame-cut",
        "boolean/corner-cut",
        "boolean/disjoint-cut",
    ] {
        let (m1, _, _, b1, p1) = cut_of(name);
        let (m2, _, _, b2, p2) = cut_of(name);
        assert_eq!(
            arris_debug::dump_text(&m1, b1).unwrap(),
            arris_debug::dump_text(&m2, b2).unwrap(),
            "{name}"
        );
        assert_eq!(p1, p2, "{name}");
    }
}

// -- coincident faces (plan step 10) ----------------------------------

/// The pave model of two flush boxes: every face pair on the shared
/// plane is `Coincident`, the four edges around the shared face are
/// common blocks of the pair, nothing is a section edge, and no image
/// splits anything — the two faces are the same region.
#[test]
fn flush_boxes_share_their_rim_as_common_blocks() {
    let (m, a, b, i) = interferences_of("boolean/flush-union");
    let coincident: Vec<_> = i
        .pairs
        .iter()
        .filter(|p| p.intersection == SurfaceIntersection::Coincident)
        .collect();
    // x = 40 with x = 40, and the four side faces of A with the four
    // coplanar side faces of B.
    assert_eq!(coincident.len(), 5, "{i}");
    assert!(i.sections.is_empty(), "{i}");
    assert!(i.images.is_empty(), "{i}");
    assert_eq!(i.blocks.len(), 4, "{i}");
    for block in &i.blocks {
        assert!(m.edges(a).unwrap().iter().any(|e| e.id == block.a.0));
        assert!(m.edges(b).unwrap().iter().any(|e| e.id == block.b.0));
        assert_eq!(block.a.1, 0);
        assert_eq!(block.b.1, 0);
        // Each of B's rim edges is used by two faces of B.
        assert_eq!(block.pcurves.len(), 2, "{i}");
    }
    // Every corner of the shared face is one section vertex that merges
    // a vertex of each operand.
    assert_eq!(i.vertices.len(), 4, "{i}");
    for v in &i.vertices {
        assert_eq!(v.existing.len(), 2, "{i}");
    }
}

/// The flush union holds the shared face's rim once: 12 vertices, 20
/// edges and 10 faces, every edge used twice, and the provenance names
/// each of B's rim edges `Modified` into A's.
#[test]
fn a_flush_union_holds_the_rim_once() {
    let (m, a, b, body, p) = boolean_of("boolean/flush-union", fuse);
    assert_eq!(m.faces(body).unwrap().len(), 10);
    assert_eq!(m.edges(body).unwrap().len(), 20);
    let a_edges: Vec<EdgeId> = m.edges(a).unwrap().iter().map(|e| e.id).collect();
    let b_edges: Vec<EdgeId> = m.edges(b).unwrap().iter().map(|e| e.id).collect();
    let mut merged = 0;
    for &e in &b_edges {
        let images = p.modified_from(Shape::new(e, Orientation::Forward));
        if let [image] = images {
            if let EntityId::Edge(target) = image.id {
                if a_edges.contains(&target) {
                    merged += 1;
                }
            }
        }
    }
    assert_eq!(merged, 4, "{p:?}");
    // The shared faces are gone, one from each operand.
    let deleted = |body: Body| {
        m.faces(body)
            .unwrap()
            .iter()
            .filter(|f| p.is_deleted(Shape::new(f.id, Orientation::Forward)))
            .count()
    };
    assert_eq!((deleted(a), deleted(b)), (1, 1));
}

/// The common of two solids that share only a face has no thickness:
/// the typed refusal, by name, and the model untouched.
#[test]
fn a_flush_common_has_no_thickness() {
    let (mut m, a, b) = inputs("boolean/flush-common");
    let before = arris_debug::dump_text(&m, a).unwrap();
    match common(&mut m, a, b) {
        Err(OpError::Degenerate {
            reason: Reason::Input(InputReason::ZeroThickness),
            entities,
        }) => assert_eq!(entities.len(), 2),
        other => panic!("{other:?}"),
    }
    assert_eq!(arris_debug::dump_text(&m, a).unwrap(), before);
}

/// The boss whose bottom cap lies on the plate's top: the plate's top
/// is split by the rim into the disc, `On` the cap and dropped, and the
/// rest, kept; the cap is `Deleted`; the rim is the wall's own edge,
/// used by the wall (kept by id) and by the new top face.
#[test]
fn a_flush_boss_keeps_the_rim_as_the_walls_edge() {
    let (m, a, b, body, p) = boolean_of("boolean/boss-flush", fuse);
    assert_eq!(m.faces(body).unwrap().len(), 8);
    let top = m
        .faces(a)
        .unwrap()
        .into_iter()
        .find(|f| {
            let face = m.face(f.id).unwrap();
            let Surface::Plane { frame } = m.surface(face.surface()).unwrap() else {
                return false;
            };
            frame.origin().z == 10.0
        })
        .unwrap();
    let images = p.modified_from(Shape::new(top.id, Orientation::Forward));
    assert_eq!(images.len(), 1, "the top face is one piece: {p:?}");
    let EntityId::Face(new_top) = images[0].id else {
        panic!()
    };
    assert_eq!(m.face(new_top).unwrap().loops().len(), 2);
    let b_faces = m.faces(b).unwrap();
    let cap = b_faces
        .iter()
        .find(|f| p.is_deleted(Shape::new(f.id, Orientation::Forward)))
        .expect("the cap is deleted");
    assert!(
        p.generated_from(Shape::new(cap.id, Orientation::Forward))
            .is_empty()
    );
    // The wall and the top cap are kept by id: 8 faces, 7 of them named
    // in no record.
    let kept = b_faces
        .iter()
        .filter(|f| !p.is_deleted(Shape::new(f.id, Orientation::Forward)))
        .count();
    assert_eq!(kept, 2);
    // The rim edge of the result is B's own.
    let rim = m
        .edges(body)
        .unwrap()
        .into_iter()
        .filter(|e| {
            let edge = m.edge(e.id).unwrap();
            edge.curve().is_some_and(|(c, _)| {
                matches!(m.curve(c).unwrap(), Curve::Circle { frame, .. } if frame.origin().z == 10.0)
            })
        })
        .count();
    assert_eq!(rim, 1);
    // The new top face, the shell and the body: nothing else is named.
    assert_eq!(p.outputs().len(), 3, "{p:?}");
}

/// The tool touching the target along a face from outside: `cut` keeps
/// the target's piece under the tool (the normals oppose) and the rest,
/// so the target is whole, its top split in two along the tool's rim —
/// a new edge `Generated` from the tool's.
#[test]
fn a_cut_by_a_flush_tool_splits_the_touched_face() {
    let (m, a, _, body, p) = boolean_of("boolean/boss-flush", cut);
    assert_eq!(m.faces(body).unwrap().len(), 7);
    let volume = mass_properties(&m, body).unwrap().volume;
    assert!((volume - 12000.0).abs() < 1e-9 * 12000.0, "{volume}");
    let top = m
        .faces(a)
        .unwrap()
        .into_iter()
        .find(|f| {
            let face = m.face(f.id).unwrap();
            let Surface::Plane { frame } = m.surface(face.surface()).unwrap() else {
                return false;
            };
            frame.origin().z == 10.0
        })
        .unwrap();
    let images = p.modified_from(Shape::new(top.id, Orientation::Forward));
    assert_eq!(images.len(), 2, "{p:?}");
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
}

/// The rod in the tube: the two walls vanish, the inner circles are
/// common blocks of a periodic edge — the tube's section circles and
/// the rod's own rims, held once — as is the seam the two walls share,
/// and the result is five faces.
#[test]
fn a_rod_in_a_tube_holds_the_inner_circles_once() {
    let (m, _, b, i) = interferences_of("boolean/coaxial-fuse");
    let walls: Vec<_> = i
        .pairs
        .iter()
        .filter(|p| p.intersection == SurfaceIntersection::Coincident)
        .collect();
    // The bore wall with the rod's wall, and each annulus with a disc.
    assert_eq!(walls.len(), 3, "{i}");
    assert_eq!(i.blocks.len(), 3, "{i}");
    for block in &i.blocks {
        assert!(m.edges(b).unwrap().iter().any(|e| e.id == block.b.0));
        assert!(!block.reversed, "{i}");
    }
    let (m, _, _, body, _) = boolean_of("boolean/coaxial-fuse", fuse);
    assert_eq!(m.faces(body).unwrap().len(), 5);
    assert_eq!(m.edges(body).unwrap().len(), 5);
}

// -- `fuse` and `common` (plan step 8) --------------------------------

/// A boolean of two bodies: `fuse`, `common` or `cut`.
type Boolean = fn(&mut Model, Body, Body) -> Result<(Body, Provenance), OpError>;

/// The fixture's operands fused or intersected, the model with them.
fn boolean_of(name: &str, op: Boolean) -> (Model, Body, Body, Body, Provenance) {
    let (mut m, a, b) = inputs(name);
    let (body, p) = op(&mut m, a, b).unwrap();
    (m, a, b, body, p)
}

/// A `fuse` reuses both operands: the boss's top cap is untouched and
/// keeps its id, its wall is `Modified` into the piece above the plate,
/// its bottom cap is swallowed and `Deleted`, the plate's top face is
/// `Modified` into one piece with a hole loop, and the four sides and
/// the bottom are kept and unrecorded.
#[test]
fn a_boss_keeps_what_neither_operand_touched() {
    let (m, plate, boss, body, p) = boolean_of("boolean/boss", fuse);
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    let faces = m.faces(body).unwrap();
    assert_eq!(faces.len(), 8);

    // The tool's top cap survives whole, with its own id: the `cut`
    // rule that nothing of the tool is kept is `cut`'s alone.
    let cap = m
        .faces(boss)
        .unwrap()
        .into_iter()
        .find(|f| {
            let e = m.face(f.id).unwrap();
            matches!(m.surface(e.surface()).unwrap().kind(), SurfaceKind::Plane)
                && p.origins(shape(f.id)).is_empty()
                && !p.is_deleted(shape(f.id))
        })
        .expect("one cap of the boss is untouched");
    assert!(faces.iter().any(|f| f.id == cap.id));

    // The other cap is inside the plate: nothing of it survives.
    let swallowed: Vec<FaceHandle> = m
        .faces(boss)
        .unwrap()
        .into_iter()
        .filter(|f| p.is_deleted(shape(f.id)))
        .collect();
    assert_eq!(swallowed.len(), 1, "{p}");

    // The plate's top face becomes one piece with a hole in it; the
    // wall becomes the piece above the plate.
    let top = m
        .faces(plate)
        .unwrap()
        .into_iter()
        .find(|f| p.modified_from(Origin::Entity(shape(f.id))).len() == 1)
        .expect("the plate's top face");
    let image = p.modified_from(Origin::Entity(shape(top.id)))[0];
    let EntityId::Face(image) = image.id else {
        panic!("a face is modified into a face");
    };
    assert_eq!(m.face(image).unwrap().loops().len(), 2);

    // Both operands' shells and bodies are `Modified` into the result's.
    for b in [plate, boss] {
        assert_eq!(p.modified_from(Origin::Entity(Shape::from(b))).len(), 1);
    }
}

/// Two cubes overlapping at a corner: their common is the unit cube —
/// six faces, volume 1, centroid at its middle.
#[test]
fn the_common_of_the_corner_cubes_is_the_unit_cube() {
    let (m, _, _, body, _) = boolean_of("boolean/corner-common", common);
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    assert_eq!(m.faces(body).unwrap().len(), 6);
    let p = mass_properties(&m, body).unwrap();
    assert!((p.volume - 1.0).abs() <= 1e-12);
    assert!((p.area - 6.0).abs() <= 1e-12);
    assert!((p.centroid - Point3::new(0.5, 0.5, 0.5)).norm() <= 1e-12);
}

/// Operands that do not touch: their common holds no material, the typed
/// refusal with the model as it was; their fuse is two lumps of one solid
/// (ADR-0006), every face of both kept by id and each operand's shell
/// `Modified` into the result shell of its own faces.
#[test]
fn disjoint_operands_are_empty_in_common_and_two_lumps_in_fuse() {
    let (mut m, a, b) = inputs("boolean/disjoint-common");
    let before = arris_debug::dump_text(&m, a).unwrap();
    match common(&mut m, a, b).unwrap_err() {
        OpError::Degenerate {
            reason: Reason::Boolean(BooleanReason::Empty),
            entities,
        } => assert_eq!(entities.as_slice(), [Shape::from(a), Shape::from(b)]),
        other => panic!("{other:?}"),
    }
    assert_eq!(arris_debug::dump_text(&m, a).unwrap(), before);

    let (body, p) = fuse(&mut m, a, b).unwrap();
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    let mut faces = m.faces(a).unwrap();
    faces.extend(m.faces(b).unwrap());
    assert_eq!(m.faces(body).unwrap(), faces, "every face kept, in order");
    let shells = m.shells(body).unwrap();
    assert_eq!(shells.len(), 2);
    for (operand, out) in [a, b].into_iter().zip(&shells) {
        let own = shape(m.shells(operand).unwrap()[0].id);
        assert_eq!(p.modified_from(own), [shape(out.id)]);
    }
}

/// Every number of a dump — an id's index included — replaced by `#`:
/// what is left is its shape, the entities and their order with the
/// orientation each is used in.
fn without_numbers(dump: &str) -> String {
    let c: Vec<char> = dump.chars().collect();
    let mut out = String::with_capacity(dump.len());
    let mut i = 0;
    while i < c.len() {
        let number = c[i].is_ascii_digit()
            || (c[i] == '-' && c.get(i + 1).is_some_and(char::is_ascii_digit));
        if !number {
            out.push(c[i]);
            i += 1;
            continue;
        }
        out.push('#');
        i += usize::from(c[i] == '-');
        while c.get(i).is_some_and(|x| x.is_ascii_digit() || *x == '.') {
            i += 1;
        }
        if c.get(i) == Some(&'e') {
            let mut j = i + 1;
            j += usize::from(c.get(j).is_some_and(|x| *x == '-' || *x == '+'));
            if c.get(j).is_some_and(char::is_ascii_digit) {
                i = j;
                while c.get(i).is_some_and(char::is_ascii_digit) {
                    i += 1;
                }
            }
        }
    }
    out
}

/// A rigid motion of both operands moves the result and nothing else:
/// `posed-through-hole`'s dump is `through-hole`'s with other numbers in
/// it — the same entities with the same ids in the same order, the same
/// provenance, and the same mass properties up to the motion.
#[test]
fn a_posed_through_hole_is_the_through_hole_moved() {
    let (m, _, _, plain, plain_p) = cut_of("boolean/through-hole");
    let (posed_m, _, _, posed, posed_p) = cut_of("boolean/posed-through-hole");
    let plain_dump = arris_debug::dump_text(&m, plain).unwrap();
    let posed_dump = arris_debug::dump_text(&posed_m, posed).unwrap();
    assert_ne!(plain_dump, posed_dump);
    assert_eq!(without_numbers(&plain_dump), without_numbers(&posed_dump));
    assert_eq!(plain_p.outputs().len(), posed_p.outputs().len());
    let (x, y) = (
        mass_properties(&m, plain).unwrap(),
        mass_properties(&posed_m, posed).unwrap(),
    );
    assert!((x.volume - y.volume).abs() <= 1e-9 * x.volume);
    assert!((x.area - y.area).abs() <= 1e-9 * x.area);
}

/// The tangent case, all three selections (plan step 11). A cylinder
/// touching the plate's side face from outside: `cut` is the plate with
/// every id kept — the interior point of the touched face lies on the
/// ruling, classified `On` the wall, and the curvature rule puts the
/// face outside the rod; `common` selects nothing, `BooleanReason::Empty` and
/// not `ZeroThickness`, since no piece lay *on* the other operand in the
/// coincident sense; `fuse` keeps the face and the wall touching along
/// the contact, two lumps of one solid that touch (ADR-0088), each keeping
/// its own faces. And the blind hole whose wall touches a side face from
/// inside refuses with `TangentContact` through `cut`, before any face is
/// split: one shell would touch itself.
#[test]
fn a_touch_from_outside_is_the_plate_and_a_touch_in_one_shell_is_refused_by_name() {
    let (mut m, plate, post) = inputs("boolean/tangent-outside-cut");
    let before = arris_debug::dump_text(&m, plate).unwrap();
    let (body, p) = cut(&mut m, plate, post).unwrap();
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    assert_eq!(m.faces(body).unwrap().len(), 6);
    assert_eq!(m.edges(body).unwrap().len(), 12);
    let plate_faces: Vec<_> = m.faces(plate).unwrap().iter().map(|f| f.id).collect();
    let kept: Vec<_> = m.faces(body).unwrap().iter().map(|f| f.id).collect();
    assert_eq!(kept, plate_faces, "every face of the plate is kept by id");
    assert!(
        p.outputs()
            .iter()
            .all(|o| o.id != EntityId::Face(plate_faces[0])),
        "an untouched face is unrecorded"
    );
    assert_eq!(arris_debug::dump_text(&m, plate).unwrap(), before);

    let (mut m, plate, post) = inputs("boolean/tangent-outside-cut");
    match common(&mut m, plate, post) {
        Err(OpError::Degenerate {
            reason: Reason::Boolean(BooleanReason::Empty),
            ..
        }) => {}
        other => panic!("common: {other:?}"),
    }
    let (plate_volume, post_volume) = (
        mass_properties(&m, plate).unwrap().volume,
        mass_properties(&m, post).unwrap().volume,
    );
    let (fused, _) = fuse(&mut m, plate, post).unwrap();
    let report = check(&m, fused, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    assert_eq!(m.shells(fused).unwrap().len(), 2, "two lumps that touch");
    let volume = mass_properties(&m, fused).unwrap().volume;
    assert!(
        (volume - plate_volume - post_volume).abs() <= 1e-9 * volume,
        "the volumes add: {volume} vs {plate_volume} + {post_volume}"
    );
    let plate_faces: Vec<_> = m.faces(plate).unwrap().iter().map(|f| f.id).collect();
    let kept: Vec<_> = m.faces(fused).unwrap().iter().map(|f| f.id).collect();
    assert!(
        plate_faces.iter().all(|f| kept.contains(f)),
        "every face of the plate is kept by id"
    );

    let (mut m, plate, hole) = inputs("boolean/tangent-hole");
    let before = arris_debug::dump_text(&m, plate).unwrap();
    match cut(&mut m, plate, hole) {
        Err(OpError::Degenerate {
            reason: Reason::Boolean(BooleanReason::TangentContact),
            ..
        }) => {}
        other => panic!("tangent-hole: {other:?}"),
    }
    assert_eq!(
        arris_debug::dump_text(&m, plate).unwrap(),
        before,
        "the model is as it was"
    );
}

/// The degenerate edge of `body` on the vertex section vertex `k` of `i`
/// stands over, and its paves.
fn degenerate_paves(m: &Model, body: Body, i: &Interferences, k: usize) -> Vec<f64> {
    let over = i.vertices[k].existing[0];
    let degenerate: Vec<EdgeId> = (m.edges(body).unwrap().into_iter())
        .filter(|e| {
            let edge = m.edge(e.id).unwrap();
            edge.is_degenerate() && edge.start() == over
        })
        .map(|e| e.id)
        .collect();
    let [degenerate] = degenerate[..] else {
        panic!("{degenerate:?}");
    };
    let paves = i.paves.get(&degenerate).map_or(&[][..], |p| &p[..]);
    assert!(paves.iter().all(|p| p.vertex == k), "{i}");
    paves.iter().map(|p| p.t).collect()
}

/// A plane through a cone's apex (ADR-0021): the section vertex there is
/// the operand's own — the seam ends on the apex and pierces the plane at
/// it, an ordinary hit on a vertex — both rulings end on it, and the
/// apex's degenerate edge, which has no curve and is hit by nothing, is
/// paved once for each `u` a ruling arrives with: the nodes the cone
/// face's arrangement needs, one vertex standing for a whole line of
/// (u, v).
#[test]
fn a_degenerate_edge_is_paved_where_a_section_arrives_at_its_vertex() {
    let (m, a, _, i) = interferences_of("boolean/cone-apex-slice-cut");
    let apex = (i.vertices.iter())
        .position(|v| v.point == Point3::new(0.0, 0.0, 2.0))
        .unwrap_or_else(|| panic!("{i}"));
    assert_eq!(i.vertices[apex].existing.len(), 1, "{i}");
    let ending: Vec<_> = (i.sections.iter())
        .filter(|s| s.start == apex || s.end == apex)
        .collect();
    assert_eq!(ending.len(), 2, "{i}");
    for s in ending {
        assert!(matches!(i.curves[s.curve].curve, Curve::Line { .. }), "{i}");
    }
    let paves = degenerate_paves(&m, a, &i, apex);
    assert_eq!(paves.len(), 2, "{i}");
    assert!(paves[0] < paves[1], "{i}");
}

/// The same through a ball's pole with the face turned about the seam's
/// own tangent there: the seam only touches the face's plane at the
/// pole, and a touch makes no vertex, so nothing an edge does puts one
/// on the circle — the face's singular vertex does, its source
/// `Singular`, the touch joining it. The circle is one block from the
/// pole to the pole; it leaves along the seam's meridian, a corner of the
/// (u, v) box the degenerate edge already ends on, and comes back half a
/// turn from it, the one pave.
#[test]
fn a_singular_vertex_paves_a_section_no_edge_pierces_at_it() {
    let dir = fixtures::corpus_root().join("boolean/ball-pole-slice-cut");
    let inputs = corpus::inputs(&dir, "along_seam").unwrap();
    let (a, b) = inputs.operands().unwrap();
    let m = inputs.model;
    let i = interferences(&m, a, b).unwrap();
    let singular: Vec<usize> = (0..i.vertices.len())
        .filter(|&k| i.vertices[k].source == VertexSource::Singular)
        .collect();
    let [pole] = singular[..] else {
        panic!("{i}");
    };
    assert_eq!(i.vertices[pole].point, Point3::new(0.0, 0.0, 2.0), "{i}");
    assert_eq!(i.vertices[pole].existing.len(), 1, "{i}");
    assert!(
        i.vertices[pole].hits.iter().all(|&h| i.hits[h].tangent),
        "{i}"
    );
    let [s] = &i.sections[..] else {
        panic!("{i}");
    };
    assert_eq!((s.start, s.end), (pole, pole), "{i}");
    assert!((s.range.length() - TAU).abs() < 1e-12, "{i}");
    let paves = degenerate_paves(&m, a, &i, pole);
    assert_eq!(paves.len(), 1, "{i}");
    assert!((paves[0] - TAU / 2.0).abs() < 1e-9, "{i}");
}

/// A face a hair off a ball's pole: the section neither runs through the
/// singular point nor stays clear of it by what the sphere's (u, v)
/// resolves, and the boolean refuses it by name — the sphere's face, the
/// block's and the pole's vertex — before anything is fitted, the model
/// as it was (ADR-0021, `regression/ball-beside-pole-slice-cut`).
#[test]
fn a_section_beside_a_pole_is_refused_by_name() {
    let (mut m, ball, half) = inputs("regression/ball-beside-pole-slice-cut");
    let before = arris_debug::dump_text(&m, ball).unwrap();
    let pole = (m.closure(ball).unwrap().vertices.iter().copied())
        .find(|&v| m.vertex(v).unwrap().point().z > 0.0)
        .unwrap();
    for op in [cut, common, fuse] {
        match op(&mut m, ball, half).unwrap_err() {
            OpError::Degenerate {
                reason: Reason::Boolean(BooleanReason::BesideSingularity),
                entities,
            } => {
                let ball_faces: Vec<_> = m.faces(ball).unwrap().iter().map(|f| f.id).collect();
                let half_faces: Vec<_> = m.faces(half).unwrap().iter().map(|f| f.id).collect();
                assert_eq!(entities.len(), 3, "{entities:?}");
                assert!(matches!(entities[0].id, EntityId::Face(f) if ball_faces.contains(&f)));
                assert!(matches!(entities[1].id, EntityId::Face(f) if half_faces.contains(&f)));
                assert_eq!(entities[2].id, EntityId::Vertex(pole));
            }
            other => panic!("{other:?}"),
        }
    }
    assert_eq!(
        arris_debug::dump_text(&m, ball).unwrap(),
        before,
        "the model is as it was"
    );
}

/// A drill touching the main wall from inside at a singular point of
/// their section leaves the wall two pieces meeting only at that vertex,
/// pinched between the drill's exits: one shell touching itself at a
/// point, which a manifold `Solid` does not hold. The cut is
/// `InputReason::NonManifold` naming the vertex by the two faces whose section
/// made it — it was `Internal`, the builder unable to close the shell —
/// and the model is as it was (`regression/singular-bore-cut`).
#[test]
fn a_wall_pinched_at_a_singular_point_is_non_manifold() {
    let (mut m, main, drill) = inputs("regression/singular-bore-cut");
    let i = interferences(&m, main, drill).unwrap();
    // The singular vertex: at (0, R, 0), where the drill touches the wall.
    let touch = Point3::new(0.0, 1.0, 0.0);
    let v = i
        .vertices
        .iter()
        .find(|v| (v.point - touch).norm() <= v.tolerance)
        .expect("a section vertex at the touch");
    assert_eq!(v.source, VertexSource::SectionCrossing);
    let pair = &i.pairs[i.section_crossings[v.section_crossings[0]].pair];
    let named = vec![
        Shape::new(pair.a, Orientation::Forward),
        Shape::new(pair.b, Orientation::Forward),
    ];
    let before = arris_debug::dump_text(&m, main).unwrap();
    match cut(&mut m, main, drill) {
        Err(OpError::Degenerate {
            reason: Reason::Input(InputReason::NonManifold),
            entities,
        }) => assert_eq!(entities, named),
        other => panic!("{other:?}"),
    }
    assert_eq!(arris_debug::dump_text(&m, main).unwrap(), before);
}

/// A box and its sides split by a plane and fused back share the box's top
/// and bottom faces by id, with their edges and vertices: every boolean of
/// the two, and of the box with one side, builds checker-clean with its
/// record audited against both operands' ids, the way a box fused from
/// two boxes does (`query/interference-box-against-its-split-and-fused-sides`).
#[test]
fn operands_that_share_entities_build_with_their_own_ids() {
    use arris_check::{Level, check};
    use arris_debug::unmetered::split;
    use arris_topo::provenance::audit;

    type Op = fn(&mut Model, Body, Body) -> Result<(Body, arris_topo::Provenance), OpError>;
    let mut m = Model::default();
    let (whole, _) = primitive_box(&mut m, Point3::origin(), Point3::new(2.0, 2.0, 1.0)).unwrap();
    let plane = Frame::from_z(Point3::new(0.0, 0.0, 0.5), Vec3::z()).unwrap();
    let sides = split(&mut m, whole, &plane).unwrap();
    let (back, _) = fuse(&mut m, sides.positive, sides.negative).unwrap();
    let shared = m.closure(whole).unwrap().faces;
    assert!(
        m.closure(back)
            .unwrap()
            .faces
            .iter()
            .any(|f| shared.contains(f)),
        "the fused sides keep a face of the box by id"
    );
    let cases: [(&str, Op, Body, Body, f64); 7] = [
        ("common of the box and its sides", common, whole, back, 4.0),
        ("fuse of the box and its sides", fuse, whole, back, 4.0),
        (
            "common of the box and a side",
            common,
            whole,
            sides.positive,
            2.0,
        ),
        (
            "common of a side and the box",
            common,
            sides.positive,
            whole,
            2.0,
        ),
        (
            "cut of a side from the box",
            cut,
            whole,
            sides.positive,
            2.0,
        ),
        (
            "fuse of the box and a side",
            fuse,
            whole,
            sides.positive,
            4.0,
        ),
        (
            "cut of a side from the sides",
            cut,
            back,
            sides.negative,
            2.0,
        ),
    ];
    for (name, op, a, b, volume) in cases {
        let (r, p) = op(&mut m, a, b).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert!(
            check(&m, r, Level::Full).is_ok(),
            "{name}: not clean at Full"
        );
        audit(&m, &[a, b], r, &p).unwrap_or_else(|e| panic!("{name}: {e}"));
        // A shared entity the result holds by id is kept: the copy's
        // reading of it is no record of it.
        let (ca, cb, cr) = (
            m.closure(a).unwrap(),
            m.closure(b).unwrap(),
            m.closure(r).unwrap(),
        );
        let shared: Vec<Shape> = (ca.vertices.iter())
            .filter(|v| cb.vertices.contains(v) && cr.vertices.contains(v))
            .map(|&v| Shape::new(v, Orientation::Forward))
            .chain(
                (ca.edges.iter())
                    .filter(|e| cb.edges.contains(e) && cr.edges.contains(e))
                    .map(|&e| Shape::new(e, Orientation::Forward)),
            )
            .chain(
                (ca.faces.iter())
                    .filter(|f| cb.faces.contains(f) && cr.faces.contains(f))
                    .map(|&f| Shape::new(f, Orientation::Forward)),
            )
            .collect();
        for s in shared {
            assert!(p.is_kept(s, &m, r), "{name}: shared {s:?} not kept");
        }
        let v = mass_properties(&m, r).unwrap().volume;
        assert!((v - volume).abs() < 1e-9 * volume, "{name}: volume {v}");
        // The copy the boolean ran on is discarded: its body's slot,
        // below the result's, is the next one a body takes.
        let (next, _) =
            primitive_box(&mut m, Point3::origin(), Point3::new(1.0, 1.0, 1.0)).unwrap();
        assert!(
            next.id.index() < r.id.index(),
            "{name}: the copy was left in the model"
        );
    }
    // A refusal names the caller's bodies, not the copy it was decomposed
    // on.
    match cut(&mut m, sides.positive, whole) {
        Err(OpError::Degenerate { entities, .. }) => {
            let named: Vec<_> = entities.iter().map(|s| s.id).collect();
            assert_eq!(named, [sides.positive.id.into(), whole.id.into()]);
        }
        other => panic!("cut of the box from a side: {other:?}"),
    }
}

/// A frustum (radius 3 at z = 0, 2 at z = 4) and a 2 x 2 x 2 box standing
/// on its top plane touch at one point of the top rim, where the rim meets
/// the box's bottom edge from outside both faces: at the revolve's seam
/// (+x), inside the rim edge (-x), and at either side of the axis for a lug
/// turned 90 degrees. Each lump has its own vertex at the touch,
/// whichever operand comes first.
#[test]
fn a_frustum_and_a_box_touching_at_a_rim_point_give_two_lumps() {
    let p = |u, v| Point2::new(u, v);
    let section = Profile {
        plane: Frame::new(Point3::origin(), -Vec3::y(), Vec3::x()).unwrap(),
        outer: ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p(3.0, 0.0)),
                ProfileSegment::LineTo(p(2.0, 4.0)),
                ProfileSegment::LineTo(p(0.0, 4.0)),
                ProfileSegment::LineTo(p(0.0, 0.0)),
            ],
        },
        holes: Vec::new(),
    };
    for (min, max, touch) in [
        ([2.0, -1.0, 4.0], [4.0, 1.0, 6.0], [2.0, 0.0, 4.0]),
        ([-4.0, -1.0, 4.0], [-2.0, 1.0, 6.0], [-2.0, 0.0, 4.0]),
        ([-1.0, 2.0, 4.0], [1.0, 4.0, 6.0], [0.0, 2.0, 4.0]),
        ([-1.0, -4.0, 4.0], [1.0, -2.0, 6.0], [0.0, -2.0, 4.0]),
    ] {
        for frustum_first in [true, false] {
            let mut m = Model::default();
            let (frustum, _) =
                revolve(&mut m, &section, Axis::z_at(Point3::origin()), TAU).unwrap();
            let (lug, _) = primitive_box(&mut m, min, max).unwrap();
            let result = if frustum_first {
                fuse(&mut m, frustum, lug)
            } else {
                fuse(&mut m, lug, frustum)
            };
            let (body, provenance) = result.unwrap_or_else(|e| panic!("{min:?}: {e}"));
            let report = check(&m, body, Level::Full);
            assert!(
                report.is_ok() && report.unchecked().is_empty(),
                "{min:?}: {report}"
            );
            assert_eq!(lumps(&m, body).unwrap().len(), 2, "{min:?}");
            let touch = Point3::from(touch);
            let copies = m
                .vertices(body)
                .unwrap()
                .into_iter()
                .filter(|v| (m.vertex(v.id).unwrap().point() - touch).norm() < 1e-9)
                .count();
            assert_eq!(copies, 2, "{min:?}");
            arris_topo::provenance::audit(&m, &[frustum, lug], body, &provenance).unwrap();
        }
    }
}
