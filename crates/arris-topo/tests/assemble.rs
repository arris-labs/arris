//! `Builder::assemble` (ADR-0004, ADR-0006, `docs/DATA-MODEL.md` §Euler
//! operators): a body described as `Keep`/`New` specs rather than built by
//! an operator sequence. A body assembled from its own entities is the
//! same body — the same ids when every spec is `Keep`, the same dump up to
//! ids when every spec is `New` — a body of several shells is several
//! closed surfaces that share nothing, and every way of describing
//! something that is not is a typed refusal.

use core::f64::consts::{FRAC_1_SQRT_2, TAU};
use std::collections::BTreeMap;

use arris_check::{Level, check};
use arris_debug::{corpus, dump_text, euler_line, fixtures, sample};
use arris_topo::arris_geom::{Curve2, Profile, ProfileLoop, ProfileSegment, Surface};
use arris_topo::arris_math::{Axis, Frame, Frame2, Interval, Point2, Point3, Vec3};
use arris_topo::builder::{
    Assembly, AssemblySlots, BuildError, Builder, EdgeKey, EdgeSpec, FaceSpec, KeepGeometry,
    UseSpec, VertexKey, VertexSpec, effective_uses,
};
use arris_topo::entity::BodyKind;
use arris_topo::euler::EulerLine;
use arris_topo::{Body, Curve2Id, EdgeId, FaceId, Model, Orientation, VertexId};

/// One named body in its own model.
type Solid = (&'static str, fn(&mut Model) -> Body);

/// The bodies every round trip is run over, each in its own model: every
/// `sample` body an Euler operator sequence could have made, and the
/// sphere. `patch` is a sheet and `finish` makes solids. `sphere`'s two
/// degenerate pole edges are used once each: singular points, which
/// `assemble` and `finish` take since a revolve makes them at an apex or a
/// pole (`sample::sphere` predates that and goes through the raw insert).
fn solids() -> Vec<Solid> {
    vec![
        ("unit_box", |m| sample::unit_box(m).unwrap()),
        ("cuboid", |m| {
            sample::cuboid(m, Point3::origin(), Point3::new(4.0, 3.0, 2.0)).unwrap()
        }),
        ("cuboid_nurbs", |m| {
            sample::cuboid_nurbs(m, Point3::origin(), Point3::new(4.0, 3.0, 2.0)).unwrap()
        }),
        ("cylinder", |m| sample::cylinder(m, 4.0, 12.0).unwrap()),
        ("frame", |m| {
            sample::frame(
                m,
                Point3::origin(),
                Point3::new(40.0, 30.0, 10.0),
                Point2::new(10.0, 10.0),
                Point2::new(30.0, 20.0),
            )
            .unwrap()
        }),
        ("torus", |m| {
            sample::torus(m, Point3::origin(), 5.0, 2.0).unwrap()
        }),
        ("sphere", |m| {
            sample::sphere(m, Point3::new(1.0, -2.0, 0.5), 3.0).unwrap()
        }),
        ("primitive_box", |m| {
            arris_debug::unmetered::primitive_box(
                m,
                Point3::origin(),
                Point3::new(40.0, 30.0, 10.0),
            )
            .unwrap()
            .0
        }),
        ("primitive_cylinder", |m| {
            arris_debug::unmetered::primitive_cylinder(m, Axis::z_at(Point3::origin()), 4.0, 12.0)
                .unwrap()
                .0
        }),
    ]
}

/// The assembly that describes `body` exactly: every entity `Keep` when
/// `keep`, every entity `New` otherwise, in the body's own iteration
/// order, so the slot order is the arena's.
fn describe(m: &Model, body: Body, keep: bool) -> Assembly {
    if !keep {
        // `of_body` needs `&mut Model` only for a remap that might add
        // geometry; `KeepGeometry` never does, so a scratch clone is safe
        // and the ids it returns are the original model's.
        let mut scratch = m.clone();
        return Assembly::of_body(&mut scratch, body, &mut KeepGeometry)
            .unwrap()
            .0;
    }
    // Ascending by id, so the assembled body's slot order — and hence the
    // ids `finish` appends — keep the original's relative order.
    let mut vertices: Vec<VertexId> = m.vertices(body).unwrap().iter().map(|v| v.id).collect();
    let mut edges: Vec<EdgeId> = m.edges(body).unwrap().iter().map(|e| e.id).collect();
    vertices.sort_unstable();
    edges.sort_unstable();
    Assembly {
        vertices: vertices.iter().map(|&id| VertexSpec::Keep(id)).collect(),
        edges: edges.iter().map(|&id| EdgeSpec::Keep(id)).collect(),
        shells: vec![
            m.faces(body)
                .unwrap()
                .into_iter()
                .map(FaceSpec::Keep)
                .collect(),
        ],
    }
}

fn assemble(m: &Model, assembly: Assembly) -> Result<(Builder, AssemblySlots), BuildError> {
    Builder::assemble(m, m.precision().default_tolerance, assembly)
}

/// One Euler line (`docs/DATA-MODEL.md` §Euler–Poincaré): on bodies with
/// degenerate edges — the sphere sample's two poles, and a cone and a
/// hemisphere revolved down to the axis, which close on it — and on the
/// notch-to-axis revolve's lump with its voids, the counts of a builder
/// assembled from the body, `EulerLine::of` its closure, the checker's
/// line and the dump's all print the same, every degenerate edge left
/// out.
#[test]
fn a_body_with_degenerate_edges_has_one_euler_line() {
    let p = Point2::new;
    let plane = Frame::new(Point3::origin(), -Vec3::y(), Vec3::x()).unwrap();
    let revolved = |segments: Vec<ProfileSegment>| {
        let mut m = Model::default();
        let profile = Profile {
            plane,
            outer: ProfileLoop::Path {
                start: p(0.0, 0.0),
                segments,
            },
            holes: Vec::new(),
        };
        let axis = Axis::z_at(Point3::origin());
        let (body, _) = arris_debug::unmetered::revolve(&mut m, &profile, axis, TAU).unwrap();
        (m, body)
    };
    let mut bodies: Vec<(String, Model, Body)> = Vec::new();
    let mut m = Model::default();
    let sphere = sample::sphere(&mut m, Point3::origin(), 3.0).unwrap();
    bodies.push(("sphere".into(), m, sphere));
    let (m, cone) = revolved(vec![
        ProfileSegment::LineTo(p(2.0, 0.0)),
        ProfileSegment::LineTo(p(0.0, 3.0)),
        ProfileSegment::LineTo(p(0.0, 0.0)),
    ]);
    bodies.push(("cone".into(), m, cone));
    let (m, hemisphere) = revolved(vec![
        ProfileSegment::LineTo(p(2.0, 0.0)),
        ProfileSegment::ArcTo {
            to: p(0.0, 2.0),
            via: p(2.0 * FRAC_1_SQRT_2, 2.0 * FRAC_1_SQRT_2),
        },
        ProfileSegment::LineTo(p(0.0, 0.0)),
    ]);
    bodies.push(("hemisphere".into(), m, hemisphere));
    let dir = fixtures::corpus_root().join("sweep/revolve-notch-to-axis");
    for variant in fixtures::load(&dir).unwrap().recipe.variant_names() {
        let chain = corpus::chain(&dir, &variant).unwrap();
        let body = chain.result().unwrap();
        bodies.push((format!("notch-to-axis [{variant}]"), chain.model, body));
    }
    for (name, m, body) in &bodies {
        let closure = m.closure(*body).unwrap();
        let line = EulerLine::of(m, &closure);
        let report = check(m, *body, Level::Fast);
        assert!(report.is_ok(), "{name}:\n{report}");
        assert_eq!(
            report.euler().map(|l| l.to_string()),
            Some(line.to_string()),
            "{name}"
        );
        assert_eq!(
            euler_line(m, *body).unwrap(),
            format!("euler {line}"),
            "{name}"
        );
        let (builder, _) = assemble(m, describe(m, *body, false))
            .unwrap_or_else(|e| panic!("{name}: {e}\n{}", dump_text(m, *body).unwrap()));
        assert_eq!(builder.counts().line(), line, "{name}");
        assert_eq!(builder.counts().to_string(), line.to_string(), "{name}");
    }
    // The poles and the apex are in the closures, and out of the lines.
    for (name, m, body) in &bodies[..3] {
        let closure = m.closure(*body).unwrap();
        assert!(
            closure.edges.len() > EulerLine::of(m, &closure).edges,
            "{name} has a degenerate edge"
        );
    }
}

/// The dump's `edges` and `vertices` listings sorted by geometry, which no
/// id appears in. They are the arena's iteration order, and that order
/// starts wherever each stored loop starts — which a rotation is free to
/// move — so it is the one part of the dump two descriptions of one body
/// need not agree on. Sorting happens before anything is renumbered, so
/// the vertex a listing first mentions is the same vertex in both.
fn sort_listings(lines: Vec<&str>) -> Vec<&str> {
    let mut out: Vec<&str> = Vec::new();
    let mut blocks: Vec<Vec<&str>> = Vec::new();
    fn flush<'a>(blocks: &mut Vec<Vec<&'a str>>, out: &mut Vec<&'a str>) {
        blocks.sort_by_key(|b| {
            let geometry: Vec<&str> = b.iter().skip(1).copied().collect();
            // A vertex has no geometry line of its own; its point is the
            // rest of its one line, past the id.
            let rest = b[0]
                .trim_start()
                .split_once(' ')
                .map_or("", |(_, rest)| rest);
            (geometry.join("\n"), rest.to_string())
        });
        out.extend(blocks.drain(..).flatten());
    }
    let mut listing = false;
    for line in lines {
        let heads = line.starts_with("  ") && !line.starts_with("   ");
        match line {
            "edges" | "vertices" => {
                flush(&mut blocks, &mut out);
                listing = true;
                out.push(line);
            }
            _ if listing && heads => blocks.push(vec![line]),
            _ if listing && line.starts_with("    ") => {
                if let Some(block) = blocks.last_mut() {
                    block.push(line);
                }
            }
            _ => {
                flush(&mut blocks, &mut out);
                listing = false;
                out.push(line);
            }
        }
    }
    flush(&mut blocks, &mut out);
    out
}

/// The `(index, reversed)` of the edge a `coedge` line uses.
fn edge_key(line: &str) -> (u32, bool) {
    let token = line.split_whitespace().nth(1).unwrap_or_default();
    let reversed = token.starts_with('-');
    let digits: String = token
        .trim_start_matches(['+', '-', 'e'])
        .chars()
        .take_while(char::is_ascii_digit)
        .collect();
    (digits.parse().unwrap_or(0), reversed)
}

/// A dump with each loop rotated to start at its least line and every id
/// token then replaced by the order it is first seen in, per prefix: what
/// two dumps of one shape under different ids and different builder slot
/// orders agree on. The rotation comes first so that the renumbering sees
/// both dumps in the same order.
fn up_to_ids(text: &str) -> String {
    let mut rotated: Vec<&str> = Vec::new();
    let mut in_loop: Vec<&str> = Vec::new();
    /// A loop is a cycle, and the builder rotates it to start at its
    /// least edge slot; the arena's stored loops start wherever they were
    /// written. Rotating both to start at the least *edge id* — which the
    /// two dumps agree on, since the assembled body appends its edges in
    /// the order the original's are numbered — makes the cycles line up
    /// before anything is renumbered.
    fn flush<'a>(lines: &mut Vec<&'a str>, out: &mut Vec<&'a str>) {
        if let Some(at) = (0..lines.len()).min_by_key(|&i| edge_key(lines[i])) {
            lines.rotate_left(at);
        }
        out.append(lines);
    }
    for line in text.lines() {
        if line.trim_start().starts_with("coedge ") {
            in_loop.push(line);
        } else {
            flush(&mut in_loop, &mut rotated);
            rotated.push(line);
        }
    }
    flush(&mut in_loop, &mut rotated);
    let rotated = sort_listings(rotated);

    let mut seen: BTreeMap<String, usize> = BTreeMap::new();
    let mut renumber = |token: &str| -> String {
        let Some(prefix) = token.get(..1) else {
            return token.to_string();
        };
        if !"vefsbcSp".contains(prefix) || !token[1..].chars().all(|c| c.is_ascii_digit()) {
            return token.to_string();
        }
        let next = seen.keys().filter(|k| k.starts_with(prefix)).count();
        let n = *seen.entry(token.to_string()).or_insert(next);
        format!("{prefix}#{n}")
    };
    rotated
        .into_iter()
        .map(|line| {
            line.split(' ')
                .map(|w| match w.strip_prefix(['+', '-']) {
                    Some(rest) => format!("{}{}", &w[..1], renumber(rest)),
                    None => renumber(w),
                })
                .collect::<Vec<_>>()
                .join(" ")
        })
        .collect::<Vec<_>>()
        .join("\n")
}

#[test]
fn every_face_kept_is_the_same_body_with_nothing_appended() {
    for (name, build) in solids() {
        let mut m = Model::default();
        let body = build(&mut m);
        let (v, e, f) = (
            m.vertices(body).unwrap().len(),
            m.edges(body).unwrap().len(),
            m.faces(body).unwrap().len(),
        );
        let before = dump_text(&m, body).unwrap();
        let built = assemble(&m, describe(&m, body, true))
            .unwrap_or_else(|e| panic!("{name}: {e}"))
            .0
            .finish(&mut m, BodyKind::Solid)
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_eq!(
            m.faces(built.body).unwrap(),
            m.faces(body).unwrap(),
            "{name}: the same faces with the same orientations"
        );
        assert_eq!(
            m.edges(built.body).unwrap(),
            m.edges(body).unwrap(),
            "{name}"
        );
        assert_eq!(
            m.vertices(built.body).unwrap(),
            m.vertices(body).unwrap(),
            "{name}"
        );
        // Nothing was appended: the slot one past the last is still free.
        assert!(m.vertex(VertexId::new(v as u32, 0)).is_err(), "{name}");
        assert!(m.edge(EdgeId::new(e as u32, 0)).is_err(), "{name}");
        assert!(m.face(FaceId::new(f as u32, 0)).is_err(), "{name}");
        // `Full`, but not "nothing unchecked": a NURBS surface pair has no
        // closed form, and the body it came from records the same rows.
        let report = check(&m, built.body, Level::Full);
        assert!(report.is_ok(), "{name}: {report}");
        let after = dump_text(&m, built.body).unwrap();
        assert_eq!(
            before
                .lines()
                .filter(|l| !l.starts_with("body") && !l.trim_start().starts_with("shell"))
                .collect::<Vec<_>>(),
            after
                .lines()
                .filter(|l| !l.starts_with("body") && !l.trim_start().starts_with("shell"))
                .collect::<Vec<_>>(),
            "{name}: the same dump but for the new body and shell ids"
        );
    }
}

#[test]
fn every_face_new_is_the_same_body_under_new_ids() {
    for (name, build) in solids() {
        let mut m = Model::default();
        let body = build(&mut m);
        let before = dump_text(&m, body).unwrap();
        let built = assemble(&m, describe(&m, body, false))
            .unwrap()
            .0
            .finish(&mut m, BodyKind::Solid)
            .unwrap();
        assert_ne!(built.body.id, body.id, "{name}");
        // `Full`, but not "nothing unchecked": a NURBS surface pair has no
        // closed form, and the body it came from records the same rows.
        let report = check(&m, built.body, Level::Full);
        assert!(report.is_ok(), "{name}: {report}");
        let after = dump_text(&m, built.body).unwrap();
        assert_eq!(up_to_ids(&before), up_to_ids(&after), "{name}");
    }
}

/// `Assembly::of_body` with the identity remap describes a body exactly:
/// assembled and finished, its dump equals the original's up to ids —
/// proven above of every Euler-operator-built sample through
/// `describe(false)`, which calls it; here of a multi-shell body from an
/// actual boolean result (two boxes apart, two lumps of one solid,
/// ADR-0006), which no `describe` caller builds.
#[test]
fn of_body_with_the_identity_remap_reproduces_a_multi_shell_boolean_result() {
    let mut m = Model::default();
    let a =
        arris_debug::unmetered::primitive_box(&mut m, Point3::origin(), Point3::new(1.0, 1.0, 1.0))
            .unwrap()
            .0;
    let b = arris_debug::unmetered::primitive_box(
        &mut m,
        Point3::new(5.0, 0.0, 0.0),
        Point3::new(6.0, 1.0, 1.0),
    )
    .unwrap()
    .0;
    let (body, _) = arris_debug::unmetered::fuse(&mut m, a, b).unwrap();
    assert_eq!(
        m.shells(body).unwrap().len(),
        2,
        "two boxes apart are two lumps"
    );

    let before = dump_text(&m, body).unwrap();
    let (assembly, _index) = Assembly::of_body(&mut m, body, &mut KeepGeometry).unwrap();
    let built = assemble(&m, assembly)
        .unwrap()
        .0
        .finish(&mut m, BodyKind::Solid)
        .unwrap();
    assert_ne!(built.body.id, body.id);
    let report = check(&m, built.body, Level::Full);
    assert!(report.is_ok(), "{report}");
    let after = dump_text(&m, built.body).unwrap();
    assert_eq!(up_to_ids(&before), up_to_ids(&after));
}

/// `effective_uses` walks a loop both ways with the same function
/// (composing an orientation with itself and reversing a list are each
/// their own inverse): for every face of every sample body, under both an
/// assumed `Forward` and `Reversed` face orientation, `keep_face`'s walk
/// (stored to effective) followed by `finish`'s (effective to stored) —
/// `effective_uses` applied twice — returns the stored loop unchanged.
#[test]
fn effective_uses_is_its_own_inverse_on_every_sample_face() {
    for (name, build) in solids() {
        let mut m = Model::default();
        let body = build(&mut m);
        for face in m.faces(body).unwrap() {
            let entity = m.face(face.id).unwrap();
            for (li, l) in entity.loops().iter().enumerate() {
                let raw: Vec<(EdgeId, Orientation, Curve2Id)> = l
                    .coedges()
                    .iter()
                    .map(|c| (c.edge(), c.orientation(), c.pcurve()))
                    .collect();
                for orientation in [Orientation::Forward, Orientation::Reversed] {
                    let effective = effective_uses(orientation, raw.clone());
                    let back = effective_uses(orientation, effective);
                    assert_eq!(back, raw, "{name} {} loop {li}", face.id);
                }
            }
        }
    }
}

#[test]
fn the_counts_and_the_genus_are_the_bodys() {
    let mut m = Model::default();
    let torus = sample::torus(&mut m, Point3::origin(), 5.0, 2.0).unwrap();
    let (b, _) = assemble(&m, describe(&m, torus, true)).unwrap();
    assert_eq!(b.counts().to_string(), "1/2/1/1/1 g1 = 0");
    let mut m = Model::default();
    let sphere = sample::sphere(&mut m, Point3::origin(), 3.0).unwrap();
    let (b, _) = assemble(&m, describe(&m, sphere, true)).unwrap();
    assert_eq!(
        b.counts().to_string(),
        "2/1/1/1/1 g0 = 0",
        "the two pole edges, each used once, are not counted"
    );
    let mut m = Model::default();
    let frame = sample::frame(
        &mut m,
        Point3::origin(),
        Point3::new(40.0, 30.0, 10.0),
        Point2::new(10.0, 10.0),
        Point2::new(30.0, 20.0),
    )
    .unwrap();
    let (b, _) = assemble(&m, describe(&m, frame, true)).unwrap();
    assert_eq!(b.counts().genus, 1);
    assert_eq!(b.counts().euler(), 0);
}

#[test]
fn an_operator_on_a_kept_face_drops_the_mark_and_finish_appends_it() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let faces = m.faces(body).unwrap();
    let (mut b, _) = assemble(&m, describe(&m, body, true)).unwrap();
    let touched = b.faces().next().map(|(f, _)| f).unwrap();
    let pcurve = b.face(touched).unwrap().loops()[0].uses()[0]
        .pcurve
        .unwrap();
    assert!(b.face(touched).unwrap().kept().is_some());
    assert_eq!(
        b.set_pcurve(arris_topo::builder::Position::new(touched, 0, 0), pcurve),
        Ok(Some(pcurve))
    );
    assert!(
        b.face(touched).unwrap().kept().is_none(),
        "set_pcurve drops the mark even when it writes the same pcurve back"
    );
    let built = b.finish(&mut m, BodyKind::Solid).unwrap();
    let after = m.faces(built.body).unwrap();
    assert_eq!(after.len(), faces.len());
    assert_eq!(
        after.iter().filter(|f| !faces.contains(f)).count(),
        1,
        "exactly the touched face is new"
    );
}

#[test]
fn an_edge_used_once_none_or_three_times_is_refused() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    // One face left out: its edges lose a use.
    let mut assembly = describe(&m, body, true);
    assembly.shells[0].pop();
    assert!(matches!(
        assemble(&m, assembly),
        Err(BuildError::EdgeUses { uses: 1, .. })
    ));
    // A spare edge no face uses.
    let mut assembly = describe(&m, body, true);
    let spare = m.edges(body).unwrap()[0].id;
    assembly.edges.push(EdgeSpec::New {
        geometry: m.edge(spare).unwrap().geometry(),
        start: VertexKey::Kept(m.edge(spare).unwrap().start()),
        end: VertexKey::Kept(m.edge(spare).unwrap().end()),
        tolerance: 1e-7,
    });
    assert!(matches!(
        assemble(&m, assembly),
        Err(BuildError::EdgeUses { uses: 0, .. })
    ));
    // A third use of the bottom circle, which is a closed edge, so the
    // extra loop closes and only the use count is wrong.
    let mut assembly = describe(&m, body, true);
    let circle = closed_edge(&m, body);
    let plane = m.add_surface(Surface::Plane {
        frame: Frame::world(),
    });
    let pcurve = m.add_curve2(Curve2::Circle {
        frame: Frame2::identity(),
        radius: 4.0,
    });
    assembly.shells[0].push(FaceSpec::New {
        surface: plane,
        orientation: Orientation::Forward,
        loops: vec![vec![UseSpec {
            edge: EdgeKey::Kept(circle),
            orientation: Orientation::Forward,
            pcurve,
        }]],
        tolerance: 1e-7,
    });
    assert!(matches!(
        assemble(&m, assembly),
        Err(BuildError::EdgeUses { uses: 3, .. })
    ));
}

/// The first closed edge of `body` — the cylinder's bottom circle.
fn closed_edge(m: &Model, body: Body) -> EdgeId {
    m.edges(body)
        .unwrap()
        .into_iter()
        .map(|e| e.id)
        .find(|&e| m.edge(e).unwrap().is_closed())
        .unwrap()
}

#[test]
fn an_edge_used_twice_the_same_way_is_refused() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let circle = closed_edge(&m, body);
    let mut assembly = describe(&m, body, false);
    let mut flipped = 0;
    for face in &mut assembly.shells[0] {
        if let FaceSpec::New { loops, .. } = face {
            for walk in loops.iter_mut() {
                if walk.len() == 1 && walk[0].edge == EdgeKey::New(edge_slot(&m, body, circle)) {
                    walk[0].orientation = walk[0].orientation.flipped();
                    flipped += 1;
                    break;
                }
            }
        }
        if flipped == 1 {
            break;
        }
    }
    // Writing a `Reversed` face's uses in the orientation they are stored
    // with rather than the effective one is exactly this mistake.
    assert_eq!(flipped, 1, "the bottom cap's one use of the closed circle");
    assert!(matches!(
        assemble(&m, assembly),
        Err(BuildError::SameDirection { .. })
    ));
}

fn edge_slot(m: &Model, body: Body, edge: EdgeId) -> usize {
    m.edges(body)
        .unwrap()
        .iter()
        .position(|e| e.id == edge)
        .unwrap()
}

#[test]
fn a_loop_that_does_not_close_is_refused() {
    let mut m = Model::default();
    let body = sample::unit_box(&mut m).unwrap();
    let mut assembly = describe(&m, body, false);
    if let FaceSpec::New { loops, .. } = &mut assembly.shells[0][0] {
        loops[0][0].orientation = loops[0][0].orientation.flipped();
    }
    assert!(matches!(
        assemble(&m, assembly),
        Err(BuildError::LoopOpen { .. })
    ));
}

#[test]
fn a_loop_given_backwards_is_refused() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let mut assembly = describe(&m, body, false);
    let wall = assembly.shells[0]
        .iter_mut()
        .find(|f| matches!(f, FaceSpec::New { loops, .. } if loops[0].len() == 4))
        .expect("the wall's loop: bottom circle, seam up, top circle, seam down");
    if let FaceSpec::New { loops, .. } = wall {
        loops[0].reverse();
    }
    assert!(matches!(
        assemble(&m, assembly),
        Err(BuildError::LoopOpen { .. })
    ));
}

#[test]
fn two_components_in_one_shell_are_refused() {
    let mut m = Model::default();
    let a = sample::unit_box(&mut m).unwrap();
    let b = sample::cuboid(
        &mut m,
        Point3::new(5.0, 0.0, 0.0),
        Point3::new(6.0, 1.0, 1.0),
    )
    .unwrap();
    let mut assembly = describe(&m, a, true);
    let other = describe(&m, b, true);
    assembly.shells[0].extend(other.shells.into_iter().flatten());
    assert!(matches!(
        assemble(&m, assembly),
        Err(BuildError::Disconnected { .. })
    ));
}

/// Two boxes apart, `a`'s faces the first shell and `b`'s the second.
fn two_boxes(m: &mut Model) -> (Body, Body) {
    let a = sample::unit_box(m).unwrap();
    let b = sample::cuboid(m, Point3::new(5.0, 0.0, 0.0), Point3::new(6.0, 1.0, 1.0)).unwrap();
    (a, b)
}

/// The same two components as two shells are one body: every face kept,
/// the counts the sum of the two boxes', a body storing the shells in the
/// assembly's order, and each shell the faces of one box.
#[test]
fn two_components_as_two_shells_are_one_body() {
    let mut m = Model::default();
    let (a, b) = two_boxes(&mut m);
    let mut assembly = describe(&m, a, true);
    assembly.shells.extend(describe(&m, b, true).shells);
    let (builder, _) = assemble(&m, assembly).unwrap();
    assert_eq!(builder.counts().to_string(), "16/24/12/12/2 g0 = 0");
    assert!(builder.dump().contains(" shell 1 "), "{}", builder.dump());
    let built = builder.finish(&mut m, BodyKind::Solid).unwrap();
    assert_eq!(built.shells.len(), 2);
    let shells = m.shells(built.body).unwrap();
    assert_eq!(
        shells.iter().map(|s| s.id).collect::<Vec<_>>(),
        built.shells
    );
    for (shell, box_body) in shells.iter().zip([a, b]) {
        let faces: Vec<FaceId> = m
            .shell(shell.id)
            .unwrap()
            .faces()
            .iter()
            .map(|f| f.id)
            .collect();
        let own: Vec<FaceId> = m.faces(box_body).unwrap().iter().map(|f| f.id).collect();
        assert_eq!(faces, own);
    }
    assert_eq!(m.faces(built.body).unwrap().len(), 12);
    let report = check(&m, built.body, Level::Fast);
    assert!(report.is_ok(), "{report}");
    assert_eq!(
        report.euler().map(|l| l.to_string()),
        Some("16/24/12/12/2 g0 = 0".to_string())
    );

    // Described with every entity new, the same counts.
    let mut assembly = describe(&m, a, false);
    let other = describe(&m, b, false);
    let (nv, ne) = (assembly.vertices.len(), assembly.edges.len());
    assembly.vertices.extend(other.vertices);
    assembly
        .edges
        .extend(other.edges.into_iter().map(|e| shifted_edge(e, nv)));
    assembly.shells.extend(
        other
            .shells
            .into_iter()
            .map(|faces| faces.into_iter().map(|f| shifted_face(f, ne)).collect()),
    );
    let (builder, _) = assemble(&m, assembly).unwrap();
    assert_eq!(builder.counts().to_string(), "16/24/12/12/2 g0 = 0");
}

/// An edge spec of a second description appended after `by` vertices.
fn shifted_edge(e: EdgeSpec, by: usize) -> EdgeSpec {
    match e {
        EdgeSpec::New {
            geometry,
            start,
            end,
            tolerance,
        } => EdgeSpec::New {
            geometry,
            start: shifted_key(start, by),
            end: shifted_key(end, by),
            tolerance,
        },
        kept @ EdgeSpec::Keep(_) => kept,
    }
}

fn shifted_key(k: VertexKey, by: usize) -> VertexKey {
    match k {
        VertexKey::New(i) => VertexKey::New(i + by),
        kept @ VertexKey::Kept(_) => kept,
    }
}

/// A face spec of a second description appended after `by` edges.
fn shifted_face(f: FaceSpec, by: usize) -> FaceSpec {
    match f {
        FaceSpec::New {
            surface,
            orientation,
            loops,
            tolerance,
        } => FaceSpec::New {
            surface,
            orientation,
            loops: loops
                .into_iter()
                .map(|l| {
                    l.into_iter()
                        .map(|u| UseSpec {
                            edge: match u.edge {
                                EdgeKey::New(i) => EdgeKey::New(i + by),
                                kept @ EdgeKey::Kept(_) => kept,
                            },
                            ..u
                        })
                        .collect()
                })
                .collect(),
            tolerance,
        },
        kept @ FaceSpec::Keep(_) => kept,
    }
}

/// One box's faces split between two shells: every edge between the
/// halves is used once by each, so neither half is closed.
#[test]
fn an_edge_used_by_two_shells_is_refused() {
    let mut m = Model::default();
    let body = sample::unit_box(&mut m).unwrap();
    let mut assembly = describe(&m, body, true);
    let second = assembly.shells[0].split_off(3);
    assembly.shells.push(second);
    assert!(matches!(
        assemble(&m, assembly),
        Err(BuildError::SharedEdge { shells: [0, 1], .. })
    ));
}

/// Two boxes touching at a corner, the corner one vertex of both: two
/// closed shells meeting at a point.
#[test]
fn a_vertex_on_two_shells_is_refused() {
    let mut m = Model::default();
    let a = sample::unit_box(&mut m).unwrap();
    let b = sample::cuboid(
        &mut m,
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(2.0, 2.0, 2.0),
    )
    .unwrap();
    let corner = |body: Body| {
        m.vertices(body)
            .unwrap()
            .into_iter()
            .map(|v| v.id)
            .find(|&v| (m.vertex(v).unwrap().point() - Point3::new(1.0, 1.0, 1.0)).norm() == 0.0)
            .unwrap()
    };
    let (ca, cb) = (corner(a), corner(b));
    let mut assembly = describe(&m, a, false);
    let other = describe(&m, b, false);
    let nv = assembly.vertices.len();
    let ne = assembly.edges.len();
    let mut b_vertices: Vec<VertexId> = m.vertices(b).unwrap().iter().map(|v| v.id).collect();
    b_vertices.sort_unstable();
    let mut a_vertices: Vec<VertexId> = m.vertices(a).unwrap().iter().map(|v| v.id).collect();
    a_vertices.sort_unstable();
    let (ia, ib) = (
        a_vertices.iter().position(|&v| v == ca).unwrap(),
        b_vertices.iter().position(|&v| v == cb).unwrap(),
    );
    assembly.vertices.extend(other.vertices);
    assembly.edges.extend(other.edges.into_iter().map(|e| {
        // `b`'s corner is `a`'s: one vertex, on edges of both shells.
        let e = shifted_edge(e, nv);
        match e {
            EdgeSpec::New {
                geometry,
                start,
                end,
                tolerance,
            } => {
                let at = |k: VertexKey| {
                    if k == VertexKey::New(nv + ib) {
                        VertexKey::New(ia)
                    } else {
                        k
                    }
                };
                EdgeSpec::New {
                    geometry,
                    start: at(start),
                    end: at(end),
                    tolerance,
                }
            }
            kept @ EdgeSpec::Keep(_) => kept,
        }
    }));
    assembly.shells.extend(
        other
            .shells
            .into_iter()
            .map(|faces| faces.into_iter().map(|f| shifted_face(f, ne)).collect()),
    );
    assert!(matches!(
        assemble(&m, assembly),
        Err(BuildError::SharedVertex { shells: [0, 1], .. })
    ));
}

#[test]
fn an_empty_shell_is_refused() {
    let mut m = Model::default();
    let body = sample::unit_box(&mut m).unwrap();
    let mut assembly = describe(&m, body, true);
    assembly.shells.insert(0, Vec::new());
    assert_eq!(
        assemble(&m, assembly).err(),
        Some(BuildError::EmptyShell { shell: 0 })
    );
}

#[test]
fn an_open_surface_does_not_close_the_euler_line() {
    let mut m = Model::default();
    let surface = Surface::Sphere {
        frame: Frame::world(),
        radius: 2.0,
    };
    let sheet = sample::patch(
        &mut m,
        surface,
        Interval::new(0.2, 1.4).unwrap(),
        Interval::new(-0.5, 0.9).unwrap(),
    )
    .unwrap();
    // A sheet's four edges are used once each, which is the first thing
    // `assemble` reports about it.
    assert!(matches!(
        assemble(&m, describe(&m, sheet, true)),
        Err(BuildError::EdgeUses { uses: 1, .. })
    ));
}

#[test]
fn a_kept_entity_that_does_not_resolve_is_refused() {
    let mut m = Model::default();
    let body = sample::unit_box(&mut m).unwrap();
    let mut assembly = describe(&m, body, true);
    assembly.shells[0][0] = FaceSpec::Keep(arris_topo::Face::forward(FaceId::new(99, 0)));
    assert!(matches!(
        assemble(&m, assembly),
        Err(BuildError::NotFound(_))
    ));
    let mut assembly = describe(&m, body, true);
    assembly.edges[0] = EdgeSpec::Keep(EdgeId::new(99, 0));
    assert!(matches!(
        assemble(&m, assembly),
        Err(BuildError::NotFound(_))
    ));
}

#[test]
fn keeping_one_entity_twice_is_refused() {
    let mut m = Model::default();
    let body = sample::unit_box(&mut m).unwrap();
    let mut assembly = describe(&m, body, true);
    let again = assembly.shells[0][0].clone();
    assembly.shells[0].push(again);
    assert!(matches!(
        assemble(&m, assembly),
        Err(BuildError::Duplicate(_))
    ));
    let mut assembly = describe(&m, body, true);
    assembly.vertices.push(assembly.vertices[0]);
    assert!(matches!(
        assemble(&m, assembly),
        Err(BuildError::Duplicate(_))
    ));
}

/// `perm[old] = new`, from `order[new] = old`.
fn permutation(order: &[usize]) -> Vec<usize> {
    let mut perm = vec![0; order.len()];
    for (new, &old) in order.iter().enumerate() {
        perm[old] = new;
    }
    perm
}

/// `items` reordered so that position `new` holds `items[order[new]]`.
fn reordered<T: Clone>(items: &[T], order: &[usize]) -> Vec<T> {
    order.iter().map(|&old| items[old].clone()).collect()
}

fn permute_vertex_key(k: VertexKey, perm: &[usize]) -> VertexKey {
    match k {
        VertexKey::New(i) => VertexKey::New(perm[i]),
        kept @ VertexKey::Kept(_) => kept,
    }
}

fn permute_edge_spec(e: EdgeSpec, perm: &[usize]) -> EdgeSpec {
    match e {
        EdgeSpec::New {
            geometry,
            start,
            end,
            tolerance,
        } => EdgeSpec::New {
            geometry,
            start: permute_vertex_key(start, perm),
            end: permute_vertex_key(end, perm),
            tolerance,
        },
        kept @ EdgeSpec::Keep(_) => kept,
    }
}

fn permute_edge_key(k: EdgeKey, perm: &[usize]) -> EdgeKey {
    match k {
        EdgeKey::New(i) => EdgeKey::New(perm[i]),
        kept @ EdgeKey::Kept(_) => kept,
    }
}

fn permute_face_spec(f: FaceSpec, perm: &[usize]) -> FaceSpec {
    match f {
        FaceSpec::New {
            surface,
            orientation,
            loops,
            tolerance,
        } => FaceSpec::New {
            surface,
            orientation,
            loops: loops
                .into_iter()
                .map(|l| {
                    l.into_iter()
                        .map(|u| UseSpec {
                            edge: permute_edge_key(u.edge, perm),
                            ..u
                        })
                        .collect()
                })
                .collect(),
            tolerance,
        },
        kept @ FaceSpec::Keep(_) => kept,
    }
}

/// `assemble` hands back one slot per spec, in spec order — not the order
/// `built.*` would fall in under some other, equally plausible order such
/// as ascending arena id. An assembly interleaving `Keep` and `New` specs,
/// and ordered by a permutation of the body's own ascending-id order,
/// proves it: `built.*[&slot]` names exactly the entity each spec named,
/// while pairing the body's ascending-id lists with `built.*.values()` —
/// what a positional zip assumes — does not.
#[test]
fn assemble_hands_back_a_slot_per_spec_in_spec_order() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();

    let mut vertex_ids: Vec<VertexId> = m.vertices(body).unwrap().iter().map(|v| v.id).collect();
    vertex_ids.sort_unstable();
    let mut edge_ids: Vec<EdgeId> = m.edges(body).unwrap().iter().map(|e| e.id).collect();
    edge_ids.sort_unstable();
    let faces = m.faces(body).unwrap();

    let mut assembly = describe(&m, body, false);

    // Vertices: the two swapped, the one now at position 0 turned `Keep`.
    let v_order = [1_usize, 0];
    let v_perm = permutation(&v_order);
    assembly.vertices = reordered(&assembly.vertices, &v_order);
    let kept_vertex = vertex_ids[v_order[0]];
    assembly.vertices[0] = VertexSpec::Keep(kept_vertex);

    // Edges: a 3-cycle. The one closed edge whose single vertex is
    // `kept_vertex` is turned `Keep` too — a `Keep` edge resolves its ends
    // by arena identity, bypassing the position, so it may only name
    // vertices `Keep` at the same identity, which only `kept_vertex` is.
    let e_order = [2_usize, 0, 1];
    let e_perm = permutation(&e_order);
    assembly.edges = reordered(&assembly.edges, &e_order)
        .into_iter()
        .map(|e| permute_edge_spec(e, &v_perm))
        .collect();
    let keep_edge = edge_ids
        .iter()
        .copied()
        .find(|&e| {
            let edge = m.edge(e).unwrap();
            edge.is_closed() && edge.start() == kept_vertex
        })
        .expect("the cap circle at the kept vertex");
    let ie_keep = edge_ids.iter().position(|&e| e == keep_edge).unwrap();
    assembly.edges[e_perm[ie_keep]] = EdgeSpec::Keep(keep_edge);

    // Faces: a 3-cycle. The one face bounded solely by `keep_edge` — the
    // flat cap at `kept_vertex`, not the wall — is turned `Keep` too, for
    // the same reason: every edge and vertex it names must already be
    // `Keep` at the same identity, and that face names only `keep_edge`.
    let f_order = [1_usize, 2, 0];
    let f_perm = permutation(&f_order);
    assembly.shells[0] = reordered(&assembly.shells[0], &f_order)
        .into_iter()
        .map(|f| permute_face_spec(f, &e_perm))
        .collect();
    let keep_face = faces
        .iter()
        .copied()
        .find(|f| {
            let entity = m.face(f.id).unwrap();
            matches!(entity.loops(), [l] if matches!(l.coedges(), [c] if c.edge() == keep_edge))
        })
        .expect("the cap face over the kept edge");
    let if_keep = faces.iter().position(|f| f.id == keep_face.id).unwrap();
    assembly.shells[0][f_perm[if_keep]] = FaceSpec::Keep(keep_face);

    let (builder, slots) = assemble(&m, assembly).unwrap();
    let built = builder.finish(&mut m, BodyKind::Solid).unwrap();

    for (i, &slot) in slots.vertices.iter().enumerate() {
        let id = built.vertices[&slot];
        let named = vertex_ids[v_order[i]];
        if i == 0 {
            assert_eq!(id, named, "spec {i} keeps {named}");
        } else {
            assert_eq!(
                m.vertex(id).unwrap().point(),
                m.vertex(named).unwrap().point(),
                "spec {i} is new, at {named}'s point"
            );
        }
    }
    for (i, &slot) in slots.edges.iter().enumerate() {
        let id = built.edges[&slot];
        let named = edge_ids[e_order[i]];
        if named == keep_edge {
            assert_eq!(id, named, "spec {i} keeps {named}");
        } else {
            assert_eq!(
                m.edge(id).unwrap().geometry(),
                m.edge(named).unwrap().geometry(),
                "spec {i} is new, over {named}'s geometry"
            );
        }
    }
    for (i, &slot) in slots.faces[0].iter().enumerate() {
        let id = built.faces[&slot];
        let named = faces[f_order[i]];
        if named.id == keep_face.id {
            assert_eq!(id, named.id, "spec {i} keeps {}", named.id);
        } else {
            assert_eq!(
                m.face(id).unwrap().surface(),
                m.face(named.id).unwrap().surface(),
                "spec {i} is new, on {}'s surface",
                named.id
            );
        }
    }

    // A positional zip against the body's own ascending-id order — what
    // `slots` replaces — pairs spec 0 with `vertex_ids[0]`, not the vertex
    // it keeps.
    let by_slot: Vec<VertexId> = built.vertices.values().copied().collect();
    assert_eq!(by_slot[0], kept_vertex, "spec 0 keeps {kept_vertex}");
    assert_ne!(
        by_slot[0], vertex_ids[0],
        "ascending-id order and spec order disagree at 0"
    );
}

#[test]
fn a_key_past_its_list_is_refused() {
    let mut m = Model::default();
    let body = sample::unit_box(&mut m).unwrap();
    let mut assembly = describe(&m, body, false);
    if let EdgeSpec::New { start, .. } = &mut assembly.edges[0] {
        *start = VertexKey::New(99);
    }
    assert_eq!(
        assemble(&m, assembly).err(),
        Some(BuildError::NoSpec {
            kind: arris_topo::EntityKind::Vertex,
            index: 99
        })
    );
}
