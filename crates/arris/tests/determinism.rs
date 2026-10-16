//! The determinism corpus (`.agents/rules/kernel.md` §Correctness): each
//! recipe builds a body whose native-format bytes, volume and area are
//! held to `determinism/digests.json` bit for bit. The same test runs
//! natively and on `wasm32-wasip1` under Node (`tools/wasi-run.mjs`), so a
//! body that differs between targets fails here, not in a consumer.
//!
//! The recipes' inputs are computed with `libm`, never `std`'s
//! transcendentals, so the input bits are the same on every target and
//! only the kernel can make the outputs differ.
//!
//! `ARRIS_BLESS=1` rewrites the digests from a native run; a wasm run only
//! compares.

use std::collections::BTreeMap;

use arris::geom::{FrameLaw, NurbsCurve, Path, PathSegment, Profile, ProfileLoop, ProfileSegment};
use arris::io::step::ReadOptions;
use arris::math::{Axis, Control, Frame, Point2, Point3, Vec3};
use arris::topo::Model;
use arris_debug::unmetered::{
    cut, extrude, fillet, mass_properties, primitive_box, primitive_cylinder, revolve, step_read,
    tessellate,
};
use sha2::{Digest, Sha256};

/// The committed digests, embedded so a wasm run needs no filesystem.
const DIGESTS: &str = include_str!("determinism/digests.json");

/// The hex SHA-256 of `bytes`.
fn sha256(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

/// One recipe's result: the body's native bytes digested, and its volume
/// and area printed shortest-round-trip, so a last-ulp move shows.
fn record(m: &Model, body: arris::topo::Body, prov: &arris::topo::Provenance) -> serde_json::Value {
    let bytes = arris::io::body::write(m, body, prov).expect("a built body writes");
    let hex = sha256(&bytes);
    let mp = mass_properties(m, body).unwrap_or_else(|e| panic!("{e:?}"));
    serde_json::json!({
        "body_sha256": hex,
        "volume": format!("{:?}", mp.volume),
        "area": format!("{:?}", mp.area),
    })
}

/// Point `t` of the involute of the circle of radius `rb`, turned by
/// `angle` and mirrored across its start ray when `mirror`.
fn involute(rb: f64, t: f64, angle: f64, mirror: bool) -> Point2 {
    let (st, ct) = (libm::sin(t), libm::cos(t));
    let (x, y) = (rb * (ct + t * st), rb * (st - t * ct));
    let y = if mirror { -y } else { y };
    let (sa, ca) = (libm::sin(angle), libm::cos(angle));
    Point2::new(ca * x - sa * y, sa * x + ca * y)
}

fn polar(r: f64, angle: f64) -> Point2 {
    Point2::new(r * libm::cos(angle), r * libm::sin(angle))
}

/// A 20-tooth spur gear's outline, module 1, pressure angle 20°: per
/// tooth a root arc, a line up to the base circle, four arcs along each
/// involute flank and a tip arc between them, 40 `LineTo` and 200
/// three-point `ArcTo` in all. The shape of the plugin CAD's gear in the
/// backlog's `arc-sweep-differs-on-wasm32`.
fn gear_profile() -> Profile {
    const TEETH: usize = 20;
    const FLANK_ARCS: usize = 4;
    let module = 1.0;
    let pitch = module * TEETH as f64 / 2.0;
    let alpha = 20.0_f64.to_radians();
    let rb = pitch * libm::cos(alpha);
    let (ra, rf) = (pitch + module, pitch - 1.25 * module);
    let t_max = libm::sqrt((ra / rb) * (ra / rb) - 1.0);
    // Half the tooth's angular width at the base circle.
    let beta = core::f64::consts::PI / (2.0 * TEETH as f64) + (libm::tan(alpha) - alpha);
    let step = core::f64::consts::TAU / TEETH as f64;
    let ts: Vec<f64> = (0..=2 * FLANK_ARCS)
        .map(|i| t_max * i as f64 / (2 * FLANK_ARCS) as f64)
        .collect();

    let start = polar(rf, -beta);
    let mut segments = Vec::with_capacity(TEETH * 12);
    for k in 0..TEETH {
        let centre = k as f64 * step;
        let (right, left) = (centre - beta, centre + beta);
        if k > 0 {
            let from = (centre - step) + beta;
            segments.push(ProfileSegment::ArcTo {
                to: polar(rf, right),
                via: polar(rf, (from + right) / 2.0),
            });
        }
        segments.push(ProfileSegment::LineTo(involute(rb, 0.0, right, false)));
        for i in 0..FLANK_ARCS {
            segments.push(ProfileSegment::ArcTo {
                to: involute(rb, ts[2 * i + 2], right, false),
                via: involute(rb, ts[2 * i + 1], right, false),
            });
        }
        segments.push(ProfileSegment::ArcTo {
            to: involute(rb, t_max, left, true),
            via: polar(ra, centre),
        });
        for i in (0..FLANK_ARCS).rev() {
            segments.push(ProfileSegment::ArcTo {
                to: involute(rb, ts[2 * i], left, true),
                via: involute(rb, ts[2 * i + 1], left, true),
            });
        }
        segments.push(ProfileSegment::LineTo(polar(rf, left)));
    }
    // The last root arc closes the loop back to the start.
    let from = (TEETH - 1) as f64 * step + beta;
    segments.push(ProfileSegment::ArcTo {
        to: start,
        via: polar(rf, (from + core::f64::consts::TAU - beta) / 2.0),
    });
    Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path { start, segments },
        holes: Vec::new(),
    }
}

fn circle(radius: f64) -> Profile {
    Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Circle {
            center: Point2::new(0.0, 0.0),
            radius,
        },
        holes: Vec::new(),
    }
}

/// A quarter-turn-ish revolve: a rectangle off the y axis, turned 5 rad.
fn revolve_recipe(m: &mut Model) -> (arris::topo::Body, arris::topo::Provenance) {
    let p = |x, y| Point2::new(x, y);
    let profile = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: p(2.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p(4.0, 0.0)),
                ProfileSegment::LineTo(p(4.0, 3.0)),
                ProfileSegment::ArcTo {
                    to: p(2.0, 3.0),
                    via: p(3.0, 3.5),
                },
                ProfileSegment::LineTo(p(2.0, 0.0)),
            ],
        },
        holes: Vec::new(),
    };
    let axis = Axis::new(Point3::origin(), Vec3::y()).expect("the y axis is an axis");
    revolve(m, &profile, axis, 5.0).expect("the profile revolves")
}

/// A cylinder cut across by a thinner one along x.
fn boolean_recipe(m: &mut Model) -> (arris::topo::Body, arris::topo::Provenance) {
    let (a, _) =
        primitive_cylinder(m, Axis::z_at(Point3::origin()), 2.0, 6.0).expect("the cylinder builds");
    let along_x = Axis::new(Point3::new(-4.0, 0.3, 3.0), Vec3::x()).expect("an axis");
    let (b, _) = primitive_cylinder(m, along_x, 0.7, 8.0).expect("the cylinder builds");
    cut(m, a, b).expect("the cylinders cut")
}

/// A box with all twelve edges rounded.
fn fillet_recipe(m: &mut Model) -> (arris::topo::Body, arris::topo::Provenance) {
    let (b, _) =
        primitive_box(m, Point3::origin(), Point3::new(4.0, 3.0, 2.0)).expect("the box builds");
    let edges = m.edges(b).expect("the box has edges");
    fillet(m, b, &edges, 0.4).expect("the box fillets")
}

/// A round pipe along a line and a 60° arc, every span analytic.
fn sweep_recipe(m: &mut Model) -> (arris::topo::Body, arris::topo::Provenance) {
    let (r, a) = (8.0, core::f64::consts::FRAC_PI_3);
    let at = |t: f64| Point3::new(r * (1.0 - libm::cos(t)), 0.0, 10.0 + r * libm::sin(t));
    let path = Path {
        start: Point3::origin(),
        segments: vec![
            PathSegment::LineTo(Point3::new(0.0, 0.0, 10.0)),
            PathSegment::ArcTo {
                via: at(a / 2.0),
                to: at(a),
            },
        ],
    };
    arris::ops::sweep(
        m,
        &circle(1.5),
        &path,
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .expect("the pipe sweeps")
}

/// A round wire along a cubic NURBS in space.
fn nurbs_sweep_recipe(m: &mut Model) -> (arris::topo::Body, arris::topo::Provenance) {
    let curve = NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(10.0, 8.0, 18.0),
            Point3::new(20.0, 12.0, 20.0),
        ],
        vec![1.0; 4],
    )
    .expect("the curve is valid");
    let path = Path {
        start: Point3::origin(),
        segments: vec![PathSegment::Nurbs(curve)],
    };
    arris::ops::sweep(
        m,
        &circle(1.5),
        &path,
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .expect("the wire sweeps")
}

/// The committed half ball, a sphere face through its poles, read.
fn step_recipe(m: &mut Model) -> (arris::topo::Body, arris::topo::Provenance) {
    const HALF_BALL: &str =
        include_str!("../../../tests/fixtures/real/edge-through-sphere-pole/half-ball.stp");
    let read = step_read(m, HALF_BALL, &ReadOptions::default()).expect("the half ball reads");
    let solid = read
        .solids
        .into_iter()
        .next()
        .expect("the half ball has a solid");
    let body = solid.result.expect("the half ball builds");
    (body.body, body.provenance)
}

/// The tessellation of the boolean recipe: positions and triangles.
fn tessellation_record(m: &mut Model) -> serde_json::Value {
    let (body, _) = boolean_recipe(m);
    let mesh = tessellate(m, body, 0.01).expect("the body tessellates");
    let mut bytes = Vec::new();
    for p in mesh.positions().iter().flatten() {
        bytes.extend_from_slice(&p.to_bits().to_le_bytes());
    }
    for i in mesh.triangles().iter().flatten() {
        bytes.extend_from_slice(&i.to_le_bytes());
    }
    serde_json::json!({
        "mesh_sha256": sha256(&bytes),
        "triangles": mesh.triangles().len(),
    })
}

/// A recipe: builds a body into the model.
type Recipe = fn(&mut Model) -> (arris::topo::Body, arris::topo::Provenance);

/// The recipes, by name, in the order the digests file lists them.
fn recipes() -> BTreeMap<&'static str, serde_json::Value> {
    let mut out = BTreeMap::new();
    let mut m = Model::default();
    let (body, prov) =
        extrude(&mut m, &gear_profile(), Vec3::new(0.0, 0.0, 1.0), 8.0).expect("the gear extrudes");
    out.insert("gear-extrude", record(&m, body, &prov));
    let builds: [(&'static str, Recipe); 6] = [
        ("revolve", revolve_recipe),
        ("boolean-cut", boolean_recipe),
        ("fillet", fillet_recipe),
        ("sweep-analytic", sweep_recipe),
        ("sweep-nurbs", nurbs_sweep_recipe),
        ("step-read", step_recipe),
    ];
    for (name, build) in builds {
        let mut m = Model::default();
        let (body, prov) = build(&mut m);
        out.insert(name, record(&m, body, &prov));
    }
    out.insert("tessellation", tessellation_record(&mut Model::default()));
    out
}

#[test]
fn the_gear_has_40_lines_and_200_arcs() {
    let ProfileLoop::Path { segments, .. } = gear_profile().outer else {
        panic!("the gear's outline is a path");
    };
    let arcs = segments
        .iter()
        .filter(|s| matches!(s, ProfileSegment::ArcTo { .. }))
        .count();
    assert_eq!((segments.len() - arcs, arcs), (40, 200));
}

#[test]
fn every_recipe_matches_its_digest() {
    let got = serde_json::to_value(recipes()).expect("records serialise");
    #[cfg(not(target_arch = "wasm32"))]
    if std::env::var_os("ARRIS_BLESS").is_some() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/determinism/digests.json"
        );
        let text = serde_json::to_string_pretty(&got).expect("records print") + "\n";
        std::fs::write(path, text).expect("the digests file writes");
        return;
    }
    let want: serde_json::Value = serde_json::from_str(DIGESTS).expect("digests.json parses");
    assert_eq!(
        got,
        want,
        "a body differs from its committed digest on {}",
        std::env::consts::ARCH
    );
}
