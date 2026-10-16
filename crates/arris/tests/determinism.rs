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

use arris::geom::{Profile, ProfileLoop, ProfileSegment};
use arris::math::{Frame, Point2, Vec3};
use arris::topo::Model;
use arris_debug::unmetered::{extrude, mass_properties};
use sha2::{Digest, Sha256};

/// The committed digests, embedded so a wasm run needs no filesystem.
const DIGESTS: &str = include_str!("determinism/digests.json");

/// One recipe's result: the body's native bytes digested, and its volume
/// and area printed shortest-round-trip, so a last-ulp move shows.
fn record(m: &Model, body: arris::topo::Body, prov: &arris::topo::Provenance) -> serde_json::Value {
    let bytes = arris::io::body::write(m, body, prov).expect("a built body writes");
    let hex: String = Sha256::digest(&bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    let mp = mass_properties(m, body).expect("a built body measures");
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

/// The recipes, by name, in the order the digests file lists them.
fn recipes() -> BTreeMap<&'static str, serde_json::Value> {
    let mut out = BTreeMap::new();
    let mut m = Model::default();
    let (body, prov) =
        extrude(&mut m, &gear_profile(), Vec3::new(0.0, 0.0, 1.0), 8.0).expect("the gear extrudes");
    out.insert("gear-extrude", record(&m, body, &prov));
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
#[cfg_attr(
    target_arch = "wasm32",
    ignore = "arc-sweep-differs-on-wasm32: std's atan2 is glibc's natively and wasi-libc's on \
              wasm32-wasip1, so the gear's arcs differ by an ulp (docs/BACKLOG.md)"
)]
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
