//! `measure::surface_properties` (C9): the area, centroid and inertia of a
//! body's faces at unit area density, against closed forms (a sphere, a
//! cylinder, a torus), against `mass_properties`' area on a battery of
//! solid fixtures, and against Open CASCADE's
//! `BRepGProp::SurfaceProperties` of the same recipe.

use core::f64::consts::PI;

use arris_debug::unmetered::{mass_properties, surface_properties};
use arris_debug::{corpus, fixtures, oracle, sample};
use arris_math::{Matrix3, Point3};
use arris_topo::Model;

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * a.abs().max(b.abs()).max(1e-300)
}

fn tensor(rows: [[f64; 3]; 3]) -> Matrix3 {
    Matrix3::from_fn(|i, j| rows[i][j])
}

#[test]
fn a_sphere_is_a_thin_shell() {
    let mut m = Model::default();
    let centre = Point3::new(1.0, -2.0, 3.0);
    let r = 2.5;
    let body = sample::sphere(&mut m, centre, r).unwrap();
    let p = surface_properties(&m, body).unwrap();
    let area = 4.0 * PI * r * r;
    assert!(close(p.area, area, 1e-11), "{} vs {area}", p.area);
    assert!((p.centroid - centre).norm() < 1e-10);
    for i in 0..3 {
        assert!(close(p.inertia[(i, i)], 2.0 / 3.0 * area * r * r, 1e-10));
    }
}

#[test]
fn a_cylinder_is_its_lateral_surface_and_caps() {
    let mut m = Model::default();
    let (r, h) = (4.0, 12.0);
    let body = sample::cylinder(&mut m, r, h).unwrap();
    let p = surface_properties(&m, body).unwrap();
    let (lateral, cap) = (2.0 * PI * r * h, PI * r * r);
    assert!(close(p.area, lateral + 2.0 * cap, 1e-11));
    // About its own axis: the lateral surface m r², the two discs m r²/2.
    let izz = lateral * r * r + 2.0 * cap * r * r / 2.0;
    assert!(
        close(p.inertia[(2, 2)], izz, 1e-10),
        "{}",
        p.inertia[(2, 2)]
    );
}

#[test]
fn a_torus_is_the_closed_form() {
    let mut m = Model::default();
    let (big, small) = (5.0, 2.0);
    let body = sample::torus(&mut m, Point3::origin(), big, small).unwrap();
    let p = surface_properties(&m, body).unwrap();
    let area = 4.0 * PI * PI * big * small;
    assert!(close(p.area, area, 1e-10), "{} vs {area}", p.area);
    assert!(p.centroid.coords.norm() < 1e-9);
    let izz = area * (big * big + 1.5 * small * small);
    let ixx = area * (big * big / 2.0 + 1.25 * small * small);
    assert!(
        close(p.inertia[(2, 2)], izz, 1e-10),
        "{}",
        p.inertia[(2, 2)]
    );
    assert!(
        close(p.inertia[(0, 0)], ixx, 1e-10),
        "{}",
        p.inertia[(0, 0)]
    );
    assert!(
        close(p.inertia[(1, 1)], ixx, 1e-10),
        "{}",
        p.inertia[(1, 1)]
    );
}

/// Solid fixtures whose faces are planes, quadrics and tori: the area is
/// `mass_properties`', the centroid and inertia Open CASCADE's.
const BATTERY: [&str; 9] = [
    "primitive/box",
    "primitive/cylinder",
    "boolean/blind-hole",
    "boolean/cross-cylinders-common",
    "boolean/ball-ball-common",
    "boolean/coaxial-fuse",
    "boolean/cone-apex-slice-cut",
    "sweep/revolve-frustum",
    "sweep/revolve-ring",
];

#[test]
fn a_solid_fixture_matches_mass_properties_area_and_open_cascade() {
    let root = fixtures::corpus_root();
    for name in BATTERY {
        let dir = root.join(name);
        let chain = corpus::chain(&dir, "default").unwrap();
        let body = chain.result().unwrap();
        let p = surface_properties(&chain.model, body).unwrap();
        let mass = mass_properties(&chain.model, body).unwrap();
        assert!(
            close(p.area, mass.area, 1e-9),
            "{name}: {} vs {}",
            p.area,
            mass.area
        );

        let occt = oracle::occt_surface(&dir, None).unwrap();
        assert!(
            close(p.area, occt.area, 1e-9),
            "{name}: area {} vs {}",
            p.area,
            occt.area
        );
        let c = occt.centroid;
        let apart = (p.centroid - Point3::new(c[0], c[1], c[2])).norm();
        assert!(
            apart <= 1e-7 * occt.area.sqrt(),
            "{name}: centroid {} vs {c:?}",
            p.centroid
        );
        let want = tensor(occt.inertia);
        let scale = want.iter().fold(0.0f64, |s, x| s.max(x.abs()));
        assert!(
            (p.inertia - want).iter().all(|d| d.abs() <= 1e-7 * scale),
            "{name}: inertia\n{}\nvs\n{want}",
            p.inertia
        );
    }
}
