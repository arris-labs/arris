//! `measure::surface_properties` (C9): the area, centroid and inertia of a
//! body's faces at unit area density, against closed forms (a sphere, a
//! cylinder, a torus), against `mass_properties`' area on a battery of
//! solid fixtures, and against Open CASCADE's
//! `BRepGProp::SurfaceProperties` of the same recipe.

use core::f64::consts::PI;

use arris_debug::unmetered::{
    face_surface_properties, mass_properties, revolve, split, surface_properties,
};
use arris_debug::{corpus, fixtures, oracle, sample};
use arris_geom::{Profile, ProfileLoop, ProfileSegment, SurfaceKind};
use arris_math::{Axis, Frame, Matrix3, Point2, Point3, Vec3};
use arris_ops::OpError;
use arris_topo::{Body, Face, FaceId, Model};
use core::f64::consts::TAU;

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

/// The faces of `body` whose surface is of `kind`, in the walk's order.
fn faces_of(m: &Model, body: Body, kind: SurfaceKind) -> Vec<Face> {
    m.faces(body)
        .unwrap()
        .into_iter()
        .filter(|f| m.surface(m.face(f.id).unwrap().surface()).unwrap().kind() == kind)
        .collect()
}

fn only(mut faces: Vec<Face>) -> Face {
    assert_eq!(faces.len(), 1, "{faces:?}");
    faces.remove(0)
}

#[test]
fn a_boxs_planar_faces_are_rectangles() {
    let mut m = Model::default();
    let body = sample::cuboid(
        &mut m,
        Point3::new(1.0, 2.0, 3.0),
        Point3::new(3.0, 5.0, 9.0),
    )
    .unwrap();
    let faces = m.faces(body).unwrap();
    assert_eq!(faces.len(), 6);
    let mut total = 0.0;
    for f in faces {
        let p = face_surface_properties(&m, f).unwrap();
        // A rectangle of sides a and b: area ab, inertia about the normal
        // through its centre A (a² + b²) / 12.
        let (lo, hi) = (Point3::new(1.0, 2.0, 3.0), Point3::new(3.0, 5.0, 9.0));
        let d = hi - lo;
        let normal = (0..3)
            .find(|&i| {
                (p.centroid[i] - lo[i]).abs() < 1e-12 || (p.centroid[i] - hi[i]).abs() < 1e-12
            })
            .unwrap();
        let (a, b) = match normal {
            0 => (d.y, d.z),
            1 => (d.x, d.z),
            _ => (d.x, d.y),
        };
        assert!(close(p.area, a * b, 1e-12), "{} vs {}", p.area, a * b);
        let mid = (lo[(normal + 1) % 3] + hi[(normal + 1) % 3]) / 2.0;
        assert!((p.centroid[(normal + 1) % 3] - mid).abs() < 1e-12);
        assert!(close(
            p.inertia[(normal, normal)],
            p.area * (a * a + b * b) / 12.0,
            1e-11
        ));
        total += p.area;
    }
    let whole = surface_properties(&m, body).unwrap();
    assert!(close(total, whole.area, 1e-12));
}

#[test]
fn a_cylinders_wall_centres_on_its_axis() {
    let mut m = Model::default();
    let (r, h) = (3.0, 7.0);
    let body = sample::cylinder(&mut m, r, h).unwrap();
    let wall = only(faces_of(&m, body, SurfaceKind::Cylinder));
    let p = face_surface_properties(&m, wall).unwrap();
    let area = TAU * r * h;
    assert!(close(p.area, area, 1e-11), "{} vs {area}", p.area);
    assert!((p.centroid - Point3::new(0.0, 0.0, h / 2.0)).norm() < 1e-10);
    // About the axis the wall is a thin ring of mass A: A r².
    assert!(close(p.inertia[(2, 2)], area * r * r, 1e-10));
}

#[test]
fn a_half_cylinders_wall_centres_two_r_over_pi_off_its_axis() {
    let mut m = Model::default();
    let (r, h) = (3.0, 7.0);
    let body = sample::cylinder(&mut m, r, h).unwrap();
    // The plane holds the axis and the seam, so each half is one wall face.
    let plane = Frame::from_z(Point3::origin(), Vec3::y()).unwrap();
    let halves = split(&mut m, body, &plane).unwrap();
    for (side, sign) in [(halves.positive, 1.0), (halves.negative, -1.0)] {
        let wall = only(faces_of(&m, side, SurfaceKind::Cylinder));
        let p = face_surface_properties(&m, wall).unwrap();
        let area = PI * r * h;
        assert!(close(p.area, area, 1e-11), "{} vs {area}", p.area);
        let want = Point3::new(0.0, sign * 2.0 * r / PI, h / 2.0);
        assert!(
            (p.centroid - want).norm() < 1e-10,
            "{} vs {want}",
            p.centroid
        );
        // About the cylinder's axis every point is at r: A r².
        let about = p.inertia_about(Point3::new(0.0, 0.0, h / 2.0));
        assert!(
            close(about[(2, 2)], area * r * r, 1e-10),
            "{}",
            about[(2, 2)]
        );
    }
}

#[test]
fn a_frustums_wall_centres_at_the_closed_form_height() {
    let mut m = Model::default();
    let (big, small, h) = (4.0, 1.5, 6.0);
    let p2 = |u, v| Point2::new(u, v);
    let profile = Profile {
        plane: Frame::new(Point3::origin(), -Vec3::y(), Vec3::x()).unwrap(),
        outer: ProfileLoop::Path {
            start: p2(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p2(big, 0.0)),
                ProfileSegment::LineTo(p2(small, h)),
                ProfileSegment::LineTo(p2(0.0, h)),
                ProfileSegment::LineTo(p2(0.0, 0.0)),
            ],
        },
        holes: Vec::new(),
    };
    let (body, _) = revolve(&mut m, &profile, Axis::z_at(Point3::origin()), TAU).unwrap();
    let wall = only(faces_of(&m, body, SurfaceKind::Cone));
    let p = face_surface_properties(&m, wall).unwrap();
    let slant = (h * h + (big - small) * (big - small)).sqrt();
    let area = PI * (big + small) * slant;
    assert!(close(p.area, area, 1e-11), "{} vs {area}", p.area);
    let zbar = h * (big + 2.0 * small) / (3.0 * (big + small));
    assert!(
        (p.centroid - Point3::new(0.0, 0.0, zbar)).norm() < 1e-10,
        "{}",
        p.centroid
    );
    // About the axis: the integral of r² over the lateral surface,
    // 2 pi slant / h ∫ r(z)³ dz with r linear from big to small.
    let izz = PI * slant * (big * big + small * small) * (big + small) / 2.0;
    assert!(
        close(p.inertia[(2, 2)], izz, 1e-10),
        "{} vs {izz}",
        p.inertia[(2, 2)]
    );
}

#[test]
fn a_hemispheres_dome_centres_half_a_radius_above_the_equator() {
    let mut m = Model::default();
    let (centre, r) = (Point3::new(1.0, -2.0, 3.0), 2.5);
    let body = sample::sphere(&mut m, centre, r).unwrap();
    let plane = Frame::from_z(centre, Vec3::z()).unwrap();
    let halves = split(&mut m, body, &plane).unwrap();
    let dome = only(faces_of(&m, halves.positive, SurfaceKind::Sphere));
    let p = face_surface_properties(&m, dome).unwrap();
    let area = 2.0 * PI * r * r;
    assert!(close(p.area, area, 1e-11), "{} vs {area}", p.area);
    let want = centre + Vec3::new(0.0, 0.0, r / 2.0);
    assert!(
        (p.centroid - want).norm() < 1e-10,
        "{} vs {want}",
        p.centroid
    );
    // A hemispherical shell about its pole axis: (2/3) A r².
    assert!(close(
        p.inertia_about(centre)[(2, 2)],
        2.0 / 3.0 * area * r * r,
        1e-10
    ));
}

#[test]
fn a_torus_face_is_the_closed_form() {
    let mut m = Model::default();
    let (big, small) = (5.0, 2.0);
    let body = sample::torus(&mut m, Point3::origin(), big, small).unwrap();
    let face = only(m.faces(body).unwrap());
    let p = face_surface_properties(&m, face).unwrap();
    let whole = surface_properties(&m, body).unwrap();
    assert_eq!(p.area.to_bits(), whole.area.to_bits());
    assert!(close(p.area, 4.0 * PI * PI * big * small, 1e-10));
    assert!(p.centroid.coords.norm() < 1e-9);
}

#[test]
fn a_planar_face_alone_and_a_reversed_use_agree_to_the_bit() {
    let mut m = Model::default();
    let body = sample::unit_box(&mut m).unwrap();
    for face in m.faces(body).unwrap() {
        let a = face_surface_properties(&m, face).unwrap();
        let b = face_surface_properties(&m, face.reversed()).unwrap();
        assert_eq!(a.area.to_bits(), b.area.to_bits());
        assert_eq!(a.centroid, b.centroid);
        assert_eq!(a.inertia, b.inertia);
        assert!(close(a.area, 1.0, 1e-12));
    }
}

#[test]
fn a_face_that_does_not_resolve_is_not_found() {
    let mut m = Model::default();
    let body = sample::unit_box(&mut m).unwrap();
    let real = m.faces(body).unwrap()[0];
    let gone = Face::forward(FaceId::new(real.id.index() + 1000, 0));
    assert!(matches!(
        face_surface_properties(&m, gone),
        Err(OpError::NotFound(_))
    ));
}
