//! A sheet body as an operand (plans/step-unit-and-sheet-bodies step 5):
//! a planar face and two read surface models — a cylinder patch and the
//! three faces of a box's corner — through `transform` and `mirror` (kind
//! kept, surface properties carried by the motion, a mirror's front turned
//! as its faces are), tessellation (every triangle's normal on the
//! sheet's front, every mesh point on its face), the body bytes and the
//! native format (a round trip identical), the checker at `Full`; and the
//! operations a sheet is not an operand of, each refusing it as
//! `InputReason::NotSolid`. Surface properties are invariant under random
//! rigid motions and mirrors.

use arris_check::{Level, check};
use arris_debug::fixtures;
use arris_debug::testing::fail;
use arris_debug::unmetered::{planar_face, step_read, surface_properties};
use arris_debug::{dump_text, prop, prop_shards};
use arris_geom::{Profile, ProfileLoop, ProfileSegment};
use arris_io::step::ReadOptions;
use arris_io::{body, native};
use arris_math::nalgebra::{Matrix3, UnitQuaternion};
use arris_math::{Axis, Control, Frame, Isometry, Point2, Point3, Reflection, Vec3};
use arris_mesh::tessellate;
use arris_ops::{InputReason, OpError, Reason};
use arris_topo::entity::BodyKind;
use arris_topo::{Body, Model, Orientation, Provenance, Shape};
use proptest::prelude::*;

const NONE: &Control<'static> = &Control::NONE;

/// The chord the sheets are meshed at.
const CHORD: f64 = 1e-2;

/// How near a motion's image must be to the motion of the properties.
const REL: f64 = 1e-9;

fn p(u: f64, v: f64) -> Point2 {
    Point2::new(u, v)
}

/// A 12 × 8 plate with a bore, on a tilted plane off the origin.
fn plate() -> Profile {
    Profile {
        plane: Frame::new(
            Point3::new(1.0, -2.0, 3.0),
            Vec3::new(1.0, 2.0, 2.0),
            Vec3::new(2.0, 0.0, -1.0),
        )
        .unwrap(),
        outer: ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p(12.0, 0.0)),
                ProfileSegment::LineTo(p(12.0, 8.0)),
                ProfileSegment::LineTo(p(0.0, 8.0)),
                ProfileSegment::LineTo(p(0.0, 0.0)),
            ],
        },
        holes: vec![ProfileLoop::Circle {
            center: p(6.0, 4.0),
            radius: 2.0,
        }],
    }
}

/// The text of a surface-model file under `tests/fixtures/real/`.
fn file(path: &str) -> String {
    std::fs::read_to_string(fixtures::corpus_root().join("real").join(path)).unwrap()
}

/// The only body the file reads, a sheet.
fn read_sheet(m: &mut Model, text: &str) -> Body {
    let mut read = step_read(m, text, &ReadOptions::default()).unwrap();
    assert_eq!(read.solids.len(), 1);
    let sheet = read.solids.remove(0).result.unwrap().body;
    assert_eq!(m.body(sheet.id).unwrap().kind(), BodyKind::Sheet);
    sheet
}

/// The sheets under test, each in its own model: a built planar face and
/// the surface models read from three files.
fn samples() -> Vec<(&'static str, Model, Body)> {
    let mut out = Vec::new();
    let mut m = Model::default();
    let (face, _) = planar_face(&mut m, &plate()).unwrap();
    out.push(("a planar face", m, face));
    for (name, path) in [
        (
            "an extract (a cylinder patch)",
            "surface-model-extract/extract.stp",
        ),
        (
            "a corner of three faces",
            "face-based-surface-model/corner.stp",
        ),
        ("a fill (a tilted plane)", "surface-model-fill/fill.stp"),
    ] {
        let mut m = Model::default();
        let sheet = read_sheet(&mut m, &file(path));
        out.push((name, m, sheet));
    }
    out
}

/// The matrix of a linear map, from its images of the basis.
fn matrix(f: impl Fn(Vec3) -> Vec3) -> Matrix3<f64> {
    let columns = [Vec3::x(), Vec3::y(), Vec3::z()].map(f);
    Matrix3::from_columns(&columns)
}

fn full_and_clean(m: &Model, body: Body, what: &str) {
    let report = check(m, body, Level::Full);
    assert!(
        report.is_ok() && report.unchecked().is_empty(),
        "{what}: {report}"
    );
}

/// `image`'s surface properties against `original`'s carried by the linear
/// map with matrix `l` (a rotation or a reflection) at `place`.
fn carried(
    m: &Model,
    original: Body,
    image: Body,
    place: impl Fn(Point3) -> Point3,
    l: &Matrix3<f64>,
) -> Result<(), TestCaseError> {
    let a = surface_properties(m, original).map_err(fail)?;
    let b = surface_properties(m, image).map_err(fail)?;
    prop_assert!(
        (a.area - b.area).abs() <= REL * a.area,
        "area {} vs {}",
        a.area,
        b.area
    );
    let moved = place(a.centroid);
    let scale = 1.0 + moved.coords.norm();
    prop_assert!(
        (b.centroid - moved).norm() <= REL * scale,
        "centroid {} vs {}",
        b.centroid,
        moved
    );
    let carried = l * a.inertia * l.transpose();
    let size = a.inertia.norm().max(f64::MIN_POSITIVE);
    prop_assert!(
        (b.inertia - carried).norm() <= 1e-7 * size,
        "inertia {} vs {}",
        b.inertia,
        carried
    );
    Ok(())
}

fn motion() -> Isometry {
    Isometry::new(
        UnitQuaternion::from_euler_angles(0.3, -0.7, 1.1),
        Vec3::new(5.0, -3.0, 2.0),
    )
}

fn mirror_plane() -> Reflection {
    Reflection::new(Point3::new(0.5, 1.0, -2.0), Vec3::new(1.0, -2.0, 0.5)).unwrap()
}

#[test]
fn a_sheet_passes_the_checker_at_full() {
    for (name, m, sheet) in samples() {
        full_and_clean(&m, sheet, name);
    }
}

#[test]
fn transform_keeps_the_kind_and_carries_the_surface_properties() {
    for (name, mut m, sheet) in samples() {
        let (moved, record) = arris_ops::transform(&mut m, sheet, &motion(), NONE).unwrap();
        assert_eq!(m.body(moved.id).unwrap().kind(), BodyKind::Sheet, "{name}");
        assert_eq!(m.faces(moved).unwrap().len(), m.faces(sheet).unwrap().len());
        assert_eq!(record.modified_from(Shape::from(sheet)).len(), 1, "{name}");
        full_and_clean(&m, moved, name);
        let motion = motion();
        let l = matrix(|v| motion.apply_vec(v));
        carried(&m, sheet, moved, |c| motion.apply(c), &l)
            .unwrap_or_else(|e| panic!("{name}: {e}"));
    }
}

#[test]
fn mirror_keeps_the_kind_and_carries_the_surface_properties() {
    for (name, mut m, sheet) in samples() {
        let plane = mirror_plane();
        let (image, record) = arris_ops::mirror(&mut m, sheet, &plane, NONE).unwrap();
        assert_eq!(m.body(image.id).unwrap().kind(), BodyKind::Sheet, "{name}");
        assert_eq!(record.modified_from(Shape::from(sheet)).len(), 1, "{name}");
        full_and_clean(&m, image, name);
        let l = matrix(|v| plane.apply_vec(v));
        carried(&m, sheet, image, |c| plane.apply(c), &l).unwrap_or_else(|e| panic!("{name}: {e}"));
    }
}

/// The front normal of a face at a point of it: the surface's normal at
/// the point's projection, turned by the face's use.
fn front_at(m: &Model, face: arris_topo::Face, at: Point3) -> Vec3 {
    let surface = m.surface(m.face(face.id).unwrap().surface()).unwrap();
    let uv = surface.project(at).unwrap().uv;
    let n = surface.normal(uv.x, uv.y).unwrap().into_inner();
    match face.orientation {
        Orientation::Forward => n,
        Orientation::Reversed => -n,
    }
}

/// Every triangle of every face of `sheet` at `CHORD` has its normal on
/// the face's front, and every mesh point of it lies on the face's
/// surface; returns the mesh's area-weighted mean front per face.
fn mesh_on_the_front(m: &Model, sheet: Body, what: &str) -> Vec<Vec3> {
    let mesh = tessellate(m, sheet, CHORD, NONE).unwrap();
    assert!(!mesh.is_closed(), "{what}: a sheet's mesh is open");
    let mut fronts = Vec::new();
    for face in m.faces(sheet).unwrap() {
        let surface = m.surface(m.face(face.id).unwrap().surface()).unwrap();
        let mut mean = Vec3::zeros();
        let triangles = mesh.face_triangles(face.id).expect("a mesh of every face");
        assert!(!triangles.is_empty(), "{what}: a face with no triangle");
        for t in triangles {
            let [a, b, c] = t.map(|i| {
                let q = mesh.positions()[i as usize];
                Point3::new(q[0], q[1], q[2])
            });
            for q in [a, b, c] {
                let d = surface.project(q).unwrap().distance;
                assert!(d <= 1e-6, "{what}: a mesh point {d} off its surface");
            }
            let cross = (b - a).cross(&(c - a));
            let centre = Point3::from((a.coords + b.coords + c.coords) / 3.0);
            let front = front_at(m, face, centre);
            assert!(
                cross.dot(&front) > 0.0,
                "{what}: a triangle faces away from the front"
            );
            mean += cross * 0.5;
        }
        fronts.push(mean.normalize());
    }
    fronts
}

#[test]
fn a_sheet_meshes_on_its_front_and_so_does_its_image() {
    for (name, mut m, sheet) in samples() {
        let before = mesh_on_the_front(&m, sheet, name);
        let area = surface_properties(&m, sheet).unwrap().area;
        let mesh = tessellate(&m, sheet, CHORD, NONE).unwrap();
        assert!(
            (mesh.area() - area).abs() <= 1e-2 * area,
            "{name}: mesh area {} vs {area}",
            mesh.area()
        );
        let motion = motion();
        let (moved, _) = arris_ops::transform(&mut m, sheet, &motion, NONE).unwrap();
        for (a, b) in before.iter().zip(mesh_on_the_front(&m, moved, name)) {
            assert!((motion.apply_vec(*a) - b).norm() < 1e-2, "{name}");
        }
        // A mirror turns the fronts as it turns the faces: the image's
        // front is the reflection of the original's.
        let plane = mirror_plane();
        let (image, _) = arris_ops::mirror(&mut m, sheet, &plane, NONE).unwrap();
        for (a, b) in before.iter().zip(mesh_on_the_front(&m, image, name)) {
            assert!((plane.apply_vec(*a) - b).norm() < 1e-2, "{name}");
        }
    }
}

#[test]
fn body_bytes_round_trip_a_sheet() {
    for (name, m, sheet) in samples() {
        let bytes = body::write(&m, sheet, &Provenance::default()).unwrap();
        let mut fresh = Model::default();
        let back = body::read(&mut fresh, &bytes, NONE).unwrap();
        assert_eq!(fresh.body(back.body.id).unwrap().kind(), BodyKind::Sheet);
        full_and_clean(&fresh, back.body, name);
        let again = body::write(&fresh, back.body, &Provenance::default()).unwrap();
        assert_eq!(again, bytes, "{name}: the bytes of the bytes");
        let a = surface_properties(&m, sheet).unwrap();
        let b = surface_properties(&fresh, back.body).unwrap();
        assert_eq!(a, b, "{name}: the same numbers");
    }
}

#[test]
fn the_native_format_round_trips_a_sheet() {
    for (name, m, sheet) in samples() {
        let bytes = native::to_bytes(&m).unwrap();
        let back = native::from_bytes(&bytes).unwrap();
        assert_eq!(
            back.body(sheet.id).unwrap().kind(),
            BodyKind::Sheet,
            "{name}"
        );
        assert_eq!(
            dump_text(&back, sheet).unwrap(),
            dump_text(&m, sheet).unwrap(),
            "{name}"
        );
        assert_eq!(native::to_bytes(&back).unwrap(), bytes, "{name}");
    }
}

/// A solid for the operations that take one beside the sheet.
fn solid(m: &mut Model) -> Body {
    arris_ops::primitive_cylinder(m, Axis::z_at(Point3::new(0.0, 0.0, -5.0)), 4.0, 12.0, NONE)
        .unwrap()
        .0
}

fn not_solid(result: Result<impl Sized, OpError>, sheet: Body, what: &str) {
    match result {
        Err(OpError::Degenerate { entities, reason }) => {
            assert_eq!(reason, Reason::Input(InputReason::NotSolid), "{what}");
            assert_eq!(entities, vec![Shape::from(sheet)], "{what}");
        }
        Err(other) => panic!("{what}: {other:?}"),
        Ok(_) => panic!("{what}: a sheet was accepted"),
    }
}

/// The operations that need a body with an inside refuse a sheet by name,
/// the model untouched.
#[test]
fn the_operations_that_need_an_inside_refuse_a_sheet_as_not_solid() {
    let (_, mut m, sheet) = samples().remove(0);
    let block = solid(&mut m);
    let plane = Frame::world();
    let edges = m.edges(sheet).unwrap();
    let faces = m.faces(sheet).unwrap();
    let before = dump_text(&m, sheet).unwrap();

    not_solid(
        arris_ops::measure::mass_properties(&m, sheet, NONE),
        sheet,
        "mass_properties",
    );
    not_solid(arris_ops::fuse(&mut m, block, sheet, NONE), sheet, "fuse");
    not_solid(
        arris_ops::fuse(&mut m, sheet, block, NONE),
        sheet,
        "fuse, sheet first",
    );
    not_solid(
        arris_ops::cut(&mut m, block, sheet, NONE),
        sheet,
        "cut tool",
    );
    not_solid(
        arris_ops::cut(&mut m, sheet, block, NONE),
        sheet,
        "cut target",
    );
    not_solid(
        arris_ops::common(&mut m, block, sheet, NONE),
        sheet,
        "common",
    );
    not_solid(
        arris_ops::cut_many(&mut m, block, &[sheet], NONE),
        sheet,
        "cut_many",
    );
    not_solid(
        arris_ops::fuse_many(&mut m, &[block, sheet], NONE),
        sheet,
        "fuse_many",
    );
    not_solid(
        arris_ops::split(&mut m, sheet, &plane, NONE),
        sheet,
        "split",
    );
    not_solid(
        arris_ops::section(&mut m, sheet, &plane, NONE),
        sheet,
        "section",
    );
    not_solid(
        arris_ops::fillet(&mut m, sheet, &edges[..1], 0.1, NONE),
        sheet,
        "fillet",
    );
    not_solid(
        arris_ops::chamfer(&mut m, sheet, &edges[..1], 0.1, NONE),
        sheet,
        "chamfer",
    );
    not_solid(
        arris_ops::offset_faces(&mut m, sheet, &faces, 0.1, NONE),
        sheet,
        "offset_faces",
    );
    not_solid(
        arris_ops::shell(&mut m, sheet, &[], 0.1, arris_ops::ShellSide::Inward, NONE),
        sheet,
        "shell",
    );
    assert_eq!(
        dump_text(&m, sheet).unwrap(),
        before,
        "the sheet is as it was"
    );
}

// A surface-property invariant: any sheet of the general profiles, moved
// by a random rigid motion or mirrored in a random plane, keeps its area
// and carries its centroid and inertia.
fn invariant_under(
    profile: &Profile,
    pose: &Isometry,
    plane: &Reflection,
) -> Result<(), TestCaseError> {
    let mut m = Model::default();
    let (sheet, _) = planar_face(&mut m, profile).map_err(fail)?;
    let (moved, _) = arris_ops::transform(&mut m, sheet, pose, NONE).map_err(fail)?;
    carried(
        &m,
        sheet,
        moved,
        |c| pose.apply(c),
        &matrix(|v| pose.apply_vec(v)),
    )?;
    let (image, _) = arris_ops::mirror(&mut m, sheet, plane, NONE).map_err(fail)?;
    carried(
        &m,
        sheet,
        image,
        |c| plane.apply(c),
        &matrix(|v| plane.apply_vec(v)),
    )?;
    Ok(())
}

prop_shards! {
    /// The surface properties of a planar face are invariant under random
    /// rigid motions and mirrors: the area kept, the centroid and the
    /// inertia tensor carried.
    surface_properties_are_invariant_under_motions_and_mirrors [shard_0 shard_1]
        ((sweep, pose, normal, origin)) = (
            prop::profile::general(),
            prop::pose(),
            prop::unit_vec3(),
            prop::point_in_box(20.0),
        ) => {
            let plane = Reflection::new(origin, normal.into_inner()).map_err(fail)?;
            invariant_under(&sweep.profile, &pose, &plane)
        }
}
