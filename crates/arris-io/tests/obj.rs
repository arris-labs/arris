//! OBJ round-trips through a parser this test owns
//! (`docs/ARCHITECTURE.md` §Formats and tools): the parsed positions,
//! (u, v)s, normals and both triangle index spaces reconstruct the
//! corner block exactly — every position an `f` line names is the one
//! `Corners` gives it, and every corner it does not (one whose own
//! triangle collapsed and was dropped, so nothing references it) is left
//! unchecked, since no OBJ file can say what it stood on; every `vt`
//! that is referenced evaluates on its face's surface to its `v` and
//! every `vn` is the outward normal there; the `g` groups partition the
//! triangles by face id in iteration order; a mesh with no corner block
//! writes `v`/`f`/`g` and no `vt`/`vn`; two writes are byte-identical.

use arris_debug::unmetered::{fillet, primitive_box};
use std::ops::Range;

use arris_debug::sample;
use arris_debug::unmetered::{tessellate, tessellate_with};
use arris_io::obj;
use arris_math::{Point3, Vec3};
use arris_mesh::{MeshRequest, TriMesh};
use arris_topo::Edge;
use arris_topo::{Body, Model, Orientation};
/// Rounding at the scale of a coordinate.
const EXACT: f64 = 1e-9;

/// One `g` group parsed back: its name (`FaceId`'s `Display` form) and
/// the triangle range it holds.
struct Group {
    name: String,
    triangles: Range<usize>,
}

struct Parsed {
    positions: Vec<[f64; 3]>,
    uvs: Vec<[f64; 2]>,
    normals: Vec<[f64; 3]>,
    /// Shared position index per triangle corner, parsed from `f`'s `v`.
    triangles: Vec<[u32; 3]>,
    /// Face-local vertex index per triangle corner, parsed from `f`'s
    /// `vt` (equal to its `vn`, since OBJ has no way to name one alone).
    corner_triangles: Vec<[u32; 3]>,
    /// The shared position each face-local vertex stands on, read off
    /// wherever an `f` line names both. `None` for a face-local vertex no
    /// triangle references — a corner whose own triangle collapsed and
    /// was dropped from both index spaces (`docs/ARCHITECTURE.md`
    /// §Tessellation) still gets a `vt`/`vn` line, to keep the two arrays
    /// parallel, but nothing in an OBJ file can then say what position it
    /// stood on.
    corner_positions: Vec<Option<u32>>,
    groups: Vec<Group>,
}

fn parse_obj(text: &str) -> Parsed {
    let mut positions = Vec::new();
    let mut uvs = Vec::new();
    let mut normals = Vec::new();
    let mut triangles = Vec::new();
    let mut corner_triangles = Vec::new();
    let mut corner_positions: Vec<Option<u32>> = Vec::new();
    let mut groups: Vec<Group> = Vec::new();

    let floats =
        |it: core::str::SplitWhitespace| -> Vec<f64> { it.map(|s| s.parse().unwrap()).collect() };

    for line in text.lines() {
        let mut it = line.split_whitespace();
        match it.next() {
            Some("v") => {
                let c = floats(it);
                positions.push([c[0], c[1], c[2]]);
            }
            Some("vt") => {
                let c = floats(it);
                uvs.push([c[0], c[1]]);
                corner_positions.push(None);
            }
            Some("vn") => {
                let c = floats(it);
                normals.push([c[0], c[1], c[2]]);
            }
            Some("g") => {
                let name = it.next().unwrap().to_string();
                groups.push(Group {
                    name,
                    triangles: triangles.len()..triangles.len(),
                });
            }
            Some("f") => {
                let group = groups.last_mut().expect("f after a g");
                let mut v = [0u32; 3];
                let mut c = [0u32; 3];
                let mut has_corners = false;
                for (k, field) in it.enumerate() {
                    let mut parts = field.split('/');
                    v[k] = parts.next().unwrap().parse::<u32>().unwrap() - 1;
                    if let Some(vt) = parts.next().filter(|s| !s.is_empty()) {
                        c[k] = vt.parse::<u32>().unwrap() - 1;
                        has_corners = true;
                        corner_positions[c[k] as usize] = Some(v[k]);
                    }
                }
                triangles.push(v);
                if has_corners {
                    corner_triangles.push(c);
                }
                group.triangles.end = triangles.len();
            }
            _ => {}
        }
    }
    Parsed {
        positions,
        uvs,
        normals,
        triangles,
        corner_triangles,
        corner_positions,
        groups,
    }
}

fn mesh_and_parsed(m: &Model, body: Body, chord: f64) -> (TriMesh, Parsed) {
    let mesh = tessellate_with(m, body, &MeshRequest::new(chord).with_corners()).unwrap();
    let text = obj::write(std::slice::from_ref(&mesh)).unwrap();
    let parsed = parse_obj(&text);
    (mesh, parsed)
}

fn filleted_box(m: &mut Model) -> Body {
    let (body, _) = primitive_box(m, Point3::origin(), Point3::new(2.0, 2.0, 2.0)).unwrap();
    let edge: Edge = m
        .edges(body)
        .unwrap()
        .into_iter()
        .find(|e| {
            let entity = m.edge(e.id).unwrap();
            entity.curve().is_some_and(|(curve, range)| {
                (m.curve(curve).unwrap().point(range.midpoint()) - Point3::new(2.0, 2.0, 1.0))
                    .norm()
                    < 1e-9
            })
        })
        .expect("the vertical edge at (2, 2)");
    fillet(m, body, &[edge], 0.3).unwrap().0
}

/// The parser's reading of an OBJ written with corners equals the
/// `TriMesh`/`Corners` it came from field for field, and every `vt`/`vn`
/// it read is the face's surface at that point.
fn assert_round_trips(m: &Model, body: Body, name: &str) {
    let chord = 1e-2;
    let (mesh, parsed) = mesh_and_parsed(m, body, chord);
    let corners = mesh.corners().expect("corners were asked for");

    assert_eq!(parsed.positions, mesh.positions(), "{name}: positions");
    assert_eq!(parsed.uvs, corners.uvs(), "{name}: uvs");
    assert_eq!(parsed.normals, corners.normals(), "{name}: normals");
    assert_eq!(parsed.triangles, mesh.triangles(), "{name}: triangles");
    assert_eq!(
        parsed.corner_triangles,
        corners.triangles(),
        "{name}: corner triangles"
    );
    // Every corner an `f` line named stands on the position `Corners`
    // says it does; a corner no triangle names (its own triangle
    // collapsed and was dropped) is `None` — the OBJ file has no way to
    // record its position, though its `vt`/`vn` are still written.
    for (i, (parsed_position, &original)) in parsed
        .corner_positions
        .iter()
        .zip(corners.positions())
        .enumerate()
    {
        if let Some(p) = parsed_position {
            assert_eq!(*p, original, "{name}: corner {i}: position");
        }
    }

    let faces = m.faces(body).unwrap();
    assert_eq!(parsed.groups.len(), faces.len(), "{name}: group count");
    for (used, group) in faces.iter().zip(&parsed.groups) {
        let cf = corners.face(used.id).expect("a CornerFace for this face");
        assert_eq!(group.name, used.id.to_string(), "{name}: group name order");
        assert_eq!(
            group.triangles,
            mesh.faces()
                .iter()
                .find(|f| f.face == used.id)
                .unwrap()
                .triangles,
            "{name}: {}: triangle range",
            used.id
        );

        let face = m.face(used.id).unwrap();
        let surface = m.surface(face.surface()).unwrap();
        let tol = face.tolerance();
        let reversed = used.orientation == Orientation::Reversed;
        for i in cf.vertices.clone() {
            // A corner no triangle references (dropped with a collapsed
            // triangle) has no position to check against.
            let Some(shared) = parsed.corner_positions[i] else {
                continue;
            };
            let [u, v] = parsed.uvs[i];
            let at = Point3::from(parsed.positions[shared as usize]);
            let on_surface = surface.point(u, v);
            assert!(
                (on_surface - at).norm() <= tol,
                "{name}: {}: vt ({u}, {v}) evaluates to {on_surface}, not {at}",
                used.id
            );
            if let Some(own) = surface.normal(u, v) {
                let outward = if reversed {
                    -own.into_inner()
                } else {
                    own.into_inner()
                };
                let n = Vec3::from(parsed.normals[i]);
                assert!(
                    (n - outward).norm() <= EXACT,
                    "{name}: {}: vn at ({u}, {v}) is {n}, not the outward {outward}",
                    used.id
                );
            }
        }
    }
}

#[test]
fn obj_of_every_sample_and_a_filleted_box_round_trips_with_corners() {
    let mut m = Model::default();
    let bodies = [
        (
            "cuboid",
            sample::cuboid(&mut m, Point3::origin(), Point3::new(3.0, 2.0, 1.0)).unwrap(),
        ),
        ("cylinder", sample::cylinder(&mut m, 4.0, 12.0).unwrap()),
        (
            "sphere",
            sample::sphere(&mut m, Point3::new(1.0, -2.0, 0.5), 3.0).unwrap(),
        ),
        (
            "torus",
            sample::torus(&mut m, Point3::origin(), 5.0, 2.0).unwrap(),
        ),
        ("filleted box", filleted_box(&mut m)),
    ];
    for (name, body) in bodies {
        assert_round_trips(&m, body, name);
    }
}

#[test]
fn a_mesh_without_corners_writes_no_vt_or_vn() {
    let mut m = Model::default();
    let body = sample::cuboid(&mut m, Point3::origin(), Point3::new(1.0, 1.0, 1.0)).unwrap();
    let mesh = tessellate(&m, body, 1e-2).unwrap();
    let text = obj::write(std::slice::from_ref(&mesh)).unwrap();
    assert!(!text.contains("vt "));
    assert!(!text.contains("vn "));
    let groups = text.lines().filter(|l| l.starts_with("g ")).count();
    assert_eq!(groups, mesh.faces().len(), "one g per face");
    let parsed = parse_obj(&text);
    assert_eq!(parsed.positions, mesh.positions());
    assert_eq!(parsed.triangles, mesh.triangles());
    assert!(parsed.corner_triangles.is_empty());
}

#[test]
fn two_writes_are_byte_identical() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let mesh = tessellate_with(&m, body, &MeshRequest::new(1e-2).with_corners()).unwrap();
    let one = std::slice::from_ref(&mesh);
    assert_eq!(obj::write(one).unwrap(), obj::write(one).unwrap());
}
