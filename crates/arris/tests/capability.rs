//! The capability matrix (ADR-0067): what each operation
//! does with each kind of face, one representative case per cell, built
//! through the public operations on a model of its own.
//!
//! Rows are operations, columns are the kind of face the operation meets.
//! A cell reads `ok` when the operation returned and the result passes the
//! checker at `Full` with nothing unchecked; `unchecked S5` when it passes
//! except for a pair of faces the intersector does not decide (a NURBS
//! surface against another in a pose it refuses); `-` when the column's body
//! has nothing for the row to act on (no edge between two faces of a full
//! torus); otherwise the typed refusal's kind, `INTERNAL` and the fault's
//! kind for a kernel fault, `checker-red` and the row's code, or `PANIC`.
//! The table is `capability.txt`, a Markdown table the README and
//! `docs/COMPARISON.md` quote: each column's cells are held to it in a test
//! of their own (the two rows that cost seconds a cell are `_heavy` shards,
//! so no test is a large share of a gate), in both directions, so a refusal
//! that becomes `ok` is as visible as one that appears. `ARRIS_BLESS=1
//! cargo nextest run -p arris --test capability --test-threads 1` rewrites
//! it. A cell that faults rather than refuses is a `regression/` fixture and
//! a line of `docs/BACKLOG.md`, never an accepted cell.
//!
//! Every column's body spans the box [-5, 5]² × [0, 5]. A pairwise row meets
//! it with a plane-faced box through its +x side, or with a cylinder in the
//! plane column (a box against a box is the other cells' ground). `sweep
//! operand` and `loft operand` cut the body with a bent pipe (a torus) and
//! a smoothly lofted blob (a NURBS surface), so those two rows are
//! the column against a swept and a lofted tool.

use arris::Control;
use arris::check::{Level, check};
use arris::geom::{FrameLaw, Path, PathSegment, Profile, ProfileLoop, ProfileSegment, SurfaceKind};
use arris::io::step;
use arris::math::{Axis, Frame, Point2, Point3, Vec2, Vec3};
use arris::ops::query::{Operand, Ray, RayHits, distance, ray_fire};
use arris::ops::{LoftKind, OpError, ShellSide, loft, sweep};
use arris::topo::{Body, Edge, Face, Model, Provenance, Shape};
use arris_debug::unmetered::{
    chamfer, common, cut, extrude, fillet, fuse, mass_properties, offset_faces, primitive_box,
    primitive_cylinder, revolve, section, shell, split,
};
use std::collections::BTreeSet;
use std::panic::{AssertUnwindSafe, catch_unwind};

/// The columns: the kind of face the operation meets.
const COLUMNS: [(&str, SurfaceKind); 7] = [
    ("plane", SurfaceKind::Plane),
    ("cylinder", SurfaceKind::Cylinder),
    ("elliptic cylinder", SurfaceKind::EllipticCylinder),
    ("cone", SurfaceKind::Cone),
    ("sphere", SurfaceKind::Sphere),
    ("torus", SurfaceKind::Torus),
    ("NURBS", SurfaceKind::Nurbs),
];

/// The rows: the operations.
const ROWS: [&str; 14] = [
    "cut",
    "fuse",
    "common",
    "split",
    "section",
    "fillet",
    "chamfer",
    "offset",
    "shell",
    "sweep operand",
    "loft operand",
    "ray",
    "distance",
    "STEP round trip",
];

/// The rows that cost seconds a cell (a NURBS blend, a boolean against a
/// lofted tool): their own shards, so no test is a large share of a gate.
const HEAVY: [&str; 2] = ["fillet", "loft operand"];

/// The size of every fillet, chamfer, offset and shell.
const SMALL: f64 = 0.3;

fn pt(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

fn flat(plane: Frame, outer: ProfileLoop) -> Profile {
    Profile {
        plane,
        outer,
        holes: vec![],
    }
}

/// A profile in the xz plane (its u is x, its v is z), through the points.
fn in_xz(points: &[(f64, f64)]) -> Profile {
    let plane = Frame::new(Point3::origin(), -Vec3::y(), Vec3::x()).unwrap();
    let segments = points[1..]
        .iter()
        .chain(&points[..1])
        .map(|&(u, v)| ProfileSegment::LineTo(Point2::new(u, v)))
        .collect();
    flat(
        plane,
        ProfileLoop::Path {
            start: Point2::new(points[0].0, points[0].1),
            segments,
        },
    )
}

/// The column's body, in a model of its own.
fn body_of(m: &mut Model, kind: SurfaceKind) -> Body {
    let up = Vec3::z();
    let below = Frame::new(Point3::origin(), up, Vec3::x()).unwrap();
    let xz = Frame::new(Point3::origin(), -Vec3::y(), Vec3::x()).unwrap();
    match kind {
        SurfaceKind::Plane => primitive_box(m, pt(-5.0, -5.0, 0.0), pt(5.0, 5.0, 5.0)),
        SurfaceKind::Cylinder => primitive_cylinder(m, Axis::z_at(Point3::origin()), 4.0, 5.0),
        SurfaceKind::EllipticCylinder => {
            let outer = ProfileLoop::Ellipse {
                center: Point2::origin(),
                major: Vec2::new(5.0, 0.0),
                minor_radius: 3.0,
            };
            extrude(m, &flat(below, outer), up, 5.0)
        }
        SurfaceKind::Cone => revolve(
            m,
            &in_xz(&[(0.0, 0.0), (5.0, 0.0), (2.5, 5.0), (0.0, 5.0)]),
            Axis::z_at(Point3::origin()),
            std::f64::consts::TAU,
        ),
        SurfaceKind::Sphere => {
            let c = std::f64::consts::FRAC_1_SQRT_2 * 5.0;
            let outer = ProfileLoop::Path {
                start: Point2::new(0.0, 0.0),
                segments: vec![
                    ProfileSegment::LineTo(Point2::new(5.0, 0.0)),
                    ProfileSegment::ArcTo {
                        to: Point2::new(0.0, 5.0),
                        via: Point2::new(c, c),
                    },
                    ProfileSegment::LineTo(Point2::new(0.0, 0.0)),
                ],
            };
            revolve(
                m,
                &flat(xz, outer),
                Axis::z_at(Point3::origin()),
                std::f64::consts::TAU,
            )
        }
        SurfaceKind::Torus => {
            let outer = ProfileLoop::Circle {
                center: Point2::new(4.0, 2.5),
                radius: 2.0,
            };
            revolve(
                m,
                &flat(xz, outer),
                Axis::z_at(Point3::origin()),
                std::f64::consts::TAU,
            )
        }
        SurfaceKind::Nurbs => {
            let section = |z: f64, radius: f64| {
                let plane = Frame::new(pt(0.0, 0.0, z), up, Vec3::x()).unwrap();
                let outer = ProfileLoop::Circle {
                    center: Point2::origin(),
                    radius,
                };
                flat(plane, outer)
            };
            loft(
                m,
                &[section(0.0, 3.5), section(2.5, 4.5), section(5.0, 3.5)],
                LoftKind::Smooth,
                false,
                &Control::NONE,
            )
        }
    }
    .unwrap()
    .0
}

/// A plane-faced box through the +x side of every column's body.
fn box_tool(m: &mut Model) -> Body {
    primitive_box(m, pt(0.0, -2.0, 1.5), pt(8.0, 2.0, 3.5))
        .unwrap()
        .0
}

/// A cylinder through the +x side of the box.
fn cylinder_tool(m: &mut Model) -> Body {
    let axis = Axis::new(pt(0.0, 0.0, 2.5), Vec3::x()).unwrap();
    primitive_cylinder(m, axis, 1.5, 8.0).unwrap().0
}

/// A quarter turn of radius 4 about the y axis, swept with a disc: a bent
/// pipe, the torus kind.
fn pipe_tool(m: &mut Model) -> Body {
    let c = std::f64::consts::FRAC_1_SQRT_2;
    let path = Path {
        start: pt(4.0, 0.0, 0.0),
        segments: vec![PathSegment::ArcTo {
            to: pt(0.0, 0.0, 4.0),
            via: pt(4.0 * c, 0.0, 4.0 * c),
        }],
    };
    let plane = Frame::new(pt(4.0, 0.0, 0.0), Vec3::z(), Vec3::x()).unwrap();
    let disc = flat(
        plane,
        ProfileLoop::Circle {
            center: Point2::origin(),
            radius: 0.8,
        },
    );
    sweep(m, &disc, &path, FrameLaw::default(), &Control::NONE)
        .unwrap()
        .0
}

/// Three discs along x, lofted smoothly: a blob through every column's body.
fn blob_tool(m: &mut Model) -> Body {
    let section = |x: f64, radius: f64| {
        let plane = Frame::new(pt(x, 0.3, 2.6), Vec3::x(), Vec3::y()).unwrap();
        let outer = ProfileLoop::Circle {
            center: Point2::origin(),
            radius,
        };
        flat(plane, outer)
    };
    loft(
        m,
        &[section(-8.0, 1.0), section(0.0, 1.5), section(8.0, 1.0)],
        LoftKind::Smooth,
        false,
        &Control::NONE,
    )
    .unwrap()
    .0
}

fn kind_of(m: &Model, face: Face) -> SurfaceKind {
    m.surface(m.face(face.id).unwrap().surface())
        .unwrap()
        .kind()
}

/// The faces on the two sides of `edge`, distinct ones only.
fn faces_at(m: &Model, edge: Edge) -> BTreeSet<arris::topo::FaceId> {
    m.edge_uses(edge.id)
        .unwrap()
        .iter()
        .map(|use_| use_.face)
        .collect()
}

/// The first edge with two distinct faces, one of them of `kind`.
fn edge_of(m: &Model, body: Body, kind: SurfaceKind) -> Option<Edge> {
    m.edges(body).unwrap().into_iter().find(|&e| {
        let faces = faces_at(m, e);
        faces.len() == 2
            && faces
                .iter()
                .any(|&f| m.surface(m.face(f).unwrap().surface()).unwrap().kind() == kind)
    })
}

/// The column's first face of `kind`.
fn face_of(m: &Model, body: Body, kind: SurfaceKind) -> Option<Face> {
    m.faces(body)
        .unwrap()
        .into_iter()
        .find(|&f| kind_of(m, f) == kind)
}

/// What a cell shows.
#[derive(Debug, Clone, PartialEq)]
enum Cell {
    Ok,
    /// The body has nothing for the row to act on.
    NothingToDo,
    Refused(String),
    Internal(String),
    CheckerRed(String),
    /// A result the checker passes except for a pair of faces it cannot
    /// decide: the row's code.
    Unchecked(String),
    Panic(String),
    /// The operation returned, and its result is not the input's: a STEP
    /// round trip that moved the volume.
    Differs(String),
}

impl Cell {
    fn text(&self) -> String {
        match self {
            Cell::Ok => "ok".into(),
            Cell::NothingToDo => "-".into(),
            Cell::Refused(kind) => format!("refused {kind}"),
            Cell::Internal(kind) => format!("INTERNAL {kind}"),
            Cell::CheckerRed(code) => format!("checker-red {code}"),
            Cell::Unchecked(code) => format!("unchecked {code}"),
            Cell::Panic(line) => format!("PANIC {line}"),
            Cell::Differs(what) => format!("differs {what}"),
        }
    }
}

/// The leading identifier of a `Debug` text, parentheses included, so
/// `Boolean(TangentContact)` stays and `Fault { .. }` is cut at its space.
fn head(debug: &str) -> String {
    debug
        .chars()
        .take_while(|c| c.is_alphanumeric() || matches!(c, '(' | ')' | '_'))
        .collect()
}

/// A refusal's kind, or a fault's.
fn refusal(error: &OpError) -> Cell {
    match error {
        OpError::Internal(fault) => Cell::Internal(head(&format!("{fault:?}"))),
        OpError::Unsupported { a, b } => Cell::Refused(format!("Unsupported({}, {})", a.0, b.0)),
        OpError::Degenerate { reason, .. } => {
            Cell::Refused(format!("Degenerate({})", head(&format!("{reason:?}"))))
        }
        other => Cell::Refused(head(&format!("{other:?}"))),
    }
}

/// A result body: `Ok` when the checker at `Full` is green with nothing
/// unchecked.
fn judged(m: &Model, result: Result<(Body, Provenance), OpError>) -> Cell {
    match result {
        Err(error) => refusal(&error),
        Ok((body, _)) => verdict(m, body),
    }
}

fn verdict(m: &Model, body: Body) -> Cell {
    let report = check(m, body, Level::Full);
    if let Some(violation) = report.violations().first() {
        Cell::CheckerRed(first_word(&violation.to_string()))
    } else if let Some(unchecked) = report.unchecked().first() {
        Cell::Unchecked(first_word(&unchecked.to_string()))
    } else {
        Cell::Ok
    }
}

/// The checker's code at the head of a row's text (`S5`, `B1`).
fn first_word(text: &str) -> String {
    text.chars().take_while(|c| c.is_alphanumeric()).collect()
}

/// One cell: `run` on a fresh model, a panic caught as a cell.
fn cell(run: impl FnOnce(&mut Model) -> Cell) -> Cell {
    let mut m = Model::default();
    match catch_unwind(AssertUnwindSafe(|| run(&mut m))) {
        Ok(cell) => cell,
        Err(payload) => {
            let message = payload
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| payload.downcast_ref::<&str>().map(|s| s.to_string()))
                .unwrap_or_default();
            let line = message.lines().next().unwrap_or("");
            Cell::Panic(line.chars().take(60).collect())
        }
    }
}

fn the_cell(row: &str, column: SurfaceKind) -> Cell {
    cell(|m| {
        let body = body_of(m, column);
        match row {
            "cut" | "fuse" | "common" => {
                let tool = if column == SurfaceKind::Plane {
                    cylinder_tool(m)
                } else {
                    box_tool(m)
                };
                let boolean = match row {
                    "cut" => cut,
                    "fuse" => fuse,
                    _ => common,
                };
                {
                    let result = boolean(m, body, tool);
                    judged(m, result)
                }
            }
            "split" => {
                let plane = Frame::new(pt(0.7, 0.0, 0.0), Vec3::x(), Vec3::y()).unwrap();
                match split(m, body, &plane) {
                    Err(error) => refusal(&error),
                    Ok(pieces) => {
                        match (verdict(m, pieces.positive), verdict(m, pieces.negative)) {
                            (Cell::Ok, Cell::Ok) => Cell::Ok,
                            (Cell::Ok, red) | (red, _) => red,
                        }
                    }
                }
            }
            "section" => {
                let plane = Frame::new(pt(0.7, 0.0, 0.0), Vec3::x(), Vec3::y()).unwrap();
                match section(m, body, &plane) {
                    Err(error) => refusal(&error),
                    Ok(None) => Cell::Differs("no section".into()),
                    Ok(Some(section)) if section.area > 0.0 => Cell::Ok,
                    Ok(Some(_)) => Cell::Differs("empty section".into()),
                }
            }
            "fillet" | "chamfer" => match edge_of(m, body, column) {
                None => Cell::NothingToDo,
                Some(edge) if row == "fillet" => {
                    let result = fillet(m, body, &[edge], SMALL);
                    judged(m, result)
                }
                Some(edge) => {
                    let result = chamfer(m, body, &[edge], SMALL);
                    judged(m, result)
                }
            },
            "offset" => {
                let face = face_of(m, body, column).unwrap();
                {
                    let result = offset_faces(m, body, &[face], SMALL);
                    judged(m, result)
                }
            }
            "shell" => {
                let opening = m
                    .faces(body)
                    .unwrap()
                    .into_iter()
                    .find(|&f| kind_of(m, f) != column)
                    .or_else(|| face_of(m, body, column))
                    .unwrap();
                {
                    let result = shell(m, body, &[opening], SMALL, ShellSide::Inward);
                    judged(m, result)
                }
            }
            "sweep operand" => {
                let pipe = pipe_tool(m);
                {
                    let result = cut(m, body, pipe);
                    judged(m, result)
                }
            }
            "loft operand" => {
                let blob = blob_tool(m);
                {
                    let result = cut(m, body, blob);
                    judged(m, result)
                }
            }
            "ray" => {
                let ray = Ray {
                    origin: pt(-20.0, 0.2, 2.0),
                    direction: Vec3::x(),
                };
                match ray_fire(m, body, &ray, RayHits::All) {
                    Err(error) => refusal(&error),
                    Ok(hits) if hits.len() >= 2 => Cell::Ok,
                    Ok(hits) => Cell::Differs(format!("{} hits", hits.len())),
                }
            }
            "distance" => {
                let far = Operand::Point(pt(20.0, 0.3, 2.2));
                let shape = Operand::Shape(Shape::new(body.id, body.orientation));
                match distance(m, shape, far, &Control::NONE) {
                    Err(error) => refusal(&error),
                    Ok(d) if d.distance.is_finite() && d.distance > 0.0 => Cell::Ok,
                    Ok(_) => Cell::Differs("no distance".into()),
                }
            }
            "STEP round trip" => {
                let text = step::write(m, &[body]).unwrap();
                let mut back = Model::default();
                let read = match step::read(&mut back, &text, &Default::default(), &Control::NONE) {
                    Ok(read) => read,
                    Err(error) => return Cell::Refused(head(&format!("{error:?}"))),
                };
                match read.solids.first().map(|s| &s.result) {
                    Some(Ok(solid)) => {
                        let (before, after) = (
                            mass_properties(m, body).unwrap().volume,
                            mass_properties(&back, solid.body).unwrap().volume,
                        );
                        match verdict(&back, solid.body) {
                            Cell::Ok if (after - before).abs() <= 1e-6 * before.abs() => Cell::Ok,
                            Cell::Ok => Cell::Differs("volume".into()),
                            red => red,
                        }
                    }
                    Some(Err(refusal)) => Cell::Refused(head(&format!("{refusal:?}"))),
                    None => Cell::Differs("no solid".into()),
                }
            }
            other => unreachable!("no row {other}"),
        }
    })
}

/// The record's rows: the operation, then one cell per column.
fn record() -> Vec<Vec<String>> {
    let text = std::fs::read_to_string(record_path()).unwrap_or_default();
    text.lines()
        .skip(2)
        .map(|line| {
            let line = line.trim().trim_start_matches('|').trim_end_matches('|');
            line.split('|')
                .map(|cell| cell.trim().to_string())
                .collect()
        })
        .collect()
}

fn render(rows: &[Vec<String>]) -> String {
    let mut out = String::from("| operation |");
    for (name, _) in COLUMNS {
        out += &format!(" {name} |");
    }
    out += "\n|---|";
    out += &"---|".repeat(COLUMNS.len());
    out += "\n";
    for row in rows {
        out += "|";
        for cell in row {
            out += &format!(" {cell} |");
        }
        out += "\n";
    }
    out
}

fn record_path() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/capability.txt")
}

/// One column's cells of the table equal the record's, the `heavy` rows or
/// the others: a refusal that now builds is as much a change as one that
/// appears. With `ARRIS_BLESS=1` the cells are rewritten in the record
/// instead (run the shards one at a time, `--test-threads 1`, since each
/// edits the one file).
fn column_is_the_record(index: usize, heavy: bool) {
    std::panic::set_hook(Box::new(|_| {}));
    let (name, kind) = COLUMNS[index];
    let rows: Vec<usize> = (0..ROWS.len())
        .filter(|&i| HEAVY.contains(&ROWS[i]) == heavy)
        .collect();
    let cells: Vec<String> = rows
        .iter()
        .map(|&i| the_cell(ROWS[i], kind).text())
        .collect();
    let _ = std::panic::take_hook();
    if std::env::var_os(arris_debug::corpus::BLESS_VAR).is_some() {
        let mut held = record();
        if held.len() != ROWS.len() {
            held = ROWS
                .iter()
                .map(|row| {
                    std::iter::once(row.to_string())
                        .chain(COLUMNS.map(|_| String::new()))
                        .collect()
                })
                .collect();
        }
        for (&i, cell) in rows.iter().zip(cells) {
            held[i][1 + index] = cell;
        }
        std::fs::write(record_path(), render(&held)).expect("the record is written");
        return;
    }
    let held = record();
    let moved: Vec<String> = rows
        .iter()
        .zip(&cells)
        .filter_map(|(&i, cell)| {
            let was = held.get(i).and_then(|r| r.get(1 + index)).map_or("", |c| c);
            (was != cell).then(|| format!("{} x {name}: now `{cell}`, record `{was}`", ROWS[i]))
        })
        .collect();
    assert!(
        moved.is_empty(),
        "the capability table moved; if intended, bless with {}=1:\n{}",
        arris_debug::corpus::BLESS_VAR,
        moved.join("\n")
    );
}

macro_rules! column_tests {
    ($($test:ident, $heavy:ident: $index:expr,)*) => {$(
        #[test]
        fn $test() {
            column_is_the_record($index, false);
        }

        #[test]
        fn $heavy() {
            column_is_the_record($index, true);
        }
    )*};
}

column_tests! {
    capability_plane, capability_plane_heavy: 0,
    capability_cylinder, capability_cylinder_heavy: 1,
    capability_elliptic_cylinder, capability_elliptic_cylinder_heavy: 2,
    capability_cone, capability_cone_heavy: 3,
    capability_sphere, capability_sphere_heavy: 4,
    capability_torus, capability_torus_heavy: 5,
    capability_nurbs, capability_nurbs_heavy: 6,
}

/// The record is the table and nothing else: its header, its rows in order,
/// one cell per column.
#[test]
fn capability_record_has_the_table_shape() {
    let text = std::fs::read_to_string(record_path()).unwrap();
    let rows = record();
    let names: Vec<&str> = rows.iter().map(|r| r[0].as_str()).collect();
    assert_eq!(names, ROWS);
    assert!(
        rows.iter()
            .all(|r| r.len() == 1 + COLUMNS.len() && r.iter().all(|c| !c.is_empty()))
    );
    assert_eq!(text, render(&rows));
}
