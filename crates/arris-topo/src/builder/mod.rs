//! The builder and the Euler operators (`docs/DATA-MODEL.md` §Euler
//! operators, ADR-0002): the only way an operation makes topology.
//!
//! Entities in the arena are immutable and Euler operators mutate, so the
//! two meet in a [`Builder`]: a staging area holding one body under
//! construction, edited by Mäntylä's ten operators (*An Introduction to
//! Solid Modeling*, ch. 9, adapted to coedges and seams), and frozen into
//! the arena by [`Builder::finish`], which appends every entity in a
//! deterministic order inside a transaction.
//!
//! What the builder guarantees: every operator keeps the Euler–Poincaré
//! line at zero ([`Builder::counts`]); every operator has an inverse that
//! restores the builder byte for byte ([`Builder::dump`]) — a killed slot
//! is reused by the next make, loops are kept in a canonical rotation and
//! order, so the state after `op` then `op⁻¹` is the state before; and
//! `finish` refuses a body that is not closed as the kind asked for, a
//! loop without coedges, a coedge without a pcurve, or a geometry id that
//! does not resolve — each a typed [`BuildError`] — and then the model is
//! exactly as it was.
//!
//! What it does not do: it never computes a pcurve and never evaluates
//! geometry. Every pcurve is the caller's ([`Strut::pcurves`],
//! [`Split::pcurves`], [`Builder::set_pcurve`]), since only the caller
//! knows which use of a seam it is drawing; and the checker
//! (`arris-check`, above this crate) is where the finished body is
//! proven.
//!
//! Orientations inside the builder are *effective*: a [`Use`] is walked
//! as seen from outside the material with the face's outward normal up,
//! whatever the surface's own normal, and each [`StagedFace`] carries the
//! orientation the shell will use it with. `finish` stores a loop of a
//! `Reversed` face backwards with every use flipped, which is what
//! `docs/DATA-MODEL.md` §Orientation says a stored loop is.

mod assemble;
mod error;
mod euler;
mod finish;
mod refs;
mod spec;
mod staged;
mod validate;

pub use error::{BuildError, ClosureError, EulerError, SlotError};
pub use refs::{EdgeRef, FaceRef, Position, Use, VertexRef};
pub use spec::{
    Assembly, AssemblySlots, BodyIndex, EdgeKey, EdgeSpec, FaceRemap, FaceSpec, GeometryRemap,
    KeepGeometry, UseSpec, VertexKey, VertexSpec, effective_uses,
};
pub use staged::{
    Built, Counts, Join, Seed, Split, StagedEdge, StagedFace, StagedLoop, StagedVertex, Strut,
};

use core::fmt;
use std::collections::BTreeSet;

use crate::entity::EdgeGeometry;
#[cfg(doc)]
use crate::euler::EulerLine;
#[cfg(doc)]
use crate::model::Model;
use crate::orientation::Orientation;

use self::staged::Slots;
use self::validate::is_degenerate;

/// ` kept f3` for a slot [`Builder::assemble`] took from the model and no
/// operator has touched since, the empty string for one that will be
/// appended: what tells two [`Builder::dump`]s of the same shape apart.
fn kept(id: Option<impl fmt::Display>) -> String {
    id.map_or_else(String::new, |id| format!(" kept {id}"))
}

/// One body under construction, edited by the Euler operators and frozen
/// into a [`Model`] by [`Builder::finish`]. Module docs for the
/// guarantees.
///
/// The cylinder of `docs/DATA-MODEL.md` §Seams, as the primitive
/// builds it: a seed vertex on the bottom cap, the bottom circle as a
/// closed edge splitting off the wall, the seam as a strut whose two
/// pcurves differ by the period, the top circle splitting off the top
/// cap.
///
/// ```
/// use arris_geom::{Curve, Curve2, Surface};
/// use arris_math::{Frame, Frame2, Interval, Point2, Point3, Vec2};
/// use arris_topo::builder::{Builder, Position, Seed, Split, Strut};
/// use arris_topo::entity::{BodyKind, EdgeGeometry};
/// use arris_topo::{Model, Orientation};
/// use core::f64::consts::TAU;
///
/// let mut m = Model::default();
/// let (r, h) = (4.0, 12.0);
/// let base = Frame::world();
/// let top = base.with_origin(Point3::new(0.0, 0.0, h));
/// let wall = m.add_surface(Surface::Cylinder { frame: base, radius: r });
/// let bottom = m.add_surface(Surface::Plane { frame: base });
/// let top_plane = m.add_surface(Surface::Plane { frame: top });
/// let uv_line = |m: &mut Model, u, v, along_u: bool| m.add_curve2(Curve2::Line {
///     origin: Point2::new(u, v),
///     direction: if along_u { Vec2::x_axis() } else { Vec2::y_axis() },
/// });
/// let cap_circle = |m: &mut Model| m.add_curve2(Curve2::Circle { frame: Frame2::identity(), radius: r });
///
/// let mut b = Builder::new(m.precision().default_tolerance);
/// let (v0, f_bottom) = b.mvfs(Seed { point: Point3::new(r, 0.0, 0.0), surface: bottom, orientation: Orientation::Reversed })?;
/// let at = Position::new(f_bottom, 0, 0);
/// let circle = m.add_curve(Curve::Circle { frame: base, radius: r });
/// let (p_wall_bottom, p_cap_bottom) = (uv_line(&mut m, 0.0, 0.0, true), cap_circle(&mut m));
/// let (_e_bottom, f_wall) = b.mef(at, at, Split {
///     geometry: EdgeGeometry::Curve { curve: circle, range: Interval::TURN },
///     surface: wall,
///     orientation: Orientation::Forward,
///     pcurves: [Some(p_wall_bottom), Some(p_cap_bottom)],
/// })?;
/// let seam = m.add_curve(Curve::Line { origin: Point3::new(r, 0.0, 0.0), direction: arris_math::Vec3::z_axis() });
/// let (up, down) = (uv_line(&mut m, TAU, 0.0, false), uv_line(&mut m, 0.0, 0.0, false));
/// let (v1, _e_seam) = b.mev(b.find_position(f_wall, 0, v0)?, Strut {
///     point: Point3::new(r, 0.0, h),
///     geometry: EdgeGeometry::Curve { curve: seam, range: Interval::new(0.0, h).unwrap() },
///     pcurves: [Some(up), Some(down)],
/// })?;
/// let top_circle = m.add_curve(Curve::Circle { frame: top, radius: r });
/// let (p_wall_top, p_cap_top) = (uv_line(&mut m, 0.0, h, true), cap_circle(&mut m));
/// let at = b.find_position(f_wall, 0, v1)?;
/// let (_e_top, _f_top) = b.mef(at, at, Split {
///     geometry: EdgeGeometry::Curve { curve: top_circle, range: Interval::TURN },
///     surface: top_plane,
///     orientation: Orientation::Forward,
///     pcurves: [Some(p_cap_top), Some(p_wall_top)],
/// })?;
/// assert_eq!(b.counts().to_string(), "2/3/3/3/1 g0 = 0");
/// let built = b.finish(&mut m, BodyKind::Solid)?;
/// assert_eq!(m.faces(built.body)?.len(), 3);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
#[derive(Debug, Clone)]
pub struct Builder {
    tolerance: f64,
    vertices: Slots<StagedVertex>,
    edges: Slots<StagedEdge>,
    faces: Slots<StagedFace>,
    genus: usize,
}

impl Builder {
    /// An empty builder whose every entity will carry `tolerance` — a
    /// primitive passes the model's `default_tolerance`.
    pub fn new(tolerance: f64) -> Self {
        Builder {
            tolerance,
            vertices: Slots::default(),
            edges: Slots::default(),
            faces: Slots::default(),
            genus: 0,
        }
    }

    /// The tolerance every entity is made with.
    pub const fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// `true` before `mvfs` and after `kvfs`.
    pub fn is_empty(&self) -> bool {
        self.faces.len() == 0 && self.edges.len() == 0 && self.vertices.len() == 0
    }

    /// The counts and the Euler line they make. No degenerate edge is
    /// counted — a singular point of its face, not a boundary between two
    /// — by the one rule every Euler line keeps ([`EulerLine`]).
    pub fn counts(&self) -> Counts {
        let loops = self.faces.iter().map(|(_, f)| f.loops.len()).sum();
        let degenerate = self.edges.iter().filter(|&(_, e)| is_degenerate(e)).count();
        let shells = self
            .faces
            .iter()
            .map(|(_, f)| f.shell)
            .collect::<BTreeSet<_>>()
            .len();
        Counts {
            vertices: self.vertices.len(),
            edges: self.edges.len() - degenerate,
            faces: self.faces.len(),
            loops,
            shells,
            genus: self.genus,
        }
    }

    /// The vertex under `v`. Errors: the slot is not live.
    pub fn vertex(&self, v: VertexRef) -> Result<&StagedVertex, BuildError> {
        self.vertices
            .get(v.0)
            .ok_or(BuildError::Slot(SlotError::NoVertex(v)))
    }

    /// The edge under `e`. Errors: the slot is not live.
    pub fn edge(&self, e: EdgeRef) -> Result<&StagedEdge, BuildError> {
        self.edges
            .get(e.0)
            .ok_or(BuildError::Slot(SlotError::NoEdge(e)))
    }

    /// The face under `f`. Errors: the slot is not live.
    pub fn face(&self, f: FaceRef) -> Result<&StagedFace, BuildError> {
        self.faces
            .get(f.0)
            .ok_or(BuildError::Slot(SlotError::NoFace(f)))
    }

    /// Every live vertex, in slot order.
    pub fn vertices(&self) -> impl Iterator<Item = (VertexRef, &StagedVertex)> {
        self.vertices.iter().map(|(i, v)| (VertexRef(i), v))
    }

    /// Every live edge, in slot order.
    pub fn edges(&self) -> impl Iterator<Item = (EdgeRef, &StagedEdge)> {
        self.edges.iter().map(|(i, e)| (EdgeRef(i), e))
    }

    /// Every live face, in slot order.
    pub fn faces(&self) -> impl Iterator<Item = (FaceRef, &StagedFace)> {
        self.faces.iter().map(|(i, f)| (FaceRef(i), f))
    }

    fn face_mut(&mut self, f: FaceRef) -> Result<&mut StagedFace, BuildError> {
        self.faces
            .get_mut(f.0)
            .ok_or(BuildError::Slot(SlotError::NoFace(f)))
    }

    fn loop_ref(&self, f: FaceRef, loop_index: usize) -> Result<&StagedLoop, BuildError> {
        self.face(f)?
            .loops
            .get(loop_index)
            .ok_or(BuildError::Slot(SlotError::NoLoop {
                face: f,
                loop_index,
            }))
    }

    fn loop_mut(&mut self, f: FaceRef, loop_index: usize) -> Result<&mut StagedLoop, BuildError> {
        self.face_mut(f)?
            .loops
            .get_mut(loop_index)
            .ok_or(BuildError::Slot(SlotError::NoLoop {
                face: f,
                loop_index,
            }))
    }

    /// The effective start vertex of `u`.
    fn use_start(&self, u: &Use) -> Result<VertexRef, BuildError> {
        let e = self.edge(u.edge)?;
        Ok(match u.orientation {
            Orientation::Forward => e.start,
            Orientation::Reversed => e.end,
        })
    }

    /// The builder's state as text: every live slot in order with its
    /// geometry ids — a face's shell index too once there are several —
    /// every loop's uses, the genus and the counts line. Identical for two
    /// builders that went through the same calls, and restored byte for
    /// byte by an operator's inverse.
    pub fn dump(&self) -> String {
        use core::fmt::Write as _;
        let several_shells = self.counts().shells > 1;
        let mut out = String::new();
        let _ = writeln!(
            out,
            "builder tolerance {} genus {}",
            self.tolerance, self.genus
        );
        let _ = writeln!(out, "vertices");
        for (i, v) in self.vertices.iter() {
            let _ = writeln!(
                out,
                "  v{i} ({}, {}, {}) tol {}{}",
                v.point.x,
                v.point.y,
                v.point.z,
                v.tolerance,
                kept(v.kept)
            );
        }
        let _ = writeln!(out, "edges");
        for (i, e) in self.edges.iter() {
            let _ = write!(out, "  e{i} {} -> {} ", e.start, e.end);
            match e.geometry {
                EdgeGeometry::Curve { curve, range } => {
                    let _ = write!(out, "{curve} [{}, {}]", range.lo(), range.hi());
                }
                EdgeGeometry::Degenerate { range } => {
                    let _ = write!(out, "degenerate [{}, {}]", range.lo(), range.hi());
                }
            }
            let _ = writeln!(out, " tol {}{}", e.tolerance, kept(e.kept));
        }
        let _ = writeln!(out, "faces");
        for (i, f) in self.faces.iter() {
            let shell = if several_shells {
                format!(" shell {}", f.shell)
            } else {
                String::new()
            };
            let _ = writeln!(
                out,
                "  f{i} {} {}{shell} tol {}{}",
                f.surface,
                f.orientation,
                f.tolerance,
                kept(f.kept)
            );
            for (li, lp) in f.loops.iter().enumerate() {
                if lp.uses.is_empty() {
                    let _ = writeln!(out, "    loop {li} at {}", lp.seed);
                } else {
                    let _ = writeln!(out, "    loop {li}");
                }
                for u in &lp.uses {
                    let _ = writeln!(out, "      {u}");
                }
            }
        }
        let _ = writeln!(out, "counts {}", self.counts());
        out
    }
}
