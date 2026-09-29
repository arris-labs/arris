//! One solid's topology: a `MANIFOLD_SOLID_BREP` or a `BREP_WITH_VOIDS`
//! read into a body through [`Builder::assemble`] (ADR-0025).
//!
//! - **Vertices** come from `VERTEX_POINT`, one per instance.
//! - **Edges** come from `EDGE_CURVE`. An Arris edge runs along its
//!   curve, so one whose `same_sense` — composed with a `TRIMMED_CURVE`'s
//!   sense — runs against it swaps its two vertices, and every use of it
//!   is turned. The range is the parameters of the two vertices projected
//!   onto the curve, the upper one moved on by the period where it falls
//!   below; a closed edge on a periodic curve takes the full period from
//!   its vertex, and one on a closed curve that is not periodic the
//!   curve's whole domain.
//! - **Faces** take their orientation from the surface variant's normal
//!   against the file's ([`ReadSurface::reversed`]), the face's
//!   `same_sense`, an `ORIENTED_FACE`'s flag and the shell's. A bound's
//!   edges are walked in the `EDGE_LOOP`'s order with each
//!   `ORIENTED_EDGE`'s flag, the whole bound turned where its own flag is
//!   false; that walk keeps the face on its left about the file face's
//!   normal, and it is turned again where the shell (an
//!   `ORIENTED_CLOSED_SHELL` of orientation false, a void's) or an
//!   `ORIENTED_FACE` turns that normal, which gives the builder's
//!   effective walk.
//! - **Pcurves** are rebuilt, never read: [`pcurve_on`] for every edge on
//!   every face it bounds, each use's copy moved by whole periods so that
//!   it starts where the use before it in the loop ends. An edge used
//!   twice in one loop is a seam, and the walk around the loop puts its
//!   two copies a period apart. The widest loop is then moved by whole
//!   periods so that its box's centre lies in the surface's own period —
//!   the translate a projection answers in, within a period of which a
//!   face's domain looks for a point — and every other so that its box's
//!   centre lies in the period that starts at the widest loop's lower
//!   bound, so a hole lies inside its outer loop.
//! - **Degenerate edges** the writer left out are rebuilt where two uses
//!   meet at a singular point of the surface — a sphere's pole, a cone's
//!   apex, a NURBS surface's collapsed row ([`Surface::singularities`]):
//!   a degenerate coedge along the row between where the one ends and
//!   the other starts, run in the sense that keeps the face on its left,
//!   a whole turn where the two meet the row at one value and the walk
//!   turns back on itself (a seam's two uses), none where it turns in
//!   towards the face. A `VERTEX_LOOP` at a singular point is a loop of
//!   one degenerate coedge, a whole turn along its row. A jump in (u, v)
//!   anywhere else is [`Refusal::OpenLoop`].
//! - **Singular points an edge runs through** split it there, before any
//!   face is walked, a vertex on the point: no pcurve runs through a
//!   pole or an apex, and each side of it has one. Every use of the edge
//!   walks its pieces.
//! - **Seams the writer left out** are rebuilt where a face's loops wrap
//!   a period of its surface, which ISO 10303-42 allows and Arris's face,
//!   one region in (u, v), does not. Two loops wrapping once each — a
//!   cylinder's side bounded by its two circles — are joined by a seam
//!   along the surface's isocurve through a vertex of each, the other
//!   loop's edge split where no vertex faces the first, on every face
//!   that uses it ([`band`]). One loop wrapping once — a cone bounded by
//!   its base circle alone — gets a vertex at the singular point on the
//!   side the face lies on, joined as a `VERTEX_LOOP` is ([`apex`]).
//!
//! - **Gaps** are measured, never assumed (ADR-0025 §4). An edge curve
//!   off a face's surface has its pcurve fitted at the gap, and two
//!   pcurves ending apart in (u, v) at a vertex — two curves ending apart
//!   in 3D — are ended on one point, the seam's end where one of them is
//!   a seam, else the vertex's own (u, v), as a boolean ends a section edge on its
//!   vertex. Each edge then takes the largest distance of its pcurves'
//!   images from its curve — at the checker's samples and at the
//!   [`PCURVE_SAMPLES`] a fit is held to, each peak climbed to its top
//!   between them — a closed edge also
//!   its curve's own gap; each vertex the largest distance from its point
//!   to each edge curve's end and each pcurve's image there, and the span
//!   of a degenerate edge's image; each face the model's default. Every
//!   value is floored at the default and raised to keep vertex ≥ edge ≥
//!   face. A gap past the cap — [`READ_GAP_FRACTION`] of the part's size,
//!   at most the model's `max_tolerance` — is [`Refusal::Gap`].
//!
//! The body is checked at `Level::Fast` in every build (ADR-0025 §5), and
//! a violation is the file's: [`Refusal::Invalid`].

use std::collections::BTreeMap;
use std::f64::consts::TAU;

use arris_check::arris_topo::arris_geom::{
    Curve, Curve2, GeomError, NurbsCurve2, PCURVE_SAMPLES, PCURVE_SINGULAR_BAND, Singularity,
    Surface, pcurve_ending_on, pcurve_on,
};
use arris_check::arris_topo::arris_math::{
    Aabb, Frame, Interval, Point2, Point3, Precision, READ_GAP_FRACTION, RELATIVE_ROUNDING,
    Tolerance, UnitVec2, UnitVec3, Vec2, Vec3,
};
use arris_check::arris_topo::builder::{
    Assembly, Builder, EdgeKey, EdgeSpec, FaceSpec, UseSpec, VertexKey, VertexSpec,
};
use arris_check::arris_topo::entity::{BodyKind, EdgeGeometry};
use arris_check::arris_topo::provenance::{FileEntity, Role};
use arris_check::arris_topo::{Body, CurveId, Model, Orientation, Provenance, Shape, SurfaceId};
use arris_check::domain::bands;
use arris_check::{Level, check};

use super::Refusal;
use super::entities::{Args, describe, malformed};
use super::geometry::{Ball, Geometry, ReadCurve, ReadSurface};

/// A solid read: its body and the record naming the file entity each of
/// its entities was generated from.
pub(crate) struct Solid {
    pub(crate) body: Body,
    pub(crate) provenance: Provenance,
}

/// An `EDGE_CURVE` as the file gives it.
struct FileEdge {
    id: u64,
    /// The start and end vertices, as indices into the vertex list.
    start: usize,
    end: usize,
    curve: ReadCurve,
    /// The edge runs from its start to its end along the file curve's
    /// parametrisation.
    same_sense: bool,
}

/// One use of an edge by a loop: the edge's index and whether the loop
/// walks it from its file start to its file end.
#[derive(Clone, Copy)]
struct Step {
    edge: usize,
    forward: bool,
}

/// A face as the file gives it.
struct FileFace {
    id: u64,
    surface: ReadSurface,
    /// The file face's normal is its surface's.
    same_sense: bool,
    /// The face is turned by an `ORIENTED_FACE` or its shell, so its
    /// bounds' walk is turned with it.
    turned: bool,
    bounds: Vec<Bound>,
}

/// A face bound as the file gives it.
struct Bound {
    id: u64,
    walk: Walk,
}

/// What a bound walks.
enum Walk {
    /// An `EDGE_LOOP`'s edges, the face on its left about the file face's
    /// normal.
    Edges(Vec<Step>),
    /// A `VERTEX_LOOP`'s vertex, as an index into the vertex list: a
    /// singular row of the surface the bound runs along.
    Vertex(usize),
}

/// A closed shell as the file gives it.
struct FileShell {
    id: u64,
    faces: Vec<FileFace>,
}

/// What the file says about one solid, before any geometry is resolved.
struct FileSolid {
    /// Every `VERTEX_POINT`, in first-use order.
    vertices: Vec<u64>,
    edges: Vec<FileEdge>,
    shells: Vec<FileShell>,
}

/// How deep an `ORIENTED_FACE` may nest before the reader stops: a file
/// nests none or one, and a cycle of references must end.
const ORIENTED_DEPTH: u8 = 4;

impl Geometry<'_> {
    /// Reads the solid `solid`, placement `instance`, into `model`.
    /// Errors: a refusal naming the file entity; the model is then as it
    /// was.
    pub(crate) fn solid(
        &self,
        model: &mut Model,
        solid: u64,
        instance: u32,
    ) -> Result<Solid, Refusal> {
        let file = self.file_solid(solid)?;
        model.transaction(|m| self.build(m, solid, instance, &file))
    }

    /// The shells, faces, bounds, edges and vertices of `solid`, with every
    /// curve and surface read but not yet bounded.
    fn file_solid(&self, solid: u64) -> Result<FileSolid, Refusal> {
        let instance = self.entities.get(solid, solid)?;
        let mut shells = Vec::new();
        if let Some(r) = instance.record("BREP_WITH_VOIDS") {
            let args = Args {
                id: solid,
                record: r,
            };
            shells.push(args.reference(1)?);
            shells.extend(args.references(2)?);
        } else if let Some(r) = instance.record("MANIFOLD_SOLID_BREP") {
            shells.push(
                Args {
                    id: solid,
                    record: r,
                }
                .reference(1)?,
            );
        } else {
            return Err(Refusal::Unsupported {
                entity: solid,
                name: describe(instance),
            });
        }
        let mut read = FileSolid {
            vertices: Vec::new(),
            edges: Vec::new(),
            shells: Vec::new(),
        };
        let mut vertex_of: BTreeMap<u64, usize> = BTreeMap::new();
        let mut edge_of: BTreeMap<u64, usize> = BTreeMap::new();
        for shell in shells {
            let (closed, turned) = self.closed_shell(solid, shell)?;
            let args = self.entities.record(shell, closed, "CLOSED_SHELL")?;
            let mut faces = Vec::new();
            for face in args.references(1)? {
                faces.push(self.face(
                    closed,
                    face,
                    turned,
                    ORIENTED_DEPTH,
                    &mut read,
                    &mut vertex_of,
                    &mut edge_of,
                )?);
            }
            read.shells.push(FileShell { id: closed, faces });
        }
        Ok(read)
    }

    /// The `CLOSED_SHELL` a solid's shell reference names, and whether an
    /// `ORIENTED_CLOSED_SHELL` turns it.
    fn closed_shell(&self, from: u64, id: u64) -> Result<(u64, bool), Refusal> {
        let instance = self.entities.get(from, id)?;
        if instance.record("CLOSED_SHELL").is_some() {
            return Ok((id, false));
        }
        if let Some(r) = instance.record("ORIENTED_CLOSED_SHELL") {
            let args = Args { id, record: r };
            let element = args.reference(2)?;
            let orientation = logical(&args, 3)?;
            self.entities.record(id, element, "CLOSED_SHELL")?;
            return Ok((element, !orientation));
        }
        Err(Refusal::Unsupported {
            entity: id,
            name: describe(instance),
        })
    }

    /// A face of a closed shell: an `ADVANCED_FACE` (or its supertype
    /// `FACE_SURFACE`), or an `ORIENTED_FACE` of one.
    #[allow(clippy::too_many_arguments)]
    fn face(
        &self,
        from: u64,
        id: u64,
        turned: bool,
        depth: u8,
        read: &mut FileSolid,
        vertex_of: &mut BTreeMap<u64, usize>,
        edge_of: &mut BTreeMap<u64, usize>,
    ) -> Result<FileFace, Refusal> {
        let instance = self.entities.get(from, id)?;
        if let Some(r) = instance.record("ORIENTED_FACE") {
            let args = Args { id, record: r };
            let element = args.reference(2)?;
            let orientation = logical(&args, 3)?;
            let Some(depth) = depth.checked_sub(1) else {
                return Err(malformed(id, "ORIENTED_FACE nested past any file's need"));
            };
            return self.face(
                id,
                element,
                turned ^ !orientation,
                depth,
                read,
                vertex_of,
                edge_of,
            );
        }
        let record = instance
            .record("ADVANCED_FACE")
            .or_else(|| instance.record("FACE_SURFACE"))
            .ok_or_else(|| Refusal::Unsupported {
                entity: id,
                name: describe(instance),
            })?;
        let args = Args { id, record };
        let surface = self.surface(id, args.reference(2)?)?;
        let same_sense = logical(&args, 3)?;
        let mut bounds = Vec::new();
        for bound in args.references(1)? {
            bounds.push(self.bound(id, bound, read, vertex_of, edge_of)?);
        }
        Ok(FileFace {
            id,
            surface,
            same_sense,
            turned,
            bounds,
        })
    }

    /// A `FACE_BOUND` or `FACE_OUTER_BOUND`: its `EDGE_LOOP`'s walk, turned
    /// where the bound's flag is false.
    fn bound(
        &self,
        face: u64,
        id: u64,
        read: &mut FileSolid,
        vertex_of: &mut BTreeMap<u64, usize>,
        edge_of: &mut BTreeMap<u64, usize>,
    ) -> Result<Bound, Refusal> {
        let instance = self.entities.get(face, id)?;
        let record = instance
            .record("FACE_OUTER_BOUND")
            .or_else(|| instance.record("FACE_BOUND"))
            .ok_or_else(|| {
                malformed(
                    id,
                    format!("is {}, where a face bound belongs", describe(instance)),
                )
            })?;
        let args = Args { id, record };
        let lp = args.reference(1)?;
        let orientation = logical(&args, 2)?;
        let lp_instance = self.entities.get(id, lp)?;
        if let Some(r) = lp_instance.record("VERTEX_LOOP") {
            let v = (Args { id: lp, record: r }).reference(1)?;
            let vertex = self.vertex(lp, v, read, vertex_of)?;
            return Ok(Bound {
                id,
                walk: Walk::Vertex(vertex),
            });
        }
        let Some(edges) = lp_instance.record("EDGE_LOOP") else {
            // A POLY_LOOP is a faceted face's.
            return Err(Refusal::Unsupported {
                entity: lp,
                name: describe(lp_instance),
            });
        };
        let mut walk = Vec::new();
        for oe in (Args {
            id: lp,
            record: edges,
        })
        .references(1)?
        {
            let args = self.entities.record(lp, oe, "ORIENTED_EDGE")?;
            let element = args.reference(3)?;
            let forward = logical(&args, 4)?;
            let edge = match edge_of.get(&element) {
                Some(&i) => i,
                None => {
                    let e = self.edge(oe, element, read, vertex_of)?;
                    read.edges.push(e);
                    edge_of.insert(element, read.edges.len() - 1);
                    read.edges.len() - 1
                }
            };
            walk.push(Step { edge, forward });
        }
        if walk.is_empty() {
            return Err(malformed(lp, "an EDGE_LOOP of no edges"));
        }
        if !orientation {
            turn(&mut walk);
        }
        Ok(Bound {
            id,
            walk: Walk::Edges(walk),
        })
    }

    /// A `VERTEX_POINT`'s index in the solid's list, entered at its first
    /// use.
    fn vertex(
        &self,
        from: u64,
        id: u64,
        read: &mut FileSolid,
        vertex_of: &mut BTreeMap<u64, usize>,
    ) -> Result<usize, Refusal> {
        self.entities.record(from, id, "VERTEX_POINT")?;
        Ok(*vertex_of.entry(id).or_insert_with(|| {
            read.vertices.push(id);
            read.vertices.len() - 1
        }))
    }

    /// An `EDGE_CURVE`, its vertices entered in the solid's list.
    fn edge(
        &self,
        from: u64,
        id: u64,
        read: &mut FileSolid,
        vertex_of: &mut BTreeMap<u64, usize>,
    ) -> Result<FileEdge, Refusal> {
        let args = self.entities.record(from, id, "EDGE_CURVE")?;
        let start = self.vertex(id, args.reference(1)?, read, vertex_of)?;
        let end = self.vertex(id, args.reference(2)?, read, vertex_of)?;
        let curve = self.curve(id, args.reference(3)?)?;
        let same_sense = logical(&args, 4)?;
        Ok(FileEdge {
            id,
            start,
            end,
            curve,
            same_sense,
        })
    }

    /// The body of `file`, appended to `model`.
    fn build(
        &self,
        model: &mut Model,
        solid: u64,
        instance: u32,
        file: &FileSolid,
    ) -> Result<Solid, Refusal> {
        let precision = model.precision();
        let tol = precision.tolerance();
        let tolerance = precision.default_tolerance;
        let file_entity = |id: u64| Role::File(FileEntity { id, instance });

        let mut points = file
            .vertices
            .iter()
            .map(|&v| {
                let args = self.entities.record(v, v, "VERTEX_POINT")?;
                self.units.point(&self.entities, v, args.reference(1)?)
            })
            .collect::<Result<Vec<Point3>, Refusal>>()?;

        // An unbounded curve is bounded by the vertices alone; every
        // surface by those and every edge.
        let mut part = ball_of(points.iter().map(|&p| Aabb::of_point(p)), tolerance);
        let mut curves: BTreeMap<u64, (Curve, CurveId)> = BTreeMap::new();
        let mut edges = Vec::with_capacity(file.edges.len());
        for e in &file.edges {
            let (curve, curve_id) = match curves.get(&e.curve.id) {
                Some(c) => c.clone(),
                None => {
                    let c = e.curve.resolve(&part)?;
                    let id = model.add_curve(c.clone());
                    curves.insert(e.curve.id, (c.clone(), id));
                    (c, id)
                }
            };
            // The Arris edge runs along its curve.
            let along = e.same_sense != e.curve.reversed;
            let (start, end) = if along {
                (e.start, e.end)
            } else {
                (e.end, e.start)
            };
            let range = edge_range(e.id, &curve, points[start], points[end], start == end)?;
            edges.push(Edge {
                geometry: curve,
                curve: curve_id,
                range,
                start,
                end,
                along,
            });
        }
        let boxes = edges.iter().filter_map(|e| e.geometry.bounds(e.range));
        part = ball_of(
            points.iter().map(|&p| Aabb::of_point(p)).chain(boxes),
            tolerance,
        );
        let cap = (READ_GAP_FRACTION * 2.0 * part.radius)
            .min(precision.max_tolerance)
            .max(tolerance);

        let mut surfaces: BTreeMap<u64, (Surface, SurfaceId)> = BTreeMap::new();
        // The edges the file left out, and the face each is rebuilt on.
        let mut rebuilt: Vec<Rebuilt> = Vec::new();
        let mut rebuilt_faces: Vec<u64> = Vec::new();
        // The file entity each vertex and edge stands for: a vertex or an
        // edge a band's seam splits off stands for the edge it splits.
        let mut vertex_ids: Vec<u64> = file.vertices.clone();
        let mut edge_ids: Vec<u64> = file.edges.iter().map(|e| e.id).collect();
        let mut splits: Vec<Split> = Vec::new();

        // An edge whose curve runs through a singular point of a face it
        // bounds — a meridian circle through a sphere's pole — is split
        // there before any face is walked, a vertex on the point: no
        // pcurve runs through it, and each side of it has one (ADR-0021).
        // Each file edge walks as its pieces, in its curve's order.
        let mut pieces: Vec<Vec<usize>> = (0..edges.len()).map(|i| vec![i]).collect();
        let on_point = PCURVE_SINGULAR_BAND * tol.linear;
        for face in file.shells.iter().flat_map(|shell| &shell.faces) {
            let surface = match surfaces.get(&face.surface.id) {
                Some(s) => s.0.clone(),
                None => {
                    let s = face.surface.resolve(&part)?;
                    let id = model.add_surface(s.clone());
                    surfaces.insert(face.surface.id, (s.clone(), id));
                    s
                }
            };
            for row in surface.singularities() {
                for bound in &face.bounds {
                    let Walk::Edges(ref walk) = bound.walk else {
                        continue;
                    };
                    for step in walk {
                        while let Some((at, p, t)) = (pieces[step.edge].iter().enumerate())
                            .find_map(|(at, &p)| {
                                Some((at, p, through(&edges[p], row.point, on_point)?))
                            })
                        {
                            let old = &edges[p];
                            let (Ok(before), Ok(after)) = (
                                Interval::new(old.range.lo(), t),
                                Interval::new(t, old.range.hi()),
                            ) else {
                                break;
                            };
                            let (vertex, new) = (points.len(), edges.len());
                            let piece = Edge {
                                geometry: old.geometry.clone(),
                                curve: old.curve,
                                range: after,
                                start: vertex,
                                end: old.end,
                                along: old.along,
                            };
                            points.push(row.point);
                            vertex_ids.push(edge_ids[p]);
                            edges[p].range = before;
                            edges[p].end = vertex;
                            edges.push(piece);
                            edge_ids.push(edge_ids[p]);
                            pieces[step.edge].insert(at + 1, new);
                        }
                    }
                }
            }
        }

        let mut walked = Vec::with_capacity(file.shells.len());
        for shell in &file.shells {
            let mut faces = Vec::with_capacity(shell.faces.len());
            for face in &shell.faces {
                let (surface, surface_id) = match surfaces.get(&face.surface.id) {
                    Some(s) => s.clone(),
                    None => {
                        let s = face.surface.resolve(&part)?;
                        let id = model.add_surface(s.clone());
                        surfaces.insert(face.surface.id, (s.clone(), id));
                        (s, id)
                    }
                };
                // Outward is the file face's normal turned by the shell or
                // an ORIENTED_FACE, the file face's normal is the file
                // surface's turned where `same_sense` is false, and the
                // variant's normal is the file surface's turned by
                // `reversed`.
                let flipped = face.surface.reversed ^ !face.same_sense ^ face.turned;
                let mut pcurves: BTreeMap<usize, Curve2> = BTreeMap::new();
                let singular = surface.singularities();
                let at = junctions(&surface, &singular, &points, cap, precision, flipped);
                // A VERTEX_LOOP is joined to the face's one other bound by
                // a seam, which Arris's face needs and the file left out.
                let vertex_bounds: Vec<(u64, usize)> = (face.bounds.iter())
                    .filter_map(|b| match b.walk {
                        Walk::Vertex(v) => Some((b.id, v)),
                        Walk::Edges(_) => None,
                    })
                    .collect();
                let joined = match vertex_bounds[..] {
                    [] => None,
                    [one] if face.bounds.len() == 2 => Some(one),
                    [(bound, _), ..] => {
                        return Err(Refusal::Unsupported {
                            entity: bound,
                            name: "a VERTEX_LOOP beside other than one EDGE_LOOP".into(),
                        });
                    }
                };
                let mut loops = Vec::with_capacity(face.bounds.len());
                for bound in &face.bounds {
                    let Walk::Edges(ref walk) = bound.walk else {
                        continue;
                    };
                    let mut walk = walk.clone();
                    if face.turned {
                        turn(&mut walk);
                    }
                    // Each file edge's pieces, in the walk's direction.
                    let walk: Vec<Step> = (walk.iter())
                        .flat_map(|s| {
                            let mut run = pieces[s.edge].clone();
                            if s.forward != edges[s.edge].along {
                                run.reverse();
                            }
                            run.into_iter().map(|edge| Step {
                                edge,
                                forward: s.forward,
                            })
                        })
                        .collect();
                    let mut uses = Vec::with_capacity(walk.len());
                    for s in walk {
                        let edge = &edges[s.edge];
                        let pcurve = match pcurves.get(&s.edge) {
                            Some(p) => p.clone(),
                            None => {
                                let id = edge_ids[s.edge];
                                let p =
                                    fitted(&edge.geometry, edge.range, &surface, precision, cap)
                                        .map_err(|fault| fault.refusal(id, face.id, cap))?;
                                pcurves.insert(s.edge, p.clone());
                                p
                            }
                        };
                        let (orientation, vertices) = if s.forward == edge.along {
                            (Orientation::Forward, [edge.start, edge.end])
                        } else {
                            (Orientation::Reversed, [edge.end, edge.start])
                        };
                        uses.push(Use {
                            edge: EdgeRef::File(s.edge),
                            orientation,
                            pcurve,
                            range: edge.range,
                            vertices,
                        });
                    }
                    let seam = match joined {
                        Some((vertex_bound, apex)) => {
                            let unsupported = |name: &str| Refusal::Unsupported {
                                entity: vertex_bound,
                                name: name.into(),
                            };
                            let row = at.singular_at(apex).ok_or_else(|| {
                                unsupported("a VERTEX_LOOP off its surface's singular points")
                            })?;
                            let seam = seam_to(model, &surface, row, apex, &uses, &points, tol)
                                .map_err(unsupported)?;
                            Some(seam.join(&mut uses, &mut rebuilt))
                        }
                        None => None,
                    };
                    let mut uses = at.walk(uses, &mut rebuilt).map_err(|jump| match jump {
                        Jump::Open(vertex) => Refusal::OpenLoop {
                            face: face.id,
                            bound: bound.id,
                            vertex: vertex_ids[vertex],
                        },
                        Jump::Gap(vertex, gap) => Refusal::Gap {
                            entity: vertex_ids[vertex],
                            gap,
                            cap,
                        },
                    })?;
                    at.meet(&mut uses).map_err(|(edge, e)| Refusal::Pcurve {
                        edge: match edge {
                            EdgeRef::File(i) => edge_ids[i],
                            EdgeRef::Rebuilt(_) => face.id,
                        },
                        face: face.id,
                        what: e.to_string(),
                    })?;
                    if let (Some(i), Some((vertex_bound, _))) = (seam, joined) {
                        if seam_crosses(
                            &uses,
                            i,
                            &surface,
                            precision.parametric_tolerance,
                            precision.check_samples,
                        ) {
                            return Err(Refusal::Unsupported {
                                entity: vertex_bound,
                                name: "a VERTEX_LOOP whose seam to its face's bound crosses it"
                                    .into(),
                            });
                        }
                    }
                    loops.push(uses);
                }
                if joined.is_none() {
                    let unsupported = |name: &str| Refusal::Unsupported {
                        entity: face.id,
                        name: name.into(),
                    };
                    let samples = precision.check_samples;
                    let cut = band(&at, &edges, model, &mut loops, &mut rebuilt, tol, samples)
                        .map_err(unsupported)?;
                    if let Some(Fix::Apex { face_loop, row }) = cut {
                        let apex = points.len();
                        points.push(row.point);
                        vertex_ids.push(face.id);
                        let at = junctions(&surface, &singular, &points, cap, precision, flipped);
                        let mut uses = std::mem::take(&mut loops[face_loop]);
                        let seam = seam_to(model, &surface, row, apex, &uses, &points, tol)
                            .map_err(unsupported)?;
                        let i = seam.join(&mut uses, &mut rebuilt);
                        let mut uses = at.walk(uses, &mut rebuilt).map_err(|_| {
                            unsupported("a face of one wrapping loop whose seam to its apex jumps")
                        })?;
                        at.meet(&mut uses).map_err(|_| {
                            unsupported("a face of one wrapping loop whose seam to its apex does not meet it")
                        })?;
                        if seam_crosses(&uses, i, &surface, at.parametric, samples) {
                            return Err(unsupported(
                                "a face of one wrapping loop whose seam to its apex crosses it",
                            ));
                        }
                        loops[face_loop] = uses;
                    } else if let Some(Fix::Cut(cut)) = cut {
                        // The seam's far end is a vertex on the other
                        // loop's edge, which is split there.
                        if cut.edge >= file.edges.len() || splits.iter().any(|s| s.edge == cut.edge)
                        {
                            return Err(unsupported(
                                "a band whose seam would split an edge a second time",
                            ));
                        }
                        let old = &edges[cut.edge];
                        let split = Split {
                            edge: cut.edge,
                            t: cut.t,
                            vertex: points.len(),
                            new: edges.len(),
                        };
                        let range = Interval::new(cut.t, old.range.hi())
                            .map_err(|_| unsupported("a band's seam at its edge's end"))?;
                        points.push(cut.point);
                        vertex_ids.push(edge_ids[cut.edge]);
                        edges.push(Edge {
                            geometry: old.geometry.clone(),
                            curve: old.curve,
                            range,
                            start: split.vertex,
                            end: old.end,
                            along: old.along,
                        });
                        edge_ids.push(edge_ids[cut.edge]);
                        for uses in &mut loops {
                            split.apply(uses);
                        }
                        splits.push(split);
                        let at = junctions(&surface, &singular, &points, cap, precision, flipped);
                        if band(&at, &edges, model, &mut loops, &mut rebuilt, tol, samples)
                            .map_err(unsupported)?
                            .is_some()
                        {
                            return Err(unsupported(
                                "a band whose split edge still leaves no seam",
                            ));
                        }
                    }
                }
                rebuilt_faces.resize(rebuilt.len(), face.id);
                faces.push((surface, surface_id, flipped, loops));
            }
            walked.push(faces);
        }

        // Every other use of a split edge is split with it, and the edge
        // ends at the new vertex.
        for split in &splits {
            for (_, _, _, loops) in walked.iter_mut().flatten() {
                for uses in loops.iter_mut() {
                    split.apply(uses);
                }
            }
            let edge = &mut edges[split.edge];
            edge.range =
                Interval::new(edge.range.lo(), split.t).map_err(|e| Refusal::Degenerate {
                    entity: edge_ids[split.edge],
                    what: e.to_string(),
                })?;
            edge.end = split.vertex;
        }

        let mut gaps = Gaps::new(points.len());
        let mut shells = Vec::with_capacity(walked.len());
        for faces in walked {
            let mut specs = Vec::with_capacity(faces.len());
            for (surface, surface_id, flipped, mut loops) in faces {
                place_loops(&mut loops, surface.period(), surface.domain());
                for u in loops.iter_mut().flatten() {
                    hold_in_domain(&mut u.pcurve, u.range, &surface, precision);
                }
                for uses in &loops {
                    gaps.measure_uses(&surface, uses, &points, &edges, &rebuilt, model, precision);
                }
                let loops = loops
                    .into_iter()
                    .map(|uses| {
                        uses.into_iter()
                            .map(|u| UseSpec {
                                edge: EdgeKey::New(match u.edge {
                                    EdgeRef::File(i) => i,
                                    EdgeRef::Rebuilt(i) => edges.len() + i,
                                }),
                                orientation: u.orientation,
                                pcurve: model.add_curve2(u.pcurve),
                            })
                            .collect()
                    })
                    .collect();
                specs.push(FaceSpec::New {
                    surface: surface_id,
                    orientation: if flipped {
                        Orientation::Reversed
                    } else {
                        Orientation::Forward
                    },
                    loops,
                    tolerance,
                });
            }
            shells.push(specs);
        }

        gaps.measure_edges(&points, &edges, &rebuilt, model);
        let (vertex_tolerances, edge_tolerances) = gaps
            .tolerances(&edges, &rebuilt, tolerance, cap)
            .map_err(|(entity, gap)| Refusal::Gap {
                entity: match entity {
                    Entity::Vertex(v) => vertex_ids[v],
                    Entity::Edge(k) if k < edges.len() => edge_ids[k],
                    Entity::Edge(k) => rebuilt_faces[k - edges.len()],
                },
                gap,
                cap,
            })?;
        let assembly = Assembly {
            vertices: (points.iter().zip(&vertex_tolerances))
                .map(|(&point, &tolerance)| VertexSpec::New { point, tolerance })
                .collect(),
            edges: edges
                .iter()
                .map(|e| {
                    (
                        EdgeGeometry::Curve {
                            curve: e.curve,
                            range: e.range,
                        },
                        e.start,
                        e.end,
                    )
                })
                .chain(rebuilt.iter().map(|r| (r.geometry, r.start, r.end)))
                .zip(&edge_tolerances)
                .map(|((geometry, start, end), &tolerance)| EdgeSpec::New {
                    geometry,
                    start: VertexKey::New(start),
                    end: VertexKey::New(end),
                    tolerance,
                })
                .collect(),
            shells,
        };
        let topology = |e: arris_check::arris_topo::builder::BuildError| Refusal::Topology {
            entity: solid,
            what: e.to_string(),
        };
        let (builder, slots) = Builder::assemble(model, tolerance, assembly).map_err(topology)?;
        let built = builder.finish(model, BodyKind::Solid).map_err(topology)?;

        let report = check(model, built.body, Level::Fast);
        if !report.is_ok() {
            return Err(Refusal::Invalid {
                entity: solid,
                report: Box::new(report),
            });
        }

        let mut provenance = Provenance::new();
        for (&v, slot) in vertex_ids.iter().zip(&slots.vertices) {
            if let Some(&id) = built.vertices.get(slot) {
                provenance.add_generated(file_entity(v), Shape::new(id, Orientation::Forward));
            }
        }
        // A rebuilt edge is generated from its face's entity.
        let edge_entities = edge_ids.into_iter().chain(rebuilt_faces);
        for (e, slot) in edge_entities.zip(&slots.edges) {
            if let Some(&id) = built.edges.get(slot) {
                provenance.add_generated(file_entity(e), Shape::new(id, Orientation::Forward));
            }
        }
        for (shell, shell_slots) in file.shells.iter().zip(&slots.faces) {
            for (face, slot) in shell.faces.iter().zip(shell_slots) {
                if let Some(&id) = built.faces.get(slot) {
                    provenance
                        .add_generated(file_entity(face.id), Shape::new(id, Orientation::Forward));
                }
            }
        }
        for (shell, &id) in file.shells.iter().zip(&built.shells) {
            provenance.add_generated(file_entity(shell.id), Shape::new(id, Orientation::Forward));
        }
        provenance.add_generated(file_entity(solid), built.body);
        Ok(Solid {
            body: built.body,
            provenance,
        })
    }
}

/// An edge with its geometry resolved.
struct Edge {
    geometry: Curve,
    curve: CurveId,
    range: Interval,
    /// The Arris edge's vertices: the file's, swapped where it runs
    /// against its curve.
    start: usize,
    end: usize,
    /// The file edge runs along its Arris curve.
    along: bool,
}

/// The edge a use walks: a file edge, by its index, or one the reader
/// rebuilt, by its index among those.
#[derive(Clone, Copy)]
enum EdgeRef {
    File(usize),
    Rebuilt(usize),
}

/// An edge the file left out: a degenerate edge at a singular point, or
/// the seam a `VERTEX_LOOP` needs to join its face's other loop, or a
/// band's two loops.
#[derive(Clone)]
struct Rebuilt {
    geometry: EdgeGeometry,
    start: usize,
    end: usize,
}

/// One use of a loop, with its pcurve moved to where the loop needs it.
#[derive(Clone)]
struct Use {
    edge: EdgeRef,
    orientation: Orientation,
    pcurve: Curve2,
    range: Interval,
    /// The vertices it starts and ends at, in the walk's direction.
    vertices: [usize; 2],
}

impl Use {
    /// Where the use is in (u, v) half way along it.
    fn midpoint(&self) -> Point2 {
        self.pcurve.point(self.range.midpoint())
    }

    /// The direction the walk heads in (u, v) at its start (`0`) or its
    /// end (`1`).
    fn heading(&self, end: usize) -> Vec2 {
        let forward = matches!(self.orientation, Orientation::Forward);
        let t = if forward == (end == 1) {
            self.range.hi()
        } else {
            self.range.lo()
        };
        let d1 = self.pcurve.eval(t).d1;
        if forward { d1 } else { -d1 }
    }

    /// Where the use starts and ends in (u, v).
    fn ends(&self) -> (Point2, Point2) {
        let (a, b) = (
            self.pcurve.point(self.range.lo()),
            self.pcurve.point(self.range.hi()),
        );
        match self.orientation {
            Orientation::Forward => (a, b),
            Orientation::Reversed => (b, a),
        }
    }
}

/// A walk turned: the other way round, each use the other way.
fn turn(walk: &mut [Step]) {
    walk.reverse();
    for s in walk {
        s.forward = !s.forward;
    }
}

/// A `.T.`/`.F.` parameter.
fn logical(args: &Args<'_>, i: usize) -> Result<bool, Refusal> {
    match args.enumeration(i)? {
        "T" => Ok(true),
        "F" => Ok(false),
        other => Err(args.malformed(format!(
            "{} parameter {} is .{other}., not .T. or .F.",
            args.record.name,
            i + 1
        ))),
    }
}

/// The ball around a union of boxes, its radius at least `floor` so that
/// a part of one point still has one.
fn ball_of(boxes: impl Iterator<Item = Aabb>, floor: f64) -> Ball {
    match boxes.reduce(Aabb::union) {
        Some(b) => {
            let [x, y, z] = b.center();
            Ball {
                centre: Point3::new(x, y, z),
                radius: (0.5 * b.diagonal()).max(floor),
            }
        }
        None => Ball {
            centre: Point3::origin(),
            radius: floor,
        },
    }
}

/// The range of the edge `id` on `curve` from `start` to `end`: the
/// vertices' parameters, the upper moved on by the period where it falls
/// below the lower. A closed edge — one vertex — takes the full period
/// from it, or the domain of a curve that closes without one.
fn edge_range(
    id: u64,
    curve: &Curve,
    start: Point3,
    end: Point3,
    closed: bool,
) -> Result<Interval, Refusal> {
    let degenerate = |what: String| Refusal::Degenerate { entity: id, what };
    let at = |p: Point3| -> Result<f64, Refusal> {
        let t = curve
            .project(p)
            .map_err(|e| {
                degenerate(format!(
                    "a vertex of the edge does not project onto its curve: {e}"
                ))
            })?
            .t;
        Ok(match curve.period() {
            Some(_) => t,
            None => curve.domain().clamp(t),
        })
    };
    let range = |lo: f64, mut hi: f64| {
        // `lo + p − lo` can round past `p`, which a range on a periodic
        // curve may not span (the checker's E1): the end comes down by
        // the units in the last place it went over.
        if let Some(p) = curve.period() {
            while hi - lo > p {
                hi = below(hi);
            }
        }
        Interval::new(lo, hi).map_err(|e| degenerate(e.to_string()))
    };
    let t0 = at(start)?;
    match (closed, curve.period()) {
        (true, Some(p)) => range(t0, t0 + p),
        (true, None) => {
            let d = curve.domain();
            if !d.is_bounded() {
                return Err(degenerate("a closed edge on an open curve".into()));
            }
            range(d.lo(), d.hi())
        }
        (false, period) => {
            let mut t1 = at(end)?;
            if let Some(p) = period {
                if t1 <= t0 {
                    t1 += p * ((t0 - t1) / p).floor() + p;
                }
            }
            if t1 <= t0 {
                return Err(degenerate(format!(
                    "the edge's end is at {t1} on its curve, not past its start at {t0}"
                )));
            }
            range(t0, t1)
        }
    }
}

/// The largest `f64` below the finite `x`: `f64::next_down`, stable from
/// Rust 1.86, where the workspace supports 1.85.
fn below(x: f64) -> f64 {
    if x == 0.0 {
        return -f64::from_bits(1);
    }
    let bits = x.to_bits();
    f64::from_bits(if x > 0.0 { bits - 1 } else { bits + 1 })
}

/// What a loop's junctions are judged against: the face's surface, its
/// singular points, and the solid's vertices.
struct Junctions<'a> {
    surface: &'a Surface,
    singular: &'a [Singularity],
    points: &'a [Point3],
    /// How near a singular point a vertex is on it: `pcurve_on`'s own
    /// band, so a pcurve ends on the row exactly where a junction is
    /// judged to be there.
    band: f64,
    /// How far the image of a jump in (u, v) may stray from its vertex
    /// before the loop is open there: the gap cap.
    cap: f64,
    /// The model's parametric tolerance: two values along a singular row
    /// nearer than it are one.
    parametric: f64,
    /// The model's angular tolerance: two headings nearer than it to one
    /// line make no turn.
    angular: f64,
    /// The tolerance a pcurve ended on a junction is first fitted to,
    /// where it has to be fitted first: the model's default, grown by
    /// [`GAP_GROWTH`] up to the cap where the fit misses.
    fit: f64,
    /// The face's effective normal is its surface's turned, so a walk in
    /// effective order keeps the face on its right in (u, v).
    flipped: bool,
}

impl Junctions<'_> {
    /// The singular point of the surface the vertex is on, if any.
    fn singular_at(&self, vertex: usize) -> Option<Singularity> {
        let at = self.points.get(vertex)?;
        (self.singular.iter().copied()).find(|s| (at - s.point).norm() <= self.band)
    }

    /// The sense, `±1`, in which a walk runs along the row of the free
    /// parameter `free` to keep the face, which lies on `side` of the
    /// row in the fixed parameter, on its left about the effective
    /// normal: `+u` with the face above, `−v` with it to the right.
    fn sense(&self, free: usize, side: f64) -> f64 {
        let left = if free == 0 { side } else { -side };
        if self.flipped { -left } else { left }
    }

    /// The loop's uses made one walk in (u, v): each use's pcurve moved by
    /// whole periods so that it starts where the use before it ends —
    /// which puts a seam's two uses a period apart — and, where two uses
    /// meet at a singular point, the degenerate edge the file left out
    /// put between them along the row, in the sense that keeps the face
    /// on the walk's left. The walk starts at a use that does not start
    /// at a singular point, where there is one, so it closes where it
    /// needs no move. Errors: the index of the vertex where the walk
    /// jumps in (u, v) and not along a singular row.
    fn walk(&self, mut uses: Vec<Use>, rebuilt: &mut Vec<Rebuilt>) -> Result<Vec<Use>, Jump> {
        if let Some(first) = (uses.iter()).position(|u| self.singular_at(u.vertices[0]).is_none()) {
            uses.rotate_left(first);
        }
        let mut rest = uses.into_iter();
        let Some(first) = rest.next() else {
            return Ok(Vec::new());
        };
        let mut out = Vec::with_capacity(rest.len() + 2);
        let mut prev = first;
        for mut next in rest {
            let (by, row) = self.junction(&prev, &next, true, rebuilt)?;
            if by != Vec2::zeros() {
                next.pcurve = next.pcurve.translated(by);
            }
            out.push(prev);
            out.extend(row);
            prev = next;
        }
        let first = out.first().unwrap_or(&prev);
        let (_, row) = self.junction(&prev, first, false, rebuilt)?;
        out.push(prev);
        out.extend(row);
        Ok(out)
    }

    /// The junction from `prev` to `next`: the whole periods `next` is
    /// to be moved by (none where it may not be moved, the loop's
    /// closing junction, which is then only checked), and the degenerate
    /// use between them at a singular point.
    fn junction(
        &self,
        prev: &Use,
        next: &Use,
        movable: bool,
        rebuilt: &mut Vec<Rebuilt>,
    ) -> Result<(Vec2, Option<Use>), Jump> {
        let period = self.surface.period();
        let vertex = prev.vertices[1];
        let (_, end) = prev.ends();
        let (start, _) = next.ends();
        let d = start - end;
        let Some(row) = self.singular_at(vertex) else {
            let by = whole_periods(-d, period);
            if self.jumps(end, d + by) {
                let gap = self.gap(end, d + by);
                if gap > self.cap {
                    return Err(Jump::Gap(vertex, gap));
                }
                return Err(Jump::Open(vertex));
            }
            return Ok((if movable { by } else { Vec2::zeros() }, None));
        };
        let (fixed, free) = (row.fixed, 1 - row.fixed);
        let side = [prev.midpoint(), next.midpoint()]
            .iter()
            .map(|m| m[fixed] - row.value)
            .find(|&x| x != 0.0)
            .ok_or(Jump::Open(vertex))?
            .signum();
        let sense = self.sense(free, side);
        // Where the two uses meet the row at one value of it, the walk
        // runs along it not at all when it turns there towards the face,
        // keeping a wedge of it between them; where it turns back on
        // itself or away — a seam's two uses — it runs a whole turn.
        let meet = |run: f64| run <= self.parametric && self.turns_in(prev, next);
        let mut by = whole_periods(-d, period);
        let run = match period[free] {
            Some(p) => {
                let run = (sense * d[free]).rem_euclid(p);
                let run = if meet(run.min(p - run)) {
                    0.0
                } else if run <= self.parametric {
                    p
                } else {
                    run
                };
                by[free] = p * ((sense * run - d[free]) / p).round();
                run
            }
            None if meet(d[free].abs()) => 0.0,
            None => sense * d[free],
        };
        if !movable && by != Vec2::zeros() {
            return Err(Jump::Open(vertex));
        }
        if run == 0.0 {
            return Ok((by, None));
        }
        let along = d + by;
        if run < 0.0 || sense * along[free] <= 0.0 {
            return Err(Jump::Open(vertex));
        }
        let degenerate = self.degenerate(vertex, end, along, rebuilt);
        Ok((by, Some(degenerate.map_err(Jump::Open)?)))
    }

    /// `true` when the walk turns from `prev` into `next` towards the
    /// face: left in (u, v), or right on a face whose effective normal is
    /// its surface's turned. Headings within the angular tolerance of one
    /// line make no turn: a seam's two uses at a pole head exactly apart,
    /// and one of them moved by a period rounds its tangent.
    fn turns_in(&self, prev: &Use, next: &Use) -> bool {
        let (a, b) = (prev.heading(1), next.heading(0));
        let turn = a.perp(&b);
        if turn.abs() <= self.angular * a.norm() * b.norm() {
            return false;
        }
        if self.flipped { turn < 0.0 } else { turn > 0.0 }
    }

    /// A degenerate use at `vertex` from `from` by `along` in (u, v), its
    /// edge entered in `rebuilt`.
    fn degenerate(
        &self,
        vertex: usize,
        from: Point2,
        along: Vec2,
        rebuilt: &mut Vec<Rebuilt>,
    ) -> Result<Use, usize> {
        let direction = UnitVec2::try_new(along, 0.0).ok_or(vertex)?;
        let range = Interval::new(0.0, along.norm()).map_err(|_| vertex)?;
        rebuilt.push(Rebuilt {
            geometry: EdgeGeometry::Degenerate { range },
            start: vertex,
            end: vertex,
        });
        Ok(Use {
            edge: EdgeRef::Rebuilt(rebuilt.len() - 1),
            orientation: Orientation::Forward,
            pcurve: Curve2::Line {
                origin: from,
                direction,
            },
            range,
            vertices: [vertex, vertex],
        })
    }

    /// The vertex's own (u, v) on the surface: its point's projection,
    /// in the translate nearest `near`. `None` where it does not project.
    fn own_uv(&self, vertex: usize, near: Point2) -> Option<Point2> {
        let p = *self.points.get(vertex)?;
        let uv = self.surface.project(p).ok()?.uv;
        Some(uv + whole_periods(near - uv, self.surface.period()))
    }

    /// How far apart in 3D the two ends of the step `gap` from `at` in
    /// (u, v) are.
    fn gap(&self, at: Point2, gap: Vec2) -> f64 {
        let to = at + gap;
        let (a, b) = (
            self.surface.point(at.x, at.y),
            self.surface.point(to.x, to.y),
        );
        (a - b).norm()
    }

    /// `true` when the step `gap` from `at` in (u, v) is a jump: the
    /// surface half way along it is farther than the tolerance from where
    /// it starts, so two edges that meet in 3D do not meet on the face.
    fn jumps(&self, at: Point2, gap: Vec2) -> bool {
        if gap == Vec2::zeros() {
            return false;
        }
        let mid = at + 0.5 * gap;
        let (a, b) = (
            self.surface.point(at.x, at.y),
            self.surface.point(mid.x, mid.y),
        );
        (a - b).norm() > self.cap || (a - b).norm().is_nan()
    }

    /// The loop's pcurves ended on one point at every junction where they
    /// end apart in (u, v) by more than the band L2 holds a junction to —
    /// two curves ending apart in 3D on a vertex; a file whose pcurves
    /// meet within it is read as it is: the seam's end where one of the two is a
    /// seam of the loop, whose two uses must stay a period apart (E7),
    /// else the vertex's own (u, v) — its point's projection — as a boolean
    /// ends a section edge on its vertex, or half way where it does not
    /// project. Each use moved keeps its image within the move of
    /// where it was ([`pcurve_ending_on`]); the gap it opens is measured
    /// afterwards with the rest. Errors: the edge whose pcurve could not
    /// be ended, and why.
    fn meet(&self, uses: &mut [Use]) -> Result<(), (EdgeRef, GeomError)> {
        let n = uses.len();
        let seam = |u: &Use| match u.edge {
            EdgeRef::File(i) => {
                (uses.iter())
                    .filter(|w| matches!(w.edge, EdgeRef::File(j) if j == i))
                    .count()
                    > 1
            }
            EdgeRef::Rebuilt(_) => false,
        };
        let period = self.surface.period();
        // Per use, where its start and end (in the walk's direction) go.
        let mut moves: Vec<[Option<Point2>; 2]> = vec![[None; 2]; n];
        for i in 0..n {
            let j = (i + 1) % n;
            let (prev, next) = (&uses[i], &uses[j]);
            let (_, end) = prev.ends();
            let (start, _) = next.ends();
            let d = start - end;
            let wrap = whole_periods(-d, period);
            let gap = d + wrap;
            let band = bands(self.surface, end, self.parametric);
            if gap.x.abs() <= band[0] && gap.y.abs() <= band[1] {
                continue;
            }
            let (ends_prev, ends_next) = match (seam(prev), seam(next)) {
                (true, true) => continue,
                (true, false) => (None, Some(end - wrap)),
                (false, true) => (Some(start + wrap), None),
                (false, false) => {
                    let at = self
                        .own_uv(prev.vertices[1], end)
                        .unwrap_or(end + 0.5 * gap);
                    (Some(at), Some(at - wrap))
                }
            };
            if ends_prev.is_some() {
                moves[i][1] = ends_prev;
            }
            if ends_next.is_some() {
                moves[j][0] = ends_next;
            }
        }
        for (u, [start, end]) in uses.iter_mut().zip(moves) {
            if start.is_none() && end.is_none() {
                continue;
            }
            let ends = match u.orientation {
                Orientation::Forward => [start, end],
                Orientation::Reversed => [end, start],
            };
            // A pcurve that must be fitted first is fitted as `fitted`
            // fits one: from the default, grown up to the cap.
            let mut linear = self.fit;
            u.pcurve = loop {
                match pcurve_ending_on(
                    &u.pcurve,
                    u.range,
                    ends,
                    self.surface,
                    linear,
                    &mut arris_check::arris_topo::arris_math::Meter::default(),
                ) {
                    Ok(p) => break p,
                    Err(GeomError::Fit(_)) if linear < self.cap => {
                        linear = (GAP_GROWTH * linear).min(self.cap);
                    }
                    Err(e) => return Err((u.edge, e)),
                }
            };
        }
        Ok(())
    }
}

/// How much the tolerance a pcurve is fitted to grows each time its
/// curve is found farther from the surface than it, or its fit misses —
/// a file's curve fitted within a tolerance of the surface, which
/// wanders about it by a fraction of it: from the model's default to the
/// cap in a handful of fits. A search step, not a tolerance — the pcurve's own deviation is
/// measured afterwards and is what its edge carries.
const GAP_GROWTH: f64 = 2.0;

/// Why an edge has no pcurve on a face.
enum PcurveFault {
    /// The curve is farther from the surface than the cap: the distance,
    /// or the tolerance a fit would need.
    Gap(f64),
    /// What [`pcurve_on`] refuses at a tolerance the gap does not explain.
    Geom(GeomError),
}

impl PcurveFault {
    fn refusal(self, edge: u64, face: u64, cap: f64) -> Refusal {
        match self {
            PcurveFault::Gap(gap) => Refusal::Gap {
                entity: edge,
                gap,
                cap,
            },
            PcurveFault::Geom(e) => Refusal::Pcurve {
                edge,
                face,
                what: e.to_string(),
            },
        }
    }
}

/// The pcurve of `curve` over `range` on `surface`, fitted at the model's
/// default tolerance, or — where the curve lies farther from the surface
/// than that, or wanders about it so that no fit reaches it — at the gap
/// it lies at, grown by [`GAP_GROWTH`] up to one growth past the cap.
///
/// The tolerance a fit is asked for is a search step, not the edge's: a
/// fit holds its image to a fraction of it, and that image is never
/// nearer the curve than the curve is to the surface. A curve 0.0065 off
/// its surface cannot be fitted at a cap of 0.01 for that reason alone,
/// where its pcurve, fitted past the cap, lies within 0.0066 of it
/// (NIST's CTC-01). So the fit may be asked for more than the cap, and
/// the gap its pcurve leaves, measured afterwards, is what the cap judges
/// ([`Gaps::tolerances`]).
///
/// For the same reason, a fit asked for less than twice the curve's
/// farthest distance from the surface cannot pass, and on a NURBS surface
/// such a miss costs seconds: a fit refined to thousands of spans before
/// it gives up. So after the first miss the curve's farthest distance is
/// measured ([`off_surface`]), and the next fit is asked for
/// [`GAP_GROWTH`]² of it, which a smooth projection meets at once, rather
/// than climbing to it one miss at a time (NIST's FTC-07 and CTC-05).
/// Errors: the curve farther from the surface
/// than the cap, as [`PcurveFault::Gap`] with the distance; a fit that
/// fails even past the cap, as [`PcurveFault::Geom`] with the fit's own
/// error — a fit, not a gap.
fn fitted(
    curve: &Curve,
    range: Interval,
    surface: &Surface,
    precision: Precision,
    cap: f64,
) -> Result<Curve2, PcurveFault> {
    let ceiling = GAP_GROWTH * cap;
    let mut linear = precision.default_tolerance;
    let mut measured = false;
    loop {
        let tol = Tolerance::new(linear, precision.angular_tolerance);
        let needed = match pcurve_on(
            curve,
            range,
            surface,
            tol,
            &mut arris_check::arris_topo::arris_math::Meter::default(),
        ) {
            Ok(p) => return Ok(p),
            Err(GeomError::NotOnSurface { distance, .. }) => {
                if distance > cap {
                    return Err(PcurveFault::Gap(distance));
                }
                GAP_GROWTH * linear.max(distance)
            }
            Err(e @ GeomError::Fit(_)) if linear >= ceiling => {
                return Err(PcurveFault::Geom(e));
            }
            Err(GeomError::Fit(_)) => GAP_GROWTH * linear,
            Err(e) => return Err(PcurveFault::Geom(e)),
        };
        if linear >= ceiling {
            return Err(PcurveFault::Gap(needed));
        }
        let needed = if measured {
            needed
        } else {
            measured = true;
            let off = off_surface(curve, range, surface, precision.default_tolerance);
            if off > cap {
                return Err(PcurveFault::Gap(off));
            }
            needed.max(GAP_GROWTH * GAP_GROWTH * off)
        };
        linear = needed.min(ceiling);
    }
}

/// The farthest `curve` lies from `surface` over `range`: at the
/// [`PCURVE_SAMPLES`] a fit is checked at, each peak past `floor` climbed
/// to its top ([`worst_gap`]). A parameter whose point does not project
/// counts as on the surface, and the fit that follows says otherwise.
fn off_surface(curve: &Curve, range: Interval, surface: &Surface, floor: f64) -> f64 {
    let ts = samples(range, PCURVE_SAMPLES + 1);
    worst_gap(&ts, floor, |t| {
        surface.project(curve.point(t)).map_or(0.0, |p| p.distance)
    })
}

/// A vertex or an edge of the solid, by its index: an edge past the
/// file's is one the reader rebuilt.
#[derive(Clone, Copy)]
enum Entity {
    Vertex(usize),
    Edge(usize),
}

/// The gaps measured on the solid's vertices and edges, which their
/// tolerances are set from (ADR-0025 §4).
struct Gaps {
    vertex: Vec<f64>,
    /// By edge index: the file's edges, then the rebuilt ones.
    edge: Vec<f64>,
}

/// `n` parameters over `range`, both ends included — the checker's own
/// samples, so a gap is measured where the checker will look.
fn samples(range: Interval, n: usize) -> Vec<f64> {
    if n <= 1 {
        return vec![range.midpoint()];
    }
    (0..n)
        .map(|i| range.lerp(i as f64 / (n - 1) as f64))
        .collect()
}

/// The largest of `gap` over the span of the sorted parameters `ts`: its
/// largest value at them, each local peak past `floor` climbed by golden
/// section between its neighbours. A peak at or below `floor` is left as
/// sampled, since the tolerance is floored there anyway — every exact
/// pcurve's gap is rounding, and costs nothing more.
fn worst_gap(ts: &[f64], floor: f64, gap: impl Fn(f64) -> f64) -> f64 {
    let gaps: Vec<f64> = ts.iter().map(|&t| gap(t)).collect();
    let mut worst = gaps.iter().copied().fold(0.0, f64::max);
    let last = gaps.len().saturating_sub(1);
    for (i, &g) in gaps.iter().enumerate() {
        let (before, after) = (i.saturating_sub(1), (i + 1).min(last));
        if g <= floor || g < gaps[before] || g < gaps[after] || before == after {
            continue;
        }
        let ratio = 0.5 * (5f64.sqrt() - 1.0);
        let (mut a, mut b) = (ts[before], ts[after]);
        let (mut x1, mut x2) = (b - ratio * (b - a), a + ratio * (b - a));
        let (mut g1, mut g2) = (gap(x1), gap(x2));
        for _ in 0..PEAK_STEPS {
            if g1 >= g2 {
                (b, x2, g2) = (x2, x1, g1);
                x1 = b - ratio * (b - a);
                g1 = gap(x1);
            } else {
                (a, x1, g1) = (x1, x2, g2);
                x2 = a + ratio * (b - a);
                g2 = gap(x2);
            }
        }
        worst = worst.max(g1).max(g2);
    }
    worst
}

/// Golden-section steps of [`worst_gap`]'s climb: each shrinks the
/// bracket by 0.618, so forty take two sample intervals to below `1e-8`
/// of one, where a smooth peak's value has settled to rounding. An
/// iteration count, not a tolerance.
const PEAK_STEPS: usize = 40;

/// A distance between `a` and `b`, raised by the rounding at their own
/// scale: what keeps an entity within its tolerance when the body is
/// moved, which rounds every point it compares.
fn distance(a: Point3, b: Point3) -> f64 {
    (a - b).norm() + RELATIVE_ROUNDING * a.coords.norm().max(b.coords.norm())
}

impl Gaps {
    fn new(vertices: usize) -> Gaps {
        Gaps {
            vertex: vec![0.0; vertices],
            edge: Vec::new(),
        }
    }

    fn raise_vertex(&mut self, v: usize, gap: f64) {
        if let Some(g) = self.vertex.get_mut(v) {
            *g = g.max(gap);
        }
    }

    fn raise_edge(&mut self, k: usize, gap: f64) {
        if self.edge.len() <= k {
            self.edge.resize(k + 1, 0.0);
        }
        self.edge[k] = self.edge[k].max(gap);
    }

    /// The gaps one loop's uses leave on the face's surface: each use's
    /// pcurve image from its curve at the checker's samples (E4), from
    /// its vertices at its ends (V3), and a degenerate use's image's span
    /// (E6).
    #[allow(clippy::too_many_arguments)]
    fn measure_uses(
        &mut self,
        surface: &Surface,
        uses: &[Use],
        points: &[Point3],
        edges: &[Edge],
        rebuilt: &[Rebuilt],
        model: &Model,
        precision: Precision,
    ) {
        for u in uses {
            let (key, curve) = match u.edge {
                EdgeRef::File(i) => (i, edges.get(i).map(|e| &e.geometry)),
                EdgeRef::Rebuilt(i) => (
                    edges.len() + i,
                    match rebuilt.get(i).map(|r| r.geometry) {
                        Some(EdgeGeometry::Curve { curve, .. }) => model.curve(curve).ok(),
                        _ => None,
                    },
                ),
            };
            let image = |t: f64| {
                let q = u.pcurve.point(t);
                surface.point(q.x, q.y)
            };
            let [first, last] = match u.orientation {
                Orientation::Forward => u.vertices,
                Orientation::Reversed => [u.vertices[1], u.vertices[0]],
            };
            for (v, t) in [(first, u.range.lo()), (last, u.range.hi())] {
                if let Some(&p) = points.get(v) {
                    self.raise_vertex(v, distance(image(t), p));
                }
            }
            let ts = samples(u.range, precision.check_samples);
            match curve {
                Some(c) => {
                    // The checker's samples, and the ones a fit is held
                    // to, each peak then climbed: between samples a
                    // fitted pcurve strays as far as the fit allows it,
                    // and a finer look — the mesh's — finds it there.
                    let mut ts = ts;
                    ts.extend(samples(u.range, PCURVE_SAMPLES + 1));
                    ts.sort_by(f64::total_cmp);
                    ts.dedup();
                    let worst = worst_gap(&ts, precision.default_tolerance, |t| {
                        distance(image(t), c.point(t))
                    });
                    self.raise_edge(key, worst);
                }
                None => {
                    let images: Vec<Point3> = ts.iter().map(|&t| image(t)).collect();
                    let mut extent: f64 = 0.0;
                    for (i, &p) in images.iter().enumerate() {
                        for &q in &images[i + 1..] {
                            extent = extent.max(distance(p, q));
                        }
                    }
                    self.raise_vertex(first, extent);
                }
            }
        }
    }

    /// The gaps each edge curve leaves at its ends: from its vertices
    /// (V2), and, on a closed edge, from itself (E2).
    fn measure_edges(
        &mut self,
        points: &[Point3],
        edges: &[Edge],
        rebuilt: &[Rebuilt],
        model: &Model,
    ) {
        let curves = (edges.iter())
            .map(|e| (Some(&e.geometry), e.range, e.start, e.end))
            .chain(rebuilt.iter().map(|r| match r.geometry {
                EdgeGeometry::Curve { curve, range } => {
                    (model.curve(curve).ok(), range, r.start, r.end)
                }
                EdgeGeometry::Degenerate { range } => (None, range, r.start, r.end),
            }));
        for (k, (curve, range, start, end)) in curves.enumerate() {
            let Some(c) = curve else { continue };
            let (a, b) = (c.point(range.lo()), c.point(range.hi()));
            for (v, at) in [(start, a), (end, b)] {
                if let Some(&p) = points.get(v) {
                    self.raise_vertex(v, distance(at, p));
                }
            }
            if start == end {
                self.raise_edge(k, distance(a, b));
            }
        }
    }

    /// The vertices' and edges' tolerances: each its gap floored at
    /// `floor`, each vertex raised to the edges it bounds. Errors: the
    /// first edge, then the first vertex, whose gap is past `cap`, with
    /// the gap.
    fn tolerances(
        &self,
        edges: &[Edge],
        rebuilt: &[Rebuilt],
        floor: f64,
        cap: f64,
    ) -> Result<(Vec<f64>, Vec<f64>), (Entity, f64)> {
        let ends: Vec<(usize, usize)> = (edges.iter().map(|e| (e.start, e.end)))
            .chain(rebuilt.iter().map(|r| (r.start, r.end)))
            .collect();
        let gap = |k: usize| self.edge.get(k).copied().unwrap_or(0.0);
        if let Some(k) = (0..ends.len()).find(|&k| gap(k) > cap) {
            return Err((Entity::Edge(k), gap(k)));
        }
        if let Some(v) = (0..self.vertex.len()).find(|&v| self.vertex[v] > cap) {
            return Err((Entity::Vertex(v), self.vertex[v]));
        }
        let edge: Vec<f64> = (0..ends.len()).map(|k| floor.max(gap(k))).collect();
        let mut vertex: Vec<f64> = self.vertex.iter().map(|&g| floor.max(g)).collect();
        for (&(start, end), &t) in ends.iter().zip(&edge) {
            for v in [start, end] {
                if let Some(x) = vertex.get_mut(v) {
                    *x = x.max(t);
                }
            }
        }
        Ok((vertex, edge))
    }
}

/// Why a loop's walk in (u, v) breaks at a junction, naming the vertex
/// there.
enum Jump {
    /// The two uses meet in 3D but not on the face, and not along a
    /// singular row: [`Refusal::OpenLoop`].
    Open(usize),
    /// The two uses end farther apart in 3D than the cap:
    /// [`Refusal::Gap`], with the distance.
    Gap(usize, f64),
}

/// The seam from a vertex of a face's bound to the singular point a
/// `VERTEX_LOOP` of the face stands on, which the face needs to be one
/// region in (u, v) and the file left out.
struct Seam {
    /// The bound's vertex it starts at, and the `VERTEX_LOOP`'s.
    from: usize,
    to: usize,
    curve: CurveId,
    range: Interval,
    pcurve: Curve2,
}

impl Seam {
    /// The bound's uses turned to start at the seam's vertex and closed
    /// by the seam to the singular point and back, the degenerate edge
    /// between left to the walk; the seam entered in `rebuilt`, whose
    /// index is returned.
    fn join(self, uses: &mut Vec<Use>, rebuilt: &mut Vec<Rebuilt>) -> usize {
        if let Some(i) = uses.iter().position(|u| u.vertices[0] == self.from) {
            uses.rotate_left(i);
        }
        rebuilt.push(Rebuilt {
            geometry: EdgeGeometry::Curve {
                curve: self.curve,
                range: self.range,
            },
            start: self.from,
            end: self.to,
        });
        let i = rebuilt.len() - 1;
        for (orientation, vertices) in [
            (Orientation::Forward, [self.from, self.to]),
            (Orientation::Reversed, [self.to, self.from]),
        ] {
            uses.push(Use {
                edge: EdgeRef::Rebuilt(i),
                orientation,
                pcurve: self.pcurve.clone(),
                range: self.range,
                vertices,
            });
        }
        i
    }
}

/// The seam from the vertex of `uses` nearest the singular point `row`
/// — the vertex `apex`, a `VERTEX_LOOP`'s — to it, along the row's
/// other parameter: a cone's ruling or a sphere's meridian, exact.
/// Errors: why there is none: a NURBS surface's row, whose seam would
/// be an isocurve Arris has no exact form of, or a vertex on the point.
fn seam_to(
    model: &mut Model,
    surface: &Surface,
    row: Singularity,
    apex: usize,
    uses: &[Use],
    points: &[Point3],
    tol: Tolerance,
) -> Result<Seam, &'static str> {
    let from = (uses.iter())
        .map(|u| u.vertices[0])
        .filter_map(|v| Some((v, (points.get(v)? - row.point).norm())))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(v, _)| v)
        .ok_or("a VERTEX_LOOP beside a bound of no vertex")?;
    let start = points
        .get(from)
        .copied()
        .ok_or("a vertex the solid does not list")?;
    let (curve, span) = match *surface {
        Surface::Cone { .. } => {
            let direction = UnitVec3::try_new(row.point - start, 0.0)
                .ok_or("a VERTEX_LOOP whose face's bound reaches its apex")?;
            let length = (row.point - start).norm();
            (
                Curve::Line {
                    origin: start,
                    direction,
                },
                length,
            )
        }
        Surface::Sphere { ref frame, radius } => {
            let (x, pole) = (start - frame.origin(), row.point - frame.origin());
            let circle = Frame::new(frame.origin(), x.cross(&pole), x)
                .map_err(|_| "a VERTEX_LOOP whose face's bound reaches its pole")?;
            let angle = pole.dot(&circle.y()).atan2(pole.dot(&circle.x()));
            (
                Curve::Circle {
                    frame: circle,
                    radius,
                },
                angle,
            )
        }
        Surface::Nurbs(_) => {
            return Err("a VERTEX_LOOP on a NURBS surface, whose seam has no exact curve");
        }
        Surface::Plane { .. }
        | Surface::Cylinder { .. }
        | Surface::EllipticCylinder { .. }
        | Surface::Torus { .. } => return Err("a VERTEX_LOOP on a surface with no singular point"),
    };
    let range = Interval::new(0.0, span).map_err(|_| "a seam of no length")?;
    let pcurve = pcurve_on(
        &curve,
        range,
        surface,
        tol,
        &mut arris_check::arris_topo::arris_math::Meter::default(),
    )
    .map_err(|_| "a VERTEX_LOOP whose seam has no pcurve on its face")?;
    Ok(Seam {
        from,
        to: apex,
        curve: model.add_curve(curve),
        range,
        pcurve,
    })
}

/// The parameter strictly inside `edge`'s range at which its curve runs
/// through `point` within `band`, away from its ends: where a pcurve on a
/// surface singular there cannot run on.
fn through(edge: &Edge, point: Point3, band: f64) -> Option<f64> {
    let (lo, hi) = (edge.range.lo(), edge.range.hi());
    let at_end = |t: f64| (edge.geometry.point(t) - point).norm() <= band;
    if at_end(lo) || at_end(hi) {
        return None;
    }
    let near = edge.geometry.project(point).ok()?;
    if near.distance > band {
        return None;
    }
    let t = match edge.geometry.period() {
        Some(p) => lo + (near.t - lo).rem_euclid(p),
        None => near.t,
    };
    (lo < t && t < hi).then_some(t)
}

/// Holds `pcurve` over `range` inside a NURBS surface's domain where it
/// leaves it — past the parametric band at the checker's samples, F1's
/// test — in a direction the surface does not close in: its control
/// points moved into the domain, which holds the whole curve there, since
/// a B-spline of positive weights lies in its control points' hull. A
/// fit between projections the domain clamps overshoots it on an edge
/// along the surface's border, by a fraction of the fit's tolerance —
/// NIST's FTC-07, 6e-7 of a unit domain on edges fitted at 5.5e-3 — and
/// the move is of that order, in the gap the pcurve leaves, which is
/// measured after it. A pcurve inside its domain is left as it is, even
/// where its control polygon is not.
fn hold_in_domain(pcurve: &mut Curve2, range: Interval, surface: &Surface, precision: Precision) {
    let (Surface::Nurbs(_), Curve2::Nurbs(c)) = (surface, &*pcurve) else {
        return;
    };
    let (domain, period) = (surface.domain(), surface.period());
    let leaves = samples(range, precision.check_samples)
        .into_iter()
        .any(|t| {
            let uv = c.eval(t).point;
            let band = bands(surface, uv, precision.parametric_tolerance);
            (0..2).any(|k| {
                period[k].is_none()
                    && !(domain[k].lo() - band[k] <= uv[k] && uv[k] <= domain[k].hi() + band[k])
            })
        });
    if !leaves {
        return;
    }
    let points: Vec<Point2> = (c.control_points().iter())
        .map(|p| {
            let mut q = *p;
            for k in 0..2 {
                if period[k].is_none() {
                    q[k] = domain[k].clamp(q[k]);
                }
            }
            q
        })
        .collect();
    if points == c.control_points() {
        return;
    }
    if let Ok(held) = NurbsCurve2::new(c.degree(), c.knots().to_vec(), points, c.weights().to_vec())
    {
        *pcurve = Curve2::Nurbs(held);
    }
}

/// The index a band's seam walks under while its arc is tried, before it
/// is entered in `rebuilt`: no rebuilt edge has it.
const TRIED_SEAM: usize = usize::MAX;

/// A face whose two loops each wrap once around the same period of its
/// surface — a cylinder's side bounded by its two circles, as ISO
/// 10303-42 allows and the file writes it — made one loop, as Arris's
/// face needs: the two joined by a seam along the surface's isocurve
/// through a vertex of each, walked out along it and back (docs/
/// DATA-MODEL.md §Invariants, L4). The seam is a ruling on a cylinder or
/// a cone, a meridian on a sphere, and a circle on a torus, whichever of
/// its two arcs crosses neither loop. A face with other than two wrapping
/// loops is left as it is. Errors: why there is no such seam — a NURBS
/// surface's isocurve, which Arris has no exact form of, or loops whose
/// vertices lie on no one isocurve, where a seam would need an edge
/// split.
fn band(
    at: &Junctions<'_>,
    edges: &[Edge],
    model: &mut Model,
    loops: &mut Vec<Vec<Use>>,
    rebuilt: &mut Vec<Rebuilt>,
    tol: Tolerance,
    check_samples: usize,
) -> Result<Option<Fix>, &'static str> {
    let period = at.surface.period();
    // A loop that does not chain vertex to vertex is no loop, and is left
    // to the builder to refuse.
    let wraps = |uses: &[Use]| -> Option<usize> {
        let chains = (uses.iter().zip(uses.iter().cycle().skip(1)))
            .all(|(u, next)| u.vertices[1] == next.vertices[0]);
        if !chains {
            return None;
        }
        let (first, last) = (uses.first()?, uses.last()?);
        // Once round, in one parameter: a loop wound twice is no band.
        let turns = whole_periods(last.ends().1 - first.ends().0, period);
        let once = |k: usize| period[k].is_some_and(|p| (turns[k].abs() / p - 1.0).abs() < 0.5);
        match (turns[0] != 0.0, turns[1] != 0.0) {
            (true, false) if once(0) => Some(0),
            (false, true) if once(1) => Some(1),
            _ => None,
        }
    };
    let wrapping: Vec<(usize, usize)> = (loops.iter().enumerate())
        .filter_map(|(i, l)| Some((i, wraps(l)?)))
        .collect();
    let (i, k, j, other) = match wrapping[..] {
        [(i, k), (j, other)] => (i, k, j, other),
        [(i, k)] => return apex(at, &loops[i], i, k).map(Some),
        _ => return Ok(None),
    };
    if k != other {
        return Err("a face of two loops wrapping different periods of its surface");
    }
    for a in 0..loops[i].len() {
        for b in 0..loops[j].len() {
            let (va, vb) = (loops[i][a].vertices[0], loops[j][b].vertices[0]);
            let (Some(&pa), Some(&pb)) = (at.points.get(va), at.points.get(vb)) else {
                continue;
            };
            for (curve, range) in isocurves(at.surface, k, pa, pb, tol)? {
                let Ok(pcurve) = pcurve_on(
                    &curve,
                    range,
                    at.surface,
                    tol,
                    &mut arris_check::arris_topo::arris_math::Meter::default(),
                ) else {
                    continue;
                };
                let (mut first, mut second) = (loops[i].clone(), loops[j].clone());
                first.rotate_left(a);
                second.rotate_left(b);
                let seam = |orientation, vertices| Use {
                    edge: EdgeRef::Rebuilt(TRIED_SEAM),
                    orientation,
                    pcurve: pcurve.clone(),
                    range,
                    vertices,
                };
                first.push(seam(Orientation::Forward, [va, vb]));
                first.extend(second);
                first.push(seam(Orientation::Reversed, [vb, va]));
                let mut tried = rebuilt.clone();
                let Ok(mut uses) = at.walk(first, &mut tried) else {
                    continue;
                };
                if at.meet(&mut uses).is_err()
                    || seam_crosses(&uses, TRIED_SEAM, at.surface, at.parametric, check_samples)
                {
                    continue;
                }
                tried.push(Rebuilt {
                    geometry: EdgeGeometry::Curve {
                        curve: model.add_curve(curve),
                        range,
                    },
                    start: va,
                    end: vb,
                });
                let index = tried.len() - 1;
                for u in &mut uses {
                    if matches!(u.edge, EdgeRef::Rebuilt(TRIED_SEAM)) {
                        u.edge = EdgeRef::Rebuilt(index);
                    }
                }
                *rebuilt = tried;
                let (lo, hi) = (i.min(j), i.max(j));
                loops.remove(hi);
                loops[lo] = uses;
                return Ok(None);
            }
        }
    }
    // No vertex of the one loop faces one of the other: the other's edge
    // is to be cut where the first vertex of the one faces it.
    let from = (loops[i].first())
        .and_then(|u| at.points.get(u.vertices[0]))
        .and_then(|&p| at.surface.project(p).ok())
        .ok_or("a band whose loop has no vertex")?;
    let (target, turn) = (from.uv[k], period[k].unwrap_or(TAU));
    // How far the use's periodic parameter is from the target, wrapped to
    // half a period either side.
    let off = |u: &Use, t: f64| {
        let d = u.pcurve.point(t)[k] - target;
        d - turn * (d / turn).round()
    };
    for u in &loops[j] {
        let EdgeRef::File(edge) = u.edge else {
            continue;
        };
        let ts = samples(u.range, PCURVE_SAMPLES + 1);
        for w in ts.windows(2) {
            let (mut a, mut b) = (w[0], w[1]);
            let (fa, fb) = (off(u, a), off(u, b));
            // A sign change, not the wrap of the offset at half a turn.
            if fa * fb > 0.0 || (fa - fb).abs() > 0.25 * turn {
                continue;
            }
            for _ in 0..ROOT_STEPS {
                let m = 0.5 * (a + b);
                if off(u, a) * off(u, m) <= 0.0 {
                    b = m;
                } else {
                    a = m;
                }
            }
            let t = 0.5 * (a + b);
            if t <= u.range.lo() || t >= u.range.hi() {
                continue;
            }
            let point = edges
                .get(edge)
                .ok_or("a band's edge the solid does not list")?
                .geometry
                .point(t);
            return Ok(Some(Fix::Cut(Cut { edge, t, point })));
        }
    }
    Err("a band whose two loops have no vertices on one isocurve of its surface")
}

/// Bisections of [`band`]'s search for where a loop faces a vertex of
/// the other: sixty-four halve a sampling interval past the spacing of
/// `f64` parameters. An iteration count, not a tolerance.
const ROOT_STEPS: usize = 64;

/// What a face of wrapping loops needs before [`band`] can make it one
/// loop: an edge cut, or a vertex at its apex.
enum Fix {
    /// The other loop's edge split where the seam meets it.
    Cut(Cut),
    /// The one wrapping loop `face_loop` joined to the singular point
    /// `row`, a vertex the file left out, as a `VERTEX_LOOP` is.
    Apex { face_loop: usize, row: Singularity },
}

/// The singular point a face of one loop wrapping the period `k` closes
/// at: its surface's, on the side of the loop the face lies on — a cone
/// bounded by its base circle alone, its apex implicit, or a sphere by one
/// parallel. The side is the walk's: the face is on its left about the
/// effective normal ([`Junctions::sense`]). Errors: no singular point on
/// that side, where the face would be unbounded.
fn apex(at: &Junctions<'_>, uses: &[Use], face_loop: usize, k: usize) -> Result<Fix, &'static str> {
    let (Some(first), Some(last)) = (uses.first(), uses.last()) else {
        return Err("a face of one wrapping loop of no use");
    };
    let (start, end) = (first.ends().0, last.ends().1);
    let heading = (end[k] - start[k]).signum();
    let free = 1 - k;
    // `sense(k, side)` is the heading that keeps a face on `side` of the
    // loop on its left; it is ±1, so this is the side it keeps.
    let side = heading * at.sense(k, 1.0);
    (at.singular.iter())
        .filter(|s| s.fixed == free && (s.value - start[free]) * side > 0.0)
        .min_by(|a, b| {
            (a.value - start[free])
                .abs()
                .total_cmp(&(b.value - start[free]).abs())
        })
        .map(|&row| Fix::Apex { face_loop, row })
        .ok_or("a face of one loop wrapping a period, with no singular point on its side")
}

/// Where a band's seam cuts the other loop: the file edge, at its
/// parameter `t`, where its curve is at `point`.
struct Cut {
    edge: usize,
    t: f64,
    point: Point3,
}

/// A file edge split at `t` by a band's seam: the part before `t` keeps
/// the edge's index and ends at `vertex`, the part after is edge `new`,
/// from `vertex` on.
struct Split {
    edge: usize,
    t: f64,
    vertex: usize,
    new: usize,
}

impl Split {
    /// Each use of the edge whose range holds `t` inside it made two, in
    /// the walk's order, on the one pcurve: the edge's parameter is the
    /// pcurve's, so each part is the use over its part of the range.
    fn apply(&self, uses: &mut Vec<Use>) {
        let mut out = Vec::with_capacity(uses.len() + 1);
        for u in uses.drain(..) {
            let inside = matches!(u.edge, EdgeRef::File(e) if e == self.edge)
                && u.range.lo() < self.t
                && self.t < u.range.hi();
            let (Ok(before), Ok(after), true) = (
                Interval::new(u.range.lo(), self.t),
                Interval::new(self.t, u.range.hi()),
                inside,
            ) else {
                out.push(u);
                continue;
            };
            let [first, last] = u.vertices;
            let part = |edge, range, vertices| Use {
                edge: EdgeRef::File(edge),
                orientation: u.orientation,
                pcurve: u.pcurve.clone(),
                range,
                vertices,
            };
            let (a, b) = (
                part(self.edge, before, [first, self.vertex]),
                part(self.new, after, [self.vertex, last]),
            );
            match u.orientation {
                Orientation::Forward => out.extend([a, b]),
                Orientation::Reversed => out.extend([
                    part(self.new, after, [first, self.vertex]),
                    part(self.edge, before, [self.vertex, last]),
                ]),
            }
        }
        *uses = out;
    }
}

/// The judge of a face's junctions (see [`Junctions`]'s fields).
fn junctions<'a>(
    surface: &'a Surface,
    singular: &'a [Singularity],
    points: &'a [Point3],
    cap: f64,
    precision: Precision,
    flipped: bool,
) -> Junctions<'a> {
    Junctions {
        surface,
        singular,
        points,
        band: PCURVE_SINGULAR_BAND * precision.default_tolerance,
        cap,
        parametric: precision.parametric_tolerance,
        angular: precision.angular_tolerance,
        fit: precision.default_tolerance,
        flipped,
    }
}

/// The pieces of the isocurve of `surface` along which the parameter `k`
/// stays at `a`'s value, from `a` to `b`, the shorter arc first where
/// there are two; none where `b` is not on it. Errors: a surface whose
/// isocurve Arris has no exact form of.
fn isocurves(
    surface: &Surface,
    k: usize,
    a: Point3,
    b: Point3,
    tol: Tolerance,
) -> Result<Vec<(Curve, Interval)>, &'static str> {
    let (Ok(pa), Ok(pb)) = (surface.project(a), surface.project(b)) else {
        return Ok(Vec::new());
    };
    let mut across = pb.uv;
    across[k] = pa.uv[k];
    if (surface.point(across.x, across.y) - pb.point).norm() > tol.linear {
        return Ok(Vec::new());
    }
    let line = || -> Vec<(Curve, Interval)> {
        let length = (b - a).norm();
        match (UnitVec3::try_new(b - a, 0.0), Interval::new(0.0, length)) {
            (Some(direction), Ok(range)) => vec![(
                Curve::Line {
                    origin: a,
                    direction,
                },
                range,
            )],
            _ => Vec::new(),
        }
    };
    // The two arcs from `a` to `b` of the circle about `centre` in the
    // plane normal to `normal`, the shorter first.
    let arcs = |centre: Point3, normal: Vec3| -> Vec<(Curve, Interval)> {
        let radius = (a - centre).norm();
        let mut out = Vec::new();
        for n in [normal, -normal] {
            let Ok(frame) = Frame::new(centre, n, a - centre) else {
                continue;
            };
            let d = b - centre;
            let angle = d.dot(&frame.y()).atan2(d.dot(&frame.x()));
            let angle = if angle > 0.0 { angle } else { angle + TAU };
            if let Ok(range) = Interval::new(0.0, angle) {
                out.push((Curve::Circle { frame, radius }, range));
            }
        }
        out.sort_by(|x, y| x.1.hi().total_cmp(&y.1.hi()));
        out
    };
    Ok(match *surface {
        Surface::Cylinder { .. } | Surface::EllipticCylinder { .. } | Surface::Cone { .. } => {
            line()
        }
        Surface::Sphere { ref frame, .. } => arcs(
            frame.origin(),
            (a - frame.origin()).cross(&(b - frame.origin())),
        ),
        Surface::Torus {
            ref frame,
            major_radius,
            minor_radius,
        } => {
            let (u, v) = (pa.uv.x, pa.uv.y);
            let radial = u.cos() * frame.x().into_inner() + u.sin() * frame.y().into_inner();
            let axis = frame.z().into_inner();
            if k == 0 {
                arcs(frame.origin() + major_radius * radial, axis.cross(&radial))
            } else {
                arcs(frame.origin() + minor_radius * v.sin() * axis, axis)
            }
        }
        Surface::Nurbs(_) => {
            return Err("a band on a NURBS surface, whose seam has no exact curve");
        }
        Surface::Plane { .. } => Vec::new(),
    })
}

/// `true` when a bound's use crosses the seam `seam` of the walk `uses`
/// anywhere but at its ends, judged on `samples` chords of each use's
/// pcurve, each moved whole by the periods that bring its start nearest
/// the seam — its two ends moved apart would make a chord across the
/// domain of one that passes the seam's far side. A
/// crossing within the surface's parametric band (`bands`) of an end of
/// the seam is the loop meeting it at its vertex, where rounding decides
/// the side.
fn seam_crosses(
    uses: &[Use],
    seam: usize,
    surface: &Surface,
    parametric: f64,
    samples: usize,
) -> bool {
    let period = surface.period();
    let Some(s) = uses.iter().find(|u| {
        matches!(u.edge, EdgeRef::Rebuilt(i) if i == seam)
            && matches!(u.orientation, Orientation::Forward)
    }) else {
        return false;
    };
    let (a, b) = s.ends();
    let side = |p: Point2, q: Point2, r: Point2| (q - p).perp(&(r - p));
    let near = |p: Point2, end: Point2| {
        let band = bands(surface, end, parametric);
        (p.x - end.x).abs() <= band[0] && (p.y - end.y).abs() <= band[1]
    };
    let proper = |c: Point2, d: Point2| {
        let (sa, sb) = (side(c, d, a), side(c, d, b));
        if !(side(a, b, c) * side(a, b, d) < 0.0 && sa * sb < 0.0) {
            return false;
        }
        let x = a + (sa / (sa - sb)) * (b - a);
        !near(x, a) && !near(x, b)
    };
    let n = samples.max(1);
    uses.iter()
        .filter(|u| matches!(u.edge, EdgeRef::File(_)))
        .any(|u| {
            let at = |k: usize| u.pcurve.point(u.range.lerp(k as f64 / n as f64));
            (0..n).any(|k| {
                let (c, d) = (at(k), at(k + 1));
                let by = whole_periods(a - c, period);
                proper(c + by, d + by)
            })
        })
}

/// `d` rounded to whole periods in each periodic parameter, zero in the
/// others.
fn whole_periods(d: Vec2, period: [Option<f64>; 2]) -> Vec2 {
    let round = |x: f64, p: Option<f64>| p.map_or(0.0, |p| p * (x / p).round());
    Vec2::new(round(d.x, period[0]), round(d.y, period[1]))
}

/// Moves each loop by whole periods, parameter by parameter: the widest
/// loop so that the centre of its box lies in the surface's own period
/// (`domain`'s, `[0, 2π)` on the analytic surfaces) — the translate a
/// projection answers in, within a period of which a face's domain looks
/// for a point (`arris_check::domain::shifts`) — and every other so that
/// its centre lies in the period starting at the widest loop's lower
/// bound: a hole lands inside its outer loop, whose box spans at most a
/// period.
fn place_loops(loops: &mut [Vec<Use>], period: [Option<f64>; 2], domain: [Interval; 2]) {
    let boxes: Vec<[[f64; 2]; 2]> = loops.iter().map(|l| loop_box(l)).collect();
    for k in 0..2 {
        let Some(p) = period[k] else { continue };
        let Some(widest) = (0..boxes.len()).max_by(|&a, &b| {
            let w = |i: usize| boxes[i][1][k] - boxes[i][0][k];
            w(a).total_cmp(&w(b)).then(b.cmp(&a))
        }) else {
            continue;
        };
        let [lo, hi] = [boxes[widest][0][k], boxes[widest][1][k]];
        let base = if domain[k].lo().is_finite() {
            domain[k].lo()
        } else {
            0.0
        };
        let lo = lo - p * ((0.5 * (lo + hi) - base) / p).floor();
        for (i, l) in loops.iter_mut().enumerate() {
            let centre = 0.5 * (boxes[i][0][k] + boxes[i][1][k]);
            let shift = -p * ((centre - lo) / p).floor();
            if shift != 0.0 {
                let mut by = Vec2::zeros();
                by[k] = shift;
                for u in l.iter_mut() {
                    u.pcurve = u.pcurve.translated(by);
                }
            }
        }
    }
}

/// The box in (u, v) of a loop's uses' ends and midpoints.
fn loop_box(uses: &[Use]) -> [[f64; 2]; 2] {
    let mut b = [[f64::INFINITY; 2], [f64::NEG_INFINITY; 2]];
    for u in uses {
        let (s, e) = u.ends();
        let m = u.pcurve.point(u.range.midpoint());
        for p in [s, e, m] {
            for k in 0..2 {
                b[0][k] = b[0][k].min(p[k]);
                b[1][k] = b[1][k].max(p[k]);
            }
        }
    }
    b
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_pcurve_past_a_nurbs_domain_is_held_inside_it() {
        use arris_check::arris_topo::arris_geom::NurbsSurface;
        let corners = [(0.0, 0.0), (0.0, 1.0), (1.0, 0.0), (1.0, 1.0)];
        let sheet = Surface::Nurbs(
            NurbsSurface::new(
                [1, 1],
                [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
                corners.map(|(x, y)| Point3::new(x, y, 0.0)).to_vec(),
                vec![1.0; 4],
            )
            .unwrap(),
        );
        // Along the border u = 0, dipping 6e-7 below it half way.
        let dip = vec![
            Point2::new(0.0, 0.0),
            Point2::new(-6e-7, 0.5),
            Point2::new(0.0, 1.0),
        ];
        let mut pcurve = Curve2::Nurbs(
            NurbsCurve2::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], dip, vec![1.0; 3]).unwrap(),
        );
        let unit = Interval::new(0.0, 1.0).unwrap();
        hold_in_domain(&mut pcurve, unit, &sheet, Precision::DEFAULT);
        for i in 0..=64 {
            let q = pcurve.point(f64::from(i) / 64.0);
            assert!(
                (0.0..=1.0).contains(&q.x) && (0.0..=1.0).contains(&q.y),
                "{q}"
            );
        }
        // Inside already, a pcurve is left as it is.
        let inside = Curve2::Line {
            origin: Point2::new(0.5, 0.0),
            direction: UnitVec2::new_normalize(Vec2::new(0.0, 1.0)),
        };
        let mut same = inside.clone();
        hold_in_domain(&mut same, unit, &sheet, Precision::DEFAULT);
        assert_eq!(same, inside);
    }

    #[test]
    fn a_gap_peaking_between_samples_is_measured_at_its_top() {
        // A bump of height 3 centred between the samples at 0.5 and 0.75,
        // which see it at a fraction of its top.
        let bump = |t: f64| 3.0 * (-((t - 0.6) / 0.08).powi(2)).exp();
        let ts = samples(Interval::new(0.0, 1.0).unwrap(), 5);
        assert!(ts.iter().all(|&t| bump(t) < 1.0));
        assert!((worst_gap(&ts, 1e-7, bump) - 3.0).abs() < 1e-12);
        // At or below the floor, the samples stand.
        let low = |t: f64| 1e-8 * bump(t);
        assert!(worst_gap(&ts, 1e-7, low) < 1e-8);
    }
}
