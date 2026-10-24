use std::collections::BTreeMap;

use arris_math::Point3;

#[cfg(doc)]
use super::{BuildError, Builder, ClosureError, EulerError, SlotError};
use super::{EdgeRef, FaceRef, VertexRef};
use crate::entity::EdgeGeometry;
use crate::error::NotFound;
use crate::handle::{self, Body};
use crate::id::{Curve2Id, CurveId, EdgeId, FaceId, ShellId, SurfaceId, VertexId};
use crate::model::Model;
use crate::orientation::Orientation;

/// Which vertex an [`EdgeSpec`] is over: one already in the model, or the
/// one the assembly's `index`-th [`VertexSpec`] stands for — whichever
/// kind that spec is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VertexKey {
    /// The vertex with this arena id, kept.
    Kept(VertexId),
    /// The assembly's `0`-based vertex spec at this index.
    New(usize),
}

/// A vertex of an [`Assembly`]: one the model already holds, kept with its
/// id, or one to append.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VertexSpec {
    /// The vertex with this arena id, kept: its point and tolerance are the
    /// model's and [`Builder::finish`] appends nothing for it.
    Keep(VertexId),
    /// A vertex to append.
    New {
        /// Its point.
        point: Point3,
        /// The tolerance it is stored with.
        tolerance: f64,
    },
}

/// Which edge a [`UseSpec`] is over: one already in the model, or the one
/// the assembly's `index`-th [`EdgeSpec`] stands for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeKey {
    /// The edge with this arena id, kept.
    Kept(EdgeId),
    /// The assembly's `0`-based edge spec at this index.
    New(usize),
}

/// An edge of an [`Assembly`]: one the model already holds, kept with its
/// id, or one to append.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EdgeSpec {
    /// The edge with this arena id, kept: its geometry, vertices and
    /// tolerance are the model's, and its two vertices are kept with it.
    Keep(EdgeId),
    /// An edge to append.
    New {
        /// Its geometry.
        geometry: EdgeGeometry,
        /// The vertex at the lower end of the range.
        start: VertexKey,
        /// The vertex at the upper end.
        end: VertexKey,
        /// The tolerance it is stored with.
        tolerance: f64,
    },
}

/// One use of an edge by a loop of an assembled face, in the *effective*
/// orientation the Euler operators take: the direction the loop is walked
/// in as seen from outside the material, whatever the surface's own normal
/// (`docs/DATA-MODEL.md` §Orientation).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UseSpec {
    /// The edge.
    pub edge: EdgeKey,
    /// Along (`Forward`) or against (`Reversed`) the edge's curve.
    pub orientation: Orientation,
    /// The pcurve of this use on the face's surface.
    pub pcurve: Curve2Id,
}

/// The uses of one loop the other way it can be walked: from stored order
/// to *effective* order (as seen from outside the material, whatever the
/// surface's own normal, `docs/DATA-MODEL.md` §Orientation) or back —
/// composing an orientation with itself and reversing a list are each
/// their own inverse, so one function walks both ways. Every orientation
/// is composed with `face_orientation`, and the whole walk is reversed
/// when `face_orientation` is `Reversed`.
pub fn effective_uses<T>(
    face_orientation: Orientation,
    uses: impl IntoIterator<Item = (T, Orientation, Curve2Id)>,
) -> Vec<(T, Orientation, Curve2Id)> {
    let mut out: Vec<(T, Orientation, Curve2Id)> = uses
        .into_iter()
        .map(|(id, orientation, pcurve)| (id, face_orientation.compose(orientation), pcurve))
        .collect();
    if face_orientation.is_reversed() {
        out.reverse();
    }
    out
}

/// A face of an [`Assembly`]: one the model already holds, kept whole with
/// its id, or one to append.
#[derive(Debug, Clone, PartialEq)]
pub enum FaceSpec {
    /// The face with this arena id, used by its new shell with this
    /// orientation, kept: its surface, loops, pcurves and tolerance are the
    /// model's, and every edge and vertex it names is kept with it.
    Keep(handle::Face),
    /// A face to append.
    New {
        /// Its surface.
        surface: SurfaceId,
        /// `Forward` when the surface's normal is the outward one.
        orientation: Orientation,
        /// Its loops, each a walk in effective orientation. The first is
        /// no more the outer one than any other: a loop's role is the
        /// pcurves' business, not the builder's.
        loops: Vec<Vec<UseSpec>>,
        /// The tolerance it is stored with.
        tolerance: f64,
    },
}

impl FaceSpec {
    /// The [`FaceSpec::New`] that reads `face_use` whole from `model`: its
    /// surface, tolerance, and every loop's uses in effective order, each
    /// edge named through `edge_key` — what an operation that copies or
    /// moves a face whole wants, differing from another only in how it
    /// names an edge. Errors: `face_use` does not resolve.
    pub fn from_face(
        model: &Model,
        face_use: handle::Face,
        mut edge_key: impl FnMut(EdgeId) -> EdgeKey,
    ) -> Result<FaceSpec, NotFound> {
        let entity = model.face(face_use.id)?;
        let loops = entity
            .loops()
            .iter()
            .map(|l| {
                effective_uses(
                    face_use.orientation,
                    l.coedges()
                        .iter()
                        .map(|c| (c.edge(), c.orientation(), c.pcurve())),
                )
                .into_iter()
                .map(|(edge, orientation, pcurve)| UseSpec {
                    edge: edge_key(edge),
                    orientation,
                    pcurve,
                })
                .collect()
            })
            .collect();
        Ok(FaceSpec::New {
            surface: entity.surface(),
            orientation: face_use.orientation,
            loops,
            tolerance: entity.tolerance(),
        })
    }
}

/// The faces of one body, grouped into its shells, and the edges and
/// vertices they are over, each kept from the model or new: what
/// [`Builder::assemble`] takes.
///
/// The lists are addressed positionally by [`VertexKey::New`] and
/// [`EdgeKey::New`], so a caller that has a slot per entity of its
/// operands — which is what a boolean has — writes the table it already
/// holds. A `Keep` spec and a `Kept` key that name one arena entity are
/// one slot; naming an entity by two `Keep` specs is
/// [`ClosureError::Duplicate`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Assembly {
    /// The vertices, addressed by [`VertexKey::New`].
    pub vertices: Vec<VertexSpec>,
    /// The edges, addressed by [`EdgeKey::New`].
    pub edges: Vec<EdgeSpec>,
    /// The body's shells in the order the body stores them, each the
    /// faces it uses in order. A shell shares no edge and no vertex with
    /// another (`docs/DATA-MODEL.md` §Entities: a solid's lumps).
    pub shells: Vec<Vec<FaceSpec>>,
}

/// The slot [`Builder::assemble`] gave to each spec of the [`Assembly`] it
/// took, in spec order: a caller that has a slot per entity of its own
/// operands — a boolean, a sweep, a transform — looks its output up as
/// `built.vertices[&slots.vertices[i]]` rather than assuming the builder's
/// slot order matches the spec order, which interleaved `Keep` and `New`
/// specs need not.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct AssemblySlots {
    /// The slot of `assembly.vertices[i]`, at index `i`.
    pub vertices: Vec<VertexRef>,
    /// The slot of `assembly.edges[i]`, at index `i`.
    pub edges: Vec<EdgeRef>,
    /// The slot of `assembly.shells[s][i]`, at `[s][i]`.
    pub faces: Vec<Vec<FaceRef>>,
}

/// How [`Assembly::of_body`] treats a body's geometry as it copies it,
/// each asked once per occurrence: the identity ([`KeepGeometry`]) keeps
/// it; a transform moves it and, for a curve or surface, adds the result
/// (a vertex's point is a value, not an id, and has no arena entry of its
/// own to add).
pub trait GeometryRemap {
    /// The point to give a copied vertex, in place of `p`.
    fn point(&mut self, model: &mut Model, p: Point3) -> Point3;
    /// The curve to describe an edge over `c` with.
    fn curve(&mut self, model: &mut Model, c: CurveId) -> CurveId;
    /// The surface to describe a face over `s` with.
    fn surface(&mut self, model: &mut Model, s: SurfaceId) -> SurfaceId;
    /// The pcurve to give a copied use in place of `p`, which lay on
    /// `surface`, the *original* surface (not the one
    /// [`GeometryRemap::surface`] returned). The default keeps `p`: right
    /// wherever the surface's parametrisation is carried along, as a rigid
    /// motion does.
    fn pcurve(&mut self, _model: &mut Model, p: Curve2Id, _surface: SurfaceId) -> Curve2Id {
        p
    }
    /// What to change about a copied face over `surface`, the original
    /// surface. The default is [`FaceRemap::KEEP`]: the face is copied as
    /// it stands.
    fn face(&mut self, _model: &mut Model, _surface: SurfaceId) -> FaceRemap {
        FaceRemap::KEEP
    }
}

/// What [`Assembly::of_body`] changes about a face it copies, answered by
/// [`GeometryRemap::face`]: the two facts a map that reverses the winding
/// of a surface's parameters (a mirror, ADR-0031) needs said.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FaceRemap {
    /// The copy's use of its surface has the opposite orientation.
    pub toggle_use: bool,
    /// The copy's *stored* loops are the original's walked backwards, each
    /// coedge use's orientation toggled: the winding a stored loop must
    /// have in the `(u, v)` of a surface whose parameters were reflected.
    pub reverse_loops: bool,
}

impl FaceRemap {
    /// Change nothing: what a rigid motion asks.
    pub const KEEP: FaceRemap = FaceRemap {
        toggle_use: false,
        reverse_loops: false,
    };
}

/// A [`GeometryRemap`] that keeps every point, curve and surface as it is:
/// what a caller that copies a body's geometry unchanged passes.
#[derive(Debug, Clone, Copy, Default)]
pub struct KeepGeometry;

impl GeometryRemap for KeepGeometry {
    fn point(&mut self, _model: &mut Model, p: Point3) -> Point3 {
        p
    }

    fn curve(&mut self, _model: &mut Model, c: CurveId) -> CurveId {
        c
    }

    fn surface(&mut self, _model: &mut Model, s: SurfaceId) -> SurfaceId {
        s
    }
}

/// Where [`Assembly::of_body`] put each entity of the body it read: which
/// spec of the [`Assembly`], or, for a face, which shell and which spec
/// within it. What provenance is built from, reading the built id behind
/// a spec out of the matching [`AssemblySlots`].
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct BodyIndex {
    /// The vertex's spec index in [`Assembly::vertices`].
    pub vertices: BTreeMap<VertexId, usize>,
    /// The edge's spec index in [`Assembly::edges`].
    pub edges: BTreeMap<EdgeId, usize>,
    /// The face's shell index and spec index within it, into
    /// [`Assembly::shells`].
    pub faces: BTreeMap<FaceId, (usize, usize)>,
    /// The shell's index into [`Assembly::shells`].
    pub shells: BTreeMap<ShellId, usize>,
}

impl Assembly {
    /// The [`Assembly`] that describes `body` shell by shell with every
    /// entity `New`: a copy of its vertices, edges and faces, each curve
    /// and surface named through `remap` (the identity, [`KeepGeometry`],
    /// for a caller that wants the same geometry). What [`transform`]
    /// builds over, its own work reduced to `remap` and the provenance
    /// [`BodyIndex`] gives.
    ///
    /// Errors: `body` does not resolve.
    ///
    /// [`transform`]: ../../arris_ops/fn.transform.html
    ///
    /// ```
    /// use arris_topo::builder::{Assembly, Builder, KeepGeometry};
    /// use arris_topo::entity::BodyKind;
    /// use arris_topo::Model;
    /// use arris_debug::{dump_text, sample};
    ///
    /// let mut m = Model::default();
    /// let body = sample::cylinder(&mut m, 4.0, 12.0)?;
    /// let before = dump_text(&m, body)?;
    /// let (assembly, _index) = Assembly::of_body(&mut m, body, &mut KeepGeometry)?;
    /// let (b, _slots) = Builder::assemble(&m, m.precision().default_tolerance, assembly)?;
    /// let built = b.finish(&mut m, BodyKind::Solid)?;
    /// assert_ne!(built.body.id, body.id, "every entity is new");
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn of_body(
        model: &mut Model,
        body: Body,
        remap: &mut impl GeometryRemap,
    ) -> Result<(Assembly, BodyIndex), NotFound> {
        let closure = model.closure(body)?;

        let mut curve_of: BTreeMap<CurveId, CurveId> = BTreeMap::new();
        for &c in &closure.curves {
            curve_of.insert(c, remap.curve(model, c));
        }
        let mut surface_of: BTreeMap<SurfaceId, SurfaceId> = BTreeMap::new();
        for &s in &closure.surfaces {
            surface_of.insert(s, remap.surface(model, s));
        }

        let mut vertex_index = BTreeMap::new();
        let mut vertices = Vec::with_capacity(closure.vertices.len());
        for (i, &v) in closure.vertices.iter().enumerate() {
            let old = *model.vertex(v)?;
            vertices.push(VertexSpec::New {
                point: remap.point(model, old.point()),
                tolerance: old.tolerance(),
            });
            vertex_index.insert(v, i);
        }

        let mut edge_index = BTreeMap::new();
        let mut edges = Vec::with_capacity(closure.edges.len());
        for (i, &e) in closure.edges.iter().enumerate() {
            let old = *model.edge(e)?;
            let geometry = match old.geometry() {
                EdgeGeometry::Curve { curve, range } => EdgeGeometry::Curve {
                    curve: curve_of[&curve],
                    range,
                },
                degenerate @ EdgeGeometry::Degenerate { .. } => degenerate,
            };
            edges.push(EdgeSpec::New {
                geometry,
                start: VertexKey::New(vertex_index[&old.start()]),
                end: VertexKey::New(vertex_index[&old.end()]),
                tolerance: old.tolerance(),
            });
            edge_index.insert(e, i);
        }

        let mut face_index: BTreeMap<FaceId, (usize, usize)> = BTreeMap::new();
        let mut shell_index: BTreeMap<ShellId, usize> = BTreeMap::new();
        let mut shells = Vec::new();
        for (si, shell) in model.shells(body)?.into_iter().enumerate() {
            let uses: Vec<handle::Face> = model
                .shell(shell.id)?
                .faces()
                .iter()
                .map(|f| f.oriented_by(shell.orientation))
                .collect();
            let mut face_specs = Vec::with_capacity(uses.len());
            for (fi, f) in uses.iter().enumerate() {
                let old_surface = model.face(f.id)?.surface();
                let mut spec = FaceSpec::from_face(model, *f, |id| EdgeKey::New(edge_index[&id]))?;
                let change = remap.face(model, old_surface);
                if let FaceSpec::New {
                    surface,
                    orientation,
                    loops,
                    ..
                } = &mut spec
                {
                    *surface = surface_of[&old_surface];
                    let old_orientation = *orientation;
                    if change.toggle_use {
                        *orientation = orientation.compose(Orientation::Reversed);
                    }
                    for l in loops.iter_mut() {
                        // Effective order back to stored, the change, and
                        // forward again under the new use orientation.
                        let mut stored = effective_uses(
                            old_orientation,
                            l.iter().map(|u| (u.edge, u.orientation, u.pcurve)),
                        );
                        if change.reverse_loops {
                            stored.reverse();
                            for u in &mut stored {
                                u.1 = u.1.compose(Orientation::Reversed);
                            }
                        }
                        *l = effective_uses(*orientation, stored)
                            .into_iter()
                            .map(|(edge, orientation, pcurve)| UseSpec {
                                edge,
                                orientation,
                                pcurve: remap.pcurve(model, pcurve, old_surface),
                            })
                            .collect();
                    }
                }
                face_specs.push(spec);
                face_index.insert(f.id, (si, fi));
            }
            shell_index.insert(shell.id, si);
            shells.push(face_specs);
        }

        Ok((
            Assembly {
                vertices,
                edges,
                shells,
            },
            BodyIndex {
                vertices: vertex_index,
                edges: edge_index,
                faces: face_index,
                shells: shell_index,
            },
        ))
    }
}
