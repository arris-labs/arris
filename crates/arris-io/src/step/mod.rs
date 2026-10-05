//! STEP AP214 Part 21: the exchange structure's parser ([`part21`]), the
//! reader ([`read`], ADR-0025) and the writer (`docs/ARCHITECTURE.md`
//! §Formats and tools). The writer
//! emits the B-Rep entity subset with every pcurve written out, so a
//! reader takes the model's own trimming instead of recomputing it. The
//! Open CASCADE oracle reads the result (`tools/oracle/compare.py`).
//!
//! What is written, per lump of a solid body (`arris_check::lumps`,
//! ADR-0006): a `MANIFOLD_SOLID_BREP` over a `CLOSED_SHELL` of
//! `ADVANCED_FACE`s for a lump without voids, and for one with them a
//! `BREP_WITH_VOIDS` over the outer `CLOSED_SHELL` and an
//! `ORIENTED_CLOSED_SHELL` of orientation false per void, whose
//! `CLOSED_SHELL` holds the void's faces turned — the reversed shell the
//! reference tree's writer emits and its reader turns back into a hole;
//! per face its surface entity
//! (shared by every face and pcurve that references the same
//! `SurfaceId`), `same_sense` from the shell's use of the face, one
//! `FACE_OUTER_BOUND` for the loop of positive winding in (u, v) and a
//! `FACE_BOUND` for every other, each with the same orientation flag as
//! `same_sense` (the loop as stored is counter-clockwise about the
//! *surface* normal; STEP's bound is about the face's effective normal);
//! per coedge an `ORIENTED_EDGE` whose flag is the coedge's orientation;
//! per edge an `EDGE_CURVE` from its start to its end vertex over a
//! `SURFACE_CURVE` holding the 3D curve and one `PCURVE` per use — a
//! `SEAM_CURVE` with the `Forward` use's pcurve first when both uses are in
//! one loop — the curve shared by every edge on it, but an edge on a
//! periodic NURBS written on its own piece over its range
//! (`NurbsCurve::segment`), since a reader finds the range from the
//! vertices and a closed curve puts the block past its knots' end on the
//! wrong side of them; `LINE`, `CIRCLE`, `ELLIPSE`, `PLANE`, `CYLINDRICAL_SURFACE`,
//! `CONICAL_SURFACE`, `SPHERICAL_SURFACE`, `TOROIDAL_SURFACE` placed by
//! `AXIS2_PLACEMENT_3D` (origin, `Z`, `X`), a
//! `SURFACE_OF_LINEAR_EXTRUSION` of an `ELLIPSE` along its `Z` for an
//! elliptic cylinder (ADR-0014), and the `B_SPLINE_*` entities —
//! as complex entities with `RATIONAL_B_SPLINE_*` when a weight is not
//! one — so the writer is exhaustive over `Surface`, `Curve` and `Curve2`.
//! Lengths are written as millimetres and angles as radians because the
//! oracle's reader scales to millimetres by default and Arris carries no
//! unit, so every number passes through unchanged; the context's
//! uncertainty is the model's `default_tolerance`.
//!
//! Deterministic: entity numbers follow the bodies' iteration order, the
//! header's time stamp is empty (the kernel never reads the clock), and
//! every real is Rust's shortest round-trip decimal, so two writes of one
//! model are byte-identical and a reader parses the same `f64`.
//!
//! What STEP cannot carry, and what this writer does about it:
//!
//! - a **degenerate edge** has no 3D curve and `EDGE_CURVE` requires one:
//!   its coedge is left out of the `EDGE_LOOP`, as the reference tree's
//!   own writer does (its reader rebuilds the edge at the singularity); a
//!   loop with nothing else is [`Unsupported::DegenerateLoop`];
//! - a **left-handed pcurve conic** (clockwise in (u, v)) has no
//!   `AXIS2_PLACEMENT_2D`, which is always direct: it is written on the
//!   direct placement with the same `X`, so the point set is exact and
//!   only the direction of traversal is lost — a reader that trusts
//!   pcurves reprojects one whose sense disagrees with the edge, and Open
//!   CASCADE ignores pcurves on planes altogether, where every
//!   left-handed conic lives (`docs/DATA-MODEL.md`
//!   §Pcurves). The native format is the lossless one.

mod assembly;
pub mod part21;
mod reader;

pub use assembly::{TreeError, write_products};
pub use reader::{
    FaceColour, LengthUnit, Occurrence, ProductTree, Read, ReadBody, ReadError, ReadOptions,
    ReadSolid, Refusal, RefusalKind, Rgb, read,
};

use core::fmt::Write as _;
use std::collections::{BTreeMap, BTreeSet};

use arris_check::{LumpError, lumps};
use arris_geom::region2::{Piece, discretise};
use arris_geom::{Curve, Curve2, NurbsCurve, NurbsCurve2, NurbsSurface, Surface};
use arris_math::{Frame, Frame2, Interrupted, Interval, Point2, Point3, Vec2, Vec3};
use arris_topo::entity::{BodyKind, Coedge, Loop};
use arris_topo::{
    AnyId, Body, CoedgeRef, Curve2Id, CurveId, EdgeId, FaceId, Model, NotFound, Orientation, Shell,
    SurfaceId, VertexId,
};

/// Why a body could not be written.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum StepError {
    /// A body, or something it references, does not resolve in the model.
    #[error(transparent)]
    NotFound(#[from] NotFound),
    /// The body has a structure the AP214 B-Rep subset written here has no
    /// form for.
    #[error("{body}: {what}")]
    Unsupported {
        /// The body.
        body: Body,
        /// What could not be written.
        what: Unsupported,
    },
    /// A solid body's shells could not be read as lumps: they do not nest,
    /// or the nesting could not be decided (`arris_check::lumps`) — a body
    /// the checker does not pass at `Full`.
    #[error("{body}: {source}")]
    Lumps {
        /// The body.
        body: Body,
        /// Why.
        source: LumpError,
    },
    /// A coordinate, parameter, knot, weight or tolerance is not finite,
    /// and Part 21 has no spelling for it.
    #[error("{id}: a non-finite number cannot be written")]
    NonFinite {
        /// The entity or geometry value holding the number.
        id: AnyId,
    },
    /// Nothing to write: `write` was given no bodies, or
    /// [`write_products`] a tree with no root.
    #[error("no bodies to write")]
    NoBodies,
    /// The product tree given to [`write_products`] cannot be written.
    #[error(transparent)]
    Tree(#[from] TreeError),
    /// The caller's poll or budget stopped [`write_products`] (ADR-0030).
    /// The model is read only, so there is nothing to roll back.
    #[error("{0}")]
    Interrupted(Interrupted),
}

impl From<Interrupted> for StepError {
    fn from(stop: Interrupted) -> Self {
        StepError::Interrupted(stop)
    }
}

/// What [`StepError::Unsupported`] could not write.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Unsupported {
    /// Only a `Solid` is written.
    Kind(BodyKind),
    /// A loop whose every coedge is degenerate has no `EDGE_LOOP`.
    DegenerateLoop {
        /// The face.
        face: FaceId,
        /// The loop's index in the face.
        loop_index: usize,
    },
    /// An edge with more than two uses has no `SURFACE_CURVE`, which holds
    /// at most two pcurves.
    EdgeUses {
        /// The edge.
        edge: EdgeId,
        /// Its uses within the bodies written.
        uses: usize,
    },
}

impl core::fmt::Display for Unsupported {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Unsupported::Kind(kind) => write!(f, "a {kind} body has no STEP form yet"),
            Unsupported::DegenerateLoop { face, loop_index } => write!(
                f,
                "loop {loop_index} of {face} has only degenerate edges, which STEP cannot hold"
            ),
            Unsupported::EdgeUses { edge, uses } => {
                write!(
                    f,
                    "{edge} has {uses} uses; a SURFACE_CURVE holds at most two"
                )
            }
        }
    }
}

/// The STEP AP214 Part 21 text of `bodies` in the order given, as one
/// product whose shape representation lists one solid entity per lump of
/// each body, bodies in order and each body's lumps in `lumps`' order — a
/// `MANIFOLD_SOLID_BREP`, or a `BREP_WITH_VOIDS` for a lump with voids
/// (module docs for the entity subset and the conventions). Errors:
/// [`StepError::NoBodies`] for an empty slice; [`StepError::Unsupported`]
/// for a body that is not a solid, a loop of degenerate edges only, or an
/// edge with more than two uses; [`StepError::Lumps`] for a solid whose
/// shells do not nest into lumps; [`StepError::NotFound`] for a reference
/// that does not resolve; [`StepError::NonFinite`] for a number Part 21
/// cannot spell. The model is read only.
///
/// ```
/// use arris_debug::sample;
/// use arris_topo::Model;
/// use arris_io::step;
///
/// let mut m = Model::default();
/// let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
/// let text = step::write(&m, &[body]).unwrap();
/// assert!(text.starts_with("ISO-10303-21;\n"));
/// assert_eq!(text.matches("SEAM_CURVE(").count(), 1, "the seam edge, once");
/// assert_eq!(text.matches("ADVANCED_FACE(").count(), 3);
/// assert_eq!(step::write(&m, &[body]).unwrap(), text, "deterministic");
/// ```
pub fn write(model: &Model, bodies: &[Body]) -> Result<String, StepError> {
    if bodies.is_empty() {
        return Err(StepError::NoBodies);
    }
    let mut faces_written = BTreeSet::new();
    for &body in bodies {
        faces_written.extend(model.closure(body)?.faces);
    }
    let mut w = Writer::new(model, faces_written, bodies[0].id.into(), "arris")?;
    let mut solids = Vec::with_capacity(bodies.len());
    for &body in bodies {
        solids.extend(w.solids(body)?);
    }
    let representation = w.shape_representation;
    let text = format!(
        "ADVANCED_BREP_SHAPE_REPRESENTATION('',({}),#{})",
        refs(std::iter::once(w.world_placement).chain(solids)),
        w.context_3d
    );
    w.set(representation, text);
    Ok(w.finish())
}

/// The entity accumulator: `entities[i]` is `#i+1`. A number is reserved
/// before its children are written so a parent precedes them in the file,
/// and the memo maps give every geometry value, vertex and edge one entity.
struct Writer<'m> {
    model: &'m Model,
    /// The faces of every body being written: the uses of an edge outside
    /// them contribute no pcurve.
    faces_written: BTreeSet<FaceId>,
    entities: Vec<String>,
    context_3d: usize,
    context_2d: usize,
    world_placement: usize,
    shape_representation: usize,
    /// The `PRODUCT_CONTEXT` and `PRODUCT_DEFINITION_CONTEXT` every product
    /// shares, and the first product's `PRODUCT_DEFINITION`.
    product_context: usize,
    definition_context: usize,
    first_definition: usize,
    /// The `ADVANCED_FACE` each face was written as, for the styled items
    /// that colour it.
    face_entities: BTreeMap<FaceId, usize>,
    surfaces: BTreeMap<SurfaceId, usize>,
    curves: BTreeMap<CurveId, usize>,
    pcurves: BTreeMap<(Curve2Id, SurfaceId), usize>,
    vertices: BTreeMap<VertexId, usize>,
    edges: BTreeMap<EdgeId, usize>,
}

/// A Part 21 string's content: an apostrophe doubled, a backslash
/// doubled, and anything outside printable ASCII as an `\X4\` run, which
/// the reader decodes back to the same text.
fn string(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        match c {
            '\'' => out.push_str("''"),
            '\\' => out.push_str("\\\\"),
            ' '..='~' => out.push(c),
            c => {
                let _ = write!(out, "\\X4\\{:08X}\\X0\\", u32::from(c));
            }
        }
    }
    out
}

/// `#a,#b,#c`.
fn refs(numbers: impl IntoIterator<Item = usize>) -> String {
    numbers
        .into_iter()
        .map(|n| format!("#{n}"))
        .collect::<Vec<_>>()
        .join(",")
}

/// `.T.` or `.F.`.
fn flag(b: bool) -> &'static str {
    if b { ".T." } else { ".F." }
}

/// A finite real in Part 21 spelling: Rust's shortest round-trip decimal
/// with a decimal point always present and no negative zero, so the
/// reader parses back the same `f64`. Errors: `x` is not finite.
fn real(x: f64, owner: AnyId) -> Result<String, StepError> {
    if !x.is_finite() {
        return Err(StepError::NonFinite { id: owner });
    }
    let x = if x == 0.0 { 0.0 } else { x };
    let s = format!("{x}");
    Ok(if s.contains('.') { s } else { format!("{s}.") })
}

/// `(x,y,z)` or `(x,y)` of finite reals.
fn reals(xs: impl IntoIterator<Item = f64>, owner: AnyId) -> Result<String, StepError> {
    let parts: Vec<String> = xs
        .into_iter()
        .map(|x| real(x, owner))
        .collect::<Result<_, _>>()?;
    Ok(format!("({})", parts.join(",")))
}

/// Distinct knots and their multiplicities, in order, as the two lists a
/// `B_SPLINE_*_WITH_KNOTS` takes.
fn knot_lists(knots: &[f64], owner: AnyId) -> Result<(String, String), StepError> {
    let mut distinct: Vec<f64> = Vec::new();
    let mut multiplicity: Vec<usize> = Vec::new();
    for &k in knots {
        match distinct.last() {
            Some(&last) if last == k => {
                if let Some(m) = multiplicity.last_mut() {
                    *m += 1;
                }
            }
            _ => {
                distinct.push(k);
                multiplicity.push(1);
            }
        }
    }
    let mults = format!(
        "({})",
        multiplicity
            .iter()
            .map(|m| m.to_string())
            .collect::<Vec<_>>()
            .join(",")
    );
    Ok((mults, reals(distinct, owner)?))
}

impl<'m> Writer<'m> {
    /// The product structure, the units, the two contexts and the world
    /// placement: everything before the first body. `owner` is what a
    /// non-finite number here would be reported against; the model's
    /// `Precision` is finite by construction, so it never is.
    fn new(
        model: &'m Model,
        faces_written: BTreeSet<FaceId>,
        owner: AnyId,
        name: &str,
    ) -> Result<Self, StepError> {
        let mut w = Writer {
            model,
            faces_written,
            entities: Vec::new(),
            context_3d: 0,
            context_2d: 0,
            world_placement: 0,
            shape_representation: 0,
            product_context: 0,
            definition_context: 0,
            first_definition: 0,
            face_entities: BTreeMap::new(),
            surfaces: BTreeMap::new(),
            curves: BTreeMap::new(),
            pcurves: BTreeMap::new(),
            vertices: BTreeMap::new(),
            edges: BTreeMap::new(),
        };
        let application = w.push(
            "APPLICATION_CONTEXT('core data for automotive mechanical design processes')"
                .to_string(),
        );
        w.push(format!(
            "APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{application})"
        ));
        let product_context = w.push(format!("PRODUCT_CONTEXT('',#{application},'mechanical')"));
        let name = string(name);
        let product = w.push(format!(
            "PRODUCT('{name}','{name}','',(#{product_context}))"
        ));
        let formation = w.push(format!("PRODUCT_DEFINITION_FORMATION('','',#{product})"));
        let definition_context = w.push(format!(
            "PRODUCT_DEFINITION_CONTEXT('part definition',#{application},'design')"
        ));
        let definition = w.push(format!(
            "PRODUCT_DEFINITION('design','',#{formation},#{definition_context})"
        ));
        let definition_shape = w.push(format!("PRODUCT_DEFINITION_SHAPE('','',#{definition})"));
        w.push(format!(
            "PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#{product}))"
        ));
        let length = w.push("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )".to_string());
        let angle = w.push("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )".to_string());
        let solid_angle =
            w.push("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )".to_string());
        let uncertainty = w.push(format!(
            "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{length},'distance_accuracy_value','confusion accuracy')",
            real(model.precision().default_tolerance, owner)?
        ));
        w.context_3d = w.push(format!(
            "( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{uncertainty})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{length},#{angle},#{solid_angle})) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') )"
        ));
        w.context_2d = w.push(
            "( GEOMETRIC_REPRESENTATION_CONTEXT(2) PARAMETRIC_REPRESENTATION_CONTEXT() REPRESENTATION_CONTEXT('2D SPACE','') )"
                .to_string(),
        );
        w.world_placement = w.placement_3d(&Frame::world(), owner)?;
        w.shape_representation = w.reserve();
        w.push(format!(
            "SHAPE_DEFINITION_REPRESENTATION(#{definition_shape},#{})",
            w.shape_representation
        ));
        w.product_context = product_context;
        w.definition_context = definition_context;
        w.first_definition = definition;
        Ok(w)
    }

    /// Appends an entity and returns its number.
    fn push(&mut self, text: String) -> usize {
        self.entities.push(text);
        self.entities.len()
    }

    /// Reserves a number for an entity written by [`Writer::set`] once its
    /// children exist.
    fn reserve(&mut self) -> usize {
        self.push(String::new())
    }

    fn set(&mut self, number: usize, text: String) {
        self.entities[number - 1] = text;
    }

    /// The whole file.
    fn finish(self) -> String {
        let mut out = String::new();
        out.push_str("ISO-10303-21;\nHEADER;\n");
        out.push_str("FILE_DESCRIPTION(('Arris B-Rep'),'2;1');\n");
        out.push_str("FILE_NAME('','',(''),(''),'Arris','Arris','');\n");
        out.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n");
        out.push_str("ENDSEC;\nDATA;\n");
        for (i, entity) in self.entities.iter().enumerate() {
            let _ = writeln!(out, "#{} = {entity};", i + 1);
        }
        out.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
        out
    }

    /// A solid body as one solid entity per lump: a `MANIFOLD_SOLID_BREP`
    /// for a lump without voids, a `BREP_WITH_VOIDS` for one with them.
    fn solids(&mut self, body: Body) -> Result<Vec<usize>, StepError> {
        let entity = self.model.body(body.id)?;
        if entity.kind() != BodyKind::Solid {
            return Err(StepError::Unsupported {
                body,
                what: Unsupported::Kind(entity.kind()),
            });
        }
        let found = lumps(self.model, body).map_err(|source| match source {
            LumpError::NotFound(e) => StepError::NotFound(e),
            source => StepError::Lumps { body, source },
        })?;
        let mut solids = Vec::with_capacity(found.len());
        for lump in found {
            let solid = self.reserve();
            let outer = self.closed_shell(body, lump.outer, false)?;
            if lump.voids.is_empty() {
                self.set(solid, format!("MANIFOLD_SOLID_BREP('',#{outer})"));
            } else {
                let mut voids = Vec::with_capacity(lump.voids.len());
                for void in lump.voids {
                    let oriented = self.reserve();
                    let shell = self.closed_shell(body, void, true)?;
                    self.set(
                        oriented,
                        format!("ORIENTED_CLOSED_SHELL('',*,#{shell},.F.)"),
                    );
                    voids.push(oriented);
                }
                self.set(
                    solid,
                    format!("BREP_WITH_VOIDS('',#{outer},({}))", refs(voids)),
                );
            }
            solids.push(solid);
        }
        Ok(solids)
    }

    /// A shell use as a `CLOSED_SHELL` of its face uses seen through it,
    /// every face turned when `turned`: a void's, which STEP holds
    /// reversed under an `ORIENTED_CLOSED_SHELL` of orientation false.
    fn closed_shell(&mut self, body: Body, shell: Shell, turned: bool) -> Result<usize, StepError> {
        let model = self.model;
        let number = self.reserve();
        let entity = model.shell(shell.id)?;
        let through = if turned {
            shell.orientation.flipped()
        } else {
            shell.orientation
        };
        let mut faces = Vec::with_capacity(entity.faces().len());
        for face_use in entity.faces() {
            let face = face_use.oriented_by(through);
            faces.push(self.face(body, face.id, face.orientation)?);
        }
        self.set(number, format!("CLOSED_SHELL('',({}))", refs(faces)));
        Ok(number)
    }

    /// A face use as an `ADVANCED_FACE`.
    fn face(
        &mut self,
        body: Body,
        id: FaceId,
        orientation: Orientation,
    ) -> Result<usize, StepError> {
        let face = self.model.face(id)?.clone();
        let number = self.reserve();
        self.face_entities.entry(id).or_insert(number);
        let surface = self.surface(face.surface())?;
        let same_sense = orientation == Orientation::Forward;
        let mut bounds = Vec::with_capacity(face.loops().len());
        let areas: Vec<f64> = face
            .loops()
            .iter()
            .map(|l| self.loop_area(l))
            .collect::<Result<_, _>>()?;
        // The outer loop is the one of positive winding; with several (a
        // domain of several components, or an invalid face) the first
        // positive one is named outer and the rest are plain bounds,
        // which a reader treats the same way.
        let outer = areas.iter().position(|&a| a > 0.0);
        for (loop_index, l) in face.loops().iter().enumerate() {
            let bound = self.reserve();
            let mut oriented = Vec::with_capacity(l.coedges().len());
            for coedge in l.coedges() {
                let Some((curve, _)) = self.model.edge(coedge.edge())?.curve() else {
                    continue;
                };
                let edge = self.edge(body, coedge.edge(), curve)?;
                oriented.push(self.push(format!(
                    "ORIENTED_EDGE('',*,*,#{edge},{})",
                    flag(coedge.orientation() == Orientation::Forward)
                )));
            }
            if oriented.is_empty() {
                return Err(StepError::Unsupported {
                    body,
                    what: Unsupported::DegenerateLoop {
                        face: id,
                        loop_index,
                    },
                });
            }
            let edge_loop = self.push(format!("EDGE_LOOP('',({}))", refs(oriented)));
            let kind = if outer == Some(loop_index) {
                "FACE_OUTER_BOUND"
            } else {
                "FACE_BOUND"
            };
            self.set(
                bound,
                format!("{kind}('',#{edge_loop},{})", flag(same_sense)),
            );
            bounds.push(bound);
        }
        self.set(
            number,
            format!(
                "ADVANCED_FACE('',({}),#{surface},{})",
                refs(bounds),
                flag(same_sense)
            ),
        );
        Ok(number)
    }

    /// The signed area of a loop in (u, v) through `region2`, its pieces
    /// sampled at the toolkit's minimum counts: only the sign is used, and
    /// the minimum counts resolve the turn of any loop the checker accepts.
    /// Degenerate coedges contribute nothing.
    fn loop_area(&self, l: &Loop) -> Result<f64, StepError> {
        let mut pcurves = Vec::with_capacity(l.coedges().len());
        for coedge in l.coedges() {
            let edge = self.model.edge(coedge.edge())?;
            let Some((_, range)) = edge.curve() else {
                continue;
            };
            let pcurve = self.model.curve2(coedge.pcurve())?;
            pcurves.push((pcurve, range, coedge.orientation() == Orientation::Reversed));
        }
        let pieces: Vec<Piece<'_>> = pcurves
            .iter()
            .map(|&(curve, range, reversed)| Piece {
                curve,
                range,
                reversed,
            })
            .collect();
        Ok(discretise(&pieces, f64::INFINITY).signed_area())
    }

    /// The `EDGE_CURVE` of a non-degenerate edge, once: its vertices, its
    /// curve `curve_id` and one pcurve per use within the bodies written,
    /// as a `SEAM_CURVE` when both uses are in one loop.
    fn edge(&mut self, body: Body, id: EdgeId, curve_id: CurveId) -> Result<usize, StepError> {
        if let Some(&n) = self.edges.get(&id) {
            return Ok(n);
        }
        let edge = *self.model.edge(id)?;
        let number = self.reserve();
        let start = self.vertex(edge.start())?;
        let end = self.vertex(edge.end())?;
        let mut uses: Vec<(CoedgeRef, Coedge, SurfaceId)> = Vec::new();
        for &u in self.model.edge_uses(id)?.iter() {
            if !self.faces_written.contains(&u.face) {
                continue;
            }
            let face = self.model.face(u.face)?;
            let Some(coedge) = face
                .loops()
                .get(u.loop_index)
                .and_then(|l| l.coedges().get(u.coedge_index))
                .copied()
            else {
                continue;
            };
            uses.push((u, coedge, face.surface()));
        }
        if uses.len() > 2 {
            return Err(StepError::Unsupported {
                body,
                what: Unsupported::EdgeUses {
                    edge: id,
                    uses: uses.len(),
                },
            });
        }
        let curve = self.edge_curve(curve_id, edge.curve().map(|(_, range)| range))?;
        let geometry = if uses.is_empty() {
            curve
        } else {
            let seam = uses.len() == 2
                && uses[0].0.face == uses[1].0.face
                && uses[0].0.loop_index == uses[1].0.loop_index;
            let mut ordered = uses.clone();
            if seam {
                // The forward use's pcurve first (the reference tree's
                // reader picks the forward one by geometry anyway).
                ordered.sort_by_key(|(_, c, _)| c.orientation() == Orientation::Reversed);
            }
            let mut pcurves = Vec::with_capacity(ordered.len());
            for (_, coedge, surface) in &ordered {
                pcurves.push(self.pcurve(coedge.pcurve(), *surface)?);
            }
            let kind = if seam { "SEAM_CURVE" } else { "SURFACE_CURVE" };
            self.push(format!(
                "{kind}('',#{curve},({}),.PCURVE_S1.)",
                refs(pcurves)
            ))
        };
        self.set(
            number,
            format!("EDGE_CURVE('',#{start},#{end},#{geometry},.T.)"),
        );
        self.edges.insert(id, number);
        Ok(number)
    }

    /// A `VERTEX_POINT`, once per vertex.
    fn vertex(&mut self, id: VertexId) -> Result<usize, StepError> {
        if let Some(&n) = self.vertices.get(&id) {
            return Ok(n);
        }
        let vertex = *self.model.vertex(id)?;
        let number = self.reserve();
        let point = self.point_3d(vertex.point(), id.into())?;
        self.set(number, format!("VERTEX_POINT('',#{point})"));
        self.vertices.insert(id, number);
        Ok(number)
    }

    /// A `PCURVE` on `surface`, once per (pcurve, surface).
    fn pcurve(&mut self, id: Curve2Id, surface: SurfaceId) -> Result<usize, StepError> {
        if let Some(&n) = self.pcurves.get(&(id, surface)) {
            return Ok(n);
        }
        let pcurve = self.model.curve2(id)?.clone();
        let surface_number = self.surface(surface)?;
        let number = self.reserve();
        let representation = self.reserve();
        let curve = self.curve_2d(&pcurve, id.into())?;
        self.set(
            representation,
            format!(
                "DEFINITIONAL_REPRESENTATION('',(#{curve}),#{})",
                self.context_2d
            ),
        );
        self.set(
            number,
            format!("PCURVE('',#{surface_number},#{representation})"),
        );
        self.pcurves.insert((id, surface), number);
        Ok(number)
    }

    /// The surface entity, once per `SurfaceId`.
    fn surface(&mut self, id: SurfaceId) -> Result<usize, StepError> {
        if let Some(&n) = self.surfaces.get(&id) {
            return Ok(n);
        }
        let surface = self.model.surface(id)?.clone();
        let owner: AnyId = id.into();
        let number = self.reserve();
        let text = match &surface {
            Surface::Plane { frame } => {
                let placement = self.placement_3d(frame, owner)?;
                format!("PLANE('',#{placement})")
            }
            Surface::Cylinder { frame, radius } => {
                let placement = self.placement_3d(frame, owner)?;
                format!(
                    "CYLINDRICAL_SURFACE('',#{placement},{})",
                    real(*radius, owner)?
                )
            }
            Surface::Cone {
                frame,
                radius,
                half_angle,
            } => {
                let placement = self.placement_3d(frame, owner)?;
                format!(
                    "CONICAL_SURFACE('',#{placement},{},{})",
                    real(*radius, owner)?,
                    real(*half_angle, owner)?
                )
            }
            Surface::Sphere { frame, radius } => {
                let placement = self.placement_3d(frame, owner)?;
                format!(
                    "SPHERICAL_SURFACE('',#{placement},{})",
                    real(*radius, owner)?
                )
            }
            Surface::Torus {
                frame,
                major_radius,
                minor_radius,
            } => {
                let placement = self.placement_3d(frame, owner)?;
                format!(
                    "TOROIDAL_SURFACE('',#{placement},{},{})",
                    real(*major_radius, owner)?,
                    real(*minor_radius, owner)?
                )
            }
            Surface::EllipticCylinder {
                frame,
                major_radius,
                minor_radius,
            } => {
                // STEP has no elementary elliptic cylinder: the section
                // ellipse extruded along the axis, which the reference
                // tree reads back with the same (u, v) (ADR-0014).
                let placement = self.placement_3d(frame, owner)?;
                let ellipse = self.push(format!(
                    "ELLIPSE('',#{placement},{},{})",
                    real(*major_radius, owner)?,
                    real(*minor_radius, owner)?
                ));
                let z = self.direction_3d(frame.z().into_inner(), owner)?;
                let axis = self.push(format!("VECTOR('',#{z},1.)"));
                format!("SURFACE_OF_LINEAR_EXTRUSION('',#{ellipse},#{axis})")
            }
            Surface::Nurbs(n) => self.nurbs_surface(n, owner)?,
        };
        self.set(number, text);
        self.surfaces.insert(id, number);
        Ok(number)
    }

    /// The 3D curve entity an edge over `range` of curve `id` is written
    /// on: the curve's own, shared by every edge on it — but for a
    /// periodic NURBS, the edge's own piece over its range, clamped
    /// (`NurbsCurve::segment`), in the same parameter as its pcurves.
    /// STEP has no periodic B-spline a reader keeps, and an `EDGE_CURVE`
    /// carries no range: a reader finds it by projecting the vertices,
    /// and on a closed curve an edge running past the knots' end — the
    /// wrap-around block of a traced section loop — projects onto the
    /// other side of its vertices.
    fn edge_curve(&mut self, id: CurveId, range: Option<Interval>) -> Result<usize, StepError> {
        let curve = self.model.curve(id)?;
        let piece = match (curve, range) {
            (Curve::Nurbs(n), Some(range)) if n.period().is_some() => {
                // A range longer than a period is the checker's E1; the
                // body the writer is given passes it.
                n.segment(range).ok()
            }
            _ => None,
        };
        let Some(piece) = piece else {
            return self.curve(id);
        };
        let owner: AnyId = id.into();
        let number = self.reserve();
        let text = self.nurbs_curve(&piece, owner)?;
        self.set(number, text);
        Ok(number)
    }

    /// The 3D curve entity, once per `CurveId`.
    fn curve(&mut self, id: CurveId) -> Result<usize, StepError> {
        if let Some(&n) = self.curves.get(&id) {
            return Ok(n);
        }
        let curve = self.model.curve(id)?.clone();
        let owner: AnyId = id.into();
        let number = self.reserve();
        let text = match &curve {
            Curve::Line { origin, direction } => {
                let point = self.point_3d(*origin, owner)?;
                let dir = self.direction_3d(direction.into_inner(), owner)?;
                let vector = self.push(format!("VECTOR('',#{dir},1.)"));
                format!("LINE('',#{point},#{vector})")
            }
            Curve::Circle { frame, radius } => {
                let placement = self.placement_3d(frame, owner)?;
                format!("CIRCLE('',#{placement},{})", real(*radius, owner)?)
            }
            Curve::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => {
                let placement = self.placement_3d(frame, owner)?;
                format!(
                    "ELLIPSE('',#{placement},{},{})",
                    real(*major_radius, owner)?,
                    real(*minor_radius, owner)?
                )
            }
            Curve::Nurbs(n) => self.nurbs_curve(n, owner)?,
        };
        self.set(number, text);
        self.curves.insert(id, number);
        Ok(number)
    }

    /// A pcurve's 2D curve entity.
    fn curve_2d(&mut self, pcurve: &Curve2, owner: AnyId) -> Result<usize, StepError> {
        let number = self.reserve();
        let text = match pcurve {
            Curve2::Line { origin, direction } => {
                let point = self.point_2d(*origin, owner)?;
                let dir = self.direction_2d(direction.into_inner(), owner)?;
                let vector = self.push(format!("VECTOR('',#{dir},1.)"));
                format!("LINE('',#{point},#{vector})")
            }
            Curve2::Circle { frame, radius } => {
                let placement = self.placement_2d(frame, owner)?;
                format!("CIRCLE('',#{placement},{})", real(*radius, owner)?)
            }
            Curve2::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => {
                let placement = self.placement_2d(frame, owner)?;
                format!(
                    "ELLIPSE('',#{placement},{},{})",
                    real(*major_radius, owner)?,
                    real(*minor_radius, owner)?
                )
            }
            Curve2::Nurbs(n) => self.nurbs_curve_2d(n, owner)?,
        };
        self.set(number, text);
        Ok(number)
    }

    fn point_3d(&mut self, p: Point3, owner: AnyId) -> Result<usize, StepError> {
        let text = format!("CARTESIAN_POINT('',{})", reals([p.x, p.y, p.z], owner)?);
        Ok(self.push(text))
    }

    fn direction_3d(&mut self, d: Vec3, owner: AnyId) -> Result<usize, StepError> {
        let text = format!("DIRECTION('',{})", reals([d.x, d.y, d.z], owner)?);
        Ok(self.push(text))
    }

    fn point_2d(&mut self, p: Point2, owner: AnyId) -> Result<usize, StepError> {
        let text = format!("CARTESIAN_POINT('',{})", reals([p.x, p.y], owner)?);
        Ok(self.push(text))
    }

    fn direction_2d(&mut self, d: Vec2, owner: AnyId) -> Result<usize, StepError> {
        let text = format!("DIRECTION('',{})", reals([d.x, d.y], owner)?);
        Ok(self.push(text))
    }

    /// `AXIS2_PLACEMENT_3D(origin, Z, X)`: the frame as `gp_Ax3` takes it,
    /// so the surface's parametrisation survives the reader.
    fn placement_3d(&mut self, frame: &Frame, owner: AnyId) -> Result<usize, StepError> {
        let number = self.reserve();
        let origin = self.point_3d(frame.origin(), owner)?;
        let z = self.direction_3d(frame.z().into_inner(), owner)?;
        let x = self.direction_3d(frame.x().into_inner(), owner)?;
        self.set(
            number,
            format!("AXIS2_PLACEMENT_3D('',#{origin},#{z},#{x})"),
        );
        Ok(number)
    }

    /// `AXIS2_PLACEMENT_2D(origin, X)`: always direct, so a left-handed
    /// frame's sense is not carried (module docs).
    fn placement_2d(&mut self, frame: &Frame2, owner: AnyId) -> Result<usize, StepError> {
        let number = self.reserve();
        let origin = self.point_2d(frame.origin(), owner)?;
        let x = self.direction_2d(frame.x().into_inner(), owner)?;
        self.set(number, format!("AXIS2_PLACEMENT_2D('',#{origin},#{x})"));
        Ok(number)
    }

    /// `B_SPLINE_CURVE_WITH_KNOTS`, or the rational complex entity when a
    /// weight is not one. The points are written after the curve's
    /// reserved number, so the text refers to numbers already allocated.
    fn nurbs_curve(&mut self, n: &NurbsCurve, owner: AnyId) -> Result<String, StepError> {
        let mut points = Vec::with_capacity(n.control_points().len());
        for &p in n.control_points() {
            points.push(self.point_3d(p, owner)?);
        }
        bspline_curve_text(n.degree(), &points, n.knots(), n.weights(), owner)
    }

    fn nurbs_curve_2d(&mut self, n: &NurbsCurve2, owner: AnyId) -> Result<String, StepError> {
        let mut points = Vec::with_capacity(n.control_points().len());
        for &p in n.control_points() {
            points.push(self.point_2d(p, owner)?);
        }
        bspline_curve_text(n.degree(), &points, n.knots(), n.weights(), owner)
    }

    /// `B_SPLINE_SURFACE_WITH_KNOTS`, or the rational complex entity. The
    /// control net is written as a list of `u` rows.
    fn nurbs_surface(&mut self, n: &NurbsSurface, owner: AnyId) -> Result<String, StepError> {
        let [p, q] = n.degree();
        let [cu, cv] = n.counts();
        let mut rows = Vec::with_capacity(cu);
        let mut weight_rows = Vec::with_capacity(cu);
        // The net is row-major over `u` and the constructor sized it to
        // `cu × cv`, so each chunk is one `u` row.
        for (i, weights) in n.weights().chunks(cv).enumerate() {
            let mut row = Vec::with_capacity(cv);
            for j in 0..weights.len() {
                let point = n
                    .control_point(i, j)
                    .ok_or_else(|| StepError::NotFound(NotFound::new(owner)))?;
                row.push(self.point_3d(point, owner)?);
            }
            rows.push(format!("({})", refs(row)));
            weight_rows.push(reals(weights.iter().copied(), owner)?);
        }
        let net = format!("({})", rows.join(","));
        let [ku, kv] = n.knots();
        let (mu, u_knots) = knot_lists(ku, owner)?;
        let (mv, v_knots) = knot_lists(kv, owner)?;
        let rational = n.weights().iter().any(|&w| w != 1.0);
        Ok(if rational {
            format!(
                "( BOUNDED_SURFACE() B_SPLINE_SURFACE({p},{q},{net},.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS({mu},{mv},{u_knots},{v_knots},.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(({})) REPRESENTATION_ITEM('') SURFACE() )",
                weight_rows.join(",")
            )
        } else {
            format!(
                "B_SPLINE_SURFACE_WITH_KNOTS('',{p},{q},{net},.UNSPECIFIED.,.F.,.F.,.F.,{mu},{mv},{u_knots},{v_knots},.UNSPECIFIED.)"
            )
        })
    }
}

/// The text of a B-spline curve entity over already-written control
/// points, 2D or 3D alike.
fn bspline_curve_text(
    degree: usize,
    points: &[usize],
    knots: &[f64],
    weights: &[f64],
    owner: AnyId,
) -> Result<String, StepError> {
    let list = format!("({})", refs(points.iter().copied()));
    let (mults, knot_values) = knot_lists(knots, owner)?;
    let rational = weights.iter().any(|&w| w != 1.0);
    Ok(if rational {
        format!(
            "( BOUNDED_CURVE() B_SPLINE_CURVE({degree},{list},.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS({mults},{knot_values},.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE({}) REPRESENTATION_ITEM('') )",
            reals(weights.iter().copied(), owner)?
        )
    } else {
        format!(
            "B_SPLINE_CURVE_WITH_KNOTS('',{degree},{list},.UNSPECIFIED.,.F.,.F.,{mults},{knot_values},.UNSPECIFIED.)"
        )
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn owner() -> AnyId {
        AnyId::Geometry(CurveId::new(0, 0).into())
    }

    #[test]
    fn reals_are_shortest_round_trip_with_a_point() {
        assert_eq!(real(40.0, owner()).unwrap(), "40.");
        assert_eq!(real(0.5, owner()).unwrap(), "0.5");
        assert_eq!(real(-0.0, owner()).unwrap(), "0.");
        assert_eq!(real(1e-7, owner()).unwrap(), "0.0000001");
        assert_eq!(
            real(core::f64::consts::TAU, owner()).unwrap(),
            "6.283185307179586"
        );
        assert!(matches!(
            real(f64::NAN, owner()),
            Err(StepError::NonFinite { .. })
        ));
        assert_eq!(reals([1.0, 2.5], owner()).unwrap(), "(1.,2.5)");
    }

    #[test]
    fn knots_split_into_distinct_values_and_multiplicities() {
        let (m, k) = knot_lists(&[0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 3.0, 3.0, 3.0], owner()).unwrap();
        assert_eq!(m, "(3,1,2,3)");
        assert_eq!(k, "(0.,1.,2.,3.)");
    }
}
