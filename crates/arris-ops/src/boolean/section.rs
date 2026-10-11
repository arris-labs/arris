//! A solid's planar section as a sheet (ADR-0056): the cap [`split`]
//! makes on its positive side, run on a scratch copy of the model and
//! imported alone, so the body and every entity of the model are left as
//! they were.
//!
//! [`split`]: super::split

use std::collections::{BTreeMap, BTreeSet};

use arris_check::classify::{Classification, Classifier};
use arris_check::domain::FaceDomain;
use arris_geom::integrate::{region_integral, surface_grid};
use arris_geom::region2::{Polygon2, interior_point};
use arris_math::{Control, Frame, Meter, Point3};
use arris_topo::builder::{Assembly, Builder, FaceSpec};
use arris_topo::entity::BodyKind;
use arris_topo::{
    Body, EntityId, FaceId, Model, Orientation, Origin, Provenance, Shape, TopoError,
};

use super::split::half_space;
use super::{pave, result};
use crate::error::{BooleanReason, Fault, InputReason, OpError, Reason};

/// What [`section`] returns: the sheet the plane cuts out of a solid, its
/// area and centroid, and the record of where each of its entities lies
/// on the solid.
#[derive(Debug, Clone, PartialEq)]
pub struct Section {
    /// A `BodyKind::Sheet` body of one open shell per connected region of
    /// the section, each shell one planar face whose outer loop and holes
    /// are the section's wires. Every face lies on a surface equal to the
    /// plane itself, so its (u, v) are the frame's `x` and `y`, and faces
    /// the frame's `z`.
    pub sheet: Body,
    /// The total area of the sheet's faces.
    pub area: f64,
    /// The area centroid of the sheet's faces, on the plane.
    pub centroid: Point3,
    /// Where the sheet came from, against the solid: every face, shell and
    /// the sheet itself `Generated` from the solid; a section edge from the
    /// solid's faces it lies on; a section vertex from the solid's vertex
    /// it is, else the solid's edges it lies on, else the faces.
    pub provenance: Provenance,
}

/// The section of the solid `body` by the plane of `plane`: the plane's
/// points interior to the solid, as a sheet body (`docs/ARCHITECTURE.md`
/// §Operations, ADR-0056).
///
/// Guarantees. The sheet is the cap [`split`](super::split) at the same
/// plane makes on its positive side, from the same decomposition — the
/// General Fuse of ADR-0004 over the body and the half-space beyond the
/// plane — reversed to face the frame's `z`, less any region where a face
/// of the body lies on the plane: there the material is on one side only,
/// and the sheet holds the points with material on both. Where nothing is
/// flush the two agree to the last digit, and every way the plane can
/// meet the body that the booleans settle (through an edge, a vertex, a
/// seam, an apex) is settled here as there. It is a `Sheet` that passes the
/// checker, one open shell per face; `area` and `centroid` are integrated
/// exactly over each face's (u, v) region, the frame's own coordinates.
/// The body and every entity of the model are untouched: the sheet's
/// closure is all the call adds, and its ids depend only on the input.
/// The record is described at [`Section::provenance`].
///
/// `Ok(None)` when the plane misses the body or only touches it — along a
/// face, an edge or a vertex — so there is no interior to cut: a plane
/// within the tolerance of a face is flush with it, as for `split`.
///
/// Errors, the model untouched on each: [`OpError::InvalidInput`] and
/// [`OpError::NotFound`] as every operation; [`OpError::Degenerate`] with
/// [`InputReason::NotSolid`] for a sheet, a shell or a wire body; the
/// booleans' refusals where the plane meets the body as they refuse —
/// `Unsupported` for a pair with no closed form or a NURBS face,
/// [`BooleanReason::TangentContact`] for a plane tangent to a face along
/// a curve where it crosses the body elsewhere; [`OpError::Tolerance`],
/// [`OpError::Internal`] and [`OpError::Interrupted`] as theirs.
///
/// ```
/// use arris_ops::{primitive_box, section};
/// use arris_topo::Model;
/// use arris_topo::entity::BodyKind;
/// use arris_math::{Control, Frame, Point3, Vec3};
///
/// let mut m = Model::default();
/// let none = Control::NONE;
/// let (block, _) = primitive_box(&mut m, Point3::origin(), Point3::new(10.0, 6.0, 4.0), &none)?;
/// let plane = Frame::from_z(Point3::new(0.0, 0.0, 1.5), Vec3::z())?;
/// let cut = section(&mut m, block, &plane, &none)?.expect("the plane crosses the block");
/// assert_eq!(m.body(cut.sheet.id)?.kind(), BodyKind::Sheet);
/// assert_eq!(m.faces(cut.sheet)?.len(), 1);
/// assert!((cut.area - 60.0).abs() < 1e-9);
/// assert!((cut.centroid - Point3::new(5.0, 3.0, 1.5)).norm() < 1e-9);
///
/// let above = Frame::from_z(Point3::new(0.0, 0.0, 9.0), Vec3::z())?;
/// assert!(section(&mut m, block, &above, &none)?.is_none());
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn section(
    m: &mut Model,
    body: Body,
    plane: &Frame,
    control: &Control<'_>,
) -> Result<Option<Section>, OpError> {
    crate::verify_input(m, body)?;
    if m.body(body.id)?.kind() != BodyKind::Solid {
        return Err(OpError::Degenerate {
            entities: vec![Shape::new(body.id, body.orientation)],
            reason: Reason::Input(InputReason::NotSolid),
        });
    }
    let Some((lo, hi)) = half_space(m, body, plane)? else {
        return Ok(None);
    };
    let mut meter = Meter::new(control);
    // The decomposition and the cap are made in a copy of the model, as
    // `split` makes them in the model itself, so the ids of the sheet's
    // closure in the copy are `split`'s; only the sheet is imported.
    let mut scratch = m.clone();
    let tool = crate::primitive::plane_box(&mut scratch, plane, lo, hi)?;
    let i = pave::build(&scratch, &[body, tool], &mut meter)?;
    let (common, record) = match result::boolean(&mut scratch, &i, result::Op::Common, &mut meter) {
        Ok(made) => made,
        Err(OpError::Degenerate {
            reason:
                Reason::Boolean(BooleanReason::Empty) | Reason::Input(InputReason::ZeroThickness),
            ..
        }) => return Ok(None),
        Err(e) => return Err(e),
    };

    let tool_entities = entity_set(&scratch, tool)?;
    let body_entities = entity_set(m, body)?;
    // Every output of the common's record, by id, with the body's
    // entities it came from; and the outputs made from the tool.
    let mut from_body: BTreeMap<EntityId, BTreeSet<EntityId>> = BTreeMap::new();
    let mut from_tool: BTreeSet<EntityId> = BTreeSet::new();
    for origin in record.origins_recorded() {
        let Origin::Entity(s) = origin else { continue };
        for y in record
            .generated_from(origin)
            .iter()
            .chain(record.modified_from(origin))
        {
            if tool_entities.contains(&s.id) {
                from_tool.insert(y.id);
            } else if body_entities.contains(&s.id) {
                from_body.entry(y.id).or_default().insert(s.id);
            }
        }
    }

    // The cap: the common's faces made from the half-space's near face,
    // the only one of its faces that reaches the body, less the regions
    // where a face of the body lies on the plane. Where the two normals
    // agree the common keeps the near face's copy of such a region — the
    // half-space's boundary, with material on one side of the plane only
    // — so a cap piece is section only when a point inside it lies inside
    // the body, as the selection table classifies a piece.
    let classifier =
        Classifier::of_body(&scratch, body).map_err(|e| OpError::Internal(Fault::Classify(e)))?;
    let mut caps = Vec::new();
    for f in scratch.faces(common)? {
        if !from_tool.contains(&EntityId::Face(f.id)) {
            continue;
        }
        meter.tick()?;
        let point = interior(&scratch, f.id)?;
        match classifier
            .classify(point)
            .map_err(|e| result::classify_fault(&scratch, f.id, body, e))?
        {
            Classification::Inside => caps.push(f),
            Classification::On(_) => {}
            Classification::Outside => {
                return Err(OpError::Internal(Fault::Invariant {
                    what: "a common's cap piece inside or on the body",
                }));
            }
        }
    }
    if caps.is_empty() {
        // Flush with a face of the body and crossing nowhere else.
        return Ok(None);
    }
    let assembly = Assembly {
        shells: caps
            .iter()
            .map(|&f| vec![FaceSpec::Keep(f.reversed())])
            .collect(),
        ..Assembly::default()
    };
    let tolerance = scratch.precision().default_tolerance;
    let (builder, _) = Builder::assemble_sheet(&scratch, tolerance, assembly)?;
    let built = builder.finish(&mut scratch, BodyKind::Sheet)?;
    let (area, centroid) = measure_caps(&scratch, built.body, &mut meter)?;

    let closure = scratch.closure(built.body)?;
    let (sheet, map) = m.import(&scratch, built.body).map_err(|e| match e {
        TopoError::NotFound(e) => OpError::from(e),
        // The copy has the model's own precision.
        TopoError::Precision(_) => OpError::Internal(Fault::Invariant {
            what: "a scratch copy with the model's precision",
        }),
    })?;
    let forward = |id: EntityId| Shape::new(id, Orientation::Forward);
    let mapped = |id: EntityId| {
        map.map_entity(id)
            .map(forward)
            .ok_or(OpError::Internal(Fault::Invariant {
                what: "an imported sheet entity in the import's map",
            }))
    };
    // Where a common entity lies on the body: what its record names, and
    // itself when it is the body's own, kept by id.
    let sources = |id: EntityId| -> BTreeSet<EntityId> {
        let mut out = from_body.get(&id).cloned().unwrap_or_default();
        if body_entities.contains(&id) {
            out.insert(id);
        }
        out
    };
    let solid = Origin::Entity(Shape::new(body.id, body.orientation));
    let mut provenance = Provenance::new();
    provenance.add_generated(solid, forward(EntityId::Body(sheet.id)));
    for &s in &closure.shells {
        provenance.add_generated(solid, mapped(EntityId::Shell(s))?);
    }
    for &f in &closure.faces {
        provenance.add_generated(solid, mapped(EntityId::Face(f))?);
    }
    for &e in &closure.edges {
        let y = mapped(EntityId::Edge(e))?;
        let mut faces = BTreeSet::new();
        for s in sources(EntityId::Edge(e)) {
            match s {
                EntityId::Face(_) => {
                    faces.insert(s);
                }
                EntityId::Edge(along) => {
                    for u in m.edge_uses(along)? {
                        let face = EntityId::Face(u.face);
                        if body_entities.contains(&face) {
                            faces.insert(face);
                        }
                    }
                }
                EntityId::Vertex(_) | EntityId::Shell(_) | EntityId::Body(_) => {}
            }
        }
        for f in faces {
            provenance.add_generated(forward(f), y);
        }
    }
    for &v in &closure.vertices {
        let y = mapped(EntityId::Vertex(v))?;
        let all = sources(EntityId::Vertex(v));
        // The lowest-dimensional entities of the body it lies on.
        let of = |kind: fn(&EntityId) -> bool| all.iter().copied().filter(kind).collect::<Vec<_>>();
        let vertices = of(|s| matches!(s, EntityId::Vertex(_)));
        let edges = of(|s| matches!(s, EntityId::Edge(_)));
        let faces = of(|s| matches!(s, EntityId::Face(_)));
        let on = [vertices, edges, faces]
            .into_iter()
            .find(|on| !on.is_empty())
            .unwrap_or_default();
        for s in on {
            provenance.add_generated(forward(s), y);
        }
    }
    crate::verify(m, sheet)?;
    Ok(Some(Section {
        sheet,
        area,
        centroid,
        provenance,
    }))
}

/// A point inside `face`, on its surface: the middle of an inside span of
/// its loops' polygons in (u, v), clear of their chords, as the booleans
/// take a piece's interior point.
fn interior(m: &Model, face: FaceId) -> Result<Point3, OpError> {
    let domain = FaceDomain::of(m, face, m.precision().parametric_tolerance)?;
    let polygons = domain.polygons();
    let clearance = polygons
        .iter()
        .map(Polygon2::chord_deviation)
        .fold(0.0, f64::max);
    let uv = interior_point(polygons, clearance).ok_or(OpError::Internal(Fault::Invariant {
        what: "a section face with an interior",
    }))?;
    Ok(domain.surface().eval(uv.x, uv.y).point)
}

/// Every entity of `body`'s closure, by id, the body included.
fn entity_set(m: &Model, body: Body) -> Result<BTreeSet<EntityId>, OpError> {
    let c = m.closure(body)?;
    let mut out: BTreeSet<EntityId> = BTreeSet::new();
    out.extend(c.vertices.iter().map(|&v| EntityId::Vertex(v)));
    out.extend(c.edges.iter().map(|&e| EntityId::Edge(e)));
    out.extend(c.faces.iter().map(|&f| EntityId::Face(f)));
    out.extend(c.shells.iter().map(|&s| EntityId::Shell(s)));
    out.insert(EntityId::Body(body.id));
    Ok(out)
}

/// The total area of the sheet's planar faces and their area centroid:
/// `∬ 1`, `∬ u` and `∬ v` over each face's (u, v) region, the moments
/// taken to the plane by its own affine map, which is exact for a plane.
fn measure_caps(m: &Model, sheet: Body, meter: &mut Meter<'_>) -> Result<(f64, Point3), OpError> {
    let mut area = 0.0;
    let mut moment = arris_math::Vec3::zeros();
    for face in m.faces(sheet)? {
        meter.tick()?;
        let entity = m.face(face.id)?;
        let surface = m.surface(entity.surface())?;
        let grid = surface_grid(surface);
        let (mut a, mut au, mut av) = (0.0, 0.0, 0.0);
        for l in entity.loops() {
            let pieces = m.loop_pieces(l)?;
            a += region_integral(&pieces, &grid, |_, _| 1.0);
            au += region_integral(&pieces, &grid, |u, _| u);
            av += region_integral(&pieces, &grid, |_, v| v);
        }
        if !(a.is_finite() && a > 0.0) {
            return Err(OpError::Internal(Fault::Invariant {
                what: "a section face of positive area",
            }));
        }
        area += a;
        moment += a * surface.eval(au / a, av / a).point.coords;
    }
    Ok((area, Point3::from(moment / area)))
}
