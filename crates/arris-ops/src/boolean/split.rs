//! A body cut by a plane into the solids on either side (ADR-0051): one
//! General Fuse of the body and a scratch box beyond the plane, two
//! selections from it, the box freed afterwards.

use std::collections::BTreeSet;

use arris_math::{Aabb, Control, Frame, Meter, Point3};
use arris_topo::provenance::{PlaneSide, SplitPart};
use arris_topo::{Body, EntityId, Model, Origin, Provenance, Role, Shape};

use super::faces::FaceInfo;
use super::{pave, result};
use crate::error::{BooleanReason, InputReason, OpError, Reason, SplitReason};

/// What [`split`] returns: the solid on each side of the plane and one
/// record of both.
#[derive(Debug, Clone, PartialEq)]
pub struct Split {
    /// The body on the side the plane frame's `z` points to: one solid,
    /// possibly of several lumps, never empty.
    pub positive: Body,
    /// The body on the other side, as `positive`.
    pub negative: Body,
    /// What became of the input body's entities on both sides, positive
    /// side first wherever an origin has outputs on both.
    pub provenance: Provenance,
}

/// `body` cut by the plane of `plane` into the solid on each side of it
/// (`docs/ARCHITECTURE.md` §Operations, ADR-0051).
///
/// Guarantees. `positive` is the body's material on the side the frame's
/// `z` points to, `negative` the rest, and each is a `Solid` that passes
/// the checker; either may be several lumps — a U-bracket split across
/// both arms. Both come from one decomposition, the General Fuse of
/// ADR-0004 over the body and the half-space beyond the plane, so the
/// body's faces are split once and each piece is kept on its side. Each
/// side has a cap face on the plane for every region of the section, one
/// copy per side with opposite orientation, on the surface `plane` itself:
/// a cap's (u, v) are the frame's `x` and `y`. The sides are
/// self-contained, each with its own section edges and vertices, and share
/// no entity. Every entity of the body the plane does not touch keeps its
/// id on its side. The record: the body's faces, edges, shell and the body
/// itself are `Modified` into their pieces on each side, the body into
/// both `positive` and `negative`; a cap face is `Generated` from
/// `Role::Split(SplitPart::Cap(side))`, a section edge from the body face
/// it lies on and that role, a section vertex from the body edge it splits
/// and that role; an entity of the body with no image on either side is
/// `Deleted`. Every origin with outputs on both sides lists the positive
/// side's first (ADR-0009). Nothing of the half-space is left in the
/// model, and the output ids depend only on the input. The cases where the
/// plane runs through the body's own vertices, edges or faces follow the
/// booleans' rulings.
///
/// Errors, the model untouched on each: [`OpError::InvalidInput`] and
/// [`OpError::NotFound`] as every operation; [`OpError::Degenerate`] with
/// [`SplitReason::NoCrossing`] naming the body when the plane misses it
/// or only touches it, so one side would be empty; the booleans' refusals
/// where the plane meets the body as they refuse — `Unsupported` for a
/// pair with no closed form or a NURBS face,
/// [`BooleanReason::TangentContact`] for a plane tangent to a face along
/// a curve where it crosses the body elsewhere; [`OpError::Tolerance`],
/// [`OpError::Internal`] and [`OpError::Interrupted`] as theirs.
///
/// ```
/// use arris_ops::{primitive_box, split};
/// use arris_ops::measure::mass_properties;
/// use arris_topo::Model;
/// use arris_math::{Control, Frame, Point3, Vec3};
///
/// let mut m = Model::default();
/// let none = Control::NONE;
/// let (block, _) = primitive_box(&mut m, Point3::origin(), Point3::new(10.0, 10.0, 10.0), &none)?;
/// let plane = Frame::from_z(Point3::new(0.0, 0.0, 4.0), Vec3::z())?;
/// let halves = split(&mut m, block, &plane, &none)?;
/// let above = mass_properties(&m, halves.positive, &none)?.volume;
/// let below = mass_properties(&m, halves.negative, &none)?.volume;
/// assert!((above - 600.0).abs() < 1e-9 && (below - 400.0).abs() < 1e-9);
/// assert_eq!(m.faces(halves.positive)?.len(), 6);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn split(
    m: &mut Model,
    body: Body,
    plane: &Frame,
    control: &Control<'_>,
) -> Result<Split, OpError> {
    crate::verify_input(m, body)?;
    let no_crossing = || OpError::Degenerate {
        entities: vec![Shape::new(body.id, body.orientation)],
        reason: Reason::Split(SplitReason::NoCrossing),
    };
    let bounds = FaceInfo::of_body(m, body)?
        .iter()
        .map(|f| f.bounds)
        .reduce(Aabb::union)
        .ok_or_else(no_crossing)?;
    // The body's box in the plane's frame: its eight corners there.
    let corners: Vec<[f64; 3]> = (0..8)
        .map(|k| {
            let pick = |axis: usize| {
                if k >> axis & 1 == 0 {
                    bounds.min[axis]
                } else {
                    bounds.max[axis]
                }
            };
            let local = plane.to_local(Point3::new(pick(0), pick(1), pick(2)));
            [local.x, local.y, local.z]
        })
        .collect();
    let local = Aabb::of_points(&corners).ok_or_else(no_crossing)?;
    if !(local.min[2] < 0.0 && local.max[2] > 0.0) {
        return Err(no_crossing());
    }
    // The box beyond the plane: its near face on the plane, its other five
    // past the body's box by the box's own diagonal, so none of them
    // pairs with a face of the body.
    let margin = local.diagonal();
    let lo = Point3::new(local.min[0] - margin, local.min[1] - margin, 0.0);
    let hi = Point3::new(
        local.max[0] + margin,
        local.max[1] + margin,
        local.max[2] + margin,
    );
    let mut meter = Meter::new(control);
    let (scratch, made) = m.transaction(|m| {
        let scratch = crate::primitive::plane_box(m, plane, lo, hi)?;
        let i = pave::build(m, &[body, scratch], &mut meter)?;
        let made = result::boolean_each(m, &i, &[result::Op::Common, result::Op::Cut], &mut meter)
            .map_err(|e| match e {
                OpError::Degenerate {
                    reason:
                        Reason::Boolean(BooleanReason::Empty)
                        | Reason::Input(InputReason::ZeroThickness),
                    ..
                } => no_crossing(),
                e => e,
            })?;
        Ok::<_, OpError>((scratch, made))
    })?;
    let [(positive, p), (negative, n)] =
        <[(Body, Provenance); 2]>::try_from(made).map_err(|_| {
            OpError::Internal(crate::Fault::Invariant {
                what: "two results for two selections",
            })
        })?;
    let closure = m.closure(scratch)?;
    let mut plane_entities: BTreeSet<EntityId> = BTreeSet::new();
    plane_entities.extend(closure.vertices.iter().map(|&v| EntityId::Vertex(v)));
    plane_entities.extend(closure.edges.iter().map(|&e| EntityId::Edge(e)));
    plane_entities.extend(closure.faces.iter().map(|&f| EntityId::Face(f)));
    plane_entities.extend(closure.shells.iter().map(|&s| EntityId::Shell(s)));
    plane_entities.insert(EntityId::Body(scratch.id));
    m.discard(scratch, &[positive, negative])?;
    let provenance = record(
        [(&p, PlaneSide::Positive), (&n, PlaneSide::Negative)],
        &plane_entities,
    );
    Ok(Split {
        positive,
        negative,
        provenance,
    })
}

/// One record of both sides' records, against the body alone: an origin
/// of the scratch box's face, edge or vertex becomes its side's cap role,
/// what the side made from it `Generated` from that role, and the box's
/// shell, body and deletions drop out. Origins with outputs on both
/// sides list the positive side's first; an input is `Deleted` only when
/// neither side holds an image of it.
fn record(sides: [(&Provenance, PlaneSide); 2], plane: &BTreeSet<EntityId>) -> Provenance {
    let mut out = Provenance::new();
    for (p, side) in sides {
        let cap = Origin::Role(Role::Split(SplitPart::Cap(side)));
        for origin in p.origins_recorded() {
            let (to, from_plane) = match origin {
                Origin::Entity(s) if plane.contains(&s.id) => match s.id {
                    EntityId::Face(_) | EntityId::Edge(_) | EntityId::Vertex(_) => (cap, true),
                    EntityId::Shell(_) | EntityId::Body(_) => continue,
                },
                o => (o, false),
            };
            for &y in p.generated_from(origin) {
                out.add_generated(to, y);
            }
            for &y in p.modified_from(origin) {
                if from_plane {
                    out.add_generated(to, y);
                } else {
                    out.add_modified(to, y);
                }
            }
        }
    }
    let [(first, _), (second, _)] = sides;
    for s in first.deleted() {
        if !plane.contains(&s.id) && second.is_deleted(s) {
            out.add_deleted(s);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use arris_math::{Axis, Control, Frame, Point3, Vec3};
    use arris_topo::{BodyId, FaceId, Model, Shape};

    use super::split;
    use crate::error::{OpError, Reason, SplitReason};
    use crate::{primitive_box, primitive_cylinder};

    fn refused_naming(e: OpError, body: arris_topo::Body) {
        match e {
            OpError::Degenerate { entities, reason } => {
                assert_eq!(reason, Reason::Split(SplitReason::NoCrossing));
                assert_eq!(entities, [Shape::new(body.id, body.orientation)]);
            }
            e => panic!("expected NoCrossing, found {e}"),
        }
    }

    /// The scratch box is the body after the input's, its faces the six
    /// after the input's: none of them resolves once the split returns.
    #[test]
    fn nothing_of_the_half_space_is_left() {
        let n = &Control::NONE;
        let mut m = Model::default();
        let (block, _) =
            primitive_box(&mut m, Point3::origin(), Point3::new(10.0, 10.0, 10.0), n).unwrap();
        let plane = Frame::from_z(Point3::new(0.0, 0.0, 4.0), Vec3::new(0.2, 0.1, 1.0)).unwrap();
        let s = split(&mut m, block, &plane, n).unwrap();
        assert!(m.body(BodyId::new(1, 0)).is_err());
        for f in 6..12 {
            assert!(m.face(FaceId::new(f, 0)).is_err(), "scratch face {f}");
        }
        for side in [s.positive, s.negative] {
            assert!(arris_check::check(&m, side, arris_check::Level::Full).is_ok());
        }
        arris_topo::provenance::audit_many(&m, &[block], &[s.positive, s.negative], &s.provenance)
            .unwrap();
    }

    /// A plane past the body's box, and one through the corner of a
    /// cylinder's box that misses the cylinder: both refused naming the
    /// body, the model as it was.
    #[test]
    fn a_plane_that_misses_is_refused_naming_the_body() {
        let n = &Control::NONE;
        let mut m = Model::default();
        let (block, _) =
            primitive_box(&mut m, Point3::origin(), Point3::new(10.0, 10.0, 10.0), n).unwrap();
        let above = Frame::from_z(Point3::new(0.0, 0.0, 20.0), Vec3::z()).unwrap();
        refused_naming(split(&mut m, block, &above, n).unwrap_err(), block);

        let (cylinder, _) =
            primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 3.0, 10.0, n).unwrap();
        let corner = Frame::from_z(Point3::new(2.9, 2.9, 0.0), Vec3::new(1.0, 1.0, 0.0)).unwrap();
        refused_naming(split(&mut m, cylinder, &corner, n).unwrap_err(), cylinder);

        let mut fresh = Model::default();
        primitive_box(
            &mut fresh,
            Point3::origin(),
            Point3::new(10.0, 10.0, 10.0),
            n,
        )
        .unwrap();
        primitive_cylinder(&mut fresh, Axis::z_at(Point3::origin()), 3.0, 10.0, n).unwrap();
        let next = primitive_box(&mut m, Point3::origin(), Point3::new(1.0, 1.0, 1.0), n).unwrap();
        let want = primitive_box(&mut fresh, Point3::origin(), Point3::new(1.0, 1.0, 1.0), n);
        assert_eq!(next.0, want.unwrap().0, "the refusals consumed no id");
    }
}
