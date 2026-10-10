//! Whether two solids interfere (ADR-0055): [`distance`]'s contact and
//! [`common`](crate::common)'s volume, read as one verdict.
//!
//! The distance decides apart or not; the common, built on a scratch
//! clone of the model so the model is never written, decides touching or
//! overlapping and measures the shared volume.

use arris_check::domain::FaceDomain;
use arris_math::{Aabb, Control, Meter, Point3};
use arris_topo::{Body, Model, Shape};

use super::distance::{Distance, Operand, distance};
use crate::boolean::common_probe;
use crate::error::{BooleanReason, InputReason, OpError, Reason};
use crate::measure::mass_properties;

/// What dimension two touching solids share.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Contact {
    /// One point: two balls, a ball on a plane, a cone's apex on a face.
    Point,
    /// A curve: two cylinders side by side, a box on a cylinder's ruling.
    Curve,
    /// A face: two boxes sharing a side.
    Face,
}

/// How two solids interfere.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Interference {
    /// The solids are farther apart than the model's linear tolerance;
    /// the minimum distance between them.
    Apart(Distance),
    /// The solids meet and share no volume.
    Touching {
        /// The dimension of the contact.
        contact: Contact,
        /// A point of the contact.
        at: Point3,
    },
    /// The solids share volume.
    Overlapping {
        /// The shared volume: `common`'s, measured on a scratch copy.
        volume: f64,
        /// `true` when the first solid lies wholly inside the second.
        a_in_b: bool,
        /// `true` when the second lies wholly inside the first.
        b_in_a: bool,
    },
}

/// How solid `a` interferes with solid `b`: [`Interference::Apart`] with
/// their [`distance`] when it exceeds the model's linear tolerance;
/// otherwise [`Interference::Touching`] when they share no volume — the
/// `common` of the pair has no material ([`BooleanReason::Empty`]) or no
/// thickness ([`InputReason::ZeroThickness`]) — and
/// [`Interference::Overlapping`] with the volume of that common and
/// whether either solid lies wholly in the other. Two equal solids are
/// each inside the other.
///
/// The model is never written: the common is built on a scratch clone of
/// it, which is dropped. The verdict agrees with [`distance`] on every
/// pose: not apart exactly when the distance is within the linear
/// tolerance.
///
/// A touching pair's [`Contact`] is a face when the common has no thickness,
/// a curve when the pave model holds a section edge, a tangent contact, a
/// common edge or an edge lying in a face, and a point otherwise; `at` is
/// the closest point [`distance`] chose, on both solids.
///
/// A body given as both operands is a solid with itself: it lies inside
/// itself, and the shared volume is its own.
///
/// Errors: [`distance`]'s for the pair, and `common`'s when the solids
/// are not apart — [`OpError::Unsupported`] naming the faces for a pair
/// with no closed form (a NURBS face meeting the other solid),
/// [`OpError::Degenerate`] with a boolean reason for a contact the
/// selection cannot read (`TangentContact`, `NonManifold`);
/// [`OpError::Interrupted`] when the control stops it.
///
/// ```
/// use arris_math::{Control, Point3};
/// use arris_ops::primitive_box;
/// use arris_ops::query::{Interference, interference};
/// use arris_topo::Model;
///
/// let mut m = Model::default();
/// let none = Control::NONE;
/// let (a, _) = primitive_box(&mut m, Point3::origin(), Point3::new(2.0, 2.0, 2.0), &none)?;
/// let (b, _) = primitive_box(&mut m, Point3::new(1.0, 1.0, 1.0), Point3::new(3.0, 3.0, 3.0), &none)?;
/// let Interference::Overlapping { volume, a_in_b, b_in_a } = interference(&m, a, b, &none)? else {
///     panic!("the boxes overlap")
/// };
/// assert!((volume - 1.0).abs() < 1e-12 && !a_in_b && !b_in_a);
/// # Ok::<(), arris_ops::OpError>(())
/// ```
pub fn interference(
    m: &Model,
    a: Body,
    b: Body,
    control: &Control<'_>,
) -> Result<Interference, OpError> {
    if a.id == b.id {
        // The pave model of a body with itself has no meaning; the answer
        // does not need one.
        let volume = mass_properties(m, a, control)?.volume;
        return Ok(Interference::Overlapping {
            volume,
            a_in_b: true,
            b_in_a: true,
        });
    }
    let shape = |body: Body| Operand::Shape(Shape::new(body.id, body.orientation));
    let d = distance(m, shape(a), shape(b), control)?;
    if d.distance > m.precision().tolerance().linear {
        return Ok(Interference::Apart(d));
    }
    let mut scratch = m.clone();
    let probe = common_probe(&mut scratch, a, b, control)?;
    match probe.common {
        Ok((common, [a_in_b, b_in_a])) => Ok(Interference::Overlapping {
            volume: mass_properties(&scratch, common, control)?.volume,
            a_in_b,
            b_in_a,
        }),
        Err(OpError::Degenerate {
            reason: Reason::Boolean(BooleanReason::Empty),
            ..
        }) => {
            // Nothing was dropped on the other operand, so there is no
            // area: a curve where the pave model holds one, else a point.
            let i = &probe.interferences;
            let curve = !i.sections.is_empty()
                || !i.contacts.is_empty()
                || !i.blocks.is_empty()
                || i.images.iter().any(|image| image.range.length() > 0.0);
            Ok(Interference::Touching {
                contact: if curve {
                    Contact::Curve
                } else {
                    Contact::Point
                },
                at: d.a.point,
            })
        }
        Err(OpError::Degenerate {
            reason: Reason::Input(InputReason::ZeroThickness),
            ..
        }) => Ok(Interference::Touching {
            contact: Contact::Face,
            at: d.a.point,
        }),
        Err(e) => Err(e),
    }
}

/// One pair of a screened assembly that is not apart.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Clash {
    /// The indices, into the list given to [`interference_many`], of the
    /// first and the second body: `operands[0] < operands[1]`.
    pub operands: [usize; 2],
    /// How they interfere: [`Interference::Touching`] or
    /// [`Interference::Overlapping`], never [`Interference::Apart`], read
    /// with the first body as `a`.
    pub verdict: Interference,
}

/// The box of `body`: the union of its faces' boxes, `None` when any face
/// has none (it then comes within reach of every body).
fn body_bounds(m: &Model, body: Body) -> Result<Option<Aabb>, OpError> {
    let tolerance = m.precision().parametric_tolerance;
    let mut all: Option<Aabb> = None;
    for face in m.closure(body)?.faces {
        let Some(bounds) = FaceDomain::of(m, face, tolerance)
            .ok()
            .and_then(|d| d.bounds())
        else {
            return Ok(None);
        };
        all = Some(all.map_or(bounds, |a| a.union(bounds)));
    }
    Ok(all)
}

/// Screens an assembly: the [`Interference`] of every pair of `bodies`
/// that is not [`Interference::Apart`], as [`Clash`]es in ascending pair
/// order `(0, 1)`, `(0, 2)`, …, `(1, 2)`, …, each read with the lower
/// index as `a`. A pair whose bodies' boxes, grown by the model's linear
/// tolerance, are disjoint is apart and costs a box test; every other
/// pair is [`interference`]'s question, so the result is that of calling
/// it on every pair (the same list a brute-force loop gives).
///
/// The model is never written. The control is polled once per candidate
/// pair, here and inside it; a budget limits the pairs examined here and
/// each pair's own steps separately, as [`interference`] reads it.
///
/// Errors: [`interference`]'s for the first candidate pair that fails, in
/// pair order — one NURBS face meeting another body stops the screen,
/// naming it — and [`OpError::Interrupted`] when the control stops it.
///
/// ```
/// use arris_math::{Control, Point3};
/// use arris_ops::primitive_box;
/// use arris_ops::query::{Interference, interference_many};
/// use arris_topo::Model;
///
/// let mut m = Model::default();
/// let none = Control::NONE;
/// let mut cube = |x: f64| -> Result<_, arris_ops::OpError> {
///     Ok(primitive_box(&mut m, Point3::new(x, 0.0, 0.0), Point3::new(x + 2.0, 2.0, 2.0), &none)?.0)
/// };
/// let (a, b, c) = (cube(0.0)?, cube(1.0)?, cube(10.0)?);
/// let clashes = interference_many(&m, &[a, b, c], &none)?;
/// assert_eq!(clashes.len(), 1);
/// assert_eq!(clashes[0].operands, [0, 1]);
/// assert!(matches!(clashes[0].verdict, Interference::Overlapping { .. }));
/// # Ok::<(), arris_ops::OpError>(())
/// ```
pub fn interference_many(
    m: &Model,
    bodies: &[Body],
    control: &Control<'_>,
) -> Result<Vec<Clash>, OpError> {
    let reach = m.precision().tolerance().linear;
    let boxes = bodies
        .iter()
        .map(|&b| Ok(body_bounds(m, b)?.map(|bounds| bounds.inflated(reach))))
        .collect::<Result<Vec<_>, OpError>>()?;
    let mut meter = Meter::new(control);
    let mut out = Vec::new();
    for i in 0..bodies.len() {
        for j in i + 1..bodies.len() {
            if let (Some(bi), Some(bj)) = (&boxes[i], &boxes[j])
                && !bi.intersects(bj)
            {
                continue;
            }
            meter.tick()?;
            let verdict = interference(m, bodies[i], bodies[j], control)?;
            if !matches!(verdict, Interference::Apart(_)) {
                out.push(Clash {
                    operands: [i, j],
                    verdict,
                });
            }
        }
    }
    Ok(out)
}
