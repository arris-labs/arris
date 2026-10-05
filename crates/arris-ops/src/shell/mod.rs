//! Shell (ADR-0049): a solid hollowed to a wall of constant thickness. By
//! phase over `offset/`: the offset's pieces of every face but the
//! openings (`offset::pieces`, the walls moved by the thickness to the
//! side), then the assembly (`assemble`) — the body's faces kept, the
//! moved faces beside them as a second skin, each opening a rim face
//! between the two — and the checker at `Level::Full` in every profile.

use std::collections::BTreeSet;

use arris_math::{Control, Meter};
use arris_topo::{Body, Face, FaceId, Model, Provenance};

use crate::error::{InputReason, OpError, Reason, ShellReason};
use crate::rebuild::forward;

mod assemble;

/// The side of a body's faces a [`shell`]'s wall grows on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum ShellSide {
    /// The body's faces stay the outside; the cavity grows inside them,
    /// and the result fits within the body.
    Inward,
    /// The body's faces become the cavity; a skin grows outside them, and
    /// the result encloses the body.
    Outward,
}

/// Hollows the solid `body` to a wall of `thickness`, open at
/// `openings`, and returns the result with its provenance (ADR-0049).
/// The wall's second skin is `offset_faces`' offset of every face but the
/// openings — each face on the offset of its own surface, the same kind,
/// joined sharp at every edge — by `thickness` to `side`: `Inward` keeps
/// the body's faces as the outside and puts the skin, reversed, inside
/// them; `Outward` keeps the body's faces, reversed, as the cavity and
/// puts the skin outside them. Each loop of an opening becomes a rim face
/// on the opening's own surface, bounded by the body's loop and the
/// skin's. With no openings the cavity is a closed void: the result has
/// a second shell, nested inside the first. The result passes the checker
/// at `Level::Full` in every build profile.
///
/// Provenance: each wall face, and each edge and vertex the skin copies,
/// is kept and `Modified` into itself, with its skin copy `Generated` from
/// it — a consumer names the inner face after the face it came from; each
/// opening `Modified` into its rim faces; each shell `Modified` into the
/// shell holding its kept faces, a void's second shell `Generated` from
/// the body, and the body `Modified`. `arris_topo::provenance::audit`
/// holds on every result.
///
/// Errors, the model untouched: [`OpError::Degenerate`] with
/// [`InputReason::NonFinite`] on a non-finite thickness and
/// [`InputReason::NotPositive`] on one at or below zero;
/// [`ShellReason::RepeatedOpening`] for a face listed twice,
/// [`ShellReason::OpeningNotInBody`] for one that is not the body's,
/// [`ShellReason::NoWalls`] when every face is an opening,
/// [`ShellReason::OpeningDragged`] naming an opening tangent to a wall;
/// every refusal of the walls' offset as [`Reason::Offset`] naming its
/// entity, as [`crate::offset_faces`] documents them — the skin running
/// into the outer faces, a thickness past the thinnest wall, is
/// [`crate::OffsetReason::SelfIntersects`]; [`OpError::Unsupported`]
/// naming two openings that share an edge, which this release does not
/// merge yet; [`OpError::NotFound`] for a face id that does not resolve;
/// [`OpError::Internal`] with the report where the result fails the
/// checker.
///
/// ```
/// use arris_ops::measure::mass_properties;
/// use arris_ops::{Control, ShellSide, primitive_box, shell};
/// use arris_math::Point3;
/// use arris_topo::Model;
///
/// let mut m = Model::default();
/// let (cube, _) = primitive_box(&mut m, Point3::origin(), Point3::new(10.0, 10.0, 10.0), &Control::NONE).unwrap();
/// // The top face, at z = 10.
/// let top = m.faces(cube).unwrap().into_iter().find(|f| {
///     arris_check::classify::classify_point(&m, cube, Point3::new(5.0, 5.0, 10.0)).unwrap()
///         == arris_check::classify::Classification::On(f.shape())
/// }).unwrap();
/// let (cup, provenance) = shell(&mut m, cube, &[top], 1.0, ShellSide::Inward, &Control::NONE).unwrap();
/// let volume = mass_properties(&m, cup, &Control::NONE).unwrap().volume;
/// assert!((volume - (1000.0 - 8.0 * 8.0 * 9.0)).abs() < 1e-9);
/// // Five walls outside, five inside, and the rim.
/// assert_eq!(m.faces(cup).unwrap().len(), 11);
/// assert_eq!(provenance.modified_from(top.shape()).len(), 1);
/// ```
pub fn shell(
    m: &mut Model,
    body: Body,
    openings: &[Face],
    thickness: f64,
    side: ShellSide,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    crate::verify_input(m, body)?;
    let b = body.shape();
    let refuse = |entities, reason| OpError::Degenerate { entities, reason };
    if !thickness.is_finite() {
        return Err(refuse(
            vec![b],
            Reason::Input(InputReason::NonFinite { what: "thickness" }),
        ));
    }
    if thickness <= 0.0 {
        return Err(refuse(
            vec![b],
            Reason::Input(InputReason::NotPositive {
                what: "thickness",
                value: thickness,
            }),
        ));
    }
    let closure = m.closure(body)?;
    let mut open: BTreeSet<FaceId> = BTreeSet::new();
    for face in openings {
        m.face(face.id)?;
        if !open.insert(face.id) {
            return Err(refuse(
                vec![forward(face.id)],
                Reason::Shell(ShellReason::RepeatedOpening),
            ));
        }
        if closure.faces.binary_search(&face.id).is_err() {
            return Err(refuse(
                vec![forward(face.id), b],
                Reason::Shell(ShellReason::OpeningNotInBody),
            ));
        }
    }
    let walls: BTreeSet<FaceId> = closure
        .faces
        .iter()
        .copied()
        .filter(|f| !open.contains(f))
        .collect();
    if walls.is_empty() {
        return Err(refuse(vec![b], Reason::Shell(ShellReason::NoWalls)));
    }
    let distance = match side {
        ShellSide::Inward => -thickness,
        ShellSide::Outward => thickness,
    };
    let mut meter = Meter::new(control);
    m.transaction(|m| assemble::build(m, body, &open, &walls, distance, side, &mut meter))
}
