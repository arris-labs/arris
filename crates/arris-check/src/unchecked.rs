//! A `Full` row the kernel could not decide.

use core::fmt;

use arris_geom::SurfaceKind;
use arris_topo::{BodyId, EntityId, FaceId, ShellId};

/// A `Full` invariant the checker could not decide on this body, listed
/// by [`Report::unchecked`](crate::Report::unchecked). Never a violation
/// and never a silent pass: an operation that cannot afford an undecided
/// row asks for the list and refuses.
///
/// Every variant names the row it belongs to and the entities it is
/// about; [`Unchecked::code`] gives the row's number in
/// `docs/DATA-MODEL.md` §Invariants.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum Unchecked {
    /// **S5** — the intersector does not decide this pair of surfaces —
    /// a [`SurfaceKind::Nurbs`] in it, or a pose one of the two tracers
    /// refuses (ADR-0018, ADR-0019) — so whether the two faces meet away
    /// from their shared edges is not known. Every pair of analytic
    /// surfaces is decided in every pose, a torus against anything among
    /// them.
    FacePair {
        /// The shell holding both faces.
        shell: ShellId,
        /// The first face.
        face_a: FaceId,
        /// The second face.
        face_b: FaceId,
        /// Their surfaces' kinds, in that order.
        kinds: (SurfaceKind, SurfaceKind),
    },
    /// **B1** — no ray from this shell could be classified against another
    /// shell of the solid, so which shell it lies inside is not known.
    ShellNesting {
        /// The body.
        body: BodyId,
        /// The shell that could not be placed.
        shell: ShellId,
    },
    /// **B1** — the intersector does not decide a face of one shell
    /// against a face of another — [`Unchecked::FacePair`]'s pair, on two
    /// shells — so whether the two shells meet is not known.
    ShellFacePair {
        /// The body.
        body: BodyId,
        /// The face of the first shell.
        face_a: FaceId,
        /// The face of the second.
        face_b: FaceId,
        /// Their surfaces' kinds, in that order.
        kinds: (SurfaceKind, SurfaceKind),
    },
}

impl Unchecked {
    /// The row number in `docs/DATA-MODEL.md` §Invariants.
    pub const fn code(&self) -> &'static str {
        match self {
            Unchecked::FacePair { .. } => "S5",
            Unchecked::ShellNesting { .. } | Unchecked::ShellFacePair { .. } => "B1",
        }
    }

    /// The entity the row is reported against: what the list is sorted
    /// by, as a violation's is.
    pub fn entity(&self) -> EntityId {
        match *self {
            Unchecked::FacePair { shell, .. } => shell.into(),
            Unchecked::ShellNesting { body, .. } | Unchecked::ShellFacePair { body, .. } => {
                body.into()
            }
        }
    }
}

impl fmt::Display for Unchecked {
    /// One line, marked `?` where a violation's line has its code alone.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}? {}: ", self.code(), self.entity())?;
        match self {
            Unchecked::FacePair {
                face_a,
                face_b,
                kinds,
                ..
            } => write!(
                f,
                "{face_a} ({}) against {face_b} ({}) is not decided",
                kinds.0, kinds.1
            ),
            Unchecked::ShellNesting { shell, .. } => {
                write!(f, "no ray from {shell} could be classified")
            }
            Unchecked::ShellFacePair {
                face_a,
                face_b,
                kinds,
                ..
            } => write!(
                f,
                "{face_a} ({}) against {face_b} ({}), of two shells, is not decided",
                kinds.0, kinds.1
            ),
        }
    }
}
