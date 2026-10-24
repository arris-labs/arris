use core::fmt;

#[cfg(doc)]
use super::Builder;
use crate::id::Curve2Id;
#[cfg(doc)]
use crate::id::{EdgeId, FaceId, VertexId};
use crate::orientation::Orientation;

macro_rules! refs {
    ($( $(#[$doc:meta])* $name:ident => $prefix:literal ),* $(,)?) => {$(
        $(#[$doc])*
        ///
        /// A slot in the builder, not an arena id: the entity gets its
        /// [`VertexId`]/[`EdgeId`]/[`FaceId`] from [`Builder::finish`],
        /// which returns the map. Slots are reused by the next make after
        /// a kill, most recently freed first.
        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(pub(super) u32);

        impl $name {
            /// The reference to slot `index`. The operators mint the ones
            /// that resolve; a test that needs a dangling one builds it here.
            pub const fn new(index: u32) -> Self {
                Self(index)
            }

            /// The slot index.
            pub const fn index(self) -> u32 {
                self.0
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{}{}", $prefix, self.0)
            }
        }
    )*};
}

refs! {
    /// A vertex under construction.
    VertexRef => "v",
    /// An edge under construction.
    EdgeRef => "e",
    /// A face under construction.
    FaceRef => "f",
}

/// A place in a loop: the junction before coedge `coedge_index` of loop
/// `loop_index` of `face`, which is the effective start vertex of that
/// coedge. `coedge_index` may equal the loop's length, naming the junction
/// after its last coedge — the same vertex as index 0, and the same
/// insertion point for [`Builder::mev`], but a different split for
/// [`Builder::mef`]: the coedges from `to` up to `from` go to the new face,
/// so `to = 0, from = n` moves them all and `to = from` moves none. For
/// the full turn from any other junction `s`, `from` is `s + n` — the one
/// place a position exceeds the length, and only as `mef`'s `from`.
///
/// Operators take positions rather than vertices because a vertex may
/// occur several times in one loop — at a seam, at a closed edge — and
/// only the position says which occurrence is meant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Position {
    /// The face.
    pub face: FaceRef,
    /// Which loop of the face, in the builder's canonical order.
    pub loop_index: usize,
    /// Which junction of the loop, `0..=len`.
    pub coedge_index: usize,
}

impl Position {
    /// The junction before coedge `coedge_index` of loop `loop_index` of
    /// `face`.
    pub const fn new(face: FaceRef, loop_index: usize, coedge_index: usize) -> Self {
        Position {
            face,
            loop_index,
            coedge_index,
        }
    }
}

impl fmt::Display for Position {
    /// `f3/0/2`: face, loop, coedge.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}/{}/{}", self.face, self.loop_index, self.coedge_index)
    }
}

/// One use of an edge by a loop under construction: the edge, the
/// *effective* direction it is walked in (as seen from outside the
/// material), and the pcurve of this use on the face's surface, if the
/// caller has given it yet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Use {
    /// The edge.
    pub edge: EdgeRef,
    /// Along (`Forward`) or against (`Reversed`) the edge's curve, as seen
    /// from outside with the face's outward normal up.
    pub orientation: Orientation,
    /// The pcurve of this use on the face's surface; `None` until
    /// [`Builder::set_pcurve`] or the operator that made the use gave it.
    /// [`Builder::finish`] refuses a `None`.
    pub pcurve: Option<Curve2Id>,
}

impl Use {
    pub(super) const fn key(&self) -> (EdgeRef, Orientation) {
        (self.edge, self.orientation)
    }
}

impl fmt::Display for Use {
    /// `+e3 p7`, or `+e3 ?` without a pcurve.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}{} ", self.orientation, self.edge)?;
        match self.pcurve {
            Some(p) => write!(f, "{p}"),
            None => f.write_str("?"),
        }
    }
}
