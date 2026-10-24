use core::fmt;
use std::collections::BTreeMap;

use arris_math::Point3;

#[cfg(doc)]
use super::{Assembly, Builder};
use super::{EdgeRef, FaceRef, Use, VertexRef};
use crate::entity::EdgeGeometry;
use crate::euler::EulerLine;
use crate::handle::Body;
use crate::id::{Curve2Id, EdgeId, FaceId, ShellId, SurfaceId, VertexId};
use crate::orientation::Orientation;

/// What a loop sorts by within its face: loops without uses first, by
/// their seed, then the rest by their first use — so a face's loop order
/// is a function of its content alone.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(super) enum LoopKey {
    Seed(VertexRef),
    Use(EdgeRef, Orientation),
}

/// A vertex under construction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StagedVertex {
    pub(super) point: Point3,
    pub(super) tolerance: f64,
    pub(super) kept: Option<VertexId>,
}

impl StagedVertex {
    /// The point.
    pub const fn point(&self) -> Point3 {
        self.point
    }

    /// The tolerance it will be stored with.
    pub const fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// The arena vertex this slot *is*, when [`Builder::assemble`] took it
    /// from the model and no operator has touched it since: [`Builder::finish`]
    /// appends nothing for it and returns this id. `None` for a slot that
    /// will be appended.
    pub const fn kept(&self) -> Option<VertexId> {
        self.kept
    }
}

/// An edge under construction: its geometry and its two vertices, the
/// start at the lower end of the range.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StagedEdge {
    pub(super) geometry: EdgeGeometry,
    pub(super) start: VertexRef,
    pub(super) end: VertexRef,
    pub(super) tolerance: f64,
    pub(super) kept: Option<EdgeId>,
}

impl StagedEdge {
    /// The geometry.
    pub const fn geometry(&self) -> EdgeGeometry {
        self.geometry
    }

    /// The vertex at the lower end of the range.
    pub const fn start(&self) -> VertexRef {
        self.start
    }

    /// The vertex at the upper end of the range.
    pub const fn end(&self) -> VertexRef {
        self.end
    }

    /// The tolerance it will be stored with.
    pub const fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// The arena edge this slot *is*, as [`StagedVertex::kept`].
    pub const fn kept(&self) -> Option<EdgeId> {
        self.kept
    }
}

/// A loop under construction: its uses in effective walking order, kept
/// rotated so that the use with the lowest `(edge, orientation)` comes
/// first. A loop with no uses is one vertex, `seed` — what
/// [`Builder::mvfs`] starts from and what a kill leaves behind; `finish`
/// refuses it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StagedLoop {
    pub(super) uses: Vec<Use>,
    pub(super) seed: VertexRef,
}

impl StagedLoop {
    /// The uses in walking order.
    pub fn uses(&self) -> &[Use] {
        &self.uses
    }

    /// The vertex a loop without uses stands at.
    pub const fn seed(&self) -> VertexRef {
        self.seed
    }

    pub(super) fn key(&self) -> LoopKey {
        match self.uses.first() {
            Some(u) => LoopKey::Use(u.edge, u.orientation),
            None => LoopKey::Seed(self.seed),
        }
    }

    pub(super) fn canonicalise(&mut self) {
        let Some(at) = (0..self.uses.len()).min_by_key(|&i| self.uses[i].key()) else {
            return;
        };
        self.uses.rotate_left(at);
    }
}

/// A face under construction: its surface, the orientation the shell will
/// use it with, the shell it belongs to, and its loops in the builder's
/// canonical order (loops without uses first, by their seed; then by their
/// first use).
#[derive(Debug, Clone, PartialEq)]
pub struct StagedFace {
    pub(super) surface: SurfaceId,
    pub(super) orientation: Orientation,
    pub(super) shell: usize,
    pub(super) loops: Vec<StagedLoop>,
    pub(super) tolerance: f64,
    pub(super) kept: Option<FaceId>,
}

impl StagedFace {
    /// The surface.
    pub const fn surface(&self) -> SurfaceId {
        self.surface
    }

    /// The shell the face belongs to: its index in the
    /// [`Assembly::shells`] the builder was assembled from, and `0` for a
    /// builder of operators, which makes one shell. A face an operator
    /// makes out of another belongs to that face's shell.
    pub const fn shell(&self) -> usize {
        self.shell
    }

    /// `Forward` when the surface's normal is the outward one, `Reversed`
    /// when it points into the material.
    pub const fn orientation(&self) -> Orientation {
        self.orientation
    }

    /// The loops in canonical order.
    pub fn loops(&self) -> &[StagedLoop] {
        &self.loops
    }

    /// The tolerance it will be stored with.
    pub const fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// The arena face this slot *is*, as [`StagedVertex::kept`]. An
    /// operator that changes the face drops the mark, and `finish` then
    /// appends a new face carrying the change.
    pub const fn kept(&self) -> Option<FaceId> {
        self.kept
    }

    pub(super) fn canonicalise(&mut self) {
        for lp in &mut self.loops {
            lp.canonicalise();
        }
        self.loops.sort_by_key(StagedLoop::key);
    }
}

/// What [`Builder::mvfs`] makes and [`Builder::kvfs`] returns: the first
/// vertex, and the surface and orientation of the first face.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Seed {
    /// The vertex's point.
    pub point: Point3,
    /// The face's surface.
    pub surface: SurfaceId,
    /// The orientation the shell will use the face with.
    pub orientation: Orientation,
}

/// What [`Builder::mev`] makes and [`Builder::kev`] returns: a new vertex
/// and the edge from the position's vertex to it, used twice in a row by
/// the loop — away and back.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Strut {
    /// The new vertex's point.
    pub point: Point3,
    /// The edge's geometry; its range runs from the existing vertex to the
    /// new one. Never `Degenerate`: a strut has two vertices.
    pub geometry: EdgeGeometry,
    /// The pcurves of the two uses on the face's surface: the `Forward`
    /// use away from the position, then the `Reversed` use back. A seam is
    /// exactly a strut whose two pcurves differ by the period.
    pub pcurves: [Option<Curve2Id>; 2],
}

/// What [`Builder::mef`] makes and [`Builder::kef`] returns: an edge
/// across a loop and the face on its left.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Split {
    /// The edge's geometry; its range runs from `from`'s vertex to `to`'s.
    /// `Degenerate` only when both are the same vertex.
    pub geometry: EdgeGeometry,
    /// The new face's surface.
    pub surface: SurfaceId,
    /// The orientation the shell will use the new face with.
    pub orientation: Orientation,
    /// The pcurves of the edge's two uses: `Forward` on the new face,
    /// `Reversed` on the old one.
    pub pcurves: [Option<Curve2Id>; 2],
}

/// What [`Builder::mekr`] makes and [`Builder::kemr`] returns: an edge
/// joining two loops of one face into one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Join {
    /// The edge's geometry; its range runs from `from`'s vertex to `to`'s.
    pub geometry: EdgeGeometry,
    /// The pcurves of the two uses, both on the face's surface: `Forward`
    /// from `from` to `to`, then `Reversed` back.
    pub pcurves: [Option<Curve2Id>; 2],
}

/// The counts of a builder and the genus it has made with
/// [`Builder::kfmrh`]: the Euler–Poincaré line
/// `V − E + F − (L − F) − 2(S − G)`, which every operator keeps at zero.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Counts {
    /// Live vertices.
    pub vertices: usize,
    /// Live edges, less every degenerate edge: a singular point of its
    /// face, which the Euler–Poincaré line does not count.
    pub edges: usize,
    /// Live faces.
    pub faces: usize,
    /// Loops over those faces.
    pub loops: usize,
    /// The shells the live faces belong to: one while a face exists for a
    /// builder of operators, the assembly's shells for an assembled one.
    pub shells: usize,
    /// Handles made by `kfmrh` and not yet removed by `mfkrh`.
    pub genus: usize,
}

impl Counts {
    /// `V − E + F − (L − F) − 2(S − G)`: zero for every state an operator
    /// leaves behind.
    pub const fn euler(&self) -> i64 {
        self.line().at_genus(self.genus as i64)
    }

    /// The Euler line of the counts, its genus derived from them rather
    /// than the handles made.
    pub const fn line(&self) -> EulerLine {
        EulerLine::new(
            self.vertices,
            self.edges,
            self.faces,
            self.loops,
            self.shells,
        )
    }
}

impl fmt::Display for Counts {
    /// `V/E/F/L/S g<G> = <euler>`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}/{}/{}/{}/{} g{} = {}",
            self.vertices,
            self.edges,
            self.faces,
            self.loops,
            self.shells,
            self.genus,
            self.euler()
        )
    }
}

/// What [`Builder::finish`] appended: the body, its shells, and the arena
/// id of every slot — what an operation's provenance is built from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Built {
    /// The body, `Forward`.
    pub body: Body,
    /// Its shells, one per shell index the faces carry, in index order —
    /// the order the body stores them in. One for a builder of operators.
    pub shells: Vec<ShellId>,
    /// Slot → arena id, every live vertex.
    pub vertices: BTreeMap<VertexRef, VertexId>,
    /// Slot → arena id, every live edge.
    pub edges: BTreeMap<EdgeRef, EdgeId>,
    /// Slot → arena id, every live face.
    pub faces: BTreeMap<FaceRef, FaceId>,
}

/// Tombstoned slots with a LIFO free list: a kill leaves a hole and the
/// next make of the same kind fills the most recently freed one, so a
/// kill undone by its make gives the slot back and a make undone by its
/// kill leaves only a hole.
#[derive(Debug, Clone)]
pub(super) struct Slots<T> {
    items: Vec<Option<T>>,
    free: Vec<u32>,
}

impl<T> Default for Slots<T> {
    fn default() -> Self {
        Slots {
            items: Vec::new(),
            free: Vec::new(),
        }
    }
}

impl<T> Slots<T> {
    pub(super) fn insert(&mut self, value: T) -> u32 {
        if let Some(i) = self.free.pop() {
            self.items[i as usize] = Some(value);
            return i;
        }
        // Slots are addressed by `u32`, as the arena's are; running out is
        // resource exhaustion, not a geometric condition.
        let i = u32::try_from(self.items.len()).expect("the builder holds at most u32::MAX slots");
        self.items.push(Some(value));
        i
    }

    pub(super) fn remove(&mut self, i: u32) -> Option<T> {
        let value = self.items.get_mut(i as usize)?.take();
        if value.is_some() {
            self.free.push(i);
        }
        value
    }

    pub(super) fn get(&self, i: u32) -> Option<&T> {
        self.items.get(i as usize)?.as_ref()
    }

    pub(super) fn get_mut(&mut self, i: u32) -> Option<&mut T> {
        self.items.get_mut(i as usize)?.as_mut()
    }

    /// Live slots in index order.
    pub(super) fn iter(&self) -> impl Iterator<Item = (u32, &T)> {
        self.items
            .iter()
            .enumerate()
            .filter_map(|(i, v)| v.as_ref().map(|v| (i as u32, v)))
    }

    pub(super) fn len(&self) -> usize {
        self.items.iter().filter(|v| v.is_some()).count()
    }

    /// The index the next `insert` will use.
    pub(super) fn next_slot(&self) -> u32 {
        self.free.last().copied().unwrap_or_else(|| {
            u32::try_from(self.items.len()).expect("the builder holds at most u32::MAX slots")
        })
    }

    pub(super) fn clear(&mut self) {
        self.items.clear();
        self.free.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slots_reuse_the_most_recently_freed_first() {
        let mut s = Slots::default();
        assert_eq!((s.insert('a'), s.insert('b'), s.insert('c')), (0, 1, 2));
        assert_eq!(s.remove(0), Some('a'));
        assert_eq!(s.remove(2), Some('c'));
        assert_eq!(s.remove(2), None, "already free");
        assert_eq!(s.len(), 1);
        assert_eq!(s.insert('d'), 2, "the last freed slot comes back first");
        assert_eq!(s.insert('e'), 0);
        assert_eq!(s.insert('f'), 3);
        let live: Vec<_> = s.iter().collect();
        assert_eq!(live, [(0, &'e'), (1, &'b'), (2, &'d'), (3, &'f')]);
    }
}
