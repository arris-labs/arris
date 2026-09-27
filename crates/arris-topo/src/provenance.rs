//! What an operation returns beside its body: which output entity came
//! from which input, and how (`docs/DATA-MODEL.md` §Provenance,
//! ADR-0002).
//!
//! Three relations in Open CASCADE's `BRepTools_History` vocabulary —
//! `Generated`, `Modified`, `Deleted` — over origins that are either an
//! input entity or a [`Role`]: what an entity *is* to the operation that
//! made it from nothing, so that a chain of records followed through
//! [`Provenance::then`] ends at a role, which is what a consumer's
//! persistent name is a function of.

use core::fmt;
use std::collections::{BTreeMap, BTreeSet};

use crate::error::NotFound;
use crate::handle::{Body, Shape};
use crate::idmap::IdMap;
use crate::model::Model;
use crate::orientation::Orientation;

/// How an output entity relates to an origin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Relation {
    /// The output is a new entity of a different kind or role built from
    /// the origin: a hole's wall from the tool's cylindrical face, an
    /// intersection edge from a pair of faces, every entity of a
    /// primitive from its role.
    Generated,
    /// The output is a trimmed, split or re-tolerated piece of the
    /// origin, same kind.
    Modified,
    /// The input has no image in the output.
    Deleted,
}

impl Relation {
    /// The relation of a chain: `self` from an origin to an entity, then
    /// `next` from that entity on. A piece of a generated entity is
    /// generated; only pieces of pieces stay `Modified`.
    pub const fn then(self, next: Relation) -> Relation {
        match (self, next) {
            (Relation::Modified, Relation::Modified) => Relation::Modified,
            (Relation::Deleted, _) | (_, Relation::Deleted) => Relation::Deleted,
            _ => Relation::Generated,
        }
    }
}

impl fmt::Display for Relation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Relation::Generated => "generated",
            Relation::Modified => "modified",
            Relation::Deleted => "deleted",
        })
    }
}

/// A coordinate axis of a box.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Coord {
    /// `x`.
    X,
    /// `y`.
    Y,
    /// `z`.
    Z,
}

/// The lower or the upper side of a box along a coordinate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Side {
    /// At the minimum corner's coordinate.
    Min,
    /// At the maximum corner's coordinate.
    Max,
}

/// What an entity of `primitive_box` is: a face by the coordinate its
/// outward normal runs along and the side; an edge by the coordinate it
/// runs along and its sides along the other two coordinates in `x, y, z`
/// order; a vertex by its three sides.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum BoxPart {
    /// The body.
    Body,
    /// The one shell.
    Shell,
    /// The face whose outward normal is along `coord` towards `side`.
    Face(Coord, Side),
    /// The edge along `along`, at `sides` along the other two coordinates
    /// in `x, y, z` order.
    Edge {
        /// The coordinate the edge runs along.
        along: Coord,
        /// Its sides along the two other coordinates.
        sides: [Side; 2],
    },
    /// The vertex at these sides along `x, y, z`.
    Vertex([Side; 3]),
}

/// What an entity of `primitive_cylinder` is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CylinderPart {
    /// The body.
    Body,
    /// The one shell.
    Shell,
    /// The cylindrical face.
    Wall,
    /// The cap at the axis origin, facing against the axis.
    BottomCap,
    /// The cap at the axis origin plus the height, facing along the axis.
    TopCap,
    /// The circular edge of the bottom cap.
    BottomRim,
    /// The circular edge of the top cap.
    TopRim,
    /// The wall's seam, along the axis where the surface's `X` points.
    Seam,
    /// The seam's vertex on the bottom rim.
    BottomVertex,
    /// The seam's vertex on the top rim.
    TopVertex,
}

/// What an entity of a sweep — `extrude` or `revolve` — is, named by the
/// part of the consumer's sketch it came from: the caps from the profile
/// face, a side face and its start and end edges from one segment, a rise
/// and its two vertices from one vertex of the profile. The indices are
/// the consumer's own (`docs/DATA-MODEL.md` §Profiles): `loop_index` `0`
/// is the outer loop and the holes count from `1`; `segment` is the
/// segment's position in its loop as written; `vertex` is the index of
/// the segment that *starts* there, so a circle loop has segment `0` and
/// vertex `0`. In a full revolve there is no `EndCap`, no `EndEdge` and
/// no `EndVertex`: the start edges are the seams and the start vertices
/// the only ones; and a segment perpendicular to the axis sweeps an
/// annulus of two closed rises, so it has no `StartEdge` either.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SweepPart {
    /// The body.
    Body,
    /// The outer shell: the one shell of every sweep but a full revolve
    /// with voids, where it is the lump's outer shell.
    Shell,
    /// A void shell of a full revolve, a cavity of the body's one lump
    /// (ADR-0006): the shell a hole closes into, or a notch cut in from the
    /// axis — any chain of a loop's segments off the axis but the one whose
    /// ends span the others.
    Cavity {
        /// The chain's loop, as the consumer wrote it.
        loop_index: usize,
        /// The lowest index the consumer wrote among the chain's segments:
        /// `0` for a hole, and a different one for each notch of a loop.
        segment: usize,
    },
    /// The profile face where the sweep starts, its outward normal against
    /// the sweep.
    StartCap,
    /// The profile face where the sweep ends.
    EndCap,
    /// The face one segment sweeps.
    Side {
        /// The segment's loop.
        loop_index: usize,
        /// The segment.
        segment: usize,
    },
    /// The segment itself, where the sweep starts.
    StartEdge {
        /// The segment's loop.
        loop_index: usize,
        /// The segment.
        segment: usize,
    },
    /// The segment carried to where the sweep ends.
    EndEdge {
        /// The segment's loop.
        loop_index: usize,
        /// The segment.
        segment: usize,
    },
    /// The edge one vertex of the profile sweeps: a line for an extrude, a
    /// circle about the axis for a revolve.
    Rise {
        /// The vertex's loop.
        loop_index: usize,
        /// The vertex: the index of the segment that starts there.
        vertex: usize,
    },
    /// The vertex itself, where the sweep starts.
    StartVertex {
        /// The vertex's loop.
        loop_index: usize,
        /// The vertex.
        vertex: usize,
    },
    /// The vertex carried to where the sweep ends.
    EndVertex {
        /// The vertex's loop.
        loop_index: usize,
        /// The vertex.
        vertex: usize,
    },
}

/// An instance of an exchange file an entity was read from: the file's
/// `#id`, and which placement of it, since an assembly places one solid
/// several times and each placement is a body of its own. Instances are
/// numbered from `0` in the reader's deterministic order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FileEntity {
    /// The instance's `#id` in the file.
    pub id: u64,
    /// Which placement of it.
    pub instance: u32,
}

/// A consumer's own name for what an entity is: a `key` in the
/// consumer's `namespace`. The kernel never reads either field — it
/// carries the pair, orders it by `(namespace, key)` and prints it — so
/// the words stay the consumer's (ADR-0009, ADR-0028). Two slots under
/// one key are two outputs of one role, in the order they were recorded,
/// and a key may be reused across entity kinds: the kernel does not
/// decide what a key means.
///
/// ```
/// use arris_topo::provenance::{ConsumerKey, Role};
///
/// let wall = Role::Consumer(ConsumerKey { namespace: 7, key: 42 });
/// assert_eq!(wall.to_string(), "consumer:7/42");
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ConsumerKey {
    /// Which of the consumer's vocabularies the key belongs to: a plugin,
    /// a feature kind — the consumer's choice.
    pub namespace: u32,
    /// The name within the namespace.
    pub key: u64,
}

/// What an entity is to the operation that made it from nothing:
/// exhaustive over the operations that generate from no input body, one
/// variant per operation kind, and the consumer's own key for a body it
/// built or a record it re-rooted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Role {
    /// An entity of `primitive_box`.
    Box(BoxPart),
    /// An entity of `primitive_cylinder`.
    Cylinder(CylinderPart),
    /// An entity of `extrude`.
    Extrude(SweepPart),
    /// An entity of `revolve`.
    Revolve(SweepPart),
    /// An entity of a file a reader read: the file entity it stands for
    /// (`docs/DATA-MODEL.md` §Provenance).
    File(FileEntity),
    /// An entity a consumer named: built by `ops::build` from topology
    /// the consumer filled, or re-rooted there from another operation's
    /// role. Appended last so the serialised index of every earlier
    /// variant stays where it was.
    Consumer(ConsumerKey),
}

impl fmt::Display for Role {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Role::Box(part) => write!(f, "box:{part:?}"),
            Role::Cylinder(part) => write!(f, "cylinder:{part:?}"),
            Role::Extrude(part) => write!(f, "extrude:{part:?}"),
            Role::Revolve(part) => write!(f, "revolve:{part:?}"),
            Role::File(e) => write!(f, "file:#{}/{}", e.id, e.instance),
            Role::Consumer(k) => write!(f, "consumer:{}/{}", k.namespace, k.key),
        }
    }
}

/// Where an output entity comes from: an entity of an input body, or a
/// role for an operation with no input body. Orders entities before
/// roles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Origin {
    /// An entity of an input body.
    Entity(Shape),
    /// A role in an operation that made the entity from nothing.
    Role(Role),
}

impl From<Shape> for Origin {
    fn from(s: Shape) -> Self {
        Origin::Entity(s)
    }
}

impl From<Role> for Origin {
    fn from(r: Role) -> Self {
        Origin::Role(r)
    }
}

impl fmt::Display for Origin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Origin::Entity(s) => write!(f, "{s}"),
            Origin::Role(r) => write!(f, "{r}"),
        }
    }
}

/// The record an operation returns: `generated` and `modified` from each
/// origin to its outputs, in the order the operation added them and
/// without duplicates, and the `deleted` inputs. **The order of an
/// origin's outputs is a contract** (ADR-0009, `docs/DATA-MODEL.md`
/// §Provenance, "Split order"): an operation adds pieces in split order —
/// a face's by the origins that bound each piece, an edge's along its
/// curve — so `Split(k)` in a consumer's persistent name means the same
/// piece after a parameter edit that keeps which entities bound which
/// piece. Two records are equal only when they list every origin's
/// outputs in the same order. Every entity of every input body is kept — present in the
/// output and unrecorded — or recorded: `Modified` into pieces,
/// `Generated` from, `Deleted`, or both `Deleted` and `Generated` from
/// (the tool face that is gone and whose image is the hole's wall); never
/// both `Deleted` and `Modified`, since a piece is an image. That
/// accounting is what an operation's tests assert, not something the
/// record enforces. An intersection edge generated from two faces is two
/// `Generated` records, one per face, and [`Provenance::generated_pair`]
/// is their intersection.
///
/// ```
/// use arris_topo::provenance::{BoxPart, Coord, Origin, Provenance, Relation, Role, Side};
/// use arris_topo::{FaceId, Shape, Orientation};
///
/// let top = Shape::new(FaceId::new(5, 0), Orientation::Forward);
/// let role = Role::Box(BoxPart::Face(Coord::Z, Side::Max));
/// let mut p = Provenance::default();
/// p.add_generated(role, top);
/// assert_eq!(p.generated_from(role), [top]);
/// assert_eq!(p.origins(top), [(Relation::Generated, Origin::Role(role))]);
/// ```
#[derive(Debug, Clone, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Provenance {
    #[cfg_attr(feature = "serde", serde(with = "records"))]
    generated: BTreeMap<Origin, Vec<Shape>>,
    #[cfg_attr(feature = "serde", serde(with = "records"))]
    modified: BTreeMap<Origin, Vec<Shape>>,
    deleted: BTreeSet<Shape>,
}

/// A relation's map on the wire: a sequence of `(origin, outputs)` pairs
/// in origin order, since JSON has no map keyed by anything but text.
/// Read back, it is refused unless it is what a record could have held:
/// origins strictly ascending, no output twice in one list (`push_once`).
#[cfg(feature = "serde")]
mod records {
    use std::collections::BTreeMap;

    use serde::de::Error as _;
    use serde::{Deserialize, Deserializer, Serializer};

    use super::Origin;
    use crate::handle::Shape;

    pub(super) fn serialize<S: Serializer>(
        map: &BTreeMap<Origin, Vec<Shape>>,
        serializer: S,
    ) -> Result<S::Ok, S::Error> {
        serializer.collect_seq(map)
    }

    pub(super) fn deserialize<'de, D: Deserializer<'de>>(
        deserializer: D,
    ) -> Result<BTreeMap<Origin, Vec<Shape>>, D::Error> {
        let pairs = Vec::<(Origin, Vec<Shape>)>::deserialize(deserializer)?;
        let mut map = BTreeMap::new();
        for (origin, outputs) in pairs {
            if map
                .last_key_value()
                .is_some_and(|(last, _)| *last >= origin)
            {
                return Err(D::Error::custom(format!(
                    "record origins out of order at {origin}"
                )));
            }
            for (i, s) in outputs.iter().enumerate() {
                if outputs[..i].contains(s) {
                    return Err(D::Error::custom(format!("{origin} lists {s} twice")));
                }
            }
            map.insert(origin, outputs);
        }
        Ok(map)
    }
}

/// Appends `s` unless the list already holds it: the order is the
/// caller's, the split order, and never sorted (ADR-0009).
fn push_once(list: &mut Vec<Shape>, s: Shape) {
    if !list.contains(&s) {
        list.push(s);
    }
}

impl Provenance {
    /// A record with nothing in it: an operation that touched nothing.
    pub fn new() -> Self {
        Self::default()
    }

    /// Records `output` as generated from `origin`, after every output
    /// recorded from `origin` so far; recording it again changes nothing.
    pub fn add_generated(&mut self, origin: impl Into<Origin>, output: impl Into<Shape>) {
        push_once(
            self.generated.entry(origin.into()).or_default(),
            output.into(),
        );
    }

    /// Records `output` as a piece of `origin`, after every piece recorded
    /// from `origin` so far; recording it again changes nothing.
    pub fn add_modified(&mut self, origin: impl Into<Origin>, output: impl Into<Shape>) {
        push_once(
            self.modified.entry(origin.into()).or_default(),
            output.into(),
        );
    }

    /// Records `input` as having no image in the output.
    pub fn add_deleted(&mut self, input: impl Into<Shape>) {
        self.deleted.insert(input.into());
    }

    /// The outputs generated from `origin`, in the order they were added:
    /// the split order for pieces (ADR-0009).
    pub fn generated_from(&self, origin: impl Into<Origin>) -> &[Shape] {
        self.generated
            .get(&origin.into())
            .map_or(&[], Vec::as_slice)
    }

    /// The outputs that are pieces of `origin`, in the order they were
    /// added: the split order (ADR-0009).
    pub fn modified_from(&self, origin: impl Into<Origin>) -> &[Shape] {
        self.modified.get(&origin.into()).map_or(&[], Vec::as_slice)
    }

    /// The outputs generated from both `a` and `b`: an intersection edge
    /// of two faces.
    pub fn generated_pair(&self, a: impl Into<Origin>, b: impl Into<Origin>) -> Vec<Shape> {
        let (a, b) = (self.generated_from(a), self.generated_from(b));
        a.iter().filter(|s| b.contains(s)).copied().collect()
    }

    /// `true` when `input` has no image in the output.
    pub fn is_deleted(&self, input: impl Into<Shape>) -> bool {
        self.deleted.contains(&input.into())
    }

    /// The deleted inputs, ascending.
    pub fn deleted(&self) -> impl Iterator<Item = Shape> + '_ {
        self.deleted.iter().copied()
    }

    /// Every origin with a `generated` or `modified` record, ascending.
    pub fn origins_recorded(&self) -> impl Iterator<Item = Origin> + '_ {
        self.generated
            .keys()
            .chain(self.modified.keys())
            .copied()
            .collect::<BTreeSet<_>>()
            .into_iter()
    }

    /// Every output with a record, ascending.
    pub fn outputs(&self) -> BTreeSet<Shape> {
        self.generated
            .values()
            .chain(self.modified.values())
            .flatten()
            .copied()
            .collect()
    }

    /// The inverse: every `(relation, origin)` that `output` came from,
    /// ascending by origin then relation.
    pub fn origins(&self, output: impl Into<Shape>) -> Vec<(Relation, Origin)> {
        let output = output.into();
        let mut out: Vec<(Relation, Origin)> = Vec::new();
        for (relation, map) in [
            (Relation::Generated, &self.generated),
            (Relation::Modified, &self.modified),
        ] {
            for (origin, outputs) in map {
                if outputs.contains(&output) {
                    out.push((relation, *origin));
                }
            }
        }
        out.sort_by(|a, b| a.1.cmp(&b.1).then(a.0.cmp(&b.0)));
        out
    }

    /// `true` when `input` is untouched by the operation and present in
    /// `output_body`'s closure: kept, which is not recorded.
    pub fn is_kept(&self, input: impl Into<Shape>, model: &Model, output_body: Body) -> bool {
        let input = input.into();
        if self.deleted.contains(&input)
            || !self.origins_recorded().all(|o| o != Origin::Entity(input))
        {
            return false;
        }
        let Ok(c) = model.closure(output_body) else {
            return false;
        };
        match input.id {
            crate::EntityId::Vertex(v) => c.vertices.contains(&v),
            crate::EntityId::Edge(e) => c.edges.contains(&e),
            crate::EntityId::Face(f) => c.faces.contains(&f),
            crate::EntityId::Shell(s) => c.shells.contains(&s),
            crate::EntityId::Body(b) => b == output_body.id,
        }
    }

    /// `true` when nothing was recorded.
    pub fn is_empty(&self) -> bool {
        self.generated.is_empty() && self.modified.is_empty() && self.deleted.is_empty()
    }

    /// The record of `self` followed by `next`, against `self`'s origins:
    /// an output of `self` that `next` modifies is replaced by its pieces
    /// and one `next` deletes is dropped, with the relations chained
    /// ([`Relation::then`]); one `next` generates from stays and gains
    /// the children; one `next` leaves alone stays; an input `self`
    /// modified whose every piece is gone is deleted; `next`'s records
    /// from anything else are carried as they are. Intermediate entities
    /// — outputs of `self` that `next` consumed — are nobody's input and
    /// appear nowhere. Associative over well-formed chains (an output is
    /// a new entity and a record names only what exists when it runs), so
    /// a chain of operations reports against its first inputs whatever
    /// the bracketing. The order nests (ADR-0009): the outputs that stand
    /// for piece `i` of an origin — what `next` generated from it, then
    /// its pieces, then the piece itself when it stays — come before
    /// those of piece `i + 1`, so `Split(k)` composes.
    pub fn then(&self, next: &Provenance) -> Provenance {
        let mine = self.outputs();
        let next_origins: BTreeSet<Shape> = next
            .origins_recorded()
            .filter_map(|o| match o {
                Origin::Entity(s) => Some(s),
                Origin::Role(_) => None,
            })
            .collect();
        let mut out = Provenance::default();
        let mut deleted: BTreeSet<Shape> = self.deleted.clone();
        deleted.extend(next.deleted.iter().filter(|s| !mine.contains(s)));
        for (relation, map) in [
            (Relation::Generated, &self.generated),
            (Relation::Modified, &self.modified),
        ] {
            for (origin, outputs) in map {
                for &x in outputs {
                    if next_origins.contains(&x) {
                        for (r2, list) in [
                            (Relation::Generated, next.generated_from(x)),
                            (Relation::Modified, next.modified_from(x)),
                        ] {
                            for &y in list {
                                out.add(relation.then(r2), *origin, y);
                            }
                        }
                    }
                    // Only a piece or a deletion replaces `x`; an entity
                    // that generated children is still there.
                    let consumed = next.deleted.contains(&x) || !next.modified_from(x).is_empty();
                    if !consumed {
                        out.add(relation, *origin, x);
                    }
                }
            }
        }
        for (relation, map) in [
            (Relation::Generated, &next.generated),
            (Relation::Modified, &next.modified),
        ] {
            for (origin, outputs) in map {
                let carried = match origin {
                    Origin::Entity(s) => !mine.contains(s),
                    Origin::Role(_) => true,
                };
                if carried {
                    for &y in outputs {
                        out.add(relation, *origin, y);
                    }
                }
            }
        }
        // Deletion follows `Modified` alone: an input `self` split into
        // pieces that `next` all consumed has no image of its own kind
        // left and is deleted, whatever it generated; an input `self`
        // only generated from is still there.
        for origin in self.modified.keys() {
            if let Origin::Entity(s) = origin {
                if out.modified_from(*origin).is_empty() {
                    deleted.insert(*s);
                }
            }
        }
        out.deleted = deleted;
        out
    }

    fn add(&mut self, relation: Relation, origin: Origin, output: Shape) {
        match relation {
            Relation::Generated => self.add_generated(origin, output),
            Relation::Modified => self.add_modified(origin, output),
            Relation::Deleted => self.add_deleted(output),
        }
    }

    /// The same record with every role origin `r` replaced by `f(r)`:
    /// how a consumer roots another operation's record — a primitive's,
    /// a sweep's, a file's — at names of its own, typically
    /// `Role::Consumer` keys that prefix its own key to the operation's
    /// part (ADR-0028). Entity origins, every output and `deleted` are
    /// untouched, and each role's outputs keep their order. Two roles `f`
    /// maps to one concatenate their outputs in the old roles' ascending
    /// order, without duplicates, so the result is deterministic for any
    /// `f`; but only an injective `f` keeps `Split(k)` meaning the same
    /// piece (ADR-0009), since a merged list no longer says which role a
    /// piece came from.
    ///
    /// ```
    /// use arris_topo::provenance::{BoxPart, ConsumerKey, Coord, Provenance, Role, Side};
    /// use arris_topo::{FaceId, Orientation, Shape};
    ///
    /// let top = Shape::new(FaceId::new(5, 0), Orientation::Forward);
    /// let role = Role::Box(BoxPart::Face(Coord::Z, Side::Max));
    /// let mut p = Provenance::default();
    /// p.add_generated(role, top);
    /// // The consumer's feature 12 made the box: its top is key 12·64 + 1.
    /// let lid = Role::Consumer(ConsumerKey { namespace: 3, key: 12 * 64 + 1 });
    /// let mine = p.rerooted(|r| if r == role { lid } else { r });
    /// assert_eq!(mine.generated_from(lid), [top]);
    /// assert!(mine.generated_from(role).is_empty());
    /// ```
    pub fn rerooted(&self, f: impl Fn(Role) -> Role) -> Provenance {
        let origin = |o: &Origin| match o {
            Origin::Entity(s) => Origin::Entity(*s),
            Origin::Role(r) => Origin::Role(f(*r)),
        };
        let mut out = Provenance::default();
        for (o, outputs) in &self.generated {
            for &y in outputs {
                out.add_generated(origin(o), y);
            }
        }
        for (o, outputs) in &self.modified {
            for &y in outputs {
                out.add_modified(origin(o), y);
            }
        }
        out.deleted = self.deleted.clone();
        out
    }

    /// The same record with every entity id translated through `map`
    /// (an [`IdMap`] from `import`); an id the map does not hold stays as
    /// it is, since an origin in another body is not imported with this
    /// one. Every origin's outputs keep their order.
    pub fn mapped(&self, map: &IdMap) -> Provenance {
        let translate = |s: Shape| map.map(s).unwrap_or(s);
        let origin = |o: &Origin| match o {
            Origin::Entity(s) => Origin::Entity(translate(*s)),
            Origin::Role(r) => Origin::Role(*r),
        };
        let mut out = Provenance::default();
        for (o, outputs) in &self.generated {
            for &y in outputs {
                out.add_generated(origin(o), translate(y));
            }
        }
        for (o, outputs) in &self.modified {
            for &y in outputs {
                out.add_modified(origin(o), translate(y));
            }
        }
        out.deleted = self.deleted.iter().map(|&s| translate(s)).collect();
        out
    }
}

impl fmt::Display for Provenance {
    /// One line per record, `<origin> generated <outputs…>`, then
    /// `<input> deleted`, in record order.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (origin, outputs) in &self.generated {
            write!(f, "{origin} generated")?;
            for o in outputs {
                write!(f, " {o}")?;
            }
            writeln!(f)?;
        }
        for (origin, outputs) in &self.modified {
            write!(f, "{origin} modified")?;
            for o in outputs {
                write!(f, " {o}")?;
            }
            writeln!(f)?;
        }
        for s in &self.deleted {
            writeln!(f, "{s} deleted")?;
        }
        Ok(())
    }
}

/// Why a [`Provenance`] does not account for the operation it claims to
/// record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum AuditError {
    /// A body named as an input or the output does not resolve.
    #[error(transparent)]
    NotFound(#[from] NotFound),
    /// An output entity has no origin and is not an unrecorded input kept
    /// by id.
    #[error("{0} is in the output with no origin and is not kept")]
    NoOrigin(Shape),
    /// An input entity is absent from the output, `Generated` nowhere,
    /// `Modified` nowhere and not `Deleted`.
    #[error("{0} is an input that is neither kept nor recorded")]
    Unaccounted(Shape),
    /// An input entity is both `Deleted` and `Modified`: a piece keeps
    /// its parent an image, never both.
    #[error("{0} is both deleted and modified")]
    DeletedAndModified(Shape),
    /// A `Deleted` entity is not one of the audited inputs.
    #[error("{0} is deleted but is not an input")]
    NotAnInput(Shape),
}

/// The accounting rule every operation's record keeps
/// (`docs/DATA-MODEL.md` §Provenance): every entity of every input
/// body's closure is either present in `output`'s closure and
/// unrecorded (kept by id) or recorded — `Generated`, `Modified` into
/// pieces, `Deleted`, or both `Deleted` and `Generated` from (a tool
/// face gone, its image the hole's wall) — never both `Deleted` and
/// `Modified`; every entity of `output`'s closure is either a kept input
/// or has an origin; and a `Deleted` entity is one of the inputs.
///
/// ```
/// use arris_topo::provenance::{audit, AuditError, Provenance};
/// use arris_topo::{Body, BodyId, Model};
///
/// let m = Model::default();
/// let missing = Body::forward(BodyId::new(0, 0));
/// assert!(matches!(
///     audit(&m, &[], missing, &Provenance::new()),
///     Err(AuditError::NotFound(_))
/// ));
/// ```
pub fn audit(
    model: &Model,
    inputs: &[Body],
    output: Body,
    provenance: &Provenance,
) -> Result<(), AuditError> {
    let entities = |body: Body| -> Result<BTreeSet<Shape>, AuditError> {
        let c = model.closure(body)?;
        let mut set: BTreeSet<Shape> = BTreeSet::new();
        set.extend(
            c.vertices
                .iter()
                .map(|&v| Shape::new(v, Orientation::Forward)),
        );
        set.extend(c.edges.iter().map(|&e| Shape::new(e, Orientation::Forward)));
        set.extend(c.faces.iter().map(|&f| Shape::new(f, Orientation::Forward)));
        set.extend(
            c.shells
                .iter()
                .map(|&s| Shape::new(s, Orientation::Forward)),
        );
        set.insert(Shape::new(body.id, Orientation::Forward));
        Ok(set)
    };
    let output = entities(output)?;
    let mut ins: BTreeSet<Shape> = BTreeSet::new();
    for &b in inputs {
        ins.extend(entities(b)?);
    }
    let recorded = |s: Shape| {
        !provenance.generated_from(s).is_empty()
            || !provenance.modified_from(s).is_empty()
            || provenance.is_deleted(s)
    };
    for &e in &output {
        let kept = ins.contains(&e) && !recorded(e);
        if !kept && provenance.origins(e).is_empty() {
            return Err(AuditError::NoOrigin(e));
        }
    }
    for &e in &ins {
        if !(output.contains(&e) && !recorded(e)) && !recorded(e) {
            return Err(AuditError::Unaccounted(e));
        }
        if provenance.is_deleted(e) && !provenance.modified_from(e).is_empty() {
            return Err(AuditError::DeletedAndModified(e));
        }
    }
    for s in provenance.deleted() {
        if !ins.contains(&s) {
            return Err(AuditError::NotAnInput(s));
        }
    }
    Ok(())
}
