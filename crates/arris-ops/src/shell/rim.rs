//! An opening's rim faces (ADR-0049 §3). Each loop of an opening and the
//! loop the skin cuts on its surface bound one rim face, an annulus. Where
//! two openings share an edge — or an opening meets itself across its
//! seam — the body's edge and the skin's copy of it lie on one curve and
//! are walked opposite ways: the stretch they share cancels and lies in no
//! face, and what is left of either at each end is an edge of its own,
//! shared by the two rims. The rims' loops are spliced there into one
//! loop, and on a plane the loops left are grouped into faces by winding.

use std::collections::{BTreeMap, BTreeSet};

use arris_geom::region2::{Piece as Piece2, Polygon2, discretise};
use arris_geom::{GeomKind, Surface};
use arris_math::{Interval, Tolerance};
use arris_topo::builder::{EdgeKey, EdgeSpec, VertexKey};
use arris_topo::entity::EdgeGeometry;
use arris_topo::{CurveId, EdgeId, FaceId, Model, Orientation, VertexId};

use super::ShellSide;
use crate::body_view::BodyView;
use crate::error::{Fault, OffsetReason, OpError, Reason};
use crate::rebuild::{Rewrite, StoredUse, forward};

/// A piece of an edge two openings share, left at one end once the
/// body's edge and the skin's copy cancel: an edge of the result between
/// the two rims.
#[derive(Debug, Clone, Copy)]
pub(super) struct Piece {
    pub(super) spec: EdgeSpec,
    /// The body's edge it is a piece of.
    pub(super) parent: EdgeId,
    /// Whether it lies on the body's edge — `Modified` from it — rather
    /// than beyond it on the skin's copy, `Generated` from it.
    pub(super) on_parent: bool,
}

/// The pieces made so far, each made once and shared by both rims that
/// meet along it: addressed after the rewrite's own edges.
pub(super) struct Pieces {
    base: usize,
    pub(super) list: Vec<Piece>,
    by_end: BTreeMap<(EdgeId, VertexId), usize>,
}

impl Pieces {
    /// No pieces yet, the first to be addressed as `EdgeKey::New(base)`.
    pub(super) fn new(base: usize) -> Self {
        Pieces {
            base,
            list: Vec::new(),
            by_end: BTreeMap::new(),
        }
    }
}

/// Where a use starts or ends: the parameter on its edge's curve and the
/// vertex there.
#[derive(Debug, Clone, Copy)]
struct End {
    t: f64,
    vertex: VertexKey,
}

/// A stored use's edge read: its curve, range, ends in walking order.
struct Walked {
    curve: Option<CurveId>,
    range: Interval,
    from: End,
    to: End,
}

fn invariant(what: &'static str) -> OpError {
    OpError::Internal(Fault::Invariant { what })
}

fn walked(m: &Model, rw: &Rewrite, u: &StoredUse) -> Result<Walked, OpError> {
    let (geometry, start, end) = match u.edge {
        EdgeKey::Kept(e) => {
            let edge = m.edge(e)?;
            (
                edge.geometry(),
                VertexKey::Kept(edge.start()),
                VertexKey::Kept(edge.end()),
            )
        }
        EdgeKey::New(k) => match rw.edges.get(k) {
            Some((
                EdgeSpec::New {
                    geometry,
                    start,
                    end,
                    ..
                },
                _,
            )) => (*geometry, *start, *end),
            _ => return Err(invariant("a skin edge appended")),
        },
    };
    let (curve, range) = match geometry {
        EdgeGeometry::Curve { curve, range } => (Some(curve), range),
        EdgeGeometry::Degenerate { range } => (None, range),
    };
    let lo = End {
        t: range.lo(),
        vertex: start,
    };
    let hi = End {
        t: range.hi(),
        vertex: end,
    };
    let (from, to) = match u.orientation {
        Orientation::Forward => (lo, hi),
        Orientation::Reversed => (hi, lo),
    };
    Ok(Walked {
        curve,
        range,
        from,
        to,
    })
}

/// One loop of the rim before splicing: the loop walked as stored and the
/// one walked back, position by position the same edge's uses, and at each
/// position the pieces left where the two cancel.
struct Pair {
    forth: Vec<StoredUse>,
    back: Vec<StoredUse>,
    cancels: Vec<bool>,
    /// From where the stored walk enters a cancelled position to where
    /// the walk back leaves it; and from where the walk back enters it to
    /// where the stored walk leaves it.
    first: Vec<Option<StoredUse>>,
    second: Vec<Option<StoredUse>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Item {
    Forth(usize),
    Back(usize),
    First(usize),
    Second(usize),
}

impl Pair {
    fn next(&self, j: usize) -> usize {
        (j + 1) % self.forth.len()
    }

    fn prev(&self, j: usize) -> usize {
        (j + self.forth.len() - 1) % self.forth.len()
    }

    /// The item the splice continues with on arriving at the start of the
    /// stored walk's position `j` (`forth`) or the walk back's (`!forth`).
    fn enter(&self, forth: bool, j: usize) -> Result<Item, OpError> {
        let (mut forth, mut j) = (forth, j);
        for _ in 0..=2 * self.forth.len() {
            match forth {
                true if !self.cancels[j] => return Ok(Item::Forth(j)),
                true if self.first[j].is_some() => return Ok(Item::First(j)),
                true => (forth, j) = (false, self.prev(j)),
                false if !self.cancels[j] => return Ok(Item::Back(j)),
                false if self.second[j].is_some() => return Ok(Item::Second(j)),
                false => (forth, j) = (true, self.next(j)),
            }
        }
        Err(invariant("a rim loop that does not close"))
    }

    fn succ(&self, item: Item) -> Result<Item, OpError> {
        match item {
            Item::Forth(j) | Item::Second(j) => self.enter(true, self.next(j)),
            Item::Back(j) | Item::First(j) => self.enter(false, self.prev(j)),
        }
    }

    fn use_of(&self, item: Item) -> Result<StoredUse, OpError> {
        let missing = || invariant("a rim piece made");
        Ok(match item {
            Item::Forth(j) => self.forth[j],
            Item::Back(j) => StoredUse {
                orientation: self.back[j].orientation.flipped(),
                ..self.back[j]
            },
            Item::First(j) => self.first[j].ok_or_else(missing)?,
            Item::Second(j) => self.second[j].ok_or_else(missing)?,
        })
    }

    /// The spliced loops: every item once, in the walk's order.
    fn loops(&self) -> Result<Vec<Vec<StoredUse>>, OpError> {
        let n = self.forth.len();
        let mut items: Vec<Item> = Vec::new();
        for j in 0..n {
            if !self.cancels[j] {
                items.extend([Item::Forth(j), Item::Back(j)]);
            }
            if self.first[j].is_some() {
                items.push(Item::First(j));
            }
            if self.second[j].is_some() {
                items.push(Item::Second(j));
            }
        }
        let mut seen: BTreeSet<Item> = BTreeSet::new();
        let mut loops = Vec::new();
        for &start in &items {
            if seen.contains(&start) {
                continue;
            }
            let mut walk = Vec::new();
            let mut at = start;
            loop {
                if !seen.insert(at) {
                    return Err(invariant("a rim item walked once"));
                }
                walk.push(self.use_of(at)?);
                at = self.succ(at)?;
                if at == start {
                    break;
                }
            }
            loops.push(walk);
        }
        Ok(loops)
    }
}

/// The rim faces of the opening `face`, each its loops in the stored
/// sense of the opening's surface: one face with the body's loops and the
/// skin's walked back where it shares no edge with an opening, else the
/// spliced loops, grouped into faces — none where the skin's loop is the
/// body's.
#[allow(clippy::too_many_arguments)]
pub(super) fn faces(
    m: &Model,
    rw: &Rewrite,
    view: &BodyView,
    open: &BTreeSet<FaceId>,
    face: FaceId,
    side: ShellSide,
    pieces: &mut Pieces,
    tol: Tolerance,
) -> Result<Vec<Vec<Vec<StoredUse>>>, OpError> {
    let entity = m.face(face)?;
    // An opening the rewrite leaves alone has no vertex that moves: its
    // loops on the skin are its own.
    let new_loops = rw.faces.get(&face);
    let mut pairs = Vec::with_capacity(entity.loops().len());
    for (i, l) in entity.loops().iter().enumerate() {
        let old: Vec<StoredUse> = l
            .coedges()
            .iter()
            .map(|c| StoredUse {
                edge: EdgeKey::Kept(c.edge()),
                orientation: c.orientation(),
                pcurve: c.pcurve(),
            })
            .collect();
        let new = match new_loops {
            Some(loops) => loops
                .get(i)
                .cloned()
                .ok_or(invariant("an opening's loop on the skin"))?,
            None => old.clone(),
        };
        if new.len() != old.len() {
            return Err(invariant("a skin loop use for use the body's"));
        }
        // The larger loop as stored, the other walked back: the body's for
        // an inward wall, the skin's for an outward.
        let (forth, back) = match side {
            ShellSide::Inward => (old.clone(), new),
            ShellSide::Outward => (new, old.clone()),
        };
        let n = old.len();
        let mut pair = Pair {
            forth,
            back,
            cancels: vec![false; n],
            first: vec![None; n],
            second: vec![None; n],
        };
        for (j, o) in old.iter().enumerate() {
            let EdgeKey::Kept(e) = o.edge else {
                return Err(invariant("a body loop over the body's edges"));
            };
            let between_openings = view
                .uses
                .get(&e)
                .is_some_and(|uses| uses.iter().all(|u| open.contains(&u.face)));
            if !between_openings {
                continue;
            }
            pair.cancels[j] = true;
            cancel(m, rw, view, open, &mut pair, j, side, e, face, pieces, tol)?;
        }
        pairs.push(pair);
    }
    if pairs.iter().all(|p| !p.cancels.iter().any(|&c| c)) {
        let loops = pairs
            .into_iter()
            .flat_map(|p| [p.forth, walked_back(&p.back)])
            .collect();
        return Ok(vec![loops]);
    }
    let mut loops = Vec::new();
    for p in &pairs {
        loops.extend(p.loops()?);
    }
    if loops.len() <= 1 {
        return Ok(loops.into_iter().map(|l| vec![l]).collect());
    }
    group(m, rw, face, loops, tol)
}

/// The pieces left at position `j` of `pair`, where the body's edge `e`
/// and the skin's copy cancel, made once each and shared with the rim on
/// the other side of `e`.
#[allow(clippy::too_many_arguments)]
fn cancel(
    m: &Model,
    rw: &Rewrite,
    view: &BodyView,
    open: &BTreeSet<FaceId>,
    pair: &mut Pair,
    j: usize,
    side: ShellSide,
    e: EdgeId,
    face: FaceId,
    pieces: &mut Pieces,
    tol: Tolerance,
) -> Result<(), OpError> {
    let (f, k) = (pair.forth[j], pair.back[j]);
    if f.edge == k.edge {
        // Nothing at the edge moved: it cancels whole.
        return Ok(());
    }
    let edge = m.edge(e)?;
    let (wf, wk) = (walked(m, rw, &f)?, walked(m, rw, &k)?);
    let parent = |u: &StoredUse| match u.edge {
        EdgeKey::New(i) => rw.edges.get(i).and_then(|(_, p)| *p),
        EdgeKey::Kept(id) => Some(id),
    };
    let same_curve = wf.curve.is_some() && wf.curve == wk.curve;
    if !same_curve || parent(&f) != Some(e) || parent(&k) != Some(e) || edge.is_closed() {
        // The skin's copy off the body's curve, or a closed edge whose
        // vertex moved along it: not a stretch that cancels.
        let other = view
            .uses
            .get(&e)
            .and_then(|uses| uses.iter().map(|u| u.face).find(|&g| g != face))
            .filter(|g| open.contains(g))
            .unwrap_or(face);
        let kind = |g: FaceId| -> Result<GeomKind, OpError> {
            Ok(GeomKind::Surface(m.surface(m.face(g)?.surface())?.kind()))
        };
        return Err(OpError::Unsupported {
            a: (kind(face)?, forward(face)),
            b: (kind(other)?, forward(other)),
        });
    }
    let old_range = edge.range();
    let tolerance = edge.tolerance().max(tol.linear);
    let forth_is_old = side == ShellSide::Inward;
    let (old_use, new_use) = if forth_is_old { (f, k) } else { (k, f) };
    // `first` runs from the stored walk's start to the walk back's end
    // (the stored back use's start), `second` from the walk back's start
    // (the stored back use's end) to the stored walk's end; each as
    // (from, to, its end on the stored walk, its end on the walk back).
    let ends = [
        (wf.from, wk.from, wf.from, wk.from),
        (wk.to, wf.to, wf.to, wk.to),
    ];
    let mut made = [None, None];
    for (slot, &(from, to, on_forth, on_back)) in made.iter_mut().zip(&ends) {
        if from.vertex == to.vertex {
            continue;
        }
        let (old_end, other) = if forth_is_old {
            (on_forth, on_back)
        } else {
            (on_back, on_forth)
        };
        let VertexKey::Kept(old_end) = old_end.vertex else {
            return Err(invariant("a piece ending at the body's vertex"));
        };
        let on_parent =
            other.t >= old_range.lo() - tolerance && other.t <= old_range.hi() + tolerance;
        let (lo, hi) = (from.t.min(to.t), from.t.max(to.t));
        if hi - lo <= tolerance {
            return Err(OpError::Degenerate {
                entities: vec![forward(e)],
                reason: Reason::Offset(OffsetReason::Vanishes),
            });
        }
        let index = match pieces.by_end.get(&(e, old_end)) {
            Some(&index) => index,
            None => {
                let curve = wf.curve.ok_or(invariant("a curve under a shared edge"))?;
                let range =
                    Interval::new(lo, hi).map_err(|_| invariant("a piece's range ordered"))?;
                let (start, end) = if from.t < to.t {
                    (from.vertex, to.vertex)
                } else {
                    (to.vertex, from.vertex)
                };
                pieces.list.push(Piece {
                    spec: EdgeSpec::New {
                        geometry: EdgeGeometry::Curve { curve, range },
                        start,
                        end,
                        tolerance: edge.tolerance(),
                    },
                    parent: e,
                    on_parent,
                });
                pieces.by_end.insert((e, old_end), pieces.list.len() - 1);
                pieces.list.len() - 1
            }
        };
        *slot = Some(StoredUse {
            edge: EdgeKey::New(pieces.base + index),
            orientation: if from.t < to.t {
                Orientation::Forward
            } else {
                Orientation::Reversed
            },
            pcurve: if on_parent {
                old_use.pcurve
            } else {
                new_use.pcurve
            },
        });
    }
    let [first, second] = made;
    pair.first[j] = first;
    pair.second[j] = second;
    Ok(())
}

/// The spliced `loops` of the opening `face` grouped into faces: on a
/// plane each loop turning counter-clockwise in (u, v) bounds a face, and
/// each turning clockwise is a hole of the innermost one around it. A
/// loop elsewhere is outside this release ([`OpError::Unsupported`]).
fn group(
    m: &Model,
    rw: &Rewrite,
    face: FaceId,
    loops: Vec<Vec<StoredUse>>,
    tol: Tolerance,
) -> Result<Vec<Vec<Vec<StoredUse>>>, OpError> {
    let surface = m.surface(m.face(face)?.surface())?;
    if !matches!(surface, Surface::Plane { .. }) {
        let kind = GeomKind::Surface(surface.kind());
        return Err(OpError::Unsupported {
            a: (kind, forward(face)),
            b: (kind, forward(face)),
        });
    }
    let mut polygons: Vec<Polygon2> = Vec::with_capacity(loops.len());
    for l in &loops {
        let mut parts = Vec::with_capacity(l.len());
        for u in l {
            parts.push((m.curve2(u.pcurve)?, walked(m, rw, u)?.range, u.orientation));
        }
        let pieces: Vec<Piece2<'_>> = parts
            .iter()
            .map(|&(curve, range, o)| match o {
                Orientation::Forward => Piece2::along(curve, range),
                Orientation::Reversed => Piece2::against(curve, range),
            })
            .collect();
        polygons.push(discretise(&pieces, tol.linear));
    }
    let outer: Vec<usize> = (0..loops.len())
        .filter(|&i| polygons[i].signed_area() > 0.0)
        .collect();
    let mut faces: Vec<Vec<Vec<StoredUse>>> =
        outer.iter().map(|&i| vec![loops[i].clone()]).collect();
    for (i, l) in loops.iter().enumerate() {
        if outer.contains(&i) {
            continue;
        }
        let at = *polygons[i]
            .points()
            .first()
            .ok_or(invariant("a hole with points"))?;
        let host = outer
            .iter()
            .enumerate()
            .filter(|&(_, &o)| polygons[o].winding_number(at) != 0)
            .min_by(|a, b| {
                polygons[*a.1]
                    .signed_area()
                    .total_cmp(&polygons[*b.1].signed_area())
            })
            .map(|(k, _)| k)
            .ok_or(invariant("a rim hole inside a rim face"))?;
        faces[host].push(l.clone());
    }
    Ok(faces)
}

/// A loop in the stored sense walked the other way: its uses in reverse
/// order, each flipped, over the same pcurves.
fn walked_back(uses: &[StoredUse]) -> Vec<StoredUse> {
    uses.iter()
        .rev()
        .map(|u| StoredUse {
            orientation: u.orientation.flipped(),
            ..*u
        })
        .collect()
}
