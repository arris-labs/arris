//! A bounding-volume hierarchy over a body's faces (ADR-0053): the
//! spatial index that ray fire, the classifier's ray cast and the other
//! C8 queries share.
//!
//! The leaves are each face's [`FaceDomain::bounds`], already inflated by
//! the face's tolerance. The build is a deterministic median split on the
//! longest axis, ties broken by [`FaceId`], and every answer comes back in
//! an order stated here, so no hash iteration reaches a geometric
//! decision.

use arris_math::{Aabb, Point3, Vec3};
use arris_topo::{Body, FaceId, Model, NotFound};

use crate::domain::FaceDomain;

/// The most faces a leaf holds before it is split.
const LEAF_SIZE: usize = 4;

/// One node: its box and either two children or a run of `items`.
#[derive(Debug, Clone)]
enum Node {
    Inner {
        bounds: Aabb,
        left: usize,
        right: usize,
    },
    Leaf {
        bounds: Aabb,
        first: usize,
        count: usize,
    },
}

impl Node {
    fn bounds(&self) -> &Aabb {
        match self {
            Node::Inner { bounds, .. } | Node::Leaf { bounds, .. } => bounds,
        }
    }
}

/// A hierarchy of face boxes over one body.
///
/// Guarantees: [`FaceTree::along`] and [`FaceTree::near`] return exactly
/// the faces a brute-force box test finds, in the order each states, and
/// the same on every run and platform. A face with no bounded box (a face
/// whose domain does not resolve, or one with an unbounded range) is in
/// every answer, since no box rejects it.
#[derive(Debug, Clone)]
pub struct FaceTree {
    /// Faces no box rejects, in `FaceId` order.
    unbounded: Vec<FaceId>,
    /// The bounded faces with their boxes, in leaf order.
    items: Vec<(FaceId, Aabb)>,
    nodes: Vec<Node>,
}

impl FaceTree {
    /// The hierarchy over every face of `body`'s closure, each box read
    /// from its [`FaceDomain`] at the model's parametric tolerance.
    /// Errors: the body does not resolve.
    ///
    /// ```
    /// use arris_check::tree::FaceTree;
    /// use arris_debug::sample;
    /// use arris_math::{Point3, Vec3};
    /// use arris_topo::Model;
    ///
    /// let mut m = Model::default();
    /// let body = sample::unit_box(&mut m)?;
    /// let tree = FaceTree::of_body(&m, body)?;
    /// // A ray down the middle of the box enters the top and bottom faces.
    /// let hit = tree.along(Point3::new(0.5, 0.5, 5.0), Vec3::new(0.0, 0.0, -1.0), (0.0, 10.0));
    /// assert!(hit.len() >= 2);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn of_body(model: &Model, body: Body) -> Result<Self, NotFound> {
        let tolerance = model.precision().parametric_tolerance;
        let boxes = model
            .closure(body)?
            .faces
            .into_iter()
            .map(|id| {
                let bounds = FaceDomain::of(model, id, tolerance)
                    .ok()
                    .and_then(|d| d.bounds());
                (id, bounds)
            })
            .collect();
        Ok(Self::of_boxes(boxes))
    }

    /// The hierarchy over `faces`, each with its box or `None` for a face
    /// no box rejects. Duplicate ids are kept as given.
    pub fn of_boxes(faces: Vec<(FaceId, Option<Aabb>)>) -> Self {
        let mut unbounded: Vec<FaceId> = Vec::new();
        let mut items: Vec<(FaceId, Aabb)> = Vec::new();
        for (id, bounds) in faces {
            match bounds {
                Some(b) => items.push((id, b)),
                None => unbounded.push(id),
            }
        }
        unbounded.sort_unstable();
        let mut nodes = Vec::new();
        if !items.is_empty() {
            let n = items.len();
            build(&mut items, 0, n, &mut nodes);
        }
        FaceTree {
            unbounded,
            items,
            nodes,
        }
    }

    /// How many faces the tree holds, bounded or not.
    pub fn len(&self) -> usize {
        self.items.len() + self.unbounded.len()
    }

    /// Whether the tree holds no face.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The faces whose boxes the segment `origin + t · direction`, `t` in
    /// `range`, enters, in ascending order of the `t` it enters at, ties
    /// by `FaceId`. The faces no box rejects come first, in `FaceId`
    /// order. A zero or non-finite `direction` yields the faces whose box
    /// holds `origin`.
    pub fn along(&self, origin: Point3, direction: Vec3, range: (f64, f64)) -> Vec<FaceId> {
        let mut found: Vec<(f64, FaceId)> = Vec::new();
        let mut stack: Vec<usize> = if self.nodes.is_empty() {
            vec![]
        } else {
            vec![0]
        };
        while let Some(at) = stack.pop() {
            let node = &self.nodes[at];
            if entry(node.bounds(), origin, direction, range).is_none() {
                continue;
            }
            match *node {
                Node::Inner { left, right, .. } => {
                    stack.push(right);
                    stack.push(left);
                }
                Node::Leaf { first, count, .. } => {
                    for (id, b) in &self.items[first..first + count] {
                        if let Some(t) = entry(b, origin, direction, range) {
                            found.push((t, *id));
                        }
                    }
                }
            }
        }
        found.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        let mut out = self.unbounded.clone();
        out.extend(found.into_iter().map(|(_, id)| id));
        out
    }

    /// The faces whose boxes overlap `region` (touching counts), in
    /// `FaceId` order, the faces no box rejects included.
    pub fn near(&self, region: &Aabb) -> Vec<FaceId> {
        let mut out = self.unbounded.clone();
        let mut stack: Vec<usize> = if self.nodes.is_empty() {
            vec![]
        } else {
            vec![0]
        };
        while let Some(at) = stack.pop() {
            let node = &self.nodes[at];
            if !node.bounds().intersects(region) {
                continue;
            }
            match *node {
                Node::Inner { left, right, .. } => {
                    stack.push(right);
                    stack.push(left);
                }
                Node::Leaf { first, count, .. } => {
                    out.extend(
                        self.items[first..first + count]
                            .iter()
                            .filter(|(_, b)| b.intersects(region))
                            .map(|(id, _)| *id),
                    );
                }
            }
        }
        out.sort_unstable();
        out
    }
}

/// Builds the node over `items[lo..hi]` and returns its index. Children
/// are pushed after their parent, so the root is node 0.
fn build(items: &mut [(FaceId, Aabb)], lo: usize, hi: usize, nodes: &mut Vec<Node>) -> usize {
    let bounds = items[lo..hi]
        .iter()
        .map(|(_, b)| *b)
        .reduce(Aabb::union)
        .unwrap_or(items[lo].1);
    let at = nodes.len();
    if hi - lo <= LEAF_SIZE {
        nodes.push(Node::Leaf {
            bounds,
            first: lo,
            count: hi - lo,
        });
        return at;
    }
    let extent = bounds.extent();
    let axis = (1..3).fold(0, |best, k| if extent[k] > extent[best] { k } else { best });
    items[lo..hi].sort_unstable_by(|(ia, a), (ib, b)| {
        let ca = a.min[axis] + a.max[axis];
        let cb = b.min[axis] + b.max[axis];
        ca.total_cmp(&cb).then(ia.cmp(ib))
    });
    let mid = lo + (hi - lo) / 2;
    // A placeholder keeps the parent ahead of its children.
    nodes.push(Node::Leaf {
        bounds,
        first: lo,
        count: 0,
    });
    let left = build(items, lo, mid, nodes);
    let right = build(items, mid, hi, nodes);
    nodes[at] = Node::Inner {
        bounds,
        left,
        right,
    };
    at
}

/// The parameter at which `origin + t · direction`, `t` in `range`, first
/// lies in `b`, or `None` when it never does (slab test).
fn entry(b: &Aabb, origin: Point3, direction: Vec3, range: (f64, f64)) -> Option<f64> {
    let (mut t0, mut t1) = range;
    if t0.is_nan() || t1.is_nan() {
        return None;
    }
    for k in 0..3 {
        let (o, d) = (origin[k], direction[k]);
        if d == 0.0 || !d.is_finite() {
            if !(b.min[k] <= o && o <= b.max[k]) {
                return None;
            }
            continue;
        }
        let (a, c) = ((b.min[k] - o) / d, (b.max[k] - o) / d);
        let (near, far) = if a <= c { (a, c) } else { (c, a) };
        t0 = t0.max(near);
        t1 = t1.min(far);
        if t0 > t1 {
            return None;
        }
    }
    Some(t0)
}
