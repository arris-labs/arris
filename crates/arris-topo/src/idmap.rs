//! The id map `import` returns: old id → new id, per kind.

use std::collections::BTreeMap;

use crate::handle::Shape;
use crate::id::{
    BodyId, Curve2Id, CurveId, EdgeId, EntityId, FaceId, GeometryId, ShellId, SurfaceId, VertexId,
};

/// Old id → new id for every entity and geometry value `Model::import`
/// copied, one ordered map per kind. What `Provenance::mapped` translates
/// a record through. On the wire each map is a sequence of pairs in key
/// order, so it has a JSON form.
///
/// ```
/// use arris_topo::{FaceId, IdMap, Orientation, Shape};
///
/// let mut map = IdMap::default();
/// map.faces.insert(FaceId::new(3, 0), FaceId::new(0, 0));
/// let old = Shape::new(FaceId::new(3, 0), Orientation::Reversed);
/// assert_eq!(map.map(old), Some(Shape::new(FaceId::new(0, 0), Orientation::Reversed)));
/// assert_eq!(map.map(Shape::new(FaceId::new(4, 0), Orientation::Forward)), None);
/// ```
#[derive(Debug, Clone, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct IdMap {
    /// Vertices.
    #[cfg_attr(feature = "serde", serde(with = "crate::pairs"))]
    pub vertices: BTreeMap<VertexId, VertexId>,
    /// Edges.
    #[cfg_attr(feature = "serde", serde(with = "crate::pairs"))]
    pub edges: BTreeMap<EdgeId, EdgeId>,
    /// Faces.
    #[cfg_attr(feature = "serde", serde(with = "crate::pairs"))]
    pub faces: BTreeMap<FaceId, FaceId>,
    /// Shells.
    #[cfg_attr(feature = "serde", serde(with = "crate::pairs"))]
    pub shells: BTreeMap<ShellId, ShellId>,
    /// Bodies.
    #[cfg_attr(feature = "serde", serde(with = "crate::pairs"))]
    pub bodies: BTreeMap<BodyId, BodyId>,
    /// Curves.
    #[cfg_attr(feature = "serde", serde(with = "crate::pairs"))]
    pub curves: BTreeMap<CurveId, CurveId>,
    /// Surfaces.
    #[cfg_attr(feature = "serde", serde(with = "crate::pairs"))]
    pub surfaces: BTreeMap<SurfaceId, SurfaceId>,
    /// Pcurves.
    #[cfg_attr(feature = "serde", serde(with = "crate::pairs"))]
    pub curve2s: BTreeMap<Curve2Id, Curve2Id>,
}

impl IdMap {
    /// The new id of an entity, `None` when it was not copied.
    pub fn map_entity(&self, id: EntityId) -> Option<EntityId> {
        Some(match id {
            EntityId::Vertex(v) => EntityId::Vertex(*self.vertices.get(&v)?),
            EntityId::Edge(e) => EntityId::Edge(*self.edges.get(&e)?),
            EntityId::Face(f) => EntityId::Face(*self.faces.get(&f)?),
            EntityId::Shell(s) => EntityId::Shell(*self.shells.get(&s)?),
            EntityId::Body(b) => EntityId::Body(*self.bodies.get(&b)?),
        })
    }

    /// The new id of a geometry value, `None` when it was not copied.
    pub fn map_geometry(&self, id: GeometryId) -> Option<GeometryId> {
        Some(match id {
            GeometryId::Curve(c) => GeometryId::Curve(*self.curves.get(&c)?),
            GeometryId::Surface(s) => GeometryId::Surface(*self.surfaces.get(&s)?),
            GeometryId::Curve2(p) => GeometryId::Curve2(*self.curve2s.get(&p)?),
        })
    }

    /// The same handle on the new entity, orientation kept; `None` when
    /// the entity was not copied.
    pub fn map(&self, shape: Shape) -> Option<Shape> {
        Some(Shape {
            id: self.map_entity(shape.id)?,
            orientation: shape.orientation,
        })
    }

    /// How many ids the map holds, all kinds together.
    pub fn len(&self) -> usize {
        self.vertices.len()
            + self.edges.len()
            + self.faces.len()
            + self.shells.len()
            + self.bodies.len()
            + self.curves.len()
            + self.surfaces.len()
            + self.curve2s.len()
    }

    /// `true` when nothing was copied.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// New id → old id: the way back. A map `import` returned is
    /// injective, so its inverse holds every pair; a map that sends two
    /// old ids to one new id keeps the greater old id.
    ///
    /// A consumer that imported entities into another model — a plugin's
    /// operands — translates the ids a record there names back to its own
    /// through this.
    ///
    /// ```
    /// use arris_topo::{FaceId, IdMap};
    ///
    /// let mut map = IdMap::default();
    /// map.faces.insert(FaceId::new(3, 0), FaceId::new(0, 0));
    /// map.faces.insert(FaceId::new(7, 1), FaceId::new(1, 0));
    /// let back = map.inverse();
    /// assert_eq!(back.faces[&FaceId::new(1, 0)], FaceId::new(7, 1));
    /// assert_eq!(back.inverse(), map);
    /// ```
    pub fn inverse(&self) -> IdMap {
        fn flip<K: Ord + Copy>(m: &BTreeMap<K, K>) -> BTreeMap<K, K> {
            m.iter().map(|(&a, &b)| (b, a)).collect()
        }
        IdMap {
            vertices: flip(&self.vertices),
            edges: flip(&self.edges),
            faces: flip(&self.faces),
            shells: flip(&self.shells),
            bodies: flip(&self.bodies),
            curves: flip(&self.curves),
            surfaces: flip(&self.surfaces),
            curve2s: flip(&self.curve2s),
        }
    }
}
