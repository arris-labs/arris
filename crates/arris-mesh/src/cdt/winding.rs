//! The winding number of the polygons around every triangle.

use std::collections::VecDeque;

use super::mesh::*;
use super::*;

impl Mesh {
    /// The winding number of the polygons around every alive triangle,
    /// counted exactly: zero at the bounding triangle's corners, and one
    /// more each time a constraint is crossed from its right to its left
    /// (into the region a counter-clockwise polygon bounds), one less the
    /// other way. Path-independent because every polygon is closed; a
    /// triangle reached with two different counts is a structural fault.
    pub(super) fn windings(&self) -> Result<Vec<Option<i32>>, CdtError> {
        let mut winding: Vec<Option<i32>> = vec![None; self.tris.len()];
        let start = self.vertex_tri[self.points.len() - 3];
        if !self.tris[start].alive {
            return Err(CdtError::Internal(
                "the bounding triangle's corner has no triangle",
            ));
        }
        winding[start] = Some(0);
        let mut queue = VecDeque::from([start]);
        while let Some(t) = queue.pop_front() {
            // Only triangles with a winding are queued.
            let w = winding[t].unwrap_or(0);
            let Tri { v, n, .. } = self.tris[t];
            for i in 0..3 {
                let Some(t2) = n[i] else {
                    continue;
                };
                let (x, y) = (v[i], v[(i + 1) % 3]);
                let delta = match self.constrained.get(&key(x, y)) {
                    None => 0,
                    Some(c) if (c.from, c.to) == (x, y) => -1,
                    Some(_) => 1,
                };
                match winding[t2] {
                    None => {
                        winding[t2] = Some(w + delta);
                        queue.push_back(t2);
                    }
                    Some(seen) if seen != w + delta => {
                        return Err(CdtError::Internal(
                            "the constraints do not close: two winding numbers for one triangle",
                        ));
                    }
                    Some(_) => {}
                }
            }
        }
        Ok(winding)
    }
}
