//! Disjoint sets over `0..n`: the connected components of a graph the
//! caller walks, labelled the same way on every platform.

/// Disjoint sets over `0..n`, the lowest member of a set its root, so a
/// numbering derived from the roots depends on the elements' order and on
/// nothing else.
///
/// ```
/// use arris_math::DisjointSets;
///
/// let mut sets = DisjointSets::new(5);
/// assert!(sets.union(3, 1));
/// assert!(sets.union(4, 3));
/// assert!(!sets.union(1, 4)); // already together
/// assert_eq!((sets.find(4), sets.find(3), sets.find(2)), (1, 1, 2));
/// ```
#[derive(Debug, Clone)]
pub struct DisjointSets(Vec<usize>);

impl DisjointSets {
    /// `n` singletons.
    pub fn new(n: usize) -> Self {
        DisjointSets((0..n).collect())
    }

    /// The root of `i`'s set: its lowest member. Halves the path it walks.
    ///
    /// Panics if `i` is not below the `n` of [`DisjointSets::new`]: an
    /// index is the caller's own, never geometry.
    pub fn find(&mut self, mut i: usize) -> usize {
        while self.0[i] != i {
            self.0[i] = self.0[self.0[i]];
            i = self.0[i];
        }
        i
    }

    /// Joins the sets of `a` and `b`; `false` if they were one already.
    pub fn union(&mut self, a: usize, b: usize) -> bool {
        let (a, b) = (self.find(a), self.find(b));
        if a == b {
            return false;
        }
        self.0[a.max(b)] = a.min(b);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// Against a naive relabelling partition: the same sets, each
        /// rooted at its least member.
        #[test]
        fn matches_a_naive_partition(
            n in 1usize..40,
            pairs in prop::collection::vec((0usize..40, 0usize..40), 0..60),
        ) {
            let mut sets = DisjointSets::new(n);
            let mut label: Vec<usize> = (0..n).collect();
            for (a, b) in pairs {
                let (a, b) = (a % n, b % n);
                let merged = sets.union(a, b);
                let (la, lb) = (label[a], label[b]);
                prop_assert_eq!(merged, la != lb);
                let (keep, drop) = (la.min(lb), la.max(lb));
                for l in &mut label {
                    if *l == drop {
                        *l = keep;
                    }
                }
            }
            for (i, &l) in label.iter().enumerate() {
                prop_assert_eq!(sets.find(i), l);
            }
        }
    }
}
