// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0
//! An implementation of the [simple stupid funnel algorithm](https://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html).

use approx::AbsDiffEq;
use core::mem;
use num_traits::Num;

use crate::math::*;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub struct SimpleFunnel<K, D = ()> {
    pub apex: ([K; 2], D),
    pub lhs: ([K; 2], D),
    pub rhs: ([K; 2], D),
}

impl<K, D> SimpleFunnel<K, D>
where
    K: AbsDiffEq + Clone + Num + PartialOrd,
    <K as AbsDiffEq>::Epsilon: Clone,
    D: Clone,
{
    pub fn new(apex: ([K; 2], D)) -> Self {
        Self {
            apex: apex.clone(),
            lhs: apex.clone(),
            rhs: apex,
        }
    }

    /// ## Well-formed `portal`s
    ///
    /// Note that consecutive `portal`s, and the current portal formed by `[self.lhs, self.rhs]`
    /// and the currently passed `portal` aren't allowed to intersect anywhere except at their
    /// endpoints, otherwise this calculation breaks down / yields usless results.
    ///
    /// If routing across the faces of some subdivision of 2D space, this means passing
    /// edges of a _single_ triangulation (with faces being triangles) yields valid funnels,
    /// and similarly, using 2D polygonal complices (complexes) with convex faces and passing
    /// their edges into this method also works.
    pub fn advance(
        &mut self,
        epsilon: <K as AbsDiffEq>::Epsilon,
        portal: [([K; 2], D); 2],
    ) -> Option<(([K; 2], D), &([K; 2], D))> {
        let apex = &mut self.apex;
        macro_rules! update_vertex {
            ($ahs:expr, $bhs:expr, $bportal:expr, $flip:expr) => {{
                let comp = |v: K| {
                    if $flip { v > K::zero() } else { v < K::zero() }
                };
                if !comp(triarea2(&apex.0, &$bhs.0, &$bportal.0)) {
                    if apex.0.abs_diff_eq(&$bhs.0, epsilon.clone())
                        || comp(triarea2(&apex.0, &$ahs.0, &$bportal.0))
                    {
                        // Tighten the funnel
                        *$bhs = $bportal.clone();
                    } else {
                        // B over A, insert A to path and restart scan from portal A point
                        let old_apex = mem::replace(apex, (*$ahs).clone());
                        *$bhs = apex.clone();
                        if old_apex.0 != apex.0 {
                            return Some((old_apex, apex));
                        }
                    }
                }
                None
            }};
        }

        // Update right vertex
        if let Some(ret) = update_vertex!(&mut self.lhs, &mut self.rhs, &portal[1], false) {
            return Some(ret);
        }

        // Update left vertex
        if let Some(ret) = update_vertex!(&mut self.rhs, &mut self.lhs, &portal[0], true) {
            return Some(ret);
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::SimpleFunnel;

    #[test]
    fn t_simple() {
        let mut x = SimpleFunnel::<i32, u8>::new(([0, 0], 0));
        assert_eq!(x.advance(0, [([30, 0], 1), ([10, 40], 2)]), None);
        assert_eq!(x.advance(0, [([60, 20], 3), ([60, 40], 4)]), None);
        assert_eq!(
            x.advance(0, [([70, -10], 5), ([90, 20], 6)]),
            Some((([0, 0], 0), &([60, 20], 3)))
        );
        assert_eq!(x.advance(0, [([80, -10], 7), ([80, -10], 8)]), None);
    }
}
