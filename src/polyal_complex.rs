// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0
//
//! An implementation of polyal complexes (polygonal complexes with discrete covering layers)
//!
//! There are two equivalent representations of slots/circle segments between neighbors of a point:
//! * [`OpenTriangle`], denoting a slot by the anchoring point and two neighbors
//! * `{ anchor: usize, neigh: usize }`, denoting a slot by the anchoring point and the preceding neighbor
//!
//! The user can convert between these two representations via [`PolygonalComplex::find_next_open_triangle`] and [`PolygonalComplex::resolve_open_triangle`].

use core::mem;
use stable_vec::StableVec;

pub struct PolygonalComplex<PT> {
    /// The points of the polyal complex,
    /// with attached data `.0 : PT` and edges to the other points `.1` noted counter-clockwise.
    points: StableVec<(PT, Box<[usize]>)>,
}

/// An open triangle, denoting the circle segment between `.0 .. .1` and `.1 .. .2`
/// in mathematical positive direction (counter-clockwise).
pub struct OpenTriangle(pub usize, pub usize, pub usize);

impl<PT> PolygonalComplex<PT> {
    /// Creates a polygonal complex consisting of two connected points.
    pub fn new(a: PT, b: PT) -> Self {
        let mut points = StableVec::with_capacity(2);
        points.push((a, vec![1].into_boxed_slice()));
        points.push((b, vec![0].into_boxed_slice()));
        Self {
            points,
        }
    }

    /// The points of the polyal complex,
    /// with attached data `.0 : PT` and edges to the other points `.1` noted counter-clockwise.
    #[inline]
    pub fn points(&self) -> &StableVec<(PT, Box<[usize]>)> {
        &self.points
    }

    /// Given an `anchor` point, find the open triangle
    /// that follows counter-clockwise after an edge to a `neigh`bor.
    pub fn find_next_open_triangle(&self, anchor: usize, neigh: usize) -> Option<OpenTriangle> {
        let ptneighs = &self.points.get(anchor)?.1;
        let mut it = ptneighs.iter();
        it.position(|i| i == &neigh)?;
        it.next().map(|j| OpenTriangle(*j, anchor, neigh))
    }

    /// Resolve `ot` to a position in the neighbor vector of the point `ot.1`.
    pub fn resolve_open_triangle(&self, ot: &OpenTriangle) -> Option<usize> {
        let ptneighs = &self.points.get(ot.1)?.1;
        let pos2 = ptneighs.iter().position(|i| i == &ot.2)?;
        if ptneighs[(pos2 + 1) % ptneighs.len()] == ot.0 {
            Some(pos2)
        } else {
            None
        }
    }

    /// Create an edge between `a.1` and `b.1`, oriented to land inside the circle segments `a` and `b`.
    /// Returns `false` if `a` or `b` aren't present in the complex, or the edge already exists.
    pub fn split_polygon(&mut self, a: &OpenTriangle, b: &OpenTriangle) -> bool {
        let Some(a_res) = self.resolve_open_triangle(a) else {
            return false;
        };
        let Some(b_res) = self.resolve_open_triangle(b) else {
            return false;
        };
        // PANIC SAFETY:
        // If the given points were invalid, `resolve_open_triangle` would've failed.
        if self.points[a.1].1.contains(&b.1) || self.points[b.1].1.contains(&a.1) {
            return false;
        }
        edit_point_neighs(&mut self.points[a.1].1, |tmp| tmp.insert(a_res, b.1));
        edit_point_neighs(&mut self.points[b.1].1, |tmp| tmp.insert(b_res, a.1));
        true
    }

    /// Subdivide an edge between `from` and `to`, and associating the `new_data` to the newly created point.
    /// Returns the index of the newly created point if successful.
    /// Fails (and returns the unused `new_data`) if there is no edge between `from` and `to`.
    pub fn subdivide_edge<I>(&mut self, mut from: usize, mut to: usize, new_data: I) -> Result<Vec<usize>, I>
    where
        I: Iterator<Item = PT>,
    {
        // Make ordering canonical
        if from > to {
            mem::swap(&mut from, &mut to);
        }
        let Some(pos_in_from) = self.points.get(from).and_then(|i| i.1.iter().position(|j| j == &to)) else {
            return Err(new_data);
        };
        let Some(pos_in_to) = self.points.get(to).and_then(|i| i.1.iter().position(|j| j == &from)) else {
            return Err(new_data);
        };
        let neighs_of_new_points = vec![from, to].into_boxed_slice();
        let new_indices: Vec<_> = new_data.map(|new_data| self.points.push((new_data, neighs_of_new_points.clone()))).collect();
        // Never replace an edge with no edge, just do nothing instead.
        if !new_indices.is_empty() {
            edit_point_neighs(&mut self.points[from].1, |tmp| {
                let _ = tmp.splice(pos_in_from..(pos_in_from + 1), new_indices.iter().copied());
            });
            edit_point_neighs(&mut self.points[to].1, |tmp| {
                let _ = tmp.splice(pos_in_to..(pos_in_to + 1), new_indices.iter().copied().rev());
            });
        }
        Ok(new_indices)
    }

    /// Merge two points `from` and `to` along their connecting edge.
    /// Returns the index of the newly created point and the old data if successful.
    pub fn merge_along_edge<F>(&mut self, mut from: usize, mut to: usize, make_new_data: F) -> Option<(usize, [PT; 2])>
    where
        F: FnOnce(&PT, &PT) -> PT,
    {
        // Make ordering canonical
        let did_swap = if from > to {
            mem::swap(&mut from, &mut to);
            true
        } else {
            false
        };

        let Some(pos_in_from) = self.points.get(from)?.1.iter().position(|i| i == &to) else {
            return None;
        };
        let Some(pos_in_to) = self.points.get(to)?.1.iter().position(|i| i == &from) else {
            return None;
        };

        let (mut data_from, mut neighs_from) = self.points.remove(from)?;
        neighs_from.rotate_left(pos_in_from);
        let (mut data_to, mut neighs_to) = self.points.remove(to)?;
        neighs_to.rotate_left(pos_in_to);

        let mut new_neighs: Vec<_> = neighs_from[1..].iter().chain(neighs_to[1..].iter()).copied().collect();
        new_neighs.rotate_right(pos_in_from);

        // Remove duplicate links
        new_neighs.dedup();
        if new_neighs.len() > 1 && new_neighs.first() == new_neighs.last() {
            new_neighs.pop();
        }

        if did_swap {
            mem::swap(&mut data_from, &mut data_to);
        }

        let new_index = self.points.push((make_new_data(&data_from, &data_to), new_neighs.into_boxed_slice()));

        Some((new_index, [data_from, data_to]))
    }
}

fn edit_point_neighs<R>(neighs: &mut Box<[usize]>, f: impl FnOnce(&mut Vec<usize>) -> R) -> R {
    let mut tmp = neighs.to_vec();
    let ret = f(&mut tmp);
    *neighs = tmp.into_boxed_slice();
    ret
}

/*
pub struct PolyalComplex<L, PT> {
    pgon_cplx: PolygonalComplex<PT>,
    layers: BTreeMap<L, Vec<usize>>,
}

impl<PT> AsRef<PolygonalComplex<PT>> for PolyalComplex {
    #[inline]
    fn as_ref(&self) -> &PolygonalComplex<PT> {
        &self.pgon_cplx
    }
}
*/

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_insert_hole() {
        let mut pgc = PolygonalComplex::new((), ());

        let x = pgc.subdivide_edge(0, 1, vec![(), ()].into_iter()).unwrap();
        assert_eq!(x, [2, 3]);

        assert!(pgc.split_polygon(
            &OpenTriangle(2, 0, 3),
            &OpenTriangle(3, 1, 2),
        ));

        let x = pgc.subdivide_edge(0, 1, vec![()].into_iter()).unwrap();
        assert_eq!(x, [4]);
        let x = pgc.subdivide_edge(4, 1, vec![()].into_iter()).unwrap();
        assert_eq!(x, [5]);

        let x = pgc.subdivide_edge(4, 5, vec![(), ()].into_iter()).unwrap();
        assert_eq!(x, [6, 7]);

        assert_eq!(pgc.points(), &{
            let mut expected = StableVec::new();
            let inp: &[&[usize]] = &[
                &[2, 4, 3],
                &[3, 5, 2],
                &[0, 1],
                &[0, 1],
                &[0, 6, 7],
                &[1, 7, 6],
                &[4, 5],
                &[4, 5],
            ];
            for i in inp {
                expected.push(((), i.to_vec().into_boxed_slice()));
            }
            expected
        });
    }

    #[test]
    fn test_divide_the_land() {
        let mut pgc = PolygonalComplex::new((), ());

        let x = pgc.subdivide_edge(0, 1, vec![(), ()].into_iter()).unwrap();
        assert_eq!(x, [2, 3]);

        let x = pgc.subdivide_edge(0, 3, vec![()].into_iter()).unwrap();
        assert_eq!(x, [4]);

        let x = pgc.subdivide_edge(1, 2, vec![()].into_iter()).unwrap();
        assert_eq!(x, [5]);

        assert!(pgc.split_polygon(
            &OpenTriangle(2, 0, 4),
            &OpenTriangle(1, 5, 2),
        ));

        assert!(pgc.split_polygon(
            &OpenTriangle(0, 4, 3),
            &OpenTriangle(3, 1, 5),
        ));

        assert_eq!(pgc.points(), &{
            let mut expected = StableVec::new();
            let inp: &[&[usize]] = &[
                &[2, 5, 4],
                &[3, 4, 5],
                &[0, 5],
                &[4, 1],
                &[0, 1, 3],
                &[1, 0, 2],
            ];
            for i in inp {
                expected.push(((), i.to_vec().into_boxed_slice()));
            }
            expected
        });
    }
}
