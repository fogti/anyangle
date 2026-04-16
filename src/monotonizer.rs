// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use std::{
    cmp::Ordering,
    ops::{ControlFlow, Mul, Sub},
};

use num_traits::Zero;

use crate::{
    chain::{ChainVertex, ChainVertexType, ChainVertexWithIndex, VertexIndex},
    point::Point,
};

#[derive(Clone, Copy, Debug)]
struct Section<K> {
    latest: ChainVertexWithIndex<K>,
    latest_merge: Option<ChainVertexWithIndex<K>>,
    latest_bottom: ChainVertexWithIndex<K>,
}

pub struct Monotonizer<K> {
    vertices: Vec<ChainVertex<K>>,
    curr_vertex: VertexIndex,
    sections: Vec<Section<K>>,
}

#[derive(Clone, Copy, Debug)]
pub struct Diagonal<K> {
    pub from: ChainVertexWithIndex<K>,
    pub to: ChainVertexWithIndex<K>,
}

impl<K: Clone + PartialOrd> Monotonizer<K> {
    #[inline]
    pub fn new(polygons: impl IntoIterator<Item = impl IntoIterator<Item = [K; 2]>>) -> Self {
        let vertices = Self::link_and_sort_vertices(polygons);

        Self {
            vertices,
            curr_vertex: VertexIndex(0),
            sections: Vec::new(),
        }
    }

    #[inline]
    fn link_and_sort_vertices(
        polygons: impl IntoIterator<Item = impl IntoIterator<Item = [K; 2]>>,
    ) -> Vec<ChainVertex<K>> {
        let mut vertices = Vec::new();

        for polygon in polygons.into_iter() {
            let polygon: Vec<Point<K>> =
                polygon.into_iter().map(|[x, y]| Point::new(x, y)).collect();
            let mut prev = polygon[polygon.len() - 2].clone();
            let mut this = polygon[polygon.len() - 1].clone();

            for next in polygon {
                vertices.push(ChainVertex::new(prev, this.clone(), next.clone()));
                prev = this.clone();
                this = next.clone();
            }
        }

        vertices.sort_unstable_by(|a, b| match a.partial_cmp(b) {
            Some(ord) => ord,
            None => {
                // Numbers are incomparable. For floats, this means that one of the
                // numbers is NaN.

                // Only NaN is not equal to self. This way we can detect it without
                // knowing what is the actual underlying number type.
                let a_is_nan = a != a;
                let b_is_nan = b != b;

                match (a_is_nan, b_is_nan) {
                    (true, true) => Ordering::Equal,
                    (true, false) => Ordering::Greater, // NaN is always placed last.
                    (false, true) => Ordering::Less,
                    (false, false) => Ordering::Equal, // Should never happen.
                }
            }
        });

        vertices
    }

    // TODO: These methods are only for debugging. Remove them.

    #[inline]
    pub fn sweep_vertex_count(&self) -> usize {
        self.vertices.len()
    }

    #[inline]
    pub fn current_sweep_index(&self) -> usize {
        self.curr_vertex.0
    }

    #[inline]
    pub fn sweep_vertex(&self, index: usize) -> Option<ChainVertex<K>> {
        self.vertices.get(index).cloned()
    }
}

impl<K: Clone + Mul<Output = K> + Sub<Output = K> + PartialOrd + Ord + Zero> Monotonizer<K> {
    #[inline]
    pub fn run(&mut self) -> Vec<Diagonal<K>> {
        let mut diagonals = vec![];

        while let ControlFlow::Continue(curr_diagonals) = self.step() {
            diagonals.extend(curr_diagonals);
        }

        diagonals
    }

    pub fn step(&mut self) -> ControlFlow<(), Vec<Diagonal<K>>> {
        let diagonals = match self.curr_vertex().typ() {
            ChainVertexType::Start => {
                self.start();
                vec![]
            }
            ChainVertexType::Split => vec![self.split()],
            ChainVertexType::Regular => self.regular().into_iter().collect(),
            ChainVertexType::Merge => self.merge().into_iter().collect(),
            ChainVertexType::End => self.end().into_iter().collect(),
        };

        self.curr_vertex.0 += 1;

        if self.curr_vertex.0 < self.vertices.len() {
            ControlFlow::Continue(diagonals)
        } else {
            ControlFlow::Break(())
        }
    }

    fn start(&mut self) {
        let section_index = self.find_vertex_section_index(self.curr_vertex());
        self.sections.insert(
            section_index.map(|i| i + 1).unwrap_or(0),
            Section {
                latest: self.curr_vertex_with_index(),
                latest_merge: None,
                latest_bottom: self.curr_vertex_with_index(),
            },
        );
    }

    fn split(&mut self) -> Diagonal<K> {
        let section_index = self.find_vertex_section_index(self.curr_vertex()).unwrap();
        let diagonal = self
            .make_diagonal_from_merge_if_present(section_index)
            .unwrap_or(Diagonal {
                from: self.sections[section_index].latest.clone(),
                to: self.curr_vertex_with_index(),
            });

        self.sections[section_index].latest = self.curr_vertex_with_index();

        self.sections.insert(
            section_index + 1,
            Section {
                latest: self.curr_vertex_with_index(),
                latest_merge: None,
                latest_bottom: self.curr_vertex_with_index(),
            },
        );

        diagonal
    }

    fn regular(&mut self) -> Option<Diagonal<K>> {
        let section = self.find_vertex_section_index(self.curr_vertex()).unwrap();
        let diagonal = self.make_diagonal_from_merge_if_present(section);

        if self.sections[section].latest_bottom.vertex.next
            == self.curr_vertex_with_index().vertex.this
        {
            self.sections[section].latest_bottom = self.curr_vertex_with_index();
        }

        self.sections[section].latest = self.curr_vertex_with_index();

        diagonal
    }

    fn merge(&mut self) -> Vec<Diagonal<K>> {
        let section = self.find_vertex_section_index(self.curr_vertex()).unwrap();
        let mut diagonals = vec![];
        diagonals.extend(
            self.make_diagonal_from_merge_if_present(section)
                .into_iter(),
        );
        diagonals.extend(
            self.make_diagonal_from_merge_if_present(section - 1)
                .into_iter(),
        );

        self.sections[section - 1].latest = self.curr_vertex_with_index();
        self.sections[section - 1].latest_merge = Some(self.curr_vertex_with_index());

        self.sections.remove(section);

        diagonals
    }

    fn end(&mut self) -> Option<Diagonal<K>> {
        let section_index = self.find_vertex_section_index(self.curr_vertex()).unwrap();
        let diagonal = self.make_diagonal_from_merge_if_present(section_index);

        self.sections.remove(section_index);

        diagonal
    }

    fn find_vertex_section_index(&mut self, vertex: ChainVertex<K>) -> Option<usize> {
        self.sections.iter().rposition(|section| {
            section
                .latest_bottom
                .vertex
                .clone()
                .segment()
                .vertical_cmp_with_point(vertex.this.clone())
                .is_le()
        })
    }

    fn make_diagonal_from_merge_if_present(&mut self, section: usize) -> Option<Diagonal<K>> {
        self.sections[section]
            .latest_merge
            .take()
            .map(|middle| Diagonal {
                from: middle,
                to: self.curr_vertex_with_index(),
            })
    }

    fn curr_vertex_with_index(&self) -> ChainVertexWithIndex<K> {
        ChainVertexWithIndex {
            vertex: self.curr_vertex(),
            index: self.curr_vertex,
        }
    }

    fn curr_vertex(&self) -> ChainVertex<K> {
        self.vertices[self.curr_vertex.0].clone()
    }
}
