// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use std::{
    cmp::Ordering,
    ops::{Mul, Sub},
};

use num_traits::Zero;

use crate::{point::Point, triangle};

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ChainVertexWithIndex<K> {
    pub vertex: ChainVertex<K>,
    pub index: VertexIndex,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, PartialOrd, Ord)]
pub struct VertexIndex(pub usize);

#[derive(Clone, Copy, Debug)]
pub enum ChainVertexType {
    Start,
    Split,
    Regular,
    Merge,
    End,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ChainVertex<K> {
    pub prev: Point<K>,
    pub this: Point<K>,
    pub next: Point<K>,
}

impl<K: PartialEq> PartialEq for ChainVertex<K> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.this == other.this
    }
}

impl<K: Eq> Eq for ChainVertex<K> {}

impl<K> ChainVertex<K> {
    #[inline]
    pub fn new(prev: Point<K>, this: Point<K>, next: Point<K>) -> Self {
        Self { prev, this, next }
    }

    #[inline]
    pub(crate) fn segment(self) -> ChainSegment<K> {
        ChainSegment {
            a: self.this,
            b: self.next,
        }
    }
}

impl<K: PartialOrd> PartialOrd for ChainVertex<K> {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.this.partial_cmp(&other.this)
    }
}

impl<K: Ord> Ord for ChainVertex<K> {
    #[inline]
    fn cmp(&self, other: &Self) -> Ordering {
        self.this.cmp(&other.this)
    }
}

impl<K: Clone + Mul<Output = K> + Sub<Output = K> + PartialOrd + Zero> ChainVertex<K> {
    #[inline]
    pub fn typ(&self) -> ChainVertexType {
        if self.prev < self.this && self.next < self.this {
            if triangle::is_clockwise(self.prev.clone(), self.this.clone(), self.next.clone()) {
                ChainVertexType::Merge
            } else {
                ChainVertexType::End
            }
        } else if self.this < self.next && self.this < self.prev {
            if triangle::is_clockwise(self.prev.clone(), self.this.clone(), self.next.clone()) {
                ChainVertexType::Split
            } else {
                ChainVertexType::Start
            }
        } else {
            ChainVertexType::Regular
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq)]
pub(crate) struct ChainSegment<K> {
    pub a: Point<K>,
    pub b: Point<K>,
}

impl<K: Clone + Mul<Output = K> + Ord + Sub<Output = K> + Zero> ChainSegment<K> {
    #[inline]
    pub fn vertical_cmp_with_point(&self, point: Point<K>) -> Ordering {
        triangle::clockwise_cmp(self.a.clone(), point, self.b.clone())
    }
}

impl<K: Clone + Mul<Output = K> + Ord + Sub<Output = K> + Zero> PartialOrd for ChainSegment<K> {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<K: Clone + Mul<Output = K> + Ord + Sub<Output = K> + Zero> Ord for ChainSegment<K> {
    #[inline]
    fn cmp(&self, other: &Self) -> Ordering {
        match self.b.cmp(&other.b) {
            Ordering::Less => {
                triangle::clockwise_cmp(self.b.clone(), other.a.clone(), other.b.clone())
            }
            Ordering::Equal => {
                triangle::clockwise_cmp(self.b.clone(), self.a.clone(), other.a.clone())
            }
            Ordering::Greater => {
                triangle::clockwise_cmp(other.b.clone(), self.b.clone(), self.a.clone())
            }
        }
    }
}
