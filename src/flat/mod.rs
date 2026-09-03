// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0
//! Annotated (polygonal) 2D complex/complices such that the intersection of boundaries between
//! two faces is [connected](https://en.wikipedia.org/wiki/Connected_space).

use alloc::collections::BTreeSet;
use approx::AbsDiffEq;
use core::fmt;
use num_traits::Num;

pub mod astar;
pub mod constrained_pathing;

mod layer;
use crate::layer::LayerId;
pub use layer::{GetLayerIds, Iter as LayerIdsIter, LayerIds};

mod tesselation;
pub use tesselation::{
    Face, FrozenFace, FrozenFaceNeighbour, FrozenTesselation, NotATesselation, Tesselation,
};

pub trait Topo2DComplex
where
    <Self::Scalar as AbsDiffEq>::Epsilon: Clone,
{
    type VertexId: Sized + Copy + Eq + Ord + fmt::Debug;
    type FaceId: Sized + Copy + Eq + Ord + fmt::Debug;
    type Scalar: Sized + Clone + AbsDiffEq + Num + PartialOrd + fmt::Debug;

    fn vertex_position(&self, vertex: Self::VertexId) -> [Self::Scalar; 2];

    fn vertex_adjacent_faces(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = Self::FaceId> + '_;
    fn face_adjacent_vertices(
        &self,
        face: Self::FaceId,
    ) -> impl Iterator<Item = Self::VertexId> + '_;
    fn face_adjacent_faces(&self, face: Self::FaceId) -> impl Iterator<Item = Self::FaceId> + '_;

    /// The returned value should be (if any) of the form `[left vertex, right vertex]`, i.e. ordered clockwise.
    fn portal_between(
        &self,
        face_from: Self::FaceId,
        face_to: Self::FaceId,
    ) -> Option<[Self::VertexId; 2]>;
}

pub trait MultiLayerNavmesh: Topo2DComplex {
    fn face_layers(&self, face: <Self as Topo2DComplex>::FaceId) -> LayerIds;
}

/// A node of a path from source to sink, including layer information
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Node<T> {
    pub fixed: T,
    pub layer: LayerId,
}

/// An endpoint for a pathing operation
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Endpoint<T> {
    pub fixed: BTreeSet<T>,
    pub layers: LayerIds,
}

fn point_distance<Scalar, Score, PnF>(point_norm: PnF, a: &[Scalar; 2], b: &[Scalar; 2]) -> Score
where
    PnF: Fn(&[Scalar; 2]) -> Score,
    Scalar: Clone + Num,
{
    point_norm(&crate::math::delta(a, b))
}
