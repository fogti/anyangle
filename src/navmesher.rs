// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use std::marker::PhantomData;

use polygon_unionfind::{
    Difference, Inflate, Intersection, Laminate, PolygonWithData, Rings, Union,
};
use rstar::RTreeNum;

use crate::navmesh::Remesh;

pub struct Navmesher<K: RTreeNum, N, D> {
    laminate: Laminate<K, PolygonWithData<K, D>>,
    navmesh: N,
    scalar_marker: PhantomData<K>,
}

impl<K: RTreeNum, N, D> Navmesher<K, N, D> {
    pub fn navmesh(&self) -> &N {
        &self.navmesh
    }
}

impl<K, N, D> Navmesher<K, N, D>
where
    K: RTreeNum + Default + Clone,
    N: Default + Remesh<K>,
    D: Clone + Default,
    PolygonWithData<K, D>: Union<PolygonWithData<K, D>> + Difference<PolygonWithData<K, D>>,
{
    pub fn new(
        boundary: impl IntoIterator<Item = [K; 2]>,
        num_layers: usize,
        parallel_inflations: impl IntoIterator<Item = K>,
    ) -> Self {
        let boundary = PolygonWithData {
            exterior: boundary.into_iter().collect(),
            interiors: Vec::new(),
            data: D::default(),
        };

        let exterior = boundary.exterior.clone();
        let laminate =
            Laminate::<K, PolygonWithData<K, D>>::new(boundary, num_layers, parallel_inflations);

        let shapes_per_layer: Vec<Vec<Vec<Vec<[K; 2]>>>> = (0..num_layers)
            .map(|_| vec![vec![exterior.clone()]])
            .collect();

        let mut navmesh = N::default();
        navmesh.remesh(&shapes_per_layer);

        Self {
            laminate,
            navmesh,
            scalar_marker: PhantomData,
        }
    }
}

impl<K, N, D> Navmesher<K, N, D>
where
    K: RTreeNum + Default + Clone + Ord,
    N: Remesh<K>,
    D: Clone + Default,
    PolygonWithData<K, D>: Clone
        + Rings<K>
        + Inflate<K>
        + Union<PolygonWithData<K, D>>
        + Difference<PolygonWithData<K, D>>
        + Intersection<PolygonWithData<K, D>>,
{
    pub fn insert_polygon(&mut self, lamina_index: usize, polygon: PolygonWithData<K, D>) {
        self.laminate.add_into_lamina(lamina_index, polygon);

        let shapes_per_layer: Vec<Vec<Vec<Vec<[K; 2]>>>> = self
            .laminate
            .laminas()
            .iter()
            .map(|lamina| {
                let polygon_set = lamina.primary().primary().minuend();
                polygon_set
                    .polygons()
                    .values()
                    .map(|polygon| {
                        let mut contours = vec![polygon.exterior().to_vec()];
                        for interior in polygon.interiors() {
                            contours.push(interior.to_vec());
                        }
                        contours
                    })
                    .collect()
            })
            .collect();

        self.navmesh.remesh(&shapes_per_layer);
    }
}
