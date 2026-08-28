// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0
//! Refinement methods for batches of obstacles

use approx::AbsDiffEq;
use core::{fmt, slice};
use i_triangle::i_overlay::{
    core::{
        extract::BooleanExtractionBuffer,
        fill_rule::FillRule,
        integer::OverlayInt,
        overlay::{Overlay, ShapeType},
        overlay_rule::OverlayRule,
        relate::PredicateOverlay,
    },
    i_float::int::number::int::IntNumber,
    i_shape::int::IntPoint,
    string::clip::{ClipRule, IntClip},
};
use i_triangle::int::triangulatable::IntTriangulatable as _;
use maplike::ops as mlops;
use rstar::{AABB, RTree, RTreeNum, RTreeObject, RTreeParams};
use std::{collections::BTreeMap, ops::ControlFlow};

use crate::flat::{GetLayerIds, LayerIds, MultiLayerNavmesh, Topo2DComplex};

#[derive(Clone, PartialEq, Eq)]
pub struct Face<Scalar, T> {
    pub contour: Box<[[Scalar; 2]]>,
    pub data: T,
}

impl<Scalar, T> Face<Scalar, T> {
    pub fn contour_intpoints(&self) -> Vec<IntPoint<Scalar>>
    where
        Scalar: IntNumber,
    {
        self.contour[..]
            .iter()
            .map(|i| IntPoint { x: i[0], y: i[1] })
            .collect()
    }
}

impl<Scalar: fmt::Debug, T: fmt::Debug> fmt::Debug for Face<Scalar, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Face")
            .field("contour", &format_args!("{:?}", &self.contour[..]))
            .field("data", &self.data)
            .finish()
    }
}

impl<Scalar: RTreeNum, T> RTreeObject for Face<Scalar, T> {
    type Envelope = AABB<[Scalar; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_points(self.contour.iter())
    }
}

#[derive(Clone)]
pub struct Tesselation<Scalar: RTreeNum, T, Params: RTreeParams = rstar::DefaultParams> {
    rtree: RTree<Face<Scalar, T>, Params>,
}

impl<Scalar, T, Params> fmt::Debug for Tesselation<Scalar, T, Params>
where
    Scalar: RTreeNum,
    T: fmt::Debug,
    Params: RTreeParams,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Tesselation ")?;
        f.debug_set().entries(self.rtree.iter()).finish()
    }
}

impl<Scalar, T, Params> Default for Tesselation<Scalar, T, Params>
where
    Scalar: RTreeNum,
    Params: RTreeParams,
{
    #[inline]
    fn default() -> Self {
        Self {
            rtree: RTree::new_with_params(),
        }
    }
}

impl<Scalar, T, Params> RTreeObject for Tesselation<Scalar, T, Params>
where
    Scalar: RTreeNum,
    Params: RTreeParams,
{
    type Envelope = AABB<[Scalar; 2]>;

    #[inline]
    fn envelope(&self) -> Self::Envelope {
        self.rtree.root().envelope()
    }
}

impl<Scalar, T, Params> Tesselation<Scalar, T, Params>
where
    Scalar: OverlayInt + IntNumber + RTreeNum,
    T: Clone,
    Params: RTreeParams,
{
    fn insert_impl<II: IntoIterator<Item = (Vec<Vec<Vec<IntPoint<Scalar>>>>, T)>>(
        &mut self,
        iter: II,
    ) {
        for i in iter
            .into_iter()
            .filter(|(shapes, _)| !shapes.is_empty())
            .flat_map(|(shapes, data)| {
                shapes
                    .triangulate()
                    .into_delaunay()
                    .to_convex_polygons()
                    .into_iter()
                    .map(move |contour| Face {
                        contour: contour.into_iter().map(|ip| [ip.x, ip.y]).collect(),
                        data: data.clone(),
                    })
            })
        {
            self.rtree.insert(i);
        }
    }

    /// Given a set of shapea, make sure that this tesselation contains a set of faces
    /// whose union has this shape as boundary.
    ///
    /// Usually, one invokes this method just after creation of the tesselation object,
    /// once to allocate the outer boundary of the plane,
    /// and then e.g. once for each layer with the obstacles on a layer.
    ///
    /// Note that the shapes in `alloc_shapes` aren't allowed to overlap (otherwise they'll get merged).
    // The main reason this function takes a set of shapes as its argument is that
    // Every iteration produces a set of shapes for the remaining area to "allocate".
    pub fn allocate_shapes(&mut self, mut alloc_shapes: Vec<Vec<Vec<IntPoint<Scalar>>>>)
    where
        T: Default,
    {
        let mut tmp = Vec::new();
        let mut bx_buffer = BooleanExtractionBuffer::default();
        for Face { contour, data } in self.rtree.drain_in_envelope_intersecting(AABB::from_points(
            alloc_shapes
                .iter()
                .flat_map(|i| i.iter().flat_map(|j| j.iter().map(|k| [k.x, k.y])))
                .collect::<Vec<_>>()
                .iter(),
        )) {
            let shape = vec![contour.into_iter().map(Into::into).collect::<Vec<_>>()];
            if !alloc_shapes.is_empty() {
                let mut overlay = Overlay::with_shapes(&alloc_shapes, slice::from_ref(&shape));
                let local_graph = overlay
                    .build_graph_view(FillRule::EvenOdd)
                    .expect("unable to build graph view");

                let inters = local_graph.extract_shapes(OverlayRule::Intersect, &mut bx_buffer);

                if !inters.is_empty() {
                    tmp.push((inters, data.clone()));

                    // update involved obstacles
                    alloc_shapes =
                        local_graph.extract_shapes(OverlayRule::Difference, &mut bx_buffer);
                    tmp.push((
                        local_graph.extract_shapes(OverlayRule::InverseDifference, &mut bx_buffer),
                        data.clone(),
                    ));
                    continue;
                }
            };

            // `contour` has no overlap with `alloc_contour`
            tmp.push((vec![shape], data));
        }

        // handle whatever remains of `alloc_shapes`
        if !alloc_shapes.is_empty() {
            tmp.push((alloc_shapes, T::default()));
        }

        self.insert_impl(tmp);
    }

    /// In the given envelope `envelope`, find all faces with the same data and re-segment them
    /// into convex faces.
    pub fn optimize_envelope(&mut self, envelope: AABB<[Scalar; 2]>)
    where
        T: Eq + Ord,
    {
        let mut buckets = BTreeMap::<T, Vec<Box<[[Scalar; 2]]>>>::new();
        for Face { contour, data } in self.rtree.drain_in_envelope_intersecting(envelope) {
            buckets.entry(data).or_default().push(contour);
        }
        // The insertion implementation automatically retriangulates and collects into convex polygons,
        // implicitly also mering adjacent polygons.
        self.insert_impl(buckets.into_iter().map(|(data, shape)| {
            (
                shape
                    .into_iter()
                    .map(|i| vec![i.into_iter().map(Into::into).collect()])
                    .collect(),
                data,
            )
        }));
    }
}

impl<Scalar, T, Params> Tesselation<Scalar, T, Params>
where
    Scalar: RTreeNum,
    Params: RTreeParams,
{
    pub fn rebalance(&mut self) {
        self.rtree =
            RTree::bulk_load_with_params(core::mem::take(&mut self.rtree).into_iter().collect());
    }

    pub fn update_data<B, F>(&mut self, outer_contour: &[IntPoint<Scalar>], f: F) -> ControlFlow<B>
    where
        Scalar: IntNumber + OverlayInt,
        F: Fn(&[[Scalar; 2]], &mut T) -> ControlFlow<B>,
    {
        self.rtree.locate_in_envelope_intersecting_int_mut(
            AABB::from_points(
                outer_contour
                    .iter()
                    .map(|k| [k.x, k.y])
                    .collect::<Vec<_>>()
                    .iter(),
            ),
            |Face { contour, data }| {
                let mut overlay = PredicateOverlay::new(outer_contour.len() + contour.len());
                overlay.add_contour(outer_contour, ShapeType::Subject);
                overlay.add_contour(
                    &contour.iter().map(|i| (*i).into()).collect::<Vec<_>>(),
                    ShapeType::Clip,
                );
                if !overlay.interiors_intersect() {
                    return ControlFlow::Continue(());
                }
                f(&*contour, data)
            },
        )
    }

    pub fn map_data<U, F: FnMut(T) -> U>(self, mut f: F) -> Tesselation<Scalar, U, Params> {
        Tesselation {
            rtree: RTree::bulk_load_with_params(
                self.rtree
                    .into_iter()
                    .map(|Face { contour, data }| Face {
                        contour,
                        data: f(data),
                    })
                    .collect(),
            ),
        }
    }
}

impl<'a, Scalar, T, Params> Topo2DComplex for &'a Tesselation<Scalar, T, Params>
where
    Scalar: Copy + AbsDiffEq + IntNumber + RTreeNum + PartialOrd + OverlayInt,
    Params: RTreeParams,
{
    type VertexId = [Scalar; 2];
    type FaceId = &'a [[Scalar; 2]];
    type Scalar = Scalar;

    fn vertex_position(&self, vertex: Self::VertexId) -> [Scalar; 2] {
        vertex
    }

    fn vertex_adjacent_faces(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = Self::FaceId> + '_ {
        self.rtree
            .locate_in_envelope_intersecting(AABB::from_point(vertex))
            .map(|i| &i.contour[..])
    }

    fn face_adjacent_vertices(
        &self,
        face: Self::FaceId,
    ) -> impl Iterator<Item = Self::VertexId> + '_ {
        face.iter().copied()
    }

    fn face_adjacent_faces(&self, face: Self::FaceId) -> impl Iterator<Item = Self::FaceId> + '_ {
        self.rtree
            .locate_in_envelope_intersecting(AABB::from_points(face.iter()))
            .map(|i| &i.contour[..])
            .filter(move |&i| i != face && self.portal_between(i, face).is_some())
    }

    /// The returned value should be (if any) of the form `[left vertex, right vertex]`, i.e. ordered clockwise.
    fn portal_between(
        &self,
        face_from: Self::FaceId,
        face_to: Self::FaceId,
    ) -> Option<[Self::VertexId; 2]> {
        // All `contour`s in .rtree are CCW, so the same applies to `face_*` arguments.
        let mut face_from = face_from.to_vec();
        face_from.push(*face_from.first()?);
        let ipface_from: Vec<IntPoint<Scalar>> =
            face_from.iter().copied().map(Into::into).collect();
        let mut face_to = face_to.to_vec();
        face_to.push(*face_to.first()?);
        let ipface_to: Vec<IntPoint<Scalar>> = face_to.iter().copied().map(Into::into).collect();

        let ret = ipface_from
            .clip_path(
                &ipface_to,
                FillRule::EvenOdd,
                ClipRule {
                    invert: false,
                    boundary_included: true,
                },
            )
            .into_iter()
            .map(|path| path.into_iter().map(|i| [i.x, i.y]).collect::<Vec<_>>())
            .collect::<Vec<_>>();

        let mut iter = ret.into_iter();
        let fi = iter.next()?;
        if iter.next().is_some() {
            return None;
        }
        let [x, y] = fi[..] else {
            return None;
        };
        // `clip_path` already seems to produce a CW result from a CCW input.
        Some([y, x])
    }
}

impl<Scalar, T, Params> MultiLayerNavmesh for &Tesselation<Scalar, T, Params>
where
    Scalar: Copy + AbsDiffEq + IntNumber + RTreeNum + PartialOrd + OverlayInt,
    T: GetLayerIds,
    Params: RTreeParams,
{
    #[inline]
    fn face_layers(&self, face: <Self as Topo2DComplex>::FaceId) -> LayerIds {
        use mlops::Get;
        self.get(face).unwrap().layers()
    }
}

impl<Scalar, T, Params> AsRef<RTree<Face<Scalar, T>, Params>> for Tesselation<Scalar, T, Params>
where
    Scalar: RTreeNum,
    Params: RTreeParams,
{
    #[inline(always)]
    fn as_ref(&self) -> &RTree<Face<Scalar, T>, Params> {
        &self.rtree
    }
}

impl<Scalar, T, Params> maplike::containers::Container for Tesselation<Scalar, T, Params>
where
    Scalar: RTreeNum,
    Params: RTreeParams,
{
    type Key = Box<[[Scalar; 2]]>;
    type Value = T;
}

impl<Scalar, T, Params> mlops::WithOne<Face<Scalar, T>> for Tesselation<Scalar, T, Params>
where
    Scalar: IntNumber + OverlayInt + RTreeNum,
    T: Clone + Default,
    Params: RTreeParams,
{
    fn with_one(element: Face<Scalar, T>) -> Self {
        let mut this = Self::default();

        let contour: Vec<_> = element.contour.into_iter().map(Into::into).collect();
        this.allocate_shapes(vec![vec![contour]]);

        for i in &mut this.rtree {
            i.data = element.data.clone();
        }

        this
    }
}

impl<Scalar, T, Params> mlops::Get<Box<[[Scalar; 2]]>, [[Scalar; 2]]>
    for Tesselation<Scalar, T, Params>
where
    Scalar: RTreeNum,
    Params: RTreeParams,
{
    fn get(&self, face: &[[Scalar; 2]]) -> Option<&T> {
        self.rtree
            .locate_in_envelope(AABB::from_points(face.iter()))
            .find(move |&i| &i.contour[..] == face)
            .map(|i| &i.data)
    }
}

impl<Scalar, T, Params> mlops::Clear for Tesselation<Scalar, T, Params>
where
    Scalar: RTreeNum,
    Params: RTreeParams,
{
    #[inline]
    fn clear(&mut self) {
        self.rtree = RTree::new_with_params();
    }
}

impl<Scalar, T, Params> mlops::Len for Tesselation<Scalar, T, Params>
where
    Scalar: RTreeNum,
    Params: RTreeParams,
{
    #[inline]
    fn len(&self) -> usize {
        self.rtree.size()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LayerId;
    use i_triangle::i_overlay::i_shape::int_path;

    #[test]
    fn two_rectangles() {
        let mut tess = Tesselation::<i32, LayerIds>::default();
        tess.allocate_shapes(vec![vec![int_path![[0i32, 0], [2, 0], [2, 2], [0, 2]]]]);

        let _ = tess.update_data::<(), _>(
            &int_path![[0i32, 0], [2, 0], [2, 2], [0, 2]],
            |_, layers| {
                layers.insert(LayerId::from(0));
                ControlFlow::Continue(())
            },
        );

        tess.allocate_shapes(vec![vec![int_path![[1i32, 1], [3, 1], [3, 3], [1, 3]]]]);

        let _ = tess.update_data::<(), _>(
            &int_path![[1i32, 1], [3, 1], [3, 3], [1, 3]],
            |_, layers| {
                layers.insert(LayerId::from(1));
                ControlFlow::Continue(())
            },
        );

        assert!(tess.as_ref().contains(&Face {
            contour: vec![[2i32, 2], [1, 2], [1, 1], [2, 1]].into_boxed_slice(),
            data: {
                let mut layers = LayerIds::default();
                layers.insert(LayerId::from(0));
                layers.insert(LayerId::from(1));
                layers
            },
        }));
    }
}
