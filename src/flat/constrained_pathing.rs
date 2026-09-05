// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use alloc::{
    collections::{BTreeMap, BTreeSet, BinaryHeap},
    vec,
    vec::Vec,
};
use core::{
    cmp::{self, Ordering},
    fmt, iter,
};

use approx::AbsDiffEq;
use num_traits::Zero;

use super::{Endpoint, MultiLayerNavmesh, Node, Topo2DComplex};
use crate::{LayerId, funnel::SimpleFunnel};

#[derive(Clone, Debug)]
pub enum Output<T: Topo2DComplex, Score> {
    /// A successful pathing result
    Result(Vec<Node<T::VertexId>>),

    IntermediateStep(Node<T::FaceId>, Vec<(Node<T::FaceId>, Score)>),
}

struct BestPaths<S, T>(BTreeMap<Node<T>, (S, Node<T>)>);

impl<S, T: Copy + Ord> BestPaths<S, T> {
    fn reconstruct_path(&self, current: Node<T>) -> Vec<Node<T>> {
        let mut ret = vec![current];
        while let Some((_, key)) = self.0.get(ret.last().unwrap()) {
            ret.push(*key);
        }
        ret.reverse();
        ret
    }
}

struct QueueEntry<T: Topo2DComplex> {
    node: Node<T::FaceId>,
    /// Nodes processed since the last `apex` change
    //prev_nodes: BTreeSet<Node<T::FaceId>>,
    funnel: SimpleFunnel<T::Scalar, Node<T::VertexId>>,
}

impl<T> Clone for QueueEntry<T>
where
    T: Topo2DComplex,
{
    fn clone(&self) -> Self {
        Self {
            //prev_nodes: self.prev_nodes.clone(),
            node: self.node,
            funnel: self.funnel.clone(),
        }
    }
}

impl<T> PartialEq for QueueEntry<T>
where
    T: Topo2DComplex,
{
    fn eq(&self, other: &Self) -> bool {
        //self.prev_nodes == other.prev_nodes
            //&&
            self.node == other.node
            && self.funnel == other.funnel
    }
}

impl<T> Eq for QueueEntry<T>
where
    T: Topo2DComplex,
    T::Scalar: Eq,
{
}

impl<T> cmp::PartialOrd for QueueEntry<T>
where
    T: Topo2DComplex,
    T::Scalar: cmp::Ord,
{
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<T> cmp::Ord for QueueEntry<T>
where
    T: Topo2DComplex,
    T::Scalar: cmp::Ord,
{
    fn cmp(&self, other: &Self) -> Ordering {
        self.node
            .cmp(&other.node)
            //.then_with(|| self.prev_nodes.cmp(&other.prev_nodes))
            .then_with(|| self.funnel.cmp(&other.funnel))
    }
}

impl<T> fmt::Debug for QueueEntry<T>
where
    T: Topo2DComplex,
    T::Scalar: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("QueueEntry")
            .field("node", &self.node)
            //.field("prev_nodes", &self.prev_nodes)
            .field("funnel", &self.funnel)
            .finish()
    }
}

impl<T: Topo2DComplex> QueueEntry<T> {
    fn funnel(&mut self, mesh: &T, next: Node<T::FaceId>) -> bool {
        if self.node == next /* || self.prev_nodes.contains(&next) */ {
            // invalid move
            return false;
        }

        if self.node.fixed != next.fixed {
            // face transition
            let Some(portal) = mesh.portal_between(self.node.fixed, next.fixed) else {
                return false;
            };
            let portal = portal.map(|fixed| {
                (
                    mesh.vertex_position(fixed),
                    Node {
                        fixed,
                        layer: next.layer,
                    },
                )
            });
            self.funnel.push(portal);
        }

        //self.prev_nodes
        //    .insert(core::mem::replace(&mut self.node, next));
        true
    }
}

pub struct Pathing<'a, T: Topo2DComplex, Score, Pnf> {
    // The constant environment during a search (i.e. what never changes during a [`pathing`] invocation).
    mesh: &'a T,
    point_norm: Pnf,
    final_end: Endpoint<T::VertexId>,
    epsilon: <T::Scalar as AbsDiffEq>::Epsilon,
    layer_transition_penality: Score,

    // The dynamic environment
    heap: BinaryHeap<(cmp::Reverse<Score>, QueueEntry<T>)>,
    best_paths: BestPaths<Score, T::VertexId>,
}

impl<T, Score, Pnf> Pathing<'_, T, Score, Pnf>
where
    T: MultiLayerNavmesh,
    T::Scalar: Ord,
    <<T as Topo2DComplex>::Scalar as AbsDiffEq>::Epsilon: Clone,
    Score: Clone + Ord + Zero + fmt::Debug,
    Pnf: Fn(&[T::Scalar; 2]) -> Score,
{
    fn funnel(
        &self,
        base: &QueueEntry<T>,
        end: Option<&Node<T::FaceId>>,
    ) -> Option<(QueueEntry<T>, Vec<Node<T::VertexId>>, usize)> {
        let mut edat = base.clone();
        let mut best_path = self.best_paths.reconstruct_path(edat.funnel.apex.1);
        let orig_best_path_len = best_path.len();
        if let Some(&end) = end {
            //let orig_node = edat.node;
            if !edat.funnel(self.mesh, end) {
                return None;
            }
            best_path.extend(
                edat.funnel
                    .with_epsilon(self.epsilon.clone())
                    .map(|(_, node)| node),
            );
            //if best_path.len() != orig_best_path_len {
            //    edat.prev_nodes.clear();
            //    edat.prev_nodes.insert(orig_node);
            //}
        }
        let edat = edat;

        let final_measure_point = &edat.funnel.apex;
        let mut choices: Vec<_> = self
            .final_end
            .fixed
            .iter()
            .map(|&final_end_fixed| {
                let final_end_pos = (
                    self.mesh.vertex_position(final_end_fixed),
                    Node {
                        fixed: final_end_fixed,
                        layer: edat.funnel.apex.1.layer,
                    },
                );
                let mut final_funnel = edat.funnel.clone();
                final_funnel.push([final_end_pos.clone(), final_end_pos.clone()]);
                let final_stretch = final_funnel
                    .with_epsilon(self.epsilon.clone())
                    .collect::<Vec<_>>();
                let final_stretch_positions = iter::once(final_measure_point.0.clone())
                    .chain(final_stretch.iter().map(|(pos, _)| pos.clone()))
                    .chain(iter::once(final_end_pos.0))
                    .collect::<Vec<_>>();
                let length = final_stretch_positions.array_windows::<2>().fold(
                    Zero::zero(),
                    |length: Score, [from, to]| {
                        length + super::point_distance(&self.point_norm, from, to)
                    },
                );
                (length, final_stretch, final_end_pos.1)
            })
            .collect();
        choices.sort_by_key(|(k, _, _)| k.clone());
        let &(_, ref final_stretch, mut final_end) = choices.first().unwrap();
        for (_, new_apex) in final_stretch {
            best_path.push(*new_apex);
            final_end.layer = new_apex.layer;
        }
        if !self.final_end.layers.is_on_layer(final_end.layer) {
            let mut current_layer = final_end.layer;
            let (closest, next) = match self
                .final_end
                .layers
                .closest_contained_layer_to(current_layer)
            {
                [None, None] => {
                    // all moves are illegal
                    return None;
                }
                [Some(closest), _] => (
                    closest,
                    LayerId::checked_sub_one as fn(LayerId) -> Option<LayerId>,
                ),
                [_, Some(closest)] => (
                    closest,
                    LayerId::checked_add_one as fn(LayerId) -> Option<LayerId>,
                ),
            };
            while current_layer != closest {
                let prev_layer = current_layer;
                // UNWRAP SAFETY: closest is reachable from current_layer via next
                // because the same function is used in `closest_contained_layer_to`.
                current_layer = next(prev_layer).unwrap();
            }
            final_end.layer = current_layer;
        }
        best_path.push(final_end);
        Some((edat, best_path, orig_best_path_len))
    }

    fn calculate_score(
        &self,
        base: &QueueEntry<T>,
        end: Option<&Node<T::FaceId>>,
    ) -> Option<(Score, QueueEntry<T>, Vec<Node<T::VertexId>>, usize)> {
        let (edat, best_path, orig_best_path_len) = self.funnel(base, end)?;

        let mut layer_transitions = 0;
        let score = best_path.windows(2).fold(Score::zero(), |score, i| {
            layer_transitions += i[0].layer.distance(&i[1].layer);
            score
                + super::point_distance(
                    &self.point_norm,
                    &self.mesh.vertex_position(i[0].fixed),
                    &self.mesh.vertex_position(i[1].fixed),
                )
        });
        let score = (0..layer_transitions).fold(score, |score, _| {
            score + self.layer_transition_penality.clone()
        });

        Some((score, edat, best_path, orig_best_path_len))
    }
}

impl<T, Score, Pnf> Iterator for Pathing<'_, T, Score, Pnf>
where
    T: MultiLayerNavmesh,
    T::Scalar: Ord,
    <T::Scalar as AbsDiffEq>::Epsilon: Clone,
    Score: Clone + Ord + Zero + fmt::Debug,
    Pnf: Fn(&[T::Scalar; 2]) -> Score,
{
    type Item = Output<T, Score>;

    fn next(&mut self) -> Option<Output<T, Score>> {
        // main search loop
        let cur = self.heap.pop()?;
        println!("Pathing::next: {:?}", cur.1);
        println!(
            "  with face vertices: {:?}",
            self.mesh
                .face_adjacent_vertices(cur.1.node.fixed)
                .map(|v| self.mesh.vertex_position(v))
                .collect::<Vec<_>>()
        );
        let ckn = cur.1.node;
        if self.final_end.layers.is_on_layer(ckn.layer)
            && self
                .mesh
                .face_adjacent_vertices(ckn.fixed)
                .find(|fixed| self.final_end.fixed.contains(fixed))
                .is_some()
        {
            let tmp = self
                .funnel(&cur.1, None)
                .expect("trivial path should be always funnelable");
            return Some(Output::Result(tmp.1));
        }

        let face_transition = self
            .mesh
            .face_adjacent_faces(ckn.fixed)
            // filter untraversable faces
            .filter(|&inner_face| self.mesh.face_layers(inner_face).is_on_layer(ckn.layer))
            .map(|fixed| Node {
                fixed,
                layer: ckn.layer,
            });

        let cur_face_layers = self.mesh.face_layers(ckn.fixed);

        let layer_transition = {
            let ckfa1l = cur.1.funnel.apex.1.layer;
            if ckn.layer == ckfa1l {
                cur_face_layers.adjacent_layers_to(ckn.layer)
            } else {
                // avoid running in circles
                cur_face_layers
                    .adjacent_layer_to_but_away_from(ckn.layer, ckfa1l)
                    .into_iter()
                    .collect()
            }
        }
        .into_iter()
        .map(|layer| Node {
            fixed: ckn.fixed,
            layer,
        });

        let mut ret = Vec::new();
        for next_node in face_transition.chain(layer_transition) {
            let Some((score, edat, best_path, orig_best_path_len)) =
                self.calculate_score(&cur.1, Some(&next_node))
            else {
                continue;
            };

            let mut unskip = false;
            for pair in best_path[orig_best_path_len.checked_sub(1).unwrap()..].array_windows::<2>()
            {
                use alloc::collections::btree_map::Entry;
                let bp_entry = self.best_paths.0.entry(pair[1]);
                // filter cases of worse newer scores
                // this is quite cursed and might lead to some problems
                // maybe we should instead store entire chains of nodes
                // in best-path entries instead
                if let Entry::Occupied(occ) = &bp_entry
                    && let (old_score, _) = occ.get()
                    && &score >= old_score
                {
                    continue;
                }
                unskip = true;
                let new_bp_data = (score.clone(), pair[0]);
                match bp_entry {
                    Entry::Occupied(mut occ) => {
                        occ.insert(new_bp_data);
                    }
                    Entry::Vacant(vac) => {
                        vac.insert(new_bp_data);
                    }
                }
            }
            if unskip {
                println!("  yield {:?} -> {:?}", ckn, edat);
                ret.push((next_node, score.clone()));
                self.heap.push((cmp::Reverse(score), edat));
            }
        }

        Some(Output::IntermediateStep(ckn, ret))
    }
}

pub fn pathing<'mesh, T, Score, Pnf>(
    mesh: &'mesh T,
    point_norm: Pnf,
    start: Endpoint<T::VertexId>,
    end: Endpoint<T::VertexId>,
    epsilon: <T::Scalar as AbsDiffEq>::Epsilon,
    layer_transition_penality: Score,
) -> Pathing<'mesh, T, Score, Pnf>
where
    T: MultiLayerNavmesh,
    T::Scalar: Ord,
    <T::Scalar as AbsDiffEq>::Epsilon: Clone,
    Score: Clone + Ord + Zero + fmt::Debug,
    Pnf: Fn(&[T::Scalar; 2]) -> Score,
{
    let mut env = Pathing {
        mesh,
        point_norm,
        final_end: end,
        epsilon,
        layer_transition_penality,

        heap: BinaryHeap::new(),
        best_paths: BestPaths(Default::default()),
    };

    // initialize heap
    for &fixed_vertex in &start.fixed {
        let start_pos = env.mesh.vertex_position(fixed_vertex);
        for inner_face in mesh
            .vertex_adjacent_faces(fixed_vertex)
            .collect::<BTreeSet<_>>()
        {
            // filter untraversable triangles
            for layer in &((&mesh.face_layers(inner_face)) & (&start.layers)) {
                let start_vertex_node = Node {
                    fixed: fixed_vertex,
                    layer,
                };
                let edat = QueueEntry {
                    //prev_nodes: Default::default(),
                    node: Node {
                        fixed: inner_face,
                        layer,
                    },
                    funnel: SimpleFunnel::new((start_pos.clone(), start_vertex_node)),
                };
                let Some((score, _, _, _)) = env.calculate_score(&edat, None) else {
                    continue;
                };

                env.heap.push((cmp::Reverse(score), edat));
            }
        }
    }

    env
}
