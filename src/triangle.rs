// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use std::{
    cmp::Ordering,
    ops::{Mul, Sub},
};

use num_traits::Zero;

use crate::point::Point;

#[inline(always)]
pub fn double_signed_area<K: Clone + Mul<Output = K> + Sub<Output = K>>(
    p0: Point<K>,
    p1: Point<K>,
    p2: Point<K>,
) -> K {
    (p1.clone() - p0).perp_dot_product(p1 - p2)
}

#[inline(always)]
pub fn is_clockwise<K: Clone + Mul<Output = K> + PartialOrd + Sub<Output = K> + Zero>(
    p0: Point<K>,
    p1: Point<K>,
    p2: Point<K>,
) -> bool {
    double_signed_area(p0, p1, p2) > K::zero()
}

#[inline(always)]
pub fn clockwise_cmp<K: Clone + Mul<Output = K> + Ord + Sub<Output = K> + Zero>(
    p0: Point<K>,
    p1: Point<K>,
    p2: Point<K>,
) -> Ordering {
    (K::zero()).cmp(&double_signed_area(p0, p1, p2))
}
