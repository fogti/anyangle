// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use std::ops::{Add, Mul, Sub};

use num_traits::{Bounded, Zero};

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, PartialOrd, Ord)]
pub struct Point<K> {
    pub x: K,
    pub y: K,
}

impl<K> Point<K> {
    pub fn new(x: K, y: K) -> Self {
        Self { x, y }
    }
}

impl<K: Zero> Point<K> {
    #[inline(always)]
    pub fn zero() -> Self {
        Self {
            x: K::zero(),
            y: K::zero(),
        }
    }
}

impl<K: Bounded> Point<K> {
    #[inline(always)]
    pub fn empty() -> Self {
        Self {
            x: K::max_value(),
            y: K::max_value(),
        }
    }
}

impl<K: Mul<Output = K> + Add<Output = K>> Point<K> {
    #[inline(always)]
    pub fn dot_product(self, other: Self) -> K {
        self.x * other.x + self.y * other.y
    }
}

impl<K: Mul<Output = K> + Sub<Output = K>> Point<K> {
    #[inline(always)]
    pub fn perp_dot_product(self, other: Self) -> K {
        self.x * other.y - self.y * other.x
    }
}

impl<K: Add<Output = K> + Clone + Mul<Output = K>> Point<K> {
    #[inline(always)]
    pub fn squared_length(self) -> K {
        self.x.clone() * self.x + self.y.clone() * self.y
    }
}

impl<K: Add<Output = K> + Copy + Mul<Output = K> + Sub<Output = K>> Point<K> {
    #[inline(always)]
    pub fn squared_distance(self, other: Self) -> K {
        (self - other).squared_length()
    }
}

impl<K: Add<Output = K>> Add for Point<K> {
    type Output = Point<K>;

    #[inline(always)]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl<K: Sub<Output = K>> Sub for Point<K> {
    type Output = Point<K>;

    #[inline(always)]
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}
