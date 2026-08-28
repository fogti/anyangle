// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use core::fmt;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
#[repr(transparent)]
pub struct LayerId(u32);

impl LayerId {
    pub const MIN: LayerId = LayerId(0);
    pub const MAX: LayerId = LayerId(u32::MAX);
}

impl fmt::Debug for LayerId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "LayerId({})", self.0)
    }
}

impl From<usize> for LayerId {
    #[inline]
    fn from(x: usize) -> LayerId {
        LayerId(x.try_into().unwrap())
    }
}

impl From<LayerId> for usize {
    #[inline]
    fn from(x: LayerId) -> usize {
        x.0.try_into().unwrap()
    }
}
