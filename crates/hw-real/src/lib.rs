//! Numeric layer: the `Real` trait (IEEE binary64 semantics as executed by the 32-bit Flash
//! Player's SSE2 code) and AS3-specific helpers. `f64` is the native implementation.
#![cfg_attr(not(feature = "std"), no_std)]

use core::ops::{Add, Div, Mul, Neg, Sub};

/// A double with exactly the semantics of AS3 `Number` arithmetic (round-to-nearest-even, no FMA,
/// no flush-to-zero). Comparisons return false on NaN like AS3's `<`, `>`, `==`.
pub trait Real:
    Copy + Add<Output = Self> + Sub<Output = Self> + Mul<Output = Self> + Div<Output = Self> + Neg<Output = Self>
{
    fn from_words(w: [u32; 2]) -> Self;
    fn to_words(self) -> [u32; 2];
    fn from_i32(v: i32) -> Self;
    fn from_u32(v: u32) -> Self;
    fn sqrt(self) -> Self;
    fn lt(self, o: Self) -> bool;
    fn le(self, o: Self) -> bool;
    fn gt(self, o: Self) -> bool;
    fn ge(self, o: Self) -> bool;
    fn eq_num(self, o: Self) -> bool;
    fn is_nan(self) -> bool;
    /// ECMAScript ToUint32: NaN/Inf -> 0, truncate toward zero, modulo 2^32.
    fn to_uint32(self) -> u32;
    /// AS3 `int(x)`: ToInt32.
    fn to_int32(self) -> i32 {
        self.to_uint32() as i32
    }
    fn floor(self) -> Self;
    fn ceil(self) -> Self;
    fn zero() -> Self {
        Self::from_words([0, 0])
    }
    fn one() -> Self {
        Self::from_i32(1)
    }
    fn is_zero(self) -> bool {
        self.eq_num(Self::zero())
    }
}

/// Bit-pattern literal: `rl!(0x400921FB, 0x54442D18)` or `rl!(0.5)` (const-folded to bits).
#[macro_export]
macro_rules! rl {
    ($ty:ty; $v:expr) => {
        <$ty as $crate::Real>::from_words(const { $crate::words_of_f64($v as f64) })
    };
    ($hi:literal, $lo:literal) => {
        $crate::Real::from_words([$hi as u32, $lo as u32])
    };
    ($v:literal) => {
        $crate::Real::from_words(const { $crate::words_of_f64($v as f64) })
    };
}

/// Compile-time split of an f64 literal into words (only used through `rl!`).
pub const fn words_of_f64(v: f64) -> [u32; 2] {
    let b = v.to_bits();
    [(b >> 32) as u32, b as u32]
}

#[cfg(feature = "std")]
impl Real for f64 {
    #[inline(always)]
    fn from_words(w: [u32; 2]) -> Self {
        f64::from_bits(((w[0] as u64) << 32) | w[1] as u64)
    }
    #[inline(always)]
    fn to_words(self) -> [u32; 2] {
        let b = self.to_bits();
        [(b >> 32) as u32, b as u32]
    }
    #[inline(always)]
    fn from_i32(v: i32) -> Self {
        v as f64
    }
    #[inline(always)]
    fn from_u32(v: u32) -> Self {
        v as f64
    }
    #[inline(always)]
    fn sqrt(self) -> Self {
        f64::sqrt(self)
    }
    #[inline(always)]
    fn lt(self, o: Self) -> bool {
        self < o
    }
    #[inline(always)]
    fn le(self, o: Self) -> bool {
        self <= o
    }
    #[inline(always)]
    fn gt(self, o: Self) -> bool {
        self > o
    }
    #[inline(always)]
    fn ge(self, o: Self) -> bool {
        self >= o
    }
    #[inline(always)]
    fn eq_num(self, o: Self) -> bool {
        self == o
    }
    #[inline(always)]
    fn is_nan(self) -> bool {
        self != self
    }
    fn to_uint32(self) -> u32 {
        to_uint32_f64(self)
    }
    #[inline(always)]
    fn floor(self) -> Self {
        f64::floor(self)
    }
    #[inline(always)]
    fn ceil(self) -> Self {
        f64::ceil(self)
    }
}

/// ECMAScript ToUint32 on a native double.
#[cfg(feature = "std")]
pub fn to_uint32_f64(v: f64) -> u32 {
    if !v.is_finite() {
        return 0;
    }
    let t = v.trunc();
    let m = t % 4294967296.0;
    let m = if m < 0.0 { m + 4294967296.0 } else { m };
    m as u32
}

/// AS3 idioms with their exact NaN / signed-zero behaviour.
pub mod as3 {
    use super::Real;
    /// `b2Math.b2Abs`: `a > 0 ? a : -a` (so b2Abs(0.0) == -0.0).
    #[inline(always)]
    pub fn b2_abs<R: Real>(a: R) -> R {
        if a.gt(R::zero()) {
            a
        } else {
            -a
        }
    }
    /// `b2Min`: `a < b ? a : b`.
    #[inline(always)]
    pub fn b2_min<R: Real>(a: R, b: R) -> R {
        if a.lt(b) {
            a
        } else {
            b
        }
    }
    /// `b2Max`: `a > b ? a : b`.
    #[inline(always)]
    pub fn b2_max<R: Real>(a: R, b: R) -> R {
        if a.gt(b) {
            a
        } else {
            b
        }
    }
    /// `b2Clamp`: `b2Max(low, b2Min(a, high))`.
    #[inline(always)]
    pub fn b2_clamp<R: Real>(a: R, low: R, high: R) -> R {
        b2_max(low, b2_min(a, high))
    }
    /// AVM2 `Math.abs`: IEEE abs (clears the sign bit).
    #[inline(always)]
    pub fn math_abs<R: Real>(a: R) -> R {
        let w = a.to_words();
        R::from_words([w[0] & 0x7FFF_FFFF, w[1]])
    }
    /// Flash 32 `Math.min(a,b)`: preserve the first NaN; equal values select b,
    /// including mixed signed zeros (measured in arithmetic-edges-v1).
    #[inline(always)]
    pub fn math_min<R: Real>(a: R, b: R) -> R {
        if a.is_nan() { return a; }
        if b.is_nan() { return b; }
        if a.lt(b) { a } else { b }
    }
    /// Flash 32 `Math.max(a,b)`: same NaN and equality ordering as Math.min.
    #[inline(always)]
    pub fn math_max<R: Real>(a: R, b: R) -> R {
        if a.is_nan() { return a; }
        if b.is_nan() { return b; }
        if a.gt(b) { a } else { b }
    }
    /// AVM2 `Math.round`: `floor(x + 0.5)` (Tamarin MathUtils::round); yields +0 for small negatives,
    /// as measured in the Flash 32 math probe.
    #[inline(always)]
    pub fn math_round<R: Real>(x: R) -> R {
        let half = R::from_words([0x3FE0_0000, 0]);
        (x + half).floor()
    }
    pub const MIN_VALUE_WORDS: [u32; 2] = [0, 1];
    pub const MAX_VALUE_WORDS: [u32; 2] = [0x7FEF_FFFF, 0xFFFF_FFFF];
    pub const PI_WORDS: [u32; 2] = [0x4009_21FB, 0x5444_2D18];
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn uint32() {
        assert_eq!(to_uint32_f64(-1.5), 4294967295);
        assert_eq!(to_uint32_f64(f64::NAN), 0);
        assert_eq!(to_uint32_f64(4294967296.0 + 5.0), 5);
        assert_eq!(to_uint32_f64(1e10), 1410065408);
    }
    #[test]
    fn abs_zero() {
        assert_eq!(as3::b2_abs(0.0f64).to_bits(), (-0.0f64).to_bits());
    }
}
