//! Transcendental functions matching the 32-bit Flash Player 32.
//!
//! Dispatch:
//! - On `i686-pc-windows-msvc` with `std`, sin/cos use the measured CRT through raw
//!   words. atan2 uses the player's x87 body, avoiding newer CRT changes to NaN
//!   payloads and underflow. The native f64 implementation uses this host math path.
//! - Otherwise (or with `portable-math`) u32 portable reimplementations are used.
//!   They match the recorded player math inputs and host differential fuzzing.
#![cfg_attr(not(feature = "std"), no_std)]
pub mod candidates;
mod x87_wide;
mod x87_constants;
mod x87_trig;
mod x87_atan;
mod x87_atan_constants;

use hw_real::Real;
#[cfg(all(target_arch = "x86", target_os = "windows", feature = "std", not(feature = "portable-math")))]
mod host_x87;

/// (cos, sin) of `angle` with Flash 32 x86 results.
#[inline(always)]
pub fn sin_cos<R: Real>(angle: R) -> (R, R) {
    #[cfg(all(target_arch = "x86", target_os = "windows", feature = "std", not(feature = "portable-math")))]
    {
        // Host CRT path: the result is independent of R's storage implementation.
        let a = f64::from_bits(((angle.to_words()[0] as u64) << 32) | angle.to_words()[1] as u64);
        let c = a.cos();
        let s = a.sin();
        return (R::from_words([(c.to_bits() >> 32) as u32, c.to_bits() as u32]), R::from_words([(s.to_bits() >> 32) as u32, s.to_bits() as u32]));
    }
    #[allow(unreachable_code)]
    portable::sin_cos(angle)
}

/// Math.atan2(y, x) with Flash 32 x86 results.
#[inline(always)]
pub fn atan2<R: Real>(y: R, x: R) -> R {
    #[cfg(all(target_arch = "x86", target_os = "windows", feature = "std", not(feature = "portable-math")))]
    {
        let yy = f64::from_bits(((y.to_words()[0] as u64) << 32) | y.to_words()[1] as u64);
        let xx = f64::from_bits(((x.to_words()[0] as u64) << 32) | x.to_words()[1] as u64);
        let r = host_x87::atan2(yy, xx);
        return R::from_words([(r.to_bits() >> 32) as u32, r.to_bits() as u32]);
    }
    #[allow(unreachable_code)]
    portable::atan2(y, x)
}

/// Math.pow(x, 2): measured equal to x * x.
#[inline(always)]
pub fn pow2<R: Real>(x: R) -> R {
    x * x
}

/// Portable implementations validated against the recorded player inputs and
/// host differential fuzzing. See docs/MATH.md for measured scope.
pub mod portable {
    use hw_real::Real;
    pub fn sin_cos<R: Real>(angle: R) -> (R, R) {
        if angle.to_words()[0] & 0x7ff0_0000 == 0x7ff0_0000 {
            // Flash math-v1: infinities produce negative invalid NaN; NaNs keep
            // their payload and sign. Subtraction also quiets signaling inputs.
            let result = angle - angle;
            return (result, result);
        }
        if super::candidates::flash11_sse2::supported(angle) {
            return super::candidates::flash11_sse2::sin_cos(angle);
        }
        let (cos, sin) = super::x87_trig::sin_cos(angle.to_words());
        (R::from_words(cos), R::from_words(sin))
    }
    pub fn atan2<R: Real>(y: R, x: R) -> R {
        R::from_words(super::x87_atan::atan2(y.to_words(), x.to_words()))
    }
}
