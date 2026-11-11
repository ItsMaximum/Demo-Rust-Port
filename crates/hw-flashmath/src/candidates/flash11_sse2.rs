//! Flash 11.4 x86 SSE2 sin/cos as reverse-engineered in an earlier project (scalar f64 form),
//! rewritten over the `Real` trait. First candidate for Flash 32 x86.
use hw_real::{rl, Real};

use super::trig_table::TABLE;

/// Returns (cos, sin). Callers must check `supported(angle)` first.
pub fn sin_cos<R: Real>(angle: R) -> (R, R) {
    let exponent_word = (angle.to_words()[0] >> 16) & 0x7fff;
    if exponent_word < 0x3030 {
        return (R::one(), angle);
    }
    (reduced_trig(angle, 16), reduced_trig(angle, 0))
}

pub fn supported<R: Real>(angle: R) -> bool {
    let w = angle.to_words()[0];
    let e = (w >> 20) & 0x7ff;
    if e == 0x7ff {
        return false;
    }
    let exponent_word = (w >> 16) & 0x7fff;
    exponent_word <= 0x40f5
}

fn reduced_trig<R: Real>(angle: R, phase: i32) -> R {
    // Three-part pi/32 range reduction, nearest-even index selection.
    let scaled = angle * rl!(0x40245f30, 0x6dc9c883);
    let magic: R = rl!(0x43380000, 0x00000000); // 6755399441055744.0
    let multiple = (scaled + magic) - magic;
    let index = multiple.to_int32();
    let first = multiple * rl!(0x3fb921fb, 0x54400000);
    let second = multiple * rl!(0x3d90b461, 0x1a600000);
    let third = multiple * rl!(0x3b63198a, 0x2e037073);
    let initial = angle - first;
    let residual = initial - second;
    let reduction_error = (initial - residual) - second;
    let tail = third - reduction_error;
    let row = TABLE[((index + phase) & 63) as usize];
    let cos_delta: R = R::from_words(row[0]);
    let sin_high: R = R::from_words(row[1]);
    let sin_low: R = R::from_words(row[2]);
    let sigma: R = R::from_words(row[3]);
    let square = residual * residual;
    let fourth = square * square;
    let c9: R = rl!(0x3ec71de3, 0xa556c734); // 1/362880
    let c7: R = rl!(0x3f2a01a0, 0x1a01a01a); // 1/5040
    let c8: R = rl!(0x3efa01a0, 0x1a01a01a); // 1/40320
    let c6: R = rl!(0x3f56c16c, 0x16c16c17); // 1/720
    let c5: R = rl!(0x3f811111, 0x11111111); // 1/120
    let c3: R = rl!(0x3fc55555, 0x55555555); // 1/6
    let c4: R = rl!(0x3fa55555, 0x55555555); // 1/24
    let half: R = rl!(0x3fe00000, 0x00000000);
    let odd_high = ((c9 * initial) * residual - c7) * fourth;
    let even_high = ((c8 * initial) * residual - c6) * fourth;
    let odd = (c5 * square - c3) + odd_high;
    let even = (c4 * square - half) + even_high;
    let cosine = cos_delta + sigma;
    let odd_scale = (cosine * residual) * square;
    let even_scale = sin_high * square;
    let tail_scale = sin_high * residual - cosine;
    let coarse = sigma * residual;
    let fine = cos_delta * residual;
    let sum = coarse + sin_high;
    let main = fine + sum;
    let compensation1 = (sin_high - sum) + coarse;
    let compensation2 = (sum - main) + fine;
    let correction = (((((tail * tail_scale) + sin_low) + compensation1) + compensation2) + odd * odd_scale) + even * even_scale;
    main + correction
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn red_ball_recorded_angles() {
        let (c, s) = sin_cos::<f64>(f64::from_bits(0x3fcacdf7c0c95d28));
        assert_eq!(c.to_bits(), 0x3fef4d08c9077396);
        assert_eq!(s.to_bits(), 0x3fca9becc44885dc);
        assert_eq!(sin_cos::<f64>(f64::from_bits(0x4027d0e164d76bea)).0.to_bits(), 0x3fe94fa3ab1fd401);
    }
    #[test]
    fn constants_are_exact() {
        assert_eq!((1.0f64 / 362880.0).to_bits(), 0x3ec71de3a556c734);
        assert_eq!((1.0f64 / 5040.0).to_bits(), 0x3f2a01a01a01a01a);
        assert_eq!((1.0f64 / 40320.0).to_bits(), 0x3efa01a01a01a01a);
        assert_eq!((1.0f64 / 720.0).to_bits(), 0x3f56c16c16c16c17);
        assert_eq!((1.0f64 / 120.0).to_bits(), 0x3f81111111111111);
        assert_eq!((1.0f64 / 6.0).to_bits(), 0x3fc5555555555555);
        assert_eq!((1.0f64 / 24.0).to_bits(), 0x3fa5555555555555);
    }
}
