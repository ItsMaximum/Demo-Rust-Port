//! fdlibm / musl atan and atan2 (the algorithm used by `libm::atan2`), written over `Real` so it
//! needs no host math. Candidate for the i686 CRT `atan2`.
use hw_real::{rl, Real};

fn hi<R: Real>(v: R) -> u32 {
    v.to_words()[0]
}
fn lo<R: Real>(v: R) -> u32 {
    v.to_words()[1]
}
fn fabs<R: Real>(v: R) -> R {
    let w = v.to_words();
    R::from_words([w[0] & 0x7fff_ffff, w[1]])
}

pub fn atan<R: Real>(x: R) -> R {
    let atanhi: [R; 4] = [rl!(0x3FDDAC67, 0x0561BB4F), rl!(0x3FE921FB, 0x54442D18), rl!(0x3FEF730B, 0xD281F69B), rl!(0x3FF921FB, 0x54442D18)];
    let atanlo: [R; 4] = [rl!(0x3C7A2B7F, 0x222F65E2), rl!(0x3C81A626, 0x33145C07), rl!(0x3C700788, 0x7AF0CBBD), rl!(0x3C91A626, 0x33145C07)];
    let at: [R; 11] = [
        rl!(0x3FD55555, 0x5555550D),
        rl!(0xBFC99999, 0x9998EBC4),
        rl!(0x3FC24924, 0x920083FF),
        rl!(0xBFBC71C6, 0xFE231671),
        rl!(0x3FB745CD, 0xC54C206E),
        rl!(0xBFB3B0F2, 0xAF749A6D),
        rl!(0x3FB10D66, 0xA0D03D51),
        rl!(0xBFADDE2D, 0x52DEFD9A),
        rl!(0x3FA97B4B, 0x24760DEB),
        rl!(0xBFA2B444, 0x2C6A6C2F),
        rl!(0x3F90AD3A, 0xE322DA11),
    ];
    let one: R = R::one();
    let hx = hi(x);
    let ix = hx & 0x7fff_ffff;
    let mut x = x;
    let id: i32;
    if ix >= 0x4410_0000 {
        // |x| >= 2^66
        if ix > 0x7ff0_0000 || (ix == 0x7ff0_0000 && lo(x) != 0) {
            return x + x;
        }
        return if hx & 0x8000_0000 == 0 { atanhi[3] + atanlo[3] } else { -atanhi[3] - atanlo[3] };
    }
    if ix < 0x3fdc_0000 {
        // |x| < 0.4375
        if ix < 0x3e20_0000 {
            // |x| < 2^-29
            let huge: R = rl!(0x7E37E43C, 0x8800759C); // 1e300
            if (huge + x).gt(one) {
                return x;
            }
        }
        id = -1;
    } else {
        x = fabs(x);
        if ix < 0x3ff3_0000 {
            if ix < 0x3fe6_0000 {
                id = 0;
                x = (x + x - one) / (R::from_i32(2) + x);
            } else {
                id = 1;
                x = (x - one) / (x + one);
            }
        } else if ix < 0x4003_8000 {
            id = 2;
            let c15: R = rl!(0x3FF80000, 0x00000000);
            x = (x - c15) / (one + c15 * x);
        } else {
            id = 3;
            x = -one / x;
        }
    }
    let z = x * x;
    let w = z * z;
    let s1 = z * (at[0] + w * (at[2] + w * (at[4] + w * (at[6] + w * (at[8] + w * at[10])))));
    let s2 = w * (at[1] + w * (at[3] + w * (at[5] + w * (at[7] + w * at[9]))));
    if id < 0 {
        return x - x * (s1 + s2);
    }
    let id = id as usize;
    let z = atanhi[id] - ((x * (s1 + s2) - atanlo[id]) - x);
    if hx & 0x8000_0000 != 0 {
        -z
    } else {
        z
    }
}

pub fn atan2<R: Real>(y: R, x: R) -> R {
    let tiny: R = rl!(0x01A56E1F, 0xC2F8F359); // 1e-300
    let pi_o_4: R = rl!(0x3FE921FB, 0x54442D18);
    let pi_o_2: R = rl!(0x3FF921FB, 0x54442D18);
    let pi: R = rl!(0x400921FB, 0x54442D18);
    let pi_lo: R = rl!(0x3CA1A626, 0x33145C07);
    let hx = hi(x);
    let lx = lo(x);
    let ix = hx & 0x7fff_ffff;
    let hy = hi(y);
    let ly = lo(y);
    let iy = hy & 0x7fff_ffff;
    if (ix | ((lx | lx.wrapping_neg()) >> 31)) > 0x7ff0_0000 || (iy | ((ly | ly.wrapping_neg()) >> 31)) > 0x7ff0_0000 {
        return x + y;
    }
    if hx.wrapping_sub(0x3ff0_0000) | lx == 0 {
        return atan(y);
    }
    let m = ((hy >> 31) & 1) | ((hx >> 30) & 2);
    if iy | ly == 0 {
        return match m {
            0 | 1 => y,
            2 => pi + tiny,
            _ => -pi - tiny,
        };
    }
    if ix | lx == 0 {
        return if hy & 0x8000_0000 != 0 { -pi_o_2 - tiny } else { pi_o_2 + tiny };
    }
    if ix == 0x7ff0_0000 {
        if iy == 0x7ff0_0000 {
            return match m {
                0 => pi_o_4 + tiny,
                1 => -pi_o_4 - tiny,
                2 => R::from_i32(3) * pi_o_4 + tiny,
                _ => -R::from_i32(3) * pi_o_4 - tiny,
            };
        }
        return match m {
            0 => R::zero(),
            1 => -R::zero(),
            2 => pi + tiny,
            _ => -pi - tiny,
        };
    }
    if iy == 0x7ff0_0000 {
        return if hy & 0x8000_0000 != 0 { -pi_o_2 - tiny } else { pi_o_2 + tiny };
    }
    let k = (iy as i32).wrapping_sub(ix as i32) >> 20;
    let z = if k > 60 {
        { let half: R = rl!(0x3FE00000, 0x00000000); pi_o_2 + half * pi_lo }
    } else if hx & 0x8000_0000 != 0 && k < -60 {
        R::zero()
    } else {
        atan(fabs(y / x))
    };
    match m {
        0 => z,
        1 => -z,
        2 => pi - (z - pi_lo),
        _ => (z - pi_lo) - pi,
    }
}
