//! Standalone FSIN/FCOS, derived from Bochs. See x87-LICENSE.txt.
//! The player calls these separately; FSINCOS has different last-bit behavior.
use super::x87_constants::{A, B, COS, SIN, U, V};
use super::x87_wide::{a, an, m, me, mn, X};

pub(super) fn sin_cos(words: [u32; 2]) -> ([u32; 2], [u32; 2]) {
    let mut x = X::from_words(words);
    if x.exp >= 63 {
        // The player's FPREM1 divisor is an exact 64-bit integer: extended pi
        // scaled by 2^62. Keep the extended remainder until FSIN/FCOS completes.
        let divisor = X::extended(0xc90f_daa2, 0x2168_c235, 63);
        let (mut rem, q) = x.remainder(divisor);
        let half = divisor.scale(-1);
        if half.abs_lt(rem) || (half.abs_eq(rem) && q & 1 != 0) {
            rem = rem.add(divisor.negative(), 128, false);
        }
        x = rem.sign(rem.neg ^ x.neg);
    }
    let pi2 = X::quad([0x3fff_921f, 0xb544_42d1, 0x8469_8000, 0]);
    let (mut r, mut q) = x.remainder(pi2);
    let rn = pi2.scale(-1).abs_lt(r);
    if rn {
        r = pi2.add(r.negative(), 128, false);
        q = q.wrapping_add(1);
    }
    (
        evaluate(r, rn, q.wrapping_add(1), false).to_words(),
        evaluate(r, rn, q, x.neg).to_words(),
    )
}

fn evaluate(r: X, rn: bool, q: u32, sign: bool) -> X {
    let odd = q & 1 != 0;
    let s1 = sign ^ (q & 2 != 0);
    if !r.is_zero() && r.exp >= -2 {
        let top = r.top3();
        let index = ((((r.exp + 16383) & 1) as u32) * 4 + (top & 3)) as usize;
        let c = X::integer(top * 2 + 1).scale(r.exp - 3);
        let sigma = SIN[index];
        let gamma = COS[index];
        let d = a(r, c.negative());
        let e = m(d, d);
        let mut p = U[3];
        let mut q = V[3];
        for j in (0..3).rev() {
            p = an(m(e, p), U[j]);
            q = an(m(e, q), V[j]);
        }
        p = an(d, m(m(e, p), d));
        q = mn(e, q);
        if !odd {
            let sigma = sigma.sign(s1 ^ rn);
            let gamma = gamma.sign(s1 ^ rn);
            an(sigma, a(m(sigma, q), m(gamma, p)))
        } else {
            let sigma = sigma.sign(s1);
            let gamma = gamma.sign(s1);
            an(gamma, a(m(sigma, p.negative()), m(gamma, q)))
        }
    } else {
        let s = me(r, r);
        let t = me(s, s);
        let coeff = if odd { B } else { A };
        let p = m(s, an(m(t, an(m(t, coeff[4]), coeff[2])), coeff[0]));
        let q = m(t, an(m(t, an(m(t, coeff[5]), coeff[3])), coeff[1]));
        if !odd {
            let rr = r.sign(s1 ^ rn);
            an(rr, m(rr, an(p, q)))
        } else {
            let f = a(p, q);
            an(X::ONE.sign(s1), f.sign(f.neg ^ s1))
        }
    }
}
