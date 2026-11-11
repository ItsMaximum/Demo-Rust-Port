//! FPATAN approximation derived from Bochs; see x87-LICENSE.txt.
//! Followed by the original player's extended pi subtraction and sign change.
use super::x87_atan_constants::{A, ATAN, B, PIBY2};
use super::x87_wide::{a, an, m, mn, X};

fn nan(w: [u32; 2]) -> bool {
    w[0] & 0x7ff0_0000 == 0x7ff0_0000 && (w[0] & 0xfffff != 0 || w[1] != 0)
}

pub(super) fn atan2(y: [u32; 2], x: [u32; 2]) -> [u32; 2] {
    if nan(y) || nan(x) {
        let mut chosen = if !nan(y) {
            x
        } else if !nan(x) {
            y
        } else {
            // Loading each binary64 operand into x87 quiets it before FADDP's
            // payload comparison. On an equal payload the positive NaN wins.
            let ym = [(y[0] & 0xfffff) | 0x80000, y[1]];
            let xm = [(x[0] & 0xfffff) | 0x80000, x[1]];
            let greater = ym[0] > xm[0] || (ym[0] == xm[0] && ym[1] > xm[1]);
            let equal = ym[0] == xm[0] && ym[1] == xm[1];
            if greater || (equal && y[0] >> 31 < x[0] >> 31) {
                y
            } else {
                x
            }
        };
        chosen[0] |= 0x80000;
        return chosen;
    }
    let sx = x[0] >> 31 != 0;
    let sy = y[0] >> 31 != 0;
    let xi = x[0] & 0x7fffffff == 0x7ff00000;
    let yi = y[0] & 0x7fffffff == 0x7ff00000;
    let xx = X::from_words(x).sign(false);
    let yy = X::from_words(y).sign(false);
    let mut r = if yi && xi {
        PIBY2.scale(-1).round(64, true)
    } else if yi {
        PIBY2.round(64, true)
    } else if xi || yy.is_zero() {
        X::ZERO
    } else if xx.is_zero() {
        PIBY2.round(64, true)
    } else {
        positive(yy, xx)
    };
    if sx {
        // FLDPI; FSUBRP under the player's 64-bit-significand control word.
        r = an(X::extended(0xc90f_daa2, 0x2168_c235, 1), r.negative());
    }
    r.sign(sy).to_words()
}

fn positive(y: X, x: X) -> X {
    let swap = x.abs_lt(y);
    let (u, v) = if swap { (y, x) } else { (x, y) };
    let v = v.scale(-u.exp);
    let u = u.scale(-u.exp);
    let (mut r, mut k) = v.scale(5).remainder(u);
    if u.scale(-1).abs_lt(r) {
        r = r.add(u.negative(), 113, true);
        k += 1;
    }
    let (f, g) = if k > 1 {
        let denominator = a(u.scale(5), m(v, X::integer(k)));
        let mut z = r.div(denominator, 67, false);
        let c = mn(z, z);
        let h = m(c, c);
        z = z.sign(z.neg ^ swap);
        let t = m(z, c);
        let p = a(A[0], m(h, A[2]));
        let q = m(c, an(A[1], m(h, A[3])));
        (ATAN[k as usize].sign(swap), a(z, m(t, an(p, q))))
    } else {
        let z = v.div(u, 67, false).sign(swap);
        if z.exp < -40 {
            (z, X::ZERO)
        } else {
            let c = mn(z, z);
            let h = m(c, c);
            let t = m(z, c);
            let p = a(m(h, an(m(h, B[4]), B[2])), B[0]);
            let q = m(a(m(h, an(m(h, B[5]), B[3])), B[1]), c);
            (z, m(t, an(p, q)))
        }
    };
    if swap {
        an(PIBY2, a(f, g))
    } else {
        an(f, g)
    }
}
