//! Finite extended arithmetic for portable x87 transcendental evaluation.
//! u32 limbs only; fixed storage, no native floats or wide integer operations.

#[derive(Clone, Copy, Debug, Eq)]
struct U([u32; 8]);
impl PartialEq for U {
    fn eq(&self, other: &Self) -> bool {
        for i in 0..8 { if self.0[i] != other.0[i] { return false; } }
        true
    }
}
impl U {
    const ZERO: Self = Self([0; 8]);
    fn zero(self) -> bool {
        for i in 0..8 { if self.0[i] != 0 { return false; } }
        true
    }
    fn lt(self, b: Self) -> bool {
        for i in (0..8).rev() {
            if self.0[i] != b.0[i] {
                return self.0[i] < b.0[i];
            }
        }
        false
    }
    fn add(self, b: Self) -> (Self, bool) {
        let mut r = Self::ZERO;
        let mut carry = false;
        for i in 0..8 {
            let (v, c1) = self.0[i].overflowing_add(b.0[i]);
            let (v, c2) = v.overflowing_add(carry as u32);
            r.0[i] = v;
            carry = c1 || c2;
        }
        (r, carry)
    }
    fn sub(self, b: Self) -> Self {
        let mut r = Self::ZERO;
        let mut borrow = false;
        for i in 0..8 {
            let (v, b1) = self.0[i].overflowing_sub(b.0[i]);
            let (v, b2) = v.overflowing_sub(borrow as u32);
            r.0[i] = v;
            borrow = b1 || b2;
        }
        r
    }
    fn shl(self, n: u32) -> Self {
        let mut r = Self::ZERO;
        if n >= 256 {
            return r;
        }
        let words = (n / 32) as usize;
        let bits = n % 32;
        for i in words..8 {
            r.0[i] = self.0[i - words] << bits;
            if bits != 0 && i > words {
                r.0[i] |= self.0[i - words - 1] >> (32 - bits);
            }
        }
        r
    }
    fn shr(self, n: u32) -> Self {
        let mut r = Self::ZERO;
        if n >= 256 {
            return r;
        }
        let words = (n / 32) as usize;
        let bits = n % 32;
        for i in 0..8 - words {
            r.0[i] = self.0[i + words] >> bits;
            if bits != 0 && i + words + 1 < 8 {
                r.0[i] |= self.0[i + words + 1] << (32 - bits);
            }
        }
        r
    }
    fn jam(self, n: u32) -> Self {
        let mut r = self.shr(n);
        if r.shl(n) != self {
            r.0[0] |= 1;
        }
        r
    }
    fn clz(self) -> u32 {
        for i in (0..8).rev() {
            if self.0[i] != 0 {
                return (7 - i) as u32 * 32 + self.0[i].leading_zeros();
            }
        }
        256
    }
    fn bit(n: u32) -> Self {
        let mut r = Self::ZERO;
        if n < 256 {
            r.0[(n / 32) as usize] = 1 << (n % 32);
        }
        r
    }
    fn has(self, n: u32) -> bool {
        n < 256 && self.0[(n / 32) as usize] & (1 << (n % 32)) != 0
    }
}

/// Normalized magnitude `sig / 2^255 * 2^exp`. Public operations retain at
/// most 128 significant bits. Exponents need only cover binary64 intermediates.
#[derive(Clone, Copy, Debug)]
pub(super) struct X {
    sig: U,
    pub exp: i32,
    pub neg: bool,
}
impl X {
    pub const ZERO: Self = Self {
        sig: U::ZERO,
        exp: 0,
        neg: false,
    };
    pub const ONE: Self = Self::extended(0x8000_0000, 0, 0);
    pub const fn extended(hi: u32, lo: u32, exp: i32) -> Self {
        Self {
            sig: U([0, 0, 0, 0, 0, 0, lo, hi]),
            exp,
            neg: false,
        }
    }
    /// A finite, normal IEEE binary128 constant in most-significant-word order.
    pub const fn quad(w: [u32; 4]) -> Self {
        let h = (w[0] & 0xffff) | 0x10000;
        Self {
            sig: U([
                0,
                0,
                0,
                0,
                w[3] << 15,
                (w[2] << 15) | (w[3] >> 17),
                (w[1] << 15) | (w[2] >> 17),
                (h << 15) | (w[1] >> 17),
            ]),
            exp: ((w[0] >> 16) & 0x7fff) as i32 - 16383,
            neg: w[0] >> 31 != 0,
        }
    }
    pub fn from_words(w: [u32; 2]) -> Self {
        let e = (w[0] >> 20) & 0x7ff;
        let h = (w[0] & 0xfffff) | if e != 0 { 0x100000 } else { 0 };
        let mut r = Self::extended(
            (h << 11) | (w[1] >> 21),
            w[1] << 11,
            if e == 0 { -1022 } else { e as i32 - 1023 },
        );
        r.neg = w[0] >> 31 != 0;
        r.normalized()
    }
    pub fn integer(v: u32) -> Self {
        let mut r = Self::ZERO;
        r.sig.0[7] = v;
        r.exp = 31;
        r.normalized()
    }
    pub fn is_zero(self) -> bool {
        self.sig.zero()
    }
    pub fn sign(mut self, negative: bool) -> Self {
        self.neg = negative;
        self
    }
    pub fn negative(self) -> Self {
        self.sign(!self.neg)
    }
    pub fn scale(mut self, n: i32) -> Self {
        self.exp += n;
        self
    }
    pub fn abs_lt(self, b: Self) -> bool {
        if self.is_zero() {
            return !b.is_zero();
        }
        if b.is_zero() {
            return false;
        }
        self.exp < b.exp || (self.exp == b.exp && self.sig.lt(b.sig))
    }
    pub fn abs_eq(self, b: Self) -> bool {
        (self.is_zero() && b.is_zero()) || (self.exp == b.exp && self.sig == b.sig)
    }
    fn normalized(mut self) -> Self {
        let shift = self.sig.clz();
        if shift < 256 {
            self.sig = self.sig.shl(shift);
            self.exp -= shift as i32;
        } else {
            self.exp = 0;
        }
        self
    }
    pub fn round(mut self, precision: u32, nearest: bool) -> Self {
        if self.is_zero() {
            return self;
        }
        let drop = 256 - precision;
        let kept = self.sig.shr(drop).shl(drop);
        if nearest
            && self.sig.has(drop - 1)
            && (self.sig != kept.add(U::bit(drop - 1)).0 || self.sig.has(drop))
        {
            let (sig, carry) = kept.add(U::bit(drop));
            self.sig = sig;
            if carry {
                self.sig = U::bit(255);
                self.exp += 1;
            }
        } else {
            self.sig = kept;
        }
        self
    }
    pub fn add(self, b: Self, precision: u32, nearest: bool) -> Self {
        if self.is_zero() {
            return b.round(precision, nearest);
        }
        if b.is_zero() {
            return self.round(precision, nearest);
        }
        let (large, small) = if self.abs_lt(b) { (b, self) } else { (self, b) };
        let mut r = large;
        let small_sig = small.sig.jam((large.exp - small.exp) as u32);
        if large.neg == small.neg {
            let (sum, carry) = large.sig.add(small_sig);
            r.sig = sum;
            if carry {
                r.sig = r.sig.jam(1);
                r.sig.0[7] |= 0x8000_0000;
                r.exp += 1;
            }
        } else {
            r.sig = large.sig.sub(small_sig);
            r = r.normalized();
            if r.is_zero() {
                r.neg = false;
            }
        }
        r.round(precision, nearest)
    }
    pub fn mul(self, b: Self, precision: u32, nearest: bool) -> Self {
        let mut product = U::ZERO;
        // 16-bit limbs allow multiply-accumulate with a single u32 carry.
        let mut p = [0u32; 16];
        for i in 0..8 {
            let av = (self.sig.0[4 + i / 2] >> ((i % 2) * 16)) & 0xffff;
            let mut carry = 0;
            for j in 0..8 {
                let bv = (b.sig.0[4 + j / 2] >> ((j % 2) * 16)) & 0xffff;
                let v = p[i + j] + av * bv + carry;
                p[i + j] = v & 0xffff;
                carry = v >> 16;
            }
            p[i + 8] = carry;
        }
        for i in 0..8 {
            product.0[i] = p[i * 2] | (p[i * 2 + 1] << 16);
        }
        Self {
            sig: product,
            exp: self.exp + b.exp + 1,
            neg: self.neg ^ b.neg,
        }
        .normalized()
        .round(precision, nearest)
    }
    pub fn div(self, b: Self, precision: u32, nearest: bool) -> Self {
        if self.is_zero() {
            return self.sign(self.neg ^ b.neg);
        }
        let mut rem = self.sig;
        let mut exp = self.exp - b.exp;
        if rem.lt(b.sig) {
            rem = rem.shl(1);
            exp -= 1;
        }
        // A shift above can carry out of bit 255. The normalized quotient
        // still starts with one and subtraction modulo 2^256 is exact here.
        rem = rem.sub(b.sig);
        let mut quotient = U::bit(255);
        for i in 1..precision + 2 {
            let carry = rem.has(255);
            rem = rem.shl(1);
            if carry || !rem.lt(b.sig) {
                rem = rem.sub(b.sig);
                quotient.0[((255 - i) / 32) as usize] |= 1 << ((255 - i) % 32);
            }
        }
        if !rem.zero() {
            quotient.0[0] |= 1;
        }
        Self {
            sig: quotient,
            exp,
            neg: self.neg ^ b.neg,
        }
        .round(precision, nearest)
    }
    pub fn to_words(self) -> [u32; 2] {
        let sign = (self.neg as u32) << 31;
        if self.is_zero() || self.exp < -1075 {
            return [sign, 0];
        }
        if self.exp == -1075 {
            return [sign, (self.sig != U::bit(255)) as u32];
        }
        let precision = if self.exp < -1022 {
            (self.exp + 1075) as u32
        } else {
            53
        };
        let r = self.round(precision, true);
        if r.exp > 1023 {
            return [sign | 0x7ff0_0000, 0];
        }
        let shift = if r.exp < -1022 {
            (255 - r.exp - 1074) as u32
        } else {
            203
        };
        let sig = r.sig.shr(shift);
        let exp = if r.exp < -1022 {
            0
        } else {
            (r.exp + 1023) as u32
        };
        [sign | (exp << 20) | (sig.0[1] & 0xfffff), sig.0[0]]
    }
    /// Exact remainder for the bounded-width operands used in argument reduction.
    /// Returns low quotient bits; numerator/divisor magnitudes are positive.
    pub fn remainder(self, divisor: Self) -> (Self, u32) {
        let mut r = self.sign(false);
        let mut quotient = 0;
        while !r.abs_lt(divisor) {
            let mut shift = r.exp - divisor.exp;
            let mut d = divisor.scale(shift);
            if r.abs_lt(d) {
                shift -= 1;
                d = divisor.scale(shift);
            }
            r = r.add(d.negative(), 128, false);
            if shift < 32 {
                quotient |= 1 << shift;
            }
        }
        (r, quotient)
    }
    /// Top three significand bits, for the sine/cosine table breakpoint.
    pub fn top3(self) -> u32 {
        self.sig.0[7] >> 29
    }
}

pub(super) fn m(a: X, b: X) -> X {
    a.mul(b, 67, false)
}
pub(super) fn me(a: X, b: X) -> X {
    m(a, b.round(64, false))
}
pub(super) fn mn(a: X, b: X) -> X {
    a.mul(b.round(64, false), 113, true).round(64, true)
}
pub(super) fn a(a: X, b: X) -> X {
    a.add(b, 67, false)
}
pub(super) fn an(a: X, b: X) -> X {
    a.add(b, 113, true).round(64, true)
}
