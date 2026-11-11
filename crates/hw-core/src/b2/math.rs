//! b2Vec2 / b2Mat22 / b2XForm / b2Sweep / b2Math with the exact AS3 operation order.
//! Vectors are values (AS3 `new b2Vec2` allocations become locals); in-place AS3 methods map to
//! `&mut self` methods with the same statement order.
use hw_real::as3::{b2_max, b2_min};
use hw_real::Real;

use super::settings::min_value;

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Vec2<R> {
    pub x: R,
    pub y: R,
}

impl<R: Real> Vec2<R> {
    #[inline(always)]
    pub fn new(x: R, y: R) -> Self {
        Vec2 { x, y }
    }
    #[inline(always)]
    pub fn zero() -> Self {
        Vec2 { x: R::zero(), y: R::zero() }
    }
    #[inline(always)]
    pub fn set(&mut self, x: R, y: R) {
        self.x = x;
        self.y = y;
    }
    #[inline(always)]
    pub fn set_v(&mut self, v: Vec2<R>) {
        self.x = v.x;
        self.y = v.y;
    }
    #[inline(always)]
    pub fn set_zero(&mut self) {
        self.x = R::zero();
        self.y = R::zero();
    }
    #[inline(always)]
    pub fn add(&mut self, v: Vec2<R>) {
        self.x = self.x + v.x;
        self.y = self.y + v.y;
    }
    #[inline(always)]
    pub fn subtract(&mut self, v: Vec2<R>) {
        self.x = self.x - v.x;
        self.y = self.y - v.y;
    }
    #[inline(always)]
    pub fn multiply(&mut self, a: R) {
        self.x = self.x * a;
        self.y = self.y * a;
    }
    #[inline(always)]
    pub fn length(self) -> R {
        (self.x * self.x + self.y * self.y).sqrt()
    }
    #[inline(always)]
    pub fn length_squared(self) -> R {
        self.x * self.x + self.y * self.y
    }
    /// b2Vec2.MulM: this = M * this
    #[inline(always)]
    pub fn mul_m(&mut self, m: Mat22<R>) {
        let t = self.x;
        self.x = m.col1.x * t + m.col2.x * self.y;
        self.y = m.col1.y * t + m.col2.y * self.y;
    }
    /// b2Vec2.MulTM: this = M^T * this
    #[inline(always)]
    pub fn mul_tm(&mut self, m: Mat22<R>) {
        let t = b2_dot(*self, m.col1);
        self.y = b2_dot(*self, m.col2);
        self.x = t;
    }
    #[inline(always)]
    pub fn min_v(&mut self, v: Vec2<R>) {
        self.x = if self.x.lt(v.x) { self.x } else { v.x };
        self.y = if self.y.lt(v.y) { self.y } else { v.y };
    }
    #[inline(always)]
    pub fn max_v(&mut self, v: Vec2<R>) {
        self.x = if self.x.gt(v.x) { self.x } else { v.x };
        self.y = if self.y.gt(v.y) { self.y } else { v.y };
    }
    /// b2Vec2.Normalize
    #[inline(always)]
    pub fn normalize(&mut self) -> R {
        let len = (self.x * self.x + self.y * self.y).sqrt();
        if len.lt(min_value::<R>()) {
            return R::zero();
        }
        let inv = R::one() / len;
        self.x = self.x * inv;
        self.y = self.y * inv;
        len
    }
    /// b2Vec2.CrossVF: this = cross(this, s)
    #[inline(always)]
    pub fn cross_vf(&mut self, s: R) {
        let t = self.x;
        self.x = s * self.y;
        self.y = -s * t;
    }
    /// b2Vec2.CrossFV: this = cross(s, this)
    #[inline(always)]
    pub fn cross_fv(&mut self, s: R) {
        let t = self.x;
        self.x = -s * self.y;
        self.y = s * t;
    }
    #[inline(always)]
    pub fn negative(self) -> Vec2<R> {
        Vec2::new(-self.x, -self.y)
    }
    /// b2Vec2.Abs (in place, `< 0` test)
    #[inline(always)]
    pub fn abs(&mut self) {
        if self.x.lt(R::zero()) {
            self.x = -self.x;
        }
        if self.y.lt(R::zero()) {
            self.y = -self.y;
        }
    }
    #[inline(always)]
    pub fn is_valid(self) -> bool {
        b2_is_valid(self.x) && b2_is_valid(self.y)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Mat22<R> {
    pub col1: Vec2<R>,
    pub col2: Vec2<R>,
}

impl<R: Real> Mat22<R> {
    #[inline(always)]
    pub fn identity() -> Self {
        Mat22 { col1: Vec2::new(R::one(), R::zero()), col2: Vec2::new(R::zero(), R::one()) }
    }
    #[inline(always)]
    pub fn zero() -> Self {
        Mat22 { col1: Vec2::zero(), col2: Vec2::zero() }
    }
    #[inline(always)]
    pub fn from_cols(c1: Vec2<R>, c2: Vec2<R>) -> Self {
        Mat22 { col1: c1, col2: c2 }
    }
    /// `new b2Mat22(angle)` / `b2Mat22.Set(angle)`: uses the Flash-matching cos/sin.
    #[inline(always)]
    pub fn from_angle(angle: R) -> Self {
        let (c, s) = crate::b2::math::sin_cos(angle);
        Mat22 { col1: Vec2::new(c, s), col2: Vec2::new(-s, c) }
    }
    #[inline(always)]
    pub fn set(&mut self, angle: R) {
        let (c, s) = crate::b2::math::sin_cos(angle);
        self.col1.x = c;
        self.col2.x = -s;
        self.col1.y = s;
        self.col2.y = c;
    }
    #[inline(always)]
    pub fn set_identity(&mut self) {
        self.col1.x = R::one();
        self.col2.x = R::zero();
        self.col1.y = R::zero();
        self.col2.y = R::one();
    }
    #[inline(always)]
    pub fn set_zero(&mut self) {
        self.col1.x = R::zero();
        self.col2.x = R::zero();
        self.col1.y = R::zero();
        self.col2.y = R::zero();
    }
    #[inline(always)]
    pub fn set_vv(&mut self, c1: Vec2<R>, c2: Vec2<R>) {
        self.col1 = c1;
        self.col2 = c2;
    }
    #[inline(always)]
    pub fn set_m(&mut self, m: Mat22<R>) {
        self.col1 = m.col1;
        self.col2 = m.col2;
    }
    #[inline(always)]
    pub fn add_m(&mut self, m: Mat22<R>) {
        self.col1.x = self.col1.x + m.col1.x;
        self.col1.y = self.col1.y + m.col1.y;
        self.col2.x = self.col2.x + m.col2.x;
        self.col2.y = self.col2.y + m.col2.y;
    }
    #[inline(always)]
    pub fn abs(&mut self) {
        self.col1.abs();
        self.col2.abs();
    }
    /// b2Mat22.Invert(out): returns the inverse.
    #[inline(always)]
    pub fn invert(self) -> Mat22<R> {
        let a = self.col1.x;
        let b = self.col2.x;
        let c = self.col1.y;
        let d = self.col2.y;
        let mut det = a * d - b * c;
        det = R::one() / det;
        let mut out = Mat22::zero();
        out.col1.x = det * d;
        out.col2.x = -det * b;
        out.col1.y = -det * c;
        out.col2.y = det * a;
        out
    }
    /// b2Mat22.Solve(out, bx, by)
    #[inline(always)]
    pub fn solve(self, bx: R, by: R) -> Vec2<R> {
        let a11 = self.col1.x;
        let a12 = self.col2.x;
        let a21 = self.col1.y;
        let a22 = self.col2.y;
        let mut det = a11 * a22 - a12 * a21;
        det = R::one() / det;
        Vec2::new(det * (a22 * bx - a12 * by), det * (a11 * by - a21 * bx))
    }
    /// b2Mat22.GetAngle: Math.atan2(col1.y, col1.x)
    #[inline(always)]
    pub fn get_angle(self) -> R {
        crate::b2::math::atan2(self.col1.y, self.col1.x)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct XForm<R> {
    pub position: Vec2<R>,
    pub r: Mat22<R>,
}

impl<R: Real> XForm<R> {
    #[inline(always)]
    pub fn identity() -> Self {
        XForm { position: Vec2::zero(), r: Mat22::identity() }
    }
    /// `new b2XForm()`: position (0,0), R = new b2Mat22() = rotation of angle 0 via cos/sin(0).
    #[inline(always)]
    pub fn new_default() -> Self {
        XForm { position: Vec2::zero(), r: Mat22::from_angle(R::zero()) }
    }
    #[inline(always)]
    pub fn set(&mut self, o: XForm<R>) {
        self.position = o.position;
        self.r = o.r;
    }
    #[inline(always)]
    pub fn set_identity(&mut self) {
        self.position.set_zero();
        self.r.set_identity();
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Sweep<R> {
    pub local_center: Vec2<R>,
    pub c0: Vec2<R>,
    pub c: Vec2<R>,
    pub a0: R,
    pub a: R,
    pub t0: R,
}

impl<R: Real> Sweep<R> {
    /// `new b2Sweep()`: vectors zero, Numbers NaN (uninitialised `Number` fields are NaN in AS3).
    pub fn new_default() -> Self {
        let nan = R::from_words([0x7fffffff, 0xe0000000]); // Flash uninitialized Number payload.
        Sweep { local_center: Vec2::zero(), c0: Vec2::zero(), c: Vec2::zero(), a0: nan, a: nan, t0: nan }
    }
    /// b2Sweep.Advance(t)
    pub fn advance(&mut self, t: R) {
        if self.t0.lt(t) && (R::one() - self.t0).gt(min_value::<R>()) {
            let alpha = (t - self.t0) / (R::one() - self.t0);
            self.c0.x = (R::one() - alpha) * self.c0.x + alpha * self.c.x;
            self.c0.y = (R::one() - alpha) * self.c0.y + alpha * self.c.y;
            self.a0 = (R::one() - alpha) * self.a0 + alpha * self.a;
            self.t0 = t;
        }
    }
    /// b2Sweep.GetXForm(xf, t)
    pub fn get_xform(&self, xf: &mut XForm<R>, t: R) {
        if (R::one() - self.t0).gt(min_value::<R>()) {
            let alpha = (t - self.t0) / (R::one() - self.t0);
            xf.position.x = (R::one() - alpha) * self.c0.x + alpha * self.c.x;
            xf.position.y = (R::one() - alpha) * self.c0.y + alpha * self.c.y;
            let angle = (R::one() - alpha) * self.a0 + alpha * self.a;
            xf.r.set(angle);
        } else {
            xf.position.set_v(self.c);
            xf.r.set(self.a);
        }
        let r = xf.r;
        xf.position.x = xf.position.x - (r.col1.x * self.local_center.x + r.col2.x * self.local_center.y);
        xf.position.y = xf.position.y - (r.col1.y * self.local_center.x + r.col2.y * self.local_center.y);
    }
}

// ---- b2Math statics -------------------------------------------------------------------------

#[inline(always)]
pub fn b2_is_valid<R: Real>(x: R) -> bool {
    // isFinite
    let w = x.to_words()[0];
    (w & 0x7FF0_0000) != 0x7FF0_0000
}
#[inline(always)]
pub fn b2_dot<R: Real>(a: Vec2<R>, b: Vec2<R>) -> R {
    a.x * b.x + a.y * b.y
}
#[inline(always)]
pub fn b2_cross_vv<R: Real>(a: Vec2<R>, b: Vec2<R>) -> R {
    a.x * b.y - a.y * b.x
}
#[inline(always)]
pub fn b2_cross_vf<R: Real>(a: Vec2<R>, s: R) -> Vec2<R> {
    Vec2::new(s * a.y, -s * a.x)
}
#[inline(always)]
pub fn b2_cross_fv<R: Real>(s: R, a: Vec2<R>) -> Vec2<R> {
    Vec2::new(-s * a.y, s * a.x)
}
#[inline(always)]
pub fn b2_mul_mv<R: Real>(m: Mat22<R>, v: Vec2<R>) -> Vec2<R> {
    Vec2::new(m.col1.x * v.x + m.col2.x * v.y, m.col1.y * v.x + m.col2.y * v.y)
}
#[inline(always)]
pub fn b2_mul_tmv<R: Real>(m: Mat22<R>, v: Vec2<R>) -> Vec2<R> {
    Vec2::new(b2_dot(v, m.col1), b2_dot(v, m.col2))
}
/// b2MulX: R*v then += position (separate additions, as AS3).
#[inline(always)]
pub fn b2_mul_x<R: Real>(xf: XForm<R>, v: Vec2<R>) -> Vec2<R> {
    let mut u = b2_mul_mv(xf.r, v);
    u.x = u.x + xf.position.x;
    u.y = u.y + xf.position.y;
    u
}
/// b2MulXT
#[inline(always)]
pub fn b2_mul_xt<R: Real>(xf: XForm<R>, v: Vec2<R>) -> Vec2<R> {
    let mut u = subtract_vv(v, xf.position);
    let tx = u.x * xf.r.col1.x + u.y * xf.r.col1.y;
    u.y = u.x * xf.r.col2.x + u.y * xf.r.col2.y;
    u.x = tx;
    u
}
#[inline(always)]
pub fn add_vv<R: Real>(a: Vec2<R>, b: Vec2<R>) -> Vec2<R> {
    Vec2::new(a.x + b.x, a.y + b.y)
}
#[inline(always)]
pub fn subtract_vv<R: Real>(a: Vec2<R>, b: Vec2<R>) -> Vec2<R> {
    Vec2::new(a.x - b.x, a.y - b.y)
}
#[inline(always)]
pub fn mul_fv<R: Real>(s: R, a: Vec2<R>) -> Vec2<R> {
    Vec2::new(s * a.x, s * a.y)
}
#[inline(always)]
pub fn b2_mul_mm<R: Real>(a: Mat22<R>, b: Mat22<R>) -> Mat22<R> {
    Mat22::from_cols(b2_mul_mv(a, b.col1), b2_mul_mv(a, b.col2))
}
#[inline(always)]
pub fn b2_mul_tmm<R: Real>(a: Mat22<R>, b: Mat22<R>) -> Mat22<R> {
    let c1 = Vec2::new(b2_dot(a.col1, b.col1), b2_dot(a.col2, b.col1));
    let c2 = Vec2::new(b2_dot(a.col1, b.col2), b2_dot(a.col2, b.col2));
    Mat22::from_cols(c1, c2)
}
#[inline(always)]
pub fn b2_abs_v<R: Real>(a: Vec2<R>) -> Vec2<R> {
    Vec2::new(hw_real::as3::b2_abs(a.x), hw_real::as3::b2_abs(a.y))
}
#[inline(always)]
pub fn b2_abs_m<R: Real>(m: Mat22<R>) -> Mat22<R> {
    Mat22::from_cols(b2_abs_v(m.col1), b2_abs_v(m.col2))
}
#[inline(always)]
pub fn b2_min_v<R: Real>(a: Vec2<R>, b: Vec2<R>) -> Vec2<R> {
    Vec2::new(b2_min(a.x, b.x), b2_min(a.y, b.y))
}
#[inline(always)]
pub fn b2_max_v<R: Real>(a: Vec2<R>, b: Vec2<R>) -> Vec2<R> {
    Vec2::new(b2_max(a.x, b.x), b2_max(a.y, b.y))
}
#[inline(always)]
pub fn b2_clamp_v<R: Real>(a: Vec2<R>, lo: Vec2<R>, hi: Vec2<R>) -> Vec2<R> {
    b2_max_v(lo, b2_min_v(a, hi))
}
#[inline(always)]
pub fn b2_distance<R: Real>(a: Vec2<R>, b: Vec2<R>) -> R {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    (dx * dx + dy * dy).sqrt()
}
#[inline(always)]
pub fn b2_distance_squared<R: Real>(a: Vec2<R>, b: Vec2<R>) -> R {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    dx * dx + dy * dy
}

// ---- transcendental hooks (Flash 32 x86 semantics) --------------------------------------------

/// Math.cos / Math.sin of the same angle, returned as (cos, sin).
#[inline(always)]
pub fn sin_cos<R: Real>(angle: R) -> (R, R) {
    hw_flashmath::sin_cos(angle)
}
#[inline(always)]
pub fn atan2<R: Real>(y: R, x: R) -> R {
    hw_flashmath::atan2(y, x)
}
