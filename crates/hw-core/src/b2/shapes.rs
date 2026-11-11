//! b2Shape / b2CircleShape / b2PolygonShape / b2ShapeDef / b2CircleDef / b2PolygonDef / b2MassData
//! (Box2DFlashAS3 2.0.2, `Collision/Shapes/*.as`), ported statement by statement.
//!
//! A shape is a `Shape<R>` record in `ctx.world.shapes` addressed by an `H`; the circle and polygon
//! payloads live side by side in the record and `type_` selects the AS3 subclass. The static scratch
//! objects (`s_proxyAABB`, `s_syncAABB`, `s_resetAABB`, `s_computeMat`, `s_sweptAABB1/2`,
//! `s_supportVec`, `b2PolygonDef.s_mat`) are locals here: every use fully overwrites them before
//! reading and nothing keeps a reference to them.
//!
//! `b2Segment` / `TestSegment` (circle and polygon overrides) are never called by the game and are
//! not ported. b2Shape.Destroy is an empty static in AS3; here `destroy` releases the arena slot.
use hw_real::Real;

/// AS3 Number literal as an `R` (bit-exact double).

use super::broad_phase;
use super::math::{b2_mul_x, Mat22, Vec2, XForm};
use super::settings::{b2_pi, b2_toi_slop, max_value, B2_MAX_POLYGON_VERTICES};
use super::state::{MassData, Obb, Shape, ShapeDef, B2_NULL_PROXY, E_CIRCLE_SHAPE, E_POLYGON_SHAPE};
use crate::ctx::Ctx;
use crate::{Flow, H, NONE};

// ---- b2PolygonDef -----------------------------------------------------------------------------

impl<R: Real> ShapeDef<R> {
    /// b2PolygonDef.SetAsBox(hx, hy)
    pub fn set_as_box(&mut self, hx: R, hy: R) {
        self.vertex_count = 4;
        self.vertices[0].set(-hx, -hy);
        self.vertices[1].set(hx, -hy);
        self.vertices[2].set(hx, hy);
        self.vertices[3].set(-hx, hy);
    }

    /// b2PolygonDef.SetAsOrientedBox(hx, hy, center = null, angle = 0). `center == None` is the AS3
    /// null default (vertices left axis-aligned). The shared static `s_mat` is a local: `Set(angle)`
    /// overwrites all four entries before use.
    pub fn set_as_oriented_box(&mut self, hx: R, hy: R, center: Option<Vec2<R>>, angle: R) {
        self.vertex_count = 4;
        self.vertices[0].set(-hx, -hy);
        self.vertices[1].set(hx, -hy);
        self.vertices[2].set(hx, hy);
        self.vertices[3].set(-hx, hy);
        if let Some(xf_position) = center {
            // _loc6_ = s_mat; _loc6_.Set(param4)
            let mut xf_r = Mat22::identity();
            xf_r.set(angle);
            let mut i: i32 = 0;
            while i < self.vertex_count {
                let v = &mut self.vertices[i as usize];
                // param1 = cx + (col1.x * v.x + col2.x * v.y)   (reuses the hx parameter slot)
                let new_x = xf_position.x + (xf_r.col1.x * v.x + xf_r.col2.x * v.y);
                // v.y is written before v.x (AS3 order); the y line reads the still-old v.x.
                v.y = xf_position.y + (xf_r.col1.y * v.x + xf_r.col2.y * v.y);
                v.x = new_x;
                i += 1;
            }
        }
    }
}

/// Free-function form of `b2PolygonDef.SetAsBox` (used by fixtures/game code).
pub fn set_as_box<R: Real>(def: &mut ShapeDef<R>, hx: R, hy: R) {
    def.set_as_box(hx, hy);
}

/// Free-function form of `b2PolygonDef.SetAsOrientedBox` with a non-null center. For the AS3
/// `center = null` default call the method with `None`.
pub fn set_as_oriented_box<R: Real>(def: &mut ShapeDef<R>, hx: R, hy: R, center: Vec2<R>, angle: R) {
    def.set_as_oriented_box(hx, hy, Some(center), angle);
}

// ---- b2Shape.Create / constructors -----------------------------------------------------------

/// `b2Shape.Create(def, allocator)` followed by the `b2CircleShape` / `b2PolygonShape` constructor.
/// Returns `NONE` for an unknown def type (AS3 returns null) and also when the port arena is full.
pub fn create<R: Real>(ctx: &mut Ctx<R>, def: &ShapeDef<R>) -> H {
    let mut s = Shape::blank();
    // b2Shape(def)
    s.user_data = def.user_data;
    s.friction = def.friction;
    s.restitution = def.restitution;
    s.density = def.density;
    s.body = NONE;
    s.sweep_radius = R::zero();
    s.next = NONE;
    s.proxy_id = B2_NULL_PROXY;
    s.filter = def.filter; // param1.filter.Copy()
    s.is_sensor = def.is_sensor;
    // m_id (null), m_material (0), m_userData handlers: blank() defaults.
    match def.type_ {
        E_CIRCLE_SHAPE => {
            // b2CircleShape(def)
            s.type_ = E_CIRCLE_SHAPE;
            s.local_position.set_v(def.local_position);
            s.radius = def.radius;
        }
        E_POLYGON_SHAPE => {
            polygon_construct(&mut s, def);
        }
        _ => return NONE,
    }
    ctx.world.shapes.alloc(s)
}

/// `b2Shape.Destroy(shape, allocator)` is a no-op in AS3; the port frees the arena slot (contents stay
/// readable until the end of the frame, see `Arena`).
pub fn destroy<R: Real>(ctx: &mut Ctx<R>, shape: H) {
    ctx.world.shapes.destroy(shape);
}

/// Body of the `b2PolygonShape(def)` constructor (normals, centroid, OBB, core vertices).
fn polygon_construct<R: Real>(s: &mut Shape<R>, def: &ShapeDef<R>) {
    // this.s_supportVec = new b2Vec2(); this.m_obb = new b2OBB() -> R = new b2Mat22() (angle 0)
    s.obb = Obb { r: Mat22::from_angle(R::zero()), center: Vec2::zero(), extents: Vec2::zero() };
    s.type_ = E_POLYGON_SHAPE;
    s.vertex_count = def.vertex_count;
    let n = s.vertex_count;
    let mut i: i32 = 0;
    while i < n {
        s.vertices[i as usize] = def.vertices[i as usize]; // .Copy()
        i += 1;
    }
    i = 0;
    while i < n {
        let i1 = i;
        let i2 = if i + 1 < n { i + 1 } else { 0 };
        let edge_x = s.vertices[i2 as usize].x - s.vertices[i1 as usize].x;
        let edge_y = s.vertices[i2 as usize].y - s.vertices[i1 as usize].y;
        let len = (edge_x * edge_x + edge_y * edge_y).sqrt();
        // Flash's JIT folds -(x2-x1) to x1-x2 here. Unlike negating the
        // rounded difference, this yields +0 for equal coordinates. Confirmed
        // through typed and dynamic reads in number-storage-v1 (raw bits).
        let negative_edge_x = s.vertices[i1 as usize].x - s.vertices[i2 as usize].x;
        s.normals[i as usize] = Vec2::new(edge_y / len, negative_edge_x / len);
        i += 1;
    }
    s.centroid = compute_centroid(&def.vertices, def.vertex_count);
    compute_obb(&mut s.obb, &s.vertices, n);
    i = 0;
    while i < n {
        let i1 = if i - 1 >= 0 { i - 1 } else { n - 1 };
        let i2 = i;
        let n1x = s.normals[i1 as usize].x;
        let n1y = s.normals[i1 as usize].y;
        let n2x = s.normals[i2 as usize].x;
        let n2y = s.normals[i2 as usize].y;
        let vx = s.vertices[i as usize].x - s.centroid.x;
        let vy = s.vertices[i as usize].y - s.centroid.y;
        let d1 = n1x * vx + n1y * vy - b2_toi_slop::<R>();
        let d2 = n2x * vx + n2y * vy - b2_toi_slop::<R>();
        let det = R::one() / (n1x * n2y - n1y * n2x);
        s.core_vertices[i as usize] =
            Vec2::new(det * (n2y * d1 - n1y * d2) + s.centroid.x, det * (n1x * d2 - n2x * d1) + s.centroid.y);
        i += 1;
    }
}

/// b2PolygonShape.ComputeCentroid(vs, count)
pub fn compute_centroid<R: Real>(vs: &[Vec2<R>; B2_MAX_POLYGON_VERTICES], count: i32) -> Vec2<R> {
    let mut c = Vec2::zero();
    let mut area = R::zero();
    let p_ref_x = R::zero();
    let p_ref_y = R::zero();
    let inv3 = R::one() / R::from_i32(3);
    let mut i: i32 = 0;
    while i < count {
        let p1 = vs[i as usize];
        let p2 = if i + 1 < count { vs[(i + 1) as usize] } else { vs[0] };
        let e1x = p1.x - p_ref_x;
        let e1y = p1.y - p_ref_y;
        let e2x = p2.x - p_ref_x;
        let e2y = p2.y - p_ref_y;
        let d = e1x * e2y - e1y * e2x;
        let triangle_area = hw_real::rl!(R; 0.5) * d;
        area = area + triangle_area;
        c.x = c.x + triangle_area * inv3 * (p_ref_x + p1.x + p2.x);
        c.y = c.y + triangle_area * inv3 * (p_ref_y + p1.y + p2.y);
        i += 1;
    }
    c.x = c.x * (R::one() / area);
    c.y = c.y * (R::one() / area);
    c
}

/// b2PolygonShape.ComputeOBB(obb, vs, count)
pub fn compute_obb<R: Real>(obb: &mut Obb<R>, vs: &[Vec2<R>; B2_MAX_POLYGON_VERTICES], count: i32) {
    // p = new Array(b2_maxPolygonVertices + 1); p[i] = vs[i]; p[count] = p[0]
    let mut p = [Vec2::zero(); B2_MAX_POLYGON_VERTICES + 1];
    let mut i: i32 = 0;
    while i < count {
        p[i as usize] = vs[i as usize];
        i += 1;
    }
    p[count as usize] = p[0];
    let mut min_area = max_value::<R>();
    i = 1;
    while i <= count {
        let root = p[(i - 1) as usize];
        let mut ux = p[i as usize].x - root.x;
        let mut uy = p[i as usize].y - root.y;
        let length = (ux * ux + uy * uy).sqrt();
        ux = ux / length;
        uy = uy / length;
        let vx = -uy;
        let vy = ux;
        let mut lower_x = max_value::<R>();
        let mut lower_y = max_value::<R>();
        let mut upper_x = -max_value::<R>();
        let mut upper_y = -max_value::<R>();
        let mut j: i32 = 0;
        while j < count {
            let dx = p[j as usize].x - root.x;
            let dy = p[j as usize].y - root.y;
            let rx = ux * dx + uy * dy;
            let ry = vx * dx + vy * dy;
            if rx.lt(lower_x) {
                lower_x = rx;
            }
            if ry.lt(lower_y) {
                lower_y = ry;
            }
            if rx.gt(upper_x) {
                upper_x = rx;
            }
            if ry.gt(upper_y) {
                upper_y = ry;
            }
            j += 1;
        }
        let area = (upper_x - lower_x) * (upper_y - lower_y);
        if area.lt(hw_real::rl!(R; 0.95) * min_area) {
            min_area = area;
            obb.r.col1.x = ux;
            obb.r.col1.y = uy;
            obb.r.col2.x = vx;
            obb.r.col2.y = vy;
            let center_x = hw_real::rl!(R; 0.5) * (lower_x + upper_x);
            let center_y = hw_real::rl!(R; 0.5) * (lower_y + upper_y);
            let t_mat = obb.r;
            obb.center.x = root.x + (t_mat.col1.x * center_x + t_mat.col2.x * center_y);
            obb.center.y = root.y + (t_mat.col1.y * center_x + t_mat.col2.y * center_y);
            obb.extents.x = hw_real::rl!(R; 0.5) * (upper_x - lower_x);
            obb.extents.y = hw_real::rl!(R; 0.5) * (upper_y - lower_y);
        }
        i += 1;
    }
}

// ---- proxy management (b2Shape) --------------------------------------------------------------

/// b2Shape.CreateProxy(broadPhase, transform)
pub fn create_proxy<R: Real>(ctx: &mut Ctx<R>, shape: H, xf: &XForm<R>) -> Flow {
    // var aabb:b2AABB = s_proxyAABB; ComputeAABB(aabb, xf)
    let (lower, upper) = compute_aabb(ctx, shape, xf);
    let in_range = broad_phase::in_range(ctx, lower, upper);
    if in_range {
        let (f, id) = broad_phase::create_proxy(ctx, lower, upper, shape);
        crate::flow!(f);
        ctx.world.shape_mut(shape).proxy_id = id;
    } else {
        ctx.world.shape_mut(shape).proxy_id = B2_NULL_PROXY;
    }
    Flow::Continue
}

/// b2Shape.DestroyProxy(broadPhase)
pub fn destroy_proxy<R: Real>(ctx: &mut Ctx<R>, shape: H) -> Flow {
    let proxy_id = ctx.world.shape(shape).proxy_id;
    if proxy_id != B2_NULL_PROXY {
        crate::flow!(broad_phase::destroy_proxy(ctx, proxy_id));
        ctx.world.shape_mut(shape).proxy_id = B2_NULL_PROXY;
    }
    Flow::Continue
}

/// b2Shape.RefilterProxy(broadPhase, transform)
pub fn refilter_proxy<R: Real>(ctx: &mut Ctx<R>, shape: H, xf: &XForm<R>) -> Flow {
    let proxy_id = ctx.world.shape(shape).proxy_id;
    if proxy_id == B2_NULL_PROXY {
        return Flow::Continue;
    }
    crate::flow!(broad_phase::destroy_proxy(ctx, proxy_id));
    // var aabb:b2AABB = s_resetAABB; ComputeAABB(aabb, xf)
    let (lower, upper) = compute_aabb(ctx, shape, xf);
    let in_range = broad_phase::in_range(ctx, lower, upper);
    if in_range {
        // On a throw inside CreateProxy m_proxyId keeps the (now destroyed) old id.
        let (f, id) = broad_phase::create_proxy(ctx, lower, upper, shape);
        crate::flow!(f);
        ctx.world.shape_mut(shape).proxy_id = id;
    } else {
        ctx.world.shape_mut(shape).proxy_id = B2_NULL_PROXY;
    }
    Flow::Continue
}

/// `ResetProxy` does not exist in this AS3 build (only `s_resetAABB`, used by RefilterProxy);
/// provided under the documented API name as an alias of `refilter_proxy`.
pub fn reset_proxy<R: Real>(ctx: &mut Ctx<R>, shape: H, xf: &XForm<R>) -> Flow {
    refilter_proxy(ctx, shape, xf)
}

/// b2Shape.Synchronize(broadPhase, transform1, transform2) -> Boolean
pub fn synchronize<R: Real>(ctx: &mut Ctx<R>, shape: H, xf1: &XForm<R>, xf2: &XForm<R>) -> (Flow, bool) {
    let proxy_id = ctx.world.shape(shape).proxy_id;
    if proxy_id == B2_NULL_PROXY {
        return (Flow::Continue, false);
    }
    // var aabb:b2AABB = s_syncAABB; ComputeSweptAABB(aabb, xf1, xf2)
    let (lower, upper) = compute_swept_aabb(ctx, shape, xf1, xf2);
    if broad_phase::in_range(ctx, lower, upper) {
        let f = broad_phase::move_proxy(ctx, proxy_id, lower, upper);
        if f.is_abort() {
            return (f, false);
        }
        return (Flow::Continue, true);
    }
    (Flow::Continue, false)
}

// ---- geometry queries --------------------------------------------------------------------------

/// b2Shape.ComputeMass(massData)
pub fn compute_mass<R: Real>(ctx: &Ctx<R>, shape: H) -> MassData<R> {
    compute_mass_shape(ctx.world.shape(shape))
}

/// Value-level ComputeMass: `b2CircleShape.ComputeMass` / `b2PolygonShape.ComputeMass`.
pub fn compute_mass_shape<R: Real>(s: &Shape<R>) -> MassData<R> {
    let mut md = MassData::new();
    if s.type_ == E_CIRCLE_SHAPE {
        md.mass = s.density * b2_pi::<R>() * s.radius * s.radius;
        md.center.set_v(s.local_position);
        md.i = md.mass
            * (hw_real::rl!(R; 0.5) * s.radius * s.radius
                + (s.local_position.x * s.local_position.x + s.local_position.y * s.local_position.y));
        return md;
    }
    if s.type_ == E_POLYGON_SHAPE {
        let mut center_x = R::zero();
        let mut center_y = R::zero();
        let mut area = R::zero();
        let mut i_total = R::zero();
        let p_ref_x = R::zero();
        let p_ref_y = R::zero();
        let k_inv3 = R::one() / R::from_i32(3);
        let mut i: i32 = 0;
        while i < s.vertex_count {
            let p1 = s.vertices[i as usize];
            let p2 = if i + 1 < s.vertex_count { s.vertices[(i + 1) as usize] } else { s.vertices[0] };
            let e1x = p1.x - p_ref_x;
            let e1y = p1.y - p_ref_y;
            let e2x = p2.x - p_ref_x;
            let e2y = p2.y - p_ref_y;
            let d = e1x * e2y - e1y * e2x;
            let triangle_area = hw_real::rl!(R; 0.5) * d;
            area = area + triangle_area;
            center_x = center_x + triangle_area * k_inv3 * (p_ref_x + p1.x + p2.x);
            center_y = center_y + triangle_area * k_inv3 * (p_ref_y + p1.y + p2.y);
            let px = p_ref_x;
            let py = p_ref_y;
            let ex1 = e1x;
            let ey1 = e1y;
            let ex2 = e2x;
            let ey2 = e2y;
            let intx2 = k_inv3 * (hw_real::rl!(R; 0.25) * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px * ex2))
                + hw_real::rl!(R; 0.5) * px * px;
            let inty2 = k_inv3 * (hw_real::rl!(R; 0.25) * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py * ey2))
                + hw_real::rl!(R; 0.5) * py * py;
            i_total = i_total + d * (intx2 + inty2);
            i += 1;
        }
        md.mass = s.density * area;
        center_x = center_x * (R::one() / area);
        center_y = center_y * (R::one() / area);
        md.center.set(center_x, center_y);
        md.i = s.density * i_total;
        return md;
    }
    // b2Shape.ComputeMass base: no-op (massData left untouched).
    md
}

/// b2Shape.ComputeAABB(aabb, xf) -> (lowerBound, upperBound)
pub fn compute_aabb<R: Real>(ctx: &Ctx<R>, shape: H, xf: &XForm<R>) -> (Vec2<R>, Vec2<R>) {
    compute_aabb_shape(ctx.world.shape(shape), xf)
}

/// Value-level ComputeAABB for `b2CircleShape` / `b2PolygonShape`.
pub fn compute_aabb_shape<R: Real>(s: &Shape<R>, xf: &XForm<R>) -> (Vec2<R>, Vec2<R>) {
    if s.type_ == E_CIRCLE_SHAPE {
        let t_mat = xf.r;
        let px = xf.position.x + (t_mat.col1.x * s.local_position.x + t_mat.col2.x * s.local_position.y);
        let py = xf.position.y + (t_mat.col1.y * s.local_position.x + t_mat.col2.y * s.local_position.y);
        let lower = Vec2::new(px - s.radius, py - s.radius);
        let upper = Vec2::new(px + s.radius, py + s.radius);
        return (lower, upper);
    }
    if s.type_ == E_POLYGON_SHAPE {
        // var R:b2Mat22 = s_computeMat (fully overwritten below)
        let mut r = Mat22::identity();
        let mut t_mat = xf.r;
        let mut t_vec = s.obb.r.col1;
        r.col1.x = t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y;
        r.col1.y = t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y;
        t_vec = s.obb.r.col2;
        r.col2.x = t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y;
        r.col2.y = t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y;
        r.abs();
        let abs_r = r;
        t_vec = s.obb.extents;
        let hx = abs_r.col1.x * t_vec.x + abs_r.col2.x * t_vec.y;
        let hy = abs_r.col1.y * t_vec.x + abs_r.col2.y * t_vec.y;
        t_mat = xf.r;
        t_vec = s.obb.center;
        let px = xf.position.x + (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y);
        let py = xf.position.y + (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y);
        let lower = Vec2::new(px - hx, py - hy);
        let upper = Vec2::new(px + hx, py + hy);
        return (lower, upper);
    }
    // b2Shape.ComputeAABB base: no-op; the static scratch AABB keeps its previous contents. Unreachable
    // for the two shape types the game creates.
    (Vec2::zero(), Vec2::zero())
}

/// b2Shape.ComputeSweptAABB(aabb, xf1, xf2) -> (lowerBound, upperBound)
pub fn compute_swept_aabb<R: Real>(ctx: &Ctx<R>, shape: H, xf1: &XForm<R>, xf2: &XForm<R>) -> (Vec2<R>, Vec2<R>) {
    compute_swept_aabb_shape(ctx.world.shape(shape), xf1, xf2)
}

/// Value-level ComputeSweptAABB for `b2CircleShape` / `b2PolygonShape`.
pub fn compute_swept_aabb_shape<R: Real>(s: &Shape<R>, xf1: &XForm<R>, xf2: &XForm<R>) -> (Vec2<R>, Vec2<R>) {
    if s.type_ == E_CIRCLE_SHAPE {
        let mut t_mat = xf1.r;
        let p1x = xf1.position.x + (t_mat.col1.x * s.local_position.x + t_mat.col2.x * s.local_position.y);
        let p1y = xf1.position.y + (t_mat.col1.y * s.local_position.x + t_mat.col2.y * s.local_position.y);
        t_mat = xf2.r;
        let p2x = xf2.position.x + (t_mat.col1.x * s.local_position.x + t_mat.col2.x * s.local_position.y);
        let p2y = xf2.position.y + (t_mat.col1.y * s.local_position.x + t_mat.col2.y * s.local_position.y);
        let lower = Vec2::new(
            (if p1x.lt(p2x) { p1x } else { p2x }) - s.radius,
            (if p1y.lt(p2y) { p1y } else { p2y }) - s.radius,
        );
        let upper = Vec2::new(
            (if p1x.gt(p2x) { p1x } else { p2x }) + s.radius,
            (if p1y.gt(p2y) { p1y } else { p2y }) + s.radius,
        );
        return (lower, upper);
    }
    if s.type_ == E_POLYGON_SHAPE {
        // s_sweptAABB1 / s_sweptAABB2 -> locals
        let (l1, u1) = compute_aabb_shape(s, xf1);
        let (l2, u2) = compute_aabb_shape(s, xf2);
        let lower = Vec2::new(if l1.x.lt(l2.x) { l1.x } else { l2.x }, if l1.y.lt(l2.y) { l1.y } else { l2.y });
        let upper = Vec2::new(if u1.x.gt(u2.x) { u1.x } else { u2.x }, if u1.y.gt(u2.y) { u1.y } else { u2.y });
        return (lower, upper);
    }
    (Vec2::zero(), Vec2::zero())
}

/// b2Shape.UpdateSweepRadius(center)
pub fn update_sweep_radius<R: Real>(ctx: &mut Ctx<R>, shape: H, center: Vec2<R>) {
    let s = ctx.world.shape_mut(shape);
    if s.type_ == E_CIRCLE_SHAPE {
        let mut dx = s.local_position.x - center.x;
        let dy = s.local_position.y - center.y;
        dx = (dx * dx + dy * dy).sqrt();
        s.sweep_radius = dx + s.radius - b2_toi_slop::<R>();
        return;
    }
    if s.type_ == E_POLYGON_SHAPE {
        s.sweep_radius = R::zero();
        let mut i: i32 = 0;
        while i < s.vertex_count {
            let t_vec = s.core_vertices[i as usize];
            let mut dx = t_vec.x - center.x;
            let dy = t_vec.y - center.y;
            dx = (dx * dx + dy * dy).sqrt();
            if dx.gt(s.sweep_radius) {
                s.sweep_radius = dx;
            }
            i += 1;
        }
    }
    // b2Shape base: no-op
}

/// b2Shape.TestPoint(xf, p)
pub fn test_point<R: Real>(ctx: &Ctx<R>, shape: H, xf: &XForm<R>, p: Vec2<R>) -> bool {
    let s = ctx.world.shape(shape);
    if s.type_ == E_CIRCLE_SHAPE {
        let t_mat = xf.r;
        let mut dx = xf.position.x + (t_mat.col1.x * s.local_position.x + t_mat.col2.x * s.local_position.y);
        let mut dy = xf.position.y + (t_mat.col1.y * s.local_position.x + t_mat.col2.y * s.local_position.y);
        dx = p.x - dx;
        dy = p.y - dy;
        return (dx * dx + dy * dy).le(s.radius * s.radius);
    }
    if s.type_ == E_POLYGON_SHAPE {
        let t_mat = xf.r;
        let mut dx = p.x - xf.position.x;
        let mut dy = p.y - xf.position.y;
        let p_local_x = dx * t_mat.col1.x + dy * t_mat.col1.y;
        let p_local_y = dx * t_mat.col2.x + dy * t_mat.col2.y;
        let mut i: i32 = 0;
        while i < s.vertex_count {
            let mut t_vec = s.vertices[i as usize];
            dx = p_local_x - t_vec.x;
            dy = p_local_y - t_vec.y;
            t_vec = s.normals[i as usize];
            let dot = t_vec.x * dx + t_vec.y * dy;
            if dot.gt(R::zero()) {
                return false;
            }
            i += 1;
        }
        return true;
    }
    false
}

/// `GetMaxRadius` does not exist in this AS3 build; the documented API name maps to
/// `b2Shape.GetSweepRadius()` (m_sweepRadius), which is what b2TimeOfImpact reads.
pub fn get_max_radius<R: Real>(ctx: &Ctx<R>, shape: H) -> R {
    ctx.world.shape(shape).sweep_radius
}

/// b2Shape.GetSweepRadius()
pub fn get_sweep_radius<R: Real>(ctx: &Ctx<R>, shape: H) -> R {
    ctx.world.shape(shape).sweep_radius
}

/// b2PolygonShape.Support(xf, dx, dy). The per-instance `s_supportVec` result object is returned by
/// value (callers only read it).
pub fn support<R: Real>(s: &Shape<R>, xf: &XForm<R>, dx: R, dy: R) -> Vec2<R> {
    let mut t_mat = xf.r;
    let d_local_x = dx * t_mat.col1.x + dy * t_mat.col1.y;
    let d_local_y = dx * t_mat.col2.x + dy * t_mat.col2.y;
    let mut best_index: i32 = 0;
    let mut t_vec = s.core_vertices[0];
    let mut best_value = t_vec.x * d_local_x + t_vec.y * d_local_y;
    let mut i: i32 = 1;
    while i < s.vertex_count {
        t_vec = s.core_vertices[i as usize];
        let value = t_vec.x * d_local_x + t_vec.y * d_local_y;
        if value.gt(best_value) {
            best_index = i;
            best_value = value;
        }
        i += 1;
    }
    t_mat = xf.r;
    t_vec = s.core_vertices[best_index as usize];
    Vec2::new(
        xf.position.x + (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y),
        xf.position.y + (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y),
    )
}

/// b2PolygonShape.GetFirstVertex(xf) = b2MulX(xf, m_coreVertices[0])
pub fn get_first_vertex<R: Real>(s: &Shape<R>, xf: &XForm<R>) -> Vec2<R> {
    b2_mul_x(*xf, s.core_vertices[0])
}

/// b2PolygonShape.Centroid(xf) = b2MulX(xf, m_centroid)
pub fn centroid<R: Real>(s: &Shape<R>, xf: &XForm<R>) -> Vec2<R> {
    b2_mul_x(*xf, s.centroid)
}

// ---- trivial accessors (b2Shape) --------------------------------------------------------------

pub fn get_type<R: Real>(ctx: &Ctx<R>, shape: H) -> i32 {
    ctx.world.shape(shape).type_
}
pub fn get_body<R: Real>(ctx: &Ctx<R>, shape: H) -> H {
    ctx.world.shape(shape).body
}
pub fn get_next<R: Real>(ctx: &Ctx<R>, shape: H) -> H {
    ctx.world.shape(shape).next
}
pub fn is_sensor<R: Real>(ctx: &Ctx<R>, shape: H) -> bool {
    ctx.world.shape(shape).is_sensor.get()
}
pub fn get_friction<R: Real>(ctx: &Ctx<R>, shape: H) -> R {
    ctx.world.shape(shape).friction
}
pub fn get_restitution<R: Real>(ctx: &Ctx<R>, shape: H) -> R {
    ctx.world.shape(shape).restitution
}
pub fn get_user_data<R: Real>(ctx: &Ctx<R>, shape: H) -> u32 {
    ctx.world.shape(shape).user_data
}
pub fn set_user_data<R: Real>(ctx: &mut Ctx<R>, shape: H, user_data: u32) {
    ctx.world.shape_mut(shape).user_data = user_data;
}
/// SetId(String): the tag stands in for the string (0 = null).
pub fn set_id<R: Real>(ctx: &mut Ctx<R>, shape: H, id_tag: u32) {
    ctx.world.shape_mut(shape).id_tag = id_tag;
}
pub fn get_id<R: Real>(ctx: &Ctx<R>, shape: H) -> u32 {
    ctx.world.shape(shape).id_tag
}
pub fn set_material<R: Real>(ctx: &mut Ctx<R>, shape: H, material: i32) {
    ctx.world.shape_mut(shape).material = material;
}
pub fn get_material<R: Real>(ctx: &Ctx<R>, shape: H) -> i32 {
    ctx.world.shape(shape).material
}
/// SetFilterData(filter): stores a copy.
pub fn set_filter_data<R: Real>(ctx: &mut Ctx<R>, shape: H, filter: super::state::FilterData) {
    ctx.world.shape_mut(shape).filter = filter;
}
/// GetFilterData(): returns a copy.
pub fn get_filter_data<R: Real>(ctx: &Ctx<R>, shape: H) -> super::state::FilterData {
    ctx.world.shape(shape).filter
}
/// b2CircleShape.GetLocalPosition()
pub fn get_local_position<R: Real>(ctx: &Ctx<R>, shape: H) -> Vec2<R> {
    ctx.world.shape(shape).local_position
}
/// b2CircleShape.GetRadius()
pub fn get_radius<R: Real>(ctx: &Ctx<R>, shape: H) -> R {
    ctx.world.shape(shape).radius
}
/// b2PolygonShape.GetVertexCount()
pub fn get_vertex_count<R: Real>(ctx: &Ctx<R>, shape: H) -> i32 {
    ctx.world.shape(shape).vertex_count
}

#[cfg(test)]
mod tests {
    use crate::b2::geometry_fixture as golden;
    use super::*;
    use crate::b2::state::WorldState;

    /// `Ctx` is several MB (arena arrays); build it on a thread with a large stack.
    fn with_ctx(f: fn(&mut Ctx<f64>)) {
        std::thread::Builder::new()
            .stack_size(256 << 20)
            .spawn(move || {
                let mut c = Box::new(Ctx { world: WorldState::blank(), scratch: crate::b2::island::StepScratch::blank(), listener_mode: 0, level: crate::game::specials::LevelState::blank(), character: crate::game::actors::Actors::blank() });
                f(&mut c);
            })
            .unwrap()
            .join()
            .unwrap();
    }

    #[test]
    fn set_as_box_vertex_order() {
        let mut d = ShapeDef::<f64>::polygon();
        d.set_as_box(2.0, 1.0);
        assert_eq!(d.vertex_count, 4);
        assert_eq!(d.vertices[0], Vec2::new(-2.0, -1.0));
        assert_eq!(d.vertices[1], Vec2::new(2.0, -1.0));
        assert_eq!(d.vertices[2], Vec2::new(2.0, 1.0));
        assert_eq!(d.vertices[3], Vec2::new(-2.0, 1.0));
    }

    #[test]
    fn set_as_oriented_box_translates() {
        let mut d = ShapeDef::<f64>::polygon();
        d.set_as_oriented_box(2.0, 1.0, Some(Vec2::new(10.0, 20.0)), 0.0);
        assert_eq!(d.vertices[0], Vec2::new(8.0, 19.0));
        assert_eq!(d.vertices[2], Vec2::new(12.0, 21.0));
        // null center leaves the box axis-aligned at the origin
        let mut e = ShapeDef::<f64>::polygon();
        e.set_as_oriented_box(2.0, 1.0, None, 1.0);
        assert_eq!(e.vertices[1], Vec2::new(2.0, -1.0));
    }

    #[test]
    fn polygon_construct_box() {
        with_ctx(|c| {
            let mut d = ShapeDef::<f64>::polygon();
            d.density = 1.0;
            d.set_as_box(2.0, 1.0);
            let h = create(c, &d);
            assert_ne!(h, NONE);
            let s = *c.world.shape(h);
            assert_eq!(s.type_, E_POLYGON_SHAPE);
            // normals: edge 0 goes (-2,-1)->(2,-1): normal (0,-1)
            golden::vector("box.normals[0]", s.normals[0]);
            golden::vector("box.normals[1]", s.normals[1]);
            golden::vector("box.normals[2]", s.normals[2]);
            golden::vector("box.normals[3]", s.normals[3]);
            golden::vector("box.centroid", s.centroid);
            golden::vector("box.obb.extents", s.obb.extents);
            golden::vector("box.obb.center", s.obb.center);
            // core vertices shrunk by toiSlop = 0.04
            let cv = s.core_vertices[2];
            golden::vector("box.core2", cv);
            let md = compute_mass(c, h);
            golden::number("box.mass", md.mass);
            golden::number("box.inertia", md.i);
            golden::vector("box.mass_center", md.center);
            let xf = XForm::<f64>::identity();
            let (lo, hi) = compute_aabb(c, h, &xf);
            golden::vector("box.lower", lo);
            golden::vector("box.upper", hi);
            assert!(test_point(c, h, &xf, Vec2::new(0.5, 0.5)));
            assert!(!test_point(c, h, &xf, Vec2::new(2.5, 0.5)));
            update_sweep_radius(c, h, Vec2::zero());
            golden::number("box.sweep_radius", c.world.shape(h).sweep_radius);
        });
    }

    #[test]
    fn circle_mass_and_aabb() {
        with_ctx(|c| {
            let mut d = ShapeDef::<f64>::circle();
            d.density = 2.0;
            d.radius = 0.5;
            d.local_position = Vec2::new(1.0, 0.0);
            let h = create(c, &d);
            let md = compute_mass(c, h);
            golden::number("circle.mass", md.mass);
            golden::number("circle.inertia", md.i);
            golden::vector("circle.mass_center", md.center);
            let xf = XForm::<f64>::identity();
            let (lo, hi) = compute_aabb(c, h, &xf);
            golden::vector("circle.lower", lo);
            golden::vector("circle.upper", hi);
            assert!(test_point(c, h, &xf, Vec2::new(1.4, 0.0)));
            assert!(!test_point(c, h, &xf, Vec2::new(0.0, 0.0)));
        });
    }
}
