//! Dynamics/b2Body.as + b2BodyDef.as, ported statement by statement.
//!
//! `create` performs only the `new b2Body(def, world)` work; the world list linking is done by
//! `world::create_body`. The shared statics `s_massData` / `s_xf1` are locals here (nothing keeps a
//! reference to them).
use hw_real::Real;

use super::math::{Mat22, Vec2, XForm};
use super::shapes;
use super::state::*;
use crate::flow::{AbortSite, Flow};
use crate::{flow, Ctx, H, NONE};

/// `new b2Body(def, world)`. Returns `NONE` only when the port's body arena is exhausted.
pub fn create<R: Real>(ctx: &mut Ctx<R>, def: &BodyDef<R>) -> H {
    let mut b: Body<R> = Body::blank();
    b.flags = 0;
    if def.is_bullet.get() {
        b.flags |= E_BULLET_FLAG;
    }
    if def.fixed_rotation.get() {
        b.flags |= E_FIXED_ROTATION_FLAG;
    }
    if def.allow_sleep.get() {
        b.flags |= E_ALLOW_SLEEP_FLAG;
    }
    if def.is_sleeping.get() {
        b.flags |= E_SLEEP_FLAG;
    }
    // this.m_world = world (implicit: the single world in ctx)
    b.xf.position.set_v(def.position);
    b.xf.r.set(def.angle);
    b.sweep.local_center.set_v(def.mass_data.center);
    b.sweep.t0 = R::one();
    b.sweep.a = def.angle;
    b.sweep.a0 = b.sweep.a;
    let t_mat: Mat22<R> = b.xf.r;
    let t_vec: Vec2<R> = b.sweep.local_center;
    b.sweep.c.x = t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y;
    b.sweep.c.y = t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y;
    b.sweep.c.x = b.sweep.c.x + b.xf.position.x;
    b.sweep.c.y = b.sweep.c.y + b.xf.position.y;
    b.sweep.c0.set_v(b.sweep.c);
    b.joint_list = NONE;
    b.contact_list = NONE;
    b.prev = NONE;
    b.next = NONE;
    b.linear_damping = def.linear_damping;
    b.angular_damping = def.angular_damping;
    b.force.set(R::zero(), R::zero());
    b.torque = R::zero();
    b.linear_velocity.set_zero();
    b.angular_velocity = R::zero();
    b.sleep_time = R::zero();
    b.inv_mass = R::zero();
    b.i = R::zero();
    b.inv_i = R::zero();
    b.mass = def.mass_data.mass;
    if b.mass.gt(R::zero()) {
        b.inv_mass = R::one() / b.mass;
    }
    if (b.flags & E_FIXED_ROTATION_FLAG) == 0 {
        b.i = def.mass_data.i;
    }
    if b.i.gt(R::zero()) {
        b.inv_i = R::one() / b.i;
    }
    if b.inv_mass.eq_num(R::zero()) && b.inv_i.eq_num(R::zero()) {
        b.type_ = E_STATIC_TYPE;
    } else {
        b.type_ = E_DYNAMIC_TYPE;
    }
    b.user_data = def.user_data;
    b.shape_list = NONE;
    b.shape_count = 0;
    ctx.world.bodies.alloc(b)
}

/// b2Body.CreateShape. Returns `(Continue, NONE)` when the world is locked.
pub fn create_shape<R: Real>(ctx: &mut Ctx<R>, body: H, def: &ShapeDef<R>) -> (Flow, H) {
    if ctx.world.lock == crate::Flag32::TRUE {
        return (Flow::Continue, NONE);
    }
    let s = shapes::create(ctx, def);
    if s == NONE {
        return (Flow::Abort(AbortSite::PortCapacity), NONE);
    }
    let head = ctx.world.body(body).shape_list;
    ctx.world.shape_mut(s).next = head;
    ctx.world.body_mut(body).shape_list = s;
    ctx.world.body_mut(body).shape_count += 1;
    ctx.world.shape_mut(s).body = body;
    let xf: XForm<R> = ctx.world.body(body).xf;
    match shapes::create_proxy(ctx, s, &xf) {
        Flow::Continue => {}
        f => return (f, s),
    }
    let lc = ctx.world.body(body).sweep.local_center;
    shapes::update_sweep_radius(ctx, s, lc);
    (Flow::Continue, s)
}

/// b2Body.DestroyShape (silently returns when locked).
pub fn destroy_shape<R: Real>(ctx: &mut Ctx<R>, body: H, shape: H) -> Flow {
    if ctx.world.lock == crate::Flag32::TRUE {
        return Flow::Continue;
    }
    flow!(shapes::destroy_proxy(ctx, shape));
    let mut node: H = ctx.world.body(body).shape_list;
    let mut prev: H = NONE;
    let mut _found = false;
    while node != NONE {
        if node == shape {
            let after = ctx.world.shape(shape).next;
            if prev != NONE {
                ctx.world.shape_mut(prev).next = after;
            } else {
                ctx.world.body_mut(body).shape_list = after;
            }
            _found = true;
            break;
        }
        prev = node;
        node = ctx.world.shape(node).next;
    }
    ctx.world.shape_mut(shape).body = NONE;
    ctx.world.shape_mut(shape).next = NONE;
    ctx.world.body_mut(body).shape_count -= 1;
    shapes::destroy(ctx, shape);
    Flow::Continue
}

/// b2Body.SetMass (silently returns when locked).
pub fn set_mass<R: Real>(ctx: &mut Ctx<R>, body: H, mass_data: &MassData<R>) -> Flow {
    if ctx.world.lock == crate::Flag32::TRUE {
        return Flow::Continue;
    }
    {
        let b = ctx.world.body_mut(body);
        b.inv_mass = R::zero();
        b.i = R::zero();
        b.inv_i = R::zero();
        b.mass = mass_data.mass;
        if b.mass.gt(R::zero()) {
            b.inv_mass = R::one() / b.mass;
        }
        if (b.flags & E_FIXED_ROTATION_FLAG) == 0 {
            b.i = mass_data.i;
        }
        if b.i.gt(R::zero()) {
            b.inv_i = R::one() / b.i;
        }
        b.sweep.local_center.set_v(mass_data.center);
        let t_mat: Mat22<R> = b.xf.r;
        let t_vec: Vec2<R> = b.sweep.local_center;
        b.sweep.c.x = t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y;
        b.sweep.c.y = t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y;
        b.sweep.c.x = b.sweep.c.x + b.xf.position.x;
        b.sweep.c.y = b.sweep.c.y + b.xf.position.y;
        b.sweep.c0.set_v(b.sweep.c);
    }
    let mut s = ctx.world.body(body).shape_list;
    while s != NONE {
        let lc = ctx.world.body(body).sweep.local_center;
        shapes::update_sweep_radius(ctx, s, lc);
        s = ctx.world.shape(s).next;
    }
    let old_type: i32 = ctx.world.body(body).type_;
    {
        let b = ctx.world.body_mut(body);
        if b.inv_mass.eq_num(R::zero()) && b.inv_i.eq_num(R::zero()) {
            b.type_ = E_STATIC_TYPE;
        } else {
            b.type_ = E_DYNAMIC_TYPE;
        }
    }
    if old_type != ctx.world.body(body).type_ {
        let mut s = ctx.world.body(body).shape_list;
        while s != NONE {
            let xf: XForm<R> = ctx.world.body(body).xf;
            flow!(shapes::refilter_proxy(ctx, s, &xf));
            s = ctx.world.shape(s).next;
        }
    }
    Flow::Continue
}

/// b2Body.SetMassFromShapes (silently returns when locked). `s_massData` is a local.
pub fn set_mass_from_shapes<R: Real>(ctx: &mut Ctx<R>, body: H) -> Flow {
    if ctx.world.lock == crate::Flag32::TRUE {
        return Flow::Continue;
    }
    {
        let b = ctx.world.body_mut(body);
        b.mass = R::zero();
        b.inv_mass = R::zero();
        b.i = R::zero();
        b.inv_i = R::zero();
    }
    let mut center_x: R = R::zero();
    let mut center_y: R = R::zero();
    let mut s = ctx.world.body(body).shape_list;
    while s != NONE {
        let md: MassData<R> = shapes::compute_mass(ctx, s);
        let b = ctx.world.body_mut(body);
        b.mass = b.mass + md.mass;
        center_x = center_x + md.mass * md.center.x;
        center_y = center_y + md.mass * md.center.y;
        b.i = b.i + md.i;
        s = ctx.world.shape(s).next;
    }
    {
        let b = ctx.world.body_mut(body);
        if b.mass.gt(R::zero()) {
            b.inv_mass = R::one() / b.mass;
            center_x = center_x * b.inv_mass;
            center_y = center_y * b.inv_mass;
        }
        if b.i.gt(R::zero()) && (b.flags & E_FIXED_ROTATION_FLAG) == 0 {
            b.i = b.i - b.mass * (center_x * center_x + center_y * center_y);
            b.inv_i = R::one() / b.i;
        } else {
            b.i = R::zero();
            b.inv_i = R::zero();
        }
        b.sweep.local_center.set(center_x, center_y);
        let t_mat: Mat22<R> = b.xf.r;
        let t_vec: Vec2<R> = b.sweep.local_center;
        b.sweep.c.x = t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y;
        b.sweep.c.y = t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y;
        b.sweep.c.x = b.sweep.c.x + b.xf.position.x;
        b.sweep.c.y = b.sweep.c.y + b.xf.position.y;
        b.sweep.c0.set_v(b.sweep.c);
    }
    let mut s = ctx.world.body(body).shape_list;
    while s != NONE {
        let lc = ctx.world.body(body).sweep.local_center;
        shapes::update_sweep_radius(ctx, s, lc);
        s = ctx.world.shape(s).next;
    }
    let old_type: i32 = ctx.world.body(body).type_;
    {
        let b = ctx.world.body_mut(body);
        if b.inv_mass.eq_num(R::zero()) && b.inv_i.eq_num(R::zero()) {
            b.type_ = E_STATIC_TYPE;
        } else {
            b.type_ = E_DYNAMIC_TYPE;
        }
    }
    if old_type != ctx.world.body(body).type_ {
        let mut s = ctx.world.body(body).shape_list;
        while s != NONE {
            let xf: XForm<R> = ctx.world.body(body).xf;
            flow!(shapes::refilter_proxy(ctx, s, &xf));
            s = ctx.world.shape(s).next;
        }
    }
    Flow::Continue
}

/// b2Body.SetXForm. Returns `true` (without doing anything) when the world is locked.
pub fn set_xform<R: Real>(ctx: &mut Ctx<R>, body: H, position: Vec2<R>, angle: R) -> (Flow, bool) {
    if ctx.world.lock == crate::Flag32::TRUE {
        return (Flow::Continue, true);
    }
    if is_frozen(ctx, body) {
        return (Flow::Continue, false);
    }
    {
        let b = ctx.world.body_mut(body);
        b.xf.r.set(angle);
        b.xf.position.set_v(position);
        let t_mat: Mat22<R> = b.xf.r;
        let t_vec: Vec2<R> = b.sweep.local_center;
        b.sweep.c.x = t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y;
        b.sweep.c.y = t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y;
        b.sweep.c.x = b.sweep.c.x + b.xf.position.x;
        b.sweep.c.y = b.sweep.c.y + b.xf.position.y;
        b.sweep.c0.set_v(b.sweep.c);
        b.sweep.a = angle;
        b.sweep.a0 = b.sweep.a;
    }
    let mut freeze = false;
    let mut s = ctx.world.body(body).shape_list;
    while s != NONE {
        let xf: XForm<R> = ctx.world.body(body).xf;
        let (f, in_range) = shapes::synchronize(ctx, s, &xf, &xf);
        if f.is_abort() {
            return (f, false);
        }
        if in_range == false {
            freeze = true;
            break;
        }
        s = ctx.world.shape(s).next;
    }
    if freeze == true {
        {
            let b = ctx.world.body_mut(body);
            b.flags |= E_FROZEN_FLAG;
            b.linear_velocity.set_zero();
            b.angular_velocity = R::zero();
        }
        let mut s = ctx.world.body(body).shape_list;
        while s != NONE {
            match shapes::destroy_proxy(ctx, s) {
                Flow::Continue => {}
                f => return (f, false),
            }
            s = ctx.world.shape(s).next;
        }
        return (Flow::Continue, false);
    }
    match super::broad_phase::commit(ctx) {
        Flow::Continue => {}
        f => return (f, false),
    }
    (Flow::Continue, true)
}

/// b2Body.SynchronizeShapes. `s_xf1` is a local.
pub fn synchronize_shapes<R: Real>(ctx: &mut Ctx<R>, body: H) -> (Flow, bool) {
    let mut xf1: XForm<R> = XForm::new_default();
    {
        let b = ctx.world.body(body);
        xf1.r.set(b.sweep.a0);
        let t_mat: Mat22<R> = xf1.r;
        let t_vec: Vec2<R> = b.sweep.local_center;
        xf1.position.x = b.sweep.c0.x - (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y);
        xf1.position.y = b.sweep.c0.y - (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y);
    }
    let mut in_range = true;
    let mut s = ctx.world.body(body).shape_list;
    while s != NONE {
        let xf: XForm<R> = ctx.world.body(body).xf;
        let (f, r) = shapes::synchronize(ctx, s, &xf1, &xf);
        if f.is_abort() {
            return (f, false);
        }
        in_range = r;
        if in_range == false {
            break;
        }
        s = ctx.world.shape(s).next;
    }
    if in_range == false {
        lab! {
            if crate::lab::watched(body) {
                crate::lab::record(crate::lab::Event::Freeze { body });
            }
        }
        {
            let b = ctx.world.body_mut(body);
            b.flags |= E_FROZEN_FLAG;
            b.linear_velocity.set_zero();
            b.angular_velocity = R::zero();
        }
        let mut s = ctx.world.body(body).shape_list;
        while s != NONE {
            match shapes::destroy_proxy(ctx, s) {
                Flow::Continue => {}
                f => return (f, false),
            }
            s = ctx.world.shape(s).next;
        }
        return (Flow::Continue, false);
    }
    (Flow::Continue, true)
}

/// b2Body.SynchronizeTransform
#[inline]
pub fn synchronize_transform<R: Real>(ctx: &mut Ctx<R>, body: H) {
    let b = ctx.world.body_mut(body);
    b.xf.r.set(b.sweep.a);
    let t_mat: Mat22<R> = b.xf.r;
    let t_vec: Vec2<R> = b.sweep.local_center;
    b.xf.position.x = b.sweep.c.x - (t_mat.col1.x * t_vec.x + t_mat.col2.x * t_vec.y);
    b.xf.position.y = b.sweep.c.y - (t_mat.col1.y * t_vec.x + t_mat.col2.y * t_vec.y);
}

/// b2Body.Advance
pub fn advance<R: Real>(ctx: &mut Ctx<R>, body: H, t: R) {
    {
        let b = ctx.world.body_mut(body);
        b.sweep.advance(t);
        let c0 = b.sweep.c0;
        b.sweep.c.set_v(c0);
        b.sweep.a = b.sweep.a0;
    }
    synchronize_transform(ctx, body);
}

/// b2Body.WakeUp
#[inline]
pub fn wake_up<R: Real>(ctx: &mut Ctx<R>, body: H) {
    let b = ctx.world.body_mut(body);
    b.flags &= !E_SLEEP_FLAG;
    b.sleep_time = R::zero();
}

/// b2Body.PutToSleep
pub fn put_to_sleep<R: Real>(ctx: &mut Ctx<R>, body: H) {
    let b = ctx.world.body_mut(body);
    b.flags |= E_SLEEP_FLAG;
    b.sleep_time = R::zero();
    b.linear_velocity.set_zero();
    b.angular_velocity = R::zero();
    b.force.set_zero();
    b.torque = R::zero();
}

/// b2Body.ApplyForce(force, point)
pub fn apply_force<R: Real>(ctx: &mut Ctx<R>, body: H, force: Vec2<R>, point: Vec2<R>) {
    if is_sleeping(ctx, body) {
        wake_up(ctx, body);
    }
    let b = ctx.world.body_mut(body);
    b.force.x = b.force.x + force.x;
    b.force.y = b.force.y + force.y;
    b.torque = b.torque + ((point.x - b.sweep.c.x) * force.y - (point.y - b.sweep.c.y) * force.x);
}

/// b2Body.ApplyTorque
pub fn apply_torque<R: Real>(ctx: &mut Ctx<R>, body: H, torque: R) {
    if is_sleeping(ctx, body) {
        wake_up(ctx, body);
    }
    let b = ctx.world.body_mut(body);
    b.torque = b.torque + torque;
}

/// b2Body.ApplyImpulse(impulse, point)
pub fn apply_impulse<R: Real>(ctx: &mut Ctx<R>, body: H, impulse: Vec2<R>, point: Vec2<R>) {
    if is_sleeping(ctx, body) {
        wake_up(ctx, body);
    }
    let b = ctx.world.body_mut(body);
    b.linear_velocity.x = b.linear_velocity.x + b.inv_mass * impulse.x;
    b.linear_velocity.y = b.linear_velocity.y + b.inv_mass * impulse.y;
    b.angular_velocity =
        b.angular_velocity + b.inv_i * ((point.x - b.sweep.c.x) * impulse.y - (point.y - b.sweep.c.y) * impulse.x);
}

/// b2Body.SetLinearVelocity
#[inline]
pub fn set_linear_velocity<R: Real>(ctx: &mut Ctx<R>, body: H, v: Vec2<R>) {
    ctx.world.body_mut(body).linear_velocity.set_v(v);
}

/// b2Body.SetAngularVelocity
#[inline]
pub fn set_angular_velocity<R: Real>(ctx: &mut Ctx<R>, body: H, w: R) {
    ctx.world.body_mut(body).angular_velocity = w;
}

/// b2Body.GetLinearVelocity (by value)
#[inline]
pub fn get_linear_velocity<R: Real>(ctx: &Ctx<R>, body: H) -> Vec2<R> {
    ctx.world.body(body).linear_velocity
}

/// b2Body.GetAngularVelocity
#[inline]
pub fn get_angular_velocity<R: Real>(ctx: &Ctx<R>, body: H) -> R {
    ctx.world.body(body).angular_velocity
}

/// b2Body.GetWorldPoint(localPoint)
#[inline]
pub fn get_world_point<R: Real>(ctx: &Ctx<R>, body: H, local_point: Vec2<R>) -> Vec2<R> {
    let b = ctx.world.body(body);
    let a: Mat22<R> = b.xf.r;
    let mut u = Vec2::new(
        a.col1.x * local_point.x + a.col2.x * local_point.y,
        a.col1.y * local_point.x + a.col2.y * local_point.y,
    );
    u.x = u.x + b.xf.position.x;
    u.y = u.y + b.xf.position.y;
    u
}

/// b2Body.GetWorldVector(localVector) = b2MulMV(xf.R, v)
#[inline]
pub fn get_world_vector<R: Real>(ctx: &Ctx<R>, body: H, local_vector: Vec2<R>) -> Vec2<R> {
    super::math::b2_mul_mv(ctx.world.body(body).xf.r, local_vector)
}

/// b2Body.GetLocalPoint(worldPoint) = b2MulXT(xf, p)
#[inline]
pub fn get_local_point<R: Real>(ctx: &Ctx<R>, body: H, world_point: Vec2<R>) -> Vec2<R> {
    super::math::b2_mul_xt(ctx.world.body(body).xf, world_point)
}

/// b2Body.GetLocalVector(worldVector) = b2MulTMV(xf.R, v)
#[inline]
pub fn get_local_vector<R: Real>(ctx: &Ctx<R>, body: H, world_vector: Vec2<R>) -> Vec2<R> {
    super::math::b2_mul_tmv(ctx.world.body(body).xf.r, world_vector)
}

/// b2Body.GetLinearVelocityFromWorldPoint
#[inline]
pub fn get_linear_velocity_from_world_point<R: Real>(ctx: &Ctx<R>, body: H, world_point: Vec2<R>) -> Vec2<R> {
    let b = ctx.world.body(body);
    Vec2::new(
        b.linear_velocity.x + b.angular_velocity * (world_point.y - b.sweep.c.y),
        b.linear_velocity.y - b.angular_velocity * (world_point.x - b.sweep.c.x),
    )
}

/// b2Body.GetLinearVelocityFromLocalPoint
#[inline]
pub fn get_linear_velocity_from_local_point<R: Real>(ctx: &Ctx<R>, body: H, local_point: Vec2<R>) -> Vec2<R> {
    let b = ctx.world.body(body);
    let a: Mat22<R> = b.xf.r;
    let mut p = Vec2::new(
        a.col1.x * local_point.x + a.col2.x * local_point.y,
        a.col1.y * local_point.x + a.col2.y * local_point.y,
    );
    p.x = p.x + b.xf.position.x;
    p.y = p.y + b.xf.position.y;
    Vec2::new(
        b.linear_velocity.x + b.angular_velocity * (p.y - b.sweep.c.y),
        b.linear_velocity.y - b.angular_velocity * (p.x - b.sweep.c.x),
    )
}

#[inline(always)]
pub fn is_static<R: Real>(ctx: &Ctx<R>, body: H) -> bool {
    ctx.world.body(body).type_ == E_STATIC_TYPE
}
#[inline(always)]
pub fn is_dynamic<R: Real>(ctx: &Ctx<R>, body: H) -> bool {
    ctx.world.body(body).type_ == E_DYNAMIC_TYPE
}
#[inline(always)]
pub fn is_frozen<R: Real>(ctx: &Ctx<R>, body: H) -> bool {
    (ctx.world.body(body).flags & E_FROZEN_FLAG) == E_FROZEN_FLAG
}
#[inline(always)]
pub fn is_sleeping<R: Real>(ctx: &Ctx<R>, body: H) -> bool {
    (ctx.world.body(body).flags & E_SLEEP_FLAG) == E_SLEEP_FLAG
}
#[inline(always)]
pub fn is_bullet<R: Real>(ctx: &Ctx<R>, body: H) -> bool {
    (ctx.world.body(body).flags & E_BULLET_FLAG) == E_BULLET_FLAG
}

/// b2Body.IsConnected(other): walks this body's joint edges.
pub fn is_connected<R: Real>(ctx: &Ctx<R>, body: H, other: H) -> bool {
    let mut jn: H = ctx.world.body(body).joint_list;
    while jn != NONE {
        let e = *ctx.world.joint_edge(jn);
        if e.other == other {
            return ctx.world.joint(edge_owner(jn)).collide_connected == crate::Flag32::FALSE;
        }
        jn = e.next;
    }
    false
}

/// b2Body.SetBullet
pub fn set_bullet<R: Real>(ctx: &mut Ctx<R>, body: H, flag: bool) {
    let b = ctx.world.body_mut(body);
    if flag {
        b.flags |= E_BULLET_FLAG;
    } else {
        b.flags &= !E_BULLET_FLAG;
    }
}

/// b2Body.AllowSleeping
pub fn allow_sleeping<R: Real>(ctx: &mut Ctx<R>, body: H, flag: bool) {
    if flag {
        ctx.world.body_mut(body).flags |= E_ALLOW_SLEEP_FLAG;
    } else {
        ctx.world.body_mut(body).flags &= !E_ALLOW_SLEEP_FLAG;
        wake_up(ctx, body);
    }
}

#[inline(always)]
pub fn get_position<R: Real>(ctx: &Ctx<R>, body: H) -> Vec2<R> {
    ctx.world.body(body).xf.position
}
#[inline(always)]
pub fn get_angle<R: Real>(ctx: &Ctx<R>, body: H) -> R {
    ctx.world.body(body).sweep.a
}
#[inline(always)]
pub fn get_xform<R: Real>(ctx: &Ctx<R>, body: H) -> XForm<R> {
    ctx.world.body(body).xf
}
#[inline(always)]
pub fn get_world_center<R: Real>(ctx: &Ctx<R>, body: H) -> Vec2<R> {
    ctx.world.body(body).sweep.c
}
#[inline(always)]
pub fn get_local_center<R: Real>(ctx: &Ctx<R>, body: H) -> Vec2<R> {
    ctx.world.body(body).sweep.local_center
}
#[inline(always)]
pub fn get_mass<R: Real>(ctx: &Ctx<R>, body: H) -> R {
    ctx.world.body(body).mass
}
#[inline(always)]
pub fn get_inertia<R: Real>(ctx: &Ctx<R>, body: H) -> R {
    ctx.world.body(body).i
}
#[inline(always)]
pub fn get_shape_list<R: Real>(ctx: &Ctx<R>, body: H) -> H {
    ctx.world.body(body).shape_list
}
#[inline(always)]
pub fn get_joint_list<R: Real>(ctx: &Ctx<R>, body: H) -> H {
    ctx.world.body(body).joint_list
}
#[inline(always)]
pub fn get_next<R: Real>(ctx: &Ctx<R>, body: H) -> H {
    ctx.world.body(body).next
}
#[inline(always)]
pub fn get_user_data<R: Real>(ctx: &Ctx<R>, body: H) -> u32 {
    ctx.world.body(body).user_data
}
#[inline(always)]
pub fn set_user_data<R: Real>(ctx: &mut Ctx<R>, body: H, data: u32) {
    ctx.world.body_mut(body).user_data = data;
}
