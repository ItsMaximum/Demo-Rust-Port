//! Dynamics/b2Island.as + Contacts/b2ContactSolver.as + b2ContactConstraint(Point).as.
//!
//! The AS3 allocates a fresh `b2Island` (and a fresh `b2ContactSolver` with fresh constraints) per
//! `b2World.Solve` / `SolveTOI`; here they live in `ctx.scratch` (`StepScratch`) and are rebuilt
//! in place. `Solve` and `SolveTOI` never nest, so one island record serves both (the TOI island's
//! 32-contact cap is `Island::contact_capacity`, set by the world before use).
//! `b2Island.s_reportCR.normal` is `ctx.world.report_cr_normal` (aliased by game code).
use hw_real::as3::{b2_clamp, b2_max, b2_min};
use hw_real::{rl, Real};

use super::body;
use super::joints;
use super::math::{b2_dot, Mat22, Vec2};
use super::settings;
use super::state::*;
use crate::flow::Flow;
use crate::{flow, Ctx, H, NONE};

// ---- b2ContactConstraintPoint / b2ContactConstraint ---------------------------------------------

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct ContactConstraintPoint<R> {
    pub local_anchor1: Vec2<R>,
    pub local_anchor2: Vec2<R>,
    pub r1: Vec2<R>,
    pub r2: Vec2<R>,
    pub normal_impulse: R,
    pub tangent_impulse: R,
    pub position_impulse: R,
    pub normal_mass: R,
    pub tangent_mass: R,
    pub equalized_mass: R,
    pub separation: R,
    pub velocity_bias: R,
}

impl<R: Real> ContactConstraintPoint<R> {
    /// `new b2ContactConstraintPoint()`: vectors zero, Numbers NaN.
    pub fn blank() -> Self {
        let nan = R::from_words([0x7FF80000, 0]);
        ContactConstraintPoint {
            local_anchor1: Vec2::zero(),
            local_anchor2: Vec2::zero(),
            r1: Vec2::zero(),
            r2: Vec2::zero(),
            normal_impulse: nan,
            tangent_impulse: nan,
            position_impulse: nan,
            normal_mass: nan,
            tangent_mass: nan,
            equalized_mass: nan,
            separation: nan,
            velocity_bias: nan,
        }
    }
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct ContactConstraint<R> {
    pub points: [ContactConstraintPoint<R>; settings::B2_MAX_MANIFOLD_POINTS],
    pub normal: Vec2<R>,
    /// The contact owning `manifold` (the AS3 keeps a reference to the b2Manifold itself).
    pub contact: H,
    pub body1: H,
    pub body2: H,
    pub friction: R,
    pub restitution: R,
    pub point_count: i32,
}

impl<R: Real> ContactConstraint<R> {
    /// `new b2ContactConstraint()`
    pub fn blank() -> Self {
        let nan = R::from_words([0x7FF80000, 0]);
        ContactConstraint {
            points: [ContactConstraintPoint::blank(); settings::B2_MAX_MANIFOLD_POINTS],
            normal: Vec2::zero(),
            contact: NONE,
            body1: NONE,
            body2: NONE,
            friction: nan,
            restitution: nan,
            point_count: 0,
        }
    }
}

// ---- b2Island ---------------------------------------------------------------------------------

/// b2Island: the per-call arrays (AS3 Arrays grow silently; each body/contact/joint enters an
/// island at most once per call, so the port capacities are true upper bounds).
#[derive(Clone, Copy)]
#[repr(C)]
pub struct Island {
    pub bodies: [H; MAX_BODIES],
    pub contacts: [H; MAX_CONTACTS],
    pub joints: [H; MAX_JOINTS],
    pub body_count: i32,
    pub contact_count: i32,
    pub joint_count: i32,
    pub body_capacity: i32,
    /// m_contactCapacity: only ever compared in b2World.SolveTOI (`!=` against 32).
    pub contact_capacity: i32,
    pub joint_capacity: i32,
    pub position_iteration_count: i32,
}

impl Island {
    pub fn blank() -> Self {
        Island {
            bodies: [NONE; MAX_BODIES],
            contacts: [NONE; MAX_CONTACTS],
            joints: [NONE; MAX_JOINTS],
            body_count: 0,
            contact_count: 0,
            joint_count: 0,
            body_capacity: 0,
            contact_capacity: 0,
            joint_capacity: 0,
            position_iteration_count: 0,
        }
    }
    /// `new b2Island(bodyCapacity, contactCapacity, jointCapacity, ...)`
    #[inline]
    pub fn init(&mut self, body_capacity: i32, contact_capacity: i32, joint_capacity: i32) {
        self.body_capacity = body_capacity;
        self.contact_capacity = contact_capacity;
        self.joint_capacity = joint_capacity;
        self.body_count = 0;
        self.contact_count = 0;
        self.joint_count = 0;
        self.position_iteration_count = 0;
    }
    /// b2Island.Clear
    #[inline(always)]
    pub fn clear(&mut self) {
        self.body_count = 0;
        self.contact_count = 0;
        self.joint_count = 0;
    }
    #[inline(always)]
    pub fn add_body(&mut self, b: H) {
        let i = self.body_count;
        self.body_count += 1;
        self.bodies[i as usize] = b;
    }
    #[inline(always)]
    pub fn add_contact(&mut self, c: H) {
        let i = self.contact_count;
        self.contact_count += 1;
        self.contacts[i as usize] = c;
    }
    #[inline(always)]
    pub fn add_joint(&mut self, j: H) {
        let i = self.joint_count;
        self.joint_count += 1;
        self.joints[i as usize] = j;
    }
}

/// Per-step scratch of b2World.Solve / SolveTOI, stored in `Ctx::scratch`.
#[derive(Clone, Copy)]
#[repr(C)]
pub struct StepScratch<R> {
    pub island: Island,
    /// The explicit DFS stack of b2World.Solve / SolveTOI (`new Array(m_bodyCount)`).
    pub stack: [H; MAX_BODIES],
    /// b2ContactSolver.m_constraints (one per manifold; manifoldCount <= 1 per contact).
    pub constraints: [ContactConstraint<R>; MAX_CONTACTS],
    pub constraint_count: i32,
}

impl<R: Real> StepScratch<R> {
    pub fn blank() -> Self {
        StepScratch {
            island: Island::blank(),
            stack: [NONE; MAX_BODIES],
            constraints: [ContactConstraint::blank(); MAX_CONTACTS],
            constraint_count: 0,
        }
    }
}

// ---- b2Island.Solve / SolveTOI / Report -------------------------------------------------------

/// b2Island.Solve(step, gravity, correctPositions, allowSleep)
pub fn solve<R: Real>(
    ctx: &mut Ctx<R>,
    step: &TimeStep<R>,
    gravity: Vec2<R>,
    correct_positions: bool,
    allow_sleep: bool,
) -> Flow {
    let body_count = ctx.scratch.island.body_count;
    let joint_count = ctx.scratch.island.joint_count;
    // Integrate velocities and apply damping.
    let mut i: i32 = 0;
    while i < body_count {
        let bh = ctx.scratch.island.bodies[i as usize];
        if !body::is_static(ctx, bh) {
            let b = ctx.world.body_mut(bh);
            b.linear_velocity.x = b.linear_velocity.x + step.dt * (gravity.x + b.inv_mass * b.force.x);
            b.linear_velocity.y = b.linear_velocity.y + step.dt * (gravity.y + b.inv_mass * b.force.y);
            b.angular_velocity = b.angular_velocity + step.dt * b.inv_i * b.torque;
            b.force.set_zero();
            b.torque = R::zero();
            b.linear_velocity.multiply(b2_clamp(R::one() - step.dt * b.linear_damping, R::zero(), R::one()));
            b.angular_velocity = b.angular_velocity * b2_clamp(R::one() - step.dt * b.angular_damping, R::zero(), R::one());
            if b.linear_velocity.length_squared().gt(settings::b2_max_linear_velocity_squared::<R>()) {
                b.linear_velocity.normalize();
                b.linear_velocity.x = b.linear_velocity.x * settings::b2_max_linear_velocity::<R>();
                b.linear_velocity.y = b.linear_velocity.y * settings::b2_max_linear_velocity::<R>();
            }
            if (b.angular_velocity * b.angular_velocity).gt(settings::b2_max_angular_velocity_squared::<R>()) {
                if b.angular_velocity.lt(R::zero()) {
                    b.angular_velocity = -settings::b2_max_angular_velocity::<R>();
                } else {
                    b.angular_velocity = settings::b2_max_angular_velocity::<R>();
                }
            }
        }
        i += 1;
    }
    // new b2ContactSolver(step, m_contacts, m_contactCount, allocator)
    contact_solver_new(ctx, step);
    contact_solver_init_velocity_constraints(ctx, step);
    i = 0;
    while i < joint_count {
        let j = ctx.scratch.island.joints[i as usize];
        joints::init_velocity_constraints(ctx, j, step);
        i += 1;
    }
    i = 0;
    while i < step.max_iterations {
        contact_solver_solve_velocity_constraints(ctx);
        let mut j: i32 = 0;
        while j < joint_count {
            let jh = ctx.scratch.island.joints[j as usize];
            joints::solve_velocity_constraints(ctx, jh, step);
            j += 1;
        }
        i += 1;
    }
    contact_solver_finalize_velocity_constraints(ctx);
    // Integrate positions.
    i = 0;
    while i < body_count {
        let bh = ctx.scratch.island.bodies[i as usize];
        if !body::is_static(ctx, bh) {
            {
                let b = ctx.world.body_mut(bh);
                let c = b.sweep.c;
                b.sweep.c0.set_v(c);
                b.sweep.a0 = b.sweep.a;
                b.sweep.c.x = b.sweep.c.x + step.dt * b.linear_velocity.x;
                b.sweep.c.y = b.sweep.c.y + step.dt * b.linear_velocity.y;
                b.sweep.a = b.sweep.a + step.dt * b.angular_velocity;
            }
            body::synchronize_transform(ctx, bh);
        }
        i += 1;
    }
    if correct_positions {
        i = 0;
        while i < joint_count {
            let j = ctx.scratch.island.joints[i as usize];
            joints::init_position_constraints(ctx, j);
            i += 1;
        }
        ctx.scratch.island.position_iteration_count = 0;
        while ctx.scratch.island.position_iteration_count < step.max_iterations {
            let contacts_okay = contact_solver_solve_position_constraints(ctx, settings::b2_contact_baumgarte::<R>());
            let mut joints_okay = true;
            i = 0;
            while i < joint_count {
                let j = ctx.scratch.island.joints[i as usize];
                let joint_okay = joints::solve_position_constraints(ctx, j);
                joints_okay = joints_okay && joint_okay;
                i += 1;
            }
            if contacts_okay && joints_okay {
                break;
            }
            ctx.scratch.island.position_iteration_count += 1;
        }
    }
    lab! {
        if crate::lab::armed() {
            let mut k: i32 = 0;
            while k < body_count {
                let bh = ctx.scratch.island.bodies[k as usize];
                if crate::lab::watched(bh) {
                    let mut injected = false;
                    if let Some(inj) = crate::lab::take_injection(bh) {
                        let b = ctx.world.body_mut(bh);
                        if let Some(o) = inj.origin {
                            b.sweep.c0.x = crate::lab::r(o[0]);
                            b.sweep.c0.y = crate::lab::r(o[1]);
                        }
                        b.sweep.c.x = b.sweep.c0.x + crate::lab::r::<R>(inj.displacement[0]);
                        b.sweep.c.y = b.sweep.c0.y + crate::lab::r::<R>(inj.displacement[1]);
                        b.linear_velocity.x = crate::lab::r(inj.velocity[0]);
                        b.linear_velocity.y = crate::lab::r(inj.velocity[1]);
                        if let Some(w) = inj.angular {
                            b.angular_velocity = crate::lab::r(w);
                        }
                        if let Some(da) = inj.angle_delta {
                            b.sweep.a = b.sweep.a0 + crate::lab::r::<R>(da);
                        }
                        body::synchronize_transform(ctx, bh);
                        injected = true;
                    }
                    let b = ctx.world.body(bh);
                    use crate::lab::f;
                    crate::lab::record(crate::lab::Event::Solve {
                        body: bh,
                        c0: [f(b.sweep.c0.x), f(b.sweep.c0.y)],
                        c: [f(b.sweep.c.x), f(b.sweep.c.y)],
                        v: [f(b.linear_velocity.x), f(b.linear_velocity.y)],
                        w: f(b.angular_velocity),
                        injected,
                    });
                }
                k += 1;
            }
        }
    }
    {
        lab! { let _report_phase = crate::lab::report_phase(crate::lab::ReportPhase::Main); }
        flow!(report(ctx));
    }
    if allow_sleep {
        let mut min_sleep_time: R = settings::max_value::<R>();
        let lin_tol_sqr: R = settings::b2_linear_sleep_tolerance::<R>() * settings::b2_linear_sleep_tolerance::<R>();
        let ang_tol_sqr: R = settings::b2_angular_sleep_tolerance::<R>() * settings::b2_angular_sleep_tolerance::<R>();
        i = 0;
        while i < body_count {
            let bh = ctx.scratch.island.bodies[i as usize];
            let b = ctx.world.body_mut(bh);
            if !b.inv_mass.eq_num(R::zero()) {
                if (b.flags & E_ALLOW_SLEEP_FLAG) == 0 {
                    b.sleep_time = R::zero();
                    min_sleep_time = R::zero();
                }
                if (b.flags & E_ALLOW_SLEEP_FLAG) == 0
                    || (b.angular_velocity * b.angular_velocity).gt(ang_tol_sqr)
                    || b2_dot(b.linear_velocity, b.linear_velocity).gt(lin_tol_sqr)
                {
                    b.sleep_time = R::zero();
                    min_sleep_time = R::zero();
                } else {
                    b.sleep_time = b.sleep_time + step.dt;
                    min_sleep_time = b2_min(min_sleep_time, b.sleep_time);
                }
            }
            i += 1;
        }
        if min_sleep_time.ge(settings::b2_time_to_sleep::<R>()) {
            i = 0;
            while i < body_count {
                let bh = ctx.scratch.island.bodies[i as usize];
                let b = ctx.world.body_mut(bh);
                b.flags |= E_SLEEP_FLAG;
                b.linear_velocity.set_zero();
                b.angular_velocity = R::zero();
                i += 1;
            }
        }
    }
    Flow::Continue
}

/// b2Island.SolveTOI(subStep)
pub fn solve_toi<R: Real>(ctx: &mut Ctx<R>, sub_step: &TimeStep<R>) -> Flow {
    let body_count = ctx.scratch.island.body_count;
    contact_solver_new(ctx, sub_step);
    // No warm starting needed for TOI events.
    let mut i: i32 = 0;
    while i < sub_step.max_iterations {
        contact_solver_solve_velocity_constraints(ctx);
        i += 1;
    }
    i = 0;
    while i < body_count {
        let bh = ctx.scratch.island.bodies[i as usize];
        if !body::is_static(ctx, bh) {
            {
                let b = ctx.world.body_mut(bh);
                let c = b.sweep.c;
                b.sweep.c0.set_v(c);
                b.sweep.a0 = b.sweep.a;
                b.sweep.c.x = b.sweep.c.x + sub_step.dt * b.linear_velocity.x;
                b.sweep.c.y = b.sweep.c.y + sub_step.dt * b.linear_velocity.y;
                b.sweep.a = b.sweep.a + sub_step.dt * b.angular_velocity;
            }
            body::synchronize_transform(ctx, bh);
        }
        i += 1;
    }
    let k_toi_baumgarte: R = rl!(0.75);
    i = 0;
    while i < sub_step.max_iterations {
        let contacts_okay = contact_solver_solve_position_constraints(ctx, k_toi_baumgarte);
        if contacts_okay {
            break;
        }
        i += 1;
    }
    {
        lab! { let _report_phase = crate::lab::report_phase(crate::lab::ReportPhase::Toi); }
        flow!(report(ctx));
    }
    Flow::Continue
}

/// b2Island.Report(constraints): island-contact order -> manifold -> point. The constraint is
/// indexed by the contact index (holds because every island contact has manifoldCount == 1).
pub fn report<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    if ctx.listener_mode == 0 {
        return Flow::Continue;
    }
    let contact_count = ctx.scratch.island.contact_count;
    let mut i: i32 = 0;
    while i < contact_count {
        let c = ctx.scratch.island.contacts[i as usize];
        let (shape1, shape2, manifold_count) = {
            let ct = ctx.world.contact(c);
            (ct.shape1, ct.shape2, ct.manifold_count)
        };
        let b1 = ctx.world.shape(shape1).body;
        // GetManifolds() is the single-element [m_manifold] array.
        let mut j: i32 = 0;
        while j < manifold_count {
            let normal = ctx.world.contact(c).manifold.normal;
            ctx.world.report_cr_normal.set_v(normal);
            let mut k: i32 = 0;
            while k < ctx.world.contact(c).manifold.point_count {
                let point: ManifoldPoint<R> = ctx.world.contact(c).manifold.points[k as usize];
                let ccp: ContactConstraintPoint<R> = ctx.scratch.constraints[i as usize].points[k as usize];
                let position = body::get_world_point(ctx, b1, point.local_point1);
                let cr = ContactResult {
                    shape1,
                    shape2,
                    position,
                    normal_impulse: ccp.normal_impulse,
                    tangent_impulse: ccp.tangent_impulse,
                    id: point.id,
                };
                flow!(crate::listener::result(ctx, &cr));
                k += 1;
            }
            j += 1;
        }
        i += 1;
    }
    Flow::Continue
}

// ---- b2ContactSolver --------------------------------------------------------------------------

/// `new b2ContactSolver(step, contacts, contactCount, allocator)` over the island contacts.
pub fn contact_solver_new<R: Real>(ctx: &mut Ctx<R>, _step: &TimeStep<R>) {
    let contact_count = ctx.scratch.island.contact_count;
    let mut constraint_count: i32 = 0;
    let mut i: i32 = 0;
    while i < contact_count {
        let c = ctx.scratch.island.contacts[i as usize];
        constraint_count += ctx.world.contact(c).manifold_count;
        i += 1;
    }
    ctx.scratch.constraint_count = constraint_count;
    i = 0;
    while i < constraint_count {
        ctx.scratch.constraints[i as usize] = ContactConstraint::blank();
        i += 1;
    }
    let mut count: i32 = 0;
    i = 0;
    while i < contact_count {
        let c = ctx.scratch.island.contacts[i as usize];
        let (shape1, shape2, manifold_count, friction, restitution) = {
            let ct = ctx.world.contact(c);
            (ct.shape1, ct.shape2, ct.manifold_count, ct.friction, ct.restitution)
        };
        let b1h = ctx.world.shape(shape1).body;
        let b2h = ctx.world.shape(shape2).body;
        let b1: Body<R> = *ctx.world.body(b1h);
        let b2: Body<R> = *ctx.world.body(b2h);
        let v1x = b1.linear_velocity.x;
        let v1y = b1.linear_velocity.y;
        let v2x = b2.linear_velocity.x;
        let v2y = b2.linear_velocity.y;
        let w1 = b1.angular_velocity;
        let w2 = b2.angular_velocity;
        let mut j: i32 = 0;
        while j < manifold_count {
            let manifold: Manifold<R> = ctx.world.contact(c).manifold;
            let normal_x = manifold.normal.x;
            let normal_y = manifold.normal.y;
            let cc = &mut ctx.scratch.constraints[count as usize];
            cc.body1 = b1h;
            cc.body2 = b2h;
            cc.contact = c;
            cc.normal.x = normal_x;
            cc.normal.y = normal_y;
            cc.point_count = manifold.point_count;
            cc.friction = friction;
            cc.restitution = restitution;
            let mut k: u32 = 0;
            while (k as i32) < cc.point_count {
                let cp: ManifoldPoint<R> = manifold.points[k as usize];
                let ccp = &mut cc.points[k as usize];
                ccp.normal_impulse = cp.normal_impulse;
                ccp.tangent_impulse = cp.tangent_impulse;
                ccp.separation = cp.separation;
                ccp.position_impulse = R::zero();
                ccp.local_anchor1.set_v(cp.local_point1);
                ccp.local_anchor2.set_v(cp.local_point2);
                let mut t_mat: Mat22<R> = b1.xf.r;
                let mut r1x = cp.local_point1.x - b1.sweep.local_center.x;
                let mut r1y = cp.local_point1.y - b1.sweep.local_center.y;
                let mut t_x = t_mat.col1.x * r1x + t_mat.col2.x * r1y;
                r1y = t_mat.col1.y * r1x + t_mat.col2.y * r1y;
                r1x = t_x;
                ccp.r1.set(r1x, r1y);
                t_mat = b2.xf.r;
                let mut r2x = cp.local_point2.x - b2.sweep.local_center.x;
                let mut r2y = cp.local_point2.y - b2.sweep.local_center.y;
                t_x = t_mat.col1.x * r2x + t_mat.col2.x * r2y;
                r2y = t_mat.col1.y * r2x + t_mat.col2.y * r2y;
                r2x = t_x;
                ccp.r2.set(r2x, r2y);
                let r1_sqr = r1x * r1x + r1y * r1y;
                let r2_sqr = r2x * r2x + r2y * r2y;
                let rn1 = r1x * normal_x + r1y * normal_y;
                let rn2 = r2x * normal_x + r2y * normal_y;
                let mut k_normal = b1.inv_mass + b2.inv_mass;
                k_normal = k_normal + (b1.inv_i * (r1_sqr - rn1 * rn1) + b2.inv_i * (r2_sqr - rn2 * rn2));
                ccp.normal_mass = R::one() / k_normal;
                let mut k_equalized = b1.mass * b1.inv_mass + b2.mass * b2.inv_mass;
                k_equalized = k_equalized
                    + (b1.mass * b1.inv_i * (r1_sqr - rn1 * rn1) + b2.mass * b2.inv_i * (r2_sqr - rn2 * rn2));
                ccp.equalized_mass = R::one() / k_equalized;
                let tangent_x = normal_y;
                let tangent_y = -normal_x;
                let rt1 = r1x * tangent_x + r1y * tangent_y;
                let rt2 = r2x * tangent_x + r2y * tangent_y;
                let mut k_tangent = b1.inv_mass + b2.inv_mass;
                k_tangent = k_tangent + (b1.inv_i * (r1_sqr - rt1 * rt1) + b2.inv_i * (r2_sqr - rt2 * rt2));
                ccp.tangent_mass = R::one() / k_tangent;
                ccp.velocity_bias = R::zero();
                if ccp.separation.gt(R::zero()) {
                    ccp.velocity_bias = -R::from_i32(60) * ccp.separation;
                }
                t_x = v2x + -w2 * r2y - v1x - -w1 * r1y;
                let t_y = v2y + w2 * r2x - v1y - w1 * r1x;
                let v_rel = cc.normal.x * t_x + cc.normal.y * t_y;
                if v_rel.lt(-settings::b2_velocity_threshold::<R>()) {
                    ccp.velocity_bias = ccp.velocity_bias + -cc.restitution * v_rel;
                }
                k += 1;
            }
            count += 1;
            j += 1;
        }
        i += 1;
    }
}

/// b2ContactSolver.InitVelocityConstraints(step)
pub fn contact_solver_init_velocity_constraints<R: Real>(ctx: &mut Ctx<R>, step: &TimeStep<R>) {
    let constraint_count = ctx.scratch.constraint_count;
    let mut i: i32 = 0;
    while i < constraint_count {
        let (b1h, b2h, normal, point_count) = {
            let c = &ctx.scratch.constraints[i as usize];
            (c.body1, c.body2, c.normal, c.point_count)
        };
        let inv_mass1 = ctx.world.body(b1h).inv_mass;
        let inv_i1 = ctx.world.body(b1h).inv_i;
        let inv_mass2 = ctx.world.body(b2h).inv_mass;
        let inv_i2 = ctx.world.body(b2h).inv_i;
        let normal_x = normal.x;
        let normal_y = normal.y;
        let tangent_x = normal_y;
        let tangent_y = -normal_x;
        if step.warm_starting.get() {
            let mut j: i32 = 0;
            while j < point_count {
                let ccp = &mut ctx.scratch.constraints[i as usize].points[j as usize];
                ccp.normal_impulse = ccp.normal_impulse * step.dt_ratio;
                ccp.tangent_impulse = ccp.tangent_impulse * step.dt_ratio;
                let p_x = ccp.normal_impulse * normal_x + ccp.tangent_impulse * tangent_x;
                let p_y = ccp.normal_impulse * normal_y + ccp.tangent_impulse * tangent_y;
                let r1 = ccp.r1;
                let r2 = ccp.r2;
                {
                    let b1 = ctx.world.body_mut(b1h);
                    b1.angular_velocity = b1.angular_velocity - inv_i1 * (r1.x * p_y - r1.y * p_x);
                    b1.linear_velocity.x = b1.linear_velocity.x - inv_mass1 * p_x;
                    b1.linear_velocity.y = b1.linear_velocity.y - inv_mass1 * p_y;
                }
                {
                    let b2 = ctx.world.body_mut(b2h);
                    b2.angular_velocity = b2.angular_velocity + inv_i2 * (r2.x * p_y - r2.y * p_x);
                    b2.linear_velocity.x = b2.linear_velocity.x + inv_mass2 * p_x;
                    b2.linear_velocity.y = b2.linear_velocity.y + inv_mass2 * p_y;
                }
                j += 1;
            }
        } else {
            let mut j: i32 = 0;
            while j < point_count {
                let ccp = &mut ctx.scratch.constraints[i as usize].points[j as usize];
                ccp.normal_impulse = R::zero();
                ccp.tangent_impulse = R::zero();
                j += 1;
            }
        }
        i += 1;
    }
}

/// b2ContactSolver.SolveVelocityConstraints. Keeps the AS3 asymmetry: the linear velocities are
/// the bodies' shared b2Vec2 objects (mutated in place per point), the angular velocities are
/// locals written back after the point loop.
pub fn contact_solver_solve_velocity_constraints<R: Real>(ctx: &mut Ctx<R>) {
    let constraint_count = ctx.scratch.constraint_count;
    let mut i: i32 = 0;
    while i < constraint_count {
        let (b1h, b2h, normal, friction, point_count) = {
            let c = &ctx.scratch.constraints[i as usize];
            (c.body1, c.body2, c.normal, c.friction, c.point_count)
        };
        let mut w1 = ctx.world.body(b1h).angular_velocity;
        let mut w2 = ctx.world.body(b2h).angular_velocity;
        let inv_mass1 = ctx.world.body(b1h).inv_mass;
        let inv_i1 = ctx.world.body(b1h).inv_i;
        let inv_mass2 = ctx.world.body(b2h).inv_mass;
        let inv_i2 = ctx.world.body(b2h).inv_i;
        let normal_x = normal.x;
        let normal_y = normal.y;
        let tangent_x = normal_y;
        let tangent_y = -normal_x;
        let mut j: i32 = 0;
        while j < point_count {
            let ccp: ContactConstraintPoint<R> = ctx.scratch.constraints[i as usize].points[j as usize];
            let v1 = ctx.world.body(b1h).linear_velocity;
            let v2 = ctx.world.body(b2h).linear_velocity;
            let dvx = v2.x + -w2 * ccp.r2.y - v1.x - -w1 * ccp.r1.y;
            let dvy = v2.y + w2 * ccp.r2.x - v1.y - w1 * ccp.r1.x;
            let vn = dvx * normal_x + dvy * normal_y;
            let mut lambda = -ccp.normal_mass * (vn - ccp.velocity_bias);
            let vt = dvx * tangent_x + dvy * tangent_y;
            let mut lambda_t = ccp.tangent_mass * -vt;
            let new_impulse = b2_max(ccp.normal_impulse + lambda, R::zero());
            lambda = new_impulse - ccp.normal_impulse;
            let max_friction = friction * ccp.normal_impulse;
            let new_impulse_t = b2_clamp(ccp.tangent_impulse + lambda_t, -max_friction, max_friction);
            lambda_t = new_impulse_t - ccp.tangent_impulse;
            let p_x = lambda * normal_x + lambda_t * tangent_x;
            let p_y = lambda * normal_y + lambda_t * tangent_y;
            {
                let b1 = ctx.world.body_mut(b1h);
                b1.linear_velocity.x = b1.linear_velocity.x - inv_mass1 * p_x;
                b1.linear_velocity.y = b1.linear_velocity.y - inv_mass1 * p_y;
            }
            w1 = w1 - inv_i1 * (ccp.r1.x * p_y - ccp.r1.y * p_x);
            {
                let b2 = ctx.world.body_mut(b2h);
                b2.linear_velocity.x = b2.linear_velocity.x + inv_mass2 * p_x;
                b2.linear_velocity.y = b2.linear_velocity.y + inv_mass2 * p_y;
            }
            w2 = w2 + inv_i2 * (ccp.r2.x * p_y - ccp.r2.y * p_x);
            {
                let p = &mut ctx.scratch.constraints[i as usize].points[j as usize];
                p.normal_impulse = new_impulse;
                p.tangent_impulse = new_impulse_t;
            }
            j += 1;
        }
        ctx.world.body_mut(b1h).angular_velocity = w1;
        ctx.world.body_mut(b2h).angular_velocity = w2;
        i += 1;
    }
}

/// b2ContactSolver.FinalizeVelocityConstraints: copy the impulses back into the manifolds.
pub fn contact_solver_finalize_velocity_constraints<R: Real>(ctx: &mut Ctx<R>) {
    let constraint_count = ctx.scratch.constraint_count;
    let mut i: i32 = 0;
    while i < constraint_count {
        let (c, point_count) = {
            let cc = &ctx.scratch.constraints[i as usize];
            (cc.contact, cc.point_count)
        };
        let mut j: i32 = 0;
        while j < point_count {
            let (ni, ti) = {
                let p = &ctx.scratch.constraints[i as usize].points[j as usize];
                (p.normal_impulse, p.tangent_impulse)
            };
            let mp = &mut ctx.world.contact_mut(c).manifold.points[j as usize];
            mp.normal_impulse = ni;
            mp.tangent_impulse = ti;
            j += 1;
        }
        i += 1;
    }
}

/// b2ContactSolver.SolvePositionConstraints(baumgarte) -> minSeparation >= -1.5 * b2_linearSlop.
/// `b1.m_sweep.c` is the body's shared b2Vec2 (mutated in place per point); the angle is a local
/// written back before each SynchronizeTransform.
pub fn contact_solver_solve_position_constraints<R: Real>(ctx: &mut Ctx<R>, baumgarte: R) -> bool {
    let mut min_separation: R = R::zero();
    let constraint_count = ctx.scratch.constraint_count;
    let mut i: i32 = 0;
    while i < constraint_count {
        let (b1h, b2h, normal, point_count) = {
            let c = &ctx.scratch.constraints[i as usize];
            (c.body1, c.body2, c.normal, c.point_count)
        };
        let (mut a1, inv_mass1, inv_i1) = {
            let b1 = ctx.world.body(b1h);
            (b1.sweep.a, b1.mass * b1.inv_mass, b1.mass * b1.inv_i)
        };
        let (mut a2, inv_mass2, inv_i2) = {
            let b2 = ctx.world.body(b2h);
            (b2.sweep.a, b2.mass * b2.inv_mass, b2.mass * b2.inv_i)
        };
        let normal_x = normal.x;
        let normal_y = normal.y;
        let mut j: i32 = 0;
        while j < point_count {
            let (local_anchor1, local_anchor2, separation, equalized_mass, position_impulse) = {
                let p = &ctx.scratch.constraints[i as usize].points[j as usize];
                (p.local_anchor1, p.local_anchor2, p.separation, p.equalized_mass, p.position_impulse)
            };
            let mut t_mat: Mat22<R> = ctx.world.body(b1h).xf.r;
            let mut t_vec: Vec2<R> = ctx.world.body(b1h).sweep.local_center;
            let mut r1x = local_anchor1.x - t_vec.x;
            let mut r1y = local_anchor1.y - t_vec.y;
            let mut t_x = t_mat.col1.x * r1x + t_mat.col2.x * r1y;
            r1y = t_mat.col1.y * r1x + t_mat.col2.y * r1y;
            r1x = t_x;
            t_mat = ctx.world.body(b2h).xf.r;
            t_vec = ctx.world.body(b2h).sweep.local_center;
            let mut r2x = local_anchor2.x - t_vec.x;
            let mut r2y = local_anchor2.y - t_vec.y;
            t_x = t_mat.col1.x * r2x + t_mat.col2.x * r2y;
            r2y = t_mat.col1.y * r2x + t_mat.col2.y * r2y;
            r2x = t_x;
            let c1 = ctx.world.body(b1h).sweep.c;
            let c2 = ctx.world.body(b2h).sweep.c;
            let p1x = c1.x + r1x;
            let p1y = c1.y + r1y;
            let p2x = c2.x + r2x;
            let p2y = c2.y + r2y;
            let dpx = p2x - p1x;
            let dpy = p2y - p1y;
            let sep = dpx * normal_x + dpy * normal_y + separation;
            min_separation = b2_min(min_separation, sep);
            let c = baumgarte
                * b2_clamp(
                    sep + settings::b2_linear_slop::<R>(),
                    -settings::b2_max_linear_correction::<R>(),
                    R::zero(),
                );
            let mut d_impulse = -equalized_mass * c;
            let impulse0 = position_impulse;
            let new_position_impulse = b2_max(impulse0 + d_impulse, R::zero());
            ctx.scratch.constraints[i as usize].points[j as usize].position_impulse = new_position_impulse;
            d_impulse = new_position_impulse - impulse0;
            let impulse_x = d_impulse * normal_x;
            let impulse_y = d_impulse * normal_y;
            {
                let b1 = ctx.world.body_mut(b1h);
                b1.sweep.c.x = b1.sweep.c.x - inv_mass1 * impulse_x;
                b1.sweep.c.y = b1.sweep.c.y - inv_mass1 * impulse_y;
                a1 = a1 - inv_i1 * (r1x * impulse_y - r1y * impulse_x);
                b1.sweep.a = a1;
            }
            body::synchronize_transform(ctx, b1h);
            {
                let b2 = ctx.world.body_mut(b2h);
                b2.sweep.c.x = b2.sweep.c.x + inv_mass2 * impulse_x;
                b2.sweep.c.y = b2.sweep.c.y + inv_mass2 * impulse_y;
                a2 = a2 + inv_i2 * (r2x * impulse_y - r2y * impulse_x);
                b2.sweep.a = a2;
            }
            body::synchronize_transform(ctx, b2h);
            j += 1;
        }
        i += 1;
    }
    let one_and_half: R = rl!(1.5);
    min_separation.ge(-one_and_half * settings::b2_linear_slop::<R>())
}
