//! b2Joint / b2JointDef / b2Jacobian / b2RevoluteJoint / b2PrismaticJoint / b2DistanceJoint
//! (Box2DFlashAS3 2.0.2, force-based joints). Line-by-line port; statement order and floating
//! point association follow the AS3 exactly.
//!
//! Object model: a joint is a `Joint<R>` record in `ctx.world.joints` addressed by `H`; the
//! record is a superset of the three joint types and `type_` selects the behaviour. The AS3 body
//! accesses (`m_body1.m_xf`, `m_linearVelocity`, ...) are reads/writes of `ctx.world.body(h)` at
//! the same points: the AS3 mutates `b1.m_linearVelocity` / `b1.m_angularVelocity` through the
//! body object immediately, so every AS3 write becomes an immediate write to the body record and
//! every later read re-reads the record (this keeps `body1 == body2` aliasing exact too).
//! The joint's own fields are copied into a local at entry and written back at exit (nothing
//! else observes the joint in between; `SynchronizeTransform` only touches the body).
//!
//! Gear joints are used by BicycleGuy. Mouse/pulley joints remain outside the game port.
use hw_real::as3::{b2_abs, b2_clamp, b2_max, b2_min};
use hw_real::Real;

use super::body;
use super::math::{Mat22, Vec2};
use super::settings::{
    b2_angular_slop, b2_linear_slop, b2_max_angular_correction, b2_max_linear_correction, b2_pi, min_value,
};
use super::state::{
    Edge, Jacobian, Joint, JointDef, TimeStep, E_AT_LOWER_LIMIT, E_AT_UPPER_LIMIT, E_DISTANCE_JOINT, E_EQUAL_LIMITS,
    E_INACTIVE_LIMIT, E_PRISMATIC_JOINT, E_REVOLUTE_JOINT,
};
use crate::{Ctx, H, NONE};

// ---- b2Jacobian ------------------------------------------------------------------------------

impl<R: Real> Jacobian<R> {
    /// `new b2Jacobian()`: vectors zero, angular Numbers NaN (uninitialised).
    pub fn new() -> Self {
        let nan = R::from_words([0x7fffffff, 0xe0000000]); // Flash uninitialized Number payload.
        Jacobian { linear1: Vec2::zero(), angular1: nan, linear2: Vec2::zero(), angular2: nan }
    }
    /// b2Jacobian.SetZero
    #[inline(always)]
    pub fn set_zero(&mut self) {
        self.linear1.set_zero();
        self.angular1 = R::zero();
        self.linear2.set_zero();
        self.angular2 = R::zero();
    }
    /// b2Jacobian.Set
    #[inline(always)]
    pub fn set(&mut self, x1: Vec2<R>, a1: R, x2: Vec2<R>, a2: R) {
        self.linear1.set_v(x1);
        self.angular1 = a1;
        self.linear2.set_v(x2);
        self.angular2 = a2;
    }
    /// b2Jacobian.Compute (exact AS3 association: the linear2 dot product is parenthesised).
    #[inline(always)]
    pub fn compute(&self, x1: Vec2<R>, a1: R, x2: Vec2<R>, a2: R) -> R {
        self.linear1.x * x1.x + self.linear1.y * x1.y + self.angular1 * a1 + (self.linear2.x * x2.x + self.linear2.y * x2.y) + self.angular2 * a2
    }
}

// ---- body access helpers (each is one AS3 statement pattern) ---------------------------------

/// `r = b2Mul(b.m_xf.R, localAnchor - b.m_sweep.localCenter)` written the AS3 way (tX temp).
#[inline(always)]
fn anchor_r<R: Real>(ctx: &Ctx<R>, b: H, local_anchor: Vec2<R>) -> (R, R) {
    let body = ctx.world.body(b);
    let t_mat = body.xf.r;
    let mut r_x = local_anchor.x - body.sweep.local_center.x;
    let mut r_y = local_anchor.y - body.sweep.local_center.y;
    let t_x = t_mat.col1.x * r_x + t_mat.col2.x * r_y;
    r_y = t_mat.col1.y * r_x + t_mat.col2.y * r_y;
    r_x = t_x;
    (r_x, r_y)
}

/// `b2Mul(b.m_xf.R, v)` as written inline in the joints (`R.col1.x * v.x + R.col2.x * v.y`, ...).
#[inline(always)]
fn rotate<R: Real>(ctx: &Ctx<R>, b: H, v: Vec2<R>) -> (R, R) {
    let t_mat = ctx.world.body(b).xf.r;
    let x = t_mat.col1.x * v.x + t_mat.col2.x * v.y;
    let y = t_mat.col1.y * v.x + t_mat.col2.y * v.y;
    (x, y)
}

#[inline(always)]
fn lv<R: Real>(ctx: &Ctx<R>, b: H) -> Vec2<R> {
    ctx.world.body(b).linear_velocity
}
#[inline(always)]
fn av<R: Real>(ctx: &Ctx<R>, b: H) -> R {
    ctx.world.body(b).angular_velocity
}
/// `b.m_linearVelocity.x -= dx; b.m_linearVelocity.y -= dy;`
#[inline(always)]
fn lv_sub<R: Real>(ctx: &mut Ctx<R>, b: H, dx: R, dy: R) {
    let body = ctx.world.body_mut(b);
    body.linear_velocity.x = body.linear_velocity.x - dx;
    body.linear_velocity.y = body.linear_velocity.y - dy;
}
/// `b.m_linearVelocity.x += dx; b.m_linearVelocity.y += dy;`
#[inline(always)]
fn lv_add<R: Real>(ctx: &mut Ctx<R>, b: H, dx: R, dy: R) {
    let body = ctx.world.body_mut(b);
    body.linear_velocity.x = body.linear_velocity.x + dx;
    body.linear_velocity.y = body.linear_velocity.y + dy;
}
/// `b.m_angularVelocity -= d`
#[inline(always)]
fn av_sub<R: Real>(ctx: &mut Ctx<R>, b: H, d: R) {
    let body = ctx.world.body_mut(b);
    body.angular_velocity = body.angular_velocity - d;
}
/// `b.m_angularVelocity += d`
#[inline(always)]
fn av_add<R: Real>(ctx: &mut Ctx<R>, b: H, d: R) {
    let body = ctx.world.body_mut(b);
    body.angular_velocity = body.angular_velocity + d;
}
/// `b.m_sweep.c.x -= dx; b.m_sweep.c.y -= dy;`
#[inline(always)]
fn sc_sub<R: Real>(ctx: &mut Ctx<R>, b: H, dx: R, dy: R) {
    let body = ctx.world.body_mut(b);
    body.sweep.c.x = body.sweep.c.x - dx;
    body.sweep.c.y = body.sweep.c.y - dy;
}
/// `b.m_sweep.c.x += dx; b.m_sweep.c.y += dy;`
#[inline(always)]
fn sc_add<R: Real>(ctx: &mut Ctx<R>, b: H, dx: R, dy: R) {
    let body = ctx.world.body_mut(b);
    body.sweep.c.x = body.sweep.c.x + dx;
    body.sweep.c.y = body.sweep.c.y + dy;
}
/// `b.m_sweep.a -= d`
#[inline(always)]
fn sa_sub<R: Real>(ctx: &mut Ctx<R>, b: H, d: R) {
    let body = ctx.world.body_mut(b);
    body.sweep.a = body.sweep.a - d;
}
/// `b.m_sweep.a += d`
#[inline(always)]
fn sa_add<R: Real>(ctx: &mut Ctx<R>, b: H, d: R) {
    let body = ctx.world.body_mut(b);
    body.sweep.a = body.sweep.a + d;
}

// ---- b2Joint ---------------------------------------------------------------------------------

/// `b2Joint(def)` base constructor: fills the common fields of a blank record.
fn base_construct<R: Real>(def: &JointDef<R>) -> Joint<R> {
    let mut j = Joint::blank();
    // field initialisers: m_node1/m_node2 = new b2JointEdge(); broken = false (Boolean default)
    j.node1 = Edge::new();
    j.node2 = Edge::new();
    j.broken = crate::Flag32::FALSE;
    // constructor body
    j.type_ = def.type_;
    j.prev = NONE;
    j.next = NONE;
    j.body1 = def.body1;
    j.body2 = def.body2;
    j.collide_connected = def.collide_connected;
    j.island_flag = crate::Flag32::FALSE;
    j.user_data = def.user_data;
    j
}

/// b2Joint.Create: dispatch on `def.type_` to the concrete constructor and allocate the record.
/// Returns `NONE` when the port's joint capacity is exhausted (caller: `Flow::Abort(PortCapacity)`)
/// or for an unsupported joint type.
pub fn create<R: Real>(ctx: &mut Ctx<R>, def: &JointDef<R>) -> H {
    let record = match def.type_ {
        E_DISTANCE_JOINT => distance_construct(def),
        E_PRISMATIC_JOINT => prismatic_construct(def),
        E_REVOLUTE_JOINT => revolute_construct(def),
        6 => gear_construct(ctx,def),
        _ => {
            debug_assert!(false, "unported joint type");
            return NONE;
        }
    };
    ctx.world.joints.alloc(record)
}

/// b2Joint.Destroy: the AS3 static is an empty function (the block allocator is unused in Flash).
/// The port frees the arena slot here *if it is still alive*, so it is safe whether the world
/// calls `ctx.world.joints.destroy(h)` itself (this becomes a no-op) or relies on this hook.
pub fn destroy<R: Real>(ctx: &mut Ctx<R>, joint: H) {
    if ctx.world.joints.is_alive(joint) {
        ctx.world.joints.destroy(joint);
    }
}

pub fn get_type<R: Real>(ctx: &Ctx<R>, joint: H) -> i32 {
    ctx.world.joint(joint).type_
}
pub fn get_body1<R: Real>(ctx: &Ctx<R>, joint: H) -> H {
    ctx.world.joint(joint).body1
}
pub fn get_body2<R: Real>(ctx: &Ctx<R>, joint: H) -> H {
    ctx.world.joint(joint).body2
}
pub fn get_next<R: Real>(ctx: &Ctx<R>, joint: H) -> H {
    ctx.world.joint(joint).next
}
pub fn get_user_data<R: Real>(ctx: &Ctx<R>, joint: H) -> u32 {
    ctx.world.joint(joint).user_data
}
pub fn set_user_data<R: Real>(ctx: &mut Ctx<R>, joint: H, data: u32) {
    ctx.world.joint_mut(joint).user_data = data;
}

/// GetAnchor1: all three types return `m_body1.GetWorldPoint(m_localAnchor1)`.
pub fn get_anchor1<R: Real>(ctx: &Ctx<R>, joint: H) -> Vec2<R> {
    let j = ctx.world.joint(joint);
    body::get_world_point(ctx, j.body1, j.local_anchor1)
}
/// GetAnchor2
pub fn get_anchor2<R: Real>(ctx: &Ctx<R>, joint: H) -> Vec2<R> {
    let j = ctx.world.joint(joint);
    body::get_world_point(ctx, j.body2, j.local_anchor2)
}

/// GetReactionForce dispatch.
pub fn get_reaction_force<R: Real>(ctx: &Ctx<R>, joint: H) -> Vec2<R> {
    let j = ctx.world.joint(joint);
    match j.type_ {
        E_REVOLUTE_JOINT => j.pivot_force,
        E_PRISMATIC_JOINT => prismatic_get_reaction_force(ctx, j),
        E_DISTANCE_JOINT => distance_get_reaction_force(j),
        6 => Vec2::new(j.force*j.linear_jacobian.linear2.x,j.force*j.linear_jacobian.linear2.y),
        _ => Vec2::zero(),
    }
}

/// GetReactionTorque dispatch.
pub fn get_reaction_torque<R: Real>(ctx: &Ctx<R>, joint: H) -> R {
    let j = ctx.world.joint(joint);
    match j.type_ {
        E_REVOLUTE_JOINT => j.limit_force,
        E_PRISMATIC_JOINT => j.torque,
        6 => {
            // Source uses localAnchor1 with body2, deliberately.
            let (x,y)=anchor_r(ctx,j.body2,j.local_anchor1);
            j.force*j.linear_jacobian.angular2-(x*(j.force*j.linear_jacobian.linear2.y)-y*(j.force*j.linear_jacobian.linear2.x))
        },
        _ => R::zero(),
    }
}

/// InitVelocityConstraints dispatch.
pub fn init_velocity_constraints<R: Real>(ctx: &mut Ctx<R>, joint: H, step: &TimeStep<R>) {
    match ctx.world.joint(joint).type_ {
        E_REVOLUTE_JOINT => revolute_init_velocity_constraints(ctx, joint, step),
        E_PRISMATIC_JOINT => prismatic_init_velocity_constraints(ctx, joint, step),
        E_DISTANCE_JOINT => distance_init_velocity_constraints(ctx, joint, step),
        6=>gear_init_velocity_constraints(ctx,joint,step),
        _ => {}
    }
}

/// SolveVelocityConstraints dispatch.
pub fn solve_velocity_constraints<R: Real>(ctx: &mut Ctx<R>, joint: H, step: &TimeStep<R>) {
    match ctx.world.joint(joint).type_ {
        E_REVOLUTE_JOINT => revolute_solve_velocity_constraints(ctx, joint, step),
        E_PRISMATIC_JOINT => prismatic_solve_velocity_constraints(ctx, joint, step),
        E_DISTANCE_JOINT => distance_solve_velocity_constraints(ctx, joint, step),
        6=>gear_solve_velocity_constraints(ctx,joint,step),
        _ => {}
    }
}

/// InitPositionConstraints: none of the three joint types overrides the empty base method.
pub fn init_position_constraints<R: Real>(_ctx: &mut Ctx<R>, _joint: H) {}

/// SolvePositionConstraints dispatch (base returns false).
pub fn solve_position_constraints<R: Real>(ctx: &mut Ctx<R>, joint: H) -> bool {
    match ctx.world.joint(joint).type_ {
        E_REVOLUTE_JOINT => revolute_solve_position_constraints(ctx, joint),
        E_PRISMATIC_JOINT => prismatic_solve_position_constraints(ctx, joint),
        E_DISTANCE_JOINT => distance_solve_position_constraints(ctx, joint),
        6=>gear_solve_position_constraints(ctx,joint),
        _ => false,
    }
}

// ---- shared revolute/prismatic accessors ----------------------------------------------------

fn gear_coordinate<R:Real>(ctx:&Ctx<R>,h:H)->R {
    if ctx.world.joint(h).type_==E_REVOLUTE_JOINT {get_joint_angle(ctx,h)}else{get_joint_translation(ctx,h)}
}
fn gear_construct<R:Real>(ctx:&Ctx<R>,def:&JointDef<R>)->Joint<R> {
    let mut j=base_construct(def);j.gear_joints=def.gear_joints;
    let a=ctx.world.joint(def.gear_joints[0]);let b=ctx.world.joint(def.gear_joints[1]);
    j.grounds[0]=a.body1;j.body1=a.body2;j.ground_anchors[0]=a.local_anchor1;j.local_anchor1=a.local_anchor2;
    let c1=gear_coordinate(ctx,def.gear_joints[0]);
    j.grounds[1]=b.body1;j.body2=b.body2;j.ground_anchors[1]=b.local_anchor1;j.local_anchor2=b.local_anchor2;
    let c2=gear_coordinate(ctx,def.gear_joints[1]);j.ratio=def.ratio;j.constant=c1+j.ratio*c2;j.force=R::zero();j
}
fn gear_apply_velocity<R:Real>(ctx:&mut Ctx<R>,j:&Joint<R>,impulse:R) {
    let rows = [(j.body1,j.linear_jacobian.linear1,j.linear_jacobian.angular1),(j.body2,j.linear_jacobian.linear2,j.linear_jacobian.angular2)];
    for i in 0..2 {
        let (b,linear,angular) = rows[i];
        lv_add(ctx,b,ctx.world.body(b).inv_mass*impulse*linear.x,ctx.world.body(b).inv_mass*impulse*linear.y);
        av_add(ctx,b,ctx.world.body(b).inv_i*impulse*angular);
    }
}
fn gear_init_velocity_constraints<R:Real>(ctx:&mut Ctx<R>,h:H,step:&TimeStep<R>) {
    let mut j=*ctx.world.joint(h);let b1=j.body1;let b2=j.body2;let mut mass=R::zero();j.linear_jacobian.set_zero();
    if ctx.world.joint(j.gear_joints[0]).type_==E_REVOLUTE_JOINT {
        j.linear_jacobian.angular1=-R::one();mass=mass+ctx.world.body(b1).inv_i;
    }else{
        let (x,y)=rotate(ctx,j.grounds[0],ctx.world.joint(j.gear_joints[0]).local_x_axis1);
        let(rx,ry)=anchor_r(ctx,b1,j.local_anchor1);let cross=rx*y-ry*x;
        j.linear_jacobian.linear1=Vec2::new(-x,-y);j.linear_jacobian.angular1=-cross;
        mass=mass+(ctx.world.body(b1).inv_mass+ctx.world.body(b1).inv_i*cross*cross);
    }
    if ctx.world.joint(j.gear_joints[1]).type_==E_REVOLUTE_JOINT {
        j.linear_jacobian.angular2=-j.ratio;mass=mass+j.ratio*j.ratio*ctx.world.body(b2).inv_i;
    }else{
        let(x,y)=rotate(ctx,j.grounds[1],ctx.world.joint(j.gear_joints[1]).local_x_axis1);
        let(rx,ry)=anchor_r(ctx,b2,j.local_anchor2);let cross=rx*y-ry*x;
        j.linear_jacobian.linear2=Vec2::new(-j.ratio*x,-j.ratio*y);j.linear_jacobian.angular2=-j.ratio*cross;
        mass=mass+j.ratio*j.ratio*(ctx.world.body(b2).inv_mass+ctx.world.body(b2).inv_i*cross*cross);
    }
    j.mass=R::one()/mass;
    if step.warm_starting.get() {gear_apply_velocity(ctx,&j,step.dt*j.force);}else{j.force=R::zero();}
    *ctx.world.joint_mut(h)=j;
}
fn gear_solve_velocity_constraints<R:Real>(ctx:&mut Ctx<R>,h:H,step:&TimeStep<R>) {
    let mut j=*ctx.world.joint(h);let velocity=j.linear_jacobian.compute(lv(ctx,j.body1),av(ctx,j.body1),lv(ctx,j.body2),av(ctx,j.body2));
    let force=-step.inv_dt*j.mass*velocity;j.force=j.force+force;let impulse=step.dt*force;gear_apply_velocity(ctx,&j,impulse);*ctx.world.joint_mut(h)=j;
}
fn gear_solve_position_constraints<R:Real>(ctx:&mut Ctx<R>,h:H)->bool {
    let j=*ctx.world.joint(h);let c1=gear_coordinate(ctx,j.gear_joints[0]);let c2=gear_coordinate(ctx,j.gear_joints[1]);let error=j.constant-(c1+j.ratio*c2);let impulse=-j.mass*error;
    let rows = [(j.body1,j.linear_jacobian.linear1,j.linear_jacobian.angular1),(j.body2,j.linear_jacobian.linear2,j.linear_jacobian.angular2)];
    for i in 0..2 {
        let (b,linear,angular) = rows[i];
        sc_add(ctx,b,ctx.world.body(b).inv_mass*impulse*linear.x,ctx.world.body(b).inv_mass*impulse*linear.y);sa_add(ctx,b,ctx.world.body(b).inv_i*impulse*angular);
    }
    body::synchronize_transform(ctx,j.body1);body::synchronize_transform(ctx,j.body2);
    // AS3's local error variable remains zero, regardless of the actual constraint error.
    R::zero().lt(b2_linear_slop())
}

pub fn is_limit_enabled<R: Real>(ctx: &Ctx<R>, joint: H) -> bool {
    ctx.world.joint(joint).enable_limit.get()
}
pub fn enable_limit<R: Real>(ctx: &mut Ctx<R>, joint: H, flag: bool) {
    ctx.world.joint_mut(joint).enable_limit = crate::Flag32::new(flag);
}
pub fn is_motor_enabled<R: Real>(ctx: &Ctx<R>, joint: H) -> bool {
    ctx.world.joint(joint).enable_motor.get()
}
pub fn enable_motor<R: Real>(ctx: &mut Ctx<R>, joint: H, flag: bool) {
    ctx.world.joint_mut(joint).enable_motor = crate::Flag32::new(flag);
}
pub fn set_motor_speed<R: Real>(ctx: &mut Ctx<R>, joint: H, speed: R) {
    ctx.world.joint_mut(joint).motor_speed = speed;
}
pub fn get_motor_speed<R: Real>(ctx: &Ctx<R>, joint: H) -> R {
    ctx.world.joint(joint).motor_speed
}
/// Revolute: SetLimits(lower, upper) on the angles; prismatic: on the translations.
pub fn set_limits<R: Real>(ctx: &mut Ctx<R>, joint: H, lower: R, upper: R) {
    let j = ctx.world.joint_mut(joint);
    if j.type_ == E_PRISMATIC_JOINT {
        j.lower_translation = lower;
        j.upper_translation = upper;
    } else {
        j.lower_angle = lower;
        j.upper_angle = upper;
    }
}
pub fn get_lower_limit<R: Real>(ctx: &Ctx<R>, joint: H) -> R {
    let j = ctx.world.joint(joint);
    if j.type_ == E_PRISMATIC_JOINT { j.lower_translation } else { j.lower_angle }
}
pub fn get_upper_limit<R: Real>(ctx: &Ctx<R>, joint: H) -> R {
    let j = ctx.world.joint(joint);
    if j.type_ == E_PRISMATIC_JOINT { j.upper_translation } else { j.upper_angle }
}
/// b2RevoluteJoint.GetMotorTorque (= m_motorForce)
pub fn get_motor_torque<R: Real>(ctx: &Ctx<R>, joint: H) -> R {
    ctx.world.joint(joint).motor_force
}
/// b2PrismaticJoint.GetMotorForce (= m_motorForce)
pub fn get_motor_force<R: Real>(ctx: &Ctx<R>, joint: H) -> R {
    ctx.world.joint(joint).motor_force
}
/// b2RevoluteJoint.SetMaxMotorTorque
pub fn set_max_motor_torque<R: Real>(ctx: &mut Ctx<R>, joint: H, torque: R) {
    ctx.world.joint_mut(joint).max_motor_torque = torque;
}
/// b2PrismaticJoint.SetMaxMotorForce
pub fn set_max_motor_force<R: Real>(ctx: &mut Ctx<R>, joint: H, force: R) {
    ctx.world.joint_mut(joint).max_motor_force = force;
}
/// GetJointSpeed: revolute `w2 - w1`; prismatic the full expression.
pub fn get_joint_speed<R: Real>(ctx: &Ctx<R>, joint: H) -> R {
    let j = ctx.world.joint(joint);
    match j.type_ {
        E_REVOLUTE_JOINT => av(ctx, j.body2) - av(ctx, j.body1),
        E_PRISMATIC_JOINT => prismatic_get_joint_speed(ctx, j),
        _ => R::zero(),
    }
}

// =============================================================================================
// b2RevoluteJoint
// =============================================================================================

/// `new b2RevoluteJoint(def)`
fn revolute_construct<R: Real>(def: &JointDef<R>) -> Joint<R> {
    let mut j = base_construct(def);
    // field initialisers: m_pivotMass = new b2Mat22() (blank() already holds Mat22 of angle 0),
    // m_localAnchor1/2, K1/K2/K3/K, m_pivotForce = new b2Vec2() (zero).
    j.local_anchor1.set_v(def.local_anchor1);
    j.local_anchor2.set_v(def.local_anchor2);
    j.reference_angle = def.reference_angle;
    j.pivot_force.set(R::zero(), R::zero());
    j.motor_force = R::zero();
    j.limit_force = R::zero();
    j.limit_position_impulse = R::zero();
    j.lower_angle = def.lower_angle;
    j.upper_angle = def.upper_angle;
    j.max_motor_torque = def.max_motor_torque;
    j.motor_speed = def.motor_speed;
    j.enable_limit = def.enable_limit;
    j.enable_motor = def.enable_motor;
    j
}

/// b2RevoluteJointDef.Initialize(body1, body2, anchor)
pub fn revolute_def_initialize<R: Real>(ctx: &Ctx<R>, def: &mut JointDef<R>, body1: H, body2: H, anchor: Vec2<R>) {
    def.body1 = body1;
    def.body2 = body2;
    def.local_anchor1 = body::get_local_point(ctx, def.body1, anchor);
    def.local_anchor2 = body::get_local_point(ctx, def.body2, anchor);
    def.reference_angle = body::get_angle(ctx, def.body2) - body::get_angle(ctx, def.body1);
}

/// b2RevoluteJoint.GetJointAngle
pub fn get_joint_angle<R: Real>(ctx: &Ctx<R>, joint: H) -> R {
    let j = ctx.world.joint(joint);
    ctx.world.body(j.body2).sweep.a - ctx.world.body(j.body1).sweep.a - j.reference_angle
}

fn revolute_init_velocity_constraints<R: Real>(ctx: &mut Ctx<R>, joint: H, step: &TimeStep<R>) {
    let mut j = *ctx.world.joint(joint);
    let b1 = j.body1;
    let b2 = j.body2;
    let (r1_x, r1_y) = anchor_r(ctx, b1, j.local_anchor1);
    let (r2_x, r2_y) = anchor_r(ctx, b2, j.local_anchor2);
    let inv_mass1 = ctx.world.body(b1).inv_mass;
    let inv_mass2 = ctx.world.body(b2).inv_mass;
    let inv_i1 = ctx.world.body(b1).inv_i;
    let inv_i2 = ctx.world.body(b2).inv_i;
    // K1/K2/K3/K are per-instance scratch matrices in AS3; locals here.
    let mut k1 = Mat22::zero();
    let mut k2 = Mat22::zero();
    let mut k3 = Mat22::zero();
    let mut k = Mat22::zero();
    k1.col1.x = inv_mass1 + inv_mass2;
    k1.col2.x = R::zero();
    k1.col1.y = R::zero();
    k1.col2.y = inv_mass1 + inv_mass2;
    k2.col1.x = inv_i1 * r1_y * r1_y;
    k2.col2.x = -inv_i1 * r1_x * r1_y;
    k2.col1.y = -inv_i1 * r1_x * r1_y;
    k2.col2.y = inv_i1 * r1_x * r1_x;
    k3.col1.x = inv_i2 * r2_y * r2_y;
    k3.col2.x = -inv_i2 * r2_x * r2_y;
    k3.col1.y = -inv_i2 * r2_x * r2_y;
    k3.col2.y = inv_i2 * r2_x * r2_x;
    k.set_m(k1);
    k.add_m(k2);
    k.add_m(k3);
    j.pivot_mass = k.invert();
    j.motor_mass = R::one() / (inv_i1 + inv_i2);
    if j.enable_motor == crate::Flag32::FALSE {
        j.motor_force = R::zero();
    }
    if j.enable_limit.get() {
        let joint_angle = ctx.world.body(b2).sweep.a - ctx.world.body(b1).sweep.a - j.reference_angle;
        if b2_abs(j.upper_angle - j.lower_angle).lt(R::from_i32(2) * b2_angular_slop::<R>()) {
            j.limit_state = E_EQUAL_LIMITS;
        } else if joint_angle.le(j.lower_angle) {
            if j.limit_state != E_AT_LOWER_LIMIT {
                j.limit_force = R::zero();
            }
            j.limit_state = E_AT_LOWER_LIMIT;
        } else if joint_angle.ge(j.upper_angle) {
            if j.limit_state != E_AT_UPPER_LIMIT {
                j.limit_force = R::zero();
            }
            j.limit_state = E_AT_UPPER_LIMIT;
        } else {
            j.limit_state = E_INACTIVE_LIMIT;
            j.limit_force = R::zero();
        }
    } else {
        j.limit_force = R::zero();
    }
    if step.warm_starting.get() {
        // 2.0.2 force-based warm start: the accumulated *forces* are applied as `dt * force`
        // impulses (no dtRatio scaling for revolute/prismatic joints).
        lv_sub(ctx, b1, step.dt * inv_mass1 * j.pivot_force.x, step.dt * inv_mass1 * j.pivot_force.y);
        av_sub(ctx, b1, step.dt * inv_i1 * (r1_x * j.pivot_force.y - r1_y * j.pivot_force.x + j.motor_force + j.limit_force));
        lv_add(ctx, b2, step.dt * inv_mass2 * j.pivot_force.x, step.dt * inv_mass2 * j.pivot_force.y);
        av_add(ctx, b2, step.dt * inv_i2 * (r2_x * j.pivot_force.y - r2_y * j.pivot_force.x + j.motor_force + j.limit_force));
    } else {
        j.pivot_force.set_zero();
        j.motor_force = R::zero();
        j.limit_force = R::zero();
    }
    j.limit_position_impulse = R::zero();
    *ctx.world.joint_mut(joint) = j;
}

fn revolute_solve_velocity_constraints<R: Real>(ctx: &mut Ctx<R>, joint: H, step: &TimeStep<R>) {
    let mut j = *ctx.world.joint(joint);
    let b1 = j.body1;
    let b2 = j.body2;
    let (r1_x, r1_y) = anchor_r(ctx, b1, j.local_anchor1);
    let (r2_x, r2_y) = anchor_r(ctx, b2, j.local_anchor2);
    // Solve point to point constraint
    let v1 = lv(ctx, b1);
    let v2 = lv(ctx, b2);
    let w1 = av(ctx, b1);
    let w2 = av(ctx, b2);
    let pivot_cdot_x = v2.x + -w2 * r2_y - v1.x - -w1 * r1_y;
    let pivot_cdot_y = v2.y + w2 * r2_x - v1.y - w1 * r1_x;
    let pivot_force_x = -step.inv_dt * (j.pivot_mass.col1.x * pivot_cdot_x + j.pivot_mass.col2.x * pivot_cdot_y);
    let pivot_force_y = -step.inv_dt * (j.pivot_mass.col1.y * pivot_cdot_x + j.pivot_mass.col2.y * pivot_cdot_y);
    j.pivot_force.x = j.pivot_force.x + pivot_force_x;
    j.pivot_force.y = j.pivot_force.y + pivot_force_y;
    let p_x = step.dt * pivot_force_x;
    let p_y = step.dt * pivot_force_y;
    let inv_mass1 = ctx.world.body(b1).inv_mass;
    let inv_i1 = ctx.world.body(b1).inv_i;
    lv_sub(ctx, b1, inv_mass1 * p_x, inv_mass1 * p_y);
    av_sub(ctx, b1, inv_i1 * (r1_x * p_y - r1_y * p_x));
    let inv_mass2 = ctx.world.body(b2).inv_mass;
    let inv_i2 = ctx.world.body(b2).inv_i;
    lv_add(ctx, b2, inv_mass2 * p_x, inv_mass2 * p_y);
    av_add(ctx, b2, inv_i2 * (r2_x * p_y - r2_y * p_x));
    if j.enable_motor.get() && j.limit_state != E_EQUAL_LIMITS {
        let motor_cdot = av(ctx, b2) - av(ctx, b1) - j.motor_speed;
        let mut motor_force = -step.inv_dt * j.motor_mass * motor_cdot;
        let old_motor_force = j.motor_force;
        j.motor_force = b2_clamp(j.motor_force + motor_force, -j.max_motor_torque, j.max_motor_torque);
        motor_force = j.motor_force - old_motor_force;
        av_sub(ctx, b1, inv_i1 * step.dt * motor_force);
        av_add(ctx, b2, inv_i2 * step.dt * motor_force);
    }
    if j.enable_limit.get() && j.limit_state != E_INACTIVE_LIMIT {
        let limit_cdot = av(ctx, b2) - av(ctx, b1);
        let mut limit_force = -step.inv_dt * j.motor_mass * limit_cdot;
        if j.limit_state == E_EQUAL_LIMITS {
            j.limit_force = j.limit_force + limit_force;
        } else if j.limit_state == E_AT_LOWER_LIMIT {
            let old_limit_force = j.limit_force;
            j.limit_force = b2_max(j.limit_force + limit_force, R::zero());
            limit_force = j.limit_force - old_limit_force;
        } else if j.limit_state == E_AT_UPPER_LIMIT {
            let old_limit_force = j.limit_force;
            j.limit_force = b2_min(j.limit_force + limit_force, R::zero());
            limit_force = j.limit_force - old_limit_force;
        }
        av_sub(ctx, b1, inv_i1 * step.dt * limit_force);
        av_add(ctx, b2, inv_i2 * step.dt * limit_force);
    }
    *ctx.world.joint_mut(joint) = j;
}

fn revolute_solve_position_constraints<R: Real>(ctx: &mut Ctx<R>, joint: H) -> bool {
    let mut j = *ctx.world.joint(joint);
    let b1 = j.body1;
    let b2 = j.body2;
    // Solve point to point constraint.
    let (r1_x, r1_y) = anchor_r(ctx, b1, j.local_anchor1);
    let (r2_x, r2_y) = anchor_r(ctx, b2, j.local_anchor2);
    let p1_x = ctx.world.body(b1).sweep.c.x + r1_x;
    let p1_y = ctx.world.body(b1).sweep.c.y + r1_y;
    let p2_x = ctx.world.body(b2).sweep.c.x + r2_x;
    let p2_y = ctx.world.body(b2).sweep.c.y + r2_y;
    let ptp_c_x = p2_x - p1_x;
    let ptp_c_y = p2_y - p1_y;
    // AS3: `var positionError:Number = 0;` then assigned; the initial 0 is never read.
    let position_error = (ptp_c_x * ptp_c_x + ptp_c_y * ptp_c_y).sqrt();
    let inv_mass1 = ctx.world.body(b1).inv_mass;
    let inv_mass2 = ctx.world.body(b2).inv_mass;
    let inv_i1 = ctx.world.body(b1).inv_i;
    let inv_i2 = ctx.world.body(b2).inv_i;
    let mut k1 = Mat22::zero();
    let mut k2 = Mat22::zero();
    let mut k3 = Mat22::zero();
    let mut k = Mat22::zero();
    k1.col1.x = inv_mass1 + inv_mass2;
    k1.col2.x = R::zero();
    k1.col1.y = R::zero();
    k1.col2.y = inv_mass1 + inv_mass2;
    k2.col1.x = inv_i1 * r1_y * r1_y;
    k2.col2.x = -inv_i1 * r1_x * r1_y;
    k2.col1.y = -inv_i1 * r1_x * r1_y;
    k2.col2.y = inv_i1 * r1_x * r1_x;
    k3.col1.x = inv_i2 * r2_y * r2_y;
    k3.col2.x = -inv_i2 * r2_x * r2_y;
    k3.col1.y = -inv_i2 * r2_x * r2_y;
    k3.col2.y = inv_i2 * r2_x * r2_x;
    k.set_m(k1);
    k.add_m(k2);
    k.add_m(k3);
    // static shared b2RevoluteJoint.tImpulse -> local
    let t_impulse = k.solve(-ptp_c_x, -ptp_c_y);
    let impulse_x = t_impulse.x;
    let impulse_y = t_impulse.y;
    sc_sub(ctx, b1, inv_mass1 * impulse_x, inv_mass1 * impulse_y);
    sa_sub(ctx, b1, inv_i1 * (r1_x * impulse_y - r1_y * impulse_x));
    sc_add(ctx, b2, inv_mass2 * impulse_x, inv_mass2 * impulse_y);
    sa_add(ctx, b2, inv_i2 * (r2_x * impulse_y - r2_y * impulse_x));
    body::synchronize_transform(ctx, b1);
    body::synchronize_transform(ctx, b2);
    // Handle limits.
    let mut angular_error = R::zero();
    if j.enable_limit.get() && j.limit_state != E_INACTIVE_LIMIT {
        let angle = ctx.world.body(b2).sweep.a - ctx.world.body(b1).sweep.a - j.reference_angle;
        let mut limit_impulse = R::zero();
        if j.limit_state == E_EQUAL_LIMITS {
            // Prevent large angular corrections
            let limit_c = b2_clamp(angle, -b2_max_angular_correction::<R>(), b2_max_angular_correction::<R>());
            limit_impulse = -j.motor_mass * limit_c;
            angular_error = b2_abs(limit_c);
        } else if j.limit_state == E_AT_LOWER_LIMIT {
            let mut limit_c = angle - j.lower_angle;
            angular_error = b2_max(R::zero(), -limit_c);
            // Prevent large angular corrections and allow some slop.
            limit_c = b2_clamp(limit_c + b2_angular_slop::<R>(), -b2_max_angular_correction::<R>(), R::zero());
            limit_impulse = -j.motor_mass * limit_c;
            let old_limit_impulse = j.limit_position_impulse;
            j.limit_position_impulse = b2_max(j.limit_position_impulse + limit_impulse, R::zero());
            limit_impulse = j.limit_position_impulse - old_limit_impulse;
        } else if j.limit_state == E_AT_UPPER_LIMIT {
            let mut limit_c = angle - j.upper_angle;
            angular_error = b2_max(R::zero(), limit_c);
            // Prevent large angular corrections and allow some slop.
            limit_c = b2_clamp(limit_c - b2_angular_slop::<R>(), R::zero(), b2_max_angular_correction::<R>());
            limit_impulse = -j.motor_mass * limit_c;
            let old_limit_impulse = j.limit_position_impulse;
            j.limit_position_impulse = b2_min(j.limit_position_impulse + limit_impulse, R::zero());
            limit_impulse = j.limit_position_impulse - old_limit_impulse;
        }
        let inv_i1 = ctx.world.body(b1).inv_i;
        sa_sub(ctx, b1, inv_i1 * limit_impulse);
        let inv_i2 = ctx.world.body(b2).inv_i;
        sa_add(ctx, b2, inv_i2 * limit_impulse);
        body::synchronize_transform(ctx, b1);
        body::synchronize_transform(ctx, b2);
    }
    *ctx.world.joint_mut(joint) = j;
    position_error.le(b2_linear_slop::<R>()) && angular_error.le(b2_angular_slop::<R>())
}

// =============================================================================================
// b2PrismaticJoint
// =============================================================================================

/// `new b2PrismaticJoint(def)`
fn prismatic_construct<R: Real>(def: &JointDef<R>) -> Joint<R> {
    let mut j = base_construct(def);
    // m_localAnchor1/2, m_localXAxis1, m_localYAxis1 = new b2Vec2(); jacobians = new b2Jacobian()
    j.local_anchor1 = Vec2::zero();
    j.local_anchor2 = Vec2::zero();
    j.local_x_axis1 = Vec2::zero();
    j.local_y_axis1 = Vec2::zero();
    j.linear_jacobian = Jacobian::new();
    j.motor_jacobian = Jacobian::new();
    j.local_anchor1.set_v(def.local_anchor1);
    j.local_anchor2.set_v(def.local_anchor2);
    j.local_x_axis1.set_v(def.local_axis1);
    j.local_y_axis1.x = -j.local_x_axis1.y;
    j.local_y_axis1.y = j.local_x_axis1.x;
    j.ref_angle = def.reference_angle;
    j.linear_jacobian.set_zero();
    j.linear_mass = R::zero();
    j.force = R::zero();
    j.angular_mass = R::zero();
    j.torque = R::zero();
    j.motor_jacobian.set_zero();
    j.motor_mass = R::zero();
    j.motor_force = R::zero();
    j.limit_force = R::zero();
    j.limit_position_impulse = R::zero();
    j.lower_translation = def.lower_translation;
    j.upper_translation = def.upper_translation;
    j.max_motor_force = def.max_motor_force;
    j.motor_speed = def.motor_speed;
    j.enable_limit = def.enable_limit;
    j.enable_motor = def.enable_motor;
    j
}

/// b2PrismaticJointDef.Initialize(body1, body2, anchor, axis)
pub fn prismatic_def_initialize<R: Real>(ctx: &Ctx<R>, def: &mut JointDef<R>, body1: H, body2: H, anchor: Vec2<R>, axis: Vec2<R>) {
    def.body1 = body1;
    def.body2 = body2;
    def.local_anchor1 = body::get_local_point(ctx, def.body1, anchor);
    def.local_anchor2 = body::get_local_point(ctx, def.body2, anchor);
    def.local_axis1 = body::get_local_vector(ctx, def.body1, axis);
    def.reference_angle = body::get_angle(ctx, def.body2) - body::get_angle(ctx, def.body1);
}

/// b2PrismaticJoint.GetJointTranslation
pub fn get_joint_translation<R: Real>(ctx: &Ctx<R>, joint: H) -> R {
    let j = ctx.world.joint(joint);
    let b1 = j.body1;
    let b2 = j.body2;
    let p1 = body::get_world_point(ctx, b1, j.local_anchor1);
    let p2 = body::get_world_point(ctx, b2, j.local_anchor2);
    let d_x = p2.x - p1.x;
    let d_y = p2.y - p1.y;
    let axis = body::get_world_vector(ctx, b1, j.local_x_axis1);
    axis.x * d_x + axis.y * d_y
}

fn prismatic_get_joint_speed<R: Real>(ctx: &Ctx<R>, j: &Joint<R>) -> R {
    let b1 = j.body1;
    let b2 = j.body2;
    let (r1_x, r1_y) = anchor_r(ctx, b1, j.local_anchor1);
    let (r2_x, r2_y) = anchor_r(ctx, b2, j.local_anchor2);
    let p1_x = ctx.world.body(b1).sweep.c.x + r1_x;
    let p1_y = ctx.world.body(b1).sweep.c.y + r1_y;
    let p2_x = ctx.world.body(b2).sweep.c.x + r2_x;
    let p2_y = ctx.world.body(b2).sweep.c.y + r2_y;
    let d_x = p2_x - p1_x;
    let d_y = p2_y - p1_y;
    let axis = body::get_world_vector(ctx, b1, j.local_x_axis1);
    let v1 = lv(ctx, b1);
    let v2 = lv(ctx, b2);
    let w1 = av(ctx, b1);
    let w2 = av(ctx, b2);
    d_x * (-w1 * axis.y) + d_y * (w1 * axis.x) + (axis.x * (v2.x + -w2 * r2_y - v1.x - -w1 * r1_y) + axis.y * (v2.y + w2 * r2_x - v1.y - w1 * r1_x))
}

/// b2PrismaticJoint.GetReactionForce (stock quirk: the force scalars are applied twice).
fn prismatic_get_reaction_force<R: Real>(ctx: &Ctx<R>, j: &Joint<R>) -> Vec2<R> {
    let t_mat = ctx.world.body(j.body1).xf.r;
    let ax1_x = j.limit_force * (t_mat.col1.x * j.local_x_axis1.x + t_mat.col2.x * j.local_x_axis1.y);
    let ax1_y = j.limit_force * (t_mat.col1.y * j.local_x_axis1.x + t_mat.col2.y * j.local_x_axis1.y);
    let ay1_x = j.force * (t_mat.col1.x * j.local_y_axis1.x + t_mat.col2.x * j.local_y_axis1.y);
    let ay1_y = j.force * (t_mat.col1.y * j.local_y_axis1.x + t_mat.col2.y * j.local_y_axis1.y);
    Vec2::new(j.limit_force * ax1_x + j.force * ay1_x, j.limit_force * ax1_y + j.force * ay1_y)
}

fn prismatic_init_velocity_constraints<R: Real>(ctx: &mut Ctx<R>, joint: H, step: &TimeStep<R>) {
    let mut j = *ctx.world.joint(joint);
    let b1 = j.body1;
    let b2 = j.body2;
    // Compute the effective masses.
    let (r1_x, r1_y) = anchor_r(ctx, b1, j.local_anchor1);
    let (r2_x, r2_y) = anchor_r(ctx, b2, j.local_anchor2);
    let inv_mass1 = ctx.world.body(b1).inv_mass;
    let inv_mass2 = ctx.world.body(b2).inv_mass;
    let inv_i1 = ctx.world.body(b1).inv_i;
    let inv_i2 = ctx.world.body(b2).inv_i;
    // Compute point to line constraint effective mass.
    let (ay1_x, ay1_y) = rotate(ctx, b1, j.local_y_axis1);
    let e_x = ctx.world.body(b2).sweep.c.x + r2_x - ctx.world.body(b1).sweep.c.x;
    let e_y = ctx.world.body(b2).sweep.c.y + r2_y - ctx.world.body(b1).sweep.c.y;
    j.linear_jacobian.linear1.x = -ay1_x;
    j.linear_jacobian.linear1.y = -ay1_y;
    j.linear_jacobian.linear2.x = ay1_x;
    j.linear_jacobian.linear2.y = ay1_y;
    // The player's JIT reverses the subtraction under negate, including +0.
    j.linear_jacobian.angular1 = e_y * ay1_x - e_x * ay1_y;
    j.linear_jacobian.angular2 = r2_x * ay1_y - r2_y * ay1_x;
    j.linear_mass = inv_mass1 + inv_i1 * j.linear_jacobian.angular1 * j.linear_jacobian.angular1 + inv_mass2 + inv_i2 * j.linear_jacobian.angular2 * j.linear_jacobian.angular2;
    j.linear_mass = R::one() / j.linear_mass;
    // Compute angular constraint effective mass.
    j.angular_mass = inv_i1 + inv_i2;
    if j.angular_mass.gt(min_value::<R>()) {
        j.angular_mass = R::one() / j.angular_mass;
    }
    // Compute motor and limit terms.
    if j.enable_limit.get() || j.enable_motor.get() {
        // The motor and limit share a Jacobian and effective mass.
        let (ax1_x, ax1_y) = rotate(ctx, b1, j.local_x_axis1);
        j.motor_jacobian.linear1.x = -ax1_x;
        j.motor_jacobian.linear1.y = -ax1_y;
        j.motor_jacobian.linear2.x = ax1_x;
        j.motor_jacobian.linear2.y = ax1_y;
        j.motor_jacobian.angular1 = -(e_x * ax1_y - e_y * ax1_x);
        j.motor_jacobian.angular2 = r2_x * ax1_y - r2_y * ax1_x;
        j.motor_mass = inv_mass1 + inv_i1 * j.motor_jacobian.angular1 * j.motor_jacobian.angular1 + inv_mass2 + inv_i2 * j.motor_jacobian.angular2 * j.motor_jacobian.angular2;
        j.motor_mass = R::one() / j.motor_mass;
        if j.enable_limit.get() {
            let d_x = e_x - r1_x;
            let d_y = e_y - r1_y;
            let joint_translation = ax1_x * d_x + ax1_y * d_y;
            if b2_abs(j.upper_translation - j.lower_translation).lt(R::from_i32(2) * b2_linear_slop::<R>()) {
                j.limit_state = E_EQUAL_LIMITS;
            } else if joint_translation.le(j.lower_translation) {
                if j.limit_state != E_AT_LOWER_LIMIT {
                    j.limit_force = R::zero();
                }
                j.limit_state = E_AT_LOWER_LIMIT;
            } else if joint_translation.ge(j.upper_translation) {
                if j.limit_state != E_AT_UPPER_LIMIT {
                    j.limit_force = R::zero();
                }
                j.limit_state = E_AT_UPPER_LIMIT;
            } else {
                j.limit_state = E_INACTIVE_LIMIT;
                j.limit_force = R::zero();
            }
        }
    }
    if j.enable_motor == crate::Flag32::FALSE {
        j.motor_force = R::zero();
    }
    if j.enable_limit == crate::Flag32::FALSE {
        j.limit_force = R::zero();
    }
    if step.warm_starting.get() {
        // Force-based warm start: `dt * force` impulses along the jacobians.
        let p1_x = step.dt * (j.force * j.linear_jacobian.linear1.x + (j.motor_force + j.limit_force) * j.motor_jacobian.linear1.x);
        let p1_y = step.dt * (j.force * j.linear_jacobian.linear1.y + (j.motor_force + j.limit_force) * j.motor_jacobian.linear1.y);
        let p2_x = step.dt * (j.force * j.linear_jacobian.linear2.x + (j.motor_force + j.limit_force) * j.motor_jacobian.linear2.x);
        let p2_y = step.dt * (j.force * j.linear_jacobian.linear2.y + (j.motor_force + j.limit_force) * j.motor_jacobian.linear2.y);
        let l1 = step.dt * (j.force * j.linear_jacobian.angular1 - j.torque + (j.motor_force + j.limit_force) * j.motor_jacobian.angular1);
        let l2 = step.dt * (j.force * j.linear_jacobian.angular2 + j.torque + (j.motor_force + j.limit_force) * j.motor_jacobian.angular2);
        lv_add(ctx, b1, inv_mass1 * p1_x, inv_mass1 * p1_y);
        av_add(ctx, b1, inv_i1 * l1);
        lv_add(ctx, b2, inv_mass2 * p2_x, inv_mass2 * p2_y);
        av_add(ctx, b2, inv_i2 * l2);
    } else {
        j.force = R::zero();
        j.torque = R::zero();
        j.limit_force = R::zero();
        j.motor_force = R::zero();
    }
    j.limit_position_impulse = R::zero();
    *ctx.world.joint_mut(joint) = j;
}

fn prismatic_solve_velocity_constraints<R: Real>(ctx: &mut Ctx<R>, joint: H, step: &TimeStep<R>) {
    let mut j = *ctx.world.joint(joint);
    let b1 = j.body1;
    let b2 = j.body2;
    let inv_mass1 = ctx.world.body(b1).inv_mass;
    let inv_mass2 = ctx.world.body(b2).inv_mass;
    let inv_i1 = ctx.world.body(b1).inv_i;
    let inv_i2 = ctx.world.body(b2).inv_i;
    // Solve linear constraint.
    let linear_cdot = j.linear_jacobian.compute(lv(ctx, b1), av(ctx, b1), lv(ctx, b2), av(ctx, b2));
    let force = -step.inv_dt * j.linear_mass * linear_cdot;
    j.force = j.force + force;
    let mut p = step.dt * force;
    lv_add(ctx, b1, inv_mass1 * p * j.linear_jacobian.linear1.x, inv_mass1 * p * j.linear_jacobian.linear1.y);
    av_add(ctx, b1, inv_i1 * p * j.linear_jacobian.angular1);
    lv_add(ctx, b2, inv_mass2 * p * j.linear_jacobian.linear2.x, inv_mass2 * p * j.linear_jacobian.linear2.y);
    av_add(ctx, b2, inv_i2 * p * j.linear_jacobian.angular2);
    // Solve angular constraint.
    let angular_cdot = av(ctx, b2) - av(ctx, b1);
    let torque = -step.inv_dt * j.angular_mass * angular_cdot;
    j.torque = j.torque + torque;
    let l = step.dt * torque;
    av_sub(ctx, b1, inv_i1 * l);
    av_add(ctx, b2, inv_i2 * l);
    // Solve linear motor constraint.
    if j.enable_motor.get() && j.limit_state != E_EQUAL_LIMITS {
        let motor_cdot = j.motor_jacobian.compute(lv(ctx, b1), av(ctx, b1), lv(ctx, b2), av(ctx, b2)) - j.motor_speed;
        let mut motor_force = -step.inv_dt * j.motor_mass * motor_cdot;
        let old_motor_force = j.motor_force;
        j.motor_force = b2_clamp(j.motor_force + motor_force, -j.max_motor_force, j.max_motor_force);
        motor_force = j.motor_force - old_motor_force;
        p = step.dt * motor_force;
        lv_add(ctx, b1, inv_mass1 * p * j.motor_jacobian.linear1.x, inv_mass1 * p * j.motor_jacobian.linear1.y);
        av_add(ctx, b1, inv_i1 * p * j.motor_jacobian.angular1);
        lv_add(ctx, b2, inv_mass2 * p * j.motor_jacobian.linear2.x, inv_mass2 * p * j.motor_jacobian.linear2.y);
        av_add(ctx, b2, inv_i2 * p * j.motor_jacobian.angular2);
    }
    // Solve linear limit constraint.
    if j.enable_limit.get() && j.limit_state != E_INACTIVE_LIMIT {
        let limit_cdot = j.motor_jacobian.compute(lv(ctx, b1), av(ctx, b1), lv(ctx, b2), av(ctx, b2));
        let mut limit_force = -step.inv_dt * j.motor_mass * limit_cdot;
        if j.limit_state == E_EQUAL_LIMITS {
            j.limit_force = j.limit_force + limit_force;
        } else if j.limit_state == E_AT_LOWER_LIMIT {
            let old_limit_force = j.limit_force;
            j.limit_force = b2_max(j.limit_force + limit_force, R::zero());
            limit_force = j.limit_force - old_limit_force;
        } else if j.limit_state == E_AT_UPPER_LIMIT {
            let old_limit_force = j.limit_force;
            j.limit_force = b2_min(j.limit_force + limit_force, R::zero());
            limit_force = j.limit_force - old_limit_force;
        }
        p = step.dt * limit_force;
        lv_add(ctx, b1, inv_mass1 * p * j.motor_jacobian.linear1.x, inv_mass1 * p * j.motor_jacobian.linear1.y);
        av_add(ctx, b1, inv_i1 * p * j.motor_jacobian.angular1);
        lv_add(ctx, b2, inv_mass2 * p * j.motor_jacobian.linear2.x, inv_mass2 * p * j.motor_jacobian.linear2.y);
        av_add(ctx, b2, inv_i2 * p * j.motor_jacobian.angular2);
    }
    *ctx.world.joint_mut(joint) = j;
}

fn prismatic_solve_position_constraints<R: Real>(ctx: &mut Ctx<R>, joint: H) -> bool {
    let mut j = *ctx.world.joint(joint);
    let b1 = j.body1;
    let b2 = j.body2;
    let inv_mass1 = ctx.world.body(b1).inv_mass;
    let inv_mass2 = ctx.world.body(b2).inv_mass;
    let inv_i1 = ctx.world.body(b1).inv_i;
    let inv_i2 = ctx.world.body(b2).inv_i;
    let (r1_x, r1_y) = anchor_r(ctx, b1, j.local_anchor1);
    let (r2_x, r2_y) = anchor_r(ctx, b2, j.local_anchor2);
    let p1_x = ctx.world.body(b1).sweep.c.x + r1_x;
    let p1_y = ctx.world.body(b1).sweep.c.y + r1_y;
    let p2_x = ctx.world.body(b2).sweep.c.x + r2_x;
    let p2_y = ctx.world.body(b2).sweep.c.y + r2_y;
    let d_x = p2_x - p1_x;
    let d_y = p2_y - p1_y;
    let (ay1_x, ay1_y) = rotate(ctx, b1, j.local_y_axis1);
    // Solve linear (point-to-line) constraint.
    let mut linear_c = ay1_x * d_x + ay1_y * d_y;
    // Prevent overly large corrections.
    linear_c = b2_clamp(linear_c, -b2_max_linear_correction::<R>(), b2_max_linear_correction::<R>());
    let linear_impulse = -j.linear_mass * linear_c;
    sc_add(ctx, b1, inv_mass1 * linear_impulse * j.linear_jacobian.linear1.x, inv_mass1 * linear_impulse * j.linear_jacobian.linear1.y);
    sa_add(ctx, b1, inv_i1 * linear_impulse * j.linear_jacobian.angular1);
    sc_add(ctx, b2, inv_mass2 * linear_impulse * j.linear_jacobian.linear2.x, inv_mass2 * linear_impulse * j.linear_jacobian.linear2.y);
    sa_add(ctx, b2, inv_i2 * linear_impulse * j.linear_jacobian.angular2);
    let mut position_error = b2_abs(linear_c);
    // Solve angular constraint.
    let mut angular_c = ctx.world.body(b2).sweep.a - ctx.world.body(b1).sweep.a - j.ref_angle;
    // Prevent overly large corrections.
    angular_c = b2_clamp(angular_c, -b2_max_angular_correction::<R>(), b2_max_angular_correction::<R>());
    let angular_impulse = -j.angular_mass * angular_c;
    let b1_inv_i = ctx.world.body(b1).inv_i;
    sa_sub(ctx, b1, b1_inv_i * angular_impulse);
    let b2_inv_i = ctx.world.body(b2).inv_i;
    sa_add(ctx, b2, b2_inv_i * angular_impulse);
    body::synchronize_transform(ctx, b1);
    body::synchronize_transform(ctx, b2);
    let angular_error = b2_abs(angular_c);
    // Solve linear limit constraint.
    if j.enable_limit.get() && j.limit_state != E_INACTIVE_LIMIT {
        let (r1_x, r1_y) = anchor_r(ctx, b1, j.local_anchor1);
        let (r2_x, r2_y) = anchor_r(ctx, b2, j.local_anchor2);
        let p1_x = ctx.world.body(b1).sweep.c.x + r1_x;
        let p1_y = ctx.world.body(b1).sweep.c.y + r1_y;
        let p2_x = ctx.world.body(b2).sweep.c.x + r2_x;
        let p2_y = ctx.world.body(b2).sweep.c.y + r2_y;
        let d_x = p2_x - p1_x;
        let d_y = p2_y - p1_y;
        let (ax1_x, ax1_y) = rotate(ctx, b1, j.local_x_axis1);
        let translation = ax1_x * d_x + ax1_y * d_y;
        let mut limit_impulse = R::zero();
        if j.limit_state == E_EQUAL_LIMITS {
            // Prevent large angular corrections
            let limit_c = b2_clamp(translation, -b2_max_linear_correction::<R>(), b2_max_linear_correction::<R>());
            limit_impulse = -j.motor_mass * limit_c;
            // stock quirk: b2Abs(angularC), not b2Abs(limitC)
            position_error = b2_max(position_error, b2_abs(angular_c));
        } else if j.limit_state == E_AT_LOWER_LIMIT {
            let mut limit_c = translation - j.lower_translation;
            position_error = b2_max(position_error, -limit_c);
            // Prevent large linear corrections and allow some slop.
            limit_c = b2_clamp(limit_c + b2_linear_slop::<R>(), -b2_max_linear_correction::<R>(), R::zero());
            limit_impulse = -j.motor_mass * limit_c;
            let old_limit_impulse = j.limit_position_impulse;
            j.limit_position_impulse = b2_max(j.limit_position_impulse + limit_impulse, R::zero());
            limit_impulse = j.limit_position_impulse - old_limit_impulse;
        } else if j.limit_state == E_AT_UPPER_LIMIT {
            let mut limit_c = translation - j.upper_translation;
            position_error = b2_max(position_error, limit_c);
            // Prevent large linear corrections and allow some slop.
            limit_c = b2_clamp(limit_c - b2_linear_slop::<R>(), R::zero(), b2_max_linear_correction::<R>());
            limit_impulse = -j.motor_mass * limit_c;
            let old_limit_impulse = j.limit_position_impulse;
            j.limit_position_impulse = b2_min(j.limit_position_impulse + limit_impulse, R::zero());
            limit_impulse = j.limit_position_impulse - old_limit_impulse;
        }
        sc_add(ctx, b1, inv_mass1 * limit_impulse * j.motor_jacobian.linear1.x, inv_mass1 * limit_impulse * j.motor_jacobian.linear1.y);
        sa_add(ctx, b1, inv_i1 * limit_impulse * j.motor_jacobian.angular1);
        sc_add(ctx, b2, inv_mass2 * limit_impulse * j.motor_jacobian.linear2.x, inv_mass2 * limit_impulse * j.motor_jacobian.linear2.y);
        sa_add(ctx, b2, inv_i2 * limit_impulse * j.motor_jacobian.angular2);
        body::synchronize_transform(ctx, b1);
        body::synchronize_transform(ctx, b2);
    }
    *ctx.world.joint_mut(joint) = j;
    position_error.le(b2_linear_slop::<R>()) && angular_error.le(b2_angular_slop::<R>())
}

// =============================================================================================
// b2DistanceJoint
// =============================================================================================

/// `new b2DistanceJoint(def)`
fn distance_construct<R: Real>(def: &JointDef<R>) -> Joint<R> {
    let mut j = base_construct(def);
    j.local_anchor1 = Vec2::zero();
    j.local_anchor2 = Vec2::zero();
    j.u = Vec2::zero();
    j.local_anchor1.set_v(def.local_anchor1);
    j.local_anchor2.set_v(def.local_anchor2);
    j.length = def.length;
    j.frequency_hz = def.frequency_hz;
    j.damping_ratio = def.damping_ratio;
    j.impulse = R::zero();
    j.gamma = R::zero();
    j.bias = R::zero();
    j.inv_dt = R::zero();
    j
}

/// b2DistanceJointDef.Initialize(body1, body2, anchor1, anchor2)
pub fn distance_def_initialize<R: Real>(ctx: &Ctx<R>, def: &mut JointDef<R>, body1: H, body2: H, anchor1: Vec2<R>, anchor2: Vec2<R>) {
    def.body1 = body1;
    def.body2 = body2;
    def.local_anchor1.set_v(body::get_local_point(ctx, def.body1, anchor1));
    def.local_anchor2.set_v(body::get_local_point(ctx, def.body2, anchor2));
    let d_x = anchor2.x - anchor1.x;
    let d_y = anchor2.y - anchor1.y;
    def.length = (d_x * d_x + d_y * d_y).sqrt();
    def.frequency_hz = R::zero();
    def.damping_ratio = R::zero();
}

/// b2DistanceJoint.GetReactionForce: `m_u * (m_inv_dt * m_impulse)`
fn distance_get_reaction_force<R: Real>(j: &Joint<R>) -> Vec2<R> {
    let mut f = Vec2::zero();
    f.set_v(j.u);
    f.multiply(j.inv_dt * j.impulse);
    f
}

fn distance_init_velocity_constraints<R: Real>(ctx: &mut Ctx<R>, joint: H, step: &TimeStep<R>) {
    let mut j = *ctx.world.joint(joint);
    j.inv_dt = step.inv_dt;
    let b1 = j.body1;
    let b2 = j.body2;
    // Compute the effective mass matrix.
    let (r1_x, r1_y) = anchor_r(ctx, b1, j.local_anchor1);
    let (r2_x, r2_y) = anchor_r(ctx, b2, j.local_anchor2);
    j.u.x = ctx.world.body(b2).sweep.c.x + r2_x - ctx.world.body(b1).sweep.c.x - r1_x;
    j.u.y = ctx.world.body(b2).sweep.c.y + r2_y - ctx.world.body(b1).sweep.c.y - r1_y;
    // Handle singularity.
    let length = (j.u.x * j.u.x + j.u.y * j.u.y).sqrt();
    if length.gt(b2_linear_slop::<R>()) {
        j.u.multiply(R::one() / length);
    } else {
        j.u.set_zero();
    }
    let cr1u = r1_x * j.u.y - r1_y * j.u.x;
    let cr2u = r2_x * j.u.y - r2_y * j.u.x;
    let inv_mass = ctx.world.body(b1).inv_mass + ctx.world.body(b1).inv_i * cr1u * cr1u + ctx.world.body(b2).inv_mass + ctx.world.body(b2).inv_i * cr2u * cr2u;
    j.mass = R::one() / inv_mass;
    if j.frequency_hz.gt(R::zero()) {
        let c = length - j.length;
        // Frequency
        let omega = R::from_i32(2) * b2_pi::<R>() * j.frequency_hz;
        // Damping coefficient
        let d = R::from_i32(2) * j.mass * j.damping_ratio * omega;
        // Spring stiffness
        let k = j.mass * omega * omega;
        // magic formulas
        j.gamma = R::one() / (step.dt * (d + step.dt * k));
        j.bias = c * step.dt * k * j.gamma;
        j.mass = R::one() / (inv_mass + j.gamma);
    }
    if step.warm_starting.get() {
        // The distance joint is impulse-based in 2.0.2: warm start scaled by dtRatio.
        j.impulse = j.impulse * step.dt_ratio;
        let p_x = j.impulse * j.u.x;
        let p_y = j.impulse * j.u.y;
        let inv_mass1 = ctx.world.body(b1).inv_mass;
        lv_sub(ctx, b1, inv_mass1 * p_x, inv_mass1 * p_y);
        let inv_i1 = ctx.world.body(b1).inv_i;
        av_sub(ctx, b1, inv_i1 * (r1_x * p_y - r1_y * p_x));
        let inv_mass2 = ctx.world.body(b2).inv_mass;
        lv_add(ctx, b2, inv_mass2 * p_x, inv_mass2 * p_y);
        let inv_i2 = ctx.world.body(b2).inv_i;
        av_add(ctx, b2, inv_i2 * (r2_x * p_y - r2_y * p_x));
    } else {
        j.impulse = R::zero();
    }
    *ctx.world.joint_mut(joint) = j;
}

fn distance_solve_velocity_constraints<R: Real>(ctx: &mut Ctx<R>, joint: H, _step: &TimeStep<R>) {
    let mut j = *ctx.world.joint(joint);
    let b1 = j.body1;
    let b2 = j.body2;
    let (r1_x, r1_y) = anchor_r(ctx, b1, j.local_anchor1);
    let (r2_x, r2_y) = anchor_r(ctx, b2, j.local_anchor2);
    // Cdot = dot(u, v + cross(w, r))
    let v1 = lv(ctx, b1);
    let w1 = av(ctx, b1);
    let v2 = lv(ctx, b2);
    let w2 = av(ctx, b2);
    let v1_x = v1.x + -w1 * r1_y;
    let v1_y = v1.y + w1 * r1_x;
    let v2_x = v2.x + -w2 * r2_y;
    let v2_y = v2.y + w2 * r2_x;
    let cdot = j.u.x * (v2_x - v1_x) + j.u.y * (v2_y - v1_y);
    let impulse = -j.mass * (cdot + j.bias + j.gamma * j.impulse);
    j.impulse = j.impulse + impulse;
    let p_x = impulse * j.u.x;
    let p_y = impulse * j.u.y;
    let inv_mass1 = ctx.world.body(b1).inv_mass;
    lv_sub(ctx, b1, inv_mass1 * p_x, inv_mass1 * p_y);
    let inv_i1 = ctx.world.body(b1).inv_i;
    av_sub(ctx, b1, inv_i1 * (r1_x * p_y - r1_y * p_x));
    let inv_mass2 = ctx.world.body(b2).inv_mass;
    lv_add(ctx, b2, inv_mass2 * p_x, inv_mass2 * p_y);
    let inv_i2 = ctx.world.body(b2).inv_i;
    av_add(ctx, b2, inv_i2 * (r2_x * p_y - r2_y * p_x));
    *ctx.world.joint_mut(joint) = j;
}

fn distance_solve_position_constraints<R: Real>(ctx: &mut Ctx<R>, joint: H) -> bool {
    let mut j = *ctx.world.joint(joint);
    if j.frequency_hz.gt(R::zero()) {
        return true;
    }
    let b1 = j.body1;
    let b2 = j.body2;
    let (r1_x, r1_y) = anchor_r(ctx, b1, j.local_anchor1);
    let (r2_x, r2_y) = anchor_r(ctx, b2, j.local_anchor2);
    let mut d_x = ctx.world.body(b2).sweep.c.x + r2_x - ctx.world.body(b1).sweep.c.x - r1_x;
    let mut d_y = ctx.world.body(b2).sweep.c.y + r2_y - ctx.world.body(b1).sweep.c.y - r1_y;
    let length = (d_x * d_x + d_y * d_y).sqrt();
    d_x = d_x / length;
    d_y = d_y / length;
    let mut c = length - j.length;
    c = b2_clamp(c, -b2_max_linear_correction::<R>(), b2_max_linear_correction::<R>());
    let impulse = -j.mass * c;
    j.u.set(d_x, d_y);
    let p_x = impulse * j.u.x;
    let p_y = impulse * j.u.y;
    let inv_mass1 = ctx.world.body(b1).inv_mass;
    sc_sub(ctx, b1, inv_mass1 * p_x, inv_mass1 * p_y);
    let inv_i1 = ctx.world.body(b1).inv_i;
    sa_sub(ctx, b1, inv_i1 * (r1_x * p_y - r1_y * p_x));
    let inv_mass2 = ctx.world.body(b2).inv_mass;
    sc_add(ctx, b2, inv_mass2 * p_x, inv_mass2 * p_y);
    let inv_i2 = ctx.world.body(b2).inv_i;
    sa_add(ctx, b2, inv_i2 * (r2_x * p_y - r2_y * p_x));
    body::synchronize_transform(ctx, b1);
    body::synchronize_transform(ctx, b2);
    *ctx.world.joint_mut(joint) = j;
    b2_abs(c).lt(b2_linear_slop::<R>())
}
