//! Character contact dictionaries and damage transitions. Buffered normals are NOT copied:
//! consumers read WorldState.report_cr_normal, the alias held by Flash ContactEvent.
use super::{character::*, composites, poses, wheelchair};
use crate::b2::{
    body, joints,
    math::{atan2, sin_cos, Vec2},
    state::{BodyDef, JointDef, ShapeDef, MAX_SHAPES},
    world,
};
use crate::{flow, flow::AbortSite, Ctx, Flow, H, NONE};
use hw_data::{GuideKey, GuideLookup};
use hw_real::Real;
pub const CONTACT_KEYS: usize = MAX_SHAPES + 1;
pub fn key(shape: H) -> usize {
    if shape == NONE {
        MAX_SHAPES
    } else {
        shape as usize
    }
}
pub fn undefined<R: Real>() -> R {
    R::from_words([0x7fffffff, 0xe0000000])
}
#[derive(Clone, Copy)]
#[repr(C)]
pub struct BufferedResult<R: Real> {
    pub present: crate::Flag32,
    pub shape: H,
    pub other: H,
    pub impulse: R,
}
impl<R: Real> BufferedResult<R> {
    pub fn blank() -> Self {
        Self {
            present: crate::Flag32::FALSE,
            shape: NONE,
            other: NONE,
            impulse: R::zero(),
        }
    }
}

#[cfg(feature = "lab")]
fn lab_primary_chest<R: Real>(ctx: &Ctx<R>) -> Option<H> {
    if !crate::lab::armed() || ctx.character.child_active.get() { return None; }
    let body = ctx.character.primary.bodies[CHEST];
    if body == NONE || body as usize >= crate::b2::state::MAX_BODIES { return None; }
    crate::lab::watched(body).then_some(body)
}

#[cfg(feature = "lab")]
fn lab_readable<T: Copy, const N: usize>(arena: &crate::arena::Arena<T, N>, h: H) -> bool {
    if h == NONE || h as usize >= N { return false; }
    arena.alive[h as usize] == 1 || arena.retained[h as usize] != 0
        || arena.pending[..(arena.pending_len as usize).min(N)].contains(&h)
}

#[cfg(feature = "lab")]
fn lab_shape_body<R: Real>(ctx: &Ctx<R>, shape: H) -> Option<H> {
    if !lab_readable(&ctx.world.shapes, shape) { return None; }
    let body = ctx.world.shapes.items[shape as usize].body;
    lab_readable(&ctx.world.bodies, body).then_some(body)
}

#[cfg(feature = "lab")]
fn lab_buffer<R: Real>(ctx: &Ctx<R>, r: BufferedResult<R>) -> crate::lab::DamageBuffer {
    crate::lab::DamageBuffer {
        present: r.present.get(), shape: r.shape, other: r.other,
        impulse: crate::lab::f(r.impulse),
        own_body: lab_shape_body(ctx, r.shape), other_body: lab_shape_body(ctx, r.other),
    }
}

pub fn result<R: Real>(ctx: &mut Ctx<R>, own: H, other: H, impulse: R, tag: u32) -> Flow {
    let shape = if tag == (CHARACTER_HANDLER | 32) {
        ctx.character.wheelchair.shapes[0]
    } else if tag == (CHARACTER_HANDLER | 50) {
        ctx.character.bicycle.shapes[0]
    } else if tag == (CHARACTER_HANDLER | 70) {
        ctx.character.motorcart.shapes[0]
    } else {
        own
    };
    let k = key(shape);
    lab! {
        if let Some(body) = lab_primary_chest(ctx) {
            if shape == ctx.character.primary.shapes[CHEST] && k < CONTACT_KEYS {
                crate::lab::record(crate::lab::Event::DamageResult {
                    phase: crate::lab::current_report_phase(), body,
                    actor_kind: ctx.character.primary.kind, child: ctx.character.child_active.get(),
                    dead: ctx.character.primary.dead.get(), shape, own, other,
                    own_body: lab_shape_body(ctx, own), other_body: lab_shape_body(ctx, other),
                    impulse: crate::lab::f(impulse),
                    threshold: crate::lab::f(ctx.character.contact_thresholds[k]),
                    previous: lab_buffer(ctx, ctx.character.contact_results[k]),
                });
            }
        }
    }
    if impulse.gt(ctx.character.contact_thresholds[k]) {
        let old = ctx.character.contact_results[k];
        if !old.present || impulse.gt(old.impulse) {
            ctx.character.contact_results[k] = BufferedResult {
                present: crate::Flag32::TRUE,
                shape: own,
                other,
                impulse,
            };
        }
    }
    Flow::Continue
}
pub fn disarm<R: Real>(ctx: &mut Ctx<R>, shape: H) {
    ctx.character.contact_thresholds[key(shape)] = undefined();
    ctx.world.shape_mut(shape).handlers[3] = 0;
    ctx.world.shape_mut(shape).handlers[0] = 0;
}
pub fn set_dead<R: Real>(ctx: &mut Ctx<R>, dead: bool) -> Flow {
    if ctx.character.kind == 5 && ctx.character.dead == crate::Flag32::new(dead) {
        return Flow::Continue;
    }
    ctx.character.dead = crate::Flag32::new(dead);
    if !ctx.character.child_active {
        ctx.level.dead = crate::Flag32::new(dead);
    }
    if dead {
        poses::set_pose(ctx, 0);
        flow!(poses::release_grip(ctx));
        if ctx.character.kind == 1 {
            joints::enable_motor(ctx, ctx.character.wheelchair.joints[0], false);
        } else if ctx.character.kind == 2 {
            joints::enable_motor(
                ctx,
                ctx.character.segway.joints[super::segway::WHEEL_J],
                false,
            );
        } else if ctx.character.kind == 3 {
            joints::enable_motor(
                ctx,
                ctx.character.bicycle.joints[super::bicycle::BACK_J],
                false,
            );
            joints::enable_motor(
                ctx,
                ctx.character.bicycle.joints[super::bicycle::FRONT_J],
                false,
            );
        } else if ctx.character.kind == 4 {
            joints::enable_motor(
                ctx,
                ctx.character.motorcart.joints[super::motorcart::BACK_J],
                false,
            );
            joints::enable_motor(
                ctx,
                ctx.character.motorcart.joints[super::motorcart::FRONT_J],
                false,
            );
        } else if ctx.character.kind != 0 && ctx.character.kind != 5 {
            return Flow::Abort(AbortSite::PortUnimplemented);
        }
    }
    Flow::Continue
}
pub fn eject<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    match ctx.character.kind {
        0 => Flow::Continue,
        1 => wheelchair::eject(ctx),
        2 => super::segway::eject(ctx),
        3 => super::bicycle::eject(ctx),
        4 => super::motorcart::eject(ctx),
        5 => super::child::eject(ctx),
        _ => Flow::Abort(AbortSite::PortUnimplemented),
    }
}
macro_rules! new_body {
    ($ctx:expr,$d:expr) => {{
        let b = world::create_body($ctx, $d);
        if b == NONE {
            return Flow::Abort(if ($ctx.world.lock).get() {
                AbortSite::GameNullDeref
            } else {
                AbortSite::PortCapacity
            });
        }
        b
    }};
}

pub fn neck_break<R: Real>(ctx: &mut Ctx<R>, impulse: R) -> Flow {
    flow!(set_dead(ctx, true));
    let neck = ctx.character.joints[NECK];
    ctx.world.joint_mut(neck).broken = crate::Flag32::TRUE;
    flow!(world::destroy_joint(ctx, neck));
    let head = ctx.character.bodies[HEAD];
    let sh = body::get_shape_list(ctx, head);
    ctx.world.shape_mut(sh).filter = ctx.character.zero_filter;
    flow!(world::refilter(ctx, sh));
    ctx.character.head_blood_flow = crate::Flag32::TRUE;
    let limit = R::from_i32(ctx.character.break_limits[1]);
    if impulse.lt(limit) {
        let width =
            R::from_i32(ctx.character.break_limits[1].wrapping_sub(ctx.character.break_limits[0]));
        let excess = impulse - R::from_i32(ctx.character.break_limits[0]);
        let step = width / hw_real::rl!(R; 9.0);
        let count = (excess / step).ceil() + R::one();
        let Some(data) = super::character::data(ctx) else {
            return Flow::Abort(AbortSite::PortUnimplemented);
        };
        let mut out = NONE;
        flow!(composites::spine(
            ctx,
            &data,
            count.to_int32(),
            ctx.character.ligament_length,
            &mut out
        ));
        ctx.character.spine = out;
    }
    if matches!(ctx.character.kind, 2 | 3 | 4) {
        flow!(eject(ctx));
    }
    Flow::Continue
}
pub fn torso_break<R: Real>(ctx: &mut Ctx<R>, impulse: R) -> Flow {
    let waist = ctx.character.joints[WAIST];
    ctx.world.joint_mut(waist).broken = crate::Flag32::TRUE;
    ctx.character.dying = crate::Flag32::TRUE;
    flow!(world::destroy_joint(ctx, waist));
    let chest = body::get_position(ctx, ctx.character.bodies[CHEST]);
    let pelvis = body::get_position(ctx, ctx.character.bodies[PELVIS]);
    let angle = atan2(chest.y - pelvis.y, chest.x - pelvis.x);
    let _unused_x = sin_cos(angle).0 * hw_real::rl!(R; 10.0) / ctx.character.scale;
    let _unused_y = sin_cos(angle).1 * hw_real::rl!(R; 10.0) / ctx.character.scale;
    let items = [PELVIS, UPPER_LEG1, UPPER_LEG2, LOWER_LEG1, LOWER_LEG2];
    for item_index in 0..items.len() {
        let part = items[item_index];
        let sh = body::get_shape_list(ctx, ctx.character.bodies[part]);
        ctx.world.shape_mut(sh).filter = ctx.character.lower_body_filter;
    }
    let items = [PELVIS, UPPER_LEG1, UPPER_LEG2, LOWER_LEG1, LOWER_LEG2];
    for item_index in 0..items.len() {
        let part = items[item_index];
        flow!(world::refilter(
            ctx,
            body::get_shape_list(ctx, ctx.character.bodies[part])
        ));
    }
    let limit = R::from_i32(ctx.character.break_limits[3]);
    if impulse.lt(limit) {
        let width =
            R::from_i32(ctx.character.break_limits[3].wrapping_sub(ctx.character.break_limits[2]));
        let excess = impulse - R::from_i32(ctx.character.break_limits[2]);
        let step = width / hw_real::rl!(R; 8.0);
        let count = (excess / step).ceil() + R::one();
        let mut out = NONE;
        flow!(composites::intestine(
            ctx,
            count.to_int32(),
            ctx.character.intestine_length,
            &mut out
        ));
        ctx.character.intestine = out;
    }
    if matches!(ctx.character.kind, 1 | 2 | 3 | 4 | 5) {
        flow!(eject(ctx));
    }
    Flow::Continue
}
pub fn head_smash<R: Real>(ctx: &mut Ctx<R>, _impulse: R) -> Flow {
    disarm(ctx, ctx.character.head1_shape);
    flow!(set_dead(ctx, true));
    let neck = ctx.character.joints[NECK];
    if !ctx.world.joint(neck).broken {
        ctx.world.joint_mut(neck).broken = crate::Flag32::TRUE;
    } else {
        if !ctx.character.head_blood_flow {
            return Flow::Abort(AbortSite::GameNullDeref);
        }
        if ctx.character.spine != NONE {
            flow!(composites::break_end(ctx, ctx.character.spine, 1));
        }
    }
    let head = ctx.character.bodies[HEAD];
    let scale = ctx.character.scale;
    let mut circle = ShapeDef::circle();
    circle.density = R::one();
    circle.friction = hw_real::rl!(R; 0.3);
    circle.restitution = hw_real::rl!(R; 0.1);
    circle.radius = ctx.character.chunk_radius[0] / scale;
    circle.filter = ctx.character.zero_filter;
    let p = body::get_position(ctx, head);
    let mut angle = body::get_angle(ctx, head);
    let radius = hw_real::rl!(R; 4.0) / scale;
    for _ in 0..4 {
        let mut bd = BodyDef::new();
        bd.position = Vec2::new(
            p.x + sin_cos(angle).1 * radius,
            p.y + sin_cos(angle).0 * radius,
        );
        let b = new_body!(ctx, &bd);
        flow!(body::create_shape(ctx, b, &circle).0);
        flow!(body::set_mass_from_shapes(ctx, b));
        body::set_linear_velocity(
            ctx,
            b,
            body::get_linear_velocity_from_world_point(ctx, head, bd.position),
        );
        body::set_angular_velocity(ctx, b, body::get_angular_velocity(ctx, head));
        angle = angle + hw_real::rl!(R; core::f64::consts::PI) / hw_real::rl!(R; 2.0);
    }
    circle.radius = hw_real::rl!(R; 12.0) / scale;
    let mut bd = BodyDef::new();
    bd.position = p;
    bd.angle = body::get_angle(ctx, head);
    let b = new_body!(ctx, &bd);
    ctx.character.brain = b;
    ctx.world.bodies.retain(b);
    flow!(body::create_shape(ctx, b, &circle).0);
    flow!(body::set_mass_from_shapes(ctx, b));
    body::set_linear_velocity(ctx, b, body::get_linear_velocity(ctx, head));
    body::set_angular_velocity(ctx, b, body::get_angular_velocity(ctx, head));
    flow!(world::destroy_body(ctx, head));
    if matches!(ctx.character.kind, 2 | 3 | 4) {
        flow!(eject(ctx));
    }
    Flow::Continue
}
pub fn foot_smash<R: Real>(ctx: &mut Ctx<R>, part: usize, _impulse: R) -> Flow {
    disarm(ctx, ctx.character.shapes[part]);
    let leg = ctx.character.bodies[part];
    let Some(data) = super::character::data(ctx) else {
        return Flow::Abort(AbortSite::PortUnimplemented);
    };
    let Some(p) = data.guide_numbers(GuideKey::FootShape) else {
        return Flow::Abort(AbortSite::PortUnimplemented);
    };
    let scale = ctx.character.scale;
    let mut polygon = ShapeDef::polygon();
    let mut bd = BodyDef::new();
    bd.position = body::get_world_point(
        ctx,
        leg,
        Vec2::new(R::from_words(p.x) / scale, R::from_words(p.y) / scale),
    );
    bd.angle = body::get_angle(ctx, leg);
    polygon.density = R::one();
    polygon.friction = hw_real::rl!(R; 0.3);
    polygon.restitution = hw_real::rl!(R; 0.1);
    polygon.filter = ctx.character.zero_filter;
    polygon.set_as_box(
        R::from_words(p.scale_x) * hw_real::rl!(R; 5.0) / scale,
        R::from_words(p.scale_y) * hw_real::rl!(R; 5.0) / scale,
    );
    let b = new_body!(ctx, &bd);
    flow!(body::create_shape(ctx, b, &polygon).0);
    flow!(body::set_mass_from_shapes(ctx, b));
    body::set_linear_velocity(ctx, b, body::get_linear_velocity(ctx, leg));
    body::set_angular_velocity(ctx, b, body::get_angular_velocity(ctx, leg));
    Flow::Continue
}
pub fn grab<R: Real>(ctx: &mut Ctx<R>, arm: H, other: H) -> Flow {
    if other == NONE {
        return Flow::Abort(AbortSite::GameNullDeref);
    }
    let shape = body::get_shape_list(ctx, arm);
    let p = body::get_world_point(
        ctx,
        arm,
        Vec2::new(R::zero(), ctx.world.shape(shape).vertices[2].y),
    );
    let mut d = JointDef::revolute();
    if !body::is_static(ctx, other) {
        d.enable_limit = crate::Flag32::TRUE;
    }
    d.max_motor_torque = hw_real::rl!(R; 4.0);
    joints::revolute_def_initialize(ctx, &mut d, other, arm, p);
    let items = [LOWER_ARM1, LOWER_ARM2];
    for i in 0..items.len() {
        let part = items[i];
        if arm == ctx.character.bodies[part] {
            let (f, j) = world::create_joint(ctx, &d);
            flow!(f);
            ctx.world.joints.release_root(ctx.character.grip_joints[i]);
            ctx.character.grip_joints[i] = j;
            ctx.world.joints.retain(j);
        }
    }
    Flow::Continue
}

/// Elbow/knee transitions share arithmetic and mutation order in the original.
pub fn distal_break<R: Real>(ctx: &mut Ctx<R>, joint: usize, impulse: R) -> Flow {
    let (upper, lower, limit, knee) = match joint {
        ELBOW1 => (UPPER_ARM1, LOWER_ARM1, 9, false),
        ELBOW2 => (UPPER_ARM2, LOWER_ARM2, 9, false),
        KNEE1 => (UPPER_LEG1, LOWER_LEG1, 11, true),
        KNEE2 => (UPPER_LEG2, LOWER_LEG2, 11, true),
        _ => return Flow::Abort(AbortSite::PortUnimplemented),
    };
    let j = ctx.character.joints[joint];
    ctx.world.joint_mut(j).broken = crate::Flag32::TRUE;
    flow!(world::destroy_joint(ctx, j));
    let upper = ctx.character.bodies[upper];
    let lower = ctx.character.bodies[lower];
    let sh = body::get_shape_list(ctx, lower);
    ctx.world.shape_mut(sh).filter = ctx.character.zero_filter;
    if knee {
        ctx.world.shape_mut(sh).is_sensor = crate::Flag32::FALSE;
    }
    flow!(world::refilter(ctx, sh));
    if impulse.lt(R::from_i32(ctx.character.break_limits[limit])) {
        let mut out = NONE;
        flow!(composites::ligament(
            ctx,
            upper,
            lower,
            ctx.character.ligament_length,
            ctx.character.scale,
            &mut out
        ));
    }
    if ctx.character.kind == 2 {
        flow!(super::segway::after_distal_break(ctx, joint));
    }
    if ctx.character.kind == 3 {
        flow!(super::bicycle_damage::after_distal_break(ctx, joint));
    }
    if ctx.character.kind == 4 {
        flow!(super::motorcart_damage::after_distal_break(ctx, joint));
    }
    Flow::Continue
}

pub fn proximal_break<R: Real>(ctx: &mut Ctx<R>, joint: usize, impulse: R) -> Flow {
    let (upper, lower, parent, limit, leg) = match joint {
        SHOULDER1 => (UPPER_ARM1, LOWER_ARM1, CHEST, 5, false),
        SHOULDER2 => (UPPER_ARM2, LOWER_ARM2, CHEST, 5, false),
        HIP1 => (UPPER_LEG1, LOWER_LEG1, PELVIS, 7, true),
        HIP2 => (UPPER_LEG2, LOWER_LEG2, PELVIS, 7, true),
        _ => return Flow::Abort(AbortSite::PortUnimplemented),
    };
    let j = ctx.character.joints[joint];
    ctx.world.joint_mut(j).broken = crate::Flag32::TRUE;
    let upper = ctx.character.bodies[upper];
    let lower = ctx.character.bodies[lower];
    let parent = ctx.character.bodies[parent];
    if impulse.gt(R::from_i32(ctx.character.break_limits[limit])) {
        flow!(world::destroy_joint(ctx, j));
        let a = body::get_shape_list(ctx, upper);
        let b = body::get_shape_list(ctx, lower);
        ctx.world.shape_mut(a).filter = ctx.character.zero_filter;
        ctx.world.shape_mut(a).material = 1;
        if leg {
            ctx.world.shape_mut(a).is_sensor = crate::Flag32::FALSE;
        }
        ctx.world.shape_mut(b).filter = ctx.character.zero_filter;
        flow!(world::refilter(ctx, a));
        flow!(world::refilter(ctx, b));
    } else {
        let Some(data) = super::character::data(ctx) else {
            return Flow::Abort(AbortSite::PortUnimplemented);
        };
        let Some(p) = data.guide_numbers(if leg { GuideKey::SevLeg1Shape } else { GuideKey::SevArm1Shape }) else {
            return Flow::Abort(AbortSite::PortUnimplemented);
        };
        let anchor = joints::get_anchor1(ctx, j);
        let scale = ctx.character.scale;
        let mut bd = BodyDef::new();
        bd.position = anchor;
        bd.angle = body::get_angle(ctx, upper);
        let mut polygon = ShapeDef::polygon();
        polygon.density = R::one();
        polygon.friction = hw_real::rl!(R; 0.3);
        polygon.restitution = hw_real::rl!(R; 0.1);
        polygon.filter = ctx.character.default_filter;
        polygon.set_as_oriented_box(
            R::from_words(p.scale_x) * hw_real::rl!(R; 5.0) / scale,
            R::from_words(p.scale_y) * hw_real::rl!(R; 5.0) / scale,
            Some(Vec2::new(R::zero(), R::from_words(p.y) / scale)),
            R::zero(),
        );
        let stump = new_body!(ctx, &bd);
        ctx.character.stumps[match joint {
            SHOULDER1 => 0,
            SHOULDER2 => 1,
            HIP1 => 2,
            _ => 3,
        }] = stump;
        ctx.world.bodies.retain(stump);
        flow!(body::create_shape(ctx, stump, &polygon).0);
        flow!(body::set_mass_from_shapes(ctx, stump));
        body::set_linear_velocity(ctx, stump, body::get_linear_velocity(ctx, upper));
        body::set_angular_velocity(ctx, stump, body::get_angular_velocity(ctx, upper));
        flow!(world::destroy_joint(ctx, j));
        ctx.character.joints[joint] = NONE;
        let mut d = JointDef::revolute();
        d.enable_limit = crate::Flag32::TRUE;
        d.max_motor_torque = hw_real::rl!(R; 4.0);
        let diff = body::get_angle(ctx, stump) - body::get_angle(ctx, parent);
        d.lower_angle = (if leg { -hw_real::rl!(R; 150.0) } else { -hw_real::rl!(R; 180.0) })
            / (hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI))
            - diff;
        d.upper_angle = (if leg { hw_real::rl!(R; 10.0) } else { hw_real::rl!(R; 60.0) })
            / (hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI))
            - diff;
        joints::revolute_def_initialize(ctx, &mut d, parent, stump, anchor);
        let (f, new_joint) = world::create_joint(ctx, &d);
        flow!(f);
        ctx.character.joints[joint] = new_joint;
        ctx.world.joints.retain(new_joint);
        ctx.world.joint_mut(new_joint).broken = crate::Flag32::TRUE;
        if leg {
            let Some(p) = data.guide_numbers(GuideKey::SevLeg2Shape) else {
                return Flow::Abort(AbortSite::PortUnimplemented);
            };
            flow!(body::destroy_shape(
                ctx,
                upper,
                body::get_shape_list(ctx, upper)
            ));
            polygon.filter = ctx.character.zero_filter;
            polygon.user_data = 1;
            polygon.set_as_oriented_box(
                R::from_words(p.scale_x) * hw_real::rl!(R; 5.0) / scale,
                R::from_words(p.scale_y) * hw_real::rl!(R; 5.0) / scale,
                Some(Vec2::new(R::zero(), R::from_words(p.y) / scale)),
                R::zero(),
            );
            flow!(body::create_shape(ctx, upper, &polygon).0);
            flow!(body::set_mass_from_shapes(ctx, upper));
            let sh = body::get_shape_list(ctx, lower);
            ctx.world.shape_mut(sh).filter = ctx.character.zero_filter;
            flow!(world::refilter(ctx, sh));
        } else {
            let a = body::get_shape_list(ctx, upper);
            let b = body::get_shape_list(ctx, lower);
            ctx.world.shape_mut(a).filter = ctx.character.zero_filter;
            ctx.world.shape_mut(a).material = 1;
            ctx.world.shape_mut(b).filter = ctx.character.zero_filter;
            flow!(world::refilter(ctx, a));
            flow!(world::refilter(ctx, b));
        }
    }
    if leg && ctx.character.kind == 5 {
        let i = if joint == HIP1 {
            super::child::SEAT_LEG1
        } else {
            super::child::SEAT_LEG2
        };
        let j = ctx.character.child_seat.joints[i];
        if j != NONE {
            flow!(world::destroy_joint(ctx, j));
            ctx.character.child_seat.joints[i] = NONE;
        }
    }
    if leg && ctx.character.kind == 1 {
        let idx = if joint == HIP1 { 4 } else { 5 };
        let j = ctx.character.wheelchair.joints[idx];
        if j != NONE {
            flow!(world::destroy_joint(ctx, j));
            ctx.character.wheelchair.joints[idx] = NONE;
        }
    }
    Flow::Continue
}

pub fn check_joints<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    let items = [
        (WAIST, 2),
        (NECK, 0),
        (SHOULDER1, 4),
        (SHOULDER2, 4),
        (ELBOW1, 8),
        (ELBOW2, 8),
        (HIP1, 6),
        (HIP2, 6),
        (KNEE1, 10),
        (KNEE2, 10),
    ];
    for item_index in 0..items.len() {
        let (index, limit) = items[item_index];
        let j = ctx.character.joints[index];
        let Some(impulse) = break_impulse(ctx, j, R::from_i32(ctx.character.break_limits[limit]))
        else {
            continue;
        };
        flow!(match index {
            WAIST => torso_break(ctx, impulse),
            NECK => neck_break(ctx, impulse),
            SHOULDER1 | SHOULDER2 | HIP1 | HIP2 => proximal_break(ctx, index, impulse),
            _ => distal_break(ctx, index, impulse),
        });
    }
    flow!(composites::check_joints(ctx));
    if ctx.character.kind == 5 {
        let j = ctx.character.child_seat.joints[super::child::SEAT_FRAME];
        if let Some(impulse) = break_impulse(ctx, j, hw_real::rl!(R; 200.0)) {
            flow!(super::child::detach(ctx, impulse));
        }
    }
    Flow::Continue
}
pub fn break_impulse<R: Real>(ctx: &Ctx<R>, j: H, limit: R) -> Option<R> {
    if j == NONE || ctx.world.joint(j).broken.get() {
        return None;
    }
    let force = joints::get_reaction_force(ctx, j);
    let magnitude = (force.x * force.x + force.y * force.y).sqrt();
    if magnitude.gt(limit) {
        return Some(magnitude);
    }
    let a = joints::get_anchor1(ctx, j);
    let b = joints::get_anchor2(ctx, j);
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    if (dx * dx + dy * dy).sqrt().gt(hw_real::rl!(R; 0.5)) {
        Some(hw_real::rl!(R; 1000.0))
    } else {
        None
    }
}

pub fn bleed_out<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    if !ctx.character.dead && ctx.character.dying.get() {
        ctx.character.bleed_counter = ctx.character.bleed_counter.wrapping_add(1);
        if ctx.character.bleed_counter == 150 {
            flow!(set_dead(ctx, true));
        }
    }
    Flow::Continue
}

fn release_stump<R: Real>(ctx: &mut Ctx<R>, joint: usize, stump: usize, snap: usize) -> Flow {
    let j = ctx.character.joints[joint];
    if j != NONE {
        if !ctx.world.joint(j).broken {
            flow!(proximal_break(
                ctx,
                joint,
                R::from_i32(ctx.character.break_limits[snap]) + R::one()
            ));
        } else if ctx.character.stumps[stump] != NONE {
            flow!(world::destroy_joint(ctx, j));
            let sh = body::get_shape_list(ctx, ctx.character.stumps[stump]);
            ctx.world.shape_mut(sh).filter = ctx.character.zero_filter;
            flow!(world::refilter(ctx, sh));
        }
    }
    Flow::Continue
}

pub fn chest_smash<R: Real>(ctx: &mut Ctx<R>, _impulse: R) -> Flow {
    lab! {
        crate::lab::record(crate::lab::Event::ChestSmash);
    }
    disarm(ctx, ctx.character.shapes[CHEST]);
    flow!(set_dead(ctx, true));
    if ctx.world.joint(ctx.character.joints[NECK]).broken.get() {
        if ctx.character.spine != NONE {
            flow!(composites::break_end(ctx, ctx.character.spine, 0));
        }
    } else {
        flow!(neck_break(ctx, R::from_i32(ctx.character.break_limits[1])));
    }
    flow!(release_stump(ctx, SHOULDER1, 0, 5));
    flow!(release_stump(ctx, SHOULDER2, 1, 5));
    if ctx.world.joint(ctx.character.joints[WAIST]).broken.get() {
        if ctx.character.intestine != NONE {
            flow!(composites::break_end(ctx, ctx.character.intestine, 1));
        }
    } else {
        flow!(torso_break(
            ctx,
            R::from_i32(ctx.character.break_limits[3]) - R::one()
        ));
        if ctx.character.intestine == NONE {
            return Flow::Abort(AbortSite::GameNullDeref);
        }
        flow!(composites::break_end(ctx, ctx.character.intestine, 1));
    }
    let chest = ctx.character.bodies[CHEST];
    let scale = ctx.character.scale;
    let mut circle = ShapeDef::circle();
    circle.density = R::one();
    circle.friction = hw_real::rl!(R; 0.3);
    circle.restitution = hw_real::rl!(R; 0.1);
    circle.radius = ctx.character.chunk_radius[1] / scale;
    circle.filter = ctx.character.zero_filter;
    let p = body::get_position(ctx, chest);
    let mut angle = body::get_angle(ctx, chest) + hw_real::rl!(R; core::f64::consts::PI) / hw_real::rl!(R; 4.0);
    let radius = hw_real::rl!(R; 5.0) / scale;
    for _ in 0..4 {
        let mut bd = BodyDef::new();
        bd.position = Vec2::new(
            p.x + sin_cos(angle).1 * radius,
            p.y + sin_cos(angle).0 * radius,
        );
        bd.angle = body::get_angle(ctx, chest);
        let b = new_body!(ctx, &bd);
        flow!(body::create_shape(ctx, b, &circle).0);
        flow!(body::set_mass_from_shapes(ctx, b));
        body::set_linear_velocity(
            ctx,
            b,
            body::get_linear_velocity_from_world_point(ctx, chest, bd.position),
        );
        body::set_angular_velocity(ctx, b, body::get_angular_velocity(ctx, chest));
        angle = angle + hw_real::rl!(R; core::f64::consts::PI) / hw_real::rl!(R; 2.0);
    }
    circle.radius = ctx.character.chunk_radius[0] / scale;
    let mut bd = BodyDef::new();
    bd.position = p;
    bd.angle = body::get_angle(ctx, chest);
    let b = new_body!(ctx, &bd);
    ctx.character.heart = b;
    ctx.world.bodies.retain(b);
    flow!(body::create_shape(ctx, b, &circle).0);
    flow!(body::set_mass_from_shapes(ctx, b));
    body::set_linear_velocity(ctx, b, body::get_linear_velocity(ctx, chest));
    body::set_angular_velocity(ctx, b, body::get_angular_velocity(ctx, chest));
    body::set_angular_velocity(ctx, chest, R::zero());
    body::set_linear_velocity(ctx, chest, Vec2::zero());
    flow!(world::destroy_body(ctx, chest));
    ctx.character.camera_focus = b;
    if !ctx.character.child_active {
        ctx.level.focus = b;
    }
    if matches!(ctx.character.kind, 1 | 2 | 3 | 4 | 5) {
        flow!(eject(ctx));
    }
    Flow::Continue
}

pub fn pelvis_smash<R: Real>(ctx: &mut Ctx<R>, _impulse: R) -> Flow {
    disarm(ctx, ctx.character.shapes[PELVIS]);
    flow!(release_stump(ctx, HIP1, 2, 7));
    flow!(release_stump(ctx, HIP2, 3, 7));
    if ctx.world.joint(ctx.character.joints[WAIST]).broken.get() {
        if ctx.character.intestine != NONE {
            flow!(composites::break_end(ctx, ctx.character.intestine, 0));
        }
    } else {
        flow!(torso_break(ctx, R::from_i32(ctx.character.break_limits[3])));
    }
    let pelvis = ctx.character.bodies[PELVIS];
    let scale = ctx.character.scale;
    let mut circle = ShapeDef::circle();
    circle.density = R::one();
    circle.friction = hw_real::rl!(R; 0.3);
    circle.restitution = hw_real::rl!(R; 0.1);
    circle.radius = ctx.character.chunk_radius[2] / scale;
    circle.filter = ctx.character.zero_filter;
    let p = body::get_position(ctx, pelvis);
    let mut angle = body::get_angle(ctx, pelvis);
    let radius = hw_real::rl!(R; 3.0) / scale;
    for _ in 0..3 {
        let mut bd = BodyDef::new();
        bd.position = Vec2::new(
            p.x + sin_cos(angle).1 * radius,
            p.y + sin_cos(angle).0 * radius,
        );
        bd.angle = body::get_angle(ctx, pelvis);
        let b = new_body!(ctx, &bd);
        flow!(body::create_shape(ctx, b, &circle).0);
        flow!(body::set_mass_from_shapes(ctx, b));
        body::set_linear_velocity(
            ctx,
            b,
            body::get_linear_velocity_from_world_point(ctx, pelvis, bd.position),
        );
        body::set_angular_velocity(ctx, b, body::get_angular_velocity(ctx, pelvis));
        angle = angle + hw_real::rl!(R; core::f64::consts::PI) * hw_real::rl!(R; 2.0) / hw_real::rl!(R; 3.0);
    }
    flow!(world::destroy_body(ctx, pelvis));
    if matches!(ctx.character.kind, 1 | 2 | 3 | 4 | 5) {
        flow!(eject(ctx));
    }
    Flow::Continue
}

pub fn consume_results<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    lab! {
        if let Some(body) = lab_primary_chest(ctx) {
            let shape = ctx.character.primary.shapes[CHEST];
            let k = key(shape);
            if k < CONTACT_KEYS {
                crate::lab::record(crate::lab::Event::DamageSnapshot {
                    body, actor_kind: ctx.character.primary.kind, child: ctx.character.child_active.get(),
                    dead: ctx.character.primary.dead.get(), shape,
                    threshold: crate::lab::f(ctx.character.contact_thresholds[k]),
                    smash_limit: crate::lab::f(ctx.character.primary.smash_limits[1]),
                    buffer: lab_buffer(ctx, ctx.character.contact_results[k]),
                });
            }
        }
    }
    if ctx.character.kind == 3 {
        let k = key(ctx.character.bicycle.helmet_shape);
        let r = ctx.character.contact_results[k];
        if r.present.get() {
            flow!(super::bicycle_damage::helmet_smash(ctx, r.impulse));
            let k = key(ctx.character.bicycle.helmet_shape);
            ctx.character.contact_results[k] = BufferedResult::blank();
            let k = key(ctx.character.head1_shape);
            ctx.character.contact_results[k] = BufferedResult::blank();
        }
    }
    if ctx.character.kind == 2 {
        let k = key(ctx.character.segway.helmet_shape);
        let r = ctx.character.contact_results[k];
        if r.present.get() {
            flow!(super::segway::helmet_smash(ctx, r.impulse));
            // Both dictionary keys are evaluated AFTER helmetSmash mutates them.
            let k = key(ctx.character.segway.helmet_shape);
            ctx.character.contact_results[k] = BufferedResult::blank();
            let k = key(ctx.character.head1_shape);
            ctx.character.contact_results[k] = BufferedResult::blank();
        }
    }
    let items = [
        HEAD, CHEST, PELVIS, LOWER_LEG1, LOWER_LEG2, LOWER_ARM1, LOWER_ARM2,
    ];
    for item_index in 0..items.len() {
        let part = items[item_index];
        if part == LOWER_ARM1 && ctx.character.kind == 3 {
            flow!(super::bicycle_damage::consume_vehicle(ctx));
        }
        let shape = if part == HEAD {
            ctx.character.head1_shape
        } else {
            ctx.character.shapes[part]
        };
        let k = key(shape);
        let r = ctx.character.contact_results[k];
        if !r.present {
            continue;
        }
        flow!(match part {
            HEAD => head_smash(ctx, r.impulse),
            CHEST => {
                lab! {
                    if let Some(body) = lab_primary_chest(ctx) {
                        crate::lab::record(crate::lab::Event::DamageConsume {
                            body, actor_kind: ctx.character.primary.kind, child: ctx.character.child_active.get(),
                            dead: ctx.character.primary.dead.get(), shape,
                            threshold: crate::lab::f(ctx.character.contact_thresholds[k]),
                            smash_limit: crate::lab::f(ctx.character.primary.smash_limits[1]),
                            buffer: lab_buffer(ctx, r),
                        });
                    }
                }
                chest_smash(ctx, r.impulse)
            },
            PELVIS => pelvis_smash(ctx, r.impulse),
            LOWER_LEG1 | LOWER_LEG2 => foot_smash(ctx, part, r.impulse),
            _ => {
                if r.other == NONE {
                    return Flow::Abort(AbortSite::GameNullDeref);
                }
                grab(
                    ctx,
                    ctx.character.bodies[part],
                    ctx.world.shape(r.other).body,
                )
            }
        });
        ctx.character.contact_results[k] = BufferedResult::blank();
        if part != LOWER_ARM1 && part != LOWER_ARM2 {
            ctx.character.contact_adds[k] = crate::Flag32::FALSE;
        }
    }
    if ctx.character.kind == 1 {
        return super::wheelchair_damage::consume_results(ctx);
    }
    if ctx.character.kind == 4 {
        return super::motorcart_damage::consume_results(ctx);
    }
    if ctx.character.kind == 2 {
        let k = key(ctx.character.segway.shapes[0]);
        let r = ctx.character.contact_results[k];
        if r.present.get() {
            flow!(super::segway::frame_smash(ctx, r.impulse));
            ctx.character.contact_results[k] = BufferedResult::blank();
        }
    }
    Flow::Continue
}
pub fn contact_add<R: Real>(
    ctx: &mut Ctx<R>,
    own: H,
    other: H,
    velocity: Vec2<R>,
    normal: Vec2<R>,
) -> Flow {
    let a = ctx.world.shape(own).body;
    let b = ctx.world.shape(other).body;
    if b == NONE {
        return Flow::Abort(AbortSite::GameNullDeref);
    }
    let mass = ctx.world.body(b).mass;
    let k = key(own);
    if ctx.character.contact_adds[k].get() || ctx.world.shape(other).is_sensor.get() {
        return Flow::Continue;
    }
    if !mass.eq_num(R::zero()) {
        if a == NONE {
            return Flow::Abort(AbortSite::GameNullDeref);
        }
        if mass.lt(ctx.world.body(a).mass) {
            return Flow::Continue;
        }
    }
    if hw_real::as3::math_abs(velocity.x * normal.x + velocity.y * normal.y).gt(hw_real::rl!(R; 4.0)) {
        ctx.character.contact_adds[k] = crate::Flag32::TRUE;
        ctx.world.shapes.retain(own);
    }
    Flow::Continue
}
pub fn consume_adds<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    for k in 0..CONTACT_KEYS {
        if ctx.character.contact_adds[k].get() {
            if k == MAX_SHAPES || ctx.world.shape(k as H).body == NONE {
                return Flow::Abort(AbortSite::GameNullDeref);
            }
            ctx.character.contact_adds[k] = crate::Flag32::FALSE;
        }
    }
    Flow::Continue
}
