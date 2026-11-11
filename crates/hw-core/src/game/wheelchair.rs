//! WheelChairGuy. Construction first; controls and break transitions follow separately.
use super::character::{
    self, CharacterState, CHARACTER_HANDLER, CHEST, HIP1, HIP2, LOWER_ARM1, LOWER_ARM2, PELVIS,
    UPPER_LEG1, UPPER_LEG2,
};
use crate::b2::{
    body, joints,
    math::Vec2,
    state::{BodyDef, FilterData, JointDef, ShapeDef},
    world,
};
use crate::{flow, flow::AbortSite, Ctx, Flow, H, NONE};
use hw_data::CharTable;
use hw_real::Real;

#[derive(Clone, Copy)]
#[repr(C)]
pub struct WheelchairState<R: Real> {
    pub chair: H,
    pub wheels: [H; 2],
    /// chair 1/2/3, big wheel. The small wheel has no retained shape reference in AS3.
    pub shapes: [H; 4],
    /// big wheel, small wheel, chair/chest, chair/pelvis, chair/leg1, chair/leg2.
    pub joints: [H; 6],
    pub ejected: crate::Flag32,
    pub chair_smashed: crate::Flag32,
    pub firing: crate::Flag32,
    pub jet_angle: R,
    pub wheel_current_speed: R,
    pub wheel_new_speed: R,
    pub impulse_thresholds: [R; 2],
    pub wheel_contacts: R,
    pub jet: H,
    pub fuel: H,
    pub jet_shape: H,
    pub fuel_shape: H,
}
impl<R: Real> WheelchairState<R> {
    pub fn blank() -> Self {
        Self {
            chair: NONE,
            wheels: [NONE; 2],
            shapes: [NONE; 4],
            joints: [NONE; 6],
            ejected: crate::Flag32::FALSE,
            chair_smashed: crate::Flag32::FALSE,
            firing: crate::Flag32::FALSE,
            jet_angle: R::zero(),
            wheel_current_speed: R::from_words([0x7fffffff, 0xe0000000]),
            wheel_new_speed: R::from_words([0x7fffffff, 0xe0000000]),
            impulse_thresholds: [hw_real::rl!(R; 200.0); 2],
            wheel_contacts: R::zero(),
            jet: NONE,
            fuel: NONE,
            jet_shape: NONE,
            fuel_shape: NONE,
        }
    }
}

pub fn create_bodies<R: Real>(ctx: &mut Ctx<R>, data: &CharTable) -> Flow {
    let start = ctx.character.start;
    let cs = ctx.character.scale;
    let mut polygon = ShapeDef::polygon();
    polygon.density = hw_real::rl!(R; 2.0);
    polygon.friction = hw_real::rl!(R; 0.3);
    polygon.restitution = hw_real::rl!(R; 0.1);
    polygon.filter.category_bits = 1;
    let mut circle = ShapeDef::circle();
    circle.density = hw_real::rl!(R; 5.0);
    circle.friction = R::one();
    circle.restitution = hw_real::rl!(R; 0.1);
    circle.filter.category_bits = 1;
    for (i, name) in ["chair1Shape", "chair2Shape", "chair3Shape"]
        .iter()
        .enumerate()
    {
        let Some(p) = data.guide_part(name) else {
            return Flow::Abort(AbortSite::PortUnimplemented);
        };
        let pos = Vec2::new(
            start.x + R::from_words(p.x) / cs,
            start.y + R::from_words(p.y) / cs,
        );
        polygon.set_as_oriented_box(
            R::from_words(p.width) / hw_real::rl!(R; 2.0) / cs,
            R::from_words(p.height) / hw_real::rl!(R; 2.0) / cs,
            Some(pos),
            R::zero(),
        );
        if i == 0 {
            let b = world::create_body(ctx, &BodyDef::new());
            if b == NONE {
                return Flow::Abort(AbortSite::PortCapacity);
            }
            ctx.character.wheelchair.chair = b;
            ctx.world.bodies.retain(b);
        }
        let (f, sh) = body::create_shape(ctx, ctx.character.wheelchair.chair, &polygon);
        flow!(f);
        if sh == NONE {
            return Flow::Abort(AbortSite::GameNullDeref);
        }
        ctx.character.wheelchair.shapes[i] = sh;
        ctx.world.shapes.retain(sh);
        ctx.world.shape_mut(sh).handlers[3] = CHARACTER_HANDLER | 32;
        ctx.world.shape_mut(sh).handlers[0] = CHARACTER_HANDLER | 32;
    }
    flow!(body::set_mass_from_shapes(
        ctx,
        ctx.character.wheelchair.chair
    ));
    for (i, name) in ["bigWheelShape", "smallWheelShape"].iter().enumerate() {
        let Some(p) = data.guide_part(name) else {
            return Flow::Abort(AbortSite::PortUnimplemented);
        };
        let mut bd = BodyDef::new();
        bd.position = Vec2::new(
            start.x + R::from_words(p.x) / cs,
            start.y + R::from_words(p.y) / cs,
        );
        bd.angle = R::from_words(p.rotation) / (hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI));
        circle.radius = R::from_words(p.width) / hw_real::rl!(R; 2.0) / cs;
        let b = world::create_body(ctx, &bd);
        if b == NONE {
            return Flow::Abort(AbortSite::PortCapacity);
        }
        ctx.character.wheelchair.wheels[i] = b;
        ctx.world.bodies.retain(b);
        let (f, sh) = body::create_shape(ctx, b, &circle);
        flow!(f);
        if sh == NONE {
            return Flow::Abort(AbortSite::GameNullDeref);
        }
        if i == 0 {
            ctx.character.wheelchair.shapes[3] = sh;
            ctx.world.shapes.retain(sh);
            ctx.world.shape_mut(sh).handlers[3] = CHARACTER_HANDLER | 33;
            ctx.world.shape_mut(sh).handlers[0] = CHARACTER_HANDLER | 33;
            ctx.world.shape_mut(sh).handlers[1] = CHARACTER_HANDLER | 33;
        }
        flow!(body::set_mass_from_shapes(ctx, b));
    }
    let mut filter = FilterData::new();
    filter.category_bits = 4;
    filter.group_index = ctx.character.default_filter.group_index;
    filter.mask_bits = 8;
    for part in [LOWER_ARM1, LOWER_ARM2] {
        let sh = ctx.character.shapes[part];
        ctx.world.shape_mut(sh).filter = filter;
        flow!(world::refilter(ctx, sh));
    }
    Flow::Continue
}

pub fn create_joints<R: Real>(ctx: &mut Ctx<R>, data: &CharTable) -> Flow {
    let degrees = hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI);
    for (part, j) in [(UPPER_LEG1, HIP1), (UPPER_LEG2, HIP2)] {
        let angle = body::get_angle(ctx, ctx.character.bodies[part])
            - body::get_angle(ctx, ctx.character.bodies[PELVIS]);
        let lower = -hw_real::rl!(R; 110.0) / degrees - angle;
        let upper = -hw_real::rl!(R; 90.0) / degrees - angle;
        joints::set_limits(ctx, ctx.character.joints[j], lower, upper);
    }
    let start = ctx.character.start;
    let cs = ctx.character.scale;
    let chair = ctx.character.wheelchair.chair;
    let mut d = JointDef::revolute();
    d.max_motor_torque = hw_real::rl!(R; 20.0);
    for (i, name) in ["bigWheelShape", "smallWheelShape"].iter().enumerate() {
        let Some(p) = data.guide_part(name) else {
            return Flow::Abort(AbortSite::PortUnimplemented);
        };
        let pos = Vec2::new(
            start.x + R::from_words(p.x) / cs,
            start.y + R::from_words(p.y) / cs,
        );
        joints::revolute_def_initialize(
            ctx,
            &mut d,
            chair,
            ctx.character.wheelchair.wheels[i],
            pos,
        );
        let (f, j) = world::create_joint(ctx, &d);
        flow!(f);
        ctx.character.wheelchair.joints[i] = j;
        ctx.world.joints.retain(j);
    }
    // The source initializes this unused distance definition before adding the four attachments.
    let mut unused = JointDef::distance();
    let pelvis = ctx.character.bodies[PELVIS];
    let pos = body::get_position(ctx, pelvis);
    joints::distance_def_initialize(ctx, &mut unused, chair, pelvis, pos, pos);
    for (i, part) in [CHEST, PELVIS, UPPER_LEG1, UPPER_LEG2].iter().enumerate() {
        let b = ctx.character.bodies[*part];
        let pos = body::get_position(ctx, b);
        joints::revolute_def_initialize(ctx, &mut d, chair, b, pos);
        let (f, j) = world::create_joint(ctx, &d);
        flow!(f);
        ctx.character.wheelchair.joints[i + 2] = j;
        ctx.world.joints.retain(j);
    }
    Flow::Continue
}

/// Construction-stage entry point; vehicle controls and contacts are not yet enabled.
pub fn create<R: Real>(ctx: &mut Ctx<R>, data: &CharTable, x: R, y: R) -> Flow {
    *ctx.character = CharacterState::blank();
    ctx.character.kind = 1;
    ctx.character.start = Vec2::new(x / hw_real::rl!(R; 62.5), y / hw_real::rl!(R; 62.5));
    ctx.character.scale = hw_real::rl!(R; 62.5) * hw_real::rl!(R; 2.0);
    character::create_filters(ctx, -1);
    flow!(character::create_bodies(ctx, data));
    flow!(create_bodies(ctx, data));
    flow!(character::create_joints(ctx, data));
    flow!(create_joints(ctx, data));
    character::set_limits(ctx);
    character::create_dictionaries(ctx);
    for (i, part) in [0, 3].iter().enumerate() {
        let k = super::damage::key(ctx.character.wheelchair.shapes[*part]);
        ctx.character.contact_thresholds[k] = ctx.character.wheelchair.impulse_thresholds[i];
    }
    Flow::Continue
}

pub fn eject<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    use super::poses;
    if ctx.character.wheelchair.ejected.get() {
        return Flow::Continue;
    }
    ctx.character.wheelchair.ejected = crate::Flag32::TRUE;
    flow!(poses::reset_joint_limits(ctx));
    let items = [3, 2, 4, 5];
    for item_index in 0..items.len() {
        let i = items[item_index];
        let j = ctx.character.wheelchair.joints[i];
        if i < 4 && j == NONE {
            return Flow::Abort(AbortSite::GameNullDeref);
        }
        if j != NONE {
            flow!(world::destroy_joint(ctx, j));
            ctx.character.wheelchair.joints[i] = NONE;
        }
    }
    let small = body::get_shape_list(ctx, ctx.character.wheelchair.wheels[1]);
    let big = body::get_shape_list(ctx, ctx.character.wheelchair.wheels[0]);
    ctx.world.shape_mut(small).filter = ctx.character.zero_filter;
    ctx.world.shape_mut(big).filter = ctx.character.zero_filter;
    flow!(world::refilter(ctx, small));
    flow!(world::refilter(ctx, big));
    let mut sh = body::get_shape_list(ctx, ctx.character.wheelchair.chair);
    while sh != NONE {
        ctx.world.shape_mut(sh).filter = ctx.character.zero_filter;
        flow!(world::refilter(ctx, sh));
        sh = ctx.world.shape(sh).next;
    }
    let items = [LOWER_ARM1, LOWER_ARM2];
    for item_index in 0..items.len() {
        let part = items[item_index];
        let sh = body::get_shape_list(ctx, ctx.character.bodies[part]);
        if ctx.world.shape(sh).filter.mask_bits == 8 {
            ctx.world.shape_mut(sh).filter = ctx.character.default_filter;
            flow!(world::refilter(ctx, sh));
        }
    }
    Flow::Continue
}

/// Replay key order: horizontal, vertical, space, shift, ctrl, z.
/// Callers gate this on dead for checkReplayData; doNothing calls it unconditionally.
/// Compatibility entry for existing CPU callers.
#[inline]
pub fn input<R: Real>(ctx: &mut Ctx<R>, keys: [bool; 8]) -> Flow {
    input_bits(ctx, crate::Input::from_bools(keys))
}
pub fn input_bits<R: Real>(ctx: &mut Ctx<R>, keys: crate::Input) -> Flow {
    use super::poses::{self, set_pose};
    let ejected = ctx.character.wheelchair.ejected;
    let chair = ctx.character.wheelchair.chair;
    if keys.pressed(0) != keys.pressed(1) {
        if ejected.get() {
            set_pose(ctx, if keys.pressed(0) { 1 } else { 2 });
        } else {
            let angle = body::get_angle(ctx, chair);
            let velocity = body::get_angular_velocity(ctx, chair);
            let mut factor = if keys.pressed(0) {
                (velocity + hw_real::rl!(R; 5.0)) / hw_real::rl!(R; 5.0)
            } else {
                (velocity - hw_real::rl!(R; 5.0)) / -hw_real::rl!(R; 5.0)
            };
            if factor.lt(R::zero()) {
                factor = R::zero();
            }
            if factor.gt(R::one()) {
                factor = R::one();
            }
            let x = crate::b2::math::sin_cos(angle).0 * hw_real::rl!(R; 0.5) * factor;
            let y = crate::b2::math::sin_cos(angle).1 * hw_real::rl!(R; 0.5) * factor;
            let c = body::get_local_center(ctx, chair);
            let p = body::get_world_point(
                ctx,
                chair,
                Vec2::new(
                    if keys.pressed(0) {
                        c.x + R::one()
                    } else {
                        c.x - R::one()
                    },
                    c.y,
                ),
            );
            body::apply_impulse(ctx, chair, Vec2::new(y, -x), p);
        }
    } else if ejected.get() && (ctx.character.current_pose == 1 || ctx.character.current_pose == 2) {
        set_pose(ctx, 0);
    }
    let wheel_joint = ctx.character.wheelchair.joints[0];
    if keys.pressed(2) != keys.pressed(3) {
        if ejected.get() {
            set_pose(ctx, if keys.pressed(2) { 3 } else { 4 });
        } else {
            if !joints::is_motor_enabled(ctx, wheel_joint) {
                joints::enable_motor(ctx, wheel_joint, true);
            }
            let current = joints::get_joint_speed(ctx, wheel_joint);
            ctx.character.wheelchair.wheel_current_speed = current;
            let speed = if keys.pressed(2) {
                if current.lt(R::zero()) {
                    R::zero()
                } else if current.lt(hw_real::rl!(R; 20.0)) {
                    current + hw_real::rl!(R; 0.2)
                } else {
                    current
                }
            } else if current.gt(R::zero()) {
                R::zero()
            } else if current.gt(-hw_real::rl!(R; 20.0)) {
                current - hw_real::rl!(R; 0.2)
            } else {
                current
            };
            ctx.character.wheelchair.wheel_new_speed = speed;
            joints::set_motor_speed(ctx, wheel_joint, speed);
            set_pose(ctx, 5);
        }
    } else {
        if joints::is_motor_enabled(ctx, wheel_joint) {
            joints::enable_motor(ctx, wheel_joint, false);
        }
        if ejected.get() {
            if ctx.character.current_pose == 3 || ctx.character.current_pose == 4 {
                set_pose(ctx, 0);
            }
        } else if ctx.character.current_pose == 5 {
            set_pose(ctx, 0);
        }
    }
    if keys.pressed(4) {
        if ejected.get() {
            flow!(poses::start_grab(ctx));
        } else if !ctx.character.wheelchair.chair_smashed && !ctx.character.wheelchair.firing {
            ctx.character.wheelchair.firing = crate::Flag32::TRUE;
        }
    } else if ejected.get() {
        flow!(poses::release_grip(ctx));
    } else if ctx.character.wheelchair.firing.get() {
        ctx.character.wheelchair.firing = crate::Flag32::FALSE;
    }
    if keys.pressed(5) {
        if ejected.get() {
            set_pose(ctx, 6);
        } else {
            ctx.character.wheelchair.jet_angle = ctx.character.wheelchair.jet_angle + hw_real::rl!(R; 0.1);
        }
    } else if ctx.character.current_pose == 6 {
        set_pose(ctx, 0);
    }
    if keys.pressed(6) {
        if ejected.get() {
            set_pose(ctx, 7);
        } else {
            ctx.character.wheelchair.jet_angle = ctx.character.wheelchair.jet_angle - hw_real::rl!(R; 0.1);
        }
    } else if ctx.character.current_pose == 7 {
        set_pose(ctx, 0);
    }
    if keys.pressed(7) {
        flow!(eject(ctx));
    }
    Flow::Continue
}

pub fn actions<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    if ctx.character.wheelchair.firing.get() {
        let chair = ctx.character.wheelchair.chair;
        let wheel = ctx.character.wheelchair.wheels[0];
        let angle = body::get_angle(ctx, chair) + ctx.character.wheelchair.jet_angle;
        let x = crate::b2::math::sin_cos(angle).0 * hw_real::rl!(R; 2.0);
        let y = crate::b2::math::sin_cos(angle).1 * hw_real::rl!(R; 2.0);
        body::apply_impulse(
            ctx,
            wheel,
            Vec2::new(x, y),
            body::get_world_center(ctx, chair),
        );
    }
    // wheelContacts controls rolling sound only. Base checkPose always runs, even when dead.
    flow!(super::poses::wheelchair_pose(ctx));
    super::damage::bleed_out(ctx)
}
