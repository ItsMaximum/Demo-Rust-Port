//! BicycleGuy / IrresponsibleDad construction. ChildSeatKid is a separate actor.
use super::{
    character::{self, *},
    damage,
};
use crate::b2::{
    body, joints,
    math::Vec2,
    state::{BodyDef, JointDef, ShapeDef},
    world,
};
use crate::{flow, flow::AbortSite, Ctx, Flow, H, NONE};
use hw_data::CharTable;
use hw_real::Real;
pub const FRAME: usize = 0;
pub const BACK: usize = 1;
pub const FRONT: usize = 2;
pub const GEAR: usize = 3;
pub const BACK_J: usize = 0;
pub const FRONT_J: usize = 1;
pub const PELVIS_J: usize = 2;
pub const HAND1: usize = 3;
pub const HAND2: usize = 4;
pub const FRAME_GEAR: usize = 5;
pub const GEAR_J: usize = 6;
pub const FOOT1: usize = 7;
pub const FOOT2: usize = 8;
#[derive(Clone, Copy)]
#[repr(C)]
pub struct BicycleState<R: Real> {
    pub bodies: [H; 4],
    pub shapes: [H; 5],
    pub joints: [H; 9],
    pub ejected: crate::Flag32,
    pub helmet_shape: H,
    pub helmet: H,
    pub helmet_on: crate::Flag32,
    pub wheel_current_speed: R,
    pub wheel_new_speed: R,
    pub wheel_contacts: R,
    pub front_contacts: R,
    pub back_contacts: R,
}
impl<R: Real> BicycleState<R> {
    pub fn blank() -> Self {
        Self {
            bodies: [NONE; 4],
            shapes: [NONE; 5],
            joints: [NONE; 9],
            ejected: crate::Flag32::FALSE,
            helmet_shape: NONE,
            helmet: NONE,
            helmet_on: crate::Flag32::FALSE,
            wheel_current_speed: damage::undefined(),
            wheel_new_speed: damage::undefined(),
            wheel_contacts: R::zero(),
            front_contacts: R::zero(),
            back_contacts: R::zero(),
        }
    }
}
macro_rules! part {
    ($data:expr,$name:expr) => {
        match $data.guide_part($name) {
            Some(p) => p,
            None => return Flow::Abort(AbortSite::PortUnimplemented),
        }
    };
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
        $ctx.world.bodies.retain(b);
        b
    }};
}
fn save_joint<R: Real>(ctx: &mut Ctx<R>, i: usize, d: &JointDef<R>) -> Flow {
    let (f, j) = world::create_joint(ctx, d);
    flow!(f);
    ctx.character.bicycle.joints[i] = j;
    ctx.world.joints.retain(j);
    Flow::Continue
}
pub fn create<R: Real>(ctx: &mut Ctx<R>, data: &CharTable, x: R, y: R) -> Flow {
    *ctx.character = CharacterState::blank();
    ctx.character.kind = 3;
    ctx.character.start = Vec2::new(x / hw_real::rl!(R; 62.5), y / hw_real::rl!(R; 62.5));
    ctx.character.scale = hw_real::rl!(R; 62.5) * hw_real::rl!(R; 2.0);
    character::create_filters(ctx, -1);
    flow!(character::create_bodies(ctx, data));
    flow!(create_bodies(ctx, data));
    flow!(character::create_joints(ctx, data));
    flow!(create_joints(ctx, data));
    character::set_limits(ctx);
    character::create_dictionaries(ctx);
    for i in [0, 4, 3] {
        let k = damage::key(ctx.character.bicycle.shapes[i]);
        ctx.character.contact_thresholds[k] = hw_real::rl!(R; 200.0);
    }
    ctx.character.bicycle.helmet_shape = ctx.character.head1_shape;
    ctx.character.head1_shape = NONE;
    let k = damage::key(ctx.character.bicycle.helmet_shape);
    ctx.character.contact_thresholds[k] = hw_real::rl!(R; 2.0);
    ctx.character.child_present = crate::Flag32::TRUE;
    ctx.character.child_active = crate::Flag32::TRUE;
    let result = super::child::create(ctx, data, x, y);
    ctx.character.child_active = crate::Flag32::FALSE;
    result
}
pub fn create_bodies<R: Real>(ctx: &mut Ctx<R>, data: &CharTable) -> Flow {
    let start = ctx.character.start;
    let scale = ctx.character.scale;
    let mut p = ShapeDef::polygon();
    p.density = hw_real::rl!(R; 2.0);
    p.friction = hw_real::rl!(R; 0.3);
    p.restitution = hw_real::rl!(R; 0.1);
    p.filter.category_bits = 1;
    let mut c = ShapeDef::circle();
    c.density = hw_real::rl!(R; 5.0);
    c.friction = R::one();
    c.restitution = hw_real::rl!(R; 0.3);
    c.filter.category_bits = 1;
    let frame = new_body!(ctx, &BodyDef::new());
    ctx.character.bicycle.bodies[FRAME] = frame;
    for (i, names) in [
        ["seatVert1", "seatVert2", "seatVert3", ""],
        ["frameVert1", "frameVert2", "frameVert3", "frameVert4"],
        ["forkVert1", "forkVert2", "forkVert3", ""],
    ]
    .iter()
    .enumerate()
    {
        if i == 1 {
            p.filter = ctx.character.zero_filter;
        }
        p.vertex_count = if i == 1 { 4 } else { 3 };
        for k in 0..p.vertex_count as usize {
            let v = part!(data, names[k]);
            p.vertices[k] = Vec2::new(
                start.x + R::from_words(v.x) / scale,
                start.y + R::from_words(v.y) / scale,
            );
        }
        let (f, sh) = body::create_shape(ctx, frame, &p);
        flow!(f);
        ctx.character.bicycle.shapes[i] = sh;
        ctx.world.shapes.retain(sh);
        ctx.world.shape_mut(sh).handlers[3] = CHARACTER_HANDLER | 50;
        ctx.world.shape_mut(sh).handlers[0] = CHARACTER_HANDLER | 50;
    }
    flow!(body::set_mass_from_shapes(ctx, frame));
    for (i, name) in ["backWheelShape", "frontWheelShape", "gearShape"]
        .iter()
        .enumerate()
    {
        let v = part!(data, name);
        let mut bd = BodyDef::new();
        bd.position = Vec2::new(
            start.x + R::from_words(v.x) / scale,
            start.y + R::from_words(v.y) / scale,
        );
        bd.angle = R::from_words(v.rotation) / (hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI));
        c.radius = R::from_words(v.width) / hw_real::rl!(R; 2.0) / scale;
        let b = new_body!(ctx, &bd);
        ctx.character.bicycle.bodies[i + 1] = b;
        let (f, sh) = body::create_shape(ctx, b, &c);
        flow!(f);
        flow!(body::set_mass_from_shapes(ctx, b));
        if i < 2 {
            ctx.character.bicycle.shapes[i + 3] = sh;
            ctx.world.shapes.retain(sh);
            for e in [3, 0, 1] {
                ctx.world.shape_mut(sh).handlers[e] = CHARACTER_HANDLER | 51 + i as u32;
            }
        }
    }
    for part in [UPPER_LEG1, UPPER_LEG2, LOWER_LEG1, LOWER_LEG2] {
        let sh = body::get_shape_list(ctx, ctx.character.bodies[part]);
        ctx.world.shape_mut(sh).is_sensor = crate::Flag32::TRUE;
    }
    Flow::Continue
}
pub fn create_joints<R: Real>(ctx: &mut Ctx<R>, data: &CharTable) -> Flow {
    let degrees = hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI);
    for (j, a, b, lo, hi) in [
        (HIP1, UPPER_LEG1, PELVIS, -hw_real::rl!(R; 110.0), hw_real::rl!(R; 10.0)),
        (HIP2, UPPER_LEG2, PELVIS, -hw_real::rl!(R; 110.0), hw_real::rl!(R; 10.0)),
        (ELBOW1, LOWER_ARM1, UPPER_ARM1, -hw_real::rl!(R; 60.0), hw_real::rl!(R; 0.0)),
        (ELBOW2, LOWER_ARM2, UPPER_ARM2, -hw_real::rl!(R; 60.0), hw_real::rl!(R; 0.0)),
        (NECK, HEAD, CHEST, -hw_real::rl!(R; 20.0), hw_real::rl!(R; 0.0)),
    ] {
        let angle = body::get_angle(ctx, ctx.character.bodies[a])
            - body::get_angle(ctx, ctx.character.bodies[b]);
        joints::set_limits(
            ctx,
            ctx.character.joints[j],
            lo / degrees - angle,
            hi / degrees - angle,
        );
    }
    let bike = ctx.character.bicycle;
    let start = ctx.character.start;
    let scale = ctx.character.scale;
    let mut d = JointDef::revolute();
    d.max_motor_torque = hw_real::rl!(R; 20.0);
    for (i, name) in ["backWheelShape", "frontWheelShape"].iter().enumerate() {
        let v = part!(data, name);
        let pos = Vec2::new(
            start.x + R::from_words(v.x) / scale,
            start.y + R::from_words(v.y) / scale,
        );
        joints::revolute_def_initialize(ctx, &mut d, bike.bodies[FRAME], bike.bodies[i + 1], pos);
        flow!(save_joint(ctx, i, &d));
    }
    let pelvis = ctx.character.bodies[PELVIS];
    let pos = body::get_position(ctx, pelvis);
    let mut unused = JointDef::distance();
    joints::distance_def_initialize(ctx, &mut unused, bike.bodies[FRAME], pelvis, pos, pos);
    joints::revolute_def_initialize(ctx, &mut d, bike.bodies[FRAME], pelvis, pos);
    flow!(save_joint(ctx, PELVIS_J, &d));
    let v = part!(data, "handleAnchor");
    let pos = Vec2::new(
        start.x + R::from_words(v.x) / scale,
        start.y + R::from_words(v.y) / scale,
    );
    for (i, part) in [(HAND1, LOWER_ARM1), (HAND2, LOWER_ARM2)] {
        joints::revolute_def_initialize(
            ctx,
            &mut d,
            bike.bodies[FRAME],
            ctx.character.bodies[part],
            pos,
        );
        flow!(save_joint(ctx, i, &d));
    }
    let v = part!(data, "gearShape");
    let pos = Vec2::new(
        start.x + R::from_words(v.x) / scale,
        start.y + R::from_words(v.y) / scale,
    );
    joints::revolute_def_initialize(ctx, &mut d, bike.bodies[FRAME], bike.bodies[GEAR], pos);
    flow!(save_joint(ctx, FRAME_GEAR, &d));
    let mut gear = JointDef::gear();
    gear.body1 = bike.bodies[BACK];
    gear.body2 = bike.bodies[GEAR];
    gear.gear_joints = [
        ctx.character.bicycle.joints[BACK_J],
        ctx.character.bicycle.joints[FRAME_GEAR],
    ];
    flow!(save_joint(ctx, GEAR_J, &gear));
    let h = ctx.character.bicycle.joints[GEAR_J];
    ctx.world.joint_mut(h).ratio = ctx.world.joint(h).ratio * -hw_real::rl!(R; 1.0);
    for (i, part, name) in [
        (FOOT1, LOWER_LEG1, "gearAnchor1"),
        (FOOT2, LOWER_LEG2, "gearAnchor2"),
    ] {
        let v = part!(data, name);
        let pos = Vec2::new(
            start.x + R::from_words(v.x) / scale,
            start.y + R::from_words(v.y) / scale,
        );
        joints::revolute_def_initialize(
            ctx,
            &mut d,
            bike.bodies[GEAR],
            ctx.character.bodies[part],
            pos,
        );
        flow!(save_joint(ctx, i, &d));
    }
    Flow::Continue
}

pub fn eject<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    if ctx.character.bicycle.ejected.get() {
        return Flow::Continue;
    }
    ctx.character.bicycle.ejected = crate::Flag32::TRUE;
    flow!(super::poses::reset_joint_limits(ctx));
    let items = [PELVIS_J, HAND1, HAND2, FOOT1, FOOT2];
    for item_index in 0..items.len() {
        let i = items[item_index];
        let j = ctx.character.bicycle.joints[i];
        if j != NONE {
            flow!(world::destroy_joint(ctx, j));
            ctx.character.bicycle.joints[i] = NONE;
        }
    }
    let front = body::get_shape_list(ctx, ctx.character.bicycle.bodies[FRONT]);
    let back = body::get_shape_list(ctx, ctx.character.bicycle.bodies[BACK]);
    ctx.world.shape_mut(front).filter = ctx.character.zero_filter;
    ctx.world.shape_mut(back).filter = ctx.character.zero_filter;
    flow!(world::refilter(ctx, front));
    flow!(world::refilter(ctx, back));
    let mut sh = body::get_shape_list(ctx, ctx.character.bicycle.bodies[FRAME]);
    while sh != NONE {
        ctx.world.shape_mut(sh).filter = ctx.character.zero_filter;
        flow!(world::refilter(ctx, sh));
        sh = ctx.world.shape(sh).next;
    }
    let items = [UPPER_LEG1, UPPER_LEG2, LOWER_LEG1, LOWER_LEG2];
    for item_index in 0..items.len() {
        let part = items[item_index];
        let sh = body::get_shape_list(ctx, ctx.character.bodies[part]);
        ctx.world.shape_mut(sh).is_sensor = crate::Flag32::FALSE;
        flow!(world::refilter(ctx, sh));
    }
    Flow::Continue
}
/// Compatibility entry for existing CPU callers.
#[inline]
pub fn input<R: Real>(ctx: &mut Ctx<R>, keys: [bool; 8]) -> Flow {
    input_bits(ctx, crate::Input::from_bools(keys))
}
pub fn input_bits<R: Real>(ctx: &mut Ctx<R>, keys: crate::Input) -> Flow {
    use super::poses;
    let ejected = ctx.character.bicycle.ejected;
    if keys.pressed(0) != keys.pressed(1) {
        if ejected.get() {
            poses::set_pose(ctx, if keys.pressed(0) { 1 } else { 2 });
        } else {
            let frame = ctx.character.bicycle.bodies[FRAME];
            let angle = body::get_angle(ctx, frame);
            let av = body::get_angular_velocity(ctx, frame);
            let mut factor = if keys.pressed(0) {
                (av + hw_real::rl!(R; 5.0)) / hw_real::rl!(R; 5.0)
            } else {
                (av - hw_real::rl!(R; 5.0)) / -hw_real::rl!(R; 5.0)
            };
            if factor.lt(R::zero()) {
                factor = R::zero();
            }
            if factor.gt(R::one()) {
                factor = R::one();
            }
            let x = crate::b2::math::sin_cos(angle).0 * hw_real::rl!(R; 3.0) * factor;
            let y = crate::b2::math::sin_cos(angle).1 * hw_real::rl!(R; 3.0) * factor;
            let c = body::get_local_center(ctx, frame);
            let point = body::get_world_point(
                ctx,
                frame,
                Vec2::new(
                    if keys.pressed(0) {
                        c.x + R::one()
                    } else {
                        c.x - R::one()
                    },
                    c.y,
                ),
            );
            body::apply_impulse(ctx, frame, Vec2::new(y, -x), point);
            let items = [
                (
                    NECK,
                    if keys.pressed(0) { hw_real::rl!(R; 0.0) } else { hw_real::rl!(R; 1.0) },
                    if keys.pressed(0) { hw_real::rl!(R; 2.0) } else { hw_real::rl!(R; 1.0) },
                ),
                (ELBOW1, if keys.pressed(0) { hw_real::rl!(R; 1.04) } else { hw_real::rl!(R; 0.0) }, hw_real::rl!(R; 15.0)),
                (ELBOW2, if keys.pressed(0) { hw_real::rl!(R; 1.04) } else { hw_real::rl!(R; 0.0) }, hw_real::rl!(R; 15.0)),
            ];
            for i in 0..items.len() {
                let (j, offset, k) = items[i];
                flow!(poses::set_joint(
                    ctx,
                    ctx.character.joints[j],
                    offset,
                    k,
                    hw_real::rl!(R; 10.0)
                ));
            }
        }
    } else if ejected.get() && (ctx.character.current_pose == 1 || ctx.character.current_pose == 2) {
        poses::set_pose(ctx, 0);
    }
    let back = ctx.character.bicycle.joints[BACK_J];
    let front = ctx.character.bicycle.joints[FRONT_J];
    if keys.pressed(2) != keys.pressed(3) {
        if ejected.get() {
            poses::set_pose(ctx, if keys.pressed(2) { 3 } else { 4 });
        } else {
            if !joints::is_motor_enabled(ctx, back) {
                joints::enable_motor(ctx, back, true);
                joints::enable_motor(ctx, front, true);
            }
            let speed = joints::get_joint_speed(ctx, back);
            ctx.character.bicycle.wheel_current_speed = speed;
            let next = if keys.pressed(2) {
                if speed.lt(R::zero()) {
                    R::zero()
                } else if speed.lt(hw_real::rl!(R; 20.0)) {
                    speed + R::one()
                } else {
                    speed
                }
            } else if speed.gt(R::zero()) {
                R::zero()
            } else if speed.gt(-hw_real::rl!(R; 20.0)) {
                speed - R::one()
            } else {
                speed
            };
            ctx.character.bicycle.wheel_new_speed = next;
            joints::set_motor_speed(ctx, back, next);
            joints::set_motor_speed(ctx, front, next);
        }
    } else {
        if joints::is_motor_enabled(ctx, back) {
            joints::enable_motor(ctx, back, false);
            joints::enable_motor(ctx, front, false);
        }
        if ejected.get() && (ctx.character.current_pose == 3 || ctx.character.current_pose == 4) {
            poses::set_pose(ctx, 0);
        }
    }
    if keys.pressed(4) {
        if ejected.get() {
            flow!(poses::start_grab(ctx));
        } else {
            if !joints::is_motor_enabled(ctx, back) {
                joints::enable_motor(ctx, back, true);
                joints::enable_motor(ctx, front, true);
            }
            joints::set_motor_speed(ctx, back, R::zero());
            joints::set_motor_speed(ctx, front, R::zero());
        }
    } else if ejected.get() {
        flow!(poses::release_grip(ctx));
    }
    if keys.pressed(5) {
        flow!(eject(ctx));
    }
    if keys.pressed(7) {
        flow!(eject(ctx));
    }
    Flow::Continue
}
pub fn actions<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    if ctx.character.current_pose <= 4 {
        flow!(super::poses::wheelchair_pose(ctx));
    }
    flow!(damage::bleed_out(ctx));
    ctx.character.child_active = crate::Flag32::TRUE;
    let result = super::child::actions(ctx);
    ctx.character.child_active = crate::Flag32::FALSE;
    result
}
