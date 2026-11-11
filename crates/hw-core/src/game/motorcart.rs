//! MotorCart, including its replacement chest/pelvis and grocery bodies.
use super::{
    character::{self, CharacterState, *},
    damage,
};

pub fn eject<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    if ctx.character.motorcart.ejected.get() {
        return Flow::Continue;
    }
    ctx.character.motorcart.ejected = crate::Flag32::TRUE;
    flow!(super::poses::reset_joint_limits(ctx));
    let items = [PELVIS_J, HAND1, HAND2, LEG1, LEG2];
    for item_index in 0..items.len() {
        let i = items[item_index];
        let j = ctx.character.motorcart.joints[i];
        if j != NONE {
            flow!(world::destroy_joint(ctx, j));
            ctx.character.motorcart.joints[i] = NONE;
        }
    }
    let mut filter = ctx.character.zero_filter;
    filter.group_index = -2;
    let front = body::get_shape_list(ctx, ctx.character.motorcart.bodies[FRONT]);
    let back = body::get_shape_list(ctx, ctx.character.motorcart.bodies[BACK]);
    ctx.world.shape_mut(front).filter = filter;
    ctx.world.shape_mut(back).filter = filter;
    flow!(world::refilter(ctx, front));
    flow!(world::refilter(ctx, back));
    reset_shocks(ctx);
    let items = [MAIN, SHAFT];
    for item_index in 0..items.len() {
        let i = items[item_index];
        let mut sh = body::get_shape_list(ctx, ctx.character.motorcart.bodies[i]);
        while sh != NONE {
            ctx.world.shape_mut(sh).filter = if i == MAIN {
                filter
            } else {
                ctx.character.zero_filter
            };
            flow!(world::refilter(ctx, sh));
            sh = ctx.world.shape(sh).next;
        }
    }
    Flow::Continue
}
fn reset_shocks<R: Real>(ctx: &mut Ctx<R>) {
    let a = ctx.character.motorcart.joints[BACK_SHOCK_J];
    let b = ctx.character.motorcart.joints[FRONT_SHOCK_J];
    joints::enable_motor(ctx, a, false);
    joints::enable_motor(ctx, b, false);
    joints::set_limits(ctx, a, R::zero(), R::zero());
    joints::set_limits(ctx, b, R::zero(), R::zero());
    joints::set_motor_speed(ctx, a, R::zero());
    joints::set_motor_speed(ctx, b, R::zero());
}
/// Compatibility entry for existing CPU callers.
#[inline]
pub fn input<R: Real>(ctx: &mut Ctx<R>, keys: [bool; 8]) -> Flow {
    input_bits(ctx, crate::Input::from_bools(keys))
}
pub fn input_bits<R: Real>(ctx: &mut Ctx<R>, keys: crate::Input) -> Flow {
    use super::poses;
    let ejected = ctx.character.motorcart.ejected;
    if keys.pressed(0) != keys.pressed(1) {
        if ejected.get() {
            poses::set_pose(ctx, if keys.pressed(0) { 1 } else { 2 });
        } else {
            let b = ctx.character.motorcart.bodies[MAIN];
            let angle = body::get_angle(ctx, b);
            let av = body::get_angular_velocity(ctx, b);
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
            let x = crate::b2::math::sin_cos(angle).0 * hw_real::rl!(R; 1.25) * factor;
            let y = crate::b2::math::sin_cos(angle).1 * hw_real::rl!(R; 1.25) * factor;
            let center = body::get_local_center(ctx, b);
            let pos = body::get_world_point(
                ctx,
                b,
                Vec2::new(
                    if keys.pressed(0) {
                        center.x + R::one()
                    } else {
                        center.x - R::one()
                    },
                    center.y,
                ),
            );
            body::apply_impulse(ctx, b, Vec2::new(y, -x), pos);
        }
    } else if ejected.get() && (ctx.character.current_pose == 1 || ctx.character.current_pose == 2) {
        poses::set_pose(ctx, 0);
    }
    let back = ctx.character.motorcart.joints[BACK_J];
    let front = ctx.character.motorcart.joints[FRONT_J];
    if keys.pressed(2) != keys.pressed(3) {
        if ejected.get() {
            poses::set_pose(ctx, if keys.pressed(2) { 3 } else { 4 });
        } else {
            if !joints::is_motor_enabled(ctx, back) {
                joints::enable_motor(ctx, back, true);
                joints::enable_motor(ctx, front, true);
            }
            let speed = joints::get_joint_speed(ctx, back);
            ctx.character.motorcart.wheel_current_speed = speed;
            let next = if keys.pressed(2) {
                if speed.lt(R::zero()) {
                    R::zero()
                } else if speed.lt(hw_real::rl!(R; 50.0)) {
                    speed + hw_real::rl!(R; 2.0)
                } else {
                    speed
                }
            } else if speed.gt(R::zero()) {
                R::zero()
            } else if speed.gt(-hw_real::rl!(R; 50.0)) {
                speed - hw_real::rl!(R; 2.0)
            } else {
                speed
            };
            ctx.character.motorcart.wheel_new_speed = next;
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
    if ejected.get() {
        if keys.pressed(4) {
            flow!(poses::start_grab(ctx));
        } else {
            flow!(poses::release_grip(ctx));
        }
    } else {
        let a = ctx.character.motorcart.joints[BACK_SHOCK_J];
        let b = ctx.character.motorcart.joints[FRONT_SHOCK_J];
        if keys.pressed(4) && !joints::is_motor_enabled(ctx, a) {
            joints::set_motor_speed(ctx, a, hw_real::rl!(R; 5.0));
            joints::set_motor_speed(ctx, b, hw_real::rl!(R; 5.0));
            joints::set_limits(ctx, a, R::zero(), ctx.character.motorcart.jump_translation);
            joints::set_limits(ctx, b, R::zero(), ctx.character.motorcart.jump_translation);
            joints::enable_motor(ctx, a, true);
            joints::enable_motor(ctx, b, true);
        } else if joints::is_motor_enabled(ctx, a) {
            let speed = joints::get_motor_speed(ctx, a);
            if speed.gt(R::zero()) {
                if joints::get_joint_translation(ctx, a)
                    .gt(ctx.character.motorcart.jump_translation)
                {
                    joints::set_motor_speed(ctx, a, -hw_real::rl!(R; 1.0));
                    joints::set_motor_speed(ctx, b, -hw_real::rl!(R; 1.0));
                }
            } else if speed.lt(R::zero()) && joints::get_joint_translation(ctx, a).lt(R::zero()) {
                reset_shocks(ctx);
            }
        }
    }
    if keys.pressed(5) {
        if ejected.get() {
            poses::set_pose(ctx, 6);
        }
    } else if ctx.character.current_pose == 6 {
        poses::set_pose(ctx, 0);
    }
    if keys.pressed(6) {
        if ejected.get() {
            poses::set_pose(ctx, 7);
        }
    } else if ctx.character.current_pose == 7 {
        poses::set_pose(ctx, 0);
    }
    if keys.pressed(7) {
        flow!(eject(ctx));
    }
    Flow::Continue
}
pub fn actions<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    let pose = ctx.character.current_pose;
    if pose <= 4 {
        flow!(super::poses::wheelchair_pose(ctx));
    } else if pose == 6 || pose == 7 {
        let left = pose == 6;
        let items = [
            (NECK, hw_real::rl!(R; 0.5), hw_real::rl!(R; 2.0)),
            (HIP1, if left { hw_real::rl!(R; 3.5) } else { hw_real::rl!(R; 0.0) }, hw_real::rl!(R; 2.0)),
            (HIP2, if left { hw_real::rl!(R; 0.0) } else { hw_real::rl!(R; 3.5) }, hw_real::rl!(R; 2.0)),
            (KNEE1, if left { hw_real::rl!(R; 0.0) } else { hw_real::rl!(R; 2.0) }, hw_real::rl!(R; 10.0)),
            (KNEE2, if left { hw_real::rl!(R; 2.0) } else { hw_real::rl!(R; 0.0) }, hw_real::rl!(R; 10.0)),
            (SHOULDER1, if left { hw_real::rl!(R; 3.0) } else { hw_real::rl!(R; 1.0) }, hw_real::rl!(R; 20.0)),
            (SHOULDER2, if left { hw_real::rl!(R; 1.0) } else { hw_real::rl!(R; 3.0) }, hw_real::rl!(R; 20.0)),
            (ELBOW1, if left { hw_real::rl!(R; 1.5) } else { hw_real::rl!(R; 3.0) }, hw_real::rl!(R; 15.0)),
            (ELBOW2, if left { hw_real::rl!(R; 3.0) } else { hw_real::rl!(R; 1.5) }, hw_real::rl!(R; 15.0)),
        ];
        for item_index in 0..items.len() {
            let (j, offset, k) = items[item_index];
            flow!(super::poses::set_joint(
                ctx,
                ctx.character.joints[j],
                offset,
                k,
                hw_real::rl!(R; 10.0)
            ));
        }
    }
    damage::bleed_out(ctx)
}
use crate::b2::{
    body, joints,
    math::Vec2,
    state::{BodyDef, FilterData, JointDef, ShapeDef},
    world,
};
use crate::{flow, flow::AbortSite, Ctx, Flow, H, NONE};
use hw_data::{CharPart, CharTable};
use hw_real::Real;
pub const MAIN: usize = 0;
pub const SHAFT: usize = 1;
pub const BACK: usize = 2;
pub const FRONT: usize = 3;
pub const BACK_SHOCK: usize = 4;
pub const FRONT_SHOCK: usize = 5;
pub const BACK_SHOCK_J: usize = 0;
pub const FRONT_SHOCK_J: usize = 1;
pub const MAIN_SHAFT: usize = 2;
pub const PELVIS_J: usize = 3;
pub const BACK_J: usize = 4;
pub const FRONT_J: usize = 5;
pub const HAND1: usize = 6;
pub const HAND2: usize = 7;
pub const LEG1: usize = 8;
pub const LEG2: usize = 9;
#[derive(Clone, Copy)]
#[repr(C)]
pub struct MotorCartState<R: Real> {
    pub bodies: [H; 6],
    pub shapes: [H; 9],
    pub joints: [H; 10],
    pub groceries: [H; 10],
    pub cart: H,
    pub cart_joint: H,
    pub ejected: crate::Flag32,
    pub main_smashed: crate::Flag32,
    pub wheel_current_speed: R,
    pub wheel_new_speed: R,
    pub wheel_contacts: R,
    pub front_contacts: R,
    pub back_contacts: R,
    pub jump_translation: R,
}
impl<R: Real> MotorCartState<R> {
    pub fn blank() -> Self {
        Self {
            bodies: [NONE; 6],
            shapes: [NONE; 9],
            joints: [NONE; 10],
            groceries: [NONE; 10],
            cart: NONE,
            cart_joint: NONE,
            ejected: crate::Flag32::FALSE,
            main_smashed: crate::Flag32::FALSE,
            wheel_current_speed: damage::undefined(),
            wheel_new_speed: damage::undefined(),
            wheel_contacts: R::zero(),
            front_contacts: R::zero(),
            back_contacts: R::zero(),
            jump_translation: hw_real::rl!(R; 20.0) / hw_real::rl!(R; 62.5),
        }
    }
}
macro_rules! part {
    ($data:expr,$name:expr) => {
        match $data.guide_part($name) {
            Some(v) => v,
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
fn point<R: Real>(ctx: &Ctx<R>, v: &CharPart) -> Vec2<R> {
    Vec2::new(
        ctx.character.start.x + R::from_words(v.x) / ctx.character.scale,
        ctx.character.start.y + R::from_words(v.y) / ctx.character.scale,
    )
}
fn box_at<R: Real>(ctx: &Ctx<R>, p: &mut ShapeDef<R>, v: &CharPart) {
    p.set_as_oriented_box(
        R::from_words(v.scale_x) * hw_real::rl!(R; 5.0) / ctx.character.scale,
        R::from_words(v.scale_y) * hw_real::rl!(R; 5.0) / ctx.character.scale,
        Some(point(ctx, v)),
        R::from_words(v.rotation) / (hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI)),
    );
}
fn save_shape<R: Real>(
    ctx: &mut Ctx<R>,
    b: H,
    p: &ShapeDef<R>,
    i: usize,
    result: u32,
    add: u32,
) -> Flow {
    let (f, sh) = body::create_shape(ctx, b, p);
    flow!(f);
    ctx.character.motorcart.shapes[i] = sh;
    ctx.world.shapes.retain(sh);
    if result != 0 {
        ctx.world.shape_mut(sh).handlers[3] = CHARACTER_HANDLER | result;
    }
    if add != 0 {
        ctx.world.shape_mut(sh).handlers[0] = CHARACTER_HANDLER | add;
    }
    Flow::Continue
}
fn save_joint<R: Real>(ctx: &mut Ctx<R>, d: &JointDef<R>, i: usize) -> Flow {
    let (f, j) = world::create_joint(ctx, d);
    flow!(f);
    ctx.character.motorcart.joints[i] = j;
    ctx.world.joints.retain(j);
    Flow::Continue
}
pub fn create<R: Real>(ctx: &mut Ctx<R>, data: &CharTable, x: R, y: R) -> Flow {
    *ctx.character = CharacterState::blank();
    ctx.character.kind = 4;
    ctx.character.start = Vec2::new(x / hw_real::rl!(R; 62.5), y / hw_real::rl!(R; 62.5));
    ctx.character.scale = hw_real::rl!(R; 62.5) * hw_real::rl!(R; 2.0);
    character::create_filters(ctx, -1);
    flow!(character::create_bodies(ctx, data));
    flow!(create_bodies(ctx, data));
    flow!(character::create_joints(ctx, data));
    flow!(create_joints(ctx, data));
    character::set_limits(ctx);
    character::create_dictionaries(ctx);
    for (i, limit) in [(4, hw_real::rl!(R; 100.0)), (0, hw_real::rl!(R; 200.0)), (7, hw_real::rl!(R; 1.0)), (8, hw_real::rl!(R; 1.0))] {
        let k = damage::key(ctx.character.motorcart.shapes[i]);
        ctx.character.contact_thresholds[k] = limit;
    }
    Flow::Continue
}
pub fn create_bodies<R: Real>(ctx: &mut Ctx<R>, data: &CharTable) -> Flow {
    let scale = ctx.character.scale;
    let degrees = hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI);
    let mut p = ShapeDef::polygon();
    p.density = R::one();
    p.friction = hw_real::rl!(R; 0.3);
    p.restitution = hw_real::rl!(R; 0.1);
    p.filter = ctx.character.default_filter;
    flow!(world::destroy_body(ctx, ctx.character.bodies[CHEST]));
    flow!(world::destroy_body(ctx, ctx.character.bodies[PELVIS]));
    for (part, name, vertices) in [
        (
            CHEST,
            "chestShape",
            &[
                "chestVert0",
                "chestVert1",
                "chestVert2",
                "chestVert3",
                "chestVert4",
                "chestVert5",
            ][..],
        ),
        (
            PELVIS,
            "pelvisShape",
            &[
                "pelvisVert0",
                "pelvisVert1",
                "pelvisVert2",
                "pelvisVert3",
                "pelvisVert4",
            ][..],
        ),
    ] {
        let v = part!(data, name);
        let mut bd = BodyDef::new();
        bd.position = point(ctx, v);
        bd.angle = R::from_words(v.rotation) / degrees;
        let b = new_body!(ctx, &bd);
        ctx.character.bodies[part] = b;
        p.vertex_count = vertices.len() as i32;
        for (i, name) in vertices.iter().enumerate() {
            let v = part!(data, name);
            p.vertices[i] = Vec2::new(R::from_words(v.x) / scale, R::from_words(v.y) / scale);
        }
        let (f, sh) = body::create_shape(ctx, b, &p);
        flow!(f);
        ctx.character.shapes[part] = sh;
        ctx.world.shapes.retain(sh);
        ctx.world.shape_mut(sh).material = 1;
        ctx.world.shape_mut(sh).handlers[3] = handler(ctx, part as u32);
        ctx.world.shape_mut(sh).handlers[0] = handler(ctx, part as u32);
        flow!(body::set_mass_from_shapes(ctx, b));
        body::allow_sleeping(ctx, b, false);
        if part == CHEST {
            ctx.character.camera_focus = b;
            ctx.level.focus = b;
        }
    }
    p.density = hw_real::rl!(R; 4.0);
    p.filter = FilterData::new();
    p.filter.category_bits = 1;
    p.filter.group_index = -2;
    let main = new_body!(ctx, &BodyDef::new());
    ctx.character.motorcart.bodies[MAIN] = main;
    let shaft = new_body!(ctx, &BodyDef::new());
    ctx.character.motorcart.bodies[SHAFT] = shaft;
    p.vertex_count = 4;
    for (i, name) in ["frontVert1", "frontVert2", "frontVert3", "frontVert4"]
        .iter()
        .enumerate()
    {
        p.vertices[i] = point(ctx, part!(data, name));
    }
    flow!(save_shape(ctx, main, &p, 0, 70, 0));
    box_at(ctx, &mut p, part!(data, "baseShape"));
    flow!(save_shape(ctx, main, &p, 1, 70, 0));
    box_at(ctx, &mut p, part!(data, "cartShape"));
    flow!(save_shape(ctx, main, &p, 4, 71, 71));
    box_at(ctx, &mut p, part!(data, "handleShape"));
    flow!(body::create_shape(ctx, shaft, &p).0);
    p.filter = ctx.character.zero_filter;
    p.filter.group_index = -2;
    box_at(ctx, &mut p, part!(data, "seatShape"));
    flow!(save_shape(ctx, main, &p, 2, 70, 70));
    p.vertex_count = 4;
    for (i, name) in ["rearVert1", "rearVert2", "rearVert3", "rearVert4"]
        .iter()
        .enumerate()
    {
        p.vertices[i] = point(ctx, part!(data, name));
    }
    flow!(save_shape(ctx, main, &p, 3, 70, 0));
    flow!(body::set_mass_from_shapes(ctx, main));
    box_at(ctx, &mut p, part!(data, "shaftShape"));
    flow!(body::create_shape(ctx, shaft, &p).0);
    flow!(body::set_mass_from_shapes(ctx, shaft));
    let mut c = ShapeDef::circle();
    c.density = hw_real::rl!(R; 5.0);
    c.friction = R::one();
    c.restitution = hw_real::rl!(R; 0.3);
    c.filter.category_bits = 1;
    c.filter.group_index = -2;
    let mut wheel_pos = [Vec2::zero(); 2];
    for (i, name) in ["backWheelShape", "frontWheelShape"].iter().enumerate() {
        let v = part!(data, name);
        let mut bd = BodyDef::new();
        bd.position = point(ctx, v);
        wheel_pos[i] = bd.position;
        bd.angle = R::from_words(v.rotation) / degrees;
        c.radius = R::from_words(v.width) / hw_real::rl!(R; 2.0) / scale;
        let b = new_body!(ctx, &bd);
        ctx.character.motorcart.bodies[BACK + i] = b;
        flow!(save_shape(ctx, b, &c, 5 + i, 0, 72 + i as u32));
        flow!(body::set_mass_from_shapes(ctx, b));
        let sh = ctx.character.motorcart.shapes[5 + i];
        ctx.world.shape_mut(sh).handlers[1] = CHARACTER_HANDLER | 72 + i as u32;
    }
    for (i, pos) in wheel_pos.iter().enumerate() {
        p.set_as_oriented_box(
            hw_real::rl!(R; 12.0) / scale,
            hw_real::rl!(R; 12.0) / scale,
            Some(*pos),
            R::zero(),
        );
        let b = new_body!(ctx, &BodyDef::new());
        ctx.character.motorcart.bodies[BACK_SHOCK + i] = b;
        flow!(body::create_shape(ctx, b, &p).0);
        flow!(body::set_mass_from_shapes(ctx, b));
    }
    p.is_sensor = crate::Flag32::FALSE;
    p.filter.group_index = 0;
    p.density = hw_real::rl!(R; 0.25);
    for (i, name) in [
        "box0", "box1", "box2", "box3", "box4", "box5", "box6", "box7", "box8", "box9",
    ]
    .iter()
    .enumerate()
    {
        let v = part!(data, name);
        let mut bd = BodyDef::new();
        bd.position = point(ctx, v);
        bd.angle = R::from_words(v.rotation) / degrees;
        p.set_as_box(
            R::from_words(v.scale_x) * hw_real::rl!(R; 5.0) / scale,
            R::from_words(v.scale_y) * hw_real::rl!(R; 5.0) / scale,
        );
        let b = new_body!(ctx, &bd);
        ctx.character.motorcart.groceries[i] = b;
        flow!(body::create_shape(ctx, b, &p).0);
        flow!(body::set_mass_from_shapes(ctx, b));
    }
    for (i, g) in [(7, 4), (8, 3)] {
        let sh = body::get_shape_list(ctx, ctx.character.motorcart.groceries[g]);
        ctx.character.motorcart.shapes[i] = sh;
        ctx.world.shapes.retain(sh);
        ctx.world.shape_mut(sh).handlers[3] = CHARACTER_HANDLER | 74;
    }
    Flow::Continue
}
pub fn create_joints<R: Real>(ctx: &mut Ctx<R>, data: &CharTable) -> Flow {
    let b = ctx.character.motorcart.bodies;
    let degrees = hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI);
    let angle = body::get_angle(ctx, ctx.character.bodies[HEAD])
        - body::get_angle(ctx, ctx.character.bodies[CHEST]);
    joints::set_limits(
        ctx,
        ctx.character.joints[NECK],
        -hw_real::rl!(R; 10.0) / degrees - angle,
        hw_real::rl!(R; 10.0) / degrees - angle,
    );
    // The source computes new hip limits but never applies them.
    let mut pd = JointDef::prismatic();
    pd.max_motor_force = hw_real::rl!(R; 1000.0);
    pd.enable_limit = crate::Flag32::TRUE;
    for (i, name) in ["backWheelShape", "frontWheelShape"].iter().enumerate() {
        let pos = point(ctx, part!(data, name));
        joints::prismatic_def_initialize(
            ctx,
            &mut pd,
            b[MAIN],
            b[BACK_SHOCK + i],
            pos,
            Vec2::new(R::zero(), R::one()),
        );
        flow!(save_joint(ctx, &pd, i));
    }
    let mut d = JointDef::revolute();
    d.max_motor_torque = hw_real::rl!(R; 100000.0);
    d.enable_limit = crate::Flag32::TRUE;
    let pos = point(ctx, part!(data, "shaftAnchor"));
    joints::revolute_def_initialize(ctx, &mut d, b[MAIN], b[SHAFT], pos);
    flow!(save_joint(ctx, &d, MAIN_SHAFT));
    let pelvis = ctx.character.bodies[PELVIS];
    let pos = body::get_position(ctx, pelvis);
    joints::revolute_def_initialize(ctx, &mut d, b[MAIN], pelvis, pos);
    flow!(save_joint(ctx, &d, PELVIS_J));
    d.enable_limit = crate::Flag32::FALSE;
    for (i, name) in ["backWheelShape", "frontWheelShape"].iter().enumerate() {
        let pos = point(ctx, part!(data, name));
        joints::revolute_def_initialize(ctx, &mut d, b[BACK_SHOCK + i], b[BACK + i], pos);
        flow!(save_joint(ctx, &d, BACK_J + i));
    }
    let pos = point(ctx, part!(data, "handleAnchor"));
    for (i, p) in [(HAND1, LOWER_ARM1), (HAND2, LOWER_ARM2)] {
        joints::revolute_def_initialize(ctx, &mut d, b[MAIN], ctx.character.bodies[p], pos);
        flow!(save_joint(ctx, &d, i));
    }
    for (i, p) in [(LEG1, UPPER_LEG1), (LEG2, UPPER_LEG2)] {
        let limb = ctx.character.bodies[p];
        let pos = body::get_position(ctx, limb);
        joints::revolute_def_initialize(ctx, &mut d, b[MAIN], limb, pos);
        flow!(save_joint(ctx, &d, i));
    }
    for (i, name) in ["sausageAnchor1", "sausageAnchor2"].iter().enumerate() {
        let pos = point(ctx, part!(data, name));
        joints::revolute_def_initialize(
            ctx,
            &mut d,
            ctx.character.motorcart.groceries[7 + i],
            ctx.character.motorcart.groceries[8 + i],
            pos,
        );
        flow!(world::create_joint(ctx, &d).0);
    }
    Flow::Continue
}
