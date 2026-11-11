//! SegwayGuy. Statement and shared-definition mutation order follows the AS3 source.
use super::{
    character::{self, CharacterState, *},
    damage, poses,
};
use crate::b2::{
    body, joints,
    math::Vec2,
    state::{BodyDef, FilterData, JointDef, ShapeDef},
    world,
};
use crate::{flow, flow::AbortSite, Ctx, Flow, H, NONE};
use hw_data::CharTable;
use hw_data::{GuideKey, GuideLookup};
use hw_real::Real;
pub const FRAME: usize = 0;
pub const STAND: usize = 1;
pub const SHOCK: usize = 2;
pub const WHEEL: usize = 3;
pub const SHOCK_J: usize = 0;
pub const WHEEL_J: usize = 1;
pub const STAND_FRAME: usize = 2;
pub const HAND1: usize = 3;
pub const HAND2: usize = 4;
pub const FOOT1: usize = 5;
pub const FOOT2: usize = 6;
#[derive(Clone, Copy)]
#[repr(C)]
pub struct SegwayState<R: Real> {
    pub bodies: [H; 4],
    pub shapes: [H; 3],
    pub joints: [H; 7],
    pub helmet_shape: H,
    pub helmet: H,
    pub ejected: crate::Flag32,
    pub helmet_on: crate::Flag32,
    pub wheel_current_speed: R,
    pub wheel_new_speed: R,
    pub wheel_contacts: R,
    pub jump_translation: R,
}
impl<R: Real> SegwayState<R> {
    pub fn blank() -> Self {
        Self {
            bodies: [NONE; 4],
            shapes: [NONE; 3],
            joints: [NONE; 7],
            helmet_shape: NONE,
            helmet: NONE,
            ejected: crate::Flag32::FALSE,
            helmet_on: crate::Flag32::FALSE,
            wheel_current_speed: damage::undefined(),
            wheel_new_speed: damage::undefined(),
            wheel_contacts: R::zero(),
            jump_translation: hw_real::rl!(R; 30.0) / hw_real::rl!(R; 62.5),
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
    ctx.character.segway.joints[i] = j;
    ctx.world.joints.retain(j);
    Flow::Continue
}
pub fn create<R: Real>(ctx: &mut Ctx<R>, data: &CharTable, x: R, y: R) -> Flow {
    *ctx.character = CharacterState::blank();
    ctx.character.kind = 2;
    ctx.character.start = Vec2::new(x / hw_real::rl!(R; 62.5), y / hw_real::rl!(R; 62.5));
    ctx.character.scale = hw_real::rl!(R; 62.5) * hw_real::rl!(R; 2.0);
    character::create_filters(ctx, -1);
    flow!(character::create_bodies(ctx, data));
    flow!(create_bodies(ctx, data));
    flow!(character::create_joints(ctx, data));
    flow!(create_joints(ctx, data));
    character::set_limits(ctx);
    character::create_dictionaries(ctx);
    ctx.character.segway.helmet_shape = ctx.character.head1_shape;
    ctx.character.head1_shape = NONE;
    let k = damage::key(ctx.character.segway.helmet_shape);
    ctx.character.contact_thresholds[k] = hw_real::rl!(R; 2.0);
    let k = damage::key(ctx.character.segway.shapes[0]);
    ctx.character.contact_thresholds[k] = hw_real::rl!(R; 20.0);
    Flow::Continue
}
pub fn create_bodies<R: Real>(ctx: &mut Ctx<R>, data: &CharTable) -> Flow {
    let start = ctx.character.start;
    let scale = ctx.character.scale;
    let mut p = ShapeDef::polygon();
    p.density = hw_real::rl!(R; 5.0);
    p.friction = hw_real::rl!(R; 0.3);
    p.restitution = hw_real::rl!(R; 0.1);
    p.filter.category_bits = 4;
    let mut c = ShapeDef::circle();
    c.density = hw_real::rl!(R; 5.0);
    c.friction = R::one();
    c.restitution = hw_real::rl!(R; 0.3);
    c.filter.category_bits = 4;
    c.filter.mask_bits = 8;
    let v = part!(data, "frameShape");
    let pos = Vec2::new(
        start.x + R::from_words(v.x) / scale,
        start.y + R::from_words(v.y) / scale,
    );
    let angle = R::from_words(v.rotation) / (hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI));
    p.set_as_oriented_box(
        R::from_words(v.scale_x) * hw_real::rl!(R; 5.0) / scale,
        R::from_words(v.scale_y) * hw_real::rl!(R; 5.0) / scale,
        Some(pos),
        angle,
    );
    let frame = new_body!(ctx, &BodyDef::new());
    ctx.character.segway.bodies[FRAME] = frame;
    let (f, sh) = body::create_shape(ctx, frame, &p);
    flow!(f);
    ctx.character.segway.shapes[0] = sh;
    ctx.world.shapes.retain(sh);
    let v = part!(data, "handleShape");
    let pos = Vec2::new(
        start.x + R::from_words(v.x) / scale,
        start.y + R::from_words(v.y) / scale,
    );
    let angle = R::from_words(v.rotation) / (hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI));
    p.set_as_oriented_box(
        R::from_words(v.scale_x) * hw_real::rl!(R; 5.0) / scale,
        R::from_words(v.scale_y) * hw_real::rl!(R; 5.0) / scale,
        Some(pos),
        angle,
    );
    flow!(body::create_shape(ctx, frame, &p).0);
    flow!(body::set_mass_from_shapes(ctx, frame));
    ctx.world.shape_mut(sh).handlers[3] = CHARACTER_HANDLER | 40;
    let v = part!(data, "standShape");
    let mut bd = BodyDef::new();
    bd.position = Vec2::new(
        start.x + R::from_words(v.x) / scale,
        start.y + R::from_words(v.y) / scale,
    );
    p.set_as_box(
        R::from_words(v.scale_x) * hw_real::rl!(R; 5.0) / scale,
        R::from_words(v.scale_y) * hw_real::rl!(R; 5.0) / scale,
    );
    p.is_sensor = crate::Flag32::TRUE;
    bd.fixed_rotation = crate::Flag32::TRUE;
    let stand = new_body!(ctx, &bd);
    ctx.character.segway.bodies[STAND] = stand;
    let (f, sh) = body::create_shape(ctx, stand, &p);
    flow!(f);
    ctx.character.segway.shapes[1] = sh;
    ctx.world.shapes.retain(sh);
    flow!(body::set_mass_from_shapes(ctx, stand));
    ctx.world.shape_mut(sh).handlers[0] = CHARACTER_HANDLER | 41;
    let shock = new_body!(ctx, &bd);
    ctx.character.segway.bodies[SHOCK] = shock;
    flow!(body::create_shape(ctx, shock, &p).0);
    flow!(body::set_mass_from_shapes(ctx, shock));
    let v = part!(data, "wheelShape");
    bd = BodyDef::new();
    bd.position = Vec2::new(
        start.x + R::from_words(v.x) / scale,
        start.y + R::from_words(v.y) / scale,
    );
    bd.angle = R::from_words(v.rotation) / (hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI));
    c.radius = R::from_words(v.width) / hw_real::rl!(R; 2.0) / scale;
    let wheel = new_body!(ctx, &bd);
    ctx.character.segway.bodies[WHEEL] = wheel;
    let (f, sh) = body::create_shape(ctx, wheel, &c);
    flow!(f);
    ctx.character.segway.shapes[2] = sh;
    ctx.world.shapes.retain(sh);
    flow!(body::set_mass_from_shapes(ctx, wheel));
    let items = [3, 0, 1];
    for item_index in 0..items.len() {
        let event = items[item_index];
        ctx.world.shape_mut(sh).handlers[event] = CHARACTER_HANDLER | 42;
    }
    Flow::Continue
}
pub fn create_joints<R: Real>(ctx: &mut Ctx<R>, data: &CharTable) -> Flow {
    let degrees = hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI);
    let items = [
        (HIP1, UPPER_LEG1, PELVIS, -hw_real::rl!(R; 50.0), hw_real::rl!(R; 10.0)),
        (HIP2, UPPER_LEG2, PELVIS, -hw_real::rl!(R; 50.0), hw_real::rl!(R; 10.0)),
        (ELBOW1, LOWER_ARM1, UPPER_ARM1, -hw_real::rl!(R; 60.0), hw_real::rl!(R; 0.0)),
        (ELBOW2, LOWER_ARM2, UPPER_ARM2, -hw_real::rl!(R; 60.0), hw_real::rl!(R; 0.0)),
        (NECK, HEAD, CHEST, hw_real::rl!(R; 0.0), hw_real::rl!(R; 20.0)),
    ];
    for item_index in 0..items.len() {
        let (j, a, b, lo, hi) = items[item_index];
        let angle = body::get_angle(ctx, ctx.character.bodies[a])
            - body::get_angle(ctx, ctx.character.bodies[b]);
        joints::set_limits(
            ctx,
            ctx.character.joints[j],
            lo / degrees - angle,
            hi / degrees - angle,
        );
    }
    let sg = ctx.character.segway;
    let start = ctx.character.start;
    let scale = ctx.character.scale;
    let v = part!(data, "wheelShape");
    let pos = Vec2::new(
        start.x + R::from_words(v.x) / scale,
        start.y + R::from_words(v.y) / scale,
    );
    let mut d = JointDef::prismatic();
    d.max_motor_force = hw_real::rl!(R; 1000.0);
    d.enable_limit = crate::Flag32::TRUE;
    d.upper_translation = R::zero();
    d.lower_translation = R::zero();
    joints::prismatic_def_initialize(
        ctx,
        &mut d,
        sg.bodies[STAND],
        sg.bodies[SHOCK],
        pos,
        Vec2::new(R::zero(), R::one()),
    );
    flow!(save_joint(ctx, SHOCK_J, &d));
    d = JointDef::revolute();
    d.max_motor_torque = hw_real::rl!(R; 20.0);
    joints::revolute_def_initialize(ctx, &mut d, sg.bodies[SHOCK], sg.bodies[WHEEL], pos);
    flow!(save_joint(ctx, WHEEL_J, &d));
    let v = part!(data, "frameAnchor");
    let pos = Vec2::new(
        start.x + R::from_words(v.x) / scale,
        start.y + R::from_words(v.y) / scale,
    );
    d.enable_limit = crate::Flag32::TRUE;
    d.lower_angle = -hw_real::rl!(R; 15.0) / degrees;
    d.upper_angle = hw_real::rl!(R; 15.0) / degrees;
    joints::revolute_def_initialize(ctx, &mut d, sg.bodies[STAND], sg.bodies[FRAME], pos);
    flow!(save_joint(ctx, STAND_FRAME, &d));
    d.lower_angle = -hw_real::rl!(R; 100.0) / degrees;
    d.upper_angle = hw_real::rl!(R; 10.0) / degrees;
    let v = part!(data, "handleAnchor");
    let pos = Vec2::new(
        start.x + R::from_words(v.x) / scale,
        start.y + R::from_words(v.y) / scale,
    );
    let items = [(HAND1, LOWER_ARM1), (HAND2, LOWER_ARM2)];
    for item_index in 0..items.len() {
        let (i, part) = items[item_index];
        joints::revolute_def_initialize(
            ctx,
            &mut d,
            sg.bodies[FRAME],
            ctx.character.bodies[part],
            pos,
        );
        flow!(save_joint(ctx, i, &d));
    }
    d.lower_angle = -hw_real::rl!(R; 10.0) / degrees;
    d.upper_angle = hw_real::rl!(R; 10.0) / degrees;
    let v = part!(data, "footAnchor");
    let pos = Vec2::new(
        start.x + R::from_words(v.x) / scale,
        start.y + R::from_words(v.y) / scale,
    );
    let items = [(FOOT1, LOWER_LEG1), (FOOT2, LOWER_LEG2)];
    for item_index in 0..items.len() {
        let (i, part) = items[item_index];
        joints::revolute_def_initialize(
            ctx,
            &mut d,
            sg.bodies[FRAME],
            ctx.character.bodies[part],
            pos,
        );
        flow!(save_joint(ctx, i, &d));
    }
    Flow::Continue
}

pub fn eject<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    if ctx.character.segway.ejected.get() {
        return Flow::Continue;
    }
    ctx.character.segway.ejected = crate::Flag32::TRUE;
    flow!(poses::reset_joint_limits(ctx));
    let items = [HAND1, HAND2, FOOT1, FOOT2];
    for item_index in 0..items.len() {
        let i = items[item_index];
        let j = ctx.character.segway.joints[i];
        if j != NONE {
            flow!(world::destroy_joint(ctx, j));
            ctx.character.segway.joints[i] = NONE;
        }
    }
    let j = ctx.character.segway.joints[SHOCK_J];
    joints::enable_motor(ctx, j, false);
    joints::set_limits(ctx, j, R::zero(), R::zero());
    joints::set_motor_speed(ctx, j, R::zero());
    let mut f = FilterData::new();
    f.category_bits = 4;
    f.group_index = -2;
    let mut sh = body::get_shape_list(ctx, ctx.character.segway.bodies[FRAME]);
    while sh != NONE {
        ctx.world.shape_mut(sh).filter = f;
        flow!(world::refilter(ctx, sh));
        sh = ctx.world.shape(sh).next;
    }
    let sh = body::get_shape_list(ctx, ctx.character.segway.bodies[WHEEL]);
    ctx.world.shape_mut(sh).filter = f;
    world::refilter(ctx, sh)
}
fn jump<R: Real>(ctx: &mut Ctx<R>, pressed: bool) -> Flow {
    if ctx.character.segway.ejected.get() {
        return if pressed {
            poses::start_grab(ctx)
        } else {
            poses::release_grip(ctx)
        };
    }
    let j = ctx.character.segway.joints[SHOCK_J];
    if pressed && !joints::is_motor_enabled(ctx, j) {
        joints::set_motor_speed(ctx, j, hw_real::rl!(R; 7.0));
        joints::set_limits(ctx, j, R::zero(), ctx.character.segway.jump_translation);
        joints::enable_motor(ctx, j, true);
    } else if joints::is_motor_enabled(ctx, j) {
        let speed = joints::get_motor_speed(ctx, j);
        if speed.gt(R::zero()) {
            if joints::get_joint_translation(ctx, j).gt(ctx.character.segway.jump_translation) {
                joints::set_motor_speed(ctx, j, -R::one());
            }
        } else if speed.lt(R::zero()) && joints::get_joint_translation(ctx, j).lt(R::zero()) {
            joints::enable_motor(ctx, j, false);
            joints::set_limits(ctx, j, R::zero(), R::zero());
            joints::set_motor_speed(ctx, j, R::zero());
        }
    }
    Flow::Continue
}
/// Compatibility entry for existing CPU callers.
#[inline]
pub fn input<R: Real>(ctx: &mut Ctx<R>, keys: [bool; 8]) -> Flow {
    input_bits(ctx, crate::Input::from_bools(keys))
}
pub fn input_bits<R: Real>(ctx: &mut Ctx<R>, keys: crate::Input) -> Flow {
    let ejected = ctx.character.segway.ejected;
    if keys.pressed(0) != keys.pressed(1) {
        if ejected.get() {
            poses::set_pose(ctx, if keys.pressed(0) { 1 } else { 2 });
        } else {
            flow!(poses::set_joint(
                ctx,
                ctx.character.segway.joints[STAND_FRAME],
                if keys.pressed(0) { hw_real::rl!(R; 0.0) } else { hw_real::rl!(R; 0.52) },
                hw_real::rl!(R; 20.0),
                hw_real::rl!(R; 10.0)
            ));
        }
    } else if ejected.get() {
        if ctx.character.current_pose == 1 || ctx.character.current_pose == 2 {
            poses::set_pose(ctx, 0);
        }
    } else {
        flow!(poses::set_joint(
            ctx,
            ctx.character.segway.joints[STAND_FRAME],
            hw_real::rl!(R; 0.26),
            hw_real::rl!(R; 20.0),
            hw_real::rl!(R; 10.0)
        ));
    }
    let j = ctx.character.segway.joints[WHEEL_J];
    if keys.pressed(2) != keys.pressed(3) {
        if ejected.get() {
            poses::set_pose(ctx, if keys.pressed(2) { 3 } else { 4 });
        } else {
            if !joints::is_motor_enabled(ctx, j) {
                joints::enable_motor(ctx, j, true);
            }
            let speed = joints::get_joint_speed(ctx, j);
            ctx.character.segway.wheel_current_speed = speed;
            let next = if keys.pressed(2) {
                if speed.lt(hw_real::rl!(R; 40.0)) {
                    speed + R::one()
                } else {
                    speed
                }
            } else if speed.gt(-hw_real::rl!(R; 40.0)) {
                speed - R::one()
            } else {
                speed
            };
            ctx.character.segway.wheel_new_speed = next;
            joints::set_motor_speed(ctx, j, next);
        }
    } else {
        if joints::is_motor_enabled(ctx, j) {
            joints::enable_motor(ctx, j, false);
        }
        if ctx.character.current_pose == 3 || ctx.character.current_pose == 4 {
            poses::set_pose(ctx, 0);
        }
    }
    flow!(jump(ctx, keys.pressed(4)));
    if keys.pressed(5) {
        poses::set_pose(ctx, if ejected.get() { 7 } else { 5 });
    } else if ctx.character.current_pose == 5 || ctx.character.current_pose == 7 {
        poses::set_pose(ctx, 0);
    }
    if keys.pressed(6) {
        poses::set_pose(ctx, if ejected.get() { 8 } else { 6 });
    } else if ctx.character.current_pose == 6 || ctx.character.current_pose == 8 {
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
        flow!(poses::wheelchair_pose(ctx));
    } else if pose == 5 || pose == 6 {
        let items = [KNEE1, KNEE2];
        for item_index in 0..items.len() {
            let i = items[item_index];
            flow!(poses::set_joint(
                ctx,
                ctx.character.joints[i],
                if pose == 5 { hw_real::rl!(R; 0.0) } else { hw_real::rl!(R; 1.5) },
                hw_real::rl!(R; 10.0),
                hw_real::rl!(R; 10.0)
            ));
        }
    } else if pose == 7 || pose == 8 {
        let row = if pose == 7 {
            [
                (hw_real::rl!(R; 0.5), hw_real::rl!(R; 2.0)),
                (hw_real::rl!(R; 1.5), hw_real::rl!(R; 2.0)),
                (hw_real::rl!(R; 3.5), hw_real::rl!(R; 2.0)),
                (hw_real::rl!(R; 1.7), hw_real::rl!(R; 10.0)),
                (hw_real::rl!(R; 0.0), hw_real::rl!(R; 10.0)),
                (hw_real::rl!(R; 3.0), hw_real::rl!(R; 20.0)),
                (hw_real::rl!(R; 1.0), hw_real::rl!(R; 20.0)),
                (hw_real::rl!(R; 1.5), hw_real::rl!(R; 15.0)),
                (hw_real::rl!(R; 3.0), hw_real::rl!(R; 15.0)),
            ]
        } else {
            [
                (hw_real::rl!(R; 0.5), hw_real::rl!(R; 2.0)),
                (hw_real::rl!(R; 3.5), hw_real::rl!(R; 2.0)),
                (hw_real::rl!(R; 1.5), hw_real::rl!(R; 2.0)),
                (hw_real::rl!(R; 0.0), hw_real::rl!(R; 10.0)),
                (hw_real::rl!(R; 1.7), hw_real::rl!(R; 10.0)),
                (hw_real::rl!(R; 1.0), hw_real::rl!(R; 20.0)),
                (hw_real::rl!(R; 3.0), hw_real::rl!(R; 20.0)),
                (hw_real::rl!(R; 3.0), hw_real::rl!(R; 15.0)),
                (hw_real::rl!(R; 1.5), hw_real::rl!(R; 15.0)),
            ]
        };
        let items = [
            NECK, HIP1, HIP2, KNEE1, KNEE2, SHOULDER1, SHOULDER2, ELBOW1, ELBOW2,
        ];
        for i in 0..items.len() {
            let j = items[i];
            flow!(poses::set_joint(
                ctx,
                ctx.character.joints[j],
                row[i].0,
                row[i].1,
                hw_real::rl!(R; 10.0)
            ));
        }
    }
    damage::bleed_out(ctx)
}

pub fn helmet_smash<R: Real>(ctx: &mut Ctx<R>, _impulse: R) -> Flow {
    let k = damage::key(ctx.character.segway.helmet_shape);
    ctx.character.contact_thresholds[k] = damage::undefined();
    ctx.character.head1_shape = ctx.character.segway.helmet_shape;
    let k = damage::key(ctx.character.head1_shape);
    ctx.character.contact_thresholds[k] = ctx.character.smash_limits[0];
    ctx.character.segway.helmet_shape = NONE;
    ctx.character.segway.helmet_on = crate::Flag32::FALSE;
    let mut p = ShapeDef::polygon();
    p.density = R::one();
    p.friction = hw_real::rl!(R; 0.3);
    p.restitution = hw_real::rl!(R; 0.1);
    p.filter = ctx.character.zero_filter;
    let head = ctx.character.bodies[HEAD];
    let mut bd = BodyDef::new();
    bd.position = body::get_position(ctx, head);
    bd.angle = body::get_angle(ctx, head);
    p.vertex_count = 4;
    let Some(data) = hw_data::RuntimeGuide::new(ctx.character.data_index, false) else {
        return Flow::Abort(AbortSite::PortUnimplemented);
    };
    let items = [GuideKey::HelmetVert1, GuideKey::HelmetVert2, GuideKey::HelmetVert3, GuideKey::HelmetVert4];
    for i in 0..items.len() {
        let name = items[i];
        let Some(v) = data.guide_numbers(name) else {
            return Flow::Abort(AbortSite::PortUnimplemented);
        };
        p.vertices[i] = Vec2::new(
            R::from_words(v.x) / ctx.character.scale,
            R::from_words(v.y) / ctx.character.scale,
        );
    }
    let b = new_body!(ctx, &bd);
    ctx.character.segway.helmet = b;
    flow!(body::create_shape(ctx, b, &p).0);
    flow!(body::set_mass_from_shapes(ctx, b));
    body::set_linear_velocity(ctx, b, body::get_linear_velocity(ctx, head));
    body::set_angular_velocity(ctx, b, body::get_angular_velocity(ctx, head));
    Flow::Continue
}
pub fn frame_smash<R: Real>(ctx: &mut Ctx<R>, _impulse: R) -> Flow {
    let sg = ctx.character.segway;
    ctx.character.contact_thresholds[damage::key(sg.shapes[0])] = damage::undefined();
    ctx.world.shape_mut(sg.shapes[0]).handlers[3] = 0;
    ctx.world.shape_mut(sg.shapes[1]).handlers[0] = 0;
    flow!(world::destroy_joint(ctx, sg.joints[STAND_FRAME]));
    ctx.character.segway.joints[STAND_FRAME] = NONE;
    flow!(eject(ctx));
    let mut sh = body::get_shape_list(ctx, sg.bodies[FRAME]);
    while sh != NONE {
        ctx.world.shape_mut(sh).filter = ctx.character.zero_filter;
        flow!(world::refilter(ctx, sh));
        sh = ctx.world.shape(sh).next;
    }
    let sh = body::get_shape_list(ctx, sg.bodies[WHEEL]);
    ctx.world.shape_mut(sh).filter = ctx.character.zero_filter;
    world::refilter(ctx, sh)
}
pub fn after_distal_break<R: Real>(ctx: &mut Ctx<R>, joint: usize) -> Flow {
    if joint != KNEE2 && ctx.character.segway.ejected.get() {
        return Flow::Continue;
    }
    let i = match joint {
        ELBOW1 => HAND1,
        ELBOW2 => HAND2,
        KNEE1 => FOOT1,
        KNEE2 => FOOT2,
        _ => return Flow::Abort(AbortSite::PortUnimplemented),
    };
    let j = ctx.character.segway.joints[i];
    if j != NONE {
        flow!(world::destroy_joint(ctx, j));
        ctx.character.segway.joints[i] = NONE;
        if (joint == ELBOW1 && ctx.character.segway.joints[HAND2] == NONE)
            || (joint == ELBOW2 && ctx.character.segway.joints[HAND1] == NONE)
        {
            flow!(eject(ctx));
        }
    }
    Flow::Continue
}
