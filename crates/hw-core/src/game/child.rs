//! ChildSeatKid construction in the second actor slot.
use super::character::{self, *};
use crate::b2::{
    body, joints,
    math::Vec2,
    state::{BodyDef, FilterData, JointDef, ShapeDef},
    world,
};
use crate::{flow, flow::AbortSite, Ctx, Flow, H, NONE};
use hw_data::CharTable;
use hw_real::Real;
pub const SEAT_FRAME: usize = 0;
pub const SEAT_CHEST: usize = 1;
pub const SEAT_PELVIS: usize = 2;
pub const SEAT_LEG1: usize = 3;
pub const SEAT_LEG2: usize = 4;
#[derive(Clone, Copy)]
#[repr(C)]
pub struct ChildState {
    pub seat: H,
    pub seat_shape: H,
    pub joints: [H; 5],
    pub extra_filter: FilterData,
    pub ejected: crate::Flag32,
    pub detached: crate::Flag32,
}
impl ChildState {
    pub fn blank() -> Self {
        Self {
            seat: NONE,
            seat_shape: NONE,
            joints: [NONE; 5],
            extra_filter: FilterData::new(),
            ejected: crate::Flag32::FALSE,
            detached: crate::Flag32::FALSE,
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
fn save_joint<R: Real>(ctx: &mut Ctx<R>, i: usize, d: &JointDef<R>) -> Flow {
    let (f, j) = world::create_joint(ctx, d);
    flow!(f);
    ctx.character.child_seat.joints[i] = j;
    ctx.world.joints.retain(j);
    Flow::Continue
}
pub fn create<R: Real>(ctx: &mut Ctx<R>, data: &CharTable, x: R, y: R) -> Flow {
    let mut data = *data;
    data.guide = data.kid_guide;
    *ctx.character = CharacterState::blank();
    ctx.character.kind = 5;
    ctx.character.start = Vec2::new(x / hw_real::rl!(R; 62.5), y / hw_real::rl!(R; 62.5));
    ctx.character.scale = hw_real::rl!(R; 62.5) * hw_real::rl!(R; 2.0);
    character::create_filters(ctx, -2);
    ctx.character.lower_body_filter.mask_bits = 65535;
    let mut extra = FilterData::new();
    extra.group_index = -2;
    extra.category_bits = 4;
    ctx.character.child_seat.extra_filter = extra;
    flow!(character::create_bodies(ctx, &data));
    flow!(create_bodies(ctx, &data));
    flow!(character::create_joints(ctx, &data));
    flow!(create_joints(ctx, &data));
    character::set_limits(ctx);
    character::create_dictionaries(ctx);
    Flow::Continue
}
pub fn create_bodies<R: Real>(ctx: &mut Ctx<R>, data: &CharTable) -> Flow {
    let mut p = ShapeDef::polygon();
    p.density = R::one();
    p.friction = hw_real::rl!(R; 0.3);
    p.restitution = hw_real::rl!(R; 0.1);
    p.filter = ctx.character.default_filter;
    p.vertex_count = 4;
    let start = ctx.character.start;
    let scale = ctx.character.scale;
    let names = [
        ["seat1Vert1", "seat1Vert2", "seat1Vert3", "seat1Vert4"],
        ["seat2Vert1", "seat2Vert2", "seat2Vert3", "seat2Vert4"],
        ["seat3Vert1", "seat3Vert2", "seat3Vert3", "seat3Vert4"],
    ];
    for (i, row) in names.iter().enumerate() {
        for k in 0..4 {
            let v = part!(data, row[k]);
            p.vertices[k] = Vec2::new(
                start.x + R::from_words(v.x) / scale,
                start.y + R::from_words(v.y) / scale,
            );
        }
        if i == 0 {
            let b = world::create_body(ctx, &BodyDef::new());
            if b == NONE {
                return Flow::Abort(if ctx.world.lock.get() {
                    AbortSite::GameNullDeref
                } else {
                    AbortSite::PortCapacity
                });
            }
            ctx.character.child_seat.seat = b;
            ctx.world.bodies.retain(b);
        }
        if i == 2 {
            p.is_sensor = crate::Flag32::TRUE;
        }
        let (f, sh) = body::create_shape(ctx, ctx.character.child_seat.seat, &p);
        flow!(f);
        if i == 0 {
            ctx.character.child_seat.seat_shape = sh;
            ctx.world.shapes.retain(sh);
            ctx.world.shape_mut(sh).handlers[0] = handler(ctx, 60);
        }
    }
    body::set_mass_from_shapes(ctx, ctx.character.child_seat.seat)
}
pub fn create_joints<R: Real>(ctx: &mut Ctx<R>, data: &CharTable) -> Flow {
    let mut d = JointDef::revolute();
    let v = part!(data, "frameSeatAnchor");
    let start = ctx.character.start;
    let scale = ctx.character.scale;
    d.enable_limit = crate::Flag32::TRUE;
    d.lower_angle = R::zero();
    d.upper_angle = R::zero();
    let pos = Vec2::new(
        start.x + R::from_words(v.x) / scale,
        start.y + R::from_words(v.y) / scale,
    );
    let seat = ctx.character.child_seat.seat;
    joints::revolute_def_initialize(
        ctx,
        &mut d,
        ctx.character.primary.bicycle.bodies[super::bicycle::FRAME],
        seat,
        pos,
    );
    flow!(save_joint(ctx, SEAT_FRAME, &d));
    d.enable_limit = crate::Flag32::FALSE;
    for (i, part) in [
        (SEAT_CHEST, CHEST),
        (SEAT_PELVIS, PELVIS),
        (SEAT_LEG1, UPPER_LEG1),
        (SEAT_LEG2, UPPER_LEG2),
    ] {
        let b = ctx.character.bodies[part];
        let pos = body::get_position(ctx, b);
        joints::revolute_def_initialize(ctx, &mut d, seat, b, pos);
        flow!(save_joint(ctx, i, &d));
    }
    Flow::Continue
}
fn refilter_limb<R: Real>(ctx: &mut Ctx<R>, shape: H) -> Flow {
    if shape == NONE {
        return Flow::Abort(AbortSite::GameNullDeref);
    }
    if ctx.world.shape(shape).filter.mask_bits == ctx.character.default_filter.mask_bits {
        ctx.world.shape_mut(shape).filter = ctx.character.child_seat.extra_filter;
        flow!(world::refilter(ctx, shape));
    }
    Flow::Continue
}
fn refilter_limbs<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    let items = [
        HEAD, PELVIS, CHEST, UPPER_ARM1, UPPER_ARM2, LOWER_ARM1, LOWER_ARM2, UPPER_LEG1,
        UPPER_LEG2, LOWER_LEG1, LOWER_LEG2,
    ];
    for item_index in 0..items.len() {
        let p = items[item_index];
        flow!(refilter_limb(
            ctx,
            body::get_shape_list(ctx, ctx.character.bodies[p])
        ));
    }
    Flow::Continue
}
pub fn eject<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    if ctx.character.child_seat.ejected.get() {
        return Flow::Continue;
    }
    ctx.character.child_seat.ejected = crate::Flag32::TRUE;
    flow!(super::poses::reset_joint_limits(ctx));
    let items = [SEAT_PELVIS, SEAT_CHEST, SEAT_LEG1, SEAT_LEG2];
    for item_index in 0..items.len() {
        let i = items[item_index];
        let j = ctx.character.child_seat.joints[i];
        if i == SEAT_PELVIS || i == SEAT_CHEST || j != NONE {
            flow!(world::destroy_joint(ctx, j));
            ctx.character.child_seat.joints[i] = NONE;
        }
    }
    if !ctx.character.child_seat.detached {
        flow!(refilter_limbs(ctx));
    }
    let mut sh = body::get_shape_list(ctx, ctx.character.child_seat.seat);
    while sh != NONE {
        ctx.world.shape_mut(sh).filter = ctx.character.zero_filter;
        flow!(world::refilter(ctx, sh));
        sh = ctx.world.shape(sh).next;
    }
    Flow::Continue
}
pub fn detach<R: Real>(ctx: &mut Ctx<R>, _impulse: R) -> Flow {
    let j = ctx.character.child_seat.joints[SEAT_FRAME];
    ctx.world.joint_mut(j).broken = crate::Flag32::TRUE;
    ctx.character.child_seat.detached = crate::Flag32::TRUE;
    flow!(world::destroy_joint(ctx, j));
    if !ctx.character.child_seat.ejected {
        flow!(refilter_limbs(ctx));
    }
    let mut sh = body::get_shape_list(ctx, ctx.character.child_seat.seat);
    while sh != NONE {
        if !ctx.character.child_seat.ejected {
            ctx.world.shape_mut(sh).filter = ctx.character.child_seat.extra_filter;
        }
        ctx.world.shape_mut(sh).is_sensor = crate::Flag32::FALSE;
        flow!(world::refilter(ctx, sh));
        sh = ctx.world.shape(sh).next;
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
    if ctx.character.child_seat.ejected.get() {
        if keys.pressed(0) != keys.pressed(1) {
            poses::set_pose(ctx, if keys.pressed(0) { 1 } else { 2 });
        } else if ctx.character.current_pose == 1 || ctx.character.current_pose == 2 {
            poses::set_pose(ctx, 0);
        }
        if keys.pressed(2) != keys.pressed(3) {
            poses::set_pose(ctx, if keys.pressed(2) { 3 } else { 4 });
        } else if ctx.character.current_pose == 3 || ctx.character.current_pose == 4 {
            poses::set_pose(ctx, 0);
        }
        if keys.pressed(4) {
            flow!(poses::start_grab(ctx));
        } else {
            flow!(poses::release_grip(ctx));
        }
    }
    if keys.pressed(6) {
        flow!(eject(ctx));
    }
    if keys.pressed(7) {
        flow!(eject(ctx));
    }
    Flow::Continue
}
pub fn actions<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    flow!(super::poses::wheelchair_pose(ctx));
    super::damage::bleed_out(ctx)
}
