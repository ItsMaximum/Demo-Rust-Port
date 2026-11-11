//! Ligament, SpinalChord and IntestineChain. The ligament's dy=y-y and live-joint
//! body reassignment are source behavior; neither is repaired here.
use super::character::{CHEST, HEAD, PELVIS};
use crate::b2::{
    body, joints,
    math::{sin_cos, Vec2},
    state::{BodyDef, FilterData, JointDef, ShapeDef},
    world,
};
use crate::{flow, flow::AbortSite, Ctx, Flow, H, NONE};
use hw_data::{GuideKey, GuideLookup};
use hw_real::{as3, Real};

pub const MAX_COMPOSITES: usize = 32;
pub const MAX_NODES: usize = 16;
pub const LIGAMENT: u32 = 0;
pub const SPINE: u32 = 1;
pub const INTESTINE: u32 = 2;
#[derive(Clone, Copy)]
#[repr(C)]
pub struct Composite<R: Real> {
    pub kind: u32,
    pub upper: H,
    pub ends: [H; 2],
    pub scale: R,
}
impl<R: Real> Composite<R> {
    pub fn blank() -> Self {
        Self {
            kind: LIGAMENT,
            upper: NONE,
            ends: [NONE; 2],
            scale: R::zero(),
        }
    }
}
macro_rules! create_joint {
    ($ctx:expr,$d:expr) => {{
        let (f, j) = world::create_joint($ctx, $d);
        flow!(f);
        if j == NONE {
            return Flow::Abort(AbortSite::PortCapacity);
        }
        j
    }};
}

fn polygon_end<R: Real>(ctx: &Ctx<R>, b: H, vertex: usize) -> Vec2<R> {
    let height = ctx.world.shape(body::get_shape_list(ctx, b)).vertices[vertex].y;
    let angle = body::get_angle(ctx, b) + hw_real::rl!(R; core::f64::consts::PI) / hw_real::rl!(R; 2.0);
    Vec2::new(
        body::get_position(ctx, b).x + sin_cos(angle).0 * height,
        body::get_position(ctx, b).y + sin_cos(angle).1 * height,
    )
}

fn append<R: Real>(ctx: &mut Ctx<R>, value: Composite<R>, out: &mut H) -> Flow {
    let at = ctx.character.composite_count;
    if at == (MAX_COMPOSITES as u32) {
        return Flow::Abort(AbortSite::PortCapacity);
    }
    ctx.character.composites[at as usize] = value;
    ctx.character.composite_count += 1;
    *out = at as H;
    Flow::Continue
}

/// Fixed-rotation circle nodes and distance joints, preserving the source's creation order.
fn chain<R: Real>(
    ctx: &mut Ctx<R>,
    kind: u32,
    first: H,
    last: H,
    start: Vec2<R>,
    end: Vec2<R>,
    velocity_body: H,
    count: i32,
    length: R,
    scale: R,
    out: &mut H,
) -> Flow {
    if count < 1 {
        return Flow::Abort(AbortSite::GameNullDeref);
    }
    if count as usize > MAX_NODES || ctx.character.composite_count == (MAX_COMPOSITES as u32) {
        return Flow::Abort(AbortSite::PortCapacity);
    }
    let mut bd = BodyDef::new();
    let mut circle = ShapeDef::circle();
    circle.density = R::one();
    circle.friction = hw_real::rl!(R; 0.3);
    circle.restitution = hw_real::rl!(R; 0.1);
    circle.radius = if kind == LIGAMENT {
        hw_real::rl!(R; 7.0) / scale
    } else {
        hw_real::rl!(R; 5.0) / scale
    };
    circle.filter = if kind == LIGAMENT {
        let mut f = FilterData::new();
        f.category_bits = 8;
        f.mask_bits = 8;
        f
    } else {
        ctx.character.zero_filter
    };
    let dx = end.x - start.x;
    let dy = if kind == LIGAMENT {
        end.y - end.y
    } else {
        end.y - start.y
    };
    let step_x = dx / (R::from_i32(count) + R::one());
    let step_y = dy / (R::from_i32(count) + R::one());
    let mut nodes = [NONE; MAX_NODES];
    for i in 1..=count {
        bd.position = Vec2::new(
            start.x + step_x * R::from_i32(i),
            start.y + step_y * R::from_i32(i),
        );
        bd.fixed_rotation = crate::Flag32::TRUE;
        let b = world::create_body(ctx, &bd);
        if b == NONE {
            return Flow::Abort(AbortSite::PortCapacity);
        }
        flow!(body::create_shape(ctx, b, &circle).0);
        flow!(body::set_mass_from_shapes(ctx, b));
        body::set_linear_velocity(ctx, b, body::get_linear_velocity(ctx, velocity_body));
        nodes[(i - 1) as usize] = b;
    }
    let mut d = JointDef::distance();
    let distance = length / scale;
    joints::distance_def_initialize(
        ctx,
        &mut d,
        first,
        nodes[0],
        start,
        body::get_position(ctx, nodes[0]),
    );
    d.length = distance;
    d.collide_connected = crate::Flag32::FALSE;
    let first_joint = create_joint!(ctx, &d);
    for i in 0..count as usize - 1 {
        joints::distance_def_initialize(
            ctx,
            &mut d,
            nodes[i],
            nodes[i + 1],
            body::get_position(ctx, nodes[i]),
            body::get_position(ctx, nodes[i + 1]),
        );
        d.length = distance;
        d.collide_connected = crate::Flag32::FALSE;
        create_joint!(ctx, &d);
    }
    let b = nodes[count as usize - 1];
    joints::distance_def_initialize(ctx, &mut d, b, last, body::get_position(ctx, b), end);
    d.length = distance;
    d.collide_connected = crate::Flag32::FALSE;
    let last_joint = create_joint!(ctx, &d);
    append(
        ctx,
        Composite {
            kind,
            upper: first,
            ends: [first_joint, last_joint],
            scale,
        },
        out,
    )
}

pub fn ligament<R: Real>(
    ctx: &mut Ctx<R>,
    upper: H,
    lower: H,
    length: R,
    scale: R,
    out: &mut H,
) -> Flow {
    let start = polygon_end(ctx, upper, 2);
    let end = polygon_end(ctx, lower, 0);
    chain(
        ctx, LIGAMENT, upper, lower, start, end, upper, 2, length, scale, out,
    )
}
pub fn intestine<R: Real>(ctx: &mut Ctx<R>, count: i32, length: R, out: &mut H) -> Flow {
    let chest = ctx.character.bodies[CHEST];
    let pelvis = ctx.character.bodies[PELVIS];
    let start = polygon_end(ctx, pelvis, 0);
    let end = polygon_end(ctx, chest, 2);
    chain(
        ctx,
        INTESTINE,
        pelvis,
        chest,
        start,
        end,
        chest,
        count,
        length,
        ctx.character.scale,
        out,
    )
}
pub fn spine<R: Real>(
    ctx: &mut Ctx<R>,
    data: &impl GuideLookup,
    count: i32,
    length: R,
    out: &mut H,
) -> Flow {
    let chest = ctx.character.bodies[CHEST];
    let head = ctx.character.bodies[HEAD];
    let scale = ctx.character.scale;
    let radius = ctx.world.shape(body::get_shape_list(ctx, head)).radius;
    let _unused_angle = body::get_angle(ctx, head) + hw_real::rl!(R; core::f64::consts::PI) / hw_real::rl!(R; 2.0);
    let end = body::get_world_point(ctx, head, Vec2::new(R::zero(), radius));
    let Some(p) = data.guide_numbers(GuideKey::SpineAnchor) else {
        return Flow::Abort(AbortSite::PortUnimplemented);
    };
    let start = body::get_world_point(
        ctx,
        chest,
        Vec2::new(R::from_words(p.x) / scale, R::from_words(p.y) / scale),
    );
    chain(
        ctx, SPINE, chest, head, start, end, head, count, length, scale, out,
    )
}

pub fn break_end<R: Real>(ctx: &mut Ctx<R>, composite: H, end: usize) -> Flow {
    let j = ctx.character.composites[composite as usize].ends[end];
    if j == NONE {
        return Flow::Continue;
    }
    ctx.world.joint_mut(j).broken = crate::Flag32::TRUE;
    flow!(world::destroy_joint(ctx, j));
    ctx.character.composites[composite as usize].ends[end] = NONE;
    Flow::Continue
}

fn tear_ligament<R: Real>(ctx: &mut Ctx<R>, composite: usize) -> Flow {
    let c = ctx.character.composites[composite];
    let j = c.ends[0];
    let mut bd = BodyDef::new();
    let mut circle = ShapeDef::circle();
    circle.density = R::one();
    circle.friction = hw_real::rl!(R; 0.3);
    circle.restitution = hw_real::rl!(R; 0.1);
    circle.radius = hw_real::rl!(R; 3.0) / c.scale;
    circle.filter.category_bits = 8;
    circle.filter.mask_bits = 8;
    let anchor = joints::get_anchor1(ctx, j);
    bd.position = anchor;
    bd.fixed_rotation = crate::Flag32::TRUE;
    let b = world::create_body(ctx, &bd);
    if b == NONE {
        return Flow::Abort(AbortSite::PortCapacity);
    }
    flow!(body::create_shape(ctx, b, &circle).0);
    flow!(body::set_mass_from_shapes(ctx, b));
    body::set_linear_velocity(ctx, b, body::get_linear_velocity(ctx, c.upper));
    // Deliberately no edge-list relinking: source assigns m_body1 directly.
    ctx.world.joint_mut(j).broken = crate::Flag32::TRUE;
    ctx.world.joint_mut(j).body1 = b;
    ctx.world.joint_mut(j).local_anchor1 = Vec2::zero();
    Flow::Continue
}
pub fn check_joints<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    let mut i = 0;
    while i < ctx.character.composite_count {
        let c = ctx.character.composites[i as usize];
        if c.kind == LIGAMENT {
            if !ctx.world.joint(c.ends[0]).broken
                && as3::math_abs(ctx.world.joint(c.ends[0]).impulse).gt(hw_real::rl!(R; 0.5))
            {
                flow!(tear_ligament(ctx, i as usize));
            }
        } else {
            for end in 0..2 {
                let j = ctx.character.composites[i as usize].ends[end];
                if j != NONE && !ctx.world.joint(j).broken {
                    let force = as3::math_abs(ctx.world.joint(j).impulse);
                    let limit = if c.kind == SPINE { hw_real::rl!(R; 0.5) } else { hw_real::rl!(R; 1.5) };
                    if force.gt(limit) {
                        flow!(break_end(ctx, i as H, end));
                    }
                }
            }
        }
        i += 1;
    }
    Flow::Continue
}
