//! IrresponsibleDad helmet and BicycleGuy vehicle damage.
use super::{
    bicycle::{self, *},
    character::*,
    damage,
};
use crate::b2::{
    body,
    math::{atan2, Vec2},
    state::{BodyDef, ShapeDef},
    world,
};
use crate::{flow, flow::AbortSite, Ctx, Flow, NONE};
use hw_data::{GuideKey, GuideLookup};
use hw_real::Real;
macro_rules! part {
    ($data:expr,$name:expr) => {
        match $data.guide_numbers($name) {
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
        b
    }};
}
pub fn helmet_smash<R: Real>(ctx: &mut Ctx<R>, _impulse: R) -> Flow {
    let k = damage::key(ctx.character.bicycle.helmet_shape);
    ctx.character.contact_thresholds[k] = damage::undefined();
    ctx.character.head1_shape = ctx.character.bicycle.helmet_shape;
    let k = damage::key(ctx.character.head1_shape);
    ctx.character.contact_thresholds[k] = ctx.character.smash_limits[0];
    ctx.character.bicycle.helmet_shape = NONE;
    ctx.character.bicycle.helmet_on = crate::Flag32::FALSE;
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
    let Some(data) = super::character::data(ctx) else {
        return Flow::Abort(AbortSite::PortUnimplemented);
    };
    let items = [GuideKey::HelmetVert1, GuideKey::HelmetVert2, GuideKey::HelmetVert3, GuideKey::HelmetVert4];
    for i in 0..items.len() {
        let name = items[i];
        let v = part!(data, name);
        p.vertices[i] = Vec2::new(
            R::from_words(v.x) / ctx.character.scale,
            R::from_words(v.y) / ctx.character.scale,
        );
    }
    let b = new_body!(ctx, &bd);
    ctx.character.bicycle.helmet = b;
    ctx.world.bodies.retain(b);
    flow!(body::create_shape(ctx, b, &p).0);
    flow!(body::set_mass_from_shapes(ctx, b));
    body::set_linear_velocity(ctx, b, body::get_linear_velocity(ctx, head));
    body::set_angular_velocity(ctx, b, body::get_angular_velocity(ctx, head));
    Flow::Continue
}
pub fn wheel_smash<R: Real>(ctx: &mut Ctx<R>, wheel: usize, _impulse: R, normal: Vec2<R>) -> Flow {
    let bike = ctx.character.bicycle;
    let sh = bike.shapes[if wheel == FRONT { 4 } else { 3 }];
    damage::disarm(ctx, sh);
    ctx.world.shape_mut(sh).handlers[1] = 0;
    let b = bike.bodies[wheel];
    flow!(body::destroy_shape(ctx, b, body::get_shape_list(ctx, b)));
    let mut p = ShapeDef::polygon();
    p.density = hw_real::rl!(R; 2.0);
    p.friction = hw_real::rl!(R; 0.3);
    p.restitution = hw_real::rl!(R; 0.1);
    if bike.ejected.get() {
        p.filter = ctx.character.zero_filter;
    } else {
        p.filter.category_bits = 1;
    }
    let angle = atan2(normal.y, normal.x);
    let local = angle - body::get_angle(ctx, b);
    p.set_as_oriented_box(
        hw_real::rl!(R; 25.0) / ctx.character.scale,
        hw_real::rl!(R; 50.0) / ctx.character.scale,
        Some(Vec2::zero()),
        local,
    );
    flow!(body::create_shape(ctx, b, &p).0);
    Flow::Continue
}
pub fn frame_smash<R: Real>(ctx: &mut Ctx<R>, _impulse: R, _normal: Vec2<R>) -> Flow {
    let bike = ctx.character.bicycle;
    let k = damage::key(bike.shapes[0]);
    ctx.character.contact_thresholds[k] = damage::undefined();
    for i in 0..3 {
        let sh = bike.shapes[i];
        ctx.world.shape_mut(sh).handlers[3] = 0;
    }
    for i in 0..3 {
        let sh = bike.shapes[i];
        ctx.world.shape_mut(sh).handlers[0] = 0;
    }
    let items = [4, 3];
    for item_index in 0..items.len() {
        let i = items[item_index];
        ctx.world.shape_mut(bike.shapes[i]).handlers[0] = 0;
        ctx.world.shape_mut(bike.shapes[i]).handlers[1] = 0;
    }
    flow!(bicycle::eject(ctx));
    flow!(world::destroy_joint(ctx, bike.joints[FRONT_J]));
    flow!(world::destroy_joint(ctx, bike.joints[BACK_J]));
    let frame = bike.bodies[FRAME];
    let mut p = ShapeDef::polygon();
    p.density = hw_real::rl!(R; 2.0);
    p.friction = hw_real::rl!(R; 0.3);
    p.restitution = hw_real::rl!(R; 0.1);
    p.filter = ctx.character.zero_filter;
    let mut bd = BodyDef::new();
    bd.position = body::get_position(ctx, frame);
    bd.angle = body::get_angle(ctx, frame);
    // All three bodies precede all three shapes in the original.
    let chunks = [
        new_body!(ctx, &bd),
        new_body!(ctx, &bd),
        new_body!(ctx, &bd),
    ];
    let mut broken_shape = NONE;
    let Some(data) = super::character::data(ctx) else {
        return Flow::Abort(AbortSite::PortUnimplemented);
    };
    let start = ctx.character.start;
    let scale = ctx.character.scale;
    let items = [
        [GuideKey::ForkVert1, GuideKey::ForkVert2, GuideKey::ForkVert3, GuideKey::ForkVert3],
        [GuideKey::BrokenVert1, GuideKey::BrokenVert2, GuideKey::BrokenVert3, GuideKey::BrokenVert4],
        [GuideKey::SeatVert1, GuideKey::SeatVert2, GuideKey::SeatVert3, GuideKey::SeatVert3],
    ];
    for i in 0..items.len() {
        let names = items[i];
        p.vertex_count = if i == 1 { 4 } else { 3 };
        for k in 0..p.vertex_count as usize {
            let v = part!(data, names[k]);
            p.vertices[k] = Vec2::new(
                start.x + R::from_words(v.x) / scale,
                start.y + R::from_words(v.y) / scale,
            );
        }
        let b = chunks[i];
        let (f, sh) = body::create_shape(ctx, b, &p);
        flow!(f);
        if i == 1 {
            broken_shape = sh;
        }
        flow!(body::set_mass_from_shapes(ctx, b));
        body::set_linear_velocity(ctx, b, body::get_linear_velocity(ctx, frame));
        body::set_angular_velocity(ctx, b, body::get_angular_velocity(ctx, frame));
    }
    flow!(world::destroy_body(ctx, frame));
    ctx.world.shape_mut(broken_shape).handlers[0] = CHARACTER_HANDLER | 50;
    Flow::Continue
}
pub fn consume_vehicle<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    let items = [0, 4, 3];
    for item_index in 0..items.len() {
        let i = items[item_index];
        let k = damage::key(ctx.character.bicycle.shapes[i]);
        let r = ctx.character.contact_results[k];
        if !r.present {
            continue;
        }
        let normal = ctx.world.report_cr_normal;
        flow!(if i == 0 {
            frame_smash(ctx, r.impulse, normal)
        } else {
            wheel_smash(ctx, if i == 4 { FRONT } else { BACK }, r.impulse, normal)
        });
        ctx.character.contact_results[k] = damage::BufferedResult::blank();
        if i != 0 {
            ctx.character.contact_adds[k] = crate::Flag32::FALSE;
        }
    }
    Flow::Continue
}
pub fn after_distal_break<R: Real>(ctx: &mut Ctx<R>, joint: usize) -> Flow {
    if ctx.character.bicycle.ejected.get() {
        return Flow::Continue;
    }
    let i = match joint {
        ELBOW1 => HAND1,
        ELBOW2 => HAND2,
        KNEE1 => FOOT1,
        KNEE2 => FOOT2,
        _ => return Flow::Abort(AbortSite::PortUnimplemented),
    };
    let j = ctx.character.bicycle.joints[i];
    if j != NONE {
        flow!(world::destroy_joint(ctx, j));
        ctx.character.bicycle.joints[i] = NONE;
    }
    if ctx.character.bicycle.joints[HAND1] == NONE
        && ctx.character.bicycle.joints[HAND2] == NONE
        && ctx.character.bicycle.joints[FOOT1] == NONE
        && ctx.character.bicycle.joints[FOOT2] == NONE
    {
        flow!(bicycle::eject(ctx));
    }
    Flow::Continue
}
