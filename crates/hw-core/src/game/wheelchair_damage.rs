//! WheelChairGuy damage. The original intentionally omits mass recomputation after
//! replacing the chair and large-wheel shapes.
use super::{character::CHARACTER_HANDLER, damage, wheelchair};
use crate::b2::{
    body,
    math::{atan2, sin_cos, Vec2},
    state::{BodyDef, ShapeDef},
    world,
};
use crate::{flow, flow::AbortSite, Ctx, Flow, H, NONE};
use hw_real::{as3, Real};
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

fn register<R: Real>(ctx: &mut Ctx<R>, shape: H, tag: u32) {
    ctx.world.shapes.retain(shape);
    ctx.world.shape_mut(shape).handlers[0] = CHARACTER_HANDLER | tag;
    ctx.world.shape_mut(shape).handlers[3] = CHARACTER_HANDLER | tag;
}
pub fn chair_smash<R: Real>(ctx: &mut Ctx<R>, _impulse: R, _normal: Vec2<R>) -> Flow {
    let wc = ctx.character.wheelchair;
    let chair = wc.chair;
    let scale = ctx.character.scale;
    ctx.character.contact_thresholds[damage::key(wc.shapes[0])] = damage::undefined();
    for i in 0..3 {
        let sh = wc.shapes[i];
        ctx.world.shape_mut(sh).handlers[3] = 0;
    }
    for i in 0..3 {
        let sh = wc.shapes[i];
        ctx.world.shape_mut(sh).handlers[0] = 0;
    }
    ctx.character.wheelchair.chair_smashed = crate::Flag32::TRUE;
    flow!(world::destroy_joint(ctx, wc.joints[0]));
    flow!(world::destroy_joint(ctx, wc.joints[1]));
    let mut sh = body::get_shape_list(ctx, chair);
    while sh != NONE {
        flow!(body::destroy_shape(ctx, chair, sh));
        sh = body::get_shape_list(ctx, chair);
    }
    let mut p = ShapeDef::polygon();
    p.density = hw_real::rl!(R; 2.0);
    p.friction = hw_real::rl!(R; 0.3);
    p.restitution = hw_real::rl!(R; 0.1);
    p.filter = ctx.character.zero_filter;
    let center = body::get_local_center(ctx, chair);
    p.set_as_oriented_box(
        hw_real::rl!(R; 40.0) / scale,
        hw_real::rl!(R; 45.0) / scale,
        Some(Vec2::new(
            center.x + hw_real::rl!(R; 8.0) / scale,
            center.y + hw_real::rl!(R; 2.0) / scale,
        )),
        R::zero(),
    );
    let (f, sh) = body::create_shape(ctx, chair, &p);
    flow!(f);
    ctx.world.shape_mut(sh).handlers[0] = CHARACTER_HANDLER | 32;
    let mut bd = BodyDef::new();
    let handle = new_body!(ctx, &bd);
    let mut angle = body::get_angle(ctx, chair);
    p.set_as_oriented_box(
        hw_real::rl!(R; 5.0) / scale,
        hw_real::rl!(R; 35.0) / scale,
        Some(body::get_world_point(
            ctx,
            chair,
            Vec2::new(
                center.x - hw_real::rl!(R; 30.0) / scale,
                center.y - hw_real::rl!(R; 55.0) / scale,
            ),
        )),
        angle,
    );
    flow!(body::create_shape(ctx, handle, &p).0);
    flow!(body::set_mass_from_shapes(ctx, handle));
    let velocity = body::get_linear_velocity(ctx, chair);
    let angular = body::get_angular_velocity(ctx, chair);
    body::set_angular_velocity(ctx, handle, angular);
    body::set_linear_velocity(ctx, handle, velocity);
    bd.position = body::get_world_point(
        ctx,
        chair,
        Vec2::new(
            center.x + hw_real::rl!(R; 8.0) / scale,
            center.y + hw_real::rl!(R; 14.0) / scale,
        ),
    );
    bd.angle = angle;
    let fuel = new_body!(ctx, &bd);
    ctx.character.wheelchair.fuel = fuel;
    ctx.world.bodies.retain(fuel);
    p.set_as_box(hw_real::rl!(R; 30.0) / scale, hw_real::rl!(R; 9.0) / scale);
    let (f, sh) = body::create_shape(ctx, fuel, &p);
    flow!(f);
    ctx.character.wheelchair.fuel_shape = sh;
    register(ctx, sh, 35);
    flow!(body::set_mass_from_shapes(ctx, fuel));
    body::set_angular_velocity(ctx, fuel, angular);
    body::set_linear_velocity(ctx, fuel, velocity);
    ctx.character.contact_thresholds[damage::key(sh)] = hw_real::rl!(R; 30.0);
    bd = BodyDef::new();
    let jet = new_body!(ctx, &bd);
    ctx.character.wheelchair.jet = jet;
    ctx.world.bodies.retain(jet);
    angle = body::get_angle(ctx, chair) + wc.jet_angle;
    p.set_as_oriented_box(
        hw_real::rl!(R; 21.0) / scale,
        hw_real::rl!(R; 16.5) / scale,
        Some(body::get_world_center(ctx, wc.wheels[0])),
        angle,
    );
    let (f, sh) = body::create_shape(ctx, jet, &p);
    flow!(f);
    ctx.character.wheelchair.jet_shape = sh;
    register(ctx, sh, 34);
    flow!(body::set_mass_from_shapes(ctx, jet));
    body::set_angular_velocity(ctx, jet, angular);
    body::set_linear_velocity(ctx, jet, velocity);
    ctx.character.contact_thresholds[damage::key(sh)] = hw_real::rl!(R; 30.0);
    if ctx.character.wheelchair.firing.get() {
        ctx.character.wheelchair.firing = crate::Flag32::FALSE;
    }
    wheelchair::eject(ctx)
}

pub fn wheel_smash<R: Real>(ctx: &mut Ctx<R>, _impulse: R, normal: Vec2<R>) -> Flow {
    let wc = ctx.character.wheelchair;
    damage::disarm(ctx, wc.shapes[3]);
    ctx.world.shape_mut(wc.shapes[3]).handlers[1] = 0;
    let wheel = wc.wheels[0];
    flow!(body::destroy_shape(
        ctx,
        wheel,
        body::get_shape_list(ctx, wheel)
    ));
    let mut p = ShapeDef::polygon();
    p.density = hw_real::rl!(R; 2.0);
    p.friction = hw_real::rl!(R; 0.3);
    p.restitution = hw_real::rl!(R; 0.1);
    if wc.ejected.get() {
        p.filter = ctx.character.zero_filter;
    } else {
        p.filter.category_bits = 1;
    }
    let angle = atan2(normal.y, normal.x);
    let local = angle - body::get_angle(ctx, wheel);
    p.set_as_oriented_box(
        hw_real::rl!(R; 15.0) / ctx.character.scale,
        hw_real::rl!(R; 50.0) / ctx.character.scale,
        Some(Vec2::zero()),
        local,
    );
    flow!(body::create_shape(ctx, wheel, &p).0);
    ctx.character.wheelchair.wheel_current_speed = R::zero();
    Flow::Continue
}
pub fn jet_smash<R: Real>(ctx: &mut Ctx<R>, _impulse: R) -> Flow {
    damage::disarm(ctx, ctx.character.wheelchair.jet_shape);
    world::destroy_body(ctx, ctx.character.wheelchair.jet)
}
pub fn fuel_smash<R: Real>(ctx: &mut Ctx<R>, _impulse: R) -> Flow {
    damage::disarm(ctx, ctx.character.wheelchair.fuel_shape);
    let fuel = ctx.character.wheelchair.fuel;
    let radius = hw_real::rl!(R; 2.0);
    let center = body::get_world_center(ctx, fuel);
    let mut out = [NONE; 30];
    let (query_flow, count) = world::query(
        ctx,
        Vec2::new(center.x - radius, center.y - radius),
        Vec2::new(center.x + radius, center.y + radius),
        &mut out,
        30,
    );
    flow!(query_flow);
    for i in 0..count as usize {
        let b = ctx.world.shape(out[i]).body;
        if !body::is_static(ctx, b) {
            let p = body::get_world_center(ctx, b);
            let d = Vec2::new(p.x - center.x, p.y - center.y);
            let length = as3::math_min(radius, d.length());
            let factor = R::one() - length / radius;
            let angle = atan2(d.y, d.x);
            let (cos, sin) = sin_cos(angle);
            body::apply_impulse(
                ctx,
                b,
                Vec2::new(cos * factor * hw_real::rl!(R; 10.0), sin * factor * hw_real::rl!(R; 10.0)),
                p,
            );
        }
    }
    world::destroy_body(ctx, fuel)
}

pub fn consume_results<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    for i in 0..4 {
        // Fetch the shape each time: chairSmash creates the last two in this same pass.
        let wc = ctx.character.wheelchair;
        let sh = match i {
            0 => wc.shapes[0],
            1 => wc.shapes[3],
            2 => wc.jet_shape,
            _ => wc.fuel_shape,
        };
        let k = damage::key(sh);
        let r = ctx.character.contact_results[k];
        if !r.present {
            continue;
        }
        let normal = ctx.world.report_cr_normal;
        flow!(match i {
            0 => chair_smash(ctx, r.impulse, normal),
            1 => wheel_smash(ctx, r.impulse, normal),
            2 => jet_smash(ctx, r.impulse),
            _ => fuel_smash(ctx, r.impulse),
        });
        ctx.character.contact_results[k] = damage::BufferedResult::blank();
        ctx.character.contact_adds[k] = crate::Flag32::FALSE;
        if i == 0 {
            for i in 0..3 {
                let sh = wc.shapes[i];
                let k = damage::key(sh);
                ctx.character.contact_adds[k] = crate::Flag32::FALSE;
            }
        }
    }
    Flow::Continue
}
