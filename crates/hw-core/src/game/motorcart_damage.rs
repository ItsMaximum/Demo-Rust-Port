//! MotorCart destruction; replacement fragments keep the source's mass/velocity order.
use super::{
    character::*,
    damage,
    motorcart::{self, *},
};
use crate::b2::{
    body,
    state::{BodyDef, ShapeDef},
    world,
};
use crate::{flow, flow::AbortSite, Ctx, Flow, NONE};
use hw_real::Real;
pub fn cart_smash<R: Real>(ctx: &mut Ctx<R>, _impulse: R) -> Flow {
    let sh = ctx.character.motorcart.shapes[4];
    damage::disarm(ctx, sh);
    body::destroy_shape(ctx, ctx.character.motorcart.bodies[MAIN], sh)
}
pub fn cracker_smash<R: Real>(ctx: &mut Ctx<R>, _impulse: R) -> Flow {
    let sh = ctx.character.motorcart.shapes[7];
    let k = damage::key(sh);
    ctx.character.contact_thresholds[k] = damage::undefined();
    ctx.world.shape_mut(sh).handlers[3] = 0;
    world::destroy_body(ctx, ctx.character.motorcart.groceries[4])
}
pub fn cola_smash<R: Real>(ctx: &mut Ctx<R>, _impulse: R) -> Flow {
    let sh = ctx.character.motorcart.shapes[8];
    let k = damage::key(sh);
    ctx.character.contact_thresholds[k] = damage::undefined();
    ctx.world.shape_mut(sh).handlers[3] = 0;
    Flow::Continue
}
pub fn main_smash<R: Real>(ctx: &mut Ctx<R>, _impulse: R) -> Flow {
    let cart = ctx.character.motorcart;
    let k = damage::key(cart.shapes[0]);
    ctx.character.contact_thresholds[k] = damage::undefined();
    for i in 0..3 {
        ctx.world.shape_mut(cart.shapes[i]).handlers[3] = 0;
    }
    ctx.world.shape_mut(cart.shapes[2]).handlers[0] = 0;
    ctx.world.shape_mut(cart.shapes[3]).handlers[3] = 0;
    flow!(motorcart::eject(ctx));
    ctx.character.motorcart.main_smashed = crate::Flag32::TRUE;
    if ctx.world.shape(cart.shapes[4]).body != NONE {
        flow!(cart_smash(ctx, hw_real::rl!(R; 100.0)));
    }
    let main = cart.bodies[MAIN];
    let mut bd = BodyDef::new();
    bd.position = body::get_position(ctx, main);
    bd.angle = body::get_angle(ctx, main);
    let v = body::get_linear_velocity(ctx, main);
    let w = body::get_angular_velocity(ctx, main);
    let mut p = ShapeDef::polygon();
    p.density = hw_real::rl!(R; 4.0);
    p.friction = hw_real::rl!(R; 0.3);
    p.restitution = hw_real::rl!(R; 0.1);
    p.filter = ctx.character.zero_filter;
    let mut chunks = [NONE; 4];
    for i in 0..4 {
        let b = &mut chunks[i];
        *b = world::create_body(ctx, &bd);
        if *b == NONE {
            return Flow::Abort(if ctx.world.lock.get() {
                AbortSite::GameNullDeref
            } else {
                AbortSite::PortCapacity
            });
        }
        body::set_linear_velocity(ctx, *b, v);
        body::set_angular_velocity(ctx, *b, w);
    }
    let mut sh = body::get_shape_list(ctx, main);
    p.vertex_count = 4;
    for i in 0..4 {
        let b = chunks[i];
        if sh == NONE {
            return Flow::Abort(AbortSite::GameNullDeref);
        }
        p.vertices = ctx.world.shape(sh).vertices;
        flow!(body::create_shape(ctx, b, &p).0);
        flow!(body::set_mass_from_shapes(ctx, b));
        sh = ctx.world.shape(sh).next;
    }
    flow!(world::destroy_body(ctx, main));
    flow!(world::destroy_body(ctx, cart.bodies[FRONT_SHOCK]));
    world::destroy_body(ctx, cart.bodies[BACK_SHOCK])
}
pub fn consume_results<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    let items = [4, 0, 7, 8];
    for item_index in 0..items.len() {
        let i = items[item_index];
        let k = damage::key(ctx.character.motorcart.shapes[i]);
        let r = ctx.character.contact_results[k];
        if !r.present {
            continue;
        }
        flow!(match i {
            4 => cart_smash(ctx, r.impulse),
            0 => main_smash(ctx, r.impulse),
            7 => cracker_smash(ctx, r.impulse),
            _ => cola_smash(ctx, r.impulse),
        });
        ctx.character.contact_results[k] = damage::BufferedResult::blank();
        if i == 4 {
            ctx.character.contact_adds[k] = crate::Flag32::FALSE;
            let k = damage::key(ctx.character.motorcart.shapes[2]);
            ctx.character.contact_adds[k] = crate::Flag32::FALSE;
        }
    }
    Flow::Continue
}
pub fn after_distal_break<R: Real>(ctx: &mut Ctx<R>, joint: usize) -> Flow {
    if ctx.character.motorcart.ejected.get() {
        return Flow::Continue;
    }
    let i = match joint {
        ELBOW1 => HAND1,
        ELBOW2 => HAND2,
        KNEE1 => LEG1,
        KNEE2 => LEG2,
        _ => return Flow::Abort(AbortSite::PortUnimplemented),
    };
    let j = ctx.character.motorcart.joints[i];
    if j != NONE {
        flow!(world::destroy_joint(ctx, j));
        ctx.character.motorcart.joints[i] = NONE;
    }
    if ctx.character.motorcart.joints[HAND1] == NONE
        && ctx.character.motorcart.joints[HAND2] == NONE
        && ctx.character.motorcart.joints[LEG1] == NONE
        && ctx.character.motorcart.joints[LEG2] == NONE
    {
        flow!(motorcart::eject(ctx));
    }
    Flow::Continue
}
