//! com.totaljerkface.game.ContactListener. Lookup the second handler AFTER the first returns.
//! Handler lookup remains live across each callback, including refilter/destroy side effects.
use hw_real::Real;

use crate::b2::math::Vec2;
use crate::b2::state::{ContactPoint, ContactResult};
use crate::flow::Flow;
use crate::{flow, Ctx, H};

fn dispatch<R: Real>(
    ctx: &mut Ctx<R>,
    event: usize,
    own: H,
    other: H,
    impulse: R,
    velocity: Vec2<R>,
    normal: Vec2<R>,
) -> Flow {
    let tag = ctx.world.shape(own).handlers[event];
    if tag == super::level::LEVEL_ADD_HANDLER {
        return super::level::contact_add(ctx, own, other, velocity, normal);
    }
    if tag & super::specials::SOUND_HANDLER != 0 {
        return super::specials::sound_contact(ctx, tag, own, other, velocity, normal);
    }
    if tag & super::character::CHARACTER_HANDLER == 0 {
        return super::specials::contact(ctx, event, own, other, impulse);
    }
    let previous = ctx.character.child_active;
    ctx.character.child_active = crate::Flag32::new(tag & super::character::CHILD_HANDLER != 0);
    let result = character_dispatch(
        ctx,
        event,
        own,
        other,
        impulse,
        tag & !super::character::CHILD_HANDLER,
        velocity,
        normal,
    );
    ctx.character.child_active = previous;
    result
}
fn character_dispatch<R: Real>(
    ctx: &mut Ctx<R>,
    event: usize,
    own: H,
    other: H,
    impulse: R,
    tag: u32,
    velocity: Vec2<R>,
    normal: Vec2<R>,
) -> Flow {
    if event == 3 {
        return super::damage::result(ctx, own, other, impulse, tag);
    }
    // Wheel ADD/REMOVE return on sensors before reading shape2.m_body. The
    // base character ADD deliberately reads its mass before its sensor guard.
    if matches!(
        tag & !super::character::CHARACTER_HANDLER,
        33 | 42 | 51 | 52 | 72 | 73
    ) && ctx.world.shape(other).is_sensor.get()
    {
        return Flow::Continue;
    }
    if tag == super::character::CHARACTER_HANDLER | 33 && !ctx.world.shape(other).is_sensor {
        if event == 0 {
            ctx.character.wheelchair.wheel_contacts =
                ctx.character.wheelchair.wheel_contacts + R::one();
        }
        if event == 1 {
            ctx.character.wheelchair.wheel_contacts =
                ctx.character.wheelchair.wheel_contacts - R::one();
        }
    }
    if (tag == super::character::CHARACTER_HANDLER | 51
        || tag == super::character::CHARACTER_HANDLER | 52)
        && !ctx.world.shape(other).is_sensor
    {
        let delta = if event == 0 {
            R::one()
        } else if event == 1 {
            -R::one()
        } else {
            R::zero()
        };
        if tag == super::character::CHARACTER_HANDLER | 51 {
            ctx.character.bicycle.back_contacts = ctx.character.bicycle.back_contacts + delta;
        } else {
            ctx.character.bicycle.front_contacts = ctx.character.bicycle.front_contacts + delta;
        }
        ctx.character.bicycle.wheel_contacts =
            ctx.character.bicycle.front_contacts + ctx.character.bicycle.back_contacts;
    }
    if (tag == super::character::CHARACTER_HANDLER | 72
        || tag == super::character::CHARACTER_HANDLER | 73)
        && !ctx.world.shape(other).is_sensor
    {
        let delta = if event == 0 {
            R::one()
        } else if event == 1 {
            -R::one()
        } else {
            R::zero()
        };
        if tag == super::character::CHARACTER_HANDLER | 72 {
            ctx.character.motorcart.back_contacts = ctx.character.motorcart.back_contacts + delta;
        } else {
            ctx.character.motorcart.front_contacts = ctx.character.motorcart.front_contacts + delta;
        }
        ctx.character.motorcart.wheel_contacts =
            ctx.character.motorcart.front_contacts + ctx.character.motorcart.back_contacts;
    }
    // Other character point callbacks select sounds only.
    if tag == super::character::CHARACTER_HANDLER | 42 && !ctx.world.shape(other).is_sensor {
        if event == 0 {
            ctx.character.segway.wheel_contacts = ctx.character.segway.wheel_contacts + R::one();
        }
        if event == 1 {
            ctx.character.segway.wheel_contacts = ctx.character.segway.wheel_contacts - R::one();
        }
    }
    if event == 0 {
        return super::damage::contact_add(ctx, own, other, velocity, normal);
    }
    Flow::Continue
}

pub fn add<R: Real>(ctx: &mut Ctx<R>, cp: &ContactPoint<R>) -> Flow {
    point(ctx, cp, 0)
}
pub fn persist<R: Real>(ctx: &mut Ctx<R>, cp: &ContactPoint<R>) -> Flow {
    point(ctx, cp, 2)
}
pub fn remove<R: Real>(ctx: &mut Ctx<R>, cp: &ContactPoint<R>) -> Flow {
    point(ctx, cp, 1)
}
fn point<R: Real>(ctx: &mut Ctx<R>, cp: &ContactPoint<R>, event: usize) -> Flow {
    flow!(dispatch(
        ctx,
        event,
        cp.shape1,
        cp.shape2,
        R::zero(),
        cp.velocity,
        cp.normal
    ));
    dispatch(
        ctx,
        event,
        cp.shape2,
        cp.shape1,
        R::zero(),
        // The game swaps shapes but shares the original velocity; ADD also
        // shares the original normal. REMOVE/PERSIST leave the new normal zero.
        cp.velocity,
        if event == 0 { cp.normal } else { Vec2::zero() },
    )
}
pub fn result<R: Real>(ctx: &mut Ctx<R>, cr: &ContactResult<R>) -> Flow {
    flow!(dispatch(
        ctx,
        3,
        cr.shape1,
        cr.shape2,
        cr.normal_impulse,
        Vec2::zero(),
        Vec2::zero()
    ));
    dispatch(
        ctx,
        3,
        cr.shape2,
        cr.shape1,
        cr.normal_impulse,
        Vec2::zero(),
        Vec2::zero(),
    )
}
