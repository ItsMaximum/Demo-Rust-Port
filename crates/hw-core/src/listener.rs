//! b2ContactListener dispatch. `Ctx::listener_mode` selects the installed listener:
//! 0 = none (Box2D-only fixtures), 1 = the game's `ContactListener` (implemented in `game`).
use hw_real::Real;

use crate::b2::state::{ContactPoint, ContactResult};
use crate::flow::Flow;
use crate::Ctx;

#[inline]
pub fn add<R: Real>(ctx: &mut Ctx<R>, cp: &ContactPoint<R>) -> Flow {
    match ctx.listener_mode {
        0 => Flow::Continue,
        _ => crate::game::contact_listener::add(ctx, cp),
    }
}
#[inline]
pub fn persist<R: Real>(ctx: &mut Ctx<R>, cp: &ContactPoint<R>) -> Flow {
    match ctx.listener_mode {
        0 => Flow::Continue,
        _ => crate::game::contact_listener::persist(ctx, cp),
    }
}
#[inline]
pub fn remove<R: Real>(ctx: &mut Ctx<R>, cp: &ContactPoint<R>) -> Flow {
    match ctx.listener_mode {
        0 => Flow::Continue,
        _ => crate::game::contact_listener::remove(ctx, cp),
    }
}
/// `Result(cr)`: `cr.normal` is `ctx.world.report_cr_normal` (already set by the island).
#[inline]
pub fn result<R: Real>(ctx: &mut Ctx<R>, cr: &ContactResult<R>) -> Flow {
    match ctx.listener_mode {
        0 => Flow::Continue,
        _ => crate::game::contact_listener::result(ctx, cr),
    }
}
