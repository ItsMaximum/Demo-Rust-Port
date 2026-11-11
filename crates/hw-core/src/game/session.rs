//! Headless SessionReplay loop. Presentation-only work is omitted; aborts retain all
//! mutations and skip the rest of that frame, exactly like the ENTER_FRAME handler.
use super::{damage, level, segway, specials, wheelchair};
use crate::b2::world;
use crate::{flow, flow::AbortSite, Ctx, Flow};
use hw_real::Real;

#[derive(Clone, Copy)]
#[repr(C)]
pub struct ReplayState {
    pub iteration: u32,
    pub frames: u32,
    pub total_iterations: u32,
    pub input_allowed: crate::Flag32,
    pub completed: crate::Flag32,
    pub finish_frame: u32,
}
impl ReplayState {
    pub fn new(total_iterations: u32) -> Self {
        Self {
            iteration: 0,
            frames: 0,
            total_iterations,
            input_allowed: crate::Flag32::TRUE,
            completed: crate::Flag32::FALSE,
            finish_frame: 0,
        }
    }
}

/// Requires a freshly initialized Ctx. Character indices are 1 through 4.
pub fn create<R: Real>(ctx: &mut Ctx<R>, level_id: u32, character_index: u32) -> Flow {
    let Some(data) = hw_data::level(level_id) else {
        return Flow::Abort(AbortSite::PortUnimplemented);
    };
    let Some(character) = hw_data::character(character_index) else {
        return Flow::Abort(AbortSite::PortUnimplemented);
    };
    if !(1..=4).contains(&character_index) {
        return Flow::Abort(AbortSite::PortUnimplemented);
    }
    level::create_world(ctx);
    ctx.listener_mode = 1;
    flow!(level::create(ctx, data));
    match character_index {
        1 => wheelchair::create(
            ctx,
            character,
            R::from_words(data.start_x),
            R::from_words(data.start_y),
        ),
        2 => segway::create(
            ctx,
            character,
            R::from_words(data.start_x),
            R::from_words(data.start_y),
        ),
        3 => super::bicycle::create(
            ctx,
            character,
            R::from_words(data.start_x),
            R::from_words(data.start_y),
        ),
        4 => super::motorcart::create(
            ctx,
            character,
            R::from_words(data.start_x),
            R::from_words(data.start_y),
        ),
        _ => Flow::Abort(AbortSite::PortUnimplemented),
    }
}

/// `keys` is the eight-bit entry at the current iteration (all false past the input).
/// The final allowed frame still dispatches that empty entry and increments iteration.
pub fn step<R: Real>(ctx: &mut Ctx<R>, state: &mut ReplayState, keys: [bool; 8]) -> Flow {
    step_bits(ctx, state, crate::Input::from_bools(keys))
}
pub fn step_bits<R: Real>(ctx: &mut Ctx<R>, state: &mut ReplayState, keys: crate::Input) -> Flow {
    let result = step_inner(ctx, state, keys);
    // FinishLine deliberately skips levelComplete in SessionReplay. Keep feeding
    // recorded input; retain the first geometric finish even if later work throws.
    if ctx.level.finished.get() && state.finish_frame == 0 {
        state.finish_frame = state.frames;
    }
    result
}
fn step_inner<R: Real>(ctx: &mut Ctx<R>, state: &mut ReplayState, keys: crate::Input) -> Flow {
    state.frames = state.frames.wrapping_add(1);
    if state.input_allowed.get() {
        if state.iteration >= state.total_iterations {
            state.input_allowed = crate::Flag32::FALSE;
            state.completed = crate::Flag32::TRUE;
        }
        flow!(replay_input_bits(ctx, keys));
        state.iteration = state.iteration.wrapping_add(1);
    } else {
        flow!(input_bits(ctx, crate::Input::EMPTY));
    }
    flow!(actions(ctx));
    flow!(world::step(
        ctx,
        hw_real::rl!(R; 1.0 / 30.0),
        10
    ));
    flow!(consume_results(ctx));
    flow!(check_joints(ctx));
    flow!(specials::actions(ctx));
    flow!(super::level::consume_adds(ctx));
    Flow::Continue
}
pub fn input<R: Real>(ctx: &mut Ctx<R>, keys: [bool; 8]) -> Flow {
    input_bits(ctx, crate::Input::from_bools(keys))
}
pub fn input_bits<R: Real>(ctx: &mut Ctx<R>, keys: crate::Input) -> Flow {
    match ctx.character.kind {
        1 => wheelchair::input_bits(ctx, keys),
        2 => segway::input_bits(ctx, keys),
        3 => super::bicycle::input_bits(ctx, keys),
        4 => super::motorcart::input_bits(ctx, keys),
        _ => Flow::Abort(AbortSite::PortUnimplemented),
    }
}
pub fn actions<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    match ctx.character.kind {
        1 => wheelchair::actions(ctx),
        2 => segway::actions(ctx),
        3 => super::bicycle::actions(ctx),
        4 => super::motorcart::actions(ctx),
        _ => Flow::Abort(AbortSite::PortUnimplemented),
    }
}
pub fn replay_input<R: Real>(ctx: &mut Ctx<R>, keys: [bool; 8]) -> Flow {
    replay_input_bits(ctx, crate::Input::from_bools(keys))
}
pub fn replay_input_bits<R: Real>(ctx: &mut Ctx<R>, keys: crate::Input) -> Flow {
    if !ctx.character.dead {
        flow!(input_bits(ctx, keys));
    }
    if ctx.character.kind == 3 && !ctx.character.child.dead {
        ctx.character.child_active = crate::Flag32::TRUE;
        let result = super::child::input_bits(ctx, keys);
        ctx.character.child_active = crate::Flag32::FALSE;
        flow!(result);
    }
    Flow::Continue
}
pub fn consume_results<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    flow!(damage::consume_results(ctx));
    flow!(damage::consume_adds(ctx));
    if ctx.character.kind == 3 {
        ctx.character.child_active = crate::Flag32::TRUE;
        let mut result = damage::consume_results(ctx);
        if !result.is_abort() {
            result = damage::consume_adds(ctx);
        }
        ctx.character.child_active = crate::Flag32::FALSE;
        flow!(result);
    }
    Flow::Continue
}
pub fn check_joints<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    flow!(damage::check_joints(ctx));
    if ctx.character.kind == 3 {
        ctx.character.child_active = crate::Flag32::TRUE;
        let result = damage::check_joints(ctx);
        ctx.character.child_active = crate::Flag32::FALSE;
        flow!(result);
    }
    Flow::Continue
}
