//! Simulation context: the physics world plus the game/session state that its callbacks mutate.
use hw_real::Real;

use crate::b2::island::StepScratch;
use crate::b2::state::WorldState;

/// Whole simulation state. `Copy` so searches can snapshot/restore it with a memcpy.
#[derive(Clone, Copy)]
#[repr(C)]
pub struct Ctx<R: Real> {
    pub world: WorldState<R>,
    /// Per-step scratch of b2World.Solve/SolveTOI (b2Island arrays, DFS stack, b2ContactSolver
    /// constraints). Rebuilt every call; kept here so no allocation happens on the hot path.
    pub scratch: StepScratch<R>,
    /// 0 = no contact listener installed (Box2D-only fixtures), 1 = the game's ContactListener.
    pub listener_mode: u32,
    pub level: crate::game::specials::LevelState<R>,
    pub character: crate::game::actors::Actors<R>,
}

impl<R: Real> Ctx<R> {
    pub fn new() -> Self {
        Self {
            world: WorldState::blank(),
            scratch: StepScratch::blank(),
            listener_mode: 0,
            level: crate::game::specials::LevelState::blank(),
            character: crate::game::actors::Actors::blank(),
        }
    }

    pub fn end_frame(&mut self) {
        self.world.recycle_pending();
    }

    /// Heap copy without a multi-megabyte stack temporary (search snapshots).
    #[cfg(feature = "std")]
    pub fn boxed_copy(&self) -> std::boxed::Box<Self> {
        let mut b = std::boxed::Box::<Self>::new_uninit();
        // Ctx is Copy (plain data): a bitwise copy is a valid value.
        unsafe {
            core::ptr::copy_nonoverlapping(self as *const Self, b.as_mut_ptr(), 1);
            b.assume_init()
        }
    }

    /// Restore a snapshot taken between frames. `scratch` is skipped: every Solve/SolveTOI
    /// rebuilds it before reading (tests/snapshot_restore.rs checks this).
    pub fn restore_from(&mut self, other: &Self) {
        unsafe {
            core::ptr::copy_nonoverlapping(&other.world, &mut self.world, 1);
            core::ptr::copy_nonoverlapping(&other.level, &mut self.level, 1);
            core::ptr::copy_nonoverlapping(&other.character, &mut self.character, 1);
        }
        self.listener_mode = other.listener_mode;
    }
}

impl<R: Real> Default for Ctx<R> {
    fn default() -> Self {
        Self::new()
    }
}
