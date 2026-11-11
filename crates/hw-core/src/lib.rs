//! Bit-exact Happy Wheels demo game logic (Box2DFlashAS3 2.0.2 + game code), generic over `Real`.
//!
//! Conventions (see docs/PORTING.md):
//! - AS3 objects live in fixed-capacity arenas and are addressed by `H` handles (`NONE` = null).
//! - Every physics function takes the whole `Ctx<R>` (world + game state) so that game callbacks
//!   invoked from inside `b2World.Step` can mutate the world exactly like the AS3 code does.
//! - Functions that can throw an AS3 runtime error return `Flow` and callers use `flow!(...)`.
//! - Statement order and floating-point operation order follow the AS3 source line by line.
#![cfg_attr(not(feature = "std"), no_std)]
#![allow(clippy::too_many_arguments, clippy::needless_range_loop)]

/// Research hook: its statements exist only with the `lab` feature (see `lab.rs`).
#[cfg(feature = "lab")]
macro_rules! lab {
    ($($t:tt)*) => { $($t)* };
}
#[cfg(not(feature = "lab"))]
macro_rules! lab {
    ($($t:tt)*) => {};
}

#[cfg(feature = "lab")]
pub mod lab;
pub mod arena;
pub mod b2;
pub mod ctx;
#[cfg(feature = "std")]
pub mod diagnostic;
#[cfg(feature = "std")]
pub mod physics_observer;
#[cfg(feature = "std")]
pub mod game_observer;
#[cfg(feature = "std")]
pub mod links_observer;
#[cfg(feature = "std")]
pub mod mechanics_observer;
#[cfg(feature = "std")]
pub mod callbacks_observer;
#[cfg(feature = "std")]
pub mod level_observer;
#[cfg(feature = "std")]
pub mod listeners_observer;
pub mod flow;
pub mod flag;
pub mod input;
pub mod game;
pub mod listener;

pub use ctx::Ctx;
pub use flow::{AbortSite, Flow, Status};
pub use flag::Flag32;
pub use input::Input;

/// Object handle (index into an arena). `NONE` plays the role of AS3 `null`.
pub type H = u32;
pub const NONE: H = u32::MAX;
