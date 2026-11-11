//! Two independent CharacterB2D instances for IrresponsibleDad and ChildSeatKid.
//! Selecting an actor changes only this flag: it never copies the actor's dictionaries.
use super::character::CharacterState;
use core::ops::{Deref, DerefMut};
use hw_real::Real;
#[derive(Clone, Copy)]
#[repr(C)]
pub struct Actors<R: Real> {
    pub primary: CharacterState<R>,
    pub child: CharacterState<R>,
    pub child_active: crate::Flag32,
    pub child_present: crate::Flag32,
}
impl<R: Real> Actors<R> {
    pub fn blank() -> Self {
        Self {
            primary: CharacterState::blank(),
            child: CharacterState::blank(),
            child_active: crate::Flag32::FALSE,
            child_present: crate::Flag32::FALSE,
        }
    }
}
impl<R: Real> Deref for Actors<R> {
    type Target = CharacterState<R>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        if self.child_active.get() {
            &self.child
        } else {
            &self.primary
        }
    }
}
impl<R: Real> DerefMut for Actors<R> {
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut Self::Target {
        if self.child_active.get() {
            &mut self.child
        } else {
            &mut self.primary
        }
    }
}
