//! Boolean state stored as a full word, so simulation state is plain words.
//! Control-flow expressions remain Rust bools; only stored flags use this type.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Flag32(u32);

impl Flag32 {
    pub const FALSE: Self = Self(0);
    pub const TRUE: Self = Self(1);
    #[inline]
    pub const fn new(value: bool) -> Self { Self(value as u32) }
    #[inline]
    pub const fn get(self) -> bool { self.0 != 0 }
    #[inline]
    pub const fn word(self) -> u32 { self.0 }
}

impl From<bool> for Flag32 {
    #[inline]
    fn from(value: bool) -> Self { Self::new(value) }
}

impl core::ops::Not for Flag32 {
    type Output = bool;
    #[inline]
    fn not(self) -> bool { !self.get() }
}
