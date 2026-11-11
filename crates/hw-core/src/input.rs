//! The eight TAS key positions packed into one storage word.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Input(u32);

impl Input {
    pub const EMPTY: Self = Self(0);
    #[inline]
    pub const fn from_mask(mask: u32) -> Self { Self(mask & 255) }
    #[inline]
    pub const fn mask(self) -> u32 { self.0 }
    #[inline]
    pub const fn pressed(self, key: u32) -> bool { key < 8 && self.0 & (1 << key) != 0 }
    #[inline]
    pub const fn from_bools(keys: [bool; 8]) -> Self {
        Self((keys[0] as u32) | ((keys[1] as u32) << 1) | ((keys[2] as u32) << 2)
            | ((keys[3] as u32) << 3) | ((keys[4] as u32) << 4) | ((keys[5] as u32) << 5)
            | ((keys[6] as u32) << 6) | ((keys[7] as u32) << 7))
    }
}

#[cfg(test)]
mod tests {
    use super::Input;
    #[test]
    fn preserves_all_tas_key_combinations() {
        for mask in 0..256 {
            let input = Input::from_mask(mask);
            let keys = core::array::from_fn(|i| mask & (1 << i) != 0);
            assert_eq!(Input::from_bools(keys), input);
            for i in 0..8 { assert_eq!(input.pressed(i), keys[i as usize]); }
            assert!(!input.pressed(8));
        }
        assert_eq!(Input::from_mask(u32::MAX).mask(), 255);
    }
}
