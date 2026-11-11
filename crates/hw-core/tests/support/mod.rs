#![allow(unused_imports, dead_code)]
pub use hw_core::diagnostic::{broadphase_string, oaat, state_string};

pub type TestReal = f64;

pub fn lit(value: f64) -> TestReal {
    <TestReal as hw_real::Real>::from_words(hw_real::words_of_f64(value))
}

pub trait RawBits: hw_real::Real {
    fn to_bits(self) -> u64 {
        let [hi, lo] = self.to_words();
        ((hi as u64) << 32) | lo as u64
    }
}
impl<T: hw_real::Real> RawBits for T {}
