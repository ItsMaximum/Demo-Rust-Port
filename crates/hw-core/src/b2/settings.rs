//! b2Settings (stock Box2DFlashAS3 2.0.2 values). All Number constants are given as bit words so
//! every `Real` implementation sees the identical double.
use hw_real::Real;

pub const USHRT_MAX: u32 = 65535;
pub const B2_MAX_MANIFOLD_POINTS: usize = 2;
pub const B2_MAX_POLYGON_VERTICES: usize = 8;
pub const B2_MAX_PROXIES: usize = 512;
pub const B2_MAX_PAIRS: usize = 8 * B2_MAX_PROXIES; // 4096
pub const B2_MAX_TOI_CONTACTS_PER_ISLAND: usize = 32;

/// b2_pi = Math.PI
pub fn b2_pi<R: Real>() -> R {
    R::from_words([0x400921FB, 0x54442D18])
}
/// b2_linearSlop = 0.005
pub fn b2_linear_slop<R: Real>() -> R {
    R::from_words([0x3F747AE1, 0x47AE147B])
}
/// b2_angularSlop = 2 / 180 * b2_pi (evaluated as (2/180)*pi at class init).
pub fn b2_angular_slop<R: Real>() -> R {
    (R::from_i32(2) / R::from_i32(180)) * b2_pi::<R>()
}
/// b2_toiSlop = 8 * b2_linearSlop
pub fn b2_toi_slop<R: Real>() -> R {
    R::from_i32(8) * b2_linear_slop::<R>()
}
/// b2_velocityThreshold = 1
pub fn b2_velocity_threshold<R: Real>() -> R {
    R::from_i32(1)
}
/// b2_maxLinearCorrection = 0.2
pub fn b2_max_linear_correction<R: Real>() -> R {
    R::from_words([0x3FC99999, 0x9999999A])
}
/// b2_maxAngularCorrection = 8 / 180 * b2_pi
pub fn b2_max_angular_correction<R: Real>() -> R {
    (R::from_i32(8) / R::from_i32(180)) * b2_pi::<R>()
}
/// b2_maxLinearVelocity = 200
pub fn b2_max_linear_velocity<R: Real>() -> R {
    R::from_i32(200)
}
/// b2_maxLinearVelocitySquared = 200 * 200
pub fn b2_max_linear_velocity_squared<R: Real>() -> R {
    b2_max_linear_velocity::<R>() * b2_max_linear_velocity::<R>()
}
/// b2_maxAngularVelocity = 250
pub fn b2_max_angular_velocity<R: Real>() -> R {
    R::from_i32(250)
}
pub fn b2_max_angular_velocity_squared<R: Real>() -> R {
    b2_max_angular_velocity::<R>() * b2_max_angular_velocity::<R>()
}
/// b2_contactBaumgarte = 0.2
pub fn b2_contact_baumgarte<R: Real>() -> R {
    R::from_words([0x3FC99999, 0x9999999A])
}
/// b2_timeToSleep = 0.5
pub fn b2_time_to_sleep<R: Real>() -> R {
    R::from_words([0x3FE00000, 0x00000000])
}
/// b2_linearSleepTolerance = 0.01
pub fn b2_linear_sleep_tolerance<R: Real>() -> R {
    R::from_words([0x3F847AE1, 0x47AE147B])
}
/// b2_angularSleepTolerance = 2 / 180 (NO pi: Flash-port bug, keep it).
pub fn b2_angular_sleep_tolerance<R: Real>() -> R {
    R::from_i32(2) / R::from_i32(180)
}
/// Number.MIN_VALUE (smallest subnormal).
pub fn min_value<R: Real>() -> R {
    R::from_words([0, 1])
}
/// Number.MAX_VALUE
pub fn max_value<R: Real>() -> R {
    R::from_words([0x7FEFFFFF, 0xFFFFFFFF])
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn literals_match_f64() {
        assert_eq!(b2_linear_slop::<f64>(), 0.005);
        assert_eq!(b2_max_linear_correction::<f64>(), 0.2);
        assert_eq!(b2_linear_sleep_tolerance::<f64>(), 0.01);
        assert_eq!(b2_angular_slop::<f64>(), 2.0 / 180.0 * core::f64::consts::PI);
        assert_eq!(b2_angular_sleep_tolerance::<f64>(), 2.0 / 180.0);
        assert_eq!(min_value::<f64>(), 5e-324);
        assert_eq!(max_value::<f64>(), f64::MAX);
    }
}
