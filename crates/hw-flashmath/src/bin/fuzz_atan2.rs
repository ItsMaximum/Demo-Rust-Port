//! Compare portable atan2 candidates against this build's CRT over many random and structured
//! (y, x) pairs. Only meaningful on the i686-pc-windows-msvc target (CRT == Flash 32 x86).
//! Usage: fuzz_atan2 <count> [seed] [player]
#[allow(dead_code)]
#[path = "../host_x87.rs"]
mod x87;

#[cfg(target_arch = "x86")]
fn player_reference(y: f64, x: f64) -> f64 { x87::atan2(y, x) }
#[cfg(not(target_arch = "x86"))]
fn player_reference(_: f64, _: f64) -> f64 { panic!("player reference requires x86") }

fn splitmix(seed: &mut u64) -> u64 {
    *seed = seed.wrapping_add(0x9E3779B97F4A7C15);
    let mut z = *seed;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
    z ^ (z >> 31)
}

/// Random double with an exponent drawn from a mix of distributions (all exponents incl. zero,
/// subnormal, inf/nan; angle-like range; near-unit ratio).
fn rand_double(seed: &mut u64, i: u64) -> f64 {
    let z = splitmix(seed);
    let exp: u64 = match i % 5 {
        0 => (z >> 52) & 0x7ff,                        // any, incl. 0 (subnormal/zero) and 0x7ff
        1 => 1023 - 30 + ((z >> 52) % 60),              // moderate
        2 => 1023 - 3 + ((z >> 52) % 6),                // near unit
        3 => ((z >> 52) % 3) * 1023,                     // extremes: 0, 1023, 2046
        _ => (z >> 52) % 2047,
    };
    f64::from_bits((z & 0x800F_FFFF_FFFF_FFFF) | (exp.min(2047) << 52))
}

struct Stat {
    name: &'static str,
    tested: u64,
    bad: u64,
    finite_tested: u64,
    finite_bad: u64,
    tiny_result_bad: u64,
    first: String,
    f: fn(f64, f64) -> f64,
}

fn check(stats: &mut [Stat], y: f64, x: f64, reference: fn(f64, f64) -> f64) {
    let want = reference(y, x).to_bits();
    for s in stats.iter_mut() {
        s.tested += 1;
        let finite = y.is_finite() && x.is_finite();
        s.finite_tested += finite as u64;
        let got = (s.f)(y, x).to_bits();
        let same = got == want;
        if !same {
            s.bad += 1;
            s.finite_bad += finite as u64;
            s.tiny_result_bad += (finite && (want & 0x7ff0_0000_0000_0000) == 0
                && (got & 0x7ff0_0000_0000_0000) == 0) as u64;
            if finite && s.finite_bad == 1 {
                eprintln!("{} first finite mismatch y={:016x} x={:016x} reference={want:016x} cand={got:016x}",
                    s.name, y.to_bits(), x.to_bits());
            }
            if s.first.is_empty() {
                s.first = format!("y={:016x} x={:016x} reference={want:016x} cand={got:016x}", y.to_bits(), x.to_bits());
            }
        }
    }
}

fn main() {
    let count: u64 = std::env::args().nth(1).map(|s| s.parse().unwrap()).unwrap_or(20_000_000);
    let mut seed: u64 = std::env::args().nth(2).map(|s| s.parse().unwrap()).unwrap_or(0x1234_5678_9ABC_DEF0);
    let player = std::env::args().nth(3).as_deref() == Some("player");
    let reference: fn(f64, f64) -> f64 = if player { player_reference } else { f64::atan2 };
    let mut stats = vec![
        Stat { name: "libm", tested: 0, bad: 0, finite_tested: 0, finite_bad: 0, tiny_result_bad: 0, first: String::new(), f: libm::atan2 },
        Stat { name: "portable", tested: 0, bad: 0, finite_tested: 0, finite_bad: 0, tiny_result_bad: 0, first: String::new(), f: if player { hw_flashmath::portable::atan2::<f64> } else { hw_flashmath::candidates::atan2_fdlibm::atan2::<f64> } },
    ];
    #[cfg(target_arch = "x86")]
    stats.push(Stat { name: "x87-body", tested: 0, bad: 0, finite_tested: 0, finite_bad: 0, tiny_result_bad: 0, first: String::new(), f: x87::atan2_body });
    // Structured cases.
    let specials = [0.0, -0.0, 1.0, -1.0, f64::INFINITY, f64::NEG_INFINITY, f64::NAN, f64::MIN_POSITIVE, 5e-324, f64::MAX, -f64::MAX, 0.5, 2.0, 1e-300, 1e300];
    for &y in &specials {
        for &x in &specials {
            check(&mut stats, y, x, reference);
        }
    }
    let mut structured = 0u64;
    for i in 0..count / 10 {
        let z = splitmix(&mut seed);
        let x = rand_double(&mut seed, i);
        // y/x near 1 (and near -1): y = x * (1 +- tiny)
        let d = f64::from_bits(0x3ff0_0000_0000_0000 | (z & 0xFFFF)) - 1.0; // in [0, 2^-36)
        let sign = if z & 0x1_0000 != 0 { -1.0 } else { 1.0 };
        check(&mut stats, sign * x * (1.0 + d), x, reference);
        check(&mut stats, sign * x * (1.0 - d), x, reference);
        // near axes: y tiny relative to x, and x tiny relative to y
        let tiny = f64::from_bits(z.rotate_left(17) & 0x000F_FFFF_FFFF_FFFF | ((1023u64 - 20 - (z >> 58)) << 52));
        check(&mut stats, sign * x * tiny, x, reference);
        check(&mut stats, x, sign * x * tiny, reference);
        // ratio thresholds of fdlibm: |y/x| near 2^60, 2^-60, 2^64
        check(&mut stats, x, x * f64::from_bits(((1023u64 + 60) << 52) | (z & 0xFF)), reference);
        check(&mut stats, x, x * f64::from_bits(((1023u64 - 60) << 52) | (z & 0xFF)), reference);
        check(&mut stats, x, x * f64::from_bits(((1023u64 + 64) << 52) | (z & 0xFF)), reference);
        structured += 7;
    }
    for i in 0..count {
        let y = rand_double(&mut seed, i);
        let x = rand_double(&mut seed, i.wrapping_mul(7).wrapping_add(3));
        check(&mut stats, y, x, reference);
    }
    println!("target {} structured {structured} reference {}", std::env::consts::ARCH, if player { "player-x87-body-with-NaN-handling" } else { "installed-crt" });
    for s in &stats {
        println!("{:10} tested {} bad {} {}", s.name, s.tested, s.bad, s.first);
        println!("           finite tested {} bad {}", s.finite_tested, s.finite_bad);
        println!("           finite bad with both results zero/subnormal {}", s.tiny_result_bad);
    }
}
