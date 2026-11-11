//! Compare the portable Flash-11 SSE2 sin/cos candidate against this build's CRT over many random
//! inputs. Only meaningful on the i686-pc-windows-msvc target, whose CRT matches Flash 32 x86.
//! Usage: fuzz_trig <count> [seed] [all]
use hw_flashmath::candidates::flash11_sse2;

fn main() {
    let count: u64 = std::env::args().nth(1).map(|s| s.parse().unwrap()).unwrap_or(10_000_000);
    let mut seed: u64 = std::env::args().nth(2).map(|s| s.parse().unwrap()).unwrap_or(0x9E3779B97F4A7C15);
    let all = std::env::args().nth(3).as_deref() == Some("all");
    let mut tested = 0u64;
    let mut bad_sin = 0u64;
    let mut bad_cos = 0u64;
    let mut first = String::new();
    for i in 0..count {
        // splitmix64
        seed = seed.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = seed;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^= z >> 31;
        // exponent spread: mostly angle-like, some full range
        let exp: u64 = if i % 4 == 0 { (z >> 52) & 0x7ff } else { 1023 - 12 + ((z >> 52) % 30) };
        let bits = (z & 0x800F_FFFF_FFFF_FFFF) | (exp.min(2046) << 52);
        let x = f64::from_bits(bits);
        if !all && !flash11_sse2::supported(x) {
            continue;
        }
        tested += 1;
        let (c, s) = if all { hw_flashmath::portable::sin_cos(x) } else { flash11_sse2::sin_cos(x) };
        if s.to_bits() != x.sin().to_bits() {
            bad_sin += 1;
            if first.is_empty() {
                first = format!("sin x={:016x} crt={:016x} cand={:016x}", bits, x.sin().to_bits(), s.to_bits());
            }
        }
        if c.to_bits() != x.cos().to_bits() {
            bad_cos += 1;
            if first.is_empty() {
                first = format!("cos x={:016x} crt={:016x} cand={:016x}", bits, x.cos().to_bits(), c.to_bits());
            }
        }
    }
    println!("target {} tested {tested} bad_sin {bad_sin} bad_cos {bad_cos} {first}", std::env::consts::ARCH);
}
