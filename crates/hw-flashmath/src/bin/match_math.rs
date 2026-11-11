//! Candidate-matching harness: compares recorded Flash math results with candidate implementations.
//! Usage: match_math <trace.jsonl>
use hw_flashmath::candidates::flash11_sse2;
use hw_real::as3;
use std::collections::HashMap;

fn bits(s: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(s, 16).unwrap())
}

fn field<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    let k = format!("\"{key}\":\"");
    let start = line.find(&k)? + k.len();
    let end = line[start..].find('"')? + start;
    Some(&line[start..end])
}

struct Stat {
    total: usize,
    mismatch: usize,
    first: Option<String>,
}
impl Stat {
    fn new() -> Self {
        Stat { total: 0, mismatch: 0, first: None }
    }
    fn add(&mut self, expected: f64, actual: f64, input: &str) {
        self.total += 1;
        let same = expected.to_bits() == actual.to_bits();
        if !same {
            self.mismatch += 1;
            if self.first.is_none() {
                self.first = Some(format!("input {input} expected {:016x} got {:016x}", expected.to_bits(), actual.to_bits()));
            }
        }
    }
}

fn main() {
    let path = std::env::args().nth(1).expect("trace path");
    let text = std::fs::read_to_string(path).unwrap();
    let mut stats: HashMap<&str, Stat> = HashMap::new();
    let mut unsupported = 0usize;
    for line in text.lines() {
        let (Some(x), Some(y)) = (field(line, "x"), field(line, "y")) else { continue };
        let xin = x;
        let x = bits(x);
        let y = bits(y);
        let get = |k: &str| bits(field(line, k).unwrap());
        // native CRT of this build target
        stats.entry("sin:crt").or_insert_with(Stat::new).add(get("sin"), x.sin(), xin);
        stats.entry("cos:crt").or_insert_with(Stat::new).add(get("cos"), x.cos(), xin);
        stats.entry("atan2:crt").or_insert_with(Stat::new).add(get("atan2"), y.atan2(x), xin);
        stats.entry("atan2:port").or_insert_with(Stat::new).add(get("atan2"), hw_flashmath::atan2(y, x), xin);
        stats.entry("pow2:crt").or_insert_with(Stat::new).add(get("pow2"), x.powf(2.0), xin);
        stats.entry("pow2:x*x").or_insert_with(Stat::new).add(get("pow2"), x * x, xin);
        stats.entry("sqrt").or_insert_with(Stat::new).add(get("sqrt"), x.abs().sqrt(), xin);
        stats.entry("mul").or_insert_with(Stat::new).add(get("mul"), x * y, xin);
        stats.entry("div").or_insert_with(Stat::new).add(get("div"), x / y, xin);
        stats.entry("add").or_insert_with(Stat::new).add(get("add"), x + y, xin);
        stats.entry("round:as3").or_insert_with(Stat::new).add(get("round"), as3::math_round(x), xin);
        stats.entry("floor").or_insert_with(Stat::new).add(get("floor"), x.floor(), xin);
        stats.entry("ceil").or_insert_with(Stat::new).add(get("ceil"), x.ceil(), xin);
        if flash11_sse2::supported(x) {
            let (c, s) = flash11_sse2::sin_cos(x);
            stats.entry("sin:flash11").or_insert_with(Stat::new).add(get("sin"), s, xin);
            stats.entry("cos:flash11").or_insert_with(Stat::new).add(get("cos"), c, xin);
        } else {
            unsupported += 1;
        }
    }
    let mut keys: Vec<_> = stats.keys().copied().collect();
    keys.sort();
    println!("target: {}", std::env::consts::ARCH);
    for k in keys {
        let s = &stats[k];
        println!("{k:14} mismatches {:6}/{:6} {}", s.mismatch, s.total, s.first.as_deref().unwrap_or(""));
    }
    println!("flash11 unsupported inputs: {unsupported}");
}
