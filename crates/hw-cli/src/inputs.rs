//! Deterministic coverage inputs. This PRNG is host tooling, not game randomness.
use crate::{number, Result};
use serde_json::json;
use std::collections::BTreeMap;
use std::io::Write;

struct SplitMix32(u32);
impl SplitMix32 {
    fn next(&mut self) -> u32 {
        self.0 = self.0.wrapping_add(0x9e3779b9);
        let mut z = self.0;
        z = (z ^ (z >> 16)).wrapping_mul(0x21f0aaad);
        z = (z ^ (z >> 15)).wrapping_mul(0x735a2d97);
        z ^ (z >> 15)
    }
}

pub fn run(args: &BTreeMap<String, String>) -> Result<()> {
    for flag in args.keys() {
        if ![
            "--level", "--char", "--seed", "--frames", "--style", "--out",
        ]
        .contains(&flag.as_str())
        {
            return Err(format!("{flag} is not valid for gen-inputs"));
        }
    }
    let n = |key| number(args.get(key).ok_or_else(|| format!("missing {key}"))?, key);
    let level = n("--level")?;
    let character = n("--char")?;
    let seed = n("--seed")?;
    let frames = n("--frames")?;
    let style = args.get("--style").ok_or("missing --style")?;
    if hw_data::level(level).is_none() {
        return Err(format!("unknown level {level}"));
    }
    if !(1..=4).contains(&character) {
        return Err("--char must be 1..4".into());
    }
    if frames == 0 {
        return Err("--frames must be positive".into());
    }
    if !["idle", "forward-jumpy", "random", "mash", "death-seek"].contains(&style.as_str()) {
        return Err("--style must be idle, forward-jumpy, random, mash or death-seek".into());
    }
    let mut random = SplitMix32(seed);
    let mut output: Box<dyn Write> = if let Some(path) = args.get("--out") {
        Box::new(std::io::BufWriter::new(
            std::fs::File::create_new(path).map_err(|e| format!("{path}: {e}"))?,
        ))
    } else {
        Box::new(std::io::BufWriter::new(std::io::stdout()))
    };
    let mut held = 0;
    for frame in 0..frames {
        let draw = random.next(); // Exactly one draw per frame for every style.
        if frame % 12 == 0 {
            held = draw;
        }
        let mut keys = [false; 8];
        match style.as_str() {
            "idle" => {}
            "forward-jumpy" => {
                keys[2] = true;
                keys[0] = frame % 120 >= 70 && frame % 120 < 85;
                keys[1] = frame % 120 < 12;
                keys[4] = frame % 47 < 6;
                keys[5] = character == 1 && frame % 90 < 4;
            }
            "random" => {
                for k in 0..7 {
                    keys[k] = held & (1 << (k + 8)) != 0;
                }
                keys[7] = frame % 181 == 180;
            }
            "mash" => {
                for k in 0..8 {
                    keys[k] = draw & (1 << (k + 12)) != 0;
                }
            }
            "death-seek" => {
                keys[0] = frame % 60 >= 30;
                keys[1] = !keys[0];
                keys[2] = true;
                keys[4] = frame % 11 < 7;
                keys[5] = frame >= 100 && frame % 31 < 12;
                keys[6] = frame >= 100 && !keys[5];
                keys[7] = frame == 90 || frame % 211 == 210;
            }
            _ => unreachable!(),
        }
        let bits: String = keys.iter().map(|k| if *k { '1' } else { '0' }).collect();
        writeln!(output, "{bits}").map_err(|e| e.to_string())?;
    }
    output.flush().map_err(|e| e.to_string())?;
    if let Some(path) = args.get("--out") {
        let meta = json!({"generator":"hw-inputs-v1","prng":"SplitMix32","seed":seed,"style":style,"level":level,"character":character,"frames":frames});
        let sidecar = format!("{path}.metadata.json");
        let file = std::fs::File::create_new(&sidecar).map_err(|e| format!("{sidecar}: {e}"))?;
        serde_json::to_writer_pretty(file, &meta).map_err(|e| e.to_string())?;
    }
    Ok(())
}
