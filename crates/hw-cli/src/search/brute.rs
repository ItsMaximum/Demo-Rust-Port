//! `hw-search brute`: exhaustive enumeration of the inputs on the last `depth` frames of a
//! launch-prone string (the TMP-style brute force that found the Dawn/SMF/BMX launches).
//!
//! One snapshot is taken just before the first enumerated frame; every combination replays only
//! `depth + extra` frames from it. The first launch (alive entering the frame, chest moved at
//! least `min_launch` m) of each combination is recorded; a finish is confirmed from scratch.
use super::explore::confirm;
use super::sim::{tas_text, Sim};
use serde_json::json;
use std::io::Write;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::Mutex;
use std::time::Instant;

pub struct Config {
    pub level: u32,
    pub character: u32,
    /// Base inputs; the last `depth` entries are replaced by each combination.
    pub base: Vec<u8>,
    pub depth: usize,
    /// Per-frame alphabet (input masks) to enumerate.
    pub alphabet: Vec<u8>,
    /// Frames played after the enumerated ones, holding the last enumerated input.
    pub extra: u32,
    pub min_launch: f64,
    pub target: [f64; 2],
    pub threads: usize,
    pub out: std::path::PathBuf,
    /// Optional cap on combinations (random subset order is not used: enumeration is exhaustive
    /// up to the cap, in index order).
    pub limit: u64,
}

/// Shaft depth band of an end position (GBE: shaft x 277..300.8).
fn band(p: [f64; 2]) -> usize {
    if p[0] >= 277.0 && p[0] <= 300.8 {
        if p[1] >= 85.0 {
            return 3;
        }
        if p[1] >= 60.0 {
            return 2;
        }
        if p[1] >= 40.0 {
            return 1;
        }
    }
    0
}

pub fn run(cfg: Config) -> Result<(), String> {
    let n = cfg.base.len();
    if cfg.depth == 0 || cfg.depth > n {
        return Err("depth must be 1..=len(base)".into());
    }
    let start_at = n - cfg.depth;
    // Snapshot before the first enumerated frame.
    let mut base = Sim::new(cfg.level, cfg.character)?;
    base.masks = cfg.base[..start_at].to_vec();
    for _ in 0..start_at {
        if base.step(0).port_abort {
            return Err("port-only abort in the prefix".into());
        }
    }
    let a = cfg.alphabet.len() as u64;
    let total = a.checked_pow(cfg.depth as u32).ok_or("too many combinations")?.min(cfg.limit);
    let next = AtomicUsize::new(0);
    let launches = AtomicU64::new(0);
    let frames = AtomicU64::new(0);
    let bands = [AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0)];
    let seen = Mutex::new(std::collections::HashSet::new());
    let best: Mutex<Vec<(f64, [f64; 2], u32, Vec<u8>)>> = Mutex::new(Vec::new());
    let stop = AtomicBool::new(false);
    let winner: Mutex<Option<(u32, Vec<u8>)>> = Mutex::new(None);
    let t0 = Instant::now();
    std::thread::scope(|scope| {
        for _ in 0..cfg.threads {
            let (cfg, base, next, launches, frames, bands, seen, best, stop, winner) =
                (&cfg, &base, &next, &launches, &frames, &bands, &seen, &best, &stop, &winner);
            std::thread::Builder::new()
                .stack_size(16 << 20)
                .spawn_scoped(scope, move || {
                    let mut work = base.fork();
                    while !stop.load(Ordering::Relaxed) {
                        let i = next.fetch_add(1, Ordering::Relaxed) as u64;
                        if i >= total {
                            break;
                        }
                        let mut masks = cfg.base[..start_at].to_vec();
                        let mut k = i;
                        for _ in 0..cfg.depth {
                            masks.push(cfg.alphabet[(k % a) as usize]);
                            k /= a;
                        }
                        work.restore(base);
                        work.masks.clear();
                        work.masks.extend_from_slice(&masks);
                        let last = *masks.last().unwrap();
                        for _ in 0..cfg.depth as u32 + cfg.extra {
                            let f = work.step(last);
                            frames.fetch_add(1, Ordering::Relaxed);
                            if f.port_abort || !f.alive_before {
                                break;
                            }
                            if f.displacement >= cfg.min_launch {
                                launches.fetch_add(1, Ordering::Relaxed);
                                let key = (f.chest_after[0].to_bits(), f.chest_after[1].to_bits());
                                if !seen.lock().unwrap().insert(key) {
                                    break;
                                }
                                bands[band(f.chest_after)].fetch_add(1, Ordering::Relaxed);
                                let miss = ((f.chest_after[0] - cfg.target[0]).powi(2) + (f.chest_after[1] - cfg.target[1]).powi(2)).sqrt();
                                let used = work.masks[..work.replay.iteration as usize].to_vec();
                                {
                                    let mut b = best.lock().unwrap();
                                    b.push((miss, f.chest_after, f.frame, used.clone()));
                                    b.sort_by(|x, y| x.0.total_cmp(&y.0));
                                    b.truncate(20);
                                }
                                if f.finished {
                                    if let Ok((frame, _)) = confirm(cfg.level, cfg.character, &used, cfg.min_launch) {
                                        *winner.lock().unwrap() = Some((frame, used));
                                        stop.store(true, Ordering::Relaxed);
                                    }
                                }
                                break;
                            }
                            if !f.alive_after {
                                break;
                            }
                        }
                    }
                })
                .unwrap();
        }
    });
    let secs = t0.elapsed().as_secs_f64();
    let distinct = seen.lock().unwrap().len();
    let summary = json!({
        "combinations": total, "seconds": secs, "frames": frames.load(Ordering::Relaxed),
        "launches": launches.load(Ordering::Relaxed), "distinct_launches": distinct,
        "distinct_per_second": distinct as f64 / secs,
        "bands_other_shafttop_mid_low": bands.iter().map(|b| b.load(Ordering::Relaxed)).collect::<Vec<_>>(),
        "winner": winner.lock().unwrap().as_ref().map(|w| w.0),
    });
    println!("{summary}");
    let mut out = std::fs::File::create(&cfg.out).map_err(|e| e.to_string())?;
    writeln!(out, "{summary}").ok();
    for (miss, end, frame, masks) in best.lock().unwrap().iter() {
        writeln!(out, "{}", json!({"miss_m": miss, "end": end, "frame": frame, "tas": tas_text(masks)})).ok();
    }
    if let Some((frame, masks)) = winner.lock().unwrap().as_ref() {
        let path = cfg.out.with_extension("winner.tas");
        std::fs::write(&path, tas_text(masks)).ok();
        eprintln!("WINNER: finish frame {frame} -> {}", path.display());
    }
    Ok(())
}
