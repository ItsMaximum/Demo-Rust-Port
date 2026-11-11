//! `mlaunch`: from start lineages with a QE growing (see `cqe`), back Dad into the two start
//! mines and score the launch: the in-frame peak pair count while his chest rises under the spike
//! ceiling (window::up_zone), and a pairmax onset there wins outright (logged with its input).
//!
//! A trial picks a seed, keeps a random prefix of its last `--slack` frames, then plays an
//! approach (pedal back, random lean / brake) until a start mine is gone or `--approach` frames
//! pass, then `--after` frames of random poses. Children keep the whole played input so later
//! trials can re-roll the approach or the post-blast poses.
use crate::sim::{tas_text, Sim};
use crate::window::Rng;
use std::io::Write;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

pub struct Config {
    pub run_dir: std::path::PathBuf,
    pub threads: usize,
    pub hours: f64,
    pub seed: u64,
    pub slack: usize,
    pub approach: u32,
    pub after: u32,
    pub pool: usize,
    pub temp: f64,
    pub seeds: Vec<Vec<u8>>,
}

#[derive(Clone)]
struct Member {
    masks: Vec<u8>,
    /// Frames before the approach (kept fixed by default).
    base: usize,
    score: f64,
    visits: u64,
}

fn up_zone(c: [f64; 2], v: [f64; 2]) -> bool {
    c[0] > 0.0 && c[0] < 26.0 && c[1] > 62.0 && c[1] < 84.0 && v[1] < -15.0
}

fn approach_mask(rng: &mut Rng, prev: u8) -> u8 {
    if rng.below(10) < 5 {
        return prev;
    }
    // Pedal back (bit 3) mostly, with a lean or the brake sometimes.
    const OPTS: [u8; 8] = [8, 8, 8, 1 | 8, 2 | 8, 8 | 16, 0, 2];
    OPTS[rng.below(OPTS.len() as u64) as usize]
}

fn after_mask(rng: &mut Rng, prev: u8) -> u8 {
    if rng.below(10) < 5 {
        return prev;
    }
    const POSE: [u8; 9] = [0, 1, 2, 4, 8, 1 | 4, 2 | 4, 1 | 8, 2 | 8];
    let mut m = POSE[rng.below(9) as usize];
    if rng.below(10) < 4 {
        m |= 16;
    }
    m
}

pub fn run(cfg: Config) -> Result<(), String> {
    std::fs::create_dir_all(&cfg.run_dir).map_err(|e| e.to_string())?;
    let template = Sim::new(crate::LEVEL, crate::CHAR)?;
    if cfg.seeds.is_empty() {
        return Err("mlaunch needs --seed-file".into());
    }
    let pool = Mutex::new(cfg.seeds.iter().map(|m| Member { masks: m.clone(), base: m.len(), score: 0.0, visits: 0 }).collect::<Vec<_>>());
    let stop = AtomicBool::new(false);
    let trials = AtomicU64::new(0);
    let frames = AtomicU64::new(0);
    let blasts = AtomicU64::new(0);
    let onsets = AtomicU64::new(0);
    let best = Mutex::new(f64::NEG_INFINITY);
    let start = Instant::now();
    let deadline = start + Duration::from_secs_f64(cfg.hours * 3600.0);
    let events = Mutex::new(std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("events.jsonl")).map_err(|e| e.to_string())?);
    std::thread::scope(|scope| {
        for w in 0..cfg.threads {
            let (pool, stop, trials, frames, blasts, onsets, best, events, template, cfg) =
                (&pool, &stop, &trials, &frames, &blasts, &onsets, &best, &events, &template, &cfg);
            std::thread::Builder::new().stack_size(32 << 20).spawn_scoped(scope, move || {
                let mut rng = Rng((cfg.seed.wrapping_add(w as u64 + 101)).wrapping_mul(0xd1b5_4a32_d192_ed03) ^ 0x2545_f491_4f6c_dd1d);
                let mut s = template.fork();
                while !stop.load(Ordering::Relaxed) {
                    let parent = {
                        let mut p = pool.lock().unwrap();
                        let n = p.len();
                        let i = if rng.below(4) == 0 {
                            rng.below(n as u64) as usize
                        } else {
                            let mx = p.iter().map(|m| m.score).fold(f64::NEG_INFINITY, f64::max);
                            let wts: Vec<f64> = p.iter().map(|m| ((m.score - mx) / cfg.temp).exp() / (1.0 + m.visits as f64).sqrt()).collect();
                            let tot: f64 = wts.iter().sum();
                            let mut x = rng.unit() * tot;
                            let mut k = n - 1;
                            for (j, wj) in wts.iter().enumerate() {
                                x -= wj;
                                if x <= 0.0 {
                                    k = j;
                                    break;
                                }
                            }
                            k
                        };
                        p[i].visits += 1;
                        p[i].clone()
                    };
                    trials.fetch_add(1, Ordering::Relaxed);
                    // Re-roll from a cut: inside the slack before the approach, inside the
                    // approach, or after it.
                    let lo = parent.base.saturating_sub(cfg.slack).max(1);
                    let cut = lo + rng.below((parent.masks.len().max(lo + 1) - lo) as u64) as usize;
                    let mut masks = parent.masks[..cut.min(parent.masks.len())].to_vec();
                    s.restore(template);
                    s.masks = masks.clone();
                    let mut blast_at = 0u32;
                    let mut peak = 0i32;
                    let mut peak_any = 0i32;
                    let mut onset = None;
                    let mut prev = *masks.last().unwrap_or(&0);
                    let mut approach_frames = 0u32;
                    let mut after_frames = 0u32;
                    // Start mines alive at the cut; a blast is any of them going after it.
                    let mut mines0 = 0u64;
                    let mut at_cut = false;
                    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        loop {
                            let i = s.replay.iteration as usize;
                            if i >= s.masks.len() {
                                let m = if blast_at == 0 { approach_mask(&mut rng, prev) } else { after_mask(&mut rng, prev) };
                                s.masks.push(m);
                            }
                            prev = s.masks[i.min(s.masks.len() - 1)];
                            let f = s.step(0);
                            frames.fetch_add(1, Ordering::Relaxed);
                            if f.port_abort || f.step_us > 80_000 {
                                break;
                            }
                            if f.site == 2 && f.alive_before {
                                if blast_at != 0 && up_zone(f.chest_before, [0.0, -20.0]) {
                                    onset = Some(f.frame);
                                }
                                break;
                            }
                            if f.site != 0 || !f.alive_after {
                                break;
                            }
                            if !at_cut {
                                if (s.replay.iteration as usize) < cut {
                                    continue;
                                }
                                at_cut = true;
                                mines0 = crate::qe::mines(&s.ctx) & 0b1100;
                                if mines0 == 0 {
                                    break;
                                }
                                continue;
                            }
                            if blast_at == 0 {
                                if crate::qe::mines(&s.ctx) & mines0 != mines0 {
                                    blast_at = f.frame;
                                    blasts.fetch_add(1, Ordering::Relaxed);
                                } else {
                                    approach_frames += 1;
                                    if approach_frames > cfg.approach && s.replay.iteration as usize >= s.masks.len() {
                                        break;
                                    }
                                }
                            } else {
                                after_frames += 1;
                                peak_any = peak_any.max(f.pairs);
                                if up_zone(f.chest_after, f.chest_v) {
                                    peak = peak.max(f.pairs);
                                }
                                if after_frames >= cfg.after {
                                    break;
                                }
                            }
                        }
                    }));
                    if r.is_err() {
                        s = template.fork();
                        continue;
                    }
                    masks = s.masks[..(s.replay.iteration as usize).min(s.masks.len())].to_vec();
                    let score = if let Some(fr) = onset {
                        onsets.fetch_add(1, Ordering::Relaxed);
                        writeln!(events.lock().unwrap(), "{{\"kind\":\"onset\",\"frame\":{},\"blast\":{},\"tas\":\"{}\"}}", fr, blast_at, tas_text(&masks)).ok();
                        10_000.0
                    } else if blast_at != 0 {
                        peak as f64 + 0.05 * peak_any as f64
                    } else {
                        -1.0
                    };
                    {
                        let mut b = best.lock().unwrap();
                        if score > *b {
                            *b = score;
                            std::fs::write(cfg.run_dir.join("best.tas"), tas_text(&masks)).ok();
                        }
                    }
                    if blast_at == 0 {
                        continue;
                    }
                    let child = Member { masks, base: parent.base, score, visits: 0 };
                    let mut p = pool.lock().unwrap();
                    if p.len() < cfg.pool {
                        p.push(child);
                    } else {
                        let (mi, ms) = p.iter().enumerate().map(|(i, m)| (i, m.score)).fold((0, f64::MAX), |a, b| if b.1 < a.1 { b } else { a });
                        if score > ms {
                            p[mi] = child;
                        }
                    }
                }
            }).unwrap();
        }
        let mut st = std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("stats.jsonl")).unwrap();
        loop {
            std::thread::sleep(Duration::from_secs(10));
            let t = start.elapsed().as_secs_f64();
            let line = format!(
                "{{\"t\":{:.0},\"fps\":{:.0},\"trials\":{},\"blasts\":{},\"onsets\":{},\"pool\":{},\"best\":{:.1}}}",
                t,
                frames.load(Ordering::Relaxed) as f64 / t,
                trials.load(Ordering::Relaxed),
                blasts.load(Ordering::Relaxed),
                onsets.load(Ordering::Relaxed),
                pool.lock().unwrap().len(),
                *best.lock().unwrap()
            );
            writeln!(st, "{line}").ok();
            eprintln!("{line}");
            if Instant::now() > deadline || cfg.run_dir.join("STOP").exists() {
                stop.store(true, Ordering::Relaxed);
                break;
            }
        }
    });
    if let Ok(mut out) = std::fs::File::create(cfg.run_dir.join("pool.jsonl")) {
        for m in pool.lock().unwrap().iter() {
            writeln!(out, "{{\"score\":{:.1},\"frames\":{},\"tas\":\"{}\"}}", m.score, m.masks.len(), tas_text(&m.masks)).ok();
        }
    }
    Ok(())
}
