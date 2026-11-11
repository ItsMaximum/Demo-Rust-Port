//! `tune`: evolve the inputs around a launch. Each pool member is a prefix ending at its launch
//! frame (`--cut`). A trial copies a member, re-rolls a few entries in the last `--window`
//! entries, plays the prefix and then `--tail` (default no input) to frame 299, and scores the
//! closest approach of Dad's living chest to the finish region (a finish wins outright).
//! Children keep the parent's length, so the search always varies the launch.
use crate::sim::{tas_text, Sim};
use crate::window::{finish_distance, Rng};
use std::io::Write;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

pub struct Config {
    pub run_dir: std::path::PathBuf,
    pub threads: usize,
    pub hours: f64,
    pub seed: u64,
    pub window: usize,
    pub tail: u8,
    pub pool: usize,
    pub temp: f64,
    pub seeds: Vec<Vec<u8>>,
}

#[derive(Clone)]
struct Member {
    masks: Vec<u8>,
    score: f64,
    visits: u64,
}

fn choose(rng: &mut Rng, orig: u8) -> u8 {
    const POSE: [u8; 9] = [0, 1, 2, 4, 8, 1 | 4, 2 | 4, 1 | 8, 2 | 8];
    let mut m = POSE[rng.below(9) as usize];
    if rng.below(10) < 6 {
        m |= 16;
    }
    // Occasionally toggle an eject bit too (the launch comes from the eject explosion).
    let mut ej = orig & (32 | 64 | 128);
    if rng.below(12) == 0 {
        ej ^= [32u8, 64, 128][rng.below(3) as usize];
    }
    m | ej
}

/// Launch zone (start area, chest rising faster than 15 m/s): see window::up_zone.
fn in_zone(c: [f64; 2], v: [f64; 2]) -> bool {
    c[0] > 0.0 && c[0] < 26.0 && c[1] > 62.0 && c[1] < 84.0 && v[1] < -15.0
}

/// Combined objective (`OBBY_COMBINED`): a pairmax onset while the chest is in the launch zone
/// scores minus the closest living approach to the finish afterwards; without one, the score is
/// -1000 - (4096 - best pair count seen in the zone).
fn evaluate_combined(s: &mut Sim, template: &Sim, masks: &[u8], tail: u8) -> (f64, u32) {
    s.restore(template);
    s.masks.clear();
    s.masks.extend_from_slice(masks);
    let mut near = f64::INFINITY;
    let mut onset = false;
    let mut peak = 0i32;
    let t0 = std::time::Instant::now();
    while s.frame() < 299 {
        let f = s.step(tail);
        if f.port_abort || f.step_us > 80_000 || t0.elapsed().as_millis() > 2000 {
            break;
        }
        if f.finished && f.alive_before {
            return (0.0, f.frame);
        }
        if !onset {
            if f.site == 2 && f.alive_before && in_zone(f.chest_before, [0.0, -20.0]) {
                onset = true;
            } else if f.site == 0 && f.alive_after && in_zone(f.chest_after, f.chest_v) {
                peak = peak.max(f.pairs);
            }
        }
        if f.site == 0 && !f.alive_after {
            break;
        }
        if onset && f.alive_after {
            near = near.min(finish_distance(f.chest_after));
        }
        if !onset && s.frame() > 140 {
            break;
        }
    }
    if onset {
        (near.min(999.0), 0)
    } else {
        (1000.0 + (4096 - peak.min(4096)) as f64, 0)
    }
}

/// Play `masks` then the tail to frame 299; returns (closest alive distance, finish frame).
fn evaluate(s: &mut Sim, template: &Sim, masks: &[u8], tail: u8) -> (f64, u32) {
    static COMBINED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    if *COMBINED.get_or_init(|| std::env::var("OBBY_COMBINED").is_ok()) {
        return evaluate_combined(s, template, masks, tail);
    }
    s.restore(template);
    s.masks.clear();
    s.masks.extend_from_slice(masks);
    let mut near = f64::INFINITY;
    let t0 = std::time::Instant::now();
    while s.frame() < 299 {
        let f = s.step(tail);
        if f.port_abort || f.step_us > 80_000 || t0.elapsed().as_millis() > 2000 {
            break;
        }
        if f.finished && f.alive_before {
            return (0.0, f.frame);
        }
        if f.site == 0 && !f.alive_after {
            break;
        }
        if f.alive_after {
            near = near.min(finish_distance(f.chest_after));
        }
    }
    (near, 0)
}

pub fn run(cfg: Config) -> Result<(), String> {
    std::fs::create_dir_all(&cfg.run_dir).map_err(|e| e.to_string())?;
    let template = Sim::new(crate::LEVEL, crate::CHAR)?;
    let pool: Mutex<Vec<Member>> = Mutex::new(cfg.seeds.iter().map(|m| Member { masks: m.clone(), score: f64::NEG_INFINITY, visits: 0 }).collect());
    let stop = AtomicBool::new(false);
    let trials = AtomicU64::new(0);
    let best = Mutex::new((f64::INFINITY, Vec::<u8>::new()));
    let start = Instant::now();
    let deadline = start + Duration::from_secs_f64(cfg.hours * 3600.0);
    let winner: Mutex<Option<Vec<u8>>> = Mutex::new(None);
    // Score the seeds first.
    {
        let next = std::sync::atomic::AtomicUsize::new(0);
        std::thread::scope(|scope| {
            for _ in 0..cfg.threads {
                let (next, pool, template, cfg) = (&next, &pool, &template, &cfg);
                std::thread::Builder::new().stack_size(32 << 20).spawn_scoped(scope, move || {
                    let mut s = template.fork();
                    loop {
                        let i = next.fetch_add(1, Ordering::Relaxed);
                        let masks = {
                            let p = pool.lock().unwrap();
                            if i >= p.len() {
                                break;
                            }
                            p[i].masks.clone()
                        };
                        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| evaluate(&mut s, template, &masks, cfg.tail)));
                        let near = match r {
                            Ok((near, _)) => near,
                            Err(_) => {
                                s = template.fork();
                                f64::INFINITY
                            }
                        };
                        pool.lock().unwrap()[i].score = -near;
                    }
                }).unwrap();
            }
        });
    }
    {
        let p = pool.lock().unwrap();
        let b = p.iter().map(|m| m.score).fold(f64::NEG_INFINITY, f64::max);
        eprintln!("tune: {} seeds, best seed distance {:.2}", p.len(), -b);
    }
    std::thread::scope(|scope| {
        for w in 0..cfg.threads {
            let (pool, template, cfg, stop, trials, best, winner) = (&pool, &template, &cfg, &stop, &trials, &best, &winner);
            std::thread::Builder::new().stack_size(32 << 20).spawn_scoped(scope, move || {
                let mut rng = Rng((cfg.seed.wrapping_add(w as u64 + 1)).wrapping_mul(0xd1b5_4a32_d192_ed03) ^ 0x2545_f491_4f6c_dd1d);
                let mut s = template.fork();
                while !stop.load(Ordering::Relaxed) {
                    // Pick a parent (softmax on score, damped by visits; a fifth uniform).
                    let parent = {
                        let mut p = pool.lock().unwrap();
                        let n = p.len();
                        let i = if rng.below(5) == 0 {
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
                    let l = parent.masks.len();
                    let lo = l.saturating_sub(cfg.window);
                    let mut masks = parent.masks.clone();
                    let k = 1 + rng.below(3) as usize;
                    for _ in 0..k {
                        let i = lo + rng.below((l - lo) as u64) as usize;
                        masks[i] = choose(&mut rng, masks[i]);
                    }
                    trials.fetch_add(1, Ordering::Relaxed);
                    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| evaluate(&mut s, template, &masks, cfg.tail)));
                    let Ok((near, fin)) = r else {
                        s = template.fork();
                        continue;
                    };
                    if fin != 0 {
                        let used = s.masks[..s.replay.iteration as usize].to_vec();
                        std::fs::write(cfg.run_dir.join("winner-1.tas"), tas_text(&used)).ok();
                        *winner.lock().unwrap() = Some(used);
                        stop.store(true, Ordering::Relaxed);
                        return;
                    }
                    {
                        let mut b = best.lock().unwrap();
                        if near < b.0 {
                            *b = (near, masks.clone());
                            std::fs::write(cfg.run_dir.join("best.tas"), tas_text(&masks)).ok();
                        }
                    }
                    let score = -near;
                    let mut p = pool.lock().unwrap();
                    if p.len() < cfg.pool {
                        p.push(Member { masks, score, visits: 0 });
                    } else {
                        let (mi, ms) = p.iter().enumerate().map(|(i, m)| (i, m.score)).fold((0, f64::MAX), |a, b| if b.1 < a.1 { b } else { a });
                        if score > ms {
                            p[mi] = Member { masks, score, visits: 0 };
                        }
                    }
                }
            }).unwrap();
        }
        // Monitor.
        let mut st = std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("stats.jsonl")).unwrap();
        loop {
            std::thread::sleep(Duration::from_secs(10));
            let t = start.elapsed().as_secs_f64();
            let b = best.lock().unwrap().0;
            let line = format!("{{\"t\":{:.0},\"trials\":{},\"best\":{:.3},\"winner\":{}}}", t, trials.load(Ordering::Relaxed), b, winner.lock().unwrap().is_some());
            writeln!(st, "{line}").ok();
            eprintln!("{line}");
            if Instant::now() > deadline || cfg.run_dir.join("STOP").exists() || stop.load(Ordering::Relaxed) {
                stop.store(true, Ordering::Relaxed);
                break;
            }
        }
    });
    if let Ok(mut out) = std::fs::File::create(cfg.run_dir.join("pool.jsonl")) {
        for m in pool.lock().unwrap().iter() {
            writeln!(out, "{{\"score\":{:.3},\"frames\":{},\"tas\":\"{}\"}}", m.score, m.masks.len(), tas_text(&m.masks)).ok();
        }
    }
    std::process::exit(if winner.lock().unwrap().is_some() { 0 } else { 1 });
}
