//! `onset`: search over pairmax onsets and lock releases.
//!
//! A trial re-rolls inputs shortly before a lineage's pairmax onset and plays to the new onset.
//! The onset kick moves Dad's upper body without continuous collision; while the lineage's held
//! input (normally Space) is kept, the pool stays exhausted (every frame throws), so Dad drifts
//! ballistically through the level and the character's post-step checks never run. Releasing all
//! input usually lets the next frame complete. For every held lock frame with Dad's chest in open
//! sky inside the world, a release branch is played to frame 299: a finish there is a winner.
use crate::onset::neck_gap;
use crate::sim::{chest, tas_text, Fr, Sim, Skyline};
use crate::window::{finish_distance, Rng};
use std::collections::HashSet;
use std::io::Write;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

pub struct Config {
    pub run_dir: std::path::PathBuf,
    pub threads: usize,
    pub hours: f64,
    pub seed: u64,
    pub max_frame: u32,
    /// Inputs re-rolled within this many entries before the lineage's onset.
    pub window: usize,
    /// Frames past the lineage length allowed for the new onset.
    pub extra: u32,
    /// Held lock frames followed after the onset.
    pub lock_span: u32,
    pub batch: usize,
    pub pool: usize,
    pub temp: f64,
    pub release: u8,
    pub seeds: Vec<Vec<u8>>,
}

#[derive(Clone)]
struct Member {
    masks: Vec<u8>,
    score: f64,
    visits: u64,
}

struct Shared {
    pool: Mutex<Vec<Member>>,
    seen: Mutex<HashSet<(u32, u64, u64)>>,
    stop: AtomicBool,
    frames: AtomicU64,
    trials: AtomicU64,
    onsets: AtomicU64,
    branches: AtomicU64,
    unlocked_alive: AtomicU64,
    passes: AtomicU64,
    best: Mutex<f64>,
    events: Mutex<std::fs::File>,
    winner: Mutex<Option<Vec<u8>>>,
}

fn in_world(p: [f64; 2]) -> bool {
    p[0].is_finite() && p[1].is_finite() && p[0] > 0.0 && p[0] < 320.0 && p[1] > -80.0 && p[1] < 164.0
}

/// A re-rolled input keeping the original's eject bits.
fn choose(rng: &mut Rng, orig: u8) -> u8 {
    const POSE: [u8; 9] = [0, 1, 2, 4, 8, 1 | 4, 2 | 4, 1 | 8, 2 | 8];
    let mut m = POSE[rng.below(9) as usize];
    if rng.below(10) < 8 {
        m |= 16;
    }
    m | (orig & (32 | 64 | 128))
}

pub fn run(cfg: Config) -> Result<(), String> {
    std::fs::create_dir_all(&cfg.run_dir).map_err(|e| e.to_string())?;
    let template = Sim::new(crate::LEVEL, crate::CHAR)?;
    let sky = Skyline::new(&template.ctx);
    let mut pool = Vec::new();
    let mut uniq = HashSet::new();
    for m in &cfg.seeds {
        if m.len() > cfg.window + 2 && uniq.insert(m.clone()) {
            pool.push(Member { masks: m.clone(), score: 0.0, visits: 0 });
        }
    }
    if pool.is_empty() {
        return Err("no usable seeds".into());
    }
    eprintln!("onset: {} seed lineages", pool.len());
    let shared = Shared {
        pool: Mutex::new(pool),
        seen: Mutex::new(HashSet::new()),
        stop: AtomicBool::new(false),
        frames: AtomicU64::new(0),
        trials: AtomicU64::new(0),
        onsets: AtomicU64::new(0),
        branches: AtomicU64::new(0),
        unlocked_alive: AtomicU64::new(0),
        passes: AtomicU64::new(0),
        best: Mutex::new(f64::NEG_INFINITY),
        events: Mutex::new(std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("events.jsonl")).map_err(|e| e.to_string())?),
        winner: Mutex::new(None),
    };
    let start = Instant::now();
    let deadline = start + Duration::from_secs_f64(cfg.hours * 3600.0);
    let shared: &'static Shared = Box::leak(Box::new(shared));
    let template: &'static Sim = Box::leak(Box::new(template));
    let sky: &'static Skyline = Box::leak(Box::new(sky));
    let cfg: &'static Config = Box::leak(Box::new(cfg));
    let beats: &'static Vec<AtomicU64> = Box::leak(Box::new((0..cfg.threads).map(|_| AtomicU64::new(0)).collect()));
    for w in 0..cfg.threads {
        std::thread::Builder::new()
            .stack_size(32 << 20)
            .spawn(move || worker(w as u64, cfg, template, sky, shared, &beats[w], start))
            .unwrap();
    }
    let mut st = std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("stats.jsonl")).unwrap();
    loop {
        std::thread::sleep(Duration::from_secs(10));
        let stop = Instant::now() > deadline || cfg.run_dir.join("STOP").exists() || shared.stop.load(Ordering::Relaxed);
        let t = start.elapsed().as_secs_f64();
        let now_ms = start.elapsed().as_millis() as u64;
        let stuck = beats.iter().filter(|b| now_ms.saturating_sub(b.load(Ordering::Relaxed)) > 30_000).count();
        let line = format!(
            "{{\"t\":{:.0},\"fps\":{:.0},\"trials\":{},\"onsets\":{},\"branches\":{},\"unlocked_alive\":{},\"passes\":{},\"pool\":{},\"best\":{:.2},\"stuck\":{},\"winner\":{}}}",
            t,
            shared.frames.load(Ordering::Relaxed) as f64 / t,
            shared.trials.load(Ordering::Relaxed),
            shared.onsets.load(Ordering::Relaxed),
            shared.branches.load(Ordering::Relaxed),
            shared.unlocked_alive.load(Ordering::Relaxed),
            shared.passes.load(Ordering::Relaxed),
            shared.pool.lock().unwrap().len(),
            *shared.best.lock().unwrap(),
            stuck,
            shared.winner.lock().unwrap().is_some()
        );
        writeln!(st, "{line}").ok();
        eprintln!("{line}");
        if stop {
            shared.stop.store(true, Ordering::Relaxed);
            std::thread::sleep(Duration::from_secs(5));
            if let Ok(mut out) = std::fs::File::create(cfg.run_dir.join("pool.jsonl")) {
                for m in shared.pool.lock().unwrap().iter() {
                    writeln!(out, "{{\"score\":{:.2},\"frames\":{},\"visits\":{},\"tas\":\"{}\"}}", m.score, m.masks.len(), m.visits, tas_text(&m.masks)).ok();
                }
            }
            std::process::exit(if shared.winner.lock().unwrap().is_some() { 0 } else { 1 });
        }
    }
}

fn pick(_cfg: &Config, shared: &Shared, rng: &mut Rng) -> Member {
    let mut p = shared.pool.lock().unwrap();
    let n = p.len();
    let w: Vec<f64> = p.iter().map(|m| (1.0 + 4.0 * m.score) / (1.0 + m.visits as f64).sqrt()).collect();
    let total: f64 = w.iter().sum();
    let mut x = rng.unit() * total;
    let mut i = n - 1;
    for (j, wj) in w.iter().enumerate() {
        x -= wj;
        if x <= 0.0 {
            i = j;
            break;
        }
    }
    p[i].visits += 1;
    p[i].clone()
}

fn log(shared: &Shared, line: String) {
    writeln!(shared.events.lock().unwrap(), "{line}").ok();
}

/// Result of one release branch.
struct Branch {
    unlocked: bool,
    alive: bool,
    sky: bool,
    near: f64,
    finish: u32,
    end: [f64; 2],
    masks: Vec<u8>,
}

/// Play a release from `cur` (a held-lock state) to the frame limit.
fn release_branch(cfg: &Config, sky: &Skyline, br: &mut Sim, shared: &Shared, beat: &AtomicU64, start: Instant) -> Branch {
    let mut b = Branch { unlocked: false, alive: false, sky: false, near: f64::INFINITY, finish: 0, end: [f64::NAN; 2], masks: Vec::new() };
    let mut locked_run = 0;
    while br.frame() < cfg.max_frame {
        let f: Fr = br.step(cfg.release);
        shared.frames.fetch_add(1, Ordering::Relaxed);
        beat.store(start.elapsed().as_millis() as u64, Ordering::Relaxed);
        if f.port_abort {
            break;
        }
        if f.finished && f.alive_before {
            b.finish = f.frame;
            b.masks = br.masks[..br.replay.iteration as usize].to_vec();
            break;
        }
        if f.site != 0 {
            if !b.unlocked {
                locked_run += 1;
                if locked_run > 4 {
                    break;
                }
            }
            continue;
        }
        if !b.unlocked {
            b.unlocked = true;
            b.alive = f.alive_after;
            b.sky = f.alive_after && sky.above(f.chest_after);
        }
        if !f.alive_after {
            break;
        }
        b.near = b.near.min(finish_distance(f.chest_after));
        b.end = f.chest_after;
    }
    if b.masks.is_empty() {
        b.masks = br.masks[..br.replay.iteration as usize].to_vec();
    }
    b
}

fn worker(w: u64, cfg: &Config, template: &Sim, sky: &Skyline, shared: &Shared, beat: &AtomicU64, start: Instant) {
    let mut rng = Rng(cfg.seed ^ (w + 1).wrapping_mul(0x5851_f42d_4c95_7f2d));
    let mut base = template.fork();
    let mut s = template.fork();
    let mut br = template.fork();
    while !shared.stop.load(Ordering::Relaxed) {
        let m = pick(cfg, shared, &mut rng);
        let l = m.masks.len();
        let root = l.saturating_sub(cfg.window).max(1);
        base.restore(template);
        base.masks.clear();
        base.masks.extend_from_slice(&m.masks[..root]);
        let mut ok = true;
        while (base.replay.iteration as usize) < root {
            let f = base.step(0);
            beat.store(start.elapsed().as_millis() as u64, Ordering::Relaxed);
            shared.frames.fetch_add(1, Ordering::Relaxed);
            if f.port_abort || !f.alive_after || f.site != 0 || base.frame() >= cfg.max_frame {
                ok = false;
                break;
            }
        }
        if !ok {
            continue;
        }
        for _ in 0..cfg.batch {
            if shared.stop.load(Ordering::Relaxed) {
                break;
            }
            shared.trials.fetch_add(1, Ordering::Relaxed);
            let from = root + rng.below((l - root) as u64) as usize;
            let mut masks = m.masks[..from].to_vec();
            let orig = m.masks[from];
            masks.push(loop {
                let c = choose(&mut rng, orig);
                if c != orig {
                    break c;
                }
            });
            for t in from + 1..l {
                masks.push(if rng.below(10) < 7 { m.masks[t] } else { choose(&mut rng, m.masks[t]) });
            }
            let hold = *masks.last().unwrap() & !(32 | 64 | 128);
            s.restore(&base);
            s.masks.clear();
            s.masks.extend_from_slice(&masks);
            // Play to the onset.
            let mut onset: Option<Fr> = None;
            while (s.frame() as usize) < l + cfg.extra as usize && s.frame() < cfg.max_frame {
                let f = s.step(hold);
                beat.store(start.elapsed().as_millis() as u64, Ordering::Relaxed);
                shared.frames.fetch_add(1, Ordering::Relaxed);
                if f.port_abort || f.step_us > 80_000 {
                    break;
                }
                if f.site == 2 && f.alive_before {
                    onset = Some(f);
                    break;
                }
                if f.site != 0 || !f.alive_after {
                    break;
                }
            }
            let Some(of) = onset else { continue };
            shared.onsets.fetch_add(1, Ordering::Relaxed);
            s.masks.truncate(s.replay.iteration as usize);
            let onset_masks = s.masks.clone();
            let kick = ((of.chest_after[0] - of.chest_before[0]).powi(2) + (of.chest_after[1] - of.chest_before[1]).powi(2)).sqrt();
            let big = !(kick <= 2.0) || std::env::var("OBBY_ALL_ONSETS").is_ok();
            if big && !shared.seen.lock().unwrap().insert((of.frame, of.chest_after[0].to_bits(), of.chest_after[1].to_bits())) {
                continue;
            }
            // Follow the held lock to the frame limit (or its natural end, then the free flight).
            // At every lock frame with the chest in open sky inside the world, branch a release.
            let mut drift_near = f64::INFINITY;
            let mut drift_at = 0u32;
            let mut after_near = f64::INFINITY;
            let mut held = 0u32;
            let mut sky_frames = 0u32;
            let mut released = 0u32;
            let mut unlocked_at = 0u32;
            let mut best_rel = f64::INFINITY;
            while big && s.frame() < cfg.max_frame && !shared.stop.load(Ordering::Relaxed) {
                let c = chest(&s.ctx);
                if unlocked_at == 0 {
                    let d = finish_distance(c);
                    if d < drift_near {
                        drift_near = d;
                        drift_at = s.frame();
                    }
                    let neck = neck_gap(&s.ctx);
                    if in_world(c) && sky.above(c) && neck < 0.45 && held < cfg.lock_span {
                        sky_frames += 1;
                        br.restore(&s);
                        shared.branches.fetch_add(1, Ordering::Relaxed);
                        released += 1;
                        let b = release_branch(cfg, sky, &mut br, shared, beat, start);
                        if b.unlocked && b.alive {
                            shared.unlocked_alive.fetch_add(1, Ordering::Relaxed);
                            best_rel = best_rel.min(b.near);
                        }
                        if b.unlocked && b.alive && b.sky {
                            shared.passes.fetch_add(1, Ordering::Relaxed);
                            log(shared, format!(
                                "{{\"kind\":\"pass\",\"onset\":{},\"release\":{},\"chest\":[{:.2},{:.2}],\"near\":{:.2},\"end\":[{:.2},{:.2}],\"finish\":{},\"tas\":\"{}\"}}",
                                of.frame, s.frame() + 1, c[0], c[1], b.near, b.end[0], b.end[1], b.finish, tas_text(&b.masks)
                            ));
                        }
                        if b.finish != 0 {
                            log(shared, format!("{{\"kind\":\"finish\",\"frame\":{},\"tas\":\"{}\"}}", b.finish, tas_text(&b.masks)));
                            std::fs::write(cfg.run_dir.join("winner-1.tas"), tas_text(&b.masks)).ok();
                            *shared.winner.lock().unwrap() = Some(b.masks.clone());
                            shared.stop.store(true, Ordering::Relaxed);
                            return;
                        }
                    }
                }
                let f = s.step(hold);
                shared.frames.fetch_add(1, Ordering::Relaxed);
                beat.store(start.elapsed().as_millis() as u64, Ordering::Relaxed);
                if f.port_abort {
                    break;
                }
                if f.finished && f.alive_before {
                    let used = s.masks[..s.replay.iteration as usize].to_vec();
                    log(shared, format!("{{\"kind\":\"finish\",\"frame\":{},\"tas\":\"{}\"}}", f.frame, tas_text(&used)));
                    std::fs::write(cfg.run_dir.join("winner-1.tas"), tas_text(&used)).ok();
                    *shared.winner.lock().unwrap() = Some(used);
                    shared.stop.store(true, Ordering::Relaxed);
                    return;
                }
                if unlocked_at == 0 {
                    held += 1;
                    if f.site == 0 {
                        unlocked_at = f.frame;
                    }
                }
                if unlocked_at != 0 {
                    if f.site == 0 && !f.alive_after {
                        break;
                    }
                    if f.alive_after {
                        after_near = after_near.min(finish_distance(f.chest_after));
                    }
                }
            }
            if big {
                let candidate = drift_near < 10.0 || after_near < 10.0 || best_rel < 10.0;
                log(shared, format!(
                    "{{\"kind\":\"{}\",\"onset\":{},\"p0\":[{:.2},{:.2}],\"p1\":[{:.2},{:.2}],\"kick\":{:.1},\"held\":{},\"unlocked\":{},\"sky_frames\":{},\"released\":{},\"drift_near\":{:.1},\"drift_at\":{},\"after_near\":{:.1},\"rel_near\":{:.1},\"tas\":\"{}\"}}",
                    if candidate { "candidate" } else { "onset" }, of.frame, of.chest_before[0], of.chest_before[1], of.chest_after[0], of.chest_after[1], kick.min(1e9), held, unlocked_at,
                    sky_frames, released, drift_near.min(1e9), drift_at, after_near.min(1e9), best_rel.min(1e9), tas_text(&onset_masks)
                ));
            }
            // Parent quality: a big kick (Dad coupled to the exploding system) is what we need.
            let score = if big { 1.0 } else { 0.0 };
            {
                let mut b = shared.best.lock().unwrap();
                let v = -drift_near.min(after_near).min(best_rel).min(1e9);
                if big && v > *b {
                    *b = v;
                }
            }
            let child = Member { masks: onset_masks, score, visits: 0 };
            let mut p = shared.pool.lock().unwrap();
            if p.len() < cfg.pool {
                p.push(child);
            } else {
                // Replace the most-visited member of no better quality.
                let mut mi = usize::MAX;
                for (i, m) in p.iter().enumerate() {
                    if m.score <= score && (mi == usize::MAX || m.visits > p[mi].visits) {
                        mi = i;
                    }
                }
                if mi != usize::MAX {
                    p[mi] = child;
                }
            }
        }
    }
}
