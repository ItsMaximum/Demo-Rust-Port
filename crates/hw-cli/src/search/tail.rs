//! `hw-search tail`: exhaustive low-input tails in effective-input space.
//!
//! After eject the effective input of a frame is one of 14 classes: pose (by key priority:
//! ctrl 7 > shift 6 > up/down 3/4 > left/right 1/2 > 0) x grab (space). Masks in one class give
//! identical physics. Keep a finishing string up to entry `cut`, hold its class from there, and
//! try every tail with up to `k` class changes at entries in [cut, cut + window). Each change
//! costs about one frame-perfect input, so a finish found here has roughly
//! (frame-perfect inputs before the cut) + k of them. Finishes are canonicalized and scored with
//! `sparse::fp_count` like the fp-mode search.
use super::sim::{tas_text, Sim};
use super::sparse::{canonicalize, finish_of, fp_count};
use serde_json::json;
use std::collections::HashSet;
use std::io::Write;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::Mutex;
use std::time::Instant;

pub struct Config {
    pub level: u32,
    pub character: u32,
    pub base: Vec<u8>,
    pub cut: usize,
    pub window: usize,
    pub k: usize,
    pub limit: u32,
    pub min_launch: f64,
    pub threads: usize,
    pub run_dir: std::path::PathBuf,
    pub seed: u64,
    /// Held mask from the cut (default: the base's entry before the cut).
    pub hold: Option<u8>,
    /// Changes start this many entries after the cut.
    pub from: usize,
    /// Chest bounds entering a launch frame (Satan: in front of the teeth).
    pub bounds: Option<[f64; 4]>,
    /// Skip tails whose whole string has a larger aspect cost (0 = no limit).
    pub max_cost: usize,
}

/// Post-eject pose of a mask (motorcart input priority).
pub fn pose(m: u8) -> u8 {
    let b = |k: u8| (m >> k) & 1 == 1;
    if b(6) {
        7
    } else if b(5) {
        6
    } else if b(2) != b(3) {
        if b(2) { 3 } else { 4 }
    } else if b(0) != b(1) {
        if b(0) { 1 } else { 2 }
    } else {
        0
    }
}

const POSES: [u8; 7] = [0, 1, 2, 3, 4, 6, 7];

/// One mask per class: pose key + space; z (free after eject) copied from `z`.
fn class_mask(class: u8, z: u8) -> u8 {
    let p = POSES[(class / 2) as usize];
    let key = match p {
        1 => 1,
        2 => 2,
        3 => 4,
        4 => 8,
        6 => 32,
        7 => 64,
        _ => 0,
    };
    key | if class % 2 == 1 { 16 } else { 0 } | (z & 128)
}

fn class_of(m: u8) -> u8 {
    let p = POSES.iter().position(|&x| x == pose(m)).unwrap() as u8;
    p * 2 + ((m >> 4) & 1)
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
}

pub fn run(cfg: Config) -> Result<(), String> {
    std::fs::create_dir_all(&cfg.run_dir).map_err(|e| e.to_string())?;
    let limit = cfg.limit as usize;
    if cfg.cut == 0 || cfg.cut >= limit {
        return Err("cut must be in 1..limit".into());
    }
    let hold = cfg.hold.unwrap_or(cfg.base[cfg.cut.min(cfg.base.len()) - 1]);
    let hold_class = class_of(hold);
    let mut held: Vec<u8> = cfg.base[..cfg.cut.min(cfg.base.len())].to_vec();
    held.resize(limit, hold);
    // Snapshots along the held trajectory, one before each entry in [cut, cut + window).
    let mut s = Sim::new(cfg.level, cfg.character)?;
    s.masks = held.clone();
    let start = cfg.cut + cfg.from;
    let end = (start + cfg.window).min(limit);
    let mut snaps: Vec<Sim> = Vec::new();
    let mut held_launch = None;
    while (s.replay.iteration as usize) < end {
        let i = s.replay.iteration as usize;
        if i >= start && snaps.len() == i - start {
            snaps.push(s.fork());
        }
        let f = s.step(0);
        if f.port_abort || !f.alive_after || s.replay.finish_frame != 0 {
            held_launch = Some(i);
            break;
        }
        if i >= cfg.cut && f.alive_before && f.displacement >= cfg.min_launch {
            held_launch = Some(i);
            break;
        }
    }
    let span = snaps.len();
    eprintln!("tail: cut {} hold {hold:#04x} (class {hold_class}), changes from entry {start}, {span} change frames, held launch/end at {held_launch:?}", cfg.cut);
    // Jobs: sorted change lists [(entry offset, class)], consecutive classes differ.
    let mut jobs: Vec<Vec<(usize, u8)>> = Vec::new();
    fn extend(jobs: &mut Vec<Vec<(usize, u8)>>, cur: &mut Vec<(usize, u8)>, from: usize, span: usize, prev: u8, k: usize) {
        if !cur.is_empty() {
            jobs.push(cur.clone());
        }
        if cur.len() == k {
            return;
        }
        for t in from..span {
            for c in 0..14u8 {
                if c == prev {
                    continue;
                }
                cur.push((t, c));
                extend(jobs, cur, t + 1, span, c, k);
                cur.pop();
            }
        }
    }
    extend(&mut jobs, &mut Vec::new(), 0, span, hold_class, cfg.k);
    // Fewer changes first; shuffle within each size so partial runs sample evenly.
    let mut rng = Rng(cfg.seed);
    for i in (1..jobs.len()).rev() {
        let j = (rng.next() % (i as u64 + 1)) as usize;
        jobs.swap(i, j);
    }
    jobs.sort_by_key(|j| j.len());
    eprintln!("tail: {} tails", jobs.len());
    let next = AtomicUsize::new(0);
    let done = AtomicU64::new(0);
    let launches = AtomicU64::new(0);
    let finishes = AtomicU64::new(0);
    let seen = Mutex::new(HashSet::new());
    let best = Mutex::new((usize::MAX, usize::MAX, u32::MAX));
    let log = Mutex::new(std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("finishes.jsonl")).map_err(|e| e.to_string())?);
    let stop = AtomicBool::new(false);
    let bases = Mutex::new(std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("launches.jsonl")).map_err(|e| e.to_string())?);
    let prof = super::qe::profile(cfg.level);
    let t0 = Instant::now();
    std::thread::scope(|scope| {
        for _ in 0..cfg.threads {
            let (cfg, jobs, snaps, held, next, done, launches, finishes, seen, best, log, stop, bases, prof) =
                (&cfg, &jobs, &snaps, &held, &next, &done, &launches, &finishes, &seen, &best, &log, &stop, &bases, &prof);
            std::thread::Builder::new()
                .stack_size(16 << 20)
                .spawn_scoped(scope, move || {
                    let mut work = snaps[0].fork();
                    loop {
                        let i = next.fetch_add(1, Ordering::Relaxed);
                        if i >= jobs.len() || stop.load(Ordering::Relaxed) {
                            break;
                        }
                        let job = &jobs[i];
                        let mut masks = held.clone();
                        for (n, &(t, c)) in job.iter().enumerate() {
                            let e = start + t;
                            let upto = job.get(n + 1).map(|x| start + x.0).unwrap_or(masks.len());
                            for x in e..upto {
                                masks[x] = class_mask(c, held[x]);
                            }
                        }
                        if cfg.max_cost > 0 && super::events::aspect_cost(&masks) > cfg.max_cost {
                            done.fetch_add(1, Ordering::Relaxed);
                            continue;
                        }
                        work.restore(&snaps[job[0].0]);
                        work.masks.clear();
                        work.masks.extend_from_slice(&masks);
                        let mut lag = 0;
                        while work.frame() < cfg.limit {
                            let chest = work.ctx.character.bodies[hw_core::game::character::CHEST];
                            hw_core::lab::arm(&[chest]);
                            let f = work.step(0);
                            hw_core::lab::disarm();
                            let ev = hw_core::lab::take_events();
                            if f.port_abort || !f.alive_before {
                                break;
                            }
                            if work.replay.finish_frame != 0 {
                                finishes.fetch_add(1, Ordering::Relaxed);
                                let used = masks[..work.replay.iteration as usize].to_vec();
                                let c = canonicalize(cfg.level, cfg.character, &used, cfg.limit);
                                let fin = finish_of(cfg.level, cfg.character, &c, cfg.limit);
                                if fin != 0 {
                                    if let Some((fp, ev)) = fp_count(cfg.level, cfg.character, &c, cfg.limit, 1) {
                                        writeln!(log.lock().unwrap(), "{}", json!({"fp": fp, "events": ev, "finish": fin, "changes": job, "tas": tas_text(&c)})).ok();
                                        let mut b = best.lock().unwrap();
                                        if (fp, ev, fin) < *b {
                                            *b = (fp, ev, fin);
                                            let name = format!("best-fp{fp}-ev{ev}-f{fin}.tas");
                                            std::fs::write(cfg.run_dir.join(&name), tas_text(&c)).ok();
                                            eprintln!("IMPROVED: {fp} frame-perfect, {ev} events, finish {fin} -> {name}");
                                        }
                                    }
                                }
                                break;
                            }
                            if f.displacement >= cfg.min_launch {
                                let inb = cfg.bounds.is_none_or(|b| {
                                    let p = f.chest_before;
                                    p[0] >= b[0] && p[0] <= b[1] && p[1] >= b[2] && p[1] <= b[3]
                                });
                                if inb && seen.lock().unwrap().insert((f.chest_after[0].to_bits(), f.chest_after[1].to_bits())) {
                                    launches.fetch_add(1, Ordering::Relaxed);
                                    // Every distinct launch is a candidate base for aiming.
                                    let rel = super::qe::score(&work.ctx, &ev, chest, &prof);
                                    let used = masks[..work.replay.iteration as usize].to_vec();
                                    writeln!(bases.lock().unwrap(), "{}", json!({"frame": f.frame, "cost": super::events::aspect_cost(&used),
                                        "score": rel.score, "score_version": super::qe::SCORE_VERSION, "stage": rel.stage, "route": rel.route, "near": if rel.near.is_finite() { rel.near } else { -1.0 },
                                        "report_near": if rel.report_near.is_finite() { rel.report_near } else { -1.0 },
                                        "impact_near": if rel.impact_near.is_finite() { rel.impact_near } else { -1.0 },
                                        "strand_near": if rel.strand_near.is_finite() { rel.strand_near } else { -1.0 },
                                        "finish_inside": rel.finish_inside, "alive_after": f.alive_after, "chest_alive": f.chest_alive, "site": f.site,
                                        "to": f.chest_after, "changes": job, "tas": tas_text(&used)})).ok();
                                }
                                break;
                            }
                            if !f.alive_after {
                                break;
                            }
                            lag = if f.lagging { lag + 1 } else { 0 };
                            if lag >= super::sim::lag_frames() {
                                break;
                            }
                        }
                        let d = done.fetch_add(1, Ordering::Relaxed) + 1;
                        if d % 5000 == 0 {
                            eprintln!(
                                "{}",
                                json!({"t": t0.elapsed().as_secs(), "done": d, "of": jobs.len(), "distinct_launches": launches.load(Ordering::Relaxed),
                                    "finishes": finishes.load(Ordering::Relaxed), "best": *best.lock().unwrap(), "changes": job.len()})
                            );
                        }
                        if cfg.run_dir.join("STOP").exists() {
                            stop.store(true, Ordering::Relaxed);
                        }
                    }
                })
                .unwrap();
        }
    });
    eprintln!(
        "{}",
        json!({"t": t0.elapsed().as_secs(), "done": done.load(Ordering::Relaxed), "of": jobs.len(), "distinct_launches": launches.load(Ordering::Relaxed),
            "finishes": finishes.load(Ordering::Relaxed), "best": *best.lock().unwrap()})
    );
    Ok(())
}
