//! `cqe`: grow a child-arm QE at the start (the obby.mp4 setup: Dad rocks back into the child
//! until an elbow breaks into a ligament, the ligament tears and its anchor whips), keeping Dad
//! alive, healthy and on the bike at the start with both start mines intact for a later launch.
//!
//! Go-Explore over input prefixes with a cell archive (child broken joints, ligament and tear
//! counts, coarse score, frame bucket; best member per cell). A trial picks a cell, re-rolls its
//! inputs from a cut near its end, plays on for up to `--grow` frames and offers every visited
//! cell's best prefix to the archive.
//! Score (higher is better): torn ligament anchors whipping (log speed), then a child ligament
//! close to tearing, then the child's elbow/knee reaction force against its break limit; minus a
//! small per-frame cost so earlier QEs win.
use crate::sim::{chest, tas_text, Sim};
use crate::window::Rng;
use hw_core::b2::joints;
use hw_core::NONE;
use std::collections::HashMap;
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
    pub grow: u32,
    pub temp: f64,
    pub x_lo: f64,
    pub x_hi: f64,
    /// Mines (bitmask of special indices) that must stay intact.
    pub mines: u64,
    pub seeds: Vec<Vec<u8>>,
}

#[derive(Clone)]
struct Member {
    masks: Vec<u8>,
    score: f64,
    visits: u64,
}

/// Per-frame QE score of the current state and its archive cell, or None if unusable.
pub fn frame_score(s: &Sim, x_lo: f64, x_hi: f64, need: u64) -> Option<(f64, f64, u64)> {
    let ctx = &s.ctx;
    static ALLOW_EJECT: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    let allow_eject = *ALLOW_EJECT.get_or_init(|| std::env::var("OBBY_CQE_EJECT").is_ok());
    if ctx.level.dead.get() || ctx.character.primary.dying.get() || (!allow_eject && ctx.character.primary.bicycle.ejected.get()) {
        return None;
    }
    let c = chest(ctx);
    if !(c[0] > x_lo && c[0] < x_hi && c[1] > 70.0 && c[1] < 84.0) {
        return None;
    }
    if crate::qe::mines(ctx) & need != need {
        return None;
    }
    let (speed, _) = crate::qe::anchors(ctx);
    let mut torn = 0u64;
    // Best torn ligament: whip speed and anchor-to-node separation (ignition needs the anchor to
    // leave node0's contact, about 0.16 m, so the two are solved in different islands).
    let mut tq = 0.0f64;
    for ch in [&ctx.character.primary, &ctx.character.child] {
        for i in 0..ch.composite_count as usize {
            let cp = ch.composites[i];
            if cp.kind != 0 {
                continue;
            }
            let j = cp.ends[0];
            if j != NONE && ctx.world.joints.is_alive(j) && ctx.world.joint(j).broken.get() {
                torn += 1;
                let (a, n) = (ctx.world.joint(j).body1, ctx.world.joint(j).body2);
                let va = ctx.world.body(a).linear_velocity;
                let sp = (va.x * va.x + va.y * va.y).sqrt();
                let pa = hw_core::b2::body::get_world_center(ctx, a);
                let pn = hw_core::b2::body::get_world_center(ctx, n);
                let d = ((pa.x - pn.x).powi(2) + (pa.y - pn.y).powi(2)).sqrt();
                if sp.is_finite() && d.is_finite() {
                    tq = tq.max(30.0 * (1.0 + sp).log10() + 40.0 * (1.0 + 10.0 * d.min(1000.0)).log10());
                }
            }
        }
    }
    let ch = &ctx.character.child;
    let score = if torn > 0 {
        200.0 + tq + 20.0 * (torn as f64 - 1.0)
    } else {
        // Untorn child ligament: closeness to tearing.
        let mut lig = -1.0f64;
        for i in 0..ch.composite_count as usize {
            let cp = ch.composites[i];
            if cp.kind == 0 && cp.ends[0] != NONE && ctx.world.joints.is_alive(cp.ends[0]) {
                let imp = ctx.world.joint(cp.ends[0]).impulse.abs();
                lig = lig.max((imp / 0.5).min(1.0));
            }
        }
        if lig >= 0.0 {
            120.0 + 60.0 * lig
        } else {
            // Child elbows/knees: reaction force against the break limit.
            let mut r = 0.0f64;
            for (ji, li) in [(4usize, 8usize), (5, 8), (8, 10), (9, 10)] {
                let j = ch.joints[ji];
                if j == NONE || !ctx.world.joints.is_alive(j) || ctx.world.joint(j).broken.get() {
                    continue;
                }
                let f = joints::get_reaction_force(ctx, j);
                let m = (f.x * f.x + f.y * f.y).sqrt() / ch.break_limits[li] as f64;
                if m.is_finite() {
                    r = r.max(m.min(1.5));
                }
            }
            80.0 * r
        }
    };
    let mut broken = 0u64;
    for i in 0..10 {
        let j = ch.joints[i];
        if j == NONE || !ctx.world.joints.is_alive(j) || ctx.world.joint(j).broken.get() {
            broken |= 1 << i;
        }
    }
    let cell = broken | (torn.min(15) << 10) | ((ch.composite_count as u64).min(15) << 14) | (((score / 8.0).max(0.0) as u64).min(255) << 20);
    Some((score, speed, cell))
}

fn choose(rng: &mut Rng, prev: u8) -> u8 {
    if rng.below(10) < 6 {
        return prev;
    }
    const POSE: [u8; 9] = [0, 1, 2, 4, 8, 1 | 4, 2 | 4, 1 | 8, 2 | 8];
    let mut m = POSE[rng.below(9) as usize];
    if rng.below(10) < 2 {
        m |= 16;
    }
    m
}

fn select(archive: &Mutex<HashMap<u64, Member>>, rng: &mut Rng, temp: f64) -> Option<Member> {
    let mut a = archive.lock().unwrap();
    if a.is_empty() {
        return None;
    }
    let keys: Vec<u64> = a.keys().copied().collect();
    let k = if rng.below(2) == 0 {
        keys[rng.below(keys.len() as u64) as usize]
    } else {
        let mx = a.values().map(|m| m.score).fold(f64::NEG_INFINITY, f64::max);
        let wts: Vec<f64> = keys.iter().map(|k| ((a[k].score - mx) / temp).exp() / (1.0 + a[k].visits as f64).sqrt()).collect();
        let tot: f64 = wts.iter().sum();
        let mut x = rng.unit() * tot;
        let mut pick = keys[keys.len() - 1];
        for (j, wj) in wts.iter().enumerate() {
            x -= wj;
            if x <= 0.0 {
                pick = keys[j];
                break;
            }
        }
        pick
    };
    let m = a.get_mut(&k).unwrap();
    m.visits += 1;
    Some(m.clone())
}

pub fn run(cfg: Config) -> Result<(), String> {
    std::fs::create_dir_all(&cfg.run_dir).map_err(|e| e.to_string())?;
    let template = Sim::new(crate::LEVEL, crate::CHAR)?;
    let mut seeds = cfg.seeds.clone();
    if seeds.is_empty() {
        seeds.push(vec![0u8; 4]);
    }
    let archive: Mutex<HashMap<u64, Member>> = Mutex::new(HashMap::new());
    let stop = AtomicBool::new(false);
    let trials = AtomicU64::new(0);
    let frames = AtomicU64::new(0);
    let best = Mutex::new((f64::NEG_INFINITY, 0.0f64, 0u32));
    let start = Instant::now();
    let deadline = start + Duration::from_secs_f64(cfg.hours * 3600.0);
    let events = Mutex::new(std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("events.jsonl")).map_err(|e| e.to_string())?);
    std::thread::scope(|scope| {
        for w in 0..cfg.threads {
            let (archive, seeds, stop, trials, frames, best, events, template, cfg) = (&archive, &seeds, &stop, &trials, &frames, &best, &events, &template, &cfg);
            std::thread::Builder::new().stack_size(32 << 20).spawn_scoped(scope, move || {
                let mut rng = Rng((cfg.seed.wrapping_add(w as u64 + 1)).wrapping_mul(0xd1b5_4a32_d192_ed03) ^ 0x2545_f491_4f6c_dd1d);
                let mut s = template.fork();
                let mut logged = 0u32;
                while !stop.load(Ordering::Relaxed) {
                    let parent = match select(archive, &mut rng, cfg.temp) {
                        Some(m) if rng.below(20) != 0 => m,
                        _ => Member { masks: seeds[rng.below(seeds.len() as u64) as usize].clone(), score: 0.0, visits: 0 },
                    };
                    trials.fetch_add(1, Ordering::Relaxed);
                    let l = parent.masks.len();
                    let cut = l.saturating_sub(rng.below(16) as usize).max(1);
                    let mut masks = parent.masks[..cut].to_vec();
                    let target = (cut as u32 + 4 + rng.below(cfg.grow as u64) as u32).min(cfg.max_frame) as usize;
                    let mut prev = *masks.last().unwrap_or(&0);
                    while masks.len() < target {
                        prev = choose(&mut rng, prev);
                        masks.push(prev);
                    }
                    s.restore(template);
                    s.masks = masks.clone();
                    let (mut bs, mut bat, mut bspeed) = (f64::NEG_INFINITY, 0usize, 0.0f64);
                    let mut cells: HashMap<u64, (f64, usize)> = HashMap::new();
                    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        while (s.replay.iteration as usize) < masks.len() && s.frame() < cfg.max_frame {
                            let f = s.step(0);
                            frames.fetch_add(1, Ordering::Relaxed);
                            if f.port_abort || f.site != 0 || !f.alive_after {
                                break;
                            }
                            match frame_score(&s, cfg.x_lo, cfg.x_hi, cfg.mines) {
                                Some((sc, sp, cell)) => {
                                    let sc = sc - 0.2 * f.frame as f64;
                                    let at = s.replay.iteration as usize;
                                    let e = cells.entry(cell | (((f.frame / 10) as u64) << 40)).or_insert((f64::NEG_INFINITY, 0));
                                    if sc > e.0 {
                                        *e = (sc, at);
                                    }
                                    if sc > bs {
                                        bs = sc;
                                        bat = at;
                                        bspeed = sp;
                                    }
                                }
                                None => {
                                    if f.frame > 8 {
                                        break;
                                    }
                                }
                            }
                        }
                    }));
                    if r.is_err() {
                        s = template.fork();
                        continue;
                    }
                    if bat < 2 || !bs.is_finite() {
                        continue;
                    }
                    {
                        let mut b = best.lock().unwrap();
                        if bs > b.0 {
                            *b = (bs, bspeed, bat as u32);
                            std::fs::write(cfg.run_dir.join("best.tas"), tas_text(&masks[..bat])).ok();
                        }
                    }
                    if bspeed > 1e4 && logged < 50 {
                        logged += 1;
                        writeln!(events.lock().unwrap(), "{{\"kind\":\"whip\",\"frame\":{},\"speed\":{:.3e},\"score\":{:.1},\"tas\":\"{}\"}}", bat, bspeed, bs, tas_text(&masks[..bat])).ok();
                    }
                    let mut a = archive.lock().unwrap();
                    for (key, (sc, at)) in cells {
                        if at < 2 {
                            continue;
                        }
                        match a.get_mut(&key) {
                            Some(m) if m.score >= sc => {}
                            Some(m) => {
                                let v = m.visits;
                                *m = Member { masks: masks[..at].to_vec(), score: sc, visits: v };
                            }
                            None => {
                                a.insert(key, Member { masks: masks[..at].to_vec(), score: sc, visits: 0 });
                            }
                        }
                    }
                }
            }).unwrap();
        }
        let mut st = std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("stats.jsonl")).unwrap();
        loop {
            std::thread::sleep(Duration::from_secs(10));
            let t = start.elapsed().as_secs_f64();
            let b = *best.lock().unwrap();
            let line = format!(
                "{{\"t\":{:.0},\"fps\":{:.0},\"trials\":{},\"cells\":{},\"best\":{:.1},\"speed\":{:.3e},\"at\":{}}}",
                t,
                frames.load(Ordering::Relaxed) as f64 / t,
                trials.load(Ordering::Relaxed),
                archive.lock().unwrap().len(),
                b.0,
                b.1,
                b.2
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
        for m in archive.lock().unwrap().values() {
            writeln!(out, "{{\"score\":{:.1},\"frames\":{},\"tas\":\"{}\"}}", m.score, m.masks.len(), tas_text(&m.masks)).ok();
        }
    }
    Ok(())
}
