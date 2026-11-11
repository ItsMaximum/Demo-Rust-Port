//! `window`: local search around lineage peaks (like hw-search qe): replay a lineage once to
//! `len - window`, then run a batch of trials that change inputs from a random frame in the
//! window and play up to `extra` frames past the lineage end.
//!
//! Objective `pairs`: maximise the peak broadphase pair count (4096 = pool exhausted, the
//! pairmax throw). Every world-stage throw and every frame that leaves Dad alive with his chest
//! above the static skyline (open sky) is logged with its exact input; an above-skyline state
//! gets a large score bonus so the pool keeps those lineages.
use crate::sim::{tas_text, Fr, Sim, Skyline};
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
    pub window: usize,
    pub extra: u32,
    pub batch: usize,
    pub pool: usize,
    pub temp: f64,
    pub seeds: Vec<Vec<u8>>,
    /// "pairs" (maximise peak pair count) or "finish" (minimise launch distance to the finish).
    pub objective: String,
}

/// Start-area launch zone: chest under the spike ceiling (x 0..26, y 62..84) rising faster than
/// 15 m/s (pass the velocity as [0, -20] to test position only).
fn up_zone(c: [f64; 2], v: [f64; 2]) -> bool {
    c[0] > 0.0 && c[0] < 26.0 && c[1] > 62.0 && c[1] < 84.0 && v[1] < -15.0
}

/// A launch that, rising to the spike ceiling, arrives at about 40 degrees and 32-38 m/s
/// (the flight map's fast finishing arcs): 38..54 degrees at 33..48 m/s.
fn launch_ok(v: [f64; 2]) -> bool {
    let sp = (v[0] * v[0] + v[1] * v[1]).sqrt();
    let ang = (-v[1]).atan2(v[0]).to_degrees();
    (33.0..48.0).contains(&sp) && (38.0..54.0).contains(&ang)
}

/// Shaft-top launch zone: chest just under the corridor ceiling (x 24.5..35.5, y 67.3..72),
/// moving up and right (vx > 5, vy < -8).
fn corridor_zone(c: [f64; 2], v: [f64; 2]) -> bool {
    c[0] > 24.5 && c[0] < 35.5 && c[1] > 67.3 && c[1] < 72.0 && v[0] > 5.0 && v[1] < -8.0
}

/// Shaft launch zone: chest in the boost pit / shaft (x 20..31, y 67..92) rising faster than 15 m/s.
fn shaft_zone(c: [f64; 2], v: [f64; 2]) -> bool {
    c[0] > 20.0 && c[0] < 31.0 && c[1] > 67.0 && c[1] < 92.0 && v[1] < -15.0
}

/// Up-right launch under the spike ceiling: chest x 0..24.5, y 66.5..84, moving right (> 5 m/s)
/// and up (> 10 m/s) at 20..80 m/s.
fn upr_ok(c: [f64; 2], v: [f64; 2]) -> bool {
    let sp = v[0].hypot(v[1]);
    c[0] > 0.0 && c[0] < 24.5 && c[1] > 66.5 && c[1] < 80.0 && v[0] > 12.0 && v[1] < -15.0 && sp > 25.0 && sp < 80.0
}

/// `upr_ok` for a frame that actually moved the chest (not pinned against a wall).
fn upr_moving(c0: [f64; 2], c1: [f64; 2], v: [f64; 2]) -> bool {
    let d = (c1[0] - c0[0]).hypot(c1[1] - c0[1]);
    d > 0.5 && d < 5.0 && c1[0] > c0[0] && c1[1] < c0[1] && upr_ok(c1, v)
}

/// Corridor launch: chest under the corridor roof (x 24..36, y 66..75) moving 25..45 m/s at
/// 25..65 degrees up-right (through the roof into the flight-map finishing arcs).
fn upc2_ok(c: [f64; 2], v: [f64; 2]) -> bool {
    let sp = v[0].hypot(v[1]);
    let ang = (-v[1]).atan2(v[0]).to_degrees();
    c[0] > 24.0 && c[0] < 36.0 && c[1] > 66.0 && c[1] < 75.0 && sp > 25.0 && sp < 45.0 && ang > 25.0 && ang < 65.0
}

/// Start-area or corridor launch: chest x 0..36, y 64..85, moving 25..55 m/s at 30..60 degrees
/// up-right, actually moving (not pinned).
fn upx_ok(c0: [f64; 2], c: [f64; 2], v: [f64; 2]) -> bool {
    let sp = v[0].hypot(v[1]);
    let ang = (-v[1]).atan2(v[0]).to_degrees();
    let d = (c[0] - c0[0]).hypot(c[1] - c0[1]);
    c[0] > 0.0 && c[0] < 36.0 && c[1] > 64.0 && c[1] < 85.0 && sp > 25.0 && sp < 55.0 && ang > 30.0 && ang < 60.0 && d > 0.5 && d < 3.0
}

/// Dad is not dying, or will live at least 130 more frames (time for the arc to the finish).
fn life_ok(s: &Sim) -> bool {
    let d = &s.ctx.character.primary;
    !d.dying.get() || 150 - d.bleed_counter >= 130
}

/// Video-style zone: Dad's chest anywhere under the spike ceiling (x 0..24, y 68..86).
fn start_lie_zone(c: [f64; 2]) -> bool {
    c[0] > 0.0 && c[0] < 24.0 && c[1] > 68.0 && c[1] < 86.0
}

/// After a pairmax onset (state `s`, just past the onset frame): for each held input (Space or
/// Space + a lean) and hold length, play the hold then no input to frame 299. Returns the closest
/// living chest approach to the finish region, a finish frame (0 if none) and its input.
fn aftermath_sweep(s: &Sim, br: &mut Sim) -> (f64, u32, Vec<u8>) {
    const HOLDS: [(u8, u32); 9] = [(0, 0), (16, 2), (16, 4), (16, 8), (16, 14), (16, 22), (24, 6), (17, 6), (18, 6)];
    let mut near = f64::INFINITY;
    for &(hm, h) in HOLDS.iter() {
        br.restore(s);
        let i0 = br.replay.iteration as usize;
        br.masks.truncate(i0);
        for _ in 0..h {
            br.masks.push(hm);
        }
        let t0 = std::time::Instant::now();
        while br.frame() < 299 {
            let f = br.step(0);
            if f.port_abort || t0.elapsed().as_millis() > 1500 {
                break;
            }
            if f.finished && f.alive_before {
                let used = br.masks[..br.replay.iteration as usize].to_vec();
                return (0.0, f.frame, used);
            }
            if f.site != 0 {
                continue;
            }
            if !f.alive_after {
                break;
            }
            near = near.min(finish_distance(f.chest_after));
        }
    }
    (near, 0, Vec::new())
}

/// Closest approach to the finish region of the ballistic path (gravity only, velocity clamped to
/// 200 m/s) from a chest state at `frame`, up to frame 299.
pub fn ballistic_finish_distance(p: [f64; 2], v: [f64; 2], frame: u32) -> f64 {
    if !(p[0].is_finite() && p[1].is_finite() && v[0].is_finite() && v[1].is_finite()) {
        return f64::INFINITY;
    }
    let (mut x, mut y) = (p[0], p[1]);
    let sp = v[0].hypot(v[1]);
    let k = if sp > 200.0 { 200.0 / sp } else { 1.0 };
    let (mut vx, mut vy) = (v[0] * k, v[1] * k);
    let mut best = finish_distance([x, y]);
    let dt = 1.0 / 30.0;
    for _ in frame..299 {
        vy += 10.0 * dt;
        x += vx * dt;
        y += vy * dt;
        best = best.min(finish_distance([x, y]));
    }
    best
}

/// Finish focus region (FinishLine #120): x (117.312, 123.712), y (55.504, 58.704) metres.
pub fn finish_distance(p: [f64; 2]) -> f64 {
    if !p[0].is_finite() || !p[1].is_finite() {
        return f64::INFINITY;
    }
    let dx = if p[0] < 117.312 { 117.312 - p[0] } else if p[0] > 123.712 { p[0] - 123.712 } else { 0.0 };
    let dy = if p[1] < 55.504 { 55.504 - p[1] } else if p[1] > 58.704 { p[1] - 58.704 } else { 0.0 };
    (dx * dx + dy * dy).sqrt()
}

/// Closest approach of the chest to the finish region this frame: its solved TOI impact points,
/// strand point and end position, plus any finish check (focus position read by the FinishLine).
/// Returns (distance, finish-check distance, route letters).
fn frame_route(events: &[hw_core::lab::Event], chest: hw_core::H, end: [f64; 2]) -> (f64, f64, String) {
    use hw_core::lab::Event;
    let mut near = finish_distance(end);
    let mut fc = f64::INFINITY;
    let mut route = String::new();
    let mut stranded = false;
    for e in events {
        match *e {
            Event::AfterToi { body, c0, .. } if body == chest && !stranded => {
                near = near.min(finish_distance(c0));
            }
            Event::Toi { body1, body2, .. } if (body1 == chest || body2 == chest) && !stranded => {
                if route.len() < 16 {
                    route.push('.');
                }
            }
            Event::Strand { body1, body2, pos1, pos2, .. } if (body1 == chest || body2 == chest) && !stranded => {
                stranded = true;
                near = near.min(finish_distance(if body1 == chest { pos1 } else { pos2 }));
            }
            Event::Finish { x, y, dead: false, .. } => {
                fc = fc.min(finish_distance([x, y]));
            }
            _ => {}
        }
    }
    (near, fc, route)
}

#[derive(Clone)]
struct Member {
    masks: Vec<u8>,
    score: f64,
    visits: u64,
}

struct Shared {
    pool: Mutex<Vec<Member>>,
    seen: Mutex<HashSet<(i32, u64, u64)>>,
    stop: AtomicBool,
    frames: AtomicU64,
    trials: AtomicU64,
    pairmax: AtomicU64,
    throws: AtomicU64,
    above: AtomicU64,
    best: Mutex<f64>,
    events: Mutex<std::fs::File>,
    winner: Mutex<Option<Vec<u8>>>,
}

pub struct Rng(pub u64);
impl Rng {
    pub fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    pub fn below(&mut self, n: u64) -> u64 {
        self.next() % n.max(1)
    }
    pub fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// A replacement input for a frame whose original input was `orig`. Eject bits (Shift Ctrl Z)
/// are kept from the original; arrows and Space are re-rolled (after ejection these are the
/// poses and the grab for both living actors).
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
    // Seeds: trim each to its peak-pairs frame (an end inside the trial window); replayed in parallel.
    let pool: Mutex<Vec<Member>> = Mutex::new(Vec::new());
    {
        let next = std::sync::atomic::AtomicUsize::new(0);
        let (cfgr, templater, poolr) = (&cfg, &template, &pool);
        std::thread::scope(|scope| {
            for _ in 0..cfgr.threads {
                let next = &next;
                std::thread::Builder::new()
                    .stack_size(32 << 20)
                    .spawn_scoped(scope, move || {
                        let mut s = templater.fork();
                        loop {
                            let i = next.fetch_add(1, Ordering::Relaxed);
                            if i >= cfgr.seeds.len() {
                                break;
                            }
                            let sd = &cfgr.seeds[i];
                            s.restore(templater);
                            s.masks = sd.clone();
                            let (mut best, mut at) = (0, 0usize);
                            while (s.replay.iteration as usize) < sd.len() && s.frame() < cfgr.max_frame {
                                let f = s.step(0);
                                if f.port_abort || !f.alive_after || f.step_us > 80_000 {
                                    break;
                                }
                                let sp = f.chest_v[0].hypot(f.chest_v[1]);
                                let ok = match cfgr.objective.as_str() {
                                    "fast" => sp > 25.0 && sp < 250.0,
                                    "upr" => upr_moving(f.chest_before, f.chest_after, f.chest_v),
                                    "upc2" => upc2_ok(f.chest_after, f.chest_v),
                                    "upx" => upx_ok(f.chest_before, f.chest_after, f.chest_v) && life_ok(&s),
                                    _ => true,
                                };
                                if f.pairs > best && ok {
                                    best = f.pairs;
                                    at = s.replay.iteration as usize;
                                }
                            }
                            let m = if cfgr.objective == "finish" || cfgr.objective == "pass" || cfgr.objective == "passfin" {
                                (sd.len() > cfgr.window + 2).then(|| Member { masks: sd.clone(), score: -1000.0, visits: 0 })
                            } else {
                                (at > cfgr.window + 2).then(|| Member { masks: sd[..at].to_vec(), score: best as f64, visits: 0 })
                            };
                            if let Some(m) = m {
                                poolr.lock().unwrap().push(m);
                            }
                        }
                    })
                    .unwrap();
            }
        });
    }
    let pool = pool.into_inner().unwrap();
    if pool.is_empty() {
        return Err("no usable seeds".into());
    }
    eprintln!("window: {} seed lineages, best seed peak {}", pool.len(), pool.iter().map(|m| m.score as i64).max().unwrap());
    let shared = Shared {
        pool: Mutex::new(pool),
        seen: Mutex::new(HashSet::new()),
        stop: AtomicBool::new(false),
        frames: AtomicU64::new(0),
        trials: AtomicU64::new(0),
        pairmax: AtomicU64::new(0),
        throws: AtomicU64::new(0),
        above: AtomicU64::new(0),
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
            "{{\"t\":{:.0},\"fps\":{:.0},\"trials\":{},\"pool\":{},\"best\":{:.0},\"pairmax\":{},\"throws\":{},\"above\":{},\"stuck\":{},\"winner\":{}}}",
            t,
            shared.frames.load(Ordering::Relaxed) as f64 / t,
            shared.trials.load(Ordering::Relaxed),
            shared.pool.lock().unwrap().len(),
            *shared.best.lock().unwrap(),
            shared.pairmax.load(Ordering::Relaxed),
            shared.throws.load(Ordering::Relaxed),
            shared.above.load(Ordering::Relaxed),
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
                    writeln!(out, "{{\"score\":{:.0},\"frames\":{},\"visits\":{},\"tas\":\"{}\"}}", m.score, m.masks.len(), m.visits, tas_text(&m.masks)).ok();
                }
            }
            std::process::exit(if shared.winner.lock().unwrap().is_some() { 0 } else { 1 });
        }
    }
}

fn pick(cfg: &Config, shared: &Shared, rng: &mut Rng) -> Member {
    let mut p = shared.pool.lock().unwrap();
    let n = p.len();
    let i = if rng.below(5) == 0 {
        rng.below(n as u64) as usize
    } else {
        let maxs = p.iter().map(|m| m.score).fold(f64::MIN, f64::max);
        let w: Vec<f64> = p.iter().map(|m| ((m.score - maxs) / cfg.temp).exp() / (1.0 + m.visits as f64).sqrt()).collect();
        let total: f64 = w.iter().sum();
        let mut x = rng.unit() * total;
        let mut k = n - 1;
        for (j, wj) in w.iter().enumerate() {
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
}

fn log_event(shared: &Shared, kind: &str, f: &Fr, masks: &[u8]) {
    static QUIET: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    if *QUIET.get_or_init(|| std::env::var("OBBY_QUIET").is_ok()) && kind != "finish" && kind != "near" {
        return;
    }
    writeln!(
        shared.events.lock().unwrap(),
        "{{\"kind\":\"{}\",\"frame\":{},\"site\":{},\"alive_before\":{},\"alive\":{},\"pairs\":{},\"chest_before\":[{:.3},{:.3}],\"chest\":[{:.3},{:.3}],\"cv\":[{:.1},{:.1}],\"speed_max\":{:.3e},\"fast\":{},\"dej\":{},\"cej\":{},\"tas\":\"{}\"}}",
        kind, f.frame, f.site, f.alive_before, f.alive_after, f.pairs, f.chest_before[0], f.chest_before[1], f.chest_after[0], f.chest_after[1],
        f.chest_v[0], f.chest_v[1], f.speed_max, f.fast_bodies, f.dad_ejected, f.child_ejected, tas_text(masks)
    )
    .ok();
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
            if f.port_abort || !f.alive_after || base.frame() >= cfg.max_frame {
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
            let first = loop {
                let c = choose(&mut rng, orig);
                if c != orig {
                    break c;
                }
            };
            masks.push(first);
            for t in from + 1..l {
                masks.push(if rng.below(10) < 7 { m.masks[t] } else { choose(&mut rng, m.masks[t]) });
            }
            let last = *masks.last().unwrap();
            s.restore(&base);
            s.masks.clear();
            s.masks.extend_from_slice(&masks);
            let (mut best, mut best_at, mut best_chest) = (0i32, 0usize, [0.0f64; 2]);
            let mut above_bonus = 0.0;
            let finish_obj = cfg.objective == "finish";
            let pass_obj = cfg.objective == "pass" || cfg.objective == "passfin";
            let passfin = cfg.objective == "passfin";
            // passfin: after the chest first reaches open sky alive, play on to frame 299 and keep
            // the closest living approach to the finish region.
            let mut passed_at = 0u32;
            let mut near_after = f64::INFINITY;
            // pass: best (upward chest move - skyline gap before) over pairmax frames with Dad alive
            // after the frame; positive means the chest passed into open sky.
            let mut best_pass = f64::NEG_INFINITY;
            let mut best_near = f64::INFINITY;
            let mut logged_throw = false;
            while ((s.frame() as usize) < l + cfg.extra as usize && s.frame() < cfg.max_frame) || (passed_at != 0 && s.frame() < 299) {
                let chest = crate::sim::chest_body(&s.ctx);
                if finish_obj {
                    hw_core::lab::arm(&[chest]);
                }
                let f = s.step(last);
                if finish_obj {
                    hw_core::lab::disarm();
                    let ev = hw_core::lab::take_events();
                    if f.alive_before {
                        let (near, fc, _) = frame_route(&ev, chest, f.chest_after);
                        let d = near.min(fc);
                        if d < best_near {
                            best_near = d;
                            best_at = s.replay.iteration as usize;
                            best_chest = f.chest_after;
                        }
                    }
                }
                beat.store(start.elapsed().as_millis() as u64, Ordering::Relaxed);
                shared.frames.fetch_add(1, Ordering::Relaxed);
                if f.port_abort || f.step_us > 80_000 {
                    break;
                }
                let used = &s.masks[..s.replay.iteration as usize];
                if f.finished && f.alive_before {
                    log_event(shared, "finish", &f, used);
                    std::fs::write(cfg.run_dir.join("winner-1.tas"), tas_text(used)).ok();
                    *shared.winner.lock().unwrap() = Some(used.to_vec());
                    shared.stop.store(true, Ordering::Relaxed);
                    return;
                }
                if f.site != 0 {
                    shared.throws.fetch_add(1, Ordering::Relaxed);
                    if f.site == 2 {
                        shared.pairmax.fetch_add(1, Ordering::Relaxed);
                    }
                    if !logged_throw {
                        logged_throw = true;
                        log_event(shared, if f.site == 2 { "pairmax" } else { "throw" }, &f, used);
                    }
                    if ((cfg.objective == "up" || cfg.objective == "upa") && f.site == 2 && f.alive_before && up_zone(f.chest_before, [0.0, -20.0]))
                        || (cfg.objective == "upc" && f.site == 2 && f.alive_before && corridor_zone(f.chest_before, [10.0, -10.0]))
                        || (cfg.objective == "ups" && f.site == 2 && f.alive_before && shaft_zone(f.chest_before, [0.0, -20.0]))
                        || (cfg.objective == "upv" && f.site == 2 && f.alive_before && start_lie_zone(f.chest_before))
                        || (cfg.objective == "fast" && f.site == 2 && f.alive_before && f.chest_v[0].hypot(f.chest_v[1]) > 25.0)
                        || (cfg.objective == "upr" && f.site == 2 && f.alive_before && upr_moving(f.chest_before, f.chest_after, f.chest_v))
                        || (cfg.objective == "upc2" && f.site == 2 && f.alive_before && upc2_ok(f.chest_before, f.chest_v))
                        || (cfg.objective == "upx" && f.site == 2 && f.alive_before && upx_ok(f.chest_before, f.chest_after, f.chest_v) && life_ok(&s))
                    {
                        best = 4096 + (300 - f.frame.min(300) as i32) * 20;
                        best_at = s.replay.iteration as usize;
                        best_chest = f.chest_after;
                        break;
                    }
                    if cfg.objective == "aft" && f.site == 2 && f.alive_before {
                        // Onset: sweep held-Space lengths and score the closest finish approach.
                        let (near, fin, used) = aftermath_sweep(&s, &mut br);
                        if fin != 0 {
                            log_event(shared, "finish", &f, &used);
                            std::fs::write(cfg.run_dir.join("winner-1.tas"), tas_text(&used)).ok();
                            *shared.winner.lock().unwrap() = Some(used);
                            shared.stop.store(true, Ordering::Relaxed);
                            return;
                        }
                        best = 7000 - (near.min(1000.0) * 2.0) as i32;
                        best_at = s.replay.iteration as usize;
                        best_chest = f.chest_after;
                        if near < 30.0 {
                            log_event(shared, "near", &f, &s.masks[..s.replay.iteration as usize]);
                        }
                        break;
                    }
                    if cfg.objective == "osky3" && f.site == 2 && f.alive_before {
                        // Onset: actually hold Space and follow the lock. Every held lock frame with
                        // the chest in open sky is a release candidate, scored against the
                        // flight-map finishing states (finish due before Dad bleeds out).
                        let d = &s.ctx.character.primary;
                        let deadline = if d.dying.get() { f.frame + (150 - d.bleed_counter).max(0) as u32 } else { 299 };
                        br.restore(&s);
                        let i0 = br.replay.iteration as usize;
                        br.masks.truncate(i0);
                        let mut fd = 1000.0f64;
                        let mut held = 0u32;
                        let mut gap = f64::INFINITY;
                        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            for _ in 0..45 {
                                let g = br.step(16);
                                if g.port_abort || g.site == 0 || !g.chest_after[0].is_finite() {
                                    break;
                                }
                                held += 1;
                                gap = gap.min(sky.gap(g.chest_after));
                                if sky.above(g.chest_after) {
                                    fd = fd.min(crate::onarch::flight_distance_by(g.chest_after, g.chest_v, g.frame, deadline));
                                }
                            }
                        }));
                        if r.is_err() {
                            std::mem::forget(std::mem::replace(&mut br, template.fork()));
                        }
                        best = if fd < 1000.0 { 8000 - (fd.min(150.0) * 20.0) as i32 } else { 5000 - (gap.max(0.0).min(40.0) * 20.0) as i32 + held.min(10) as i32 };
                        best_at = s.replay.iteration as usize;
                        best_chest = f.chest_after;
                        if fd < 5.0 {
                            log_event(shared, "near", &f, &s.masks[..s.replay.iteration as usize]);
                        }
                        break;
                    }
                    if cfg.objective == "osky" && f.site == 2 && f.alive_before {
                        // Onset: follow the ballistic path (a held lock) to its first point in open
                        // sky and score that state against the flight-map finishing states, with
                        // the finish due before Dad bleeds out.
                        let d = &s.ctx.character.primary;
                        let deadline = if d.dying.get() { f.frame + (150 - d.bleed_counter).max(0) as u32 } else { 299 };
                        let (mut x, mut y) = (f.chest_after[0], f.chest_after[1]);
                        let sp = f.chest_v[0].hypot(f.chest_v[1]);
                        let k = if sp > 200.0 { 200.0 / sp } else { 1.0 };
                        let (vx, mut vy) = (f.chest_v[0] * k, f.chest_v[1] * k);
                        let mut fd = 1000.0f64;
                        if x.is_finite() && y.is_finite() && vx.is_finite() && vy.is_finite() {
                            // Best release point along the held-lock ballistic path: every point in
                            // open sky is a candidate (release there, then fly freely).
                            for t in 0..75u32 {
                                if sky.above([x, y]) {
                                    fd = fd.min(crate::onarch::flight_distance_by([x, y], [vx, vy], f.frame + t, deadline));
                                }
                                vy += 10.0 / 30.0;
                                x += vx / 30.0;
                                y += vy / 30.0;
                            }
                        }
                        best = 8000 - (fd.min(150.0) * 20.0) as i32;
                        best_at = s.replay.iteration as usize;
                        best_chest = f.chest_after;
                        if fd < 6.0 {
                            log_event(shared, "near", &f, &s.masks[..s.replay.iteration as usize]);
                        }
                        break;
                    }
                    if cfg.objective == "ostate" && f.site == 2 && f.alive_before {
                        // Onset: score the ballistic path from Dad's chest state (a lock that lasts
                        // until arrival) by its closest approach to the finish region in time.
                        let bfd = ballistic_finish_distance(f.chest_after, f.chest_v, f.frame);
                        best = 7000 - (bfd.min(100.0) * 20.0) as i32;
                        best_at = s.replay.iteration as usize;
                        best_chest = f.chest_after;
                        if bfd < 8.0 {
                            log_event(shared, "near", &f, &s.masks[..s.replay.iteration as usize]);
                        }
                        break;
                    }
                    if cfg.objective == "earlykick" && f.site == 2 && f.alive_before {
                        // Onsets that kick Dad's chest (coupled to the explosion) win, earliest first;
                        // plain onsets rank below every kick onset but above any pair count.
                        let kick = (f.chest_after[0] - f.chest_before[0]).hypot(f.chest_after[1] - f.chest_before[1]);
                        let c = f.chest_after;
                        let kick = if kick < 400.0 && c[0] > -4.0 && c[0] < 324.0 && c[1] > -84.0 && c[1] < 164.0 { kick } else { 0.0 };
                        best = if kick > 20.0 { 10_000 + (300 - f.frame.min(300) as i32) * 20 } else { 5_000 + (300 - f.frame.min(300) as i32) * 5 };
                        best_at = s.replay.iteration as usize;
                        best_chest = f.chest_after;
                        if kick > 20.0 {
                            log_event(shared, "near", &f, &s.masks[..s.replay.iteration as usize]);
                        }
                        break;
                    }
                    if cfg.objective == "early" && f.site == 2 && f.alive_before {
                        // Earliest onset wins: stop at the first pairmax.
                        best = 4096 + (300 - f.frame.min(300) as i32) * 20;
                        best_at = s.replay.iteration as usize;
                        best_chest = f.chest_after;
                        break;
                    }
                }
                let above = f.alive_after && sky.above(f.chest_after);
                if above {
                    shared.above.fetch_add(1, Ordering::Relaxed);
                    if above_bonus == 0.0 {
                        log_event(shared, "above", &f, used);
                    }
                    above_bonus = 10_000.0;
                }
                if passfin && passed_at != 0 && f.alive_after && f.site == 0 {
                    near_after = near_after.min(finish_distance(f.chest_after));
                }
                if pass_obj && f.alive_after && f.site == 0 && sky.above(f.chest_after) {
                    if best_pass < 1000.0 {
                        best_at = s.replay.iteration as usize;
                        best_chest = f.chest_after;
                    }
                    best_pass = 1000.0;
                    if passfin && passed_at == 0 {
                        passed_at = f.frame;
                        near_after = finish_distance(f.chest_after);
                    }
                } else if pass_obj && f.site == 2 && f.alive_after && best_pass < 1000.0 {
                    let up = f.chest_before[1] - f.chest_after[1];
                    let y = f.chest_after[1];
                    let v = if sky.above(f.chest_after) {
                        1000.0
                    } else if y.is_finite() && y <= -79.0 {
                        // Overshoot out of the top of the world: smaller kicks are better.
                        -2.0 * (-79.0 - y + 1.0).log10()
                    } else {
                        up - sky.gap(f.chest_before)
                    };
                    if v > best_pass {
                        best_pass = v;
                        best_at = s.replay.iteration as usize;
                        best_chest = f.chest_after;
                    }
                }
                let counts = match cfg.objective.as_str() {
                    "up" => up_zone(f.chest_after, f.chest_v),
                    "upa" => up_zone(f.chest_after, f.chest_v) && launch_ok(f.chest_v),
                    "upc" => corridor_zone(f.chest_after, f.chest_v),
                    "ups" => shaft_zone(f.chest_after, f.chest_v),
                    "upv" => start_lie_zone(f.chest_after),
                    "upr" => upr_moving(f.chest_before, f.chest_after, f.chest_v),
                    "upc2" => upc2_ok(f.chest_after, f.chest_v),
                    // upx/osky: raw pairs guide the search; only the onset itself is judged.
                    "upx" | "osky" | "osky3" => life_ok(&s),
                    // Dad's chest moving faster than 25 m/s (a launch in progress).
                    "fast" => f.chest_v[0].hypot(f.chest_v[1]) > 25.0 && f.chest_v[0].hypot(f.chest_v[1]) < 250.0,
                    _ => true,
                };
                if cfg.objective == "apex" && f.site == 0 && f.alive_after && life_ok(&s) {
                    // Predicted apex of the chest's free flight (lower y = higher apex) for an
                    // up-right launch under the ceilings, measured on frames the chest moved.
                    let (c0, c, v) = (f.chest_before, f.chest_after, f.chest_v);
                    let moved = (c[0] - c0[0]).hypot(c[1] - c0[1]);
                    if c[0] > 5.0 && c[0] < 30.0 && c[1] > 64.0 && c[1] < 82.0 && v[1] < -5.0 && v[0] > 20.0 && v[0] < 31.0 && moved > 0.4 && moved < 3.0 {
                        let apex = c[1] - v[1] * v[1] / 20.0;
                        let q = 2000 - (apex.max(20.0) * 20.0) as i32;
                        if q > best {
                            best = q;
                            best_at = s.replay.iteration as usize;
                            best_chest = c;
                        }
                    }
                }
                if cfg.objective == "lq" && f.site == 0 && f.alive_after && life_ok(&s) {
                    // Launch quality: chest under the spike ceiling moving close to (27, -29) m/s.
                    let c = f.chest_after;
                    let v = f.chest_v;
                    if c[0] > 4.0 && c[0] < 22.0 && c[1] > 70.0 && c[1] < 82.0 && v[0].is_finite() {
                        let d = (v[0] - 27.0).hypot(v[1] + 29.0);
                        let q = 1000 - (d.min(100.0) * 10.0) as i32;
                        if q > best {
                            best = q;
                            best_at = s.replay.iteration as usize;
                            best_chest = c;
                        }
                    }
                }
                if !finish_obj && !pass_obj && cfg.objective != "lq" && cfg.objective != "apex" && counts && f.pairs > best && f.site == 0 && f.alive_after && f.chest_after[0].is_finite() {
                    best = f.pairs;
                    best_at = s.replay.iteration as usize;
                    best_chest = f.chest_after;
                }
                // Shaped fallback for the corridor launch: reward up-and-right chest speed at the
                // shaft top (below 100, so any counted pair frame dominates).
                if cfg.objective == "upc" && best < 100 && f.site == 0 && f.alive_after {
                    let c = f.chest_after;
                    if c[0] > 24.0 && c[0] < 36.0 && c[1] > 66.0 && c[1] < 76.0 {
                        let sec = (f.chest_v[0].clamp(0.0, 40.0) + (-f.chest_v[1]).clamp(0.0, 40.0)).round() as i32;
                        if sec > best {
                            best = sec;
                            best_at = s.replay.iteration as usize;
                            best_chest = c;
                        }
                    }
                }
                if !f.alive_after {
                    break;
                }
            }
            if best_at <= cfg.window + 2 {
                continue;
            }
            // Higher is better: pairs (+ open-sky bonus), or minus the finish distance.
            let score = if finish_obj {
                -best_near
            } else if pass_obj {
                best_pass + above_bonus / 100.0 + if passfin && passed_at != 0 { (1000.0 - 10.0 * near_after.min(100.0)) + (300.0 - passed_at as f64) * 0.0 } else { 0.0 }
            } else {
                best as f64 + above_bonus
            };
            if pass_obj && !best_pass.is_finite() {
                continue;
            }
            if finish_obj && !best_near.is_finite() {
                continue;
            }
            if passfin && passed_at != 0 && near_after < 12.0 {
                let full = (s.replay.iteration as usize).min(s.masks.len());
                writeln!(shared.events.lock().unwrap(), "{{\"kind\":\"near\",\"pass\":{},\"near\":{:.2},\"tas\":\"{}\"}}", passed_at, near_after, tas_text(&s.masks[..full])).ok();
            }
            let best = if finish_obj { (best_near * 1000.0) as i32 } else if pass_obj { (best_pass * 1000.0) as i32 } else { best };
            {
                let mut b = shared.best.lock().unwrap();
                if score > *b {
                    *b = score;
                    // passfin: keep every input the trial actually used (to frame 299), so the
                    // closest approach replays exactly.
                    let full = (s.replay.iteration as usize).min(s.masks.len());
                    std::fs::write(cfg.run_dir.join("best.tas"), tas_text(&s.masks[..if passfin { full } else { best_at }])).ok();
                }
            }
            if !shared.seen.lock().unwrap().insert((best, best_chest[0].to_bits(), best_chest[1].to_bits())) {
                continue;
            }
            let child = Member { masks: s.masks[..best_at].to_vec(), score, visits: 0 };
            let mut p = shared.pool.lock().unwrap();
            if p.len() < cfg.pool {
                p.push(child);
            } else {
                let (mi, ms) = p.iter().enumerate().map(|(i, m)| (i, m.score)).fold((0, f64::MAX), |a, b| if b.1 < a.1 { b } else { a });
                if score > ms {
                    p[mi] = child;
                }
            }
        }
    }
}

/// Play `s` (inputs in `s.masks`, then `tail`) to the first pairmax onset by `max_frame` and score
/// it like `osky3`: hold Space through the lock and take the best release point in open sky
/// against the flight-map finishing states. Returns (score, onset frame); no onset scores the
/// peak pair count seen (with Dad's life ok).
pub fn osky3_eval(s: &mut Sim, br: &mut Sim, template: &Sim, sky: &Skyline, max_frame: u32, tail: u8) -> (f64, u32) {
    let mut peak = 0i32;
    while s.frame() < max_frame {
        let f = s.step(tail);
        if f.port_abort || f.step_us > 80_000 {
            break;
        }
        if f.site == 2 && f.alive_before {
            let d = &s.ctx.character.primary;
            let deadline = if d.dying.get() { f.frame + (150 - d.bleed_counter).max(0) as u32 } else { 299 };
            br.restore(s);
            let i0 = br.replay.iteration as usize;
            br.masks.truncate(i0);
            let mut fd = 1000.0f64;
            let mut gap = f64::INFINITY;
            let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                for _ in 0..45 {
                    let g = br.step(16);
                    if g.port_abort || g.site == 0 || !g.chest_after[0].is_finite() {
                        break;
                    }
                    gap = gap.min(sky.gap(g.chest_after));
                    if sky.above(g.chest_after) {
                        fd = fd.min(crate::onarch::flight_distance_by(g.chest_after, g.chest_v, g.frame, deadline));
                    }
                }
            }));
            if r.is_err() {
                std::mem::forget(std::mem::replace(br, template.fork()));
            }
            let score = if fd < 1000.0 { 8000.0 - 20.0 * fd.min(150.0) } else { 5000.0 - 20.0 * gap.max(0.0).min(40.0) };
            return (score, f.frame);
        }
        if f.site != 0 || !f.alive_after {
            break;
        }
        if life_ok(s) && f.pairs > peak {
            peak = f.pairs;
        }
    }
    (peak as f64, 0)
}
