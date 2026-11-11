//! `hw-search perturb`: local hill-climb around near-miss launches.
//!
//! A pool holds input strings that end on a launch frame, scored by how far the chest ended from
//! the finish centre. Each trial picks a pool member (biased to the best), mutates a few inputs in
//! the frames just before its launch (the QE is chaotic, so small changes redirect the launch),
//! replays from a cached snapshot, and plays on for a few frames holding the last input. Any launch
//! (alive entering the frame, chest moved >= min_launch) closer than its parent joins the pool; a
//! launch that latches the finish is confirmed by an exact from-scratch replay and stops the run.
//! With `--finish-any`, a later actual finish latch is also confirmed without a same-frame launch.
use super::explore::{confirm, confirm_any_finish};
use super::sim::{tas_text, Frame, Sim};
use serde_json::json;
use std::io::Write;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

pub struct Config {
    pub level: u32,
    pub character: u32,
    /// Base strings; a positive anchor keeps each full setup instead of trimming to its peak.
    pub bases: Vec<Vec<u8>>,
    pub run_dir: PathBuf,
    pub threads: usize,
    pub hours: f64,
    pub seed: u64,
    pub min_launch: f64,
    pub max_frame: u32,
    /// Confirm any actual finish latch, even when a prior QE launched on an earlier frame.
    pub finish_any: bool,
    /// Mutations happen within the last `window` frames of a member.
    pub window: u32,
    /// Preserve this many original input entries before mutating a member's suffix.
    pub anchor: u32,
    /// QE credit requires this many consecutive eligible frames (1 keeps scalar scoring).
    pub qe_persist: u32,
    /// Frames played past the end of a member (holding its last input, then random).
    pub extra: u32,
    pub target: [f64; 2],
    /// Optional chest bounds [x0,x1,y0,y1]: states whose chest leaves them are rejected.
    pub bounds: Option<[f64; 4]>,
    pub pool_size: usize,
    /// QE mode: members without a launch are scored by in-bounds QE, optionally persistent.
    pub qe_mode: bool,
    /// Require the launch to hit the top border, then the right wall, first (Satan): other routes are
    /// scored `WRONG_ROUTE` worse than any right-route launch.
    pub route: bool,
    /// Finish-check mode ("mechanism C"): score = distance from the finish region of the chest
    /// position read by any FinishLine check (e.g. a flying limb touching the finish box).
    pub fc_mode: bool,
}

const WRONG_ROUTE: f64 = 200.0;

/// Finish-check mode counts a check only in a frame where the chest itself moved this far
/// (a launch or teleport); otherwise the climb just walks the chest towards the finish.
const FC_MIN_MOVE: f64 = 20.0;

/// Launch score: miss distance, plus a penalty when `--route` is set and the chest's TOI chain
/// does not start top border -> right wall.
fn launch_score(cfg: &Config, template: &Sim, masks: &[u8], miss: f64) -> (f64, String) {
    if !cfg.route {
        return (miss, String::new());
    }
    let r = super::analyze::launch_route(template, masks);
    let ok = r.clean();
    (if ok { miss } else { miss + WRONG_ROUTE }, r.label())
}

#[derive(Clone)]
struct Member {
    masks: Vec<u8>,
    miss: f64,
    trials: u64,
    id: u64,
}

/// Systematic sweep of the launch-relevant bits (up, down, space, shift, ctrl) on a member's last
/// two frames: 32 x 32 combinations per member, best members first.
struct Sweep {
    queue: std::collections::VecDeque<(u64, u16)>,
    swept: std::collections::HashSet<u64>,
}

const SWEEP_BITS: [u8; 5] = [4, 8, 16, 32, 64];

fn sweep_masks(masks: &mut [u8], combo: u16) -> usize {
    let n = masks.len();
    for j in 0..2usize {
        if n < j + 1 {
            break;
        }
        let pos = n - 1 - j;
        let c = (combo >> (5 * j)) & 31;
        let mut m = masks[pos] & (1 | 2 | 128);
        for (b, &bit) in SWEEP_BITS.iter().enumerate() {
            if c & (1 << b) != 0 {
                m |= bit;
            }
        }
        masks[pos] = m;
    }
    n.saturating_sub(2)
}

fn sweep_masks_anchored(masks: &mut [u8], combo: u16, anchor: u32) -> usize {
    if anchor == 0 {
        return sweep_masks(masks, combo);
    }
    let start = (anchor as usize).min(masks.len());
    if masks.len() - start < 2 {
        return masks.len();
    }
    start + sweep_masks(&mut masks[start..], combo)
}

fn next_sweep_job(shared: &Shared, anchor: u32) -> Option<Member> {
    let mut sw = shared.sweep.lock().unwrap();
    loop {
        if let Some((id, combo)) = sw.queue.pop_front() {
            let p = shared.pool.lock().unwrap();
            if let Some(m) = p.iter().find(|m| m.id == id) {
                let mut child = m.clone();
                sweep_masks_anchored(&mut child.masks, combo, anchor);
                // carry the parent's score for acceptance; the id stays the parent's (cache)
                return Some(child);
            }
            continue;
        }
        // enqueue the best not-yet-swept member
        let p = shared.pool.lock().unwrap();
        let mut best: Option<&Member> = None;
        for m in p.iter() {
            if !sw.swept.contains(&m.id) && m.masks.len().saturating_sub(anchor as usize) >= 2 && best.map_or(true, |b| m.miss < b.miss) {
                best = Some(m);
            }
        }
        let id = best?.id;
        drop(p);
        sw.swept.insert(id);
        for combo in 0..1024u16 {
            sw.queue.push_back((id, combo));
        }
    }
}

struct Shared {
    pool: Mutex<Vec<Member>>,
    next_id: AtomicU64,
    stop: AtomicBool,
    frames: AtomicU64,
    trials: AtomicU64,
    launches: AtomicU64,
    improvements: AtomicU64,
    best: Mutex<f64>,
    /// Aggregate raw, current connected, and persistently scored QE at the best admission.
    best_qe: Mutex<Option<(f64, f64, f64)>>,
    log: Mutex<std::fs::File>,
    /// Every launch (endpoints; TAS when within 80 m of the finish centre).
    launch_log: Mutex<std::fs::File>,
    winner: Mutex<Option<Vec<u8>>>,
    /// Launch outcomes (chest end position bits) already seen: repeats are skipped.
    seen: Mutex<std::collections::HashSet<(u64, u64)>>,
    duplicates: AtomicU64,
    lag_kills: AtomicU64,
    sweep: Mutex<Sweep>,
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
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
}

fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

/// Lower is better. Launch tier: miss distance (m). QE tier: 1000 + 10*(20 - log10(peak QE)).
/// QE credit stops at 10^10 m/s and turns into a penalty above 10^12: runaway QEs lag the game
/// and never launch cleanly (every useful launch so far came from 10^4..10^10).
fn qe_score(qe: f64) -> f64 {
    let l = qe.max(1.0).log10();
    1000.0 + 10.0 * (20.0 - l.min(10.0)) + if l > 12.0 { 10.0 * (l - 12.0) } else { 0.0 }
}

/// A full window is required; invalid frames break the consecutive run.
struct QePersistence {
    required: usize,
    values: [f64; 64],
    count: usize,
    next: usize,
}

impl QePersistence {
    fn new(required: u32) -> Self {
        Self { required: required as usize, values: [0.0; 64], count: 0, next: 0 }
    }

    // A repeated input slot is not a new sample, even if that frame mutated physics.
    // Original world-stage throws may advance input and remain eligible: this adds
    // no site==0 or unlocked-state policy to the existing caller's fitness guards.
    fn observe_step(&mut self, qe: f64, eligible: bool, before: u32, after: u32) -> Option<f64> {
        self.observe(qe, eligible && input_advanced(before, after))
    }

    fn observe(&mut self, qe: f64, eligible: bool) -> Option<f64> {
        if !eligible || !qe.is_finite() {
            self.count = 0;
            self.next = 0;
            return None;
        }
        // Keep the default scalar path exact, including its floating-point value.
        if self.required == 1 {
            return Some(qe);
        }
        self.values[self.next] = qe;
        self.next = (self.next + 1) % self.required;
        self.count = (self.count + 1).min(self.required);
        (self.count == self.required)
            .then(|| self.values[..self.required].iter().copied().fold(f64::INFINITY, f64::min))
    }
}

// A host search budget, not a statement that a stalled native replay cannot recover.
// Every original throw and its retained mutations stay in Sim; no reset/unlock occurs.
const MAX_STALLED_INPUT_FRAMES: u32 = 32;

fn input_advanced(before: u32, after: u32) -> bool {
    before.checked_add(1) == Some(after)
}

fn input_stalled_out(stalled: &mut u32, before: u32, after: u32) -> bool {
    *stalled = if input_advanced(before, after) { 0 } else { *stalled + 1 };
    *stalled >= MAX_STALLED_INPUT_FRAMES
}

fn base_keep(anchor: u32, scored_at: usize, original_len: usize) -> usize {
    if anchor == 0 { scored_at } else { original_len }
}

fn has_mutable_suffix(anchor: u32, len: usize) -> bool {
    anchor == 0 || len.saturating_sub(anchor as usize) >= 2
}

fn in_world(p: [f64; 2]) -> bool {
    p[0] > -4.0 && p[0] < 324.0 && p[1] > -84.0 && p[1] < 164.0
}

fn in_bounds(cfg: &Config, p: [f64; 2]) -> bool {
    match cfg.bounds {
        Some(b) => p[0] >= b[0] && p[0] <= b[1] && p[1] >= b[2] && p[1] <= b[3],
        None => true,
    }
}

/// Only the opt-in Dad finish search rejects a launch whose primary chest is no longer usable.
/// Actual finish latches are checked before this admission policy in `trial`.
fn launch_admissible(finish_any: bool, character: u32, f: &Frame, after_in_bounds: bool) -> bool {
    !finish_any || character != 3 || (f.alive_after && f.chest_alive && after_in_bounds)
}

fn trial_has_budget(finish_any: bool, iteration: u32, end: usize, frame: u32, max_frame: u32) -> bool {
    frame < max_frame && (finish_any || (iteration as usize) < end)
}

fn remember_best_launch(best: &mut Option<(f64, Vec<u8>, Frame)>, parent_score: f64,
    score: f64, masks: &[u8], f: &Frame)
{
    if score < parent_score && best.as_ref().map_or(true, |b| score < b.0) {
        *best = Some((score, masks.to_vec(), *f));
    }
}

/// Replay `masks` from scratch; returns (score, length to keep). Launch mode: the last launch's
/// miss. QE mode: the first in-bounds launch, else the best eligible QE window. A positive
/// anchor scores only after the immutable setup and retains the complete original base.
fn evaluate_base(cfg: &Config, template: &Sim, masks: &[u8]) -> Option<(f64, usize)> {
    if !has_mutable_suffix(cfg.anchor, masks.len()) {
        return None;
    }
    let mut s = template.fork();
    s.masks = masks.to_vec();
    if cfg.fc_mode {
        // Best finish check over the whole string (999 when the finish box never reports).
        let mut best = (999.0, masks.len());
        let mut eligible_after_anchor = cfg.anchor == 0;
        let mut stalled = 0;
        while (s.replay.iteration as usize) < masks.len() && s.frame() < cfg.max_frame {
            let before = s.replay.iteration;
            let f = s.step(0);
            if f.port_abort {
                break;
            }
            let n = s.replay.iteration as usize;
            if n > cfg.anchor as usize && f.alive_before && f.displacement >= FC_MIN_MOVE
                && launch_admissible(cfg.finish_any, cfg.character, &f, in_world(f.chest_after) && in_bounds(cfg, f.chest_after))
            {
                eligible_after_anchor = true;
                if f.fc < best.0 {
                    best = (f.fc, base_keep(cfg.anchor, n, masks.len()));
                }
            }
            if !f.alive_after || input_stalled_out(&mut stalled, before, s.replay.iteration) {
                break;
            }
        }
        return eligible_after_anchor.then_some(best);
    }
    let mut best: Option<(f64, usize)> = None;
    let mut qe_history = QePersistence::new(cfg.qe_persist);
    let mut stalled = 0;
    while (s.replay.iteration as usize) < masks.len() && s.frame() < cfg.max_frame {
        let before = s.replay.iteration;
        let f = s.step(0);
        if f.port_abort {
            qe_history.observe(f.qe, false);
            break;
        }
        let n = s.replay.iteration as usize;
        if n > cfg.anchor as usize && f.alive_before && f.displacement >= cfg.min_launch && in_bounds(cfg, f.chest_before)
            && launch_admissible(cfg.finish_any, cfg.character, &f, in_world(f.chest_after) && in_bounds(cfg, f.chest_after))
        {
            let m = launch_score(cfg, template, &masks[..n], distance(cfg.target, f.chest_after)).0;
            if cfg.qe_mode {
                return Some((m, base_keep(cfg.anchor, n, masks.len())));
            }
            best = Some((m, base_keep(cfg.anchor, n, masks.len())));
        } else if cfg.qe_mode {
            if let Some(qe) = qe_history.observe_step(f.qe, n > cfg.anchor as usize && f.alive_after && in_bounds(cfg, f.chest_after) && f.frame < cfg.max_frame, before, s.replay.iteration) {
                let sc = qe_score(qe);
                if best.map_or(true, |b| sc < b.0) {
                    best = Some((sc, base_keep(cfg.anchor, n, masks.len())));
                }
            }
        }
        if (cfg.qe_mode && (!f.alive_after || !in_bounds(cfg, f.chest_after)))
            || input_stalled_out(&mut stalled, before, s.replay.iteration)
        {
            break;
        }
    }
    best
}

pub fn run(cfg: Config) -> Result<Vec<u8>, String> {
    if !(1..=64).contains(&cfg.qe_persist) {
        return Err("--qe-persist must be in 1..=64".into());
    }
    if cfg.anchor > 0 && cfg.bases.iter().any(|b| cfg.anchor as usize >= b.len()) {
        return Err("--anchor must be smaller than every base's input length".into());
    }
    std::fs::create_dir_all(&cfg.run_dir).map_err(|e| e.to_string())?;
    let template = Sim::new(cfg.level, cfg.character)?;
    let log = std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("improvements.jsonl")).map_err(|e| e.to_string())?;
    let mut pool = Vec::new();
    for (i, b) in cfg.bases.iter().enumerate() {
        match evaluate_base(&cfg, &template, b) {
            Some((miss, keep)) => {
                eprintln!("base {i}: {} frames, score {miss:.2} (kept {keep} frames)", b.len());
                pool.push(Member { masks: b[..keep].to_vec(), miss, trials: 0, id: i as u64 });
            }
            None => eprintln!("base {i}: no launch on its last frames, skipped"),
        }
    }
    if pool.is_empty() {
        return Err("no base launches".into());
    }
    let best0 = pool.iter().map(|m| m.miss).fold(f64::INFINITY, f64::min);
    let shared = Shared {
        next_id: AtomicU64::new(pool.len() as u64),
        pool: Mutex::new(pool),
        stop: AtomicBool::new(false),
        frames: AtomicU64::new(0),
        trials: AtomicU64::new(0),
        launches: AtomicU64::new(0),
        improvements: AtomicU64::new(0),
        best: Mutex::new(best0),
        best_qe: Mutex::new(None),
        log: Mutex::new(log),
        launch_log: Mutex::new(std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("launches.jsonl")).map_err(|e| e.to_string())?),
        winner: Mutex::new(None),
        seen: Mutex::new(std::collections::HashSet::new()),
        duplicates: AtomicU64::new(0),
        lag_kills: AtomicU64::new(0),
        sweep: Mutex::new(Sweep { queue: Default::default(), swept: Default::default() }),
    };
    let start = Instant::now();
    let deadline = start + Duration::from_secs_f64(cfg.hours * 3600.0);
    std::thread::scope(|scope| {
        for w in 0..cfg.threads {
            let (shared, cfg, template) = (&shared, &cfg, &template);
            std::thread::Builder::new().stack_size(16 << 20).spawn_scoped(scope, move || worker(w as u64, cfg, template, shared)).unwrap();
        }
        let mut stats = std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("stats.jsonl")).unwrap();
        while !shared.stop.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_secs(10));
            if Instant::now() > deadline || cfg.run_dir.join("STOP").exists() {
                shared.stop.store(true, Ordering::Relaxed);
            }
            let (n, top) = {
                let p = shared.pool.lock().unwrap();
                let mut v: Vec<f64> = p.iter().map(|m| m.miss).collect();
                v.sort_by(|a, b| a.partial_cmp(b).unwrap());
                (p.len(), v.into_iter().take(5).collect::<Vec<_>>())
            };
            let (sw_n, sw_q) = {
                let sw = shared.sweep.lock().unwrap();
                (sw.swept.len(), sw.queue.len())
            };
            let best_qe = *shared.best_qe.lock().unwrap();
            let line = json!({"t": start.elapsed().as_secs(), "frames": shared.frames.load(Ordering::Relaxed),
                "fps": shared.frames.load(Ordering::Relaxed) as f64 / start.elapsed().as_secs_f64(),
                "trials": shared.trials.load(Ordering::Relaxed), "launches": shared.launches.load(Ordering::Relaxed), "duplicates": shared.duplicates.load(Ordering::Relaxed), "lag_kills": shared.lag_kills.load(Ordering::Relaxed), "swept_members": sw_n, "sweep_queue": sw_q,
                "improvements": shared.improvements.load(Ordering::Relaxed), "pool": n, "best_miss_m": *shared.best.lock().unwrap(), "top5": top,
                "anchor": cfg.anchor, "qe_persist": cfg.qe_persist,
                "best_qe_raw": best_qe.map(|q| q.0), "best_qe_current": best_qe.map(|q| q.1), "best_qe_scored": best_qe.map(|q| q.2),
                "winner": shared.winner.lock().unwrap().is_some()});
            writeln!(stats, "{line}").ok();
            eprintln!("{line}");
        }
    });
    let p = shared.pool.lock().unwrap();
    if let Ok(mut out) = std::fs::File::create(cfg.run_dir.join("pool.jsonl")) {
        for m in p.iter() {
            writeln!(out, "{}", json!({"miss_m": m.miss, "frame": m.masks.len(), "trials": m.trials, "tas": tas_text(&m.masks)})).ok();
        }
    }
    let w = shared.winner.lock().unwrap();
    w.clone().ok_or_else(|| "perturb ended without a confirmed winner".into())
}

fn pick(shared: &Shared, rng: &mut Rng) -> Member {
    let mut p = shared.pool.lock().unwrap();
    // Rank-based: sort by miss, pick index ~ exponential toward the best, favour less-tried.
    p.sort_by(|a, b| a.miss.partial_cmp(&b.miss).unwrap());
    let n = p.len();
    // Launch-tier members (score < 1000) sort first. Split picks between the launch tier and the
    // QE tier so a single launch lineage cannot starve QE growth (and vice versa).
    let launch_n = p.iter().filter(|m| m.miss < 1000.0).count();
    let expo = |rng: &mut Rng, len: usize| -> usize {
        let r = rng.unit();
        (((-(1.0 - r).ln()) * (len as f64 / 6.0).max(1.5)).floor() as usize).min(len.max(1) - 1)
    };
    let roll = rng.below(10);
    let i = if roll < 3 || launch_n == 0 || launch_n == n {
        if roll < 3 { rng.below(n as u64) as usize } else { expo(rng, n) }
    } else if roll < 6 {
        expo(rng, launch_n)
    } else {
        launch_n + expo(rng, n - launch_n)
    };
    let i = i.min(n - 1);
    p[i].trials += 1;
    p[i].clone()
}

/// Mutate the inputs of `masks` within its last `window` frames; returns the earliest changed index.
fn mutate(rng: &mut Rng, masks: &mut Vec<u8>, window: u32) -> usize {
    // In the QE regime (10^6+ m/s) pose motors are numerically invisible; grab/release (space,
    // bit 16) changes joints and is what actually redirects a launch. Depth is spread over the
    // whole window so the QE growth path itself varies.
    const GRAB: u8 = 16;
    let n = masks.len();
    let w = (window as usize).min(n - 1).max(1);
    // Retrigger (60%): end the string at an earlier frame t of the window and put a random
    // combination of the launch-relevant bits on its last two frames; the QE state differs at
    // every t, so this samples genuinely different launch directions.
    if rng.below(10) < 6 && n > 3 {
        let t = n - rng.below(w as u64) as usize;
        let t = t.max(3);
        masks.truncate(t);
        for j in 0..2usize {
            let pos = t - 1 - j;
            let mut m = masks[pos] & (1 | 2 | 128);
            for &bit in SWEEP_BITS.iter() {
                if rng.below(2) == 0 {
                    m |= bit;
                }
            }
            masks[pos] = m;
        }
        return t - 2;
    }
    let depth = if rng.below(2) == 0 { 1 + rng.below(10.min(w as u64)) as usize } else { 1 + rng.below(w as u64) as usize };
    let pos = n - depth;
    match rng.below(10) {
        // toggle grab on one frame
        0 | 1 => {
            masks[pos] ^= GRAB;
        }
        // toggle grab over a short run
        2 => {
            let len = 1 + rng.below(8) as usize;
            for k in pos..(pos + len).min(n) {
                masks[k] ^= GRAB;
            }
        }
        // shift the nearest grab transition by 1-3 frames
        3 => {
            let mut t = pos;
            while t + 1 < n && (masks[t] ^ masks[t + 1]) & GRAB == 0 {
                t += 1;
            }
            if t + 1 < n {
                let shift = 1 + rng.below(3) as usize;
                if rng.below(2) == 0 {
                    // extend the later state earlier
                    let v = masks[t + 1] & GRAB;
                    for k in t.saturating_sub(shift - 1)..=t {
                        masks[k] = (masks[k] & !GRAB) | v;
                    }
                    return t.saturating_sub(shift - 1);
                } else {
                    let v = masks[t] & GRAB;
                    for k in (t + 1)..(t + 1 + shift).min(n) {
                        masks[k] = (masks[k] & !GRAB) | v;
                    }
                    return t + 1;
                }
            } else {
                masks[pos] ^= GRAB;
            }
        }
        // replace a 5-40 frame span with one held pose (+grab usually)
        4 | 5 => {
            const POSES: [u8; 7] = [0, 1, 2, 4, 8, 32, 64];
            let len = 5 + rng.below(36) as usize;
            let m = POSES[rng.below(7) as usize] | if rng.below(100) < 85 { GRAB } else { 0 };
            for k in pos..(pos + len).min(n) {
                masks[k] = m;
            }
        }
        // flip one random bit of one frame
        6 => {
            masks[pos] ^= 1 << rng.below(8);
        }
        // replace one frame with a random mask
        7 => {
            masks[pos] = rng.below(256) as u8;
        }
        // flip one bit over a short run
        8 => {
            let bit = 1 << rng.below(8);
            let len = 1 + rng.below(4) as usize;
            for k in pos..(pos + len).min(n) {
                masks[k] ^= bit;
            }
        }
        // two independent grab toggles
        _ => {
            masks[pos] ^= GRAB;
            let p2 = n - 1 - rng.below(w as u64) as usize;
            masks[p2] ^= GRAB;
            return pos.min(p2);
        }
    }
    pos
}

/// Run the existing mutation on a suffix so retriggers and shifted grab edges cannot touch
/// the immutable setup. Anchor zero uses the exact original RNG path.
fn mutate_anchored(rng: &mut Rng, masks: &mut Vec<u8>, window: u32, anchor: u32) -> usize {
    if anchor == 0 {
        return mutate(rng, masks, window);
    }
    let start = (anchor as usize).min(masks.len());
    if masks.len() - start < 2 {
        return masks.len();
    }
    let mut suffix = masks[start..].to_vec();
    let changed = mutate(rng, &mut suffix, window);
    masks.truncate(start);
    masks.extend_from_slice(&suffix);
    start + changed
}

// The small cursor interface lets tests model an original input-phase throw:
// frames advance and state mutates, but the same input slot is read again.
trait PrefixCursor {
    fn cursor(&self) -> (u32, u32); // (actual consumed inputs, actual frames)
    fn prefix_step(&mut self) -> bool; // only the existing port-only rejection
}

impl PrefixCursor for Sim {
    fn cursor(&self) -> (u32, u32) { (self.replay.iteration, self.frame()) }
    fn prefix_step(&mut self) -> bool { self.step(0).port_abort }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PrefixExit { Reached, PortAbort, FrameLimit, NoInputProgress, InvalidCursor }

struct PrefixReplay { exit: PrefixExit, steps: u64 }

fn replay_prefix<S: PrefixCursor>(sim: &mut S, target: usize, max_frame: u32) -> PrefixReplay {
    let (mut steps, mut stalled) = (0, 0);
    let exit = loop {
        let (before, frame) = sim.cursor();
        if before as usize == target { break PrefixExit::Reached; }
        if before as usize > target { break PrefixExit::InvalidCursor; }
        if frame >= max_frame { break PrefixExit::FrameLimit; }
        let port_abort = sim.prefix_step();
        steps += 1;
        let (after, new_frame) = sim.cursor();
        if port_abort { break PrefixExit::PortAbort; }
        if frame.checked_add(1) != Some(new_frame)
            || !(after == before || input_advanced(before, after))
        {
            break PrefixExit::InvalidCursor;
        }
        if input_stalled_out(&mut stalled, before, after) { break PrefixExit::NoInputProgress; }
    };
    PrefixReplay { exit, steps }
}

struct Cached {
    id: u64,
    /// Snapshot after replaying the member's first `at` inputs.
    at: usize,
    sim: Sim,
}

fn worker(w: u64, cfg: &Config, template: &Sim, shared: &Shared) {
    let mut rng = Rng(cfg.seed ^ (w + 1).wrapping_mul(0x5851_f42d_4c95_7f2d));
    let mut cache: Vec<Cached> = Vec::new();
    let mut work = template.fork();
    while !shared.stop.load(Ordering::Relaxed) {
        // 80% systematic sweep jobs, 20% random mutation.
        let job = if false { next_sweep_job(shared, cfg.anchor) } else { None };
        let (parent, masks, changed) = match job {
            Some(child) => {
                let parent = {
                    let p = shared.pool.lock().unwrap();
                    match p.iter().find(|m| m.id == child.id) {
                        Some(m) => m.clone(),
                        None => continue,
                    }
                };
                let changed = parent.masks.len().saturating_sub(2).max(cfg.anchor as usize);
                (parent, child.masks, changed)
            }
            None => {
                let parent = pick(shared, &mut rng);
                let mut masks = parent.masks.clone();
                let changed = mutate_anchored(&mut rng, &mut masks, cfg.window, cfg.anchor);
                (parent, masks, changed)
            }
        };
        if masks == parent.masks {
            continue;
        }
        // Snapshot the parent at the start of its window (inputs before it are unchanged).
        // Replay the preceding N-1 consumed inputs when persistence is enabled. A cached
        // snapshot alone cannot establish a consecutive QE window.
        let replay_to = if cfg.qe_mode { changed.saturating_sub(cfg.qe_persist as usize - 1) } else { changed };
        let base_at = parent.masks.len().saturating_sub(cfg.window as usize);
        let start_at = replay_to.min(base_at);
        let cached = cache.iter().position(|c| c.id == parent.id && c.at <= replay_to
            && c.sim.replay.iteration as usize == c.at);
        match cached {
            Some(i) => {
                work.restore(&cache[i].sim);
                work.masks.clear();
                work.masks.extend_from_slice(&masks);
                // A step is not necessarily a consumed input. Stop only at the exact offset.
                let replay = replay_prefix(&mut work, replay_to, cfg.max_frame);
                shared.frames.fetch_add(replay.steps, Ordering::Relaxed);
                if replay.exit != PrefixExit::Reached { continue; }
            }
            None => {
                work.restore(template);
                work.masks.clear();
                work.masks.extend_from_slice(&masks);
                let replay = replay_prefix(&mut work, start_at, cfg.max_frame);
                shared.frames.fetch_add(replay.steps, Ordering::Relaxed);
                if replay.exit != PrefixExit::Reached { continue; }
                // Cache only exact consumed offsets on the parent's unmutated prefix.
                debug_assert_eq!(work.replay.iteration as usize, start_at);
                debug_assert_eq!(&masks[..start_at], &parent.masks[..start_at]);
                if start_at <= replay_to {
                    if cache.len() >= 6 {
                        let mut old = cache.remove(0);
                        old.sim.restore(&work);
                        old.id = parent.id;
                        old.at = start_at;
                        cache.push(old);
                    } else {
                        cache.push(Cached { id: parent.id, at: start_at, sim: work.fork() });
                    }
                }
                let replay = replay_prefix(&mut work, replay_to, cfg.max_frame);
                shared.frames.fetch_add(replay.steps, Ordering::Relaxed);
                if replay.exit != PrefixExit::Reached { continue; }
            }
        }
        shared.trials.fetch_add(1, Ordering::Relaxed);
        trial(cfg, shared, &mut rng, &mut work, &parent, template);
    }
}

/// Legacy trials play the mutated tail plus `extra` inputs and stop at a launch.
/// Finish-any trials continue through launches to the frame ceiling or an existing branch
/// stop, retaining the best useful launch and any eligible QE/finish-check improvement.
fn trial(cfg: &Config, shared: &Shared, rng: &mut Rng, s: &mut Sim, parent: &Member, template: &Sim) {
    let end = parent.masks.len() + cfg.extra as usize;
    let mut last = *s.masks.last().unwrap_or(&0);
    // Mask-change probability per frame past the string (QE mode holds much longer).
    let change = if cfg.qe_mode { 30 } else { 4 };
    let mut best_qe: Option<(f64, Vec<u8>, Frame, f64)> = None;
    let mut best_fc: Option<(f64, Vec<u8>, Frame)> = None;
    let mut best_launch: Option<(f64, Vec<u8>, Frame)> = None;
    let mut trial_launches = std::collections::HashSet::new();
    let mut qe_history = QePersistence::new(cfg.qe_persist);
    let mut lag_run = 0u32;
    let mut stalled = 0;
    while trial_has_budget(cfg.finish_any, s.replay.iteration, end, s.frame(), cfg.max_frame) {
        let mask = if rng.below(change) == 0 { rng.below(256) as u8 } else { last };
        let before = s.replay.iteration;
        let f: Frame = s.step(mask);
        last = f.mask;
        shared.frames.fetch_add(1, Ordering::Relaxed);
        if f.port_abort {
            qe_history.observe(f.qe, false);
            break;
        }
        if cfg.finish_any && f.finished {
            let masks = s.masks[..s.replay.iteration as usize].to_vec();
            record_finish(cfg, shared, masks, &f, distance(cfg.target, f.chest_after));
            return;
        }
        let after_anchor = s.replay.iteration > cfg.anchor;
        if after_anchor && cfg.fc_mode && f.alive_before && f.displacement >= FC_MIN_MOVE
            && launch_admissible(cfg.finish_any, cfg.character, &f, in_world(f.chest_after) && in_bounds(cfg, f.chest_after))
            && f.fc < parent.miss && best_fc.as_ref().map_or(true, |b| f.fc < b.0)
        {
            best_fc = Some((f.fc, s.masks[..s.replay.iteration as usize].to_vec(), f));
        }
        if !in_world(f.chest_before) || !in_bounds(cfg, f.chest_before) {
            qe_history.observe(f.qe, false);
            break;
        }
        if after_anchor && f.alive_before && f.displacement >= cfg.min_launch {
            let endpoint = (f.chest_after[0].to_bits(), f.chest_after[1].to_bits());
            // Continuing a trial must not repeatedly log/check the same launch endpoint.
            // Duplicates still pass through the death/bounds/lag/stall guards below.
            if !cfg.finish_any || trial_launches.insert(endpoint) {
                shared.launches.fetch_add(1, Ordering::Relaxed);
                let miss = distance(cfg.target, f.chest_after);
                let masks = s.masks[..s.replay.iteration as usize].to_vec();
                let duplicate = !shared.seen.lock().unwrap().insert(endpoint);
                if duplicate {
                    shared.duplicates.fetch_add(1, Ordering::Relaxed);
                    if !cfg.finish_any { return; }
                }
                let (score, route) = launch_score(cfg, template, &masks, miss);
                let useful = launch_admissible(cfg.finish_any, cfg.character, &f,
                    in_world(f.chest_after) && in_bounds(cfg, f.chest_after));
                {
                    let tas = if miss < 80.0 { tas_text(&masks) } else { String::new() };
                    writeln!(shared.launch_log.lock().unwrap(), "{}", json!({"frame": f.frame, "from": f.chest_before, "to": f.chest_after, "moved": f.displacement, "miss_m": miss, "score": score, "route": route, "qe": f.qe, "qe_raw": f.qe_raw, "finished": f.finished, "alive_after": f.alive_after, "chest_alive": f.chest_alive, "eligible_for_launch_pool": useful, "duplicate": duplicate, "continued_for_finish": cfg.finish_any, "tas": tas})).ok();
                }
                if cfg.finish_any {
                    if !cfg.fc_mode && useful {
                        remember_best_launch(&mut best_launch, parent.miss, score, &masks, &f);
                    }
                } else {
                    // Preserve the legacy first-launch confirmation/admission/return path.
                    if f.finished {
                        if record_finish(cfg, shared, masks.clone(), &f, miss) { return; }
                    }
                    if cfg.fc_mode {
                        if let Some((sc, m, bf)) = best_fc {
                            accept(cfg, shared, m, sc, &bf, None);
                        }
                    } else if score < parent.miss {
                        accept(cfg, shared, masks, score, &f, None);
                    }
                    return;
                }
            }
        }
        if !f.alive_after || !in_world(f.chest_after) || !in_bounds(cfg, f.chest_after) || s.replay.finish_frame != 0 {
            qe_history.observe(f.qe, false);
            break;
        }
        // A runaway QE that keeps lagging the game never leads to a clean launch: drop the
        // branch after `lag_frames()` laggy frames in a row.
        lag_run = if f.lagging { lag_run + 1 } else { 0 };
        if lag_run >= super::sim::lag_frames() {
            shared.lag_kills.fetch_add(1, Ordering::Relaxed);
            break;
        }
        if cfg.qe_mode {
            if let Some(qe) = qe_history.observe_step(f.qe, after_anchor, before, s.replay.iteration) {
                let sc = qe_score(qe);
                if sc < parent.miss && best_qe.as_ref().map_or(true, |b| sc < b.0) {
                    best_qe = Some((sc, s.masks[..s.replay.iteration as usize].to_vec(), f, qe));
                }
            }
        }
        if input_stalled_out(&mut stalled, before, s.replay.iteration) { break; }
    }
    if let Some((sc, masks, f, qe)) = best_qe {
        accept(cfg, shared, masks, sc, &f, Some(qe));
    }
    if let Some((sc, masks, f)) = best_fc {
        accept(cfg, shared, masks, sc, &f, None);
    }
    if let Some((sc, masks, f)) = best_launch {
        accept(cfg, shared, masks, sc, &f, None);
    }
}

fn record_finish(cfg: &Config, shared: &Shared, masks: Vec<u8>, f: &Frame, miss: f64) -> bool {
    let result = if cfg.finish_any {
        confirm_any_finish(cfg.level, cfg.character, &masks)
    } else {
        confirm(cfg.level, cfg.character, &masks, cfg.min_launch)
    };
    match result {
        Ok((frame, moved)) if frame <= cfg.max_frame => {
            let mut winner = shared.winner.lock().unwrap();
            if winner.is_some() { return true; }
            let name = cfg.run_dir.join("winner-1.tas");
            if let Err(e) = std::fs::write(&name, tas_text(&masks)) {
                eprintln!("confirmed candidate at frame {frame} could not be saved: {e}");
                return false;
            }
            eprintln!("WINNER: finish frame {frame}, chest moved {moved:.1} m -> {}", name.display());
            writeln!(shared.log.lock().unwrap(), "{}", json!({"winner": true, "frame": frame, "from": f.chest_before, "to": f.chest_after, "miss_m": miss, "tas": tas_text(&masks)})).ok();
            *winner = Some(masks);
            shared.stop.store(true, Ordering::Relaxed);
            true
        }
        Ok((frame, _)) => {
            eprintln!("candidate at frame {} confirmed after the frame limit at {frame}", f.frame);
            false
        }
        Err(e) => {
            eprintln!("candidate at frame {} failed confirmation: {e}", f.frame);
            false
        }
    }
}

fn accept(cfg: &Config, shared: &Shared, masks: Vec<u8>, score: f64, f: &Frame, scored_qe: Option<f64>) {
    // A single unprotected input cannot be mutated by the two-entry retrigger.
    if !has_mutable_suffix(cfg.anchor, masks.len()) { return; }
    shared.improvements.fetch_add(1, Ordering::Relaxed);
    let id = shared.next_id.fetch_add(1, Ordering::Relaxed);
    {
        let mut b = shared.best.lock().unwrap();
        if score < *b {
            *b = score;
            *shared.best_qe.lock().unwrap() = scored_qe.map(|qe| (f.qe_raw, f.qe, qe));
            writeln!(shared.log.lock().unwrap(), "{}", json!({"best": true, "score": score, "frame": f.frame, "from": f.chest_before, "to": f.chest_after, "moved": f.displacement,
                "qe": f.qe, "qe_raw": f.qe_raw, "qe_current": f.qe, "qe_scored": scored_qe, "qe_persist": cfg.qe_persist, "anchor": cfg.anchor, "tas": tas_text(&masks)})).ok();
            let tag = if score < 1000.0 {
                format!("miss{score:.1}m")
            } else if cfg.qe_persist > 1 && scored_qe.is_some() {
                format!("qep{}-{:.1e}", cfg.qe_persist, scored_qe.unwrap())
            } else {
                format!("qe{:.1e}", f.qe)
            };
            std::fs::write(cfg.run_dir.join(format!("best-{tag}-f{}.tas", f.frame)), tas_text(&masks)).ok();
        }
    }
    let mut p = shared.pool.lock().unwrap();
    if p.iter().any(|m| m.miss == score) {
        return;
    }
    p.push(Member { masks, miss: score, trials: 0, id });
    if p.len() > cfg.pool_size {
        p.sort_by(|a, b| a.miss.partial_cmp(&b.miss).unwrap());
        p.truncate(cfg.pool_size);
    }
}

#[cfg(test)]
mod tests {
    use super::{base_keep, launch_admissible, remember_best_launch, trial_has_budget,
        mutate, mutate_anchored, qe_score, replay_prefix, sweep_masks,
        sweep_masks_anchored, PrefixCursor, PrefixExit, QePersistence, Rng,
        MAX_STALLED_INPUT_FRAMES};

    #[test]
    fn anchored_pool_requires_two_mutable_inputs_without_changing_legacy_short_inputs() {
        for len in 0..=61 { assert!(!super::has_mutable_suffix(60, len)); }
        assert!(super::has_mutable_suffix(60, 62));
        assert!(super::has_mutable_suffix(60, 299));
        for len in 0..=2 { assert!(super::has_mutable_suffix(0, len)); }
    }


    #[test]
    fn finish_any_trial_can_reach_a_later_finish_beyond_the_parent_tail() {
        // A launch at12 must not end a finish-any trial whose later latch could be at20.
        let end = 12;
        assert!(!trial_has_budget(false, 12, end, 12, 299));
        assert!(trial_has_budget(true, 12, end, 12, 299));
        assert!(trial_has_budget(true, 20, end, 20, 299));
        assert!(trial_has_budget(true, 299, end, 298, 299));
        assert!(!trial_has_budget(true, 299, end, 299, 299));
        for iteration in 0..=40 {
            for frame in 0..=40 {
                assert_eq!(trial_has_budget(false, iteration, end, frame, 30),
                    (iteration as usize) < end && frame < 30);
            }
        }
    }

    #[test]
    fn fatal_dad_launch_admission_is_opt_in() {
        let live = super::Frame { alive_after: true, chest_alive: true, ..Default::default() };
        let dead = super::Frame { alive_after: false, ..live };
        let removed = super::Frame { chest_alive: false, ..live };
        assert!(launch_admissible(true, 3, &live, true));
        for f in [dead, removed] {
            assert!(!launch_admissible(true, 3, &f, true));
            assert!(launch_admissible(false, 3, &f, false));
            assert!(launch_admissible(true, 4, &f, false));
        }
        assert!(!launch_admissible(true, 3, &live, false));
        // This policy concerns pool fitness only: trial's actual finish check precedes it.
        let same_frame_finish = super::Frame { finished: true, ..dead };
        assert!(same_frame_finish.finished);
        assert!(!launch_admissible(true, 3, &same_frame_finish, true));
    }

    #[test]
    fn continued_trial_retains_best_useful_launch_with_exact_consumed_prefix() {
        let first = super::Frame { frame: 12, alive_after: true, chest_alive: true, ..Default::default() };
        let later = super::Frame { frame: 20, ..first };
        let fatal = super::Frame { frame: 21, alive_after: false, chest_alive: false, ..first };
        let mut best = None;
        let inputs = [17, 49, 4, 16];
        for (score, consumed, f) in [(70.0, 2, first), (80.0, 3, later),
            (60.0, 3, later), (1.0, 4, fatal)]
        {
            if launch_admissible(true, 3, &f, true) {
                remember_best_launch(&mut best, 100.0, score, &inputs[..consumed], &f);
            }
        }
        let (score, masks, f) = best.as_ref().unwrap();
        assert_eq!(*score, 60.0);
        assert_eq!(masks, &inputs[..3]);
        assert_eq!(f.frame, 20);
        remember_best_launch(&mut best, 100.0, 60.0, &inputs, &first);
        assert_eq!(best.as_ref().unwrap().1, inputs[..3]); // equal scores cannot replace the prefix
    }


    #[test]
    fn production_trial_continues_after_duplicate_launch_to_verified_gbe_finish() {
        std::thread::Builder::new().stack_size(16 << 20).spawn(|| {
            use std::collections::HashSet;
            use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
            use std::sync::Mutex;

            // Actual Flash-verified fixture: no generated physics or altered core state.
            let fixture = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../reference/tas/finishes/gbeqe-finish-f217.tas");
            let source_bytes = std::fs::read(&fixture).unwrap();
            let source_text = std::str::from_utf8(&source_bytes).unwrap();
            let original = super::super::sim::parse_tas(source_text).unwrap();
            assert_eq!(original.len(), 217);
            let literal = super::super::sim::tas_text(&original);

            struct OwnedTemp(std::path::PathBuf);
            impl Drop for OwnedTemp {
                fn drop(&mut self) { let _ = std::fs::remove_dir_all(&self.0); }
            }
            static NEXT: AtomicU64 = AtomicU64::new(0);
            let unique = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)
                .unwrap().as_nanos();
            let output = std::env::temp_dir().join(format!("hw-perturb-gbe-f217-{}-{}-{}",
                std::process::id(), unique, NEXT.fetch_add(1, Ordering::Relaxed)));
            std::fs::create_dir(&output).unwrap();
            let _cleanup = OwnedTemp(output.clone());
            let legacy_dir = output.join("legacy");
            let continuing_dir = output.join("finish-any");
            std::fs::create_dir(&legacy_dir).unwrap();
            std::fs::create_dir(&continuing_dir).unwrap();

            let template = super::Sim::new(3214, 4).unwrap();
            let preloaded = || {
                let mut sim = template.fork();
                sim.replay = hw_core::game::session::ReplayState::new(original.len() as u32);
                sim.masks = original.clone();
                sim
            };
            // Discover the endpoint with a separate deterministic first step, then start
            // both production trials from the untouched template with all217 inputs.
            let mut first_step = preloaded();
            let first = first_step.step(0);
            assert!(!first.port_abort && !first.finished && first.alive_before);
            assert!(super::in_world(first.chest_before));
            assert!(first.displacement >= 0.0);
            assert_eq!((first.frame, first_step.replay.iteration), (1, 1));
            assert_eq!(first.mask, original[0]);
            let endpoint = (first.chest_after[0].to_bits(), first.chest_after[1].to_bits());

            // Parent horizon1/extra0 recreates the early-return bug even though Sim has
            // the entire literal fixture preloaded. min_launch0 exercises a normal move.
            let parent = super::Member { masks: original[..1].to_vec(), miss: 1e6,
                trials: 0, id: 0 };
            let mut cfg = super::Config {
                level: 3214, character: 4, bases: vec![original.clone()],
                run_dir: legacy_dir.clone(), threads: 0, hours: 0.0, seed: 0,
                min_launch: 0.0, max_frame: 299, finish_any: false, window: 0,
                anchor: 0, qe_persist: 1, extra: 0, target: [0.0, 0.0], bounds: None,
                pool_size: 8, qe_mode: false, route: false, fc_mode: false,
            };
            let make_shared = |dir: &std::path::Path| super::Shared {
                pool: Mutex::new(vec![parent.clone()]), next_id: AtomicU64::new(1),
                stop: AtomicBool::new(false), frames: AtomicU64::new(0),
                trials: AtomicU64::new(0), launches: AtomicU64::new(0),
                improvements: AtomicU64::new(0), best: Mutex::new(parent.miss),
                best_qe: Mutex::new(None),
                log: Mutex::new(std::fs::File::create(dir.join("improvements.jsonl")).unwrap()),
                launch_log: Mutex::new(std::fs::File::create(dir.join("launches.jsonl")).unwrap()),
                winner: Mutex::new(None), seen: Mutex::new(HashSet::from([endpoint])),
                duplicates: AtomicU64::new(0), lag_kills: AtomicU64::new(0),
                sweep: Mutex::new(super::Sweep { queue: Default::default(), swept: Default::default() }),
            };

            let legacy = make_shared(&legacy_dir);
            let mut legacy_sim = preloaded();
            super::trial(&cfg, &legacy, &mut super::Rng(0), &mut legacy_sim, &parent, &template);
            assert_eq!(legacy_sim.frame(), 1);
            assert_eq!(legacy.duplicates.load(Ordering::Relaxed), 1);
            assert!(legacy.winner.lock().unwrap().is_none());
            assert!(!legacy_dir.join("winner-1.tas").exists());

            cfg.finish_any = true;
            cfg.run_dir = continuing_dir.clone();
            let continuing = make_shared(&continuing_dir);
            let mut continuing_sim = preloaded();
            super::trial(&cfg, &continuing, &mut super::Rng(0), &mut continuing_sim, &parent, &template);
            assert_eq!((continuing_sim.frame(), continuing_sim.replay.finish_frame), (217, 217));
            assert_eq!(continuing_sim.replay.iteration, 217);
            assert_eq!(continuing.frames.load(Ordering::Relaxed), 217);
            assert!(continuing.duplicates.load(Ordering::Relaxed) > 0);
            assert!(continuing.stop.load(Ordering::Relaxed));
            assert_eq!(continuing.winner.lock().unwrap().as_ref(), Some(&original));
            assert_eq!(std::fs::read_to_string(continuing_dir.join("winner-1.tas")).unwrap(), literal);
            // The production winner record is written only after an independent clean
            // confirm_any_finish replay succeeds; this invokes the real confirmation.
            let log = std::fs::read_to_string(continuing_dir.join("improvements.jsonl")).unwrap();
            let winners: Vec<serde_json::Value> = log.lines()
                .map(|line| serde_json::from_str::<serde_json::Value>(line).unwrap())
                .filter(|row| row["winner"].as_bool() == Some(true)).collect();
            assert_eq!(winners.len(), 1);
            assert_eq!(winners[0]["frame"].as_u64(), Some(217));
            assert_eq!(winners[0]["tas"].as_str(), Some(literal.as_str()));
            assert_eq!(legacy_sim.masks, original);
            assert_eq!(continuing_sim.masks, original);
            assert_eq!(std::fs::read(&fixture).unwrap(), source_bytes);
        }).unwrap().join().unwrap();
    }

    #[test]
    fn zero_anchor_preserves_mutation_bytes_and_rng() {
        for len in [1, 2, 3, 16, 80] {
            for window in [0, 1, 8, 100] {
                for seed in 0..128 {
                    let original: Vec<u8> = (0..len).map(|i| ((i * 37 + 19) % 256) as u8).collect();
                    let (mut legacy, mut anchored) = (original.clone(), original);
                    let (mut a, mut b) = (Rng(seed), Rng(seed));
                    let changed = mutate(&mut a, &mut legacy, window);
                    assert_eq!(mutate_anchored(&mut b, &mut anchored, window, 0), changed);
                    assert_eq!(anchored, legacy);
                    assert_eq!(b.0, a.0);
                }
            }
            for combo in 0..1024 {
                let original: Vec<u8> = (0..len).map(|i| ((i * 37 + 19) % 256) as u8).collect();
                let (mut legacy, mut anchored) = (original.clone(), original);
                assert_eq!(sweep_masks_anchored(&mut anchored, combo, 0), sweep_masks(&mut legacy, combo));
                assert_eq!(anchored, legacy);
            }
        }
    }

    #[test]
    fn anchor_survives_retriggers_and_grab_edge_shifts() {
        let prefix = [17, 17, 4, 4, 16, 49, 20, 24, 80];
        let suffix: Vec<u8> = (0..80).map(|i| if i % 6 < 3 { 1 } else { 17 }).collect();
        let original: Vec<u8> = prefix.into_iter().chain(suffix).collect();
        let (mut retriggers, mut grab_shifts) = (0, 0);
        for seed in 0..1024 {
            // Identify the existing mutation branches without calling any simulation.
            let mut probe = Rng(seed);
            let retrigger = probe.below(10) < 6;
            let grab_shift = if retrigger {
                false
            } else {
                probe.below(2);
                probe.next(); // either depth branch consumes exactly one draw
                probe.below(10) == 3
            };
            let mut masks = original.clone();
            let changed = mutate_anchored(&mut Rng(seed), &mut masks, 100, prefix.len() as u32);
            assert_eq!(&masks[..prefix.len()], &prefix);
            assert!(changed >= prefix.len());
            assert!(masks.len() >= prefix.len() + 2);
            if retrigger && masks.len() < original.len() { retriggers += 1; }
            if grab_shift && masks != original { grab_shifts += 1; }
        }
        assert!(retriggers > 100, "test must exercise truncating retriggers");
        assert!(grab_shifts > 10, "test must exercise actual grab transition shifts");
        for combo in 0..1024 {
            let mut masks = original.clone();
            assert!(sweep_masks_anchored(&mut masks, combo, prefix.len() as u32) >= prefix.len());
            assert_eq!(&masks[..prefix.len()], &prefix);
        }
    }

    #[test]
    fn exhausted_or_single_entry_suffix_does_not_consume_rng() {
        for (masks, anchor) in [(vec![], 1), (vec![17], 1), (vec![17, 49], 1), (vec![17, 49], 2), (vec![17, 49], 3)] {
            let mut actual = masks.clone();
            let mut rng = Rng(123);
            assert_eq!(mutate_anchored(&mut rng, &mut actual, 80, anchor), masks.len());
            assert_eq!(actual, masks);
            assert_eq!(rng.0, 123);
        }
        assert_eq!(base_keep(0, 24, 240), 24);
        assert_eq!(base_keep(60, 90, 240), 240);
    }

    #[test]
    fn persistent_min_suppresses_a_single_frame_spike() {
        let mut window = QePersistence::new(3);
        let scores: Vec<_> = [10.0, 10.0, 1000.0, 10.0, 10.0].into_iter()
            .map(|qe| window.observe(qe, true)).collect();
        assert_eq!(scores, vec![None, None, Some(10.0), Some(10.0), Some(10.0)]);
        assert_eq!(window.observe(500.0, true), Some(10.0));
        assert_eq!(window.observe(600.0, true), Some(10.0));
        assert_eq!(window.observe(700.0, true), Some(500.0));
        assert!(qe_score(500.0) < qe_score(10.0));
    }

    #[test]
    fn persistence_resets_on_ineligible_or_nonfinite_frames() {
        let mut window = QePersistence::new(2);
        assert_eq!(window.observe(100.0, true), None);
        assert_eq!(window.observe(1000.0, false), None); // dead, aborted, or out of bounds
        assert_eq!(window.observe(200.0, true), None);
        assert_eq!(window.observe(300.0, true), Some(200.0));
        assert_eq!(window.observe(f64::NAN, true), None);
        assert_eq!(window.observe(400.0, true), None);
        assert_eq!(window.observe(500.0, true), Some(400.0));
        assert_eq!(window.observe(f64::INFINITY, true), None);
    }

    #[test]
    fn one_frame_persistence_keeps_scalar_value_and_score_exact() {
        let mut window = QePersistence::new(1);
        for qe in [-0.0_f64, 0.0, 1.0, 1172.6064416466552, 1e13] {
            let observed = window.observe(qe, true).unwrap();
            assert_eq!(observed.to_bits(), qe.to_bits());
            assert_eq!(qe_score(observed).to_bits(), qe_score(qe).to_bits());
        }
    }

    struct ScriptedPrefix {
        iteration: u32,
        frame: u32,
        mutations: u32,
        events: std::collections::VecDeque<(bool, bool)>, // input advanced, port-only abort
    }

    impl PrefixCursor for ScriptedPrefix {
        fn cursor(&self) -> (u32, u32) { (self.iteration, self.frame) }
        fn prefix_step(&mut self) -> bool {
            let (advanced, port_abort) = self.events.pop_front().unwrap_or((false, false));
            self.frame += 1;
            self.mutations += 1; // original aborts retain their pre-throw mutations
            if advanced { self.iteration += 1; }
            port_abort
        }
    }

    #[test]
    fn prefix_waits_for_consumed_input_and_keeps_throw_mutations() {
        let mut s = ScriptedPrefix { iteration: 0, frame: 0, mutations: 0,
            events: [(false, false), (false, false), (true, false), (true, false)].into() };
        let result = replay_prefix(&mut s, 2, 299);
        assert_eq!(result.exit, PrefixExit::Reached);
        assert_eq!((result.steps, s.iteration, s.frame, s.mutations), (4, 2, 4, 4));
        // Fixed two-step replay would have falsely cached offset2 at actual offset0.
    }

    #[test]
    fn prefix_failure_cannot_be_labelled_as_a_reached_cache_offset() {
        let mut port = ScriptedPrefix { iteration: 0, frame: 0, mutations: 0,
            events: [(true, true)].into() };
        let failed = replay_prefix(&mut port, 1, 299);
        assert_eq!(failed.exit, PrefixExit::PortAbort); // even if input already advanced
        assert_eq!((failed.steps, port.mutations), (1, 1));
        let mut stalled = ScriptedPrefix { iteration: 0, frame: 0, mutations: 0, events: Default::default() };
        let failed = replay_prefix(&mut stalled, 1, 299);
        assert_eq!(failed.exit, PrefixExit::NoInputProgress);
        assert_eq!(failed.steps, MAX_STALLED_INPUT_FRAMES as u64);
        assert_eq!(stalled.iteration, 0);
        let mut ceiling = ScriptedPrefix { iteration: 0, frame: 298, mutations: 0,
            events: [(false, false), (true, false)].into() };
        let failed = replay_prefix(&mut ceiling, 1, 299);
        assert_eq!(failed.exit, PrefixExit::FrameLimit);
        assert_eq!((failed.steps, ceiling.iteration, ceiling.frame), (1, 0, 299));
    }

    #[test]
    fn world_throw_progress_resets_the_host_stall_budget() {
        let mut events = std::collections::VecDeque::new();
        events.extend(std::iter::repeat((false, false)).take(31));
        events.push_back((true, false)); // original world-stage throw may consume this entry
        events.extend(std::iter::repeat((false, false)).take(31));
        events.push_back((true, false));
        let mut s = ScriptedPrefix { iteration: 0, frame: 0, mutations: 0, events };
        let result = replay_prefix(&mut s, 2, 299);
        assert_eq!(result.exit, PrefixExit::Reached);
        assert_eq!((result.steps, s.iteration), (64, 2));
    }

    #[test]
    fn repeated_input_cannot_fill_or_preserve_a_qe_window() {
        let mut q = QePersistence::new(2);
        assert_eq!(q.observe_step(10.0, true, 0, 1), None);
        assert_eq!(q.observe_step(1e12, true, 1, 1), None); // same input slot, window reset
        assert_eq!(q.observe_step(20.0, true, 1, 2), None);
        assert_eq!(q.observe_step(30.0, true, 2, 3), Some(20.0));
        assert_eq!(q.observe_step(1e12, true, 3, 3), None);
        assert_eq!(q.observe_step(40.0, true, 3, 4), None);
        assert_eq!(q.observe_step(50.0, true, 4, 5), Some(40.0));
    }

    #[test]
    fn fresh_scalar_qe_stays_bit_exact_and_original_world_throw_is_not_filtered() {
        let mut q = QePersistence::new(1);
        for value in [-0.0_f64, 0.0, 1.0, 1172.6064416466552, 1e13] {
            assert_eq!(q.observe_step(value, true, 9, 10).unwrap().to_bits(), value.to_bits());
        }
        let f = super::Frame { site: 2, qe: 123.0, alive_after: true,
            ..Default::default() };
        assert_eq!(q.observe_step(f.qe, !f.port_abort && f.alive_after, 9, 10), Some(123.0));
    }

    #[test]
    fn exact_cached_prefix_matches_sequential_sim_without_aborts() {
        std::thread::Builder::new().stack_size(16 << 20).spawn(|| {
            let template = super::Sim::new(45, 3).unwrap();
            let masks = vec![0; 32];
            let mut sequential = template.fork();
            sequential.masks = masks.clone();
            for expected in 1..=16 {
                assert!(!sequential.step(0).port_abort);
                assert_eq!(sequential.replay.iteration, expected);
            }
            let snapshot = sequential.fork();
            for expected in 17..=32 {
                assert!(!sequential.step(0).port_abort);
                assert_eq!(sequential.replay.iteration, expected);
            }
            let mut resumed = template.fork();
            resumed.restore(&snapshot);
            resumed.masks = masks.clone();
            let result = replay_prefix(&mut resumed, 32, 299);
            assert_eq!(result.exit, PrefixExit::Reached);
            assert_eq!(result.steps, 16);
            assert_eq!(resumed.masks, masks);
            assert_eq!(hw_core::diagnostic::replay_string(&resumed.ctx, &resumed.replay),
                hw_core::diagnostic::replay_string(&sequential.ctx, &sequential.replay));
            assert_eq!(hw_core::links_observer::links_string(&resumed.ctx),
                hw_core::links_observer::links_string(&sequential.ctx));
        }).unwrap().join().unwrap();
    }
}
