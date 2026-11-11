//! `hw-search run`: Go-Explore style search for a one-frame chest launch to the finish.
//!
//! The archive keeps one input prefix per cell (chest position on a grid, grip count, QE
//! magnitude bucket, ejected). Workers pick promising cells, restore them (per-worker snapshot
//! cache, else replay the prefix), extend them with random macros of grab + pose inputs and check
//! every frame for success: alive entering the frame, finish latched during it, and the chest
//! moved at least `min_launch` metres. `--finish-any` also accepts a later finish latch without
//! a same-frame launch. Dad cells also distinguish limb, bicycle and child topology.
//! Every large chest launch is logged as a sample.
use super::sim::{tas_text, Frame, Sim, State};
use serde_json::json;
use std::collections::HashMap;
use std::io::Write;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

pub struct Config {
    pub level: u32,
    pub character: u32,
    pub seeds: Vec<Vec<u8>>,
    pub run_dir: PathBuf,
    pub threads: usize,
    pub hours: f64,
    pub seed: u64,
    pub min_launch: f64,
    /// Accept any replay-confirmed finish latch, independently of launch scoring.
    pub finish_any: bool,
    pub max_frame: u32,
    pub rollout: u32,
    /// Finish region centre (for near-miss distances).
    pub target: [f64; 2],
    /// Cannon success counts per 10 m origin grid point ("x y n" lines): where launches work.
    pub site_map: HashMap<(i32, i32), f64>,
    /// Chest region [x0, x1, y0, y1] (m) where the QE should build and launch; outside it
    /// cells are nearly ignored and launches do not mark states hot.
    pub region: Option<[f64; 4]>,
    /// Hard chest bounds [x0, x1, y0, y1] (m): a state outside them is never kept or extended
    /// (e.g. Satan: the chest must stay in front of the teeth, on or above the start platform).
    pub bounds: Option<[f64; 4]>,
    /// Drive mode: find the fastest mounted ride along the level (no QE scoring, no eject).
    pub drive: bool,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
struct Key {
    cx: i32,
    cy: i32,
    grips: u32,
    static_grips: u32,
    jointed_grips: u32,
    qe: i32,
    ejected: bool,
    dad: Option<DadTopology>,
}

/// Dad's damaged limbs, vehicle and child can share a position/QE cell but evolve differently.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
struct DadTopology {
    pose: i32,
    character_joints: u16,
    bicycle_joints: u16,
    child_ejected: bool,
    child_detached: bool,
    child_joints: u16,
    seat_joints: u16,
    held_bodies: [hw_core::H; 4],
}

struct Entry {
    masks: Vec<u8>,
    frame: u32,
    key: Key,
    visits: u64,
    /// Launches seen in rollouts from this entry (launch-prone states are revisited).
    launches: u64,
}

struct Archive {
    entries: Vec<Entry>,
    index: HashMap<Key, usize>,
}

fn qe_bucket(qe: f64) -> i32 {
    if qe < 10.0 {
        0
    } else {
        (qe.log10().floor() as i32).clamp(1, 9)
    }
}

fn live_joint_mask(ctx: &State, handles: &[hw_core::H]) -> u16 {
    handles.iter().enumerate().fold(0, |bits, (i, &j)| {
        if ctx.world.joints.is_alive(j) && !ctx.world.joint(j).broken.get() { bits | (1 << i) } else { bits }
    })
}

fn held_body(ctx: &State, joint: hw_core::H) -> hw_core::H {
    if ctx.world.joints.is_alive(joint) && !ctx.world.joint(joint).broken.get() {
        let held = ctx.world.joint(joint).body1;
        if ctx.world.bodies.is_alive(held) {
            return held;
        }
    }
    hw_core::NONE
}

fn key_of(ctx: &State, f: &Frame) -> Key {
    let dad = &ctx.character.primary;
    let child = &ctx.character.child;
    Key {
        cx: (f.chest_after[0] / 4.0).floor() as i32,
        cy: (f.chest_after[1] / 4.0).floor() as i32,
        grips: f.grips,
        static_grips: f.static_grips,
        jointed_grips: f.jointed_grips,
        qe: qe_bucket(f.qe),
        ejected: f.ejected,
        dad: (dad.kind == 3).then(|| DadTopology {
            pose: dad.current_pose,
            character_joints: live_joint_mask(ctx, &dad.joints),
            bicycle_joints: live_joint_mask(ctx, &dad.bicycle.joints),
            child_ejected: child.child_seat.ejected.get(),
            child_detached: child.child_seat.detached.get(),
            child_joints: live_joint_mask(ctx, &child.joints),
            seat_joints: live_joint_mask(ctx, &child.child_seat.joints),
            held_bodies: [
                held_body(ctx, dad.grip_joints[0]), held_body(ctx, dad.grip_joints[1]),
                held_body(ctx, child.grip_joints[0]), held_body(ctx, child.grip_joints[1]),
            ],
        }),
    }
}

fn weight(cfg: &Config, e: &Entry) -> f64 {
    let k = e.key;
    if cfg.drive {
        // Frontier progress (the level runs left to right) per frame spent, mounted only.
        if k.ejected {
            return 0.0;
        }
        let x = (k.cx * 4) as f64;
        return (x / 6.0).exp() * (-(e.frame as f64) / 40.0).exp() / (1.0 + e.visits as f64).sqrt();
    }
    let qe = (k.qe as f64 + 1.0).powi(3);
    let grip = [1.0, 2.0, 4.0][k.grips.min(2) as usize] * [1.0, 1.5, 2.0][k.static_grips.min(2) as usize] * [1.0, 20.0, 60.0][k.jointed_grips.min(2) as usize];
    // Successful launch origins are in open air above the level (cannon maps: y < ~30 m).
    // Site value from the cannon origin map (nearest 10 m grid point), else a mild prior.
    let (x, y) = (k.cx * 4 + 2, k.cy * 4 + 2);
    let open = if let Some(r) = cfg.region {
        let (xf, yf) = (x as f64, y as f64);
        if xf >= r[0] && xf <= r[1] && yf >= r[2] && yf <= r[3] { 10.0 } else { 0.02 }
    } else if cfg.site_map.is_empty() {
        1.0
    } else {
        let g = ((x as f64 / 10.0).round() as i32 * 10, (y as f64 / 10.0).round() as i32 * 10);
        0.2 + cfg.site_map.get(&g).copied().unwrap_or(0.0).min(40.0)
    };
    let eject = if k.ejected { 2.0 } else { 1.0 };
    let hot = 1.0 + 5.0 * e.launches as f64;
    qe * grip * open * eject * hot / (1.0 + e.visits as f64).sqrt()
}

/// SplitMix64.
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

// Key bits: left 1, right 2, up 4, down 8, space 16, shift 32, ctrl 64, z 128.
const DRIVE: [u8; 8] = [4, 4 | 2, 4 | 1, 8, 16, 0, 2, 1];
const DAD_DRIVE: [u8; 12] = [4, 4 | 2, 4 | 1, 8, 16, 0, 2, 1, 1 | 16, 2 | 16, 4 | 16, 8 | 16];
const POSES: [u8; 7] = [0, 1, 2, 4, 8, 32, 64];

/// A macro: (mask, frames).
fn random_macro(rng: &mut Rng, character: u32, ejected: bool, drive: bool) -> (u8, u32) {
    if drive {
        // Mounted driving only: accelerate/lean/jump combinations, never eject.
        const RIDE: [u8; 8] = [4, 4 | 2, 4 | 1, 4 | 16, 4 | 2 | 16, 4 | 1 | 16, 0, 2];
        return (RIDE[rng.below(RIDE.len() as u64) as usize], 1 + rng.below(10) as u32);
    }
    // Half the macros hold one input long enough for a gripped QE to build (reference TASes
    // hold one pose + grab for 50-70 frames).
    let len = if rng.below(2) == 0 { 1 + rng.below(12) as u32 } else { 15 + rng.below(76) as u32 };
    if !ejected {
        if rng.below(20) == 0 {
            if character == 3 {
                // Dad-only (Shift), child-only (Ctrl), or both (Z).
                const EJECT: [u8; 3] = [32, 64, 128];
                return (EJECT[rng.below(EJECT.len() as u64) as usize], 1);
            }
            return (128, 1);
        }
        let drive = if character == 3 { &DAD_DRIVE[..] } else { &DRIVE[..] };
        return (drive[rng.below(drive.len() as u64) as usize], len);
    }
    let pose = POSES[rng.below(POSES.len() as u64) as usize];
    let grab = if rng.unit() < 0.85 { 16 } else { 0 };
    (pose | grab, len)
}

struct Shared {
    archive: Mutex<Archive>,
    stop: AtomicBool,
    frames: AtomicU64,
    rollouts: AtomicU64,
    launches: AtomicU64,
    port_aborts: AtomicU64,
    lag_kills: AtomicU64,
    jolts: AtomicU64,
    best_miss: Mutex<f64>,
    max_qe: Mutex<f64>,
    log: Mutex<std::fs::File>,
    winners: Mutex<Vec<Vec<u8>>>,
    /// Prefixes ending 1-4 frames before a logged launch: re-sampled with fresh inputs.
    hot: Mutex<Vec<Vec<u8>>>,
}

fn in_bounds(cfg: &Config, p: [f64; 2]) -> bool {
    match cfg.bounds {
        Some(b) => p[0] >= b[0] && p[0] <= b[1] && p[1] >= b[2] && p[1] <= b[3],
        None => true,
    }
}

/// Inside the level's world AABB (-4,-84)-(324,164): a chest outside it can never finish.
fn in_world(p: [f64; 2]) -> bool {
    p[0] > -4.0 && p[0] < 324.0 && p[1] > -84.0 && p[1] < 164.0
}

fn distance_to(target: [f64; 2], p: [f64; 2]) -> f64 {
    ((p[0] - target[0]).powi(2) + (p[1] - target[1]).powi(2)).sqrt()
}

/// Exact TAS-semantics replay of a candidate; returns the finish frame and whether the chest
/// launched into it (alive entering the frame).
pub fn confirm(level: u32, character: u32, masks: &[u8], min_launch: f64) -> Result<(u32, f64), String> {
    confirm_replay(level, character, masks, Some(min_launch))
}

/// Exact from-scratch replay accepting any actual finish latch, without a launch requirement.
pub(super) fn confirm_any_finish(level: u32, character: u32, masks: &[u8]) -> Result<(u32, f64), String> {
    confirm_replay(level, character, masks, None)
}

fn finish_candidate(f: &Frame, min_launch: Option<f64>) -> bool {
    f.finished && min_launch.map_or(true, |minimum| f.alive_before && f.displacement >= minimum)
}

fn confirm_replay(level: u32, character: u32, masks: &[u8], min_launch: Option<f64>) -> Result<(u32, f64), String> {
    let mut s = Sim::new(level, character)?;
    s.replay = hw_core::game::session::ReplayState::new(masks.len() as u32);
    s.masks = masks.to_vec();
    confirm_frames((0..masks.len() + 60).map(|_| s.step(0)), min_launch)
}

fn confirm_frames(frames: impl Iterator<Item = Frame>, min_launch: Option<f64>) -> Result<(u32, f64), String> {
    for f in frames {
        if f.port_abort {
            return Err(format!("port-only abort at frame {}", f.frame));
        }
        if f.finished {
            if finish_candidate(&f, min_launch) {
                return Ok((f.frame, f.displacement));
            }
            return Err(format!("finish at frame {} without a launch (moved {:.1} m)", f.frame, f.displacement));
        }
    }
    Err("no finish".into())
}

#[derive(Debug, PartialEq, Eq)]
enum SeedAction { Archive, Confirm, Stop }

fn seed_frame_action(f: &Frame, min_launch: Option<f64>, max_frame: u32) -> SeedAction {
    if f.port_abort || f.frame > max_frame {
        SeedAction::Stop
    } else if f.finished {
        if finish_candidate(f, min_launch) { SeedAction::Confirm } else { SeedAction::Stop }
    } else if !f.alive_after || f.frame >= max_frame {
        SeedAction::Stop
    } else {
        SeedAction::Archive
    }
}

fn record_winner(cfg: &Config, shared: &Shared, masks: Vec<u8>, frame: u32, moved: f64) -> Result<(), String> {
    let mut winners = shared.winners.lock().unwrap();
    let name = cfg.run_dir.join(format!("winner-{}.tas", winners.len() + 1));
    std::fs::write(&name, tas_text(&masks)).map_err(|e| format!("{}: {e}", name.display()))?;
    eprintln!("WINNER: finish frame {frame}, chest moved {moved:.1} m -> {}", name.display());
    winners.push(masks);
    shared.stop.store(true, Ordering::Relaxed);
    Ok(())
}

/// Both seed replay and rollouts require a clean, from-scratch confirmation before recording.
fn confirm_winner(cfg: &Config, shared: &Shared, s: &Sim, f: &Frame) {
    if f.port_abort || f.frame > cfg.max_frame || !finish_candidate(f, if cfg.finish_any { None } else { Some(cfg.min_launch) }) {
        return;
    }
    let masks = s.masks[..s.replay.iteration as usize].to_vec();
    let confirmed = if cfg.finish_any {
        confirm_any_finish(cfg.level, cfg.character, &masks)
    } else {
        confirm(cfg.level, cfg.character, &masks, cfg.min_launch)
    };
    let result = confirmed.and_then(|(frame, moved)| {
        if frame > cfg.max_frame {
            return Err(format!("confirmed finish frame {frame} exceeds --max-frame {}", cfg.max_frame));
        }
        record_winner(cfg, shared, masks, frame, moved)
    });
    if let Err(e) = result {
        eprintln!("candidate at frame {} failed confirmation: {e}", f.frame);
    }
}

pub fn run(cfg: Config) -> Result<Vec<u8>, String> {
    std::fs::create_dir_all(&cfg.run_dir).map_err(|e| e.to_string())?;
    let template = Sim::new(cfg.level, cfg.character)?;
    let log = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(cfg.run_dir.join("launches.jsonl"))
        .map_err(|e| e.to_string())?;
    let shared = Shared {
        archive: Mutex::new(Archive { entries: Vec::new(), index: HashMap::new() }),
        stop: AtomicBool::new(false),
        frames: AtomicU64::new(0),
        rollouts: AtomicU64::new(0),
        launches: AtomicU64::new(0),
        port_aborts: AtomicU64::new(0),
        lag_kills: AtomicU64::new(0),
        jolts: AtomicU64::new(0),
        best_miss: Mutex::new(f64::INFINITY),
        max_qe: Mutex::new(0.0),
        log: Mutex::new(log),
        winners: Mutex::new(Vec::new()),
        hot: Mutex::new(Vec::new()),
    };
    // Seed entries: every frame of every seed prefix becomes a candidate cell.
    for seed in &cfg.seeds {
        let mut s = template.fork();
        s.masks = seed.clone();
        while s.frame() < cfg.max_frame && (s.replay.iteration as usize) < seed.len() {
            let f = s.step(0);
            match seed_frame_action(&f, if cfg.finish_any { None } else { Some(cfg.min_launch) }, cfg.max_frame) {
                SeedAction::Confirm => {
                    confirm_winner(&cfg, &shared, &s, &f);
                    break;
                }
                SeedAction::Stop => break,
                SeedAction::Archive => {}
            }
            if !in_world(f.chest_after) || !in_bounds(&cfg, f.chest_after) {
                break;
            }
            insert(&shared, &s, &f);
        }
        if shared.stop.load(Ordering::Relaxed) {
            return Ok(shared.winners.lock().unwrap().first().unwrap().clone());
        }
    }
    if shared.archive.lock().unwrap().entries.is_empty() {
        insert(&shared, &template, &Frame { chest_after: super::sim::chest(&template.ctx), ..Frame::default() });
    }
    let start = Instant::now();
    let deadline = start + Duration::from_secs_f64(cfg.hours * 3600.0);
    std::thread::scope(|scope| {
        for w in 0..cfg.threads {
            let shared = &shared;
            let cfg = &cfg;
            let template = &template;
            std::thread::Builder::new()
                .stack_size(16 << 20)
                .spawn_scoped(scope, move || worker(w as u64, cfg, template, shared))
                .unwrap();
        }
        // Monitor.
        let mut stats = std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("stats.jsonl")).unwrap();
        while !shared.stop.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_secs(10));
            if Instant::now() > deadline || cfg.run_dir.join("STOP").exists() {
                shared.stop.store(true, Ordering::Relaxed);
            }
            let a = shared.archive.lock().unwrap();
            if start.elapsed().as_secs() % 300 < 10 {
                if let Ok(mut out) = std::fs::File::create(cfg.run_dir.join("archive-live.jsonl")) {
                    for e in &a.entries {
                        writeln!(out, "{}", json!({"key": format!("{:?}", e.key), "frame": e.frame, "visits": e.visits, "launches": e.launches})).ok();
                    }
                }
            }
            let cells = a.entries.len();
            let qe_cells = a.entries.iter().filter(|e| e.key.qe >= 4 && e.key.grips >= 1).count();
            let static2 = a.entries.iter().filter(|e| e.key.static_grips == 2).count();
            let jointed = a.entries.iter().filter(|e| e.key.jointed_grips > 0).count();
            let jointed_qe = a.entries.iter().filter(|e| e.key.jointed_grips > 0 && e.key.qe >= 3).count();
            drop(a);
            let line = json!({
                "t": start.elapsed().as_secs(),
                "frames": shared.frames.load(Ordering::Relaxed),
                "fps": shared.frames.load(Ordering::Relaxed) as f64 / start.elapsed().as_secs_f64(),
                "rollouts": shared.rollouts.load(Ordering::Relaxed),
                "cells": cells,
                "gripped_qe_cells": qe_cells,
                "static2_cells": static2,
                "jointed_grip_cells": jointed,
                "jointed_qe_cells": jointed_qe,
                "launches": shared.launches.load(Ordering::Relaxed),
                "jolts": shared.jolts.load(Ordering::Relaxed),
                "best_miss_m": *shared.best_miss.lock().unwrap(),
                "max_qe": *shared.max_qe.lock().unwrap(),
                "port_aborts": shared.port_aborts.load(Ordering::Relaxed),
                "lag_kills": shared.lag_kills.load(Ordering::Relaxed),
                "winners": shared.winners.lock().unwrap().len(),
            });
            writeln!(stats, "{line}").ok();
            eprintln!("{line}");
        }
    });
    // Write the archive's prefixes for resume/inspection.
    let a = shared.archive.lock().unwrap();
    let mut out = std::fs::File::create(cfg.run_dir.join("archive.jsonl")).map_err(|e| e.to_string())?;
    for e in &a.entries {
        writeln!(out, "{}", json!({"key": format!("{:?}", e.key), "frame": e.frame, "visits": e.visits, "tas": tas_text(&e.masks)})).ok();
    }
    let winners = shared.winners.lock().unwrap();
    winners.first().cloned().ok_or_else(|| "search ended without a confirmed winner".to_string())
}

fn insert(shared: &Shared, s: &Sim, f: &Frame) {
    let key = key_of(&s.ctx, f);
    let mut a = shared.archive.lock().unwrap();
    match a.index.get(&key) {
        Some(&i) => {
            // Prefer the earlier frame for the same cell.
            if f.frame < a.entries[i].frame {
                a.entries[i].masks = s.masks[..s.replay.iteration as usize].to_vec();
                a.entries[i].frame = f.frame;
            }
        }
        None => {
            let i = a.entries.len();
            a.entries.push(Entry { masks: s.masks[..s.replay.iteration as usize].to_vec(), frame: f.frame, key, visits: 0, launches: 0 });
            a.index.insert(key, i);
        }
    }
}

fn pick(cfg: &Config, shared: &Shared, rng: &mut Rng) -> (usize, Vec<u8>) {
    let mut a = shared.archive.lock().unwrap();
    let total: f64 = a.entries.iter().map(|e| weight(cfg, e)).sum();
    let mut x = rng.unit() * total;
    let mut i = a.entries.len() - 1;
    for (k, e) in a.entries.iter().enumerate() {
        x -= weight(cfg, e);
        if x <= 0.0 {
            i = k;
            break;
        }
    }
    a.entries[i].visits += 1;
    (i, a.entries[i].masks.clone())
}

fn worker(w: u64, cfg: &Config, template: &Sim, shared: &Shared) {
    let mut rng = Rng(cfg.seed ^ (w + 1).wrapping_mul(0x5851_f42d_4c95_7f2d));
    // Per-worker snapshot cache: entry index -> state at the end of its prefix.
    let mut cache: Vec<(usize, Vec<u8>, Sim)> = Vec::new();
    let mut work = template.fork();
    while !shared.stop.load(Ordering::Relaxed) {
        let hot = {
            let h = shared.hot.lock().unwrap();
            if !h.is_empty() && rng.below(3) == 0 { Some(h[rng.below(h.len() as u64) as usize].clone()) } else { None }
        };
        let (id, prefix) = match hot {
            Some(p) => (usize::MAX, p),
            None => pick(cfg, shared, &mut rng),
        };
        if let Some(c) = cache.iter().find(|c| c.0 == id && c.1 == prefix) {
            work.restore(&c.2);
        } else {
            work.restore(template);
            work.masks.clear();
            work.masks.extend_from_slice(&prefix);
            let mut ok = true;
            for _ in 0..prefix.len() {
                let f = work.step(0);
                if f.port_abort {
                    ok = false;
                    break;
                }
            }
            shared.frames.fetch_add(prefix.len() as u64, Ordering::Relaxed);
            if !ok {
                continue;
            }
            if cache.len() >= 3 {
                let (_, _, mut old) = cache.remove(0);
                old.restore(&work);
                cache.push((id, prefix.clone(), old));
            } else {
                cache.push((id, prefix.clone(), work.fork()));
            }
            // Input entries past the prefix are chosen below.
            work.masks.truncate(prefix.len());
        }
        shared.rollouts.fetch_add(1, Ordering::Relaxed);
        if rollout(cfg, shared, &mut rng, &mut work) && id != usize::MAX {
            shared.archive.lock().unwrap().entries[id].launches += 1;
        }
    }
}

fn rollout(cfg: &Config, shared: &Shared, rng: &mut Rng, s: &mut Sim) -> bool {
    let mut launched = false;
    let mut steps = 0;
    let mut macro_left = 0u32;
    let mut mask = 0u8;
    let mut lag_run = 0u32;
    while steps < cfg.rollout && s.frame() < cfg.max_frame {
        if macro_left == 0 {
            let (m, n) = random_macro(rng, cfg.character, super::sim::ejected(&s.ctx), cfg.drive);
            mask = m;
            macro_left = n;
        }
        macro_left -= 1;
        let f = s.step(mask);
        steps += 1;
        if f.port_abort {
            shared.port_aborts.fetch_add(1, Ordering::Relaxed);
            break;
        }
        {
            let mut m = shared.max_qe.lock().unwrap();
            if f.qe > *m && f.qe.is_finite() {
                *m = f.qe;
            }
        }
        // Bounds prune unfinished branches; an actual latch in finish-any mode still needs
        // confirmation even if the finishing step crossed them.
        if (!in_world(f.chest_before) || !in_bounds(cfg, f.chest_before)) && !(cfg.finish_any && f.finished) {
            break;
        }
        // A chest jolt during a QE marks a launch-prone state: re-sample the frames before it.
        let in_region = |p: [f64; 2]| match cfg.region {
            Some(r) => p[0] >= r[0] && p[0] <= r[1] && p[1] >= r[2] && p[1] <= r[3],
            None => true,
        };
        if f.alive_before && f.displacement >= 3.0 && f.displacement < cfg.min_launch && f.qe > 1e3 && in_region(f.chest_before) {
            shared.jolts.fetch_add(1, Ordering::Relaxed);
            launched = true;
            let mut h = shared.hot.lock().unwrap();
            let n = s.replay.iteration as usize;
            for back in 1..=3 {
                if n > back {
                    h.push(s.masks[..n - back].to_vec());
                }
            }
            if h.len() > 400 {
                let excess = h.len() - 400;
                h.drain(..excess);
            }
        }
        if f.alive_before && f.displacement >= cfg.min_launch {
            shared.launches.fetch_add(1, Ordering::Relaxed);
            let miss = distance_to(cfg.target, f.chest_after);
            {
                let mut b = shared.best_miss.lock().unwrap();
                if miss < *b {
                    *b = miss;
                }
            }
            let line = json!({"frame": f.frame, "from": f.chest_before, "to": f.chest_after, "moved": f.displacement,
                "miss_m": miss, "finished": f.finished, "qe": f.qe, "tas": s.tas()});
            writeln!(shared.log.lock().unwrap(), "{line}").ok();
            // Only launches from promising sites (cannon map) mark their states hot.
            let site = {
                let g = ((f.chest_before[0] / 10.0).round() as i32 * 10, (f.chest_before[1] / 10.0).round() as i32 * 10);
                cfg.site_map.get(&g).copied().unwrap_or(0.0)
            };
            let in_region = match cfg.region {
                Some(r) => f.chest_before[0] >= r[0] && f.chest_before[0] <= r[1] && f.chest_before[1] >= r[2] && f.chest_before[1] <= r[3],
                None => site > 0.0 || cfg.site_map.is_empty(),
            };
            if in_region {
                launched = true;
                let mut h = shared.hot.lock().unwrap();
                let n = s.replay.iteration as usize;
                for back in 1..=4 {
                    if n > back {
                        h.push(s.masks[..n - back].to_vec());
                    }
                }
                if h.len() > 400 {
                    let excess = h.len() - 400;
                    h.drain(..excess);
                }
            }
        }
        if finish_candidate(&f, if cfg.finish_any { None } else { Some(cfg.min_launch) }) {
            confirm_winner(cfg, shared, s, &f);
            break;
        }
        // A latched finish (e.g. riding into the pocket) makes later launches meaningless.
        if !f.alive_after || !in_world(f.chest_after) || !in_bounds(cfg, f.chest_after) || s.replay.finish_frame != 0 {
            break;
        }
        // A runaway QE that keeps lagging the game never leads to a clean launch: drop the
        // branch after `lag_frames()` laggy frames in a row.
        lag_run = if f.lagging { lag_run + 1 } else { 0 };
        if lag_run >= super::sim::lag_frames() {
            shared.lag_kills.fetch_add(1, Ordering::Relaxed);
            break;
        }
        insert(shared, s, &f);
    }
    launched
}

#[cfg(test)]
mod tests {
    use super::{confirm_frames, finish_candidate, key_of, random_macro, seed_frame_action, Frame, Rng, SeedAction, Sim};

    #[test]
    fn delayed_finish_requires_opt_in_despite_an_earlier_launch() {
        let frames = || [
            Frame { frame: 10, alive_before: true, displacement: 80.0, ..Frame::default() },
            Frame { frame: 11, alive_before: true, finished: true, displacement: 0.25, ..Frame::default() },
        ];
        assert!(!finish_candidate(&frames()[1], Some(20.0)));
        assert!(finish_candidate(&frames()[1], None));
        assert!(confirm_frames(frames().into_iter(), Some(20.0)).unwrap_err().contains("without a launch"));
        assert_eq!(confirm_frames(frames().into_iter(), None).unwrap(), (11, 0.25));
    }

    #[test]
    fn finish_any_still_requires_a_latch_and_rejects_port_aborts() {
        assert!(confirm_frames([Frame::default()].into_iter(), None).is_err());
        let abort = Frame { frame: 7, finished: true, port_abort: true, ..Frame::default() };
        assert!(confirm_frames([abort].into_iter(), None).unwrap_err().contains("port-only abort"));
        let launch = Frame { frame: 8, alive_before: true, finished: true, displacement: 20.0, ..Frame::default() };
        assert_eq!(confirm_frames([launch].into_iter(), Some(20.0)).unwrap(), (8, 20.0));
    }

    #[test]
    fn mounted_dad_samples_each_eject_action_while_other_characters_keep_z() {
        for character in [1, 2, 3, 4] {
            let mut rng = Rng(1);
            let mut ejections = std::collections::BTreeSet::new();
            for _ in 0..4096 {
                let (mask, frames) = random_macro(&mut rng, character, false, false);
                if mask & (32 | 64 | 128) != 0 {
                    assert_eq!(frames, 1);
                    ejections.insert(mask);
                }
            }
            let expected: std::collections::BTreeSet<u8> = if character == 3 {
                [32, 64, 128].into_iter().collect()
            } else {
                [128].into_iter().collect()
            };
            assert_eq!(ejections, expected);
        }
        let mut rng = Rng(1);
        for _ in 0..256 {
            assert_eq!(random_macro(&mut rng, 3, false, true).0 & (32 | 64 | 128), 0);
        }
    }

    #[test]
    fn mounted_dad_can_lean_or_drive_while_braking_without_changing_other_alphabets() {
        let mut dad_rng = Rng(19);
        let mut seen = std::collections::BTreeSet::new();
        for _ in 0..4096 {
            seen.insert(random_macro(&mut dad_rng, 3, false, false).0);
        }
        for mask in [17, 18, 20, 24] {
            assert!(seen.contains(&mask));
        }
        for character in [2, 4] {
            let (mut original, mut other) = (Rng(19), Rng(19));
            for _ in 0..256 {
                let a = random_macro(&mut original, 1, false, false);
                assert_eq!(a, random_macro(&mut other, character, false, false));
                assert!(![17, 18, 20, 24].contains(&a.0));
            }
        }
    }

    #[test]
    fn seed_finish_is_confirmed_before_death_pruning_but_respects_the_frame_ceiling() {
        let finish = Frame { frame: 299, alive_before: true, alive_after: false,
            finished: true, displacement: 0.25, ..Frame::default() };
        assert_eq!(seed_frame_action(&finish, None, 299), SeedAction::Confirm);
        assert_eq!(seed_frame_action(&finish, Some(20.0), 299), SeedAction::Stop);
        assert_eq!(seed_frame_action(&Frame { frame: 300, ..finish }, None, 299), SeedAction::Stop);
        assert_eq!(seed_frame_action(&Frame { port_abort: true, ..finish }, None, 299), SeedAction::Stop);
        let unfinished = Frame { frame: 298, alive_after: true, ..Frame::default() };
        assert_eq!(seed_frame_action(&unfinished, None, 299), SeedAction::Archive);
        assert_eq!(seed_frame_action(&Frame { frame: 299, ..unfinished }, None, 299), SeedAction::Stop);
    }

    #[test]
    fn dad_archive_distinguishes_damaged_vehicle_child_and_held_body_topology() {
        // Session construction needs a large stack; no physics step is needed to check keys.
        std::thread::Builder::new().stack_size(16 << 20).spawn(|| {
            use hw_core::b2::{state::JointDef, world};
            use hw_core::game::{bicycle, character, child};
            let mut s = Sim::new(45, 3).unwrap();
            let f = Frame::default();
            let original = key_of(&s.ctx, &f);
            s.ctx.character.primary.current_pose = 1;
            assert_ne!(original, key_of(&s.ctx, &f));
            s.ctx.character.primary.current_pose = 0;
            for joint in [s.ctx.character.primary.joints[character::ELBOW1],
                s.ctx.character.primary.bicycle.joints[bicycle::HAND1],
                s.ctx.character.child.joints[character::ELBOW1],
                s.ctx.character.child.child_seat.joints[child::SEAT_FRAME]] {
                let before = key_of(&s.ctx, &f);
                assert!(s.ctx.world.joints.is_alive(joint));
                assert!(matches!(world::destroy_joint(&mut s.ctx, joint), hw_core::Flow::Continue));
                assert_ne!(before, key_of(&s.ctx, &f));
            }
            let before = key_of(&s.ctx, &f);
            s.ctx.character.child.child_seat.ejected = hw_core::Flag32::TRUE;
            assert_ne!(before, key_of(&s.ctx, &f));
            let before = key_of(&s.ctx, &f);
            s.ctx.character.child.child_seat.detached = hw_core::Flag32::TRUE;
            assert_ne!(before, key_of(&s.ctx, &f));
            let mut def = JointDef::revolute();
            def.body1 = s.ctx.character.primary.bicycle.bodies[bicycle::FRAME];
            def.body2 = s.ctx.character.primary.bodies[character::LOWER_ARM1];
            let (flow, grip) = world::create_joint(&mut s.ctx, &def);
            assert!(matches!(flow, hw_core::Flow::Continue));
            s.ctx.character.primary.grip_joints[0] = grip;
            let before = key_of(&s.ctx, &f);
            s.ctx.world.joint_mut(grip).body1 = s.ctx.character.primary.bicycle.bodies[bicycle::FRONT];
            assert_ne!(before, key_of(&s.ctx, &f));
            s.ctx.character.primary.kind = 2;
            let legacy = key_of(&s.ctx, &f);
            s.ctx.character.primary.current_pose = 4;
            s.ctx.character.child.child_seat.detached = hw_core::Flag32::FALSE;
            assert_eq!(legacy, key_of(&s.ctx, &f));
        }).unwrap().join().unwrap();
    }
}
