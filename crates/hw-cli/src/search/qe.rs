//! `hw-search qe`: high-throughput launch sampling around known launch lineages, scored by
//! where the chest's launch chain actually goes.
//!
//! Why (measured on GBE, see docs/SEARCH.md): a finish needs the chest to be inside the finish
//! region when a FinishLine check fires. End-of-frame positions are TOI artefacts (the chest
//! "strands" on a manifold-0 TOI), so the old miss distance misleads. What matters is how far
//! the chest's *solved* chain gets along the only viable route, top border -> right wall ->
//! into the shaft -> down to the plate, and whether any finish check (for example a QE limb
//! touching the finish box, "mechanism C") reads the chest near the region.
//!
//! Throughput: a worker replays a lineage once to `K` frames before its launch, then runs a
//! batch of trials from that snapshot. Each trial changes inputs within those last frames
//! (grab held + one of the 7 post-eject poses, occasionally a release) and plays until the
//! first launch. Launch outcomes are deduplicated globally; good distinct launches become new
//! lineages, so the search spreads through launch space instead of re-finding the same launch.
use super::analyze::shape_label;
use super::events::{aspect_cost, edges, pose};
use super::explore::confirm;
use super::finish_policy::is_better;
use super::pool_policy::{insert_tracked, prune_parent_ids, retain_repeated_launch, seed_index, trim, upsert, Member};
use super::repair_policy::Bank;
use super::sparse::{canonicalize, finish_of, fp_count};
use super::sim::{finish_region_distance, tas_text, Sim};
use super::window_policy::{eligible, root as window_root};
use hw_core::game::character::CHEST;
use hw_core::lab::{self, Event};
use hw_core::H;
use serde_json::json;
use std::collections::HashSet;
use std::io::Write;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

pub struct Config {
    pub level: u32,
    pub character: u32,
    /// Launch strings, each ending on its launch frame.
    pub seeds: Vec<Vec<u8>>,
    /// Known scores for seeds (same order; missing = unscored).
    pub seed_scores: Vec<f64>,
    /// Unprotected checkpoint parents: raw inputs, persisted score and visits.
    /// Exact duplicates merge into an existing parent without replacing its ID.
    pub resume_members: Vec<(Vec<u8>, f64, u64)>,
    /// Repair checkpoint rows with exact curated ancestor IDs and generations.
    pub repair_members: Vec<(u64, Vec<u8>, u32)>,
    pub run_dir: std::path::PathBuf,
    pub threads: usize,
    pub hours: f64,
    pub seed: u64,
    pub min_launch: f64,
    pub max_frame: u32,
    /// Trials change inputs within the last `window` frames before a lineage's launch.
    pub window: usize,
    /// Admit shorter nonempty parents and shorten their replay window as needed.
    pub adaptive_window: bool,
    /// Frames played past the lineage's launch frame looking for a launch.
    pub extra: usize,
    /// Trials per snapshot batch.
    pub batch: usize,
    pub pool: usize,
    /// Hard bounds (x0, x1, y0, y1) on the chest position entering the launch frame.
    pub bounds: Option<[f64; 4]>,
    /// Probability that a changed frame gets an arbitrary combination of bits 0-6 (grab usually
    /// kept) instead of one of the 14 grab x pose classes. Bit 7 is copied from the lineage.
    pub combo: f64,
    /// Probability that a trial simplifies the lineage (extend a hold over the next frames, or
    /// drop one key's press/release pulse) instead of changing inputs at random.
    pub simplify: f64,
    /// Fewest-frame-perfect mode: keep searching after finishes; measure both raw and
    /// simplified strings and rank by actual frame-perfect count, then finish frame.
    pub fp_mode: bool,
    /// In fp mode, launches with more than best_fp + ev_slack key events do not join the pool.
    pub ev_slack: usize,
    /// Pool weighting: factor exp(-events / ev_temp) (0 = off).
    pub ev_temp: f64,
    /// Probability per batch of a deep batch, replayed from the level start. Mix pulse/hold
    /// simplifications with all-key setup edge retiming and initial-run bit changes.
    pub deep: f64,
    /// fp mode: skip (before simulating) trials whose aspect cost exceeds best_fp + budget_rel.
    pub budget_rel: Option<i64>,
    /// fp mode: probability that a trial uses cost-1 edits (move a class boundary, change one
    /// class run to a neighbouring class, delete or insert a run) instead of a random re-roll.
    pub edits: f64,
    /// Pool weighting temperature on the route score: weight exp(-score / temp).
    pub temp: f64,
    /// Probability of uniform parent selection instead of fitness/visit weighting.
    pub uniform: f64,
    /// Probability of sampling an initial parent uniformly. Positive values also
    /// protect all initial parents from pool trimming, within the configured cap.
    pub seed_share: f64,
    /// Conditional probability of a repair-bank parent on nonreserved picks.
    pub repair_share: f64,
    /// FIFO capacity for full proposed input strings per exact initial ancestor.
    pub repair_per_seed: usize,
    /// Nondeep probability of an anchored all-key setup mutation. Zero preserves
    /// the ordinary mutation branch and its RNG sequence exactly.
    pub setup_share: f64,
    /// Nondeep probability of retiming complete mounted/ejected mask runs while
    /// preserving the entire later suffix. Selected before anchored setup;
    /// zero adds no RNG draws and retains the previous mutation sequence.
    pub bundle_share: f64,
    /// Retain different raw nonfinish launch parents even when their geometric
    /// outcome key was already seen. Finish measurement stays independent.
    pub retain_repeated_launches: bool,
}

/// Version of persisted pool fitness; legacy geometric scores require invalidation.
pub const SCORE_VERSION: u64 = 2;

/// Level-specific route geometry.
#[derive(Clone, Copy)]
pub struct Profile {
    /// Box (x0, x1, y0, y1) of the final approach: impacts in it after top -> right advance to
    /// stage 3 (GBE: the shaft; Satan: the finish pocket).
    pub approach: [f64; 4],
    /// Extra room to the left of `approach` when labelling hits 'S' (GBE funnel top).
    pub label_margin: f64,
    /// Reward depth below the approach box top (GBE shaft).
    pub depth: bool,
    /// Level shapes right of this x count as the right wall ('R'): the world border on Satan,
    /// the static wall right of the shaft on GBE (the finishing chain bounces off it at x 310).
    pub wall_x: f64,
}

pub fn profile(level: u32) -> Profile {
    match level {
        // Satan: pocket under the ceiling circle, finish region x 121.9..128.3, y 122.7..125.9.
        2433 => Profile { approach: [95.0, 150.0, 95.0, 135.0], label_margin: 0.0, depth: false, wall_x: 319.5 },
        _ => Profile { approach: SHAFT, label_margin: 20.0, depth: true, wall_x: 305.0 },
    }
}

/// Where a launch chain got. Lower `score` is better.
#[derive(Clone, Debug)]
pub struct Rel {
    pub score: f64,
    /// Route stage reached in order: 0 none, 1 top border, 2 then right wall, 3 then shaft.
    pub stage: u32,
    /// Deepest impact y inside the shaft (m, larger = deeper), 0 if none.
    pub depth: f64,
    /// Best finish check this frame: distance (m) of the chest from the finish region.
    pub fc: f64,
    pub speed: f64,
    pub route: String,
    pub stranded_on: String,
    /// Closest approach to the finish region (m) of any solved impact point or strand point
    /// reached after the top border -> right wall stages.
    pub near: f64,
    pub impact_near: f64,
    pub report_near: f64,
    pub strand_near: f64,
    /// A live FinishLine callback passed its strict rectangle check.
    pub finish_inside: bool,
}

impl Default for Rel {
    fn default() -> Self {
        Self {
            score: 400.0, stage: 0, depth: 0.0, fc: f64::INFINITY, speed: 0.0,
            route: String::new(), stranded_on: String::new(), near: f64::INFINITY,
            impact_near: f64::INFINITY, report_near: f64::INFINITY,
            strand_near: f64::INFINITY, finish_inside: false,
        }
    }
}

fn distance(ctx: &super::sim::State, p: [f64; 2]) -> f64 {
    // f64::max ignores a NaN operand; passing NaNs to the rectangle distance
    // could otherwise manufacture a zero-distance near-miss.
    if p[0].is_finite() && p[1].is_finite() {
        finish_region_distance(ctx, p[0], p[1])
    } else {
        f64::INFINITY
    }
}

fn fitness(r: &Rel, alive_after: bool, chest_alive: bool) -> f64 {
    if r.finish_inside {
        return 0.0;
    }
    let mut score = 400.0 - 100.0 * r.stage as f64 - r.depth.min(85.0);
    if r.stage >= 3 {
        // A strand has no Report callback; an impact c0 is not the Report c.
        // Retain those lineages for mutations, with less weight than live evidence.
        score = (score + 30.0).min(5.0 + r.report_near)
            .min(25.0 + r.impact_near).min(40.0 + r.strand_near);
    }
    score = score.min(1.0 + r.fc);
    if !alive_after || !chest_alive {
        score += 20.0;
    }
    score
}

/// GBE shaft between the static walls (x 277..300.8), from the funnel top down to the floor.
const SHAFT: [f64; 4] = [277.0, 300.8, 27.0, 112.0];

fn centroid(ctx: &super::sim::State, s: H) -> (f64, f64, bool) {
    let sh = ctx.world.shape(s);
    let b = sh.body;
    let xf = ctx.world.body(b).xf;
    let local = if sh.type_ == 0 { sh.local_position } else { sh.centroid };
    let x = xf.position.x + xf.r.col1.x * local.x + xf.r.col2.x * local.y;
    let y = xf.position.y + xf.r.col1.y * local.x + xf.r.col2.y * local.y;
    (x, y, b == ctx.level.level_body)
}

/// Score one launch frame from its lab events (chest watched).
pub fn score(ctx: &super::sim::State, events: &[Event], chest: H, prof: &Profile) -> Rel {
    let a = prof.approach;
    let inside = |p: [f64; 2]| p[0] >= a[0] && p[0] <= a[1] && p[1] >= a[2] && p[1] <= a[3];
    let mut r = Rel::default();
    let mut stranded = false;
    let mut route = String::new();
    for e in events {
        match *e {
            Event::Solve { body, v, .. } if body == chest => {
                r.speed = (v[0] * v[0] + v[1] * v[1]).sqrt();
            }
            Event::Toi { shape1, shape2, body1, body2, .. } if !stranded && (body1 == chest || body2 == chest) => {
                let other = if body1 == chest { shape2 } else { shape1 };
                if !ctx.world.shapes.is_alive(other) {
                    continue;
                }
                let (x, y, level) = centroid(ctx, other);
                let c = if level && y < -79.0 {
                    'T'
                } else if level && x > prof.wall_x {
                    'R'
                } else if x >= a[0] - prof.label_margin && x <= a[1] && y >= a[2] && y <= a[3] {
                    'S'
                } else {
                    'o'
                };
                if route.len() < 24 && !route.ends_with(c) {
                    route.push(c);
                }
                match (r.stage, c) {
                    (0, 'T') => r.stage = 1,
                    (1, 'R') => r.stage = 2,
                    (2, 'S') => r.stage = 3,
                    _ => {}
                }
            }
            Event::AfterToi { body, c0, c, .. } if !stranded && body == chest => {
                if r.stage >= 2 {
                    r.impact_near = r.impact_near.min(distance(ctx, c0));
                    r.report_near = r.report_near.min(distance(ctx, c));
                    r.near = r.near.min(r.impact_near);
                }
                if r.stage >= 2 && inside(c0) {
                    r.stage = r.stage.max(3);
                    if prof.depth {
                        r.depth = r.depth.max(c0[1] - a[2]);
                    }
                }
                if r.stage >= 2 && inside(c) {
                    r.stage = r.stage.max(3);
                    if prof.depth {
                        r.depth = r.depth.max(c[1] - a[2]);
                    }
                }
            }
            Event::Strand { body1, body2, shape1, shape2, pos1, pos2, .. } if !stranded && (body1 == chest || body2 == chest) => {
                stranded = true;
                let (other, p) = if body1 == chest { (shape2, pos1) } else { (shape1, pos2) };
                if ctx.world.shapes.is_alive(other) {
                    r.stranded_on = shape_label(ctx, other);
                }
                if r.stage >= 2 {
                    r.strand_near = r.strand_near.min(distance(ctx, p));
                    r.near = r.near.min(r.strand_near);
                }
                if r.stage >= 2 && inside(p) {
                    r.stage = r.stage.max(3);
                    if prof.depth {
                        r.depth = r.depth.max(p[1] - a[2]);
                    }
                }
            }
            Event::Finish { x, y, inside, dead: false, .. } => {
                r.fc = r.fc.min(distance(ctx, [x, y]));
                r.finish_inside |= inside;
            }
            _ => {}
        }
    }
    r.route = route;
    let current_chest = ctx.character.bodies[CHEST];
    r.score = fitness(&r, !ctx.level.dead.get(), current_chest != hw_core::NONE && ctx.world.bodies.is_alive(current_chest));
    r
}

struct Shared {
    pool: Mutex<Vec<Member>>,
    /// Only newly inserted, retained repeated-launch parents receive these tags.
    /// Any nested acquisition uses pool -> repeated_parent_ids lock order.
    repeated_parent_ids: Mutex<HashSet<u64>>,
    repeated_nonfinish_launch_hits: AtomicU64,
    repeated_parent_rejected_window: AtomicU64,
    repeated_parent_rejected_cost: AtomicU64,
    repeated_parent_exact_merged: AtomicU64,
    repeated_parent_new_inserted: AtomicU64,
    repeated_parent_new_retained: AtomicU64,
    repeated_parent_new_trimmed: AtomicU64,
    repeated_parent_picks: AtomicU64,
    repeated_log: Mutex<Option<std::fs::File>>,
    repair: Mutex<Bank>,
    repair_picks: AtomicU64,
    repair_trials: AtomicU64,
    repair_admitted: [AtomicU64; 3],
    repair_rejected: [AtomicU64; 4],
    repair_duplicates: AtomicU64,
    repair_evicted: AtomicU64,
    repair_generation_max: AtomicU64,
    repair_launches: AtomicU64,
    repair_finishes: AtomicU64,
    repair_anchor_failures: AtomicU64,
    repair_early_death_admissions: AtomicU64,
    seed_id_end: u64,
    seed_count: usize,
    resume_id_end: u64,
    repair_id_end: u64,
    repair_restored_parent_picks: AtomicU64,
    repair_restored_descendant_admissions: AtomicU64,
    resume_parent_picks: AtomicU64,
    resume_parent_launches: AtomicU64,
    short_parent_picks: AtomicU64,
    short_parent_trials: AtomicU64,
    reservoir_picks: AtomicU64,
    initial_parent_picks: AtomicU64,
    initial_parent_launches: AtomicU64,
    next_id: AtomicU64,
    seen: Mutex<HashSet<(u64, u64)>>,
    /// Hashes of input strings already simulated (identical inputs give identical launches).
    tried: Mutex<HashSet<u64>>,
    /// Exact consumed TAS strings already measured, independent of launch geometry.
    measured: Mutex<HashSet<Vec<u8>>>,
    repeats: AtomicU64,
    over_budget: AtomicU64,
    stop: AtomicBool,
    frames: AtomicU64,
    trials: AtomicU64,
    deep_trials: AtomicU64,
    setup_trials: AtomicU64,
    anchored_setup_picks: AtomicU64,
    anchored_setup_proposals: AtomicU64,
    anchored_setup_replayed: AtomicU64,
    anchored_setup_z_changes: [AtomicU64; 5],
    anchored_setup_raw_launch_attempts: AtomicU64,
    anchored_setup_raw_finish_attempts: AtomicU64,
    bundle_picks: AtomicU64,
    bundle_proposals: AtomicU64,
    bundle_replayed: AtomicU64,
    bundle_z_changes: [AtomicU64; 5],
    bundle_suffix_shifts: [AtomicU64; 3],
    bundle_raw_launch_attempts: AtomicU64,
    bundle_raw_finish_attempts: AtomicU64,
    launches: AtomicU64,
    distinct: AtomicU64,
    overflow: AtomicU64,
    oob: AtomicU64,
    profile: Profile,
    stage_counts: [AtomicU64; 4],
    best: Mutex<(f64, f64)>,
    log: Mutex<std::fs::File>,
    winner: Mutex<Option<Vec<u8>>>,
    /// fp mode: best (structural cost, measured frame-perfect count, finish frame).
    best_fp: Mutex<(usize, usize, u32)>,
    finishes: AtomicU64,
    fp_log: Mutex<Option<std::fs::File>>,
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

// Post-eject input classes (motorcart::input_bits): space held + pose; release = no space.
const HELD: [u8; 7] = [16, 16 | 1, 16 | 2, 16 | 4, 16 | 8, 16 | 32, 16 | 64];
const RELEASED: [u8; 7] = [0, 1, 2, 4, 8, 32, 64];

/// A replacement input for a frame whose lineage input was `orig`: one of the 14 grab x pose
/// classes, or (with probability `cfg.combo`) any combination of bits 0-6 with grab kept 90% of
/// the time. Combinations matter because the pose setters run in sequence within one frame
/// (Satan's launches use them); bit 7 (eject) is kept from the lineage.
fn choose(cfg: &Config, rng: &mut Rng, orig: u8) -> u8 {
    let c = if rng.unit() < cfg.combo {
        let m = rng.below(128) as u8 & !16;
        if rng.below(10) == 0 { m } else { m | 16 }
    } else if rng.below(20) == 0 {
        RELEASED[rng.below(7) as usize]
    } else {
        HELD[rng.below(7) as usize]
    };
    c | (orig & 128)
}

fn validate_selection(uniform: f64, seed_share: f64, pool_cap: usize, seeds: usize) -> Result<(), String> {
    if !uniform.is_finite() || !(0.0..=1.0).contains(&uniform) {
        return Err("--uniform must be finite and between 0 and 1".into());
    }
    if !seed_share.is_finite() || !(0.0..=1.0).contains(&seed_share) {
        return Err("--seed-share must be finite and between 0 and 1".into());
    }
    if seed_share > 0.0 && pool_cap < seeds {
        return Err("--pool must hold every unique usable initial parent with --seed-share enabled".into());
    }
    Ok(())
}

fn validate_repair(share: f64, per_seed: usize, seed_share: f64, deep: f64) -> Result<(), String> {
    if !share.is_finite() || !(0.0..=1.0).contains(&share) {
        return Err("--repair-share must be finite and between 0 and 1".into());
    }
    if share > 0.0 && (per_seed == 0 || seed_share <= 0.0 || deep != 0.0) {
        return Err("repair requires --repair-per-seed > 0, --seed-share > 0 and --deep 0".into());
    }
    Ok(())
}

fn validate_window(window: usize) -> Result<(), String> {
    if window == 0 {
        return Err("--window must be positive".into());
    }
    Ok(())
}

fn validate_setup(share: f64) -> Result<(), String> {
    if !share.is_finite() || !(0.0..=1.0).contains(&share) {
        return Err("--setup-share must be finite and between 0 and 1".into());
    }
    Ok(())
}

fn choose_anchored(share: f64, deep: bool, rng: &mut Rng) -> bool {
    !deep && share > 0.0 && rng.unit() < share
}

fn validate_bundle(share: f64) -> Result<(), String> {
    if !share.is_finite() || !(0.0..=1.0).contains(&share) {
        return Err("--bundle-share must be finite and between 0 and 1".into());
    }
    Ok(())
}

/// With bundle disabled this makes exactly the former anchored selection draw.
/// A bundle pick skips anchored selection even if its later proposal is invalid.
fn choose_setup(bundle_share: f64, setup_share: f64, deep: bool, rng: &mut Rng) -> (bool, bool) {
    let bundle = choose_anchored(bundle_share, deep, rng);
    let anchored = !bundle && choose_anchored(setup_share, deep, rng);
    (bundle, anchored)
}

/// First raw z entry coverage only; this makes no state-equivalence claim.
fn z_change(before: &[u8], after: &[u8]) -> usize {
    let first = |masks: &[u8]| masks.iter().position(|mask| mask & 128 != 0);
    match (first(before), first(after)) {
        (Some(a), Some(b)) if b < a => 0,
        (Some(a), Some(b)) if b > a => 1,
        (Some(_), None) => 2,
        (None, Some(_)) => 3,
        _ => 4,
    }
}

fn restore_repair(mut bank: Bank, rows: &[(u64, Vec<u8>, u32)], first_id: u64,
    enabled: bool) -> Result<(Bank, u64), String> {
    if !enabled && !rows.is_empty() {
        return Err("--repair-pool-file requires enabled --repair-share".into());
    }
    let next_id = first_id.checked_add(rows.len() as u64)
        .ok_or_else(|| "repair checkpoint IDs exceed u64 capacity".to_string())?;
    for (i, (ancestor, masks, generation)) in rows.iter().enumerate() {
        bank.restore_entry(*ancestor, first_id + i as u64, masks, *generation)
            .map_err(|error| format!("repair checkpoint row {}: {error}", i + 1))?;
    }
    Ok((bank, next_id))
}

fn curated_members(seeds: &[Vec<u8>], scores: &[f64], window: usize, adaptive: bool) -> Vec<Member> {
    let mut pool = Vec::new();
    for (i, masks) in seeds.iter().enumerate().filter(|(_, masks)| eligible(masks.len(), window, adaptive)) {
        upsert(&mut pool, Member { masks: masks.clone(), score: scores.get(i).copied().unwrap_or(400.0),
            visits: 0, id: i as u64, events: aspect_cost(masks) });
    }
    pool
}

/// Count eligible rows and new unprotected raw parents separately: imports that
/// merge into a curated ID refresh metadata but never enlarge the resume set.
fn append_resume(pool: &mut Vec<Member>, rows: &[(Vec<u8>, f64, u64)], seed_id_end: u64,
    window: usize, adaptive: bool) -> (usize, usize) {
    let mut eligible_rows = 0;
    let mut added = 0;
    for (i, (masks, score, visits)) in rows.iter().enumerate() {
        if !eligible(masks.len(), window, adaptive) {
            continue;
        }
        eligible_rows += 1;
        added += usize::from(upsert(pool, Member { masks: masks.clone(), score: *score,
            visits: *visits, id: seed_id_end + i as u64, events: aspect_cost(masks) }));
    }
    (eligible_rows, added)
}

/// Sample only available input entries. Fixed-window parents retain exactly the
/// former modulus and RNG draw; shorter adaptive parents cannot index past their end.
fn mutation_start(rng: &mut Rng, len: usize, root: usize) -> Option<usize> {
    let available = len.checked_sub(root)?;
    if available == 0 {
        return None;
    }
    Some(root + rng.below(available as u64) as usize)
}

fn mutation_end(len: usize, window: usize, adaptive: bool, deep: bool) -> usize {
    // Legacy deep rerolls started at zero but sampled only the configured
    // window. Adaptive deep rerolls may sample the entire available input.
    if deep && !adaptive { window } else { len }
}

fn increment_visits(visits: &mut u64) {
    *visits = visits.saturating_add(1);
}

pub fn run(cfg: Config) -> Result<Vec<u8>, String> {
    validate_window(cfg.window)?;
    validate_setup(cfg.setup_share)?;
    validate_bundle(cfg.bundle_share)?;
    validate_selection(cfg.uniform, cfg.seed_share, cfg.pool, 0)?;
    validate_repair(cfg.repair_share, cfg.repair_per_seed, cfg.seed_share, cfg.deep)?;
    let seed_id_end = cfg.seeds.len() as u64;
    let resume_id_end = seed_id_end + cfg.resume_members.len() as u64;
    let mut pool = curated_members(&cfg.seeds, &cfg.seed_scores, cfg.window, cfg.adaptive_window);
    let seed_count = pool.len();
    validate_selection(cfg.uniform, cfg.seed_share, cfg.pool, seed_count)?;
    if cfg.seed_share > 0.0 && seed_count == 0 {
        return Err("--seed-share requires at least one eligible curated parent".into());
    }
    let (resume_eligible_rows, resume_added) = append_resume(&mut pool, &cfg.resume_members,
        seed_id_end, cfg.window, cfg.adaptive_window);
    if pool.is_empty() {
        return Err("no usable seed launches".into());
    }
    // Only opting into resume loading adds startup trimming. The no-resume
    // initialization, including initial order and selection RNG, stays unchanged.
    if !cfg.resume_members.is_empty() {
        trim(&mut pool, cfg.pool, (cfg.seed_share > 0.0).then_some(seed_id_end), cfg.fp_mode && cfg.ev_temp > 0.0);
        if pool.is_empty() {
            return Err("resume loading left no parents within --pool capacity".into());
        }
    }
    let bank = if cfg.repair_share > 0.0 {
        Bank::new(pool.iter().filter(|m| m.id < seed_id_end).filter_map(|m|
            window_root(m.masks.len(), cfg.window, cfg.adaptive_window)
                .map(|root| (m.id, m.masks.clone(), root))).collect(), cfg.repair_per_seed)
    } else {
        Bank::new(Vec::new(), 0)
    };
    let (bank, next_id) = restore_repair(bank, &cfg.repair_members, resume_id_end, cfg.repair_share > 0.0)?;
    let restored_entries = bank.len();
    let restored_families = bank.populated_families();
    let restored_generation_max = bank.max_generation();
    std::fs::create_dir_all(&cfg.run_dir).map_err(|e| e.to_string())?;
    let template = Sim::new(cfg.level, cfg.character)?;
    eprintln!("qe: {} seed lineages", pool.len());
    let mut best_fp = (usize::MAX, usize::MAX, u32::MAX);
    if cfg.fp_mode {
        // The first seed that finishes sets the measured frame-perfect count to beat.
        // --seeds come after --pool-file entries; check the explicit ones first.
        for sd in cfg.seeds.iter().rev() {
            let raw_finish = finish_of(cfg.level, cfg.character, sd, cfg.max_frame);
            if raw_finish == 0 {
                continue;
            }
            if let Some((fp, ev)) = fp_count(cfg.level, cfg.character, sd, cfg.max_frame, cfg.threads) {
                best_fp = (aspect_cost(sd), fp, raw_finish);
                eprintln!("qe: fp mode, seed raw cost {} ({fp} frame-perfect, {ev} events), finish {raw_finish}", best_fp.0);
            }
            let c = canonicalize(cfg.level, cfg.character, sd, cfg.max_frame);
            if c != *sd {
                let f = finish_of(cfg.level, cfg.character, &c, cfg.max_frame);
                if f != 0 {
                    if let Some((fp, ev)) = fp_count(cfg.level, cfg.character, &c, cfg.max_frame, cfg.threads) {
                        let candidate = (aspect_cost(&c), fp, f);
                        if is_better(candidate, best_fp) {
                            best_fp = candidate;
                            eprintln!("qe: fp mode, seed canonical cost {} ({fp} frame-perfect, {ev} events), finish {f}", best_fp.0);
                        }
                    }
                }
            }
            if best_fp.1 != usize::MAX {
                break;
            }
        }
    }
    let shared = Shared {
        repeated_parent_ids: Mutex::new(HashSet::new()),
        repeated_nonfinish_launch_hits: AtomicU64::new(0),
        repeated_parent_rejected_window: AtomicU64::new(0),
        repeated_parent_rejected_cost: AtomicU64::new(0),
        repeated_parent_exact_merged: AtomicU64::new(0),
        repeated_parent_new_inserted: AtomicU64::new(0),
        repeated_parent_new_retained: AtomicU64::new(0),
        repeated_parent_new_trimmed: AtomicU64::new(0),
        repeated_parent_picks: AtomicU64::new(0),
        repeated_log: Mutex::new(if cfg.retain_repeated_launches {
            Some(std::fs::OpenOptions::new().create(true).append(true)
                .open(cfg.run_dir.join("repeated-parents.jsonl")).map_err(|e| e.to_string())?)
        } else { None }),
        repair: Mutex::new(bank),
        repair_picks: AtomicU64::new(0),
        repair_trials: AtomicU64::new(0),
        repair_admitted: Default::default(),
        repair_rejected: Default::default(),
        repair_duplicates: AtomicU64::new(0),
        repair_evicted: AtomicU64::new(0),
        repair_generation_max: AtomicU64::new(restored_generation_max as u64),
        repair_launches: AtomicU64::new(0),
        repair_finishes: AtomicU64::new(0),
        repair_anchor_failures: AtomicU64::new(0),
        repair_early_death_admissions: AtomicU64::new(0),
        seed_id_end,
        seed_count,
        resume_id_end,
        repair_id_end: next_id,
        repair_restored_parent_picks: AtomicU64::new(0),
        repair_restored_descendant_admissions: AtomicU64::new(0),
        resume_parent_picks: AtomicU64::new(0),
        resume_parent_launches: AtomicU64::new(0),
        short_parent_picks: AtomicU64::new(0),
        short_parent_trials: AtomicU64::new(0),
        reservoir_picks: AtomicU64::new(0),
        initial_parent_picks: AtomicU64::new(0),
        initial_parent_launches: AtomicU64::new(0),
        next_id: AtomicU64::new(next_id),
        pool: Mutex::new(pool),
        seen: Mutex::new(HashSet::new()),
        tried: Mutex::new(HashSet::new()),
        measured: Mutex::new(HashSet::new()),
        repeats: AtomicU64::new(0),
        over_budget: AtomicU64::new(0),
        stop: AtomicBool::new(false),
        frames: AtomicU64::new(0),
        trials: AtomicU64::new(0),
        deep_trials: AtomicU64::new(0),
        setup_trials: AtomicU64::new(0),
        anchored_setup_picks: AtomicU64::new(0),
        anchored_setup_proposals: AtomicU64::new(0),
        anchored_setup_replayed: AtomicU64::new(0),
        anchored_setup_z_changes: Default::default(),
        anchored_setup_raw_launch_attempts: AtomicU64::new(0),
        anchored_setup_raw_finish_attempts: AtomicU64::new(0),
        bundle_picks: AtomicU64::new(0),
        bundle_proposals: AtomicU64::new(0),
        bundle_replayed: AtomicU64::new(0),
        bundle_z_changes: Default::default(),
        bundle_suffix_shifts: Default::default(),
        bundle_raw_launch_attempts: AtomicU64::new(0),
        bundle_raw_finish_attempts: AtomicU64::new(0),
        launches: AtomicU64::new(0),
        distinct: AtomicU64::new(0),
        overflow: AtomicU64::new(0),
        oob: AtomicU64::new(0),
        profile: profile(cfg.level),
        stage_counts: Default::default(),
        best: Mutex::new((f64::INFINITY, f64::INFINITY)),
        log: Mutex::new(
            std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("launches.jsonl")).map_err(|e| e.to_string())?,
        ),
        winner: Mutex::new(None),
        best_fp: Mutex::new(best_fp),
        finishes: AtomicU64::new(0),
        fp_log: Mutex::new(if cfg.fp_mode {
            std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("finishes.jsonl")).ok()
        } else {
            None
        }),
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
            let secs = start.elapsed().as_secs_f64();
            let (best_score, best_fc) = *shared.best.lock().unwrap();
            let mut line = json!({
                "t": secs as u64,
                "fps": shared.frames.load(Ordering::Relaxed) as f64 / secs,
                "trials": shared.trials.load(Ordering::Relaxed),
                "deep_trials": shared.deep_trials.load(Ordering::Relaxed),
                "setup_trials": shared.setup_trials.load(Ordering::Relaxed),
                "launches": shared.launches.load(Ordering::Relaxed),
                "distinct": shared.distinct.load(Ordering::Relaxed),
                "distinct_per_s": shared.distinct.load(Ordering::Relaxed) as f64 / secs,
                "overflow": shared.overflow.load(Ordering::Relaxed),
                "out_of_bounds": shared.oob.load(Ordering::Relaxed),
                "stages_none_top_topright_shaft": shared.stage_counts.iter().map(|c| c.load(Ordering::Relaxed)).collect::<Vec<_>>(),
                "pool": shared.pool.lock().unwrap().len(),
                "uniform": cfg.uniform,
                "seed_share": cfg.seed_share,
                "initial_parents": shared.seed_count,
                "retained_initial_parents": shared.pool.lock().unwrap().iter().filter(|m| m.id < shared.seed_id_end).count(),
                "reservoir_picks": shared.reservoir_picks.load(Ordering::Relaxed),
                "initial_parent_picks": shared.initial_parent_picks.load(Ordering::Relaxed),
                "initial_parent_launches": shared.initial_parent_launches.load(Ordering::Relaxed),
                "best_score": best_score,
                "best_fc_m": best_fc,
                "winner": shared.winner.lock().unwrap().is_some(),
                "finishes": shared.finishes.load(Ordering::Relaxed),
                "repeat_inputs_skipped": shared.repeats.load(Ordering::Relaxed),
                "over_budget_skipped": shared.over_budget.load(Ordering::Relaxed),
                "best_cost_fp_finish": *shared.best_fp.lock().unwrap(),
                "pool_min_events": shared.pool.lock().unwrap().iter().map(|m| m.events).min(),
            });
            let repair_fields = json!({
                "repair_share": cfg.repair_share, "repair_per_seed": cfg.repair_per_seed,
                "repair_entries": shared.repair.lock().unwrap().len(),
                "repair_families": shared.repair.lock().unwrap().populated_families(),
                "repair_picks": shared.repair_picks.load(Ordering::Relaxed),
                "repair_trials": shared.repair_trials.load(Ordering::Relaxed),
                "repair_admitted_death_lag_horizon": shared.repair_admitted.iter().map(|n| n.load(Ordering::Relaxed)).collect::<Vec<_>>(),
                "repair_rejected_abort_overflow_bounds_shape": shared.repair_rejected.iter().map(|n| n.load(Ordering::Relaxed)).collect::<Vec<_>>(),
                "repair_duplicates": shared.repair_duplicates.load(Ordering::Relaxed),
                "repair_evicted": shared.repair_evicted.load(Ordering::Relaxed),
                "repair_generation_max": shared.repair_generation_max.load(Ordering::Relaxed),
                "repair_retained_generation_max": shared.repair.lock().unwrap().max_generation(),
                "repair_launches": shared.repair_launches.load(Ordering::Relaxed),
                "repair_finishes": shared.repair_finishes.load(Ordering::Relaxed),
                "repair_anchor_failures": shared.repair_anchor_failures.load(Ordering::Relaxed),
                "repair_early_death_admissions": shared.repair_early_death_admissions.load(Ordering::Relaxed),
            });
            if let (Some(stats), serde_json::Value::Object(fields)) = (line.as_object_mut(), repair_fields) {
                stats.extend(fields);
            }
            let window_fields = json!({
                "adaptive_window": cfg.adaptive_window,
                "resume_rows": cfg.resume_members.len(),
                "resume_eligible_rows": resume_eligible_rows,
                "resume_added": resume_added,
                "retained_resume_parents": shared.pool.lock().unwrap().iter()
                    .filter(|m| m.id >= shared.seed_id_end && m.id < shared.resume_id_end).count(),
                "resume_parent_picks": shared.resume_parent_picks.load(Ordering::Relaxed),
                "resume_parent_launches": shared.resume_parent_launches.load(Ordering::Relaxed),
                "short_parent_picks": shared.short_parent_picks.load(Ordering::Relaxed),
                "short_parent_trials": shared.short_parent_trials.load(Ordering::Relaxed),
                "short_parents": shared.pool.lock().unwrap().iter()
                    .filter(|m| cfg.adaptive_window && !eligible(m.masks.len(), cfg.window, false)).count(),
                "zero_root_parents": shared.pool.lock().unwrap().iter()
                    .filter(|m| window_root(m.masks.len(), cfg.window, cfg.adaptive_window) == Some(0)).count(),
            });
            if let (Some(stats), serde_json::Value::Object(fields)) = (line.as_object_mut(), window_fields) {
                stats.extend(fields);
            }
            let setup_fields = json!({
                "setup_share": cfg.setup_share,
                "anchored_setup_picks": shared.anchored_setup_picks.load(Ordering::Relaxed),
                "anchored_setup_proposals": shared.anchored_setup_proposals.load(Ordering::Relaxed),
                "anchored_setup_replayed": shared.anchored_setup_replayed.load(Ordering::Relaxed),
                "anchored_setup_z_changes_earlier_later_removed_added_unchanged": shared.anchored_setup_z_changes
                    .iter().map(|count| count.load(Ordering::Relaxed)).collect::<Vec<_>>(),
                "anchored_setup_raw_launch_attempts": shared.anchored_setup_raw_launch_attempts.load(Ordering::Relaxed),
                "anchored_setup_raw_finish_attempts": shared.anchored_setup_raw_finish_attempts.load(Ordering::Relaxed),
                "bundle_share": cfg.bundle_share,
                "bundle_picks": shared.bundle_picks.load(Ordering::Relaxed),
                "bundle_proposals": shared.bundle_proposals.load(Ordering::Relaxed),
                "bundle_replayed": shared.bundle_replayed.load(Ordering::Relaxed),
                "bundle_z_changes_earlier_later_removed_added_unchanged": shared.bundle_z_changes
                    .each_ref().map(|counter| counter.load(Ordering::Relaxed)),
                "bundle_suffix_shifts_earlier_same_later": shared.bundle_suffix_shifts
                    .each_ref().map(|counter| counter.load(Ordering::Relaxed)),
                "bundle_raw_launch_attempts": shared.bundle_raw_launch_attempts.load(Ordering::Relaxed),
                "bundle_raw_finish_attempts": shared.bundle_raw_finish_attempts.load(Ordering::Relaxed),
                "repair_restored_rows": cfg.repair_members.len(),
                "repair_restored_entries": restored_entries,
                "repair_restored_families": restored_families,
                "repair_restored_generation_max": restored_generation_max,
                "repair_restored_parent_picks": shared.repair_restored_parent_picks.load(Ordering::Relaxed),
                "repair_restored_descendant_admissions": shared.repair_restored_descendant_admissions.load(Ordering::Relaxed),
            });
            if let (Some(stats), serde_json::Value::Object(fields)) = (line.as_object_mut(), setup_fields) {
                stats.extend(fields);
            }
            let retained_repeated_parents = if cfg.retain_repeated_launches {
                let pool = shared.pool.lock().unwrap();
                let mut ids = shared.repeated_parent_ids.lock().unwrap();
                prune_parent_ids(&pool, &mut ids)
            } else { 0 };
            let repeated_fields = json!({
                "retain_repeated_launches": cfg.retain_repeated_launches,
                "repeated_nonfinish_launch_hits": shared.repeated_nonfinish_launch_hits.load(Ordering::Relaxed),
                "repeated_parent_rejected_window": shared.repeated_parent_rejected_window.load(Ordering::Relaxed),
                "repeated_parent_rejected_cost": shared.repeated_parent_rejected_cost.load(Ordering::Relaxed),
                "repeated_parent_exact_merged": shared.repeated_parent_exact_merged.load(Ordering::Relaxed),
                "repeated_parent_new_inserted": shared.repeated_parent_new_inserted.load(Ordering::Relaxed),
                "repeated_parent_new_retained": shared.repeated_parent_new_retained.load(Ordering::Relaxed),
                "repeated_parent_new_trimmed": shared.repeated_parent_new_trimmed.load(Ordering::Relaxed),
                "retained_repeated_parents": retained_repeated_parents,
                "repeated_parent_picks": shared.repeated_parent_picks.load(Ordering::Relaxed),
            });
            if let (Some(stats), serde_json::Value::Object(fields)) = (line.as_object_mut(), repeated_fields) {
                stats.extend(fields);
            }
            writeln!(stats, "{line}").ok();
            eprintln!("{line}");
        }
    });
    if let Ok(mut out) = std::fs::File::create(cfg.run_dir.join("pool.jsonl")) {
        for m in shared.pool.lock().unwrap().iter() {
            writeln!(out, "{}", json!({"id": m.id, "score": m.score, "score_version": SCORE_VERSION, "frame": m.masks.len(), "visits": m.visits, "tas": tas_text(&m.masks)})).ok();
        }
    }
    if cfg.repair_share > 0.0 {
        if let Ok(mut out) = std::fs::File::create(cfg.run_dir.join("repair.jsonl")) {
            for entry in shared.repair.lock().unwrap().entries() {
                writeln!(out, "{}", json!({"ancestor": entry.ancestor, "id": entry.id,
                    "generation": entry.generation, "frame": entry.masks.len(), "tas": tas_text(&entry.masks)})).ok();
            }
        }
    }
    let w = shared.winner.lock().unwrap();
    w.clone().ok_or_else(|| "qe search ended without a confirmed winner".into())
}

struct Parent {
    member: Member,
    /// Exact original initial ID and the number of chained failed edits.
    repair_origin: Option<(u64, u32)>,
}

fn note_parent_pick(cfg: &Config, shared: &Shared, id: u64, len: usize) {
    // pick holds the pool lock before consulting these tags. Repair IDs and
    // existing exact duplicates are never tagged by repeated-launch insertion.
    if cfg.retain_repeated_launches && shared.repeated_parent_ids.lock().unwrap().contains(&id) {
        shared.repeated_parent_picks.fetch_add(1, Ordering::Relaxed);
    }
    if id >= shared.seed_id_end && id < shared.resume_id_end {
        shared.resume_parent_picks.fetch_add(1, Ordering::Relaxed);
    }
    if id >= shared.resume_id_end && id < shared.repair_id_end {
        shared.repair_restored_parent_picks.fetch_add(1, Ordering::Relaxed);
    }
    if cfg.adaptive_window && !eligible(len, cfg.window, false) {
        shared.short_parent_picks.fetch_add(1, Ordering::Relaxed);
    }
}

fn pick(cfg: &Config, shared: &Shared, rng: &mut Rng) -> Parent {
    let mut p = shared.pool.lock().unwrap();
    let n = p.len();
    // A zero share preserves the existing proposal RNG sequence. Initial parents
    // remain exact raw TAS strings; neither pose classes nor geometry identify them.
    let reserved = if cfg.seed_share > 0.0 && rng.unit() < cfg.seed_share {
        let index = seed_index(&p, shared.seed_id_end, rng.below(shared.seed_count as u64) as usize);
        if index.is_some() {
            shared.reservoir_picks.fetch_add(1, Ordering::Relaxed);
        }
        index
    } else {
        None
    };
    // Repair is conditional on an unreserved pick, so seed-share retains its
    // meaning. A disabled repair path consumes no additional random draw.
    if reserved.is_none() && cfg.repair_share > 0.0 && rng.unit() < cfg.repair_share {
        let bank = shared.repair.lock().unwrap();
        let families = bank.populated_families();
        if families != 0 {
            if let Some(ancestor) = bank.populated_ancestor(rng.below(families as u64) as usize) {
                if let Some(entry) = bank.pick(ancestor, rng.below(bank.count(ancestor) as u64) as usize) {
                    shared.repair_picks.fetch_add(1, Ordering::Relaxed);
                    note_parent_pick(cfg, shared, entry.id, entry.masks.len());
                    return Parent { member: Member { events: aspect_cost(&entry.masks), masks: entry.masks,
                        score: 400.0, visits: 0, id: entry.id }, repair_origin: Some((ancestor, entry.generation)) };
                }
            }
        }
    }
    let i = if let Some(index) = reserved {
        index
    } else if rng.unit() < cfg.uniform {
        rng.below(n as u64) as usize
    } else {
        // Weight by score (route progress / finish-check distance) and novelty.
        let w: Vec<f64> = p
            .iter()
            .map(|m| {
                let ev = if cfg.ev_temp > 0.0 { (-(m.events as f64) / cfg.ev_temp).exp() } else { 1.0 };
                (-(m.score) / cfg.temp).exp() * ev / (1.0 + m.visits as f64).sqrt()
            })
            .collect();
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
    if p[i].id < shared.seed_id_end {
        shared.initial_parent_picks.fetch_add(1, Ordering::Relaxed);
    }
    note_parent_pick(cfg, shared, p[i].id, p[i].masks.len());
    increment_visits(&mut p[i].visits);
    Parent { member: p[i].clone(), repair_origin: (p[i].id < shared.seed_id_end).then_some((p[i].id, 0)) }
}

fn worker(w: u64, cfg: &Config, template: &Sim, shared: &Shared) {
    let mut rng = Rng(cfg.seed ^ (w + 1).wrapping_mul(0x5851_f42d_4c95_7f2d));
    let mut base = template.fork();
    let mut work = template.fork();
    while !shared.stop.load(Ordering::Relaxed) {
        let selected = pick(cfg, shared, &mut rng);
        let m = selected.member;
        let l = m.masks.len();
        if rng.unit() < cfg.deep {
            for _ in 0..8 {
                if shared.stop.load(Ordering::Relaxed) {
                    break;
                }
                trial(cfg, shared, &mut rng, template, &mut work, &m, true, selected.repair_origin);
            }
            continue;
        }
        let Some(root) = window_root(l, cfg.window, cfg.adaptive_window) else { continue };
        // Replay the lineage once to its window start.
        base.restore(template);
        base.masks.clear();
        base.masks.extend_from_slice(&m.masks[..root]);
        let mut ok = true;
        for _ in 0..root {
            let f = base.step(0);
            if f.port_abort || !f.alive_after {
                ok = false;
                break;
            }
        }
        shared.frames.fetch_add(root as u64, Ordering::Relaxed);
        if !ok {
            if cfg.repair_share > 0.0 && selected.repair_origin.is_some() {
                shared.repair_anchor_failures.fetch_add(1, Ordering::Relaxed);
            }
            continue;
        }
        for _ in 0..cfg.batch {
            if shared.stop.load(Ordering::Relaxed) {
                break;
            }
            trial(cfg, shared, &mut rng, &base, &mut work, &m, false, selected.repair_origin);
        }
    }
}

#[derive(Clone, Copy)]
enum RepairEnd { Death, Lag, Horizon, Abort, Overflow, Bounds }

fn keep_repair(cfg: &Config, shared: &Shared, origin: Option<(u64, u32)>, masks: &[u8], end: RepairEnd, consumed: usize, parent_id: u64) {
    if cfg.repair_share == 0.0 {
        return;
    }
    let Some((ancestor, generation)) = origin else { return };
    let reason = match end {
        RepairEnd::Death => 0, RepairEnd::Lag => 1, RepairEnd::Horizon => 2,
        RepairEnd::Abort => { shared.repair_rejected[0].fetch_add(1, Ordering::Relaxed); return }
        RepairEnd::Overflow => { shared.repair_rejected[1].fetch_add(1, Ordering::Relaxed); return }
        RepairEnd::Bounds => { shared.repair_rejected[2].fetch_add(1, Ordering::Relaxed); return }
    };
    let mut bank = shared.repair.lock().unwrap();
    if bank.entries().any(|entry| entry.ancestor == ancestor && entry.masks == masks) {
        shared.repair_duplicates.fetch_add(1, Ordering::Relaxed);
        return;
    }
    let full = bank.count(ancestor) >= cfg.repair_per_seed;
    let generation = generation.saturating_add(1);
    let id = shared.next_id.fetch_add(1, Ordering::Relaxed);
    if bank.admit(ancestor, id, masks, generation) {
        if parent_id >= shared.resume_id_end && parent_id < shared.repair_id_end {
            shared.repair_restored_descendant_admissions.fetch_add(1, Ordering::Relaxed);
        }
        if matches!(end, RepairEnd::Death) && consumed < masks.len() {
            shared.repair_early_death_admissions.fetch_add(1, Ordering::Relaxed);
        }
        shared.repair_admitted[reason].fetch_add(1, Ordering::Relaxed);
        shared.repair_evicted.fetch_add(u64::from(full), Ordering::Relaxed);
        shared.repair_generation_max.fetch_max(generation as u64, Ordering::Relaxed);
    } else {
        shared.repair_rejected[3].fetch_add(1, Ordering::Relaxed);
    }
}

fn trial(cfg: &Config, shared: &Shared, rng: &mut Rng, base: &Sim, s: &mut Sim, m: &Member, deep: bool, origin: Option<(u64, u32)>) {
    shared.trials.fetch_add(1, Ordering::Relaxed);
    if origin.is_some_and(|(_, generation)| generation > 0) {
        shared.repair_trials.fetch_add(1, Ordering::Relaxed);
    }
    let l = m.masks.len();
    if cfg.adaptive_window && !eligible(l, cfg.window, false) {
        shared.short_parent_trials.fetch_add(1, Ordering::Relaxed);
    }
    // `base` is the state before entry `root` (the level start for a deep trial).
    let root = if deep { 0 } else {
        let Some(root) = window_root(l, cfg.window, cfg.adaptive_window) else { return };
        root
    };
    let setup_mutation = deep && rng.below(2) == 1;
    let (bundle_mutation, anchored_setup) = choose_setup(cfg.bundle_share, cfg.setup_share, deep, rng);
    let mut bundle_info = None;
    // Change inputs from a random frame in the window; before it, keep the lineage's inputs.
    let simple = if deep {
        shared.deep_trials.fetch_add(1, Ordering::Relaxed);
        let candidate = if setup_mutation {
            super::setup::mutate(&m.masks, &mut |n| rng.below(n))
        } else {
            simplified(rng, &m.masks, 1)
        };
        match candidate {
            Some(x) => Some(x),
            None => return,
        }
    } else if bundle_mutation {
        shared.bundle_picks.fetch_add(1, Ordering::Relaxed);
        let Some(proposal) = super::bundle::mutate_from(&m.masks, root, &mut |n| rng.below(n)) else { return };
        shared.bundle_proposals.fetch_add(1, Ordering::Relaxed);
        shared.bundle_z_changes[z_change(&m.masks, &proposal.masks)].fetch_add(1, Ordering::Relaxed);
        let suffix_shift = proposal.delta + proposal.kappa;
        let shift_kind = if suffix_shift < 0 { 0 } else if suffix_shift == 0 { 1 } else { 2 };
        shared.bundle_suffix_shifts[shift_kind].fetch_add(1, Ordering::Relaxed);
        bundle_info = Some((proposal.before_a, proposal.before_b, proposal.after_a, proposal.after_b,
            proposal.delta, proposal.kappa));
        Some(proposal.masks)
    } else if anchored_setup {
        shared.anchored_setup_picks.fetch_add(1, Ordering::Relaxed);
        let Some(candidate) = super::setup::mutate_from(&m.masks, root, &mut |n| rng.below(n)) else { return };
        shared.anchored_setup_proposals.fetch_add(1, Ordering::Relaxed);
        shared.anchored_setup_z_changes[z_change(&m.masks, &candidate)].fetch_add(1, Ordering::Relaxed);
        Some(candidate)
    } else if rng.unit() < cfg.simplify {
        simplified(rng, &m.masks, root)
    } else if rng.unit() < cfg.edits {
        edited(rng, &m.masks, root)
    } else {
        None
    };
    let masks = match simple {
        Some(x) => x,
        None => {
            let end = mutation_end(l, cfg.window, cfg.adaptive_window, deep);
            let Some(from) = mutation_start(rng, end, root) else { return };
            let mut masks = m.masks[..from].to_vec();
            let orig = *m.masks.get(from).unwrap_or(&16);
            let first = loop {
                let c = choose(cfg, rng, orig);
                if c != orig {
                    break c;
                }
            };
            masks.push(first);
            for t in from + 1..l {
                let c = if rng.below(10) < 7 { m.masks[t] } else { choose(cfg, rng, m.masks[t]) };
                masks.push(c);
            }
            masks
        }
    };
    let setup_z = anchored_setup.then(|| (
        m.masks.iter().position(|mask| mask & 128 != 0),
        masks.iter().position(|mask| mask & 128 != 0)));
    if let (true, Some(r)) = (cfg.fp_mode, cfg.budget_rel) {
        let best = shared.best_fp.lock().unwrap().0 as i64;
        if aspect_cost(&masks) as i64 > best + r {
            shared.over_budget.fetch_add(1, Ordering::Relaxed);
            return;
        }
    }
    // FNV-1a over the inputs: skip strings already simulated.
    let h = masks.iter().fold(0xcbf2_9ce4_8422_2325u64, |h, &b| (h ^ b as u64).wrapping_mul(0x0100_0000_01b3)) ^ masks.len() as u64;
    if !shared.tried.lock().unwrap().insert(h) {
        shared.repeats.fetch_add(1, Ordering::Relaxed);
        return;
    }
    if setup_mutation {
        shared.setup_trials.fetch_add(1, Ordering::Relaxed);
    }
    if anchored_setup {
        shared.anchored_setup_replayed.fetch_add(1, Ordering::Relaxed);
    }
    if bundle_mutation {
        shared.bundle_replayed.fetch_add(1, Ordering::Relaxed);
    }
    let mut last = *masks.last().unwrap();
    s.restore(base);
    s.masks.clear();
    s.masks.extend_from_slice(&masks);
    let mut lag_run = 0;
    for _ in 0..(l - root + cfg.extra) {
        if s.frame() >= cfg.max_frame {
            break;
        }
        let chest = s.ctx.character.bodies[CHEST];
        lab::arm(&[chest]);
        let f = s.step(last);
        lab::disarm();
        let events = lab::take_events();
        shared.frames.fetch_add(1, Ordering::Relaxed);
        if f.port_abort {
            keep_repair(cfg, shared, origin, &masks, RepairEnd::Abort, s.replay.iteration as usize, m.id);
            return;
        }
        if !f.alive_before {
            keep_repair(cfg, shared, origin, &masks, RepairEnd::Death, s.replay.iteration as usize, m.id);
            return;
        }
        let rel = score(&s.ctx, &events, chest, &shared.profile);
        let launch = f.displacement >= cfg.min_launch;
        if f.finished || rel.fc.is_finite() || launch {
            if let Some(b) = cfg.bounds {
                let p = f.chest_before;
                if p[0] < b[0] || p[0] > b[1] || p[1] < b[2] || p[1] > b[3] {
                    // The chest had already left the allowed area (e.g. past Satan's teeth).
                    shared.oob.fetch_add(1, Ordering::Relaxed);
                    keep_repair(cfg, shared, origin, &masks, RepairEnd::Bounds, s.replay.iteration as usize, m.id);
                    return;
                }
            }
            shared.launches.fetch_add(u64::from(launch), Ordering::Relaxed);
            if anchored_setup {
                shared.anchored_setup_raw_launch_attempts.fetch_add(u64::from(launch), Ordering::Relaxed);
                shared.anchored_setup_raw_finish_attempts.fetch_add(u64::from(f.finished), Ordering::Relaxed);
            }
            if bundle_mutation {
                shared.bundle_raw_launch_attempts.fetch_add(u64::from(launch), Ordering::Relaxed);
                shared.bundle_raw_finish_attempts.fetch_add(u64::from(f.finished), Ordering::Relaxed);
            }
            if origin.is_some_and(|(_, generation)| generation > 0) {
                shared.repair_launches.fetch_add(u64::from(launch), Ordering::Relaxed);
                shared.repair_finishes.fetch_add(u64::from(f.finished), Ordering::Relaxed);
            }
            if launch && m.id < shared.seed_id_end {
                shared.initial_parent_launches.fetch_add(1, Ordering::Relaxed);
            }
            if launch && m.id >= shared.seed_id_end && m.id < shared.resume_id_end {
                shared.resume_parent_launches.fetch_add(1, Ordering::Relaxed);
            }
            if launch && f.site == 2 && rel.route.is_empty() {
                // Pair-pool overflow before any TOI: not a real launch.
                shared.overflow.fetch_add(1, Ordering::Relaxed);
                keep_repair(cfg, shared, origin, &masks, RepairEnd::Overflow, s.replay.iteration as usize, m.id);
                return;
            }
            let key = (f.chest_after[0].to_bits(), f.chest_after[1].to_bits() ^ (rel.fc.to_bits().rotate_left(7)));
            let first_outcome = shared.seen.lock().unwrap().insert(key);
            // These are full raw proposal indices/inputs, not physical ejection
            // times. Recorded consumed TAS may end before the full proposal.
            let bundle_proposal = bundle_info.map(|(a, b, next_a, next_b, delta, kappa)| json!({
                "before_a": a, "before_b": b, "after_a": next_a, "after_b": next_b,
                "delta": delta, "kappa": kappa, "root": root, "parent_id": m.id,
                "first_z_before_entry": a, "first_z_after_entry": next_a,
                "before_tas": tas_text(&m.masks), "proposed_tas": tas_text(&masks)}));
            if f.finished && cfg.fp_mode {
                // Different TAS strings at the same finish position can have different
                // measured FP counts. Deduplicate only identical consumed inputs.
                let used = s.masks[..s.replay.iteration as usize].to_vec();
                if first_outcome && launch {
                    shared.distinct.fetch_add(1, Ordering::Relaxed);
                    shared.stage_counts[rel.stage.min(3) as usize].fetch_add(1, Ordering::Relaxed);
                }
                if fp_finish(cfg, shared, &used) {
                    record(cfg, shared, &used, &f, &rel, setup_z, bundle_proposal.as_ref());
                }
                return;
            }
            if !first_outcome {
                if launch && !f.finished {
                    shared.repeated_nonfinish_launch_hits.fetch_add(1, Ordering::Relaxed);
                    if retain_repeated_launch(cfg.retain_repeated_launches, first_outcome, launch, f.finished) {
                        let used = &s.masks[..s.replay.iteration as usize];
                        keep_repeated_parent(cfg, shared, used, m.id, &f, &rel, key, setup_z, bundle_proposal.as_ref());
                    }
                }
                // A repeated launch or finish ends the trial. A repeated finish check without
                // a launch (a limb touching the finish box) is only a shared earlier state:
                // keep playing, since later inputs may differ.
                if launch || f.finished {
                    return;
                }
                if !f.alive_after {
                    keep_repair(cfg, shared, origin, &masks, RepairEnd::Death, s.replay.iteration as usize, m.id);
                    return;
                }
                last = f.mask;
                continue;
            }
            let used = s.masks[..s.replay.iteration as usize].to_vec();
            if f.displacement >= cfg.min_launch {
                shared.distinct.fetch_add(1, Ordering::Relaxed);
                shared.stage_counts[rel.stage.min(3) as usize].fetch_add(1, Ordering::Relaxed);
            }
            record(cfg, shared, &used, &f, &rel, setup_z, bundle_proposal.as_ref());
            if f.finished {
                if let Ok((frame, moved)) = confirm(cfg.level, cfg.character, &used, cfg.min_launch) {
                    let name = cfg.run_dir.join("winner-1.tas");
                    std::fs::write(&name, tas_text(&used)).ok();
                    eprintln!("WINNER: finish frame {frame}, chest moved {moved:.1} m -> {}", name.display());
                    *shared.winner.lock().unwrap() = Some(used);
                    shared.stop.store(true, Ordering::Relaxed);
                }
                return;
            }
            if f.displacement >= cfg.min_launch {
                return;
            }
        }
        if !f.alive_after {
            keep_repair(cfg, shared, origin, &masks, RepairEnd::Death, s.replay.iteration as usize, m.id);
            return;
        }
        lag_run = if f.lagging { lag_run + 1 } else { 0 };
        if lag_run >= super::sim::lag_frames() {
            keep_repair(cfg, shared, origin, &masks, RepairEnd::Lag, s.replay.iteration as usize, m.id);
            return;
        }
        last = f.mask;
    }
    keep_repair(cfg, shared, origin, &masks, RepairEnd::Horizon, s.replay.iteration as usize, m.id);
}

fn record(cfg: &Config, shared: &Shared, masks: &[u8], f: &super::sim::Frame, rel: &Rel,
    setup_z: Option<(Option<usize>, Option<usize>)>, bundle_proposal: Option<&serde_json::Value>) {
    let good = rel.stage >= 2 || rel.fc < 150.0;
    // Structural frame-perfect cost (aspect_cost), not raw key edges.
    let events = aspect_cost(masks);
    if !cfg.fp_mode || rel.stage >= 3 || f.finished {
        let line = json!({"frame": f.frame, "moved": f.displacement, "to": f.chest_after, "site": f.site, "speed": rel.speed,
            "score": rel.score, "score_version": SCORE_VERSION, "stage": rel.stage, "depth": rel.depth, "fc": if rel.fc.is_finite() { rel.fc } else { -1.0 },
            "report_near": if rel.report_near.is_finite() { rel.report_near } else { -1.0 },
            "impact_near": if rel.impact_near.is_finite() { rel.impact_near } else { -1.0 },
            "strand_near": if rel.strand_near.is_finite() { rel.strand_near } else { -1.0 },
            "finish_inside": rel.finish_inside, "alive_after": f.alive_after, "chest_alive": f.chest_alive,
            "route": rel.route, "stranded_on": rel.stranded_on, "near": if rel.near.is_finite() { rel.near } else { -1.0 }, "finished": f.finished, "events": events,
            "tas": if good || rel.score < 250.0 { tas_text(masks) } else { String::new() },
            "anchored_setup": setup_z.is_some(),
            "setup_first_z_before_entry": setup_z.and_then(|pair| pair.0),
            "setup_first_z_after_entry": setup_z.and_then(|pair| pair.1),
            "bundle_proposal": bundle_proposal});
        writeln!(shared.log.lock().unwrap(), "{line}").ok();
    }
    {
        let mut b = shared.best.lock().unwrap();
        let improved_score = rel.score < b.0;
        let improved_fc = rel.fc < b.1;
        if improved_score {
            b.0 = rel.score;
        }
        if improved_fc {
            b.1 = rel.fc;
        }
        if improved_score || improved_fc {
            let tag = format!("best-s{:.0}-st{}-fc{:.0}-f{}", rel.score, rel.stage, if rel.fc.is_finite() { rel.fc } else { -1.0 }, f.frame);
            std::fs::write(cfg.run_dir.join(format!("{tag}.tas")), tas_text(masks)).ok();
        }
    }
    // Launches become new lineages (their windows are fresh launch-prone states).
    // Lineages must be able to lead below the best: cost <= best - 1 + slack.
    let admit = !cfg.fp_mode || events + 1 <= shared.best_fp.lock().unwrap().0.saturating_add(cfg.ev_slack);
    if admit && f.displacement >= cfg.min_launch && eligible(masks.len(), cfg.window, cfg.adaptive_window) {
        push_member(cfg, shared, masks, rel.score, events);
    }
}

fn push_member(cfg: &Config, shared: &Shared, masks: &[u8], score: f64, events: usize) {
    let id = shared.next_id.fetch_add(1, Ordering::Relaxed);
    let mut p = shared.pool.lock().unwrap();
    upsert(&mut p, Member { masks: masks.to_vec(), score, visits: 0, id, events });
    if p.len() > cfg.pool {
        trim(&mut p, cfg.pool, (cfg.seed_share > 0.0).then_some(shared.seed_id_end), cfg.fp_mode && cfg.ev_temp > 0.0);
    }
}

/// Retain an exact consumed raw parent at a repeated nonfinish launch. This is
/// parent-input novelty only: no geometry, full-state or FP novelty is claimed.
/// Do not call record, canonicalization, confirmation or finish measurement here.
fn keep_repeated_parent(cfg: &Config, shared: &Shared, masks: &[u8], parent_id: u64,
    f: &super::sim::Frame, rel: &Rel, key: (u64, u64),
    setup_z: Option<(Option<usize>, Option<usize>)>, bundle_proposal: Option<&serde_json::Value>) {
    if !eligible(masks.len(), cfg.window, cfg.adaptive_window) {
        shared.repeated_parent_rejected_window.fetch_add(1, Ordering::Relaxed);
        return;
    }
    let events = aspect_cost(masks);
    let best_cost = shared.best_fp.lock().unwrap().0;
    // Match the existing launch-parent admission heuristic exactly. This is
    // structural cost filtering, never an estimate of measured FP windows.
    if cfg.fp_mode && events + 1 > best_cost.saturating_add(cfg.ev_slack) {
        shared.repeated_parent_rejected_cost.fetch_add(1, Ordering::Relaxed);
        return;
    }
    let id = shared.next_id.fetch_add(1, Ordering::Relaxed);
    let insertion = {
        let mut pool = shared.pool.lock().unwrap();
        let mut tracked = shared.repeated_parent_ids.lock().unwrap();
        insert_tracked(&mut pool, &mut tracked,
            Member { masks: masks.to_vec(), score: rel.score, visits: 0, id, events },
            cfg.pool, (cfg.seed_share > 0.0).then_some(shared.seed_id_end),
            cfg.fp_mode && cfg.ev_temp > 0.0)
    };
    if !insertion.inserted {
        shared.repeated_parent_exact_merged.fetch_add(1, Ordering::Relaxed);
        return;
    }
    shared.repeated_parent_new_inserted.fetch_add(1, Ordering::Relaxed);
    if !insertion.retained {
        shared.repeated_parent_new_trimmed.fetch_add(1, Ordering::Relaxed);
        return;
    }
    shared.repeated_parent_new_retained.fetch_add(1, Ordering::Relaxed);
    // Exact bit strings survive JSON consumers that round numeric u64 values.
    // Retained means present immediately after insertion/trim, not permanently
    // protected: this unprotected ID may be evicted by a later better parent.
    let line = json!({
        "id": id, "parent_id": parent_id, "frame": masks.len(), "launch_frame": f.frame,
        "tas": tas_text(masks), "score": rel.score, "score_version": SCORE_VERSION,
        "events": events, "best_cost_at_admission": best_cost, "ev_slack": cfg.ev_slack,
        "fp_mode": cfg.fp_mode, "finished": false, "outcome_key_seen": true,
        "geometry_key_bits_hex": [format!("{:016x}", key.0), format!("{:016x}", key.1)],
        "chest_after_bits_hex": [format!("{:016x}", f.chest_after[0].to_bits()), format!("{:016x}", f.chest_after[1].to_bits())],
        "finish_distance_bits_hex": format!("{:016x}", rel.fc.to_bits()),
        "anchored_setup": setup_z.is_some(),
        "setup_first_z_before_entry": setup_z.and_then(|pair| pair.0),
        "setup_first_z_after_entry": setup_z.and_then(|pair| pair.1),
        "bundle_proposal": bundle_proposal,
    });
    if let Some(log) = shared.repeated_log.lock().unwrap().as_mut() {
        writeln!(log, "{line}").ok();
    }
}

/// A simplification of `masks` changing only entries >= `root`: extend the input before a random
/// frame over the next 1-6 frames, or drop one key's press/release pulse. None if unchanged.
fn simplified(rng: &mut Rng, masks: &[u8], root: usize) -> Option<Vec<u8>> {
    let l = masks.len();
    let mut m = masks.to_vec();
    if rng.below(2) == 0 {
        let t0 = root.max(1);
        if t0 >= l {
            return None;
        }
        let t = t0 + rng.below((l - t0) as u64) as usize;
        let len = 1 + rng.below(6) as usize;
        let v = m[t - 1];
        for x in &mut m[t..(t + len).min(l)] {
            *x = v;
        }
    } else {
        let all = edges(masks);
        let es: Vec<_> = all.iter().filter(|e| e.at >= root).collect();
        if es.is_empty() {
            return None;
        }
        let e = es[rng.below(es.len() as u64) as usize];
        let end = all.iter().filter(|x| x.bit == e.bit && x.at > e.at).map(|x| x.at).min().unwrap_or(l);
        let b = 1u8 << e.bit;
        for x in &mut m[e.at..end] {
            *x = if e.press { *x & !b } else { *x | b };
        }
    }
    (m != masks).then_some(m)
}

/// Measure both the raw finish and its simplification: preserving a finish while removing
/// edges does not prove that the remaining edges have no narrower timing windows.
fn fp_finish(cfg: &Config, shared: &Shared, used: &[u8]) -> bool {
    if !shared.measured.lock().unwrap().insert(used.to_vec()) {
        return false;
    }
    shared.finishes.fetch_add(1, Ordering::Relaxed);
    let raw_events = edges(used).len();
    if !measure_finish(cfg, shared, used, raw_events, "raw") {
        shared.measured.lock().unwrap().remove(used);
    }
    let c = canonicalize(cfg.level, cfg.character, used, cfg.max_frame);
    let new_canonical = c != used && shared.measured.lock().unwrap().insert(c.clone());
    if new_canonical && !measure_finish(cfg, shared, &c, raw_events, "canonical") {
        shared.measured.lock().unwrap().remove(&c);
    }
    true
}

fn measure_finish(cfg: &Config, shared: &Shared, masks: &[u8], raw_events: usize, variant: &str) -> bool {
    let finish = finish_of(cfg.level, cfg.character, masks, cfg.max_frame);
    if finish == 0 {
        return false;
    }
    let Some((fp, ev)) = fp_count(cfg.level, cfg.character, masks, cfg.max_frame, 1) else { return false };
    let ac = aspect_cost(masks);
    if let Some(f) = shared.fp_log.lock().unwrap().as_mut() {
        writeln!(f, "{}", json!({"cost": ac, "fp": fp, "events": ev, "finish": finish, "raw_events": raw_events,
            "variant": variant, "tas": tas_text(masks)})).ok();
    }
    {
        let mut b = shared.best_fp.lock().unwrap();
        if is_better((ac, fp, finish), *b) {
            *b = (ac, fp, finish);
            let name = format!("best-cost{ac}-fp{fp}-ev{ev}-f{finish}.tas");
            std::fs::write(cfg.run_dir.join(&name), tas_text(masks)).ok();
            eprintln!("IMPROVED: cost {ac} ({fp} frame-perfect as laid out, {ev} events), finish {finish} -> {name}");
        }
    }
    if eligible(masks.len(), cfg.window, cfg.adaptive_window) {
        push_member(cfg, shared, masks, 0.0, ac);
    }
    true
}

#[cfg(test)]
mod selection_tests {
    use super::{validate_repair, validate_selection};

    #[test]
    fn repair_requires_a_bounded_protected_bank_and_a_valid_cached_anchor() {
        for invalid in [f64::NAN, f64::INFINITY, -0.01, 1.01] {
            assert!(validate_repair(invalid, 8, 0.5, 0.0).is_err());
        }
        assert!(validate_repair(0.5, 0, 0.5, 0.0).is_err());
        assert!(validate_repair(0.5, 8, 0.0, 0.0).is_err());
        assert!(validate_repair(0.5, 8, 0.5, 1.0).is_err());
        assert!(validate_repair(0.5, 8, 0.5, 0.0).is_ok());
        assert!(validate_repair(0.0, 0, 0.0, 1.0).is_ok());
    }

    #[test]
    fn reservoir_probability_and_capacity_fail_before_simulation() {
        for invalid in [f64::NAN, f64::INFINITY, -0.01, 1.01] {
            assert!(validate_selection(1.0, invalid, 135, 135).is_err());
        }
        assert!(validate_selection(1.0, 0.5, 134, 135).is_err());
        assert!(validate_selection(1.0, 0.5, 135, 135).is_ok());
        assert!(validate_selection(1.0, 1.0, 135, 135).is_ok());
        // With protection disabled, the original seed-loading policy is unchanged.
        assert!(validate_selection(1.0, 0.0, 4, 135).is_ok());
    }
}

#[cfg(test)]
mod anchored_restore_tests {
    use super::{choose_anchored, choose_setup, restore_repair, validate_bundle, validate_setup, z_change, Bank, Rng};

    fn bank(capacity: usize) -> Bank {
        Bank::new(vec![(0, vec![9, 24, 148, 176], 2),
            (2, vec![8, 16, 148, 240], 2)], capacity)
    }

    #[test]
    fn disabled_and_deep_setup_selection_preserve_rng_draws() {
        for (share, deep) in [(0.0, false), (0.0, true), (1.0, true)] {
            let mut actual = Rng(2026092922);
            let mut expected = Rng(2026092922);
            assert!(!choose_anchored(share, deep, &mut actual));
            assert_eq!(actual.unit(), expected.unit());
            assert_eq!(actual.0, expected.0);
        }
        let mut actual = Rng(2026092922);
        let mut expected = Rng(2026092922);
        assert!(choose_anchored(1.0, false, &mut actual));
        expected.unit();
        assert_eq!(actual.0, expected.0);
    }

    #[test]
    fn disabled_bundle_preserves_old_anchored_selection_and_rng() {
        for seed in 1..100 {
            for (share, deep) in [(0.0, false), (0.25, false), (1.0, false), (0.25, true)] {
                let mut actual = Rng(seed);
                let mut expected = Rng(seed);
                assert_eq!(choose_setup(0.0, share, deep, &mut actual),
                    (false, choose_anchored(share, deep, &mut expected)));
                assert_eq!(actual.0, expected.0);
            }
        }
    }

    #[test]
    fn bundle_selection_skips_anchored_draw_and_all_deep_draws() {
        let mut actual = Rng(42);
        let mut expected = Rng(42);
        assert_eq!(choose_setup(1.0, 1.0, false, &mut actual), (true, false));
        expected.unit();
        assert_eq!(actual.0, expected.0);
        assert_eq!(choose_setup(1.0, 1.0, true, &mut actual), (false, false));
        assert_eq!(actual.0, expected.0);
    }

    #[test]
    fn bundle_probability_fails_before_simulation() {
        for invalid in [f64::NAN, f64::INFINITY, -0.01, 1.01] {
            assert!(validate_bundle(invalid).unwrap_err().starts_with("--bundle-share"));
        }
        for valid in [0.0, 0.75, 1.0] { assert!(validate_bundle(valid).is_ok()); }
    }

    #[test]
    fn anchored_probability_rejects_invalid_values_before_simulation() {
        for invalid in [f64::NAN, f64::INFINITY, -0.01, 1.01] {
            assert!(validate_setup(invalid).is_err());
        }
        assert!(validate_setup(0.0).is_ok());
        assert!(validate_setup(0.5).is_ok());
        assert!(validate_setup(1.0).is_ok());
    }

    #[test]
    fn raw_first_z_coverage_distinguishes_all_five_change_kinds() {
        assert_eq!(z_change(&[0, 128, 128], &[128, 128, 128]), 0);
        assert_eq!(z_change(&[0, 128, 128], &[0, 0, 128]), 1);
        assert_eq!(z_change(&[0, 128, 128], &[0, 0, 0]), 2);
        assert_eq!(z_change(&[0, 0, 0], &[0, 128, 128]), 3);
        assert_eq!(z_change(&[0, 0, 0], &[64, 0, 0]), 4);
        // Release and lower-priority key changes do not move the first z entry.
        assert_eq!(z_change(&[0, 128, 0, 128], &[0, 129, 128, 128]), 4);
    }

    #[test]
    fn restored_repairs_keep_maximum_generation_and_use_fresh_disjoint_ids() {
        let rows = vec![(0, vec![9, 24, 149, 176], u32::MAX),
            (2, vec![8, 16, 150, 240], 3)];
        // Curated IDs end at3 and four resume rows reserve3..7. Imported old
        // IDs are ignored; restored rows start7 and the next child starts9.
        let (mut restored, next_id) = restore_repair(bank(2), &rows, 7, true).unwrap();
        assert_eq!(next_id, 9);
        assert_eq!(restored.entries().map(|entry| entry.id).collect::<Vec<_>>(), [7, 8]);
        assert_eq!(restored.max_generation(), u32::MAX);
        let parent = restored.pick(0, 0).unwrap();
        let mut child = parent.masks.clone();
        child[3] ^= 1;
        assert!(restored.admit(parent.ancestor, next_id, &child, parent.generation.saturating_add(1)));
        assert_eq!(restored.pick(0, 1).unwrap().generation, u32::MAX);
        assert_eq!(restored.pick(0, 1).unwrap().id, 9);
        assert_eq!(restored.pick(0, 1).unwrap().masks[..2], [9, 24]);
    }

    #[test]
    fn invalid_repair_checkpoints_fail_instead_of_silently_dropping_rows() {
        let valid = (0, vec![9, 24, 149, 176], 4);
        for row in [(99, vec![9, 24, 149, 176], 4),
            (0, vec![8, 24, 149, 176], 4), (0, vec![9, 24, 149], 4),
            (0, vec![9, 24, 149, 176], 0), (0, vec![9, 24, 148, 176], 4)] {
            assert!(restore_repair(bank(2), &[row], 7, true).is_err());
        }
        assert!(restore_repair(bank(2), &[valid.clone(), valid.clone()], 7, true).is_err());
        assert!(restore_repair(bank(1), &[valid.clone(), (0, vec![9, 24, 150, 176], 4)], 7, true).is_err());
        assert!(restore_repair(bank(2), &[valid.clone()], 7, false).is_err());
        assert!(restore_repair(bank(2), &[valid], u64::MAX, true).is_err());
        let (empty, next_id) = restore_repair(bank(2), &[], 7, false).unwrap();
        assert_eq!(empty.len(), 0);
        assert_eq!(next_id, 7);
    }
}

#[cfg(test)]
mod import_window_tests {
    use super::{append_resume, curated_members, increment_visits, mutation_end, mutation_start, validate_window, window_root, Rng};
    use super::super::pool_policy::trim;

    #[test]
    fn resume_duplicates_preserve_curated_identity_and_add_visits_without_protection() {
        let seeds = vec![vec![9; 4], vec![9; 4], vec![24; 4]];
        let mut pool = curated_members(&seeds, &[400.0, 200.0, 300.0], 1, false);
        let curated_count = pool.len();
        let resume = vec![(vec![9; 4], 10.0, 7), (vec![24; 4], 30.0, 3),
            (vec![44; 4], 1.0, 11), (vec![44; 4], 0.0, 5)];
        assert_eq!(append_resume(&mut pool, &resume, seeds.len() as u64, 1, false), (4, 1));
        assert_eq!(curated_count, 2);
        assert_eq!(pool.iter().map(|m| m.id).collect::<Vec<_>>(), [0, 2, 5]);
        assert_eq!(pool.iter().map(|m| m.visits).collect::<Vec<_>>(), [7, 3, 16]);
        assert_eq!(pool.iter().map(|m| m.score).collect::<Vec<_>>(), [10.0, 30.0, 0.0]);
        // The strongest resumed score still cannot displace either protected
        // curated ID, including the gap left by the duplicate curated row.
        trim(&mut pool, 2, Some(seeds.len() as u64), false);
        assert_eq!(pool.iter().map(|m| m.id).collect::<Vec<_>>(), [0, 2]);
    }

    #[test]
    fn selecting_a_maximum_visit_import_saturates_without_resetting_its_history() {
        let seeds = vec![vec![9; 4]];
        let mut pool = curated_members(&seeds, &[], 1, false);
        let resume = vec![(vec![9; 4], 25.0, u64::MAX)];
        assert_eq!(append_resume(&mut pool, &resume, 1, 1, false), (1, 0));
        assert_eq!(pool[0].id, 0);
        assert_eq!(pool[0].visits, u64::MAX);
        increment_visits(&mut pool[0].visits);
        increment_visits(&mut pool[0].visits);
        assert_eq!(pool[0].visits, u64::MAX);
        assert_eq!(pool[0].id, 0);
        assert_eq!(pool[0].score, 25.0);
        assert_eq!(pool[0].masks, seeds[0]);

        for prior in [0, 1, 7, u64::MAX - 1] {
            let mut visits = prior;
            increment_visits(&mut visits);
            assert_eq!(visits, prior + 1);
        }
    }

    #[test]
    fn adaptive_imports_reuse_short_launches_without_promoting_their_resume_ids() {
        let seeds = vec![vec![1; 239]];
        let resume = vec![(Vec::new(), 0.0, 99), (vec![16; 202], 25.0, 4),
            (vec![32; 239], 40.0, 8)];
        let mut fixed = curated_members(&seeds, &[], 223, false);
        assert_eq!(append_resume(&mut fixed, &resume, 1, 223, false), (1, 1));
        assert_eq!(fixed.iter().map(|m| m.id).collect::<Vec<_>>(), [0, 3]);

        let mut adaptive = curated_members(&seeds, &[], 223, true);
        assert_eq!(append_resume(&mut adaptive, &resume, 1, 223, true), (2, 2));
        assert_eq!(adaptive.iter().map(|m| m.id).collect::<Vec<_>>(), [0, 2, 3]);
        assert_eq!(adaptive.iter().filter(|m| m.id < 1).count(), 1);
        assert_eq!(window_root(adaptive[1].masks.len(), 223, true), Some(0));
        assert_eq!(window_root(adaptive[2].masks.len(), 223, true), Some(16));
    }

    #[test]
    fn fixed_window_sampling_preserves_the_existing_random_sequence() {
        for initial in [0, 1, 0x1234_5678_9abc_def0, u64::MAX] {
            let mut previous = Rng(initial);
            let mut current = Rng(initial);
            for (len, window) in [(125, 85), (239, 223), (87, 84)] {
                let root = window_root(len, window, false).unwrap();
                for _ in 0..32 {
                    assert_eq!(mutation_start(&mut current, len, root),
                        Some(root + previous.below(window as u64) as usize));
                }
            }
            assert_eq!(current.0, previous.0);
        }
    }

    #[test]
    fn default_deep_rerolls_preserve_the_old_zero_root_window_modulus() {
        for initial in [0, 1, 0x1234_5678_9abc_def0, u64::MAX] {
            let mut previous = Rng(initial);
            let mut current = Rng(initial);
            for (len, window) in [(125, 85), (239, 223), (104, 24)] {
                let end = mutation_end(len, window, false, true);
                assert_eq!(end, window);
                for _ in 0..32 {
                    assert_eq!(mutation_start(&mut current, end, 0),
                        Some(previous.below(window as u64) as usize));
                }
            }
            assert_eq!(current.0, previous.0);
        }
    }

    #[test]
    fn adaptive_deep_rerolls_sample_the_actual_input_length() {
        let mut expected = Rng(2026092921);
        let mut current = Rng(2026092921);
        for (len, window) in [(1, 223), (202, 223), (104, 24)] {
            let end = mutation_end(len, window, true, true);
            assert_eq!(end, len);
            for _ in 0..32 {
                assert_eq!(mutation_start(&mut current, end, 0),
                    Some(expected.below(len as u64) as usize));
            }
        }
        assert_eq!(current.0, expected.0);
    }

    #[test]
    fn adaptive_sampling_handles_single_entries_and_window_boundaries() {
        let mut rng = Rng(2026092921);
        for len in [1, 2, 222, 223, 224, 225, 239] {
            let root = window_root(len, 223, true).unwrap();
            for _ in 0..64 {
                let from = mutation_start(&mut rng, len, root).unwrap();
                assert!(from >= root && from < len);
            }
        }
        let unchanged = rng.0;
        assert_eq!(mutation_start(&mut rng, 0, 0), None);
        assert_eq!(mutation_start(&mut rng, 3, 3), None);
        assert_eq!(mutation_start(&mut rng, 3, 4), None);
        assert_eq!(rng.0, unchanged);
    }

    #[test]
    fn zero_window_is_rejected_before_any_pool_or_simulation_work() {
        assert!(validate_window(0).is_err());
        assert!(validate_window(1).is_ok());
        assert!(validate_window(usize::MAX).is_ok());
    }
}

#[cfg(test)]
mod fitness_tests {
    use super::{fitness, profile, score, Rel};
    use hw_core::lab::Event;

    #[test]
    fn route_fallbacks_do_not_turn_depth_or_strands_into_solved_reports() {
        for (stage, depth, expected) in [(0, 0.0, 400.0), (1, 0.0, 300.0),
            (2, 0.0, 200.0), (3, 0.0, 130.0), (3, 85.0, 45.0)] {
            assert_eq!(fitness(&Rel { stage, depth, ..Rel::default() }, true, true), expected);
        }
        let report = Rel { stage: 3, report_near: 0.0, ..Rel::default() };
        let impact = Rel { stage: 3, impact_near: 0.0, report_near: 100.0, ..Rel::default() };
        let strand = Rel { stage: 3, strand_near: 0.0, ..Rel::default() };
        assert_eq!(fitness(&report, true, true), 5.0);
        assert_eq!(fitness(&impact, true, true), 25.0);
        assert_eq!(fitness(&strand, true, true), 40.0);
    }

    #[test]
    fn live_callbacks_outrank_geometric_inside_points_without_inventing_a_finish() {
        let boundary = Rel { fc: 0.0, ..Rel::default() };
        assert_eq!(fitness(&boundary, true, true), 1.0);
        assert_eq!(fitness(&Rel { fc: 3.0, ..Rel::default() }, true, true), 4.0);
        // Damage after a counted finish must not hide the winning result.
        assert_eq!(fitness(&Rel { finish_inside: true, ..boundary }, false, false), 0.0);
    }

    #[test]
    fn terminal_nonfinishes_remain_explorable_but_do_not_dominate_live_reports() {
        let r = Rel { stage: 3, report_near: 0.0, ..Rel::default() };
        assert_eq!(fitness(&r, false, true), 25.0);
        assert_eq!(fitness(&r, true, false), 25.0);
        assert_eq!(fitness(&r, false, false), 25.0);
    }

    #[test]
    fn invalid_proximity_fields_do_not_manufacture_low_scores() {
        let r = Rel { stage: 3, fc: f64::NAN, report_near: f64::NAN,
            impact_near: f64::NAN, strand_near: f64::NAN, ..Rel::default() };
        assert_eq!(fitness(&r, true, true), 130.0);
    }

    #[test]
    fn nan_finish_coordinates_are_not_zero_distance_callbacks() {
        std::thread::Builder::new().stack_size(16 << 20).spawn(|| {
            let sim = super::Sim::new(3214, 4).unwrap();
            let chest = sim.ctx.character.bodies[hw_core::game::character::CHEST];
            let events = [Event::Finish { x: f64::NAN, y: f64::NAN,
                inside: false, dead: false, other: hw_core::NONE }];
            let r = score(&sim.ctx, &events, chest, &profile(3214));
            assert_eq!(r.fc, f64::INFINITY);
            assert_eq!(r.score, 400.0);
        }).unwrap().join().unwrap();
    }
}

/// Mask for a post-eject class: pose key (left 1, right 2, up 3, down 4, shift 6, ctrl 7) plus
/// space for grab, with z copied from `z`.
fn class_mask(p: u8, grab: bool, z: u8) -> u8 {
    let key = match p {
        1 => 1,
        2 => 2,
        3 => 4,
        4 => 8,
        6 => 32,
        7 => 64,
        _ => 0,
    };
    key | if grab { 16 } else { 0 } | (z & 128)
}

/// A cost-1 edit of `masks` at entries >= `root` (after eject only): move a class boundary by
/// 1-3 frames, change one class run to a neighbouring class (another pose level or the other
/// grab state), delete a run, or insert a 1-8 frame run. None if nothing changed.
fn edited(rng: &mut Rng, masks: &[u8], root: usize) -> Option<Vec<u8>> {
    let l = masks.len();
    let e = masks.iter().position(|m| m & 128 != 0)?;
    let lo = root.max(e + 2);
    if lo + 1 >= l {
        return None;
    }
    let class = |m: u8| (pose(m), m & 16 != 0);
    // Runs of equal class within [lo - 1, l).
    let mut runs: Vec<(usize, usize)> = Vec::new();
    let mut a = lo - 1;
    for t in lo..=l {
        if t == l || class(masks[t]) != class(masks[a]) {
            runs.push((a, t));
            a = t;
        }
    }
    let mut m = masks.to_vec();
    match rng.below(4) {
        0 => {
            // Move a boundary.
            let bs: Vec<usize> = runs.iter().map(|r| r.0).filter(|&t| t >= lo).collect();
            if bs.is_empty() {
                return None;
            }
            let t = bs[rng.below(bs.len() as u64) as usize];
            let d = 1 + rng.below(3) as usize;
            if rng.below(2) == 0 {
                let from = t.saturating_sub(d).max(lo);
                let v = masks[t];
                for x in &mut m[from..t] {
                    *x = v;
                }
            } else {
                let v = masks[t - 1];
                for x in &mut m[t..(t + d).min(l)] {
                    *x = v;
                }
            }
        }
        1 | 3 => {
            // Change a run's class (1), or insert a short run of a new class (3).
            let (a, b) = if rng.below(4) == 3 {
                let a = lo + rng.below((l - lo) as u64) as usize;
                (a, (a + 1 + rng.below(8) as usize).min(l))
            } else {
                let r = runs[rng.below(runs.len() as u64) as usize];
                (r.0.max(lo), r.1)
            };
            if a >= b {
                return None;
            }
            let (p, g) = class(masks[a]);
            let (np, ng) = if rng.below(3) == 0 {
                (p, !g)
            } else {
                const P: [u8; 7] = [0, 1, 2, 3, 4, 6, 7];
                let lvl = |x: u8| match x {
                    0 => 0,
                    1 | 2 => 1,
                    3 | 4 => 2,
                    6 => 3,
                    _ => 4,
                };
                let cands: Vec<u8> = P.iter().copied().filter(|&x| lvl(x) != lvl(p)).collect();
                (cands[rng.below(cands.len() as u64) as usize], g)
            };
            for t in a..b {
                m[t] = class_mask(np, ng, masks[t]);
            }
        }
        _ => {
            // Delete a run (it takes the class before it).
            let rs: Vec<(usize, usize)> = runs.iter().copied().filter(|r| r.0 >= lo).collect();
            if rs.is_empty() {
                return None;
            }
            let (a, b) = rs[rng.below(rs.len() as u64) as usize];
            let v = masks[a - 1];
            for x in &mut m[a..b] {
                *x = v;
            }
        }
    }
    (m != masks).then_some(m)
}
