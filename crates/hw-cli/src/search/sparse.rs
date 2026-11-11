//! `hw-search sparse`: rewrite a finishing string with as few frame-perfect key events as
//! possible, finishing by `--limit`.
//!
//! A launch is chaotic: any key event that changes the simulation before it moves the launch, so
//! in practice every *effective* event is frame-perfect (see `events.rs`). The number of
//! frame-perfect events is therefore bounded by the number of events the string needs at all.
//! This search keeps the first `j` events of the base string (in time order), then holds the keys
//! and adds `k` new random events (bit toggles) at or after a frame `s`, playing until the first
//! launch. A finish is simplified (`canonicalize`: drop every press/release pulse whose removal
//! still finishes) and scored by its frame-perfect event count (`events::windows`).
use super::events::{edges, windows, Replayer};
use super::sim::{tas_text, Sim};
use serde_json::json;
use std::collections::HashSet;
use std::io::Write;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

pub struct Config {
    pub level: u32,
    pub character: u32,
    pub base: Vec<u8>,
    pub limit: u32,
    pub run_dir: std::path::PathBuf,
    pub threads: usize,
    pub hours: f64,
    pub seed: u64,
    pub min_launch: f64,
    /// Most new events per trial.
    pub kmax: usize,
    /// New events fall within this many frames after `s`.
    pub span: usize,
    pub batch: usize,
    /// Allowed bits for new events (mask of bit numbers).
    pub bits: u8,
    /// Smallest `j` (kept base events) to try.
    pub jmin: usize,
    /// New events start within this many frames after the cut.
    pub sreach: usize,
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
        self.next() % n.max(1)
    }
}

/// Finish frame of `masks` from scratch (0 if none by `limit`).
pub fn finish_of(level: u32, character: u32, masks: &[u8], limit: u32) -> u32 {
    match Replayer::new(level, character, masks, 1_000_000, limit) {
        Ok(rep) => {
            let mut w = rep.template().fork();
            rep.finish(&mut w, masks, 0)
        }
        Err(_) => 0,
    }
}

/// Remove press/release pulses (and trailing edges) that the finish does not need, repeatedly.
/// Returns the simplified string trimmed to its finish frame.
pub fn canonicalize(level: u32, character: u32, masks: &[u8], limit: u32) -> Vec<u8> {
    let mut cur = masks.to_vec();
    'outer: loop {
        let rep = match Replayer::new(level, character, &cur, 8, limit) {
            Ok(r) => r,
            Err(_) => return cur,
        };
        let mut work = rep.template().fork();
        let es = edges(&cur);
        // Pulses: (from, to, bit, value) = set bit to value on [from, to), i.e. drop the edge at
        // `from` and the next edge of that bit.
        let mut pulses: Vec<(usize, usize, u8, bool)> = Vec::new();
        for bit in 0..8u8 {
            let bes: Vec<_> = es.iter().filter(|e| e.bit == bit).collect();
            for (i, e) in bes.iter().enumerate() {
                let before = !e.press;
                let end = if i + 1 < bes.len() { bes[i + 1].at } else { cur.len() };
                pulses.push((e.at, end, bit, before));
            }
        }
        // Candidates: single pulses, then pairs of pulses starting on the same entry (opposing
        // keys cancel, so up+down or left+right can only go together).
        let mut cands: Vec<Vec<(usize, usize, u8, bool)>> = pulses.iter().map(|&p| vec![p]).collect();
        for (i, a) in pulses.iter().enumerate() {
            for b in &pulses[i + 1..] {
                if a.0 == b.0 {
                    cands.push(vec![*a, *b]);
                }
            }
        }
        // Latest first: late events have the shortest replays and are the likeliest noise.
        cands.sort_by_key(|c| (std::cmp::Reverse(c[0].0), c.len()));
        for cand in cands {
            let mut m = cur.clone();
            for &(from, to, bit, v) in &cand {
                for x in &mut m[from..to] {
                    *x = if v { *x | (1 << bit) } else { *x & !(1 << bit) };
                }
            }
            let from = cand[0].0;
            let f = rep.finish(&mut work, &m, from);
            if f != 0 {
                // Trim to the finish frame; if the finish moved past the end, the game read
                // all-zero entries there, so write them out (their releases are real inputs).
                m.resize(f as usize, 0);
                cur = m;
                continue 'outer;
            }
        }
        return cur;
    }
}

/// (frame-perfect events, all events) of a finishing string.
pub fn fp_count(level: u32, character: u32, masks: &[u8], limit: u32, threads: usize) -> Option<(usize, usize)> {
    let rep = Replayer::new(level, character, masks, 8, limit).ok()?;
    let mut probe = rep.template().fork();
    if rep.finish(&mut probe, masks, 0) == 0 {
        return None;
    }
    let ws = windows(&rep, masks, 3, threads);
    Some((ws.iter().filter(|w| w.earliest == w.latest).count(), ws.len()))
}

struct Shared {
    stop: AtomicBool,
    trials: AtomicU64,
    frames: AtomicU64,
    launches: AtomicU64,
    finishes: AtomicU64,
    seen: Mutex<HashSet<(u64, u64)>>,
    /// Best (frame-perfect, events, finish) so far.
    best: Mutex<(usize, usize, u32)>,
    log: Mutex<std::fs::File>,
    /// Per-j (trials, distinct launches, finishes).
    per_j: Mutex<Vec<(u64, u64, u64)>>,
}

pub fn run(cfg: Config) -> Result<(), String> {
    std::fs::create_dir_all(&cfg.run_dir).map_err(|e| e.to_string())?;
    let base = canonicalize(cfg.level, cfg.character, &cfg.base, cfg.limit);
    let (fp, ev) = fp_count(cfg.level, cfg.character, &base, cfg.limit, cfg.threads).ok_or("base does not finish")?;
    let f0 = finish_of(cfg.level, cfg.character, &base, cfg.limit);
    eprintln!("sparse: base canonical: {ev} events, {fp} frame-perfect, finish {f0}");
    std::fs::write(cfg.run_dir.join(format!("base-canonical-fp{fp}-ev{ev}-f{f0}.tas")), tas_text(&base)).ok();
    let es = edges(&base);
    // Cut points: after j events (time order), the next event's entry is where holding starts.
    let cut_at: Vec<usize> = (0..=es.len()).map(|j| if j < es.len() { es[j].at } else { base.len() }).collect();
    let shared = Shared {
        stop: AtomicBool::new(false),
        trials: AtomicU64::new(0),
        frames: AtomicU64::new(0),
        launches: AtomicU64::new(0),
        finishes: AtomicU64::new(0),
        seen: Mutex::new(HashSet::new()),
        best: Mutex::new((fp, ev, f0)),
        log: Mutex::new(std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("finishes.jsonl")).map_err(|e| e.to_string())?),
        per_j: Mutex::new(vec![(0, 0, 0); es.len() + 1]),
    };
    let template = Sim::new(cfg.level, cfg.character)?;
    let start = Instant::now();
    let deadline = start + Duration::from_secs_f64(cfg.hours * 3600.0);
    std::thread::scope(|scope| {
        for w in 0..cfg.threads {
            let (shared, cfg, template, base, cut_at) = (&shared, &cfg, &template, &base, &cut_at);
            std::thread::Builder::new()
                .stack_size(16 << 20)
                .spawn_scoped(scope, move || worker(w as u64, cfg, template, base, cut_at, shared))
                .unwrap();
        }
        let mut stats = std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("stats.jsonl")).unwrap();
        while !shared.stop.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_secs(10));
            if Instant::now() > deadline || cfg.run_dir.join("STOP").exists() {
                shared.stop.store(true, Ordering::Relaxed);
            }
            let secs = start.elapsed().as_secs_f64();
            let b = *shared.best.lock().unwrap();
            let pj: Vec<String> = shared
                .per_j
                .lock()
                .unwrap()
                .iter()
                .enumerate()
                .filter(|(_, c)| c.0 > 0)
                .map(|(j, c)| format!("{j}:{}/{}/{}", c.2, c.1, c.0))
                .collect();
            let line = json!({"t": secs as u64, "fps": shared.frames.load(Ordering::Relaxed) as f64 / secs,
                "trials": shared.trials.load(Ordering::Relaxed), "distinct_launches": shared.launches.load(Ordering::Relaxed),
                "finishes": shared.finishes.load(Ordering::Relaxed), "best_fp": b.0, "best_events": b.1, "best_finish": b.2,
                "j_finish_launch_trials": pj});
            writeln!(stats, "{line}").ok();
            eprintln!("{line}");
        }
    });
    Ok(())
}

fn worker(w: u64, cfg: &Config, template: &Sim, base: &[u8], cut_at: &[usize], shared: &Shared) {
    let mut rng = Rng(cfg.seed ^ (w + 1).wrapping_mul(0x5851_f42d_4c95_7f2d));
    let bits: Vec<u8> = (0..8u8).filter(|b| cfg.bits & (1 << b) != 0).collect();
    let limit = cfg.limit as usize;
    let mut snap = template.fork();
    let mut work = template.fork();
    while !shared.stop.load(Ordering::Relaxed) {
        let best_fp = shared.best.lock().unwrap().0;
        // Keep j base events; add k in 1..=kmax, with j + k at most best_fp + 1 (a string needs
        // fewer events than the best has frame-perfect ones, give or take tolerant events).
        let jmax = (cut_at.len() - 1).min(best_fp);
        if jmax < cfg.jmin {
            break;
        }
        let j = cfg.jmin + rng.below((jmax - cfg.jmin + 1) as u64) as usize;
        let kcap = cfg.kmax.min(best_fp + 1 - j.min(best_fp)).max(1);
        let cut = cut_at[j];
        // Held string: base before the cut, then the last kept mask.
        let hold = if cut == 0 { 0 } else { base[cut - 1] };
        let mut held: Vec<u8> = base[..cut.min(base.len())].to_vec();
        held.resize(limit, hold);
        // Snapshot at a random frame s >= cut along the held trajectory.
        let s = cut + rng.below((limit - cut).min(cfg.sreach).max(1) as u64) as usize;
        snap.restore(template);
        snap.masks.clear();
        snap.masks.extend_from_slice(&held);
        let mut ok = true;
        while (snap.replay.iteration as usize) < s {
            let f = snap.step(0);
            shared.frames.fetch_add(1, Ordering::Relaxed);
            if f.port_abort || !f.alive_after || snap.replay.finish_frame != 0 || (f.alive_before && f.displacement >= cfg.min_launch) {
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
            let k = 1 + rng.below(kcap as u64) as usize;
            let mut masks = held.clone();
            let hi = (s + cfg.span).min(limit);
            for _ in 0..k {
                let t = s + rng.below((hi - s) as u64) as usize;
                let b = bits[rng.below(bits.len() as u64) as usize];
                for x in &mut masks[t..] {
                    *x ^= 1 << b;
                }
            }
            work.restore(&snap);
            work.masks.clear();
            work.masks.extend_from_slice(&masks);
            shared.per_j.lock().unwrap()[j].0 += 1;
            while work.frame() < cfg.limit {
                let f = work.step(0);
                shared.frames.fetch_add(1, Ordering::Relaxed);
                if f.port_abort {
                    break;
                }
                let launch = f.alive_before && f.displacement >= cfg.min_launch;
                if work.replay.finish_frame != 0 {
                    let used = masks[..work.replay.iteration as usize].to_vec();
                    shared.finishes.fetch_add(1, Ordering::Relaxed);
                    shared.per_j.lock().unwrap()[j].2 += 1;
                    found(cfg, shared, &used, j, k);
                    break;
                }
                if launch {
                    let key = (f.chest_after[0].to_bits(), f.chest_after[1].to_bits());
                    if shared.seen.lock().unwrap().insert(key) {
                        shared.launches.fetch_add(1, Ordering::Relaxed);
                        shared.per_j.lock().unwrap()[j].1 += 1;
                    }
                    break;
                }
                if !f.alive_after {
                    break;
                }
            }
        }
    }
}

fn found(cfg: &Config, shared: &Shared, masks: &[u8], j: usize, k: usize) {
    let c = canonicalize(cfg.level, cfg.character, masks, cfg.limit);
    let finish = finish_of(cfg.level, cfg.character, &c, cfg.limit);
    if finish == 0 {
        return;
    }
    let Some((fp, ev)) = fp_count(cfg.level, cfg.character, &c, cfg.limit, 1) else { return };
    writeln!(shared.log.lock().unwrap(), "{}", json!({"fp": fp, "events": ev, "finish": finish, "j": j, "k": k, "tas": tas_text(&c)})).ok();
    let mut b = shared.best.lock().unwrap();
    if (fp, ev, finish) < *b {
        *b = (fp, ev, finish);
        let name = format!("best-fp{fp}-ev{ev}-f{finish}.tas");
        std::fs::write(cfg.run_dir.join(&name), tas_text(&c)).ok();
        eprintln!("IMPROVED: {fp} frame-perfect, {ev} events, finish {finish} -> {name}");
    }
}
