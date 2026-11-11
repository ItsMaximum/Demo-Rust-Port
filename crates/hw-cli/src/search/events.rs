//! `hw-search events`: the key presses and releases of a finishing string, and how far each one
//! can move while the string still finishes by `--limit`.
//!
//! An event is a change of one input bit between consecutive TAS entries (entry 0 is compared
//! with "nothing held"). Moving an event to entry t' re-times only that edge: the bit keeps its
//! old value up to t' - 1 and its new value from t'. The window of an event is the contiguous
//! range of t' around its original entry for which the string still finishes (any finish frame
//! up to `limit`). An event is frame-perfect when its window is a single entry. The window stops
//! at the neighbouring events of the same bit (moving past them would merge edges).
use super::sim::{read_tas, Sim};
use serde_json::json;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

/// Verification replays (windows, canonicalize) use a long hang guard so that their verdicts do
/// not depend on CPU load, unlike the 500 ms guard of the searches.
const VERIFY_HANG_MS: u32 = 20_000;

pub const KEYS: [&str; 8] = ["left", "right", "up", "down", "space", "shift", "ctrl", "z"];

#[derive(Clone, Copy, Debug)]
pub struct Edge {
    pub bit: u8,
    /// TAS entry (0-based) where the bit takes its new value.
    pub at: usize,
    pub press: bool,
    /// Allowed re-timing range (inclusive) before neighbouring edges of the same bit.
    pub lo: usize,
    pub hi: usize,
}

pub fn edges(masks: &[u8]) -> Vec<Edge> {
    let mut out = Vec::new();
    for bit in 0..8u8 {
        let mut at: Vec<usize> = Vec::new();
        let mut prev = 0u8;
        for (i, &m) in masks.iter().enumerate() {
            let v = (m >> bit) & 1;
            if v != prev {
                at.push(i);
            }
            prev = v;
        }
        for (k, &t) in at.iter().enumerate() {
            let lo = if k == 0 { 0 } else { at[k - 1] + 1 };
            // The last edge may move to the end: a release then happens with the implicit
            // all-zero input after the string, and a press drops out.
            let hi = if k + 1 < at.len() { at[k + 1] - 1 } else { masks.len() };
            out.push(Edge { bit, at: t, press: (masks[t] >> bit) & 1 == 1, lo, hi });
        }
    }
    out.sort_by_key(|e| (e.at, e.bit));
    out
}

/// `masks` with edge `e` moved to entry `to`.
pub fn moved(masks: &[u8], e: &Edge, to: usize) -> Vec<u8> {
    let mut m = masks.to_vec();
    let b = 1u8 << e.bit;
    let new = if e.press { b } else { 0 };
    let old = b ^ new;
    if to < e.at {
        for x in &mut m[to..e.at] {
            *x = (*x & !b) | new;
        }
    } else {
        for x in &mut m[e.at..to] {
            *x = (*x & !b) | old;
        }
    }
    m
}

/// Replays from snapshots taken every `stride` TAS entries of one base string.
pub struct Replayer {
    snaps: Vec<Sim>,
    stride: usize,
    pub limit: u32,
}

impl Replayer {
    pub fn new(level: u32, character: u32, masks: &[u8], stride: usize, limit: u32) -> Result<Self, String> {
        if masks.is_empty() {
            return Err("empty input string".into());
        }
        let _guard = super::sim::HangOverride::new(VERIFY_HANG_MS);
        let mut s = Sim::new(level, character)?;
        s.masks = masks.to_vec();
        let mut snaps = Vec::new();
        while (s.replay.iteration as usize) < masks.len() {
            let i = s.replay.iteration as usize;
            if i % stride == 0 && snaps.len() == i / stride {
                snaps.push(s.fork());
            }
            let f = s.step(0);
            if f.port_abort || s.replay.finish_frame != 0 || !f.alive_after {
                break;
            }
        }
        Ok(Replayer { snaps, stride, limit })
    }

    /// Finish frame of `masks` (which must equal the base string before entry `from`), or 0.
    pub fn finish(&self, work: &mut Sim, masks: &[u8], from: usize) -> u32 {
        let _guard = super::sim::HangOverride::new(VERIFY_HANG_MS);
        let k = (from / self.stride).min(self.snaps.len() - 1);
        work.restore(&self.snaps[k]);
        work.masks.clear();
        work.masks.extend_from_slice(masks);
        while work.frame() < self.limit {
            let f = work.step(0);
            if work.replay.finish_frame != 0 {
                return work.replay.finish_frame;
            }
            if f.port_abort || !f.alive_after {
                return 0;
            }
        }
        0
    }

    pub fn template(&self) -> &Sim {
        &self.snaps[0]
    }
}

pub struct Window {
    pub edge: Edge,
    pub earliest: usize,
    pub latest: usize,
}

/// Window of every edge (searched outward up to `reach` entries each way).
pub fn windows(rep: &Replayer, masks: &[u8], reach: usize, threads: usize) -> Vec<Window> {
    let es = edges(masks);
    let next = AtomicUsize::new(0);
    let out = Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for _ in 0..threads {
            let (es, next, out) = (&es, &next, &out);
            std::thread::Builder::new()
                .stack_size(16 << 20)
                .spawn_scoped(scope, move || {
                    let mut work = rep.template().fork();
                    loop {
                        let i = next.fetch_add(1, Ordering::Relaxed);
                        if i >= es.len() {
                            break;
                        }
                        let e = es[i];
                        let mut earliest = e.at;
                        while earliest > e.lo && e.at - (earliest - 1) <= reach {
                            let m = moved(masks, &e, earliest - 1);
                            if rep.finish(&mut work, &m, earliest - 1) == 0 {
                                break;
                            }
                            earliest -= 1;
                        }
                        let mut latest = e.at;
                        while latest < e.hi && latest + 1 - e.at <= reach {
                            let m = moved(masks, &e, latest + 1);
                            if rep.finish(&mut work, &m, e.at) == 0 {
                                break;
                            }
                            latest += 1;
                        }
                        out.lock().unwrap().push(Window { edge: e, earliest, latest });
                    }
                })
                .unwrap();
        }
    });
    let mut v = out.into_inner().unwrap();
    v.sort_by_key(|w| (w.edge.at, w.edge.bit));
    v
}

pub fn run(level: u32, character: u32, tas: &str, limit: u32, reach: usize, threads: usize) -> Result<(), String> {
    let masks = read_tas(tas)?;
    let rep = Replayer::new(level, character, &masks, 8, limit)?;
    let mut probe = rep.template().fork();
    let base = rep.finish(&mut probe, &masks, 0);
    if base == 0 {
        return Err(format!("{tas} does not finish by frame {limit}"));
    }
    let ws = windows(&rep, &masks, reach, threads);
    let perfect = ws.iter().filter(|w| w.earliest == w.latest).count();
    for w in &ws {
        println!(
            "{}",
            json!({"frame": w.edge.at + 1, "key": KEYS[w.edge.bit as usize], "press": w.edge.press,
                "window": w.latest - w.earliest + 1, "earliest": w.earliest + 1, "latest": w.latest + 1})
        );
    }
    println!("{}", json!({"tas": tas, "entries": masks.len(), "finish": base, "limit": limit, "events": ws.len(), "frame_perfect": perfect}));
    Ok(())
}

/// Post-eject pose of a mask by key priority: ctrl 7 > shift 6 > up/down 3/4 > left/right 1/2.
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

/// Priority level of a pose (poses on one level need two edges to swap).
fn pose_level(p: u8) -> u8 {
    match p {
        0 => 0,
        1 | 2 => 1,
        3 | 4 => 2,
        6 => 3,
        _ => 4,
    }
}

fn pose_step(a: u8, b: u8) -> usize {
    if a == b {
        0
    } else if a != 0 && b != 0 && pose_level(a) == pose_level(b) {
        2
    } else {
        1
    }
}

/// Direction of an opposing key pair: 0 none (neither or both), 1 first, 2 second.
fn axis(m: u8, lo: u8, hi: u8) -> u8 {
    match ((m >> lo) & 1, (m >> hi) & 1) {
        (1, 0) => 1,
        (0, 1) => 2,
        _ => 0,
    }
}

fn axis_step(a: u8, b: u8) -> usize {
    if a == b {
        0
    } else if a != 0 && b != 0 {
        2
    } else {
        1
    }
}

/// Frame-perfect inputs a string needs at least, from its per-frame effective input classes
/// alone (no simulation). Every change of an effective aspect needs one exact edge:
/// - before eject: lean (left/right) and drive (up/down) axis changes (a direct swap costs 2), a
///   jump press, and the eject itself;
/// - the frame after eject: a shift/ctrl pose and the grab can be pre-held (free);
/// - after eject: a pose change (2 between poses of the same priority level), a grab toggle, and
///   the cart-motor latch (the first up == down frame if the motor was on at eject).
/// Equals `events` on canonicalized finishes (chaos makes every effective change exact); on raw
/// strings it is a lower bound.
pub fn aspect_cost(ms: &[u8]) -> usize {
    let e = ms.iter().position(|m| m & 128 != 0);
    let mut c = 0;
    let mut prev = 0u8;
    let mut motor_on = false;
    for (i, &m) in ms.iter().enumerate() {
        match e {
            Some(e) if i > e => {
                if i == e + 1 {
                    let p0 = pose(ms[e] & 0b1111);
                    let p1 = pose(m);
                    c += if p1 == 6 || p1 == 7 { 0 } else { pose_step(p0, p1) };
                    motor_on = axis(ms[e], 2, 3) != 0;
                } else {
                    c += pose_step(pose(prev), pose(m));
                    c += usize::from((m ^ prev) & 16 != 0);
                }
                if motor_on && axis(m, 2, 3) == 0 {
                    c += 1;
                    motor_on = false;
                }
            }
            _ => {
                c += axis_step(axis(prev, 0, 1), axis(m, 0, 1));
                c += axis_step(axis(prev, 2, 3), axis(m, 2, 3));
                if Some(i) != e && m & 16 != 0 && prev & 16 == 0 {
                    c += 1;
                }
                if Some(i) == e {
                    c += 1;
                }
            }
        }
        prev = m;
    }
    c
}
