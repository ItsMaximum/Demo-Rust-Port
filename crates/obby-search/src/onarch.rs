//! `onarch`: Go-Explore archive over pairmax onsets. A cell is the onset frame bucket and Dad's
//! chest state at the onset (position, velocity direction and speed buckets); each cell keeps the
//! lineage (input up to the onset) whose ballistic path comes closest to the finish region.
//! Trials pick a cell (uniformly half the time, otherwise biased to good scores), re-roll inputs
//! from a cut in the last `--window` entries and play on to the next onset (or `--max-frame`).
//! Cells whose ballistic distance is under `--sweep` metres also get the held-Space aftermath
//! sweep (a finish there is a winner).
use crate::sim::{tas_text, Sim, Skyline};
use crate::window::{ballistic_finish_distance, Rng};
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
    pub window: usize,
    pub extra: usize,
    pub max_frame: u32,
    pub temp: f64,
    pub sweep: f64,
    /// Pass mode: after the onset hold a random input for a random length, then play to frame
    /// 299; score passes into open sky by their closest finish approach.
    pub pass: bool,
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
    m | (orig & (32 | 64 | 128))
}

static FLIGHT_MAPS: std::sync::OnceLock<Vec<(String, u32)>> = std::sync::OnceLock::new();

/// Register the flight-map files (JSON lines written by `flight`, each with the base frame the
/// teleports started from): `--flight-map file:base,file:base`.
pub fn set_flight_maps(spec: &str) -> Result<(), String> {
    let mut maps = Vec::new();
    for item in spec.split(',').filter(|x| !x.is_empty()) {
        let (file, base) = item.rsplit_once(':').ok_or_else(|| format!("--flight-map entries are file:base_frame, got {item}"))?;
        maps.push((file.to_string(), base.parse::<u32>().map_err(|_| format!("bad base frame in {item}"))?));
    }
    FLIGHT_MAPS.set(maps).map_err(|_| "flight maps already set".to_string())
}

/// Flight-map finishing samples (position, velocity, flight frames to the finish) loaded from the
/// files given with `--flight-map`.
fn flight_samples() -> &'static Vec<([f64; 2], [f64; 2], u32)> {
    static S: std::sync::OnceLock<Vec<([f64; 2], [f64; 2], u32)>> = std::sync::OnceLock::new();
    S.get_or_init(|| {
        let mut out = Vec::new();
        let maps = FLIGHT_MAPS.get().cloned().unwrap_or_default();
        if maps.is_empty() {
            eprintln!("obby-search: no --flight-map given; flight-map scoring has no finishing samples");
        }
        for (file, base) in maps.iter().map(|(f, b)| (f.as_str(), *b)) {
            let Ok(text) = std::fs::read_to_string(file) else {
                eprintln!("obby-search: cannot read flight map {file}");
                continue;
            };
            for line in text.lines() {
                let num = |key: &str| -> Option<[f64; 2]> {
                    let i = line.find(&format!("\"{}\":[", key))? + key.len() + 4;
                    let j = line[i..].find(']')? + i;
                    let mut it = line[i..j].split(',').map(|x| x.trim().parse::<f64>().ok());
                    Some([it.next()??, it.next()??])
                };
                let fin = line.find("\"finish\":").and_then(|i| line[i + 9..].trim_end_matches('}').trim().parse::<u32>().ok());
                if let (Some(p), Some(v), Some(f)) = (num("p"), num("v"), fin) {
                    if f > base {
                        out.push((p, v, f - base));
                    }
                }
            }
        }
        eprintln!("onarch: {} flight-map finishing samples", out.len());
        out
    })
}

/// Distance from a chest state at `frame` to the nearest time-feasible finishing sample.
fn flight_distance(p: [f64; 2], v: [f64; 2], frame: u32) -> f64 {
    flight_distance_by(p, v, frame, 299)
}

/// As `flight_distance`, with the finish due by `deadline` (frame 299 or Dad's bleed-out).
pub fn flight_distance_by(p: [f64; 2], v: [f64; 2], frame: u32, deadline: u32) -> f64 {
    let mut best = f64::INFINITY;
    for (ps, vs, fl) in flight_samples().iter() {
        if frame + fl > deadline.min(299) {
            continue;
        }
        let d = (p[0] - ps[0]).hypot(p[1] - ps[1]) + 0.6 * (v[0] - vs[0]).hypot(v[1] - vs[1]);
        if d < best {
            best = d;
        }
    }
    best
}

/// False if any body has a non-finite or absurd position (states where the port's polygon
/// collision panics on a NaN-selected edge).
fn sane(s: &Sim) -> bool {
    let w = &s.ctx.world;
    let mut b = w.body_list;
    let mut guard = 0;
    while b != hw_core::NONE && guard < hw_core::b2::state::MAX_BODIES {
        guard += 1;
        let bd = w.body(b);
        let p = bd.sweep.c;
        if !(p.x.is_finite() && p.y.is_finite()) {
            return false;
        }
        b = bd.next;
    }
    true
}

fn cell(frame: u32, c: [f64; 2], v: [f64; 2]) -> u64 {
    let sp = v[0].hypot(v[1]);
    let ang = (-v[1]).atan2(v[0]).to_degrees(); // up = positive
    let b = |x: f64, w: f64, lo: f64, n: u64| -> u64 { (((x - lo) / w).floor().max(0.0) as u64).min(n) };
    b(frame as f64, 6.0, 0.0, 63)
        | (b(c[0], 4.0, -8.0, 63) << 6)
        | (b(c[1], 4.0, 40.0, 31) << 12)
        | (b(ang, 30.0, -180.0, 15) << 17)
        | (b(sp.min(400.0), 10.0, 0.0, 63) << 21)
}

/// Held-Space aftermath sweep from a state just past the onset: returns (closest approach,
/// finish frame, input).
fn sweep(s: &Sim, br: &mut Sim) -> (f64, u32, Vec<u8>) {
    const HOLDS: [(u8, u32); 8] = [(0, 0), (16, 2), (16, 5), (16, 10), (16, 18), (16, 30), (24, 8), (18, 8)];
    let mut near = f64::INFINITY;
    for &(hm, h) in HOLDS.iter() {
        br.restore(s);
        let i0 = br.replay.iteration as usize;
        br.masks.truncate(i0);
        for _ in 0..h {
            br.masks.push(hm);
        }
        let t0 = Instant::now();
        while br.frame() < 299 {
            let f = br.step(0);
            if f.port_abort || t0.elapsed().as_millis() > 2500 {
                break;
            }
            if f.finished && f.alive_before {
                return (0.0, f.frame, br.masks[..br.replay.iteration as usize].to_vec());
            }
            if f.site != 0 {
                continue;
            }
            if !f.alive_after {
                break;
            }
            near = near.min(crate::window::finish_distance(f.chest_after));
        }
    }
    (near, 0, Vec::new())
}

pub fn run(cfg: Config) -> Result<(), String> {
    std::fs::create_dir_all(&cfg.run_dir).map_err(|e| e.to_string())?;
    let template = Sim::new(crate::LEVEL, crate::CHAR)?;
    let sky = Skyline::new(&template.ctx);
    let archive: Mutex<HashMap<u64, Member>> = Mutex::new(HashMap::new());
    let stop = AtomicBool::new(false);
    let (trials, onsets, frames, sweeps) = (AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0));
    let best = Mutex::new((f64::INFINITY, f64::INFINITY));
    let events = Mutex::new(std::fs::OpenOptions::new().create(true).append(true).open(cfg.run_dir.join("events.jsonl")).map_err(|e| e.to_string())?);
    let start = Instant::now();
    let deadline = start + Duration::from_secs_f64(cfg.hours * 3600.0);
    std::thread::scope(|scope| {
        for w in 0..cfg.threads {
            let (archive, stop, trials, onsets, frames, sweeps, best, events, template, cfg, sky) =
                (&archive, &stop, &trials, &onsets, &frames, &sweeps, &best, &events, &template, &cfg, &sky);
            std::thread::Builder::new().stack_size(96 << 20).spawn_scoped(scope, move || {
                let mut rng = Rng((cfg.seed.wrapping_add(w as u64 + 7)).wrapping_mul(0xd1b5_4a32_d192_ed03) ^ 0x2545_f491_4f6c_dd1d);
                let mut s = template.fork();
                let mut br = template.fork();
                while !stop.load(Ordering::Relaxed) {
                    // Parent: a seed (always while the archive is small, else 1 in 10) or a cell.
                    let parent = {
                        let mut a = archive.lock().unwrap();
                        if a.len() < 20 || rng.below(10) == 0 {
                            drop(a);
                            Member { masks: cfg.seeds[rng.below(cfg.seeds.len() as u64) as usize].clone(), score: 0.0, visits: 0 }
                        } else {
                            let keys: Vec<u64> = a.keys().copied().collect();
                            let k = if rng.below(2) == 0 {
                                keys[rng.below(keys.len() as u64) as usize]
                            } else {
                                let mx = a.values().map(|m| m.score).fold(f64::NEG_INFINITY, f64::max);
                                let wts: Vec<f64> = keys.iter().map(|k| ((a[k].score - mx) / cfg.temp).exp() / (1.0 + a[k].visits as f64).sqrt()).collect();
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
                            m.clone()
                        }
                    };
                    trials.fetch_add(1, Ordering::Relaxed);
                    let l = parent.masks.len();
                    if l < cfg.window + 2 {
                        continue;
                    }
                    let cut = l - 1 - rng.below(cfg.window as u64) as usize;
                    let mut masks = parent.masks[..cut].to_vec();
                    masks.push(loop {
                        let c = choose(&mut rng, parent.masks[cut]);
                        if c != parent.masks[cut] {
                            break c;
                        }
                    });
                    for t in cut + 1..l {
                        masks.push(if rng.below(10) < 6 { parent.masks[t] } else { choose(&mut rng, parent.masks[t]) });
                    }
                    let tail = *masks.last().unwrap();
                    for _ in 0..rng.below(cfg.extra as u64 + 1) {
                        masks.push(if rng.below(10) < 6 { tail } else { choose(&mut rng, tail) });
                    }
                    s.restore(template);
                    s.masks = masks.clone();
                    let mut result = None;
                    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        while s.frame() < cfg.max_frame {
                            if !sane(&s) {
                                break;
                            }
                            let f = s.step(tail);
                            frames.fetch_add(1, Ordering::Relaxed);
                            if f.port_abort || f.step_us > 80_000 {
                                break;
                            }
                            if f.site == 2 && f.alive_before {
                                result = Some(f);
                                break;
                            }
                            if f.site != 0 || !f.alive_after {
                                break;
                            }
                        }
                    }));
                    if r.is_err() {
                        // Never drop a state that panicked mid-step: leak it.
                        std::mem::forget(std::mem::replace(&mut s, template.fork()));
                        std::mem::forget(std::mem::replace(&mut br, template.fork()));
                        continue;
                    }
                    let Some(f) = result else { continue };
                    onsets.fetch_add(1, Ordering::Relaxed);
                    if cfg.pass {
                        // Hold an input after the onset, then release; find the first normal frame
                        // with the chest in open sky and the closest approach after it.
                        const HM: [u8; 7] = [16, 16, 17, 18, 20, 24, 0];
                        let hm = HM[rng.below(HM.len() as u64) as usize];
                        let h = rng.below(41) as usize;
                        let i0 = s.replay.iteration as usize;
                        s.masks.truncate(i0);
                        for _ in 0..h {
                            s.masks.push(hm);
                        }
                        let mut pass_at = 0usize;
                        let mut pass_state = ([0.0f64; 2], [0.0f64; 2]);
                        let mut pass_frame = 0u32;
                        let mut pass_deadline = 299u32;
                        let onset_rem = {
                            let d = &s.ctx.character.primary;
                            if d.dying.get() { (150 - d.bleed_counter).max(0) as u32 } else { 1000 }
                        };
                        let mut near = f64::INFINITY;
                        let mut fin = 0u32;
                        let mut min_gap = f64::INFINITY;
                        let t0 = Instant::now();
                        let rr = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            while s.frame() < 299 {
                                if !sane(&s) {
                                    break;
                                }
                                let g = s.step(0);
                                frames.fetch_add(1, Ordering::Relaxed);
                                if g.port_abort || t0.elapsed().as_millis() > 3000 {
                                    break;
                                }
                                if g.finished && g.alive_before {
                                    fin = g.frame;
                                    break;
                                }
                                if g.alive_after && pass_at == 0 {
                                    min_gap = min_gap.min(sky.gap(g.chest_after));
                                }
                                if g.site != 0 {
                                    continue;
                                }
                                if !g.alive_after {
                                    break;
                                }
                                if pass_at == 0 && sky.above(g.chest_after) {
                                    pass_at = s.replay.iteration as usize;
                                    pass_state = (g.chest_after, g.chest_v);
                                    pass_frame = g.frame;
                                    let d = &s.ctx.character.primary;
                                    pass_deadline = if d.dying.get() { g.frame + (150 - d.bleed_counter).max(0) as u32 } else { 299 };
                                }
                                if pass_at != 0 {
                                    near = near.min(crate::window::finish_distance(g.chest_after));
                                } else if s.frame() > f.frame + 60 {
                                    break;
                                }
                            }
                        }));
                        if rr.is_err() {
                            std::mem::forget(std::mem::replace(&mut s, template.fork()));
                            continue;
                        }
                        let full = s.masks[..(s.replay.iteration as usize).min(s.masks.len())].to_vec();
                        if fin != 0 {
                            writeln!(events.lock().unwrap(), "{{\"kind\":\"finish\",\"frame\":{},\"tas\":\"{}\"}}", fin, tas_text(&full)).ok();
                            std::fs::write(cfg.run_dir.join("winner-1.tas"), tas_text(&full)).ok();
                            stop.store(true, Ordering::Relaxed);
                            return;
                        }
                        if pass_at == 0 {
                            // Stepping stone: the onset state, scored by how close the chest got to
                            // open sky after it.
                            let score = -300.0 - 10.0 * min_gap.max(0.0).min(60.0) - if onset_rem < 130 { 300.0 } else { 0.0 };
                            let key = (1u64 << 61) | cell(f.frame, f.chest_after, f.chest_v);
                            let member_masks = s.masks[..i0.min(s.masks.len())].to_vec();
                            let mut a = archive.lock().unwrap();
                            match a.get_mut(&key) {
                                Some(m) if m.score >= score => {}
                                Some(m) => {
                                    let vv = m.visits;
                                    *m = Member { masks: member_masks, score, visits: vv };
                                }
                                None => {
                                    a.insert(key, Member { masks: member_masks, score, visits: 0 });
                                }
                            }
                            continue;
                        }
                        let fd = flight_distance_by(pass_state.0, pass_state.1, pass_frame, pass_deadline).min(100.0);
                        let score = 1000.0 - 10.0 * fd.min(100.0) - 0.1 * near.min(100.0);
                        {
                            let mut b = best.lock().unwrap();
                            if 1000.0 - score < b.1 {
                                b.1 = 1000.0 - score;
                                std::fs::write(cfg.run_dir.join("best.tas"), tas_text(&full)).ok();
                                writeln!(events.lock().unwrap(), "{{\"kind\":\"pass\",\"onset\":{},\"fd\":{:.2},\"near\":{:.2},\"p\":[{:.2},{:.2}],\"v\":[{:.1},{:.1}],\"tas\":\"{}\"}}", f.frame, fd, near, pass_state.0[0], pass_state.0[1], pass_state.1[0], pass_state.1[1], tas_text(&full)).ok();
                            }
                        }
                        let (p, v) = pass_state;
                        let b3 = |x: f64, lo: f64, n: u64| -> u64 { (((x - lo) / 3.0).floor().max(0.0) as u64).min(n) };
                        let key = (1u64 << 62) | b3(p[0], -4.0, 63) | (b3(p[1], 30.0, 31) << 6) | (b3(v[0], -60.0, 63) << 11) | (b3(v[1], -60.0, 63) << 17);
                        let member_masks = s.masks[..pass_at.min(s.masks.len())].to_vec();
                        let mut a = archive.lock().unwrap();
                        match a.get_mut(&key) {
                            Some(m) if m.score >= score => {}
                            Some(m) => {
                                let vv = m.visits;
                                *m = Member { masks: member_masks, score, visits: vv };
                            }
                            None => {
                                a.insert(key, Member { masks: member_masks, score, visits: 0 });
                            }
                        }
                        continue;
                    }
                    let used = s.masks[..s.replay.iteration as usize].to_vec();
                    let bfd = ballistic_finish_distance(f.chest_after, f.chest_v, f.frame);
                    let mut score = -bfd.min(500.0);
                    if bfd < cfg.sweep {
                        sweeps.fetch_add(1, Ordering::Relaxed);
                        let rr = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| sweep(&s, &mut br)));
                        match rr {
                            Ok((near, fin, win)) => {
                                if fin != 0 {
                                    writeln!(events.lock().unwrap(), "{{\"kind\":\"finish\",\"frame\":{},\"tas\":\"{}\"}}", fin, tas_text(&win)).ok();
                                    std::fs::write(cfg.run_dir.join("winner-1.tas"), tas_text(&win)).ok();
                                    stop.store(true, Ordering::Relaxed);
                                    return;
                                }
                                score = score.max(-near.min(500.0)) + 0.01;
                                let mut b = best.lock().unwrap();
                                if near < b.1 {
                                    b.1 = near;
                                    writeln!(events.lock().unwrap(), "{{\"kind\":\"sweep\",\"frame\":{},\"bfd\":{:.2},\"near\":{:.2},\"chest\":[{:.2},{:.2}],\"cv\":[{:.1},{:.1}],\"tas\":\"{}\"}}", f.frame, bfd, near, f.chest_after[0], f.chest_after[1], f.chest_v[0], f.chest_v[1], tas_text(&used)).ok();
                                }
                            }
                            Err(_) => {
                                std::mem::forget(std::mem::replace(&mut br, template.fork()));
                            }
                        }
                    }
                    {
                        let mut b = best.lock().unwrap();
                        if bfd < b.0 {
                            b.0 = bfd;
                            std::fs::write(cfg.run_dir.join("best.tas"), tas_text(&used)).ok();
                        }
                    }
                    let key = cell(f.frame, f.chest_after, f.chest_v);
                    let mut a = archive.lock().unwrap();
                    match a.get_mut(&key) {
                        Some(m) if m.score >= score => {}
                        Some(m) => {
                            let v = m.visits;
                            *m = Member { masks: used, score, visits: v };
                        }
                        None => {
                            a.insert(key, Member { masks: used, score, visits: 0 });
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
                "{{\"t\":{:.0},\"fps\":{:.0},\"trials\":{},\"onsets\":{},\"sweeps\":{},\"cells\":{},\"best_bfd\":{:.2},\"best_near\":{:.2}}}",
                t,
                frames.load(Ordering::Relaxed) as f64 / t,
                trials.load(Ordering::Relaxed),
                onsets.load(Ordering::Relaxed),
                sweeps.load(Ordering::Relaxed),
                archive.lock().unwrap().len(),
                b.0,
                b.1
            );
            writeln!(st, "{line}").ok();
            eprintln!("{line}");
            if Instant::now() > deadline || cfg.run_dir.join("STOP").exists() || stop.load(Ordering::Relaxed) {
                stop.store(true, Ordering::Relaxed);
                break;
            }
        }
    });
    if let Ok(mut out) = std::fs::File::create(cfg.run_dir.join("pool.jsonl")) {
        for m in archive.lock().unwrap().values() {
            writeln!(out, "{{\"score\":{:.2},\"frames\":{},\"tas\":\"{}\"}}", m.score, m.masks.len(), tas_text(&m.masks)).ok();
        }
    }
    Ok(())
}
