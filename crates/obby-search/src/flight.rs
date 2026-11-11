//! `flight`: map which open-sky states finish. From a replay state, teleport Dad's whole jointed
//! component (BFS over joint edges from the chest, dynamic bodies only) so the chest sits at a
//! sampled point p, give every moved body the sampled velocity v (no spin), and play the
//! unchanged engine to frame 299 with no input. Counts finishes and prints the (p, v) that do.
//! This is an experiment on reachable targets, not a game input.
use crate::sim::{chest, chest_body, Sim, Skyline};
use crate::window::Rng;
use hw_core::NONE;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;

fn component(s: &Sim) -> Vec<hw_core::H> {
    let w = &s.ctx.world;
    let start = chest_body(&s.ctx);
    let mut out = vec![start];
    let mut i = 0;
    while i < out.len() {
        let b = out[i];
        i += 1;
        let mut je = w.body(b).joint_list;
        let mut g = 0;
        while je != NONE && g < 256 {
            g += 1;
            let e = w.joint_edge(je);
            let o = e.other;
            if o != NONE && w.body(o).inv_mass > 0.0 && !out.contains(&o) {
                out.push(o);
            }
            je = e.next;
        }
    }
    out
}

fn teleport(s: &mut Sim, bodies: &[hw_core::H], d: [f64; 2], v: [f64; 2]) {
    for &b in bodies {
        let bd = s.ctx.world.body_mut(b);
        bd.sweep.c.x += d[0];
        bd.sweep.c.y += d[1];
        bd.sweep.c0.x += d[0];
        bd.sweep.c0.y += d[1];
        bd.xf.position.x += d[0];
        bd.xf.position.y += d[1];
        bd.linear_velocity.x = v[0];
        bd.linear_velocity.y = v[1];
        bd.angular_velocity = 0.0;
    }
}

pub struct Region {
    pub x: [f64; 2],
    pub y: [f64; 2],
    pub speed: [f64; 2],
    pub angle: [f64; 2],
}

pub fn flight_cmd(tas: &str, frame: u32, n: u64, threads: usize, seed: u64, r: Region) -> Result<(), String> {
    let mut base = Sim::new(crate::LEVEL, crate::CHAR)?;
    base.masks = crate::sim::read_tas(tas)?;
    while base.frame() < frame {
        base.step(0);
    }
    let sky = Skyline::new(&base.ctx);
    let comp = component(&base);
    eprintln!("flight: base frame {} chest {:?} alive {} component {} bodies", base.frame(), chest(&base.ctx), !base.ctx.level.dead.get(), comp.len());
    let next = AtomicU64::new(0);
    let wins = Mutex::new(Vec::new());
    let alive_end = AtomicU64::new(0);
    std::thread::scope(|scope| {
        for t in 0..threads {
            let (next, wins, base, comp, sky, r, alive_end) = (&next, &wins, &base, &comp, &sky, &r, &alive_end);
            std::thread::Builder::new().stack_size(32 << 20).spawn_scoped(scope, move || {
                let mut rng = Rng((seed.wrapping_add(t as u64 + 1)).wrapping_mul(0xd1b5_4a32_d192_ed03) ^ 0x2545_f491_4f6c_dd1d);
                let mut s = base.fork();
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    if i >= n {
                        break;
                    }
                    let p = [r.x[0] + rng.unit() * (r.x[1] - r.x[0]), r.y[0] + rng.unit() * (r.y[1] - r.y[0])];
                    if !sky.above(p) {
                        continue;
                    }
                    let sp = (r.speed[0].ln() + rng.unit() * (r.speed[1].ln() - r.speed[0].ln())).exp();
                    let an = (r.angle[0] + rng.unit() * (r.angle[1] - r.angle[0])).to_radians();
                    let v = [sp * an.cos(), -sp * an.sin()];
                    s.restore(base);
                    let c = chest(&s.ctx);
                    teleport(&mut s, comp, [p[0] - c[0], p[1] - c[1]], v);
                    let mut fin = 0;
                    let mut alive = true;
                    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        while s.frame() < 299 {
                            let f = s.step(0);
                            if f.finished && f.alive_before {
                                fin = f.frame;
                                break;
                            }
                            if f.site == 0 && !f.alive_after {
                                alive = false;
                                break;
                            }
                        }
                    }));
                    if r.is_err() {
                        std::mem::forget(std::mem::replace(&mut s, base.fork()));
                        continue;
                    }
                    if alive {
                        alive_end.fetch_add(1, Ordering::Relaxed);
                    }
                    if fin != 0 {
                        wins.lock().unwrap().push((p, v, fin));
                    }
                }
            }).unwrap();
        }
    });
    let w = wins.into_inner().unwrap();
    eprintln!("flight: {} samples, {} finishes, {} alive at end", n, w.len(), alive_end.load(Ordering::Relaxed));
    for (p, v, f) in &w {
        println!("{{\"p\":[{:.2},{:.2}],\"v\":[{:.1},{:.1}],\"speed\":{:.1},\"angle\":{:.1},\"finish\":{}}}", p[0], p[1], v[0], v[1], (v[0] * v[0] + v[1] * v[1]).sqrt(), (-v[1]).atan2(v[0]).to_degrees(), f);
    }
    Ok(())
}

/// One teleported flight from `base` (a clean state at the same frame): chest to p, component
/// velocity v. Returns the finish frame (0 = none).
pub fn fly(base: &Sim, work: &mut Sim, comp: &[hw_core::H], p: [f64; 2], v: [f64; 2]) -> u32 {
    work.restore(base);
    let c = chest(&work.ctx);
    teleport(work, comp, [p[0] - c[0], p[1] - c[1]], v);
    while work.frame() < 299 {
        let f = work.step(0);
        if f.finished && f.alive_before {
            return f.frame;
        }
        if f.site == 0 && !f.alive_after {
            return 0;
        }
    }
    0
}

/// `driftcheck`: for each onset lineage in --files, hold the lock with the lineage's last input
/// and, at every lock frame with the chest in open sky inside the world, test whether a flight
/// from that chest state (teleported into the clean --base replay at the same frame) finishes.
pub fn driftcheck_cmd(files: &[String], base_tas: &str, threads: usize) -> Result<(), String> {
    use std::collections::HashSet;
    let mut work: Vec<Vec<u8>> = Vec::new();
    let mut uniq = HashSet::new();
    for f in files {
        for line in std::fs::read_to_string(f).map_err(|e| format!("{f}: {e}"))?.lines() {
            if line.contains("\"kick\":") {
                // onset-search rows: skip tiny kicks
                if let Some(i) = line.find("\"kick\":") {
                    let v: f64 = line[i + 7..].split(|c| c == ',' || c == '}').next().and_then(|x| x.parse().ok()).unwrap_or(0.0);
                    if v <= 2.0 {
                        continue;
                    }
                }
            }
            if let Some(m) = crate::tas_field(line) {
                if uniq.insert(m.clone()) {
                    work.push(m);
                }
            }
        }
    }
    eprintln!("driftcheck: {} lineages", work.len());
    // Clean bases per frame 250..299 from the base replay.
    let mut b = Sim::new(crate::LEVEL, crate::CHAR)?;
    b.masks = crate::sim::read_tas(base_tas)?;
    let mut bases: Vec<Option<Sim>> = (0..300).map(|_| None).collect();
    while b.frame() < 299 {
        b.step(0);
        if b.frame() >= 250 {
            bases[b.frame() as usize] = Some(b.fork());
        }
    }
    let comp = component(bases[255].as_ref().unwrap());
    let sky = Skyline::new(&b.ctx);
    let next = AtomicU64::new(0);
    let out = Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for _ in 0..threads {
            let (next, out, work, bases, comp, sky) = (&next, &out, &work, &bases, &comp, &sky);
            std::thread::Builder::new().stack_size(32 << 20).spawn_scoped(scope, move || {
                let mut s = Sim::new(crate::LEVEL, crate::CHAR).unwrap();
                let mut fw = Sim::new(crate::LEVEL, crate::CHAR).unwrap();
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed) as usize;
                    if i >= work.len() {
                        break;
                    }
                    let tpl = Sim::new(crate::LEVEL, crate::CHAR).unwrap();
                    s.restore(&tpl);
                    s.masks = work[i].clone();
                    let hold = *work[i].last().unwrap() & !(32 | 64 | 128);
                    let mut onset = 0;
                    while s.frame() < 299 {
                        let f = s.step(hold);
                        if f.port_abort || (f.site == 0 && !f.alive_after) {
                            break;
                        }
                        if f.site == 2 {
                            onset = f.frame;
                            break;
                        }
                    }
                    if onset == 0 {
                        continue;
                    }
                    s.masks.truncate(s.replay.iteration as usize);
                    let (mut tested, mut wins) = (0, Vec::new());
                    while s.frame() < 298 {
                        let f = s.step(hold);
                        if f.port_abort || f.site == 0 {
                            break;
                        }
                        let c = f.chest_after;
                        let v = f.chest_v;
                        if sky.above(c) && crate::onset::neck_gap(&s.ctx) < 0.45 {
                            if let Some(bs) = bases[f.frame as usize].as_ref() {
                                tested += 1;
                                let fin = fly(bs, &mut fw, comp, c, v);
                                if fin != 0 {
                                    wins.push((f.frame, c, v, fin));
                                }
                            }
                        }
                    }
                    out.lock().unwrap().push((onset, tested, wins, crate::sim::tas_text(&s.masks)));
                }
            }).unwrap();
        }
    });
    let out = out.into_inner().unwrap();
    let (mut lines, mut tested_lines, mut win_lines) = (0, 0, 0);
    for (onset, tested, wins, tas) in &out {
        lines += 1;
        if *tested > 0 {
            tested_lines += 1;
        }
        if !wins.is_empty() {
            win_lines += 1;
            for (fr, c, v, fin) in wins {
                println!("{{\"onset\":{},\"frame\":{},\"chest\":[{:.2},{:.2}],\"v\":[{:.1},{:.1}],\"finish\":{},\"tas\":\"{}\"}}", onset, fr, c[0], c[1], v[0], v[1], fin, tas);
            }
        }
    }
    eprintln!("driftcheck: {} onset lineages, {} with in-sky lock frames, {} with a finishing drift state", lines, tested_lines, win_lines);
    Ok(())
}
