//! QE state probes: torn-ligament anchors (the QE bodies) and the level's mines.
//!
//! `qescan --seed-file F --frames N`: for each lineage, report when a torn-ligament anchor first
//! whips faster than 1e3 / 1e4 / 1e5 m/s, where Dad is then, and when the start mines explode.
use crate::sim::{chest, chest_velocity, State, Sim};
use hw_core::NONE;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

/// Fastest torn-ligament anchor (both characters) and how many anchors exceed 1e4 m/s.
pub fn anchors(ctx: &State) -> (f64, u32) {
    let (mut best, mut n) = (0.0f64, 0u32);
    for ch in [&ctx.character.primary, &ctx.character.child] {
        for i in 0..ch.composite_count as usize {
            let c = ch.composites[i];
            if c.kind != 0 {
                continue;
            }
            let j = c.ends[0];
            if j == NONE || !ctx.world.joints.is_alive(j) || !ctx.world.joint(j).broken.get() {
                continue;
            }
            let a = ctx.world.joint(j).body1;
            if a == NONE || !ctx.world.bodies.is_alive(a) {
                continue;
            }
            let v = ctx.world.body(a).linear_velocity;
            let sp = (v.x * v.x + v.y * v.y).sqrt();
            if sp.is_finite() {
                best = best.max(sp);
                if sp > 1e4 {
                    n += 1;
                }
            }
        }
    }
    (best, n)
}

/// Bitmask of live mines by special index (bit i = special i is a mine whose body still exists).
pub fn mines(ctx: &State) -> u64 {
    let mut m = 0u64;
    for i in 0..(ctx.level.count as usize).min(64) {
        let sp = &ctx.level.specials[i];
        if format!("{:?}", sp.kind) == "Mine"
            && sp.body != NONE
            && ctx.world.bodies.is_alive(sp.body)
            && sp.shape != NONE
            && ctx.world.shapes.is_alive(sp.shape)
            && ctx.world.shape(sp.shape).body == sp.body
        {
            m |= 1 << i;
        }
    }
    m
}

pub fn qescan_cmd(files: &[String], frames: u32, threads: usize) -> Result<(), String> {
    let mut work: Vec<Vec<u8>> = Vec::new();
    let mut uniq = std::collections::HashSet::new();
    for f in files {
        for line in std::fs::read_to_string(f).map_err(|e| format!("{f}: {e}"))?.lines() {
            if let Some(m) = crate::tas_field(line) {
                if uniq.insert(m.clone()) {
                    work.push(m);
                }
            }
        }
    }
    let template = Sim::new(crate::LEVEL, crate::CHAR)?;
    let next = AtomicUsize::new(0);
    let out = Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for _ in 0..threads {
            let (next, out, work, template) = (&next, &out, &work, &template);
            std::thread::Builder::new().stack_size(32 << 20).spawn_scoped(scope, move || {
                let mut s = template.fork();
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    if i >= work.len() {
                        break;
                    }
                    s.restore(template);
                    s.masks = work[i].clone();
                    let mut first = [0u32; 3];
                    let mut at = [[f64::NAN; 4]; 3];
                    let mut mine_frame = 0u32;
                    let mut maxn = (0u32, 0u32);
                    let m0 = mines(&s.ctx) & 0b1100;
                    while s.frame() < frames {
                        let f = s.step(0);
                        if f.port_abort {
                            break;
                        }
                        let (sp, n) = anchors(&s.ctx);
                        if n > maxn.0 {
                            maxn = (n, f.frame);
                        }
                        for (k, th) in [1e3, 1e4, 1e5].iter().enumerate() {
                            if first[k] == 0 && sp > *th {
                                first[k] = f.frame;
                                let c = chest(&s.ctx);
                                let v = chest_velocity(&s.ctx);
                                at[k] = [c[0], c[1], v[0], v[1]];
                            }
                        }
                        if mine_frame == 0 && (mines(&s.ctx) & m0) != m0 {
                            mine_frame = f.frame;
                        }
                        if !f.alive_after {
                            break;
                        }
                    }
                    out.lock().unwrap().push((i, work[i].len(), first, at, mine_frame, maxn, s.frame()));
                }
            }).unwrap();
        }
    });
    let mut r = out.into_inner().unwrap();
    r.sort_by_key(|x| (if x.2[1] == 0 { u32::MAX } else { x.2[1] }, x.0));
    for (i, len, first, at, mine, maxn, end) in r {
        println!(
            "{{\"i\":{},\"len\":{},\"q3\":{},\"q4\":{},\"q5\":{},\"at4\":[{:.1},{:.1},{:.1},{:.1}],\"mine\":{},\"maxn\":[{},{}],\"end\":{}}}",
            i, len, first[0], first[1], first[2], at[1][0], at[1][1], at[1][2], at[1][3], mine, maxn.0, maxn.1, end
        );
    }
    Ok(())
}

/// `lstate --files F --threads N`: end-of-lineage features (chest, Dad dying/ejected, start mines,
/// torn ligaments per character, composites, child dead) for each lineage, as JSON lines.
pub fn lstate_cmd(files: &[String], threads: usize) -> Result<(), String> {
    let mut work: Vec<Vec<u8>> = Vec::new();
    let mut uniq = std::collections::HashSet::new();
    for f in files {
        for line in std::fs::read_to_string(f).map_err(|e| format!("{f}: {e}"))?.lines() {
            if let Some(m) = crate::tas_field(line) {
                if uniq.insert(m.clone()) {
                    work.push(m);
                }
            }
        }
    }
    let template = Sim::new(crate::LEVEL, crate::CHAR)?;
    let next = AtomicUsize::new(0);
    let out = Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for _ in 0..threads {
            let (next, out, work, template) = (&next, &out, &work, &template);
            std::thread::Builder::new().stack_size(32 << 20).spawn_scoped(scope, move || {
                let mut s = template.fork();
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    if i >= work.len() {
                        break;
                    }
                    s.restore(template);
                    s.masks = work[i].clone();
                    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        while (s.replay.iteration as usize) < work[i].len() {
                            let f = s.step(0);
                            if f.port_abort || !f.alive_after {
                                break;
                            }
                        }
                    }));
                    if r.is_err() {
                        std::mem::forget(std::mem::replace(&mut s, template.fork()));
                        continue;
                    }
                    let ctx = &s.ctx;
                    let mut torn = [0u32; 2];
                    let mut comps = [0u32; 2];
                    for (k, ch) in [&ctx.character.primary, &ctx.character.child].iter().enumerate() {
                        comps[k] = ch.composite_count as u32;
                        for j in 0..ch.composite_count as usize {
                            let cp = ch.composites[j];
                            if cp.kind == 0 && cp.ends[0] != NONE && ctx.world.joints.is_alive(cp.ends[0]) && ctx.world.joint(cp.ends[0]).broken.get() {
                                torn[k] += 1;
                            }
                        }
                    }
                    let c = chest(ctx);
                    let d = &ctx.character.primary;
                    let line = format!(
                        "{{\"i\":{},\"frame\":{},\"chest\":[{:.2},{:.2}],\"dying\":{},\"bleed\":{},\"ejected\":{},\"mines\":{},\"torn\":[{},{}],\"comps\":[{},{}],\"child_dead\":{},\"speed\":{:.1}}}",
                        i,
                        s.frame(),
                        c[0],
                        c[1],
                        d.dying.get(),
                        d.bleed_counter,
                        d.bicycle.ejected.get(),
                        mines(ctx) & 0b1100,
                        torn[0],
                        torn[1],
                        comps[0],
                        comps[1],
                        ctx.character.child.dead.get(),
                        anchors(ctx).0
                    );
                    out.lock().unwrap().push(line);
                }
            }).unwrap();
        }
    });
    for l in out.into_inner().unwrap() {
        println!("{l}");
    }
    Ok(())
}
