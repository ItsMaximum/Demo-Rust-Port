//! Pairmax onsets: the first pair-pool exhaustion of a lineage and what it does to Dad.
//!
//! On the onset frame the throw ends `b2World.Solve` inside the synchronize loop, so the bodies
//! after the throwing body keep stale broadphase proxies and the frame never reaches SolveTOI or
//! the character's post-step checks (joint breaks, death). The chest's whole move that frame is
//! therefore never swept against the level: if it ends in open sky with the head still on the
//! neck when the pool frees, Dad has passed the ceiling alive.
use crate::sim::{chest, tas_text, Sim, Skyline, State};
use hw_core::game::character::{HEAD, NECK};
use hw_core::NONE;
use std::collections::HashSet;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

pub fn head(ctx: &State) -> [f64; 2] {
    let h = ctx.character.primary.bodies[HEAD];
    if h == NONE || !ctx.world.bodies.is_alive(h) {
        return [f64::NAN; 2];
    }
    let p = hw_core::b2::body::get_world_center(ctx, h);
    [p.x, p.y]
}

/// Neck anchor separation (m), or NaN when the neck joint is gone or broken.
pub fn neck_gap(ctx: &State) -> f64 {
    let j = ctx.character.primary.joints[NECK];
    if j == NONE || !ctx.world.joints.is_alive(j) || ctx.world.joint(j).broken.get() {
        return f64::NAN;
    }
    let a = hw_core::b2::joints::get_anchor1(ctx, j);
    let b = hw_core::b2::joints::get_anchor2(ctx, j);
    ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt()
}

/// What one onset led to.
#[derive(Clone, Debug, Default)]
pub struct Outcome {
    pub onset: u32,
    pub p0: [f64; 2],
    pub p1: [f64; 2],
    pub h0: [f64; 2],
    pub h1: [f64; 2],
    /// Consecutive throw frames starting at the onset (capped by the horizon).
    pub lock: u32,
    /// State after the first completed frame following the lock (the first post-step checks).
    pub end_alive: bool,
    pub end_chest: [f64; 2],
    pub end_neck: f64,
    /// Frames after the onset with Dad alive (verified by a completed frame) and the chest in
    /// open sky inside the world; first such frame.
    pub sky_frames: u32,
    pub first_sky: u32,
    pub final_alive: bool,
    pub final_chest: [f64; 2],
    pub finish: u32,
    /// Masks up to and including the onset frame.
    pub masks: Vec<u8>,
}

impl Outcome {
    pub fn json(&self) -> String {
        format!(
            "{{\"onset\":{},\"p0\":[{:.3},{:.3}],\"p1\":[{:.3},{:.3}],\"h0\":[{:.3},{:.3}],\"h1\":[{:.3},{:.3}],\"lock\":{},\"end_alive\":{},\"end_chest\":[{:.3},{:.3}],\"end_neck\":{:.3},\"sky_frames\":{},\"first_sky\":{},\"final_alive\":{},\"final_chest\":[{:.3},{:.3}],\"finish\":{},\"tas\":\"{}\"}}",
            self.onset, self.p0[0], self.p0[1], self.p1[0], self.p1[1], self.h0[0], self.h0[1], self.h1[0], self.h1[1],
            self.lock, self.end_alive, self.end_chest[0], self.end_chest[1], self.end_neck, self.sky_frames, self.first_sky,
            self.final_alive, self.final_chest[0], self.final_chest[1], self.finish, tas_text(&self.masks)
        )
    }
}

/// Play `s` (positioned before its onset, inputs already in `s.masks` for the prefix) until the
/// first pairmax throw, then `horizon` more frames feeding `tail` for new input entries.
/// Returns None if Dad dies, the branch leaves verified territory, or no onset happens by
/// `max_frame`.
pub fn run_onset(s: &mut Sim, sky: &Skyline, tail: u8, horizon: u32, max_frame: u32) -> Option<Outcome> {
    let mut o = Outcome::default();
    loop {
        if s.frame() >= max_frame {
            return None;
        }
        let (c0, h0) = (chest(&s.ctx), head(&s.ctx));
        let f = s.step(tail);
        if f.port_abort || f.step_us > 80_000 {
            return None;
        }
        if f.site == 2 && f.alive_before {
            o.onset = f.frame;
            o.p0 = c0;
            o.h0 = h0;
            o.p1 = f.chest_after;
            o.h1 = head(&s.ctx);
            o.masks = s.masks[..s.replay.iteration as usize].to_vec();
            break;
        }
        if f.site != 0 || !f.alive_after {
            return None;
        }
    }
    o.lock = 1;
    let mut locked = true;
    let mut seen_end = false;
    for _ in 0..horizon {
        if s.frame() >= max_frame + 30 {
            break;
        }
        let f = s.step(tail);
        if f.port_abort {
            break;
        }
        if f.finished && f.alive_before && o.finish == 0 {
            o.finish = f.frame;
        }
        if locked && f.site != 0 {
            o.lock += 1;
        } else {
            locked = false;
        }
        if f.site == 0 && !seen_end {
            seen_end = true;
            o.end_alive = f.alive_after;
            o.end_chest = f.chest_after;
            o.end_neck = neck_gap(&s.ctx);
        }
        if f.site == 0 && f.alive_after && sky.above(f.chest_after) {
            o.sky_frames += 1;
            if o.first_sky == 0 {
                o.first_sky = f.frame;
            }
        }
        o.final_alive = f.alive_after;
        o.final_chest = f.chest_after;
        if f.site == 0 && !f.alive_after {
            break;
        }
    }
    Some(o)
}

/// `aftermath`: replay the distinct pairmax onsets found in event/pool files and report each.
pub fn aftermath(files: &[String], horizon: u32, threads: usize, tail_zero: bool) -> Result<(), String> {
    let mut cands: Vec<Vec<u8>> = Vec::new();
    let mut uniq: HashSet<Vec<u8>> = HashSet::new();
    for f in files {
        let text = std::fs::read_to_string(f).map_err(|e| format!("{f}: {e}"))?;
        for line in text.lines() {
            if !line.contains("\"kind\":\"pairmax\"") && !line.contains("\"onset\"") {
                continue;
            }
            if let Some(m) = crate::tas_field(line) {
                if uniq.insert(m.clone()) {
                    cands.push(m);
                }
            }
        }
    }
    // Shortest first; a lineage's later lock frames share its onset prefix.
    cands.sort_by_key(|m| m.len());
    let mut lens: Vec<usize> = Vec::new();
    let mut onset_prefixes: HashSet<Vec<u8>> = HashSet::new();
    let mut work = Vec::new();
    for m in cands {
        if lens.iter().any(|&l| l < m.len() && onset_prefixes.contains(&m[..l])) {
            continue;
        }
        if !lens.contains(&m.len()) {
            lens.push(m.len());
        }
        onset_prefixes.insert(m.clone());
        work.push(m);
    }
    eprintln!("aftermath: {} candidate onset lineages", work.len());
    let template = Sim::new(crate::LEVEL, crate::CHAR)?;
    let sky = Skyline::new(&template.ctx);
    let next = AtomicUsize::new(0);
    let out: Mutex<Vec<Outcome>> = Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for _ in 0..threads {
            let (next, out, work, template, sky) = (&next, &out, &work, &template, &sky);
            std::thread::Builder::new()
                .stack_size(32 << 20)
                .spawn_scoped(scope, move || {
                    let mut s = template.fork();
                    loop {
                        let i = next.fetch_add(1, Ordering::Relaxed);
                        if i >= work.len() {
                            break;
                        }
                        s.restore(template);
                        s.masks = work[i].clone();
                        let tail = if tail_zero { 0 } else { *work[i].last().unwrap() & !(32 | 64 | 128) };
                        if let Some(o) = run_onset(&mut s, sky, tail, horizon, 299) {
                            out.lock().unwrap().push(o);
                        }
                    }
                })
                .unwrap();
        }
    });
    let mut out = out.into_inner().unwrap();
    out.sort_by(|a, b| a.onset.cmp(&b.onset).then(a.p1[1].partial_cmp(&b.p1[1]).unwrap_or(std::cmp::Ordering::Equal)));
    let mut keys = HashSet::new();
    for o in &out {
        if keys.insert((o.onset, o.p1[0].to_bits(), o.p1[1].to_bits())) {
            println!("{}", o.json());
        }
    }
    eprintln!("aftermath: {} distinct onsets", keys.len());
    Ok(())
}

/// One "release" branch: from a locked state, feed `release` for every new input entry.
#[derive(Clone, Debug, Default)]
pub struct Release {
    /// Frame on which the release input is first read.
    pub at: u32,
    /// Lock frames (throws) seen after the release before the first completed frame.
    pub lock_after: u32,
    pub alive: bool,
    pub chest: [f64; 2],
    pub neck: f64,
    pub sky_frames: u32,
    pub first_sky: u32,
    pub finish: u32,
    pub final_chest: [f64; 2],
    pub final_alive: bool,
    pub max_y_up: f64,
}

/// From `locked` (just after the onset frame, with the lock tail held), for each of the next
/// `span` frames branch a release and follow it `k` frames.
pub fn release_scan(locked: &Sim, sky: &Skyline, hold: u8, release: u8, span: u32, k: u32) -> Vec<(u32, [f64; 2], bool, Release)> {
    let mut out = Vec::new();
    let mut cur = locked.fork();
    let mut br = locked.fork();
    for _ in 0..span {
        // Branch: release from the next input entry.
        br.restore(&cur);
        let mut r = Release { at: br.frame() + 1, max_y_up: f64::INFINITY, ..Default::default() };
        let mut done = false;
        for _ in 0..k {
            let f = br.step(release);
            if f.port_abort {
                break;
            }
            if f.finished && f.alive_before && r.finish == 0 {
                r.finish = f.frame;
            }
            if !done {
                if f.site != 0 {
                    r.lock_after += 1;
                } else {
                    done = true;
                    r.alive = f.alive_after;
                    r.chest = f.chest_after;
                    r.neck = neck_gap(&br.ctx);
                }
            }
            if f.site == 0 && f.alive_after && sky.above(f.chest_after) {
                r.sky_frames += 1;
                if r.first_sky == 0 {
                    r.first_sky = f.frame;
                }
            }
            if f.chest_after[1].is_finite() && f.chest_after[1] < r.max_y_up {
                r.max_y_up = f.chest_after[1];
            }
            r.final_chest = f.chest_after;
            r.final_alive = f.alive_after;
            if f.site == 0 && !f.alive_after {
                break;
            }
        }
        // Advance the held lock one frame.
        let f = cur.step(hold);
        out.push((f.frame, f.chest_after, f.site != 0, r));
        if f.port_abort || (f.site == 0 && !f.alive_after) {
            break;
        }
    }
    out
}

/// `release`: for each onset lineage in --files (aftermath/event rows), hold `hold` through the
/// lock and try a release on each of the next `span` frames.
pub fn release_cmd(files: &[String], hold: Option<u8>, release: u8, span: u32, k: u32, threads: usize) -> Result<(), String> {
    let mut work: Vec<Vec<u8>> = Vec::new();
    let mut uniq = HashSet::new();
    for f in files {
        for line in std::fs::read_to_string(f).map_err(|e| format!("{f}: {e}"))?.lines() {
            if let Some(m) = crate::tas_field(line) {
                if uniq.insert(m.clone()) {
                    work.push(m);
                }
            }
        }
    }
    eprintln!("release: {} lineages", work.len());
    let template = Sim::new(crate::LEVEL, crate::CHAR)?;
    let sky = Skyline::new(&template.ctx);
    let next = AtomicUsize::new(0);
    let print = Mutex::new(());
    std::thread::scope(|scope| {
        for _ in 0..threads {
            let (next, work, template, sky, print) = (&next, &work, &template, &sky, &print);
            std::thread::Builder::new()
                .stack_size(32 << 20)
                .spawn_scoped(scope, move || {
                    let mut s = template.fork();
                    loop {
                        let i = next.fetch_add(1, Ordering::Relaxed);
                        if i >= work.len() {
                            break;
                        }
                        s.restore(template);
                        s.masks = work[i].clone();
                        let tail = hold.unwrap_or(*work[i].last().unwrap() & !(32 | 64 | 128));
                        // Play to the onset (first pairmax throw).
                        let mut onset = 0;
                        while s.frame() < 299 {
                            let f = s.step(tail);
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
                        let base = s.masks[..s.replay.iteration as usize].to_vec();
                        let scan = release_scan(&s, sky, tail, release, span, k);
                        let _g = print.lock().unwrap();
                        for (j, (fr, c, locked, r)) in scan.iter().enumerate() {
                            if r.alive || r.sky_frames > 0 || r.finish > 0 || std::env::var("OBBY_ALL").is_ok() {
                                println!(
                                    "{{\"onset\":{},\"j\":{},\"frame\":{},\"locked\":{},\"chest\":[{:.2},{:.2}],\"at\":{},\"lock_after\":{},\"alive\":{},\"rchest\":[{:.2},{:.2}],\"neck\":{:.3},\"sky\":{},\"first_sky\":{},\"finish\":{},\"final_alive\":{},\"final\":[{:.2},{:.2}],\"min_y\":{:.2},\"tas\":\"{}\"}}",
                                    onset, j, fr, locked, c[0], c[1], r.at, r.lock_after, r.alive, r.chest[0], r.chest[1], r.neck, r.sky_frames, r.first_sky,
                                    r.finish, r.final_alive, r.final_chest[0], r.final_chest[1], r.max_y_up,
                                    {
                                        let mut m = base.clone();
                                        m.extend(std::iter::repeat(tail).take(j));
                                        tas_text(&m)
                                    }
                                );
                            }
                        }
                    }
                })
                .unwrap();
        }
    });
    Ok(())
}

/// Broadphase bound values (x lo, x hi, y lo, y hi) of every dynamic body's shapes, in body-list
/// order: (list index, body, values).
pub fn proxy_snapshot(ctx: &State) -> Vec<(usize, hw_core::H, Vec<[u32; 4]>)> {
    let w = &ctx.world;
    let bp = &w.broad_phase;
    let mut out = Vec::new();
    let mut b = w.body_list;
    let mut idx = 0;
    while b != NONE && idx < hw_core::b2::state::MAX_BODIES {
        let bd = w.body(b);
        if bd.inv_mass > 0.0 {
            let mut vals = Vec::new();
            let mut sh = bd.shape_list;
            let mut g = 0;
            while sh != NONE && g < 64 {
                g += 1;
                let s = w.shape(sh);
                let pid = s.proxy_id as usize;
                if pid < bp.proxy_pool.len() {
                    let pr = &bp.proxy_pool[pid];
                    let get = |axis: usize, i: u32| bp.bounds[axis].get(i as usize).map(|b| b.value).unwrap_or(u32::MAX);
                    vals.push([get(0, pr.lower_bounds[0]), get(0, pr.upper_bounds[0]), get(1, pr.lower_bounds[1]), get(1, pr.upper_bounds[1])]);
                } else {
                    vals.push([u32::MAX; 4]);
                }
                sh = s.next;
            }
            out.push((idx, b, vals));
        }
        idx += 1;
        b = bd.next;
    }
    out
}

/// `trace`: replay --tas then feed --tail; print Dad's upper-body state each frame from --from.
pub fn trace_cmd(tas: &str, tail: u8, frames: u32, from: u32) -> Result<(), String> {
    let mut s = Sim::new(crate::LEVEL, crate::CHAR)?;
    s.masks = crate::sim::read_tas(tas)?;
    let sky = Skyline::new(&s.ctx);
    let throw_trace = std::env::var("OBBY_TRACE_THROW").is_ok();
    for _ in 0..frames {
        let before = if throw_trace && s.frame() + 1 >= from { proxy_snapshot(&s.ctx) } else { Vec::new() };
        let f = s.step(tail);
        if f.frame < from {
            continue;
        }
        if throw_trace {
            let after = proxy_snapshot(&s.ctx);
            let owner = crate::owners(&s.ctx);
            let mut changed = Vec::new();
            for (i, b, v) in &before {
                if let Some((_, _, v2)) = after.iter().find(|(_, b2, _)| b2 == b) {
                    if v2 != v {
                        changed.push((*i, *b));
                    }
                }
            }
            let last = changed.last().map(|&(i, b)| {
                let c = hw_core::b2::body::get_world_center(&s.ctx, b);
                let v = s.ctx.world.body(b).linear_velocity;
                format!("#{} {} at ({:.1},{:.1}) v ({:.0},{:.0})", i, owner.get(&(b as u32)).cloned().unwrap_or_else(|| "x".into()), c.x, c.y, v.x, v.y)
            });
            println!("   throw-scan: site {} changed {} of {} dynamic; last changed {}", f.site, changed.len(), before.len(), last.unwrap_or_default());
        }
        let p = &s.ctx.character.primary;
        let neck = p.joints[NECK];
        let broken = neck == NONE || !s.ctx.world.joints.is_alive(neck) || s.ctx.world.joint(neck).broken.get();
        let h = head(&s.ctx);
        println!(
            "f{} m{} site{} pairs{} alive{} chest[{:.2},{:.2}] head[{:.2},{:.2}] neck{:.3} broken{} dying{} sky{} finish{}",
            f.frame, f.mask, f.site, f.pairs, f.alive_after as u8, f.chest_after[0], f.chest_after[1], h[0], h[1], neck_gap(&s.ctx), broken as u8,
            p.dying.get() as u8, sky.above(f.chest_after) as u8, s.replay.finish_frame
        );
        if let Ok(n) = std::env::var("OBBY_TRACE_HEAD") {
            let n: usize = n.parse().unwrap_or(5);
            let owner = crate::owners(&s.ctx);
            let w = &s.ctx.world;
            let mut b = w.body_list;
            let mut idx = 0;
            while b != NONE && idx < n {
                let bd = w.body(b);
                let c = hw_core::b2::body::get_world_center(&s.ctx, b);
                let mut nj = 0;
                let mut je = bd.joint_list;
                while je != NONE && nj < 64 {
                    nj += 1;
                    je = w.joint_edge(je).next;
                }
                println!(
                    "   body#{} h{} {} ud{} mass{:.3} shapes{} joints{} flags{:#x} at ({:.2},{:.2}) v ({:.1},{:.1})",
                    idx, b, owner.get(&(b as u32)).cloned().unwrap_or_else(|| "x".into()), bd.user_data, bd.mass, bd.shape_count, nj, bd.flags, c.x, c.y,
                    bd.linear_velocity.x, bd.linear_velocity.y
                );
                idx += 1;
                b = bd.next;
            }
        }
        if std::env::var("OBBY_TRACE_COMP").is_ok() {
            for (who, ch) in [("dad", &s.ctx.character.primary), ("child", &s.ctx.character.child)] {
                let mut line = String::new();
                for i in 0..ch.composite_count as usize {
                    let c = ch.composites[i];
                    line.push_str(&format!(" {}{}", ["lig", "spine", "int"][c.kind.min(2) as usize], i));
                    if c.kind == 0 {
                        let j = c.ends[0];
                        if j != NONE && s.ctx.world.joints.is_alive(j) && s.ctx.world.joint(j).broken.get() {
                            let a = s.ctx.world.joint(j).body1;
                            let v = s.ctx.world.body(a).linear_velocity;
                            line.push_str(&format!("T{:.0e}", (v.x * v.x + v.y * v.y).sqrt()));
                        }
                    }
                }
                let broken: Vec<usize> = (0..10).filter(|&i| { let j = ch.joints[i]; j == NONE || !s.ctx.world.joints.is_alive(j) || s.ctx.world.joint(j).broken.get() }).collect();
                println!("   {} composites:{} | broken joints {:?} dead {} dying {}", who, line, broken, ch.dead.get() as u8, ch.dying.get() as u8);
            }
        }
        if std::env::var("OBBY_TRACE_CONTACTS").is_ok() {
            // Touching contacts on Dad's bodies: Dad part <- other body (owner, speed).
            let owner = crate::owners(&s.ctx);
            let w = &s.ctx.world;
            let dad: std::collections::HashSet<u32> = s.ctx.character.primary.bodies.iter().map(|&b| b as u32).collect();
            let mut line = String::new();
            let mut c = w.contact_list;
            let mut guard = 0;
            while c != NONE && guard < 100_000 {
                guard += 1;
                let ct = w.contact(c);
                if ct.manifold_count > 0 {
                    let (b1, b2) = (w.shape(ct.shape1).body as u32, w.shape(ct.shape2).body as u32);
                    for (me, other) in [(b1, b2), (b2, b1)] {
                        if dad.contains(&me) {
                            let v = w.body(other as hw_core::H).linear_velocity;
                            line.push_str(&format!(
                                " {}<{}@{:.0}",
                                owner.get(&me).cloned().unwrap_or_default(),
                                owner.get(&other).cloned().unwrap_or_else(|| if w.body(other as hw_core::H).inv_mass == 0.0 { "static".into() } else { format!("x{}", other) }),
                                v.x.hypot(v.y)
                            ));
                        }
                    }
                }
                c = ct.next;
            }
            println!("   contacts:{line}");
        }
        if std::env::var("OBBY_TRACE_QE").is_ok() {
            // Torn ligaments: anchor speed / contacts / sleep, node0 speed, anchor-node0 distance.
            let w = &s.ctx.world;
            let mut line = String::new();
            for (who, ch) in [("d", &s.ctx.character.primary), ("c", &s.ctx.character.child)] {
                for i in 0..ch.composite_count as usize {
                    let cp = ch.composites[i];
                    if cp.kind != 0 || cp.ends[0] == NONE || !w.joints.is_alive(cp.ends[0]) || !w.joint(cp.ends[0]).broken.get() {
                        continue;
                    }
                    let j = w.joint(cp.ends[0]);
                    let (a, n) = (j.body1, j.body2);
                    let va = w.body(a).linear_velocity;
                    let vn = w.body(n).linear_velocity;
                    let pa = hw_core::b2::body::get_world_center(&s.ctx, a);
                    let pn = hw_core::b2::body::get_world_center(&s.ctx, n);
                    line.push_str(&format!(
                        " {}{}:a{:.1e}{}{} n{:.1e} d{:.2} imp{:.2}",
                        who,
                        i,
                        (va.x * va.x + va.y * va.y).sqrt(),
                        if w.body(a).contact_list != NONE { "C" } else { "" },
                        if w.body(a).flags & 0x8 != 0 { "S" } else if w.body(a).flags & 0x2 != 0 { "F" } else { "" },
                        (vn.x * vn.x + vn.y * vn.y).sqrt(),
                        ((pa.x - pn.x).powi(2) + (pa.y - pn.y).powi(2)).sqrt(),
                        j.impulse
                    ));
                }
            }
            println!("   qe:{line}");
        }
        if std::env::var("OBBY_TRACE_MINES").is_ok() {
            // Mines: index, alive body position (or "gone").
            let mut line = String::new();
            for i in 0..s.ctx.level.count as usize {
                let sp = &s.ctx.level.specials[i];
                if format!("{:?}", sp.kind) == "Mine" {
                    if sp.body != NONE
                        && s.ctx.world.bodies.is_alive(sp.body)
                        && sp.shape != NONE
                        && s.ctx.world.shapes.is_alive(sp.shape)
                        && s.ctx.world.shape(sp.shape).body == sp.body
                    {
                        let c = hw_core::b2::body::get_world_center(&s.ctx, sp.body);
                        let v = s.ctx.world.body(sp.body).linear_velocity;
                        line.push_str(&format!(" m{}({:.2},{:.2} v{:.0},{:.0})", i, c.x, c.y, v.x, v.y));
                    } else {
                        line.push_str(&format!(" m{}:gone", i));
                    }
                }
            }
            println!("   mines:{line}");
        }
        if std::env::var("OBBY_TRACE_BP").is_ok() {
            let (ix, iy, bad) = bp_check(&s.ctx);
            println!("   bp: inversions x {} y {} bad-proxy-axes {}", ix, iy, bad);
        }
        if std::env::var("OBBY_TRACE_BODIES").is_ok() {
            // Dynamic bodies faster than 50 m/s in body-list order (synchronize order).
            let owner = crate::owners(&s.ctx);
            let w = &s.ctx.world;
            let mut b = w.body_list;
            let (mut idx, mut line) = (0, String::new());
            while b != NONE && idx < hw_core::b2::state::MAX_BODIES {
                let bd = w.body(b);
                if bd.inv_mass > 0.0 {
                    let v = bd.linear_velocity;
                    let sp = (v.x * v.x + v.y * v.y).sqrt();
                    if sp > 50.0 {
                        let c = hw_core::b2::body::get_world_center(&s.ctx, b);
                        line.push_str(&format!(" #{}:{}@{:.0}({:.0},{:.0}){}", idx, owner.get(&(b as u32)).cloned().unwrap_or_else(|| "x".into()), sp, c.x, c.y, if bd.flags & 0x2 != 0 { "F" } else { "" }));
                    }
                }
                idx += 1;
                b = bd.next;
            }
            println!("   fast:{line}");
        }
    }
    Ok(())
}

/// `sens`: for each frame in [from, onset), replace that one input with each alternative of a
/// small set and report whether the first pairmax onset (frame, chest after) changes.
pub fn sens_cmd(tas: &str, from: usize, threads: usize) -> Result<(), String> {
    let masks = crate::sim::read_tas(tas)?;
    let template = Sim::new(crate::LEVEL, crate::CHAR)?;
    let first = |m: &[u8]| -> Option<(u32, [f64; 2])> {
        let mut s = template.fork();
        s.masks = m.to_vec();
        let tail = *m.last().unwrap() & !(32 | 64 | 128);
        while s.frame() < 299 {
            let f = s.step(tail);
            if f.port_abort || (f.site == 0 && !f.alive_after) {
                return None;
            }
            if f.site == 2 {
                return Some((f.frame, f.chest_after));
            }
        }
        None
    };
    let base = first(&masks).ok_or("no onset")?;
    eprintln!("base onset {:?}", base);
    let next = AtomicUsize::new(from);
    let out = Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for _ in 0..threads {
            let (next, out, masks, first) = (&next, &out, &masks, &first);
            std::thread::Builder::new().stack_size(32 << 20).spawn_scoped(scope, move || loop {
                let i = next.fetch_add(1, Ordering::Relaxed);
                if i + 1 >= masks.len() {
                    break;
                }
                let mut diff = 0;
                let mut tried = 0;
                for alt in [0u8, 16, 1 | 16, 2 | 16, 4 | 16, 8 | 16, 4, 8] {
                    let alt = alt | (masks[i] & (32 | 64 | 128));
                    if alt == masks[i] {
                        continue;
                    }
                    tried += 1;
                    let mut m = masks.clone();
                    m[i] = alt;
                    let r = first(&m);
                    if r.map(|(f, c)| (f, c[0].to_bits(), c[1].to_bits())) != Some((base.0, base.1[0].to_bits(), base.1[1].to_bits())) {
                        diff += 1;
                    }
                }
                out.lock().unwrap().push((i, masks[i], diff, tried));
            }).unwrap();
        }
    });
    let mut v = out.into_inner().unwrap();
    v.sort();
    for (i, m, d, t) in v {
        println!("frame-entry {} mask {} changed {}/{}", i, m, d, t);
    }
    Ok(())
}

/// Bound-array order violations per axis (value decreasing between neighbours), and proxies whose
/// stored bound indices do not point back at themselves.
pub fn bp_check(ctx: &State) -> (usize, usize, usize) {
    let bp = &ctx.world.broad_phase;
    let n = (2 * bp.proxy_count).max(0) as usize;
    let mut inv = [0usize; 2];
    for axis in 0..2 {
        for i in 1..n.min(bp.bounds[axis].len()) {
            if bp.bounds[axis][i].value < bp.bounds[axis][i - 1].value {
                inv[axis] += 1;
            }
        }
    }
    let mut bad = 0;
    for (pid, pr) in bp.proxy_pool.iter().enumerate() {
        if pr.shape == NONE {
            continue;
        }
        for axis in 0..2 {
            let lo = pr.lower_bounds[axis] as usize;
            let hi = pr.upper_bounds[axis] as usize;
            let ok = lo < n && hi < n && bp.bounds[axis][lo].proxy_id as usize == pid && bp.bounds[axis][hi].proxy_id as usize == pid && lo < hi;
            if !ok {
                bad += 1;
            }
        }
    }
    (inv[0], inv[1], bad)
}

/// `branch`: from the end of --tas, try --n random input continuations of --depth frames
/// (poses and Space; eject bits kept from the last input) and report any finish, plus the
/// closest approach of the alive chest to the finish region.
pub fn branch_cmd(tas: &str, n: u64, depth: u32, threads: usize, seed: u64) -> Result<(), String> {
    let masks = crate::sim::read_tas(tas)?;
    let mut base = Sim::new(crate::LEVEL, crate::CHAR)?;
    base.masks = masks.clone();
    while (base.replay.iteration as usize) < masks.len() {
        base.step(0);
    }
    eprintln!("branch: base at frame {} chest {:?} alive {}", base.frame(), chest(&base.ctx), !base.ctx.level.dead.get());
    let last = *masks.last().unwrap() & (32 | 64 | 128);
    let next = std::sync::atomic::AtomicU64::new(0);
    let best = Mutex::new((f64::INFINITY, Vec::<u8>::new()));
    let found = Mutex::new(Vec::<Vec<u8>>::new());
    std::thread::scope(|scope| {
        for t in 0..threads {
            let (next, best, found, base) = (&next, &best, &found, &base);
            std::thread::Builder::new().stack_size(32 << 20).spawn_scoped(scope, move || {
                let mut rng = crate::window::Rng((seed.wrapping_add(t as u64 + 1)).wrapping_mul(0xd1b5_4a32_d192_ed03) ^ 0x2545_f491_4f6c_dd1d);
                let mut s = base.fork();
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    if i >= n {
                        break;
                    }
                    s.restore(base);
                    let mut near = f64::INFINITY;
                    for _ in 0..depth {
                        if s.frame() >= 299 {
                            break;
                        }
                        const POSE: [u8; 9] = [0, 1, 2, 4, 8, 1 | 4, 2 | 4, 1 | 8, 2 | 8];
                        let m = POSE[rng.below(9) as usize] | if rng.below(2) == 0 { 16 } else { 0 } | last;
                        let f = s.step(m);
                        if f.finished && f.alive_before {
                            found.lock().unwrap().push(s.masks[..s.replay.iteration as usize].to_vec());
                            break;
                        }
                        if f.site == 0 && !f.alive_after {
                            break;
                        }
                        if f.alive_after {
                            near = near.min(crate::window::finish_distance(f.chest_after));
                        }
                    }
                    let mut b = best.lock().unwrap();
                    if near < b.0 {
                        *b = (near, s.masks[..s.replay.iteration as usize].to_vec());
                    }
                }
            }).unwrap();
        }
    });
    let f = found.into_inner().unwrap();
    let b = best.into_inner().unwrap();
    eprintln!("branch: {} finishes; closest alive approach {:.2}", f.len(), b.0);
    for m in &f {
        println!("{{\"kind\":\"finish\",\"tas\":\"{}\"}}", tas_text(m));
    }
    println!("{{\"kind\":\"closest\",\"near\":{:.3},\"tas\":\"{}\"}}", b.0, tas_text(&b.1));
    Ok(())
}

/// `geom`: static (and dynamic) shape AABBs intersecting a box (x0,y0,x1,y1).
pub fn geom_cmd(x0: f64, y0: f64, x1: f64, y1: f64) -> Result<(), String> {
    use hw_core::b2::shapes;
    let s = Sim::new(crate::LEVEL, crate::CHAR)?;
    let ctx = &s.ctx;
    let w = &ctx.world;
    let mut b = w.body_list;
    while b != NONE {
        let bd = w.body(b);
        let xf = bd.xf;
        let mut sh = bd.shape_list;
        while sh != NONE {
            let sp = w.shape(sh);
            let (lo, hi) = shapes::compute_aabb(ctx, sh, &xf);
            if hi.x >= x0 && lo.x <= x1 && hi.y >= y0 && lo.y <= y1 {
                println!(
                    "shape h{} body h{} {} sensor{} aabb x [{:.3},{:.3}] y [{:.3},{:.3}] handlers {:?} type {}",
                    sh, b, if bd.inv_mass == 0.0 { "static" } else { "dynamic" }, sp.is_sensor.get() as u8, lo.x, hi.x, lo.y, hi.y, sp.handlers, sp.type_
                );
            }
            sh = sp.next;
        }
        b = bd.next;
    }
    Ok(())
}

/// `info`: Dad's break/smash limits and body masses at the start.
pub fn info_cmd() -> Result<(), String> {
    let mut s = Sim::new(crate::LEVEL, crate::CHAR)?;
    s.step(0);
    let p = &s.ctx.character.primary;
    println!("break_limits (neck, spine, torso, intestine, shoulder, shoulderSnap, hip, hipSnap, elbow, elbowLig, knee, kneeLig): {:?}", p.break_limits);
    println!("smash_limits (head, chest, pelvis, foot): {:?}", p.smash_limits);
    for (i, &b) in p.bodies.iter().enumerate() {
        if b != NONE {
            println!("body {} mass {:.4}", i, s.ctx.world.body(b).mass);
        }
    }
    Ok(())
}

/// `lockctl`: from the onset lineage --tas (ending at its onset frame), try --n random input
/// sequences for the next --k frames (then --hold), and report when the lock ends and where Dad is.
pub fn lockctl_cmd(tas: &str, n: u64, k: u32, hold: u8, threads: usize) -> Result<(), String> {
    let masks = crate::sim::read_tas(tas)?;
    let mut base = Sim::new(crate::LEVEL, crate::CHAR)?;
    base.masks = masks.clone();
    while (base.replay.iteration as usize) < masks.len() {
        base.step(0);
    }
    let sky = Skyline::new(&base.ctx);
    eprintln!("lockctl: base frame {} chest {:?}", base.frame(), chest(&base.ctx));
    let next = std::sync::atomic::AtomicU64::new(0);
    let out = Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for t in 0..threads {
            let (next, out, base, sky) = (&next, &out, &base, &sky);
            std::thread::Builder::new().stack_size(32 << 20).spawn_scoped(scope, move || {
                let mut rng = crate::window::Rng((t as u64 + 11).wrapping_mul(0xd1b5_4a32_d192_ed03) ^ 0x2545_f491_4f6c_dd1d);
                let mut s = base.fork();
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    if i >= n {
                        break;
                    }
                    s.restore(base);
                    let mut end = 0;
                    let mut alive = false;
                    let mut c = [f64::NAN; 2];
                    let mut j = 0;
                    while s.frame() < 299 {
                        const POSE: [u8; 9] = [0, 1, 2, 4, 8, 1 | 4, 2 | 4, 1 | 8, 2 | 8];
                        let m = if j < k { POSE[rng.below(9) as usize] | if rng.below(2) == 0 { 16 } else { 0 } } else { hold };
                        j += 1;
                        let f = s.step(m);
                        if f.port_abort {
                            break;
                        }
                        if f.site == 0 {
                            end = f.frame;
                            alive = f.alive_after;
                            c = f.chest_after;
                            break;
                        }
                    }
                    out.lock().unwrap().push((end, alive, c, sky.above(c), s.masks[..s.replay.iteration as usize].to_vec()));
                }
            }).unwrap();
        }
    });
    let mut v = out.into_inner().unwrap();
    v.sort_by_key(|r| r.0);
    let mut hist = std::collections::BTreeMap::new();
    for r in &v {
        *hist.entry(r.0 / 5 * 5).or_insert(0) += 1;
    }
    eprintln!("lock end histogram (frame bucket: count): {:?}", hist);
    for r in v.iter().filter(|r| r.1 && r.3) {
        println!("{{\"end\":{},\"chest\":[{:.2},{:.2}],\"tas\":\"{}\"}}", r.0, r.2[0], r.2[1], tas_text(&r.4));
    }
    Ok(())
}

/// `pairmap`: at frame --frame of --tas, list each proxy's AABB extent (from its bound values) and
/// its overlap count with all other proxies, grouped by owner, largest first.
pub fn pairmap_cmd(tas: &str, frame: u32) -> Result<(), String> {
    let mut s = Sim::new(crate::LEVEL, crate::CHAR)?;
    s.masks = crate::sim::read_tas(tas)?;
    while s.frame() < frame {
        s.step(0);
    }
    let owner = crate::owners(&s.ctx);
    let w = &s.ctx.world;
    let bp = &w.broad_phase;
    let mut items = Vec::new();
    let mut b = w.body_list;
    while b != NONE {
        let bd = w.body(b);
        let mut sh = bd.shape_list;
        while sh != NONE {
            let sp = w.shape(sh);
            let pid = sp.proxy_id as usize;
            if pid < bp.proxy_pool.len() {
                let pr = &bp.proxy_pool[pid];
                let get = |axis: usize, i: u32| bp.bounds[axis].get(i as usize).map(|x| x.value).unwrap_or(0);
                items.push((b, owner.get(&(b as u32)).cloned().unwrap_or_else(|| if bd.inv_mass == 0.0 { "static".into() } else { "x".into() }), [get(0, pr.lower_bounds[0]), get(0, pr.upper_bounds[0]), get(1, pr.lower_bounds[1]), get(1, pr.upper_bounds[1])]));
            }
            sh = sp.next;
        }
        b = bd.next;
    }
    let n = items.len();
    let mut cnt = vec![0usize; n];
    for i in 0..n {
        for j in i + 1..n {
            let (a, c) = (items[i].2, items[j].2);
            if a[0] <= c[1] && c[0] <= a[1] && a[2] <= c[3] && c[2] <= a[3] {
                cnt[i] += 1;
                cnt[j] += 1;
            }
        }
    }
    let total: usize = cnt.iter().sum::<usize>() / 2;
    println!("frame {} proxies {} overlapping pairs {} (pair_count {})", s.frame(), n, total, bp.pair_count);
    let mut idx: Vec<usize> = (0..n).collect();
    idx.sort_by_key(|&i| std::cmp::Reverse(cnt[i]));
    for &i in idx.iter().take(std::env::var("OBBY_PAIRMAP_ALL").map(|_| usize::MAX).unwrap_or(60)) {
        let v = items[i].2;
        println!("  {:>8} h{:<4} overlaps {:>4}  x-span {:>6} y-span {:>6}", items[i].1, items[i].0, cnt[i], v[1].saturating_sub(v[0]), v[3].saturating_sub(v[2]));
    }
    Ok(())
}

/// `launches`: for each lineage in --files, find the first frame Dad's chest rises faster than
/// 15 m/s in the start area and report the velocity a few frames later, dying state and alive.
pub fn launches_cmd(files: &[String], threads: usize) -> Result<(), String> {
    let mut work: Vec<Vec<u8>> = Vec::new();
    let mut uniq = HashSet::new();
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
                    let mut launch = None;
                    while s.frame() < 140 {
                        let f = s.step(0);
                        if f.port_abort || (f.site == 0 && !f.alive_after) {
                            break;
                        }
                        let c = f.chest_after;
                        if launch.is_none() && c[0] > 0.0 && c[0] < 26.0 && c[1] > 62.0 && c[1] < 84.0 && f.chest_v[1] < -15.0 {
                            launch = Some(f.frame);
                        }
                        if let Some(l) = launch {
                            if f.frame == l + 3 {
                                out.lock().unwrap().push((l, c, f.chest_v, s.ctx.character.primary.dying.get(), f.alive_after));
                                break;
                            }
                        }
                    }
                }
            }).unwrap();
        }
    });
    let v = out.into_inner().unwrap();
    let mut hist = std::collections::BTreeMap::new();
    for (_, _, cv, dying, alive) in &v {
        let ang = (-cv[1]).atan2(cv[0]).to_degrees();
        *hist.entry(((ang / 10.0).floor() as i32 * 10, *dying, *alive)).or_insert(0) += 1;
    }
    println!("launches {} of {} lineages; (angle bucket, dying, alive): count", v.len(), work.len());
    for (k, c) in hist {
        println!("  {:?}: {}", k, c);
    }
    let mut best: Vec<_> = v.iter().map(|(l, c, cv, d, a)| (((-cv[1]).atan2(cv[0]).to_degrees() - 40.0).abs(), *l, *c, *cv, *d, *a)).collect();
    best.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    for b in best.iter().take(10) {
        println!("  closest to 40deg: launch f{} chest [{:.2},{:.2}] v [{:.1},{:.1}] dying {} alive {}", b.1, b.2[0], b.2[1], b.3[0], b.3[1], b.4, b.5);
    }
    Ok(())
}

/// `nearceil`: for each lineage, the best pair count on a frame where Dad's chest is 0..6 m below
/// the spike ceiling (x 0..24, y 67..73) and rising; prints the top lineages.
pub fn nearceil_cmd(files: &[String], threads: usize) -> Result<(), String> {
    let mut work: Vec<Vec<u8>> = Vec::new();
    let mut uniq = HashSet::new();
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
                    let mut best = (0, 0u32, [0.0; 2], [0.0; 2]);
                    while s.frame() < 140 {
                        let f = s.step(0);
                        if f.port_abort || (f.site == 0 && !f.alive_after) {
                            break;
                        }
                        let c = f.chest_after;
                        if c[0] > 0.0 && c[0] < 24.0 && c[1] > 67.0 && c[1] < 73.0 && f.chest_v[1] < -10.0 && f.pairs > best.0 {
                            best = (f.pairs, f.frame, c, f.chest_v);
                        }
                    }
                    out.lock().unwrap().push((best, i));
                }
            }).unwrap();
        }
    });
    let mut v = out.into_inner().unwrap();
    v.sort_by_key(|r| std::cmp::Reverse(r.0 .0));
    for ((p, fr, c, cv), i) in v.iter().take(12) {
        println!("pairs {} frame {} chest [{:.2},{:.2}] v [{:.1},{:.1}] lineage {}", p, fr, c[0], c[1], cv[0], cv[1], i);
    }
    Ok(())
}
