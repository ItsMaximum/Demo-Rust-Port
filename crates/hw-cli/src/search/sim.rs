//! One replay under search control: the `hw sim` frame contract with an open-ended input.
//! Inputs are indexed by iteration exactly like a TAS string: a frame whose input phase aborts
//! does not consume its entry, so the next frame re-reads it (and ignores the caller's mask).
use hw_core::b2::{body, state::{edge_owner, MAX_BODIES, MAX_JOINTS}};
use hw_core::game::character::CHEST;
use hw_core::game::session::{self, ReplayState};
use hw_core::{AbortSite, Ctx, Flow, Input, NONE};

pub type State = Ctx<f64>;

#[derive(Clone)]
pub struct Sim {
    pub ctx: Box<State>,
    pub replay: ReplayState,
    /// Input entries by iteration; the TAS string of this run.
    pub masks: Vec<u8>,
}

/// What one frame did, measured from outside the engine.
#[derive(Clone, Copy, Debug, Default)]
pub struct Frame {
    pub frame: u32,
    pub mask: u8,
    pub site: u32,
    /// Port-only abort: the run left Flash-verified territory.
    pub port_abort: bool,
    pub alive_before: bool,
    pub alive_after: bool,
    pub finished: bool,
    pub chest_before: [f64; 2],
    pub chest_after: [f64; 2],
    /// Distance the chest moved this frame.
    pub displacement: f64,
    /// Fastest character body after the frame; for Dad, the dynamic live-joint
    /// component rooted at his chest. Contacts and angular velocity are not scored.
    pub qe: f64,
    /// Previous owner-body aggregate, ignoring connectivity: Dad, bicycle, child and
    /// seat. Diagnostic only; detached live parts can dominate this value.
    pub qe_raw: f64,
    pub ejected: bool,
    pub grips: u32,
    /// Grip joints holding a static body (the level).
    pub static_grips: u32,
    /// Grips holding a dynamic body that has other joints (teeth, chain links: QE fuel).
    pub jointed_grips: u32,
    /// Host BODY-edge traversal reached every terminator. False means the count
    /// is partial diagnostic data, never a physics abort or a repaired world.
    pub jointed_grips_complete: bool,
    pub chest_alive: bool,
    /// CPU cost of this frame in microseconds (this thread's cycles, so preemption does not count).
    pub step_us: u32,
    /// `step_us` exceeded the lag cut-off (`set_lag_ms`): a runaway QE that lags the game.
    pub lagging: bool,
    /// Finish checks this frame (only when `set_finish_checks(true)`): the smallest distance (m)
    /// from the finish region of the chest position a FinishLine RESULT read, and that position.
    /// Any limb (or anything) touching the finish box triggers such a check; 0 means a finish.
    pub fc: f64,
    pub fc_pos: [f64; 2],
    /// Broadphase pairs and world contacts after the frame.
    pub pairs: i32,
    pub contacts: i32,
}

/// Per-frame CPU cut-off in microseconds (0 = off).
static LAG_US: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

/// A frame costing more than `ms` of CPU is laggy; a run of `lag_frames()` of them marks a
/// runaway QE (severe lag in the game) and searches stop extending that branch.
pub fn set_lag_ms(ms: f64) {
    LAG_US.store((ms * 1000.0) as u32, std::sync::atomic::Ordering::Relaxed);
}

/// Wall-clock budget of one frame in ms (0 = off): past it, `SolveTOI` aborts the frame as a
/// port-only abort (a single frame hung by a runaway QE).
static HANG_MS: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

thread_local! {
    static HANG_OVERRIDE: std::cell::Cell<Option<u32>> = const { std::cell::Cell::new(None) };
}

/// Replaces this thread's hang guard (ms) until dropped.
pub struct HangOverride(Option<u32>);

impl HangOverride {
    pub fn new(ms: u32) -> Self {
        HangOverride(HANG_OVERRIDE.with(|h| h.replace(Some(ms))))
    }
}

impl Drop for HangOverride {
    fn drop(&mut self) {
        HANG_OVERRIDE.with(|h| h.set(self.0));
    }
}

pub fn set_hang_ms(ms: u32) {
    HANG_MS.store(ms, std::sync::atomic::Ordering::Relaxed);
}

/// Consecutive laggy frames before a search drops the branch.
static LAG_FRAMES: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(3);

pub fn set_lag_frames(n: u32) {
    LAG_FRAMES.store(n.max(1), std::sync::atomic::Ordering::Relaxed);
}

static FINISH_CHECKS: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Record every FinishLine check in `Frame::fc` (arms the `lab` recorder with no watched bodies
/// on frames nobody else armed; recording never changes the simulation).
pub fn set_finish_checks(on: bool) {
    FINISH_CHECKS.store(on, std::sync::atomic::Ordering::Relaxed);
}

/// Distance from (x, y) to the nearest FinishLine region of this level (0 inside).
pub fn finish_region_distance(ctx: &State, x: f64, y: f64) -> f64 {
    let l = &ctx.level;
    let mut best = f64::INFINITY;
    for sp in l.specials[..l.count as usize].iter() {
        if sp.kind == hw_data::SpecialKind::FinishLine {
            let dx = (sp.lower.x - x).max(0.0).max(x - sp.upper.x);
            let dy = (sp.lower.y - y).max(0.0).max(y - sp.upper.y);
            best = best.min((dx * dx + dy * dy).sqrt());
        }
    }
    best
}

pub fn lag_frames() -> u32 {
    LAG_FRAMES.load(std::sync::atomic::Ordering::Relaxed)
}

#[link(name = "kernel32")]
extern "system" {
    fn GetCurrentThread() -> isize;
    fn QueryThreadCycleTime(thread: isize, cycles: *mut u64) -> i32;
}

/// CPU cycles consumed by the calling thread (TSC units).
fn thread_cycles() -> u64 {
    let mut c = 0u64;
    unsafe { QueryThreadCycleTime(GetCurrentThread(), &mut c) };
    c
}

/// TSC cycles per microsecond, measured once with a short busy loop.
fn cycles_per_us() -> f64 {
    static CAL: std::sync::OnceLock<f64> = std::sync::OnceLock::new();
    *CAL.get_or_init(|| {
        let t0 = std::time::Instant::now();
        let c0 = thread_cycles();
        while t0.elapsed() < std::time::Duration::from_millis(30) {
            std::hint::spin_loop();
        }
        (thread_cycles() - c0) as f64 / t0.elapsed().as_secs_f64() / 1e6
    })
}

pub fn chest(ctx: &State) -> [f64; 2] {
    let c = ctx.character.bodies[CHEST];
    if c == NONE {
        return [f64::NAN; 2];
    }
    let p = body::get_world_center(ctx, c);
    [p.x, p.y]
}

pub fn ejected(ctx: &State) -> bool {
    match ctx.character.kind {
        2 => ctx.character.segway.ejected.get(),
        3 => ctx.character.bicycle.ejected.get(),
        4 => ctx.character.motorcart.ejected.get(),
        _ => ctx.character.wheelchair.ejected.get(),
    }
}

pub fn static_grips(ctx: &State) -> u32 {
    let mut n = 0;
    for &j in ctx.character.grip_joints.iter() {
        if j != NONE && ctx.world.joints.is_alive(j) && body::is_static(ctx, ctx.world.joint(j).body1) {
            n += 1;
        }
    }
    n
}

/// Bounded read of the host-visible BODY edge chain. Retired links remain readable
/// exactly as before; repeated/out-of-range links make the diagnostic incomplete.
fn other_grip_edges(start: u32, grips: &[u32], mut read: impl FnMut(u32) -> Option<(u32,u32)>) -> (u32,bool) {
    let mut seen=[false;MAX_JOINTS*2];let mut edge=start;let mut others=0;
    while edge!=NONE {
        if edge as usize>=seen.len() || seen[edge as usize] {return (others,false);}
        seen[edge as usize]=true;
        let Some((owner,next))=read(edge) else {return (others,false);};
        if !grips.contains(&owner) {others+=1;}
        edge=next;
    }
    (others,true)
}

pub fn jointed_grips_with_status(ctx: &State) -> (u32,bool) {
    let mut n = 0;
    let mut complete=true;
    for &j in ctx.character.grip_joints.iter() {
        if j==NONE {continue;}
        if j as usize>=MAX_JOINTS {complete=false;continue;}
        if !ctx.world.joints.is_alive(j) {
            continue;
        }
        let held = ctx.world.joint(j).body1;
        if held==NONE || held as usize>=MAX_BODIES {complete=false;continue;}
        if body::is_static(ctx, held) || ctx.character.bodies.contains(&held) {
            continue;
        }
        // Count joints on the held body other than the character's grips.
        let (others,ended)=other_grip_edges(ctx.world.body(held).joint_list,&ctx.character.grip_joints,|edge| {
            let owner=edge_owner(edge);
            ((owner as usize)<MAX_JOINTS).then(|| (owner,ctx.world.joint_edge(edge).next))
        });
        complete &= ended;
        // Do not convert an unknown partial chain into a positive jointed grip.
        if ended && others > 0 {
            n += 1;
        }
    }
    (n,complete)
}

pub fn jointed_grips(ctx: &State) -> u32 {jointed_grips_with_status(ctx).0}

#[cfg(test)]
mod bounded_grip_diagnostic_tests {
    use super::*;
    #[test] fn acyclic_diagnostic_preserves_non_grip_count() {
        let chain=[(5,2),(6,3),(7,NONE)];
        let result=other_grip_edges(1,&[6],|edge|chain.get((edge-1) as usize).copied());
        assert_eq!(result,(2,true));
    }
    #[test] fn cyclic_or_out_of_range_body_links_end_and_remain_incomplete() {
        let mut reads=0;let result=other_grip_edges(1,&[],|edge| {reads+=1;Some((edge_owner(edge),1))});
        assert_eq!(result,(1,false));assert_eq!(reads,1);
        let mut reads=0;let result=other_grip_edges((MAX_JOINTS*2) as u32,&[],|_| {reads+=1;None});
        assert_eq!(result,(0,false));assert_eq!(reads,0);
        let result=other_grip_edges(1,&[],|_|None);assert_eq!(result,(0,false));
    }
    #[test] fn maximal_acyclic_read_is_bounded_by_the_two_nodes_per_joint() {
        let mut reads=0;let result=other_grip_edges(0,&[],|edge| {reads+=1;
            Some((edge_owner(edge),if edge+1<(MAX_JOINTS*2) as u32 {edge+1} else {NONE}))});
        assert_eq!(result,(MAX_JOINTS as u32*2,true));assert_eq!(reads,MAX_JOINTS*2);
    }
}

pub fn qe_raw(ctx: &State) -> f64 {
    let mut m: f64 = 0.0;
    let mut measure = |b| {
        if b != NONE && ctx.world.bodies.is_alive(b) {
            let v = ctx.world.body(b).linear_velocity;
            m = m.max((v.x * v.x + v.y * v.y).sqrt());
        }
    };
    let dad = ctx.character.primary.kind == 3;
    let character = if dad { &ctx.character.primary } else { &*ctx.character };
    for &b in character.bodies.iter() {
        measure(b);
    }
    if dad {
        for &b in &ctx.character.primary.bicycle.bodies {
            measure(b);
        }
        if ctx.character.child_present.get() {
            for &b in &ctx.character.child.bodies {
                measure(b);
            }
            measure(ctx.character.child.child_seat.seat);
        }
    }
    m
}

pub fn qe(ctx: &State) -> f64 {
    if ctx.character.primary.kind != 3 {
        return qe_raw(ctx);
    }
    let root = ctx.character.primary.bodies[CHEST];
    if !ctx.world.bodies.is_alive(root) || body::is_static(ctx, root) {
        return 0.0;
    }
    // Follow physical world links, including replacement stumps and ligament chains
    // outside the actor's body arrays. A live joint marked `broken` still constrains
    // bodies; destroyed retained joint records do not. Static anchors never bridge
    // otherwise separate dynamic components.
    let mut seen = [false; MAX_BODIES];
    let mut queue = [NONE; MAX_BODIES];
    seen[root as usize] = true;
    queue[0] = root;
    let (mut head, mut tail) = (0, 1);
    let mut m: f64 = 0.0;
    while head < tail {
        let b = queue[head];
        head += 1;
        let v = ctx.world.body(b).linear_velocity;
        m = m.max((v.x * v.x + v.y * v.y).sqrt());
        let mut e = ctx.world.body(b).joint_list;
        // Even malformed linked lists cannot make this diagnostic loop forever.
        for _ in 0..2 * MAX_JOINTS {
            if e == NONE || edge_owner(e) as usize >= MAX_JOINTS {
                break;
            }
            let j = edge_owner(e);
            let link = ctx.world.joint_edge(e);
            e = link.next;
            let other = link.other;
            if !ctx.world.joints.is_alive(j) || other as usize >= MAX_BODIES
                || !ctx.world.bodies.is_alive(other) || body::is_static(ctx, other)
                || seen[other as usize]
            {
                continue;
            }
            seen[other as usize] = true;
            queue[tail] = other;
            tail += 1;
        }
    }
    m
}

impl Sim {
    /// Requires a large stack (Ctx::new builds on the stack); call once and clone.
    pub fn new(level: u32, character: u32) -> Result<Self, String> {
        let mut ctx = Box::new(State::new());
        match session::create(&mut ctx, level, character) {
            Flow::Continue => Ok(Sim { ctx, replay: ReplayState::new(u32::MAX), masks: Vec::new() }),
            f => Err(format!("session::create({level}, {character}): {f:?}")),
        }
    }

    pub fn fork(&self) -> Self {
        Sim { ctx: self.ctx.boxed_copy(), replay: self.replay, masks: self.masks.clone() }
    }

    /// Overwrite `self` with `other` without allocating (the scratch is not copied).
    pub fn restore(&mut self, other: &Sim) {
        self.ctx.restore_from(&other.ctx);
        self.replay = other.replay;
        self.masks.clear();
        self.masks.extend_from_slice(&other.masks);
    }

    pub fn frame(&self) -> u32 {
        self.replay.frames
    }

    /// Step one frame. `mask` is used only if this frame reads a new input entry.
    pub fn step(&mut self, mask: u8) -> Frame {
        let alive_before = !self.ctx.level.dead.get();
        let chest_before = chest(&self.ctx);
        let finish_before = self.replay.finish_frame;
        let i = self.replay.iteration as usize;
        let used = if i < self.masks.len() {
            self.masks[i]
        } else {
            self.masks.push(mask);
            mask
        };
        let c0 = thread_cycles();
        let hang = HANG_OVERRIDE.with(|h| h.get()).unwrap_or_else(|| HANG_MS.load(std::sync::atomic::Ordering::Relaxed));
        if hang != 0 {
            hw_core::lab::set_deadline(Some(std::time::Instant::now() + std::time::Duration::from_millis(hang as u64)));
        }
        let checks = FINISH_CHECKS.load(std::sync::atomic::Ordering::Relaxed) && !hw_core::lab::armed();
        if checks {
            hw_core::lab::arm(&[]);
        }
        let flow = session::step_bits(&mut self.ctx, &mut self.replay, Input::from_mask(used as u32));
        let (mut fc, mut fc_pos) = (f64::INFINITY, [f64::NAN; 2]);
        if checks {
            hw_core::lab::disarm();
            for e in hw_core::lab::take_events() {
                if let hw_core::lab::Event::Finish { x, y, dead: false, .. } = e {
                    let d = finish_region_distance(&self.ctx, x, y);
                    if d < fc {
                        fc = d;
                        fc_pos = [x, y];
                    }
                }
            }
        }
        if hang != 0 {
            hw_core::lab::set_deadline(None);
        }
        self.ctx.end_frame();
        let step_us = ((thread_cycles() - c0) as f64 / cycles_per_us()).min(u32::MAX as f64) as u32;
        let lag = LAG_US.load(std::sync::atomic::Ordering::Relaxed);
        let site = match flow {
            Flow::Continue => 0,
            Flow::Abort(s) => s as u32,
        };
        let chest_after = chest(&self.ctx);
        let c = self.ctx.character.bodies[CHEST];
        let qe = qe(&self.ctx);
        let qe_raw = if self.ctx.character.primary.kind == 3 { qe_raw(&self.ctx) } else { qe };
        let (jointed_grips,jointed_grips_complete)=jointed_grips_with_status(&self.ctx);
        Frame {
            frame: self.replay.frames,
            mask: used,
            site,
            port_abort: matches!(
                flow,
                Flow::Abort(AbortSite::PortCapacity | AbortSite::PortUnimplemented | AbortSite::B2AssertPolyCircle)
            ) || self.ctx.world.bodies.anomaly()
                || self.ctx.world.shapes.anomaly()
                || self.ctx.world.joints.anomaly()
                || self.ctx.world.contacts.anomaly(),
            alive_before,
            alive_after: !self.ctx.level.dead.get(),
            finished: finish_before == 0 && self.replay.finish_frame != 0,
            chest_before,
            chest_after,
            displacement: ((chest_after[0] - chest_before[0]).powi(2) + (chest_after[1] - chest_before[1]).powi(2)).sqrt(),
            qe,
            qe_raw,
            ejected: ejected(&self.ctx),
            grips: self.ctx.character.grip_joints.iter().filter(|&&j| j != NONE).count() as u32,
            static_grips: static_grips(&self.ctx),
            jointed_grips,
            jointed_grips_complete,
            chest_alive: c != NONE && self.ctx.world.bodies.is_alive(c),
            step_us,
            lagging: lag != 0 && step_us > lag,
            fc,
            fc_pos,
            pairs: self.ctx.world.broad_phase.pair_count,
            contacts: self.ctx.world.contact_count,
        }
    }

    /// The input string so far, in TAS text form (8 characters per entry).
    pub fn tas(&self) -> String {
        tas_text(&self.masks)
    }
}

pub fn tas_text(masks: &[u8]) -> String {
    let mut s = String::with_capacity(masks.len() * 8);
    for &m in masks {
        for k in 0..8 {
            s.push(if m & (1 << k) != 0 { '1' } else { '0' });
        }
    }
    s
}

pub fn parse_tas(text: &str) -> Result<Vec<u8>, String> {
    let bits: Vec<u8> = text.bytes().filter(|b| !b.is_ascii_whitespace()).collect();
    if bits.len() % 8 != 0 || !bits.iter().all(|&b| b == b'0' || b == b'1') {
        return Err("TAS must be 0/1 characters in complete eight-bit frames".into());
    }
    Ok(bits.chunks(8).map(|c| c.iter().enumerate().fold(0u8, |m, (k, &b)| m | (((b == b'1') as u8) << k))).collect())
}

/// A TAS file path or literal bits.
pub fn read_tas(arg: &str) -> Result<Vec<u8>, String> {
    if arg.bytes().all(|b| b == b'0' || b == b'1') {
        parse_tas(arg)
    } else {
        parse_tas(&std::fs::read_to_string(arg).map_err(|e| format!("{arg}: {e}"))?)
    }
}

#[cfg(test)]
mod tests {
    use super::{qe, qe_raw, Sim, State};
    use hw_core::b2::{joints, math::Vec2, state::{BodyDef, JointDef}, world};
    use hw_core::game::{bicycle, character::CHEST, segway};
    use hw_core::{Flag32, Flow, H, NONE};

    #[test]
    fn dad_qe_tracks_mounted_fuel_but_excludes_it_after_eject() {
        std::thread::Builder::new().stack_size(64 << 20).spawn(|| {
            let mut sim = Sim::new(45, 3).unwrap();
            let chest = sim.ctx.character.primary.bodies[CHEST];
            let bike = sim.ctx.character.primary.bicycle.bodies[bicycle::FRAME];
            let child = sim.ctx.character.child.bodies[CHEST];
            let seat = sim.ctx.character.child.child_seat.seat;
            sim.ctx.world.body_mut(chest).linear_velocity = Vec2::new(3.0, 4.0);
            assert_eq!(qe(&sim.ctx), 5.0);
            sim.ctx.world.body_mut(bike).linear_velocity = Vec2::new(100.0, 0.0);
            assert_eq!(qe(&sim.ctx), 100.0);
            sim.ctx.world.body_mut(child).linear_velocity = Vec2::new(0.0, 200.0);
            assert_eq!(qe(&sim.ctx), 200.0);
            sim.ctx.character.child_active = hw_core::Flag32::TRUE;
            assert_eq!(qe(&sim.ctx), 200.0);
            sim.ctx.world.body_mut(seat).linear_velocity = Vec2::new(0.0, 300.0);
            assert_eq!(qe(&sim.ctx), 300.0);
            sim.ctx.character.child_active = hw_core::Flag32::FALSE;
            assert_eq!(bicycle::eject(&mut sim.ctx), Flow::Continue);
            assert_eq!(qe(&sim.ctx), 5.0);
            assert_eq!(qe_raw(&sim.ctx), 300.0);
        }).unwrap().join().unwrap();
    }

    fn attach(ctx: &mut State, a: H, b: H) -> H {
        let mut def = JointDef::revolute();
        joints::revolute_def_initialize(ctx, &mut def, a, b, Vec2::zero());
        let (flow, joint) = world::create_joint(ctx, &def);
        assert_eq!(flow, Flow::Continue);
        assert_ne!(joint, NONE);
        joint
    }

    #[test]
    fn dad_qe_includes_live_broken_replacements_without_a_static_bridge() {
        std::thread::Builder::new().stack_size(64 << 20).spawn(|| {
            let mut sim = Sim::new(45, 3).unwrap();
            let ctx = &mut sim.ctx;
            let chest = ctx.character.primary.bodies[CHEST];
            ctx.world.body_mut(chest).linear_velocity = Vec2::new(3.0, 4.0);
            assert_eq!(bicycle::eject(ctx), Flow::Continue);
            let mut dynamic = BodyDef::new();
            dynamic.mass_data.mass = 1.0;
            let stump = world::create_body(ctx, &dynamic);
            let loose = world::create_body(ctx, &dynamic);
            let ground = world::create_body(ctx, &BodyDef::new());
            assert!(stump != NONE && loose != NONE && ground != NONE);
            ctx.world.body_mut(stump).linear_velocity = Vec2::new(900.0, 0.0);
            ctx.world.body_mut(loose).linear_velocity = Vec2::new(1500.0, 0.0);
            let replacement = attach(ctx, chest, stump);
            ctx.world.joint_mut(replacement).broken = Flag32::TRUE;
            assert_eq!(qe(ctx), 900.0);
            attach(ctx, chest, ground);
            attach(ctx, ground, loose);
            assert_eq!(qe(ctx), 900.0);
            assert_eq!(world::destroy_joint(ctx, replacement), Flow::Continue);
            assert_eq!(qe(ctx), 5.0);
        }).unwrap().join().unwrap();
    }

    #[test]
    fn other_characters_keep_the_character_only_metric() {
        std::thread::Builder::new().stack_size(64 << 20).spawn(|| {
            let mut sim = Sim::new(45, 2).unwrap();
            let chest = sim.ctx.character.bodies[CHEST];
            let vehicle = sim.ctx.character.segway.bodies[segway::FRAME];
            sim.ctx.world.body_mut(chest).linear_velocity = Vec2::new(3.0, 4.0);
            sim.ctx.world.body_mut(vehicle).linear_velocity = Vec2::new(300.0, 0.0);
            assert_eq!(qe(&sim.ctx), 5.0);
            assert_eq!(qe_raw(&sim.ctx), 5.0);
        }).unwrap().join().unwrap();
    }
}
