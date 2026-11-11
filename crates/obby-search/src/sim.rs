//! One Obstacle Course replay under search control, with the `hw sim` frame contract: inputs are
//! indexed by iteration (an input-phase abort does not consume its entry), world-stage throws keep
//! their partial mutations exactly like the Flash ENTER_FRAME handler.
use hw_core::b2::body;
use hw_core::game::character::CHEST;
use hw_core::game::session::{self, ReplayState};
use hw_core::{AbortSite, Ctx, Flow, Input, H, NONE};

pub type State = Ctx<f64>;

pub struct Sim {
    pub ctx: Box<State>,
    pub replay: ReplayState,
    pub masks: Vec<u8>,
}

/// What one frame did.
#[derive(Clone, Copy, Debug, Default)]
pub struct Fr {
    pub frame: u32,
    pub mask: u8,
    /// The frame read a new input entry (iteration advanced).
    pub consumed: bool,
    /// 0 = completed; else the AbortSite number of the throw that ended the frame.
    pub site: u32,
    /// Port-only abort or arena anomaly: the branch left verified territory.
    pub port_abort: bool,
    pub alive_before: bool,
    pub alive_after: bool,
    pub finished: bool,
    pub chest_before: [f64; 2],
    pub chest_after: [f64; 2],
    pub chest_v: [f64; 2],
    pub pairs: i32,
    pub proxies: i32,
    pub lock: bool,
    /// Fastest dynamic body in the whole world after the frame (m/s).
    pub speed_max: f64,
    /// Number of dynamic bodies faster than 300 m/s (swept boxes spanning ~10 m per frame).
    pub fast_bodies: u32,
    pub dad_ejected: bool,
    pub child_ejected: bool,
    pub step_us: u32,
}

static HANG_MS: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(400);

pub fn set_hang_ms(ms: u32) {
    HANG_MS.store(ms, std::sync::atomic::Ordering::Relaxed);
}

pub fn chest_body(ctx: &State) -> H {
    ctx.character.primary.bodies[CHEST]
}

pub fn chest(ctx: &State) -> [f64; 2] {
    let c = chest_body(ctx);
    if c == NONE || !ctx.world.bodies.is_alive(c) {
        return [f64::NAN; 2];
    }
    let p = body::get_world_center(ctx, c);
    [p.x, p.y]
}

pub fn chest_velocity(ctx: &State) -> [f64; 2] {
    let c = chest_body(ctx);
    if c == NONE || !ctx.world.bodies.is_alive(c) {
        return [f64::NAN; 2];
    }
    let v = ctx.world.body(c).linear_velocity;
    [v.x, v.y]
}

impl Sim {
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

    pub fn restore(&mut self, other: &Sim) {
        self.ctx.restore_from(&other.ctx);
        self.replay = other.replay;
        self.masks.clear();
        self.masks.extend_from_slice(&other.masks);
    }

    pub fn frame(&self) -> u32 {
        self.replay.frames
    }

    /// Step one frame; `mask` is used only if this frame reads a new input entry.
    pub fn step(&mut self, mask: u8) -> Fr {
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
        let t0 = std::time::Instant::now();
        let hang = HANG_MS.load(std::sync::atomic::Ordering::Relaxed);
        if hang != 0 {
            hw_core::lab::set_deadline(Some(t0 + std::time::Duration::from_millis(hang as u64)));
        }
        let flow = session::step_bits(&mut self.ctx, &mut self.replay, Input::from_mask(used as u32));
        if hang != 0 {
            hw_core::lab::set_deadline(None);
        }
        self.ctx.end_frame();
        let step_us = t0.elapsed().as_micros().min(u32::MAX as u128) as u32;
        let site = match flow {
            Flow::Continue => 0,
            Flow::Abort(s) => s as u32,
        };
        let w = &self.ctx.world;
        let (mut speed_max, mut fast) = (0.0f64, 0u32);
        let mut b = w.body_list;
        let mut guard = 0;
        while b != NONE && guard < hw_core::b2::state::MAX_BODIES {
            guard += 1;
            let bd = w.body(b);
            if bd.inv_mass > 0.0 {
                let v = bd.linear_velocity;
                let s = (v.x * v.x + v.y * v.y).sqrt();
                if s > speed_max {
                    speed_max = s;
                }
                if s > 300.0 {
                    fast += 1;
                }
            }
            b = bd.next;
        }
        Fr {
            frame: self.replay.frames,
            mask: used,
            consumed: self.replay.iteration as usize > i,
            site,
            port_abort: matches!(
                flow,
                Flow::Abort(AbortSite::PortCapacity | AbortSite::PortUnimplemented | AbortSite::B2AssertPolyCircle)
            ) || w.bodies.anomaly()
                || w.shapes.anomaly()
                || w.joints.anomaly()
                || w.contacts.anomaly(),
            alive_before,
            alive_after: !self.ctx.level.dead.get(),
            finished: finish_before == 0 && self.replay.finish_frame != 0,
            chest_before,
            chest_after: chest(&self.ctx),
            chest_v: chest_velocity(&self.ctx),
            pairs: w.broad_phase.pair_count,
            proxies: w.broad_phase.proxy_count,
            lock: w.lock.get(),
            speed_max,
            fast_bodies: fast,
            dad_ejected: self.ctx.character.primary.bicycle.ejected.get(),
            child_ejected: self.ctx.character.child.child_seat.ejected.get(),
            step_us,
        }
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
        return Err(format!("TAS must be whole 8-bit frames of 0/1 (got {} bits)", bits.len()));
    }
    Ok(bits.chunks(8).map(|c| c.iter().enumerate().fold(0u8, |m, (k, &b)| m | (((b == b'1') as u8) << k))).collect())
}

pub fn read_tas(arg: &str) -> Result<Vec<u8>, String> {
    let text = std::fs::read_to_string(arg).map_err(|e| format!("{arg}: {e}"))?;
    parse_tas(&text)
}

/// Top surface of the highest static geometry at each x (0.25 m bins from x = -4 m): a chest
/// centre above it is in open sky (only the world's top border remains above).
pub struct Skyline {
    pub top: Vec<f64>,
}

impl Skyline {
    pub fn new(ctx: &State) -> Self {
        use hw_core::b2::shapes;
        let n = ((328.0 / 0.25) as usize) + 1;
        let mut top = vec![-80.0; n];
        let w = &ctx.world;
        let mut b = w.body_list;
        let mut guard = 0;
        while b != NONE && guard < hw_core::b2::state::MAX_BODIES {
            guard += 1;
            let bd = w.body(b);
            if bd.inv_mass == 0.0 {
                let xf = bd.xf;
                let mut sh = bd.shape_list;
                let mut g2 = 0;
                while sh != NONE && g2 < 4096 {
                    g2 += 1;
                    let s = w.shape(sh);
                    if !s.is_sensor.get() {
                        let (lo, hi) = shapes::compute_aabb(ctx, sh, &xf);
                        // Skip the world borders.
                        if hi.x - lo.x < 200.0 && hi.y - lo.y < 150.0 && lo.y > -79.0 {
                            let i0 = (((lo.x + 4.0) / 0.25).floor().max(0.0)) as usize;
                            let i1 = (((hi.x + 4.0) / 0.25).ceil() as usize).min(n - 1);
                            for i in i0..=i1 {
                                if lo.y > top[i] {
                                    // keep the highest (smallest y) surface; start from -80 meaning open
                                }
                                if top[i] == -80.0 || lo.y < top[i] {
                                    top[i] = lo.y;
                                }
                            }
                        }
                    }
                    sh = s.next;
                }
            }
            b = bd.next;
        }
        Skyline { top }
    }

    /// Height of the chest centre below the skyline at its x (negative = above, in open sky).
    pub fn gap(&self, p: [f64; 2]) -> f64 {
        if !p[0].is_finite() || !p[1].is_finite() {
            return f64::INFINITY;
        }
        let i = (((p[0] + 4.0) / 0.25).floor().max(0.0) as usize).min(self.top.len() - 1);
        let t = self.top[i];
        if t == -80.0 { f64::INFINITY } else { p[1] - t }
    }

    /// The chest centre is above the topmost geometry at its x (with a margin for its size).
    pub fn above(&self, p: [f64; 2]) -> bool {
        if !p[0].is_finite() || !p[1].is_finite() {
            return false;
        }
        let i = (((p[0] + 4.0) / 0.25).floor().max(0.0) as usize).min(self.top.len() - 1);
        let t = self.top[i];
        // In open sky but still inside the world (the top border's underside is y = -80).
        (t == -80.0 || p[1] < t - 0.6) && p[1] > -79.0 && p[0] > 0.0 && p[0] < 320.0
    }
}
