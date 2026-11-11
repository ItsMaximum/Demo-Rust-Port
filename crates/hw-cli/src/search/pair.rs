//! Dedicated Dad/Obstacle pair-occupancy search. See pair_policy.rs for clean windows.
//! Initial full bases are protected; all replays start fresh. Native confirmation
//! remains separate from original-player verification. No physics state is repaired.
#[path = "pair_policy.rs"]
mod policy;
use policy::{LagGuard, Observation, PairWindow, ANCHOR, MAX_FRAME, WINDOW};
use super::explore::confirm_any_finish;
use super::sim::{tas_text, Sim};
use hw_core::b2::{body, state::MAX_BODIES};
use hw_core::game::{character::CHEST, session::ReplayState};
use hw_core::NONE;
use serde_json::{json, Value};
use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

pub struct Config {
    pub level: u32, pub character: u32, pub bases: Vec<Vec<u8>>, pub run_dir: PathBuf,
    pub threads: usize, pub hours: f64, pub seed: u64, pub window: usize,
    pub extra: usize, pub pool_size: usize, pub lag_ms: f64, pub lag_frames: u32,
}
impl Config {
    pub fn validate(&self) -> Result<(), String> {
        if (self.level, self.character) != (45, 3) { return Err("pair requires level45/character3".into()); }
        if !self.hours.is_finite() || self.hours < 0.0 || self.threads > 18
            || (self.threads == 0 && self.hours != 0.0) { return Err("pair requires finite nonnegative hours; live workers1..18; workers0 only with hours0".into()); }
        if Duration::try_from_secs_f64(self.hours * 3600.0).is_err() { return Err("pair hours exceed the representable budget".into()); }
        if !self.lag_ms.is_finite() || self.lag_ms < 0.0 || self.lag_ms * 1000.0 > u32::MAX as f64 || self.lag_frames == 0 {
            return Err("pair requires finite lag-ms0..4294967.295 and lag-frames>=1 (lag-ms0 disables lag detection)".into());
        }
        if self.pool_size == 0 || !(1..=MAX_FRAME as usize - ANCHOR).contains(&self.window)
            || self.extra > MAX_FRAME as usize { return Err("pair requires pool>0, window1..239, extra0..299".into()); }
        if self.bases.is_empty() || self.bases.iter().any(|b| !(ANCHOR + 2..=MAX_FRAME as usize).contains(&b.len())) {
            return Err("pair requires at least one full62..299-entry base".into());
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
struct Score { value: i32, frame: u32, iteration: u32, counts: [i32; WINDOW] }
impl Score {
    fn value(self) -> Value { json!({"sustained_pairs":self.value,"frame":self.frame,
        "iteration":self.iteration,"window_counts":self.counts,"window_frames":WINDOW}) }
}
#[derive(Clone)]
struct Member {
    id: u64, ancestor: Option<u64>, root: u64, generation: u64, source_ordinals: Vec<usize>,
    protected: bool, masks: Vec<u8>, score: Option<Score>, visits: u64,
}
impl Member {
    fn value(&self) -> Value { json!({"schema":1,"score_version":"pair-occupancy-v1",
        "id":self.id,"ancestor":self.ancestor,"root":self.root,"generation":self.generation,
        "protected":self.protected,"source_ordinals":self.source_ordinals,"visits":self.visits,
        "frame":self.masks.len(),"tas":tas_text(&self.masks),"best_window":self.score.map(Score::value)}) }
}
struct Pool { members: Vec<Member>, next_id: u64 }
#[derive(Debug, PartialEq, Eq)]
enum Admission { Duplicate, BelowFloor, Added { id: u64, evicted: Option<u64>, equal_count: bool } }
impl Pool {
    fn new(bases: &[Vec<u8>]) -> Self {
        let mut members: Vec<Member> = Vec::new();
        for (ordinal, masks) in bases.iter().enumerate() {
            if let Some(m) = members.iter_mut().find(|m| m.masks == *masks) { m.source_ordinals.push(ordinal); }
            else { let id = members.len() as u64; members.push(Member { id, ancestor: None, root:id,
                generation:0, source_ordinals:vec![ordinal], protected:true, masks:masks.clone(), score:None, visits:0 }); }
        }
        Self { next_id:members.len() as u64, members }
    }
    fn pick(&mut self, rng: &mut Rng) -> Member {
        let n = self.members.len();
        // 25% uniform, otherwise best of two. Equal scores favour less visited;
        // no score-only dedup and unscored protected bases remain reachable.
        let a = rng.below(n); let b = if rng.below(4) == 0 { a } else { rng.below(n) };
        let rank = |m: &Member| (m.score.map(|s| s.value).unwrap_or(-1), u64::MAX - m.visits);
        let i = if rank(&self.members[b]) > rank(&self.members[a]) { b } else { a };
        self.members[i].visits = self.members[i].visits.saturating_add(1);
        self.members[i].clone()
    }
    fn admit(&mut self, parent: &Member, masks: Vec<u8>, score: Score, cap: usize) -> Admission {
        if self.members.iter().any(|m| m.masks == masks) { return Admission::Duplicate; }
        let equal_count = self.members.iter().any(|m| m.score.is_some_and(|s| s.value == score.value));
        let ordinary: Vec<usize> = self.members.iter().enumerate().filter_map(|(i,m)| (!m.protected).then_some(i)).collect();
        let mut evicted = None;
        if ordinary.len() >= cap {
            let floor = ordinary.iter().map(|&i| self.members[i].score.unwrap().value).min().unwrap();
            if score.value < floor { return Admission::BelowFloor; }
            // A new equal-floor TAS gets a real place: most tried, then oldest,
            // among the lowest-count ordinary members. Protected bases never evict.
            let i = *ordinary.iter().filter(|&&i| self.members[i].score.unwrap().value == floor)
                .max_by_key(|&&i| (self.members[i].visits, u64::MAX - self.members[i].id)).unwrap();
            evicted = Some(self.members.remove(i).id);
        }
        let id = self.next_id; self.next_id += 1;
        self.members.push(Member { id, ancestor:Some(parent.id), root:parent.root,
            generation:parent.generation.saturating_add(1), source_ordinals:vec![], protected:false,
            masks, score:Some(score), visits:0 });
        Admission::Added { id, evicted, equal_count }
    }
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 { self.0 = self.0.wrapping_add(0x9e3779b97f4a7c15);
        let mut z = self.0; z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb); z ^ (z >> 31) }
    fn below(&mut self, n: usize) -> usize { (self.next() % n as u64) as usize }
}
fn first_change(a: &[u8], b: &[u8]) -> Option<usize> {
    a.iter().zip(b).position(|(x,y)| x != y).or_else(|| (a.len() != b.len()).then_some(a.len().min(b.len())))
}
/// All edit operators see only suffix entries. Retriggers retain at least two.
fn mutate_suffix(rng: &mut Rng, masks: &mut Vec<u8>, window: usize) {
    let n = masks.len(); let w = window.min(n); let pos = n - 1 - rng.below(w);
    match rng.below(6) {
        0 => { let end = (pos + 1 + rng.below(8)).min(n); for m in &mut masks[pos..end] { *m ^= 16; } }
        1 => { let bit = 1u8 << rng.below(8); let end = (pos + 1 + rng.below(8)).min(n);
            for m in &mut masks[pos..end] { *m ^= bit; } }
        2 => { const POSES: [u8; 9] = [0,1,2,4,8,16,32,64,128];
            let pose = POSES[rng.below(POSES.len())] | if rng.below(4) != 0 {16} else {0};
            let end = (pos + 1 + rng.below(24)).min(n); masks[pos..end].fill(pose); }
        3 => { // Shift the nearest selected-key transition, remaining in suffix.
            let bit = 1u8 << rng.below(8); let mut t = pos;
            while t + 1 < n && (masks[t] ^ masks[t+1]) & bit == 0 { t += 1; }
            if t + 1 < n { let shift = 1 + rng.below(3);
                if rng.below(2) == 0 { let v = masks[t+1] & bit;
                    for m in &mut masks[t.saturating_sub(shift-1)..=t] { *m = (*m & !bit) | v; } }
                else { let v = masks[t] & bit; let end = (t+1+shift).min(n);
                    for m in &mut masks[t+1..end] { *m = (*m & !bit) | v; } }
            } else { masks[pos] ^= bit; } }
        4 => { let t = (n - rng.below(w)).max(2); masks.truncate(t);
            for m in &mut masks[t-2..t] { *m = (*m & 131) | ((rng.below(32) as u8) << 2); } }
        _ => masks[pos] = rng.below(256) as u8,
    }
}
fn proposal(parent: &[u8], rng: &mut Rng, window: usize, extra: usize) -> Result<Option<(Vec<u8>, usize)>, String> {
    let mut suffix = parent[ANCHOR..].to_vec(); mutate_suffix(rng, &mut suffix, window);
    let mut masks = parent[..ANCHOR].to_vec(); masks.extend(suffix);
    // The existing finite confirmer runs N+60 world frames. Every live plan
    // therefore has at least239 explicit entries so confirmation covers299.
    // Appended controls are saved in the literal; protected original bases stay full and unchanged.
    let end = masks.len().saturating_add(extra).max(MAX_FRAME as usize - 60).min(MAX_FRAME as usize);
    let mut last = *masks.last().unwrap();
    while masks.len() < end { if rng.below(30) == 0 { last = rng.below(256) as u8; } masks.push(last); }
    if !policy::valid_anchored_child(parent, &masks) { return Err("pair mutation violated immutable prefix/length contract".into()); }
    // No change, or a pure shortening with no post-change planned input: counted
    // unscored/no-op, never a lane error or an earlier-window improvement.
    Ok(first_change(parent, &masks).filter(|&i| i < masks.len()).map(|i| (masks,i)))
}

#[derive(Default)]
struct Counts {
    frames: AtomicU64, trials: AtomicU64, initial_replays: AtomicU64, launches: AtomicU64,
    no_progress: AtomicU64, locked: AtomicU64, port_aborts: AtomicU64, clean_frames: AtomicU64, windows: AtomicU64,
    site2: AtomicU64, admissions: AtomicU64, equal_admissions: AtomicU64, duplicates: AtomicU64,
    noops: AtomicU64, observed_finishes: AtomicU64, confirmed_finishes: AtomicU64,
    lagging_frames: AtomicU64, lag_stops: AtomicU64,
}
struct Logs { improvements: File, launches: File, pairmax: File, finishes: File, trials: File }
impl Logs {
    fn new(dir: &Path) -> Result<Self, String> { Ok(Self {
        improvements:fresh_file(&dir.join("improvements.jsonl"))?, launches:fresh_file(&dir.join("launches.jsonl"))?,
        pairmax:fresh_file(&dir.join("pairmax-events.jsonl"))?, finishes:fresh_file(&dir.join("finishes.jsonl"))?,
        trials:fresh_file(&dir.join("trials.jsonl"))? }) }
}
struct Shared {
    pool: Mutex<Pool>, stop: AtomicBool, error: Mutex<Option<String>>, winner: Mutex<Option<Vec<u8>>>,
    logs: Mutex<Logs>, counts: Counts, sites: Mutex<[u64;12]>, best: Mutex<Option<Score>>,
    maximum_current_pairs: Mutex<i32>, start: Instant, deadline: Instant,
}
impl Shared {
    fn should_stop(&self, cfg: &Config) -> bool {
        if self.stop.load(Ordering::Relaxed) { return true; }
        if Instant::now() >= self.deadline || cfg.run_dir.join("STOP").exists() {
            self.stop.store(true, Ordering::Relaxed); return true;
        }
        false
    }
    fn fail(&self, error: String) {
        let mut e = self.error.lock().unwrap(); if e.is_none() { *e = Some(error); }
        self.stop.store(true, Ordering::Relaxed);
    }
    fn emit(&self, which: &str, value: Value) -> Result<(), String> {
        let mut logs = self.logs.lock().map_err(|_| "pair log mutex poisoned")?;
        let out = match which { "improvements" => &mut logs.improvements, "launches" => &mut logs.launches,
            "pairmax" => &mut logs.pairmax, "finishes" => &mut logs.finishes, _ => &mut logs.trials };
        write_line(out, &value)
    }
}
fn fresh_file(path: &Path) -> Result<File, String> {
    OpenOptions::new().write(true).create_new(true).open(path).map_err(|e| format!("{}: {e}",path.display()))
}
fn write_line(out: &mut impl Write, value: &Value) -> Result<(), String> {
    serde_json::to_writer(&mut *out, value).map_err(|e| e.to_string())?;
    out.write_all(b"\n").map_err(|e| e.to_string())
}
fn export_pool(shared: &Shared, path: &Path) -> Result<(), String> {
    let snapshot = shared.pool.lock().map_err(|_| "pair pool mutex poisoned")?.members.clone();
    let mut out = BufWriter::new(fresh_file(path)?);
    for m in &snapshot { write_line(&mut out, &m.value())?; }
    out.flush().map_err(|e| e.to_string())?; out.get_ref().sync_all().map_err(|e| e.to_string())
}
fn stats(shared: &Shared) -> Value {
    let load = |x: &AtomicU64| x.load(Ordering::Relaxed);
    let p = shared.pool.lock().unwrap(); let c = &shared.counts;
    json!({"schema":1,"mode":"pair","t":shared.start.elapsed().as_secs_f64(),"level":45,"character":3,
        "anchor":ANCHOR,"pair_window":WINDOW,"max_frame":MAX_FRAME,"frames":load(&c.frames),"trials":load(&c.trials),
        "initial_replays":load(&c.initial_replays),"pool":p.members.len(),"protected":p.members.iter().filter(|m| m.protected).count(),
        "launches":load(&c.launches),"input_no_progress_frames":load(&c.no_progress),"locked_frames":load(&c.locked),
        "port_aborts":load(&c.port_aborts),"lagging_frames":load(&c.lagging_frames),"lag_stops":load(&c.lag_stops),
        "abort_site_histogram":*shared.sites.lock().unwrap(),"clean_frames":load(&c.clean_frames),"scored_windows":load(&c.windows),
        "site2_events":load(&c.site2),"admissions":load(&c.admissions),"equal_count_admissions":load(&c.equal_admissions),
        "current_pool_raw_duplicates":load(&c.duplicates),"noops_or_unscorable_shortening":load(&c.noops),
        "maximum_observed_current_pairs":*shared.maximum_current_pairs.lock().unwrap(),"best_window":shared.best.lock().unwrap().map(Score::value),
        "observed_finishes":load(&c.observed_finishes),"confirmed_finishes":load(&c.confirmed_finishes),
        "winner":shared.winner.lock().unwrap().is_some(),"stop_requested":shared.stop.load(Ordering::Relaxed)})
}
fn chest_state(s: &Sim) -> (bool, [f64;2]) {
    let h = s.ctx.character.primary.bodies[CHEST];
    if h == NONE || h as usize >= MAX_BODIES || !s.ctx.world.bodies.is_alive(h) { return (false,[f64::NAN;2]); }
    let p = body::get_world_center(&s.ctx,h); (true,[p.x,p.y])
}
fn in_world(p: [f64;2]) -> bool { p[0].is_finite() && p[1].is_finite()
    && p[0] > -4.0 && p[0] < 324.0 && p[1] > -84.0 && p[1] < 164.0 }
fn continuation(o: Observation) -> bool { !o.port_abort && o.alive_after && o.chest_alive_after
    && in_world(o.chest_after) && o.frame_after < MAX_FRAME }

fn observed_finish(cfg: &Config, shared: &Shared, masks: &[u8], frame: u32) -> Result<(), String> {
    shared.counts.observed_finishes.fetch_add(1, Ordering::Relaxed);
    let confirmed = confirm_any_finish(cfg.level,cfg.character,masks);
    let qualifies = matches!(&confirmed, Ok((f,_)) if *f == frame && *f <= MAX_FRAME);
    shared.emit("finishes",json!({"schema":1,"observed_frame":frame,"tas":tas_text(masks),
        "native_confirmation":match &confirmed {Ok((f,moved))=>json!({"frame":f,"moved":moved}),Err(e)=>json!({"error":e})},
        "confirmed_within299_and_same_observed_frame":qualifies,"original_player_verified":false}))?;
    if qualifies {
        let mut winner = shared.winner.lock().map_err(|_| "pair winner mutex poisoned")?;
        if winner.is_none() {
            let mut out = fresh_file(&cfg.run_dir.join("winner-1.tas"))?;
            out.write_all(tas_text(masks).as_bytes()).map_err(|e| e.to_string())?;
            out.sync_all().map_err(|e| e.to_string())?;
            *winner = Some(masks.to_vec()); shared.counts.confirmed_finishes.fetch_add(1, Ordering::Relaxed);
            shared.stop.store(true,Ordering::Relaxed);
        }
    }
    Ok(())
}
/// Fresh finite-input replay: full planned literal, then zeros, matching confirmation.
/// World frames bound work independently of aborts and consumed input entries.
fn replay(cfg: &Config, shared: &Shared, masks: &[u8], changed: usize, parent: u64, initial: bool) -> Result<Option<Score>, String> {
    let mut window = PairWindow::new(masks.len(),changed).map_err(str::to_owned)?;
    let mut s = Sim::new(cfg.level,cfg.character)?;
    s.replay = ReplayState::new(masks.len() as u32); s.masks = masks.to_vec();
    let mut best: Option<Score> = None; let mut completed = true; let mut lag=LagGuard::default();
    // Exact short initial bases retain their own finite N+60 confirmation horizon.
    // Live proposals have N>=239 and consequently cover the entire299 ceiling.
    let horizon=MAX_FRAME.min(masks.len() as u32+60);
    let mut reason=if horizon<MAX_FRAME {"finite_confirmation_horizon"} else {"ceiling"};
    for _ in 0..horizon {
        if shared.should_stop(cfg) { completed=false; reason="stop_or_budget"; break; }
        let before_frame=s.frame(); let before_iteration=s.replay.iteration;
        let before_allowed=s.replay.input_allowed.get(); let before_lock=s.ctx.world.lock.get();
        let before_alive=!s.ctx.level.dead.get() && !s.ctx.character.primary.dead.get();
        let (before_chest_alive,before_chest)=chest_state(&s);
        let f=s.step(0); shared.counts.frames.fetch_add(1,Ordering::Relaxed);
        let (after_chest_alive,after_chest)=chest_state(&s);
        let o=Observation {frame_before:before_frame,frame_after:f.frame,iteration_before:before_iteration,
            iteration_after:s.replay.iteration,input_allowed_before:before_allowed,input_allowed_after:s.replay.input_allowed.get(),
            mask_matches_requested:masks.get(before_iteration as usize).is_some_and(|&m| m==f.mask),
            locked_before:before_lock,locked_after:s.ctx.world.lock.get(),alive_before:before_alive,
            alive_after:!s.ctx.level.dead.get() && !s.ctx.character.primary.dead.get(),chest_alive_before:before_chest_alive,
            chest_alive_after:after_chest_alive,chest_before:before_chest,chest_after:after_chest,
            pairs_after:f.pairs,site:f.site,port_abort:f.port_abort,lagging:f.lagging,finished:f.finished,displacement:f.displacement};
        // This must precede port-abort, lock, fitness, liveness and world-bound gates.
        if f.finished { observed_finish(cfg,shared,masks,f.frame)?; }
        if before_allowed && s.replay.iteration <= before_iteration { shared.counts.no_progress.fetch_add(1,Ordering::Relaxed); }
        if o.locked_before || o.locked_after { shared.counts.locked.fetch_add(1,Ordering::Relaxed); }
        if f.port_abort { shared.counts.port_aborts.fetch_add(1,Ordering::Relaxed); }
        if f.lagging { shared.counts.lagging_frames.fetch_add(1,Ordering::Relaxed); }
        let lag_stop=lag.observe(f.lagging,cfg.lag_frames);
        if f.site != 0 { let mut sites=shared.sites.lock().unwrap();
            let i=(f.site as usize).min(sites.len()-1); sites[i]=sites[i].saturating_add(1); }
        { let mut m=shared.maximum_current_pairs.lock().unwrap(); *m=(*m).max(f.pairs); }
        let decision=window.observe(o);
        if window.last_was_clean() {shared.counts.clean_frames.fetch_add(1,Ordering::Relaxed);}
        if decision.observed_pair_pool_exhausted {
            shared.counts.site2.fetch_add(1,Ordering::Relaxed);
            shared.emit("pairmax",json!({"schema":1,"frame":f.frame,"iteration":s.replay.iteration,"site":f.site,
                "pairs_current":f.pairs,"locked_before":o.locked_before,"locked_after":o.locked_after,
                "input_progress":s.replay.iteration.saturating_sub(before_iteration),"parent":parent,"tas":tas_text(masks),
                "occupancy_credit":false,"original_player_verified":false}))?;
        }
        if f.displacement >= 20.0 {
            shared.counts.launches.fetch_add(1,Ordering::Relaxed);
            shared.emit("launches",json!({"schema":1,"frame":f.frame,"iteration":s.replay.iteration,"from":f.chest_before,
                "to":f.chest_after,"moved":f.displacement,"alive_before":f.alive_before,"alive_after":f.alive_after,
                "site":f.site,"locked_before":o.locked_before,"locked_after":o.locked_after,"pairs_current":f.pairs,
                "sustained_pairs":decision.sustained_pairs,"lagging":f.lagging,"parent":parent,"tas":tas_text(masks),"finished":f.finished}))?;
            // Deliberately no early return or endpoint dedup.
        }
        if let Some(value)=decision.sustained_pairs {
            shared.counts.windows.fetch_add(1,Ordering::Relaxed);
            if best.map_or(true,|b| value>b.value) { best=Some(Score {value,frame:f.frame,iteration:s.replay.iteration,
                counts:window.counts().expect("scored window has six counts")}); }
        }
        if shared.stop.load(Ordering::Relaxed) { completed=false; reason="confirmed_finish_or_stop"; break; }
        if lag_stop { shared.counts.lag_stops.fetch_add(1,Ordering::Relaxed); reason="lag_guard"; break; }
        if !continuation(o) { reason=if f.port_abort {"port_abort"} else if !o.alive_after || !o.chest_alive_after {"dead_or_removed_chest"}
            else if !in_world(o.chest_after) {"outside_world"} else {"ceiling"}; break; }
    }
    shared.emit("trials",json!({"schema":1,"parent":parent,"initial_base":initial,"planned_entries":masks.len(),
        "earliest_changed_index":if initial {None} else {Some(changed)},"credit_after_iteration":ANCHOR.max(changed),
        "confirmation_compatible_horizon":horizon,"actual_world_frames":s.frame(),"consumed_iteration":s.replay.iteration,"best_window":best.map(Score::value),
        "evaluation_ended":completed,"ended_reason":reason,"consecutive_lag_frames":lag.consecutive(),"no_finish_claim":false}))?;
    Ok(best)
}

fn admit(cfg: &Config, shared: &Shared, parent: &Member, masks: Vec<u8>, score: Score, changed: usize) -> Result<(), String> {
    let record_masks=masks.clone();
    let admission=shared.pool.lock().map_err(|_| "pair pool mutex poisoned")?.admit(parent,masks,score,cfg.pool_size);
    match admission {
        Admission::Duplicate => {shared.counts.duplicates.fetch_add(1,Ordering::Relaxed);},
        Admission::BelowFloor => {},
        Admission::Added {id,evicted,equal_count} => {
            shared.counts.admissions.fetch_add(1,Ordering::Relaxed);
            if equal_count {shared.counts.equal_admissions.fetch_add(1,Ordering::Relaxed);}
            let improved={let mut b=shared.best.lock().unwrap(); let improved=b.map_or(true,|old| score.value>old.value);
                if improved {*b=Some(score);} improved};
            shared.emit("improvements",json!({"schema":1,"id":id,"ancestor":parent.id,"root":parent.root,
                "generation":parent.generation.saturating_add(1),"new_to_current_pool":true,"equal_count_admission":equal_count,
                "best_improvement":improved,"evicted":evicted,"earliest_changed_index":changed,"anchor":ANCHOR,
                "planned_entries":record_masks.len(),"best_window":score.value(),"tas":tas_text(&record_masks),
                "qualification":"Native occupancy only; no global raw novelty, distinct physics state, pairmax or Flash match claim"}))?;
        }
    }
    Ok(())
}
fn worker(w: usize,cfg: &Config,shared: &Shared) -> Result<(),String> {
    let mut rng=Rng(cfg.seed ^ (w as u64+1).wrapping_mul(0x5851f42d4c957f2d));
    while !shared.should_stop(cfg) {
        let parent=shared.pool.lock().map_err(|_| "pair pool mutex poisoned")?.pick(&mut rng);
        let Some((masks,changed))=proposal(&parent.masks,&mut rng,cfg.window,cfg.extra)? else {
            shared.counts.noops.fetch_add(1,Ordering::Relaxed); continue;
        };
        shared.counts.trials.fetch_add(1,Ordering::Relaxed);
        if let Some(score)=replay(cfg,shared,&masks,changed,parent.id,false)? {admit(cfg,shared,&parent,masks,score,changed)?;}
    }
    Ok(())
}

pub fn run(cfg: Config) -> Result<Vec<u8>,String> {
    cfg.validate()?;
    super::sim::set_lag_ms(cfg.lag_ms); super::sim::set_lag_frames(cfg.lag_frames);
    let start=Instant::now(); let budget=Duration::try_from_secs_f64(cfg.hours*3600.0).map_err(|e| e.to_string())?;
    let deadline=start.checked_add(budget).ok_or("pair deadline exceeds platform clock")?;
    // create_dir (not create_dir_all) refuses every existing attempt.
    std::fs::create_dir(&cfg.run_dir).map_err(|e| format!("fresh pair run {}: {e}",cfg.run_dir.display()))?;
    let shared=Shared {pool:Mutex::new(Pool::new(&cfg.bases)),stop:AtomicBool::new(false),error:Mutex::new(None),winner:Mutex::new(None),
        logs:Mutex::new(Logs::new(&cfg.run_dir)?),counts:Counts::default(),sites:Mutex::new([0;12]),best:Mutex::new(None),
        maximum_current_pairs:Mutex::new(0),start,deadline};
    let mut config=fresh_file(&cfg.run_dir.join("config.json"))?;
    write_line(&mut config,&json!({"schema":1,"mode":"pair","level":45,"character":3,"anchor":ANCHOR,"pair_window":WINDOW,
        "max_frame":MAX_FRAME,"threads":cfg.threads,"hours":cfg.hours,"seed":cfg.seed,"window":cfg.window,"extra":cfg.extra,
        "lag_ms":cfg.lag_ms,"lag_cutoff_us":(cfg.lag_ms*1000.0) as u32,"lag_frames":cfg.lag_frames,"laggy_frames_receive_occupancy_credit":false,
        "ordinary_pool_capacity":cfg.pool_size,"initial_raw_bases":cfg.bases.len(),"initial_unique_bases":shared.pool.lock().unwrap().members.len(),
        "initialization_only":cfg.hours==0.0,"fresh_replay":true,"finite_planned_tas_semantics":true,"live_plan_min_entries":239,
        "qualification":"Native occupancy observations; native exact confirmation and original-player verification remain separate"}))?;
    // Preserve every input ordinal including duplicate full bases, separately from
    // the canonical protected pool membership.
    let mut initial=fresh_file(&cfg.run_dir.join("initial-bases.jsonl"))?;
    for (i,masks) in cfg.bases.iter().enumerate() {write_line(&mut initial,&json!({"ordinal":i,"frame":masks.len(),"tas":tas_text(masks)}))?;}
    export_pool(&shared,&cfg.run_dir.join("pool-initial.jsonl"))?;
    let mut stats_log=fresh_file(&cfg.run_dir.join("stats.jsonl"))?;
    write_line(&mut stats_log,&stats(&shared))?;
    if cfg.hours>0.0 {
        let bases=shared.pool.lock().unwrap().members.clone();
        for m in &bases {
            if shared.should_stop(&cfg) {break;}
            shared.counts.initial_replays.fetch_add(1,Ordering::Relaxed);
            match replay(&cfg,&shared,&m.masks,ANCHOR,m.id,true) {
                Ok(score) => {if let Some(actual)=shared.pool.lock().unwrap().members.iter_mut().find(|p| p.id==m.id) {actual.score=score;}
                    if let Some(score)=score {let mut b=shared.best.lock().unwrap(); if b.map_or(true,|old| score.value>old.value) {*b=Some(score);}}},
                Err(e) => {shared.fail(e);break;}
            }
        }
        std::thread::scope(|scope| {
            let mut handles=Vec::new();
            for w in 0..cfg.threads {
                if shared.should_stop(&cfg) {break;}
                let (cfg,shared)=(&cfg,&shared);
                match std::thread::Builder::new().name(format!("pair-{w}")).stack_size(16<<20)
                    .spawn_scoped(scope,move || {
                        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| worker(w,cfg,shared))) {
                            Ok(Ok(()))=>{}, Ok(Err(e))=>shared.fail(e), Err(_)=>shared.fail("pair worker panicked".into()),
                        }
                    }) {
                    Ok(h)=>handles.push(h),Err(e)=>{shared.fail(format!("pair worker spawn: {e}"));break;}
                }
            }
            let mut next_stats=Instant::now(); let mut next_checkpoint=Instant::now()+Duration::from_secs(60); let mut sequence=0u64;
            while !shared.should_stop(&cfg) {
                if Instant::now()>=next_stats {if let Err(e)=write_line(&mut stats_log,&stats(&shared)) {shared.fail(e);}
                    next_stats=Instant::now()+Duration::from_secs(2);}
                if Instant::now()>=next_checkpoint {let path=cfg.run_dir.join(format!("pool-checkpoint-{sequence:06}.jsonl")); sequence+=1;
                    if let Err(e)=export_pool(&shared,&path) {shared.fail(e);} next_checkpoint=Instant::now()+Duration::from_secs(60);}
                std::thread::sleep(Duration::from_millis(250));
            }
            shared.stop.store(true,Ordering::Relaxed);
            for h in handles {if h.join().is_err() {shared.fail("pair worker panicked".into());}}
        });
    }
    shared.stop.store(true,Ordering::Relaxed);
    // Only after all workers exited: immutable final complete pool and stats.
    export_pool(&shared,&cfg.run_dir.join("pool.jsonl"))?;
    let mut final_stats=stats(&shared); final_stats["ended"]=true.into();
    final_stats["initialization_only"]=(cfg.hours==0.0).into();
    final_stats["stop_file_present"]=cfg.run_dir.join("STOP").exists().into();
    final_stats["error"]=json!(shared.error.lock().unwrap().clone());
    write_line(&mut stats_log,&final_stats)?; stats_log.sync_all().map_err(|e| e.to_string())?;
    if let Some(e)=shared.error.lock().unwrap().clone() {return Err(e);}
    let winner=shared.winner.lock().unwrap().clone();
    winner.ok_or_else(|| "pair ended without a confirmed winner".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn config() -> Config {Config {level:45,character:3,bases:vec![vec![17;139]],run_dir:PathBuf::from("unused"),
        threads:2,hours:1.0,seed:1,window:239,extra:24,pool_size:2,lag_ms:15.0,lag_frames:3}}
    fn score(value:i32) -> Score {Score {value,frame:90,iteration:90,counts:[value;WINDOW]}}
    #[test] fn config_zero_workers_only_for_zero_hours_and_fixed_profile() {
        let mut c=config(); assert!(c.validate().is_ok()); c.threads=0; assert!(c.validate().is_err());
        c.hours=0.0; assert!(c.validate().is_ok()); c.hours=f64::NAN; assert!(c.validate().is_err());
        c=config(); c.character=2; assert!(c.validate().is_err()); c=config(); c.threads=19; assert!(c.validate().is_err());
        c=config(); c.lag_frames=0; assert!(c.validate().is_err()); c=config(); c.lag_ms=f64::NAN; assert!(c.validate().is_err());
        c=config(); c.lag_ms=-1.0; assert!(c.validate().is_err()); c=config(); c.lag_ms=0.0; assert!(c.validate().is_ok());
    }
    #[test] fn full_initial_bases_and_duplicate_ordinals_are_preserved() {
        let a=vec![17;139];let b=vec![49;289];let p=Pool::new(&[a.clone(),b.clone(),a.clone()]);
        assert_eq!(p.members.len(),2);assert_eq!(p.members[0].masks,a);assert_eq!(p.members[1].masks,b);
        assert_eq!(p.members[0].source_ordinals,[0,2]);assert!(p.members.iter().all(|m| m.protected&&m.score.is_none()));
    }
    #[test] fn thousands_of_suffix_edits_preserve_prefix_and_floor() {
        for len in [62,66,139,289,299] {for seed in 0..512 {
            let base:Vec<u8>=(0..len).map(|i| (i*37+17) as u8).collect();
            if let Some((m,changed))=proposal(&base,&mut Rng(seed),239,24).unwrap() {
                assert_eq!(&m[..ANCHOR],&base[..ANCHOR]);assert!((62..=299).contains(&m.len()));
                assert_eq!(first_change(&base,&m),Some(changed));assert!(changed>=ANCHOR&&changed<m.len());
            }
        }}
    }
    #[test] fn pure_shortening_is_unscorable_and_not_an_error() {
        let a=vec![17;139];let b=&a[..100];assert_eq!(first_change(&a,b),Some(100));
        assert_eq!(first_change(&a,b).filter(|&i| i<b.len()),None);
    }
    #[test] fn equal_count_diversity_gets_a_place_without_evicting_protected() {
        let a=vec![17;139];let mut p=Pool::new(&[a.clone()]);let parent=p.members[0].clone();
        let mut b=a.clone();b[90]=49;let mut c=a.clone();c[90]=80;
        assert!(matches!(p.admit(&parent,b.clone(),score(100),1),Admission::Added{evicted:None,..}));
        assert_eq!(p.admit(&parent,b,score(100),1),Admission::Duplicate);
        assert!(matches!(p.admit(&parent,c.clone(),score(100),1),Admission::Added{evicted:Some(1),equal_count:true,..}));
        assert_eq!(p.members.len(),2);assert_eq!(p.members[0].masks,a);assert!(p.members[0].protected);
        assert_eq!(p.members[1].masks,c);let mut d=c;d[91]^=16;
        assert_eq!(p.admit(&parent,d,score(99),1),Admission::BelowFloor);
    }
    #[test] fn full_tas_serialization_roundtrips_member_not_best_prefix() {
        let p=Pool::new(&[vec![17;289]]);let v=p.members[0].value();
        assert_eq!(super::super::sim::parse_tas(v["tas"].as_str().unwrap()).unwrap(),p.members[0].masks);
        assert_eq!(v["frame"],289);assert!(v["best_window"].is_null());
    }
}

/// Native integration oracle from the saved pre-search finite diagnostic traces.
/// Kept separate from pure unit tests: Root runs it explicitly and in isolation.
#[cfg(test)]
mod replay_regression {
    use super::*;

    // Exact original raw/file SHA256: ee7a4ccf9ae7b070b31931ac84ec84ccbbfb5715f0f4b7a24910037189d264b7
    const PERSIST_F262: &str = concat!(
        "001000000010000000100000001000000010000000100000001000000010000000100000001000000010000000100000",
        "001000000010000000100000001000000010000000100000001000000010000000100000001000000010000000100000",
        "001000000010000000100000001000000010000000100000100010001000100010001000100010001000100010001000",
        "100010001000100010001000100010001000100010001000100010001000100010001000100010001000100010001000",
        "100010001000100010001000100010001000100010001000100010001000100010001000100010001000100010001000",
        "100010001000100010001000100000001000100010000000100010001010001010110010101100101011001010110010",
        "101100101011001000010111111100011111000111110001111110011001111110110111000000000000100000001000",
        "000010000000100000001000000010000000100000001000000010000000100000001000000010000000100000001000",
        "000010000000100000001000000010000000101000001010000010100000101000001010000010100000101000001010",
        "000010100000101000001010000010100000101000001010000010000000100000001000000010000000100000001000",
        "000010000000100000001000000010000000100000001000000010000000100000001000000010000000100000001000",
        "000010000000100000001000000010000000100011001101000010000000100000001000000010000000100000001000",
        "000010000000100000001000000010000000111000001110000011100000111000001110000011100000111000001110",
        "000011100000111000001110010110110101101101011011010110110101101101011011010110110101101101011011",
        "010110110101101101011011010110110101101101011011010110110101101101011011010110110101101101011011",
        "010110110101101101011011010110110101101101011011010110110101101101011011010110110101101101011011",
        "010110110101101101011011010110110101101101011011010110110101101101011011010110110101101101011011",
        "010110110101101101011011010110110101101101011011010110110101101101011011010110110101101101011011",
        "010110110101101101011011010110110101101101011011010110110101101101011011010110110101101101011011",
        "010110110101101101011011010110110101101101011011010110110101101101011011010110110101101101011011",
        "010110110101101101011011010110110101101101011011010110110101101101011011010110110101101101011011",
        "01011011010110110101101101011011010110110101101101011011010110110101101101011011",
    );

    // Exact original raw/file SHA256: 8802ae34b889ea5e59c57c8d150cb2e6119f4c93a5a99a5a87b91081962fdb1f
    const LAUNCH_F286: &str = concat!(
        "001000000010000000100000001000000010000000100000001000000010000000100000001000000010000000100000",
        "001000000010000000100000001000000010000000100000001000000010000000100000001000000010000000100000",
        "001000000010000000100000001000000010000000100000100010001000100010001000100010001000100010001000",
        "100010001000100010001000100010001000100010001000100010001000100010001000100010001000100010001000",
        "100010001000100010001000100010001000100010001000100010001000100010001000100010001000100010001000",
        "100010001000100010001000100000001000100010000000100010001010001010110010101100101011001010110010",
        "101100101011001000010111111100011111000111110001111110011001111110110111000000000000100000001000",
        "000010000000100000001000000010000000100000001000000010000000100000001000000010000000100000001000",
        "000010000000100000001000000010000000100000001000000010000000100000001000000010000000100000001000",
        "000010000000100000001000000010000000100000001000000010000000100000001000000010000000100000001000",
        "000010000000100000001000000010000000100000001000000010000000100000001000000010000000100000001000",
        "000010000000100000001000000010000000100011001101000010000000100000001001000010010000100000001000",
        "000010000000100000001000000010000000111000001110000011100000111000001110000011100000111000001110",
        "000011100000111000001110010110110101101101011011010110110101101101011011010110110101101101011011",
        "010110110101101101011011010110110101101101011011010110110101101101011011010110110101101101011001",
        "010010110100101101001011010010110100101101001011010010110100101101001011010010110100101101001011",
        "010010110100101101001011010010110100101101001011010010110100101101001011010010110100101101001011",
        "010010110100101101001011010010110100101101001011010010110100101101001011010010110100101101001011",
        "010010110100101101001011010010110100101101001011010010110100101101001011010010110100101101001011",
        "010010110100101101001011010010110100101101001011010010110100101101001011010010110100101101001011",
        "010010110100101101001011010010110100101101001011010010110100101101001011010010110100101101001011",
        "010010110100101100001100000011000000110000001100000011000000110000001110001011000000110000001100",
        "000011000000110000001100000011000000110000001100000011000000110000001100000011000000110000001100",
        "00001100000011000100110001001100010011000100110001001100011010000110111001101110",
    );

    struct TempRun(PathBuf);
    impl TempRun {
        fn new(label: &str) -> Self {
            let nonce=std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos();
            for attempt in 0..8 {
                let path=std::env::temp_dir().join(format!("hw-pair-regression-{label}-{}-{nonce}-{attempt}",std::process::id()));
                match std::fs::create_dir(&path) {
                    Ok(())=>return Self(path),
                    Err(e) if e.kind()==std::io::ErrorKind::AlreadyExists=>{},
                    Err(e)=>panic!("fresh regression directory: {e}"),
                }
            }
            panic!("could not allocate a fresh regression directory");
        }
    }
    impl Drop for TempRun {
        fn drop(&mut self) {
            // Only our six known files in our create_dir-owned directory, never a
            // recursive deletion or an external artifact fixture directory.
            for name in ["improvements.jsonl","launches.jsonl","pairmax-events.jsonl",
                         "finishes.jsonl","trials.jsonl","pool-regression.jsonl"] {
                let _=std::fs::remove_file(self.0.join(name));
            }
            let _=std::fs::remove_dir(&self.0);
        }
    }
    fn rows(path: &Path) -> Vec<Value> {
        let text=std::fs::read_to_string(path).unwrap();
        assert!(text.is_empty()||text.ends_with('\n'),"complete LF records required");
        text.lines().map(|s|serde_json::from_str(s).unwrap()).collect()
    }
    fn check_production_replay(bits: &str, label: &str, frames: u32, iteration: u32, reason: &str) {
        let owned=TempRun::new(label);
        let masks=super::super::sim::parse_tas(bits).unwrap(); let original=masks.clone();
        let cfg=Config {level:45,character:3,bases:vec![masks.clone()],run_dir:owned.0.clone(),
            threads:1,hours:1.0,seed:1,window:239,extra:24,pool_size:2,lag_ms:0.0,lag_frames:3};
        cfg.validate().unwrap();
        let start=Instant::now();
        let shared=Shared {pool:Mutex::new(Pool::new(&cfg.bases)),stop:AtomicBool::new(false),error:Mutex::new(None),
            winner:Mutex::new(None),logs:Mutex::new(Logs::new(&cfg.run_dir).unwrap()),counts:Counts::default(),
            sites:Mutex::new([0;12]),best:Mutex::new(None),maximum_current_pairs:Mutex::new(0),start,
            deadline:start+Duration::from_secs(120)};
        // This calls the production driver, not a copy of its gating loop.
        let score=replay(&cfg,&shared,&masks,ANCHOR,0,true).unwrap();
        assert_eq!(masks,original); assert_eq!(tas_text(&masks),bits);
        assert!(shared.winner.lock().unwrap().is_none());
        assert_eq!(shared.counts.frames.load(Ordering::Relaxed),frames as u64);
        assert_eq!(shared.counts.observed_finishes.load(Ordering::Relaxed),0);
        assert_eq!(shared.counts.confirmed_finishes.load(Ordering::Relaxed),0);
        assert_eq!(shared.counts.site2.load(Ordering::Relaxed),0);
        assert_eq!(shared.counts.port_aborts.load(Ordering::Relaxed),0);
        assert_eq!(shared.counts.lag_stops.load(Ordering::Relaxed),0);
        if label=="surviving262" {
            // The native fixture survives its complete literal and zero tail. Its
            // EOF dispatch consumes one empty slot; later world frames consume none.
            let measured=score.expect("the surviving fixture has a real clean post-anchor window");
            assert!(measured.frame<=262&&measured.iteration<=262,"EOF slots cannot earn pair fitness");
        }
        export_pool(&shared,&cfg.run_dir.join("pool-regression.jsonl")).unwrap();
        drop(shared); // close all production output handles before examining them
        let trials=rows(&cfg.run_dir.join("trials.jsonl")); assert_eq!(trials.len(),1);
        let trial=&trials[0];
        assert_eq!(trial["actual_world_frames"],frames); assert_eq!(trial["consumed_iteration"],iteration);
        assert_eq!(trial["planned_entries"],masks.len()); assert_eq!(trial["ended_reason"],reason);
        assert_eq!(trial["confirmation_compatible_horizon"],299);
        assert_eq!(trial["evaluation_ended"],true); assert_eq!(trial["no_finish_claim"],false);
        let pool=rows(&cfg.run_dir.join("pool-regression.jsonl")); assert_eq!(pool.len(),1);
        assert_eq!(pool[0]["frame"],masks.len()); assert_eq!(pool[0]["tas"],bits);
        assert_eq!(pool[0]["protected"],true); assert_eq!(pool[0]["source_ordinals"],json!([0]));
        assert!(pool[0]["best_window"].is_null(),"direct replay does not rewrite the protected base");
        assert!(rows(&cfg.run_dir.join("finishes.jsonl")).is_empty());
        assert!(rows(&cfg.run_dir.join("pairmax-events.jsonl")).is_empty());
        assert!(!cfg.run_dir.join("winner-1.tas").exists());
        if label=="fatal286" {
            let launches=rows(&cfg.run_dir.join("launches.jsonl"));
            let launch=launches.iter().find(|r|r["frame"].as_u64()==Some(286)).expect("saved fatal chest launch must be logged");
            assert_eq!(launch["iteration"],286); assert_eq!(launch["site"],0);
            assert_eq!(launch["alive_before"],true); assert_eq!(launch["alive_after"],false);
            assert_eq!(launch["finished"],false); assert_eq!(launch["tas"],bits);
            let moved=launch["moved"].as_f64().unwrap();
            // Exact measured oracle from anchored-qe-03 launch row66; no tolerance.
            assert_eq!(moved.to_bits(),0x404a67680e99ce66);
        }
    }
    #[test]
    #[ignore = "native finite replay oracle; run explicitly with --ignored --test-threads=1"]
    fn native_pair_replay_preserves_finite_literals_and_fatal_launch_scope() {
        // Sim lag timing is global, so this test must not run concurrently with
        // other native replay tests. Pure policy tests remain independent.
        std::thread::Builder::new().name("pair-native-regression".into()).stack_size(64<<20)
            .spawn(|| {
                super::super::sim::set_lag_ms(0.0);
                let _hang=super::super::sim::HangOverride::new(0);
                check_production_replay(PERSIST_F262,"surviving262",299,263,"ceiling");
                check_production_replay(LAUNCH_F286,"fatal286",286,286,"dead_or_removed_chest");
            }).unwrap().join().expect("native pair regression thread");
    }
}
