//! Dedicated Dad/Obstacle finish-focus progress; policy is separate from physics.
//! Initial full bases are protected; all replays start fresh. Native confirmation
//! remains separate from original-player verification. No physics state is repaired.
#[path = "pair_policy.rs"]
mod base_policy;
#[path = "progress_policy.rs"]
mod policy;
#[path = "progress_eject_filter.rs"]
mod eject_filter;
#[path = "progress_route_policy.rs"]
mod route_policy;
#[path = "progress_launch_policy.rs"]
mod launch_policy;
#[path = "progress_survival_policy.rs"]
mod survival_policy;
#[path = "progress_diversity_policy.rs"]
mod diversity_policy;
#[path = "progress_family_policy.rs"]
mod family_policy;
#[path = "progress_checkpoint_policy.rs"]
mod checkpoint_policy;
use eject_filter::{CertificateBuilder, Invocation, NeutralEjectCertificate, PreInputState};
use base_policy::LagGuard;
use policy::{Observation, ProgressWindow, WindowScore, ANCHOR, MAX_FRAME, WINDOW};
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Objective { Finish, Fan, Launch }
impl Objective {
    pub fn parse(name: &str) -> Result<Self, String> {
        match name { "finish" => Ok(Self::Finish), "fan" => Ok(Self::Fan), "launch" => Ok(Self::Launch),
            _ => Err("progress objective must be finish, fan or launch".into()) }
    }
    fn name(self) -> &'static str { match self { Self::Finish => "finish", Self::Fan => "fan", Self::Launch => "launch" } }
    fn version(self) -> &'static str { match self {
        Self::Finish => "dad-finish-focus-progress-v1", Self::Fan => "dad-fan-route-progress-v1",
        Self::Launch => "dad-route-launch-survival-v2" } }
}

pub struct Config {
    pub level: u32, pub character: u32, pub bases: Vec<Vec<u8>>, pub run_dir: PathBuf,
    pub threads: usize, pub hours: f64, pub seed: u64, pub window: usize,
    pub extra: usize, pub pool_size: usize, pub lag_ms: f64, pub lag_frames: u32,
    pub objective: Objective, pub diversity_pool: usize, pub route_checkpoints: bool,
}
impl Config {
    fn reserve_version(&self) -> &'static str {
        if self.route_checkpoints {"dad-survivor-route-checkpoints-v1"} else {"dad-survivor-route-cells-v1"}
    }
    pub fn validate(&self) -> Result<(), String> {
        if (self.level, self.character) != (45, 3) { return Err("progress requires level45/character3".into()); }
        if !self.hours.is_finite() || self.hours < 0.0 || self.threads > 18
            || (self.threads == 0 && self.hours != 0.0) { return Err("progress requires finite nonnegative hours; live workers1..18; workers0 only with hours0".into()); }
        if Duration::try_from_secs_f64(self.hours * 3600.0).is_err() { return Err("pair hours exceed the representable budget".into()); }
        if !self.lag_ms.is_finite() || self.lag_ms < 0.0 || self.lag_ms * 1000.0 > u32::MAX as f64 || self.lag_frames == 0 {
            return Err("progress requires finite lag-ms0..4294967.295 and lag-frames>=1 (lag-ms0 disables lag detection)".into());
        }
        if self.pool_size == 0 || !(1..=MAX_FRAME as usize - ANCHOR).contains(&self.window)
            || self.extra > MAX_FRAME as usize { return Err("progress requires pool>0, window1..239, extra0..299".into()); }
        if self.diversity_pool > 1024 || (self.diversity_pool > 0 && self.objective != Objective::Launch) {
            return Err("diversity-pool requires Launch objective and capacity0..1024".into());
        }
        if self.route_checkpoints && checkpoint_policy::per_checkpoint_capacity(self.diversity_pool).is_none() {
            return Err("route-checkpoints requires Launch diversity-pool with even capacity2..1024".into());
        }
        if self.bases.is_empty() || self.bases.iter().any(|b| !(ANCHOR + 2..=MAX_FRAME as usize).contains(&b.len())) {
            return Err("progress requires at least one full62..299-entry base".into());
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
struct Score { value: f64, frame: u32, iteration: u32, window: WindowScore,
    objective: Objective, route: Option<route_policy::RouteScore>, launch: Option<launch_policy::LaunchScore> }
impl Score {
    fn from_window(objective: Objective, window: WindowScore, frame: u32, iteration: u32) -> Option<Self> {
        if objective!=Objective::Finish && window.focus_handle!=window.chest_handle {return None;}
        let route = if objective==Objective::Fan {Some(route_policy::score(window.focus_positions)?)} else {None};
        let launch = if objective==Objective::Launch {Some(launch_policy::score(window.focus_positions,window.chest_velocities,window.world_frames)?)} else {None};
        let value = match objective {Objective::Finish=>window.worst_distance_m,
            Objective::Fan=>route?.ranking_value,Objective::Launch=>launch?.ranking_value};
        value.is_finite().then_some(Self { value, frame, iteration, window, objective, route, launch })
    }
    fn value(self) -> Value { json!({"worst_finish_focus_distance_m":self.window.worst_distance_m,
        "distance_f64_bits":format!("{:016x}",self.window.worst_distance_m.to_bits()),"smaller_is_better":true,
        "objective":self.objective.name(),"ranking_value":self.value,
        "ranking_f64_bits":format!("{:016x}",self.value.to_bits()),"route":self.route.map(|r| r.value()),
        "launch_pace":self.launch.map(|s| s.value()),
        "frame":self.frame,"iteration":self.iteration,"window_frames":WINDOW,
        "window_distances_m":self.window.distances,"window_pairs_telemetry":self.window.pairs,
        "window_qe_telemetry":self.window.qe,"window_qe_raw_telemetry":self.window.qe_raw,
        "actual_level_focus_handle":self.window.focus_handle,"primary_chest_handle":self.window.chest_handle,
        "actual_level_focus_position":self.window.focus_position,"primary_chest_position":self.window.chest_position,
        "window_focus_positions":self.window.focus_positions,
        "window_primary_chest_velocities":self.window.chest_velocities,"window_world_frames":self.window.world_frames,
        "finish_claim":false}) }
}
#[derive(Clone)]
struct Member {
    id: u64, ancestor: Option<u64>, root: u64, generation: u64, source_ordinals: Vec<usize>,
    protected: bool, masks: Vec<u8>, score: Option<Score>, visits: u64,
    objective: Objective, route_checkpoint: Option<u32>,
    // Built only by this exact literal's fresh replay; never inherited/imported.
    neutral_eject_certificate: Option<NeutralEjectCertificate>,
}
impl Member {
    fn value(&self) -> Value { json!({"schema":1,"score_version":self.objective.version(),
        "id":self.id,"ancestor":self.ancestor,"root":self.root,"generation":self.generation,
        "protected":self.protected,"source_ordinals":self.source_ordinals,"visits":self.visits,
        "frame":self.masks.len(),"tas":tas_text(&self.masks),"best_window":self.score.map(Score::value),
        "route_checkpoint":self.route_checkpoint}) }
}
#[derive(Clone)]
struct RouteMember { member: Member, key: diversity_policy::CellKey, route_window: Score }
impl RouteMember {
    fn value(&self) -> Value {
        let mut value=self.member.value();
        value["pool_lane"]=json!("survivor_route");
        value["reserve_version"]=json!(if self.member.route_checkpoint.is_some() {
            "dad-survivor-route-checkpoints-v1"} else {"dad-survivor-route-cells-v1"});
        value["route_cell"]=json!({"x":self.key.x,"y":self.key.y,"band":self.key.band});
        value["route_window"]=self.route_window.value();value
    }
}
struct Pool { members: Vec<Member>, route_members: Vec<RouteMember>, next_id: u64, route_checkpoints: bool }
#[derive(Debug, PartialEq, Eq)]
enum Admission { Duplicate, BelowFloor, Added { id: u64, evicted: Option<u64>, equal_count: bool } }
impl Pool {
    fn new(bases: &[Vec<u8>], objective: Objective) -> Self {
        let mut members: Vec<Member> = Vec::new();
        for (ordinal, masks) in bases.iter().enumerate() {
            if let Some(m) = members.iter_mut().find(|m| m.masks == *masks) { m.source_ordinals.push(ordinal); }
            else { let id = members.len() as u64; members.push(Member { id, ancestor: None, root:id,
                generation:0, source_ordinals:vec![ordinal], protected:true, masks:masks.clone(), score:None, visits:0,
                objective, route_checkpoint:None, neutral_eject_certificate:None }); }
        }
        Self { next_id:members.len() as u64, members, route_members:Vec::new(), route_checkpoints:false }
    }
    fn with_checkpoints(bases:&[Vec<u8>],objective:Objective,route_checkpoints:bool) -> Self {
        let mut pool=Self::new(bases,objective);pool.route_checkpoints=route_checkpoints;pool
    }
    fn pick(&mut self, rng: &mut Rng) -> Member {
        let n = self.members.len();
        // 25% uniform, otherwise nearest of two. Equal distances favour less visited;
        // no score-only dedup and unscored protected bases remain reachable.
        let a = rng.below(n); let b = if rng.below(4) == 0 { a } else { rng.below(n) };
        let distance = |m: &Member| m.score.map(|s| s.value).unwrap_or(f64::INFINITY);
        let order=distance(&self.members[b]).total_cmp(&distance(&self.members[a]));
        let i=if order.is_lt() || (order.is_eq() && self.members[b].visits<self.members[a].visits) {b} else {a};
        self.members[i].visits = self.members[i].visits.saturating_add(1);
        self.members[i].clone()
    }
    fn pick_route(&mut self, rng: &mut Rng) -> (Member, bool) {
        // 25% retain access to the protected/pace pool. The opt-in checkpoint
        // mode chooses a source family first; the legacy path keeps its RNG draws.
        if !self.route_members.is_empty() && rng.below(4)!=0 {
            let i=if self.route_checkpoints {
                let roots:Vec<_>=self.route_members.iter().map(|m|m.member.root).collect();
                let Some(i)=family_policy::pick(&roots,|n|rng.below(n)) else {return (self.pick(rng),false);};i
            } else {rng.below(self.route_members.len())};
            self.route_members[i].member.visits=self.route_members[i].member.visits.saturating_add(1);
            (self.route_members[i].member.clone(),true)
        } else {(self.pick(rng),false)}
    }
    fn admit_route(&mut self, parent:&Member, masks:Vec<u8>, pace:Option<Score>, route:Score,
        certificate:NeutralEjectCertificate, cap:usize) -> Admission {
        self.admit_route_at(parent,masks,pace,route,certificate,cap,None)
    }
    fn admit_route_at(&mut self, parent:&Member, masks:Vec<u8>, pace:Option<Score>, route:Score,
        certificate:NeutralEjectCertificate, cap:usize, checkpoint:Option<u32>) -> Admission {
        if let Some(frame)=checkpoint {
            if checkpoint_policy::index(route.frame,route.window.world_frames)
                .map(|i|checkpoint_policy::FRAMES[i])!=Some(frame) {return Admission::BelowFloor;}
        }
        if self.route_members.iter().any(|m|m.member.route_checkpoint==checkpoint&&m.member.masks==masks) {
            return Admission::Duplicate;
        }
        let Some(key)=diversity_policy::key(route.window.focus_position,route.frame) else {return Admission::BelowFloor;};
        let indices:Vec<_>=self.route_members.iter().enumerate()
            .filter_map(|(i,m)|(m.member.route_checkpoint==checkpoint).then_some(i)).collect();
        let residents:Vec<_>=indices.iter().map(|&i|&self.route_members[i]).map(|m|diversity_policy::Resident {
            key:m.key,route_cost:m.route_window.value,visits:m.member.visits,id:m.member.id}).collect();
        let equal_count=indices.iter().any(|&i|self.route_members[i].key==key&&self.route_members[i].route_window.value==route.value);
        let choice=diversity_policy::admission(&residents,key,route.value,cap);
        let evicted=match choice {
            diversity_policy::Admission::BelowCellFloor=>return Admission::BelowFloor,
            diversity_policy::Admission::Append=>None,
            diversity_policy::Admission::Replace(i)=>{
                let Some(&global_index)=indices.get(i) else {return Admission::BelowFloor;};
                Some(self.route_members.remove(global_index).member.id)
            },
        };
        let id=self.next_id;self.next_id+=1;
        self.route_members.push(RouteMember {member:Member {id,ancestor:Some(parent.id),root:parent.root,
            generation:parent.generation.saturating_add(1),source_ordinals:vec![],protected:false,
            masks,score:pace,visits:0,objective:parent.objective,route_checkpoint:checkpoint,
            neutral_eject_certificate:Some(certificate)},
            key,route_window:route});
        Admission::Added {id,evicted,equal_count}
    }
    fn admit(&mut self, parent: &Member, masks: Vec<u8>, score: Score,
        neutral_eject_certificate: NeutralEjectCertificate, cap: usize) -> Admission {
        if self.members.iter().any(|m| m.masks == masks) { return Admission::Duplicate; }
        let equal_count = self.members.iter().any(|m| m.score.is_some_and(|s| s.value == score.value));
        let ordinary: Vec<usize> = self.members.iter().enumerate().filter_map(|(i,m)| (!m.protected).then_some(i)).collect();
        let mut evicted = None;
        if ordinary.len() >= cap {
            let floor = ordinary.iter().map(|&i| self.members[i].score.unwrap().value).max_by(f64::total_cmp).unwrap();
            if score.value > floor { return Admission::BelowFloor; }
            // A new equal-distance TAS gets a real place: most tried, then oldest,
            // among the farthest ordinary members. Protected bases never evict.
            let i = *ordinary.iter().filter(|&&i| self.members[i].score.unwrap().value == floor)
                .max_by_key(|&&i| (self.members[i].visits, u64::MAX - self.members[i].id)).unwrap();
            evicted = Some(self.members.remove(i).id);
        }
        let id = self.next_id; self.next_id += 1;
        self.members.push(Member { id, ancestor:Some(parent.id), root:parent.root,
            generation:parent.generation.saturating_add(1), source_ordinals:vec![], protected:false,
            masks, score:Some(score), visits:0, objective:parent.objective, route_checkpoint:None,
            neutral_eject_certificate:Some(neutral_eject_certificate) });
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
    if !base_policy::valid_anchored_child(parent, &masks) { return Err("progress mutation violated immutable prefix/length contract".into()); }
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
    incomplete_host_diagnostics: AtomicU64,
    neutral_only_proposals_rejected: AtomicU64,
    replay_ids: AtomicU64, route_admissions: AtomicU64, route_duplicates: AtomicU64,
    route_below_cell_floor: AtomicU64, pace_parent_picks: AtomicU64, route_parent_picks: AtomicU64,
}
struct Logs { improvements: File, launches: File, pairmax: File, finishes: File, trials: File, route:File }
impl Logs {
    fn new(dir: &Path) -> Result<Self, String> { Ok(Self {
        improvements:fresh_file(&dir.join("improvements.jsonl"))?, launches:fresh_file(&dir.join("launches.jsonl"))?,
        pairmax:fresh_file(&dir.join("pairmax-events.jsonl"))?, finishes:fresh_file(&dir.join("finishes.jsonl"))?,
        trials:fresh_file(&dir.join("trials.jsonl"))?,route:fresh_file(&dir.join("route-admissions.jsonl"))? }) }
}
struct Shared {
    pool: Mutex<Pool>, stop: AtomicBool, error: Mutex<Option<String>>, winner: Mutex<Option<Vec<u8>>>,
    logs: Mutex<Logs>, counts: Counts, sites: Mutex<[u64;12]>, best: Mutex<Option<Score>>,
    maximum_current_pairs: Mutex<i32>, start: Instant, deadline: Instant,
    objective: Objective,
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
        let mut logs = self.logs.lock().map_err(|_| "progress log mutex poisoned")?;
        let out = match which { "improvements" => &mut logs.improvements, "launches" => &mut logs.launches,
            "pairmax" => &mut logs.pairmax, "finishes" => &mut logs.finishes,
            "route"=>&mut logs.route, _ => &mut logs.trials };
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
    let snapshot = shared.pool.lock().map_err(|_| "progress pool mutex poisoned")?.members.clone();
    let mut out = BufWriter::new(fresh_file(path)?);
    for m in &snapshot { write_line(&mut out, &m.value())?; }
    out.flush().map_err(|e| e.to_string())?; out.get_ref().sync_all().map_err(|e| e.to_string())
}
fn export_route_pool(shared:&Shared,path:&Path) -> Result<(),String> {
    let snapshot=shared.pool.lock().map_err(|_|"progress pool mutex poisoned")?.route_members.clone();
    let mut out=BufWriter::new(fresh_file(path)?);
    for m in &snapshot {write_line(&mut out,&m.value())?;}
    out.flush().map_err(|e|e.to_string())?;out.get_ref().sync_all().map_err(|e|e.to_string())
}
fn stats(shared: &Shared) -> Value {
    let load = |x: &AtomicU64| x.load(Ordering::Relaxed);
    let p = shared.pool.lock().unwrap(); let c = &shared.counts;
    let mut value=json!({"schema":1,"mode":"progress","t":shared.start.elapsed().as_secs_f64(),"level":45,"character":3,
        "objective":shared.objective.name(),"score_version":shared.objective.version(),
        "anchor":ANCHOR,"progress_window":WINDOW,"max_frame":MAX_FRAME,"frames":load(&c.frames),"trials":load(&c.trials),
        "initial_replays":load(&c.initial_replays),"pool":p.members.len(),"protected":p.members.iter().filter(|m| m.protected).count(),
        "launches":load(&c.launches),"input_no_progress_frames":load(&c.no_progress),"locked_frames":load(&c.locked),
        "port_aborts":load(&c.port_aborts),"lagging_frames":load(&c.lagging_frames),"lag_stops":load(&c.lag_stops),
        "incomplete_host_diagnostic_frames":load(&c.incomplete_host_diagnostics),
        "abort_site_histogram":*shared.sites.lock().unwrap(),"clean_frames":load(&c.clean_frames),"scored_windows":load(&c.windows),
        "site2_events":load(&c.site2),"admissions":load(&c.admissions),"equal_score_admissions":load(&c.equal_admissions),
        "equal_distance_admissions":if shared.objective==Objective::Finish {Some(load(&c.equal_admissions))} else {None},
        "current_pool_raw_duplicates":load(&c.duplicates),"noops_or_unscorable_shortening":load(&c.noops),
        "neutral_only_proposals_rejected":load(&c.neutral_only_proposals_rejected),
        "maximum_observed_current_pairs":*shared.maximum_current_pairs.lock().unwrap(),"best_window":shared.best.lock().unwrap().map(Score::value),
        "observed_finishes":load(&c.observed_finishes),"confirmed_finishes":load(&c.confirmed_finishes),
        "winner":shared.winner.lock().unwrap().is_some(),"stop_requested":shared.stop.load(Ordering::Relaxed)});
    value["diversity"]=json!({"pool":p.route_members.len(),"admissions":load(&c.route_admissions),
        "raw_duplicates":load(&c.route_duplicates),"below_cell_floor":load(&c.route_below_cell_floor),
        "pace_parent_picks":load(&c.pace_parent_picks),"route_parent_picks":load(&c.route_parent_picks),
        "best_route_window":if p.route_checkpoints {None} else {p.route_members.iter()
            .min_by(|a,b|a.route_window.value.total_cmp(&b.route_window.value)).map(|m|m.route_window.value())}});
    if p.route_checkpoints {
        value["diversity"]["checkpoints"]=json!(checkpoint_policy::FRAMES.map(|frame| {
            let members:Vec<_>=p.route_members.iter().filter(|m|m.member.route_checkpoint==Some(frame)).collect();
            json!({"checkpoint":frame,"pool":members.len(),"best_route_window":members.iter()
                .min_by(|a,b|a.route_window.value.total_cmp(&b.route_window.value)).map(|m|m.route_window.value())})
        }));
    }
    value
}
#[derive(Clone,Copy)]
struct BodyState { handle:u32, live:bool, position:[f64;2], velocity:[f64;2], frozen:bool }
fn body_state(s:&Sim,h:u32) -> BodyState {
    let missing=BodyState {handle:h,live:false,position:[f64::NAN;2],velocity:[f64::NAN;2],frozen:false};
    if h==NONE || h as usize>=MAX_BODIES || !s.ctx.world.bodies.is_alive(h) {return missing;}
    // A retained reference alone is insufficient. Check the actual world list,
    // bounded by arena capacity; malformed links fail closed for fitness only.
    let mut found=false;let mut current=s.ctx.world.body_list;
    for _ in 0..MAX_BODIES {
        if current==NONE {break;}
        if current as usize>=MAX_BODIES || !s.ctx.world.bodies.is_alive(current) {return missing;}
        if current==h {found=true;break;}
        current=body::get_next(&s.ctx,current);
    }
    if !found {return missing;}
    let p=body::get_world_center(&s.ctx,h);
    let v=body::get_linear_velocity(&s.ctx,h);
    BodyState {handle:h,live:true,position:[p.x,p.y],velocity:[v.x,v.y],frozen:body::is_frozen(&s.ctx,h)}
}
fn in_world(p: [f64;2]) -> bool { p[0].is_finite() && p[1].is_finite()
    && p[0] > -4.0 && p[0] < 324.0 && p[1] > -84.0 && p[1] < 164.0 }
fn continuation(o: Observation) -> bool { !o.port_abort && o.dad_alive_after && o.chest_live_after
    && in_world(o.chest_after) && o.frame_after < MAX_FRAME }

/// The terminal evidence belongs to this replay, never to its parent or window.
/// Finish/Fan retain their existing policy; confirmed finishes are handled first.
fn earned_score(objective: Objective, observed: Option<Score>,
    terminal: survival_policy::TerminalEvidence, final_chest_handle: u32) -> Option<Score> {
    if objective != Objective::Launch {return observed;}
    observed.filter(|score| survival_policy::eligible(terminal)
        && score.window.chest_handle == final_chest_handle
        && score.window.focus_handle == final_chest_handle)
}

fn observed_finish(cfg: &Config, shared: &Shared, masks: &[u8], frame: u32) -> Result<(), String> {
    shared.counts.observed_finishes.fetch_add(1, Ordering::Relaxed);
    let confirmed = confirm_any_finish(cfg.level,cfg.character,masks);
    let qualifies = matches!(&confirmed, Ok((f,_)) if *f == frame && *f <= MAX_FRAME);
    shared.emit("finishes",json!({"schema":1,"observed_frame":frame,"tas":tas_text(masks),
        "native_confirmation":match &confirmed {Ok((f,moved))=>json!({"frame":f,"moved":moved}),Err(e)=>json!({"error":e})},
        "confirmed_within299_and_same_observed_frame":qualifies,"original_player_verified":false}))?;
    if qualifies {
        let mut winner = shared.winner.lock().map_err(|_| "progress winner mutex poisoned")?;
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
struct ReplayOutcome { id:u64, score:Option<Score>, route:Option<Score>,
    checkpoints:[Option<Score>;2], neutral_eject_certificate:NeutralEjectCertificate }

/// Observe every actual invocation, including original input-action retries.
/// This does not change any core call, requested mask, finite EOF or Flow result.
fn certified_step(s: &mut Sim, certificate: &mut CertificateBuilder) -> super::sim::Frame {
    let input_index=s.replay.iteration as usize;
    let input_allowed_before=s.replay.input_allowed.get();
    let before=PreInputState {dad_dead:s.ctx.character.primary.dead.get(),
        dad_ejected:s.ctx.character.primary.bicycle.ejected.get(),
        child_dead:s.ctx.character.child.dead.get(),
        child_ejected:s.ctx.character.child.child_seat.ejected.get()};
    let f=s.step(0);
    certificate.observe(Invocation {input_index,requested_mask:f.mask,input_allowed_before,
        iteration_after:s.replay.iteration as usize,port_abort:f.port_abort,before});
    f
}

/// Fresh finite-input replay: full planned literal, then zeros, matching confirmation.
/// World frames bound work independently of aborts and consumed input entries.
fn replay(cfg: &Config, shared: &Shared, masks: &[u8], changed: usize, parent: u64, initial: bool,
    parent_lane:&str,parent_route_checkpoint:Option<u32>) -> Result<ReplayOutcome, String> {
    let replay_id=shared.counts.replay_ids.fetch_add(1,Ordering::Relaxed);
    let mut window = ProgressWindow::new(masks.len(),changed).map_err(str::to_owned)?;
    let mut certificate=CertificateBuilder::new(masks).map_err(str::to_owned)?;
    let mut s = Sim::new(cfg.level,cfg.character)?;
    s.replay = ReplayState::new(masks.len() as u32); s.masks = masks.to_vec();
    if !s.ctx.level.specials[..s.ctx.level.count as usize].iter().any(|sp| sp.kind==hw_data::SpecialKind::FinishLine) {
        return Err("progress level has no FinishLine region".into());
    }
    let mut best: Option<Score> = None; let mut route_best:Option<Score>=None;
    let mut checkpoint_windows:[Option<Score>;2]=[None;2];
    let mut completed = true; let mut lag=LagGuard::default();let mut saw_port_abort=false;
    let mut milestones=Vec::new();
    // Exact short initial bases retain their own finite N+60 confirmation horizon.
    // Live proposals have N>=239 and consequently cover the entire299 ceiling.
    let horizon=MAX_FRAME.min(masks.len() as u32+60);
    let mut reason=if horizon<MAX_FRAME {"finite_confirmation_horizon"} else {"ceiling"};
    for _ in 0..horizon {
        if shared.should_stop(cfg) { completed=false; reason="stop_or_budget"; break; }
        let before_frame=s.frame(); let before_iteration=s.replay.iteration;
        let before_allowed=s.replay.input_allowed.get(); let before_lock=s.ctx.world.lock.get();
        let before_alive=!s.ctx.level.dead.get() && !s.ctx.character.primary.dead.get();
        let before_chest=body_state(&s,s.ctx.character.primary.bodies[CHEST]);
        let before_focus=body_state(&s,s.ctx.level.focus);
        let f=certified_step(&mut s,&mut certificate); shared.counts.frames.fetch_add(1,Ordering::Relaxed);
        let after_chest=body_state(&s,s.ctx.character.primary.bodies[CHEST]);
        let after_focus=body_state(&s,s.ctx.level.focus);
        let o=Observation {frame_before:before_frame,frame_after:f.frame,iteration_before:before_iteration,
            iteration_after:s.replay.iteration,input_allowed_before:before_allowed,input_allowed_after:s.replay.input_allowed.get(),
            mask_matches_requested:masks.get(before_iteration as usize).is_some_and(|&m| m==f.mask),
            locked_before:before_lock,locked_after:s.ctx.world.lock.get(),dad_alive_before:before_alive,
            dad_alive_after:!s.ctx.level.dead.get() && !s.ctx.character.primary.dead.get(),
            chest_live_before:before_chest.live,chest_live_after:after_chest.live,
            chest_handle_before:before_chest.handle,chest_handle_after:after_chest.handle,
            chest_before:before_chest.position,chest_after:after_chest.position,
            chest_velocity_after:after_chest.velocity,
            chest_frozen_before:before_chest.frozen,chest_frozen_after:after_chest.frozen,
            focus_live_before:before_focus.live,focus_live_after:after_focus.live,
            focus_handle_before:before_focus.handle,focus_handle_after:after_focus.handle,
            focus_before:before_focus.position,focus_after:after_focus.position,
            focus_frozen_before:before_focus.frozen,focus_frozen_after:after_focus.frozen,
            finish_distance_after:super::sim::finish_region_distance(&s.ctx,after_focus.position[0],after_focus.position[1]),
            pairs_after:f.pairs,qe:f.qe,qe_raw:f.qe_raw,site:f.site,port_abort:f.port_abort,lagging:f.lagging,finished:f.finished,
            host_diagnostics_complete:f.jointed_grips_complete};
        // This must precede port-abort, lock, fitness, liveness and world-bound gates.
        if f.finished { observed_finish(cfg,shared,masks,f.frame)?; }
        if before_allowed && s.replay.iteration <= before_iteration { shared.counts.no_progress.fetch_add(1,Ordering::Relaxed); }
        if o.locked_before || o.locked_after { shared.counts.locked.fetch_add(1,Ordering::Relaxed); }
        if f.port_abort {saw_port_abort=true;shared.counts.port_aborts.fetch_add(1,Ordering::Relaxed);}
        if f.lagging { shared.counts.lagging_frames.fetch_add(1,Ordering::Relaxed); }
        if !f.jointed_grips_complete {shared.counts.incomplete_host_diagnostics.fetch_add(1,Ordering::Relaxed);}
        let lag_stop=lag.observe(f.lagging,cfg.lag_frames);
        if f.site != 0 { let mut sites=shared.sites.lock().unwrap();
            let i=(f.site as usize).min(sites.len()-1); sites[i]=sites[i].saturating_add(1); }
        { let mut m=shared.maximum_current_pairs.lock().unwrap(); *m=(*m).max(f.pairs); }
        let decision=window.observe(o);
        if cfg.diversity_pool>0 && [100,140,180,240,299].contains(&f.frame) {
            milestones.push(json!({"actual_world_frame":f.frame,"primary_dad_alive":o.dad_alive_after,
                "primary_chest_live":after_chest.live,"primary_chest_handle":after_chest.handle,
                "actual_focus_handle":after_focus.handle,"actual_focus_position":after_focus.position,
                "primary_chest_velocity":after_chest.velocity,"locked":o.locked_after,"port_abort":f.port_abort}));
        }
        if decision.clean {shared.counts.clean_frames.fetch_add(1,Ordering::Relaxed);}
        if decision.observed_pair_pool_exhausted {
            shared.counts.site2.fetch_add(1,Ordering::Relaxed);
            shared.emit("pairmax",json!({"schema":1,"frame":f.frame,"iteration":s.replay.iteration,"site":f.site,
                "pairs_current":f.pairs,"locked_before":o.locked_before,"locked_after":o.locked_after,
                "input_progress":s.replay.iteration.saturating_sub(before_iteration),"parent":parent,"tas":tas_text(masks),
                "progress_credit":false,"original_player_verified":false,
                "actual_level_focus":after_focus.position,"primary_chest":after_chest.position,
                "qe_telemetry":f.qe,"qe_raw_telemetry":f.qe_raw,"host_diagnostics_complete":f.jointed_grips_complete}))?;
        }
        if f.displacement >= 20.0 {
            shared.counts.launches.fetch_add(1,Ordering::Relaxed);
            shared.emit("launches",json!({"schema":1,"frame":f.frame,"iteration":s.replay.iteration,"from":f.chest_before,
                "to":f.chest_after,"moved":f.displacement,"alive_before":f.alive_before,"alive_after":f.alive_after,
                "motion_scope":"Existing Frame active-actor chest alias; separate primary Dad/focus positions below",
                "site":f.site,"locked_before":o.locked_before,"locked_after":o.locked_after,"pairs_current":f.pairs,
                "finish_focus_distance_m":o.finish_distance_after,"actual_level_focus":after_focus.position,
                "primary_chest":after_chest.position,"primary_dad_alive":o.dad_alive_after,
                "qe_telemetry":f.qe,"qe_raw_telemetry":f.qe_raw,"host_diagnostics_complete":f.jointed_grips_complete,
                "lagging":f.lagging,"parent":parent,"tas":tas_text(masks),"finished":f.finished}))?;
            // Deliberately no early return or endpoint dedup.
        }
        if let Some(scored)=decision.score {
            if let Some(score)=Score::from_window(cfg.objective,scored,f.frame,s.replay.iteration) {
                shared.counts.windows.fetch_add(1,Ordering::Relaxed);
                if best.map_or(true,|b| score.value<b.value) { best=Some(score); }
            }
            if cfg.diversity_pool>0 {
                if let Some(route)=Score::from_window(Objective::Fan,scored,f.frame,s.replay.iteration) {
                    if cfg.route_checkpoints {
                        if let Some(i)=checkpoint_policy::index(route.frame,route.window.world_frames) {
                            checkpoint_windows[i]=Some(route);
                        }
                    } else if route_best.map_or(true,|b|route.value<b.value) {route_best=Some(route);}
                }
            }
        }
        if shared.stop.load(Ordering::Relaxed) { completed=false; reason="confirmed_finish_or_stop"; break; }
        if lag_stop { shared.counts.lag_stops.fetch_add(1,Ordering::Relaxed); reason="lag_guard"; break; }
        if !continuation(o) { reason=if f.port_abort {"port_abort"} else if !o.dad_alive_after || !o.chest_live_after {"dead_or_removed_primary_chest"}
            else if !in_world(o.chest_after) {"outside_world"} else {"ceiling"}; break; }
    }
    let final_chest=body_state(&s,s.ctx.character.primary.bodies[CHEST]);
    let final_focus=body_state(&s,s.ctx.level.focus);
    let terminal=survival_policy::TerminalEvidence {
        evaluated_to_horizon:completed && reason=="ceiling" && horizon==MAX_FRAME,
        actual_world_frames:s.frame(),dad_alive:!s.ctx.level.dead.get() && !s.ctx.character.primary.dead.get(),
        chest_live:final_chest.live,chest_in_world:in_world(final_chest.position),
        actual_focus_is_primary_chest:final_focus.live && final_focus.handle==final_chest.handle,
        port_abort:saw_port_abort,lag_stopped:reason=="lag_guard",
    };
    let earned=earned_score(cfg.objective,best,terminal,final_chest.handle);
    let earned_route=earned_score(Objective::Launch,route_best,terminal,final_chest.handle);
    let earned_checkpoints=checkpoint_windows.map(|score|earned_score(Objective::Launch,score,terminal,final_chest.handle));
    let mut trial=json!({"schema":1,"trial_id":replay_id,"parent":parent,"parent_pool_lane":parent_lane,
        "initial_base":initial,"planned_entries":masks.len(),
        "earliest_changed_index":if initial {None} else {Some(changed)},"credit_after_iteration":ANCHOR.max(changed),
        "confirmation_compatible_horizon":horizon,"actual_world_frames":s.frame(),"consumed_iteration":s.replay.iteration,"best_window":earned.map(Score::value),
        "best_observed_window":best.map(Score::value),"score_version":cfg.objective.version(),
        "route_window":earned_route.map(Score::value),"best_observed_route_window":route_best.map(Score::value),
        "actual_route_milestones":milestones,
        "launch_terminal_survival":if cfg.objective==Objective::Launch {Some(json!({
            "eligible":survival_policy::eligible(terminal),"evaluated_to_horizon":terminal.evaluated_to_horizon,
            "required_actual_world_frames":MAX_FRAME,"actual_world_frames":terminal.actual_world_frames,
            "primary_dad_alive":terminal.dad_alive,"primary_chest_live_in_world_list":terminal.chest_live,
            "primary_chest_within_world_bounds":terminal.chest_in_world,
            "actual_focus_is_primary_chest":terminal.actual_focus_is_primary_chest,
            "port_abort":terminal.port_abort,"lag_stopped":terminal.lag_stopped,
            "earlier_window_is_earned_fitness":earned.is_some(),"finish_claim":false}))} else {None},
        "evaluation_ended":completed,"ended_reason":reason,"consecutive_lag_frames":lag.consecutive(),
        "final_actual_level_focus_handle":s.ctx.level.focus,"final_actual_level_focus":final_focus.position,
        "final_primary_chest_handle":final_chest.handle,"final_primary_chest":final_chest.position,
        "final_primary_dad_dead":s.ctx.character.primary.dead.get(),"unobserved_later_tail_is_negative":false});
    trial["parent_route_checkpoint"]=json!(parent_route_checkpoint);
    if cfg.route_checkpoints {
        // Each checkpoint came only from this replay's clean ProgressWindow;
        // earning waits for the same replay's actual299 terminal evidence.
        trial["route_checkpoint_windows"]=json!(checkpoint_policy::FRAMES.iter().enumerate().map(|(i,&frame)|
            json!({"checkpoint":frame,"observed_window":checkpoint_windows[i].map(Score::value),
                "earned_window":earned_checkpoints[i].map(Score::value)})).collect::<Vec<_>>());
    }
    shared.emit("trials",trial)?;
    Ok(ReplayOutcome {id:replay_id,score:earned,route:earned_route,checkpoints:earned_checkpoints,
        neutral_eject_certificate:certificate.seal()})
}

fn admit(cfg: &Config, shared: &Shared, parent: &Member, masks: Vec<u8>, score: Score,
    neutral_eject_certificate: NeutralEjectCertificate, changed: usize,trial_id:u64) -> Result<(), String> {
    let record_masks=masks.clone();
    let admission=shared.pool.lock().map_err(|_| "progress pool mutex poisoned")?
        .admit(parent,masks,score,neutral_eject_certificate,cfg.pool_size);
    match admission {
        Admission::Duplicate => {shared.counts.duplicates.fetch_add(1,Ordering::Relaxed);},
        Admission::BelowFloor => {},
        Admission::Added {id,evicted,equal_count} => {
            shared.counts.admissions.fetch_add(1,Ordering::Relaxed);
            if equal_count {shared.counts.equal_admissions.fetch_add(1,Ordering::Relaxed);}
            let improved={let mut b=shared.best.lock().unwrap(); let improved=b.map_or(true,|old| score.value<old.value);
                if improved {*b=Some(score);} improved};
            shared.emit("improvements",json!({"schema":1,"trial_id":trial_id,"pool_lane":"pace","id":id,"ancestor":parent.id,"root":parent.root,
                "generation":parent.generation.saturating_add(1),"new_to_current_pool":true,"equal_score_admission":equal_count,
                "equal_distance_admission":if cfg.objective==Objective::Finish {Some(equal_count)} else {None},
                "best_improvement":improved,"evicted":evicted,"earliest_changed_index":changed,"anchor":ANCHOR,
                "parent_route_checkpoint":parent.route_checkpoint,
                "planned_entries":record_masks.len(),"best_window":score.value(),"tas":tas_text(&record_masks),
                "qualification":"Native six-clean-frame search surrogate with living primary Dad; QE/pair telemetry only; no global novelty, pairmax, finish or Flash match claim"}))?;
        }
    }
    Ok(())
}
fn admit_route(cfg:&Config,shared:&Shared,parent:&Member,masks:&[u8],pace:Option<Score>,route:Score,
    certificate:NeutralEjectCertificate,changed:Option<usize>,trial_id:u64,checkpoint:Option<u32>) -> Result<(),String> {
    let cap=if cfg.route_checkpoints {
        if !checkpoint.is_some_and(|frame|checkpoint_policy::FRAMES.contains(&frame)) {
            return Err("checkpoint mode admission requires an actual checkpoint".into());
        }
        checkpoint_policy::per_checkpoint_capacity(cfg.diversity_pool).ok_or("invalid checkpoint capacity")?
    } else {if checkpoint.is_some() {return Err("legacy admission has no checkpoint".into());} cfg.diversity_pool};
    let admission=shared.pool.lock().map_err(|_|"progress pool mutex poisoned")?
        .admit_route_at(parent,masks.to_vec(),pace,route,certificate,cap,checkpoint);
    match admission {
        Admission::Duplicate=>{shared.counts.route_duplicates.fetch_add(1,Ordering::Relaxed);},
        Admission::BelowFloor=>{shared.counts.route_below_cell_floor.fetch_add(1,Ordering::Relaxed);},
        Admission::Added {id,evicted,equal_count}=>{
            shared.counts.route_admissions.fetch_add(1,Ordering::Relaxed);
            let key=diversity_policy::key(route.window.focus_position,route.frame).ok_or("invalid admitted route cell")?;
            shared.emit("route",json!({"schema":1,"pool_lane":"survivor_route","reserve_version":cfg.reserve_version(),
                "trial_id":trial_id,"id":id,"ancestor":parent.id,"root":parent.root,
                "generation":parent.generation.saturating_add(1),"initial_seed":changed.is_none(),
                "evicted":evicted,"equal_cell_route_admission":equal_count,"earliest_changed_index":changed,
                "route_checkpoint":checkpoint,"parent_route_checkpoint":parent.route_checkpoint,
                "evicted_route_checkpoint":if evicted.is_some() {checkpoint}else{None},
                "route_cell":{"x":key.x,"y":key.y,"band":key.band},"route_window":route.value(),
                "best_window":pace.map(Score::value),"planned_entries":masks.len(),"tas":tas_text(masks),
                "same_replay_terminal_survival":true,"finish_claim":false,"original_player_verified":false}))?;
        }
    }
    Ok(())
}
fn admit_replay_routes(cfg:&Config,shared:&Shared,parent:&Member,masks:&[u8],
    actual:&ReplayOutcome,changed:Option<usize>) -> Result<(),String> {
    if cfg.route_checkpoints {
        for (i,route) in actual.checkpoints.iter().copied().enumerate() {
            if let Some(route)=route {admit_route(cfg,shared,parent,masks,actual.score,route,
                actual.neutral_eject_certificate.clone(),changed,actual.id,Some(checkpoint_policy::FRAMES[i]))?;}
        }
    } else if let Some(route)=actual.route {
        admit_route(cfg,shared,parent,masks,actual.score,route,actual.neutral_eject_certificate.clone(),changed,actual.id,None)?;
    }
    Ok(())
}
fn worker(w: usize,cfg: &Config,shared: &Shared) -> Result<(),String> {
    let mut rng=Rng(cfg.seed ^ (w as u64+1).wrapping_mul(0x5851f42d4c957f2d));
    while !shared.should_stop(cfg) {
        let (parent,from_route)={let mut pool=shared.pool.lock().map_err(|_|"progress pool mutex poisoned")?;
            if cfg.diversity_pool>0 && w>=cfg.threads/2 {pool.pick_route(&mut rng)} else {(pool.pick(&mut rng),false)}};
        if from_route {shared.counts.route_parent_picks.fetch_add(1,Ordering::Relaxed);}
        else {shared.counts.pace_parent_picks.fetch_add(1,Ordering::Relaxed);}
        let Some((masks,changed))=proposal(&parent.masks,&mut rng,cfg.window,cfg.extra)? else {
            shared.counts.noops.fetch_add(1,Ordering::Relaxed); continue;
        };
        if parent.neutral_eject_certificate.as_ref()
            .is_some_and(|c| c.rejects_neutral_only(&parent.masks,&masks)) {
            shared.counts.neutral_only_proposals_rejected.fetch_add(1,Ordering::Relaxed);
            continue;
        }
        shared.counts.trials.fetch_add(1,Ordering::Relaxed);
        let actual=replay(cfg,shared,&masks,changed,parent.id,false,if from_route {"survivor_route"}else{"pace"},parent.route_checkpoint)?;
        admit_replay_routes(cfg,shared,&parent,&masks,&actual,Some(changed))?;
        if let Some(score)=actual.score {admit(cfg,shared,&parent,masks,score,actual.neutral_eject_certificate,changed,actual.id)?;}
    }
    Ok(())
}

pub fn run(cfg: Config) -> Result<Vec<u8>,String> {
    cfg.validate()?;
    super::sim::set_lag_ms(cfg.lag_ms); super::sim::set_lag_frames(cfg.lag_frames);
    let start=Instant::now(); let budget=Duration::try_from_secs_f64(cfg.hours*3600.0).map_err(|e| e.to_string())?;
    let deadline=start.checked_add(budget).ok_or("progress deadline exceeds platform clock")?;
    // create_dir (not create_dir_all) refuses every existing attempt.
    std::fs::create_dir(&cfg.run_dir).map_err(|e| format!("fresh progress run {}: {e}",cfg.run_dir.display()))?;
    let shared=Shared {pool:Mutex::new(Pool::with_checkpoints(&cfg.bases,cfg.objective,cfg.route_checkpoints)),stop:AtomicBool::new(false),error:Mutex::new(None),winner:Mutex::new(None),
        logs:Mutex::new(Logs::new(&cfg.run_dir)?),counts:Counts::default(),sites:Mutex::new([0;12]),best:Mutex::new(None),
        maximum_current_pairs:Mutex::new(0),start,deadline,objective:cfg.objective};
    let mut config=fresh_file(&cfg.run_dir.join("config.json"))?;
    let mut config_value=json!({"schema":1,"mode":"progress","level":45,"character":3,"anchor":ANCHOR,"progress_window":WINDOW,
        "max_frame":MAX_FRAME,"threads":cfg.threads,"hours":cfg.hours,"seed":cfg.seed,"window":cfg.window,"extra":cfg.extra,
        "lag_ms":cfg.lag_ms,"lag_cutoff_us":(cfg.lag_ms*1000.0) as u32,"lag_frames":cfg.lag_frames,"laggy_frames_receive_progress_credit":false,
        "ordinary_pool_capacity":cfg.pool_size,"initial_raw_bases":cfg.bases.len(),"initial_unique_bases":shared.pool.lock().unwrap().members.len(),
        "initialization_only":cfg.hours==0.0,"fresh_replay":true,"finite_planned_tas_semantics":true,"live_plan_min_entries":239,
        "objective":cfg.objective.name(),"score_version":cfg.objective.version(),
        "score_objective":match cfg.objective {Objective::Finish=>"minimize worst ctx.level.focus FinishLine-region distance over six clean fresh actual samples",
            Objective::Fan=>"minimize six-clean-frame actual-focus fan-route ranking; geometry milestones are search surrogates only",
            Objective::Launch=>"minimize worst six-clean-frame remaining route path/time demand minus actual Dad route-leg velocity, earned only by the same fresh replay reaching299 with living primary Dad CHEST as actual focus; frame299 has no pace sample"},
        "launch_requires_own_terminal_survival":cfg.objective==Objective::Launch,
        "zero_distance_is_finish":false,"qe_pair_provenance_is_fitness":false,
        "neutral_eject_only_proposal_filter":true,"neutral_eject_certificate_source":"Each exact literal fresh replay; pre-input flags intersected over every actual invocation; host port failure invalidates proof",
        "qe_telemetry_scope":"Existing BODY-joint-edge metric, which can include stale links; owner aggregate separately recorded; neither contributes to progress fitness",
        "qualification":"Native search surrogate with living Dad; native exact finish confirmation and original-player verification remain separate"});
    config_value["diversity"]=json!({"capacity":cfg.diversity_pool,"cell_metres":4,"frames_per_band":30,
        "raws_per_cell":2,"reserve_version":cfg.reserve_version(),"route_checkpoints":cfg.route_checkpoints,
        "pace_designated_workers":if cfg.diversity_pool>0 {cfg.threads/2}else{cfg.threads},
        "route_designated_workers":if cfg.diversity_pool>0 {cfg.threads-cfg.threads/2}else{0},
        "route_designated_parent_selection":if cfg.route_checkpoints {
            "75% uniformly represented root then uniformly a member across both checkpoints,25% original pace/protected selection; no equal checkpoint share claim; empty reserve uses original pool"
        } else {"75% uniform route reserve,25% original pace/protected selection; empty reserve uses original pool"},
        "route_window_policy":if cfg.route_checkpoints {
            "Independent actual six-clean post-change windows ending exactly180/240; half capacity per checkpoint; same raw may occupy each once; own299 survival required; no global pace floor or inherited score"
        } else {"Each literal's minimum actual six-clean-post-change-sample Fan route cost; descriptor uses its measured final focus position/world frame; own299 survival required; no global pace floor"}});
    if cfg.route_checkpoints {
        config_value["diversity"]["checkpoint_frames"]=json!(checkpoint_policy::FRAMES);
        config_value["diversity"]["capacity_per_checkpoint"]=json!(cfg.diversity_pool/2);
    }
    write_line(&mut config,&config_value)?;
    // Preserve every input ordinal including duplicate full bases, separately from
    // the canonical protected pool membership.
    let mut initial=fresh_file(&cfg.run_dir.join("initial-bases.jsonl"))?;
    for (i,masks) in cfg.bases.iter().enumerate() {write_line(&mut initial,&json!({"ordinal":i,"frame":masks.len(),"tas":tas_text(masks)}))?;}
    export_pool(&shared,&cfg.run_dir.join("pool-initial.jsonl"))?;
    export_route_pool(&shared,&cfg.run_dir.join("route-pool-initial.jsonl"))?;
    let mut stats_log=fresh_file(&cfg.run_dir.join("stats.jsonl"))?;
    write_line(&mut stats_log,&stats(&shared))?;
    if cfg.hours>0.0 {
        let bases=shared.pool.lock().unwrap().members.clone();
        for m in &bases {
            if shared.should_stop(&cfg) {break;}
            shared.counts.initial_replays.fetch_add(1,Ordering::Relaxed);
            match replay(&cfg,&shared,&m.masks,ANCHOR,m.id,true,"protected_initial",None) {
                Ok(replayed) => {let score=replayed.score;
                    if let Some(actual)=shared.pool.lock().unwrap().members.iter_mut().find(|p| p.id==m.id) {
                        actual.score=score;actual.neutral_eject_certificate=Some(replayed.neutral_eject_certificate.clone());}
                    if let Err(e)=admit_replay_routes(&cfg,&shared,m,&m.masks,&replayed,None) {shared.fail(e);break;}
                    if let Some(score)=score {let mut b=shared.best.lock().unwrap(); if b.map_or(true,|old| score.value<old.value) {*b=Some(score);}}},
                Err(e) => {shared.fail(e);break;}
            }
        }
        std::thread::scope(|scope| {
            let mut handles=Vec::new();
            for w in 0..cfg.threads {
                if shared.should_stop(&cfg) {break;}
                let (cfg,shared)=(&cfg,&shared);
                match std::thread::Builder::new().name(format!("progress-{w}")).stack_size(16<<20)
                    .spawn_scoped(scope,move || {
                        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| worker(w,cfg,shared))) {
                            Ok(Ok(()))=>{}, Ok(Err(e))=>shared.fail(e), Err(_)=>shared.fail("progress worker panicked".into()),
                        }
                    }) {
                    Ok(h)=>handles.push(h),Err(e)=>{shared.fail(format!("progress worker spawn: {e}"));break;}
                }
            }
            let mut next_stats=Instant::now(); let mut next_checkpoint=Instant::now()+Duration::from_secs(60); let mut sequence=0u64;
            while !shared.should_stop(&cfg) {
                if Instant::now()>=next_stats {if let Err(e)=write_line(&mut stats_log,&stats(&shared)) {shared.fail(e);}
                    next_stats=Instant::now()+Duration::from_secs(2);}
                if Instant::now()>=next_checkpoint {let path=cfg.run_dir.join(format!("pool-checkpoint-{sequence:06}.jsonl")); sequence+=1;
                    if let Err(e)=export_pool(&shared,&path) {shared.fail(e);}
                    if let Err(e)=export_route_pool(&shared,&cfg.run_dir.join(format!("route-pool-checkpoint-{:06}.jsonl",sequence-1))) {shared.fail(e);}
                    next_checkpoint=Instant::now()+Duration::from_secs(60);}
                std::thread::sleep(Duration::from_millis(250));
            }
            shared.stop.store(true,Ordering::Relaxed);
            for h in handles {if h.join().is_err() {shared.fail("progress worker panicked".into());}}
        });
    }
    shared.stop.store(true,Ordering::Relaxed);
    // Only after all workers exited: immutable final complete pool and stats.
    export_pool(&shared,&cfg.run_dir.join("pool.jsonl"))?;
    export_route_pool(&shared,&cfg.run_dir.join("route-pool.jsonl"))?;
    let mut final_stats=stats(&shared); final_stats["ended"]=true.into();
    final_stats["initialization_only"]=(cfg.hours==0.0).into();
    final_stats["stop_file_present"]=cfg.run_dir.join("STOP").exists().into();
    final_stats["error"]=json!(shared.error.lock().unwrap().clone());
    write_line(&mut stats_log,&final_stats)?; stats_log.sync_all().map_err(|e| e.to_string())?;
    if let Some(e)=shared.error.lock().unwrap().clone() {return Err(e);}
    let winner=shared.winner.lock().unwrap().clone();
    winner.ok_or_else(|| "progress ended without a confirmed winner".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn uncertified(masks:&[u8]) -> NeutralEjectCertificate {
        CertificateBuilder::new(masks).unwrap().seal()
    }
    fn score(distance:f64) -> Score {Score {value:distance,frame:90,iteration:90,
        objective:Objective::Finish,route:None,launch:None,
        window:WindowScore {worst_distance_m:distance,distances:[distance;WINDOW],pairs:[100;WINDOW],
            qe:[1.0;WINDOW],qe_raw:[1.0;WINDOW],focus_handle:1,chest_handle:1,
            focus_position:[29.0,74.0],chest_position:[29.0,74.0],focus_positions:[[29.0,74.0];WINDOW],
            chest_velocities:[[0.0,0.0];WINDOW],world_frames:std::array::from_fn(|i|85+i as u32)}}}
    #[test] fn smaller_distance_wins_and_equal_distance_raw_diversity_stays_real() {
        let a=vec![17;139];let mut p=Pool::new(&[a.clone()],Objective::Finish);let parent=p.members[0].clone();
        let mut b=a.clone();b[90]=49;let mut c=a.clone();c[90]=80;
        assert!(matches!(p.admit(&parent,b.clone(),score(90.0),uncertified(&b),1),Admission::Added {evicted:None,..}));
        assert_eq!(p.admit(&parent,b.clone(),score(1.0),uncertified(&b),1),Admission::Duplicate);
        assert!(matches!(p.admit(&parent,c.clone(),score(90.0),uncertified(&c),1),Admission::Added {evicted:Some(1),equal_count:true,..}));
        let mut d=c.clone();d[91]^=16;assert_eq!(p.admit(&parent,d.clone(),score(91.0),uncertified(&d),1),Admission::BelowFloor);
        assert!(matches!(p.admit(&parent,d.clone(),score(40.0),uncertified(&d),1),Admission::Added {evicted:Some(2),equal_count:false,..}));
        assert_eq!(p.members[0].masks,a);assert!(p.members[0].protected);
        assert_eq!(p.members[1].score.unwrap().value.to_bits(),40.0_f64.to_bits());
    }
    #[test] fn initial_full_bases_remain_unscored_protected_without_imported_fitness() {
        let a=vec![17;139];let b=vec![49;289];let p=Pool::new(&[a.clone(),b.clone(),a.clone()],Objective::Finish);
        assert_eq!(p.members.len(),2);assert_eq!(p.members[0].source_ordinals,[0,2]);
        assert_eq!(p.members[0].masks,a);assert_eq!(p.members[1].masks,b);
        assert!(p.members.iter().all(|m|m.protected&&m.score.is_none()&&m.neutral_eject_certificate.is_none()));
    }
    #[test] fn later_route_scaffold_has_a_real_place_without_lowering_the_pace_floor() {
        let a=vec![16;299];let mut pool=Pool::new(&[a.clone()],Objective::Launch);
        let parent=pool.members[0].clone();let mut early=a.clone();early[74]=232;
        let mut fast=score(114.0);fast.value=-35.0;fast.objective=Objective::Launch;
        assert!(matches!(pool.admit(&parent,early.clone(),fast,uncertified(&early),1),Admission::Added {id:1,..}));
        let mut later=a.clone();later[175]^=8;
        let mut slow=fast;slow.value=27.29497688754132;
        assert_eq!(pool.admit(&parent,later.clone(),slow,uncertified(&later),1),Admission::BelowFloor);
        let mut window=score(44.0).window;
        window.focus_position=[80.90892070807715,82.92967190877987];
        window.focus_positions=[window.focus_position;WINDOW];
        window.world_frames=std::array::from_fn(|i|294+i as u32);
        let route=Score::from_window(Objective::Fan,window,299,299).unwrap();
        assert!(matches!(pool.admit_route(&parent,later.clone(),Some(slow),route,uncertified(&later),64),Admission::Added {id:2,..}));
        assert_eq!(pool.members[1].score.unwrap().value.to_bits(),(-35.0_f64).to_bits());
        assert_eq!(pool.route_members[0].member.score.unwrap().value.to_bits(),slow.value.to_bits());
        assert_eq!(pool.route_members[0].member.masks,later);
        assert_eq!(pool.route_members[0].value()["route_window"],route.value());
        assert_eq!(pool.admit_route(&parent,later.clone(),None,route,uncertified(&later),64),Admission::Duplicate);
        let mut other=later.clone();other[176]^=4;
        assert!(matches!(pool.admit_route(&parent,other.clone(),None,route,uncertified(&other),64),Admission::Added {id:3,..}));
        assert!(pool.route_members[1].member.score.is_none());
        let mut reserve_picks=0;let mut pace_picks=0;
        let mut rng=Rng(7381);
        for _ in 0..1000 {let (chosen,reserve)=pool.pick_route(&mut rng);if reserve {
            reserve_picks+=1;assert!([2,3].contains(&chosen.id));
        } else {pace_picks+=1;assert!([0,1].contains(&chosen.id));}}
        assert!(reserve_picks>0&&pace_picks>0);
    }
    fn at_checkpoint(frame:u32,x:f64) -> Score {
        let mut window=score(44.0).window;
        window.focus_position=[x,78.0];window.chest_position=window.focus_position;
        window.focus_positions=[window.focus_position;WINDOW];
        window.world_frames=std::array::from_fn(|i|frame-5+i as u32);
        Score::from_window(Objective::Fan,window,frame,frame).unwrap()
    }
    #[test] fn checkpoint_raw_dedup_and_eviction_are_independent_and_map_to_the_global_vector() {
        let a=vec![16;299];let mut pool=Pool::with_checkpoints(&[a.clone()],Objective::Launch,true);
        let parent=pool.members[0].clone();let later=at_checkpoint(240,70.0);let early=at_checkpoint(180,20.0);
        assert_eq!(pool.admit_route_at(&parent,a.clone(),None,later,uncertified(&a),1,Some(240)),
            Admission::Added {id:1,evicted:None,equal_count:false});
        assert_eq!(pool.admit_route_at(&parent,a.clone(),None,early,uncertified(&a),1,Some(180)),
            Admission::Added {id:2,evicted:None,equal_count:false});
        assert_eq!(pool.admit_route_at(&parent,a.clone(),None,early,uncertified(&a),1,Some(180)),Admission::Duplicate);
        let mut b=a.clone();b[174]^=4;
        assert_eq!(pool.admit_route_at(&parent,b.clone(),None,at_checkpoint(180,24.0),uncertified(&b),1,Some(180)),
            Admission::Added {id:3,evicted:Some(2),equal_count:false});
        assert_eq!(pool.route_members.len(),2);
        assert_eq!(pool.route_members[0].member.id,1);assert_eq!(pool.route_members[0].member.route_checkpoint,Some(240));
        assert_eq!(pool.route_members[1].member.id,3);assert_eq!(pool.route_members[1].member.route_checkpoint,Some(180));
        assert!(pool.route_members.iter().all(|m|m.member.score.is_none()));
        assert_eq!(pool.route_members[1].value()["reserve_version"],"dad-survivor-route-checkpoints-v1");
        assert_eq!(pool.route_members[1].value()["route_checkpoint"],180);
        assert_eq!(pool.admit_route_at(&parent,a.clone(),None,later,uncertified(&a),1,Some(180)),Admission::BelowFloor);
        // Pace crossfeeding keeps ancestry but does not label its member as a checkpoint slot.
        let route_parent=pool.route_members[1].member.clone();let mut c=b.clone();c[180]^=8;
        assert!(matches!(pool.admit(&route_parent,c.clone(),score(44.0),uncertified(&c),1),Admission::Added {id:4,..}));
        assert_eq!(pool.members[1].ancestor,Some(3));assert_eq!(pool.members[1].route_checkpoint,None);
    }
    #[test] fn stronger_early_velocity_has_no_role_in_checkpoint_route_replacement() {
        let a=vec![16;299];let mut pool=Pool::with_checkpoints(&[a.clone()],Objective::Launch,true);
        let parent=pool.members[0].clone();
        // Synthetic host records: fill one measured position cell atF240.
        let mut good=a.clone();good[80]^=4;let mut other=a.clone();other[81]^=8;let mut stalled=a.clone();stalled[82]^=16;
        let mut fast=score(44.0);fast.value=-100.0;
        for (raw,x) in [(&good,83.0),(&other,82.0)] {
            assert!(matches!(pool.admit_route_at(&parent,raw.clone(),None,at_checkpoint(240,x),uncertified(raw),2,Some(240)),Admission::Added {..}));
        }
        assert_eq!(pool.admit_route_at(&parent,stalled.clone(),Some(fast),at_checkpoint(240,80.0),uncertified(&stalled),2,Some(240)),Admission::BelowFloor);
        assert!(pool.route_members.iter().all(|m|m.member.score.is_none()));
    }
    #[test] fn reserve_only_parent_lineage_and_own_certificate_survive_pace_eviction() {
        let a=vec![16;299];let mut pool=Pool::new(&[a.clone()],Objective::Launch);
        let protected=pool.members[0].clone();let route=Score::from_window(Objective::Fan,score(44.0).window,90,90).unwrap();
        let mut b=a.clone();b[65]^=4;
        assert!(matches!(pool.admit_route(&protected,b.clone(),None,route,uncertified(&b),64),Admission::Added {id:1,..}));
        let reserve_parent=pool.route_members[0].member.clone();let mut c=b.clone();c[80]^=8;
        assert!(matches!(pool.admit(&reserve_parent,c.clone(),score(44.0),uncertified(&c),1),Admission::Added {id:2,..}));
        assert_eq!(pool.members[1].ancestor,Some(1));assert_eq!(pool.members[1].root,0);
        assert_eq!(pool.members[1].generation,2);assert_eq!(pool.route_members[0].member.masks,b);
        let mut d=c.clone();d[81]^=16;
        assert!(matches!(pool.admit(&protected,d.clone(),score(40.0),uncertified(&d),1),Admission::Added {id:3,evicted:Some(2),..}));
        assert_eq!(pool.route_members[0].member.id,1);assert!(pool.route_members[0].member.score.is_none());
        let mut neutral=b.clone();neutral[65]^=32;
        assert!(!pool.route_members[0].member.neutral_eject_certificate.as_ref().unwrap().rejects_neutral_only(&b,&neutral));
    }
    #[test] fn equal_route_flag_includes_the_equal_member_being_replaced() {
        let a=vec![16;299];let mut pool=Pool::new(&[a.clone()],Objective::Launch);
        let parent=pool.members[0].clone();let route=Score::from_window(Objective::Fan,score(44.0).window,90,90).unwrap();
        let mut better=route;better.value-=1.0;
        let mut b=a.clone();b[70]^=4;let mut c=a.clone();c[71]^=4;let mut d=a.clone();d[72]^=4;
        assert!(matches!(pool.admit_route(&parent,b.clone(),None,better,uncertified(&b),64),Admission::Added {..}));
        assert!(matches!(pool.admit_route(&parent,c.clone(),None,route,uncertified(&c),64),Admission::Added {id:2,..}));
        assert_eq!(pool.admit_route(&parent,d.clone(),None,route,uncertified(&d),64),
            Admission::Added {id:3,evicted:Some(2),equal_count:true});
    }
    #[test] fn admitted_child_does_not_inherit_ancestor_certificate_or_change_raw_cutoff() {
        let a=vec![16;80];let mut p=Pool::new(&[a.clone()],Objective::Finish);
        let mut builder=CertificateBuilder::new(&a).unwrap();
        builder.observe(Invocation {input_index:65,requested_mask:16,input_allowed_before:true,
            iteration_after:66,port_abort:false,before:PreInputState {dad_ejected:true,
                child_ejected:true,..PreInputState::default()}});
        p.members[0].neutral_eject_certificate=Some(builder.seal());
        let parent=p.members[0].clone();let mut inert=a.clone();inert[65]^=32;
        assert!(parent.neutral_eject_certificate.as_ref().unwrap().rejects_neutral_only(&a,&inert));
        let mut b=inert.clone();b[66]^=8;
        assert!(!parent.neutral_eject_certificate.as_ref().unwrap().rejects_neutral_only(&a,&b));
        assert_eq!(first_change(&a,&b),Some(65));
        assert!(matches!(p.admit(&parent,b.clone(),score(90.0),uncertified(&b),1),Admission::Added {..}));
        let child=&p.members[1];let mut descendant=b.clone();descendant[65]^=64;
        assert_eq!(child.masks,b);
        assert!(!child.neutral_eject_certificate.as_ref().unwrap().rejects_neutral_only(&b,&descendant));
        assert_eq!(p.members[0].masks,a);
    }
    #[test] fn suffix_edits_keep_the_original_sixty_and_exact_full_plan() {
        for len in [62,139,262,286,299] {for seed in 0..128 {
            let base:Vec<u8>=(0..len).map(|i|(i*37+17) as u8).collect();
            if let Some((m,changed))=proposal(&base,&mut Rng(seed),239,24).unwrap() {
                assert_eq!(&m[..ANCHOR],&base[..ANCHOR]);assert!((239..=299).contains(&m.len()));
                assert_eq!(first_change(&base,&m),Some(changed));assert!(changed>=ANCHOR&&changed<m.len());
            }
        }}
    }
    #[test] fn shorter_prefix_only_has_no_post_change_score_request() {
        let base=vec![17;139];let child=&base[..100];assert_eq!(first_change(&base,child),Some(100));
        assert_eq!(first_change(&base,child).filter(|&i|i<child.len()),None);
    }
    #[test] fn pool_serialization_retains_literal_and_exact_distance_bits() {
        let a=vec![17;289];let mut p=Pool::new(&[a.clone()],Objective::Finish);p.members[0].score=Some(score(52.80786306867667));
        let v=p.members[0].value();assert_eq!(super::super::sim::parse_tas(v["tas"].as_str().unwrap()).unwrap(),a);
        assert_eq!(v["frame"],289);assert_eq!(v["best_window"]["distance_f64_bits"],"404a67680e99ce66");
        assert_eq!(v["best_window"]["finish_claim"],false);
    }
    #[test] fn route_ranking_keeps_true_distance_metadata_and_rejects_another_focus() {
        let mut window=score(44.0).window;
        window.focus_positions=[[102.976,74.496];WINDOW];
        window.focus_position=window.focus_positions[WINDOW-1];
        let ranked=Score::from_window(Objective::Fan,window,180,180).unwrap();
        assert!(ranked.route.is_some());
        let exported=ranked.value();
        assert_eq!(exported["worst_finish_focus_distance_m"],44.0);
        assert_eq!(exported["distance_f64_bits"],format!("{:016x}",44.0_f64.to_bits()));
        assert_eq!(exported["ranking_f64_bits"],format!("{:016x}",ranked.value.to_bits()));
        assert_eq!(exported["objective"],"fan");assert_eq!(exported["finish_claim"],false);
        window.focus_handle=2;
        assert!(Score::from_window(Objective::Fan,window,180,180).is_none());
        // The unchanged default still uses the exact old distance even if aliases differ.
        assert_eq!(Score::from_window(Objective::Finish,window,180,180).unwrap().value.to_bits(),44.0_f64.to_bits());
    }
    #[test] fn launch_uses_actual_six_velocities_and_frames_and_does_not_admit_eof_fitness() {
        let mut window=score(44.0).window;
        window.focus_positions=[[29.0,69.0];WINDOW];
        window.chest_velocities=[[70.0,17.0];WINDOW];
        window.world_frames=std::array::from_fn(|i|178+i as u32);
        let ranked=Score::from_window(Objective::Launch,window,183,183).unwrap();
        assert!(ranked.value<0.0);assert!(ranked.launch.is_some());
        assert_eq!(ranked.value()["worst_finish_focus_distance_m"],44.0);
        assert_eq!(ranked.value()["finish_claim"],false);
        window.focus_handle=2;
        assert!(Score::from_window(Objective::Launch,window,183,183).is_none());
        window.focus_handle=1;window.world_frames=std::array::from_fn(|i|294+i as u32);
        assert!(Score::from_window(Objective::Launch,window,299,299).is_none());
        // Late fitness rejection cannot suppress the separate observed-finish path.
        assert_eq!(Score::from_window(Objective::Finish,window,299,299).unwrap().value.to_bits(),44.0_f64.to_bits());
    }
    #[test] fn terminal_admission_is_launch_only_and_requires_the_scored_body_identity() {
        let terminal=survival_policy::TerminalEvidence {evaluated_to_horizon:true,actual_world_frames:299,
            dad_alive:true,chest_live:true,chest_in_world:true,actual_focus_is_primary_chest:true,
            port_abort:false,lag_stopped:false};
        let observed=Some(score(44.0));
        assert!(earned_score(Objective::Launch,observed,terminal,1).is_some());
        assert!(earned_score(Objective::Launch,observed,terminal,2).is_none());
        let stopped=survival_policy::TerminalEvidence {evaluated_to_horizon:false,..terminal};
        assert!(earned_score(Objective::Launch,observed,stopped,1).is_none());
        for objective in [Objective::Finish,Objective::Fan] {
            assert_eq!(earned_score(objective,observed,stopped,2).unwrap().value.to_bits(),44.0_f64.to_bits());
        }
        // A freshly repaired child's own terminal evidence controls admission.
        // No parent survival certificate is an argument to this function.
        assert!(earned_score(Objective::Launch,observed,terminal,1).is_some());
    }
    #[test] fn actual_fatal_launch_is_diagnostic_and_living_299_replay_earns_its_window() {
        std::thread::Builder::new().stack_size(64<<20).spawn(|| {
            super::super::sim::set_lag_ms(0.0);
            let _hang=super::super::sim::HangOverride::new(0);
            // Native member2035: F89 rank c054dbd64bf74a39, death90.
            // This regression literal has not been matched to original Flash.
            let fatal=super::super::sim::parse_tas(include_str!(concat!(env!("CARGO_MANIFEST_DIR"),
                "/../../reference/tas/obstacle-dad-native-fatal-f90.tas"))).unwrap();
            let living=super::super::sim::parse_tas(include_str!(concat!(env!("CARGO_MANIFEST_DIR"),
                "/../../reference/tas/obstacle-dad-burst-f178.tas"))).unwrap();
            for (i,masks) in [fatal,living].into_iter().enumerate() {for route_checkpoints in [false,true] {
                let stamp=std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos();
                let dir=std::env::temp_dir().join(format!("hw-launch-survival-{}-{stamp}-{i}",std::process::id()));
                std::fs::create_dir(&dir).unwrap();
                let cfg=Config {level:45,character:3,bases:vec![masks.clone()],run_dir:dir.clone(),
                    threads:1,hours:1.0,seed:1,window:239,extra:299,pool_size:1,lag_ms:0.0,lag_frames:3,
                    objective:Objective::Launch,diversity_pool:64,route_checkpoints};
                let start=Instant::now();
                let shared=Shared {pool:Mutex::new(Pool::with_checkpoints(&cfg.bases,cfg.objective,route_checkpoints)),stop:AtomicBool::new(false),
                    error:Mutex::new(None),winner:Mutex::new(None),logs:Mutex::new(Logs::new(&dir).unwrap()),
                    counts:Counts::default(),sites:Mutex::new([0;12]),best:Mutex::new(None),
                    maximum_current_pairs:Mutex::new(0),start,deadline:start+Duration::from_secs(60),objective:cfg.objective};
                let outcome=replay(&cfg,&shared,&masks,ANCHOR,0,true,"protected_initial",None).unwrap();
                let trial:Value=serde_json::from_str(&std::fs::read_to_string(dir.join("trials.jsonl")).unwrap()).unwrap();
                if i==0 {
                    assert!(outcome.score.is_none());assert_eq!(trial["actual_world_frames"],90);
                    assert!(outcome.route.is_none());assert!(trial["route_window"].is_null());
                    assert_eq!(trial["best_observed_window"]["frame"],89);
                    assert_eq!(trial["best_observed_window"]["ranking_f64_bits"],"c054dbd64bf74a39");
                    assert!(trial["best_window"].is_null());assert_eq!(trial["launch_terminal_survival"]["eligible"],false);
                    assert!(shared.pool.lock().unwrap().members[0].protected);
                    assert!(shared.pool.lock().unwrap().members[0].score.is_none());
                    assert!(outcome.checkpoints.iter().all(Option::is_none));
                } else {
                    assert_eq!(outcome.score.unwrap().value.to_bits(),0xc02af7da8a6d7432);
                    assert_eq!(trial["actual_world_frames"],299);assert_eq!(trial["best_window"]["frame"],183);
                    assert_eq!(trial["best_window"],trial["best_observed_window"]);
                    assert_eq!(trial["launch_terminal_survival"]["eligible"],true);
                    if route_checkpoints {
                        assert!(outcome.route.is_none());assert!(trial["route_window"].is_null());
                        for (index,frame) in checkpoint_policy::FRAMES.into_iter().enumerate() {
                            let actual=outcome.checkpoints[index].unwrap();assert_eq!(actual.frame,frame);
                            assert_eq!(actual.window.world_frames,std::array::from_fn(|j|frame-5+j as u32));
                            let logged=&trial["route_checkpoint_windows"][index];
                            assert_eq!(logged["checkpoint"],frame);assert_eq!(logged["earned_window"],logged["observed_window"]);
                        }
                    } else {assert!(outcome.route.is_some());assert_eq!(trial["route_window"],trial["best_observed_route_window"]);}
                    assert_eq!(trial["actual_route_milestones"].as_array().unwrap().len(),5);
                }
                assert!(shared.winner.lock().unwrap().is_none());
                drop(shared);std::fs::remove_dir_all(&dir).unwrap();
            }}
        }).unwrap().join().unwrap();
    }
    #[test] fn actual_focus_and_primary_chest_capture_ignore_the_active_child_alias() {
        std::thread::Builder::new().stack_size(64<<20).spawn(|| {
            let mut s=Sim::new(45,3).unwrap();
            let focus=s.ctx.level.focus;let dad=s.ctx.character.primary.bodies[CHEST];
            let before_focus=body_state(&s,focus);let before_chest=body_state(&s,dad);
            assert!(before_focus.live&&before_chest.live);
            // This is a host accessor test, not a claimed simulated damage event.
            s.ctx.character.child_active=hw_core::Flag32::TRUE;
            s.ctx.character.child.dead=hw_core::Flag32::TRUE;
            assert_eq!(s.ctx.character.bodies[CHEST],s.ctx.character.child.bodies[CHEST]);
            let after_focus=body_state(&s,s.ctx.level.focus);
            let after_chest=body_state(&s,s.ctx.character.primary.bodies[CHEST]);
            assert_eq!(after_focus.handle,focus);assert_eq!(after_chest.handle,dad);
            assert!(after_focus.live&&after_chest.live);
            assert_eq!(after_focus.position.map(f64::to_bits),before_focus.position.map(f64::to_bits));
            assert_eq!(after_chest.position.map(f64::to_bits),before_chest.position.map(f64::to_bits));
            assert!(!s.ctx.character.primary.dead.get());
            assert!(!body_state(&s,NONE).live);assert!(!body_state(&s,MAX_BODIES as u32).live);
        }).unwrap().join().unwrap();
    }

    fn fresh_finite_fixture(masks:&[u8]) -> Sim {
        let mut s=Sim::new(45,3).unwrap();
        s.replay=ReplayState::new(masks.len() as u32);s.masks=masks.to_vec();s
    }
    fn five_observers(s:&Sim) -> [String;5] {
        [hw_core::diagnostic::broadphase_string(&s.ctx),
            hw_core::physics_observer::physics_string(&s.ctx),
            hw_core::game_observer::game_string(&s.ctx),
            hw_core::links_observer::links_string(&s.ctx),
            hw_core::mechanics_observer::mechanics_string(&s.ctx)]
    }
    #[test]
    #[ignore="real299-frame exact five-observer A/B replay; run explicitly with --ignored --test-threads=1"]
    fn native_certified_eject_edits_match_five_observers_and_first_ejection_diverges() {
        std::thread::Builder::new().name("progress-eject-native-regression".into()).stack_size(64<<20)
            .spawn(|| {
                super::super::sim::set_lag_ms(0.0);
                let _hang=super::super::sim::HangOverride::new(0);
                // Exact full299 nonwinning original-Flash MATCH fixture, SHA256
                // 770e410f2d831532335dcad5bbc4bf0b30de38fde235686a0d3852338ade5d2c.
                // The f178 label describes its living-Dad burst, not a finish.
                let bits=include_str!(concat!(env!("CARGO_MANIFEST_DIR"),
                    "/../../reference/tas/obstacle-dad-burst-f178.tas"));
                assert_eq!(bits.len(),299*8);assert!(bits.bytes().all(|b|b==b'0'||b==b'1'));
                let original=super::super::sim::parse_tas(bits).unwrap();
                assert_eq!(tas_text(&original),bits);
                let mut certificate=CertificateBuilder::new(&original).unwrap();
                let mut observed=fresh_finite_fixture(&original);
                for frame in 1..=299 {
                    let f=certified_step(&mut observed,&mut certificate);
                    assert_eq!(f.frame,frame);assert_eq!(observed.replay.iteration,frame);
                    assert!(!f.port_abort&&!f.finished&&!observed.ctx.character.primary.dead.get());
                }
                assert_eq!(observed.masks,original);assert_eq!(observed.replay.finish_frame,0);
                let certificate=certificate.seal();
                // Flip only bits accepted by this actual parent's certificate.
                // All active arrows/Space and full raw length remain unchanged.
                let mut neutral=original.clone();let mut changed_bits=0;
                for i in ANCHOR..original.len() {for bit in [32,64,128] {
                    let mut one=original.clone();one[i]^=bit;
                    if certificate.rejects_neutral_only(&original,&one) {neutral[i]^=bit;changed_bits+=1;}
                }}
                assert!(changed_bits>0);assert_ne!(neutral,original);
                assert_eq!(&neutral[..ANCHOR],&original[..ANCHOR]);assert_eq!(neutral.len(),original.len());
                assert!(certificate.rejects_neutral_only(&original,&neutral));
                let mut a=fresh_finite_fixture(&original);let mut b=fresh_finite_fixture(&neutral);
                for frame in 1..=299 {
                    let af=a.step(0);let bf=b.step(0);
                    assert_eq!((af.frame,af.site,af.port_abort,af.finished),(bf.frame,bf.site,bf.port_abort,bf.finished));
                    assert!(!af.port_abort&&!af.finished);
                    assert_eq!(hw_core::diagnostic::replay_string(&a.ctx,&a.replay),
                        hw_core::diagnostic::replay_string(&b.ctx,&b.replay),"replay state at frame {frame}");
                    for (kind,(left,right)) in ["broadphase","physics","game","links","mechanics"]
                        .into_iter().zip(five_observers(&a).into_iter().zip(five_observers(&b))) {
                        assert!(left==right,"{kind} observer differs at frame {frame}");
                    }
                }
                assert_eq!(a.masks,original);assert_eq!(b.masks,neutral);
                assert_eq!(a.replay.finish_frame,0);assert_eq!(b.replay.finish_frame,0);
                // Old586 first requests Dad ejection at raw74 with Shift+Z;
                // leave its Ctrl and all other bits intact, clearing both Dad requests.
                let first_eject=original.iter().position(|&m|m&160!=0).unwrap();
                assert_eq!(first_eject,74);assert_eq!(original[first_eject],232);
                let mut meaningful=original.clone();meaningful[first_eject]&=!160;
                assert_eq!(meaningful[first_eject],72);
                assert_eq!(first_change(&original,&meaningful),Some(first_eject));
                assert!(!certificate.rejects_neutral_only(&original,&meaningful));
                let mut a=fresh_finite_fixture(&original);let mut b=fresh_finite_fixture(&meaningful);
                for _ in 0..first_eject {
                    a.step(0);b.step(0);
                    assert_eq!(hw_core::game_observer::game_string(&a.ctx),hw_core::game_observer::game_string(&b.ctx));
                }
                assert!(!a.ctx.character.primary.bicycle.ejected.get());
                assert!(!b.ctx.character.primary.bicycle.ejected.get());
                a.step(0);b.step(0);
                assert!(a.ctx.character.primary.bicycle.ejected.get());
                assert!(!b.ctx.character.primary.bicycle.ejected.get());
                assert_ne!(hw_core::game_observer::game_string(&a.ctx),hw_core::game_observer::game_string(&b.ctx));
                assert_eq!(a.masks,original);assert_eq!(b.masks,meaningful);
            }).unwrap().join().expect("native eject regression thread");
    }
}
