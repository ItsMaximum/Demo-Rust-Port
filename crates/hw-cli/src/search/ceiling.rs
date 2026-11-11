//! Fresh finite Dad/Obstacle ceiling search. Lab hooks only observe: no injection,
//! cached simulation, state repair, early launch return, or child/body-max fitness.
#[path = "ceiling_policy.rs"]
mod policy;
#[path = "endpoint_policy.rs"]
mod endpoint;
use endpoint::{EndpointMoment, EndpointTarget, STRATA};
use policy::{Evidence, LaunchMoment, Observation, PressureCounts, PressureMoment, Primary, Score, MAX_FRAME, SCORE_VERSION};
use super::explore::confirm_any_finish;
use super::sim::{finish_region_distance, tas_text, Sim};
use hw_core::b2::{body, state::{MAX_BODIES, MAX_SHAPES}};
use hw_core::game::{character::CHEST, session::ReplayState};
use hw_core::{lab, H, NONE};
use serde_json::{json, Value};
use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

pub struct Config {
    pub level:u32, pub character:u32, pub bases:Vec<Vec<u8>>, pub run_dir:PathBuf,
    pub threads:usize, pub hours:f64, pub seed:u64, pub window:usize,
    pub edit_from:usize, pub edit_to:usize,
    pub pool_size:usize, pub lag_ms:f64, pub lag_frames:u32,
    pub launch_pool_size:usize, pub launch_min_frame:u32, pub launch_min_speed:f64,
    pub pressure_pool_size:usize, pub pressure_min_pairs:i32, pub pressure_min_frame:u32, pub pressure_max_frame:u32,
    pub endpoint_pool_size:usize, pub endpoint_min_frame:u32,
    pub enumeration:Option<EnumerationConfig>,
}
/// Host-only finite Cartesian enumeration. Low5 preserves each selected cell's
/// own high3; the legacy two-cell mode still replaces complete bytes. End is exclusive.
#[derive(Clone,Debug,PartialEq,Eq)]
pub struct EnumerationConfig {pub cells:Vec<usize>,pub low5:bool,pub low5_values:Vec<u8>,pub start:u32,pub end:u32}
impl EnumerationConfig {
    fn radix(&self)->u32 {if self.low5 {self.low5_values.len() as u32}else {256}}
    fn cardinality(&self)->u32 {self.radix().pow(self.cells.len() as u32)}
    fn validate(&self,cfg:&Config)->Result<(),String> {
        if self.low5 {
            if !(1..=32).contains(&self.low5_values.len()) || self.low5_values.iter().enumerate()
                .any(|(i,&v)|v>31 || self.low5_values[..i].contains(&v)) {
                return Err("low5 enumeration requires1..32 unique low5 values in0..31".into());}
        }else if !self.low5_values.is_empty() {return Err("low5 values require low5 enumeration".into());}
        let count_ok=if self.low5 {(1..=5).contains(&self.cells.len())}else {self.cells.len()==2};
        if !count_ok || cfg.bases.len()!=1
            || self.cells.iter().enumerate().any(|(i,c)|self.cells[..i].contains(c))
            || self.cells.iter().any(|&i|i>=MAX_FRAME as usize || i<cfg.edit_from || i>=cfg.edit_to)
            || self.start>=self.end || self.end>self.cardinality() {
            return Err("enumeration requires one full299 root, distinct cells in edit bounds, and a nonempty bounded ID range; legacy has two cells, low5 has1..5".into());}
        if self.low5 && cfg.threads>17 {return Err("low5 enumeration permits at most17 workers plus the coordinator (18 threads)".into());}
        Ok(())
    }
    fn baseline_id(&self,base:&[u8])->Option<u32> {self.cells.iter().try_fold(0,|id,&cell|{
        let digit=if self.low5 {self.low5_values.iter().position(|&v|v==(base[cell]&31))? as u32}
            else {u32::from(base[cell])};Some(id*self.radix()+digit)})}
    fn value(&self,base:&[u8])->Value {let baseline=self.baseline_id(base);
        if !self.low5 {return json!({"cells":self.cells,"start":self.start,"end_exclusive":self.end,"total":self.end-self.start,
            "case_id":"(mask_a << 8) | mask_b","baseline_case_id":baseline,
            "baseline_in_shard":baseline.is_some_and(|id|(self.start..self.end).contains(&id)),"one_constant_full299_root":true,
            "fresh_replay":true,"random_proposals":false,"pool_parent_selection":false,
            "events_scope":"existing selective summarized predicate, with case identity; missing event never proves absence",
            "original_player_verified":false});}
        let case_id=if self.low5_values.iter().copied().eq(0..32) {
            "base32 low5 digits in declared cell order; last cell fastest".to_string()
        }else {format!("base{} low5 alphabet indices in declared cell order; last cell fastest",self.radix())};
        json!({"cells":self.cells,"start":self.start,"end_exclusive":self.end,"total":self.end-self.start,
            "domain_cardinality":self.cardinality(),"low5_choices":self.low5_values,
            "case_id":case_id,
            "baseline_case_id":baseline,"baseline_in_shard":baseline.is_some_and(|id|(self.start..self.end).contains(&id)),
            "each_edited_cell_high3_exact":true,"all_outside_cells_exact":true,
            "one_constant_full299_root":true,"fresh_replay":true,"random_proposals":false,"pool_parent_selection":false,
            "events_scope":"existing selective summarized predicate, with case identity; missing event never proves absence",
            "original_player_verified":false,"native_only":true})
    }
}
#[derive(Clone,Debug)]
struct EnumerationCase {id:u32,cells:Vec<usize>,masks:Vec<u8>,low5:bool,changed:Option<usize>}
impl EnumerationCase {
    fn build(spec:&EnumerationConfig,id:u32,base:&[u8])->(Self,Vec<u8>) {
        debug_assert!((spec.start..spec.end).contains(&id));debug_assert_eq!(base.len(),MAX_FRAME as usize);
        let mut digits=vec![0u8;spec.cells.len()];let mut remaining=id;
        for digit in digits.iter_mut().rev() {*digit=(remaining%spec.radix()) as u8;remaining/=spec.radix();}
        let mut raw=base.to_vec();let masks=spec.cells.iter().zip(digits).map(|(&cell,digit)|{
            let mask=if spec.low5 {(base[cell]&0xe0)|spec.low5_values[digit as usize]}else {digit};raw[cell]=mask;mask}).collect();
        (Self {id,cells:spec.cells.clone(),masks,low5:spec.low5,changed:first_change(base,&raw)},raw)
    }
    fn value(&self)->Value {
        if !self.low5 {return json!({"case_id":self.id,"raw_cells":self.cells,"requested_masks":self.masks,
            "earliest_changed_index":self.changed,"is_unchanged_baseline":self.changed.is_none(),
            "planned_entries":MAX_FRAME,"outside_two_cells_exact":true});}
        json!({"case_id":self.id,"raw_cells":self.cells,"requested_masks":self.masks,
            "low5_masks":self.masks.iter().map(|m|m&31).collect::<Vec<_>>(),
            "earliest_changed_index":self.changed,"is_unchanged_baseline":self.changed.is_none(),
            "planned_entries":MAX_FRAME,"each_edited_cell_high3_exact":true,"all_outside_cells_exact":true,
            "native_only":true,"original_player_verified":false})
    }
}
// Claims are bounded, unique and independent of completion/log line order.
struct EnumerationProgress {spec:EnumerationConfig,next:AtomicU64,claimed:AtomicU64,returned:AtomicU64,
    outcomes:[AtomicU64;5]}
impl EnumerationProgress {
    fn new(spec:EnumerationConfig)->Self {let start=spec.start as u64;Self {spec,next:AtomicU64::new(start),
        claimed:AtomicU64::new(0),returned:AtomicU64::new(0),outcomes:Default::default()}}
    fn claim(&self)->Option<u32> {let id=self.next.fetch_update(Ordering::Relaxed,Ordering::Relaxed,
        |id|(id<self.spec.end as u64).then_some(id+1)).ok()?;
        self.claimed.fetch_add(1,Ordering::Relaxed);Some(id as u32)}
    fn record(&self,outcome:EnumerationOutcome) {self.outcomes[outcome as usize].fetch_add(1,Ordering::Relaxed);
        self.returned.fetch_add(1,Ordering::Release);}
    fn done(&self)->bool {self.returned.load(Ordering::Acquire)==u64::from(self.spec.end-self.spec.start)}
    fn value(&self)->Value {let load=|n:&AtomicU64|n.load(Ordering::Relaxed);let claimed=load(&self.claimed);
        let returned=load(&self.returned);let total=u64::from(self.spec.end-self.spec.start);
        json!({"start":self.spec.start,"end_exclusive":self.spec.end,"total":total,"claimed":claimed,
            "returned":returned,"unclaimed":total.saturating_sub(claimed),"claimed_without_returned_outcome":claimed.saturating_sub(returned),
            "outcomes":self.outcomes.iter().map(load).collect::<Vec<_>>(),
            "outcome_order":["horizon299","intrinsic_game_dead","observed_finish","guarded","interrupted"],
            "all_case_outcomes_present":returned==total,"counter_snapshot_may_be_concurrent":true,"exhaustive_original_negative_claimed":false,
            "independent_completed_physics_steps_claimed":false})}
}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
enum EnumerationOutcome {Horizon299=0,IntrinsicGameDead=1,ObservedFinish=2,Guarded=3,Interrupted=4}
impl EnumerationOutcome {fn name(self)->&'static str {match self {
    Self::Horizon299=>"horizon299",Self::IntrinsicGameDead=>"intrinsic_game_dead",
    Self::ObservedFinish=>"observed_finish",Self::Guarded=>"guarded",Self::Interrupted=>"interrupted"}}}
fn enumeration_outcome(finish:u32,reason:&str,frames:u32,consumed:u32,guarded:bool)->EnumerationOutcome {
    // Observation stays ahead of fatal/guard classification; confirmation is separate.
    if (1..=MAX_FRAME).contains(&finish) {EnumerationOutcome::ObservedFinish}
    else if guarded {EnumerationOutcome::Guarded}
    else if reason=="game_dead" {EnumerationOutcome::IntrinsicGameDead}
    else if frames==MAX_FRAME && consumed==MAX_FRAME {EnumerationOutcome::Horizon299}
    else {EnumerationOutcome::Interrupted}
}
impl Config {
    pub fn validate(&self)->Result<(),String> {
        if (self.level,self.character)!=(45,3) {return Err("ceiling requires level45/character3".into());}
        if !self.hours.is_finite() || self.hours<0.0 || self.threads>18 || (self.threads==0 && self.hours!=0.0)
            || Duration::try_from_secs_f64(self.hours*3600.0).is_err() {
            return Err("ceiling requires finite nonnegative hours, workers1..18, or workers0/hours0".into());}
        if !self.lag_ms.is_finite() || self.lag_ms<0.0 || self.lag_ms*1000.0>u32::MAX as f64 || self.lag_frames==0 {
            return Err("ceiling requires valid lag-ms and lag-frames>=1".into());}
        if self.pool_size==0 || !(1..=MAX_FRAME as usize).contains(&self.window) || self.bases.is_empty()
            || self.bases.iter().any(|b|b.len()!=MAX_FRAME as usize) {
            return Err("ceiling requires pool>0, window1..299, at least one exact full299 base".into());}
        if self.edit_from>=self.edit_to || self.edit_to>MAX_FRAME as usize {
            return Err("ceiling requires 0<=edit-from<edit-to<=299 (exclusive edit-to)".into());}
        if self.launch_pool_size>1024 || !(1..=MAX_FRAME).contains(&self.launch_min_frame)
            || !self.launch_min_speed.is_finite() || self.launch_min_speed<0.0 {
            return Err("ceiling requires launch-pool0..1024, launch-min-frame1..299 and finite launch-min-speed>=0".into());}
        if (self.pressure_pool_size!=0 && (self.pressure_pool_size<4 || self.pressure_pool_size>1024 || self.pressure_pool_size%4!=0))
            || !(1..=4096).contains(&self.pressure_min_pairs) || !(1..=MAX_FRAME).contains(&self.pressure_min_frame)
            || !(self.pressure_min_frame..=MAX_FRAME).contains(&self.pressure_max_frame) {
            return Err("ceiling requires pressure-pool0 or multiple-of4 in4..1024, pressure-min-pairs1..4096 and inclusive frame range1..299".into());}
        if (self.endpoint_pool_size!=0 && (self.endpoint_pool_size<STRATA || self.endpoint_pool_size>1024 || self.endpoint_pool_size%STRATA!=0))
            || !(1..=MAX_FRAME).contains(&self.endpoint_min_frame) {
            return Err("ceiling requires endpoint-pool0 or multiple-of4 in4..1024 and endpoint-min-frame1..299".into());}
        if let Some(spec)=self.enumeration.as_ref() {spec.validate(self)?;}
        Ok(())
    }
}
#[derive(Clone)]
struct Member {
    id:u64, ancestor:Option<u64>, root:u64, generation:u64, source_ordinals:Vec<usize>,
    protected:bool, masks:Vec<u8>, score:Option<Score>, visits:u64, trial_id:Option<u64>,
}
impl Member {
    fn value(&self)->Value {json!({"schema":1,"score_version":SCORE_VERSION,"id":self.id,
        "ancestor":self.ancestor,"root":self.root,"generation":self.generation,"source_ordinals":self.source_ordinals,
        "protected":self.protected,"visits":self.visits,"frame":self.masks.len(),"tas":tas_text(&self.masks),
        "best_observation":self.score.map(Score::value),"trial_id":self.trial_id})}
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ParentLane {Initial,Legacy,ProtectedRoot,LaunchArchive,ArchiveEmptyLegacy,Pressure,PressureEmptyLegacy,Endpoint,EndpointEmptyLegacy,Enumeration,Low5Enumeration}
impl ParentLane {
    fn name(self)->&'static str {match self {
        Self::Initial=>"initial_base",Self::Legacy=>"legacy_pool",Self::ProtectedRoot=>"protected_root_uniform",
        Self::LaunchArchive=>"launch_repair_uniform",Self::ArchiveEmptyLegacy=>"archive_empty_legacy_pool",
        Self::Pressure=>"pair_pressure_uniform",Self::PressureEmptyLegacy=>"pressure_empty_legacy_pool",
        Self::Endpoint=>"endpoint_stratum_uniform",Self::EndpointEmptyLegacy=>"endpoint_empty_legacy_pool",
        Self::Enumeration=>"constant_root_two_cell_enumeration",Self::Low5Enumeration=>"constant_root_low5_cartesian_enumeration"}}
}
#[derive(Clone)]
struct ArchiveEntry {member:Member,witness:LaunchMoment,initial_copy:bool,parent_lane:ParentLane}
impl ArchiveEntry {
    fn value(&self)->Value {let mut value=self.member.value();
        value["selection_class"]="primary_launch_repair".into();
        value["launch_moment"]=self.witness.value();value["initial_copy"]=self.initial_copy.into();
        value["parent_selection_lane"]=self.parent_lane.name().into();value["fitness"]=Value::Null;
        value["survival_or_finish_claimed"]=false.into();value}
}
#[derive(Clone)]
struct PressureEntry {member:Member,witness:PressureMoment,initial_copy:bool,parent_lane:ParentLane}
impl PressureEntry {
    fn value(&self)->Value {let mut v=self.member.value();v["selection_class"]="global_pair_pressure_repair".into();
        v["pressure_moment"]=self.witness.value();v["initial_copy"]=self.initial_copy.into();
        v["parent_selection_lane"]=self.parent_lane.name().into();v["fitness"]=Value::Null;
        v["survival_or_finish_claimed"]=false.into();v["Dad_caused_pair_growth_claimed"]=false.into();v}
}
#[derive(Clone)]
struct EndpointEntry {member:Member,witness:EndpointMoment,initial_copy:bool,parent_lane:ParentLane}
impl EndpointEntry {
    fn value(&self)->Value {let mut v=self.member.value();v["selection_class"]="recorded_primary_endpoint_dna".into();
        v["endpoint_moment"]=self.witness.value();v["endpoint_stratum"]=self.witness.bucket.into();
        v["initial_copy"]=self.initial_copy.into();v["parent_selection_lane"]=self.parent_lane.name().into();
        v["fitness"]=Value::Null;v["survival_or_finish_claimed"]=false.into();v}
}
#[derive(Clone)]
struct Pool {members:Vec<Member>,launches:Vec<ArchiveEntry>,pressures:[Vec<PressureEntry>;4],endpoints:[Vec<EndpointEntry>;STRATA],next_id:u64}
#[derive(Debug,PartialEq,Eq)]
enum Admission {Duplicate,BelowFloor,Added {id:u64,evicted:Option<u64>,equal_score:bool}}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
enum ArchiveAdmission {Disabled,InvalidLiteral,Duplicate,Added {id:u64,evicted:Option<u64>}}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
enum EndpointAdmission {Disabled,InvalidLiteral,Duplicate,BelowFloor,Added {id:u64,evicted:Option<u64>}}
impl Pool {
    fn new(bases:&[Vec<u8>])->Self {
        let mut members:Vec<Member>=Vec::new();
        for (ordinal,masks) in bases.iter().enumerate() {
            if let Some(m)=members.iter_mut().find(|m|m.masks==*masks) {m.source_ordinals.push(ordinal);}
            else {let id=members.len() as u64;members.push(Member {id,ancestor:None,root:id,generation:0,
                source_ordinals:vec![ordinal],protected:true,masks:masks.clone(),score:None,visits:0,trial_id:None});}
        }
        Self {next_id:members.len() as u64,members,launches:Vec::new(),pressures:std::array::from_fn(|_|Vec::new()),
            endpoints:std::array::from_fn(|_|Vec::new())}
    }
    fn pick(&mut self,rng:&mut Rng)->Member {
        let a=rng.below(self.members.len());let b=if rng.below(4)==0 {a} else {rng.below(self.members.len())};
        let compare=|a:&Member,b:&Member| match (a.score,b.score) {
            (Some(a),Some(b))=>a.compare(b),(Some(_),None)=>std::cmp::Ordering::Greater,
            (None,Some(_))=>std::cmp::Ordering::Less,(None,None)=>std::cmp::Ordering::Equal};
        let order=compare(&self.members[b],&self.members[a]);
        let i=if order.is_gt() || (order.is_eq() && self.members[b].visits<self.members[a].visits) {b} else {a};
        self.members[i].visits=self.members[i].visits.saturating_add(1);self.members[i].clone()
    }
    /// With the archive disabled, even the selection RNG stream is unchanged.
    /// The explicit root lane cannot be outcompeted by earlier living peaks.
    fn pick_with_archive(&mut self,rng:&mut Rng,cap:usize)->(Member,ParentLane) {
        if cap==0 {return (self.pick(rng),ParentLane::Legacy);}
        match rng.below(4) {
            0=>{let roots:Vec<usize>=self.members.iter().enumerate().filter_map(|(i,m)|m.protected.then_some(i)).collect();
                let i=roots[rng.below(roots.len())];self.members[i].visits=self.members[i].visits.saturating_add(1);
                (self.members[i].clone(),ParentLane::ProtectedRoot)},
            1 if !self.launches.is_empty()=>{let i=rng.below(self.launches.len());
                self.launches[i].member.visits=self.launches[i].member.visits.saturating_add(1);
                (self.launches[i].member.clone(),ParentLane::LaunchArchive)},
            1=>(self.pick(rng),ParentLane::ArchiveEmptyLegacy),
            _=>(self.pick(rng),ParentLane::Legacy),
        }
    }

    fn pick_with_pressure(&mut self,rng:&mut Rng,launch_cap:usize,pressure_cap:usize)->(Member,ParentLane) {
        // Strict compatibility path: no extra random draw or altered old pick.
        if pressure_cap==0 {return self.pick_with_archive(rng,launch_cap);}
        match rng.below(8) {
            0|1=>{let roots:Vec<usize>=self.members.iter().enumerate().filter_map(|(i,m)|m.protected.then_some(i)).collect();
                let i=roots[rng.below(roots.len())];self.members[i].visits=self.members[i].visits.saturating_add(1);
                (self.members[i].clone(),ParentLane::ProtectedRoot)},
            2|3 if launch_cap>0 && !self.launches.is_empty()=>{let i=rng.below(self.launches.len());
                self.launches[i].member.visits=self.launches[i].member.visits.saturating_add(1);
                (self.launches[i].member.clone(),ParentLane::LaunchArchive)},
            2|3=>(self.pick(rng),ParentLane::ArchiveEmptyLegacy),
            4=>{let count:usize=self.pressures.iter().map(Vec::len).sum();
                if count==0 {return (self.pick(rng),ParentLane::PressureEmptyLegacy);}
                let mut ordinal=rng.below(count);
                for bucket in &mut self.pressures {if ordinal<bucket.len() {
                    let m=&mut bucket[ordinal].member;m.visits=m.visits.saturating_add(1);
                    return (m.clone(),ParentLane::Pressure);}
                    ordinal-=bucket.len();}
                unreachable!("pressure ordinal is bounded by retained count")},
            _=>(self.pick(rng),ParentLane::Legacy),
        }
    }
    fn pressure(&mut self,parent:&Member,masks:Vec<u8>,witness:PressureMoment,cap:usize,
        trial_id:u64,initial:bool,parent_lane:ParentLane)->ArchiveAdmission {
        if cap==0 {return ArchiveAdmission::Disabled;}
        if cap<4 || cap>1024 || cap%4!=0 || witness.bucket>=4 || masks.len()!=MAX_FRAME as usize
            || (initial && masks!=parent.masks) || (!initial && masks==parent.masks) {return ArchiveAdmission::InvalidLiteral;}
        if self.pressures.iter().flatten().any(|e|e.member.masks==masks) {return ArchiveAdmission::Duplicate;}
        let bin=&mut self.pressures[witness.bucket];
        let evicted=if bin.len()>=cap/4 {Some(bin.remove(0).member.id)}else {None};
        let id=self.next_id;self.next_id+=1;
        bin.push(PressureEntry {member:Member {id,ancestor:Some(parent.id),root:parent.root,
            generation:if initial {parent.generation}else {parent.generation.saturating_add(1)},
            source_ordinals:if initial {parent.source_ordinals.clone()}else {vec![]},protected:false,
            masks,score:None,visits:0,trial_id:Some(trial_id)},witness,initial_copy:initial,parent_lane});
        ArchiveAdmission::Added {id,evicted}
    }

    fn pick_with_endpoints(&mut self,rng:&mut Rng,launch_cap:usize,pressure_cap:usize,endpoint_cap:usize)->(Member,ParentLane) {
        // Keep the complete old selector and its RNG stream when endpoint DNA is disabled.
        if endpoint_cap==0 {return self.pick_with_pressure(rng,launch_cap,pressure_cap);}
        let lane=rng.below(8);let endpoint_lane=if pressure_cap>0 {5}else {4};
        match lane {
            0|1=>{let roots:Vec<usize>=self.members.iter().enumerate().filter_map(|(i,m)|m.protected.then_some(i)).collect();
                let i=roots[rng.below(roots.len())];self.members[i].visits=self.members[i].visits.saturating_add(1);
                (self.members[i].clone(),ParentLane::ProtectedRoot)},
            2|3 if launch_cap>0 && !self.launches.is_empty()=>{let i=rng.below(self.launches.len());
                self.launches[i].member.visits=self.launches[i].member.visits.saturating_add(1);
                (self.launches[i].member.clone(),ParentLane::LaunchArchive)},
            2|3=>(self.pick(rng),ParentLane::ArchiveEmptyLegacy),
            4 if pressure_cap>0=>{let count:usize=self.pressures.iter().map(Vec::len).sum();
                if count==0 {return (self.pick(rng),ParentLane::PressureEmptyLegacy);}
                let mut ordinal=rng.below(count);
                for bucket in &mut self.pressures {if ordinal<bucket.len() {
                    let m=&mut bucket[ordinal].member;m.visits=m.visits.saturating_add(1);
                    return (m.clone(),ParentLane::Pressure);}
                    ordinal-=bucket.len();}
                unreachable!("pressure ordinal is bounded by retained count")},
            4|5 if lane==endpoint_lane=>{
                let strata:Vec<usize>=self.endpoints.iter().enumerate().filter_map(|(i,b)|(!b.is_empty()).then_some(i)).collect();
                if strata.is_empty() {return (self.pick(rng),ParentLane::EndpointEmptyLegacy);}
                let bucket=strata[rng.below(strata.len())];let i=rng.below(self.endpoints[bucket].len());
                let m=&mut self.endpoints[bucket][i].member;m.visits=m.visits.saturating_add(1);
                (m.clone(),ParentLane::Endpoint)},
            _=>(self.pick(rng),ParentLane::Legacy),
        }
    }
    /// Separate closest-distance banks: raw identity is deduplicated within each
    /// stratum only. Neither endpoint distance nor later death becomes a Score.
    fn endpoint(&mut self,parent:&Member,masks:Vec<u8>,witness:EndpointMoment,cap:usize,
        trial_id:u64,initial:bool,parent_lane:ParentLane)->EndpointAdmission {
        if cap==0 {return EndpointAdmission::Disabled;}
        if cap<STRATA || cap>1024 || cap%STRATA!=0 || witness.bucket>=STRATA || masks.len()!=MAX_FRAME as usize
            || !witness.target.distance.is_finite() || witness.target.distance<0.0
            || (initial && masks!=parent.masks) || (!initial && masks==parent.masks) {return EndpointAdmission::InvalidLiteral;}
        let bin=&mut self.endpoints[witness.bucket];
        if bin.iter().any(|e|e.member.masks==masks) {return EndpointAdmission::Duplicate;}
        let mut evicted=None;
        if bin.len()>=cap/STRATA {
            let worst=(0..bin.len()).min_by(|&a,&b|bin[a].witness.compare(bin[b].witness)
                .then_with(||bin[b].member.id.cmp(&bin[a].member.id))).unwrap();
            // Equal witnesses retain the earlier member deterministically.
            if !witness.compare(bin[worst].witness).is_gt() {return EndpointAdmission::BelowFloor;}
            evicted=Some(bin.remove(worst).member.id);
        }
        let id=self.next_id;self.next_id+=1;
        bin.push(EndpointEntry {member:Member {id,ancestor:Some(parent.id),root:parent.root,
            generation:if initial {parent.generation}else {parent.generation.saturating_add(1)},
            source_ordinals:if initial {parent.source_ordinals.clone()}else {vec![]},protected:false,
            masks,score:None,visits:0,trial_id:Some(trial_id)},witness,initial_copy:initial,parent_lane});
        EndpointAdmission::Added {id,evicted}
    }

    /// Exact raw identity only. Equal launch words never establish equal world
    /// state, so distinct literals are retained independently in bounded FIFO.
    fn archive(&mut self,parent:&Member,masks:Vec<u8>,witness:LaunchMoment,cap:usize,
        trial_id:u64,initial:bool,parent_lane:ParentLane)->ArchiveAdmission {
        if cap==0 {return ArchiveAdmission::Disabled;}
        if masks.len()!=MAX_FRAME as usize || (initial && masks!=parent.masks) || (!initial && masks==parent.masks) {
            return ArchiveAdmission::InvalidLiteral;}
        if self.launches.iter().any(|e|e.member.masks==masks) {return ArchiveAdmission::Duplicate;}
        let evicted=if self.launches.len()>=cap {Some(self.launches.remove(0).member.id)}else {None};
        let id=self.next_id;self.next_id+=1;
        self.launches.push(ArchiveEntry {member:Member {id,ancestor:Some(parent.id),root:parent.root,
            generation:if initial {parent.generation}else {parent.generation.saturating_add(1)},
            source_ordinals:if initial {parent.source_ordinals.clone()}else {vec![]},protected:false,
            masks,score:None,visits:0,trial_id:Some(trial_id)},witness,initial_copy:initial,parent_lane});
        ArchiveAdmission::Added {id,evicted}
    }
    fn admit(&mut self,parent:&Member,masks:Vec<u8>,score:Score,cap:usize,trial_id:u64)->Admission {
        if self.members.iter().any(|m|m.masks==masks) {return Admission::Duplicate;}
        let equal_score=self.members.iter().any(|m|m.score.is_some_and(|s|s.compare(score).is_eq()));
        let ordinary:Vec<usize>=self.members.iter().enumerate().filter_map(|(i,m)|(!m.protected).then_some(i)).collect();
        let mut evicted=None;
        if ordinary.len()>=cap {
            let floor=*ordinary.iter().min_by(|&&a,&&b|self.members[a].score.unwrap().compare(self.members[b].score.unwrap())
                .then_with(||self.members[b].visits.cmp(&self.members[a].visits))
                .then_with(||self.members[a].id.cmp(&self.members[b].id))).unwrap();
            if score.compare(self.members[floor].score.unwrap()).is_lt() {return Admission::BelowFloor;}
            evicted=Some(self.members.remove(floor).id);
        }
        let id=self.next_id;self.next_id+=1;
        self.members.push(Member {id,ancestor:Some(parent.id),root:parent.root,generation:parent.generation.saturating_add(1),
            source_ordinals:vec![],protected:false,masks,score:Some(score),visits:0,trial_id:Some(trial_id)});
        Admission::Added {id,evicted,equal_score}
    }
}
struct Rng(u64);
impl Rng {
    fn next(&mut self)->u64 {self.0=self.0.wrapping_add(0x9e3779b97f4a7c15);let mut z=self.0;
        z=(z^(z>>30)).wrapping_mul(0xbf58476d1ce4e5b9);z=(z^(z>>27)).wrapping_mul(0x94d049bb133111eb);z^(z>>31)}
    fn below(&mut self,n:usize)->usize {(self.next()%n as u64) as usize}
}
fn first_change(a:&[u8],b:&[u8])->Option<usize> {a.iter().zip(b).position(|(a,b)|a!=b)}
/// Fixed length, anchor zero: all opening, arrow, grip and ejection controls can change.
fn proposal(parent:&[u8],rng:&mut Rng,window:usize)->Option<(Vec<u8>,usize)> {
    let mut masks=parent.to_vec();let n=masks.len();let pos=n-1-rng.below(window.min(n));
    match rng.below(5) {
        0|1=>{let bit=1u8<<rng.below(8);let end=(pos+1+rng.below(8)).min(n);for m in &mut masks[pos..end] {*m^=bit;}},
        2=>{let value=rng.below(256) as u8;let end=(pos+1+rng.below(24)).min(n);masks[pos..end].fill(value);},
        3=>{let bit=1u8<<rng.below(8);let mut t=pos;
            while t+1<n && (masks[t]^masks[t+1])&bit==0 {t+=1;}
            if t+1<n {let shift=1+rng.below(6);
                if rng.below(2)==0 {let v=masks[t+1]&bit;for m in &mut masks[t.saturating_sub(shift-1)..=t] {*m=(*m&!bit)|v;}}
                else {let v=masks[t]&bit;let end=(t+1+shift).min(n);for m in &mut masks[t+1..end] {*m=(*m&!bit)|v;}}}
            else {masks[pos]^=bit;}},
        _=>masks[pos]=rng.below(256) as u8,
    }
    first_change(parent,&masks).map(|changed|(masks,changed))
}
/// Exact legacy full-range path; focused descendants retain all outside bytes.
/// `window` selects positions at the end of this interval. Transition scanning
/// and both retime directions remain inside the whole declared edit interval.
fn proposal_in_range(parent:&[u8],rng:&mut Rng,window:usize,from:usize,to:usize)->Option<(Vec<u8>,usize)> {
    if from==0 && to==parent.len() {return proposal(parent,rng,window);}
    debug_assert!(from<to && to<=parent.len());
    let mut masks=parent.to_vec();let pos=to-1-rng.below(window.min(to-from));
    match rng.below(5) {
        0|1=>{let bit=1u8<<rng.below(8);let end=(pos+1+rng.below(8)).min(to);for m in &mut masks[pos..end] {*m^=bit;}},
        2=>{let value=rng.below(256) as u8;let end=(pos+1+rng.below(24)).min(to);masks[pos..end].fill(value);},
        3=>{let bit=1u8<<rng.below(8);let mut t=pos;
            while t+1<to && (masks[t]^masks[t+1])&bit==0 {t+=1;}
            if t+1<to {let shift=1+rng.below(6);
                if rng.below(2)==0 {let v=masks[t+1]&bit;let begin=t.saturating_sub(shift-1).max(from);
                    for m in &mut masks[begin..=t] {*m=(*m&!bit)|v;}}
                else {let v=masks[t]&bit;let end=(t+1+shift).min(to);for m in &mut masks[t+1..end] {*m=(*m&!bit)|v;}}}
            else {masks[pos]^=bit;}},
        _=>masks[pos]=rng.below(256) as u8,
    }
    first_change(parent,&masks).map(|changed|(masks,changed))
}
#[derive(Default)]
struct Counts {
    frames:AtomicU64,trials:AtomicU64,next_trial:AtomicU64,initial_replays:AtomicU64,
    noops:AtomicU64,duplicates:AtomicU64,admissions:AtomicU64,equal_admissions:AtomicU64,
    site2:AtomicU64,port_aborts:AtomicU64,no_progress:AtomicU64,lagging:AtomicU64,lag_stops:AtomicU64,
    upward:AtomicU64,ceiling_candidates:AtomicU64,ceiling_solved:AtomicU64,
    observed_finishes:AtomicU64,confirmed_finishes:AtomicU64,
    launch_captures:AtomicU64,launch_admissions:AtomicU64,launch_duplicates:AtomicU64,
    protected_root_picks:AtomicU64,launch_picks:AtomicU64,legacy_picks:AtomicU64,archive_empty_picks:AtomicU64,
    pressure_captures:AtomicU64,pressure_admissions:AtomicU64,pressure_duplicates:AtomicU64,
    pressure_picks:AtomicU64,pressure_empty_picks:AtomicU64,pressure_evictions:[AtomicU64;4],
    endpoint_captures:AtomicU64,endpoint_admissions:AtomicU64,endpoint_duplicates:AtomicU64,endpoint_below_floor:AtomicU64,
    endpoint_picks:AtomicU64,endpoint_empty_picks:AtomicU64,endpoint_evictions:[AtomicU64;STRATA],
}
struct Logs {improvements:File,events:File,finishes:File,trials:File,launch_repairs:File,pressure_admissions:File,endpoint_admissions:File,
    enumeration:Option<File>}
struct Shared {
    pool:Mutex<Pool>,stop:AtomicBool,error:Mutex<Option<String>>,winner:Mutex<Option<Vec<u8>>>,
    logs:Mutex<Logs>,counts:Counts,best:Mutex<Option<Score>>,sites:Mutex<[u64;30]>,start:Instant,deadline:Instant,
    enumeration:Option<EnumerationProgress>,
}
impl Shared {
    fn should_stop(&self,cfg:&Config)->bool {
        if self.stop.load(Ordering::Relaxed) {return true;}
        if Instant::now()>=self.deadline || cfg.run_dir.join("STOP").exists() {self.stop.store(true,Ordering::Relaxed);return true;}false
    }
    fn fail(&self,e:String) {let mut error=self.error.lock().unwrap();if error.is_none() {*error=Some(e);}self.stop.store(true,Ordering::Relaxed);}
    fn emit(&self,which:&str,v:Value)->Result<(),String> {
        let mut log=self.logs.lock().map_err(|_|"ceiling log mutex poisoned")?;
        let out=match which {"improvements"=>&mut log.improvements,"events"=>&mut log.events,
            "finishes"=>&mut log.finishes,"launch_repairs"=>&mut log.launch_repairs,
            "pressure_admissions"=>&mut log.pressure_admissions,"endpoint_admissions"=>&mut log.endpoint_admissions,
            "enumeration"=>log.enumeration.as_mut().ok_or("enumeration log is disabled")?,_=>&mut log.trials};
        write_line(out,&v)
    }
}
fn fresh_file(path:&Path)->Result<File,String> {OpenOptions::new().write(true).create_new(true).open(path).map_err(|e|format!("{}: {e}",path.display()))}
fn write_line(out:&mut impl Write,v:&Value)->Result<(),String> {serde_json::to_writer(&mut *out,v).map_err(|e|e.to_string())?;
    out.write_all(b"\n").map_err(|e|e.to_string())}
/// Keep each json! invocation bounded without increasing the crate macro limit.
/// Callers use disjoint object keys; merging moves their values without copies.
fn joined_json_objects(first:Value,second:Value)->Value {match (first,second) {
    (Value::Object(mut a),Value::Object(b))=>{a.extend(b);Value::Object(a)},
    _=>unreachable!("joined_json_objects accepts object literals only"),
}}
fn export_pool(s:&Shared,path:&Path)->Result<(),String> {let snapshot=s.pool.lock().map_err(|_|"ceiling pool mutex poisoned")?.members.clone();
    let mut out=BufWriter::new(fresh_file(path)?);for m in snapshot {write_line(&mut out,&m.value())?;}
    out.flush().map_err(|e|e.to_string())?;out.get_ref().sync_all().map_err(|e|e.to_string())}
fn export_launch_pool(s:&Shared,path:&Path)->Result<(),String> {
    let snapshot=s.pool.lock().map_err(|_|"ceiling pool mutex poisoned")?.launches.clone();
    let mut out=BufWriter::new(fresh_file(path)?);for entry in snapshot {write_line(&mut out,&entry.value())?;}
    out.flush().map_err(|e|e.to_string())?;out.get_ref().sync_all().map_err(|e|e.to_string())
}
fn export_pressure_pool(s:&Shared,path:&Path)->Result<(),String> {
    let snapshot=s.pool.lock().map_err(|_|"ceiling pool mutex poisoned")?.pressures.clone();
    let mut out=BufWriter::new(fresh_file(path)?);for entry in snapshot.into_iter().flatten() {write_line(&mut out,&entry.value())?;}
    out.flush().map_err(|e|e.to_string())?;out.get_ref().sync_all().map_err(|e|e.to_string())
}
fn export_endpoint_pool(s:&Shared,path:&Path)->Result<(),String> {
    let snapshot=s.pool.lock().map_err(|_|"ceiling pool mutex poisoned")?.endpoints.clone();
    let mut out=BufWriter::new(fresh_file(path)?);for entry in snapshot.into_iter().flatten() {write_line(&mut out,&entry.value())?;}
    out.flush().map_err(|e|e.to_string())?;out.get_ref().sync_all().map_err(|e|e.to_string())
}
fn note_parent_pick(c:&Counts,lane:ParentLane) {let count=match lane {
    ParentLane::ProtectedRoot=>&c.protected_root_picks,ParentLane::LaunchArchive=>&c.launch_picks,
    ParentLane::ArchiveEmptyLegacy=>&c.archive_empty_picks,ParentLane::Legacy=>&c.legacy_picks,
    ParentLane::Pressure=>&c.pressure_picks,ParentLane::PressureEmptyLegacy=>&c.pressure_empty_picks,
    ParentLane::Endpoint=>&c.endpoint_picks,ParentLane::EndpointEmptyLegacy=>&c.endpoint_empty_picks,
    ParentLane::Initial|ParentLane::Enumeration|ParentLane::Low5Enumeration=>return};count.fetch_add(1,Ordering::Relaxed);}
/// This chooses one truthful witness for archive metadata, never a living Score.
fn launch_order(a:LaunchMoment,b:LaunchMoment)->std::cmp::Ordering {
    a.evidence.solved_ceiling_tois.cmp(&b.evidence.solved_ceiling_tois)
        .then_with(||a.evidence.ceiling_candidates.cmp(&b.evidence.ceiling_candidates))
        .then_with(||(-a.solve.v[1]).max(0.0).total_cmp(&(-b.solve.v[1]).max(0.0)))
}
fn stats(s:&Shared)->Value {let c=&s.counts;let load=|a:&AtomicU64|a.load(Ordering::Relaxed);let p=s.pool.lock().unwrap();
    let parent_lane_counts=json!({"protected_root_uniform":load(&c.protected_root_picks),
        "launch_repair_uniform":load(&c.launch_picks),"legacy_pool":load(&c.legacy_picks),
        "archive_empty_legacy_pool":load(&c.archive_empty_picks),"pair_pressure_uniform":load(&c.pressure_picks),
        "pressure_empty_legacy_pool":load(&c.pressure_empty_picks),"endpoint_stratum_uniform":load(&c.endpoint_picks),
        "endpoint_empty_legacy_pool":load(&c.endpoint_empty_picks)});
    let endpoint_stats=json!({"captures":load(&c.endpoint_captures),"admissions":load(&c.endpoint_admissions),
        "duplicates":load(&c.endpoint_duplicates),"below_floor":load(&c.endpoint_below_floor),
        "entries_by_stratum":p.endpoints.iter().map(Vec::len).collect::<Vec<_>>(),
        "evictions_by_stratum":c.endpoint_evictions.iter().map(load).collect::<Vec<_>>()});
    let first=json!({"schema":1,"mode":"ceiling","score_version":SCORE_VERSION,"level":45,"character":3,"anchor":0,"max_frame":MAX_FRAME,
        "t":s.start.elapsed().as_secs_f64(),"frames":load(&c.frames),"trials":load(&c.trials),"initial_replays":load(&c.initial_replays),
        "pool":p.members.len(),"protected":p.members.iter().filter(|m|m.protected).count(),"admissions":load(&c.admissions),
        "equal_score_admissions":load(&c.equal_admissions),"current_pool_raw_duplicates":load(&c.duplicates),"noops":load(&c.noops),
        "site2_events":load(&c.site2),"port_aborts":load(&c.port_aborts),"input_no_progress_frames":load(&c.no_progress)});
    let second=json!({
        "lagging_frames":load(&c.lagging),"lag_stops":load(&c.lag_stops),"upward_primary_observations":load(&c.upward),
        "world_ceiling_TOI_candidates":load(&c.ceiling_candidates),"world_ceiling_solved_TOIs":load(&c.ceiling_solved),
        "abort_site_histogram":s.sites.lock().unwrap().to_vec(),"best_observation":s.best.lock().unwrap().map(Score::value),
        "observed_finishes":load(&c.observed_finishes),"confirmed_finishes":load(&c.confirmed_finishes),
        "launch_archive_entries":p.launches.len(),"launch_captures":load(&c.launch_captures),
        "launch_admissions":load(&c.launch_admissions),"launch_duplicates":load(&c.launch_duplicates),
        "parent_lane_counts":parent_lane_counts,
        "pressure_captures":load(&c.pressure_captures),"pressure_admissions":load(&c.pressure_admissions),
        "pressure_duplicates":load(&c.pressure_duplicates),
        "pressure_entries_by_bucket":p.pressures.iter().map(Vec::len).collect::<Vec<_>>(),
        "pressure_evictions_by_bucket":c.pressure_evictions.iter().map(load).collect::<Vec<_>>(),
        "endpoint":endpoint_stats,
        "winner":s.winner.lock().unwrap().is_some(),"stop_requested":s.stop.load(Ordering::Relaxed)});
    let mut value=joined_json_objects(first,second);
    if let Some(progress)=&s.enumeration {value["enumeration"]=progress.value();}value}

/// Bounded world-list membership, not arena residency or the active-child alias.
fn primary_state(s:&Sim)->(Primary,[f64;2]) {
    let h=s.ctx.character.primary.bodies[CHEST];let mut live=false;let mut cursor=s.ctx.world.body_list;
    for _ in 0..MAX_BODIES {if cursor==NONE {break;}if cursor as usize>=MAX_BODIES || !s.ctx.world.bodies.is_alive(cursor) {break;}
        if cursor==h {live=true;break;}cursor=s.ctx.world.body(cursor).next;}
    let mut p=[f64::NAN;2];let mut finite=false;
    if h!=NONE && (h as usize)<MAX_BODIES && s.ctx.world.bodies.is_alive(h) {
        let b=s.ctx.world.body(h);p=[b.sweep.c.x,b.sweep.c.y];
        finite=p.into_iter().chain([b.linear_velocity.x,b.linear_velocity.y,b.angular_velocity]).all(f64::is_finite)
            && !body::is_static(&s.ctx,h) && !body::is_frozen(&s.ctx,h);
    }
    (Primary {handle:h,focus:s.ctx.level.focus,alive:!s.ctx.level.dead.get() && !s.ctx.character.primary.dead.get(),in_world:live,finite},p)
}
/// Verify the fixed world ceiling's full polygon in world coordinates. No vague
/// negative-y match, sensor strip or ordinary level roof can satisfy this check.
fn is_world_ceiling(s:&Sim,h:H)->bool {
    if h==NONE || h as usize>=MAX_SHAPES || !s.ctx.world.shapes.is_alive(h) {return false;}
    let sh=s.ctx.world.shape(h);let ground=s.ctx.level.level_body;
    if sh.body!=ground || ground==NONE || ground as usize>=MAX_BODIES || !s.ctx.world.bodies.is_alive(ground)
        || !body::is_static(&s.ctx,ground) || sh.type_!=1 || sh.vertex_count!=4 || sh.is_sensor.get() {return false;}
    let xf=s.ctx.world.body(ground).xf;let mut corners=[[0.0;2];4];
    for (i,p) in corners.iter_mut().enumerate() {let v=sh.vertices[i];
        *p=[xf.position.x+xf.r.col1.x*v.x+xf.r.col2.x*v.y,xf.position.y+xf.r.col1.y*v.x+xf.r.col2.y*v.y];}
    let expected=[[-4.0,-84.0],[324.0,-84.0],[324.0,-80.0],[-4.0,-80.0]];
    expected.iter().all(|p|corners.contains(p))
}
fn find_world_ceiling(s:&Sim)->Result<H,String> {let shapes:Vec<H>=(0..MAX_SHAPES as H).filter(|&h|is_world_ceiling(s,h)).collect();
    if shapes.len()==1 {Ok(shapes[0])}else {Err(format!("expected one exact static world ceiling, got {}",shapes.len()))}}
type CapturedStep=(super::sim::Frame,Observation,Evidence,[f64;2],[f64;2]);
fn endpoint_target(s:&Sim,c:[f64;2])->Option<EndpointTarget> {
    // Read the same live level specials used by FinishLine and finish_region_distance.
    // No segment projection, fixed coordinate approximation or success claim.
    s.ctx.level.specials[..s.ctx.level.count as usize].iter()
        .filter(|sp|sp.kind==hw_data::SpecialKind::FinishLine)
        .filter_map(|sp|EndpointTarget::from_bounds(c,[sp.lower.x,sp.lower.y,sp.upper.x,sp.upper.y]))
        .min_by(|a,b|a.distance.total_cmp(&b.distance))
}
fn capture_step(s:&mut Sim,ceiling:H)->CapturedStep {
    capture_step_endpoints(s,ceiling,None,0).0
}
fn capture_step_endpoints(s:&mut Sim,ceiling:H,min_frame:Option<u32>,changed:usize)
    ->(CapturedStep,[Option<EndpointMoment>;STRATA]) {
    let (captured,endpoints,_)=capture_step_endpoint_sets(s,ceiling,min_frame,changed,None);
    (captured,endpoints)
}
fn capture_step_endpoint_sets(s:&mut Sim,ceiling:H,min_frame:Option<u32>,changed:usize,diagnostic_min:Option<u32>)
    ->(CapturedStep,[Option<EndpointMoment>;STRATA],[Option<EndpointMoment>;STRATA]) {
    let (before,p0)=primary_state(s);let frame_before=s.frame();let iteration_before=s.replay.iteration;
    let allowed_before=s.replay.input_allowed.get();let requested=s.masks.get(iteration_before as usize).copied();
    let verified_ceiling=if is_world_ceiling(s,ceiling) {ceiling}else {NONE};
    if before.usable() {lab::arm(&[before.handle]);}else {lab::arm(&[]);}
    let f=s.step(0);lab::disarm();let events=lab::take_events();let (after,p1)=primary_state(s);
    let evidence=Evidence::capture(before.handle,verified_ceiling,&events,|x,y|finish_region_distance(&s.ctx,x,y));
    let o=Observation {before,after,frame_before,frame_after:f.frame,iteration_before,iteration_after:s.replay.iteration,
        allowed_before,allowed_after:s.replay.input_allowed.get(),requested_mask_matches:requested==Some(f.mask),
        site:f.site,port_abort:f.port_abort,lagging:f.lagging,finished:f.finished};
    let endpoints=match min_frame {Some(min)=>endpoint::endpoint_moments(o,&events,changed,min,verified_ceiling,|c|endpoint_target(s,c)),
        None=>[None;STRATA]};
    let diagnostics=match diagnostic_min {Some(min)=>endpoint::endpoint_moments(o,&events,0,min,verified_ceiling,|c|endpoint_target(s,c)),
        None=>[None;STRATA]};
    ((f,o,evidence,p0,p1),endpoints,diagnostics)
}
fn capture_step_with_pressure(s:&mut Sim,ceiling:H,enabled:bool)
    ->((super::sim::Frame,Observation,Evidence,[f64;2],[f64;2]),Option<PressureCounts>) {
    if !enabled {return (capture_step(s,ceiling),None);}
    let pairs_before=s.ctx.world.broad_phase.pair_count;
    let buffered_before=s.ctx.world.broad_phase.pair_buffer_count;
    let locked_before=s.ctx.world.lock.get();
    let requested_mask=s.masks.get(s.replay.iteration as usize).copied();
    let captured=capture_step(s,ceiling);
    let counts=PressureCounts {pairs_before,pairs_after:captured.0.pairs,buffered_before,
        buffered_after:s.ctx.world.broad_phase.pair_buffer_count,locked_before,
        locked_after:s.ctx.world.lock.get(),requested_mask,actual_mask:captured.0.mask};
    (captured,Some(counts))
}
fn capture_step_with_banks(s:&mut Sim,ceiling:H,pressure_enabled:bool,endpoint_min:Option<u32>,changed:usize)
    ->(CapturedStep,Option<PressureCounts>,[Option<EndpointMoment>;STRATA]) {
    // The disabled branch is the unchanged pressure recorder, including its old
    // disabled path. It does not read FinishLine bounds or evaluate endpoints.
    if endpoint_min.is_none() {let (captured,counts)=capture_step_with_pressure(s,ceiling,pressure_enabled);
        return (captured,counts,[None;STRATA]);}
    let before=pressure_enabled.then(||(s.ctx.world.broad_phase.pair_count,s.ctx.world.broad_phase.pair_buffer_count,
        s.ctx.world.lock.get(),s.masks.get(s.replay.iteration as usize).copied()));
    let (captured,endpoints)=capture_step_endpoints(s,ceiling,endpoint_min,changed);
    let counts=before.map(|(pairs_before,buffered_before,locked_before,requested_mask)|PressureCounts {
        pairs_before,pairs_after:captured.0.pairs,buffered_before,buffered_after:s.ctx.world.broad_phase.pair_buffer_count,
        locked_before,locked_after:s.ctx.world.lock.get(),requested_mask,actual_mask:captured.0.mask});
    (captured,counts,endpoints)
}
fn capture_step_with_low5_diagnostics(s:&mut Sim,ceiling:H,pressure_enabled:bool,endpoint_min:Option<u32>,
    changed:usize,diagnostic_min:u32)->(CapturedStep,Option<PressureCounts>,[Option<EndpointMoment>;STRATA],[Option<EndpointMoment>;STRATA]) {
    let before=pressure_enabled.then(||(s.ctx.world.broad_phase.pair_count,s.ctx.world.broad_phase.pair_buffer_count,
        s.ctx.world.lock.get(),s.masks.get(s.replay.iteration as usize).copied()));
    let (captured,endpoints,diagnostics)=capture_step_endpoint_sets(s,ceiling,endpoint_min,changed,Some(diagnostic_min));
    let counts=before.map(|(pairs_before,buffered_before,locked_before,requested_mask)|PressureCounts {
        pairs_before,pairs_after:captured.0.pairs,buffered_before,buffered_after:s.ctx.world.broad_phase.pair_buffer_count,
        locked_before,locked_after:s.ctx.world.lock.get(),requested_mask,actual_mask:captured.0.mask});
    (captured,counts,endpoints,diagnostics)
}
fn observed_finish(cfg:&Config,s:&Shared,masks:&[u8],frame:u32,trial_id:u64,case:Option<&EnumerationCase>)->Result<(),String> {
    s.counts.observed_finishes.fetch_add(1,Ordering::Relaxed);
    let confirmed=confirm_any_finish(cfg.level,cfg.character,masks);
    let qualifies=matches!(&confirmed,Ok((f,_)) if *f==frame && *f<=MAX_FRAME);
    let mut value=json!({"schema":1,"trial_id":trial_id,"observed_frame":frame,"tas":tas_text(masks),
        "native_confirmation":match &confirmed {Ok((f,moved))=>json!({"frame":f,"moved":moved}),Err(e)=>json!({"error":e})},
        "confirmed_within299_and_same_observed_frame":qualifies,"original_player_verified":false,"QE_or_pairmax_mechanism_verified":false});
    if let Some(case)=case {value["enumeration"]=case.value();}s.emit("finishes",value)?;
    if qualifies {let mut winner=s.winner.lock().map_err(|_|"ceiling winner mutex poisoned")?;
        if winner.is_none() {let mut out=fresh_file(&cfg.run_dir.join("winner-1.tas"))?;
            out.write_all(tas_text(masks).as_bytes()).map_err(|e|e.to_string())?;out.sync_all().map_err(|e|e.to_string())?;
            *winner=Some(masks.to_vec());s.counts.confirmed_finishes.fetch_add(1,Ordering::Relaxed);s.stop.store(true,Ordering::Relaxed);}}
    Ok(())
}
/// Diagnostic cutoff is independent of archive credit and includes the no-op root.
fn endpoint_capture_settings(cfg:&Config,case:Option<&EnumerationCase>,changed:usize)->(Option<u32>,usize,Option<u32>) {
    ((cfg.endpoint_pool_size>0).then_some(cfg.endpoint_min_frame),changed,
        case.is_some_and(|c|c.low5).then_some(cfg.endpoint_min_frame))
}
fn best_endpoint_phase(xs:&[Option<EndpointMoment>;STRATA],phase:endpoint::EndpointPhase)->Option<EndpointMoment> {
    xs.iter().flatten().copied().filter(|m|m.phase==phase)
        .reduce(|best,m|if m.compare(best).is_gt() {m}else {best})
}
fn replay(cfg:&Config,s:&Shared,masks:&[u8],changed:usize,parent:&Member,initial:bool,parent_lane:ParentLane)
    ->Result<(u64,Option<Score>,Option<LaunchMoment>,Option<PressureMoment>,[Option<EndpointMoment>;STRATA]),String> {
    replay_case(cfg,s,masks,changed,parent,initial,parent_lane,None)
}
fn replay_case(cfg:&Config,s:&Shared,masks:&[u8],changed:usize,parent:&Member,initial:bool,parent_lane:ParentLane,
    case:Option<EnumerationCase>)->Result<(u64,Option<Score>,Option<LaunchMoment>,Option<PressureMoment>,[Option<EndpointMoment>;STRATA]),String> {
    let trial_id=s.counts.next_trial.fetch_add(1,Ordering::Relaxed);let mut sim=Sim::new(cfg.level,cfg.character)?;
    sim.replay=ReplayState::new(MAX_FRAME);sim.masks=masks.to_vec();let ceiling=find_world_ceiling(&sim)?;
    let mut best:Option<Score>=None;let mut best_launch:Option<LaunchMoment>=None;let mut best_pressure:Option<PressureMoment>=None;
    let mut best_endpoints:[Option<EndpointMoment>;STRATA]=[None;STRATA];
    let mut best_diagnostics:[Option<EndpointMoment>;STRATA]=[None;STRATA];
    let mut completed=true;let mut reason="ceiling";let mut consecutive_lag=0u32;
    let mut enumeration_guarded=false;let mut enumeration_facts=[0u32;6];
    let (endpoint_min,endpoint_cutoff,diagnostic_min)=endpoint_capture_settings(cfg,case.as_ref(),changed);
    for _ in 0..MAX_FRAME {
        if s.should_stop(cfg) {completed=false;reason="stop_or_budget";break;}
        let ((f,o,evidence,p0,p1),pressure_counts,endpoints,diagnostics)=if let Some(min)=diagnostic_min {
            capture_step_with_low5_diagnostics(&mut sim,ceiling,cfg.pressure_pool_size>0,endpoint_min,endpoint_cutoff,min)
        }else {let (captured,counts,endpoints)=capture_step_with_banks(&mut sim,ceiling,cfg.pressure_pool_size>0,endpoint_min,endpoint_cutoff);
            (captured,counts,endpoints,[None;STRATA])};
        s.counts.frames.fetch_add(1,Ordering::Relaxed);
        // Finish latch first, even if the same frame later dies, leaves the world,
        // locks or aborts. Exact native confirmation may reject a port-only abort.
        if f.finished {observed_finish(cfg,s,masks,f.frame,trial_id,case.as_ref())?;}
        if case.is_some() {let advanced=o.allowed_before && o.iteration_after==o.iteration_before+1;
            let port_guard=f.port_abort || matches!(f.site,9|10|29);enumeration_facts[0]+=1;
            enumeration_facts[1]+=u32::from(advanced && o.requested_mask_matches);enumeration_facts[2]+=u32::from(!advanced);
            enumeration_facts[3]+=u32::from(!o.requested_mask_matches);enumeration_facts[4]+=u32::from(f.lagging);
            enumeration_facts[5]+=u32::from(port_guard);enumeration_guarded|=port_guard || f.lagging || !advanced || !o.requested_mask_matches;}
        if o.allowed_before && o.iteration_after<=o.iteration_before {s.counts.no_progress.fetch_add(1,Ordering::Relaxed);}
        if f.port_abort {s.counts.port_aborts.fetch_add(1,Ordering::Relaxed);}
        if f.lagging {s.counts.lagging.fetch_add(1,Ordering::Relaxed);}
        consecutive_lag=if f.lagging {consecutive_lag.saturating_add(1)}else {0};
        if f.site!=0 {let mut hist=s.sites.lock().unwrap();let i=(f.site as usize).min(hist.len()-1);hist[i]=hist[i].saturating_add(1);}
        if f.site==2 {s.counts.site2.fetch_add(1,Ordering::Relaxed);}
        s.counts.ceiling_candidates.fetch_add(evidence.ceiling_candidates as u64,Ordering::Relaxed);
        s.counts.ceiling_solved.fetch_add(evidence.solved_ceiling_tois as u64,Ordering::Relaxed);
        let decision=policy::evaluate(o,evidence,changed);
        if let Some(score)=decision.score {if score.upward_speed>0.0 {s.counts.upward.fetch_add(1,Ordering::Relaxed);}
            if best.map_or(true,|b|score.compare(b).is_gt()) {best=Some(score);}}
        let repair=if cfg.launch_pool_size>0 {policy::repair_launch(o,evidence,changed,cfg.launch_min_frame,cfg.launch_min_speed)}else {None};
        if let Some(launch)=repair {s.counts.launch_captures.fetch_add(1,Ordering::Relaxed);
            if best_launch.map_or(true,|old|launch_order(launch,old).is_gt()) {best_launch=Some(launch);}}
        let pressure=pressure_counts.and_then(|counts|
            policy::pressure_moment(o,counts,evidence.solve,changed,cfg.pressure_min_pairs,
                cfg.pressure_min_frame,cfg.pressure_max_frame));
        if let Some(moment)=pressure {s.counts.pressure_captures.fetch_add(1,Ordering::Relaxed);
            if best_pressure.map_or(true,|old|moment.compare(old).is_gt()) {best_pressure=Some(moment);}}
        for moment in endpoints.into_iter().flatten() {s.counts.endpoint_captures.fetch_add(1,Ordering::Relaxed);
            let bucket=moment.bucket;if best_endpoints[bucket].map_or(true,|old|moment.compare(old).is_gt()) {best_endpoints[bucket]=Some(moment);}}
        for moment in diagnostics.into_iter().flatten() {let bucket=moment.bucket;
            if best_diagnostics[bucket].map_or(true,|old|moment.compare(old).is_gt()) {best_diagnostics[bucket]=Some(moment);}}
        if f.finished || f.site!=0 || evidence.ceiling_candidates>0 || decision.score.is_some() || repair.is_some() || pressure.is_some()
            || endpoints.iter().any(Option::is_some) {
            let first=json!({"schema":1,"trial_id":trial_id,"parent":parent.id,"root":parent.root,"initial_base":initial,
                "parent_selection_lane":parent_lane.name(),
                "frame":f.frame,"iteration_before":o.iteration_before,"iteration_after":o.iteration_after,"mask":tas_text(&[f.mask]),
                "site":f.site,"port_abort":f.port_abort,"lagging":f.lagging,"locked_after":sim.ctx.world.lock.get(),
                "captured_primary_handle":o.before.handle,"focus_before":o.before.focus,"primary_before_usable":o.before.usable(),
                "primary_after_handle":o.after.handle,"focus_after":o.after.focus,"primary_after_usable":o.after.usable()});
            let second=json!({
                "primary_before":p0,"primary_after":p1,"verified_ceiling_shape":ceiling,"evidence":evidence.value(),
                "fitness":decision.score.map(Score::value),"launch_repair_moment":repair.map(LaunchMoment::value),
                "pair_pressure_moment":pressure.map(PressureMoment::value),
                "endpoint_moments":endpoints.map(|m|m.map(EndpointMoment::value)),
                "actual_finished":f.finished,"pairs_current":f.pairs,
                "qe_diagnostic_only":f.qe,"qe_raw_diagnostic_only":f.qe_raw,"displacement_active_alias_diagnostic_only":f.displacement});
            let mut value=joined_json_objects(first,second);
            if let Some(case)=case.as_ref() {value["enumeration"]=case.value();
                value["requested_mask"]=json!(masks.get(o.iteration_before as usize).copied());
                value["allowed_before"]=json!(o.allowed_before);value["requested_mask_matches"]=json!(o.requested_mask_matches);}
            s.emit("events",value)?;
        }
        // Never stop merely because of a launch, candidate TOI, lock or original Site2.
        if s.stop.load(Ordering::Relaxed) {completed=false;reason="confirmed_finish_or_stop";break;}
        if f.port_abort || matches!(f.site,9|10|29) {reason="port_or_host_abort";break;}
        if consecutive_lag>=cfg.lag_frames {s.counts.lag_stops.fetch_add(1,Ordering::Relaxed);reason="lag_guard";break;}
        // FinishLine excludes later victories on the game's dead flag. Keep
        // live frozen/removed/nonprimary states for finish-only continuation;
        // the policies above already reject their fitness and repair credit.
        if sim.ctx.level.dead.get() {reason="game_dead";break;}
    }
    let (terminal,p)=primary_state(&sim);
    let first=json!({"schema":1,"trial_id":trial_id,"parent":parent.id,"root":parent.root,
        "parent_generation":parent.generation,"initial_base":initial,"planned_entries":masks.len(),"tas":tas_text(masks),
        "parent_selection_lane":parent_lane.name(),
        "earliest_changed_index":if initial {None}else {Some(changed)},"credit_after_iteration":changed,"anchor":0,
        "edit_from":cfg.edit_from,"edit_to":cfg.edit_to,
        "confirmation_compatible_horizon":MAX_FRAME,"actual_world_frames":sim.frame(),"consumed_iteration":sim.replay.iteration,
        "best_observation":best.map(Score::value),"best_launch_repair_moment":best_launch.map(LaunchMoment::value)});
    let second=json!({
        "best_pair_pressure_moment":best_pressure.map(PressureMoment::value),
        "best_endpoint_moments":best_endpoints.map(|m|m.map(EndpointMoment::value)),
        "evaluation_ended":completed,"ended_reason":reason,
        "terminal_primary_handle":terminal.handle,"terminal_focus":terminal.focus,"terminal_primary_alive":terminal.alive,
        "terminal_primary_in_world":terminal.in_world,"terminal_primary_finite":terminal.finite,"terminal_primary_usable":terminal.usable(),
        "terminal_primary_position":p,"terminal_level_dead":sim.ctx.level.dead.get(),"terminal_locked":sim.ctx.world.lock.get(),
        "observed_finish_frame":sim.replay.finish_frame,"consecutive_lag_frames":consecutive_lag,
        "terminal_survival_required_for_earlier_fitness":false,"original_player_verified":false});
    let mut value=joined_json_objects(first,second);
    if let Some(case)=case.as_ref() {let outcome=enumeration_outcome(sim.replay.finish_frame,reason,sim.frame(),sim.replay.iteration,enumeration_guarded);
        value["enumeration"]=case.value();value["earliest_changed_index"]=json!(case.changed);
        value["enumeration_outcome"]=json!(outcome.name());value["enumeration_guard_observed"]=json!(enumeration_guarded);
        let facts=json!({"capture_step_calls":enumeration_facts[0],"consumed_requested_steps":enumeration_facts[1],
            "not_advancing_or_disallowed_steps":enumeration_facts[2],"requested_mask_mismatch_steps":enumeration_facts[3],
            "lagging_steps":enumeration_facts[4],"port_or_host_guard_steps":enumeration_facts[5]});
        value["enumeration_replay_facts"]=facts.clone();
        let best_main=best_endpoint_phase(&best_diagnostics,endpoint::EndpointPhase::Main).map(EndpointMoment::value);
        let best_after=best_endpoint_phase(&best_diagnostics,endpoint::EndpointPhase::AfterToi).map(EndpointMoment::value);
        if case.low5 {value["best_Main_endpoint"]=json!(best_main);value["best_AfterToi_endpoint"]=json!(best_after);
            value["endpoint_diagnostic_cutoff_raw"]=json!(0);
            value["endpoint_archive_credit_required"]=json!(false);value["endpoint_diagnostic_only"]=json!(true);}
        s.emit("trials",value)?;
        let mut outcome_value=json!({"schema":1,"record":"outcome","case":case.value(),"trial_id":trial_id,
            "outcome":outcome.name(),"ended_reason":reason,"actual_world_frames":sim.frame(),
            "consumed_iteration":sim.replay.iteration,"observed_finish_frame":sim.replay.finish_frame,
            "guard_observed":enumeration_guarded,"replay_facts":facts,"stop_file_present":cfg.run_dir.join("STOP").exists(),
            "deadline_reached":Instant::now()>=s.deadline,"shared_stop_requested":s.stop.load(Ordering::Relaxed),
            "confirmed_winner_present":s.winner.lock().unwrap().is_some(),"original_player_verified":false});
        if case.low5 {outcome_value["best_Main_endpoint"]=json!(best_main);outcome_value["best_AfterToi_endpoint"]=json!(best_after);
            outcome_value["endpoint_diagnostic_cutoff_raw"]=json!(0);
            outcome_value["endpoint_archive_credit_required"]=json!(false);outcome_value["endpoint_diagnostic_only"]=json!(true);}
        s.emit("enumeration",outcome_value)?;
        s.enumeration.as_ref().expect("enabled enumeration progress").record(outcome);
    }else {s.emit("trials",value)?;}
    Ok((trial_id,best,best_launch,best_pressure,best_endpoints))
}
fn admit(cfg:&Config,s:&Shared,parent:&Member,masks:Vec<u8>,score:Score,changed:usize,trial_id:u64,parent_lane:ParentLane)->Result<(),String> {
    let raw=tas_text(&masks);let admission=s.pool.lock().map_err(|_|"ceiling pool mutex poisoned")?.admit(parent,masks,score,cfg.pool_size,trial_id);
    match admission {Admission::Duplicate=>{s.counts.duplicates.fetch_add(1,Ordering::Relaxed);},Admission::BelowFloor=>{},
        Admission::Added {id,evicted,equal_score}=>{s.counts.admissions.fetch_add(1,Ordering::Relaxed);
            if equal_score {s.counts.equal_admissions.fetch_add(1,Ordering::Relaxed);}
            let improved={let mut b=s.best.lock().unwrap();let improved=b.map_or(true,|old|score.compare(old).is_gt());if improved {*b=Some(score);}improved};
            s.emit("improvements",json!({"schema":1,"id":id,"ancestor":parent.id,"root":parent.root,"generation":parent.generation+1,
                "trial_id":trial_id,"evicted":evicted,"equal_score_admission":equal_score,"best_improvement":improved,
                "parent_selection_lane":parent_lane.name(),
                "earliest_changed_index":changed,"anchor":0,"frame":MAX_FRAME,"tas":raw,"best_observation":score.value(),
                "qualification":"Actual primary-Dad lab observation only; no projected completion, global novelty, original Flash or QE mechanism claim"}))?;}}
    Ok(())
}
fn admit_launch(cfg:&Config,s:&Shared,parent:&Member,masks:Vec<u8>,launch:LaunchMoment,changed:usize,
    trial_id:u64,initial:bool,parent_lane:ParentLane)->Result<(),String> {
    let (admission,entry)={let mut pool=s.pool.lock().map_err(|_|"ceiling pool mutex poisoned")?;
        let admission=pool.archive(parent,masks,launch,cfg.launch_pool_size,trial_id,initial,parent_lane);
        let entry=match admission {ArchiveAdmission::Added {id,..}=>pool.launches.iter().find(|e|e.member.id==id).cloned(),_=>None};
        (admission,entry)};
    match admission {
        ArchiveAdmission::Disabled=>{},ArchiveAdmission::Duplicate=>{s.counts.launch_duplicates.fetch_add(1,Ordering::Relaxed);},
        ArchiveAdmission::InvalidLiteral=>return Err("launch archive input length or copy ancestry invariant failed".into()),
        ArchiveAdmission::Added {evicted,..}=>{s.counts.launch_admissions.fetch_add(1,Ordering::Relaxed);
            let mut value=entry.ok_or("admitted launch archive entry missing")?.value();value["evicted"]=json!(evicted);
            value["earliest_changed_index"]=if initial {Value::Null}else {json!(changed)};
            value["credit_after_iteration"]=json!(changed);value["anchor"]=json!(0);
            value["qualification"]="Exact raw repair parent from fresh pre-live primary launch evidence; no living fitness, survival, completion, state equivalence or original Flash claim".into();
            s.emit("launch_repairs",value)?;}
    }
    Ok(())
}

fn admit_pressure(cfg:&Config,s:&Shared,parent:&Member,masks:Vec<u8>,moment:PressureMoment,changed:usize,
    trial_id:u64,initial:bool,parent_lane:ParentLane)->Result<(),String> {
    let bucket=moment.bucket;
    let (admission,entry)={let mut pool=s.pool.lock().map_err(|_|"ceiling pool mutex poisoned")?;
        let admission=pool.pressure(parent,masks,moment,cfg.pressure_pool_size,trial_id,initial,parent_lane);
        let entry=match admission {ArchiveAdmission::Added {id,..}=>pool.pressures[bucket].iter().find(|e|e.member.id==id).cloned(),_=>None};
        (admission,entry)};
    match admission {
        ArchiveAdmission::Disabled=>{},ArchiveAdmission::Duplicate=>{s.counts.pressure_duplicates.fetch_add(1,Ordering::Relaxed);},
        ArchiveAdmission::InvalidLiteral=>return Err("pressure literal, stratum or ancestry invariant failed".into()),
        ArchiveAdmission::Added {evicted,..}=>{s.counts.pressure_admissions.fetch_add(1,Ordering::Relaxed);
            if evicted.is_some() {s.counts.pressure_evictions[bucket].fetch_add(1,Ordering::Relaxed);}
            let mut v=entry.ok_or("pressure entry missing")?.value();v["evicted"]=json!(evicted);
            v["earliest_changed_index"]=if initial {Value::Null}else {json!(changed)};
            v["credit_after_iteration"]=json!(changed);v["anchor"]=json!(0);
            s.emit("pressure_admissions",v)?;}
    }
    Ok(())
}
fn admit_endpoint(cfg:&Config,s:&Shared,parent:&Member,masks:Vec<u8>,moment:EndpointMoment,changed:usize,
    trial_id:u64,initial:bool,parent_lane:ParentLane)->Result<(),String> {
    let bucket=moment.bucket;
    let (admission,entry)={let mut pool=s.pool.lock().map_err(|_|"ceiling pool mutex poisoned")?;
        let admission=pool.endpoint(parent,masks,moment,cfg.endpoint_pool_size,trial_id,initial,parent_lane);
        let entry=match admission {EndpointAdmission::Added {id,..}=>pool.endpoints[bucket].iter().find(|e|e.member.id==id).cloned(),_=>None};
        (admission,entry)};
    match admission {
        EndpointAdmission::Disabled=>{},EndpointAdmission::Duplicate=>{s.counts.endpoint_duplicates.fetch_add(1,Ordering::Relaxed);},
        EndpointAdmission::BelowFloor=>{s.counts.endpoint_below_floor.fetch_add(1,Ordering::Relaxed);},
        EndpointAdmission::InvalidLiteral=>return Err("endpoint literal, distance, stratum or ancestry invariant failed".into()),
        EndpointAdmission::Added {evicted,..}=>{s.counts.endpoint_admissions.fetch_add(1,Ordering::Relaxed);
            if evicted.is_some() {s.counts.endpoint_evictions[bucket].fetch_add(1,Ordering::Relaxed);}
            let mut v=entry.ok_or("endpoint entry missing")?.value();v["evicted"]=json!(evicted);
            v["earliest_changed_index"]=if initial {Value::Null}else {json!(changed)};
            v["credit_after_iteration"]=json!(changed);v["anchor"]=json!(0);
            v["qualification"]="Closest actual primary COM sample retained only as input DNA; not living fitness, callback-position equivalence, survival, completion, original Flash or QE/pairmax proof".into();
            s.emit("endpoint_admissions",v)?;}
    }
    Ok(())
}

fn worker(w:usize,cfg:&Config,s:&Shared)->Result<(),String> {let mut rng=Rng(cfg.seed^(w as u64+1).wrapping_mul(0x5851f42d4c957f2d));
    while !s.should_stop(cfg) {let (parent,lane)=s.pool.lock().map_err(|_|"ceiling pool mutex poisoned")?.pick_with_endpoints(&mut rng,cfg.launch_pool_size,cfg.pressure_pool_size,cfg.endpoint_pool_size);
        note_parent_pick(&s.counts,lane);
        let Some((masks,changed))=proposal_in_range(&parent.masks,&mut rng,cfg.window,cfg.edit_from,cfg.edit_to) else {s.counts.noops.fetch_add(1,Ordering::Relaxed);continue;};
        s.counts.trials.fetch_add(1,Ordering::Relaxed);let (trial,score,launch,pressure,endpoints)=replay(cfg,s,&masks,changed,&parent,false,lane)?;
        let pressure_masks=pressure.map(|_|masks.clone());
        let endpoint_masks=endpoints.iter().any(Option::is_some).then(||masks.clone());
        match (score,launch) {
            (Some(score),Some(launch))=>{admit(cfg,s,&parent,masks.clone(),score,changed,trial,lane)?;
                admit_launch(cfg,s,&parent,masks,launch,changed,trial,false,lane)?;},
            (Some(score),None)=>admit(cfg,s,&parent,masks,score,changed,trial,lane)?,
            (None,Some(launch))=>admit_launch(cfg,s,&parent,masks,launch,changed,trial,false,lane)?,
            (None,None)=>{},
        }
        if let Some(moment)=pressure {admit_pressure(cfg,s,&parent,pressure_masks.unwrap(),moment,changed,trial,false,lane)?;}
        if let Some(raw)=endpoint_masks {for moment in endpoints.into_iter().flatten() {
            admit_endpoint(cfg,s,&parent,raw.clone(),moment,changed,trial,false,lane)?;}}
    }
    Ok(())}
// Enumeration never calls worker, Pool::pick*, proposal* or any RNG helper.
fn enumeration_worker(w:usize,cfg:&Config,s:&Shared)->Result<(),String> {
    let progress=s.enumeration.as_ref().ok_or("missing enumeration progress")?;
    let parent=s.pool.lock().map_err(|_|"ceiling pool mutex poisoned")?.members[0].clone();
    while !s.should_stop(cfg) {let Some(id)=progress.claim() else {break;};
        let (case,masks)=EnumerationCase::build(&progress.spec,id,&parent.masks);
        s.emit("enumeration",json!({"schema":1,"record":"claim","worker":w,"case":case.value(),"root":parent.root,"parent":parent.id}))?;
        s.counts.trials.fetch_add(1,Ordering::Relaxed);
        let result=std::panic::catch_unwind(std::panic::AssertUnwindSafe(||
            replay_case(cfg,s,&masks,case.changed.unwrap_or(MAX_FRAME as usize),&parent,false,if progress.spec.low5 {ParentLane::Low5Enumeration}else {ParentLane::Enumeration},Some(case.clone()))));
        let error=match result {Ok(Ok(_))=>continue,Ok(Err(e))=>e,Err(_)=>"enumeration case panicked".into()};
        s.emit("enumeration",json!({"schema":1,"record":"outcome","case":case.value(),"outcome":"interrupted",
            "reason":"host_error_or_panic","error":error,"trial_may_be_absent":true,"original_player_verified":false}))?;
        progress.record(EnumerationOutcome::Interrupted);return Err(error);
    }
    Ok(())
}
pub fn run(cfg:Config)->Result<Vec<u8>,String> {
    cfg.validate()?;super::sim::set_lag_ms(cfg.lag_ms);super::sim::set_lag_frames(cfg.lag_frames);
    let start=Instant::now();let deadline=start.checked_add(Duration::try_from_secs_f64(cfg.hours*3600.0).map_err(|e|e.to_string())?).ok_or("ceiling deadline overflow")?;
    std::fs::create_dir(&cfg.run_dir).map_err(|e|format!("fresh ceiling run {}: {e}",cfg.run_dir.display()))?;
    let logs=Logs {improvements:fresh_file(&cfg.run_dir.join("improvements.jsonl"))?,events:fresh_file(&cfg.run_dir.join("events.jsonl"))?,
        finishes:fresh_file(&cfg.run_dir.join("finishes.jsonl"))?,trials:fresh_file(&cfg.run_dir.join("trials.jsonl"))?,
        launch_repairs:fresh_file(&cfg.run_dir.join("launch-repairs.jsonl"))?,
        pressure_admissions:fresh_file(&cfg.run_dir.join("pair-pressure-admissions.jsonl"))?,
        endpoint_admissions:fresh_file(&cfg.run_dir.join("endpoint-admissions.jsonl"))?,
        enumeration:if cfg.enumeration.is_some() {Some(fresh_file(&cfg.run_dir.join("enumeration.jsonl"))?)}else {None}};
    let s=Shared {pool:Mutex::new(Pool::new(&cfg.bases)),stop:AtomicBool::new(false),error:Mutex::new(None),winner:Mutex::new(None),
        logs:Mutex::new(logs),counts:Counts::default(),best:Mutex::new(None),sites:Mutex::new([0;30]),start,deadline,
        enumeration:cfg.enumeration.clone().map(EnumerationProgress::new)};
    let mut config=fresh_file(&cfg.run_dir.join("config.json"))?;
    let endpoint_config=json!({"pool_size":cfg.endpoint_pool_size,"min_frame":cfg.endpoint_min_frame,
        "stratum_capacity":cfg.endpoint_pool_size/STRATA,"strata":endpoint::STRATUM_NAMES,
        "enabled":cfg.endpoint_pool_size>0,"has_living_fitness":false,
        "retention":"closest recorded COM distance; ties phase/frame/ordinal; exact raw dedup within stratum"});
    let first=json!({"schema":1,"mode":"ceiling","level":45,"character":3,"score_version":SCORE_VERSION,"anchor":0,
        "max_frame":MAX_FRAME,"planned_entries":MAX_FRAME,"threads":cfg.threads,"hours":cfg.hours,"seed":cfg.seed,"window":cfg.window,
        "edit_from":cfg.edit_from,"edit_to":cfg.edit_to,"edit_to_exclusive":true,
        "ordinary_pool_capacity":cfg.pool_size,"lag_ms":cfg.lag_ms,"lag_frames":cfg.lag_frames,"initial_raw_bases":cfg.bases.len()});
    let second=json!({
        "launch_pool_size":cfg.launch_pool_size,"launch_min_frame":cfg.launch_min_frame,"launch_min_speed":cfg.launch_min_speed,
        "launch_archive_enabled":cfg.launch_pool_size>0,"launch_archive_has_living_fitness":false,
        "pressure_pool_size":cfg.pressure_pool_size,"pressure_min_pairs":cfg.pressure_min_pairs,
        "pressure_min_frame":cfg.pressure_min_frame,"pressure_max_frame":cfg.pressure_max_frame,
        "pressure_bucket_capacity":cfg.pressure_pool_size/4,"pressure_has_living_fitness":false,
        "endpoint":endpoint_config,
        "parent_selection":if cfg.endpoint_pool_size>0 {if cfg.pressure_pool_size>0 {
            "25% protected, 25% launch, 12.5% pressure, 12.5% endpoint (nonempty stratum uniform then member uniform), 25% legacy; empty archive lanes fall back legacy"
        }else {"25% protected, 25% launch, 12.5% endpoint (nonempty stratum uniform then member uniform), 37.5% legacy; empty archive lanes fall back legacy"}}
        else if cfg.pressure_pool_size>0 {"25% protected, 25% launch (empty legacy), 12.5% global pressure (empty legacy), 37.5% legacy"}else if cfg.launch_pool_size>0 {"25% protected uniform, 25% archive uniform (empty falls back legacy), 50% legacy"}else {"legacy with unchanged RNG draws"},
        "initial_unique_bases":s.pool.lock().unwrap().members.len(),"initialization_only":cfg.hours==0.0,"fresh_replay":true,
        "finite_planned_tas_semantics":true,"physics_injection":false,"terminal_survival_required_for_earlier_fitness":false});
    let mut value=joined_json_objects(first,second);if let Some(spec)=cfg.enumeration.as_ref() {
        value["enumeration"]=spec.value(&cfg.bases[0]);value["parent_selection"]=json!("one constant protected root; no random proposals or admissions");
        if spec.low5 {value["enumeration"]["endpoint_diagnostics"]=json!({"enabled":true,"min_frame":cfg.endpoint_min_frame,
            "raw_cutoff":0,"archive_independent":true,"diagnostic_only":true,"ceiling_solve_required":false});
            value["enumeration"]["workers_plus_coordinator"]=json!(cfg.threads+1);}}
    write_line(&mut config,&value)?;
    let mut initial=fresh_file(&cfg.run_dir.join("initial-bases.jsonl"))?;
    for (ordinal,masks) in cfg.bases.iter().enumerate() {write_line(&mut initial,&json!({"ordinal":ordinal,"frame":masks.len(),"tas":tas_text(masks)}))?;}
    export_pool(&s,&cfg.run_dir.join("pool-initial.jsonl"))?;
    export_launch_pool(&s,&cfg.run_dir.join("launch-repair-pool-initial.jsonl"))?;
    export_pressure_pool(&s,&cfg.run_dir.join("pair-pressure-pool-initial.jsonl"))?;
    export_endpoint_pool(&s,&cfg.run_dir.join("endpoint-pool-initial.jsonl"))?;
    let mut stats_log=fresh_file(&cfg.run_dir.join("stats.jsonl"))?;write_line(&mut stats_log,&stats(&s))?;
    if cfg.hours>0.0 {
        let bases=s.pool.lock().unwrap().members.clone();
        for parent in bases {if cfg.enumeration.is_some() || s.should_stop(&cfg) {break;}s.counts.initial_replays.fetch_add(1,Ordering::Relaxed);
            match replay(&cfg,&s,&parent.masks,0,&parent,true,ParentLane::Initial) {Ok((trial,score,launch,pressure,endpoints))=>{
                if let Some(m)=s.pool.lock().unwrap().members.iter_mut().find(|m|m.id==parent.id) {m.score=score;m.trial_id=Some(trial);}
                if let Some(score)=score {let mut best=s.best.lock().unwrap();if best.map_or(true,|b|score.compare(b).is_gt()) {*best=Some(score);}}
                if let Some(launch)=launch {if let Err(e)=admit_launch(&cfg,&s,&parent,parent.masks.clone(),launch,0,trial,true,ParentLane::Initial) {s.fail(e);break;}}
                if let Some(moment)=pressure {if let Err(e)=admit_pressure(&cfg,&s,&parent,parent.masks.clone(),moment,0,trial,true,ParentLane::Initial) {s.fail(e);break;}}
                for moment in endpoints.into_iter().flatten() {if let Err(e)=admit_endpoint(&cfg,&s,&parent,parent.masks.clone(),moment,0,trial,true,ParentLane::Initial) {s.fail(e);break;}}},
                Err(e)=>{s.fail(e);break;}}
        }
        std::thread::scope(|scope| {let mut handles=Vec::new();
            for w in 0..cfg.threads {if s.should_stop(&cfg) {break;}let (cfg,s)=(&cfg,&s);
                match std::thread::Builder::new().name(format!("ceiling-{w}")).stack_size(16<<20).spawn_scoped(scope,move||{
                    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(||if cfg.enumeration.is_some() {enumeration_worker(w,cfg,s)}else {worker(w,cfg,s)})) {
                        Ok(Ok(()))=>{},Ok(Err(e))=>s.fail(e),Err(_)=>s.fail("ceiling worker panicked".into())}}) {
                    Ok(h)=>handles.push(h),Err(e)=>{s.fail(format!("ceiling worker spawn: {e}"));break;}}}
            let mut next_stats=Instant::now();let mut next_pool=Instant::now()+Duration::from_secs(60);let mut sequence=0;
            while !s.should_stop(&cfg) && !s.enumeration.as_ref().is_some_and(EnumerationProgress::done) {
                if Instant::now()>=next_stats {if let Err(e)=write_line(&mut stats_log,&stats(&s)) {s.fail(e);}next_stats=Instant::now()+Duration::from_secs(2);}
                if Instant::now()>=next_pool {if let Err(e)=export_pool(&s,&cfg.run_dir.join(format!("pool-checkpoint-{sequence:06}.jsonl"))) {s.fail(e);}
                    if let Err(e)=export_launch_pool(&s,&cfg.run_dir.join(format!("launch-repair-pool-checkpoint-{sequence:06}.jsonl"))) {s.fail(e);}
                    if let Err(e)=export_pressure_pool(&s,&cfg.run_dir.join(format!("pair-pressure-pool-checkpoint-{sequence:06}.jsonl"))) {s.fail(e);}
                    if let Err(e)=export_endpoint_pool(&s,&cfg.run_dir.join(format!("endpoint-pool-checkpoint-{sequence:06}.jsonl"))) {s.fail(e);}
                    sequence+=1;next_pool=Instant::now()+Duration::from_secs(60);}
                std::thread::sleep(Duration::from_millis(250));}
            s.stop.store(true,Ordering::Relaxed);for h in handles {if h.join().is_err() {s.fail("ceiling worker panicked".into());}}
        });
    }
    s.stop.store(true,Ordering::Relaxed);export_pool(&s,&cfg.run_dir.join("pool.jsonl"))?;
    export_launch_pool(&s,&cfg.run_dir.join("launch-repair-pool.jsonl"))?;
    export_pressure_pool(&s,&cfg.run_dir.join("pair-pressure-pool.jsonl"))?;
    export_endpoint_pool(&s,&cfg.run_dir.join("endpoint-pool.jsonl"))?;
    let mut final_stats=stats(&s);final_stats["ended"]=true.into();final_stats["initialization_only"]=(cfg.hours==0.0).into();
    final_stats["stop_file_present"]=cfg.run_dir.join("STOP").exists().into();final_stats["error"]=json!(s.error.lock().unwrap().clone());
    write_line(&mut stats_log,&final_stats)?;stats_log.sync_all().map_err(|e|e.to_string())?;
    {let logs=s.logs.lock().unwrap();logs.improvements.sync_all().map_err(|e|e.to_string())?;
        logs.events.sync_all().map_err(|e|e.to_string())?;logs.finishes.sync_all().map_err(|e|e.to_string())?;
        logs.trials.sync_all().map_err(|e|e.to_string())?;logs.launch_repairs.sync_all().map_err(|e|e.to_string())?;logs.pressure_admissions.sync_all().map_err(|e|e.to_string())?;
        logs.endpoint_admissions.sync_all().map_err(|e|e.to_string())?;
        if let Some(log)=&logs.enumeration {log.sync_all().map_err(|e|e.to_string())?;}}
    if let Some(e)=s.error.lock().unwrap().clone() {return Err(e);}let winner=s.winner.lock().unwrap().clone();
    winner.ok_or_else(||"ceiling ended without a confirmed winner".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn production_test(run:impl FnOnce()+Send+'static) {
        std::thread::Builder::new().name("ceiling-production-test".into()).stack_size(64<<20)
            .spawn(run).expect("production test thread").join().expect("production test failed");
    }
    fn config()->Config {Config {level:45,character:3,bases:vec![vec![17;299]],run_dir:PathBuf::from("unused"),
        threads:2,hours:1.0,seed:1,window:299,edit_from:0,edit_to:299,pool_size:1,lag_ms:15.0,lag_frames:3,
        launch_pool_size:0,launch_min_frame:128,launch_min_speed:1024.0,
        pressure_pool_size:0,pressure_min_pairs:1024,pressure_min_frame:1,pressure_max_frame:299,
        endpoint_pool_size:0,endpoint_min_frame:1,enumeration:None}}
    fn score(quality:f64)->Score {Score {tier:1,quality,frame:83,iteration:83,primary:7,upward_speed:quality,evidence:Evidence::default()}}
    #[test] fn enumeration_bijection_preserves_outside_dna_and_replays_baseline_noop() {
        let spec=EnumerationConfig {cells:vec![285,284],low5:false,low5_values:vec![],start:0,end:65536};let base=vec![156;299];
        let mut seen=std::collections::HashSet::new();let mut noops=0;
        for id in 0..65536 {let (case,raw)=EnumerationCase::build(&spec,id,&base);
            assert_eq!(case.id,(u32::from(raw[285])<<8)|u32::from(raw[284]));
            assert!(seen.insert((raw[285],raw[284])));assert_eq!(raw.len(),299);
            for i in 0..299 {if !spec.cells.contains(&i) {assert_eq!(raw[i],base[i]);}}
            assert_eq!(case.changed,first_change(&base,&raw));if case.changed.is_none() {noops+=1;assert_eq!(Some(id),spec.baseline_id(&base));}}
        assert_eq!((seen.len(),noops),(65536,1));
    }
    #[test] fn enumeration_atomic_shards_are_disjoint_and_have_no_overshoot_claim() {
        for (start,end) in [(0,1),(1,257),(257,65536)] {
            let p=EnumerationProgress::new(EnumerationConfig {cells:vec![284,285],low5:false,low5_values:vec![],start,end});
            let ids=Mutex::new(Vec::new());std::thread::scope(|scope| {for _ in 0..8 {let (p,ids)=(&p,&ids);
                scope.spawn(move||while let Some(id)=p.claim() {ids.lock().unwrap().push(id);});}});
            let mut got=ids.into_inner().unwrap();got.sort_unstable();assert_eq!(got,(start..end).collect::<Vec<_>>());
            assert!(p.claim().is_none());assert_eq!(p.claimed.load(Ordering::Relaxed),u64::from(end-start));
            assert_eq!(p.next.load(Ordering::Relaxed),u64::from(end));assert!(!p.done());
            for _ in start..end {p.record(EnumerationOutcome::Horizon299);}assert!(p.done());}
    }
    #[test] fn enumeration_rejects_invalid_shards_indices_and_more_than_one_root() {
        for spec in [EnumerationConfig {cells:vec![284,284],low5:false,low5_values:vec![],start:0,end:1},EnumerationConfig {cells:vec![284,299],low5:false,low5_values:vec![],start:0,end:1},
            EnumerationConfig {cells:vec![284,285],low5:false,low5_values:vec![],start:7,end:7},EnumerationConfig {cells:vec![284,285],low5:false,low5_values:vec![],start:0,end:65537}] {
            let mut cfg=config();cfg.enumeration=Some(spec);assert!(cfg.validate().is_err());}
        let mut cfg=config();cfg.enumeration=Some(EnumerationConfig {cells:vec![284,285],low5:false,low5_values:vec![],start:0,end:65536});assert!(cfg.validate().is_ok());
        cfg.bases.push(cfg.bases[0].clone());assert!(cfg.validate().is_err());
    }
    #[test] fn enumeration_positive_finish_precedes_fatal_and_guard_outcome_without_claiming_confirmation() {
        let l=launch();let o=Observation {before:l.before,after:l.after,frame_before:285,frame_after:286,
            iteration_before:285,iteration_after:286,allowed_before:true,allowed_after:false,requested_mask_matches:true,
            site:9,port_abort:true,lagging:true,finished:true};let d=policy::evaluate(o,Evidence::default(),285);
        assert!(d.observed_finish);assert!(d.score.is_none());
        assert_eq!(enumeration_outcome(286,"game_dead",286,286,true),EnumerationOutcome::ObservedFinish);
        assert_eq!(enumeration_outcome(0,"game_dead",286,286,false),EnumerationOutcome::IntrinsicGameDead);
        assert_eq!(enumeration_outcome(0,"lag_guard",286,286,true),EnumerationOutcome::Guarded);
        assert_eq!(enumeration_outcome(0,"stop_or_budget",286,286,false),EnumerationOutcome::Interrupted);
        assert_eq!(enumeration_outcome(0,"ceiling",299,299,false),EnumerationOutcome::Horizon299);
        assert_eq!(enumeration_outcome(0,"ceiling",299,285,false),EnumerationOutcome::Interrupted);
    }
    fn low5_spec(cells:Vec<usize>)->EnumerationConfig {
        let end=32u32.pow(cells.len() as u32);EnumerationConfig {cells,low5:true,low5_values:(0..32).collect(),start:0,end}
    }
    #[test] fn low5_custom_alphabet_three_cell_125_literals_preserve_exact_root_and_baseline60() {
        let spec=EnumerationConfig {cells:vec![280,282,284],low5:true,low5_values:vec![17,18,20,24,28],start:0,end:125};
        let mut base=(0..299).map(|i|((i*73+29)&255) as u8).collect::<Vec<_>>();
        base[280]=192|20;base[282]=128|20;base[284]=32|17;
        let mut cfg=config();cfg.bases=vec![base.clone()];cfg.enumeration=Some(spec.clone());assert!(cfg.validate().is_ok());
        assert_eq!(spec.cardinality(),125);assert_eq!(spec.baseline_id(&base),Some(60));
        let mut seen=std::collections::HashSet::new();let mut noops=Vec::new();
        for id in 0..125 {let (case,raw)=EnumerationCase::build(&spec,id,&base);
            assert_eq!(raw.len(),299);assert!(seen.insert(raw.clone()));
            let positions=spec.cells.iter().map(|&cell|spec.low5_values.iter().position(|&v|v==(raw[cell]&31)).unwrap()).collect::<Vec<_>>();
            assert_eq!((positions[0]*25+positions[1]*5+positions[2]) as u32,id);
            for i in 0..299 {if spec.cells.contains(&i) {assert_eq!(raw[i]&0xe0,base[i]&0xe0);}else {assert_eq!(raw[i],base[i]);}}
            assert_eq!(case.changed,first_change(&base,&raw));assert_eq!(case.value()["low5_masks"],json!(spec.cells.iter().map(|&i|raw[i]&31).collect::<Vec<_>>()));
            let literal=tas_text(&raw);assert_eq!(literal.len(),2392);assert!(literal.bytes().all(|b|matches!(b,b'0'|b'1')));
            assert_eq!(super::super::sim::parse_tas(&literal).unwrap(),raw);
            if case.changed.is_none() {noops.push(id);assert_eq!(raw,base);}
        }
        assert_eq!((seen.len(),noops),(125,vec![60]));
        let (_,raw)=EnumerationCase::build(&spec,42,&base);assert_eq!([raw[280]&31,raw[282]&31,raw[284]&31],[18,24,20]);
        let value=spec.value(&base);assert_eq!(value["low5_choices"],json!([17,18,20,24,28]));assert_eq!(value["domain_cardinality"],125);
        assert_eq!(value["case_id"],"base5 low5 alphabet indices in declared cell order; last cell fastest");
        assert_eq!(value["baseline_case_id"],60);assert_eq!(value["baseline_in_shard"],true);
    }
    #[test] fn low5_custom_alphabet_missing_baseline_is_null_and_never_reports_noop() {
        let spec=EnumerationConfig {cells:vec![285,280],low5:true,low5_values:vec![28,17,20],start:0,end:9};
        let mut base=vec![156;299];base[280]=192|18;
        assert_eq!(spec.baseline_id(&base),None);let value=spec.value(&base);
        assert!(value["baseline_case_id"].is_null());assert_eq!(value["baseline_in_shard"],false);
        for id in 0..9 {let (case,raw)=EnumerationCase::build(&spec,id,&base);assert!(case.changed.is_some());assert_ne!(raw,base);}
        let (case,raw)=EnumerationCase::build(&spec,0,&base);assert_eq!(case.cells,[285,280]);
        assert_eq!([raw[285],raw[280]],[128|28,192|28]);
    }
    #[test] fn low5_custom_alphabet_singleton_and_reordered_full_domain_keep_declared_digit_order() {
        let mut spec=EnumerationConfig {cells:vec![280,282,284],low5:true,low5_values:vec![20],start:0,end:1};
        let base=vec![148;299];assert_eq!(spec.baseline_id(&base),Some(0));
        let (case,raw)=EnumerationCase::build(&spec,0,&base);assert!(case.changed.is_none());assert_eq!(raw,base);
        assert_eq!(spec.value(&base)["case_id"],"base1 low5 alphabet indices in declared cell order; last cell fastest");
        spec.cells=vec![285];spec.low5_values=(0..32).rev().collect();spec.end=32;
        assert_eq!(spec.baseline_id(&base),Some(11));assert_eq!(spec.cardinality(),32);
        assert_eq!(EnumerationCase::build(&spec,0,&base).1[285],128|31);
        assert_eq!(EnumerationCase::build(&spec,31,&base).1[285],128);
        assert_eq!(spec.value(&base)["case_id"],"base32 low5 alphabet indices in declared cell order; last cell fastest");
        let default=low5_spec(vec![285]);assert_eq!(default.baseline_id(&base),Some(20));
        assert_eq!(default.value(&base)["case_id"],"base32 low5 digits in declared cell order; last cell fastest");
        assert_eq!(default.value(&base)["baseline_case_id"],20);
    }
    #[test] fn low5_custom_alphabet_validation_rejects_empty_duplicate_large_and_legacy_choices() {
        for values in [vec![],vec![17,17],vec![32],(0..33).collect()] {
            let mut cfg=config();cfg.enumeration=Some(EnumerationConfig {cells:vec![280],low5:true,low5_values:values,start:0,end:1});
            assert!(cfg.validate().is_err());
        }
        let mut cfg=config();cfg.enumeration=Some(EnumerationConfig {cells:vec![280,284],low5:false,low5_values:vec![17],start:0,end:1});
        assert!(cfg.validate().is_err());
        cfg.enumeration=Some(EnumerationConfig {cells:vec![280,282,284],low5:true,low5_values:vec![17,18,20,24,28],start:0,end:126});
        assert!(cfg.validate().is_err());cfg.enumeration.as_mut().unwrap().end=125;assert!(cfg.validate().is_ok());
    }
    #[test] fn low5_custom_alphabet_shards_use_case_indices_and_no_overshoot() {
        let mut seen=Vec::new();
        for (start,end) in [(0,60),(60,61),(61,125)] {
            let p=EnumerationProgress::new(EnumerationConfig {cells:vec![280,282,284],low5:true,low5_values:vec![17,18,20,24,28],start,end});
            while let Some(id)=p.claim() {seen.push(id);p.record(EnumerationOutcome::Horizon299);}
            assert!(p.done());assert!(p.claim().is_none());assert_eq!(p.next.load(Ordering::Relaxed),u64::from(end));
            assert_eq!(p.value()["claimed"],end-start);assert_eq!(p.value()["returned"],end-start);
            assert_eq!(p.value()["unclaimed"],0);assert_eq!(p.value()["claimed_without_returned_outcome"],0);
        }
        assert_eq!(seen,(0..125).collect::<Vec<_>>());
    }
    #[test] fn low5_one_two_three_cell_bijection_preserves_own_high3_and_every_outside_byte() {
        let base=(0..299).map(|i|((i*73+29)&255) as u8).collect::<Vec<_>>();
        for cells in [vec![285],vec![285,279],vec![280,277,285]] {
            let spec=low5_spec(cells);let mut seen=std::collections::HashSet::new();let mut noops=0;
            for id in 0..spec.cardinality() {let (case,raw)=EnumerationCase::build(&spec,id,&base);
                let digits=spec.cells.iter().map(|&cell|raw[cell]&31).collect::<Vec<_>>();
                assert!(seen.insert(digits.clone()));assert_eq!(digits.iter().fold(0u32,|n,&d|n*32+u32::from(d)),id);
                for i in 0..299 {if spec.cells.contains(&i) {assert_eq!(raw[i]&0xe0,base[i]&0xe0);}
                    else {assert_eq!(raw[i],base[i]);}}
                assert_eq!(case.changed,first_change(&base,&raw));assert_eq!(case.masks,spec.cells.iter().map(|&i|raw[i]).collect::<Vec<_>>());
                if case.changed.is_none() {noops+=1;assert_eq!(Some(id),spec.baseline_id(&base));}
            }
            assert_eq!(seen.len(),spec.cardinality() as usize);assert_eq!(noops,1);
        }
    }
    #[test] fn low5_five_cell_edges_permutations_and_exact_full299_literal() {
        let base=(0..299).map(|i|((i*41+197)&255) as u8).collect::<Vec<_>>();
        for cells in [vec![277,279,278,284,285],vec![285,284,278,279,277]] {
            let spec=low5_spec(cells);assert_eq!(spec.cardinality(),33_554_432);
            for id in [0,1,31,32,1023,1024,spec.cardinality()-1,spec.baseline_id(&base).unwrap()] {
                let (case,raw)=EnumerationCase::build(&spec,id,&base);assert_eq!(raw.len(),299);
                assert_eq!(spec.cells.iter().fold(0u32,|n,&i|n*32+u32::from(raw[i]&31)),id);
                for i in 0..299 {if spec.cells.contains(&i) {assert_eq!(raw[i]&0xe0,base[i]&0xe0);}else {assert_eq!(raw[i],base[i]);}}
                let literal=tas_text(&raw);assert_eq!(literal.len(),2392);assert!(literal.bytes().all(|b|matches!(b,b'0'|b'1')));
                assert_eq!(super::super::sim::parse_tas(&literal).unwrap(),raw);assert_eq!(case.changed,first_change(&base,&raw));
                if Some(id)==spec.baseline_id(&base) {assert_eq!(raw,base);assert!(case.changed.is_none());}
            }
        }
    }
    #[test] fn low5_shards_claim_every_range_id_once_without_boundary_overshoot() {
        for (cells,start,end) in [(vec![280,277,285],30_001,32_768),(vec![277,278,279,284,285],33_554_425,33_554_432)] {
            let p=EnumerationProgress::new(EnumerationConfig {cells,low5:true,low5_values:(0..32).collect(),start,end});let ids=Mutex::new(Vec::new());
            std::thread::scope(|scope| {for _ in 0..8 {let (p,ids)=(&p,&ids);
                scope.spawn(move||while let Some(id)=p.claim() {ids.lock().unwrap().push(id);});}});
            let mut actual=ids.into_inner().unwrap();actual.sort_unstable();assert_eq!(actual,(start..end).collect::<Vec<_>>());
            assert!(p.claim().is_none());assert_eq!(p.next.load(Ordering::Relaxed),u64::from(end));
            assert!(!p.done());for _ in start..end {p.record(EnumerationOutcome::Horizon299);}assert!(p.done());
        }
    }
    #[test] fn low5_validation_rejects_bad_cells_shards_roots_and_eighteenth_worker() {
        let specs=[EnumerationConfig {cells:vec![],low5:true,low5_values:(0..32).collect(),start:0,end:1},
            EnumerationConfig {cells:vec![0,1,2,3,4,5],low5:true,low5_values:(0..32).collect(),start:0,end:1},
            EnumerationConfig {cells:vec![284,284],low5:true,low5_values:(0..32).collect(),start:0,end:1},
            EnumerationConfig {cells:vec![299],low5:true,low5_values:(0..32).collect(),start:0,end:1},
            EnumerationConfig {cells:vec![284],low5:true,low5_values:(0..32).collect(),start:7,end:7},
            EnumerationConfig {cells:vec![284],low5:true,low5_values:(0..32).collect(),start:0,end:33},
            EnumerationConfig {cells:vec![277,278,279,284,285],low5:true,low5_values:(0..32).collect(),start:0,end:33_554_433}];
        for spec in specs {let mut cfg=config();cfg.enumeration=Some(spec);assert!(cfg.validate().is_err());}
        let mut cfg=config();cfg.enumeration=Some(low5_spec(vec![277,278,279,284,285]));cfg.threads=17;assert!(cfg.validate().is_ok());
        cfg.threads=18;assert!(cfg.validate().is_err());cfg.threads=17;cfg.edit_from=278;assert!(cfg.validate().is_err());
        cfg.edit_from=0;cfg.bases.push(cfg.bases[0].clone());assert!(cfg.validate().is_err());
    }
    #[test] fn low5_noop_diagnostics_do_not_relax_policy_or_archive_change_credit() {
        let mut cfg=config();cfg.endpoint_min_frame=274;let spec=low5_spec(vec![284,285]);let base=vec![156;299];
        let (case,raw)=EnumerationCase::build(&spec,spec.baseline_id(&base).unwrap(),&base);
        assert_eq!(raw,base);assert!(case.changed.is_none());let changed=case.changed.unwrap_or(MAX_FRAME as usize);
        assert_eq!(endpoint_capture_settings(&cfg,Some(&case),changed),(None,299,Some(274)));
        cfg.endpoint_pool_size=4;assert_eq!(endpoint_capture_settings(&cfg,Some(&case),changed),(Some(274),299,Some(274)));
        let legacy=EnumerationConfig {cells:vec![284,285],low5:false,low5_values:vec![],start:0,end:65536};
        let (old_case,_)=EnumerationCase::build(&legacy,legacy.baseline_id(&base).unwrap(),&base);
        assert_eq!(endpoint_capture_settings(&cfg,Some(&old_case),299),(Some(274),299,None));
        let before=Primary {handle:7,focus:7,alive:true,in_world:true,finite:true};
        let after=Primary {handle:7,focus:98,alive:false,in_world:false,finite:false};
        let o=Observation {before,after,frame_before:285,frame_after:286,iteration_before:285,iteration_after:286,
            allowed_before:true,allowed_after:false,requested_mask_matches:true,site:0,port_abort:false,lagging:false,finished:false};
        let events=[lab::Event::Solve {body:7,c0:[0.0;2],c:[15.0;2],v:[0.0;2],w:0.0,injected:false},
            lab::Event::AfterToi {body:7,c0:[0.0;2],c:[16.0;2],v:[0.0;2]}];
        let target=|c|EndpointTarget::from_bounds(c,[10.0,10.0,20.0,20.0]);
        let diagnostics=endpoint::endpoint_moments(o,&events,0,274,NONE,target);
        assert!(endpoint::endpoint_moments(o,&events,299,274,NONE,target).iter().all(Option::is_none));
        let main=best_endpoint_phase(&diagnostics,endpoint::EndpointPhase::Main).unwrap();
        let after=best_endpoint_phase(&diagnostics,endpoint::EndpointPhase::AfterToi).unwrap();
        assert_eq!((main.bucket,after.bucket),(3,2));assert!(!main.verified_world_ceiling_solve && !after.verified_world_ceiling_solve);
        for m in [main,after] {assert!(m.value()["fitness"].is_null());assert_eq!(m.value()["survival_or_finish_claimed"],false);
            assert_eq!(m.value()["original_player_verified"],false);assert!(!m.observation.finished);}
        assert!(policy::evaluate(o,Evidence::default(),299).score.is_none());
        assert!(policy::repair_launch(o,Evidence::default(),299,1,0.0).is_none());
        let mut strong=main;strong.bucket=0;strong.c=[30.0;2];strong.target=target(strong.c).unwrap();
        let mut mixed=diagnostics;mixed[0]=Some(strong);
        assert_eq!(best_endpoint_phase(&mixed,endpoint::EndpointPhase::Main).unwrap().bucket,3);
        assert_eq!(best_endpoint_phase(&mixed,endpoint::EndpointPhase::AfterToi).unwrap().event_ordinal,1);
    }
    fn launch()->LaunchMoment {let before=Primary {handle:7,focus:7,alive:true,in_world:true,finite:true};
        let mut after=before;after.alive=false;
        let o=Observation {before,after,frame_before:142,frame_after:143,iteration_before:142,iteration_after:143,
            allowed_before:true,allowed_after:true,requested_mask_matches:true,site:0,port_abort:false,lagging:false,finished:false};
        let e=Evidence {solve:Some(policy::Solve {body:7,c0:[2.0,81.0],c:[60.0,-90.0],v:[1000.0,-2048.0],w:1.0,injected:false}),
            ..Evidence::default()};policy::repair_launch(o,e,0,128,1024.0).unwrap()}

    fn pressure()->PressureMoment {let launch=launch();
        let o=Observation {before:launch.before,after:launch.after,frame_before:142,frame_after:143,
            iteration_before:142,iteration_after:143,allowed_before:true,allowed_after:true,
            requested_mask_matches:true,site:0,port_abort:false,lagging:false,finished:false};
        policy::pressure_moment(o,PressureCounts {pairs_before:1000,pairs_after:2048,buffered_before:0,buffered_after:0,
            locked_before:false,locked_after:false,requested_mask:Some(16),actual_mask:16},None,0,1024,1,299).unwrap()}
    fn endpoint_witness(bucket:usize,distance:f64)->EndpointMoment {
        let l=launch();let o=Observation {before:l.before,after:l.after,frame_before:142,frame_after:143,
            iteration_before:142,iteration_after:143,allowed_before:true,allowed_after:true,
            requested_mask_matches:true,site:0,port_abort:false,lagging:false,finished:false};
        let c=[20.0+distance,15.0];let events=[lab::Event::Solve {body:7,c0:[1.0,2.0],c,
            v:[0.0,if bucket<2 {-2048.0}else {-1.0}],w:0.0,injected:false},
            lab::Event::Toi {toi:0.5,shape1:7,shape2:8,body1:7,body2:8},
            lab::Event::AfterToi {body:7,c0:[1.0,2.0],c,v:[0.0,0.0]}];
        endpoint::endpoint_moments(o,&events,0,1,NONE,|c|EndpointTarget::from_bounds(c,[10.0,10.0,20.0,20.0]))[bucket].unwrap()
    }
    #[test] fn endpoint_config_requires_even_four_way_quotas_and_valid_frame_when_disabled() {
        for cap in [1,2,3,5,1025] {let mut c=config();c.endpoint_pool_size=cap;assert!(c.validate().is_err());}
        for cap in [0,4,64,1024] {let mut c=config();c.endpoint_pool_size=cap;assert!(c.validate().is_ok());}
        for frame in [0,300] {let mut c=config();c.endpoint_min_frame=frame;assert!(c.validate().is_err());}
    }
    #[test] fn disabled_endpoint_preserves_pressure_selection_visits_and_proposal_rng() {
        for launch_cap in [0,16] {for pressure_cap in [0,32] {
            let base=vec![17;299];let mut old=Pool::new(&[base.clone(),vec![16;299]]);let parent=old.members[0].clone();
            old.archive(&parent,base.clone(),launch(),16,1,true,ParentLane::Initial);
            old.pressure(&parent,base.clone(),pressure(),32,1,true,ParentLane::Initial);
            old.endpoint(&parent,base,endpoint_witness(0,5.0),16,1,true,ParentLane::Initial);
            let mut new=old.clone();let mut a=Rng(24028520);let mut b=Rng(24028520);
            for _ in 0..2048 {let (x,lx)=old.pick_with_pressure(&mut a,launch_cap,pressure_cap);
                let (y,ly)=new.pick_with_endpoints(&mut b,launch_cap,pressure_cap,0);
                assert_eq!(lx,ly);assert_eq!(x.value(),y.value());assert_eq!(a.0,b.0);
                assert_eq!(proposal_in_range(&x.masks,&mut a,45,240,285),proposal_in_range(&y.masks,&mut b,45,240,285));
                assert_eq!(a.0,b.0);}
            assert_eq!(old.next_id,new.next_id);assert_eq!(old.endpoints[0][0].member.visits,0);
            assert_eq!(new.endpoints[0][0].member.visits,0);
        }}
    }
    #[test] fn endpoint_banks_dedup_within_stratum_keep_global_ids_and_initial_copy_ancestry() {
        let base=vec![17;299];let mut p=Pool::new(&[base.clone(),base.clone()]);let parent=p.members[0].clone();
        let living=p.members.iter().map(Member::value).collect::<Vec<_>>();
        for bucket in 0..STRATA {
            assert_eq!(p.endpoint(&parent,base.clone(),endpoint_witness(bucket,5.0),4,10,true,ParentLane::Initial),
                EndpointAdmission::Added {id:1+bucket as u64,evicted:None});
            let e=&p.endpoints[bucket][0];assert_eq!(e.member.ancestor,Some(parent.id));assert_eq!(e.member.root,parent.root);
            assert_eq!(e.member.generation,parent.generation);assert_eq!(e.member.source_ordinals,[0,1]);
            assert!(e.initial_copy && !e.member.protected && e.member.score.is_none());assert!(e.value()["fitness"].is_null());
            assert_eq!(p.endpoint(&parent,base.clone(),endpoint_witness(bucket,1.0),4,11,true,ParentLane::Initial),EndpointAdmission::Duplicate);
        }
        assert_eq!(p.next_id,5);let mut raw=base.clone();raw[284]^=1;
        assert!(matches!(p.admit(&parent,raw.clone(),score(2.0),1,12),Admission::Added {id:5,..}));
        assert!(matches!(p.archive(&parent,raw.clone(),launch(),16,12,false,ParentLane::Legacy),ArchiveAdmission::Added {id:6,..}));
        assert!(matches!(p.pressure(&parent,raw.clone(),pressure(),16,12,false,ParentLane::Legacy),ArchiveAdmission::Added {id:7,..}));
        let archive_parent=p.endpoints[0][0].member.clone();
        assert_eq!(p.endpoint(&archive_parent,raw.clone(),endpoint_witness(0,1.0),4,12,false,ParentLane::Endpoint),
            EndpointAdmission::Added {id:8,evicted:Some(1)});
        let e=&p.endpoints[0][0];assert_eq!((e.member.ancestor,e.member.root,e.member.generation),(Some(1),0,1));
        assert!(e.member.source_ordinals.is_empty());assert_eq!(e.member.trial_id,Some(12));assert_eq!(e.member.masks,raw);
        assert_eq!(p.members.iter().filter(|m|m.protected).map(Member::value).collect::<Vec<_>>(),living);
        let mut ids:Vec<_>=p.members.iter().map(|m|m.id).chain(p.launches.iter().map(|e|e.member.id))
            .chain(p.pressures.iter().flatten().map(|e|e.member.id)).chain(p.endpoints.iter().flatten().map(|e|e.member.id)).collect();
        let count=ids.len();ids.sort_unstable();ids.dedup();assert_eq!(count,ids.len());
        assert!(p.endpoints.iter().all(|b|b.len()==1));
    }
    #[test] fn endpoint_floor_uses_closest_distance_then_deterministic_witness_ties() {
        let base=vec![17;299];let mut p=Pool::new(&[base.clone()]);let parent=p.members[0].clone();
        let mut raw=base.clone();raw[282]^=1;
        p.endpoint(&parent,raw.clone(),endpoint_witness(0,10.0),8,1,false,ParentLane::Legacy);
        raw[282]^=2;p.endpoint(&parent,raw.clone(),endpoint_witness(0,5.0),8,2,false,ParentLane::Legacy);
        let old_members=p.members.iter().map(Member::value).collect::<Vec<_>>();let next=p.next_id;
        raw[282]^=4;assert_eq!(p.endpoint(&parent,raw.clone(),endpoint_witness(0,11.0),8,3,false,ParentLane::Legacy),EndpointAdmission::BelowFloor);
        assert_eq!(p.next_id,next);assert_eq!(p.endpoints[0].len(),2);
        let mut equal=endpoint_witness(0,10.0);equal.event_ordinal+=1;
        assert_eq!(p.endpoint(&parent,raw.clone(),equal,8,4,false,ParentLane::Legacy),EndpointAdmission::BelowFloor);
        equal.event_ordinal=0;equal.frame-=1;
        assert_eq!(p.endpoint(&parent,raw.clone(),equal,8,5,false,ParentLane::Legacy),EndpointAdmission::Added {id:3,evicted:Some(1)});
        raw[282]^=8;
        assert_eq!(p.endpoint(&parent,raw,endpoint_witness(0,1.0),8,6,false,ParentLane::Legacy),EndpointAdmission::Added {id:4,evicted:Some(3)});
        assert_eq!(p.endpoints[0].iter().map(|e|e.witness.target.distance).collect::<Vec<_>>(),[5.0,1.0]);
        assert!(p.endpoints[1..].iter().all(Vec::is_empty));assert_eq!(p.members.iter().map(Member::value).collect::<Vec<_>>(),old_members);
    }
    #[test] fn endpoint_disabled_and_invalid_admissions_never_change_ids_or_other_banks() {
        let base=vec![17;299];let mut p=Pool::new(&[base.clone()]);let parent=p.members[0].clone();let w=endpoint_witness(0,1.0);
        assert_eq!(p.endpoint(&parent,base.clone(),w,0,1,true,ParentLane::Initial),EndpointAdmission::Disabled);
        for cap in [1,3,5,1025] {assert_eq!(p.endpoint(&parent,base.clone(),w,cap,1,true,ParentLane::Initial),EndpointAdmission::InvalidLiteral);}
        assert_eq!(p.endpoint(&parent,vec![17;298],w,4,1,true,ParentLane::Initial),EndpointAdmission::InvalidLiteral);
        assert_eq!(p.endpoint(&parent,base,w,4,1,false,ParentLane::Legacy),EndpointAdmission::InvalidLiteral);
        let mut invalid=w;invalid.target.distance=f64::NAN;
        assert_eq!(p.endpoint(&parent,parent.masks.clone(),invalid,4,1,true,ParentLane::Initial),EndpointAdmission::InvalidLiteral);
        assert_eq!(p.next_id,1);assert!(p.endpoints.iter().all(Vec::is_empty));assert!(p.launches.is_empty());assert!(p.pressures.iter().all(Vec::is_empty));
    }
    #[test] fn endpoint_parent_quota_and_nonempty_stratum_then_member_sampling_are_exact() {
        for pressure_cap in [0,32] {let base=vec![17;299];let mut p=Pool::new(&[base.clone()]);let parent=p.members[0].clone();
            p.archive(&parent,base.clone(),launch(),16,1,true,ParentLane::Initial);
            p.pressure(&parent,base.clone(),pressure(),32,1,true,ParentLane::Initial);
            p.endpoint(&parent,base.clone(),endpoint_witness(0,5.0),32,1,true,ParentLane::Initial);
            for i in 0..4 {let mut raw=base.clone();raw[282]=i;
                p.endpoint(&parent,raw,endpoint_witness(3,5.0),32,2+i as u64,false,ParentLane::Legacy);}
            let mut seen=[false;8];for seed in 0..256 {let mut expected=Rng(seed);let draw=expected.below(8);seen[draw]=true;
                let endpoint_lane=if pressure_cap>0 {5}else {4};
                let expected_endpoint=if draw==endpoint_lane {let bucket=[0,3][expected.below(2)];
                    Some(p.endpoints[bucket][expected.below(p.endpoints[bucket].len())].member.id)}else {None};
                let mut actual=Rng(seed);let (m,lane)=p.pick_with_endpoints(&mut actual,16,pressure_cap,32);
                let wanted=match draw {0|1=>ParentLane::ProtectedRoot,2|3=>ParentLane::LaunchArchive,
                    4 if pressure_cap>0=>ParentLane::Pressure,_ if draw==endpoint_lane=>ParentLane::Endpoint,_=>ParentLane::Legacy};
                assert_eq!(lane,wanted);if let Some(id)=expected_endpoint {assert_eq!(m.id,id);assert!(m.score.is_none());assert_eq!(actual.0,expected.0);}
            }assert!(seen.into_iter().all(|v|v));
        }
    }
    #[test] fn empty_endpoint_lane_falls_back_to_legacy_without_extra_stratum_draw() {
        for pressure_cap in [0,32] {let p=Pool::new(&[vec![17;299],vec![16;299]]);let mut checked=false;
            for seed in 0..64 {let mut expected_rng=Rng(seed);if expected_rng.below(8)!=if pressure_cap>0 {5}else {4} {continue;}
                let mut expected=p.clone();let old=expected.pick(&mut expected_rng);let mut actual=p.clone();let mut actual_rng=Rng(seed);
                let (new,lane)=actual.pick_with_endpoints(&mut actual_rng,16,pressure_cap,32);
                assert_eq!(lane,ParentLane::EndpointEmptyLegacy);assert_eq!(old.value(),new.value());assert_eq!(expected_rng.0,actual_rng.0);checked=true;
            }assert!(checked);
        }
    }
    #[test] fn disabled_pressure_preserves_existing_pick_and_proposal_rng_with_or_without_launches() {
        for launch_cap in [0,16] {let base=vec![17;299];let mut old=Pool::new(&[base.clone(),vec![16;299]]);
            let parent=old.members[0].clone();old.archive(&parent,base,launch(),16,1,true,ParentLane::Initial);
            let mut new=old.clone();let mut a=Rng(183);let mut b=Rng(183);
            for _ in 0..2048 {let (x,lx)=old.pick_with_archive(&mut a,launch_cap);
                let (y,ly)=new.pick_with_pressure(&mut b,launch_cap,0);
                assert_eq!(lx,ly);assert_eq!(x.value(),y.value());assert_eq!(a.0,b.0);
                assert_eq!(proposal_in_range(&x.masks,&mut a,61,82,143),proposal_in_range(&y.masks,&mut b,61,82,143));
                assert_eq!(a.0,b.0);}
            assert_eq!(old.next_id,new.next_id);assert!(new.pressures.iter().all(Vec::is_empty));}
    }
    #[test] fn pressure_has_bounded_strata_exact_raw_dedup_global_ids_and_initial_ancestry() {
        let base=vec![17;299];let mut p=Pool::new(&[base.clone()]);let parent=p.members[0].clone();
        let living=p.members.iter().map(Member::value).collect::<Vec<_>>();
        assert_eq!(p.pressure(&parent,base.clone(),pressure(),4,1,true,ParentLane::Initial),ArchiveAdmission::Added {id:1,evicted:None});
        assert_eq!(p.pressures[1][0].member.generation,parent.generation);assert!(p.pressures[1][0].initial_copy);
        assert_eq!(p.pressures[1][0].member.source_ordinals,parent.source_ordinals);
        let next=p.next_id;let mut other_witness=pressure();other_witness.bucket=3;
        assert_eq!(p.pressure(&parent,base.clone(),other_witness,4,2,true,ParentLane::Initial),ArchiveAdmission::Duplicate);
        assert_eq!(p.next_id,next);let mut child=base.clone();child[142]^=1;
        assert_eq!(p.pressure(&parent,child.clone(),pressure(),4,3,false,ParentLane::ProtectedRoot),ArchiveAdmission::Added {id:2,evicted:Some(1)});
        assert_eq!(p.pressures[1][0].member.generation,parent.generation+1);
        assert_eq!(p.pressures[1][0].member.ancestor,Some(parent.id));assert!(p.pressures[1][0].member.score.is_none());
        assert!(matches!(p.archive(&parent,child,launch(),16,3,false,ParentLane::ProtectedRoot),ArchiveAdmission::Added {id:3,..}));
        assert_eq!(p.members.iter().map(Member::value).collect::<Vec<_>>(),living);
        assert_eq!(p.pressures.iter().map(Vec::len).sum::<usize>(),1);
    }
    #[test] fn pressure_config_validation_and_explicit_parent_quota() {
        for cap in [1,3,5,1025] {let mut c=config();c.pressure_pool_size=cap;assert!(c.validate().is_err());}
        let mut c=config();c.pressure_pool_size=32;assert!(c.validate().is_ok());
        c.pressure_max_frame=0;assert!(c.validate().is_err());
        let mut p=Pool::new(&[vec![17;299]]);let parent=p.members[0].clone();
        p.pressure(&parent,parent.masks.clone(),pressure(),32,1,true,ParentLane::Initial);
        let mut seen=[false;8];for seed in 0..256 {let draw=Rng(seed).below(8);seen[draw]=true;
            let (_,lane)=p.pick_with_pressure(&mut Rng(seed),0,32);
            assert_eq!(lane,match draw {0|1=>ParentLane::ProtectedRoot,2|3=>ParentLane::ArchiveEmptyLegacy,
                4=>ParentLane::Pressure,_=>ParentLane::Legacy});}
        assert!(seen.into_iter().all(|x|x));
    }

    #[test] fn full299_profile_and_zero_replay_initialization_contract() {
        let mut c=config();assert!(c.validate().is_ok());c.bases[0].pop();assert!(c.validate().is_err());
        c=config();c.threads=0;assert!(c.validate().is_err());c.hours=0.0;assert!(c.validate().is_ok());
        c=config();c.threads=19;assert!(c.validate().is_err());c=config();c.hours=f64::NAN;assert!(c.validate().is_err());
        for (from,to) in [(0,0),(128,128),(143,128),(0,300)] {
            c=config();c.edit_from=from;c.edit_to=to;assert!(c.validate().is_err());
        }
        c=config();c.edit_from=128;c.edit_to=143;assert!(c.validate().is_ok());
    }
    #[test] fn repair_capacity_and_cutoffs_are_valid_even_when_disabled() {
        let mut c=config();c.launch_pool_size=1024;assert!(c.validate().is_ok());
        c.launch_pool_size=1025;assert!(c.validate().is_err());
        for frame in [0,300] {c=config();c.launch_min_frame=frame;assert!(c.validate().is_err());}
        for speed in [-1.0,f64::NAN,f64::INFINITY] {c=config();c.launch_min_speed=speed;assert!(c.validate().is_err());}
        c=config();c.launch_min_frame=1;c.launch_min_speed=0.0;assert!(c.validate().is_ok());
    }
    #[test] fn disabled_archive_preserves_parent_and_proposal_rng_exactly() {
        let base:Vec<u8>=(0..299).map(|i|(i*37+17) as u8).collect();
        let mut legacy=Pool::new(&[base.clone(),vec![16;299]]);let root=legacy.members[0].clone();
        let mut child=base.clone();child[142]^=1;legacy.admit(&root,child,score(100.0),2,1);
        let mut disabled=legacy.clone();let mut a=Rng(14381);let mut b=Rng(14381);
        for _ in 0..2048 {let old=legacy.pick(&mut a);let (new,lane)=disabled.pick_with_archive(&mut b,0);
            assert_eq!(lane,ParentLane::Legacy);assert_eq!(old.value(),new.value());assert_eq!(a.0,b.0);
            assert_eq!(proposal_in_range(&old.masks,&mut a,11,132,143),proposal_in_range(&new.masks,&mut b,11,132,143));
            assert_eq!(a.0,b.0);}
        assert!(disabled.launches.is_empty());assert_eq!(legacy.next_id,disabled.next_id);
    }
    #[test] fn archive_fifo_deduplicates_only_raw_and_keeps_global_ids_and_copy_ancestry() {
        let a=vec![17;299];let mut p=Pool::new(&[a.clone(),vec![16;299]]);let parent=p.members[0].clone();
        let mut ordinary=a.clone();ordinary[140]^=2;
        assert_eq!(p.admit(&parent,ordinary,score(100.0),1,10),Admission::Added {id:2,evicted:None,equal_score:false});
        assert_eq!(p.archive(&parent,a.clone(),launch(),2,11,true,ParentLane::Initial),ArchiveAdmission::Added {id:3,evicted:None});
        let copied=&p.launches[0];assert_eq!(copied.member.ancestor,Some(parent.id));assert_eq!(copied.member.generation,parent.generation);
        assert_eq!(copied.member.source_ordinals,parent.source_ordinals);assert!(copied.initial_copy);
        assert!(copied.member.score.is_none());assert!(!copied.member.protected);assert_eq!(copied.member.trial_id,Some(11));
        let mut b=a.clone();b[141]^=1;
        assert_eq!(p.archive(&parent,b.clone(),launch(),2,12,false,ParentLane::ProtectedRoot),ArchiveAdmission::Added {id:4,evicted:None});
        assert_eq!(p.archive(&parent,b.clone(),launch(),2,13,false,ParentLane::Legacy),ArchiveAdmission::Duplicate);
        assert_eq!(p.next_id,5);let archive_parent=p.launches[1].member.clone();let mut c=b;c[142]^=2;
        // Same witness words are deliberately retained for a distinct literal.
        assert_eq!(p.archive(&archive_parent,c.clone(),launch(),2,14,false,ParentLane::LaunchArchive),ArchiveAdmission::Added {id:5,evicted:Some(3)});
        assert_eq!(p.launches.iter().map(|e|e.member.id).collect::<Vec<_>>(),[4,5]);
        let chained=&p.launches[1];assert_eq!(chained.member.ancestor,Some(4));assert_eq!(chained.member.root,0);
        assert_eq!(chained.member.generation,2);assert!(!chained.initial_copy);assert_eq!(chained.member.masks,c);
        assert!(p.launches.iter().all(|e|e.member.score.is_none()));
        assert!(p.members.iter().any(|m|m.protected && m.masks==a));
        let mut ids:Vec<_>=p.members.iter().map(|m|m.id).chain(p.launches.iter().map(|e|e.member.id)).collect();
        let count=ids.len();ids.sort_unstable();ids.dedup();assert_eq!(ids.len(),count);
    }
    #[test] fn archive_has_independent_capacity_and_does_not_touch_living_floor() {
        let a=vec![17;299];let mut p=Pool::new(&[a.clone()]);let parent=p.members[0].clone();
        let mut ordinary=a.clone();ordinary[140]^=1;p.admit(&parent,ordinary,score(100.0),1,1);
        let before=p.members.iter().map(Member::value).collect::<Vec<_>>();
        let mut repair=a.clone();repair[141]^=2;
        assert_eq!(p.admit(&parent,repair.clone(),score(1.0),1,2),Admission::BelowFloor);
        assert!(matches!(p.archive(&parent,repair.clone(),launch(),1,2,false,ParentLane::Legacy),ArchiveAdmission::Added {..}));
        assert_eq!(p.members.iter().map(Member::value).collect::<Vec<_>>(),before);
        assert_eq!(p.members.iter().filter(|m|!m.protected).count(),1);assert_eq!(p.launches.len(),1);
        assert_eq!(p.archive(&parent,a.clone(),launch(),0,3,true,ParentLane::Initial),ArchiveAdmission::Disabled);
        assert_eq!(p.archive(&parent,vec![17;298],launch(),1,3,false,ParentLane::Legacy),ArchiveAdmission::InvalidLiteral);
        assert_eq!(p.archive(&parent,repair,launch(),1,3,true,ParentLane::Initial),ArchiveAdmission::InvalidLiteral);
        assert_eq!(p.archive(&parent,a,launch(),1,3,false,ParentLane::Legacy),ArchiveAdmission::InvalidLiteral);
    }
    #[test] fn explicit_root_and_archive_lanes_ignore_living_tournament_scores() {
        let a=vec![17;299];let mut p=Pool::new(&[a.clone(),vec![16;299]]);let parent=p.members[0].clone();
        let mut ordinary=a.clone();ordinary[140]^=1;p.admit(&parent,ordinary,score(1e9),1,1);
        p.archive(&parent,a,launch(),2,2,true,ParentLane::Initial);let archive_id=p.launches[0].member.id;
        let mut seen=[false;4];for seed in 0..256 {let draw=Rng(seed).below(4);seen[draw]=true;
            let (chosen,lane)=p.pick_with_archive(&mut Rng(seed),2);
            match draw {0=>{assert_eq!(lane,ParentLane::ProtectedRoot);assert!(chosen.protected);},
                1=>{assert_eq!(lane,ParentLane::LaunchArchive);assert_eq!(chosen.id,archive_id);assert!(chosen.score.is_none());},
                _=>assert_eq!(lane,ParentLane::Legacy)}
        }
        assert!(seen.into_iter().all(|x|x));
    }
    #[test] fn empty_archive_quarter_falls_back_to_the_exact_legacy_pick() {
        let p=Pool::new(&[vec![17;299],vec![16;299]]);let mut checked=false;
        for seed in 0..64 {let mut expected_rng=Rng(seed);if expected_rng.below(4)!=1 {continue;}
            let mut expected=p.clone();let old=expected.pick(&mut expected_rng);let mut actual=p.clone();let mut actual_rng=Rng(seed);
            let (new,lane)=actual.pick_with_archive(&mut actual_rng,16);assert_eq!(lane,ParentLane::ArchiveEmptyLegacy);
            assert_eq!(old.value(),new.value());assert_eq!(expected_rng.0,actual_rng.0);checked=true;}
        assert!(checked);
    }
    #[test] fn anchor_zero_mutation_preserves_full_length_and_can_edit_all_opening_keys() {
        let base=vec![17;299];let mut seen=[false;60];
        for seed in 0..4096 {if let Some((raw,changed))=proposal(&base,&mut Rng(seed),299) {assert_eq!(raw.len(),299);
            assert_eq!(first_change(&base,&raw),Some(changed));for i in 0..60 {seen[i]|=raw[i]!=base[i];}}}
        assert!(seen.into_iter().all(|x|x));
    }
    #[test] fn default_bounds_preserve_legacy_proposals_and_rng_exactly() {
        let base:Vec<u8>=(0..299).map(|i|(i*37+17) as u8).collect();
        for window in [1,15,299] {let mut old=Rng(14301);let mut bounded=Rng(14301);
            for _ in 0..1000 {assert_eq!(proposal(&base,&mut old,window),proposal_in_range(&base,&mut bounded,window,0,299));
                assert_eq!(old.0,bounded.0);}}
    }
    #[test] fn focused_all_key_edits_and_retimes_never_escape_the_interval() {
        let base:Vec<u8>=(0..299).map(|i|(i*37+17) as u8).collect();
        for (from,to) in [(0,1),(128,143),(298,299)] {for window in [1,15,299] {for seed in 0..1024 {
            if let Some((raw,changed))=proposal_in_range(&base,&mut Rng(seed),window,from,to) {
                assert_eq!(raw.len(),299);assert_eq!(&raw[..from],&base[..from]);assert_eq!(&raw[to..],&base[to..]);
                assert!((from..to).contains(&changed));assert_eq!(first_change(&base,&raw),Some(changed));
            }
        }}}
    }
    #[test] fn protected_unscored_literals_and_equal_score_new_raw_get_real_places() {
        let a=vec![17;299];let mut p=Pool::new(&[a.clone(),a.clone()]);assert_eq!(p.members[0].source_ordinals,[0,1]);
        let parent=p.members[0].clone();let mut b=a.clone();b[1]=1;let mut c=a.clone();c[1]=2;
        assert!(matches!(p.admit(&parent,b.clone(),score(2.0),1,1),Admission::Added {evicted:None,..}));
        assert!(matches!(p.admit(&parent,c,score(2.0),1,2),Admission::Added {evicted:Some(_),equal_score:true,..}));
        assert!(p.members.iter().any(|m|m.protected && m.masks==a));
        assert_eq!(p.admit(&parent,a,score(9.0),1,3),Admission::Duplicate);
    }
    #[test] fn top_ceiling_geometry_is_not_the_start_roof_or_spike_backing() {
        production_test(|| {
            let s=Sim::new(45,3).unwrap();let h=find_world_ceiling(&s).unwrap();assert!(is_world_ceiling(&s,h));
            assert!((0..MAX_SHAPES as H).filter(|&x|is_world_ceiling(&s,x)).count()==1);
        });
    }
    #[test]
    #[ignore="actual production replay; run serially with other global lag tests"]
    fn saved_upward83_scores_actual_primary_velocity_not_body_max() {
        production_test(|| {
        let _hang=super::super::sim::HangOverride::new(0);super::super::sim::set_lag_ms(0.0);
        // Exact full299 seed1555 raw SHA19028f1b3f6acc1f9f8d18339e1a07d53a36b86bc1c45d4cec89659041f068af.
        let masks=super::super::sim::parse_tas(include_str!("../../../../reference/tas/obstacle-dad-upward-f83.tas")).unwrap();assert_eq!(masks.len(),299);
        let mut s=Sim::new(45,3).unwrap();s.replay=ReplayState::new(299);s.masks=masks.clone();let ceiling=find_world_ceiling(&s).unwrap();
        for frame in 1..=83 {let (f,o,e,_,_)=capture_step(&mut s,ceiling);assert_eq!(f.mask,masks[frame as usize-1]);
            assert_eq!(o.iteration_after,frame);if frame==83 {let solve=e.solve.unwrap();
                assert_eq!(solve.body,o.before.handle);assert!(o.after.usable());
                assert_eq!(solve.v[0].to_bits(),57.085415761249585f64.to_bits());
                assert_eq!(solve.v[1].to_bits(),(-17.18792077334951f64).to_bits());
                let score=policy::evaluate(o,e,0).score.unwrap();assert_eq!(score.tier,1);
                assert_eq!(score.upward_speed.to_bits(),17.18792077334951f64.to_bits());
                assert_eq!(e.solved_ceiling_tois,0);}}
        });
    }
    #[test]
    #[ignore="actual full299 production replay; run serially with other global lag tests"]
    fn saved_fatal143_keeps_exact_pre_live_primary_launch_only_as_repair() {
        production_test(|| {
            let _hang=super::super::sim::HangOverride::new(0);super::super::sim::set_lag_ms(0.0);
            // Original-Flash-matched exact299 a83b49b98fcf668a42b9b357b695c68d9d89d6c58e892a4d6d4c344481bc59b7.
            let masks=super::super::sim::parse_tas(include_str!("../../../../reference/tas/obstacle-dad-ceiling-strand-f143.tas")).unwrap();
            assert_eq!(masks.len(),299);let mut s=Sim::new(45,3).unwrap();s.replay=ReplayState::new(299);s.masks=masks.clone();
            let ceiling=find_world_ceiling(&s).unwrap();let mut witnessed=false;
            for frame in 1..=299 {let (f,o,e,_,_)=capture_step(&mut s,ceiling);
                assert_eq!(f.frame,frame);assert_eq!(f.mask,masks[frame as usize-1]);assert_eq!(o.iteration_after,frame);
                assert!(!f.finished);assert_eq!(f.site,0);assert!(!f.port_abort);
                if frame==143 {assert!(o.before.usable());assert_eq!(o.before.handle,24);assert!(!o.after.alive);assert!(!o.after.usable());
                    let solve=e.solve.unwrap();assert_eq!(solve.body,24);assert!(!solve.injected);
                    assert_eq!([solve.c0[0].to_bits(),solve.c0[1].to_bits(),solve.c[0].to_bits(),solve.c[1].to_bits(),
                        solve.v[0].to_bits(),solve.v[1].to_bits(),solve.w.to_bits()],
                        [0x403a1db13bb77457,0x4052575f89021abe,0x406a71642eb59081,0xc058d2d321c03694,
                         0x40b5bcfadfd05152,0xc0af307d2cfaa7aa,0x40afb6bb44401d43]);
                    assert_eq!((e.ceiling_candidates,e.stranded_ceiling_tois,e.solved_ceiling_tois),(1,1,0));
                    assert!(policy::evaluate(o,e,128).score.is_none());
                    let repair=policy::repair_launch(o,e,128,128,1024.0).unwrap();
                    assert_eq!(repair.primary,24);assert_eq!(repair.frame,143);assert!(!repair.after.usable());
                    assert_eq!(repair.solve.v[1].to_bits(),0xc0af307d2cfaa7aa);witnessed=true;
                    assert!(policy::repair_launch(o,e,143,128,1024.0).is_none());}
            }
            assert!(witnessed);assert_eq!(s.replay.finish_frame,0);
        });
    }
    #[test]
    #[ignore="actual production replay; run serially with other global lag tests"]
    fn bmx_dead_after_finish_still_exposes_primary_solve_and_solved_world_ceiling() {
        production_test(|| {
        let _hang=super::super::sim::HangOverride::new(0);super::super::sim::set_lag_ms(0.0);
        let raw=include_str!("../../../../reference/tas/bmx610.tas");let masks=super::super::sim::parse_tas(raw).unwrap();
        let mut s=Sim::new(13196,3).unwrap();s.replay=ReplayState::new(masks.len() as u32);s.masks=masks.clone();let ceiling=find_world_ceiling(&s).unwrap();
        for frame in 1..=183 {let (f,o,e,_,_)=capture_step(&mut s,ceiling);assert_eq!(f.frame,frame);assert_eq!(f.mask,masks[frame as usize-1]);
            if frame==183 {assert!(f.finished);assert!(!o.after.alive);let solve=e.solve.unwrap();
                assert_eq!(solve.v[0].to_bits(),28224.8143536376f64.to_bits());assert_eq!(solve.v[1].to_bits(),(-34382.61557228829f64).to_bits());
                assert_eq!(e.solved_ceiling_tois,1);assert!(e.live_finish_checks>0);
                let d=policy::evaluate(o,e,0);assert!(d.observed_finish);assert!(d.score.is_none());}}
        });
    }

    fn observer_streams(s:&Sim)->[String;5] {
        [hw_core::diagnostic::broadphase_string(&s.ctx),
         hw_core::physics_observer::physics_string(&s.ctx),
         hw_core::game_observer::game_string(&s.ctx),
         hw_core::links_observer::links_string(&s.ctx),
         hw_core::mechanics_observer::mechanics_string(&s.ctx)]
    }
    fn evidence_words(e:Evidence)->Vec<u64> {
        let mut words=vec![e.ceiling_candidates as u64,e.solved_ceiling_tois as u64,
            e.stranded_ceiling_tois as u64,e.live_finish_checks as u64,e.primary_frozen as u64];
        if let Some(s)=e.solve {words.extend([1,s.body as u64,s.injected as u64]);
            words.extend(s.c0.into_iter().chain(s.c).chain(s.v).chain([s.w]).map(f64::to_bits));}
        else {words.push(0);}
        for point in [e.ceiling_after_c,e.ceiling_after_v] {
            if let Some(p)=point {words.extend([1,p[0].to_bits(),p[1].to_bits()]);}else {words.push(0);}
        }
        if let Some(d)=e.minimum_live_finish_distance {words.extend([1,d.to_bits()]);}else {words.push(0);}
        words
    }
    fn full299_fixture(raw:&str)->Vec<u8> {
        assert_eq!(raw.len(),2392);assert!(raw.bytes().all(|b|b==b'0' || b==b'1'));
        let masks=super::super::sim::parse_tas(raw).unwrap();assert_eq!(masks.len(),299);masks
    }
    #[test]
    #[ignore="actual full299 pressure regression; run serially with global lag tests"]
    fn flash_matched_f274_is_unscored_global_pressure_not_a_primary_launch() {
        production_test(|| {
            // Timing is controlled, not a new lag or Flash qualification. No injection or state edits.
            let _hang=super::super::sim::HangOverride::new(0);super::super::sim::set_lag_ms(0.0);
            // Exact Flash-MATCH299 fixture SHA8354697bc66d72e9c677884f569d231018f7528b7a097471acb3d295f609754b.
            let masks=full299_fixture(include_str!("../../../../reference/tas/obstacle-pairmax-f274.tas"));
            let mut s=Sim::new(45,3).unwrap();s.replay=ReplayState::new(299);s.masks=masks.clone();
            let ceiling=find_world_ceiling(&s).unwrap();let mut witness=None;let mut recovered=false;
            for frame in 1..=299 {
                let ((f,o,e,_,_),counts)=capture_step_with_pressure(&mut s,ceiling,true);
                let counts=counts.expect("enabled production recorder captures actual counters");
                assert_eq!((f.frame,f.mask),(frame,masks[frame as usize-1]));
                assert_eq!((o.frame_before,o.iteration_before,o.iteration_after),(frame-1,frame-1,frame));
                assert!(o.allowed_before && o.requested_mask_matches);assert!(!f.port_abort && !f.lagging && !f.finished);
                assert_eq!(f.site,if frame==274 {2}else {0});
                if frame==274 {
                    assert!(o.before.usable());assert!(o.before.alive && o.before.in_world && o.before.finite);
                    assert_eq!(o.before.focus,o.before.handle);assert_eq!(o.after.handle,o.before.handle);
                    assert!(o.after.alive);assert_eq!(counts.requested_mask,Some(masks[273]));
                    assert_eq!(counts.actual_mask,masks[273]);assert_eq!(counts.pairs_after,4096);
                    assert!(!counts.locked_before && counts.locked_after);
                    let solve=e.solve.expect("actual primary Main solve");assert_eq!(solve.body,o.before.handle);
                    assert!(!solve.injected);
                    assert_eq!([solve.c0[0].to_bits(),solve.c0[1].to_bits(),solve.c[0].to_bits(),solve.c[1].to_bits(),
                        solve.v[0].to_bits(),solve.v[1].to_bits(),solve.w.to_bits()],
                        [29.722512670478086f64.to_bits(),73.65752706249738f64.to_bits(),
                         29.722236032842197f64.to_bits(),73.66019225280581f64.to_bits(),
                         0.022728209926752944f64.to_bits(),(-0.008511258145643087f64).to_bits(),
                         (-0.007765985304683999f64).to_bits()]);
                    assert!((solve.v[0]*solve.v[0]+solve.v[1]*solve.v[1]).sqrt()<0.03);
                    assert_eq!((e.ceiling_candidates,e.solved_ceiling_tois,e.live_finish_checks),(0,0,0));
                    assert!(policy::repair_launch(o,e,273,128,1024.0).is_none());
                    let pressure=policy::pressure_moment(o,counts,e.solve,273,1024,1,299).unwrap();
                    assert_eq!(pressure.bucket,3);assert!(pressure.value()["fitness"].is_null());
                    assert_eq!(pressure.value()["Dad_caused_pair_growth_claimed"],false);
                    // The real pairmax cannot earn an older window for a later edit or a laggy observation.
                    assert!(policy::pressure_moment(o,counts,e.solve,274,1024,1,299).is_none());
                    let mut lagged=o;lagged.lagging=true;
                    assert!(policy::pressure_moment(lagged,counts,e.solve,273,1024,1,299).is_none());
                    witness=Some(pressure);
                }
                if frame==275 {assert!(counts.locked_before && !counts.locked_after);
                    assert_eq!(counts.pairs_after,1095);recovered=true;}
            }
            assert!(recovered);assert_eq!(s.replay.finish_frame,0);assert_eq!(s.replay.iteration,299);
            assert_eq!(s.masks,masks);
            let mut pool=Pool::new(&[masks.clone()]);let parent=pool.members[0].clone();
            assert_eq!(pool.pressure(&parent,masks,witness.unwrap(),4,1,true,ParentLane::Initial),
                ArchiveAdmission::Added {id:1,evicted:None});
            let entry=&pool.pressures[3][0];assert!(entry.member.score.is_none());assert!(entry.value()["fitness"].is_null());
            assert_eq!((entry.member.ancestor,entry.member.root,entry.member.generation),(Some(parent.id),parent.root,parent.generation));
            assert!(entry.initial_copy);assert_eq!(pool.members.len(),1);assert!(pool.launches.is_empty());
        });
    }
    #[test]
    #[ignore="three exact full299 pressure-off/on observer replays; run serially with global lag tests"]
    fn pressure_recording_preserves_all_five_observers_and_replay_for_three_controls() {
        production_test(|| {
            let _hang=super::super::sim::HangOverride::new(0);super::super::sim::set_lag_ms(0.0);
            let fixtures=[
                ("original",include_str!("../../../../reference/tas/obstacle-dad-ceiling-strand-f143.tas")),
                ("roof",include_str!("../../../../reference/tas/obstacle-dad-roofsolve-f143.tas")),
                ("f274",include_str!("../../../../reference/tas/obstacle-pairmax-f274.tas"))];
            for (name,raw) in fixtures {
                let masks=full299_fixture(raw);let mut off=Sim::new(45,3).unwrap();let mut on=Sim::new(45,3).unwrap();
                off.replay=ReplayState::new(299);off.masks=masks.clone();on.replay=ReplayState::new(299);on.masks=masks.clone();
                let ceiling=find_world_ceiling(&off).unwrap();assert_eq!(find_world_ceiling(&on).unwrap(),ceiling);
                let mut captures=0;let mut pressure_moments=0;
                for frame in 1..=299 {
                    let before=(off.ctx.world.broad_phase.pair_count,off.ctx.world.broad_phase.pair_buffer_count,
                        off.ctx.world.lock.get(),off.masks.get(off.replay.iteration as usize).copied());
                    let ((a,oa,ea,_,_),disabled)=capture_step_with_pressure(&mut off,ceiling,false);
                    let ((b,ob,eb,_,_),enabled)=capture_step_with_pressure(&mut on,ceiling,true);
                    assert!(disabled.is_none());let counts=enabled.unwrap();captures+=1;
                    assert_eq!((counts.pairs_before,counts.buffered_before,counts.locked_before,counts.requested_mask),before);
                    assert_eq!((counts.pairs_after,counts.buffered_after,counts.locked_after,counts.actual_mask),
                        (on.ctx.world.broad_phase.pair_count,on.ctx.world.broad_phase.pair_buffer_count,on.ctx.world.lock.get(),b.mask));
                    assert_eq!((a.frame,a.mask,a.site,a.port_abort,a.finished,a.pairs,a.contacts),
                        (b.frame,b.mask,b.site,b.port_abort,b.finished,b.pairs,b.contacts));
                    assert_eq!((a.frame,a.mask),(frame,masks[frame as usize-1]));assert!(!a.port_abort && !b.port_abort);
                    assert!(!a.lagging && !b.lagging);assert!(!a.finished && !b.finished);
                    assert_eq!((oa.iteration_before,oa.iteration_after),(ob.iteration_before,ob.iteration_after));
                    assert_eq!(oa.iteration_after,frame);assert_eq!(evidence_words(ea),evidence_words(eb));
                    if let Some(moment)=policy::pressure_moment(ob,counts,eb.solve,0,1024,1,299) {
                        pressure_moments+=1;assert!(moment.value()["fitness"].is_null());
                    }
                    assert!(hw_core::diagnostic::replay_string(&off.ctx,&off.replay)==
                        hw_core::diagnostic::replay_string(&on.ctx,&on.replay),"{name} replay frame{frame}");
                    for (kind,(left,right)) in observer_streams(&off).into_iter().zip(observer_streams(&on)).enumerate() {
                        assert!(left==right,"{name} observer{kind} frame{frame}");
                    }
                }
                assert_eq!(captures,299);if name=="f274" {assert!(pressure_moments>0);}
                assert_eq!(off.masks,masks);assert_eq!(on.masks,masks);
                assert_eq!((off.replay.frames,on.replay.frames),(299,299));
                assert_eq!((off.replay.finish_frame,on.replay.finish_frame),(0,0));
            }
        });
    }
    #[test]
    #[ignore="three exact full299 endpoint-off/on observer replays; run serially with global lag tests"]
    fn endpoint_recording_preserves_all_five_observers_replay_and_pressure_capture() {
        production_test(|| {
            let _hang=super::super::sim::HangOverride::new(0);super::super::sim::set_lag_ms(0.0);
            let fixtures=[
                ("original",include_str!("../../../../reference/tas/obstacle-dad-ceiling-strand-f143.tas")),
                ("roof",include_str!("../../../../reference/tas/obstacle-dad-roofsolve-f143.tas")),
                ("f274",include_str!("../../../../reference/tas/obstacle-pairmax-f274.tas"))];
            for (name,raw) in fixtures {
                let masks=full299_fixture(raw);let mut off=Sim::new(45,3).unwrap();let mut on=Sim::new(45,3).unwrap();
                off.replay=ReplayState::new(299);off.masks=masks.clone();on.replay=ReplayState::new(299);on.masks=masks.clone();
                let ceiling=find_world_ceiling(&off).unwrap();assert_eq!(find_world_ceiling(&on).unwrap(),ceiling);
                let mut captures=0;
                for frame in 1..=299 {
                    let ((a,oa,ea,_,_),pa,disabled)=capture_step_with_banks(&mut off,ceiling,true,None,0);
                    let ((b,ob,eb,_,_),pb,enabled,diagnostics)=capture_step_with_low5_diagnostics(&mut on,ceiling,true,Some(1),0,1);
                    assert_eq!(enabled.map(|m|m.map(EndpointMoment::value)),diagnostics.map(|m|m.map(EndpointMoment::value)));
                    assert!(disabled.into_iter().all(|m|m.is_none()));
                    let pa=pa.unwrap();let pb=pb.unwrap();
                    assert_eq!((pa.pairs_before,pa.pairs_after,pa.buffered_before,pa.buffered_after,pa.locked_before,pa.locked_after,pa.requested_mask,pa.actual_mask),
                        (pb.pairs_before,pb.pairs_after,pb.buffered_before,pb.buffered_after,pb.locked_before,pb.locked_after,pb.requested_mask,pb.actual_mask));
                    assert_eq!((a.frame,a.mask,a.site,a.port_abort,a.finished,a.pairs,a.contacts),
                        (b.frame,b.mask,b.site,b.port_abort,b.finished,b.pairs,b.contacts));
                    assert_eq!((a.frame,a.mask),(frame,masks[frame as usize-1]));assert!(!a.lagging && !b.lagging);
                    assert_eq!((oa.iteration_before,oa.iteration_after),(ob.iteration_before,ob.iteration_after));
                    assert_eq!(oa.iteration_after,frame);assert_eq!(evidence_words(ea),evidence_words(eb));
                    for m in enabled.into_iter().flatten() {captures+=1;assert_eq!(m.primary,ob.before.handle);
                        assert_eq!((m.frame,m.iteration),(frame,frame));assert!(m.value()["fitness"].is_null());
                        let target=endpoint_target(&on,m.c).unwrap();assert_eq!(m.target.value(),target.value());}
                    assert!(hw_core::diagnostic::replay_string(&off.ctx,&off.replay)==
                        hw_core::diagnostic::replay_string(&on.ctx,&on.replay),"{name} replay frame{frame}");
                    for (kind,(left,right)) in observer_streams(&off).into_iter().zip(observer_streams(&on)).enumerate() {
                        assert!(left==right,"{name} observer{kind} frame{frame}");
                    }
                }
                assert!(captures>0);assert_eq!(off.masks,masks);assert_eq!(on.masks,masks);
                assert_eq!((off.replay.frames,on.replay.frames),(299,299));
                assert_eq!((off.replay.iteration,on.replay.iteration),(299,299));
                assert_eq!((off.replay.finish_frame,on.replay.finish_frame),(0,0));
            }
        });
    }
}
