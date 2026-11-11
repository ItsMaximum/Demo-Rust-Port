//! One-frame, primary-Dad launch evidence. This is host selection, never physics.
//! A TOI candidate is not a collision: only its ordered, non-Strand AfterToi
//! observation earns the ceiling tier. No projected trajectory earns credit.
use hw_core::lab::Event;
use hw_core::{H, NONE};
use serde_json::{json, Value};
use std::cmp::Ordering;

pub const MAX_FRAME: u32 = 299;
pub const SCORE_VERSION: &str = "dad-primary-ceiling-v1";

#[derive(Clone, Copy, Debug)]
pub struct Primary {
    pub handle: H, pub focus: H, pub alive: bool, pub in_world: bool, pub finite: bool,
}
impl Primary {
    pub fn usable(self) -> bool {
        self.handle != NONE && self.focus == self.handle && self.alive && self.in_world && self.finite
    }
}
#[derive(Clone, Copy, Debug)]
pub struct Solve {
    pub body: H, pub c0: [f64; 2], pub c: [f64; 2], pub v: [f64; 2], pub w: f64,
    pub injected: bool,
}
impl Solve {
    fn finite(self) -> bool { self.c0.into_iter().chain(self.c).chain(self.v).chain([self.w]).all(f64::is_finite) }
    pub fn value(self) -> Value { json!({"body":self.body,"c0":self.c0,"c":self.c,"v":self.v,"w":self.w,"injected":self.injected}) }
}
#[derive(Clone, Copy, Debug, Default)]
pub struct Evidence {
    pub solve: Option<Solve>, pub ceiling_candidates: u32, pub solved_ceiling_tois: u32,
    pub stranded_ceiling_tois: u32, pub ceiling_after_c: Option<[f64;2]>,
    pub ceiling_after_v: Option<[f64;2]>, pub live_finish_checks: u32,
    pub minimum_live_finish_distance: Option<f64>, pub primary_frozen: bool,
}
impl Evidence {
    /// `ceiling` is the exact pre-step geometry-verified world-ceiling shape.
    /// Finish callbacks read level.focus; caller must independently retain that
    /// same primary/focus identity before and after the frame for fitness.
    pub fn capture(primary: H, ceiling: H, events: &[Event], mut distance: impl FnMut(f64,f64)->f64) -> Self {
        let mut out=Self::default(); let mut pending=false;
        for &event in events {
            match event {
                Event::Solve {body,c0,c,v,w,injected} if body==primary => {
                    out.solve=Some(Solve {body,c0,c,v,w,injected});
                }
                Event::Toi {toi,shape1,shape2,body1,body2} => {
                    // Every new TOI supersedes the pending ordered candidate.
                    pending=toi.is_finite() && (0.0..=1.0).contains(&toi) && ceiling!=NONE
                        && ((body1==primary && shape2==ceiling) || (body2==primary && shape1==ceiling));
                    if pending {out.ceiling_candidates=out.ceiling_candidates.saturating_add(1);}
                }
                Event::Strand {body1,body2,..} if pending && (body1==primary || body2==primary) => {
                    out.stranded_ceiling_tois=out.stranded_ceiling_tois.saturating_add(1); pending=false;
                }
                Event::AfterToi {body,c,v,..} if body==primary && pending => {
                    if c.into_iter().chain(v).all(f64::is_finite) {
                        out.solved_ceiling_tois=out.solved_ceiling_tois.saturating_add(1);
                        out.ceiling_after_c=Some(c);out.ceiling_after_v=Some(v);
                    }
                    pending=false;
                }
                Event::Finish {x,y,dead:false,..} if x.is_finite() && y.is_finite() => {
                    let d=distance(x,y);
                    if d.is_finite() && d>=0.0 {
                        out.live_finish_checks=out.live_finish_checks.saturating_add(1);
                        out.minimum_live_finish_distance=Some(out.minimum_live_finish_distance.map_or(d,|old|old.min(d)));
                    }
                }
                Event::Freeze {body} if body==primary => out.primary_frozen=true,
                _=>{}
            }
        }
        out
    }
    pub fn value(self) -> Value {json!({"primary_main_solve":self.solve.map(Solve::value),
        "world_ceiling_TOI_candidates":self.ceiling_candidates,"world_ceiling_solved_TOIs":self.solved_ceiling_tois,
        "world_ceiling_stranded_TOIs":self.stranded_ceiling_tois,"ceiling_after_c":self.ceiling_after_c,
        "ceiling_after_v":self.ceiling_after_v,"live_finish_checks":self.live_finish_checks,
        "minimum_live_finish_distance":self.minimum_live_finish_distance,"primary_frozen":self.primary_frozen,
        "ceiling_and_finish_check_are_same_frame_cooccurrence_not_force_or_order_proof":true,
        "actual_downward_reversal_after_upward_solve":self.solve.zip(self.ceiling_after_v).is_some_and(|(s,v)|s.v[1]<0.0 && v[1]>0.0)})}
}
#[derive(Clone, Copy, Debug)]
pub struct Observation {
    pub before: Primary, pub after: Primary, pub frame_before: u32, pub frame_after: u32,
    pub iteration_before: u32, pub iteration_after: u32, pub allowed_before: bool,
    pub allowed_after: bool, pub requested_mask_matches: bool, pub site: u32,
    pub port_abort: bool, pub lagging: bool, pub finished: bool,
}
#[derive(Clone, Copy, Debug)]
pub struct Score {
    /// Higher tier wins: 3 solved ceiling + live finish callback; 2 solved ceiling;
    /// 1 actual primary upward main-solve velocity. quality is larger-is-better.
    pub tier: u8, pub quality: f64, pub frame: u32, pub iteration: u32,
    pub primary: H, pub upward_speed: f64, pub evidence: Evidence,
}
impl Score {
    pub fn compare(self, other: Self) -> Ordering {
        self.tier.cmp(&other.tier).then_with(||self.quality.total_cmp(&other.quality))
            .then_with(||self.upward_speed.total_cmp(&other.upward_speed))
    }
    pub fn value(self) -> Value {json!({"score_version":SCORE_VERSION,"tier":self.tier,
        "tier_name":match self.tier {3=>"solved_ceiling_and_live_finish_check",2=>"solved_ceiling",_=>"upward_primary_main_solve"},
        "quality_larger_is_better":self.quality,"quality_bits":format!("{:016x}",self.quality.to_bits()),
        "upward_primary_speed":self.upward_speed,"frame":self.frame,"iteration":self.iteration,
        "primary_chest_handle":self.primary,"evidence":self.evidence.value(),"projected_reach_or_finish":false})}
}
#[derive(Clone, Copy, Debug)]
pub struct Decision {pub observed_finish: bool, pub score: Option<Score>}

/// Repair DNA from a real launch moment. This is deliberately separate from
/// living fitness: the captured primary may die later in the same Step.
#[derive(Clone, Copy, Debug)]
pub struct LaunchMoment {
    pub frame: u32, pub iteration: u32, pub primary: H,
    pub solve: Solve, pub evidence: Evidence, pub before: Primary, pub after: Primary, pub site: u32,
}
impl LaunchMoment {
    pub fn value(self) -> Value {json!({"selection_class":"primary_launch_repair",
        "frame":self.frame,"iteration":self.iteration,"captured_primary_handle":self.primary,
        "primary_main_solve":self.solve.value(),"evidence":self.evidence.value(),"site":self.site,
        "focus_before":self.before.focus,"primary_before_alive":self.before.alive,
        "primary_before_in_world":self.before.in_world,"primary_before_finite":self.before.finite,
        "primary_before_usable":self.before.usable(),
        "primary_after_handle":self.after.handle,"focus_after":self.after.focus,
        "primary_after_alive":self.after.alive,"primary_after_in_world":self.after.in_world,
        "primary_after_finite":self.after.finite,"primary_after_usable":self.after.usable(),
        "fitness":null,"survival_or_finish_claimed":false,"original_player_verified":false})}
}

pub fn repair_launch(o: Observation, e: Evidence, earliest_changed_index: usize,
    min_frame: u32, min_speed: f64) -> Option<LaunchMoment> {
    // Capture identity BEFORE the step. A dead/removed/stale alias afterwards
    // never supplies the launch. Input progress excludes retry and EOF slots.
    if !(1..=MAX_FRAME).contains(&min_frame) || !min_speed.is_finite() || min_speed<0.0 {return None;}
    let solve=e.solve?;let upward=(-solve.v[1]).max(0.0);
    let eligible=o.frame_before.checked_add(1)==Some(o.frame_after)
        && (min_frame..=MAX_FRAME).contains(&o.frame_after)
        && o.iteration_before>=earliest_changed_index as u32 && o.iteration_before<MAX_FRAME
        && o.iteration_before.checked_add(1)==Some(o.iteration_after) && o.iteration_after<=MAX_FRAME
        && o.allowed_before && o.requested_mask_matches && !o.port_abort
        && !matches!(o.site,9|10|29) && !o.lagging && o.before.usable()
        && solve.body==o.before.handle && solve.finite() && !solve.injected
        && ((upward>0.0 && upward>=min_speed) || e.ceiling_candidates>0 || e.solved_ceiling_tois>0);
    eligible.then_some(LaunchMoment {frame:o.frame_after,iteration:o.iteration_after,
        primary:o.before.handle,solve,evidence:e,before:o.before,after:o.after,site:o.site})
}

/// Global pair-pressure DNA, never primary force, living fitness or a finish.
#[derive(Clone, Copy, Debug)]
pub struct PressureCounts {
    pub pairs_before:i32, pub pairs_after:i32,
    pub buffered_before:i32, pub buffered_after:i32,
    pub locked_before:bool, pub locked_after:bool,
    pub requested_mask:Option<u8>, pub actual_mask:u8,
}
#[derive(Clone, Copy, Debug)]
pub struct PressureMoment {
    pub observation:Observation, pub counts:PressureCounts,
    pub primary_main_solve:Option<Solve>, pub bucket:usize,
}
impl PressureMoment {
    pub fn compare(self, other:Self)->Ordering {
        (self.observation.site==2).cmp(&(other.observation.site==2))
            .then_with(||self.counts.pairs_after.cmp(&other.counts.pairs_after))
            .then_with(||other.observation.frame_after.cmp(&self.observation.frame_after))
    }
    pub fn value(self)->Value {
        let o=self.observation;let c=self.counts;
        let primary=|p:Primary|json!({"handle":p.handle,"focus":p.focus,"alive":p.alive,
            "in_world":p.in_world,"finite_dynamic_unfrozen":p.finite,"usable":p.usable()});
        json!({"selection_class":"global_pair_pressure_repair","fitness":null,
            "frame_before":o.frame_before,"frame_after":o.frame_after,
            "iteration_before":o.iteration_before,"iteration_after":o.iteration_after,
            "input_allowed_before":o.allowed_before,"input_allowed_after":o.allowed_after,
            "requested_mask":c.requested_mask,"actual_mask":c.actual_mask,
            "requested_mask_matches":o.requested_mask_matches,"before":primary(o.before),"after":primary(o.after),
            "pairs_before":c.pairs_before,"pairs_after":c.pairs_after,
            "buffered_pairs_before":c.buffered_before,"buffered_pairs_after":c.buffered_after,
            "locked_before":c.locked_before,"locked_after":c.locked_after,
            "site":o.site,"port_abort":o.port_abort,"lagging":o.lagging,"bucket":self.bucket,
            "actual_finished_this_frame":o.finished,"primary_main_solve":self.primary_main_solve.map(Solve::value),
            "global_pair_occupancy_only":true,"Dad_caused_pair_growth_claimed":false,"intrastep_peak_claimed":false,
            "survival_or_finish_claimed":false,"selective_contact_omission_verified":false,
            "original_player_verified":false})
    }
}
/// One actually consumed post-change request. Later death/lock/alias changes
/// remain diagnostic DNA in both branches; they never relax evaluate's gates.
pub fn pressure_moment(o:Observation, counts:PressureCounts, solve:Option<Solve>,
    changed:usize, min_pairs:i32, min_frame:u32, max_frame:u32)->Option<PressureMoment> {
    if !(1..=4096).contains(&min_pairs) || !(1..=MAX_FRAME).contains(&min_frame)
        || !(min_frame..=MAX_FRAME).contains(&max_frame) {return None;}
    let counters=[counts.pairs_before,counts.pairs_after,counts.buffered_before,counts.buffered_after];
    let eligible=o.frame_before.checked_add(1)==Some(o.frame_after)
        && (min_frame..=max_frame).contains(&o.frame_after)
        && changed<MAX_FRAME as usize && o.iteration_before as usize>=changed && o.iteration_before<MAX_FRAME
        && o.iteration_before.checked_add(1)==Some(o.iteration_after) && o.iteration_after<=MAX_FRAME
        && o.allowed_before && o.requested_mask_matches && counts.requested_mask==Some(counts.actual_mask)
        && o.before.usable() && !o.port_abort && !o.lagging && matches!(o.site,0|2)
        && counters.into_iter().all(|n|(0..=4096).contains(&n))
        && solve.map_or(true,|s|s.body==o.before.handle && s.finite() && !s.injected)
        && (o.site==2 || counts.pairs_after>=min_pairs);
    if !eligible {return None;}
    let bucket=if o.site==2 {3}else if counts.pairs_after>=3072 {2}
        else if counts.pairs_after>=2048 {1}else {0};
    Some(PressureMoment {observation:o,counts,primary_main_solve:solve,bucket})
}

pub fn evaluate(o: Observation, e: Evidence, earliest_changed_index: usize) -> Decision {
    // Observation of a finish is deliberately independent of all fitness gates.
    let observed_finish=o.finished && o.frame_after<=MAX_FRAME;
    let eligible=o.frame_before.checked_add(1)==Some(o.frame_after) && o.frame_after<=MAX_FRAME
        && o.iteration_before>=earliest_changed_index as u32 && o.iteration_before<MAX_FRAME
        && o.iteration_before.checked_add(1)==Some(o.iteration_after) && o.iteration_after<=MAX_FRAME
        && o.allowed_before && o.allowed_after && o.requested_mask_matches
        && !o.port_abort && !matches!(o.site,9|10|29) && !o.lagging
        && o.before.usable() && o.after.usable() && o.before.handle==o.after.handle
        && !e.primary_frozen && e.solve.map_or(true,|s|s.body==o.before.handle && s.finite() && !s.injected);
    let mut score=None;
    if eligible {
        let upward=e.solve.map_or(0.0,|s|(-s.v[1]).max(0.0));
        let (tier,quality)=if e.solved_ceiling_tois>0 {
            if let Some(d)=e.minimum_live_finish_distance {(3,-d)} else {(2,upward)}
        } else {(1,upward)};
        if tier>1 || upward>0.0 {score=Some(Score {tier,quality,frame:o.frame_after,iteration:o.iteration_after,
            primary:o.before.handle,upward_speed:upward,evidence:e});}
    }
    Decision {observed_finish,score}
}

#[cfg(test)]
mod tests {
    use super::*;
    fn observation() -> Observation {let p=Primary {handle:7,focus:7,alive:true,in_world:true,finite:true};
        Observation {before:p,after:p,frame_before:82,frame_after:83,iteration_before:82,iteration_after:83,
            allowed_before:true,allowed_after:true,requested_mask_matches:true,site:0,port_abort:false,lagging:false,finished:false}}

    fn pressure_counts()->PressureCounts {PressureCounts {pairs_before:1000,pairs_after:2048,
        buffered_before:0,buffered_after:0,locked_before:false,locked_after:false,
        requested_mask:Some(16),actual_mask:16}}
    #[test] fn pressure_dna_is_unscored_even_without_speed_and_with_same_frame_death() {
        for site in [0,2] {let mut o=observation();o.site=site;o.after.alive=false;
            o.after.focus=73;o.after.in_world=false;o.after.finite=false;
            let p=pressure_moment(o,pressure_counts(),None,82,1024,1,299).unwrap();
            assert_eq!(p.bucket,if site==2 {3}else {1});assert!(!p.observation.after.usable());
            assert!(evaluate(o,Evidence::default(),82).score.is_none());
            assert!(repair_launch(o,Evidence::default(),82,1,1024.0).is_none());}
    }
    #[test] fn pressure_strata_and_exception_threshold_are_literal_counter_facts() {
        for (pairs,bucket) in [(1024,0),(2047,0),(2048,1),(3071,1),(3072,2),(4096,2)] {
            let mut c=pressure_counts();c.pairs_after=pairs;
            assert_eq!(pressure_moment(observation(),c,None,0,1024,1,299).unwrap().bucket,bucket);}
        let mut c=pressure_counts();c.pairs_after=1023;
        assert!(pressure_moment(observation(),c,None,0,1024,1,299).is_none());
        let mut o=observation();o.site=2;
        assert_eq!(pressure_moment(o,c,None,0,4096,1,299).unwrap().bucket,3);
    }
    #[test] fn pressure_requires_own_fresh_request_and_rejects_port_lag_and_invalid_counters() {
        for fault in 0..12 {let mut o=observation();let mut c=pressure_counts();match fault {
            0=>o.iteration_after=o.iteration_before,1=>o.allowed_before=false,
            2=>o.requested_mask_matches=false,3=>{o.iteration_before=299;o.iteration_after=300;},
            4=>o.port_abort=true,5=>o.site=29,6=>o.lagging=true,7=>o.before.focus=8,
            8=>o.before.in_world=false,9=>c.requested_mask=Some(17),10=>c.pairs_after=4097,_=>c.buffered_before=-1};
            assert!(pressure_moment(o,c,None,0,1024,1,299).is_none());}
        assert!(pressure_moment(observation(),pressure_counts(),None,83,1024,1,299).is_none());
        let mut s=Solve {body:7,c0:[0.0;2],c:[0.0;2],v:[0.0;2],w:0.0,injected:true};
        assert!(pressure_moment(observation(),pressure_counts(),Some(s),0,1024,1,299).is_none());
        s.injected=false;s.body=8;
        assert!(pressure_moment(observation(),pressure_counts(),Some(s),0,1024,1,299).is_none());
        let mut o=observation();o.finished=true;o.after.alive=false;o.port_abort=true;
        assert!(evaluate(o,Evidence::default(),0).observed_finish);
        assert!(pressure_moment(o,pressure_counts(),None,0,1024,1,299).is_none());
    }

    fn solve(body:H,vy:f64)->Event {Event::Solve {body,c0:[2.0,81.0],c:[3.0,80.0],v:[57.0,vy],w:1.0,injected:false}}
    fn toi()->Event {Event::Toi {toi:0.1,shape1:20,shape2:2,body1:7,body2:0}}
    fn after(body:H)->Event {Event::AfterToi {body,c0:[0.0,-80.0],c:[30.0,60.0],v:[90.0,50.0]}}
    #[test] fn child_or_replaced_or_dead_primary_is_never_fitness() {
        let e=Evidence::capture(7,2,&[solve(8,-1e9)],|_,_|0.0);assert!(evaluate(observation(),e,0).score.is_none());
        for fault in 0..5 {let mut o=observation();match fault {0=>o.after.handle=8,1=>o.after.focus=8,
            2=>o.after.alive=false,3=>o.before.in_world=false,_=>o.after.finite=false};
            assert!(evaluate(o,Evidence::capture(7,2,&[solve(7,-10.0)],|_,_|0.0),0).score.is_none());}
    }
    #[test] fn tail_edit_retry_eof_and_port_or_lag_cannot_reuse_a_peak() {
        let e=Evidence::capture(7,2,&[solve(7,-10.0)],|_,_|0.0);
        assert!(evaluate(observation(),e,83).score.is_none());
        for fault in 0..7 {let mut o=observation();match fault {0=>o.iteration_after=82,1=>o.allowed_before=false,
            2=>o.requested_mask_matches=false,3=>{o.iteration_before=299;o.iteration_after=300;},
            4=>o.port_abort=true,5=>o.site=29,_=>o.lagging=true};assert!(evaluate(o,e,0).score.is_none());}
    }
    #[test] fn original_pairmax_is_not_a_speed_bonus_but_can_recover() {
        let mut o=observation();o.site=2;
        assert!(evaluate(o,Evidence::default(),0).score.is_none());
        assert_eq!(evaluate(o,Evidence::capture(7,2,&[solve(7,-10.0)],|_,_|0.0),0).score.unwrap().tier,1);
    }
    #[test] fn ceiling_candidate_or_strand_or_other_after_is_not_a_solved_hit() {
        for tail in [None,Some(after(8)),Some(Event::Strand {toi:0.1,shape1:20,shape2:2,body1:7,body2:0,pos1:[0.0;2],pos2:[0.0;2]})] {
            let mut events=vec![solve(7,-10.0),toi()];if let Some(t)=tail {events.push(t);events.push(after(7));}
            let e=Evidence::capture(7,2,&events,|_,_|0.0);
            // An unrelated after does not complete, though a later primary after can.
            if matches!(tail,Some(Event::AfterToi{body:8,..})) {assert_eq!(e.solved_ceiling_tois,1);}
            else {assert_eq!(e.solved_ceiling_tois,0);}
        }
        let e=Evidence::capture(7,2,&[toi(),after(8)],|_,_|0.0);assert_eq!(e.solved_ceiling_tois,0);
    }
    #[test] fn actual_ceiling_and_live_callback_are_lexicographic_not_projection() {
        let upward=evaluate(observation(),Evidence::capture(7,2,&[solve(7,-1e9)],|_,_|1.0),0).score.unwrap();
        let ceiling=evaluate(observation(),Evidence::capture(7,2,&[toi(),after(7)],|_,_|1.0),0).score.unwrap();
        let fc=Event::Finish {x:1.0,y:2.0,inside:false,dead:false,other:20};
        let near=evaluate(observation(),Evidence::capture(7,2,&[toi(),after(7),fc],|_,_|2.0),0).score.unwrap();
        let far=evaluate(observation(),Evidence::capture(7,2,&[toi(),after(7),fc],|_,_|4.0),0).score.unwrap();
        assert!(ceiling.compare(upward).is_gt());assert!(near.compare(ceiling).is_gt());assert!(near.compare(far).is_gt());
    }
    #[test] fn invalid_toi_or_injected_or_nonfinite_solve_does_not_earn() {
        for t in [f64::NAN,-0.1,1.1] {
            let event=Event::Toi {toi:t,shape1:20,shape2:2,body1:7,body2:0};
            let e=Evidence::capture(7,2,&[event,after(7)],|_,_|0.0);assert_eq!(e.solved_ceiling_tois,0);
        }
        let mut e=Evidence::capture(7,2,&[solve(7,-100.0)],|_,_|0.0);e.solve.as_mut().unwrap().v[0]=f64::INFINITY;
        assert!(evaluate(observation(),e,0).score.is_none());
    }
    #[test] fn finish_precedes_dead_bmx_style_gates_and_injected_solve_is_rejected() {
        let mut o=observation();o.finished=true;o.after.alive=false;o.port_abort=true;
        let d=evaluate(o,Evidence::default(),0);assert!(d.observed_finish);assert!(d.score.is_none());
        let mut e=Evidence::capture(7,2,&[solve(7,-100.0)],|_,_|0.0);e.solve.as_mut().unwrap().injected=true;
        assert!(evaluate(observation(),e,0).score.is_none());
    }
    #[test] fn same_frame_death_retains_only_pre_live_launch_repair_dna() {
        let mut o=observation();o.frame_before=142;o.frame_after=143;
        o.iteration_before=142;o.iteration_after=143;o.after.alive=false;
        o.after.in_world=false;o.after.focus=73;
        let e=Evidence::capture(7,2,&[solve(7,-3992.0),toi()],|_,_|0.0);
        assert!(evaluate(o,e,0).score.is_none());
        let launch=repair_launch(o,e,128,128,1024.0).unwrap();
        assert_eq!(launch.primary,7);assert!(!launch.after.usable());
        assert_eq!(launch.solve.v[1].to_bits(),(-3992.0f64).to_bits());
        o.before.alive=false;assert!(repair_launch(o,e,0,128,1024.0).is_none());
        o.before.alive=true;o.before.focus=73;assert!(repair_launch(o,e,0,128,1024.0).is_none());
    }
    #[test] fn repair_archive_cannot_reuse_inherited_early_child_or_invalid_launches() {
        let e=Evidence::capture(7,2,&[solve(7,-2000.0)],|_,_|0.0);
        assert!(repair_launch(observation(),e,0,128,1024.0).is_none());
        assert!(repair_launch(observation(),e,83,1,1024.0).is_none());
        for fault in 0..8 {let mut o=observation();match fault {
            0=>o.iteration_after=o.iteration_before,1=>o.allowed_before=false,
            2=>o.requested_mask_matches=false,3=>{o.iteration_before=299;o.iteration_after=300;},
            4=>o.port_abort=true,5=>o.site=29,6=>o.lagging=true,_=>o.before.in_world=false};
            assert!(repair_launch(o,e,0,1,1024.0).is_none());
        }
        let child=Evidence::capture(7,2,&[solve(8,-1e9)],|_,_|0.0);
        assert!(repair_launch(observation(),child,0,1,1024.0).is_none());
        let mut invalid=e;invalid.solve.as_mut().unwrap().injected=true;
        assert!(repair_launch(observation(),invalid,0,1,1024.0).is_none());
        invalid=e;invalid.solve.as_mut().unwrap().v[0]=f64::NAN;
        assert!(repair_launch(observation(),invalid,0,1,1024.0).is_none());
    }
    #[test] fn repair_thresholds_do_not_change_finish_or_promote_candidates_to_solves() {
        let mut o=observation();o.finished=true;o.after.alive=false;
        let e=Evidence::capture(7,2,&[solve(7,-10.0),toi()],|_,_|0.0);
        let launch=repair_launch(o,e,0,1,1024.0).unwrap();
        assert_eq!(launch.evidence.solved_ceiling_tois,0);
        assert!(evaluate(o,e,0).observed_finish);
        o.port_abort=true;assert!(repair_launch(o,e,0,1,1024.0).is_none());
        assert!(evaluate(o,e,0).observed_finish);
    }
    #[test] fn repair_helpers_fail_closed_on_invalid_cutoffs_and_zero_speed() {
        let upward=Evidence::capture(7,2,&[solve(7,-2000.0),toi()],|_,_|0.0);
        for (frame,speed) in [(0,1024.0),(300,1024.0),(1,-1.0),(1,f64::NAN),(1,f64::INFINITY)] {
            assert!(repair_launch(observation(),upward,0,frame,speed).is_none());
        }
        for vy in [0.0,1.0] {
            let e=Evidence::capture(7,2,&[solve(7,vy)],|_,_|0.0);
            assert!(repair_launch(observation(),e,0,1,0.0).is_none());
            let candidate=Evidence::capture(7,2,&[solve(7,vy),toi()],|_,_|0.0);
            assert_eq!(repair_launch(observation(),candidate,0,1,1024.0).unwrap().evidence.solved_ceiling_tois,0);
        }
    }
}
