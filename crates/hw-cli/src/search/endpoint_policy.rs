//! Unscored input DNA selected by actual primary COM endpoints, never a finish.
//! Main is pre-Report; AfterToi is post-Report and can follow a Finish callback.
//! A point inside a box is geometry only. No segment, projection or alias earns
//! an endpoint, and same-frame after-death facts do not become living fitness.
use super::policy::{Observation, Primary, Solve, MAX_FRAME};
use hw_core::lab::Event;
use hw_core::{H, NONE};
use serde_json::{json, Value};
use std::cmp::Ordering;

pub const STRATA: usize = 4;
pub const STRONG_UPWARD_SPEED: f64 = 1024.0;
pub const STRATUM_NAMES: [&str; STRATA] = [
    "strong_main", "strong_or_verified_ceiling_after_toi", "weak_after_toi", "weak_main",
];

#[derive(Clone, Copy, Debug)]
pub struct EndpointTarget {
    /// Actual FinishLine bounds in [left, top, right, bottom] order.
    pub bounds: [f64; 4],
    pub distance: f64,
    pub strict_inside: bool,
}
impl EndpointTarget {
    pub fn from_bounds(c: [f64; 2], bounds: [f64; 4]) -> Option<Self> {
        if !c.into_iter().chain(bounds).all(f64::is_finite) { return None; }
        let [left, top, right, bottom] = bounds;
        if left >= right || top >= bottom { return None; }
        // Closed-box distance is zero on edges; the game's inside check is strict.
        let dx = (left - c[0]).max(0.0).max(c[0] - right);
        let dy = (top - c[1]).max(0.0).max(c[1] - bottom);
        let distance = (dx * dx + dy * dy).sqrt();
        if !distance.is_finite() { return None; }
        Some(Self { bounds, distance,
            strict_inside: c[0] > left && c[0] < right && c[1] > top && c[1] < bottom })
    }
    fn valid_for(self, c: [f64; 2]) -> bool {
        // Public fields cannot bypass finite geometry or supply an invented metric.
        Self::from_bounds(c, self.bounds).is_some_and(|expected|
            self.distance.to_bits() == expected.distance.to_bits()
                && self.strict_inside == expected.strict_inside)
    }
    pub fn value(self) -> Value { json!({"bounds":self.bounds,"bounds_bits":words(self.bounds),
        "distance":self.distance,"distance_bits":word(self.distance),
        "strict_inside_COM_at_sample":self.strict_inside,"geometry_only_not_finish":true}) }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum EndpointPhase { Main, AfterToi }
impl EndpointPhase {
    pub fn name(self) -> &'static str { match self { Self::Main => "Main", Self::AfterToi => "AfterToi" } }
}

/// Nearest preceding non-stranded TOI in event order, not force attribution.
/// The primary may be an island participant even when this seed pair excludes it.
#[derive(Clone, Copy, Debug)]
pub struct EndpointToi {
    pub event_ordinal: usize,
    pub toi: f64,
    pub shapes: [H; 2],
    pub bodies: [H; 2],
    pub includes_primary: bool,
    pub verified_world_ceiling_candidate: bool,
}
impl EndpointToi {
    pub fn value(self) -> Value { json!({"event_ordinal":self.event_ordinal,"toi":self.toi,
        "toi_bits":word(self.toi),"shapes":self.shapes,"bodies":self.bodies,
        "includes_captured_primary":self.includes_primary,
        "verified_world_ceiling_candidate":self.verified_world_ceiling_candidate,
        "ordered_seed_association_only_not_direct_impulse_attribution":true}) }
}

#[derive(Clone, Copy, Debug)]
pub struct EndpointMoment {
    pub bucket: usize,
    pub phase: EndpointPhase,
    pub event_ordinal: usize,
    pub frame: u32,
    pub iteration: u32,
    pub primary: H,
    pub observation: Observation,
    pub c0: [f64; 2],
    pub c: [f64; 2],
    pub v: [f64; 2],
    /// AfterToi does not record angular velocity. Never substitute a later value.
    pub w: Option<f64>,
    pub target: EndpointTarget,
    pub primary_main_solve: Option<Solve>,
    pub preceding_toi: Option<EndpointToi>,
    pub verified_world_ceiling_solve: bool,
}
impl EndpointMoment {
    /// Greater means preferred: smaller distance, then deterministic earlier keys.
    /// Different buckets retain separate quotas; this does not merge their capacity.
    pub fn compare(self, other: Self) -> Ordering {
        other.target.distance.total_cmp(&self.target.distance)
            .then_with(|| other.phase.cmp(&self.phase))
            .then_with(|| other.frame.cmp(&self.frame))
            .then_with(|| other.event_ordinal.cmp(&self.event_ordinal))
    }
    pub fn value(self) -> Value {
        let o = self.observation;
        let mut value = json!({"selection_class":"primary_endpoint_input_DNA","fitness":null,
            "bucket":self.bucket,"stratum":STRATUM_NAMES.get(self.bucket),
            "phase":self.phase.name(),"event_ordinal":self.event_ordinal,
            "frame":self.frame,"iteration":self.iteration,"captured_primary_handle":self.primary,
            "frame_before":o.frame_before,"frame_after":o.frame_after,
            "iteration_before":o.iteration_before,"iteration_after":o.iteration_after,
            "input_allowed_before":o.allowed_before,"input_allowed_after":o.allowed_after,
            "requested_mask_matches":o.requested_mask_matches,
            "site":o.site,"port_abort":o.port_abort,"lagging":o.lagging,
            "actual_finished_this_frame":o.finished});
        let endpoint = json!({"before":primary_value(o.before),"after":primary_value(o.after),
            "c0":self.c0,"c0_bits":words(self.c0),"c":self.c,"c_bits":words(self.c),
            "v":self.v,"v_bits":words(self.v),"w":self.w,"w_bits":self.w.map(word),
            "target":self.target.value(),"primary_main_solve":self.primary_main_solve.map(solve_value),
            "preceding_TOI":self.preceding_toi.map(EndpointToi::value),
            "verified_world_ceiling_solve":self.verified_world_ceiling_solve,
            "recorded_COM_endpoint_not_transform_or_segment":true,
            "callback_focus_identity_at_other_ordinals_verified":false,
            "survival_or_finish_claimed":false,"QE_or_pairmax_mechanism_verified":false,
            "original_player_verified":false,"selective_contact_omission_verified":false});
        if let (Value::Object(fields), Value::Object(endpoint)) = (&mut value, endpoint) {
            fields.extend(endpoint);
        }
        value
    }
}

fn word(x: f64) -> String { format!("{:016x}", x.to_bits()) }
fn words<const N: usize>(xs: [f64; N]) -> [String; N] { xs.map(word) }
fn primary_value(p: Primary) -> Value { json!({"handle":p.handle,"focus":p.focus,"alive":p.alive,
    "in_world":p.in_world,"finite_dynamic_unfrozen":p.finite,"usable":p.usable()}) }
fn solve_value(s: Solve) -> Value { json!({"body":s.body,"c0":s.c0,"c0_bits":words(s.c0),
    "c":s.c,"c_bits":words(s.c),"v":s.v,"v_bits":words(s.v),"w":s.w,"w_bits":word(s.w),
    "injected":s.injected}) }
fn finite_solve(s: Solve) -> bool {
    s.c0.into_iter().chain(s.c).chain(s.v).chain([s.w]).all(f64::is_finite)
}
fn eligible(o: Observation, changed: usize, min_frame: u32) -> bool {
    changed < MAX_FRAME as usize && (1..=MAX_FRAME).contains(&min_frame)
        && o.frame_before.checked_add(1) == Some(o.frame_after)
        && (min_frame..=MAX_FRAME).contains(&o.frame_after)
        && o.iteration_before as usize >= changed && o.iteration_before < MAX_FRAME
        && o.iteration_before.checked_add(1) == Some(o.iteration_after)
        && o.iteration_after <= MAX_FRAME && o.allowed_before && o.requested_mask_matches
        && o.before.usable() && !o.port_abort && !matches!(o.site, 9 | 10 | 29) && !o.lagging
}

/// Return the closest eligible actual record per stratum. Caller invokes this
/// only when enabled, from its own fresh step(0) event slice, and handles real
/// finishes first. `world_ceiling` is the caller's exact pre-step verified shape,
/// or NONE when unknown. No after-usability or world-coordinate clamp is imposed.
pub fn endpoint_moments(o: Observation, events: &[Event], earliest_changed_index: usize,
    min_frame: u32, world_ceiling: H,
    mut target: impl FnMut([f64; 2]) -> Option<EndpointTarget>) -> [Option<EndpointMoment>; STRATA] {
    let mut best: [Option<EndpointMoment>; STRATA] = [None; STRATA];
    // No target callback or metric calculation occurs for an ineligible frame.
    if !eligible(o, earliest_changed_index, min_frame) { return best; }
    let h = o.before.handle;
    if events.iter().any(|e| matches!(e, Event::Solve {body,injected:true,..} if *body==h)) { return best; }
    let mut main: Option<Solve> = None;
    let mut pending: Option<EndpointToi> = None;
    for (ordinal, &event) in events.iter().enumerate() {
        // Recheck each record before reduction; an old prefix endpoint never ranks
        // this child's changed tail. The observation belongs to this one step.
        if !eligible(o, earliest_changed_index, min_frame) { continue; }
        let endpoint = match event {
            Event::Solve {body,c0,c,v,w,injected} => {
                pending = None;
                if body != h { continue; }
                let s = Solve {body,c0,c,v,w,injected};
                main = finite_solve(s).then_some(s);
                if main.is_none() { continue; }
                Some((EndpointPhase::Main, c0, c, v, Some(w), false, None))
            }
            Event::Toi {toi,shape1,shape2,body1,body2} => {
                // Every new candidate supersedes the prior seed, even unrelated.
                pending = (toi.is_finite() && (0.0..=1.0).contains(&toi)).then_some(EndpointToi {
                    event_ordinal:ordinal,toi,shapes:[shape1,shape2],bodies:[body1,body2],
                    includes_primary:body1==h || body2==h,
                    verified_world_ceiling_candidate:world_ceiling!=NONE
                        && ((body1==h && shape2==world_ceiling) || (body2==h && shape1==world_ceiling)),
                });
                continue;
            }
            Event::Strand {..} => { pending = None; continue; }
            Event::AfterToi {body,c0,c,v} if body == h => {
                let seed = pending.take();
                if !c0.into_iter().chain(c).chain(v).all(f64::is_finite) { continue; }
                Some((EndpointPhase::AfterToi, c0, c, v, None,
                    seed.is_some_and(|s| s.verified_world_ceiling_candidate), seed))
            }
            _ => continue,
        };
        let Some((phase,c0,c,v,w,ceiling_solve,seed)) = endpoint else { continue; };
        let Some(t) = target(c).filter(|t| t.valid_for(c)) else { continue; };
        let strong = main.is_some_and(|s| -s.v[1] >= STRONG_UPWARD_SPEED);
        let bucket = match phase {
            EndpointPhase::Main => if strong {0} else {3},
            EndpointPhase::AfterToi => if strong || ceiling_solve {1} else {2},
        };
        let moment = EndpointMoment {bucket,phase,event_ordinal:ordinal,frame:o.frame_after,
            iteration:o.iteration_after,primary:h,observation:o,c0,c,v,w,target:t,
            primary_main_solve:main,preceding_toi:seed,verified_world_ceiling_solve:ceiling_solve};
        if best[bucket].map_or(true, |old| moment.compare(old).is_gt()) { best[bucket] = Some(moment); }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    const BOUNDS: [f64; 4] = [10.0, 10.0, 20.0, 20.0];
    fn observation() -> Observation {
        let p = Primary {handle:24,focus:24,alive:true,in_world:true,finite:true};
        Observation {before:p,after:p,frame_before:284,frame_after:285,iteration_before:284,
            iteration_after:285,allowed_before:true,allowed_after:true,requested_mask_matches:true,
            site:0,port_abort:false,lagging:false,finished:false}
    }
    fn main(c: [f64; 2], upward: f64) -> Event {
        Event::Solve {body:24,c0:[1.0,2.0],c,v:[3.0,-upward],w:4.0,injected:false}
    }
    fn after(c: [f64; 2]) -> Event { Event::AfterToi {body:24,c0:[5.0,6.0],c,v:[7.0,8.0]} }
    fn toi(ceiling: bool) -> Event { Event::Toi {toi:0.5,shape1:127,
        shape2:if ceiling {2} else {135},body1:24,body2:if ceiling {1} else {32}} }
    fn strand() -> Event { Event::Strand {toi:0.5,shape1:8,shape2:9,body1:8,body2:9,
        pos1:[0.0,0.0],pos2:[0.0,0.0]} }
    fn moments(o: Observation, events: &[Event], changed: usize) -> [Option<EndpointMoment>; STRATA] {
        endpoint_moments(o,events,changed,1,2,|c|EndpointTarget::from_bounds(c,BOUNDS))
    }
    fn none(xs: [Option<EndpointMoment>; STRATA]) -> bool { xs.into_iter().all(|x|x.is_none()) }

    #[test]
    fn strict_bounds_and_closed_distance_have_no_tolerance_or_finish_meaning() {
        let inside = EndpointTarget::from_bounds([15.0,15.0],BOUNDS).unwrap();
        assert!(inside.strict_inside);assert_eq!(inside.distance.to_bits(),0.0f64.to_bits());
        for c in [[10.0,15.0],[20.0,15.0],[15.0,10.0],[15.0,20.0],[10.0,10.0]] {
            let edge=EndpointTarget::from_bounds(c,BOUNDS).unwrap();
            assert!(!edge.strict_inside);assert_eq!(edge.distance.to_bits(),0.0f64.to_bits());
        }
        let just_inside=f64::from_bits(10.0f64.to_bits()+1);
        assert!(EndpointTarget::from_bounds([just_inside,15.0],BOUNDS).unwrap().strict_inside);
        let outside=EndpointTarget::from_bounds([23.0,24.0],BOUNDS).unwrap();
        assert_eq!(outside.distance.to_bits(),5.0f64.to_bits());assert!(!outside.strict_inside);
    }

    #[test]
    fn invalid_geometry_overflow_or_forged_target_does_not_enter_bank() {
        for bounds in [[20.0,10.0,10.0,20.0],[10.0,20.0,20.0,10.0],
            [10.0,10.0,10.0,20.0],[f64::NAN,10.0,20.0,20.0]] {
            assert!(EndpointTarget::from_bounds([15.0,15.0],bounds).is_none());
        }
        assert!(EndpointTarget::from_bounds([f64::INFINITY,15.0],BOUNDS).is_none());
        assert!(EndpointTarget::from_bounds([f64::MAX,15.0],BOUNDS).is_none());
        let events=[main([30.0,30.0],0.0)];
        assert!(none(endpoint_moments(observation(),&events,0,1,2,|_|Some(EndpointTarget {
            bounds:BOUNDS,distance:0.0,strict_inside:true}))));
    }

    #[test]
    fn invalid_advancement_or_prechange_never_calls_target() {
        let o=observation();let mut invalid=Vec::new();
        let mut v=o;v.frame_after=284;invalid.push(v);
        let mut v=o;v.frame_after=300;v.frame_before=299;invalid.push(v);
        let mut v=o;v.iteration_after=284;invalid.push(v);
        let mut v=o;v.iteration_after=300;v.iteration_before=299;invalid.push(v);
        let mut v=o;v.allowed_before=false;invalid.push(v);
        let mut v=o;v.requested_mask_matches=false;invalid.push(v);
        let mut v=o;v.port_abort=true;invalid.push(v);
        let mut v=o;v.lagging=true;invalid.push(v);
        for site in [9,10,29] {let mut v=o;v.site=site;invalid.push(v);}
        for v in invalid { assert!(none(endpoint_moments(v,&[main([15.0,15.0],2048.0)],0,1,2,
            |_|panic!("ineligible frame computed a target")))); }
        for (changed,min_frame) in [(285,1),(299,1),(usize::MAX,1),(0,0),(0,300),(0,286)] {
            assert!(none(endpoint_moments(o,&[main([15.0,15.0],2048.0)],changed,min_frame,2,
                |_|panic!("invalid cutoff computed a target"))));
        }
        assert!(moments(o,&[main([15.0,15.0],2048.0)],284)[0].is_some());
    }

    #[test]
    fn pre_identity_and_child_endpoints_are_rejected() {
        for field in 0..6 {let mut o=observation();match field {
            0=>o.before.handle=NONE,1=>o.before.focus=73,2=>o.before.alive=false,
            3=>o.before.in_world=false,4=>o.before.finite=false,_=>o.before.handle=73,
        }assert!(none(moments(o,&[main([15.0,15.0],2048.0)],0)));}
        let events=[Event::Solve {body:43,c0:[0.0;2],c:[15.0;2],v:[0.0,-1e9],w:0.0,injected:false},
            Event::AfterToi {body:43,c0:[0.0;2],c:[15.0;2],v:[0.0;2]}];
        assert!(none(moments(observation(),&events,0)));
    }

    #[test]
    fn fatal_frozen_removed_after_facts_remain_unscored_dna() {
        let mut o=observation();o.after=Primary {handle:73,focus:73,alive:false,in_world:false,finite:false};
        o.allowed_after=false;o.site=2;
        let m=moments(o,&[main([15.0,15.0],2048.0),Event::Freeze {body:24},Event::ChestSmash],284)[0].unwrap();
        assert!(!m.observation.after.usable());assert!(m.target.strict_inside);
        let value=m.value();assert!(value["fitness"].is_null());
        assert_eq!(value["survival_or_finish_claimed"],false);
        assert_eq!(value["actual_finished_this_frame"],false);
    }

    #[test]
    fn injected_primary_disqualifies_the_frame_before_metric_calls() {
        let injected=Event::Solve {body:24,c0:[0.0;2],c:[15.0;2],v:[0.0,-2048.0],w:0.0,injected:true};
        let events=[main([15.0,15.0],2048.0),after([15.0,15.0]),injected];
        assert!(none(endpoint_moments(observation(),&events,0,1,2,
            |_|panic!("injected frame computed a metric"))));
    }

    #[test]
    fn nonfinite_actual_records_are_not_endpoints_or_launch_strength() {
        for event in [main([f64::NAN,15.0],2048.0),
            Event::Solve {body:24,c0:[0.0;2],c:[15.0;2],v:[f64::INFINITY,0.0],w:0.0,injected:false},
            Event::Solve {body:24,c0:[0.0;2],c:[15.0;2],v:[0.0;2],w:f64::NAN,injected:false},
            Event::AfterToi {body:24,c0:[f64::NAN,0.0],c:[15.0;2],v:[0.0;2]}] {
            assert!(none(moments(observation(),&[event],0)));
        }
        let xs=moments(observation(),&[main([f64::NAN,15.0],1e9),after([15.0,15.0])],0);
        assert!(xs[0].is_none() && xs[1].is_none());assert!(xs[2].is_some());
    }

    #[test]
    fn phase_quotas_keep_large_main_and_actual_ceiling_after_toi_separate() {
        let strong=moments(observation(),&[main([30.0,30.0],1024.0),toi(false),after([15.0,15.0])],0);
        assert!(strong[0].is_some() && strong[1].is_some());assert!(strong[2].is_none() && strong[3].is_none());
        let weak=moments(observation(),&[main([15.0,15.0],1023.0),toi(false),after([15.0,15.0])],0);
        assert!(weak[2].is_some() && weak[3].is_some());assert!(weak[0].is_none() && weak[1].is_none());
        let ceiling=moments(observation(),&[main([30.0,30.0],0.0),toi(true),after([15.0,15.0])],0);
        assert!(ceiling[1].unwrap().verified_world_ceiling_solve);assert!(ceiling[3].is_some());
    }

    #[test]
    fn toi_supersession_strand_and_unrelated_after_cannot_invent_ceiling_solve() {
        assert!(none(moments(observation(),&[toi(true),strand()],0)));
        let unrelated=Event::Toi {toi:0.25,shape1:8,shape2:9,body1:8,body2:9};
        let xs=moments(observation(),&[toi(true),unrelated,after([15.0,15.0])],0);
        let m=xs[2].unwrap();assert!(!m.verified_world_ceiling_solve);
        let seed=m.preceding_toi.unwrap();assert_eq!(seed.event_ordinal,1);assert!(!seed.includes_primary);
        let m=moments(observation(),&[toi(true),strand(),after([15.0,15.0])],0)[2].unwrap();
        assert!(m.preceding_toi.is_none());assert!(!m.verified_world_ceiling_solve);
        let invalid=Event::Toi {toi:f64::NAN,shape1:127,shape2:2,body1:24,body2:1};
        assert!(!moments(observation(),&[toi(true),invalid,after([15.0,15.0])],0)[2].unwrap().verified_world_ceiling_solve);
        let child_after=Event::AfterToi {body:43,c0:[0.0;2],c:[15.0;2],v:[0.0;2]};
        let m=moments(observation(),&[toi(true),child_after,after([15.0,15.0])],0)[1].unwrap();
        assert_eq!(m.event_ordinal,2);assert_eq!(m.preceding_toi.unwrap().event_ordinal,0);
    }

    #[test]
    fn finish_before_after_toi_is_not_filtered_or_created_by_inside_point() {
        let mut o=observation();o.finished=true;o.after.alive=false;
        let events=[toi(true),Event::Finish {x:15.0,y:15.0,inside:true,dead:false,other:2},after([15.0,15.0])];
        let m=moments(o,&events,0)[1].unwrap();assert!(m.observation.finished);
        assert!(m.verified_world_ceiling_solve);assert_eq!(m.event_ordinal,2);
        let m=moments(observation(),&[after([15.0,15.0])],0)[2].unwrap();
        assert!(m.target.strict_inside);assert!(!m.observation.finished);
        assert!(none(moments(o,&[events[1]],0)));
    }

    #[test]
    fn only_discrete_c_is_ranked_and_exact_words_are_retained() {
        let event=Event::Solve {body:24,c0:[15.0,15.0],c:[100.0,-0.0],v:[-0.0,-2048.0],w:-0.0,injected:false};
        let m=moments(observation(),&[event],0)[0].unwrap();assert!(!m.target.strict_inside);
        assert!(m.target.distance>0.0);let value=m.value();
        assert_eq!(value["c_bits"][1],"8000000000000000");
        assert_eq!(value["v_bits"][0],"8000000000000000");
        assert_eq!(value["w_bits"],"8000000000000000");
        let m=moments(observation(),&[after([15.0,15.0])],0)[2].unwrap();
        assert!(m.value()["w"].is_null() && m.value()["w_bits"].is_null());
    }

    #[test]
    fn per_stratum_closest_and_ties_are_deterministic_without_weak_eviction() {
        let xs=moments(observation(),&[main([30.0,30.0],2048.0),main([25.0,20.0],2048.0),
            main([15.0,15.0],1.0),toi(false),after([15.0,15.0]),after([15.0,15.0])],0);
        assert_eq!(xs[0].unwrap().event_ordinal,1);assert_eq!(xs[3].unwrap().event_ordinal,2);
        assert_eq!(xs[2].unwrap().event_ordinal,4);
        assert_eq!(xs[0].unwrap().target.distance.to_bits(),5.0f64.to_bits());
        let a=xs[2].unwrap();let mut b=a;b.event_ordinal+=1;assert!(a.compare(b).is_gt());
        let mut b=a;b.frame+=1;assert!(a.compare(b).is_gt());
        assert!(a.compare(a).is_eq());
    }
}
