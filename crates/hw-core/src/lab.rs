//! Research instrumentation (feature `lab`, never a default). Read-only hooks inside
//! `world::step` record what happened to a set of watched bodies: position after the main
//! solve, each TOI impact, freezes, finish-line checks and the chest smash. An optional one-shot
//! injection overwrites one body's end-of-solve position and velocity ("cannon") so that the rest
//! of the unchanged engine shows where such a launch would end.
//!
//! State is thread-local, not part of `Ctx`, so simulation state stays plain words. Unarmed hooks
//! only read a flag; the `lab_hooks` test proves armed recording leaves every frame unchanged.
use crate::{Ctx, H, NONE};
use hw_real::Real;
use std::cell::RefCell;

/// Which unchanged island Report call delivered a damage result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReportPhase { None, Main, Toi }

/// Actual enclosing world call, distinct from the narrower island Report callback.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WorldPhase { Unknown, Collide, MainSolve, SolveToi }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProxyOperationKind { Unknown, CreateProxy, MoveProxy }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ProxyOperation {
    pub kind: ProxyOperationKind,
    pub initiating_proxy_id: Option<u32>,
}

/// A bounded proxy-slot copy. `valid` is only the slot's original IsValid flag,
/// not proof that CreateProxy completed or that the shape is still a world member.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PairProxySlot {
    pub readable: bool,
    pub valid: bool,
    pub shape: Option<H>,
}

/// The first rejected new AddPair tuple, copied before the unchanged Site2 return.
/// This is a global broadphase observation, not a watched-body physics event.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PairExhaustion {
    pub proxy_ids: [u32; 2],
    pub hash_bucket: u32,
    pub free_pair: u32,
    pub pair_count: i32,
    pub pair_buffer_count: i32,
    pub proxy_slots: [PairProxySlot; 2],
    pub operation: ProxyOperation,
    pub world_phase: WorldPhase,
    pub report_phase: ReportPhase,
    /// Boundary in the existing ordered event stream, not an additional event.
    pub events_before: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PairExhaustionCapture {
    pub first: Option<PairExhaustion>,
    pub count: u32,
    pub count_saturated: bool,
}

const EMPTY_PAIR_CAPTURE: PairExhaustionCapture = PairExhaustionCapture {
    first: None, count: 0, count_saturated: false,
};
const UNKNOWN_PROXY_OPERATION: ProxyOperation = ProxyOperation {
    kind: ProxyOperationKind::Unknown, initiating_proxy_id: None,
};

// Opt-in fixed target side channel. This deliberately is not an Event variant.
pub const TOI_TARGET_CAPACITY: usize = 128;
#[derive(Clone, Copy, Debug)]
pub struct ToiTarget { pub primary: H, pub chest_shape: H, pub ceiling_shape: H,
    pub before_commit: bool }
#[derive(Clone, Copy, Debug)]
pub enum ToiTargetStage { Initial, BeforeCommit, AfterCommit }
#[derive(Clone, Copy, Debug)]
pub enum ToiTargetBranch { SlowOrNonSolid, BothStaticOrSleeping, Cached, BeforeSweepAlignment, Computed,
    ReplacedMinimum, DidNotReplaceMinimum }
#[derive(Clone, Copy, Debug)]
pub struct ToiTargetBody { pub handle: H, pub alive: bool, pub retained: u32,
    pub flags: u32, pub body_type: i32,
    // Exact Real words: t0,c0.x,c0.y,c.x,c.y,a0,a,v.x,v.y,w.
    pub words: [[u32;2];10] }
#[derive(Clone, Copy, Debug)]
pub struct ToiTargetContact { pub handle: H, pub alive: bool, pub retained: u32,
    pub shapes: [H;2], pub flags: u32, pub manifold_count: i32, pub point_count: i32,
    pub toi_words: [u32;2] }
#[derive(Clone, Copy, Debug)]
pub struct ToiTargetSnapshot {
    pub shape_readable: [bool;2], pub shape_alive: [bool;2], pub shape_retained: [u32;2],
    pub shape_bodies: [Option<H>;2], pub bodies: [Option<ToiTargetBody>;2],
    pub proxies: [Option<u32>;2], pub proxy_readable: [bool;2],
    pub proxy_valid_slot: [bool;2], pub proxy_back_reference_matches: [bool;2],
    pub pair_lookup_complete: bool, pub pair_index: Option<u32>,
    pub pair_status: Option<u32>, pub pair_contact: Option<H>,
    pub contact: Option<ToiTargetContact>, pub contact_world_membership: Option<bool>,
    pub target_contact_membership_lookup_complete: bool, pub pairs: i32, pub buffered: i32, pub free: u32,
}
#[derive(Clone, Copy, Debug)]
pub enum ToiTargetRecord {
    Snapshot {scan:u32,events_before:usize,stage:ToiTargetStage,
        preceding_selected:H,selected_toi_words:[u32;2],value:ToiTargetSnapshot},
    Branch {scan:u32,events_before:usize,contact:ToiTargetContact,
        bodies:[Option<ToiTargetBody>;2],branch:ToiTargetBranch,
        tested_toi_words:Option<[u32;2]>,minimum_before_words:[u32;2]},
    ScanEnd {scan:u32,events_before:usize,selected:H,minimum_words:[u32;2],
        original_done_branch:bool},
    SelectedBeforeAdvance {scan:u32,events_before:usize,contact:ToiTargetContact,
        bodies:[Option<ToiTargetBody>;2],selected_toi_words:[u32;2]},
    Updated {scan:u32,events_before:usize,contact:ToiTargetContact,
        bodies:[Option<ToiTargetBody>;2]},
    PrimarySynchronized {scan:u32,events_before:usize,preceding_selected:H,
        selected_toi_words:[u32;2],in_range:bool,value:ToiTargetSnapshot},
}
#[derive(Clone, Copy, Debug)]
pub struct ToiTargetCapture {pub target:Option<ToiTarget>,
    pub records:[Option<ToiTargetRecord>;TOI_TARGET_CAPACITY],pub len:usize,
    pub attempted:u32,pub truncated:bool,pub count_saturated:bool,
    pub scan:u32,pub scan_saturated:bool,pub scan_open:bool,pub natural_toi_end:bool}
const EMPTY_TOI_TARGET:ToiTargetCapture=ToiTargetCapture {target:None,
    records:[None;TOI_TARGET_CAPACITY],len:0,attempted:0,truncated:false,
    count_saturated:false,scan:0,scan_saturated:false,scan_open:false,natural_toi_end:false};
impl ToiTargetCapture {
    // Capture completeness alone is necessary, never sufficient, for target absence.
    // Consumers must also check the relevant inventory and original frame outcome.
    pub fn records_complete(&self)->bool {
        self.target.is_some() && self.natural_toi_end && !self.truncated
            && !self.count_saturated && !self.scan_saturated && !self.scan_open
            && self.len<=TOI_TARGET_CAPACITY
    }
}
pub(crate) fn toi_target_enabled()->bool{toi_target_config().is_some()}

fn toi_target_config()->Option<ToiTarget>{
    if !armed(){return None;}
    TOI_TARGET.with(|p|p.borrow().target)
}
pub fn configure_toi_target(target:ToiTarget){
    if armed(){TOI_TARGET.with(|p|{let mut p=p.borrow_mut();
        *p=EMPTY_TOI_TARGET;p.target=Some(target);
    });}
}
pub fn take_toi_target()->ToiTargetCapture{
    TOI_TARGET.with(|p|core::mem::replace(&mut *p.borrow_mut(),EMPTY_TOI_TARGET))
}
fn target_event_boundary()->usize{LAB.with(|p|p.borrow().events.len())}
fn target_record(record:ToiTargetRecord){
    TOI_TARGET.with(|p|{let mut p=p.borrow_mut();if p.target.is_none(){return;}
        if p.attempted==u32::MAX{p.count_saturated=true;}else{p.attempted+=1;}
        if p.len<TOI_TARGET_CAPACITY{let i=p.len;p.records[i]=Some(record);p.len+=1;}
        else{p.truncated=true;}
    });
}
fn target_body<R:Real>(ctx:&Ctx<R>,h:H)->Option<ToiTargetBody>{
    if h==NONE{return None;}let w=&ctx.world;let b=w.bodies.items.get(h as usize)?;
    Some(ToiTargetBody {handle:h,alive:*w.bodies.alive.get(h as usize)?!=0,
        retained:*w.bodies.retained.get(h as usize)?,flags:b.flags,body_type:b.type_,
        words:[b.sweep.t0.to_words(),b.sweep.c0.x.to_words(),b.sweep.c0.y.to_words(),
            b.sweep.c.x.to_words(),b.sweep.c.y.to_words(),b.sweep.a0.to_words(),
            b.sweep.a.to_words(),b.linear_velocity.x.to_words(),
            b.linear_velocity.y.to_words(),b.angular_velocity.to_words()]})
}
fn target_contact<R:Real>(ctx:&Ctx<R>,h:H)->Option<ToiTargetContact>{
    if h==NONE || h==crate::b2::state::NULL_CONTACT{return None;}
    let w=&ctx.world;let c=w.contacts.items.get(h as usize)?;
    Some(ToiTargetContact {handle:h,alive:*w.contacts.alive.get(h as usize)?!=0,
        retained:*w.contacts.retained.get(h as usize)?,shapes:[c.shape1,c.shape2],
        flags:c.flags,manifold_count:c.manifold_count,point_count:c.manifold.point_count,
        toi_words:c.toi.to_words()})
}
fn matches_target(t:ToiTarget,c:ToiTargetContact)->bool{
    c.shapes==[t.chest_shape,t.ceiling_shape] || c.shapes==[t.ceiling_shape,t.chest_shape]
}
fn target_snapshot<R:Real>(ctx:&Ctx<R>,t:ToiTarget)->ToiTargetSnapshot{
    use crate::b2::state::{B2_INVALID,B2_NULL_PAIR,B2_TABLE_MASK};
    let w=&ctx.world;let bp=&w.broad_phase;
    let mut s=ToiTargetSnapshot {shape_readable:[false;2],shape_alive:[false;2],
        shape_retained:[0;2],shape_bodies:[None;2],bodies:[None;2],proxies:[None;2],
        proxy_readable:[false;2],proxy_valid_slot:[false;2],
        proxy_back_reference_matches:[false;2],pair_lookup_complete:false,
        pair_index:None,pair_status:None,pair_contact:None,contact:None,
        contact_world_membership:None,target_contact_membership_lookup_complete:false,
        pairs:bp.pair_count,buffered:bp.pair_buffer_count,free:bp.free_pair};
    for (i,h) in [t.chest_shape,t.ceiling_shape].into_iter().enumerate(){
        if h==NONE{continue;}let Some(sh)=w.shapes.items.get(h as usize) else{continue;};
        let (Some(&alive),Some(&retained))=(w.shapes.alive.get(h as usize),w.shapes.retained.get(h as usize)) else{continue;};
        s.shape_readable[i]=true;s.shape_alive[i]=alive!=0;
        s.shape_retained[i]=retained;s.shape_bodies[i]=Some(sh.body);
        s.bodies[i]=target_body(ctx,sh.body);s.proxies[i]=Some(sh.proxy_id);
        if let Some(p)=bp.proxy_pool.get(sh.proxy_id as usize){
            s.proxy_readable[i]=true;s.proxy_valid_slot[i]=p.overlap_count!=B2_INVALID;
            s.proxy_back_reference_matches[i]=p.shape==h;
        }
    }
    if !s.proxy_valid_slot.iter().all(|&x|x)
        || !s.proxy_back_reference_matches.iter().all(|&x|x){return s;}
    let Some(a)=s.proxies[0] else{return s;};let Some(b)=s.proxies[1] else{return s;};
    let ids=if a<=b{[a,b]}else{[b,a]};
    let bucket=crate::b2::broad_phase::hash(ids[0],ids[1])&B2_TABLE_MASK;
    let Some(&head)=bp.hash_table.get(bucket as usize) else{return s;};
    let mut index=head;
    // Only copied integer slots: no Find/Query/Commit or mutation. A malformed chain
    // reaching the fixed pair capacity without a match/terminator is incomplete.
    for _ in 0..bp.pairs.len(){
        if index==B2_NULL_PAIR{s.pair_lookup_complete=true;return s;}
        let Some(p)=bp.pairs.get(index as usize) else{return s;};
        if [p.proxy_id1,p.proxy_id2]==ids{
            s.pair_lookup_complete=true;s.pair_index=Some(index);s.pair_status=Some(p.status);
            s.pair_contact=Some(p.contact);s.contact=target_contact(ctx,p.contact);
            let mut current=w.contact_list;
            for _ in 0..w.contacts.items.len(){
                if current==NONE{s.target_contact_membership_lookup_complete=true;s.contact_world_membership=Some(false);return s;}
                let Some(c)=w.contacts.items.get(current as usize) else{return s;};
                if current==p.contact{s.target_contact_membership_lookup_complete=true;s.contact_world_membership=Some(true);return s;}
                current=c.next;
            }
            return s;
        }
        index=p.next;
    }
    s
}
pub(crate) fn toi_target_snapshot<R:Real>(ctx:&Ctx<R>,stage:ToiTargetStage,
    selected:H,toi:R){
    let Some(t)=toi_target_config() else{return;};
    if matches!(stage,ToiTargetStage::BeforeCommit) && !t.before_commit{return;}
    let scan=TOI_TARGET.with(|p|p.borrow().scan);
    target_record(ToiTargetRecord::Snapshot {scan,events_before:target_event_boundary(),
        stage,preceding_selected:selected,selected_toi_words:toi.to_words(),
        value:target_snapshot(ctx,t)});
}
pub(crate) fn toi_target_scan_begin(){
    if toi_target_config().is_none(){return;}
    TOI_TARGET.with(|p|{let mut p=p.borrow_mut();
        if p.scan==u32::MAX{p.scan_saturated=true;}else{p.scan+=1;}
        p.scan_open=true;
    });
}
pub(crate) fn toi_target_branch<R:Real>(ctx:&Ctx<R>,c:H,branch:ToiTargetBranch,
    toi:Option<R>,minimum:R){
    let Some(t)=toi_target_config() else{return;};
    let Some(contact)=target_contact(ctx,c) else{return;};if !matches_target(t,contact){return;}
    let bodies=[t.chest_shape,t.ceiling_shape].map(|h|
        ctx.world.shapes.items.get(h as usize).and_then(|s|target_body(ctx,s.body)));
    let scan=TOI_TARGET.with(|p|p.borrow().scan);
    target_record(ToiTargetRecord::Branch {scan,events_before:target_event_boundary(),
        contact,bodies,branch,tested_toi_words:toi.map(|v|v.to_words()),
        minimum_before_words:minimum.to_words()});
}
pub(crate) fn toi_target_scan_end<R:Real>(selected:H,minimum:R,done:bool){
    if toi_target_config().is_none(){return;}
    let scan=TOI_TARGET.with(|p|{let mut p=p.borrow_mut();p.scan_open=false;
        if done{p.natural_toi_end=true;}p.scan});
    target_record(ToiTargetRecord::ScanEnd {scan,events_before:target_event_boundary(),
        selected,minimum_words:minimum.to_words(),original_done_branch:done});
}
pub(crate) fn toi_target_selected_before_advance<R:Real>(ctx:&Ctx<R>,c:H,toi:R){
    let Some(t)=toi_target_config() else{return;};
    let Some(contact)=target_contact(ctx,c) else{return;};if !matches_target(t,contact){return;}
    let bodies=[t.chest_shape,t.ceiling_shape].map(|h|
        ctx.world.shapes.items.get(h as usize).and_then(|s|target_body(ctx,s.body)));
    let scan=TOI_TARGET.with(|p|p.borrow().scan);
    target_record(ToiTargetRecord::SelectedBeforeAdvance {scan,events_before:target_event_boundary(),
        contact,bodies,selected_toi_words:toi.to_words()});
}
pub(crate) fn toi_target_updated<R:Real>(ctx:&Ctx<R>,c:H){
    let Some(t)=toi_target_config() else{return;};
    let Some(contact)=target_contact(ctx,c) else{return;};if !matches_target(t,contact){return;}
    let bodies=[t.chest_shape,t.ceiling_shape].map(|h|
        ctx.world.shapes.items.get(h as usize).and_then(|s|target_body(ctx,s.body)));
    let scan=TOI_TARGET.with(|p|p.borrow().scan);
    target_record(ToiTargetRecord::Updated {scan,events_before:target_event_boundary(),contact,bodies});
}
pub(crate) fn toi_target_primary_synchronized<R:Real>(ctx:&Ctx<R>,body:H,
    selected:H,toi:R,in_range:bool){
    let Some(t)=toi_target_config() else{return;};if body!=t.primary{return;}
    let scan=TOI_TARGET.with(|p|p.borrow().scan);
    target_record(ToiTargetRecord::PrimarySynchronized {scan,events_before:target_event_boundary(),
        preceding_selected:selected,selected_toi_words:toi.to_words(),in_range,
        value:target_snapshot(ctx,t)});
}

/// A copied contact dictionary entry. Resolved handles are readable arena records, not
/// evidence that the corresponding body is still a member of the world list.
#[derive(Clone, Copy, Debug)]
pub struct DamageBuffer {
    pub present: bool,
    pub shape: H,
    pub other: H,
    pub impulse: f64,
    pub own_body: Option<H>,
    pub other_body: Option<H>,
}

#[derive(Clone, Copy, Debug)]
pub enum Event {
    /// Main solve, after velocity/position constraints and before `Report`.
    Solve { body: H, c0: [f64; 2], c: [f64; 2], v: [f64; 2], w: f64, injected: bool },
    /// A TOI event about to be solved (toi in fractions of the step).
    Toi { toi: f64, shape1: H, shape2: H, body1: H, body2: H },
    /// A watched body after the TOI island solve.
    /// `c0` is the impact point (the sweep start of the sub-step), `c` the report position.
    AfterToi { body: H, c0: [f64; 2], c: [f64; 2], v: [f64; 2] },
    Freeze { body: H },
    /// FinishLine RESULT handler: focus centre and whether it counted.
    Finish { x: f64, y: f64, inside: bool, dead: bool, other: H },
    ChestSmash,
    /// A TOI whose contact update gave no manifold: the body is parked at the TOI point for the
    /// rest of the step (no solve, no report). Later cached TOIs for it are stale.
    Strand { toi: f64, shape1: H, shape2: H, body1: H, body2: H, pos1: [f64; 2], pos2: [f64; 2] },
    /// Incoming primary CHEST result before either original strict comparison.
    DamageResult { phase: ReportPhase, body: H, actor_kind: u32, child: bool, dead: bool,
        shape: H, own: H, other: H, own_body: Option<H>, other_body: Option<H>,
        impulse: f64, threshold: f64, previous: DamageBuffer },
    /// Primary CHEST dictionary entry before helmet/head consumption can mutate it.
    DamageSnapshot { body: H, actor_kind: u32, child: bool, dead: bool, shape: H,
        threshold: f64, smash_limit: f64, buffer: DamageBuffer },
    /// Exact local CHEST result immediately before its original dispatch.
    DamageConsume { body: H, actor_kind: u32, child: bool, dead: bool, shape: H,
        threshold: f64, smash_limit: f64, buffer: DamageBuffer },
}

#[derive(Clone, Copy, Debug)]
pub struct Injection {
    pub body: H,
    /// Research only: also move the start of the body's sweep (a virtual launch origin).
    pub origin: Option<[f64; 2]>,
    /// New `sweep.c` relative to `sweep.c0` of the main solve.
    pub displacement: [f64; 2],
    pub velocity: [f64; 2],
    /// Research only: overwrite the angular velocity too.
    pub angular: Option<f64>,
    /// Research only: set `sweep.a = sweep.a0 + da`. TOI reads the swept angle, so a launch's
    /// spin must be set here as well as in `angular`.
    pub angle_delta: Option<f64>,
}

#[derive(Default)]
struct Lab {
    armed: bool,
    watch: Vec<H>,
    events: Vec<Event>,
    injection: Option<Injection>,
}

thread_local! {
    static LAB: RefCell<Lab> = RefCell::new(Lab::default());
    static REPORT_PHASE: core::cell::Cell<ReportPhase> = const { core::cell::Cell::new(ReportPhase::None) };
    static WORLD_PHASE: core::cell::Cell<WorldPhase> = const { core::cell::Cell::new(WorldPhase::Unknown) };
    static PROXY_OPERATION: core::cell::Cell<ProxyOperation> = const { core::cell::Cell::new(UNKNOWN_PROXY_OPERATION) };
    static PAIR_EXHAUSTION: core::cell::Cell<PairExhaustionCapture> = const { core::cell::Cell::new(EMPTY_PAIR_CAPTURE) };
    static TOI_TARGET: RefCell<ToiTargetCapture> = const { RefCell::new(EMPTY_TOI_TARGET) };
    /// Search-only hang guard: wall-clock deadline for the current frame (see `over_budget`).
    static DEADLINE: core::cell::Cell<Option<std::time::Instant>> = const { core::cell::Cell::new(None) };
}

/// Restores the caller's phase even when an original Report callback aborts.
pub(crate) struct ReportPhaseGuard { previous: ReportPhase }
impl Drop for ReportPhaseGuard {
    fn drop(&mut self) { REPORT_PHASE.with(|p| p.set(self.previous)); }
}
pub(crate) fn report_phase(phase: ReportPhase) -> ReportPhaseGuard {
    let previous = REPORT_PHASE.with(|p| p.replace(phase));
    ReportPhaseGuard { previous }
}
pub(crate) fn current_report_phase() -> ReportPhase { REPORT_PHASE.with(|p| p.get()) }

pub(crate) struct WorldPhaseGuard { previous: Option<WorldPhase> }
impl Drop for WorldPhaseGuard {
    fn drop(&mut self) {
        if let Some(previous) = self.previous { WORLD_PHASE.with(|p| p.set(previous)); }
    }
}
pub(crate) fn world_phase(phase: WorldPhase) -> WorldPhaseGuard {
    let previous = if armed() { Some(WORLD_PHASE.with(|p| p.replace(phase))) } else { None };
    WorldPhaseGuard { previous }
}

pub(crate) struct ProxyOperationGuard { previous: Option<ProxyOperation> }
impl Drop for ProxyOperationGuard {
    fn drop(&mut self) {
        if let Some(previous) = self.previous { PROXY_OPERATION.with(|p| p.set(previous)); }
    }
}
pub(crate) fn proxy_operation(kind: ProxyOperationKind, initiating_proxy_id: Option<u32>) -> ProxyOperationGuard {
    let previous = if armed() {
        Some(PROXY_OPERATION.with(|p| p.replace(ProxyOperation { kind, initiating_proxy_id })))
    } else { None };
    ProxyOperationGuard { previous }
}

pub(crate) fn pair_proxy_slot(proxies: &[crate::b2::state::Proxy], id: u32) -> PairProxySlot {
    match proxies.get(id as usize) {
        Some(proxy) => PairProxySlot {
            readable: true,
            valid: proxy.overlap_count != crate::b2::state::B2_INVALID,
            shape: Some(proxy.shape),
        },
        None => PairProxySlot { readable: false, valid: false, shape: None },
    }
}

/// Fixed-size capture only: no Event push, allocation, lookup or arena traversal.
pub(crate) fn record_pair_exhaustion(proxy_ids: [u32; 2], hash_bucket: u32, free_pair: u32,
    pair_count: i32, pair_buffer_count: i32, proxy_slots: [PairProxySlot; 2]) {
    let events_before = LAB.with(|l| {
        let l = l.borrow();
        if l.armed { Some(l.events.len()) } else { None }
    });
    let Some(events_before) = events_before else { return; };
    PAIR_EXHAUSTION.with(|p| {
        let mut capture = p.get();
        if capture.count == u32::MAX { capture.count_saturated = true; }
        else { capture.count += 1; }
        if capture.first.is_none() {
            capture.first = Some(PairExhaustion {
                proxy_ids, hash_bucket, free_pair, pair_count, pair_buffer_count, proxy_slots,
                operation: PROXY_OPERATION.with(|p| p.get()),
                world_phase: WORLD_PHASE.with(|p| p.get()),
                report_phase: current_report_phase(), events_before,
            });
        }
        p.set(capture);
    });
}

/// Drain once after the original step (including Abort) and disarm. Disarm and
/// take_events deliberately preserve this slot; the next arm starts a new frame.
pub fn take_pair_exhaustion() -> PairExhaustionCapture {
    PAIR_EXHAUSTION.with(|p| p.replace(EMPTY_PAIR_CAPTURE))
}

/// Arm (Some) or clear (None) a per-frame deadline. A runaway QE can keep `SolveTOI` finding
/// new impacts for minutes in one frame; searches abort such frames as port-only aborts.
pub fn set_deadline(deadline: Option<std::time::Instant>) {
    DEADLINE.with(|d| d.set(deadline));
}

/// True once the armed deadline has passed (never without one).
#[inline]
pub fn over_budget() -> bool {
    DEADLINE.with(|d| d.get().is_some_and(|t| std::time::Instant::now() > t))
}

/// Start recording the given bodies (clears earlier events).
pub fn arm(watch: &[H]) {
    // No fixed-array reset on the default unconfigured path. A prior optional
    // capture cannot leak into a subsequent arm even if its caller did not drain.
    TOI_TARGET.with(|p|{let mut p=p.borrow_mut();if p.target.is_some(){*p=EMPTY_TOI_TARGET;}});
    PAIR_EXHAUSTION.with(|p| p.set(EMPTY_PAIR_CAPTURE));
    WORLD_PHASE.with(|p| p.set(WorldPhase::Unknown));
    PROXY_OPERATION.with(|p| p.set(UNKNOWN_PROXY_OPERATION));
    LAB.with(|l| {
        let mut l = l.borrow_mut();
        l.armed = true;
        l.watch.clear();
        l.watch.extend_from_slice(watch);
        l.events.clear();
    });
}

pub fn disarm() {
    LAB.with(|l| {
        let mut l = l.borrow_mut();
        l.armed = false;
        l.injection = None;
    });
}

/// Replace a body's end-of-solve state once, in the next main solve that contains it.
pub fn inject(injection: Injection) {
    LAB.with(|l| l.borrow_mut().injection = Some(injection));
}

pub fn take_events() -> Vec<Event> {
    LAB.with(|l| core::mem::take(&mut l.borrow_mut().events))
}

#[inline]
pub fn armed() -> bool {
    LAB.with(|l| l.borrow().armed)
}

pub(crate) fn watched(body: H) -> bool {
    LAB.with(|l| {
        let l = l.borrow();
        l.armed && l.watch.contains(&body)
    })
}

pub(crate) fn record(event: Event) {
    LAB.with(|l| {
        let mut l = l.borrow_mut();
        if l.armed {
            l.events.push(event);
        }
    });
}

pub(crate) fn take_injection(body: H) -> Option<Injection> {
    LAB.with(|l| {
        let mut l = l.borrow_mut();
        match l.injection {
            Some(i) if l.armed && i.body == body => l.injection.take(),
            _ => None,
        }
    })
}

#[inline]
pub(crate) fn f<R: Real>(r: R) -> f64 {
    let [hi, lo] = r.to_words();
    f64::from_bits(((hi as u64) << 32) | lo as u64)
}

#[inline]
pub(crate) fn r<R: Real>(v: f64) -> R {
    R::from_words(hw_real::words_of_f64(v))
}

#[cfg(test)]
mod toi_target_tests {
    use super::*;
    fn fixed_test(run:impl FnOnce()+Send+'static){
        std::thread::Builder::new().stack_size(64<<20).spawn(run).unwrap().join().unwrap();
    }
    fn target()->ToiTarget{ToiTarget {primary:24,chest_shape:127,ceiling_shape:2,before_commit:true}}
    #[test]
    fn target_capture_lifecycle_preserves_events_and_drains_once(){fixed_test(||{
        arm(&[24]);configure_toi_target(target());record(Event::Freeze {body:24});
        toi_target_scan_begin();toi_target_scan_end::<f64>(NONE,1.0,true);disarm();
        let events=take_events();assert_eq!(events.len(),1);
        assert!(matches!(events[0],Event::Freeze {body:24}));
        let c=take_toi_target();assert_eq!(c.len,1);assert_eq!(c.attempted,1);assert!(c.records_complete());
        assert!(matches!(c.records[0],Some(ToiTargetRecord::ScanEnd {events_before:1,..})));
        let drained=take_toi_target();assert!(drained.target.is_none());assert_eq!(drained.len,0);
        arm(&[]);configure_toi_target(target());toi_target_scan_begin();
        arm(&[]);toi_target_scan_begin();toi_target_scan_end::<f64>(NONE,1.0,true);disarm();
        let c=take_toi_target();assert!(c.target.is_none());assert_eq!(c.len,0);assert!(!c.records_complete());
        assert!(take_events().is_empty());
    });}
    #[test]
    fn target_capture_truncation_saturation_and_interruption_cannot_certify_absence(){fixed_test(||{
        arm(&[]);configure_toi_target(target());
        for i in 0..=TOI_TARGET_CAPACITY {toi_target_scan_begin();toi_target_scan_end::<f64>(NONE,1.0,i==TOI_TARGET_CAPACITY);}
        disarm();let c=take_toi_target();assert_eq!(c.len,TOI_TARGET_CAPACITY);
        assert_eq!(c.attempted,TOI_TARGET_CAPACITY as u32+1);assert!(c.truncated && c.natural_toi_end);
        assert!(!c.records_complete());assert!(take_events().is_empty());
        arm(&[]);configure_toi_target(target());toi_target_scan_begin();
        toi_target_scan_end::<f64>(9,0.25,false);toi_target_scan_begin();disarm();
        let c=take_toi_target();assert!(c.scan_open && !c.natural_toi_end);assert!(!c.records_complete());
        arm(&[]);configure_toi_target(target());TOI_TARGET.with(|p|{let mut p=p.borrow_mut();p.attempted=u32::MAX;p.scan=u32::MAX;});
        toi_target_scan_begin();toi_target_scan_end::<f64>(NONE,1.0,true);disarm();
        let c=take_toi_target();assert!(c.count_saturated && c.scan_saturated);assert!(!c.records_complete());
    });}
    #[test]
    fn target_snapshot_is_read_only_and_invalid_chains_remain_unknown(){fixed_test(||{
        use crate::b2::state::{Body,Shape,B2_NULL_PAIR,B2_TABLE_MASK};
        let mut ctx=Box::new(Ctx::<f64>::new());let body=ctx.world.bodies.alloc(Body::blank());
        let a=ctx.world.shapes.alloc(Shape::blank());let b=ctx.world.shapes.alloc(Shape::blank());
        for (shape,id) in [(a,0),(b,1)] {ctx.world.shape_mut(shape).body=body;
            ctx.world.shape_mut(shape).proxy_id=id;ctx.world.broad_phase.proxy_pool[id as usize].shape=shape;
            ctx.world.broad_phase.proxy_pool[id as usize].overlap_count=0;}
        let t=ToiTarget {primary:body,chest_shape:a,ceiling_shape:b,before_commit:true};
        let bucket=(crate::b2::broad_phase::hash(0,1)&B2_TABLE_MASK) as usize;
        ctx.world.broad_phase.hash_table[bucket]=0;
        let pair=&mut ctx.world.broad_phase.pairs[0];pair.proxy_id1=3;pair.proxy_id2=4;pair.next=0;
        let bp=crate::diagnostic::broadphase_string(&ctx);let physics=crate::physics_observer::physics_string(&ctx);
        let s=target_snapshot(&ctx,t);assert!(!s.pair_lookup_complete);assert!(s.pair_index.is_none());
        assert_eq!(bp,crate::diagnostic::broadphase_string(&ctx));assert_eq!(physics,crate::physics_observer::physics_string(&ctx));
        ctx.world.broad_phase.pairs[0].next=ctx.world.broad_phase.pairs.len() as u32;
        assert!(!target_snapshot(&ctx,t).pair_lookup_complete);
        ctx.world.broad_phase.hash_table[bucket]=B2_NULL_PAIR;
        let s=target_snapshot(&ctx,t);assert!(s.pair_lookup_complete && s.pair_index.is_none());
        assert!(s.contact_world_membership.is_none());
        arm(&[]);configure_toi_target(t);toi_target_scan_begin();
        toi_target_primary_synchronized(&ctx,body,NONE,0.5,false);disarm();
        let c=take_toi_target();assert!(matches!(c.records[0],Some(ToiTargetRecord::PrimarySynchronized {in_range:false,..})));
        assert!(!c.records_complete());
    });}
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn report_phase_restores_after_nested_early_return() {
        fn aborted_callback() -> crate::Flow {
            let _phase = report_phase(ReportPhase::Toi);
            assert_eq!(current_report_phase(), ReportPhase::Toi);
            crate::Flow::Abort(crate::AbortSite::GameNullDeref)
        }
        assert_eq!(current_report_phase(), ReportPhase::None);
        {
            let _phase = report_phase(ReportPhase::Main);
            assert_eq!(aborted_callback(), crate::Flow::Abort(crate::AbortSite::GameNullDeref));
            assert_eq!(current_report_phase(), ReportPhase::Main);
        }
        assert_eq!(current_report_phase(), ReportPhase::None);
    }

    fn pair_failure(ids: [u32; 2]) {
        let slots = [PairProxySlot { readable: false, valid: false, shape: None }; 2];
        record_pair_exhaustion(ids, 7, 65535, 4096, 12, slots);
    }

    #[test]
    fn pair_capture_is_global_first_only_and_survives_disarm_and_event_drain() {
        arm(&[]);
        record(Event::ChestSmash);
        pair_failure([1, 2]);
        pair_failure([3, 4]);
        disarm();
        assert_eq!(take_events().len(), 1);
        let capture = take_pair_exhaustion();
        assert_eq!(capture.count, 2);
        assert!(!capture.count_saturated);
        let first = capture.first.unwrap();
        assert_eq!(first.proxy_ids, [1, 2]);
        assert_eq!(first.events_before, 1);
        assert_eq!(take_pair_exhaustion(), EMPTY_PAIR_CAPTURE);
        pair_failure([5, 6]);
        assert_eq!(take_pair_exhaustion(), EMPTY_PAIR_CAPTURE);
    }

    #[test]
    fn pair_capture_next_arm_clears_undrained_record_and_count_saturates() {
        arm(&[]);
        pair_failure([1, 2]);
        disarm();
        arm(&[]);
        assert_eq!(take_pair_exhaustion(), EMPTY_PAIR_CAPTURE);
        pair_failure([3, 4]);
        PAIR_EXHAUSTION.with(|p| {
            let mut capture = p.get(); capture.count = u32::MAX; p.set(capture);
        });
        pair_failure([5, 6]);
        disarm();
        let capture = take_pair_exhaustion();
        assert_eq!(capture.count, u32::MAX);
        assert!(capture.count_saturated);
        assert_eq!(capture.first.unwrap().proxy_ids, [3, 4]);
    }

    #[test]
    fn pair_proxy_slot_bounds_and_validity_are_not_arena_membership() {
        use crate::b2::state::{Proxy, B2_INVALID};
        let slots = [Proxy { lower_bounds: [0, 0], upper_bounds: [0, 0],
            overlap_count: B2_INVALID, time_stamp: 0, shape: 7 }];
        assert_eq!(pair_proxy_slot(&slots, 0), PairProxySlot {
            readable: true, valid: false, shape: Some(7),
        });
        assert_eq!(pair_proxy_slot(&slots, u32::MAX), PairProxySlot {
            readable: false, valid: false, shape: None,
        });
        assert_eq!(pair_proxy_slot(&[], 0).shape, None);
    }

    #[test]
    fn pair_context_guards_restore_after_nested_abort() {
        fn aborted_operation() -> crate::Flow {
            let _world = world_phase(WorldPhase::SolveToi);
            let _proxy = proxy_operation(ProxyOperationKind::CreateProxy, Some(9));
            pair_failure([1, 2]);
            crate::Flow::Abort(crate::AbortSite::PairPoolExhausted)
        }
        arm(&[]);
        {
            let _world = world_phase(WorldPhase::MainSolve);
            let _proxy = proxy_operation(ProxyOperationKind::MoveProxy, Some(8));
            assert_eq!(aborted_operation(), crate::Flow::Abort(crate::AbortSite::PairPoolExhausted));
            assert_eq!(WORLD_PHASE.with(|p| p.get()), WorldPhase::MainSolve);
            assert_eq!(PROXY_OPERATION.with(|p| p.get()).initiating_proxy_id, Some(8));
        }
        assert_eq!(WORLD_PHASE.with(|p| p.get()), WorldPhase::Unknown);
        assert_eq!(PROXY_OPERATION.with(|p| p.get()), UNKNOWN_PROXY_OPERATION);
        disarm();
        let first = take_pair_exhaustion().first.unwrap();
        assert_eq!(first.world_phase, WorldPhase::SolveToi);
        assert_eq!(first.operation.kind, ProxyOperationKind::CreateProxy);
        assert_eq!(first.operation.initiating_proxy_id, Some(9));
    }
}
