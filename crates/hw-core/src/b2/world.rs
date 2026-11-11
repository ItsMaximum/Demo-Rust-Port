//! Dynamics/b2World.as + b2TimeStep.as, ported statement by statement.
//!
//! Ignored members: m_destructionListener, m_boundaryListener (Violation is a no-op), m_debugDraw
//! (DrawDebugData is a no-op), the allocators. `m_lock` latches on any abort (no try/finally).
use hw_real::Real;

use super::body;
use super::broad_phase;
use super::collision;
use super::contacts;
use super::island;
use super::joints;
use super::math::Vec2;
use super::settings;
use super::shapes;
use super::state::*;
use crate::flow::{AbortSite, Flow};
use crate::{flow, Ctx, H, NONE};

/// `new b2World(worldAABB, gravity, doSleep)`.
pub fn new_world<R: Real>(ctx: &mut Ctx<R>, lower: Vec2<R>, upper: Vec2<R>, gravity: Vec2<R>, do_sleep: bool) {
    // m_destructionListener = null; m_boundaryListener = null; m_contactFilter = default;
    // m_contactListener = null; m_debugDraw = null;
    ctx.world.body_list = NONE;
    ctx.world.contact_list = NONE;
    ctx.world.joint_list = NONE;
    ctx.world.body_count = 0;
    ctx.world.contact_count = 0;
    ctx.world.joint_count = 0;
    ctx.world.position_correction = crate::Flag32::TRUE;
    ctx.world.warm_starting = crate::Flag32::TRUE;
    ctx.world.continuous_physics = crate::Flag32::TRUE;
    ctx.world.allow_sleep = crate::Flag32::new(do_sleep);
    ctx.world.gravity = gravity;
    ctx.world.lock = crate::Flag32::FALSE;
    ctx.world.inv_dt0 = R::zero();
    // m_contactManager: new b2ContactManager() -> m_world = null, m_destroyImmediate = false
    ctx.world.destroy_immediate = crate::Flag32::FALSE;
    // m_broadPhase = new b2BroadPhase(worldAABB, m_contactManager)
    broad_phase::init(ctx, lower, upper);
    let def: BodyDef<R> = BodyDef::new();
    ctx.world.ground_body = create_body(ctx, &def);
}

/// b2World.CreateBody: `NONE` when locked (or when the port's body arena is exhausted).
pub fn create_body<R: Real>(ctx: &mut Ctx<R>, def: &BodyDef<R>) -> H {
    if ctx.world.lock == crate::Flag32::TRUE {
        return NONE;
    }
    let b = body::create(ctx, def);
    if b == NONE {
        return NONE;
    }
    ctx.world.body_mut(b).prev = NONE;
    let head = ctx.world.body_list;
    ctx.world.body_mut(b).next = head;
    if head != NONE {
        ctx.world.body_mut(head).prev = b;
    }
    ctx.world.body_list = b;
    ctx.world.body_count += 1;
    b
}

/// b2World.DestroyBody (silently returns when locked).
pub fn destroy_body<R: Real>(ctx: &mut Ctx<R>, b: H) -> Flow {
    if ctx.world.lock == crate::Flag32::TRUE {
        return Flow::Continue;
    }
    let mut jn: H = ctx.world.body(b).joint_list;
    while jn != NONE {
        let jn0 = jn;
        jn = ctx.world.joint_edge(jn).next;
        // m_destructionListener.SayGoodbyeJoint: none installed
        flow!(destroy_joint(ctx, edge_owner(jn0)));
    }
    let mut s: H = ctx.world.body(b).shape_list;
    while s != NONE {
        let s0 = s;
        s = ctx.world.shape(s).next;
        // m_destructionListener.SayGoodbyeShape: none installed
        flow!(shapes::destroy_proxy(ctx, s0));
        shapes::destroy(ctx, s0);
    }
    let (prev, next) = {
        let bd = ctx.world.body(b);
        (bd.prev, bd.next)
    };
    if prev != NONE {
        ctx.world.body_mut(prev).next = next;
    }
    if next != NONE {
        ctx.world.body_mut(next).prev = prev;
    }
    if b == ctx.world.body_list {
        ctx.world.body_list = next;
    }
    ctx.world.body_count -= 1;
    ctx.world.bodies.destroy(b);
    Flow::Continue
}

/// b2World.CreateJoint (NOT lock-guarded).
pub fn create_joint<R: Real>(ctx: &mut Ctx<R>, def: &JointDef<R>) -> (Flow, H) {
    let j = joints::create(ctx, def);
    if j == NONE {
        return (Flow::Abort(AbortSite::PortCapacity), NONE);
    }
    // Connect to the world list.
    ctx.world.joint_mut(j).prev = NONE;
    let head = ctx.world.joint_list;
    ctx.world.joint_mut(j).next = head;
    if head != NONE {
        ctx.world.joint_mut(head).prev = j;
    }
    ctx.world.joint_list = j;
    ctx.world.joint_count += 1;
    let body1 = ctx.world.joint(j).body1;
    let body2 = ctx.world.joint(j).body2;
    // Connect to the bodies' doubly linked lists: node1 -> body1's list (other = body2).
    let node1 = edge_handle(j, 0);
    {
        let b1_head = ctx.world.body(body1).joint_list;
        let e = ctx.world.joint_edge_mut(node1);
        e.other = body2;
        e.prev = NONE;
        e.next = b1_head;
        if b1_head != NONE {
            ctx.world.joint_edge_mut(b1_head).prev = node1;
        }
        ctx.world.body_mut(body1).joint_list = node1;
    }
    // node2 -> body2's list (other = body1).
    let node2 = edge_handle(j, 1);
    {
        let b2_head = ctx.world.body(body2).joint_list;
        let e = ctx.world.joint_edge_mut(node2);
        e.other = body1;
        e.prev = NONE;
        e.next = b2_head;
        if b2_head != NONE {
            ctx.world.joint_edge_mut(b2_head).prev = node2;
        }
        ctx.world.body_mut(body2).joint_list = node2;
    }
    // If the joint prevents collisions, then reset collision filtering.
    if def.collide_connected == crate::Flag32::FALSE {
        let b = if ctx.world.body(def.body1).shape_count < ctx.world.body(def.body2).shape_count { def.body1 } else { def.body2 };
        let mut s = ctx.world.body(b).shape_list;
        while s != NONE {
            let xf = ctx.world.body(b).xf;
            match shapes::refilter_proxy(ctx, s, &xf) {
                Flow::Continue => {}
                f => return (f, j),
            }
            s = ctx.world.shape(s).next;
        }
    }
    (Flow::Continue, j)
}

/// b2World.DestroyJoint (NOT lock-guarded).
pub fn destroy_joint<R: Real>(ctx: &mut Ctx<R>, j: H) -> Flow {
    let collide_connected = ctx.world.joint(j).collide_connected;
    // Remove from the world list.
    let (prev, next) = {
        let jt = ctx.world.joint(j);
        (jt.prev, jt.next)
    };
    if prev != NONE {
        ctx.world.joint_mut(prev).next = next;
    }
    if next != NONE {
        ctx.world.joint_mut(next).prev = prev;
    }
    if j == ctx.world.joint_list {
        ctx.world.joint_list = next;
    }
    // Disconnect from the bodies.
    let body1 = ctx.world.joint(j).body1;
    let body2 = ctx.world.joint(j).body2;
    body::wake_up(ctx, body1);
    body::wake_up(ctx, body2);
    let node1 = edge_handle(j, 0);
    {
        let (p, n) = {
            let e = ctx.world.joint_edge(node1);
            (e.prev, e.next)
        };
        if p != NONE {
            ctx.world.joint_edge_mut(p).next = n;
        }
        if n != NONE {
            ctx.world.joint_edge_mut(n).prev = p;
        }
        if node1 == ctx.world.body(body1).joint_list {
            ctx.world.body_mut(body1).joint_list = n;
        }
        let e = ctx.world.joint_edge_mut(node1);
        e.prev = NONE;
        e.next = NONE;
    }
    let node2 = edge_handle(j, 1);
    {
        let (p, n) = {
            let e = ctx.world.joint_edge(node2);
            (e.prev, e.next)
        };
        if p != NONE {
            ctx.world.joint_edge_mut(p).next = n;
        }
        if n != NONE {
            ctx.world.joint_edge_mut(n).prev = p;
        }
        if node2 == ctx.world.body(body2).joint_list {
            ctx.world.body_mut(body2).joint_list = n;
        }
        let e = ctx.world.joint_edge_mut(node2);
        e.prev = NONE;
        e.next = NONE;
    }
    joints::destroy(ctx, j);
    ctx.world.joint_count -= 1;
    // If the joint prevented collisions, then reset collision filtering.
    if collide_connected == crate::Flag32::FALSE {
        let b = if ctx.world.body(body1).shape_count < ctx.world.body(body2).shape_count { body1 } else { body2 };
        let mut s = ctx.world.body(b).shape_list;
        while s != NONE {
            let xf = ctx.world.body(b).xf;
            flow!(shapes::refilter_proxy(ctx, s, &xf));
            s = ctx.world.shape(s).next;
        }
    }
    Flow::Continue
}

/// b2World.Refilter(shape)
pub fn refilter<R: Real>(ctx: &mut Ctx<R>, shape: H) -> Flow {
    let b = ctx.world.shape(shape).body;
    let xf = ctx.world.body(b).xf;
    shapes::refilter_proxy(ctx, shape, &xf)
}

/// b2World.Step(dt, iterations). On an abort `m_lock` stays true (AS3 has no finally).
pub fn step<R: Real>(ctx: &mut Ctx<R>, dt: R, iterations: i32) -> Flow {
    ctx.world.lock = crate::Flag32::TRUE;
    let inv_dt = if dt.gt(R::zero()) { R::one() / dt } else { R::zero() };
    let step = TimeStep {
        dt,
        inv_dt,
        dt_ratio: ctx.world.inv_dt0 * dt,
        max_iterations: iterations,
        warm_starting: ctx.world.warm_starting,
        position_correction: ctx.world.position_correction,
    };
    // Update contacts.
    #[cfg(feature = "lab")]
    let lab_collide_phase = crate::lab::world_phase(crate::lab::WorldPhase::Collide);
    flow!(contacts::collide(ctx));
    #[cfg(feature = "lab")]
    drop(lab_collide_phase);
    // Integrate velocities, solve velocity constraints, and integrate positions.
    if step.dt.gt(R::zero()) {
        #[cfg(feature = "lab")]
        let _lab_world_phase = crate::lab::world_phase(crate::lab::WorldPhase::MainSolve);
        flow!(solve(ctx, &step));
    }
    // Handle TOI events.
    if ctx.world.continuous_physics.get() && step.dt.gt(R::zero()) {
        #[cfg(feature = "lab")]
        let _lab_world_phase = crate::lab::world_phase(crate::lab::WorldPhase::SolveToi);
        flow!(solve_toi(ctx, &step));
    }
    // DrawDebugData(): no-op
    ctx.world.inv_dt0 = step.inv_dt;
    ctx.world.lock = crate::Flag32::FALSE;
    Flow::Continue
}

/// b2World.Solve(step): find islands, integrate and solve constraints, solve position constraints.
pub fn solve<R: Real>(ctx: &mut Ctx<R>, step: &TimeStep<R>) -> Flow {
    ctx.world.position_iteration_count = 0;
    // new b2Island(m_bodyCount, m_contactCount, m_jointCount, ...)
    let (bc, cc, jc) = (ctx.world.body_count, ctx.world.contact_count, ctx.world.joint_count);
    ctx.scratch.island.init(bc, cc, jc);
    // Clear all the island flags.
    let mut b = ctx.world.body_list;
    while b != NONE {
        ctx.world.body_mut(b).flags &= !E_ISLAND_FLAG;
        b = ctx.world.body(b).next;
    }
    let mut c = ctx.world.contact_list;
    while c != NONE {
        ctx.world.contact_mut(c).flags &= !E_CONTACT_ISLAND_FLAG;
        c = ctx.world.contact(c).next;
    }
    let mut j = ctx.world.joint_list;
    while j != NONE {
        ctx.world.joint_mut(j).island_flag = crate::Flag32::FALSE;
        j = ctx.world.joint(j).next;
    }
    // Build and simulate all awake islands.
    let _stack_size: i32 = ctx.world.body_count;
    let mut seed = ctx.world.body_list;
    while seed != NONE {
        if (ctx.world.body(seed).flags & (E_ISLAND_FLAG | E_SLEEP_FLAG | E_FROZEN_FLAG)) == 0 {
            if !body::is_static(ctx, seed) {
                // Reset island and stack.
                ctx.scratch.island.clear();
                let mut stack_count: i32 = 0;
                ctx.scratch.stack[stack_count as usize] = seed;
                stack_count += 1;
                ctx.world.body_mut(seed).flags |= E_ISLAND_FLAG;
                // Perform a depth first search (DFS) on the constraint graph.
                while stack_count > 0 {
                    // Grab the next body off the stack and add it to the island.
                    stack_count -= 1;
                    b = ctx.scratch.stack[stack_count as usize];
                    ctx.scratch.island.add_body(b);
                    // Make sure the body is awake.
                    ctx.world.body_mut(b).flags &= !E_SLEEP_FLAG;
                    // To keep islands as small as possible, we don't propagate islands across static bodies.
                    if !body::is_static(ctx, b) {
                        // Search all contacts connected to this body.
                        let mut ce = ctx.world.body(b).contact_list;
                        while ce != NONE {
                            let ct = edge_owner(ce);
                            // Has this contact already been added to an island? Skip non-solid contacts.
                            if (ctx.world.contact(ct).flags & (E_CONTACT_ISLAND_FLAG | E_NON_SOLID_FLAG)) == 0 {
                                // Is this contact touching?
                                if ctx.world.contact(ct).manifold_count != 0 {
                                    ctx.scratch.island.add_contact(ct);
                                    ctx.world.contact_mut(ct).flags |= E_CONTACT_ISLAND_FLAG;
                                    let other = ctx.world.contact_edge(ce).other;
                                    // Was the other body already added to this island?
                                    if (ctx.world.body(other).flags & E_ISLAND_FLAG) == 0 {
                                        ctx.scratch.stack[stack_count as usize] = other;
                                        stack_count += 1;
                                        ctx.world.body_mut(other).flags |= E_ISLAND_FLAG;
                                    }
                                }
                            }
                            ce = ctx.world.contact_edge(ce).next;
                        }
                        // Search all joints connected to this body.
                        let mut je = ctx.world.body(b).joint_list;
                        while je != NONE {
                            let jt = edge_owner(je);
                            if ctx.world.joint(jt).island_flag != crate::Flag32::TRUE {
                                ctx.scratch.island.add_joint(jt);
                                ctx.world.joint_mut(jt).island_flag = crate::Flag32::TRUE;
                                let other = ctx.world.joint_edge(je).other;
                                if (ctx.world.body(other).flags & E_ISLAND_FLAG) == 0 {
                                    ctx.scratch.stack[stack_count as usize] = other;
                                    stack_count += 1;
                                    ctx.world.body_mut(other).flags |= E_ISLAND_FLAG;
                                }
                            }
                            je = ctx.world.joint_edge(je).next;
                        }
                    }
                }
                let gravity = ctx.world.gravity;
                let position_correction = ctx.world.position_correction;
                let allow_sleep = ctx.world.allow_sleep;
                flow!(island::solve(ctx, step, gravity, position_correction.get(), allow_sleep.get()));
                if ctx.scratch.island.position_iteration_count > ctx.world.position_iteration_count {
                    ctx.world.position_iteration_count = ctx.scratch.island.position_iteration_count;
                }
                // Post solve cleanup: allow static bodies to participate in other islands.
                let mut i: i32 = 0;
                while i < ctx.scratch.island.body_count {
                    b = ctx.scratch.island.bodies[i as usize];
                    if body::is_static(ctx, b) {
                        ctx.world.body_mut(b).flags &= !E_ISLAND_FLAG;
                    }
                    i += 1;
                }
            }
        }
        seed = ctx.world.body(seed).next;
    }
    // Synchronize shapes, check for out of range bodies.
    b = ctx.world.body_list;
    while b != NONE {
        if (ctx.world.body(b).flags & (E_SLEEP_FLAG | E_FROZEN_FLAG)) == 0 {
            if !body::is_static(ctx, b) {
                // Update shapes (for broad-phase). If the shapes go out of the world AABB then
                // shapes and contacts may be destroyed, including contacts that are
                let (f, in_range) = body::synchronize_shapes(ctx, b);
                flow!(f);
                // Did the body's shapes leave the world?
                if in_range == false {
                    // m_boundaryListener.Violation(b): none installed
                }
            }
        }
        b = ctx.world.body(b).next;
    }
    // Commit shape proxy movements to the broad-phase so that new contacts are created.
    // Also, some contacts can be destroyed.
    broad_phase::commit(ctx)
}

/// b2World.SolveTOI(step): find TOI contacts and solve them.
#[allow(unused_assignments)] // `toi = 1` before the branch is the literal AS3 statement order
pub fn solve_toi<R: Real>(ctx: &mut Ctx<R>, step: &TimeStep<R>) -> Flow {
    lab! { let toi_target_probe_enabled=crate::lab::toi_target_enabled(); }
    // new b2Island(m_bodyCount, b2_maxTOIContactsPerIsland, 0, ...)
    let bc = ctx.world.body_count;
    ctx.scratch.island.init(bc, settings::B2_MAX_TOI_CONTACTS_PER_ISLAND as i32, 0);
    let _stack_size: i32 = ctx.world.body_count;
    // Initialize the island flags and reset the sweep times.
    let mut b = ctx.world.body_list;
    while b != NONE {
        let bd = ctx.world.body_mut(b);
        bd.flags &= !E_ISLAND_FLAG;
        bd.sweep.t0 = R::zero();
        b = ctx.world.body(b).next;
    }
    let mut c = ctx.world.contact_list;
    while c != NONE {
        // Invalidate TOI
        ctx.world.contact_mut(c).flags &= !(E_TOI_FLAG | E_CONTACT_ISLAND_FLAG);
        c = ctx.world.contact(c).next;
    }
    // Find TOI events and solve them.
    lab! { if toi_target_probe_enabled { crate::lab::toi_target_snapshot(ctx,crate::lab::ToiTargetStage::Initial,NONE,R::one()); } }
    lab! { let mut toi_iterations: u32 = 0; }
    loop {
        // Search-only hang guard (never armed in verification): see `lab::set_deadline`.
        lab! {
            toi_iterations = toi_iterations.wrapping_add(1);
            if toi_iterations % 64 == 0 && crate::lab::over_budget() {
                return Flow::Abort(AbortSite::PortCapacity);
            }
        }
        // Find the first TOI.
        lab! { if toi_target_probe_enabled { crate::lab::toi_target_scan_begin(); } }
        let mut min_contact: H = NONE;
        let mut min_toi: R = R::one();
        c = ctx.world.contact_list;
        while c != NONE {
            'this_contact: {
                if (ctx.world.contact(c).flags & (E_SLOW_FLAG | E_NON_SOLID_FLAG)) != 0 {
                    lab! { if toi_target_probe_enabled { crate::lab::toi_target_branch(ctx,c,crate::lab::ToiTargetBranch::SlowOrNonSolid,None::<R>,min_toi); } }
                    break 'this_contact;
                }
                // TODO_ERIN keep a counter on the contact, only respond to M TOIs per contact.
                let mut toi: R = R::one();
                if (ctx.world.contact(c).flags & E_TOI_FLAG) != 0 {
                    // This contact has a valid cached TOI.
                    toi = ctx.world.contact(c).toi;
                    lab! { if toi_target_probe_enabled { crate::lab::toi_target_branch(ctx,c,crate::lab::ToiTargetBranch::Cached,Some(toi),min_toi); } }
                } else {
                    // Compute the TOI for this contact.
                    let s1 = ctx.world.contact(c).shape1;
                    let s2 = ctx.world.contact(c).shape2;
                    let b1 = ctx.world.shape(s1).body;
                    let b2 = ctx.world.shape(s2).body;
                    if (body::is_static(ctx, b1) || body::is_sleeping(ctx, b1))
                        && (body::is_static(ctx, b2) || body::is_sleeping(ctx, b2))
                    {
                        lab! { if toi_target_probe_enabled { crate::lab::toi_target_branch(ctx,c,crate::lab::ToiTargetBranch::BothStaticOrSleeping,None::<R>,min_toi); } }
                        break 'this_contact; // continue
                    }
                    lab! { if toi_target_probe_enabled { crate::lab::toi_target_branch(ctx,c,crate::lab::ToiTargetBranch::BeforeSweepAlignment,None::<R>,min_toi); } }
                    // Put the sweeps onto the same time interval.
                    let mut t0: R = ctx.world.body(b1).sweep.t0;
                    if ctx.world.body(b1).sweep.t0.lt(ctx.world.body(b2).sweep.t0) {
                        t0 = ctx.world.body(b2).sweep.t0;
                        ctx.world.body_mut(b1).sweep.advance(t0);
                    } else if ctx.world.body(b2).sweep.t0.lt(ctx.world.body(b1).sweep.t0) {
                        t0 = ctx.world.body(b1).sweep.t0;
                        ctx.world.body_mut(b2).sweep.advance(t0);
                    }
                    // Compute the time of impact.
                    toi = {
                        let w = &ctx.world;
                        collision::time_of_impact(w.shape(s1), &w.body(b1).sweep, w.shape(s2), &w.body(b2).sweep)
                    };
                    if toi.gt(R::zero()) && toi.lt(R::one()) {
                        toi = (R::one() - toi) * t0 + toi;
                        if toi.gt(R::one()) {
                            toi = R::one();
                        }
                    }
                    let ct = ctx.world.contact_mut(c);
                    ct.toi = toi;
                    ct.flags |= E_TOI_FLAG;
                    lab! { if toi_target_probe_enabled { crate::lab::toi_target_branch(ctx,c,crate::lab::ToiTargetBranch::Computed,Some(toi),min_toi); } }
                }
                if settings::min_value::<R>().lt(toi) && toi.lt(min_toi) {
                    lab! { if toi_target_probe_enabled { crate::lab::toi_target_branch(ctx,c,crate::lab::ToiTargetBranch::ReplacedMinimum,Some(toi),min_toi); } }
                    // This is the minimum TOI found so far.
                    min_contact = c;
                    min_toi = toi;
                } else {
                    lab! { if toi_target_probe_enabled { crate::lab::toi_target_branch(ctx,c,crate::lab::ToiTargetBranch::DidNotReplaceMinimum,Some(toi),min_toi); } }
                }
            }
            c = ctx.world.contact(c).next;
        }
        if min_contact == NONE || (R::one() - R::from_i32(100) * settings::min_value::<R>()).lt(min_toi) {
            lab! { if toi_target_probe_enabled { crate::lab::toi_target_scan_end(min_contact,min_toi,true); } }
            // No more TOI events. Done!
            break;
        }
        lab! { if toi_target_probe_enabled { crate::lab::toi_target_scan_end(min_contact,min_toi,false); } }
        // Advance the bodies to the TOI.
        let s1 = ctx.world.contact(min_contact).shape1;
        let s2 = ctx.world.contact(min_contact).shape2;
        let b1 = ctx.world.shape(s1).body;
        let b2 = ctx.world.shape(s2).body;
        lab! { if toi_target_probe_enabled { crate::lab::toi_target_selected_before_advance(ctx,min_contact,min_toi); } }
        lab! {
            if crate::lab::watched(b1) || crate::lab::watched(b2) {
                crate::lab::record(crate::lab::Event::Toi {
                    toi: crate::lab::f(min_toi),
                    shape1: s1,
                    shape2: s2,
                    body1: b1,
                    body2: b2,
                });
            }
        }
        body::advance(ctx, b1, min_toi);
        body::advance(ctx, b2, min_toi);
        // The TOI contact likely has some new contact points.
        flow!(contacts::update(ctx, min_contact));
        ctx.world.contact_mut(min_contact).flags &= !E_TOI_FLAG;
        lab! { if toi_target_probe_enabled { crate::lab::toi_target_updated(ctx,min_contact); } }
        if ctx.world.contact(min_contact).manifold_count == 0 {
            lab! {
                if crate::lab::watched(b1) || crate::lab::watched(b2) {
                    use crate::lab::f;
                    let (p1, p2) = (ctx.world.body(b1).sweep.c, ctx.world.body(b2).sweep.c);
                    crate::lab::record(crate::lab::Event::Strand {
                        toi: f(min_toi),
                        shape1: s1,
                        shape2: s2,
                        body1: b1,
                        body2: b2,
                        pos1: [f(p1.x), f(p1.y)],
                        pos2: [f(p2.x), f(p2.y)],
                    });
                }
            }
            // This should not happen. Maybe the shapes are not solid?
            continue;
        }
        // Build the TOI island. We need a dynamic seed.
        let mut seed: H = b1;
        if body::is_static(ctx, seed) {
            seed = b2;
        }
        // Reset island and stack.
        ctx.scratch.island.clear();
        let mut stack_count: i32 = 0;
        ctx.scratch.stack[stack_count as usize] = seed;
        stack_count += 1;
        ctx.world.body_mut(seed).flags |= E_ISLAND_FLAG;
        // Perform a depth first search (DFS) on the contact graph.
        while stack_count > 0 {
            // Grab the next body off the stack and add it to the island.
            stack_count -= 1;
            b = ctx.scratch.stack[stack_count as usize];
            ctx.scratch.island.add_body(b);
            // Make sure the body is awake.
            ctx.world.body_mut(b).flags &= !E_SLEEP_FLAG;
            // To keep islands as small as possible, we don't propagate islands across static bodies.
            if !body::is_static(ctx, b) {
                // Search all contacts connected to this body.
                let mut ce = ctx.world.body(b).contact_list;
                while ce != NONE {
                    // Does the TOI island still have space for contacts?
                    if ctx.scratch.island.contact_count != ctx.scratch.island.contact_capacity {
                        let ct = edge_owner(ce);
                        // Has this contact already been added to an island? Skip slow or non-solid contacts.
                        if (ctx.world.contact(ct).flags & (E_CONTACT_ISLAND_FLAG | E_SLOW_FLAG | E_NON_SOLID_FLAG)) == 0 {
                            // Is this contact touching? For performance we are not updating this contact.
                            if ctx.world.contact(ct).manifold_count != 0 {
                                ctx.scratch.island.add_contact(ct);
                                ctx.world.contact_mut(ct).flags |= E_CONTACT_ISLAND_FLAG;
                                // Update other body.
                                let other = ctx.world.contact_edge(ce).other;
                                // Was the other body already added to this island?
                                if (ctx.world.body(other).flags & E_ISLAND_FLAG) == 0 {
                                    // March forward, this can do no harm since this is the min TOI.
                                    if body::is_static(ctx, other) == false {
                                        body::advance(ctx, other, min_toi);
                                        body::wake_up(ctx, other);
                                    }
                                    ctx.scratch.stack[stack_count as usize] = other;
                                    stack_count += 1;
                                    ctx.world.body_mut(other).flags |= E_ISLAND_FLAG;
                                }
                            }
                        }
                    }
                    ce = ctx.world.contact_edge(ce).next;
                }
            }
        }
        // new b2TimeStep(): warmStarting/positionCorrection false, dtRatio NaN.
        let sub_dt = (R::one() - min_toi) * step.dt;
        let sub_step = TimeStep {
            dt: sub_dt,
            inv_dt: R::one() / sub_dt,
            dt_ratio: R::from_words([0x7FF80000, 0]),
            max_iterations: step.max_iterations,
            warm_starting: crate::Flag32::FALSE,
            position_correction: crate::Flag32::FALSE,
        };
        flow!(island::solve_toi(ctx, &sub_step));
        lab! {
            if crate::lab::armed() {
                let mut k: i32 = 0;
                while k < ctx.scratch.island.body_count {
                    let bh = ctx.scratch.island.bodies[k as usize];
                    if crate::lab::watched(bh) {
                        let b = ctx.world.body(bh);
                        use crate::lab::f;
                        crate::lab::record(crate::lab::Event::AfterToi {
                            body: bh,
                            c0: [f(b.sweep.c0.x), f(b.sweep.c0.y)],
                            c: [f(b.sweep.c.x), f(b.sweep.c.y)],
                            v: [f(b.linear_velocity.x), f(b.linear_velocity.y)],
                        });
                    }
                    k += 1;
                }
            }
        }
        // Post solve cleanup.
        let mut i: i32 = 0;
        while i < ctx.scratch.island.body_count {
            // Allow bodies to participate in future TOI islands.
            b = ctx.scratch.island.bodies[i as usize];
            ctx.world.body_mut(b).flags &= !E_ISLAND_FLAG;
            if (ctx.world.body(b).flags & (E_SLEEP_FLAG | E_FROZEN_FLAG)) == 0 {
                if !body::is_static(ctx, b) {
                    // Update shapes (for broad-phase). If the shapes go out of the world AABB then
                    // shapes and contacts may be destroyed, including contacts that are
                    let (f, in_range) = body::synchronize_shapes(ctx, b);
                    flow!(f);
                    lab! { if toi_target_probe_enabled { crate::lab::toi_target_primary_synchronized(ctx,b,min_contact,min_toi,in_range); } }
                    // Did the body's shapes leave the world?
                    if in_range == false {
                        // m_boundaryListener.Violation(b): none installed
                    }
                    // Invalidate all contact TOIs associated with this body. Some of these
                    // may not be in the island because they were not touching.
                    let mut ce = ctx.world.body(b).contact_list;
                    while ce != NONE {
                        ctx.world.contact_mut(edge_owner(ce)).flags &= !E_TOI_FLAG;
                        ce = ctx.world.contact_edge(ce).next;
                    }
                }
            }
            i += 1;
        }
        i = 0;
        while i < ctx.scratch.island.contact_count {
            // Allow contacts to participate in future TOI islands.
            c = ctx.scratch.island.contacts[i as usize];
            ctx.world.contact_mut(c).flags &= !(E_TOI_FLAG | E_CONTACT_ISLAND_FLAG);
            i += 1;
        }
        // Commit shape proxy movements to the broad-phase so that new contacts are created.
        // Also, some contacts can be destroyed.
        lab! { if toi_target_probe_enabled { crate::lab::toi_target_snapshot(ctx,crate::lab::ToiTargetStage::BeforeCommit,min_contact,min_toi); } }
        flow!(broad_phase::commit(ctx));
        lab! { if toi_target_probe_enabled { crate::lab::toi_target_snapshot(ctx,crate::lab::ToiTargetStage::AfterCommit,min_contact,min_toi); } }
    }
    Flow::Continue
}

/// b2World.Query(aabb, shapes, maxCount): shape handles into `out`; returns the count.
pub fn query<R: Real>(ctx: &mut Ctx<R>, lower: Vec2<R>, upper: Vec2<R>, out: &mut (impl broad_phase::QueryOutput + ?Sized), max_count: i32) -> (Flow, i32) {
    broad_phase::query_aabb(ctx, lower, upper, out, max_count)
}

// ---- trivial accessors / setters --------------------------------------------------------------

#[inline(always)]
pub fn get_body_list<R: Real>(ctx: &Ctx<R>) -> H {
    ctx.world.body_list
}
#[inline(always)]
pub fn get_joint_list<R: Real>(ctx: &Ctx<R>) -> H {
    ctx.world.joint_list
}
#[inline(always)]
pub fn get_contact_list<R: Real>(ctx: &Ctx<R>) -> H {
    ctx.world.contact_list
}
#[inline(always)]
pub fn get_ground_body<R: Real>(ctx: &Ctx<R>) -> H {
    ctx.world.ground_body
}
#[inline(always)]
pub fn get_body_count<R: Real>(ctx: &Ctx<R>) -> i32 {
    ctx.world.body_count
}
#[inline(always)]
pub fn get_joint_count<R: Real>(ctx: &Ctx<R>) -> i32 {
    ctx.world.joint_count
}
#[inline(always)]
pub fn get_contact_count<R: Real>(ctx: &Ctx<R>) -> i32 {
    ctx.world.contact_count
}
#[inline(always)]
pub fn get_proxy_count<R: Real>(ctx: &Ctx<R>) -> i32 {
    ctx.world.broad_phase.proxy_count
}
#[inline(always)]
pub fn get_pair_count<R: Real>(ctx: &Ctx<R>) -> i32 {
    ctx.world.broad_phase.pair_count
}
#[inline(always)]
pub fn set_gravity<R: Real>(ctx: &mut Ctx<R>, gravity: Vec2<R>) {
    ctx.world.gravity = gravity;
}
#[inline(always)]
pub fn set_continuous_physics<R: Real>(ctx: &mut Ctx<R>, flag: bool) {
    ctx.world.continuous_physics = crate::Flag32::new(flag);
}
#[inline(always)]
pub fn set_warm_starting<R: Real>(ctx: &mut Ctx<R>, flag: bool) {
    ctx.world.warm_starting = crate::Flag32::new(flag);
}
#[inline(always)]
pub fn set_position_correction<R: Real>(ctx: &mut Ctx<R>, flag: bool) {
    ctx.world.position_correction = crate::Flag32::new(flag);
}
/// SetContactListener: `mode` is `Ctx::listener_mode` (0 = null, 1 = the game's listener).
#[inline(always)]
pub fn set_contact_listener<R: Real>(ctx: &mut Ctx<R>, mode: u32) {
    ctx.listener_mode = mode;
}
/// DrawDebugData / Validate: no-ops.
#[inline(always)]
pub fn draw_debug_data<R: Real>(_ctx: &mut Ctx<R>) {}
