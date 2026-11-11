//! b2Contact, b2CircleContact, b2PolyAndCircleContact, b2PolygonContact, b2NullContact and
//! b2ContactManager (+ b2ContactFilter.ShouldCollide) of Box2DFlashAS3 2.0.2.
//!
//! Contacts live in `ctx.world.contacts`; `kind` selects the AS3 subclass (`CONTACT_*`). The single
//! b2NullContact is the reserved handle `NULL_CONTACT`. The static scratch `s_evalCP` objects are
//! locals (every field is written before each listener call). Callbacks into the game go through
//! `crate::listener` and may mutate the world, so every object field is re-read from `ctx` after a
//! callback exactly as the AS3 re-reads it through its object references.
//!
//! Throw sites reproduced: `B2AssertPolyCircle` (b2PolyAndCircleContact constructor asserts),
//! `ContactManagerNullBody` (a contact whose shape has a null `m_body`, dereferenced in
//! b2ContactManager.Destroy / PairAdded / Collide and b2Contact.Evaluate at the AS3 statement that
//! would throw), `PortCapacity` (contact arena full — not a Box2D limit).
use hw_real::as3::b2_max;
use hw_real::Real;

use super::body;
use super::collision;
use super::math::Vec2;
use super::state::{
    edge_handle, Contact, ContactPoint, Edge, ManifoldPoint, CONTACT_CIRCLE_CIRCLE, CONTACT_POLY_CIRCLE,
    CONTACT_POLY_POLY, E_CIRCLE_SHAPE, E_NON_SOLID_FLAG, E_POLYGON_SHAPE, E_SLOW_FLAG, NULL_CONTACT,
};
use crate::flow::{AbortSite, Flow};
use crate::listener;
use crate::{Ctx, H, NONE};

// ---- b2Contact registers ---------------------------------------------------------------------

/// b2ContactRegister: `createFcn` is identified by the contact kind (-1 = null function).
#[derive(Clone, Copy)]
struct ContactRegister {
    kind: i32,
    primary: bool,
}

const E_SHAPE_TYPE_COUNT: usize = 2;

/// b2Contact.AddType(createFcn, destroyFcn, type1, type2)
fn add_type(regs: &mut [[ContactRegister; E_SHAPE_TYPE_COUNT]; E_SHAPE_TYPE_COUNT], kind: i32, type1: usize, type2: usize) {
    regs[type1][type2].kind = kind;
    regs[type1][type2].primary = true;
    if type1 != type2 {
        regs[type2][type1].kind = kind;
        regs[type2][type1].primary = false;
    }
}

/// b2Contact.InitializeRegisters
fn initialize_registers() -> [[ContactRegister; E_SHAPE_TYPE_COUNT]; E_SHAPE_TYPE_COUNT] {
    let mut regs = [[ContactRegister { kind: -1, primary: false }; E_SHAPE_TYPE_COUNT]; E_SHAPE_TYPE_COUNT];
    add_type(&mut regs, CONTACT_CIRCLE_CIRCLE, E_CIRCLE_SHAPE as usize, E_CIRCLE_SHAPE as usize);
    add_type(&mut regs, CONTACT_POLY_CIRCLE, E_POLYGON_SHAPE as usize, E_CIRCLE_SHAPE as usize);
    add_type(&mut regs, CONTACT_POLY_POLY, E_POLYGON_SHAPE as usize, E_POLYGON_SHAPE as usize);
    regs
}

/// b2Contact.Create(shape1, shape2, allocator) -> contact handle (`NONE` = null when no register).
/// Non-primary registers swap the shapes (circle/polygon -> polygon/circle), then negate the
/// manifold normals of the (zero) existing manifolds.
pub fn create<R: Real>(ctx: &mut Ctx<R>, shape1: H, shape2: H) -> (Flow, H) {
    if ctx.world.contact_registers_initialized == crate::Flag32::FALSE {
        ctx.world.contact_registers_initialized = crate::Flag32::TRUE;
    }
    let regs = initialize_registers();
    let type1 = ctx.world.shape(shape1).type_;
    let type2 = ctx.world.shape(shape2).type_;
    if type1 < 0 || type1 >= E_SHAPE_TYPE_COUNT as i32 || type2 < 0 || type2 >= E_SHAPE_TYPE_COUNT as i32 {
        // s_registers[-1] is undefined in AS3 (would throw); shapes always carry a valid type.
        debug_assert!(false, "unknown shape type in b2Contact.Create");
        return (Flow::Continue, NONE);
    }
    let reg = regs[type1 as usize][type2 as usize];
    if reg.kind != -1 {
        if reg.primary {
            return create_typed(ctx, reg.kind, shape1, shape2);
        }
        let (f, c) = create_typed(ctx, reg.kind, shape2, shape1);
        match f {
            Flow::Continue => {}
            f => return (f, NONE),
        }
        let mut i: i32 = 0;
        while i < ctx.world.contact(c).manifold_count {
            let m = &mut ctx.world.contact_mut(c).manifold;
            m.normal = m.normal.negative();
            i += 1;
        }
        return (Flow::Continue, c);
    }
    (Flow::Continue, NONE)
}

/// `new b2XxxContact(shape1, shape2)`: the b2Contact constructor followed by the subclass body.
fn create_typed<R: Real>(ctx: &mut Ctx<R>, kind: i32, shape1: H, shape2: H) -> (Flow, H) {
    let (type1, sensor1, friction1, restitution1) = {
        let s = ctx.world.shape(shape1);
        (s.type_, s.is_sensor, s.friction, s.restitution)
    };
    let (type2, sensor2, friction2, restitution2) = {
        let s = ctx.world.shape(shape2);
        (s.type_, s.is_sensor, s.friction, s.restitution)
    };
    let mut c = Contact::blank();
    c.kind = kind;
    // b2Contact(param1, param2)
    c.flags = 0;
    if sensor1.get() || sensor2.get() {
        c.flags |= E_NON_SOLID_FLAG;
    }
    c.shape1 = shape1;
    c.shape2 = shape2;
    c.manifold_count = 0;
    c.friction = (friction1 * friction2).sqrt();
    c.restitution = b2_max(restitution1, restitution2);
    c.prev = NONE;
    c.next = NONE;
    c.node1 = Edge::new();
    c.node2 = Edge::new();
    // subclass constructor
    if kind == CONTACT_POLY_CIRCLE {
        // b2Settings.b2Assert(m_shape1.m_type == e_polygonShape);
        // b2Settings.b2Assert(m_shape2.m_type == e_circleShape);
        // (`++null.x` -> TypeError; the half-built contact is never linked anywhere, so the
        // check is done before the arena slot is taken.)
        if !(type1 == E_POLYGON_SHAPE) || !(type2 == E_CIRCLE_SHAPE) {
            return (Flow::Abort(AbortSite::B2AssertPolyCircle), NONE);
        }
    }
    c.manifold.point_count = 0;
    if kind == CONTACT_CIRCLE_CIRCLE || kind == CONTACT_POLY_CIRCLE {
        // b2CircleContact / b2PolyAndCircleContact zero the first point's impulses;
        // b2PolygonContact leaves them at the NaN of `new b2ManifoldPoint()`.
        let mp = &mut c.manifold.points[0];
        mp.normal_impulse = R::zero();
        mp.tangent_impulse = R::zero();
    }
    let h = ctx.world.contacts.alloc(c);
    if h == NONE {
        return (Flow::Abort(AbortSite::PortCapacity), NONE);
    }
    (Flow::Continue, h)
}

/// b2Contact.Destroy(contact, allocator): wakes both bodies when the contact had a manifold, then
/// calls the (empty) type destroy function; the port frees the arena slot here.
pub fn destroy<R: Real>(ctx: &mut Ctx<R>, contact: H) {
    let (manifold_count, shape1, shape2) = {
        let c = ctx.world.contact(contact);
        (c.manifold_count, c.shape1, c.shape2)
    };
    if manifold_count > 0 {
        let body1 = ctx.world.shape(shape1).body;
        let body2 = ctx.world.shape(shape2).body;
        // Reached only through b2ContactManager.Destroy, which has already dereferenced both
        // bodies (see `manager_destroy`), so they cannot be null here.
        debug_assert!(body1 != NONE && body2 != NONE);
        body::wake_up(ctx, body1);
        body::wake_up(ctx, body2);
    }
    // s_registers[type1][type2].destroyFcn(contact, allocator): empty for all three types.
    ctx.world.contacts.destroy(contact);
}

/// b2Contact.IsSolid
#[inline(always)]
pub fn is_solid<R: Real>(ctx: &Ctx<R>, contact: H) -> bool {
    (ctx.world.contact(contact).flags & E_NON_SOLID_FLAG) == 0
}
/// b2Contact.GetShape1
#[inline(always)]
pub fn get_shape1<R: Real>(ctx: &Ctx<R>, contact: H) -> H {
    ctx.world.contact(contact).shape1
}
/// b2Contact.GetShape2
#[inline(always)]
pub fn get_shape2<R: Real>(ctx: &Ctx<R>, contact: H) -> H {
    ctx.world.contact(contact).shape2
}
/// b2Contact.GetNext
#[inline(always)]
pub fn get_next<R: Real>(ctx: &Ctx<R>, contact: H) -> H {
    ctx.world.contact(contact).next
}
/// b2Contact.GetManifoldCount
#[inline(always)]
pub fn get_manifold_count<R: Real>(ctx: &Ctx<R>, contact: H) -> i32 {
    ctx.world.contact(contact).manifold_count
}

/// b2Contact.Update(listener)
pub fn update<R: Real>(ctx: &mut Ctx<R>, contact: H) -> Flow {
    if contact == NULL_CONTACT {
        // b2NullContact never sits in a contact list; Update on it would dereference null shapes.
        debug_assert!(false, "Update on b2NullContact");
        return Flow::Continue;
    }
    let old_count = ctx.world.contact(contact).manifold_count;
    crate::flow!(evaluate(ctx, contact));
    let new_count = ctx.world.contact(contact).manifold_count;
    let (shape1, shape2) = {
        let c = ctx.world.contact(contact);
        (c.shape1, c.shape2)
    };
    let body1 = ctx.world.shape(shape1).body;
    let body2 = ctx.world.shape(shape2).body;
    if new_count == 0 && old_count > 0 {
        body::wake_up(ctx, body1);
        body::wake_up(ctx, body2);
    }
    if body::is_static(ctx, body1) || body::is_bullet(ctx, body1) || body::is_static(ctx, body2) || body::is_bullet(ctx, body2) {
        ctx.world.contact_mut(contact).flags &= !E_SLOW_FLAG;
    } else {
        ctx.world.contact_mut(contact).flags |= E_SLOW_FLAG;
    }
    Flow::Continue
}

/// b2Contact.Evaluate(listener): dispatches on the contact kind (b2NullContact: no-op).
pub fn evaluate<R: Real>(ctx: &mut Ctx<R>, contact: H) -> Flow {
    if contact == NULL_CONTACT {
        return Flow::Continue;
    }
    match ctx.world.contact(contact).kind {
        CONTACT_CIRCLE_CIRCLE => evaluate_circles(ctx, contact),
        CONTACT_POLY_CIRCLE => evaluate_polys(ctx, contact, true),
        _ => evaluate_polys(ctx, contact, false),
    }
}

/// Fill the s_evalCP position / velocity / normal / separation / id fields for one manifold point.
#[inline(always)]
fn eval_cp<R: Real>(ctx: &mut Ctx<R>, cp: &mut ContactPoint<R>, body1: H, body2: H, mp: &ManifoldPoint<R>, normal: Vec2<R>) {
    cp.position = body::get_world_point(ctx, body1, mp.local_point1);
    let v1 = body::get_linear_velocity_from_local_point(ctx, body1, mp.local_point1);
    let v2 = body::get_linear_velocity_from_local_point(ctx, body2, mp.local_point2);
    cp.velocity.set(v2.x - v1.x, v2.y - v1.y);
    cp.normal.set_v(normal);
    cp.separation = mp.separation;
    cp.id = mp.id;
}

#[inline(always)]
fn new_cp<R: Real>(shape1: H, shape2: H, friction: R, restitution: R) -> ContactPoint<R> {
    ContactPoint {
        shape1,
        shape2,
        position: Vec2::zero(),
        velocity: Vec2::zero(),
        normal: Vec2::zero(),
        separation: R::zero(),
        friction,
        restitution,
        id: 0,
    }
}

/// b2CircleContact.Evaluate
fn evaluate_circles<R: Real>(ctx: &mut Ctx<R>, contact: H) -> Flow {
    let (shape1, shape2, friction, restitution) = {
        let c = ctx.world.contact(contact);
        (c.shape1, c.shape2, c.friction, c.restitution)
    };
    let body1 = ctx.world.shape(shape1).body;
    let body2 = ctx.world.shape(shape2).body;
    // this.m0.Set(this.m_manifold)
    let m0 = ctx.world.contact(contact).manifold;
    if body1 == NONE || body2 == NONE {
        // `_loc5_.m_xf` / `_loc6_.m_xf` on a null body.
        return Flow::Abort(AbortSite::ContactManagerNullBody);
    }
    let xf1 = ctx.world.body(body1).xf;
    let xf2 = ctx.world.body(body2).xf;
    // b2CollideCircles mutates this.m_manifold in place; it is identical to m0 at this point.
    let mut m = m0;
    collision::collide_circles(&mut m, ctx.world.shape(shape1), &xf1, ctx.world.shape(shape2), &xf2);
    ctx.world.contact_mut(contact).manifold = m;
    let mut cp = new_cp(shape1, shape2, friction, restitution);
    let has_listener = ctx.listener_mode != 0;
    if m.point_count > 0 {
        ctx.world.contact_mut(contact).manifold_count = 1;
        if m0.point_count == 0 {
            {
                let mp = &mut ctx.world.contact_mut(contact).manifold.points[0];
                mp.normal_impulse = R::zero();
                mp.tangent_impulse = R::zero();
            }
            if has_listener {
                let mp = ctx.world.contact(contact).manifold.points[0];
                let normal = ctx.world.contact(contact).manifold.normal;
                eval_cp(ctx, &mut cp, body1, body2, &mp, normal);
                crate::flow!(listener::add(ctx, &cp));
            }
        } else {
            let mp0 = m0.points[0];
            {
                let mp = &mut ctx.world.contact_mut(contact).manifold.points[0];
                mp.normal_impulse = mp0.normal_impulse;
                mp.tangent_impulse = mp0.tangent_impulse;
            }
            if has_listener {
                let mp = ctx.world.contact(contact).manifold.points[0];
                let normal = ctx.world.contact(contact).manifold.normal;
                eval_cp(ctx, &mut cp, body1, body2, &mp, normal);
                crate::flow!(listener::persist(ctx, &cp));
            }
        }
    } else {
        ctx.world.contact_mut(contact).manifold_count = 0;
        if m0.point_count > 0 && has_listener {
            let mp0 = m0.points[0];
            eval_cp(ctx, &mut cp, body1, body2, &mp0, m0.normal);
            crate::flow!(listener::remove(ctx, &cp));
        }
    }
    Flow::Continue
}

/// b2PolyAndCircleContact.Evaluate (`poly_circle == true`) / b2PolygonContact.Evaluate: identical
/// bodies except for the collide call. Points are matched to the m0 snapshot by id key.
fn evaluate_polys<R: Real>(ctx: &mut Ctx<R>, contact: H, poly_circle: bool) -> Flow {
    let (shape1, shape2, friction, restitution) = {
        let c = ctx.world.contact(contact);
        (c.shape1, c.shape2, c.friction, c.restitution)
    };
    let body1 = ctx.world.shape(shape1).body;
    let body2 = ctx.world.shape(shape2).body;
    // this.m0.Set(this.m_manifold)
    let m0 = ctx.world.contact(contact).manifold;
    if body1 == NONE || body2 == NONE {
        return Flow::Abort(AbortSite::ContactManagerNullBody);
    }
    let xf1 = ctx.world.body(body1).xf;
    let xf2 = ctx.world.body(body2).xf;
    let mut m = m0;
    if poly_circle {
        collision::collide_polygon_and_circle(&mut m, ctx.world.shape(shape1), &xf1, ctx.world.shape(shape2), &xf2);
    } else {
        collision::collide_polygons(&mut m, ctx.world.shape(shape1), &xf1, ctx.world.shape(shape2), &xf2);
    }
    ctx.world.contact_mut(contact).manifold = m;
    let mut persisted = [false, false];
    let mut cp = new_cp(shape1, shape2, friction, restitution);
    let has_listener = ctx.listener_mode != 0;
    if m.point_count > 0 {
        let mut i: i32 = 0;
        while i < ctx.world.contact(contact).manifold.point_count {
            let iu = i as usize;
            {
                let mp = &mut ctx.world.contact_mut(contact).manifold.points[iu];
                mp.normal_impulse = R::zero();
                mp.tangent_impulse = R::zero();
            }
            let mut found = false;
            let id = ctx.world.contact(contact).manifold.points[iu].id;
            let mut j: i32 = 0;
            while j < m0.point_count {
                let ju = j as usize;
                if persisted[ju] != true {
                    let mp0 = m0.points[ju];
                    if mp0.id == id {
                        persisted[ju] = true;
                        {
                            let mp = &mut ctx.world.contact_mut(contact).manifold.points[iu];
                            mp.normal_impulse = mp0.normal_impulse;
                            mp.tangent_impulse = mp0.tangent_impulse;
                        }
                        found = true;
                        if has_listener {
                            let mp = ctx.world.contact(contact).manifold.points[iu];
                            let normal = ctx.world.contact(contact).manifold.normal;
                            eval_cp(ctx, &mut cp, body1, body2, &mp, normal);
                            cp.id = id;
                            crate::flow!(listener::persist(ctx, &cp));
                        }
                        break;
                    }
                }
                j += 1;
            }
            if found == false && has_listener {
                let mp = ctx.world.contact(contact).manifold.points[iu];
                let normal = ctx.world.contact(contact).manifold.normal;
                eval_cp(ctx, &mut cp, body1, body2, &mp, normal);
                cp.id = id;
                crate::flow!(listener::add(ctx, &cp));
            }
            i += 1;
        }
        ctx.world.contact_mut(contact).manifold_count = 1;
    } else {
        ctx.world.contact_mut(contact).manifold_count = 0;
    }
    if !has_listener {
        return Flow::Continue;
    }
    let mut i: i32 = 0;
    while i < m0.point_count {
        let iu = i as usize;
        if !persisted[iu] {
            let mp0 = m0.points[iu];
            eval_cp(ctx, &mut cp, body1, body2, &mp0, m0.normal);
            crate::flow!(listener::remove(ctx, &cp));
        }
        i += 1;
    }
    Flow::Continue
}

// ---- b2ContactFilter -------------------------------------------------------------------------

/// b2ContactFilter.ShouldCollide (the default filter; the game never installs another one).
pub fn should_collide<R: Real>(ctx: &Ctx<R>, shape1: H, shape2: H) -> bool {
    let filter1 = ctx.world.shape(shape1).filter;
    let filter2 = ctx.world.shape(shape2).filter;
    if filter1.group_index == filter2.group_index && filter1.group_index != 0 {
        return filter1.group_index > 0;
    }
    (filter1.mask_bits & filter2.category_bits) != 0 && (filter1.category_bits & filter2.mask_bits) != 0
}

// ---- b2ContactManager ------------------------------------------------------------------------

/// b2ContactManager.PairAdded(proxyUserData1, proxyUserData2) -> pair userData: a new contact,
/// or `NULL_CONTACT` (m_nullContact) when the pair must not collide.
pub fn pair_added<R: Real>(ctx: &mut Ctx<R>, proxy_user_data1: H, proxy_user_data2: H) -> (Flow, H) {
    let mut shape1 = proxy_user_data1;
    let mut shape2 = proxy_user_data2;
    if shape1 == NONE || shape2 == NONE {
        // `(null as b2Shape).m_body` throws; only possible after an earlier aborted Commit.
        return (Flow::Abort(AbortSite::ContactManagerNullBody), NONE);
    }
    let mut body1 = ctx.world.shape(shape1).body;
    let mut body2 = ctx.world.shape(shape2).body;
    if body1 == NONE {
        // _loc5_.IsStatic() on null
        return (Flow::Abort(AbortSite::ContactManagerNullBody), NONE);
    }
    if body::is_static(ctx, body1) {
        if body2 == NONE {
            // _loc6_.IsStatic() on null
            return (Flow::Abort(AbortSite::ContactManagerNullBody), NONE);
        }
        if body::is_static(ctx, body2) {
            return (Flow::Continue, NULL_CONTACT);
        }
    }
    if ctx.world.shape(shape1).body == ctx.world.shape(shape2).body {
        return (Flow::Continue, NULL_CONTACT);
    }
    if body2 == NONE {
        // _loc6_.IsConnected(_loc5_) on null
        return (Flow::Abort(AbortSite::ContactManagerNullBody), NONE);
    }
    if body::is_connected(ctx, body2, body1) {
        return (Flow::Continue, NULL_CONTACT);
    }
    // m_contactFilter is b2ContactFilter.b2_defaultFilter (never null).
    if should_collide(ctx, shape1, shape2) == false {
        return (Flow::Continue, NULL_CONTACT);
    }
    let (f, contact) = create(ctx, shape1, shape2);
    match f {
            Flow::Continue => {}
            f => return (f, NONE),
        }
    if contact == NONE {
        return (Flow::Continue, NULL_CONTACT);
    }
    shape1 = ctx.world.contact(contact).shape1;
    shape2 = ctx.world.contact(contact).shape2;
    body1 = ctx.world.shape(shape1).body;
    body2 = ctx.world.shape(shape2).body;
    // Insert into the world contact list (head insertion).
    let world_head = ctx.world.contact_list;
    {
        let c = ctx.world.contact_mut(contact);
        c.prev = NONE;
        c.next = world_head;
    }
    if world_head != NONE {
        ctx.world.contact_mut(world_head).prev = contact;
    }
    ctx.world.contact_list = contact;
    // Connect to body 1 (m_node1).
    let e1 = edge_handle(contact, 0);
    let body1_head = ctx.world.body(body1).contact_list;
    {
        let n = &mut ctx.world.contact_mut(contact).node1;
        n.other = body2;
        n.prev = NONE;
        n.next = body1_head;
    }
    if body1_head != NONE {
        ctx.world.contact_edge_mut(body1_head).prev = e1;
    }
    ctx.world.body_mut(body1).contact_list = e1;
    // Connect to body 2 (m_node2).
    let e2 = edge_handle(contact, 1);
    let body2_head = ctx.world.body(body2).contact_list;
    {
        let n = &mut ctx.world.contact_mut(contact).node2;
        n.other = body1;
        n.prev = NONE;
        n.next = body2_head;
    }
    if body2_head != NONE {
        ctx.world.contact_edge_mut(body2_head).prev = e2;
    }
    ctx.world.body_mut(body2).contact_list = e2;
    ctx.world.contact_count += 1;
    (Flow::Continue, contact)
}

/// b2ContactManager.PairRemoved(proxyUserData1, proxyUserData2, pairUserData)
pub fn pair_removed<R: Real>(ctx: &mut Ctx<R>, _proxy_user_data1: H, _proxy_user_data2: H, pair_user_data: H) -> Flow {
    if pair_user_data == NONE {
        return Flow::Continue;
    }
    if pair_user_data == NULL_CONTACT {
        return Flow::Continue;
    }
    manager_destroy(ctx, pair_user_data)
}

/// b2ContactManager.Destroy(contact): fires `Remove` for every manifold point (when a listener is
/// installed), unlinks the contact from the world list and both body edge lists, then
/// `b2Contact.Destroy` and `--m_contactCount`. A null shape body throws at the first AS3 statement
/// that dereferences it (`ContactManagerNullBody`), leaving the earlier unlinking done.
pub fn manager_destroy<R: Real>(ctx: &mut Ctx<R>, contact: H) -> Flow {
    let (shape1, shape2, manifold_count, friction, restitution) = {
        let c = ctx.world.contact(contact);
        (c.shape1, c.shape2, c.manifold_count, c.friction, c.restitution)
    };
    if manifold_count > 0 && ctx.listener_mode != 0 {
        let body1 = ctx.world.shape(shape1).body;
        let body2 = ctx.world.shape(shape2).body;
        let mut cp = new_cp(shape1, shape2, friction, restitution);
        let mut i: i32 = 0;
        while i < manifold_count {
            // GetManifolds()[i]: the single manifold (manifoldCount is 0 or 1).
            debug_assert!(i == 0, "GetManifolds()[i] undefined for i > 0");
            let manifold = ctx.world.contact(contact).manifold;
            cp.normal.set_v(manifold.normal);
            let mut j: i32 = 0;
            while j < manifold.point_count {
                let mp = manifold.points[j as usize];
                if body1 == NONE {
                    // _loc7_.GetWorldPoint on null
                    return Flow::Abort(AbortSite::ContactManagerNullBody);
                }
                cp.position = body::get_world_point(ctx, body1, mp.local_point1);
                let v1 = body::get_linear_velocity_from_local_point(ctx, body1, mp.local_point1);
                if body2 == NONE {
                    // _loc8_.GetLinearVelocityFromLocalPoint on null
                    return Flow::Abort(AbortSite::ContactManagerNullBody);
                }
                let v2 = body::get_linear_velocity_from_local_point(ctx, body2, mp.local_point2);
                cp.velocity.set(v2.x - v1.x, v2.y - v1.y);
                cp.separation = mp.separation;
                cp.id = mp.id;
                crate::flow!(listener::remove(ctx, &cp));
                j += 1;
            }
            i += 1;
        }
    }
    // Remove from the world contact list.
    let (prev, next) = {
        let c = ctx.world.contact(contact);
        (c.prev, c.next)
    };
    if prev != NONE {
        ctx.world.contact_mut(prev).next = next;
    }
    if next != NONE {
        ctx.world.contact_mut(next).prev = prev;
    }
    if contact == ctx.world.contact_list {
        ctx.world.contact_list = next;
    }
    let body1 = ctx.world.shape(shape1).body;
    let body2 = ctx.world.shape(shape2).body;
    // Remove from body 1.
    let e1 = edge_handle(contact, 0);
    let node1 = ctx.world.contact(contact).node1;
    if node1.prev != NONE {
        ctx.world.contact_edge_mut(node1.prev).next = node1.next;
    }
    if node1.next != NONE {
        ctx.world.contact_edge_mut(node1.next).prev = node1.prev;
    }
    if body1 == NONE {
        // _loc5_.m_contactList on null
        return Flow::Abort(AbortSite::ContactManagerNullBody);
    }
    if e1 == ctx.world.body(body1).contact_list {
        ctx.world.body_mut(body1).contact_list = node1.next;
    }
    // Remove from body 2.
    let e2 = edge_handle(contact, 1);
    let node2 = ctx.world.contact(contact).node2;
    if node2.prev != NONE {
        ctx.world.contact_edge_mut(node2.prev).next = node2.next;
    }
    if node2.next != NONE {
        ctx.world.contact_edge_mut(node2.next).prev = node2.prev;
    }
    if body2 == NONE {
        // _loc6_.m_contactList on null
        return Flow::Abort(AbortSite::ContactManagerNullBody);
    }
    if e2 == ctx.world.body(body2).contact_list {
        ctx.world.body_mut(body2).contact_list = node2.next;
    }
    destroy(ctx, contact);
    ctx.world.contact_count -= 1;
    Flow::Continue
}

/// b2ContactManager.Collide: update every contact whose bodies are not both sleeping, walking
/// `m_contactList` head -> tail and re-reading `m_next` after each update.
pub fn collide<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    let mut contact = ctx.world.contact_list;
    while contact != NONE {
        let (shape1, shape2) = {
            let c = ctx.world.contact(contact);
            (c.shape1, c.shape2)
        };
        let body1 = ctx.world.shape(shape1).body;
        let body2 = ctx.world.shape(shape2).body;
        if body1 == NONE {
            // _loc2_.IsSleeping() on null
            return Flow::Abort(AbortSite::ContactManagerNullBody);
        }
        let sleeping1 = body::is_sleeping(ctx, body1);
        if sleeping1 && body2 == NONE {
            // _loc3_.IsSleeping() on null (only evaluated when body 1 sleeps)
            return Flow::Abort(AbortSite::ContactManagerNullBody);
        }
        if !(sleeping1 && body::is_sleeping(ctx, body2)) {
            crate::flow!(update(ctx, contact));
        }
        contact = ctx.world.contact(contact).next;
    }
    Flow::Continue
}
