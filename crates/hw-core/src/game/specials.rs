//! Physics of UserLevel's special objects, including null reads in sound callbacks.
use crate::b2::math::{sin_cos, Mat22, Vec2};
use crate::b2::state::{BodyDef, JointDef, ShapeDef};
use crate::b2::{body, joints, world};
use crate::flow::AbortSite;
use crate::{flow, Ctx, Flow, H, NONE};
use hw_data::{SpecialKind, SpecialRec};
use hw_real::Real;

pub const MAX_SPECIALS: usize = 128;
pub const MAX_SPECIAL_CONTACTS: usize = 640;
pub const SOUND_HANDLER: u32 = 0x10000000;
const TIRE_SOUND_HANDLER: u32 = 0x08000000;

fn register_sound<R: Real>(ctx: &mut Ctx<R>, shape: H, body: H, index: usize, tire: bool) {
    ctx.world.shape_mut(shape).handlers[0] =
        SOUND_HANDLER | if tire { TIRE_SOUND_HANDLER } else { 0 } | (index as u32 + 1);
    ctx.world.shapes.retain(shape);
    ctx.world.bodies.retain(body);
}

/// Special ADD callbacks return on sensors before the other-body mass read.
/// They select/play audio immediately; preserve only reads that can abort.
pub fn sound_contact<R: Real>(
    ctx: &mut Ctx<R>,
    tag: u32,
    own: H,
    other: H,
    velocity: Vec2<R>,
    normal: Vec2<R>,
) -> Flow {
    if ctx.world.shape(other).is_sensor.get() {
        return Flow::Continue;
    }
    let other_body = ctx.world.shape(other).body;
    if other_body == NONE {
        return Flow::Abort(AbortSite::GameNullDeref);
    }
    let mass = ctx.world.body(other_body).mass;
    let i = ((tag & !(SOUND_HANDLER | TIRE_SOUND_HANDLER)) - 1) as usize;
    let source_body = if tag & TIRE_SOUND_HANDLER != 0 {
        ctx.world.shape(own).body
    } else {
        ctx.level.specials[i].body
    };
    if !mass.eq_num(R::zero()) {
        if source_body == NONE {
            return Flow::Abort(AbortSite::GameNullDeref);
        }
        if mass.lt(ctx.world.body(source_body).mass) {
            return Flow::Continue;
        }
    }
    if hw_real::as3::math_abs(velocity.x * normal.x + velocity.y * normal.y).gt(hw_real::rl!(R; 4.0))
        && source_body == NONE
    {
        return Flow::Abort(AbortSite::GameNullDeref);
    }
    Flow::Continue
}
/// AS3 Array order, including splice(indexOf(missing),1) removing the last element.
#[derive(Clone, Copy)]
#[repr(C)]
pub struct HandleList<const N: usize> {
    pub entries: [H; N],
    pub len: u32,
}
impl<const N: usize> HandleList<N> {
    pub fn blank() -> Self {
        Self {
            entries: [NONE; N],
            len: 0,
        }
    }
    pub fn contains(&self, h: H) -> bool {
        for i in 0..self.len as usize {
            if self.entries[i] == h { return true; }
        }
        false
    }
    pub fn push(&mut self, h: H) -> Flow {
        if self.len as usize == N {
            return Flow::Abort(AbortSite::PortCapacity);
        }
        self.entries[self.len as usize] = h;
        self.len += 1;
        Flow::Continue
    }
    pub fn remove(&mut self, h: H) {
        if self.len == 0 {
            return;
        }
        let mut at = self.len as usize - 1;
        for i in 0..self.len as usize {
            if self.entries[i] == h { at = i; break; }
        }
        for i in at..self.len as usize - 1 {
            self.entries[i] = self.entries[i + 1];
        }
        self.len -= 1;
    }
}

#[derive(Clone, Copy)]
#[repr(C)]
pub struct Special<R: Real> {
    pub kind: SpecialKind,
    pub body: H,
    pub shape: H,
    pub joints: [H; 2],
    pub angle: R,
    pub sin: R,
    pub cos: R,
    pub center: Vec2<R>,
    pub lower: Vec2<R>,
    pub upper: Vec2<R>,
    pub speed: R,
    pub delay: R,
    pub counter: R,
    pub hit: crate::Flag32,
    pub bodies: HandleList<MAX_SPECIAL_CONTACTS>,
    pub bodies_to_remove: HandleList<MAX_SPECIAL_CONTACTS>,
    pub impaled_joints: [H; crate::b2::state::MAX_BODIES],
    pub impaled_counts: [i32; crate::b2::state::MAX_BODIES],
}
impl<R: Real> Special<R> {
    pub fn blank() -> Self {
        Self {
            kind: SpecialKind::Mine,
            body: NONE,
            shape: NONE,
            joints: [NONE; 2],
            angle: R::zero(),
            sin: R::zero(),
            cos: R::zero(),
            center: Vec2::zero(),
            lower: Vec2::zero(),
            upper: Vec2::zero(),
            speed: R::zero(),
            delay: R::zero(),
            counter: R::zero(),
            hit: crate::Flag32::FALSE,
            bodies: HandleList::blank(),
            bodies_to_remove: HandleList::blank(),
            impaled_joints: [NONE; crate::b2::state::MAX_BODIES],
            impaled_counts: [0; crate::b2::state::MAX_BODIES],
        }
    }
}

#[derive(Clone, Copy)]
#[repr(C)]
pub struct LevelState<R: Real> {
    pub contact_adds: [crate::Flag32; crate::b2::state::MAX_SHAPES],
    pub level_body: H,
    pub specials: [Special<R>; MAX_SPECIALS],
    pub count: u32,
    pub actions: HandleList<MAX_SPECIALS>,
    pub removals: HandleList<MAX_SPECIALS>,
    pub focus: H,
    pub dead: crate::Flag32,
    pub finished: crate::Flag32,
}
impl<R: Real> LevelState<R> {
    pub fn blank() -> Self {
        Self {
            contact_adds: [crate::Flag32::FALSE; crate::b2::state::MAX_SHAPES],
            level_body: NONE,
            specials: [Special::blank(); MAX_SPECIALS],
            count: 0,
            actions: HandleList::blank(),
            removals: HandleList::blank(),
            focus: NONE,
            dead: crate::Flag32::FALSE,
            finished: crate::Flag32::FALSE,
        }
    }
}

// These macros preserve each AS3 call's early-abort point, including mutations made before it.
macro_rules! shape {
    ($ctx:expr,$b:expr,$d:expr) => {{
        let (f, h) = body::create_shape($ctx, $b, $d);
        flow!(f);
        if h == NONE {
            return Flow::Abort(AbortSite::GameNullDeref);
        }
        h
    }};
}
macro_rules! joint {
    ($ctx:expr,$d:expr) => {{
        let (f, h) = world::create_joint($ctx, $d);
        flow!(f);
        if h == NONE {
            return Flow::Abort(AbortSite::GameNullDeref);
        }
        h
    }};
}
macro_rules! new_body {
    ($ctx:expr,$d:expr) => {{
        let h = world::create_body($ctx, $d);
        if h == NONE {
            return Flow::Abort(AbortSite::PortCapacity);
        }
        h
    }};
}

fn polygon<R: Real>(density: R, friction: R, restitution: R) -> ShapeDef<R> {
    let mut d = ShapeDef::polygon();
    d.density = density;
    d.friction = friction;
    d.restitution = restitution;
    d.filter.category_bits = 8;
    d
}

/// Constructors run in display-list order. Only IBeam and Spikes accept a group body.
pub fn create<R: Real>(
    ctx: &mut Ctx<R>,
    rec: &SpecialRec,
    group: H,
    offset: Option<Vec2<R>>,
    out: &mut H,
) -> Flow {
    if ctx.level.count == (MAX_SPECIALS as u32) {
        return Flow::Abort(AbortSite::PortCapacity);
    }
    let index = ctx.level.count;
    let mut s = Special::blank();
    let mut van_tire_shapes = [NONE; 2];
    s.kind = rec.kind;
    s.body = group;
    let mut pos = Vec2::new(R::from_words(rec.x), R::from_words(rec.y));
    if let Some(o) = offset {
        pos.add(o);
    }
    let scale: R = hw_real::rl!(R; 62.5);
    let pi: R = hw_real::rl!(R; core::f64::consts::PI);
    let angle = R::from_words(rec.rotation) * pi / hw_real::rl!(R; 180.0);
    s.angle = angle;
    (s.cos, s.sin) = sin_cos(angle);
    let p = Vec2::new(pos.x / scale, pos.y / scale);
    let ground = ctx.level.level_body;
    let mut bd = BodyDef::new();
    bd.position = p;
    bd.angle = angle;
    match rec.kind {
        SpecialKind::IBeam => {
            let mut d = polygon::<R>(hw_real::rl!(R; 75.0), hw_real::rl!(R; 0.3), hw_real::rl!(R; 0.1));
            let hx = R::from_words(rec.shape_width) * hw_real::rl!(R; 0.5) / scale;
            let hy = R::from_words(rec.shape_height) * hw_real::rl!(R; 0.5) / scale;
            if rec.immovable2 {
                d.set_as_oriented_box(hx, hy, Some(p), angle);
                s.shape = shape!(ctx, ground, &d);
            } else if group != NONE {
                d.set_as_oriented_box(hx, hy, Some(p), angle);
                s.shape = shape!(ctx, group, &d);
            } else {
                d.set_as_box(hx, hy);
                s.body = new_body!(ctx, &bd);
                s.shape = shape!(ctx, s.body, &d);
                flow!(body::set_mass_from_shapes(ctx, s.body));
            }
            if !rec.immovable2 {
                register_sound(ctx, s.shape, s.body, index as usize, false);
            }
        }
        SpecialKind::SoccerBall => {
            let mut d = ShapeDef::circle();
            d.density = hw_real::rl!(R; 0.5);
            d.friction = hw_real::rl!(R; 0.1);
            d.restitution = hw_real::rl!(R; 0.5);
            d.filter.category_bits = 8;
            d.radius = hw_real::rl!(R; 10.0) / scale;
            bd.angle = R::zero();
            s.body = new_body!(ctx, &bd);
            s.shape = shape!(ctx, s.body, &d);
            flow!(body::set_mass_from_shapes(ctx, s.body));
        }
        SpecialKind::Building1 | SpecialKind::Building2 => {
            let (w, h) = if rec.kind == SpecialKind::Building1 {
                (
                    R::from_words(rec.floor_width) * hw_real::rl!(R; 300.0),
                    R::from_words(rec.num_floors) * hw_real::rl!(R; 165.0) + hw_real::rl!(R; 100.0),
                )
            } else {
                (
                    R::from_words(rec.floor_width) * hw_real::rl!(R; 500.0) + hw_real::rl!(R; 226.0),
                    R::from_words(rec.num_floors) * hw_real::rl!(R; 180.0) + hw_real::rl!(R; 120.0),
                )
            };
            let mut d = polygon::<R>(hw_real::rl!(R; 0.0), hw_real::rl!(R; 0.3), hw_real::rl!(R; 0.1));
            d.set_as_oriented_box(
                w * hw_real::rl!(R; 0.5) / scale,
                h * hw_real::rl!(R; 0.5) / scale,
                Some(Vec2::new(
                    (pos.x + R::zero() + w * hw_real::rl!(R; 0.5)) / scale,
                    (pos.y + R::zero() + h * hw_real::rl!(R; 0.5)) / scale,
                )),
                R::zero(),
            );
            s.shape = shape!(ctx, ground, &d);
        }
        SpecialKind::Boost => {
            s.angle = (R::from_words(rec.rotation) + hw_real::rl!(R; 90.0)) * pi / hw_real::rl!(R; 180.0);
            (s.cos, s.sin) = sin_cos(s.angle);
            let mut d = ShapeDef::polygon();
            d.filter.category_bits = 8;
            d.is_sensor = crate::Flag32::TRUE;
            d.set_as_oriented_box(
                hw_real::rl!(R; 90.0) * R::from_words(rec.num_panels) / scale,
                hw_real::rl!(R; 90.0) / scale,
                Some(p),
                angle,
            );
            s.shape = shape!(ctx, ground, &d);
        }
        SpecialKind::Fan => {
            let mut d = ShapeDef::polygon();
            d.filter.category_bits = 8;
            d.is_sensor = crate::Flag32::TRUE;
            s.center = Vec2::new(
                (pos.x + s.sin * hw_real::rl!(R; 300.0)) / scale,
                (pos.y - s.cos * hw_real::rl!(R; 300.0)) / scale,
            );
            d.set_as_oriented_box(
                hw_real::rl!(R; 150.0) / scale,
                hw_real::rl!(R; 250.0) / scale,
                Some(s.center),
                angle,
            );
            s.shape = shape!(ctx, ground, &d);
            d.is_sensor = crate::Flag32::FALSE;
            d.friction = hw_real::rl!(R; 0.3);
            d.restitution = hw_real::rl!(R; 0.1);
            d.set_as_oriented_box(
                hw_real::rl!(R; 150.0) / scale,
                hw_real::rl!(R; 25.0) / scale,
                Some(Vec2::new(
                    (pos.x - s.sin * -hw_real::rl!(R; 25.0)) / scale,
                    (pos.y + s.cos * -hw_real::rl!(R; 25.0)) / scale,
                )),
                angle,
            );
            shape!(ctx, ground, &d);
            d.vertex_count = 7;
            let m = Mat22::from_angle(angle);
            for (i, (x, y)) in [
                (hw_real::rl!(R; 150.0), hw_real::rl!(R; 0.0)),
                (hw_real::rl!(R; 125.0), hw_real::rl!(R; 84.5)),
                (hw_real::rl!(R; 65.0), hw_real::rl!(R; 137.0)),
                (hw_real::rl!(R; 0.0), hw_real::rl!(R; 150.0)),
                (-hw_real::rl!(R; 65.0), hw_real::rl!(R; 137.0)),
                (-hw_real::rl!(R; 125.0), hw_real::rl!(R; 84.5)),
                (-hw_real::rl!(R; 150.0), hw_real::rl!(R; 0.0)),
            ]
            .iter()
            .enumerate()
            {
                let mut v = Vec2::new(*x / scale, *y / scale);
                v.mul_m(m);
                v.add(p);
                d.vertices[i] = v;
            }
            shape!(ctx, ground, &d);
        }
        SpecialKind::FinishLine => {
            let mut d = polygon::<R>(hw_real::rl!(R; 0.0), hw_real::rl!(R; 0.3), hw_real::rl!(R; 0.1));
            d.set_as_oriented_box(
                hw_real::rl!(R; 200.0) / scale,
                hw_real::rl!(R; 20.0) / scale,
                Some(p),
                R::zero(),
            );
            s.shape = shape!(ctx, ground, &d);
            s.lower = Vec2::new((pos.x - hw_real::rl!(R; 200.0)) / scale, (pos.y - hw_real::rl!(R; 200.0)) / scale);
            s.upper = Vec2::new((pos.x + hw_real::rl!(R; 200.0)) / scale, pos.y / scale);
        }
        SpecialKind::Mine => {
            s.body = new_body!(ctx, &bd);
            let mut d = polygon::<R>(hw_real::rl!(R; 1.0), hw_real::rl!(R; 1.0), hw_real::rl!(R; 0.1));
            d.set_as_box(hw_real::rl!(R; 25.0) * hw_real::rl!(R; 0.5) / scale, hw_real::rl!(R; 2.0) * hw_real::rl!(R; 0.5) / scale);
            shape!(ctx, s.body, &d);
            d.set_as_oriented_box(
                hw_real::rl!(R; 2.0) * hw_real::rl!(R; 0.5) / scale,
                hw_real::rl!(R; 1.0) * hw_real::rl!(R; 0.5) / scale,
                Some(Vec2::new(R::zero(), -hw_real::rl!(R; 2.5) / scale)),
                R::zero(),
            );
            s.shape = shape!(ctx, s.body, &d);
            flow!(body::set_mass_from_shapes(ctx, s.body));
        }
        SpecialKind::Spikes => {
            let half_width = hw_real::rl!(R; 15.0) * R::from_words(rec.num_spikes) / hw_real::rl!(R; 2.0);
            let mut d = polygon::<R>(hw_real::rl!(R; 0.25), hw_real::rl!(R; 0.3), hw_real::rl!(R; 0.1));
            d.is_sensor = crate::Flag32::TRUE;
            if rec.immovable2 || group != NONE {
                let b = if rec.immovable2 { ground } else { group };
                d.set_as_oriented_box(half_width / scale, hw_real::rl!(R; 25.0) / scale, Some(p), angle);
                s.shape = shape!(ctx, b, &d);
                d.is_sensor = crate::Flag32::FALSE;
                if !rec.immovable2 {
                    d.density = hw_real::rl!(R; 3.0);
                }
                d.set_as_oriented_box(
                    half_width / scale,
                    hw_real::rl!(R; 10.0) / scale,
                    Some(Vec2::new(
                        (pos.x - s.sin * hw_real::rl!(R; 35.0)) / scale,
                        (pos.y + s.cos * hw_real::rl!(R; 35.0)) / scale,
                    )),
                    angle,
                );
                let base = shape!(ctx, b, &d);
                if !rec.immovable2 {
                    register_sound(ctx, base, s.body, index as usize, false);
                }
            } else {
                d.set_as_box(half_width / scale, hw_real::rl!(R; 25.0) / scale);
                s.body = new_body!(ctx, &bd);
                s.shape = shape!(ctx, s.body, &d);
                flow!(body::set_mass_from_shapes(ctx, s.body));
                d.is_sensor = crate::Flag32::FALSE;
                d.density = hw_real::rl!(R; 3.0);
                d.set_as_oriented_box(
                    half_width / scale,
                    hw_real::rl!(R; 10.0) / scale,
                    Some(Vec2::new(R::zero(), hw_real::rl!(R; 35.0) / scale)),
                    R::zero(),
                );
                let base = shape!(ctx, s.body, &d);
                flow!(body::set_mass_from_shapes(ctx, s.body));
                register_sound(ctx, base, s.body, index as usize, false);
            }
        }
        SpecialKind::SpringBox => {
            s.body = new_body!(ctx, &bd);
            let mut d = polygon::<R>(hw_real::rl!(R; 200.0), hw_real::rl!(R; 0.5), hw_real::rl!(R; 0.1));
            d.set_as_oriented_box(
                hw_real::rl!(R; 350.0) * hw_real::rl!(R; 0.5) / scale,
                hw_real::rl!(R; 22.0) * hw_real::rl!(R; 0.5) / scale,
                Some(Vec2::new(R::zero(), -hw_real::rl!(R; 9.0) / scale)),
                R::zero(),
            );
            s.shape = shape!(ctx, s.body, &d);
            for x in [-hw_real::rl!(R; 120.0), hw_real::rl!(R; 120.0)] {
                d.set_as_oriented_box(
                    hw_real::rl!(R; 26.0) * hw_real::rl!(R; 0.5) / scale,
                    hw_real::rl!(R; 18.0) * hw_real::rl!(R; 0.5) / scale,
                    Some(Vec2::new(x / scale, hw_real::rl!(R; 11.0) / scale)),
                    R::zero(),
                );
                shape!(ctx, s.body, &d);
            }
            flow!(body::set_mass_from_shapes(ctx, s.body));
            let center =
                body::get_world_point(ctx, s.body, Vec2::new(R::zero(), hw_real::rl!(R; 4.0) / scale));
            d.set_as_oriented_box(
                hw_real::rl!(R; 350.0) * hw_real::rl!(R; 0.5) / scale,
                hw_real::rl!(R; 32.0) * hw_real::rl!(R; 0.5) / scale,
                Some(center),
                angle,
            );
            shape!(ctx, ground, &d);
            let mut j = JointDef::prismatic();
            joints::prismatic_def_initialize(
                ctx,
                &mut j,
                ground,
                s.body,
                body::get_world_center(ctx, s.body),
                Vec2::new(s.sin, -s.cos),
            );
            j.enable_limit = crate::Flag32::TRUE;
            j.max_motor_force = hw_real::rl!(R; 1000000.0);
            s.joints[0] = joint!(ctx, &j);
            s.delay = hw_real::as3::math_round(R::from_words(rec.spring_delay) * hw_real::rl!(R; 30.0));
            s.counter = s.delay;
        }
        SpecialKind::Van => {
            s.body = new_body!(ctx, &bd);
            let mut d = polygon::<R>(hw_real::rl!(R; 10.0), hw_real::rl!(R; 0.3), hw_real::rl!(R; 0.1));
            d.set_as_box(
                hw_real::rl!(R; 132.0) * hw_real::rl!(R; 0.5) / scale,
                hw_real::rl!(R; 116.0) * hw_real::rl!(R; 0.5) / scale,
            );
            s.shape = shape!(ctx, s.body, &d);
            flow!(body::set_mass_from_shapes(ctx, s.body));
            bd.position = body::get_world_point(
                ctx,
                s.body,
                Vec2::new(-hw_real::rl!(R; 58.0) / scale, hw_real::rl!(R; 48.0) / scale),
            );
            let left = new_body!(ctx, &bd);
            d.friction = R::one();
            d.restitution = hw_real::rl!(R; 0.3);
            d.set_as_box(hw_real::rl!(R; 20.0) * hw_real::rl!(R; 0.5) / scale, hw_real::rl!(R; 45.0) * hw_real::rl!(R; 0.5) / scale);
            van_tire_shapes[0] = shape!(ctx, left, &d);
            flow!(body::set_mass_from_shapes(ctx, left));
            bd.position = body::get_world_point(
                ctx,
                s.body,
                Vec2::new(hw_real::rl!(R; 58.0) / scale, hw_real::rl!(R; 48.0) / scale),
            );
            let right = new_body!(ctx, &bd);
            van_tire_shapes[1] = shape!(ctx, right, &d);
            flow!(body::set_mass_from_shapes(ctx, right));
            let mut j = JointDef::revolute();
            j.enable_limit = crate::Flag32::TRUE;
            joints::revolute_def_initialize(
                ctx,
                &mut j,
                s.body,
                left,
                body::get_world_center(ctx, left),
            );
            s.joints[0] = joint!(ctx, &j);
            joints::revolute_def_initialize(
                ctx,
                &mut j,
                s.body,
                right,
                body::get_world_center(ctx, right),
            );
            s.joints[1] = joint!(ctx, &j);
        }
        SpecialKind::WreckingBall => {
            let length = R::from_words(rec.rope_length);
            bd.position = Vec2::new((pos.x + length) / scale, pos.y / scale);
            bd.angle = -hw_real::rl!(R; 90.0) * pi / hw_real::rl!(R; 180.0);
            s.body = new_body!(ctx, &bd);
            let mut d = polygon::<R>(hw_real::rl!(R; 500.0), hw_real::rl!(R; 0.1), hw_real::rl!(R; 0.1));
            d.set_as_oriented_box(
                hw_real::rl!(R; 18.0) / scale,
                hw_real::rl!(R; 7.5) / scale,
                Some(Vec2::new(R::zero(), -hw_real::rl!(R; 80.0) / scale)),
                R::zero(),
            );
            shape!(ctx, s.body, &d);
            let mut c = ShapeDef::circle();
            c.density = hw_real::rl!(R; 500.0);
            c.friction = hw_real::rl!(R; 0.1);
            c.restitution = hw_real::rl!(R; 0.1);
            c.filter.category_bits = 8;
            c.radius = hw_real::rl!(R; 75.0) / scale;
            s.shape = shape!(ctx, s.body, &c);
            flow!(body::set_mass_from_shapes(ctx, s.body));
            register_sound(ctx, s.shape, s.body, index as usize, false);
            d.set_as_oriented_box(
                hw_real::rl!(R; 18.0) / scale,
                hw_real::rl!(R; 16.0) / scale,
                Some(Vec2::new(pos.x / scale, (pos.y - hw_real::rl!(R; 16.0)) / scale)),
                R::zero(),
            );
            shape!(ctx, ground, &d);
            let mut j = JointDef::revolute();
            joints::revolute_def_initialize(ctx, &mut j, ground, s.body, p);
            j.enable_limit = crate::Flag32::TRUE;
            j.upper_angle = pi + hw_real::rl!(R; 0.25) * pi;
            j.lower_angle = -hw_real::rl!(R; 0.25) * pi;
            j.max_motor_torque = hw_real::rl!(R; 10000000.0);
            s.joints[0] = joint!(ctx, &j);
            s.speed = hw_real::rl!(R; 1.75) * hw_real::rl!(R; 1000.0) / length;
        }
        // These classes are absent from the nine demo levels; fail rather than approximate them.
        _ => return Flow::Abort(AbortSite::PortUnimplemented),
    }
    *out = s.body;
    let tag = index as u32 + 1;
    match s.kind {
        SpecialKind::SpringBox
        | SpecialKind::WreckingBall
        | SpecialKind::Spikes
        | SpecialKind::Boost
        | SpecialKind::Fan => {
            flow!(ctx.level.actions.push(index as H));
        }
        _ => {}
    }
    match s.kind {
        SpecialKind::Boost | SpecialKind::Fan => ctx.world.shape_mut(s.shape).handlers[2] = tag,
        SpecialKind::Spikes => {
            ctx.world.shape_mut(s.shape).handlers[0] = tag;
            ctx.world.shape_mut(s.shape).handlers[1] = tag;
        }
        SpecialKind::Mine | SpecialKind::Van | SpecialKind::SpringBox | SpecialKind::FinishLine => {
            ctx.world.shape_mut(s.shape).handlers[3] = tag
        }
        _ => {}
    }
    if s.kind == SpecialKind::Van {
        register_sound(ctx, s.shape, s.body, index as usize, false);
        for shape in van_tire_shapes {
            register_sound(ctx, shape, ctx.world.shape(shape).body, index as usize, true);
        }
        // Van remains in LevelB2D.paintArray after its smash. Its vanShape and
        // tire-joint fields still point at the unlinked AS3 objects; later
        // allocations must not alias those retained records.
        ctx.world.shapes.retain(s.shape);
        ctx.world.joints.retain(s.joints[0]);
        ctx.world.joints.retain(s.joints[1]);
    }
    ctx.level.specials[index as usize] = s;
    ctx.level.count += 1;
    Flow::Continue
}

/// Callback's own shape is first, as in ContactListener's reflected records.
pub fn contact<R: Real>(ctx: &mut Ctx<R>, event: usize, own: H, other: H, impulse: R) -> Flow {
    let tag = ctx.world.shape(own).handlers[event];
    if tag == 0 {
        return Flow::Continue;
    }
    let i = (tag - 1) as usize;
    if i >= (ctx.level.count as usize) {
        return Flow::Abort(AbortSite::PortUnimplemented);
    }
    match ctx.level.specials[i].kind {
        SpecialKind::Boost | SpecialKind::Fan if event == 2 => {
            let b = ctx.world.shape(other).body;
            if !ctx.level.specials[i].bodies.contains(b) {
                flow!(ctx.level.specials[i].bodies.push(b));
            }
        }
        SpecialKind::Mine if event == 3 => {
            ctx.world.shape_mut(own).handlers[3] = 0;
            flow!(ctx.level.actions.push(i as H));
        }
        SpecialKind::Van if event == 3 && impulse.gt(hw_real::rl!(R; 150.0)) => {
            ctx.world.shape_mut(own).handlers[3] = 0;
            ctx.world.shape_mut(own).handlers[0] = 0;
            flow!(ctx.level.actions.push(i as H));
        }
        SpecialKind::SpringBox if event == 3 => {
            ctx.world.shape_mut(own).handlers[3] = 0;
            ctx.level.specials[i].hit = crate::Flag32::TRUE;
        }
        SpecialKind::FinishLine if event == 3 => {
            if ctx.world.shape(own).id_tag != 0 {
                return Flow::Continue;
            }
            if ctx.level.dead.get() {
                lab! {
                    crate::lab::record(crate::lab::Event::Finish { x: f64::NAN, y: f64::NAN, inside: false, dead: true, other });
                }
                ctx.world.shape_mut(own).handlers[3] = 0;
                return Flow::Continue;
            }
            if ctx.level.focus == NONE {
                return Flow::Abort(AbortSite::GameNullDeref);
            }
            let p = body::get_world_center(ctx, ctx.level.focus);
            let s = &ctx.level.specials[i];
            let inside = p.x.gt(s.lower.x) && p.x.lt(s.upper.x) && p.y.gt(s.lower.y) && p.y.lt(s.upper.y);
            lab! {
                crate::lab::record(crate::lab::Event::Finish {
                    x: crate::lab::f(p.x),
                    y: crate::lab::f(p.y),
                    inside,
                    dead: false,
                    other,
                });
            }
            if inside {
                ctx.world.shape_mut(own).handlers[3] = 0;
                ctx.level.finished = crate::Flag32::TRUE;
            }
        }
        SpecialKind::Spikes if ctx.world.shape(other).material == 1 => {
            let b = ctx.world.shape(other).body;
            if event == 0 {
                flow!(ctx.level.specials[i].bodies.push(b));
            } else if event == 1
                && (ctx.level.specials[i].impaled_joints[b as usize] != NONE
                    || ctx.level.specials[i].bodies.contains(b))
            {
                flow!(ctx.level.specials[i].bodies_to_remove.push(b));
            }
        }
        _ => {}
    }
    Flow::Continue
}

pub fn actions<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    let mut at = 0;
    while at < ctx.level.actions.len {
        let i = ctx.level.actions.entries[at as usize] as usize;
        flow!(action(ctx, i));
        at += 1;
    }
    at = 0;
    while at < ctx.level.removals.len {
        let i = ctx.level.removals.entries[at as usize];
        // LevelItem.die removes display children only for the two one-shot actions here.
        ctx.level.actions.remove(i);
        at += 1;
    }
    ctx.level.removals.len = 0;
    Flow::Continue
}

fn action<R: Real>(ctx: &mut Ctx<R>, i: usize) -> Flow {
    let s = ctx.level.specials[i];
    let j = s.joints[0];
    let scale: R = hw_real::rl!(R; 62.5);
    match s.kind {
        SpecialKind::Spikes => {
            let mut at = 0;
            while at < ctx.level.specials[i].bodies.len {
                let b = ctx.level.specials[i].bodies.entries[at as usize];
                if ctx.level.specials[i].impaled_joints[b as usize] != NONE {
                    ctx.level.specials[i].impaled_counts[b as usize] =
                        ctx.level.specials[i].impaled_counts[b as usize].wrapping_add(1);
                } else {
                    flow!(super::character::shape_impale(
                        ctx,
                        body::get_shape_list(ctx, b)
                    ));
                    let mut d = JointDef::prismatic();
                    let center = body::get_world_center(ctx, b);
                    let anchor = if s.body != NONE {
                        s.body
                    } else {
                        ctx.level.level_body
                    };
                    let (cos, sin) = sin_cos(s.angle);
                    joints::prismatic_def_initialize(
                        ctx,
                        &mut d,
                        anchor,
                        b,
                        center,
                        Vec2::new(-sin, cos),
                    );
                    d.collide_connected = crate::Flag32::TRUE;
                    d.enable_motor = crate::Flag32::TRUE;
                    d.max_motor_force = hw_real::rl!(R; 30.0);
                    d.motor_speed = R::zero();
                    let created = joint!(ctx, &d);
                    ctx.level.specials[i].impaled_joints[b as usize] = created;
                    ctx.level.specials[i].impaled_counts[b as usize] = 1;
                }
                at += 1;
            }
            ctx.level.specials[i].bodies.len = 0;
            at = 0;
            while at < ctx.level.specials[i].bodies_to_remove.len {
                let b = ctx.level.specials[i].bodies_to_remove.entries[at as usize];
                let count = ctx.level.specials[i].impaled_counts[b as usize].wrapping_sub(1);
                if count == 0 {
                    let j = ctx.level.specials[i].impaled_joints[b as usize];
                    if j == NONE {
                        return Flow::Abort(AbortSite::GameNullDeref);
                    }
                    flow!(world::destroy_joint(ctx, j));
                    ctx.level.specials[i].impaled_joints[b as usize] = NONE;
                    ctx.level.specials[i].impaled_counts[b as usize] = 0;
                } else {
                    ctx.level.specials[i].impaled_counts[b as usize] = count;
                }
                at += 1;
            }
            ctx.level.specials[i].bodies_to_remove.len = 0;
        }
        SpecialKind::Boost | SpecialKind::Fan => {
            let mut at = 0;
            while at < ctx.level.specials[i].bodies.len {
                let b = ctx.level.specials[i].bodies.entries[at as usize];
                let p = body::get_world_center(ctx, b);
                let f = if s.kind == SpecialKind::Boost {
                    hw_real::rl!(R; 20.0) * body::get_mass(ctx, b)
                } else {
                    let d = Vec2::new(p.x - s.center.x, p.y - s.center.y);
                    let t = (d.x * -s.sin + d.y * s.cos) * scale + hw_real::rl!(R; 250.0);
                    let t = hw_real::as3::math_max(t, R::zero());
                    let t = hw_real::as3::math_min(t, hw_real::rl!(R; 250.0));
                    let z = t / hw_real::rl!(R; 250.0);
                    z * z * hw_real::rl!(R; 15.0)
                };
                body::apply_force(ctx, b, Vec2::new(s.sin * f, s.cos * -f), p);
                at += 1;
            }
            ctx.level.specials[i].bodies.len = 0;
        }
        SpecialKind::SpringBox => {
            if s.hit.get() {
                if s.counter.eq_num(R::zero()) {
                    ctx.level.specials[i].hit = crate::Flag32::FALSE;
                    body::wake_up(ctx, s.body);
                    joints::set_motor_speed(ctx, j, hw_real::rl!(R; 10.0));
                    joints::set_limits(ctx, j, R::zero(), hw_real::rl!(R; 32.0) / scale);
                    joints::enable_motor(ctx, j, true);
                    ctx.level.specials[i].counter = s.delay;
                } else {
                    ctx.level.specials[i].counter = s.counter - R::one();
                }
            } else if joints::is_motor_enabled(ctx, j) {
                let speed = joints::get_motor_speed(ctx, j);
                if speed.gt(R::zero()) {
                    if (joints::get_joint_translation(ctx, j) * scale).gt(hw_real::rl!(R; 32.0)) {
                        joints::set_motor_speed(ctx, j, -R::one());
                    }
                } else if speed.lt(R::zero())
                    && (joints::get_joint_translation(ctx, j) * scale).lt(R::zero())
                {
                    joints::enable_motor(ctx, j, false);
                    joints::set_limits(ctx, j, R::zero(), R::zero());
                    joints::set_motor_speed(ctx, j, R::zero());
                    ctx.world.shape_mut(s.shape).handlers[3] = i as u32 + 1;
                }
            }
        }
        SpecialKind::WreckingBall => {
            let angle = joints::get_joint_angle(ctx, j);
            let enabled = joints::is_motor_enabled(ctx, j);
            if angle.lt(hw_real::rl!(R; 0.3)) {
                if enabled {
                    joints::enable_motor(ctx, j, false);
                    if s.speed.lt(R::zero()) {
                        ctx.level.specials[i].speed = s.speed * -hw_real::rl!(R; 1.0);
                    }
                    joints::set_motor_speed(ctx, j, ctx.level.specials[i].speed);
                }
            } else if angle.gt(hw_real::rl!(R; 2.84)) {
                if enabled {
                    joints::enable_motor(ctx, j, false);
                    if s.speed.gt(R::zero()) {
                        ctx.level.specials[i].speed = s.speed * -hw_real::rl!(R; 1.0);
                    }
                    joints::set_motor_speed(ctx, j, ctx.level.specials[i].speed);
                }
            } else if !enabled {
                joints::enable_motor(ctx, j, true);
                joints::set_motor_speed(ctx, j, s.speed);
            }
        }
        SpecialKind::Van => {
            flow!(ctx.level.removals.push(i as H));
            flow!(body::destroy_shape(
                ctx,
                s.body,
                body::get_shape_list(ctx, s.body)
            ));
            let mut d = polygon::<R>(hw_real::rl!(R; 10.0), hw_real::rl!(R; 0.3), hw_real::rl!(R; 0.1));
            d.set_as_box(
                hw_real::rl!(R; 132.0) * hw_real::rl!(R; 0.5) / scale,
                hw_real::rl!(R; 105.0) * hw_real::rl!(R; 0.5) / scale,
            );
            shape!(ctx, s.body, &d);
            flow!(body::set_mass_from_shapes(ctx, s.body));
            flow!(world::destroy_joint(ctx, s.joints[0]));
            flow!(world::destroy_joint(ctx, s.joints[1]));
        }
        SpecialKind::Mine => {
            flow!(world::destroy_body(ctx, s.body));
            let center = body::get_world_center(ctx, s.body);
            let radius: R = hw_real::rl!(R; 2.0);
            let mut results = [NONE; 30];
            let (query_flow, count) = world::query(
                ctx,
                Vec2::new(center.x - radius, center.y - radius),
                Vec2::new(center.x + radius, center.y + radius),
                &mut results,
                30,
            );
            flow!(query_flow);
            for k in 0..count as usize {
                let b = ctx.world.shape(results[k]).body;
                if !body::is_static(ctx, b) {
                    let p = body::get_world_center(ctx, b);
                    let d = Vec2::new(p.x - center.x, p.y - center.y);
                    let length = hw_real::as3::math_min(radius, d.length());
                    let factor = R::one() - length / radius;
                    let angle = crate::b2::math::atan2(d.y, d.x);
                    let (cos, sin) = sin_cos(angle);
                    body::apply_impulse(
                        ctx,
                        b,
                        Vec2::new(cos * factor * hw_real::rl!(R; 10.0), sin * factor * hw_real::rl!(R; 10.0)),
                        p,
                    );
                }
            }
            flow!(ctx.level.removals.push(i as H));
        }
        _ => {}
    }
    Flow::Continue
}
