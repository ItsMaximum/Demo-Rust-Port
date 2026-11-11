//! UserLevel construction, in the order of the Flash create methods.
//! Display properties are the measured getters; arithmetic here follows UserLevel.as.
use hw_data::{LevelTable, ShapeClass, ShapeRec};
use hw_real::{rl, Real};

use crate::b2::math::{sin_cos, Vec2};
use crate::b2::state::{BodyDef, ShapeDef};
use crate::b2::{body, shapes, world};
use crate::flow::AbortSite;
use crate::{flow, Ctx, Flow, H, NONE};

pub const MAX_LEVEL_SHAPES: usize = 300;
pub const LEVEL_ADD_HANDLER: u32 = 0x20000000;

/// Creation-time dictionaries (not simulation scratch; later joints refer to these handles).
#[derive(Clone, Copy)]
#[repr(C)]
pub struct LevelBuild {
    pub level_body: H,
    pub shape_bodies: [H; MAX_LEVEL_SHAPES],
    pub special_bodies: [H; super::specials::MAX_SPECIALS],
    pub group_bodies: [H; MAX_LEVEL_SHAPES],
}

impl LevelBuild {
    pub fn blank() -> Self {
        Self {
            level_body: NONE,
            shape_bodies: [NONE; MAX_LEVEL_SHAPES],
            special_bodies: [NONE; super::specials::MAX_SPECIALS],
            group_bodies: [NONE; MAX_LEVEL_SHAPES],
        }
    }
}

pub fn create_world<R: Real>(ctx: &mut Ctx<R>) {
    world::new_world(
        ctx,
        Vec2::new(rl!(-4.0), rl!(-84.0)),
        Vec2::new(rl!(324.0), rl!(164.0)),
        Vec2::new(R::zero(), rl!(10.0)),
        true,
    );
}

fn material<R: Real>(def: &mut ShapeDef<R>) {
    def.friction = R::one();
    def.restitution = rl!(0.1);
    def.filter.category_bits = 8;
}

fn triangle<R: Real>(def: &mut ShapeDef<R>, shape: &ShapeRec) {
    let scale: R = rl!(62.5);
    let hx = R::from_words(shape.scale_x) * R::from_i32(50) / scale;
    let hy = R::from_words(shape.scale_y) * R::from_i32(100) / scale;
    def.vertex_count = 3;
    def.vertices[0] = Vec2::new(R::zero(), hy * R::from_i32(-2));
    def.vertices[1] = Vec2::new(hx, hy);
    def.vertices[2] = Vec2::new(-hx, hy);
}

fn rotate_triangle<R: Real>(def: &mut ShapeDef<R>, position: Vec2<R>, angle: R) {
    for i in 0..3 {
        let v = def.vertices[i];
        let x = sin_cos(angle).0 * v.x - sin_cos(angle).1 * v.y;
        let y = sin_cos(angle).0 * v.y + sin_cos(angle).1 * v.x;
        def.vertices[i] = Vec2::new(position.x + x, position.y + y);
    }
}

pub fn create_static_shapes<R: Real>(
    ctx: &mut Ctx<R>,
    level: &LevelTable,
    build: &mut LevelBuild,
) -> Flow {
    let scale: R = rl!(62.5);
    let pi: R = rl!(3.141592653589793);
    let ground = world::create_body(ctx, &BodyDef::new());
    if ground == NONE {
        return Flow::Abort(AbortSite::PortCapacity);
    }
    build.level_body = ground;
    ctx.level.level_body = ground;
    let mut polygon = ShapeDef::polygon();
    let mut circle = ShapeDef::circle();
    let mut tri = ShapeDef::polygon();
    material(&mut polygon);
    material(&mut circle);
    material(&mut tri);
    polygon.filter.group_index = -10;
    circle.filter.group_index = -10;
    tri.filter.group_index = -10;
    tri.vertex_count = 3;
    let border = R::from_i32(250);
    let sky = R::from_i32(5000);
    let height = R::from_i32(10000);
    let width = R::from_i32(20000);
    let half: R = rl!(0.5);
    let mut hx = border * half / scale;
    let mut hy = (border + (height + sky) * half) / scale;
    let mut position = Vec2::new(-hx, (height - sky) * half / scale);
    shapes::set_as_oriented_box(&mut polygon, hx, hy, position, R::zero());
    flow!(body::create_shape(ctx, ground, &polygon).0);
    position.set(
        (width + border * half) / scale,
        (height - sky) * half / scale,
    );
    shapes::set_as_oriented_box(&mut polygon, hx, hy, position, R::zero());
    flow!(body::create_shape(ctx, ground, &polygon).0);
    hx = (border + width * half) / scale;
    hy = border * half / scale;
    position.set(width * half / scale, -(border * half + sky) / scale);
    shapes::set_as_oriented_box(&mut polygon, hx, hy, position, R::zero());
    flow!(body::create_shape(ctx, ground, &polygon).0);
    for shape in level.shapes {
        if !shape.immovable || !shape.interactive {
            continue;
        }
        let angle = R::from_words(shape.rotation) * pi / R::from_i32(180);
        position.set(
            R::from_words(shape.x) / scale,
            R::from_words(shape.y) / scale,
        );
        match shape.class {
            ShapeClass::Rectangle => {
                shapes::set_as_oriented_box(
                    &mut polygon,
                    R::from_words(shape.scale_x) * R::from_i32(50) / scale,
                    R::from_words(shape.scale_y) * R::from_i32(50) / scale,
                    position,
                    angle,
                );
                flow!(body::create_shape(ctx, ground, &polygon).0);
            }
            ShapeClass::Circle => {
                circle.local_position = position;
                circle.radius = R::from_words(shape.scale_x) * R::from_i32(50) / scale;
                flow!(body::create_shape(ctx, ground, &circle).0);
            }
            ShapeClass::Triangle => {
                triangle(&mut tri, shape);
                rotate_triangle(&mut tri, position, angle);
                flow!(body::create_shape(ctx, ground, &tri).0);
            }
        }
    }
    Flow::Continue
}

pub fn create_dynamic_shapes<R: Real>(
    ctx: &mut Ctx<R>,
    level: &LevelTable,
    build: &mut LevelBuild,
) -> Flow {
    let scale: R = rl!(62.5);
    let pi: R = rl!(3.141592653589793);
    if level.shapes.len() > MAX_LEVEL_SHAPES {
        return Flow::Abort(AbortSite::PortCapacity);
    }
    for (i, shape) in level.shapes.iter().enumerate() {
        if shape.immovable || !shape.interactive {
            continue;
        }
        let angle = R::from_words(shape.rotation) * pi / R::from_i32(180);
        let position = Vec2::new(
            R::from_words(shape.x) / scale,
            R::from_words(shape.y) / scale,
        );
        let mut bd = BodyDef::new();
        bd.position = position;
        bd.angle = angle;
        bd.is_sleeping = crate::Flag32::new(shape.sleeping);
        let b = world::create_body(ctx, &bd);
        if b == NONE {
            return Flow::Abort(AbortSite::PortCapacity);
        }
        let mut def = match shape.class {
            ShapeClass::Circle => ShapeDef::circle(),
            _ => ShapeDef::polygon(),
        };
        def.density = R::from_words(shape.density);
        material(&mut def);
        match shape.class {
            ShapeClass::Rectangle => shapes::set_as_box(
                &mut def,
                R::from_words(shape.scale_x) * R::from_i32(50) / scale,
                R::from_words(shape.scale_y) * R::from_i32(50) / scale,
            ),
            ShapeClass::Circle => {
                def.local_position.set(R::zero(), R::zero());
                def.radius = R::from_words(shape.scale_x) * R::from_i32(50) / scale;
            }
            ShapeClass::Triangle => triangle(&mut def, shape),
        }
        let (result, shape_handle) = body::create_shape(ctx, b, &def);
        flow!(result);
        flow!(body::set_mass_from_shapes(ctx, b));
        build.shape_bodies[i] = b;
        if ctx.world.body(b).mass.gt(R::one()) {
            ctx.world.shape_mut(shape_handle).handlers[0] = LEVEL_ADD_HANDLER;
            // UserLevel.contactAddSounds retains this key for the session.
            ctx.world.shapes.retain(shape_handle);
        }
    }
    Flow::Continue
}

/// UserLevel.shapeContactAdd. Audio is omitted, but these reads can throw before
/// the sensor/buffer guards and therefore affect the remainder of World.Step.
pub fn contact_add<R: Real>(
    ctx: &mut Ctx<R>,
    own: H,
    other: H,
    velocity: Vec2<R>,
    normal: Vec2<R>,
) -> Flow {
    let own_body = ctx.world.shape(own).body;
    let other_body = ctx.world.shape(other).body;
    if other_body == NONE {
        return Flow::Abort(AbortSite::GameNullDeref);
    }
    let mass = ctx.world.body(other_body).mass;
    if ctx.level.contact_adds[own as usize].get() || ctx.world.shape(other).is_sensor.get() {
        return Flow::Continue;
    }
    if !mass.eq_num(R::zero()) {
        if own_body == NONE {
            return Flow::Abort(AbortSite::GameNullDeref);
        }
        if mass.lt(ctx.world.body(own_body).mass) {
            return Flow::Continue;
        }
    }
    if hw_real::as3::math_abs(velocity.x * normal.x + velocity.y * normal.y).gt(rl!(4.0)) {
        ctx.level.contact_adds[own as usize] = crate::Flag32::TRUE;
    }
    Flow::Continue
}

/// UserLevel.handleContactAdds follows LevelB2D.actions. AreaSoundInstance reads
/// the source body's center before any volume/cutoff check; keep that null guard.
pub fn consume_adds<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    for i in 0..crate::b2::state::MAX_SHAPES {
        if ctx.level.contact_adds[i].get() {
            if ctx.world.shape(i as H).body == NONE {
                return Flow::Abort(AbortSite::GameNullDeref);
            }
            ctx.level.contact_adds[i] = crate::Flag32::FALSE;
        }
    }
    Flow::Continue
}

pub fn create_specials<R: Real>(
    ctx: &mut Ctx<R>,
    level: &LevelTable,
    build: &mut LevelBuild,
) -> Flow {
    if level.specials.len() > build.special_bodies.len() {
        return Flow::Abort(AbortSite::PortCapacity);
    }
    for (i, rec) in level.specials.iter().enumerate() {
        flow!(super::specials::create(
            ctx,
            rec,
            NONE,
            None,
            &mut build.special_bodies[i]
        ));
    }
    Flow::Continue
}

pub fn create_groups<R: Real>(
    ctx: &mut Ctx<R>,
    level: &LevelTable,
    build: &mut LevelBuild,
) -> Flow {
    if level.groups.len() > build.group_bodies.len() {
        return Flow::Abort(AbortSite::PortCapacity);
    }
    let scale: R = rl!(62.5);
    let deg: R = rl!(3.141592653589793);
    let deg = deg / R::from_i32(180);
    for (i, g) in level.groups.iter().enumerate() {
        if g.shapes_used <= 0 {
            continue;
        }
        let mut bd = BodyDef::new();
        bd.position = Vec2::new(R::from_words(g.x) / scale, R::from_words(g.y) / scale);
        bd.angle = R::from_words(g.rotation) * deg;
        bd.is_sleeping = crate::Flag32::new(g.sleeping);
        let b = world::create_body(ctx, &bd);
        if b == NONE {
            return Flow::Abort(AbortSite::PortCapacity);
        }
        let offset = Vec2::new(R::from_words(g.offset_x), R::from_words(g.offset_y));
        for shape in g.shapes {
            if !shape.interactive {
                continue;
            }
            let pos = Vec2::new(
                (R::from_words(shape.x) + offset.x) / scale,
                (R::from_words(shape.y) + offset.y) / scale,
            );
            let angle = R::from_words(shape.rotation) * deg;
            let mut d = if shape.class == ShapeClass::Circle {
                ShapeDef::circle()
            } else {
                ShapeDef::polygon()
            };
            d.density = R::from_words(shape.density);
            material(&mut d);
            match shape.class {
                ShapeClass::Rectangle => d.set_as_oriented_box(
                    R::from_words(shape.scale_x) * R::from_i32(50) / scale,
                    R::from_words(shape.scale_y) * R::from_i32(50) / scale,
                    Some(pos),
                    angle,
                ),
                ShapeClass::Circle => {
                    d.local_position = pos;
                    d.radius = R::from_words(shape.scale_x) * R::from_i32(50) / scale;
                }
                ShapeClass::Triangle => {
                    triangle(&mut d, shape);
                    rotate_triangle(&mut d, pos, angle);
                }
            }
            flow!(body::create_shape(ctx, b, &d).0);
        }
        for rec in g.specials {
            let mut unused = NONE;
            flow!(super::specials::create(
                ctx,
                rec,
                b,
                Some(offset),
                &mut unused
            ));
        }
        flow!(body::set_mass_from_shapes(ctx, b));
        build.group_bodies[i] = b;
    }
    Flow::Continue
}

pub fn create_joints<R: Real>(ctx: &mut Ctx<R>, level: &LevelTable, build: &LevelBuild) -> Flow {
    use crate::b2::{joints, state::JointDef};
    let resolve = |r: hw_data::BodyRef| -> H {
        match r {
            hw_data::BodyRef::LevelBody => build.level_body,
            hw_data::BodyRef::Shape(i) => {
                build.shape_bodies.get(i as usize).copied().unwrap_or(NONE)
            }
            hw_data::BodyRef::Special(i) => build
                .special_bodies
                .get(i as usize)
                .copied()
                .unwrap_or(NONE),
            hw_data::BodyRef::Group(i) => {
                build.group_bodies.get(i as usize).copied().unwrap_or(NONE)
            }
        }
    };
    let scale: R = rl!(62.5);
    let pi: R = rl!(3.141592653589793);
    for r in level.joints {
        let b1 = resolve(r.body1);
        let b2 = resolve(r.body2);
        if b1 == NONE || b2 == NONE {
            return Flow::Abort(AbortSite::GameNullDeref);
        }
        let mut d = JointDef::revolute();
        joints::revolute_def_initialize(
            ctx,
            &mut d,
            b1,
            b2,
            Vec2::new(R::from_words(r.x) / scale, R::from_words(r.y) / scale),
        );
        if r.limit {
            d.enable_limit = crate::Flag32::TRUE;
            d.upper_angle = R::from_words(r.upper_angle) * pi / R::from_i32(180);
            d.lower_angle = R::from_words(r.lower_angle) * pi / R::from_i32(180);
        }
        if r.motor {
            d.enable_motor = crate::Flag32::TRUE;
            d.max_motor_torque = R::from_words(r.torque);
            d.motor_speed = R::from_words(r.speed);
        }
        if r.collide_self {
            d.collide_connected = crate::Flag32::TRUE;
        }
        flow!(world::create_joint(ctx, &d).0);
    }
    Flow::Continue
}

pub fn create<R: Real>(ctx: &mut Ctx<R>, level: &LevelTable) -> Flow {
    ctx.level = super::specials::LevelState::blank();
    let mut build = LevelBuild::blank();
    flow!(create_static_shapes(ctx, level, &mut build));
    flow!(create_dynamic_shapes(ctx, level, &mut build));
    flow!(create_specials(ctx, level, &mut build));
    flow!(create_groups(ctx, level, &mut build));
    create_joints(ctx, level, &build)
}
