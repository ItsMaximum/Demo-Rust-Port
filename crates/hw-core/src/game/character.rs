//! Character state. Level-only sessions have no character shapes.
use crate::b2::{
    body, joints,
    math::Vec2,
    state::{BodyDef, FilterData, JointDef, ShapeDef},
    world,
};
use crate::{flow, flow::AbortSite, Ctx, Flow, H, NONE};
use hw_data::CharTable;
use hw_real::Real;

pub const CHEST: usize = 0;
pub const HEAD: usize = 1;
pub const PELVIS: usize = 2;
pub const UPPER_ARM1: usize = 3;
pub const UPPER_ARM2: usize = 4;
pub const UPPER_LEG1: usize = 5;
pub const UPPER_LEG2: usize = 6;
pub const LOWER_ARM1: usize = 7;
pub const LOWER_ARM2: usize = 8;
pub const LOWER_LEG1: usize = 9;
pub const LOWER_LEG2: usize = 10;
pub const NECK: usize = 0;
pub const WAIST: usize = 1;
pub const SHOULDER1: usize = 2;
pub const SHOULDER2: usize = 3;
pub const ELBOW1: usize = 4;
pub const ELBOW2: usize = 5;
pub const HIP1: usize = 6;
pub const HIP2: usize = 7;
pub const KNEE1: usize = 8;
pub const KNEE2: usize = 9;
pub const CHARACTER_HANDLER: u32 = 0x80000000;
pub const CHILD_HANDLER: u32 = 0x40000000;
pub fn handler<R: Real>(ctx: &Ctx<R>, part: u32) -> u32 {
    CHARACTER_HANDLER
        | part
        | if ctx.character.child_active.get() {
            CHILD_HANDLER
        } else {
            0
        }
}
pub fn data<R: Real>(ctx: &Ctx<R>) -> Option<hw_data::RuntimeGuide> {
    hw_data::RuntimeGuide::new(ctx.character.data_index, ctx.character.child_active.get())
}

#[derive(Clone, Copy)]
#[repr(C)]
pub struct CharacterState<R: Real> {
    pub head1_shape: H,
    pub bodies: [H; 11],
    pub shapes: [H; 11],
    pub joints: [H; 10],
    pub start: Vec2<R>,
    pub scale: R,
    pub default_filter: FilterData,
    pub zero_filter: FilterData,
    pub lower_body_filter: FilterData,
    pub intestine_length: R,
    pub ligament_length: R,
    pub chunk_radius: [R; 3],
    /// head, chest, pelvis, foot (Number thresholds, unrounded).
    pub smash_limits: [R; 4],
    /// neck, spine, torso, intestine, shoulder, shoulderSnap, hip, hipSnap,
    /// elbow, elbowLigament, knee, kneeLigament (AS3 int fields).
    pub break_limits: [i32; 12],
    pub impulse_thresholds: [R; 11],
    pub wheelchair: super::wheelchair::WheelchairState<R>,
    pub current_pose: i32,
    pub dead: crate::Flag32,
    pub grabbing: crate::Flag32,
    pub grip_joints: [H; 2],
    pub composites: [super::composites::Composite<R>; super::composites::MAX_COMPOSITES],
    pub composite_count: u32,
    pub spine: H,
    pub intestine: H,
    pub kind: u32,
    pub data_index: u32,
    pub contact_thresholds: [R; super::damage::CONTACT_KEYS],
    pub contact_results: [super::damage::BufferedResult<R>; super::damage::CONTACT_KEYS],
    /// ADD bookkeeping is retained because a queued sound can dereference a destroyed shape.
    pub contact_adds: [crate::Flag32; super::damage::CONTACT_KEYS],
    pub head_blood_flow: crate::Flag32,
    pub dying: crate::Flag32,
    pub bleed_counter: i32,
    pub brain: H,
    pub heart: H,
    /// upperArm3/4 and upperLeg3/4, allocated by partial limb severing.
    pub stumps: [H; 4],
    pub segway: super::segway::SegwayState<R>,
    pub camera_focus: H,
    pub bicycle: super::bicycle::BicycleState<R>,
    pub child_seat: super::child::ChildState,
    pub motorcart: super::motorcart::MotorCartState<R>,
}
impl<R: Real> CharacterState<R> {
    pub fn blank() -> Self {
        Self {
            head1_shape: NONE,
            bodies: [NONE; 11],
            shapes: [NONE; 11],
            joints: [NONE; 10],
            start: Vec2::zero(),
            scale: hw_real::rl!(R; 125.0),
            default_filter: FilterData::new(),
            zero_filter: FilterData::new(),
            lower_body_filter: FilterData::new(),
            intestine_length: R::zero(),
            ligament_length: R::zero(),
            chunk_radius: [R::zero(); 3],
            smash_limits: [R::zero(); 4],
            break_limits: [0; 12],
            impulse_thresholds: [R::from_words([0x7ff80000, 0]); 11],
            wheelchair: super::wheelchair::WheelchairState::blank(),
            current_pose: 0,
            dead: crate::Flag32::FALSE,
            grabbing: crate::Flag32::FALSE,
            grip_joints: [NONE; 2],
            composites: [super::composites::Composite::blank(); super::composites::MAX_COMPOSITES],
            composite_count: 0,
            spine: NONE,
            intestine: NONE,
            kind: 0,
            data_index: 0,
            contact_thresholds: [super::damage::undefined(); super::damage::CONTACT_KEYS],
            contact_results: [super::damage::BufferedResult::blank(); super::damage::CONTACT_KEYS],
            contact_adds: [crate::Flag32::FALSE; super::damage::CONTACT_KEYS],
            head_blood_flow: crate::Flag32::FALSE,
            dying: crate::Flag32::FALSE,
            bleed_counter: 0,
            brain: NONE,
            heart: NONE,
            stumps: [NONE; 4],
            segway: super::segway::SegwayState::blank(),
            camera_focus: NONE,
            bicycle: super::bicycle::BicycleState::blank(),
            child_seat: super::child::ChildState::blank(),
            motorcart: super::motorcart::MotorCartState::blank(),
        }
    }
}

pub fn create_filters<R: Real>(ctx: &mut Ctx<R>, group: i32) {
    let mut default = FilterData::new();
    default.group_index = group;
    default.category_bits = 4;
    default.mask_bits = 12;
    let mut zero = FilterData::new();
    zero.group_index = 0;
    zero.category_bits = 4;
    let mut lower = FilterData::new();
    lower.category_bits = 4;
    lower.mask_bits = 12;
    lower.group_index = group.wrapping_sub(5);
    ctx.character.default_filter = default;
    ctx.character.zero_filter = zero;
    ctx.character.lower_body_filter = lower;
}

/// Shared CharacterB2D bodies, in AS3 creation order. Vehicle subclasses add/replace bodies later.
pub fn create_bodies<R: Real>(ctx: &mut Ctx<R>, data: &CharTable) -> Flow {
    ctx.character.data_index = data.index;
    let names = [
        "chestShape",
        "headShape",
        "pelvisShape",
        "upperArm1Shape",
        "upperArm2Shape",
        "upperLeg1Shape",
        "upperLeg2Shape",
        "lowerArm1Shape",
        "lowerArm2Shape",
        "lowerLeg1Shape",
        "lowerLeg2Shape",
    ];
    let start = ctx.character.start;
    let cs = ctx.character.scale;
    let degrees = hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI);
    let mut polygon = ShapeDef::polygon();
    let mut circle = ShapeDef::circle();
    polygon.density = R::one();
    polygon.friction = hw_real::rl!(R; 0.3);
    polygon.restitution = hw_real::rl!(R; 0.1);
    polygon.filter = ctx.character.default_filter;
    circle.density = R::one();
    circle.friction = hw_real::rl!(R; 0.3);
    circle.restitution = hw_real::rl!(R; 0.1);
    circle.filter = ctx.character.default_filter;
    for (i, name) in names.iter().enumerate() {
        let Some(p) = data.guide_part(name) else {
            return Flow::Abort(AbortSite::PortUnimplemented);
        };
        let mut bd = BodyDef::new();
        bd.position = Vec2::new(
            start.x + R::from_words(p.x) / cs,
            start.y + R::from_words(p.y) / cs,
        );
        bd.angle = R::from_words(p.rotation) / degrees;
        if i == HEAD {
            circle.radius = R::from_words(p.scale_x) * hw_real::rl!(R; 5.0) / cs;
        } else {
            polygon.set_as_box(
                R::from_words(p.scale_x) * hw_real::rl!(R; 5.0) / cs,
                R::from_words(p.scale_y) * hw_real::rl!(R; 5.0) / cs,
            );
        }
        let b = world::create_body(ctx, &bd);
        if b == NONE {
            return Flow::Abort(AbortSite::PortCapacity);
        }
        ctx.character.bodies[i] = b;
        ctx.world.bodies.retain(b);
        if i <= PELVIS {
            body::allow_sleeping(ctx, b, false);
        }
        let (f, sh) = body::create_shape(ctx, b, if i == HEAD { &circle } else { &polygon });
        flow!(f);
        if sh == NONE {
            return Flow::Abort(AbortSite::GameNullDeref);
        }
        ctx.character.shapes[i] = sh;
        ctx.world.shapes.retain(sh);
        if i <= PELVIS {
            ctx.world.shape_mut(sh).material = 1;
            ctx.world.shape_mut(sh).handlers[3] = handler(ctx, i as u32);
            // ADD is sound-only; register a tag for complete listener bookkeeping.
            ctx.world.shape_mut(sh).handlers[0] = handler(ctx, i as u32);
        } else if i == LOWER_LEG1 || i == LOWER_LEG2 {
            ctx.world.shape_mut(sh).handlers[3] = handler(ctx, i as u32);
        }
        if i == HEAD {
            ctx.character.head1_shape = sh;
            ctx.world.shape_mut(sh).user_data = 1;
        }
        flow!(body::set_mass_from_shapes(ctx, b));
        if i == CHEST {
            ctx.character.camera_focus = b;
            if !ctx.character.child_active {
                ctx.level.focus = b;
            }
        }
    }
    let Some(p) = data.guide_part("intestineShape") else {
        return Flow::Abort(AbortSite::PortUnimplemented);
    };
    ctx.character.intestine_length = R::from_words(p.width);
    let Some(p) = data.guide_part("ligamentShape") else {
        return Flow::Abort(AbortSite::PortUnimplemented);
    };
    ctx.character.ligament_length = R::from_words(p.width);
    for (i, name) in ["headChunkShape", "chestChunkShape", "pelvisChunkShape"]
        .iter()
        .enumerate()
    {
        let Some(p) = data.guide_part(name) else {
            return Flow::Abort(AbortSite::PortUnimplemented);
        };
        ctx.character.chunk_radius[i] = R::from_words(p.width) / hw_real::rl!(R; 2.0);
    }
    Flow::Continue
}

pub fn create_joints<R: Real>(ctx: &mut Ctx<R>, data: &CharTable) -> Flow {
    let start = ctx.character.start;
    let cs = ctx.character.scale;
    let mut d = JointDef::revolute();
    d.enable_limit = crate::Flag32::TRUE;
    d.max_motor_torque = hw_real::rl!(R; 4.0);
    for (i, (name, a, b, lower, upper)) in [
        ("headAnchor", CHEST, HEAD, -hw_real::rl!(R; 20.0), hw_real::rl!(R; 20.0)),
        ("pelvisAnchor", CHEST, PELVIS, -hw_real::rl!(R; 5.0), hw_real::rl!(R; 5.0)),
        ("upperArmAnchor", CHEST, UPPER_ARM1, -hw_real::rl!(R; 180.0), hw_real::rl!(R; 60.0)),
        ("upperArmAnchor", CHEST, UPPER_ARM2, -hw_real::rl!(R; 180.0), hw_real::rl!(R; 60.0)),
        ("lowerArm1Anchor", UPPER_ARM1, LOWER_ARM1, -hw_real::rl!(R; 160.0), hw_real::rl!(R; 0.0)),
        ("lowerArm2Anchor", UPPER_ARM2, LOWER_ARM2, -hw_real::rl!(R; 160.0), hw_real::rl!(R; 0.0)),
        ("upperLegAnchor", PELVIS, UPPER_LEG1, -hw_real::rl!(R; 150.0), hw_real::rl!(R; 10.0)),
        ("upperLegAnchor", PELVIS, UPPER_LEG2, -hw_real::rl!(R; 150.0), hw_real::rl!(R; 10.0)),
        ("lowerLeg1Anchor", UPPER_LEG1, LOWER_LEG1, hw_real::rl!(R; 0.0), hw_real::rl!(R; 150.0)),
        ("lowerLeg2Anchor", UPPER_LEG2, LOWER_LEG2, hw_real::rl!(R; 0.0), hw_real::rl!(R; 150.0)),
    ]
    .iter()
    .enumerate()
    {
        let Some(p) = data.guide_part(name) else {
            return Flow::Abort(AbortSite::PortUnimplemented);
        };
        let anchor = Vec2::new(
            start.x + R::from_words(p.x) / cs,
            start.y + R::from_words(p.y) / cs,
        );
        let a = ctx.character.bodies[*a];
        let b = ctx.character.bodies[*b];
        let angle = body::get_angle(ctx, b) - body::get_angle(ctx, a);
        d.lower_angle = *lower / (hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI)) - angle;
        d.upper_angle = *upper / (hw_real::rl!(R; 180.0) / hw_real::rl!(R; core::f64::consts::PI)) - angle;
        joints::revolute_def_initialize(ctx, &mut d, a, b, anchor);
        let (f, h) = world::create_joint(ctx, &d);
        flow!(f);
        ctx.character.joints[i] = h;
        ctx.world.joints.retain(h);
    }
    Flow::Continue
}

pub fn set_limits<R: Real>(ctx: &mut Ctx<R>) {
    let ratios = [
        (HEAD, hw_real::rl!(R; 0.05456)),
        (CHEST, hw_real::rl!(R; 0.17171)),
        (PELVIS, hw_real::rl!(R; 0.0735)),
        (UPPER_ARM1, hw_real::rl!(R; 0.07019)),
        (LOWER_ARM1, hw_real::rl!(R; 0.07784)),
        (UPPER_LEG1, hw_real::rl!(R; 0.14914)),
        (LOWER_LEG1, hw_real::rl!(R; 0.10218)),
    ];
    let mut r = [R::zero(); 7];
    for (i, (part, mass)) in ratios.iter().enumerate() {
        r[i] = body::get_mass(ctx, ctx.character.bodies[*part]) / *mass;
    }
    ctx.character.smash_limits = [
        hw_real::rl!(R; 3.0) * r[0],
        hw_real::rl!(R; 7.5) * r[1],
        hw_real::rl!(R; 5.5) * r[2],
        hw_real::rl!(R; 4.0) * r[6],
    ];
    for (i, (factor, ratio)) in [
        (hw_real::rl!(R; 85.0), 0),
        (hw_real::rl!(R; 105.0), 0),
        (hw_real::rl!(R; 180.0), 2),
        (hw_real::rl!(R; 260.0), 2),
        (hw_real::rl!(R; 75.0), 3),
        (hw_real::rl!(R; 90.0), 3),
        (hw_real::rl!(R; 95.0), 5),
        (hw_real::rl!(R; 110.0), 5),
        (hw_real::rl!(R; 70.0), 4),
        (hw_real::rl!(R; 80.0), 4),
        (hw_real::rl!(R; 80.0), 6),
        (hw_real::rl!(R; 95.0), 6),
    ]
    .iter()
    .enumerate()
    {
        ctx.character.break_limits[i] =
            hw_real::as3::math_round(*factor * r[*ratio]).to_int32();
    }
}

pub fn create_dictionaries<R: Real>(ctx: &mut Ctx<R>) {
    ctx.character.impulse_thresholds = [R::from_words([0x7ff80000, 0]); 11];
    for (part, index) in [
        (HEAD, 0),
        (CHEST, 1),
        (PELVIS, 2),
        (LOWER_LEG1, 3),
        (LOWER_LEG2, 3),
    ] {
        ctx.character.impulse_thresholds[part] = ctx.character.smash_limits[index];
    }
    ctx.character.impulse_thresholds[LOWER_ARM1] = hw_real::rl!(R; 1e-8);
    ctx.character.impulse_thresholds[LOWER_ARM2] = hw_real::rl!(R; 1e-8);
    ctx.character.contact_thresholds = [super::damage::undefined(); super::damage::CONTACT_KEYS];
    ctx.character.contact_results =
        [super::damage::BufferedResult::blank(); super::damage::CONTACT_KEYS];
    for part in [
        HEAD, CHEST, PELVIS, LOWER_LEG1, LOWER_LEG2, LOWER_ARM1, LOWER_ARM2,
    ] {
        let k = super::damage::key(ctx.character.shapes[part]);
        ctx.character.contact_thresholds[k] = ctx.character.impulse_thresholds[part];
    }
}

/// Construction-stage entry point, not yet a complete playable character.
pub fn create_base<R: Real>(ctx: &mut Ctx<R>, data: &CharTable, x: R, y: R) -> Flow {
    *ctx.character = CharacterState::blank();
    ctx.character.start = Vec2::new(x / hw_real::rl!(R; 62.5), y / hw_real::rl!(R; 62.5));
    ctx.character.scale = hw_real::rl!(R; 62.5) * hw_real::rl!(R; 2.0);
    create_filters(ctx, -1);
    flow!(create_bodies(ctx, data));
    flow!(create_joints(ctx, data));
    set_limits(ctx);
    create_dictionaries(ctx);
    Flow::Continue
}

/// CharacterB2D.shapeImpale: chest/pelvis only play vocals; head ejects and dies.
/// A level-only probe's synthetic material-1 shapes are none of these identities.
pub fn shape_impale<R: Real>(ctx: &mut Ctx<R>, shape: H) -> Flow {
    flow!(shape_impale_current(ctx, shape));
    if ctx.character.kind == 3 {
        ctx.character.child_active = crate::Flag32::TRUE;
        let result = shape_impale_current(ctx, shape);
        ctx.character.child_active = crate::Flag32::FALSE;
        flow!(result);
    }
    Flow::Continue
}
fn shape_impale_current<R: Real>(ctx: &mut Ctx<R>, shape: H) -> Flow {
    if shape != NONE
        && (shape == ctx.character.head1_shape
            || (ctx.character.kind == 2 && shape == ctx.character.segway.helmet_shape)
            || (ctx.character.kind == 3 && shape == ctx.character.bicycle.helmet_shape))
    {
        flow!(super::damage::eject(ctx));
        return super::damage::set_dead(ctx, true);
    }
    Flow::Continue
}
