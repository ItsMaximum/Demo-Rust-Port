//! Flat, copyable world state: every AS3 object of the Box2D world lives in a fixed arena slot.
//! Field names follow the AS3 members (m_ prefix dropped, snake_case). Object references are `H`
//! handles (`NONE` = null). All AS3 linked lists keep their exact link fields so that iteration
//! order (head insertion => reverse creation order) is reproduced by construction.
use hw_real::Real;

use super::math::{Mat22, Sweep, Vec2, XForm};
use super::settings::{B2_MAX_PAIRS, B2_MAX_POLYGON_VERTICES, B2_MAX_PROXIES};
use crate::arena::Arena;
use crate::{H, NONE};

// ---- capacities (port limits, not Box2D limits; tightened later from measured peaks) --------
// The proxy-exhaustion probe keeps creating bodies after the 512 proxy slots fill.
pub const MAX_BODIES: usize = 640;
pub const MAX_SHAPES: usize = 640;
pub const MAX_JOINTS: usize = 192;
pub const MAX_CONTACTS: usize = 4096;
// Retired contacts remain readable until end_frame. The official pair-max TAS
// destroys and replaces over 1,100 contacts in one frame while ~3,000 stay live.
pub const MAX_CONTACT_SLOTS: usize = 8192;
// Abort paths do not clear query results; repeated failed queries append duplicates.
pub const MAX_QUERY_RESULTS: usize = 8192;

// ---- b2Body ----------------------------------------------------------------------------------
pub const E_FROZEN_FLAG: u32 = 2;
pub const E_ISLAND_FLAG: u32 = 4;
pub const E_SLEEP_FLAG: u32 = 8;
pub const E_ALLOW_SLEEP_FLAG: u32 = 16;
pub const E_BULLET_FLAG: u32 = 32;
pub const E_FIXED_ROTATION_FLAG: u32 = 64;
pub const E_STATIC_TYPE: i32 = 1;
pub const E_DYNAMIC_TYPE: i32 = 2;
pub const E_MAX_TYPES: i32 = 3;

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Body<R> {
    pub flags: u32,
    pub type_: i32,
    pub xf: XForm<R>,
    pub sweep: Sweep<R>,
    pub linear_velocity: Vec2<R>,
    pub angular_velocity: R,
    pub force: Vec2<R>,
    pub torque: R,
    pub mass: R,
    pub inv_mass: R,
    pub i: R,
    pub inv_i: R,
    pub linear_damping: R,
    pub angular_damping: R,
    pub sleep_time: R,
    pub shape_list: H,
    pub shape_count: i32,
    pub joint_list: H, // head b2JointEdge (encoded: joint handle * 2 + node index)
    pub contact_list: H, // head b2ContactEdge (encoded: contact handle * 2 + node index)
    pub prev: H,
    pub next: H,
    /// Game-side tag standing in for m_userData (0 = null).
    pub user_data: u32,
}

/// b2BodyDef
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct BodyDef<R> {
    pub mass_data: MassData<R>,
    pub user_data: u32,
    pub position: Vec2<R>,
    pub angle: R,
    pub linear_damping: R,
    pub angular_damping: R,
    pub allow_sleep: crate::Flag32,
    pub is_sleeping: crate::Flag32,
    pub fixed_rotation: crate::Flag32,
    pub is_bullet: crate::Flag32,
}

impl<R: Real> BodyDef<R> {
    /// `new b2BodyDef()` (constructor defaults).
    pub fn new() -> Self {
        BodyDef {
            mass_data: MassData::new(),
            user_data: 0,
            position: Vec2::zero(),
            angle: R::zero(),
            linear_damping: R::zero(),
            angular_damping: R::zero(),
            allow_sleep: crate::Flag32::TRUE,
            is_sleeping: crate::Flag32::FALSE,
            fixed_rotation: crate::Flag32::FALSE,
            is_bullet: crate::Flag32::FALSE,
        }
    }
}

/// b2MassData
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct MassData<R> {
    pub mass: R,
    pub center: Vec2<R>,
    pub i: R,
}
impl<R: Real> MassData<R> {
    pub fn new() -> Self {
        MassData {
            mass: R::zero(),
            center: Vec2::zero(),
            i: R::zero(),
        }
    }
}

// ---- b2Shape ---------------------------------------------------------------------------------
pub const E_UNKNOWN_SHAPE: i32 = -1;
pub const E_CIRCLE_SHAPE: i32 = 0;
pub const E_POLYGON_SHAPE: i32 = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct FilterData {
    pub category_bits: u32,
    pub mask_bits: u32,
    pub group_index: i32,
}
impl FilterData {
    /// `new b2FilterData()`
    pub fn new() -> Self {
        FilterData {
            category_bits: 1,
            mask_bits: 65535,
            group_index: 0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Obb<R> {
    pub r: Mat22<R>,
    pub center: Vec2<R>,
    pub extents: Vec2<R>,
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Shape<R> {
    pub type_: i32,
    pub next: H,
    pub body: H,
    pub proxy_id: u32,
    pub sweep_radius: R,
    pub density: R,
    pub friction: R,
    pub restitution: R,
    pub filter: FilterData,
    pub is_sensor: crate::Flag32,
    /// Game addition: b2Shape.m_material (int, default 0).
    pub material: i32,
    /// Game addition: b2Shape.m_id (String; 0 = null, otherwise a game-defined tag).
    pub id_tag: u32,
    pub user_data: u32,
    /// Game contact-listener handler tags [ADD, REMOVE, PERSIST, RESULT] (0 = none).
    pub handlers: [u32; 4],
    // circle payload
    pub local_position: Vec2<R>,
    pub radius: R,
    // polygon payload
    pub vertex_count: i32,
    pub vertices: [Vec2<R>; B2_MAX_POLYGON_VERTICES],
    pub normals: [Vec2<R>; B2_MAX_POLYGON_VERTICES],
    pub core_vertices: [Vec2<R>; B2_MAX_POLYGON_VERTICES],
    pub centroid: Vec2<R>,
    pub obb: Obb<R>,
}

/// b2ShapeDef / b2CircleDef / b2PolygonDef merged (type_ selects).
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct ShapeDef<R> {
    pub type_: i32,
    pub user_data: u32,
    pub friction: R,
    pub restitution: R,
    pub density: R,
    pub is_sensor: crate::Flag32,
    pub filter: FilterData,
    // circle
    pub radius: R,
    pub local_position: Vec2<R>,
    // polygon
    pub vertex_count: i32,
    pub vertices: [Vec2<R>; B2_MAX_POLYGON_VERTICES],
}

impl<R: Real> ShapeDef<R> {
    fn base(type_: i32) -> Self {
        ShapeDef {
            type_,
            user_data: 0,
            friction: R::from_words([0x3FC99999, 0x9999999A]), // 0.2
            restitution: R::zero(),
            density: R::zero(),
            is_sensor: crate::Flag32::FALSE,
            filter: FilterData::new(),
            radius: R::zero(),
            local_position: Vec2::zero(),
            vertex_count: 0,
            vertices: [Vec2::zero(); B2_MAX_POLYGON_VERTICES],
        }
    }
    /// `new b2CircleDef()`: type circle, radius 1.
    pub fn circle() -> Self {
        let mut d = Self::base(E_CIRCLE_SHAPE);
        d.radius = R::one();
        d
    }
    /// `new b2PolygonDef()`: type polygon, vertexCount 0.
    pub fn polygon() -> Self {
        Self::base(E_POLYGON_SHAPE)
    }
}

// ---- b2Joint ---------------------------------------------------------------------------------
pub const E_UNKNOWN_JOINT: i32 = 0;
pub const E_REVOLUTE_JOINT: i32 = 1;
pub const E_PRISMATIC_JOINT: i32 = 2;
pub const E_DISTANCE_JOINT: i32 = 3;
pub const E_INACTIVE_LIMIT: i32 = 0;
pub const E_AT_LOWER_LIMIT: i32 = 1;
pub const E_AT_UPPER_LIMIT: i32 = 2;
pub const E_EQUAL_LIMITS: i32 = 3;

/// b2JointEdge / b2ContactEdge: embedded twice per joint/contact (node1, node2).
/// Edge handles are encoded as `owner * 2 + node` (see `edge_owner`, `edge_node`).
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Edge {
    pub other: H,
    pub prev: H,
    pub next: H,
}
impl Edge {
    pub const fn new() -> Self {
        Edge {
            other: NONE,
            prev: NONE,
            next: NONE,
        }
    }
}
#[inline(always)]
pub fn edge_owner(e: H) -> H {
    e >> 1
}
#[inline(always)]
pub fn edge_node(e: H) -> u32 {
    e & 1
}
#[inline(always)]
pub fn edge_handle(owner: H, node: u32) -> H {
    (owner << 1) | node
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Jacobian<R> {
    pub linear1: Vec2<R>,
    pub angular1: R,
    pub linear2: Vec2<R>,
    pub angular2: R,
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Joint<R> {
    pub type_: i32,
    pub prev: H,
    pub next: H,
    pub node1: Edge,
    pub node2: Edge,
    pub body1: H,
    pub body2: H,
    pub island_flag: crate::Flag32,
    pub collide_connected: crate::Flag32,
    /// Game addition: b2Joint.broken.
    pub broken: crate::Flag32,
    pub user_data: u32,
    pub inv_dt: R,
    // shared by all three types
    pub local_anchor1: Vec2<R>,
    pub local_anchor2: Vec2<R>,
    pub enable_limit: crate::Flag32,
    pub enable_motor: crate::Flag32,
    pub limit_state: i32,
    pub limit_force: R,
    pub limit_position_impulse: R,
    pub motor_force: R,
    pub motor_mass: R,
    pub motor_speed: R,
    // revolute
    pub pivot_force: Vec2<R>,
    pub pivot_mass: Mat22<R>,
    pub reference_angle: R,
    pub lower_angle: R,
    pub upper_angle: R,
    pub max_motor_torque: R,
    // prismatic
    pub local_x_axis1: Vec2<R>,
    pub local_y_axis1: Vec2<R>,
    pub ref_angle: R,
    pub linear_jacobian: Jacobian<R>,
    pub linear_mass: R,
    pub force: R,
    pub angular_mass: R,
    pub torque: R,
    pub motor_jacobian: Jacobian<R>,
    pub lower_translation: R,
    pub upper_translation: R,
    pub max_motor_force: R,
    // distance
    pub u: Vec2<R>,
    pub frequency_hz: R,
    pub damping_ratio: R,
    pub gamma: R,
    pub bias: R,
    pub impulse: R,
    pub mass: R,
    pub length: R,
    // gear (force, mass and linear_jacobian are shared above)
    pub gear_joints:[H;2],
    pub grounds:[H;2],
    pub ground_anchors:[Vec2<R>;2],
    pub ratio:R,
    pub constant:R,
}

/// b2JointDef + the three concrete defs merged.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct JointDef<R> {
    pub type_: i32,
    pub user_data: u32,
    pub body1: H,
    pub body2: H,
    pub collide_connected: crate::Flag32,
    pub local_anchor1: Vec2<R>,
    pub local_anchor2: Vec2<R>,
    // revolute
    pub reference_angle: R,
    pub lower_angle: R,
    pub upper_angle: R,
    pub max_motor_torque: R,
    pub motor_speed: R,
    pub enable_limit: crate::Flag32,
    pub enable_motor: crate::Flag32,
    // prismatic
    pub local_axis1: Vec2<R>,
    pub lower_translation: R,
    pub upper_translation: R,
    pub max_motor_force: R,
    // distance
    pub length: R,
    pub frequency_hz: R,
    pub damping_ratio: R,
    pub gear_joints:[H;2],
    pub ratio:R,
}

impl<R: Real> JointDef<R> {
    fn base(type_: i32) -> Self {
        JointDef {
            type_,
            user_data: 0,
            body1: NONE,
            body2: NONE,
            collide_connected: crate::Flag32::FALSE,
            local_anchor1: Vec2::zero(),
            local_anchor2: Vec2::zero(),
            reference_angle: R::zero(),
            lower_angle: R::zero(),
            upper_angle: R::zero(),
            max_motor_torque: R::zero(),
            motor_speed: R::zero(),
            enable_limit: crate::Flag32::FALSE,
            enable_motor: crate::Flag32::FALSE,
            local_axis1: Vec2::zero(),
            lower_translation: R::zero(),
            upper_translation: R::zero(),
            max_motor_force: R::zero(),
            length: R::zero(),
            frequency_hz: R::zero(),
            damping_ratio: R::zero(),
            gear_joints:[NONE;2],
            ratio:R::one(),
        }
    }
    /// `new b2RevoluteJointDef()`
    pub fn revolute() -> Self {
        Self::base(E_REVOLUTE_JOINT)
    }
    /// `new b2PrismaticJointDef()`: localAxis1 = (1, 0).
    pub fn prismatic() -> Self {
        let mut d = Self::base(E_PRISMATIC_JOINT);
        d.local_axis1 = Vec2::new(R::one(), R::zero());
        d
    }
    /// `new b2DistanceJointDef()`: length 1.
    pub fn distance() -> Self {
        let mut d = Self::base(E_DISTANCE_JOINT);
        d.length = R::one();
        d
    }
    pub fn gear()->Self {Self::base(6)}
}

// ---- contacts --------------------------------------------------------------------------------
pub const E_NON_SOLID_FLAG: u32 = 1;
pub const E_SLOW_FLAG: u32 = 2;
pub const E_CONTACT_ISLAND_FLAG: u32 = 4;
pub const E_TOI_FLAG: u32 = 8;
pub const CONTACT_CIRCLE_CIRCLE: i32 = 0;
pub const CONTACT_POLY_CIRCLE: i32 = 1;
pub const CONTACT_POLY_POLY: i32 = 2;
/// Reserved handle for the world's single b2NullContact (pairs that must not collide).
pub const NULL_CONTACT: H = u32::MAX - 1;

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct ManifoldPoint<R> {
    pub local_point1: Vec2<R>,
    pub local_point2: Vec2<R>,
    pub separation: R,
    pub normal_impulse: R,
    pub tangent_impulse: R,
    /// b2ContactID key: referenceEdge | incidentEdge<<8 | incidentVertex<<16 | flip<<24
    pub id: u32,
}
impl<R: Real> ManifoldPoint<R> {
    pub fn new() -> Self {
        let nan = R::from_words([0x7fffffff, 0xe0000000]); // Flash uninitialized Number payload.
        ManifoldPoint {
            local_point1: Vec2::zero(),
            local_point2: Vec2::zero(),
            separation: nan,
            normal_impulse: nan,
            tangent_impulse: nan,
            id: 0,
        }
    }
    pub fn set(&mut self, o: &ManifoldPoint<R>) {
        self.local_point1 = o.local_point1;
        self.local_point2 = o.local_point2;
        self.separation = o.separation;
        self.normal_impulse = o.normal_impulse;
        self.tangent_impulse = o.tangent_impulse;
        self.id = o.id;
    }
    pub fn reset(&mut self) {
        self.local_point1.set_zero();
        self.local_point2.set_zero();
        self.separation = R::zero();
        self.normal_impulse = R::zero();
        self.tangent_impulse = R::zero();
        self.id = 0;
    }
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Manifold<R> {
    pub point_count: i32,
    pub normal: Vec2<R>,
    pub points: [ManifoldPoint<R>; 2],
}
impl<R: Real> Manifold<R> {
    pub fn new() -> Self {
        Manifold {
            point_count: 0,
            normal: Vec2::zero(),
            points: [ManifoldPoint::new(); 2],
        }
    }
    pub fn set(&mut self, o: &Manifold<R>) {
        self.point_count = o.point_count;
        self.points[0].set(&o.points[0]);
        self.points[1].set(&o.points[1]);
        self.normal.set_v(o.normal);
    }
    pub fn reset(&mut self) {
        self.points[0].reset();
        self.points[1].reset();
        self.normal.set_zero();
        self.point_count = 0;
    }
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Contact<R> {
    pub kind: i32,
    pub flags: u32,
    pub prev: H,
    pub next: H,
    pub node1: Edge,
    pub node2: Edge,
    pub shape1: H,
    pub shape2: H,
    pub manifold_count: i32,
    pub friction: R,
    pub restitution: R,
    pub toi: R,
    pub manifold: Manifold<R>,
}

// ---- broadphase ------------------------------------------------------------------------------
pub const B2_INVALID: u32 = 65535;
pub const B2_NULL_EDGE: u32 = 65535;
pub const B2_NULL_PROXY: u32 = 65535;
pub const B2_NULL_PAIR: u32 = 65535;
pub const B2_TABLE_CAPACITY: usize = B2_MAX_PAIRS;
pub const B2_TABLE_MASK: u32 = (B2_TABLE_CAPACITY - 1) as u32;
pub const E_PAIR_BUFFERED: u32 = 1;
pub const E_PAIR_REMOVED: u32 = 2;
pub const E_PAIR_FINAL: u32 = 4;

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Bound {
    pub value: u32,
    pub proxy_id: u32,
    pub stabbing_count: u32,
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Proxy {
    /// lowerBounds[0] doubles as the free-list `next` (SetNext stores `n & 0xFFFF`).
    pub lower_bounds: [u32; 2],
    pub upper_bounds: [u32; 2],
    pub overlap_count: u32,
    pub time_stamp: u32,
    /// userData: the owning shape handle (NONE = null).
    pub shape: H,
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Pair {
    /// userData: contact handle, NULL_CONTACT, or NONE (null).
    pub contact: H,
    pub proxy_id1: u32,
    pub proxy_id2: u32,
    pub status: u32,
    pub next: u32,
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct BufferedPair {
    pub proxy_id1: u32,
    pub proxy_id2: u32,
}

#[derive(Clone, Copy)]
#[repr(C)]
pub struct BroadPhase<R> {
    pub bounds: [[Bound; 2 * B2_MAX_PROXIES]; 2],
    pub quantization_factor: Vec2<R>,
    pub world_aabb_lower: Vec2<R>,
    pub world_aabb_upper: Vec2<R>,
    pub free_proxy: u32,
    pub proxy_count: i32,
    /// m_proxyPool: only 0..B2_MAX_PROXIES are real objects (the AS3 array is 4096 long).
    pub proxy_pool: [Proxy; B2_MAX_PROXIES],
    pub query_result_count: i32,
    pub query_results: [u32; MAX_QUERY_RESULTS],
    pub time_stamp: u32,
    // b2PairManager
    pub pair_count: i32,
    pub pair_buffer: [BufferedPair; B2_MAX_PAIRS],
    pub hash_table: [u32; B2_TABLE_CAPACITY],
    pub pairs: [Pair; B2_MAX_PAIRS],
    pub pair_buffer_count: i32,
    pub free_pair: u32,
}

// ---- world -----------------------------------------------------------------------------------
#[derive(Clone, Copy)]
#[repr(C)]
pub struct WorldState<R: Real> {
    pub bodies: Arena<Body<R>, MAX_BODIES>,
    pub shapes: Arena<Shape<R>, MAX_SHAPES>,
    pub joints: Arena<Joint<R>, MAX_JOINTS>,
    pub contacts: Arena<Contact<R>, MAX_CONTACT_SLOTS>,
    pub broad_phase: BroadPhase<R>,
    // b2World fields
    pub lock: crate::Flag32,
    pub inv_dt0: R,
    pub contact_list: H,
    pub allow_sleep: crate::Flag32,
    pub joint_count: i32,
    pub body_count: i32,
    pub position_iteration_count: i32,
    pub ground_body: H,
    pub contact_count: i32,
    pub body_list: H,
    pub joint_list: H,
    pub gravity: Vec2<R>,
    /// Static flags of b2World (reset to true by every constructor).
    pub continuous_physics: crate::Flag32,
    pub warm_starting: crate::Flag32,
    pub position_correction: crate::Flag32,
    /// b2ContactManager.m_destroyImmediate
    pub destroy_immediate: crate::Flag32,
    /// The persistent, shared `b2Island.s_reportCR.normal` (aliased by game ContactEvents).
    pub report_cr_normal: Vec2<R>,
    /// b2Contact.s_initialized (registers) — always true after the first Step in AS3; kept for parity.
    pub contact_registers_initialized: crate::Flag32,
}

impl<R: Real> Body<R> {
    pub fn blank() -> Self {
        let nan = R::from_words([0x7fffffff, 0xe0000000]); // Flash uninitialized Number payload.
        Body {
            flags: 0,
            type_: 0,
            xf: XForm::identity(),
            sweep: Sweep::new_default(),
            linear_velocity: Vec2::zero(),
            angular_velocity: nan,
            force: Vec2::zero(),
            torque: nan,
            mass: nan,
            inv_mass: nan,
            i: nan,
            inv_i: nan,
            linear_damping: nan,
            angular_damping: nan,
            sleep_time: nan,
            shape_list: NONE,
            shape_count: 0,
            joint_list: NONE,
            contact_list: NONE,
            prev: NONE,
            next: NONE,
            user_data: 0,
        }
    }
}

impl<R: Real> Shape<R> {
    pub fn blank() -> Self {
        let nan = R::from_words([0x7fffffff, 0xe0000000]); // Flash uninitialized Number payload.
        Shape {
            type_: E_UNKNOWN_SHAPE,
            next: NONE,
            body: NONE,
            proxy_id: B2_NULL_PROXY,
            sweep_radius: nan,
            density: nan,
            friction: nan,
            restitution: nan,
            filter: FilterData::new(),
            is_sensor: crate::Flag32::FALSE,
            material: 0,
            id_tag: 0,
            user_data: 0,
            handlers: [0; 4],
            local_position: Vec2::zero(),
            radius: nan,
            vertex_count: 0,
            vertices: [Vec2::zero(); B2_MAX_POLYGON_VERTICES],
            normals: [Vec2::zero(); B2_MAX_POLYGON_VERTICES],
            core_vertices: [Vec2::zero(); B2_MAX_POLYGON_VERTICES],
            centroid: Vec2::zero(),
            obb: Obb {
                r: Mat22::identity(),
                center: Vec2::zero(),
                extents: Vec2::zero(),
            },
        }
    }
}

impl<R: Real> Joint<R> {
    pub fn blank() -> Self {
        let nan = R::from_words([0x7fffffff, 0xe0000000]); // Flash uninitialized Number payload.
        let jac = Jacobian {
            linear1: Vec2::zero(),
            angular1: nan,
            linear2: Vec2::zero(),
            angular2: nan,
        };
        Joint {
            type_: E_UNKNOWN_JOINT,
            prev: NONE,
            next: NONE,
            node1: Edge::new(),
            node2: Edge::new(),
            body1: NONE,
            body2: NONE,
            island_flag: crate::Flag32::FALSE,
            collide_connected: crate::Flag32::FALSE,
            broken: crate::Flag32::FALSE,
            user_data: 0,
            inv_dt: nan,
            local_anchor1: Vec2::zero(),
            local_anchor2: Vec2::zero(),
            enable_limit: crate::Flag32::FALSE,
            enable_motor: crate::Flag32::FALSE,
            limit_state: 0,
            limit_force: nan,
            limit_position_impulse: nan,
            motor_force: nan,
            motor_mass: nan,
            motor_speed: nan,
            pivot_force: Vec2::zero(),
            pivot_mass: Mat22::from_angle(R::zero()),
            reference_angle: nan,
            lower_angle: nan,
            upper_angle: nan,
            max_motor_torque: nan,
            local_x_axis1: Vec2::zero(),
            local_y_axis1: Vec2::zero(),
            ref_angle: nan,
            linear_jacobian: jac,
            linear_mass: nan,
            force: nan,
            angular_mass: nan,
            torque: nan,
            motor_jacobian: jac,
            lower_translation: nan,
            upper_translation: nan,
            max_motor_force: nan,
            u: Vec2::zero(),
            frequency_hz: nan,
            damping_ratio: nan,
            gamma: nan,
            bias: nan,
            impulse: nan,
            mass: nan,
            length: nan,
            gear_joints:[NONE;2],
            grounds:[NONE;2],
            ground_anchors:[Vec2::zero();2],
            ratio:nan,
            constant:nan,
        }
    }
}

impl<R: Real> Contact<R> {
    pub fn blank() -> Self {
        let nan = R::from_words([0x7fffffff, 0xe0000000]); // Flash uninitialized Number payload.
        Contact {
            kind: 0,
            flags: 0,
            prev: NONE,
            next: NONE,
            node1: Edge::new(),
            node2: Edge::new(),
            shape1: NONE,
            shape2: NONE,
            manifold_count: 0,
            friction: nan,
            restitution: nan,
            toi: nan,
            manifold: Manifold::new(),
        }
    }
}

impl<R: Real> BroadPhase<R> {
    /// `new b2BroadPhase(worldAABB)` minus the parts that depend on the AABB (set by broad_phase::new).
    pub fn blank() -> Self {
        BroadPhase {
            bounds: [[Bound {
                value: 0,
                proxy_id: 0,
                stabbing_count: 0,
            }; 2 * B2_MAX_PROXIES]; 2],
            quantization_factor: Vec2::zero(),
            world_aabb_lower: Vec2::zero(),
            world_aabb_upper: Vec2::zero(),
            free_proxy: 0,
            proxy_count: 0,
            proxy_pool: [Proxy {
                lower_bounds: [0, 0],
                upper_bounds: [0, 0],
                overlap_count: 0,
                time_stamp: 0,
                shape: NONE,
            }; B2_MAX_PROXIES],
            query_result_count: 0,
            query_results: [0; MAX_QUERY_RESULTS],
            time_stamp: 0,
            pair_count: 0,
            pair_buffer: [BufferedPair {
                proxy_id1: 0,
                proxy_id2: 0,
            }; B2_MAX_PAIRS],
            hash_table: [0; B2_TABLE_CAPACITY],
            pairs: [Pair {
                contact: NONE,
                proxy_id1: 0,
                proxy_id2: 0,
                status: 0,
                next: 0,
            }; B2_MAX_PAIRS],
            pair_buffer_count: 0,
            free_pair: 0,
        }
    }
}

impl<R: Real> WorldState<R> {
    /// Empty state; `world::new_world` performs the `new b2World(aabb, gravity, doSleep)` work.
    pub fn blank() -> Self {
        WorldState {
            bodies: Arena::new(Body::blank()),
            shapes: Arena::new(Shape::blank()),
            joints: Arena::new(Joint::blank()),
            contacts: Arena::new(Contact::blank()),
            broad_phase: BroadPhase::blank(),
            lock: crate::Flag32::FALSE,
            inv_dt0: R::zero(),
            contact_list: NONE,
            allow_sleep: crate::Flag32::FALSE,
            joint_count: 0,
            body_count: 0,
            position_iteration_count: 0,
            ground_body: NONE,
            contact_count: 0,
            body_list: NONE,
            joint_list: NONE,
            gravity: Vec2::zero(),
            continuous_physics: crate::Flag32::TRUE,
            warm_starting: crate::Flag32::TRUE,
            position_correction: crate::Flag32::TRUE,
            destroy_immediate: crate::Flag32::FALSE,
            report_cr_normal: Vec2::zero(),
            contact_registers_initialized: crate::Flag32::FALSE,
        }
    }
    #[inline(always)]
    pub fn body(&self, h: H) -> &Body<R> {
        self.bodies.get(h)
    }
    #[inline(always)]
    pub fn body_mut(&mut self, h: H) -> &mut Body<R> {
        self.bodies.get_mut(h)
    }
    #[inline(always)]
    pub fn shape(&self, h: H) -> &Shape<R> {
        self.shapes.get(h)
    }
    #[inline(always)]
    pub fn shape_mut(&mut self, h: H) -> &mut Shape<R> {
        self.shapes.get_mut(h)
    }
    #[inline(always)]
    pub fn joint(&self, h: H) -> &Joint<R> {
        self.joints.get(h)
    }
    #[inline(always)]
    pub fn joint_mut(&mut self, h: H) -> &mut Joint<R> {
        self.joints.get_mut(h)
    }
    #[inline(always)]
    pub fn contact(&self, h: H) -> &Contact<R> {
        self.contacts.get(h)
    }
    #[inline(always)]
    pub fn contact_mut(&mut self, h: H) -> &mut Contact<R> {
        self.contacts.get_mut(h)
    }
    /// Access a joint edge by encoded edge handle.
    #[inline(always)]
    pub fn joint_edge(&self, e: H) -> &Edge {
        let j = self.joints.get(edge_owner(e));
        if edge_node(e) == 0 {
            &j.node1
        } else {
            &j.node2
        }
    }
    #[inline(always)]
    pub fn joint_edge_mut(&mut self, e: H) -> &mut Edge {
        let j = self.joints.get_mut(edge_owner(e));
        if edge_node(e) == 0 {
            &mut j.node1
        } else {
            &mut j.node2
        }
    }
    #[inline(always)]
    pub fn contact_edge(&self, e: H) -> &Edge {
        let c = self.contacts.get(edge_owner(e));
        if edge_node(e) == 0 {
            &c.node1
        } else {
            &c.node2
        }
    }
    #[inline(always)]
    pub fn contact_edge_mut(&mut self, e: H) -> &mut Edge {
        let c = self.contacts.get_mut(edge_owner(e));
        if edge_node(e) == 0 {
            &mut c.node1
        } else {
            &mut c.node2
        }
    }
    /// End-of-frame slot recycling (see `Arena`).
    pub fn recycle_pending(&mut self) {
        self.recycle_joint_references();
        self.bodies.recycle_pending();
        self.shapes.recycle_pending();
        self.contacts.recycle_pending();
    }
    fn recycle_joint_references(&mut self) {
        if self.joints.pending_len == 0 && self.joints.retired_len == 0 { return; }
        // DestroyJoint leaves m_prev/m_next intact. Repeated source destruction
        // can write those old references back into a live list. Trace the graph
        // before recycling, including retained game joints and body edge roots.
        fn mark(h:H, seen:&mut [crate::Flag32;MAX_JOINTS], stack:&mut [H;MAX_JOINTS], count:&mut usize) {
            if h != NONE && !seen[h as usize] {
                seen[h as usize] = crate::Flag32::TRUE;
                stack[*count] = h;
                *count += 1;
            }
        }
        let mut seen = [crate::Flag32::FALSE;MAX_JOINTS];
        let mut stack = [NONE;MAX_JOINTS];
        let mut count = 0;
        mark(self.joint_list,&mut seen,&mut stack,&mut count);
        for i in 0..MAX_JOINTS {
            if self.joints.alive[i] != 0 || self.joints.retained[i] & 1 != 0 {
                mark(i as H,&mut seen,&mut stack,&mut count);
            }
        }
        for i in 0..MAX_BODIES {
            if self.bodies.alive[i] != 0 || self.bodies.retained[i] != 0 {
                let edge = self.bodies.items[i].joint_list;
                if edge != NONE { mark(edge_owner(edge),&mut seen,&mut stack,&mut count); }
            }
        }
        while count != 0 {
            count -= 1;
            let j = &self.joints.items[stack[count] as usize];
            let handles = [j.prev,j.next,j.gear_joints[0],j.gear_joints[1]];
            for i in 0..4 {
                let h = handles[i];
                mark(h,&mut seen,&mut stack,&mut count);
            }
            let edges = [j.node1.prev,j.node1.next,j.node2.prev,j.node2.next];
            for i in 0..4 {
                let edge = edges[i];
                if edge != NONE { mark(edge_owner(edge),&mut seen,&mut stack,&mut count); }
            }
        }
        self.joints.recycle_pending_reachable(&seen);
    }
}

// ---- step / listener records -----------------------------------------------------------------

/// b2TimeStep
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct TimeStep<R> {
    pub dt: R,
    pub inv_dt: R,
    pub dt_ratio: R,
    pub max_iterations: i32,
    pub warm_starting: crate::Flag32,
    pub position_correction: crate::Flag32,
}

/// b2ContactPoint as passed to b2ContactListener.Add/Persist/Remove.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct ContactPoint<R> {
    pub shape1: H,
    pub shape2: H,
    pub position: Vec2<R>,
    pub velocity: Vec2<R>,
    pub normal: Vec2<R>,
    pub separation: R,
    pub friction: R,
    pub restitution: R,
    pub id: u32,
}

/// b2ContactResult as passed to b2ContactListener.Result. The AS3 object is the shared static
/// `b2Island.s_reportCR`; its `normal` lives in `WorldState::report_cr_normal` because game code
/// captures that vector by reference.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct ContactResult<R> {
    pub shape1: H,
    pub shape2: H,
    pub position: Vec2<R>,
    pub normal_impulse: R,
    pub tangent_impulse: R,
    pub id: u32,
}
