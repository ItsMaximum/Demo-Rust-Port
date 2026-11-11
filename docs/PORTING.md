# Porting conventions (Box2DFlashAS3 2.0.2 to `hw-core`)

The goal is a bit-exact reproduction of the game's Flash behaviour. Read this before porting a class.

## Sources
The port follows the decompiled ActionScript 3 of the game and the Box2DFlashAS3 2.0.2 library it embeds.
That source is not distributed here; behaviour is checked against recorded Flash traces instead
(see `docs/TRACE_FORMAT.md`). Keep the traps listed below in mind when porting a class.

## Numeric rules
- All AS3 `Number` values are `R: hw_real::Real` (never `f64` literals in generic code). Use `R::from_i32(2)`,
  `rl!(0.5)` / `rl!(hi, lo)` for constants, `R::zero()`, `R::one()`.
  Use `hw_real::rl!(R; expression)` when the result type must be explicit, including
  compile-time expressions such as `core::f64::consts::PI`. The literal forms use
  inline const evaluation, so f64/u64 conversion never happens at run time.
  Numeric tables in generic code also hold `R` values made with this macro;
  do not reintroduce runtime `fn n(f64)` or `fn lit(f64)` conversion helpers.
- Keep the AS3 statement order and expression association exactly: `a*b + c*d` stays two products and one add
  (write it in the same order), no `mul_add`, no re-association, no algebraic simplification, no hoisting of
  expressions the AS3 evaluates repeatedly (e.g. `Math.cos(a)` called twice stays two calls).
- Comparisons: use `.lt/.le/.gt/.ge/.eq_num` (false on NaN) exactly where AS3 uses `< <= > >= ==`. `!=` is
  `!a.eq_num(b)` (true on NaN) — mind negated tests.
- `b2Math.b2Abs/b2Min/b2Max/b2Clamp` -> `hw_real::as3::{b2_abs,b2_min,b2_max,b2_clamp}` (ternaries). `Math.abs`
  -> `as3::math_abs`. `Math.sqrt` -> `.sqrt()`. `Math.sin/cos` -> `crate::b2::math::sin_cos(angle)` returns
  (cos, sin); `Math.atan2` -> `crate::b2::math::atan2`.
- `Number.MIN_VALUE` -> `settings::min_value::<R>()` (5e-324); `Number.MAX_VALUE` -> `settings::max_value`.
  Flash 32 `Math.min/max` preserve the first NaN (including its payload) and select
  the second operand on equal values, including mixed signed zeros. Use the
  measured `as3::math_min/math_max` helpers; do not substitute language builtins.
  Write `1 - 100 * Number.MIN_VALUE` literally as `R::one() - R::from_i32(100) * min_value()` etc.
- `uint(x)` on a Number -> `x.to_uint32()`; `int(x)` -> `x.to_int32()`; integer `/` assigned to an `int` -> compute
  in R then `.to_int32()`. Integer arithmetic on int/uint fields stays i32/u32 with wrapping ops.
- Uninitialised AS3 `Number` fields are NaN; `blank()` records already use NaN where AS3 does. Locals declared
  `var x:Number = NaN` are just uninitialised locals.
- AS3 `Array`s that grow (island arrays, DFS stacks, query results) never throw; fixed pools that run out throw a
  TypeError (null dereference). See "errors" below.

## Objects and memory
- Stored booleans use `Flag32` (one u32); use `.get()` for a condition,
  `Flag32::new(bool)` / `TRUE` / `FALSE` for writes and `.word()` for observers.
  Ordinary control-flow arguments and return values remain Rust `bool`.
  State counts are u32, converted to usize only for host/Rust indexing.
- State structs use `#[repr(C)]`; `SpecialKind` uses `#[repr(u32)]`. Do not add pointers, usize fields
  or native bools to the stored state: simulation snapshots are plain word copies.
- `Input` stores the eight TAS key positions in the low eight bits of a u32. `session::step_bits`
  is the frame entry point; the Boolean-array `step` API delegates to it. Transient joint
  reachability flags also use `Flag32`.
- Every object lives in `ctx.world.{bodies,shapes,joints,contacts}` (an `Arena`) and is referenced by `H`
  (`NONE` = null). Read with `ctx.world.body(h)`, mutate with `ctx.world.body_mut(h)`; never hold a reference
  across a call that may mutate the arena (copy the fields you need to locals, as the AS3 locals do).
- `new` of a temporary (b2Vec2, b2Mat22, b2AABB, b2ContactPoint...) becomes a local value; AS3 methods that mutate
  `this` become `&mut` methods (`Vec2::add`, `Mat22::set`...). Shared *static* scratch objects (s_evalCP,
  s_proxyAABB, tImpulse, ...) are locals too, EXCEPT `b2Island.s_reportCR.normal`, which is
  `ctx.world.report_cr_normal` because the game keeps a reference to it.
- Edges: `b2JointEdge`/`b2ContactEdge` are embedded as `node1`/`node2` in Joint/Contact; an edge *handle* is
  `edge_handle(owner, node)`; access via `ctx.world.joint_edge(e)` / `contact_edge(e)`.
- Destroy = `ctx.world.bodies.destroy(h)` etc. (slot contents stay readable until end of frame). Allocate with
  `ctx.world.shapes.alloc(record)`; `NONE` means the *port* capacity is exhausted -> return
  `Flow::Abort(AbortSite::PortCapacity)`.
- `b2NullContact` is the reserved handle `NULL_CONTACT`; pair userData is `NONE` (null), `NULL_CONTACT`, or a contact.
- Retained joint roots must follow the lifetime of the game field. When an exclusive grip field is
  cleared or overwritten, call `joints.release_root(old)` after the source operation succeeds.
  End-of-frame joint graph tracing still preserves destroyed joints reachable through world/body
  lists or other retained joints; releasing a root must not eagerly recycle its slot.

## Function shape
- One Rust module per AS3 class family; free functions named after the AS3 method in snake_case; the first
  parameter is `ctx: &mut Ctx<R>` (or `&Ctx<R>` for pure readers), then the object handle standing in for `this`.
  Example: `pub fn synchronize_shapes<R: Real>(ctx: &mut Ctx<R>, body: H) -> (Flow, bool)`.
- Constructors: `pub fn create<R: Real>(ctx, def: &XDef<R>) -> H` (or `(Flow, H)` when they can throw).
- Callbacks into the game go through `crate::listener::{add,persist,remove,result}(ctx, &point) -> Flow`.

## Errors (AS3 exceptions)
- Functions that can throw (directly or transitively) return `Flow` (or `(Flow, T)`), use `flow!(expr)` to
  propagate, and perform mutations in the exact AS3 statement order so an abort leaves the partial state.
- Throw sites: `AbortSite::ProxyPoolExhausted` (b2BroadPhase.CreateProxy when m_freeProxy == 65535),
  `PairPoolExhausted` (b2PairManager.AddPair when m_freePair == 65535), `PairBufferAdd`/`PairBufferRemove`
  (m_pairBufferCount == 4096), `CommitFindNull` (Find returns null in Commit), `ContactManagerNullBody`,
  `B2AssertPolyCircle`. The world's `lock` stays true after an abort (no finally in AS3).
- Never `panic!`/`unwrap` on a hot path; `debug_assert!` is fine.

## Cross-module API (names other modules may call; implement these exactly)
- `b2::shapes`: `create(ctx, &ShapeDef) -> H`, `destroy(ctx, shape)`, `create_proxy(ctx, shape, xf: &XForm) -> Flow`,
  `destroy_proxy(ctx, shape) -> Flow`, `refilter_proxy(ctx, shape, xf: &XForm) -> Flow`,
  `synchronize(ctx, shape, xf1: &XForm, xf2: &XForm) -> (Flow, bool)`, `reset_proxy(ctx, shape, xf) -> Flow`,
  `compute_mass(ctx, shape) -> MassData<R>`, `compute_aabb(ctx, shape, xf) -> (Vec2, Vec2)` (lower, upper),
  `compute_swept_aabb(ctx, shape, xf1, xf2) -> (Vec2, Vec2)`, `update_sweep_radius(ctx, shape, center: Vec2)`,
  `test_point(ctx, shape, xf, p) -> bool`, `get_max_radius(ctx, shape) -> R`,
  polygon helpers `support(shape: &Shape<R>, xf: &XForm, dx: R, dy: R) -> Vec2`, `get_first_vertex(shape, xf) -> Vec2`.
- `b2::collision`: `collide_circles(m: &mut Manifold, c1: &Shape, xf1, c2: &Shape, xf2)`,
  `collide_polygon_and_circle(m, poly: &Shape, xf1, circle: &Shape, xf2)`, `collide_polygons(m, pa, xfa, pb, xfb)`,
  `test_overlap(al, au, bl, bu) -> bool`, `distance(x1: &mut Vec2, x2: &mut Vec2, s1: &Shape, xf1, s2: &Shape, xf2) -> R`,
  `time_of_impact(s1: &Shape, sweep1: &Sweep, s2: &Shape, sweep2: &Sweep) -> R`.
- `b2::broad_phase`: `init(ctx, lower: Vec2, upper: Vec2)` (constructor work), `in_range(ctx, lower, upper) -> bool`,
  `create_proxy(ctx, lower, upper, shape: H) -> (Flow, u32)`, `destroy_proxy(ctx, proxy_id) -> Flow`,
  `move_proxy(ctx, proxy_id, lower, upper) -> Flow`, `commit(ctx) -> Flow`,
  `query_aabb(ctx, lower, upper, out: &mut [H], max_count: i32) -> (Flow, i32)` (shape handles), `get_proxy(ctx, id) -> Proxy`.
  Pair callbacks it must call: `contacts::pair_added(ctx, shape1, shape2) -> (Flow, H)` and
  `contacts::pair_removed(ctx, shape1, shape2, user_data: H) -> Flow`.
- `b2::contacts`: `create(ctx, shape1, shape2) -> H` (NONE when no register), `destroy(ctx, contact)`,
  `update(ctx, contact) -> Flow`, `pair_added`, `pair_removed` (above), `manager_destroy(ctx, contact) -> Flow`,
  `collide(ctx) -> Flow` (b2ContactManager.Collide), `get_shape1/2`, `is_solid`.
- `b2::body`: `create(ctx, &BodyDef) -> H` (constructor only; list insertion is done by world), plus every b2Body
  method: `create_shape(ctx, body, &ShapeDef) -> (Flow, H)`, `destroy_shape(ctx, body, shape) -> Flow`,
  `set_mass(ctx, body, &MassData) -> Flow`, `set_mass_from_shapes(ctx, body) -> Flow`,
  `set_xform(ctx, body, pos, angle) -> (Flow, bool)`, `synchronize_shapes(ctx, body) -> (Flow, bool)`,
  `synchronize_transform(ctx, body)`, `advance(ctx, body, t)`, `wake_up`, `put_to_sleep`, `apply_force`,
  `apply_torque`, `apply_impulse`, `set_linear_velocity`, `set_angular_velocity`, `get_world_point`,
  `get_world_vector`, `get_local_point`, `get_local_vector`, `get_linear_velocity_from_world_point`,
  `get_linear_velocity_from_local_point`, `is_static/is_frozen/is_sleeping/is_bullet/is_connected`, `set_bullet`,
  `allow_sleeping`, `get_position/get_angle/get_world_center/get_local_center/get_mass/get_inertia`.
- `b2::joints`: `create(ctx, &JointDef) -> H`, `destroy(ctx, joint)`, `init_velocity_constraints(ctx, joint, &TimeStep)`,
  `solve_velocity_constraints(ctx, joint, &TimeStep)`, `init_position_constraints(ctx, joint)`,
  `solve_position_constraints(ctx, joint) -> bool`, `get_anchor1/2(ctx, joint) -> Vec2`, `get_reaction_force`,
  `get_reaction_torque`, revolute/prismatic accessors (`get_joint_angle`, `get_joint_speed`, `get_joint_translation`,
  `enable_motor`, `enable_limit`, `set_motor_speed`, `set_max_motor_torque`, `set_max_motor_force`, `set_limits`,
  `is_motor_enabled`, `is_limit_enabled`, `get_motor_speed`, `get_motor_force`, `get_motor_torque`),
  def helpers `revolute_def_initialize(ctx, &mut JointDef, b1, b2, anchor)`, `prismatic_def_initialize(ctx, def, b1, b2, anchor, axis)`,
  `distance_def_initialize(ctx, def, b1, b2, anchor1, anchor2)`.
- `b2::world`: `new_world(ctx, lower, upper, gravity, do_sleep)`, `create_body(ctx, &BodyDef) -> H` (NONE when
  locked), `destroy_body(ctx, body) -> Flow`, `create_joint(ctx, &JointDef) -> (Flow, H)`, `destroy_joint(ctx, joint) -> Flow`,
  `refilter(ctx, shape) -> Flow`, `step(ctx, dt, iterations) -> Flow`, `query(ctx, lower, upper, out: &mut [H], max) -> (Flow, i32)`.
- `b2::island`: `StepScratch<R>` (island arrays, DFS stack, contact constraints) stored in `ctx.scratch`; `solve`, `solve_toi`, `report`.

## Runtime data and iteration
- Damage-time character properties use `hw_data::RuntimeGuide`, `GuideKey` and
  `GuideLookup::guide_numbers`. `tools/gen-data.py` emits their numeric records from the same measured
  constants as the named setup tables. Missing keys remain `None`; child mode selects the measured kid guide.
- Prefer fixed-array indexing in simulation loops and preserve loop order, early returns and
  overlapping-copy direction when rewriting them; the order is part of the observable behaviour.
- Broadphase/world query outputs accept `&mut (impl QueryOutput + ?Sized)`. Both `[H; N]` and `[H]`
  implement this trait. Capacity failures preserve the same partial output and broadphase state.
