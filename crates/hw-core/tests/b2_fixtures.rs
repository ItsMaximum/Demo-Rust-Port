//! Box2D-only differential fixtures: mirrors tools/probes/Box2dFixtureProbe.as statement by
//! statement and compares every frame with the Flash 32 x86 traces under reference/traces/flash32.
use hw_core::b2::math::Vec2;
use hw_core::b2::state::*;
use hw_core::b2::{body, joints, world};
use hw_core::{Ctx, Flow, H, NONE};
use hw_real::Real;

mod support;
use support::TestReal as R;
fn lit(value: f64) -> R {
    R::from_words(hw_real::words_of_f64(value))
}

fn large_stack(f: fn()) {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(f)
        .unwrap()
        .join()
        .unwrap();
}

struct Fx {
    ctx: Ctx<R>,
    seed: u32,
    bodies: Vec<H>,
    joints: Vec<H>,
}

impl Fx {
    fn new() -> Fx {
        let mut ctx = Ctx::new();
        world::new_world(
            &mut ctx,
            Vec2::new(lit(-4.0), lit(-84.0)),
            Vec2::new(lit(324.0), lit(164.0)),
            Vec2::new(lit(0.0), lit(10.0)),
            true,
        );
        Fx {
            ctx,
            seed: 12345,
            bodies: Vec::new(),
            joints: Vec::new(),
        }
    }
    fn rnd(&mut self) -> R {
        self.seed = self.seed.wrapping_mul(1664525).wrapping_add(1013904223);
        R::from_u32(self.seed) / lit(4294967296.0)
    }
    fn ground(&mut self) -> H {
        let bd = BodyDef::new();
        let g = world::create_body(&mut self.ctx, &bd);
        let mut pd = ShapeDef::polygon();
        pd.friction = R::from_i32(1);
        pd.restitution = lit(0.1);
        hw_core::b2::shapes::set_as_oriented_box(
            &mut pd,
            R::from_i32(100),
            R::from_i32(1),
            Vec2::new(lit(50.0), lit(50.0)),
            R::zero(),
        );
        assert_eq!(body::create_shape(&mut self.ctx, g, &pd).0, Flow::Continue);
        hw_core::b2::shapes::set_as_oriented_box(
            &mut pd,
            R::from_i32(1),
            R::from_i32(50),
            Vec2::new(lit(0.0), lit(25.0)),
            R::zero(),
        );
        assert_eq!(body::create_shape(&mut self.ctx, g, &pd).0, Flow::Continue);
        hw_core::b2::shapes::set_as_oriented_box(
            &mut pd,
            R::from_i32(1),
            R::from_i32(50),
            Vec2::new(lit(100.0), lit(25.0)),
            R::zero(),
        );
        assert_eq!(body::create_shape(&mut self.ctx, g, &pd).0, Flow::Continue);
        self.bodies.push(g);
        g
    }
    #[allow(clippy::too_many_arguments)]
    fn boxb(
        &mut self,
        x: R,
        y: R,
        hx: R,
        hy: R,
        density: R,
        friction: R,
        restitution: R,
        angle: R,
        sleeping: bool,
        bullet: bool,
    ) -> H {
        let mut bd = BodyDef::new();
        bd.position.set(x, y);
        bd.angle = angle;
        bd.is_sleeping = hw_core::Flag32::new(sleeping);
        bd.is_bullet = hw_core::Flag32::new(bullet);
        let b = world::create_body(&mut self.ctx, &bd);
        let mut pd = ShapeDef::polygon();
        hw_core::b2::shapes::set_as_box(&mut pd, hx, hy);
        pd.density = density;
        pd.friction = friction;
        pd.restitution = restitution;
        assert_eq!(body::create_shape(&mut self.ctx, b, &pd).0, Flow::Continue);
        assert_eq!(body::set_mass_from_shapes(&mut self.ctx, b), Flow::Continue);
        self.bodies.push(b);
        b
    }
    #[allow(clippy::too_many_arguments)]
    fn circle(
        &mut self,
        x: R,
        y: R,
        r: R,
        density: R,
        friction: R,
        restitution: R,
        sleeping: bool,
        bullet: bool,
    ) -> H {
        let mut bd = BodyDef::new();
        bd.position.set(x, y);
        bd.is_sleeping = hw_core::Flag32::new(sleeping);
        bd.is_bullet = hw_core::Flag32::new(bullet);
        let b = world::create_body(&mut self.ctx, &bd);
        let mut cd = ShapeDef::circle();
        cd.radius = r;
        cd.density = density;
        cd.friction = friction;
        cd.restitution = restitution;
        assert_eq!(body::create_shape(&mut self.ctx, b, &cd).0, Flow::Continue);
        assert_eq!(body::set_mass_from_shapes(&mut self.ctx, b), Flow::Continue);
        self.bodies.push(b);
        b
    }
    #[allow(clippy::too_many_arguments)]
    fn triangle(
        &mut self,
        x: R,
        y: R,
        s: R,
        density: R,
        friction: R,
        restitution: R,
        angle: R,
        sleeping: bool,
    ) -> H {
        let mut bd = BodyDef::new();
        bd.position.set(x, y);
        bd.angle = angle;
        bd.is_sleeping = hw_core::Flag32::new(sleeping);
        let b = world::create_body(&mut self.ctx, &bd);
        let mut pd = ShapeDef::polygon();
        pd.vertex_count = 3;
        pd.vertices[0] = Vec2::new(R::zero(), s * R::from_i32(-2));
        pd.vertices[1] = Vec2::new(s, s);
        pd.vertices[2] = Vec2::new(-s, s);
        pd.density = density;
        pd.friction = friction;
        pd.restitution = restitution;
        assert_eq!(body::create_shape(&mut self.ctx, b, &pd).0, Flow::Continue);
        assert_eq!(body::set_mass_from_shapes(&mut self.ctx, b), Flow::Continue);
        self.bodies.push(b);
        b
    }
    fn step(&mut self) -> i32 {
        match world::step(&mut self.ctx, lit(0.03333333333333333), 10) {
            Flow::Continue => 0,
            Flow::Abort(hw_core::flow::AbortSite::PortCapacity) => {
                panic!("port capacity exhausted")
            }
            Flow::Abort(_) => 1009,
        }
    }
    /// One frame record in the probe's JSON layout.
    fn record(&self, frame: i32, err: i32, with_joints: bool) -> serde_json::Value {
        let w = &self.ctx.world;
        let bits = |v: R| {
            let [hi, lo] = v.to_words();
            format!("{hi:08x}{lo:08x}")
        };
        let mut bodies = Vec::new();
        let mut b = w.body_list;
        while b != NONE {
            let bd = w.body(b);
            bodies.push(serde_json::json!([
                bits(bd.sweep.c.x),
                bits(bd.sweep.c.y),
                bits(bd.sweep.a),
                bits(bd.sweep.c0.x),
                bits(bd.sweep.c0.y),
                bits(bd.sweep.a0),
                bits(bd.linear_velocity.x),
                bits(bd.linear_velocity.y),
                bits(bd.angular_velocity),
                bd.flags,
                bits(bd.sleep_time),
                bd.shape_count
            ]));
            b = bd.next;
        }
        let mut rec = serde_json::json!({
            "frame": frame,
            "err": err,
            "lock": if w.lock.get() { 1 } else { 0 },
            "counts": {
                "bodies": w.body_count,
                "contacts": w.contact_count,
                "joints": w.joint_count,
                "proxies": w.broad_phase.proxy_count,
                "pairs": w.broad_phase.pair_count,
                "pairBuffer": w.broad_phase.pair_buffer_count,
            },
            "bodies": bodies,
        });
        if with_joints {
            let mut js = Vec::new();
            let mut j = w.joint_list;
            while j != NONE {
                let jt = w.joint(j);
                let v = match jt.type_ {
                    E_REVOLUTE_JOINT => serde_json::json!([
                        1,
                        bits(jt.pivot_force.x),
                        bits(jt.pivot_force.y),
                        bits(jt.motor_force),
                        bits(jt.limit_force),
                        jt.limit_state
                    ]),
                    E_PRISMATIC_JOINT => serde_json::json!([
                        2,
                        bits(jt.force),
                        bits(jt.torque),
                        bits(jt.motor_force),
                        bits(jt.limit_force),
                        jt.limit_state
                    ]),
                    E_DISTANCE_JOINT => {
                        serde_json::json!([3, bits(jt.impulse), bits(jt.u.x), bits(jt.u.y)])
                    }
                    _ => serde_json::json!([0]),
                };
                js.push(v);
                j = jt.next;
            }
            rec["joints"] = serde_json::Value::Array(js);
        }
        rec
    }
}

fn load_trace(name: &str) -> Vec<serde_json::Value> {
    let path = format!(
        "{}/../../reference/traces/flash32/b2-{}-v1.jsonl",
        env!("CARGO_MANIFEST_DIR"),
        name
    );
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    text.lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| serde_json::from_str(l).unwrap())
        .collect()
}

/// Compare one frame; returns a description of the first difference.
fn diff_frame(expected: &serde_json::Value, actual: &serde_json::Value) -> Option<String> {
    for key in ["err", "lock", "counts"] {
        if expected[key] != actual[key] {
            return Some(format!(
                "{key}: flash {} vs port {}",
                expected[key], actual[key]
            ));
        }
    }
    for list in ["bodies", "joints"] {
        let e = expected[list].as_array();
        let a = actual[list].as_array();
        match (e, a) {
            (None, None) => {}
            (Some(e), Some(a)) => {
                if e.len() != a.len() {
                    return Some(format!(
                        "{list} count: flash {} vs port {}",
                        e.len(),
                        a.len()
                    ));
                }
                for (i, (ee, aa)) in e.iter().zip(a.iter()).enumerate() {
                    let ev = ee.as_array().unwrap();
                    let av = aa.as_array().unwrap();
                    for (k, (x, y)) in ev.iter().zip(av.iter()).enumerate() {
                        if x != y {
                            return Some(format!("{list}[{i}] field {k}: flash {x} vs port {y}"));
                        }
                    }
                }
            }
            _ => return Some(format!("{list}: presence mismatch")),
        }
    }
    None
}

fn run(
    name: &str,
    fx: &mut Fx,
    frames: i32,
    with_joints: bool,
    mut before: impl FnMut(&mut Fx, i32),
    trace_offset: usize,
) {
    let trace = load_trace(name);
    for frame in 1..=frames {
        before(fx, frame);
        let err = fx.step();
        let actual = fx.record(frame, err, with_joints);
        let expected = &trace[trace_offset + (frame as usize) - 1];
        assert_eq!(expected["frame"].as_i64().unwrap(), frame as i64);
        if let Some(d) = diff_frame(expected, &actual) {
            panic!("fixture {name} diverges at frame {frame}: {d}");
        }
        fx.ctx.end_frame();
    }
}

#[test]
fn fixture_drop() {
    large_stack(run_drop);
}
fn run_drop() {
    let mut fx = Fx::new();
    fx.ground();
    for i in 0..40 {
        let x = R::from_i32(5) + fx.rnd() * R::from_i32(90);
        let y = R::from_i32(5) + fx.rnd() * R::from_i32(30);
        let angle = fx.rnd() * R::from_i32(6);
        let density = lit(0.5) + fx.rnd() * R::from_i32(5);
        let friction = fx.rnd();
        let restitution = fx.rnd() * lit(0.5);
        let size = lit(0.5) + fx.rnd() * lit(1.5);
        let sleeping = (i % 5) == 4;
        let bullet = (i % 7) == 6;
        if i % 3 == 0 {
            fx.boxb(
                x,
                y,
                size,
                size * lit(0.6),
                density,
                friction,
                restitution,
                angle,
                sleeping,
                bullet,
            );
        } else if i % 3 == 1 {
            fx.circle(x, y, size, density, friction, restitution, sleeping, bullet);
        } else {
            fx.triangle(x, y, size, density, friction, restitution, angle, sleeping);
        }
    }
    run(
        "drop",
        &mut fx,
        300,
        false,
        |fx, frame| {
            if frame == 100 {
                let b = fx.bodies[4];
                let c = body::get_world_center(&fx.ctx, b);
                body::apply_impulse(&mut fx.ctx, b, Vec2::new(lit(30.0), lit(-80.0)), c);
            }
            if frame == 150 {
                let b = fx.bodies[6];
                body::set_linear_velocity(&mut fx.ctx, b, Vec2::new(lit(-20.0), lit(-30.0)));
                body::set_angular_velocity(&mut fx.ctx, b, R::from_i32(15));
            }
            if frame == 200 {
                let b = fx.bodies[8];
                let _ = world::destroy_body(&mut fx.ctx, b);
            }
            if frame == 220 {
                let b = fx.bodies[10];
                let mut fd = FilterData::new();
                fd.group_index = -1;
                fd.category_bits = 4;
                fd.mask_bits = 12;
                let s = fx.ctx.world.body(b).shape_list;
                hw_core::b2::shapes::set_filter_data(&mut fx.ctx, s, fd);
                let _ = world::refilter(&mut fx.ctx, s);
            }
            if frame == 250 {
                let b = fx.bodies[12];
                let s = fx.ctx.world.body(b).shape_list;
                let _ = body::destroy_shape(&mut fx.ctx, b, s);
                let mut pd = ShapeDef::polygon();
                hw_core::b2::shapes::set_as_oriented_box(
                    &mut pd,
                    lit(1.5),
                    lit(0.5),
                    Vec2::new(lit(0.5), lit(0.0)),
                    lit(0.3),
                );
                pd.density = R::from_i32(2);
                pd.friction = lit(0.5);
                let _ = body::create_shape(&mut fx.ctx, b, &pd);
                let _ = body::set_mass_from_shapes(&mut fx.ctx, b);
            }
            if frame == 260 {
                let b = fx.bodies[14];
                let _ = body::set_xform(&mut fx.ctx, b, Vec2::new(lit(50.0), lit(10.0)), R::from_i32(1));
            }
            if frame == 270 {
                let b = fx.bodies[16];
                let c = body::get_world_center(&fx.ctx, b);
                body::apply_force(&mut fx.ctx, b, Vec2::new(lit(0.0), lit(-500.0)), c);
                body::apply_torque(&mut fx.ctx, b, R::from_i32(50));
            }
        },
        0,
    );
}

#[test]
fn fixture_joints() {
    large_stack(run_joints);
}
fn run_joints() {
    let mut fx = Fx::new();
    let g = fx.ground();
    let a = fx.boxb(lit(20.0), lit(40.0), lit(2.0), lit(0.5), lit(1.0), lit(0.5), lit(0.1), lit(0.0), false, false);
    let b = fx.boxb(lit(24.0), lit(40.0), lit(0.5), lit(2.0), lit(1.0), lit(0.5), lit(0.1), lit(0.0), false, false);
    let mut rd = JointDef::revolute();
    joints::revolute_def_initialize(&fx.ctx, &mut rd, a, b, Vec2::new(lit(22.0), lit(40.0)));
    rd.enable_limit = hw_core::Flag32::TRUE;
    rd.lower_angle = lit(-0.5);
    rd.upper_angle = lit(0.5);
    rd.max_motor_torque = R::from_i32(4);
    let j = world::create_joint(&mut fx.ctx, &rd).1;
    fx.joints.push(j);
    let c = fx.boxb(lit(40.0), lit(30.0), lit(1.5), lit(1.5), lit(3.0), lit(0.8), lit(0.2), lit(0.2), false, false);
    let mut rd = JointDef::revolute();
    joints::revolute_def_initialize(&fx.ctx, &mut rd, g, c, Vec2::new(lit(40.0), lit(30.0)));
    rd.enable_motor = hw_core::Flag32::TRUE;
    rd.motor_speed = R::from_i32(5);
    rd.max_motor_torque = R::from_i32(100);
    let j = world::create_joint(&mut fx.ctx, &rd).1;
    fx.joints.push(j);
    let mut prev = g;
    for i in 0..6 {
        let n = fx.circle(
            R::from_i32(60 + i * 2),
            lit(20.0),
            lit(0.8),
            lit(1.0),
            lit(0.3),
            lit(0.1),
            false,
            false,
        );
        let mut rd = JointDef::revolute();
        joints::revolute_def_initialize(
            &fx.ctx,
            &mut rd,
            prev,
            n,
            Vec2::new(R::from_i32(59 + i * 2), lit(20.0)),
        );
        rd.collide_connected = hw_core::Flag32::new((i % 2) == 0);
        let j = world::create_joint(&mut fx.ctx, &rd).1;
        fx.joints.push(j);
        prev = n;
    }
    let p = fx.boxb(lit(80.0), lit(44.0), lit(3.0), lit(0.4), lit(2.0), lit(0.5), lit(0.1), lit(0.0), false, false);
    let mut pd = JointDef::prismatic();
    let pc = body::get_world_center(&fx.ctx, p);
    joints::prismatic_def_initialize(&fx.ctx, &mut pd, g, p, pc, Vec2::new(lit(0.0), lit(1.0)));
    pd.enable_limit = hw_core::Flag32::TRUE;
    pd.lower_translation = R::from_i32(-3);
    pd.upper_translation = R::from_i32(3);
    pd.enable_motor = hw_core::Flag32::TRUE;
    pd.max_motor_force = R::from_i32(300);
    pd.motor_speed = R::from_i32(4);
    let j = world::create_joint(&mut fx.ctx, &pd).1;
    fx.joints.push(j);
    let d1 = fx.circle(lit(30.0), lit(20.0), lit(1.0), lit(1.0), lit(0.5), lit(0.3), false, false);
    let d2 = fx.circle(lit(35.0), lit(22.0), lit(1.0), lit(1.0), lit(0.5), lit(0.3), false, false);
    let mut dd = JointDef::distance();
    let c1 = body::get_world_center(&fx.ctx, d1);
    let c2 = body::get_world_center(&fx.ctx, d2);
    joints::distance_def_initialize(&fx.ctx, &mut dd, d1, d2, c1, c2);
    dd.frequency_hz = R::from_i32(2);
    dd.damping_ratio = lit(0.5);
    let j = world::create_joint(&mut fx.ctx, &dd).1;
    fx.joints.push(j);
    let d3 = fx.circle(lit(30.0), lit(10.0), lit(0.7), lit(1.0), lit(0.5), lit(0.3), false, true);
    let mut dd = JointDef::distance();
    let c3 = body::get_world_center(&fx.ctx, d3);
    joints::distance_def_initialize(&fx.ctx, &mut dd, g, d3, Vec2::new(lit(30.0), lit(5.0)), c3);
    let j = world::create_joint(&mut fx.ctx, &dd).1;
    fx.joints.push(j);
    run(
        "joints",
        &mut fx,
        300,
        true,
        |fx, frame| {
            if frame == 60 {
                joints::set_motor_speed(&mut fx.ctx, fx.joints[8], R::from_i32(-4));
            }
            if frame == 120 {
                let _ = world::destroy_joint(&mut fx.ctx, fx.joints[3]);
            }
            if frame == 150 {
                joints::set_motor_speed(&mut fx.ctx, fx.joints[1], R::from_i32(-5));
                joints::set_max_motor_torque(&mut fx.ctx, fx.joints[1], R::from_i32(50));
            }
            if frame == 180 {
                joints::enable_limit(&mut fx.ctx, fx.joints[0], false);
                joints::enable_motor(&mut fx.ctx, fx.joints[0], true);
                joints::set_motor_speed(&mut fx.ctx, fx.joints[0], R::from_i32(2));
            }
            if frame == 210 {
                joints::set_limits(&mut fx.ctx, fx.joints[8], R::zero(), R::from_i32(1));
                joints::enable_motor(&mut fx.ctx, fx.joints[8], false);
            }
            if frame == 240 {
                let _ = world::destroy_body(&mut fx.ctx, fx.bodies[5]);
            }
            if frame == 260 {
                joints::set_limits(&mut fx.ctx, fx.joints[0], lit(-0.2), lit(0.2));
                joints::enable_limit(&mut fx.ctx, fx.joints[0], true);
            }
        },
        0,
    );
}

#[test]
fn fixture_statics() {
    large_stack(run_statics);
}
fn run_statics() {
    let mut fx = Fx::new();
    let bd = BodyDef::new();
    let g = world::create_body(&mut fx.ctx, &bd);
    fx.bodies.push(g);
    let mut pd = ShapeDef::polygon();
    pd.friction = R::from_i32(1);
    pd.restitution = lit(0.1);
    let mut cd = ShapeDef::circle();
    cd.friction = R::from_i32(1);
    cd.restitution = lit(0.1);
    for i in 0..300 {
        let x = R::from_i32(2) + R::from_i32(i % 30) * lit(3.3);
        let y = R::from_i32(30) + R::from_i32(i / 30) * lit(2.5) + fx.rnd();
        if i % 4 == 0 {
            cd.local_position.set(x, y);
            cd.radius = lit(0.5) + fx.rnd();
            assert_eq!(body::create_shape(&mut fx.ctx, g, &cd).0, Flow::Continue);
        } else {
            let hx = R::from_i32(1) + fx.rnd();
            let hy = lit(0.3) + fx.rnd() * lit(0.5);
            let ang = fx.rnd() * R::from_i32(3);
            hw_core::b2::shapes::set_as_oriented_box(&mut pd, hx, hy, Vec2::new(x, y), ang);
            assert_eq!(body::create_shape(&mut fx.ctx, g, &pd).0, Flow::Continue);
        }
    }
    for i in 0..25 {
        let x = R::from_i32(5) + R::from_i32(i) * lit(3.7);
        let y = R::from_i32(5) + fx.rnd() * R::from_i32(10);
        let r = lit(0.6) + fx.rnd();
        let d = R::from_i32(1) + fx.rnd() * R::from_i32(3);
        let b = fx.circle(x, y, r, d, lit(0.5), lit(0.2), false, (i % 6) == 5);
        let vx = fx.rnd() * R::from_i32(20) - R::from_i32(10);
        let vy = fx.rnd() * R::from_i32(10);
        body::set_linear_velocity(&mut fx.ctx, b, Vec2::new(vx, vy));
        let w = fx.rnd() * R::from_i32(10) - R::from_i32(5);
        body::set_angular_velocity(&mut fx.ctx, b, w);
    }
    run("statics", &mut fx, 200, false, |_, _| {}, 0);
}

#[test]
fn fixture_pairs() {
    large_stack(run_pairs);
}
fn run_pairs() {
    let mut fx = Fx::new();
    for i in 0..100 {
        let ang = R::from_i32(i) * lit(0.0628318530717958);
        let (c, s) = hw_core::b2::math::sin_cos(ang);
        let x = R::from_i32(60) + R::from_i32(30) * c;
        let y = R::from_i32(60) + R::from_i32(30) * s;
        let mut bd = BodyDef::new();
        bd.position.set(x, y);
        let b = world::create_body(&mut fx.ctx, &bd);
        let mut cd = ShapeDef::circle();
        cd.radius = R::from_i32(2);
        cd.density = R::from_i32(1);
        cd.is_sensor = hw_core::Flag32::new((i % 10) != 0);
        let _ = body::create_shape(&mut fx.ctx, b, &cd);
        let _ = body::set_mass_from_shapes(&mut fx.ctx, b);
        fx.bodies.push(b);
        body::set_linear_velocity(
            &mut fx.ctx,
            b,
            Vec2::new(
                (R::from_i32(60) - x) * lit(0.5),
                (R::from_i32(60) - y) * lit(0.5),
            ),
        );
    }
    fx.ctx.world.gravity.set(R::zero(), R::zero());
    run("pairs", &mut fx, 150, false, |_, _| {}, 0);
}

#[test]
fn fixture_proxies() {
    large_stack(run_proxies);
}
fn run_proxies() {
    let mut fx = Fx::new();
    fx.ground();
    let mut created = 0;
    let mut err_create = 0;
    let mut pd = ShapeDef::polygon();
    pd.density = R::from_i32(1);
    pd.friction = lit(0.5);
    for i in 0..520 {
        let mut bd = BodyDef::new();
        bd.position.set(
            R::from_i32(5) + R::from_i32(i % 40) * lit(2.3),
            R::from_i32(5) + R::from_i32(i / 40) * lit(1.7),
        );
        let b = world::create_body(&mut fx.ctx, &bd);
        fx.bodies.push(b);
        hw_core::b2::shapes::set_as_box(&mut pd, lit(0.4), lit(0.4));
        match body::create_shape(&mut fx.ctx, b, &pd).0 {
            Flow::Continue => created += 1,
            Flow::Abort(_) => {
                if err_create == 0 {
                    err_create = 1009;
                }
            }
        }
        let _ = body::set_mass_from_shapes(&mut fx.ctx, b);
    }
    let trace = load_trace("proxies");
    assert_eq!(trace[0]["created"].as_i64().unwrap(), created);
    assert_eq!(trace[0]["errCreate"].as_i64().unwrap(), err_create);
    assert_eq!(
        trace[0]["proxies"].as_i64().unwrap(),
        fx.ctx.world.broad_phase.proxy_count as i64
    );
    run("proxies", &mut fx, 60, false, |_, _| {}, 1);
}

#[test]
fn fixture_nan() {
    large_stack(run_nan);
}
fn run_nan() {
    let mut fx = Fx::new();
    fx.ground();
    for i in 0..10 {
        fx.boxb(
            R::from_i32(10 + i * 8),
            lit(20.0),
            lit(1.0),
            lit(1.0),
            lit(1.0),
            lit(0.5),
            lit(0.1),
            lit(0.0),
            false,
            false,
        );
    }
    run(
        "nan",
        &mut fx,
        100,
        false,
        |fx, frame| {
            if frame == 30 {
                let b = fx.bodies[3];
                // Flash's AS3 NaN constant (recorded by Box2dFixtureProbe).
                body::set_linear_velocity(
                    &mut fx.ctx,
                    b,
                    Vec2::new(R::from_words([0x7FFFFFFF, 0xE0000000]), R::zero()),
                );
            }
            if frame == 40 {
                let c = fx.bodies[5];
                let wc = body::get_world_center(&fx.ctx, c);
                body::apply_impulse(
                    &mut fx.ctx,
                    c,
                    Vec2::new(R::zero(), R::from_words([0x7FF00000, 0])),
                    wc,
                );
            }
        },
        0,
    );
}

#[test]
fn fixture_bullets() {
    large_stack(run_bullets);
}
fn run_bullets() {
    let mut fx = Fx::new();
    let g = fx.ground();
    let mut pd = ShapeDef::polygon();
    pd.friction = lit(0.3);
    hw_core::b2::shapes::set_as_oriented_box(
        &mut pd,
        lit(0.05),
        R::from_i32(10),
        Vec2::new(lit(60.0), lit(30.0)),
        R::zero(),
    );
    assert_eq!(body::create_shape(&mut fx.ctx, g, &pd).0, Flow::Continue);
    hw_core::b2::shapes::set_as_oriented_box(
        &mut pd,
        R::from_i32(10),
        lit(0.05),
        Vec2::new(lit(30.0), lit(45.0)),
        R::zero(),
    );
    assert_eq!(body::create_shape(&mut fx.ctx, g, &pd).0, Flow::Continue);
    for i in 0..10 {
        let b = fx.circle(
            R::from_i32(10 + i),
            R::from_i32(25) + R::from_i32(i) * lit(1.5),
            lit(0.3),
            lit(1.0),
            lit(0.3),
            lit(0.5),
            false,
            true,
        );
        body::set_linear_velocity(
            &mut fx.ctx,
            b,
            Vec2::new(R::from_i32(150), R::from_i32(10) * R::from_i32(i - 5)),
        );
    }
    for i in 0..5 {
        let k = fx.boxb(
            R::from_i32(25 + i * 2),
            lit(10.0),
            lit(0.3),
            lit(0.3),
            lit(1.0),
            lit(0.3),
            lit(0.2),
            lit(0.5),
            false,
            true,
        );
        body::set_linear_velocity(&mut fx.ctx, k, Vec2::new(R::zero(), R::from_i32(180)));
    }
    run("bullets", &mut fx, 120, false, |_, _| {}, 0);
}
