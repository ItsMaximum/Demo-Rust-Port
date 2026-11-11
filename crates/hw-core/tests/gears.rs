//! Original b2GearJoint, including post-construction ratio mutation and mixed joint types.
use hw_core::b2::{
    body, joints,
    math::Vec2,
    state::{BodyDef, JointDef, ShapeDef},
    world,
};
use hw_core::game::{character, level};
use hw_core::{Ctx, Flow, NONE};
use hw_real::Real;
mod support;
#[allow(unused_imports)]
use support::{lit, RawBits, TestReal as R};
#[test]
fn original_gears() {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(|| {
            let mut ctx = Ctx::<R>::new();
            let data = hw_data::level(45).unwrap();
            level::create_world(&mut ctx);
            assert_eq!(level::create(&mut ctx, data), Flow::Continue);
            assert_eq!(
                character::create_base(
                    &mut ctx,
                    hw_data::character(1).unwrap(),
                    R::from_words(data.start_x),
                    R::from_words(data.start_y)
                ),
                Flow::Continue
            );
            let mut bodies = [NONE; 3];
            let mut source = [NONE; 3];
            let mut circle = ShapeDef::circle();
            circle.radius = lit(0.25);
            circle.density = lit(1.0);
            for i in 0..3 {
                let mut bd = BodyDef::new();
                bd.position = Vec2::new(lit(50.0) + R::from_i32(i as i32), lit(-40.0));
                bodies[i] = world::create_body(&mut ctx, &bd);
                assert_eq!(
                    body::create_shape(&mut ctx, bodies[i], &circle).0,
                    Flow::Continue
                );
                assert_eq!(
                    body::set_mass_from_shapes(&mut ctx, bodies[i]),
                    Flow::Continue
                );
            }
            for i in 0..2 {
                let mut d = JointDef::revolute();
                d.enable_motor = hw_core::Flag32::new(i == 0);
                d.motor_speed = lit(3.0);
                d.max_motor_torque = lit(20.0);
                let ground = ctx.world.ground_body;
                let p = body::get_position(&ctx, bodies[i]);
                joints::revolute_def_initialize(&ctx, &mut d, ground, bodies[i], p);
                let (f, j) = world::create_joint(&mut ctx, &d);
                assert_eq!(f, Flow::Continue);
                source[i] = j;
            }
            let mut d = JointDef::prismatic();
            d.enable_limit = hw_core::Flag32::TRUE;
            d.lower_translation = lit(-1.0);
            d.upper_translation = lit(1.0);
            joints::prismatic_def_initialize(
                &ctx,
                &mut d,
                ctx.world.ground_body,
                bodies[2],
                body::get_position(&ctx, bodies[2]),
                Vec2::new(lit(0.6), lit(0.8)),
            );
            let (f, j) = world::create_joint(&mut ctx, &d);
            assert_eq!(f, Flow::Continue);
            source[2] = j;
            let mut d = JointDef::gear();
            d.body1 = bodies[0];
            d.body2 = bodies[1];
            d.gear_joints = [source[0], source[1]];
            let (f, j) = world::create_joint(&mut ctx, &d);
            assert_eq!(f, Flow::Continue);
            ctx.world.joint_mut(j).ratio = ctx.world.joint(j).ratio * lit(-1.0);
            d.body1 = bodies[1];
            d.body2 = bodies[2];
            d.gear_joints = [source[1], source[2]];
            d.ratio = lit(2.25);
            assert_eq!(world::create_joint(&mut ctx, &d).0, Flow::Continue);
            let text = include_str!("../../../reference/traces/flash32/gears-v1.jsonl");
            assert_eq!(text.lines().count(), 300);
            for (i, line) in text.lines().enumerate() {
                let r: serde_json::Value = serde_json::from_str(line).unwrap();
                assert_eq!(world::step(&mut ctx, lit(1.0) / lit(30.0), 10), Flow::Continue);
                assert_eq!(r["err"], 0);
                let state = support::state_string(&ctx);
                if let Some(expected) = r["state"].as_str() {
                    if expected != state {
                        let at = expected
                            .bytes()
                            .zip(state.bytes())
                            .position(|(a, b)| a != b)
                            .unwrap_or(expected.len().min(state.len()));
                        let start = at.saturating_sub(100);
                        panic!(
                            "gear frame {} byte {at}: Flash {} vs Rust {}",
                            i + 1,
                            &expected[start..expected.len().min(at + 100)],
                            &state[start..state.len().min(at + 100)]
                        );
                    }
                }
                let hash = format!(
                    "{:08x}{:08x}",
                    support::oaat(&state, 0),
                    support::oaat(&state, 2654435769)
                );
                assert_eq!(r["hash"], hash, "gear frame {}", i + 1);
                assert_eq!(r["len"], state.len());
                ctx.end_frame();
            }
        })
        .unwrap()
        .join()
        .unwrap();
}
