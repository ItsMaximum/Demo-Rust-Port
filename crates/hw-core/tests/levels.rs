//! Full level construction and actions, without character physics.
use hw_core::b2::{body, math::Vec2, world};
use hw_core::game::{level, specials};
use hw_core::{Ctx, Flow, NONE};
mod support;
#[allow(unused_imports)]
use support::{lit, RawBits, TestReal as R};
use support::{oaat, state_string};

fn check(id: u32, kicks: bool, actors: bool) {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || {
            let mut ctx = Ctx::<R>::new();
            let data = hw_data::level(id).unwrap();
            level::create_world(&mut ctx);
            ctx.listener_mode = 1;
            assert_eq!(level::create(&mut ctx, data), Flow::Continue);
            ctx.level.focus = ctx.level.level_body;
            if actors {
                add_actors(&mut ctx);
            }
            let prefix = if actors {
                "level-actors"
            } else if kicks {
                "level-kicks"
            } else {
                "level"
            };
            let path = format!(
                "{}/../../reference/traces/flash32/{prefix}-{id}-v1.jsonl",
                env!("CARGO_MANIFEST_DIR")
            );
            let trace = std::fs::read_to_string(path).unwrap();
            assert_eq!(trace.lines().count(), 600);
            for (i, line) in trace.lines().enumerate() {
                let reference: serde_json::Value = serde_json::from_str(line).unwrap();
                if kicks && [1, 61, 181, 361].contains(&(i + 1)) {
                    let mut b = ctx.world.body_list;
                    let mut index = 0;
                    while b != NONE {
                        if !body::is_static(&ctx, b) {
                            let mass = body::get_mass(&ctx, b);
                            let p = body::get_world_center(&ctx, b);
                            let x = if index % 2 == 0 { lit(4.0) } else { lit(-4.0) };
                            body::apply_impulse(
                                &mut ctx,
                                b,
                                Vec2::new(mass * x, mass * lit(-8.0)),
                                Vec2::new(p.x + lit(0.125), p.y),
                            );
                            index += 1;
                        }
                        b = ctx.world.body(b).next;
                    }
                }
                let mut flow = world::step(&mut ctx, lit(0.03333333333333333), 10);
                if !flow.is_abort() {
                    flow = specials::actions(&mut ctx);
                }
                assert_ne!(
                    flow,
                    Flow::Abort(hw_core::flow::AbortSite::PortUnimplemented)
                );
                assert_ne!(flow, Flow::Abort(hw_core::flow::AbortSite::PortCapacity));
                assert_eq!(reference["frame"], (i + 1) as u32);
                assert_eq!(
                    reference["err"],
                    if flow.is_abort() { 1009 } else { 0 },
                    "level {id} frame {} error",
                    i + 1
                );
                assert_eq!(reference["lock"], ctx.world.lock.word());
                let state = state_string(&ctx);
                if let Some(expected) = reference["state"].as_str() {
                    if expected != state {
                        let at = expected
                            .bytes()
                            .zip(state.bytes())
                            .position(|(a, b)| a != b)
                            .unwrap_or(expected.len().min(state.len()));
                        let start = at.saturating_sub(80);
                        panic!(
                            "level {id} frame {} state byte {at}: Flash {} vs port {}",
                            i + 1,
                            &expected[start..expected.len().min(at + 80)],
                            &state[start..state.len().min(at + 80)]
                        );
                    }
                }
                let hash = format!("{:08x}{:08x}", oaat(&state, 0), oaat(&state, 2654435769));
                assert_eq!(reference["hash"], hash, "level {id} frame {}", i + 1);
                assert_eq!(reference["len"], state.len());
                ctx.end_frame();
            }
        })
        .unwrap()
        .join()
        .unwrap();
}

#[test]
fn level_45() {
    check(45, false, false);
}
#[test]
fn level_3214() {
    check(3214, false, false);
}
#[test]
fn level_13196() {
    check(13196, false, false);
}
#[test]
fn level_13812() {
    check(13812, false, false);
}
#[test]
fn level_13098() {
    check(13098, false, false);
}
#[test]
fn level_2433() {
    check(2433, false, false);
}
#[test]
fn level_10251() {
    check(10251, false, false);
}
#[test]
fn level_1144() {
    check(1144, false, false);
}
#[test]
fn level_22() {
    check(22, false, false);
}

#[test]
fn kicks_45() {
    check(45, true, false);
}

#[test]
fn kicks_3214() {
    check(3214, true, false);
}

#[test]
fn kicks_13196() {
    check(13196, true, false);
}

#[test]
fn kicks_13812() {
    check(13812, true, false);
}

#[test]
fn kicks_13098() {
    check(13098, true, false);
}

#[test]
fn kicks_2433() {
    check(2433, true, false);
}

#[test]
fn kicks_10251() {
    check(10251, true, false);
}

#[test]
fn kicks_1144() {
    check(1144, true, false);
}

#[test]
fn kicks_22() {
    check(22, true, false);
}

#[test]
fn actors_45() {
    check(45, false, true);
}

#[test]
fn actors_3214() {
    check(3214, false, true);
}

#[test]
fn actors_13196() {
    check(13196, false, true);
}

#[test]
fn actors_13812() {
    check(13812, false, true);
}

#[test]
fn actors_13098() {
    check(13098, false, true);
}

#[test]
fn actors_2433() {
    check(2433, false, true);
}

#[test]
fn actors_10251() {
    check(10251, false, true);
}

#[test]
fn actors_1144() {
    check(1144, false, true);
}

#[test]
fn actors_22() {
    check(22, false, true);
}

fn add_actors(ctx: &mut Ctx<R>) {
    use hw_core::b2::state::{BodyDef, ShapeDef, E_POLYGON_SHAPE};
    let mut targets = [Vec2::zero(); 16];
    let mut count = 0;
    let mut b = ctx.world.body_list;
    while b != NONE && count < 8 {
        let mut sh = ctx.world.body(b).shape_list;
        while sh != NONE && count < 8 {
            let shape = ctx.world.shape(sh);
            if shape.is_sensor.get() && shape.type_ == E_POLYGON_SHAPE {
                targets[count] = body::get_world_point(ctx, b, shape.centroid);
                count += 1;
            }
            sh = shape.next;
        }
        b = ctx.world.body(b).next;
    }
    b = ctx.world.body_list;
    let mut dynamic = 0;
    while b != NONE && dynamic < 8 {
        if !body::is_static(ctx, b) {
            let p = body::get_world_center(ctx, b);
            targets[count] = Vec2::new(p.x, p.y - lit(0.5));
            count += 1;
            dynamic += 1;
        }
        b = ctx.world.body(b).next;
    }
    for p in &targets[..count] {
        let mut bd = BodyDef::new();
        bd.position = *p;
        let b = world::create_body(ctx, &bd);
        assert_ne!(b, NONE);
        let mut d = ShapeDef::circle();
        d.radius = lit(0.2);
        d.density = lit(100.0);
        d.filter.category_bits = 4;
        let (f, sh) = body::create_shape(ctx, b, &d);
        assert_eq!(f, Flow::Continue);
        ctx.world.shape_mut(sh).material = 1;
        assert_eq!(body::set_mass_from_shapes(ctx, b), Flow::Continue);
        body::set_linear_velocity(ctx, b, Vec2::new(lit(0.0), lit(10.0)));
    }
}
