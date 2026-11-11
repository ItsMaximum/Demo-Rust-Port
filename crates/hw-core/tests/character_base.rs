//! CharacterB2D construction with each character's measured guide; callbacks/actions disabled.
use hw_core::b2::world;
use hw_core::game::{character, level};
use hw_core::{Ctx, Flow};
use hw_real::Real;
mod support;
#[allow(unused_imports)]
use support::{lit, RawBits, TestReal as R};

fn check(index: u32, vehicle: bool, composite: bool, breaks: bool) {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || {
            let mut ctx = Ctx::<R>::new();
            let data = hw_data::level(45).unwrap();
            level::create_world(&mut ctx);
            assert_eq!(level::create(&mut ctx, data), Flow::Continue);
            let flow = if vehicle {
                (if index == 1 {
                    hw_core::game::wheelchair::create
                } else if index == 3 {
                    hw_core::game::bicycle::create
                } else if index == 4 {
                    hw_core::game::motorcart::create
                } else {
                    hw_core::game::segway::create
                })(
                    &mut ctx,
                    hw_data::character(index).unwrap(),
                    R::from_words(data.start_x),
                    R::from_words(data.start_y),
                )
            } else {
                character::create_base(
                    &mut ctx,
                    hw_data::character(index).unwrap(),
                    R::from_words(data.start_x),
                    R::from_words(data.start_y),
                )
            };
            assert_eq!(flow, Flow::Continue);
            if composite {
                use hw_core::game::{
                    character::{LOWER_ARM1, UPPER_ARM1},
                    composites,
                };
                let upper = ctx.character.bodies[UPPER_ARM1];
                let lower = ctx.character.bodies[LOWER_ARM1];
                let mut out = hw_core::NONE;
                assert_eq!(
                    composites::ligament(&mut ctx, upper, lower, lit(7.5), lit(125.0), &mut out),
                    Flow::Continue
                );
                assert_eq!(
                    composites::spine(
                        &mut ctx,
                        hw_data::character(index).unwrap(),
                        4,
                        lit(9.25),
                        &mut out
                    ),
                    Flow::Continue
                );
                assert_eq!(
                    composites::intestine(&mut ctx, 3, lit(6.75), &mut out),
                    Flow::Continue
                );
            }
            let prefix = if vehicle {
                "character-physics"
            } else {
                "character-base"
            };
            let path = if composite {
                let name = if breaks {
                    "composites-breaks"
                } else {
                    "composites"
                };
                format!(
                    "{}/../../reference/traces/flash32/{name}-v1.jsonl",
                    env!("CARGO_MANIFEST_DIR")
                )
            } else {
                format!(
                    "{}/../../reference/traces/flash32/{prefix}-{index}-v1.jsonl",
                    env!("CARGO_MANIFEST_DIR")
                )
            };
            let text = std::fs::read_to_string(path).unwrap();
            assert_eq!(text.lines().count(), 300);
            for (frame, line) in text.lines().enumerate() {
                let r: serde_json::Value = serde_json::from_str(line).unwrap();
                assert_eq!(
                    world::step(&mut ctx, lit(0.03333333333333333), 10),
                    Flow::Continue
                );
                if composite {
                    if breaks && frame == 19 {
                        for c in 0..3 {
                            let count = if c == 0 { 1 } else { 2 };
                            for end in 0..count {
                                let j = ctx.character.composites[c].ends[end];
                                ctx.world.joint_mut(j).impulse = if c == 2 {
                                    lit(1.5000000000000002)
                                } else {
                                    lit(0.5000000000000001)
                                };
                            }
                        }
                    }
                    assert_eq!(
                        hw_core::game::composites::check_joints(&mut ctx),
                        Flow::Continue
                    );
                }
                assert_eq!(r["err"], 0);
                assert_eq!(r["lock"], 0);
                let state = support::state_string(&ctx);
                if let Some(expected) = r["state"].as_str() {
                    if expected != state {
                        let at = expected
                            .bytes()
                            .zip(state.bytes())
                            .position(|(a, b)| a != b)
                            .unwrap_or(expected.len().min(state.len()));
                        let begin = at.saturating_sub(80);
                        panic!(
                            "character {index} frame {} byte {at}: Flash {} vs Rust {}",
                            frame + 1,
                            &expected[begin..expected.len().min(at + 80)],
                            &state[begin..state.len().min(at + 80)]
                        );
                    }
                }
                let hash = format!(
                    "{:08x}{:08x}",
                    support::oaat(&state, 0),
                    support::oaat(&state, 2654435769)
                );
                assert_eq!(r["hash"], hash, "character {index} frame {}", frame + 1);
                assert_eq!(r["len"], state.len());
                ctx.end_frame();
            }
            if composite {
                let ligament = ctx.character.composites[0];
                assert_eq!(ctx.world.joint(ligament.ends[0]).broken, hw_core::Flag32::new(breaks));
                if breaks {
                    assert_ne!(ctx.world.joint(ligament.ends[0]).body1, ligament.upper);
                    for c in &ctx.character.composites[1..3] {
                        assert_eq!(c.ends, [hw_core::NONE; 2]);
                    }
                }
            }
        })
        .unwrap()
        .join()
        .unwrap();
}
#[test]
fn base_1() {
    check(1, false, false, false);
}
#[test]
fn base_2() {
    check(2, false, false, false);
}
#[test]
fn base_3() {
    check(3, false, false, false);
}
#[test]
fn base_4() {
    check(4, false, false, false);
}

#[test]
fn wheelchair() {
    check(1, true, false, false);
}
#[test]
fn segway() {
    check(2, true, false, false);
}
#[test]
fn bicycle() {
    check(3, true, false, false);
}
#[test]
fn motorcart() {
    check(4, true, false, false);
}
#[test]
fn composites() {
    check(1, false, true, false);
}

#[test]
fn composite_breaks() {
    check(1, false, true, true);
}
