//! Original CharacterB2D methods called at fixed frames; callbacks and other actions disabled.
use hw_core::b2::world;
use hw_core::game::{character, character::*, composites, damage, level};
use hw_core::{Ctx, Flow};
use hw_real::Real;
mod support;
#[allow(unused_imports)]
use support::{lit, RawBits, TestReal as R};

fn scripted(ctx: &mut Ctx<R>, frame: usize, mode: &str) {
    if mode == "wheel-first" || mode == "chair-first" {
        use hw_core::game::wheelchair_damage as wd;
        let normal = hw_core::b2::math::Vec2::new(lit(0.6), lit(-0.8));
        if frame == if mode == "wheel-first" { 20 } else { 30 } {
            assert_eq!(wd::wheel_smash(ctx, lit(1000.0), normal), Flow::Continue);
        }
        if frame == if mode == "wheel-first" { 30 } else { 20 } {
            assert_eq!(wd::chair_smash(ctx, lit(1000.0), normal), Flow::Continue);
        }
        if frame == 40 {
            assert_eq!(wd::jet_smash(ctx, lit(1000.0)), Flow::Continue);
        }
        if frame == 50 {
            assert_eq!(wd::fuel_smash(ctx, lit(1000.0)), Flow::Continue);
        }
        return;
    }
    let limits = ctx.character.break_limits;
    let mid = |a: usize, b: usize| R::from_i32(limits[a] + limits[b]) / lit(2.0);
    if mode == "head" {
        if frame == 20 {
            assert_eq!(damage::head_smash(ctx, lit(1000.0)), Flow::Continue);
        }
    } else if mode == "chest" {
        if frame == 20 {
            assert_eq!(damage::chest_smash(ctx, lit(1000.0)), Flow::Continue);
        }
    } else if mode == "pelvis" {
        if frame == 20 {
            assert_eq!(damage::pelvis_smash(ctx, lit(1000.0)), Flow::Continue);
        }
    } else {
        match frame {
            20 => assert_eq!(damage::neck_break(ctx, mid(0, 1)), Flow::Continue),
            30 => assert_eq!(damage::torso_break(ctx, mid(2, 3)), Flow::Continue),
            40 => {
                for (j, impulse) in [
                    (ELBOW1, mid(8, 9)),
                    (ELBOW2, lit(1000.0)),
                    (KNEE1, mid(10, 11)),
                    (KNEE2, lit(1000.0)),
                ] {
                    assert_eq!(damage::distal_break(ctx, j, impulse), Flow::Continue);
                }
            }
            50 => {
                for (j, impulse) in [
                    (SHOULDER1, mid(4, 5)),
                    (SHOULDER2, lit(1000.0)),
                    (HIP1, mid(6, 7)),
                    (HIP2, lit(1000.0)),
                ] {
                    assert_eq!(damage::proximal_break(ctx, j, impulse), Flow::Continue);
                }
            }
            60 => {
                assert_eq!(damage::foot_smash(ctx, LOWER_LEG1, lit(1000.0)), Flow::Continue);
                assert_eq!(damage::foot_smash(ctx, LOWER_LEG2, lit(1000.0)), Flow::Continue);
            }
            70 => assert_eq!(damage::head_smash(ctx, lit(1000.0)), Flow::Continue),
            80 if mode == "stumps" => assert_eq!(damage::chest_smash(ctx, lit(1000.0)), Flow::Continue),
            90 if mode == "stumps" => assert_eq!(damage::pelvis_smash(ctx, lit(1000.0)), Flow::Continue),
            _ => {}
        }
    }
    assert_eq!(composites::check_joints(ctx), Flow::Continue);
}

fn check(mode: &'static str) {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || {
            let mut ctx = Ctx::<R>::new();
            let data = hw_data::level(45).unwrap();
            level::create_world(&mut ctx);
            assert_eq!(level::create(&mut ctx, data), Flow::Continue);
            assert_eq!(
                (if mode == "wheel-first" || mode == "chair-first" {
                    hw_core::game::wheelchair::create
                } else {
                    character::create_base
                })(
                    &mut ctx,
                    hw_data::character(1).unwrap(),
                    R::from_words(data.start_x),
                    R::from_words(data.start_y)
                ),
                Flow::Continue
            );
            let prefix = if mode == "wheel-first" || mode == "chair-first" {
                "wheelchair"
            } else {
                "character"
            };
            let path = format!(
                "{}/../../reference/traces/flash32/{prefix}-damage-{mode}-v1.jsonl",
                env!("CARGO_MANIFEST_DIR")
            );
            let text = std::fs::read_to_string(path).unwrap();
            assert_eq!(text.lines().count(), 300);
            for (i, line) in text.lines().enumerate() {
                let frame = i + 1;
                let r: serde_json::Value = serde_json::from_str(line).unwrap();
                assert_eq!(r["err"], 0, "Flash {mode} frame {frame}");
                assert_eq!(
                    world::step(&mut ctx, lit(0.03333333333333333), 10),
                    Flow::Continue
                );
                scripted(&mut ctx, frame, mode);
                let state = support::state_string(&ctx);
                if let Some(expected) = r["state"].as_str() {
                    if expected != state {
                        let at = expected
                            .bytes()
                            .zip(state.bytes())
                            .position(|(a, b)| a != b)
                            .unwrap_or(expected.len().min(state.len()));
                        let begin = at.saturating_sub(100);
                        panic!(
                            "damage {mode} frame {frame} byte {at}: Flash {} vs Rust {}",
                            &expected[begin..expected.len().min(at + 100)],
                            &state[begin..state.len().min(at + 100)]
                        );
                    }
                }
                let hash = format!(
                    "{:08x}{:08x}",
                    support::oaat(&state, 0),
                    support::oaat(&state, 2654435769)
                );
                assert_eq!(r["hash"], hash, "damage {mode} frame {frame}");
                assert_eq!(r["len"], state.len());
                ctx.end_frame();
            }
        })
        .unwrap()
        .join()
        .unwrap();
}
#[test]
fn breaks() {
    check("breaks");
}
#[test]
fn intact_head_smash() {
    check("head");
}
#[test]
fn intact_chest_smash() {
    check("chest");
}
#[test]
fn intact_pelvis_smash() {
    check("pelvis");
}
#[test]
fn stumps_then_smash() {
    check("stumps");
}
#[test]
fn wheel_then_chair() {
    check("wheel-first");
}
#[test]
fn chair_then_wheel() {
    check("chair-first");
}
