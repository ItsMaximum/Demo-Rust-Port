//! Staged controls fixture and complete generated replay, including contacts and damage.
use hw_core::b2::{body, world};
use hw_core::game::{level, specials, wheelchair};
use hw_core::{Ctx, Flow};
use hw_real::Real;
mod support;
#[allow(unused_imports)]
use support::{lit, RawBits, TestReal as R};

fn check(full: bool, index: u32) {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || {
            let mut ctx = Ctx::<R>::new();
            let data = hw_data::level(45).unwrap();
            level::create_world(&mut ctx);
            assert_eq!(level::create(&mut ctx, data), Flow::Continue);
            assert_eq!(
                (if index == 1 {
                    wheelchair::create
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
                    R::from_words(data.start_y)
                ),
                Flow::Continue
            );
            let tas: Vec<u8> =
                include_str!("../../../reference/tas/generated/wheelchair-controls-v1.tas")
                    .bytes()
                    .filter(|c| *c == b'0' || *c == b'1')
                    .collect();
            let name = if index == 4 {
                "motorcart-controls-v1"
            } else if index == 3 {
                "bicycle-controls-v1"
            } else if index == 2 {
                "segway-controls-v1"
            } else if full {
                "wheelchair-session-v1"
            } else {
                "wheelchair-controls-v1"
            };
            let text = std::fs::read_to_string(format!(
                "{}/../../reference/traces/flash32/{name}.jsonl",
                env!("CARGO_MANIFEST_DIR")
            ))
            .unwrap();
            if full {
                ctx.listener_mode = 1;
            }
            let mut iteration = 0;
            let mut input_allowed = true;
            assert_eq!(text.lines().count(), 320);
            for (frame, line) in text.lines().enumerate() {
                let r: serde_json::Value = serde_json::from_str(line).unwrap();
                let mut keys = [false; 8];
                if input_allowed {
                    if iteration >= tas.len() / 8 {
                        input_allowed = false;
                    }
                    for k in 0..8 {
                        keys[k] = tas.get(iteration * 8 + k) == Some(&b'1');
                    }
                    assert_eq!(
                        hw_core::game::session::replay_input(&mut ctx, keys),
                        Flow::Continue
                    );
                    iteration += 1;
                } else {
                    assert_eq!(
                        hw_core::game::session::input(&mut ctx, keys),
                        Flow::Continue
                    );
                }
                assert_eq!(hw_core::game::session::actions(&mut ctx), Flow::Continue);
                assert_eq!(
                    world::step(&mut ctx, lit(0.03333333333333333), 10),
                    Flow::Continue
                );
                if full {
                    assert_eq!(
                        hw_core::game::damage::consume_results(&mut ctx),
                        Flow::Continue,
                        "results frame {}",
                        frame + 1
                    );
                    assert_eq!(
                        hw_core::game::damage::check_joints(&mut ctx),
                        Flow::Continue,
                        "joints frame {}",
                        frame + 1
                    );
                }
                assert_eq!(specials::actions(&mut ctx), Flow::Continue);
                assert_eq!(r["err"], 0);
                assert_eq!(r["lock"], 0);
                let p = body::get_world_center(&ctx, ctx.level.focus);
                let state = format!(
                    "{}#g{},{},{},{:016x},{:016x}",
                    support::state_string(&ctx),
                    ctx.character.dead.word(),
                    ctx.character.current_pose,
                    iteration,
                    p.x.to_bits(),
                    p.y.to_bits()
                );
                if let Some(expected) = r["state"].as_str() {
                    if expected != state {
                        let at = expected
                            .bytes()
                            .zip(state.bytes())
                            .position(|(a, b)| a != b)
                            .unwrap_or(expected.len().min(state.len()));
                        let begin = at.saturating_sub(80);
                        panic!(
                            "frame {} byte {at}: Flash {} vs Rust {}",
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
                assert_eq!(r["hash"], hash, "frame {}", frame + 1);
                assert_eq!(r["len"], state.len());
                assert_eq!(r["iteration"], iteration);
                assert_eq!(r["inputAllowed"], input_allowed as u32);
                ctx.end_frame();
            }
        })
        .unwrap()
        .join()
        .unwrap();
}
#[test]
fn generated_wheelchair_controls() {
    check(false, 1);
}
#[cfg(feature = "full-corpus")]
#[test]
fn generated_wheelchair_session() {
    check(true, 1);
}
#[cfg(feature = "full-corpus")]
#[test]
fn generated_segway_controls() {
    check(false, 2);
}
#[cfg(feature = "full-corpus")]
#[test]
fn generated_bicycle_controls() {
    check(false, 3);
}

#[cfg(feature = "full-corpus")]

#[test]
fn generated_motorcart_controls() {
    check(false, 4);
}
