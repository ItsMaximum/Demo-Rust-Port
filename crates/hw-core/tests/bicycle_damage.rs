//! Scripted original bicycle and child transitions with ordinary callbacks disabled.
use hw_core::b2::{math::Vec2, world};
use hw_core::game::{bicycle, bicycle_damage as bd, character::*, child, damage, session};
use hw_core::{Ctx, Flow};
mod support;
#[allow(unused_imports)]
use support::{lit, RawBits, TestReal as R};
fn scripted(ctx: &mut Ctx<R>, frame: usize, mode: &str) -> Flow {
    use hw_core::flow;
    if mode == "wheel-first" || mode == "frame-first" {
        if frame == 10 {
            flow!(bd::helmet_smash(ctx, lit(1000.0)));
        }
        if frame == if mode == "wheel-first" { 20 } else { 30 } {
            flow!(bd::wheel_smash(
                ctx,
                bicycle::FRONT,
                lit(1000.0),
                Vec2::new(lit(0.6), lit(-0.8))
            ));
            flow!(bd::wheel_smash(
                ctx,
                bicycle::BACK,
                lit(1000.0),
                Vec2::new(lit(-0.8), lit(0.6))
            ));
        }
        if frame == if mode == "wheel-first" { 30 } else { 20 } {
            flow!(bd::frame_smash(ctx, lit(1000.0), Vec2::new(lit(0.6), lit(-0.8))));
        }
    } else {
        ctx.character.child_active = hw_core::Flag32::TRUE;
        if frame == 20 {
            if mode == "child-detach" {
                flow!(child::detach(ctx, lit(1000.0)));
            } else {
                flow!(damage::proximal_break(ctx, HIP1, lit(1000.0)));
                flow!(damage::proximal_break(ctx, HIP2, lit(1000.0)));
            }
        }
        if frame == 30 {
            flow!(child::eject(ctx));
        }
        if frame == 40 {
            flow!(damage::head_smash(ctx, lit(1000.0)));
        }
        if frame == 50 {
            flow!(damage::chest_smash(ctx, lit(1000.0)));
        }
        if frame == 60 {
            flow!(damage::pelvis_smash(ctx, lit(1000.0)));
        }
        ctx.character.child_active = hw_core::Flag32::FALSE;
    }
    Flow::Continue
}
fn check(mode: &'static str) {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || {
            let mut ctx = Ctx::<R>::new();
            assert_eq!(session::create(&mut ctx, 45, 3), Flow::Continue);
            ctx.listener_mode = 0;
            let text = std::fs::read_to_string(format!(
                "{}/../../reference/traces/flash32/bicycle-damage-{mode}-v2.jsonl",
                env!("CARGO_MANIFEST_DIR")
            ))
            .unwrap();
            assert_eq!(text.lines().count(), 300);
            for (i, line) in text.lines().enumerate() {
                let r: serde_json::Value = serde_json::from_str(line).unwrap();
                let frame = i + 1;
                let mut result = world::step(&mut ctx, lit(1.0) / lit(30.0), 10);
                if !result.is_abort() {
                    result = scripted(&mut ctx, frame, mode);
                }
                ctx.character.child_active = hw_core::Flag32::FALSE;
                assert_ne!(
                    result,
                    Flow::Abort(hw_core::AbortSite::PortUnimplemented),
                    "{mode} frame {frame}"
                );
                assert_ne!(
                    result,
                    Flow::Abort(hw_core::AbortSite::PortCapacity),
                    "{mode} frame {frame}"
                );
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
                            "{mode} frame {frame} byte {at}: Flash {} vs Rust {}",
                            &expected[begin..expected.len().min(at + 100)],
                            &state[begin..state.len().min(at + 100)]
                        );
                    }
                }
                assert_eq!(
                    r["err"],
                    if result.is_abort() { 1009 } else { 0 },
                    "{mode} frame {frame}: {result:?}"
                );
                let hash = format!(
                    "{:08x}{:08x}",
                    support::oaat(&state, 0),
                    support::oaat(&state, 2654435769)
                );
                assert_eq!(r["hash"], hash, "{mode} frame {frame}");
                assert_eq!(r["len"], state.len());
                ctx.end_frame();
            }
        })
        .unwrap()
        .join()
        .unwrap();
}
#[test]
fn wheel_first() {
    check("wheel-first");
}
#[test]
fn frame_first() {
    check("frame-first");
}
#[test]
fn child_detach() {
    check("child-detach");
}
#[test]
fn child_breaks() {
    check("child-breaks");
}
