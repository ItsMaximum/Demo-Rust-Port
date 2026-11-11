//! Stage-specific differential test: UserLevel static and dynamic shapes only.
//! Specials, groups, joints, and characters are deliberately absent in both simulations.
use hw_core::b2::world;
use hw_core::game::level::{self, LevelBuild};
use hw_core::{Ctx, Flow};
mod support;
#[allow(unused_imports)]
use support::{lit, RawBits, TestReal as R};
use support::{oaat, state_string};

fn check(id: u32) {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || {
            let mut ctx = Ctx::<R>::new();
            let mut build = LevelBuild::blank();
            let data = hw_data::level(id).unwrap();
            level::create_world(&mut ctx);
            assert_eq!(
                level::create_static_shapes(&mut ctx, data, &mut build),
                Flow::Continue
            );
            assert_eq!(
                level::create_dynamic_shapes(&mut ctx, data, &mut build),
                Flow::Continue
            );
            let path = format!(
                "{}/../../reference/traces/flash32/level-shapes-{id}-v1.jsonl",
                env!("CARGO_MANIFEST_DIR")
            );
            let trace = std::fs::read_to_string(path).unwrap();
            assert_eq!(trace.lines().count(), 600);
            for (i, line) in trace.lines().enumerate() {
                let reference: serde_json::Value = serde_json::from_str(line).unwrap();
                let flow = world::step(&mut ctx, lit(0.03333333333333333), 10);
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
    check(45);
}
#[test]
fn level_3214() {
    check(3214);
}
#[test]
fn level_13196() {
    check(13196);
}
#[test]
fn level_13812() {
    check(13812);
}
#[test]
fn level_13098() {
    check(13098);
}
#[test]
fn level_2433() {
    check(2433);
}
#[test]
fn level_10251() {
    check(10251);
}
#[test]
fn level_1144() {
    check(1144);
}
#[test]
fn level_22() {
    check(22);
}
