//! Complete original-player generated replays across all demo levels.
use hw_core::b2::body;
use hw_core::game::session::{self, ReplayState};
use hw_core::{AbortSite, Ctx, Flow};
mod support;
#[allow(unused_imports)]
use support::{lit, RawBits, TestReal as R};
fn check_character(id: u32, index: u32) {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || {
            let mut ctx = Ctx::<R>::new();
            assert_eq!(session::create(&mut ctx, id, index), Flow::Continue);
            let tas: Vec<u8> =
                include_str!("../../../reference/tas/generated/wheelchair-controls-v1.tas")
                    .bytes()
                    .filter(|c| *c == b'0' || *c == b'1')
                    .collect();
            let mut replay = ReplayState::new((tas.len() / 8) as u32);
            let name = match index {
                1 => "wheelchair",
                2 => "segway",
                3 => "bicycle",
                4 => "motorcart",
                _ => unreachable!(),
            };
            let text = std::fs::read_to_string(format!(
                "{}/../../reference/traces/flash32/{name}-session-{id}-v1.jsonl",
                env!("CARGO_MANIFEST_DIR")
            ))
            .unwrap();
            assert_eq!(text.lines().count(), 600);
            for (i, line) in text.lines().enumerate() {
                let r: serde_json::Value = serde_json::from_str(line).unwrap();
                let mut keys = [false; 8];
                for k in 0..8 {
                    keys[k] = tas.get(replay.iteration as usize * 8 + k) == Some(&b'1');
                }
                let flow = session::step(&mut ctx, &mut replay, keys);
                assert_ne!(
                    flow,
                    Flow::Abort(AbortSite::PortCapacity),
                    "level {id} frame {}",
                    i + 1
                );
                assert_ne!(
                    flow,
                    Flow::Abort(AbortSite::PortUnimplemented),
                    "level {id} frame {}",
                    i + 1
                );
                assert_eq!(
                    r["err"],
                    if flow.is_abort() { 1009 } else { 0 },
                    "level {id} frame {} ({flow:?})",
                    i + 1
                );
                assert_eq!(r["lock"], ctx.world.lock.word());
                let p = body::get_world_center(&ctx, ctx.level.focus);
                let state = format!(
                    "{}#g{},{},{},{:016x},{:016x}",
                    support::state_string(&ctx),
                    ctx.character.dead.word(),
                    ctx.character.current_pose,
                    replay.iteration,
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
                        let begin = at.saturating_sub(100);
                        panic!(
                            "level {id} frame {} byte {at}: Flash {} vs Rust {}",
                            i + 1,
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
                assert_eq!(r["hash"], hash, "level {id} frame {}", i + 1);
                assert_eq!(r["len"], state.len());
                assert_eq!(r["iteration"], replay.iteration);
                assert_eq!(r["inputAllowed"], replay.input_allowed.word());
                assert_eq!(r["dead"], ctx.character.dead.word());
                ctx.end_frame();
            }
        })
        .unwrap()
        .join()
        .unwrap();
}
fn check(id: u32) {
    check_character(id, 1);
}
#[test]
fn bicycle_45() {
    check_character(45, 3);
}
#[test]
fn bicycle_22() {
    check_character(22, 3);
}
#[test]
fn bicycle_1144() {
    check_character(1144, 3);
}
#[test]
fn bicycle_2433() {
    check_character(2433, 3);
}
#[test]
fn bicycle_3214() {
    check_character(3214, 3);
}
#[test]
fn bicycle_10251() {
    check_character(10251, 3);
}
#[test]
fn bicycle_13098() {
    check_character(13098, 3);
}
#[test]
fn bicycle_13196() {
    check_character(13196, 3);
}
#[test]
fn bicycle_13812() {
    check_character(13812, 3);
}
#[test]
fn segway_22() {
    check_character(22, 2);
}
#[test]
fn segway_45() {
    check_character(45, 2);
}
#[test]
fn segway_1144() {
    check_character(1144, 2);
}
#[test]
fn segway_2433() {
    check_character(2433, 2);
}
#[test]
fn segway_3214() {
    check_character(3214, 2);
}
#[test]
fn segway_10251() {
    check_character(10251, 2);
}
#[test]
fn segway_13098() {
    check_character(13098, 2);
}
#[test]
fn segway_13196() {
    check_character(13196, 2);
}
#[test]
fn segway_13812() {
    check_character(13812, 2);
}
#[test]
fn level_22() {
    check(22);
}
#[test]
fn level_45() {
    check(45);
}
#[test]
fn level_1144() {
    check(1144);
}
#[test]
fn level_2433() {
    check(2433);
}
#[test]
fn level_3214() {
    check(3214);
}
#[test]
fn level_10251() {
    check(10251);
}
#[test]
fn level_13098() {
    check(13098);
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
fn motorcart_45() {
    check_character(45, 4);
}
#[test]
fn motorcart_22() {
    check_character(22, 4);
}
#[test]
fn motorcart_1144() {
    check_character(1144, 4);
}
#[test]
fn motorcart_2433() {
    check_character(2433, 4);
}
#[test]
fn motorcart_3214() {
    check_character(3214, 4);
}
#[test]
fn motorcart_10251() {
    check_character(10251, 4);
}
#[test]
fn motorcart_13098() {
    check_character(13098, 4);
}
#[test]
fn motorcart_13196() {
    check_character(13196, 4);
}
#[test]
fn motorcart_13812() {
    check_character(13812, 4);
}
