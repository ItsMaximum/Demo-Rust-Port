//! Generated level-v1 inputs with physics, game, retained joint links, mechanics,
//! callbacks and level state. Listener identity and level ADD use separate TAS fixtures.
use hw_core::{
    diagnostic,
    game::session::{self, ReplayState},
    Ctx, Flow,
};
mod support;
use support::TestReal;

fn check(level: u32, character: u32, style: &'static str) {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || {
            let root = format!("{}/../..", env!("CARGO_MANIFEST_DIR"));
            let name = format!("coverage-v1-L{level}-C{character}-{style}");
            let bits = std::fs::read_to_string(format!("{root}/reference/tas/extra/{name}.tas"))
                .unwrap()
                .split_whitespace()
                .collect::<String>();
            assert_eq!(bits.len(), 600 * 8);
            let trace = std::fs::read_to_string(format!(
                "{root}/reference/traces/flash32/{name}-level-v1.jsonl"
            ))
            .unwrap();
            assert_eq!(trace.lines().count(), 660);
            let mut ctx = Ctx::<TestReal>::new();
            assert_eq!(session::create(&mut ctx, level, character), Flow::Continue);
            let mut replay = ReplayState::new(600);
            for (i, line) in trace.lines().enumerate() {
                let row: serde_json::Value = serde_json::from_str(line).unwrap();
                let mut keys = [false; 8];
                for k in 0..8 {
                    keys[k] = replay.input_allowed.get()
                        && bits.as_bytes().get(replay.iteration as usize * 8 + k) == Some(&b'1');
                }
                let input: String = keys.iter().map(|v| if *v { '1' } else { '0' }).collect();
                assert_eq!(row["input"], input, "{name} frame {}", i + 1);
                assert_eq!(
                    session::step(&mut ctx, &mut replay, keys),
                    Flow::Continue,
                    "{name} frame {}",
                    i + 1
                );
                assert_eq!(row["err"], 0);
                assert_eq!(row["iteration"], replay.iteration);
                assert_eq!(row["inputAllowed"], replay.input_allowed.word());
                assert_eq!(row["finish"], replay.finish_frame);
                assert_eq!(row["lock"], ctx.world.lock.word());
                assert_eq!(row["dead"], ctx.character.dead.word());
                let state = diagnostic::replay_string(&ctx, &replay);
                assert_eq!(row["len"], state.len());
                for (hash_key, raw_key, state) in [
                    ("hash", "state", state),
                    ("bpHash", "bpState", diagnostic::broadphase_string(&ctx)),
                    (
                        "physicsHash",
                        "physicsState",
                        hw_core::physics_observer::physics_string(&ctx),
                    ),
                    (
                        "gameHash",
                        "gameState",
                        hw_core::game_observer::game_string(&ctx),
                    ),
                    (
                        "linksHash",
                        "linksState",
                        hw_core::links_observer::links_string(&ctx),
                    ),
                    (
                        "mechanicsHash",
                        "mechanicsState",
                        hw_core::mechanics_observer::mechanics_string(&ctx),
                    ),
                    (
                        "callbacksHash",
                        "callbacksState",
                        hw_core::callbacks_observer::callbacks_string(&ctx),
                    ),
                    (
                        "levelHash",
                        "levelState",
                        hw_core::level_observer::level_string(&ctx),
                    ),
                ] {
                    if let Some(expected) = row[raw_key].as_str() {
                        assert_eq!(expected, state, "{name} frame {} {raw_key}", i + 1);
                    }
                    assert_eq!(
                        row[hash_key],
                        diagnostic::hash(&state),
                        "{name} frame {} {hash_key}",
                        i + 1
                    );
                }
                ctx.end_frame();
            }
        })
        .unwrap()
        .join()
        .unwrap();
}
macro_rules! level_cases {
    ($name:ident, $level:literal, $shipped:meta) => {
        mod $name {
            #[allow(unused_imports)]
            use super::check;
            #[cfg(feature = "full-corpus")]
            #[test]
            fn wheelchair_idle() {
                check($level, 1, "idle");
            }
            #[cfg(any(feature = "full-corpus", $shipped))]
            #[test]
            fn wheelchair_forward_jumpy() {
                check($level, 1, "forward-jumpy");
            }
            #[cfg(feature = "full-corpus")]
            #[test]
            fn wheelchair_death_seek() {
                check($level, 1, "death-seek");
            }
            #[cfg(feature = "full-corpus")]
            #[test]
            fn segway_idle() {
                check($level, 2, "idle");
            }
            #[cfg(any(feature = "full-corpus", $shipped))]
            #[test]
            fn segway_forward_jumpy() {
                check($level, 2, "forward-jumpy");
            }
            #[cfg(feature = "full-corpus")]
            #[test]
            fn segway_death_seek() {
                check($level, 2, "death-seek");
            }
            #[cfg(feature = "full-corpus")]
            #[test]
            fn bicycle_idle() {
                check($level, 3, "idle");
            }
            #[cfg(any(feature = "full-corpus", $shipped))]
            #[test]
            fn bicycle_forward_jumpy() {
                check($level, 3, "forward-jumpy");
            }
            #[cfg(feature = "full-corpus")]
            #[test]
            fn bicycle_death_seek() {
                check($level, 3, "death-seek");
            }
            #[cfg(feature = "full-corpus")]
            #[test]
            fn motorcart_idle() {
                check($level, 4, "idle");
            }
            #[cfg(any(feature = "full-corpus", $shipped))]
            #[test]
            fn motorcart_forward_jumpy() {
                check($level, 4, "forward-jumpy");
            }
            #[cfg(feature = "full-corpus")]
            #[test]
            fn motorcart_death_seek() {
                check($level, 4, "death-seek");
            }
        }
    };
}
// Only the forward-jumpy traces for level 22 ship in the repository; the rest need the full corpus.
level_cases!(level_22, 22, all());
level_cases!(level_45, 45, any());
level_cases!(level_1144, 1144, any());
level_cases!(level_2433, 2433, any());
level_cases!(level_3214, 3214, any());
// Bicycle death-seek reaches 231 allocations: obsolete grip roots must be released.
level_cases!(level_10251, 10251, any());
level_cases!(level_13098, 13098, any());
level_cases!(level_13196, 13196, any());
level_cases!(level_13812, 13812, any());

#[test]
fn unreachable_joint_cycles_are_recycled() {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(|| {
            use hw_core::{b2::state::Joint, NONE};
            let mut ctx = Ctx::<TestReal>::new();
            let root = ctx.world.joints.alloc(Joint::blank());
            let a = ctx.world.joints.alloc(Joint::blank());
            let b = ctx.world.joints.alloc(Joint::blank());
            ctx.world.joint_mut(root).next = a;
            ctx.world.joint_mut(a).next = b;
            ctx.world.joint_mut(b).prev = a;
            ctx.world.joints.destroy(a);
            ctx.world.joints.destroy(b);
            let free = ctx.world.joints.free_len;
            ctx.end_frame();
            assert_eq!(ctx.world.joints.free_len, free);
            assert_eq!(ctx.world.joints.retired_len, 2);
            assert_eq!(ctx.world.joint(a).next, b);
            ctx.world.joint_mut(root).next = NONE;
            ctx.end_frame();
            assert_eq!(ctx.world.joints.free_len, free + 2);
            assert_eq!(ctx.world.joints.retired_len, 0);
            assert_eq!(ctx.world.joints.live_count(), 1);
        })
        .unwrap()
        .join()
        .unwrap();
}

#[test]
fn released_joint_roots_preserve_reachable_tombstones() {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(|| {
            use hw_core::{b2::state::Joint, NONE};
            let mut ctx = Ctx::<TestReal>::new();
            let root = ctx.world.joints.alloc(Joint::blank());
            ctx.world.joints.retain(root);
            ctx.world.joints.release_root(root);
            ctx.end_frame();
            assert!(ctx.world.joints.is_alive(root));

            let pending = ctx.world.joints.alloc(Joint::blank());
            ctx.world.joints.retain(pending);
            ctx.world.joints.destroy(pending);
            ctx.world.joints.release_root(pending);
            let free = ctx.world.joints.free_len;
            ctx.end_frame();
            assert_eq!(ctx.world.joints.free_len, free + 1);
            assert_eq!(ctx.world.joints.retired_len, 0);

            let retired = ctx.world.joints.alloc(Joint::blank());
            ctx.world.joints.retain(retired);
            ctx.world.joints.destroy(retired);
            ctx.end_frame();
            assert_eq!(ctx.world.joints.retired_len, 1);
            ctx.world.joint_mut(root).next = retired;
            ctx.world.joints.release_root(retired);
            ctx.end_frame();
            assert_eq!(ctx.world.joints.retired_len, 1);
            assert_eq!(ctx.world.joints.free_len, free);
            ctx.world.joint_mut(root).next = NONE;
            ctx.end_frame();
            assert_eq!(ctx.world.joints.retired_len, 0);
            assert_eq!(ctx.world.joints.free_len, free + 1);
            assert_eq!(ctx.world.joints.live_count(), 1);
        })
        .unwrap()
        .join()
        .unwrap();
}
