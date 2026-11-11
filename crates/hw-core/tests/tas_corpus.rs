//! Official hw_all_v98 TAS inputs against the original player's diagnostic traces.
use hw_core::game::session::{self, ReplayState};
use hw_core::{AbortSite, Ctx, Flow};
mod support;
use support::TestReal;
fn check(name: &'static str) {
    check_schema(name, "diagnostic");
}
fn check_schema(name: &'static str, schema: &'static str) {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || {
            let tas = hw_data::TAS_STRINGS
                .iter()
                .find(|t| t.trace == name)
                .unwrap();
            let bits = tas.bits.as_bytes();
            assert_eq!(bits.len(), tas.frames as usize * 8);
            let mut ctx = Ctx::<TestReal>::new();
            assert_eq!(
                session::create(&mut ctx, tas.level, tas.character),
                Flow::Continue
            );
            let mut replay = ReplayState::new(tas.frames);
            let text = std::fs::read_to_string(format!(
                "{}/../../reference/traces/flash32/tas-{name}-{schema}-v1.jsonl",
                env!("CARGO_MANIFEST_DIR")
            ))
            .unwrap();
            assert_eq!(text.lines().count(), tas.frames as usize + 60);
            for (i, line) in text.lines().enumerate() {
                let frame = i + 1;
                let r: serde_json::Value = serde_json::from_str(line).unwrap();
                let mut keys = [false; 8];
                for k in 0..8 {
                    keys[k] = replay.input_allowed.get()
                        && bits.get(replay.iteration as usize * 8 + k) == Some(&b'1');
                }
                let input: String = keys.iter().map(|v| if *v { '1' } else { '0' }).collect();
                assert_eq!(r["input"], input, "{name} frame {frame}");
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    session::step(&mut ctx, &mut replay, keys)
                }))
                .unwrap_or_else(|_| panic!("simulation panic in {name} frame {frame}"));
                if result == Flow::Abort(AbortSite::PortCapacity) {
                    eprintln!(
                        "free slots: body {} shape {} joint {} contact {}; live b{} s{} j{} c{}",
                        ctx.world.bodies.free_len,
                        ctx.world.shapes.free_len,
                        ctx.world.joints.free_len,
                        ctx.world.contacts.free_len,
                        ctx.world.body_count,
                        ctx.world.shapes.live_count(),
                        ctx.world.joint_count,
                        ctx.world.contact_count
                    );
                }
                if r["bpHash"].is_string() {
                    let bp = support::broadphase_string(&ctx);
                    if let Some(expected) = r["bpState"].as_str() {
                        if bp != expected {
                            let at = bp
                                .bytes()
                                .zip(expected.bytes())
                                .position(|(a, b)| a != b)
                                .unwrap_or(bp.len().min(expected.len()));
                            let begin = at.saturating_sub(100);
                            panic!(
                                "broadphase {name} frame {frame} byte {at}: Flash {} vs Rust {}",
                                &expected[begin..expected.len().min(at + 100)],
                                &bp[begin..bp.len().min(at + 100)]
                            );
                        }
                    }
                    let hash = format!(
                        "{:08x}{:08x}",
                        support::oaat(&bp, 0),
                        support::oaat(&bp, 2654435769)
                    );
                    assert_eq!(r["bpHash"], hash, "broadphase {name} frame {frame}");
                }
                if r["physicsHash"].is_string() {
                    let physics = hw_core::physics_observer::physics_string(&ctx);
                    if let Some(expected) = r["physicsState"].as_str() {
                        if expected != physics {
                            let first = expected
                                .split(';')
                                .zip(physics.split(';'))
                                .find(|(a, b)| a != b);
                            panic!("physics {name} frame {frame}: {first:?}");
                        }
                    }
                    assert_eq!(
                        r["physicsHash"],
                        hw_core::diagnostic::hash(&physics),
                        "physics {name} frame {frame}"
                    );
                }
                if r["gameHash"].is_string() {
                    let game = hw_core::game_observer::game_string(&ctx);
                    if let Some(expected) = r["gameState"].as_str() {
                        if expected != game {
                            let first = expected
                                .split(';')
                                .zip(game.split(';'))
                                .find(|(a, b)| a != b);
                            panic!("game {name} frame {frame}: {first:?}");
                        }
                    }
                    assert_eq!(
                        r["gameHash"],
                        hw_core::diagnostic::hash(&game),
                        "game {name} frame {frame}"
                    );
                }
                if let Some(expected) = r["levelAddState"].as_str() {
                    assert_eq!(
                        expected,
                        hw_core::level_observer::level_add_string(&ctx),
                        "level ADD {name} frame {frame}"
                    );
                }
                for (hash_key, state_key, observe) in [
                    (
                        "listenersHash",
                        "listenersState",
                        hw_core::listeners_observer::listeners_string::<TestReal>
                            as fn(&Ctx<TestReal>) -> String,
                    ),
                    (
                        "callbacksHash",
                        "callbacksState",
                        hw_core::callbacks_observer::callbacks_string::<TestReal>
                            as fn(&Ctx<TestReal>) -> String,
                    ),
                    (
                        "levelHash",
                        "levelState",
                        hw_core::level_observer::level_string::<TestReal> as fn(&Ctx<TestReal>) -> String,
                    ),
                    (
                        "linksHash",
                        "linksState",
                        hw_core::links_observer::links_string::<TestReal> as fn(&Ctx<TestReal>) -> String,
                    ),
                    (
                        "mechanicsHash",
                        "mechanicsState",
                        hw_core::mechanics_observer::mechanics_string::<TestReal>
                            as fn(&Ctx<TestReal>) -> String,
                    ),
                ] {
                    if r[hash_key].is_string() {
                        let state = observe(&ctx);
                        if let Some(expected) = r[state_key].as_str() {
                            if expected != state {
                                let first = expected
                                    .split(';')
                                    .zip(state.split(';'))
                                    .find(|(a, b)| a != b);
                                panic!("{state_key} {name} frame {frame}: {first:?}");
                            }
                        }
                        assert_eq!(
                            r[hash_key],
                            hw_core::diagnostic::hash(&state),
                            "{hash_key} {name} frame {frame}"
                        );
                    }
                }
                assert_ne!(
                    result,
                    Flow::Abort(AbortSite::PortCapacity),
                    "{name} frame {frame}"
                );
                assert_ne!(
                    result,
                    Flow::Abort(AbortSite::PortUnimplemented),
                    "{name} frame {frame}"
                );
                assert_eq!(r["lock"], ctx.world.lock.word(), "{name} frame {frame}");
                let state = hw_core::diagnostic::replay_string(&ctx, &replay);
                if let Some(expected) = r["state"].as_str() {
                    if expected != state {
                        let at = expected
                            .bytes()
                            .zip(state.bytes())
                            .position(|(a, b)| a != b)
                            .unwrap_or(expected.len().min(state.len()));
                        let begin = at.saturating_sub(100);
                        panic!(
                            "{name} frame {frame} byte {at}: Flash {} vs Rust {}",
                            &expected[begin..expected.len().min(at + 100)],
                            &state[begin..state.len().min(at + 100)]
                        );
                    }
                }
                assert_eq!(
                    r["err"],
                    if result.is_abort() { 1009 } else { 0 },
                    "{name} frame {frame}: {result:?}"
                );
                let hash = format!(
                    "{:08x}{:08x}",
                    support::oaat(&state, 0),
                    support::oaat(&state, 2654435769)
                );
                assert_eq!(r["hash"], hash, "{name} frame {frame}");
                assert_eq!(r["len"], state.len());
                assert_eq!(r["iteration"], replay.iteration);
                assert_eq!(r["dead"], ctx.character.dead.word());
                assert_eq!(r["inputAllowed"], replay.input_allowed.word());
                assert_eq!(r["finish"], replay.finish_frame, "{name} frame {frame}");
                ctx.end_frame();
            }
        })
        .unwrap()
        .join()
        .unwrap();
}
#[test]
fn dotd_pairmax_1() {
    check("dotd_pairmax_1");
}
#[test]
fn gbe_err_1() {
    check("gbe_err_1");
}
#[cfg(feature = "full-corpus")]
#[test]
fn gbe_freeze_2() {
    check("gbe_freeze_2");
}
#[test]
fn dotd6() {
    check("dotd6");
}
#[test]
fn snowy() {
    check("snowy");
}
#[test]
fn bmx1003() {
    check("bmx1003");
}
#[test]
fn bmx610() {
    check("bmx610");
}
#[test]
fn dotd4() {
    check("dotd4");
}
#[cfg(feature = "full-corpus")]
#[test]
fn physics_dotd6() {
    check_schema("dotd6", "physics");
}
#[test]
fn physics_dotd_pairmax_1() {
    check_schema("dotd_pairmax_1", "physics");
}
#[test]
fn physics_gbe_err_1() {
    check_schema("gbe_err_1", "physics");
}
#[test]
fn physics_gbe_freeze_2() {
    check_schema("gbe_freeze_2", "physics");
}
#[cfg(feature = "full-corpus")]
#[test]
fn physics_snowy() {
    check_schema("snowy", "physics");
}
#[cfg(feature = "full-corpus")]
#[test]
fn physics_bmx1003() {
    check_schema("bmx1003", "physics");
}
#[cfg(feature = "full-corpus")]
#[test]
fn physics_bmx610() {
    check_schema("bmx610", "physics");
}
#[cfg(feature = "full-corpus")]
#[test]
fn physics_dotd4() {
    check_schema("dotd4", "physics");
}
#[test]
fn game_dotd_pairmax_1() {
    check_schema("dotd_pairmax_1", "game");
}
#[test]
fn game_gbe_err_1() {
    check_schema("gbe_err_1", "game");
}
#[test]
fn game_gbe_freeze_2() {
    check_schema("gbe_freeze_2", "game");
}
#[cfg(feature = "full-corpus")]
#[test]
fn game_dotd6() {
    check_schema("dotd6", "game");
}
#[cfg(feature = "full-corpus")]
#[test]
fn game_snowy() {
    check_schema("snowy", "game");
}
#[cfg(feature = "full-corpus")]
#[test]
fn game_bmx1003() {
    check_schema("bmx1003", "game");
}
#[cfg(feature = "full-corpus")]
#[test]
fn game_bmx610() {
    check_schema("bmx610", "game");
}
#[cfg(feature = "full-corpus")]
#[test]
fn game_dotd4() {
    check_schema("dotd4", "game");
}
#[cfg(feature = "full-corpus")]
#[test]
fn links_gbe_freeze_2() {
    check_schema("gbe_freeze_2", "links");
}
#[cfg(feature = "full-corpus")]
#[test]
fn mechanics_dotd_pairmax_1() {
    check_schema("dotd_pairmax_1", "mechanics");
}
#[cfg(feature = "full-corpus")]
#[test]
fn level_dotd_pairmax_1() {
    check_schema("dotd_pairmax_1", "level");
}
#[cfg(feature = "full-corpus")]
#[test]
fn level_gbe_freeze_2() {
    check_schema("gbe_freeze_2", "level");
}
#[cfg(feature = "full-corpus")]
#[test]
fn level_gbe_err_1() {
    check_schema("gbe_err_1", "level");
}
#[cfg(feature = "full-corpus")]
#[test]
fn level_dotd6() {
    check_schema("dotd6", "level");
}
#[cfg(feature = "full-corpus")]
#[test]
fn level_snowy() {
    check_schema("snowy", "level");
}
#[cfg(feature = "full-corpus")]
#[test]
fn level_bmx1003() {
    check_schema("bmx1003", "level");
}
#[cfg(feature = "full-corpus")]
#[test]
fn level_bmx610() {
    check_schema("bmx610", "level");
}
#[cfg(feature = "full-corpus")]
#[test]
fn level_dotd4() {
    check_schema("dotd4", "level");
}

#[test]
fn level_add_dotd_pairmax_1() {
    check_schema("dotd_pairmax_1", "level-add");
}

#[test]
fn listeners_dotd_pairmax_1() {
    check_schema("dotd_pairmax_1", "listeners");
}

#[cfg(feature = "full-corpus")]

#[test]
fn listeners_gbe_err_1() {
    check_schema("gbe_err_1", "listeners");
}

#[cfg(feature = "full-corpus")]

#[test]
fn listeners_gbe_freeze_2() {
    check_schema("gbe_freeze_2", "listeners");
}

#[cfg(feature = "full-corpus")]

#[test]
fn listeners_dotd6() {
    check_schema("dotd6", "listeners");
}

#[cfg(feature = "full-corpus")]

#[test]
fn listeners_snowy() {
    check_schema("snowy", "listeners");
}

#[cfg(feature = "full-corpus")]

#[test]
fn listeners_bmx1003() {
    check_schema("bmx1003", "listeners");
}

#[cfg(feature = "full-corpus")]

#[test]
fn listeners_bmx610() {
    check_schema("bmx610", "listeners");
}

#[cfg(feature = "full-corpus")]

#[test]
fn listeners_dotd4() {
    check_schema("dotd4", "listeners");
}
