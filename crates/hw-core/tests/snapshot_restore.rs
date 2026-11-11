//! `Ctx::restore_from` skips the per-step scratch. Restoring every few frames into a context
//! whose scratch holds another replay's leftovers must reproduce a continuous run exactly.
use hw_core::game::session::{self, ReplayState};
use hw_core::{diagnostic, physics_observer, Ctx, Flow, Input};

fn masks(bits: &str) -> Vec<u32> {
    let bits: Vec<u8> = bits.bytes().filter(|b| !b.is_ascii_whitespace()).collect();
    bits.chunks(8)
        .map(|c| c.iter().enumerate().fold(0, |m, (k, &b)| m | (((b == b'1') as u32) << k)))
        .collect()
}

#[test]
fn restore_without_scratch_matches_continuous_replay() {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(|| {
            // Dirty scratch source: a different level and character.
            let mut dirty = Box::new(Ctx::<f64>::new());
            assert_eq!(session::create(&mut dirty, 3214, 4), Flow::Continue);
            let mut dirty_state = ReplayState::new(0);
            for _ in 0..40 {
                let _ = session::step_bits(&mut dirty, &mut dirty_state, Input::from_mask(0x42));
                dirty.end_frame();
            }
            for t in hw_data::TAS_STRINGS.iter().filter(|t| ["dotd4", "bmx1003", "gbe_err_1"].contains(&t.trace)) {
                let m = masks(t.bits);
                let mut continuous = Box::new(Ctx::<f64>::new());
                assert_eq!(session::create(&mut continuous, t.level, t.character), Flow::Continue);
                let mut restored = dirty.boxed_copy();
                restored.restore_from(&continuous);
                let mut a = ReplayState::new(m.len() as u32);
                let mut b = a;
                for frame in 1..=m.len() + 60 {
                    let mask = if a.input_allowed.get() && (a.iteration as usize) < m.len() { m[a.iteration as usize] } else { 0 };
                    let fa = session::step_bits(&mut continuous, &mut a, Input::from_mask(mask));
                    continuous.end_frame();
                    let fb = session::step_bits(&mut restored, &mut b, Input::from_mask(mask));
                    restored.end_frame();
                    assert_eq!(fa, fb, "{} frame {frame}", t.trace);
                    if frame % 7 == 0 {
                        // Snapshot round trip through a context with foreign scratch.
                        let snap = restored.boxed_copy();
                        restored.restore_from(&dirty);
                        restored.scratch = dirty.scratch;
                        restored.restore_from(&snap);
                    }
                    assert_eq!(diagnostic::replay_string(&continuous, &a), diagnostic::replay_string(&restored, &b), "{} frame {frame}", t.trace);
                    assert!(physics_observer::physics_string(&*continuous) == physics_observer::physics_string(&*restored), "{} frame {frame}: physics", t.trace);
                }
                println!("snapshot PASS: {}", t.trace);
            }
        })
        .unwrap()
        .join()
        .unwrap();
}
