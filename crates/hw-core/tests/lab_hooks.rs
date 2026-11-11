//! Armed `lab` recording must not change the simulation: every official string and every
//! registered launch runs twice in lockstep (unarmed, and armed watching every body slot), and
//! all five observer strings and replay state must match after every frame.
#![cfg(feature = "lab")]
use hw_core::game::session::{self, ReplayState};
use hw_core::{diagnostic, game_observer, links_observer, mechanics_observer, physics_observer, Ctx, Flow, Input};

fn masks(bits: &str) -> Vec<u32> {
    let bits: Vec<u8> = bits.bytes().filter(|b| !b.is_ascii_whitespace()).collect();
    bits.chunks(8)
        .map(|c| c.iter().enumerate().fold(0, |m, (k, &b)| m | (((b == b'1') as u32) << k)))
        .collect()
}

#[derive(Default)]
struct Counts { events: usize, damage_results: usize, damage_snapshots: usize, damage_consumes: usize }

fn lockstep(name: &str, level: u32, character: u32, masks: &[u32], frames: usize) -> Counts {
    let all: Vec<u32> = (0..hw_core::b2::state::MAX_BODIES as u32).collect();
    let mut plain = Box::new(Ctx::<f64>::new());
    assert_eq!(session::create(&mut plain, level, character), Flow::Continue);
    let mut armed = plain.clone();
    let mut a = ReplayState::new(masks.len() as u32);
    let mut b = a;
    let mut counts = Counts::default();
    for frame in 1..=frames {
        let mask = |s: &ReplayState| {
            if s.input_allowed.get() && (s.iteration as usize) < masks.len() { masks[s.iteration as usize] } else { 0 }
        };
        hw_core::lab::disarm();
        let ma = mask(&a);
        let fa = session::step_bits(&mut plain, &mut a, Input::from_mask(ma));
        plain.end_frame();
        hw_core::lab::arm(&all);
        let mb = mask(&b);
        let fb = session::step_bits(&mut armed, &mut b, Input::from_mask(mb));
        armed.end_frame();
        hw_core::lab::disarm();
        let events = hw_core::lab::take_events();
        counts.events += events.len();
        for event in events {
            match event {
                hw_core::lab::Event::DamageResult { child, .. } => {
                    assert!(!child, "{name} frame {frame}: child damage result recorded as primary");
                    counts.damage_results += 1;
                }
                hw_core::lab::Event::DamageSnapshot { child, .. } => {
                    assert!(!child, "{name} frame {frame}: child snapshot recorded as primary");
                    counts.damage_snapshots += 1;
                }
                hw_core::lab::Event::DamageConsume { child, .. } => {
                    assert!(!child, "{name} frame {frame}: child consume recorded as primary");
                    counts.damage_consumes += 1;
                }
                _ => {}
            }
        }
        assert_eq!(fa, fb, "{name} frame {frame}: flow");
        assert_eq!(diagnostic::replay_string(&plain, &a), diagnostic::replay_string(&armed, &b), "{name} frame {frame}");
        assert!(diagnostic::broadphase_string(&*plain) == diagnostic::broadphase_string(&*armed), "{name} frame {frame}: broadphase");
        assert!(physics_observer::physics_string(&*plain) == physics_observer::physics_string(&*armed), "{name} frame {frame}: physics");
        assert!(game_observer::game_string(&*plain) == game_observer::game_string(&*armed), "{name} frame {frame}: game");
        assert!(links_observer::links_string(&*plain) == links_observer::links_string(&*armed), "{name} frame {frame}: links");
        assert!(mechanics_observer::mechanics_string(&*plain) == mechanics_observer::mechanics_string(&*armed), "{name} frame {frame}: mechanics");
    }
    counts
}

#[test]
fn armed_recording_leaves_every_frame_unchanged() {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(|| {
            let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
            let mut cases: Vec<(String, u32, u32, Vec<u32>)> = hw_data::TAS_STRINGS
                .iter()
                .map(|t| (t.file.to_string(), t.level, t.character, masks(t.bits)))
                .collect();
            let text = std::fs::read_to_string(root.join("reference/tas/launches.json")).unwrap();
            let index: serde_json::Value = serde_json::from_str(&text).unwrap();
            for e in index.as_array().unwrap() {
                let file = e["file"].as_str().unwrap();
                let bits = std::fs::read_to_string(root.join("reference/tas").join(file)).unwrap();
                cases.push((file.into(), e["level"].as_u64().unwrap() as u32, e["character"].as_u64().unwrap() as u32, masks(&bits)));
            }
            for (name, level, character, m) in &cases {
                let counts = lockstep(name, *level, *character, m, m.len() + 60);
                assert!(counts.events > 0, "{name}: armed run recorded nothing");
                println!("lab hooks PASS: {name}, {} events", counts.events);
            }
        })
        .unwrap()
        .join()
        .unwrap();
}

#[test]
fn primary_damage_recording_preserves_full299_root10_and_comparator() {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(|| {
            let cases = [
                ("root10", include_str!("../../../reference/tas/obstacle-dad-ceiling-strand-f143.tas"), true),
                ("pulse comparator", include_str!("../../../reference/tas/obstacle-dad-pulse127-3-f139.tas"), false),
            ];
            for (name, bits, chest_consumed) in cases {
                let m = masks(bits);
                assert_eq!(m.len(), 299, "{name}: preserve the finite literal length");
                let counts = lockstep(name, 45, 3, &m, 299);
                assert!(counts.damage_results > 0, "{name}: no incoming primary CHEST result");
                assert_eq!(counts.damage_snapshots, 299, "{name}: snapshot before primary consumption every frame");
                if chest_consumed {
                    assert!(counts.damage_consumes > 0, "{name}: no actual primary CHEST dispatch");
                } else {
                    assert_eq!(counts.damage_consumes, 0, "{name}: measured negative control has no CHEST dispatch");
                }
            }
        })
        .unwrap()
        .join()
        .unwrap();
}

/// Exact replay fields omitted from diagnostic::replay_string must match too:
/// input allowance/completion and the first finish latch survive original aborts.
fn replay_words(replay: &ReplayState) -> [u32; 6] {
    [replay.iteration, replay.frames, replay.total_iterations,
        replay.input_allowed.word(), replay.completed.word(), replay.finish_frame]
}

#[test]
fn passive_pair_exhaustion_capture_preserves_exact_full299_controls() {
    std::thread::Builder::new()
        .name("passive-pair-full299-parity".into())
        .stack_size(64 << 20)
        .spawn(|| {
            hw_core::lab::set_deadline(None);
            let cases = [
                // Exact nonwinning Original Flash MATCH controls. Capture is a
                // diagnostic witness, never a finish or collision certificate.
                ("pairmax-f274", include_str!("../../../reference/tas/obstacle-pairmax-f274.tas"), 274),
                ("late-overflow-f291", include_str!("../../../reference/tas/obstacle-dad-late-overflow-f291.tas"), 291),
            ];
            for (name, bits, expected_first_site2) in cases {
                assert_eq!(bits.len(), 299 * 8, "{name}: preserve all 299 exact entries");
                assert!(bits.bytes().all(|b| b == b'0' || b == b'1'), "{name}: ASCII/no LF");
                let input = masks(bits);
                assert_eq!(input.len(), 299);
                // Independent initialization, rather than inheriting one run's
                // mutated state or its TLS recorder context through a clone.
                hw_core::lab::disarm();
                let mut plain = Box::new(Ctx::<f64>::new());
                let mut captured = Box::new(Ctx::<f64>::new());
                assert_eq!(session::create(&mut plain, 45, 3), Flow::Continue);
                assert_eq!(session::create(&mut captured, 45, 3), Flow::Continue);
                let mut a = ReplayState::new(299);
                let mut b = ReplayState::new(299);
                let mut first_site2 = None;
                let mut site2_frames = 0;
                for frame in 1..=299 {
                    assert_eq!(replay_words(&a), replay_words(&b), "{name} F{frame}: pre-input replay");
                    assert_eq!(a.iteration, frame - 1, "{name} F{frame}: consumed-input origin");
                    let ma = input[a.iteration as usize];
                    let mb = input[b.iteration as usize];
                    assert_eq!(ma, mb, "{name} F{frame}: requested input");
                    hw_core::lab::disarm();
                    let fa = session::step_bits(&mut plain, &mut a, Input::from_mask(ma));
                    plain.end_frame();
                    let off = hw_core::lab::take_pair_exhaustion();
                    assert!(off.first.is_none());
                    assert_eq!(off.count, 0);
                    assert!(!off.count_saturated);
                    // No watched body: the fixed failure capture is global and
                    // must not depend on a primary/child body watch selection.
                    hw_core::lab::arm(&[]);
                    let fb = session::step_bits(&mut captured, &mut b, Input::from_mask(mb));
                    captured.end_frame();
                    hw_core::lab::disarm();
                    let on = hw_core::lab::take_pair_exhaustion();
                    let drained = hw_core::lab::take_pair_exhaustion();
                    assert!(drained.first.is_none());
                    assert_eq!(drained.count, 0);
                    assert!(!drained.count_saturated);
                    let _ = hw_core::lab::take_events();
                    assert_eq!(fa, fb, "{name} F{frame}: original Continue/Abort");
                    assert_eq!(replay_words(&a), replay_words(&b), "{name} F{frame}: exact replay words");
                    assert_eq!(a.iteration, frame, "{name} F{frame}: original world abort still consumes input");
                    assert_eq!(a.frames, frame);
                    assert_eq!(a.finish_frame, 0, "{name} F{frame}: nonwinning control finish latch");
                    assert_eq!(plain.world.lock.word(), captured.world.lock.word(), "{name} F{frame}: lock");
                    let sa = diagnostic::replay_string(&plain, &a);
                    let sb = diagnostic::replay_string(&captured, &b);
                    assert!(sa == sb, "{name} F{frame}: diagnostic-v1 state words");
                    for (kind, left, right) in [
                        ("broadphase", diagnostic::broadphase_string(&*plain), diagnostic::broadphase_string(&*captured)),
                        ("physics", physics_observer::physics_string(&*plain), physics_observer::physics_string(&*captured)),
                        ("game", game_observer::game_string(&*plain), game_observer::game_string(&*captured)),
                        ("links", links_observer::links_string(&*plain), links_observer::links_string(&*captured)),
                        ("mechanics", mechanics_observer::mechanics_string(&*plain), mechanics_observer::mechanics_string(&*captured)),
                    ] {
                        assert!(left == right, "{name} F{frame}: {kind} exact observer words");
                    }
                    if fa == Flow::Abort(hw_core::AbortSite::PairPoolExhausted) {
                        first_site2.get_or_insert(frame);
                        site2_frames += 1;
                        assert!(on.first.is_some(), "{name} F{frame}: missing actual failure witness");
                        assert_eq!(on.count, 1, "{name} F{frame}: unchanged first-abort return");
                    } else {
                        assert!(on.first.is_none(), "{name} F{frame}: invented failure witness");
                        assert_eq!(on.count, 0);
                    }
                    assert!(!on.count_saturated);
                }
                assert_eq!(first_site2, Some(expected_first_site2), "{name}: fixture must exercise actual capture");
                assert!(site2_frames > 0);
                assert_eq!(replay_words(&a), [299, 299, 299, 1, 0, 0]);
                assert_eq!(replay_words(&a), replay_words(&b));
                // Capture-on/off parity does not identify a physical contact,
                // prove selective omission, or qualify any QE/finish outcome.
                println!("passive pair parity PASS: {name}, full299, {site2_frames} original Site2 frames");
            }
        })
        .unwrap()
        .join()
        .unwrap();
}
