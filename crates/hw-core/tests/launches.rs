//! Non-official strings in reference/tas/launches.json (corrected TASes, search results):
//! each replays input length + 60 frames and must reproduce its recorded finish frame.
use hw_core::game::session::{self, ReplayState};
use hw_core::{Ctx, Flow, Input};
mod support;
use support::TestReal;

#[test]
fn registered_launches_reproduce_their_finish_frames() {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(|| {
            let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
            let text = std::fs::read_to_string(root.join("reference/tas/launches.json")).unwrap();
            let index: serde_json::Value = serde_json::from_str(&text).unwrap();
            for entry in index.as_array().unwrap() {
                let file = entry["file"].as_str().unwrap();
                let text = std::fs::read_to_string(root.join("reference/tas").join(file)).unwrap();
                let bits: Vec<u8> = text.bytes().filter(|b| !b.is_ascii_whitespace()).collect();
                let masks: Vec<u32> = bits
                    .chunks(8)
                    .map(|c| c.iter().enumerate().fold(0, |m, (k, &b)| m | (((b == b'1') as u32) << k)))
                    .collect();
                assert_eq!(masks.len() as u64, entry["frames"].as_u64().unwrap(), "{file}");
                let level = entry["level"].as_u64().unwrap() as u32;
                let character = entry["character"].as_u64().unwrap() as u32;
                let mut ctx = Box::new(Ctx::<TestReal>::new());
                assert_eq!(session::create(&mut ctx, level, character), Flow::Continue, "{file}");
                let mut state = ReplayState::new(masks.len() as u32);
                for _ in 0..masks.len() + 60 {
                    let mask = if state.input_allowed.get() && (state.iteration as usize) < masks.len() {
                        masks[state.iteration as usize]
                    } else {
                        0
                    };
                    let flow = session::step_bits(&mut ctx, &mut state, Input::from_mask(mask));
                    assert!(
                        !matches!(flow, Flow::Abort(hw_core::AbortSite::PortCapacity | hw_core::AbortSite::PortUnimplemented)),
                        "{file}: port-only abort {flow:?}"
                    );
                    ctx.end_frame();
                }
                let expected = entry["finish_frame"].as_u64().unwrap_or(0) as u32;
                assert_eq!(state.finish_frame, expected, "{file}: finish frame");
                println!("launch PASS: {file}, finish {expected}");
            }
        })
        .unwrap()
        .join()
        .unwrap();
}
