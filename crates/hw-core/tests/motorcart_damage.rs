//! Forced original MotorCart methods; no ordinary callbacks/actions.
use hw_core::b2::world;
use hw_core::game::{level, motorcart, motorcart_damage as damage};
use hw_core::{Ctx, Flow};
use hw_real::Real;
mod support;
#[allow(unused_imports)]
use support::{lit, RawBits, TestReal as R};
fn check(mode: &'static str) {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || {
            let mut ctx = Ctx::<R>::new();
            let data = hw_data::level(45).unwrap();
            level::create_world(&mut ctx);
            assert_eq!(level::create(&mut ctx, data), Flow::Continue);
            assert_eq!(
                motorcart::create(
                    &mut ctx,
                    hw_data::character(4).unwrap(),
                    R::from_words(data.start_x),
                    R::from_words(data.start_y)
                ),
                Flow::Continue
            );
            let text = std::fs::read_to_string(format!(
                "{}/../../reference/traces/flash32/motorcart-damage-{mode}-v1.jsonl",
                env!("CARGO_MANIFEST_DIR")
            ))
            .unwrap();
            assert_eq!(text.lines().count(), 300);
            for (i, line) in text.lines().enumerate() {
                let r: serde_json::Value = serde_json::from_str(line).unwrap();
                let frame = i + 1;
                assert_eq!(r["err"], 0, "Flash {mode} frame {frame}");
                assert_eq!(world::step(&mut ctx, lit(1.0) / lit(30.0), 10), Flow::Continue);
                let result = match frame {
                    10 => damage::cracker_smash(&mut ctx, lit(1000.0)),
                    15 => damage::cola_smash(&mut ctx, lit(1000.0)),
                    20 if mode == "cart-first" => damage::cart_smash(&mut ctx, lit(1000.0)),
                    30 => damage::main_smash(&mut ctx, lit(1000.0)),
                    _ => Flow::Continue,
                };
                assert_eq!(result, Flow::Continue);
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
fn cart_first() {
    check("cart-first");
}
#[test]
fn main_first() {
    check("main-first");
}
