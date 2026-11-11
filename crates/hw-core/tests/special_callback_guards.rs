//! Original special-object sound callbacks still have simulation-relevant reads.
mod support;
#[allow(unused_imports)]
use support::{lit, RawBits, TestReal as R};
use hw_core::{
    b2::{
        math::Vec2,
        shapes,
        state::{ContactPoint, ShapeDef},
    },
    game::{
        contact_listener,
        session::{self, ReplayState},
    },
    AbortSite, Ctx, Flow, NONE,
};

fn check(level: u32) {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || {
            let root = format!("{}/../..", env!("CARGO_MANIFEST_DIR"));
            let trace = std::fs::read_to_string(format!(
                "{root}/reference/traces/flash32/special-callback-guards-{level}-v1.jsonl"
            ))
            .unwrap();
            assert_eq!(trace.lines().count(), 3);
            let mut ctx = Ctx::<R>::new();
            assert_eq!(session::create(&mut ctx, level, 1), Flow::Continue);
            let mut replay = ReplayState::new(1);
            let mut actual = String::new();
            for (i, line) in trace.lines().enumerate() {
                let row: serde_json::Value = serde_json::from_str(line).unwrap();
                assert_eq!(
                    session::step(&mut ctx, &mut replay, [false; 8]),
                    Flow::Continue
                );
                if i == 0 {
                    let expected = row["specialCallbackGuards"].as_str().unwrap();
                assert_eq!(expected.is_empty(), level == 13196,
                    "BMX beams/spikes are immovable; the other fixtures must exercise registered callbacks");
                    let mut map = vec![NONE];
                    let mut body = ctx.world.body_list;
                while body != NONE {
                        let mut shape = ctx.world.body(body).shape_list;
                        while shape != NONE {
                            map.push(shape);
                            shape = ctx.world.shape(shape).next;
                        }
                    body = ctx.world.body(body).next;
                }
                let registered: Vec<usize> = map.iter().enumerate().skip(1)
                    .filter(|(_,shape)| ctx.world.shape(**shape).handlers[0] & hw_core::game::specials::SOUND_HANDLER != 0)
                    .map(|(index,_)|index).collect();
                let expected_registered: Vec<usize> = expected.split(';').filter(|s|!s.is_empty())
                    .filter_map(|field| {
                        let (key,_) = field.split_once('=').unwrap();
                        let (index,sensor) = key.split_once('.').unwrap();
                        (sensor == "1").then(||index.parse().unwrap())
                    }).collect();
                assert_eq!(registered,expected_registered,"level {level} ADD registration population");
                    let other = shapes::create(&mut ctx, &ShapeDef::circle());
                    for field in expected.split(';').filter(|s| !s.is_empty()) {
                        let (key, _) = field.split_once('=').unwrap();
                        let (index, sensor) = key.split_once('.').unwrap();
                        ctx.world.shape_mut(other).is_sensor = hw_core::Flag32::new(sensor == "1");
                        let point = ContactPoint {
                            shape1: map[index.parse::<usize>().unwrap()],
                            shape2: other,
                            position: Vec2::zero(),
                            velocity: Vec2::zero(),
                            normal: Vec2::zero(),
                            separation: lit(0.0),
                            friction: lit(0.0),
                            restitution: lit(0.0),
                            id: 0,
                        };
                        let error = match contact_listener::add(&mut ctx, &point) {
                            Flow::Continue => 0,
                            Flow::Abort(AbortSite::GameNullDeref) => 1009,
                            other => panic!("unexpected special callback result {other:?}"),
                        };
                        actual += &format!("{key}={error};");
                    }
                }
                assert_eq!(
                    row["specialCallbackGuards"],
                    actual,
                    "level {level} frame {}",
                    i + 1
                );
                for (field, state) in [
                    (
                        "physicsState",
                        hw_core::physics_observer::physics_string(&ctx),
                    ),
                    ("gameState", hw_core::game_observer::game_string(&ctx)),
                    ("linksState", hw_core::links_observer::links_string(&ctx)),
                    (
                        "callbacksState",
                        hw_core::callbacks_observer::callbacks_string(&ctx),
                    ),
                    ("levelState", hw_core::level_observer::level_string(&ctx)),
                ] {
                    assert_eq!(row[field], state, "level {level} frame {} {field}", i + 1);
                }
                ctx.end_frame();
            }
        })
        .unwrap()
        .join()
        .unwrap();
}
#[cfg(feature = "full-corpus")]
#[test]
fn immovable_beams_and_spikes() {
    check(13196);
}
#[cfg(feature = "full-corpus")]
#[test]
fn vans_and_wrecking_ball() {
    check(3214);
}
#[cfg(feature = "full-corpus")]
#[test]
fn spike_bases() {
    check(45);
}
