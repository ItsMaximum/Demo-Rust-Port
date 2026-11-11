//! Original registered callbacks: sensor guards versus null-body reads.
mod support;
#[allow(unused_imports)]
use support::{lit, RawBits, TestReal as R};
use hw_core::b2::{
    math::Vec2,
    shapes,
    state::{ContactPoint, ShapeDef},
};
use hw_core::game::{
    contact_listener,
    session::{self, ReplayState},
};
use hw_core::{AbortSite, Ctx, Flow};

fn guards(ctx: &mut Ctx<R>, character: u32) -> String {
    let mut def = ShapeDef::circle();
    def.is_sensor = hw_core::Flag32::TRUE;
    let sensor = shapes::create(ctx, &def);
    let mut keys = vec![("shapes[0]", ctx.character.shapes[0])];
    match character {
        1 => keys.push(("wheelchair.shapes[3]", ctx.character.wheelchair.shapes[3])),
        2 => keys.push(("segway.shapes[2]", ctx.character.segway.shapes[2])),
        3 => {
            keys.push(("bicycle.shapes[3]", ctx.character.bicycle.shapes[3]));
            keys.push(("bicycle.shapes[4]", ctx.character.bicycle.shapes[4]));
        }
        4 => {
            keys.push(("motorcart.shapes[5]", ctx.character.motorcart.shapes[5]));
            keys.push(("motorcart.shapes[6]", ctx.character.motorcart.shapes[6]));
        }
        _ => unreachable!(),
    }
    let mut output = String::new();
    for (name, own) in keys {
        for event in 0..2 {
            let point = ContactPoint {
                shape1: own,
                shape2: sensor,
                position: Vec2::zero(),
                velocity: Vec2::zero(),
                normal: Vec2::zero(),
                separation: lit(0.0),
                friction: lit(0.0),
                restitution: lit(0.0),
                id: 0,
            };
            let result = if event == 0 {
                contact_listener::add(ctx, &point)
            } else {
                contact_listener::remove(ctx, &point)
            };
            let error = match result {
                Flow::Continue => 0,
                Flow::Abort(AbortSite::GameNullDeref) => 1009,
                other => panic!("unexpected callback result {other:?}"),
            };
            output += &format!("{name}.{event}={error};");
        }
    }
    output
}

fn check(character: u32) {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || {
            let mut ctx = Ctx::<R>::new();
            assert_eq!(session::create(&mut ctx, 45, character), Flow::Continue);
            let mut replay = ReplayState::new(1);
            let trace = std::fs::read_to_string(format!(
                "{}/../../reference/traces/flash32/callback-guards-char{character}-v1.jsonl",
                env!("CARGO_MANIFEST_DIR")
            ))
            .unwrap();
            let mut result = String::new();
            assert_eq!(trace.lines().count(), 3);
            for (i, line) in trace.lines().enumerate() {
                let row: serde_json::Value = serde_json::from_str(line).unwrap();
                assert_eq!(
                    session::step(&mut ctx, &mut replay, [false; 8]),
                    Flow::Continue
                );
                if i == 0 {
                    result = guards(&mut ctx, character);
                }
                assert_eq!(
                    row["callbackGuards"],
                    result,
                    "character {character}, frame {}",
                    i + 1
                );
                for (field, actual) in [
                    (
                        "physicsState",
                        hw_core::physics_observer::physics_string(&ctx),
                    ),
                    ("gameState", hw_core::game_observer::game_string(&ctx)),
                    ("linksState", hw_core::links_observer::links_string(&ctx)),
                    (
                        "mechanicsState",
                        hw_core::mechanics_observer::mechanics_string(&ctx),
                    ),
                    (
                        "callbacksState",
                        hw_core::callbacks_observer::callbacks_string(&ctx),
                    ),
                    ("levelState", hw_core::level_observer::level_string(&ctx)),
                ] {
                    assert_eq!(
                        row[field],
                        actual,
                        "{field}: character {character}, frame {}",
                        i + 1
                    );
                }
            }
        })
        .unwrap()
        .join()
        .unwrap();
}
#[test]
fn wheelchair() {
    check(1);
}
#[test]
fn segway() {
    check(2);
}
#[test]
fn bicycle() {
    check(3);
}
#[test]
fn motorcart() {
    check(4);
}

fn check_level_guards(buffer: bool) {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || {
            let root = format!("{}/../..", env!("CARGO_MANIFEST_DIR"));
            let version = if buffer { 2 } else { 1 };
            let trace = std::fs::read_to_string(format!(
                "{root}/reference/traces/flash32/level-callback-guards-45-v{version}.jsonl"
            ))
            .unwrap();
            let row: serde_json::Value =
                serde_json::from_str(trace.lines().next().unwrap()).unwrap();
            let expected = row["levelCallbackGuards"].as_str().unwrap();
            assert!(
                !expected.is_empty(),
                "probe must exercise registered level shapes"
            );
            let mut ctx = Ctx::<R>::new();
            assert_eq!(session::create(&mut ctx, 45, 1), Flow::Continue);
            let mut replay = ReplayState::new(1);
            assert_eq!(
                session::step(&mut ctx, &mut replay, [false; 8]),
                Flow::Continue
            );
            let mut def = ShapeDef::circle();
            def.is_sensor = hw_core::Flag32::TRUE;
            let sensor = shapes::create(&mut ctx, &def);
            let mut map = vec![hw_core::NONE];
            let mut body = ctx.world.body_list;
            while body != hw_core::NONE {
                let mut shape = ctx.world.body(body).shape_list;
                while shape != hw_core::NONE {
                    map.push(shape);
                    shape = ctx.world.shape(shape).next;
                }
                body = ctx.world.body(body).next;
            }
            let mut actual = String::new();
            for field in expected.split(';').filter(|f| !f.is_empty()) {
                ctx.world.shape_mut(sensor).is_sensor = hw_core::Flag32::TRUE;
                ctx.world.shape_mut(sensor).body = hw_core::NONE;
                let (index, _) = field.split_once('=').unwrap();
                let own = map[index.parse::<usize>().unwrap()];
                let point = ContactPoint {
                    shape1: own,
                    shape2: sensor,
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
                    other => panic!("unexpected level ADD result {other:?}"),
                };
                actual += &format!("{index}={error};");
                if buffer {
                    ctx.world.shape_mut(sensor).is_sensor = hw_core::Flag32::FALSE;
                    ctx.world.shape_mut(sensor).body = ctx.world.ground_body;
                    let mut point = point;
                    point.normal = Vec2::new(lit(1.0), lit(0.0));
                    point.velocity = Vec2::new(lit(6.0), lit(0.0));
                    assert_eq!(contact_listener::add(&mut ctx, &point), Flow::Continue);
                }
            }
            assert_eq!(expected, actual);
            assert_eq!(
                row["physicsState"],
                hw_core::physics_observer::physics_string(&ctx)
            );
            assert_eq!(
                row["callbacksState"],
                hw_core::callbacks_observer::callbacks_string(&ctx)
            );
            assert_eq!(
                row["levelState"],
                hw_core::level_observer::level_string(&ctx)
            );
            if buffer {
                assert_eq!(
                    row["levelAddState"],
                    hw_core::level_observer::level_add_string(&ctx)
                );
                for line in trace.lines().skip(1) {
                    let row: serde_json::Value = serde_json::from_str(line).unwrap();
                    assert_eq!(
                        session::step(&mut ctx, &mut replay, [false; 8]),
                        Flow::Continue
                    );
                    assert_eq!(
                        row["levelAddState"],
                        hw_core::level_observer::level_add_string(&ctx)
                    );
                    assert_eq!(
                        row["physicsState"],
                        hw_core::physics_observer::physics_string(&ctx)
                    );
                }
                assert_eq!(ctx.level.contact_adds.iter().filter(|v| (**v).get()).count(), 0);
            }
        })
        .unwrap()
        .join()
        .unwrap();
}

#[test]
fn level_add_sensor_guards() {
    check_level_guards(false);
}
#[test]
fn level_add_buffer() {
    check_level_guards(true);
}
