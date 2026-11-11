use std::process::{Command, Output};
fn hw(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_hw"))
        .args(args)
        .output()
        .unwrap()
}
fn success(output: Output) -> String {
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    String::from_utf8(output.stdout).unwrap()
}
#[cfg(feature = "full-corpus")]
#[test]
fn replay_roundtrip_and_first_field_failure() {
    let dir = std::env::temp_dir().join(format!("hw-cli-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("sim.jsonl");
    let path = path.to_str().unwrap();
    let input_path = dir.join("input.tas");
    std::fs::write(&input_path, "00101000\n00000000\n").unwrap();
    success(hw(&[
        "sim",
        "--level",
        "45",
        "--char",
        "3",
        "--tas",
        input_path.to_str().unwrap(),
        "--frames",
        "3",
        "--out",
        path,
        "--dump-from",
        "1",
        "--dump-to",
        "3",
        "--broadphase",
        "--game",
        "--links",
        "--mechanics",
        "--callbacks",
        "--level-state",
        "--level-add-state",
        "--listeners",
    ]));
    assert!(success(hw(&["verify", "--trace", path])).contains("MATCH diagnostic-v1: 3 frames"));
    assert!(success(hw(&["verify", "--trace", path, "--real", "soft"]))
        .contains("MATCH diagnostic-v1: 3 frames"));
    let text = std::fs::read_to_string(path).unwrap();
    let mut records: Vec<serde_json::Value> = text
        .lines()
        .map(|s| serde_json::from_str(s).unwrap())
        .collect();
    let state = records[1]["state"].as_str().unwrap().to_owned();
    let start = state.find('|').unwrap() + 1;
    let mut changed = state.into_bytes();
    changed[start + 15] = if changed[start + 15] == b'0' {
        b'1'
    } else {
        b'0'
    };
    records[1]["state"] = String::from_utf8(changed).unwrap().into();
    let changed = records.iter().map(|r| format!("{r}\n")).collect::<String>();
    std::fs::write(path, changed).unwrap();
    let result = hw(&["verify", "--trace", path]);
    assert!(!result.status.success());
    let message = String::from_utf8(result.stderr).unwrap();
    assert!(
        message.contains("first divergence: frame 2, bodies[0].sweep.c.x"),
        "{message}"
    );
    assert!(message.contains("--dump-from 1 --dump-to 4"));
    // A difference confined to a private dictionary must not pass verification.
    let mut records: Vec<serde_json::Value> = text
        .lines()
        .map(|s| serde_json::from_str(s).unwrap())
        .collect();
    let mut changed = records[1]["callbacksState"]
        .as_str()
        .unwrap()
        .as_bytes()
        .to_vec();
    let at = records[1]["callbacksState"]
        .as_str()
        .unwrap()
        .find("head1_shape.threshold=")
        .unwrap()
        + "head1_shape.threshold=".len()
        + 15;
    changed[at] = if changed[at] == b'0' { b'1' } else { b'0' };
    records[1]["callbacksState"] = String::from_utf8(changed).unwrap().into();
    std::fs::write(
        path,
        records.iter().map(|r| format!("{r}\n")).collect::<String>(),
    )
    .unwrap();
    let result = hw(&["verify", "--trace", path]);
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr).contains("character.head1_shape.threshold"));
    // A queued level ADD change must be detected even when every hash matches.
    let mut records: Vec<serde_json::Value> = text
        .lines()
        .map(|s| serde_json::from_str(s).unwrap())
        .collect();
    let state = records[1]["levelAddState"].as_str().unwrap();
    assert!(state.starts_with("count=0;"));
    records[1]["levelAddState"] = state.replacen("count=0;", "count=1;", 1).into();
    std::fs::write(
        path,
        records.iter().map(|r| format!("{r}\n")).collect::<String>(),
    )
    .unwrap();
    let result = hw(&["verify", "--trace", path]);
    assert!(!result.status.success());
    let message = String::from_utf8_lossy(&result.stderr);
    assert!(
        message.contains("first divergence: frame 2, count: Flash 1 vs Rust 0"),
        "{message}"
    );
    assert!(message.contains("--level-add-diagnostics"));
    let mut records: Vec<serde_json::Value> = text
        .lines()
        .map(|s| serde_json::from_str(s).unwrap())
        .collect();
    let state = records[1]["listenersState"].as_str().unwrap();
    let (field, rest) = state.split_once(';').unwrap();
    let (name, count) = field.split_once('=').unwrap();
    assert_eq!(name, "count.character.add");
    records[1]["listenersState"] =
        format!("{name}={};{rest}", count.parse::<u32>().unwrap() + 1).into();
    std::fs::write(
        path,
        records.iter().map(|r| format!("{r}\n")).collect::<String>(),
    )
    .unwrap();
    let result = hw(&["verify", "--trace", path]);
    assert!(!result.status.success());
    let message = String::from_utf8_lossy(&result.stderr);
    assert!(
        message.contains("first divergence: frame 2, count.character.add"),
        "{message}"
    );
    assert!(message.contains("--listeners-diagnostics"));
    // Removing an observer on one frame cannot silently weaken verification.
    let mut records: Vec<serde_json::Value> = text
        .lines()
        .map(|s| serde_json::from_str(s).unwrap())
        .collect();
    records[1].as_object_mut().unwrap().remove("levelAddState");
    std::fs::write(
        path,
        records.iter().map(|r| format!("{r}\n")).collect::<String>(),
    )
    .unwrap();
    let result = hw(&["verify", "--trace", path]);
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr)
        .contains("frame 2: observer scope changed for levelAddState"));
    // A matching prefix is not the complete trace promised by its sidecar.
    std::fs::write(path, format!("{}\n", text.lines().next().unwrap())).unwrap();
    let result = hw(&["verify", "--trace", path]);
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr).contains("truncated/extended trace"));
    std::fs::remove_dir_all(&dir).unwrap();
}

#[cfg(feature = "full-corpus")]

#[test]
fn arithmetic_selection_preserves_all_character_records() {
    for character in ["1", "2", "3", "4"] {
        let args = ["sim", "--level", "45", "--char", character, "--tas", "0010100000000000",
            "--frames", "3", "--dump-from", "1", "--dump-to", "3", "--broadphase",
            "--game", "--links", "--mechanics", "--callbacks", "--level-state",
            "--level-add-state", "--listeners"];
        let native = success(hw(&args));
        let mut soft_args = args.to_vec();
        soft_args.extend(["--real", "soft"]);
        assert_eq!(success(hw(&soft_args)), native, "character {character}");
    }
    let invalid = hw(&["sim", "--real", "float"]);
    assert!(!invalid.status.success());
    assert!(String::from_utf8_lossy(&invalid.stderr).contains("--real must be f64 or soft"));
}
#[cfg(feature = "full-corpus")]
#[test]
fn rejects_bad_inputs_and_fixture_modes() {
    assert!(
        !hw(&["sim", "--level", "45", "--char", "1", "--tas", "101"])
            .status
            .success()
    );
    assert!(
        !hw(&["sim", "--level", "45", "--char", "0", "--tas", "00000000"])
            .status
            .success()
    );
    assert!(
        !hw(&["sim", "--level", "999999", "--char", "1", "--tas", "00000000"])
            .status
            .success()
    );
    let path = format!(
        "{}/../../reference/traces/flash32/character-physics-4-v1.jsonl",
        env!("CARGO_MANIFEST_DIR")
    );
    let result = hw(&["verify", "--trace", &path]);
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr).contains("isolated character_physics fixture"));
    let path = format!(
        "{}/../../reference/traces/flash32/callback-guards-char1-v1.jsonl",
        env!("CARGO_MANIFEST_DIR")
    );
    let result = hw(&["verify", "--trace", &path]);
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr).contains("isolated callback_guards fixture"));
    let path = format!(
        "{}/../../reference/traces/flash32/level-callback-guards-45-v2.jsonl",
        env!("CARGO_MANIFEST_DIR")
    );
    let result = hw(&["verify", "--trace", &path]);
    assert!(!result.status.success());
    assert!(
        String::from_utf8_lossy(&result.stderr).contains("isolated level_callback_guards fixture")
    );
    let path = format!(
        "{}/../../reference/traces/flash32/special-callback-guards-3214-v1.jsonl",
        env!("CARGO_MANIFEST_DIR")
    );
    let result = hw(&["verify", "--trace", &path]);
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr)
        .contains("isolated special_callback_guards fixture"));
}
#[cfg(feature = "full-corpus")]
#[test]
fn private_fields_and_retained_roots_match_flash() {
    let root = format!("{}/../..", env!("CARGO_MANIFEST_DIR"));
    for character in 1..=4 {
        success(hw(&[
            "verify",
            "--trace",
            &format!("{root}/reference/traces/flash32/game-observer-char{character}-v1.jsonl"),
            "--tas",
            &format!("{root}/reference/tas/generated/wheelchair-controls-v1.tas"),
        ]));
    }
}
#[test]
fn coverage_inputs_are_reproducible_and_frame_aligned() {
    let args = [
        "gen-inputs",
        "--level",
        "45",
        "--char",
        "3",
        "--seed",
        "12345",
        "--frames",
        "256",
        "--style",
        "random",
    ];
    let first = success(hw(&args));
    assert_eq!(first, success(hw(&args)));
    assert_eq!(first.lines().count(), 256);
    assert!(first
        .lines()
        .all(|s| s.len() == 8 && s.bytes().all(|b| b == b'0' || b == b'1')));
    let mut idle = args;
    idle[10] = "idle";
    assert!(success(hw(&idle)).lines().all(|s| s == "00000000"));
    let mut other = args;
    other[6] = "12346";
    assert_ne!(first, success(hw(&other)));
}
#[cfg(feature = "full-corpus")]
#[test]
fn composite_and_reference_observers_match_flash() {
    let root = format!("{}/../..", env!("CARGO_MANIFEST_DIR"));
    success(hw(&[
        "verify",
        "--trace",
        &format!("{root}/reference/traces/flash32/mechanics-observer-bicycle-v1.jsonl"),
        "--tas",
        &format!("{root}/reference/tas/generated/wheelchair-controls-v1.tas"),
    ]));
}
#[cfg(feature = "full-corpus")]
#[test]
fn contact_dictionaries_match_flash_for_each_character() {
    let root = format!("{}/../..", env!("CARGO_MANIFEST_DIR"));
    for character in 1..=4 {
        success(hw(&[
            "verify",
            "--trace",
            &format!("{root}/reference/traces/flash32/callbacks-observer-char{character}-v1.jsonl"),
            "--tas",
            &format!("{root}/reference/tas/generated/wheelchair-controls-v1.tas"),
        ]));
    }
    success(hw(&[
        "verify",
        "--trace",
        &format!("{root}/reference/traces/flash32/callbacks-gbe-error-v1.jsonl"),
    ]));
}
#[cfg(feature = "full-corpus")]
#[test]
fn level_actions_and_specials_match_flash_on_all_levels() {
    let root = format!("{}/../..", env!("CARGO_MANIFEST_DIR"));
    for level in [22, 45, 1144, 2433, 3214, 10251, 13098, 13196, 13812] {
        success(hw(&[
            "verify",
            "--trace",
            &format!("{root}/reference/traces/flash32/level-observer-{level}-v1.jsonl"),
            "--tas",
            &format!("{root}/reference/tas/generated/wheelchair-controls-v1.tas"),
        ]));
    }
}

#[cfg(feature = "full-corpus")]

#[test]
fn character_listener_identities_match_flash() {
    let root = format!("{}/../..", env!("CARGO_MANIFEST_DIR"));
    for character in 1..=4 {
        success(hw(&[
            "verify",
            "--trace",
            &format!("{root}/reference/traces/flash32/listeners-char{character}-v1.jsonl"),
            "--tas",
            "00000000",
        ]));
    }
}
