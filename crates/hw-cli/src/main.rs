//! CPU host commands. Formatting and allocation happen outside session::step.
mod bench;
mod diff;
mod inputs;
use hw_core::diagnostic;
use hw_core::game::session::{self, ReplayState};
use hw_core::{AbortSite, Ctx, Flow};
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;
type Result<T> = std::result::Result<T, String>;

const HELP: &str = "hw sim --level ID --char 1..4 --tas FILE_OR_BITS [--frames N] [--real f64]
       [--out FILE] [--dump-from N --dump-to N] [--broadphase] [--physics] [--game] [--links] [--mechanics] [--callbacks] [--level-state] [--level-add-state] [--listeners]
hw verify --trace FILE [--tas FILE_OR_BITS] [--level ID --char 1..4] [--real f64]
hw gen-inputs --level ID --char 1..4 --seed N --frames N
              --style idle|forward-jumpy|random|mash|death-seek [--out FILE]
hw bench [--real f64] [--threads N] [--cases all|official|generated] [--repeat N] [--out FILE]

TAS: eight bits/frame: left right up down space shift ctrl z.
sim defaults to TAS length + 60 frames; JSONL goes to stdout unless --out is set.
Arithmetic is native f64 (the only supported value of --real).
Build with --features portable-math to use the portable transcendental math.
Dump bounds are inclusive, one-based; hashes are emitted every frame.
verify reads the adjacent .metadata.json; official TAS inputs are resolved
from tas_sha256. For other Flash traces pass --tas when needed.
These commands check diagnostic-v1, not yet the exhaustive state contract.";

fn main() {
    let result = std::thread::Builder::new()
        .name("hw-cpu".into())
        .stack_size(64 << 20)
        .spawn(run)
        .map_err(|e| e.to_string())
        .and_then(|h| h.join().map_err(|_| "CPU worker panicked".into()))
        .and_then(|r| r);
    if let Err(error) = result {
        eprintln!("hw: {error}");
        std::process::exit(1);
    }
}
fn read_json(path: &str) -> Result<Value> {
    serde_json::from_reader(File::open(path).map_err(|e| format!("{path}: {e}"))?)
        .map_err(|e| format!("{path}: {e}"))
}
fn number(s: &str, name: &str) -> Result<u32> {
    s.parse()
        .map_err(|_| format!("{name} must be an unsigned 32-bit integer"))
}
fn read_tas(value: &str) -> Result<String> {
    let s = if value.bytes().all(|b| b == b'0' || b == b'1') {
        value.to_owned()
    } else {
        std::fs::read_to_string(value).map_err(|e| format!("TAS {value}: {e}"))?
    };
    // Probe tooling permits whitespace between complete input entries.
    let s: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if !s.bytes().all(|b| b == b'0' || b == b'1') || s.len() % 8 != 0 {
        return Err("TAS must contain only 0/1 bits in complete eight-bit frames".into());
    }
    if s.len() / 8 >= u32::MAX as usize {
        return Err("TAS is too long".into());
    }
    Ok(s)
}
fn resolve_tas(args: &BTreeMap<String, String>, meta: &Value) -> Result<String> {
    if let Some(tas) = args.get("--tas") {
        return read_tas(tas);
    }
    if let Some(bits) = meta["tas_bits"].as_str() {
        return read_tas(bits);
    }
    let index: Value = serde_json::from_str(include_str!("../../../reference/tas/index.json"))
        .map_err(|e| e.to_string())?;
    if let Some(sha) = meta["tas_sha256"].as_str() {
        for entry in index.as_array().ok_or("invalid embedded TAS index")? {
            if entry["sha256"] == sha {
                if let Some(tas) = hw_data::TAS_STRINGS
                    .iter()
                    .find(|t| entry["file"] == t.file)
                {
                    return Ok(tas.bits.into());
                }
            }
        }
        // Non-official strings (corrected TASes, search results) are read from disk; a wrong
        // file cannot pass, because verify compares every recorded input against the trace.
        let launches: Value = serde_json::from_str(include_str!("../../../reference/tas/launches.json"))
            .map_err(|e| e.to_string())?;
        for entry in launches.as_array().ok_or("invalid embedded launches index")? {
            if entry["sha256"] == sha {
                let file = entry["file"].as_str().ok_or("launches entry without file")?;
                return read_tas(&format!("reference/tas/{file}"));
            }
        }
    }
    Err("provide --tas FILE_OR_BITS (input could not be resolved from metadata)".into())
}
fn resolve_number(
    args: &BTreeMap<String, String>,
    meta: &Value,
    flag: &str,
    key: &str,
) -> Result<u32> {
    let saved = meta[key]
        .as_u64()
        .map(u32::try_from)
        .transpose()
        .map_err(|_| format!("metadata {key} is out of range"))?;
    if let Some(value) = args.get(flag) {
        let value = number(value, flag)?;
        if saved.is_some_and(|v| v != value) {
            return Err(format!("{flag} conflicts with trace metadata {key}"));
        }
        Ok(value)
    } else {
        saved.ok_or_else(|| format!("missing {flag}"))
    }
}
fn flash_error(flow: Flow) -> Result<u32> {
    match flow {
        Flow::Continue => Ok(0),
        Flow::Abort(AbortSite::PortCapacity | AbortSite::PortUnimplemented | AbortSite::None) => {
            Err(format!("port failure: {flow:?}; this is not a Flash error"))
        }
        Flow::Abort(AbortSite::B2AssertPolyCircle) => {
            Err("unmapped Flash assertion error; cannot claim trace equivalence".into())
        }
        Flow::Abort(_) => Ok(1009),
    }
}
fn record<R: hw_real::Real>(
    ctx: &mut Ctx<R>,
    replay: &mut ReplayState,
    bits: &str,
    dump: bool,
    broadphase: bool,
    physics: bool,
    game: bool,
    links: bool,
    mechanics: bool,
    callbacks: bool,
    level_state: bool,
    level_add_state: bool,
    listeners: bool,
) -> Result<Value> {
    let mut keys = [false; 8];
    for (k, key) in keys.iter_mut().enumerate() {
        *key = replay.input_allowed.get()
            && bits.as_bytes().get(replay.iteration as usize * 8 + k) == Some(&b'1');
    }
    let input: String = keys.iter().map(|k| if *k { '1' } else { '0' }).collect();
    let flow = session::step(ctx, replay, keys);
    let err = flash_error(flow).map_err(|e| {
        let w = &ctx.world;
        format!(
            "frame {}: {e}; arena free/pending/retired: bodies {}/{}/{}, shapes {}/{}/{}, joints {}/{}/{}, contacts {}/{}/{}",
            replay.frames,
            w.bodies.free_len, w.bodies.pending_len, w.bodies.retired_len,
            w.shapes.free_len, w.shapes.pending_len, w.shapes.retired_len,
            w.joints.free_len, w.joints.pending_len, w.joints.retired_len,
            w.contacts.free_len, w.contacts.pending_len, w.contacts.retired_len,
        )
    })?;
    let state = diagnostic::replay_string(ctx, replay);
    let mut rec = json!({"schema_version":1, "frame":replay.frames, "iteration":replay.iteration,
        "input":input, "err":err, "lock":ctx.world.lock.word(), "hash":diagnostic::hash(&state),
        "len":state.len(), "dead":ctx.character.dead.word(), "inputAllowed":replay.input_allowed.word(), "finish":replay.finish_frame});
    if broadphase {
        let bp = diagnostic::broadphase_string(ctx);
        rec["bpHash"] = diagnostic::hash(&bp).into();
        if dump {
            rec["bpState"] = bp.into();
        }
    }
    if physics {
        let ps = hw_core::physics_observer::physics_string(ctx);
        rec["physicsHash"] = diagnostic::hash(&ps).into();
        if dump {
            rec["physicsState"] = ps.into();
        }
    }
    if game {
        let gs = hw_core::game_observer::game_string(ctx);
        rec["gameHash"] = diagnostic::hash(&gs).into();
        if dump {
            rec["gameState"] = gs.into();
        }
    }
    if links {
        let ls = hw_core::links_observer::links_string(ctx);
        rec["linksHash"] = diagnostic::hash(&ls).into();
        if dump {
            rec["linksState"] = ls.into();
        }
    }
    if mechanics {
        let ms = hw_core::mechanics_observer::mechanics_string(ctx);
        rec["mechanicsHash"] = diagnostic::hash(&ms).into();
        if dump {
            rec["mechanicsState"] = ms.into();
        }
    }
    if callbacks {
        let cs = hw_core::callbacks_observer::callbacks_string(ctx);
        rec["callbacksHash"] = diagnostic::hash(&cs).into();
        if dump {
            rec["callbacksState"] = cs.into();
        }
    }
    if level_state {
        let ls = hw_core::level_observer::level_string(ctx);
        rec["levelHash"] = diagnostic::hash(&ls).into();
        if dump {
            rec["levelState"] = ls.into();
        }
    }
    if listeners {
        let ls = hw_core::listeners_observer::listeners_string(ctx);
        rec["listenersHash"] = diagnostic::hash(&ls).into();
        if dump {
            rec["listenersState"] = ls.into();
        }
    }
    if level_add_state {
        rec["levelAddState"] = hw_core::level_observer::level_add_string(ctx).into();
    }
    if dump {
        rec["state"] = state.into();
    }
    ctx.end_frame();
    Ok(rec)
}
fn run() -> Result<()> {
    let mut iter = std::env::args().skip(1);
    let command = iter.next().unwrap_or_else(|| "--help".into());
    if ["--help", "help", "-h"].contains(&command.as_str()) {
        println!("{HELP}");
        return Ok(());
    }
    if !["sim", "verify", "gen-inputs", "bench"].contains(&command.as_str()) {
        return Err(format!("unknown command {command}\n{HELP}"));
    }
    let mut args = BTreeMap::new();
    while let Some(flag) = iter.next() {
        if flag == "--help" {
            println!("{HELP}");
            return Ok(());
        }
        if ![
            "--level",
            "--char",
            "--tas",
            "--frames",
            "--out",
            "--dump-from",
            "--dump-to",
            "--broadphase",
            "--physics",
            "--game",
            "--links",
            "--mechanics",
            "--callbacks",
            "--level-state",
            "--level-add-state",
            "--listeners",
            "--trace",
            "--seed",
            "--style",
            "--real",
            "--threads",
            "--cases",
            "--repeat",
        ]
        .contains(&flag.as_str())
        {
            return Err(format!("unknown option {flag}"));
        }
        let value = if flag == "--broadphase"
            || flag == "--physics"
            || flag == "--game"
            || flag == "--links"
            || flag == "--mechanics"
            || flag == "--callbacks"
            || flag == "--level-state"
            || flag == "--level-add-state"
            || flag == "--listeners"
        {
            "true".into()
        } else {
            iter.next()
                .ok_or_else(|| format!("missing value for {flag}"))?
        };
        if args.insert(flag.clone(), value).is_some() {
            return Err(format!("duplicate option {flag}"));
        }
    }
    if command == "gen-inputs" {
        return inputs::run(&args);
    }
    if command == "bench" {
        return bench::run(&args);
    }
    for flag in ["--threads", "--cases", "--repeat"] {
        if args.contains_key(flag) {
            return Err(format!("{flag} is only valid for bench"));
        }
    }
    match args.get("--real").map(String::as_str).unwrap_or("f64") {
        "f64" => run_replay::<f64>(&command, &args, "f64"),
        _ => Err("--real must be f64".into()),
    }
}
fn run_replay<R: hw_real::Real>(command: &str, args: &BTreeMap<String, String>, real_name: &str) -> Result<()> {
    for flag in ["--seed", "--style"] {
        if args.contains_key(flag) {
            return Err(format!("{flag} is only valid for gen-inputs"));
        }
    }
    if command == "verify" {
        for flag in [
            "--frames",
            "--out",
            "--dump-from",
            "--dump-to",
            "--broadphase",
            "--physics",
            "--game",
            "--links",
            "--mechanics",
            "--callbacks",
            "--level-state",
            "--level-add-state",
            "--listeners",
        ] {
            if args.contains_key(flag) {
                return Err(format!("{flag} is only valid for sim"));
            }
        }
    } else if args.contains_key("--trace") {
        return Err("--trace is only valid for verify".into());
    }
    let trace = if command == "verify" {
        Some(args.get("--trace").ok_or("missing --trace")?)
    } else {
        None
    };
    let mut meta = Value::Null;
    if let Some(path) = trace {
        let sidecar = format!("{path}.metadata.json");
        if Path::new(&sidecar).exists() {
            meta = read_json(&sidecar)?;
        }
        for key in [
            "level_shapes_only",
            "level_only",
            "level_impulses",
            "level_actors",
            "base_character",
            "character_physics",
            "character_controls",
            "composites",
            "composite_breaks",
            "damage",
            "wheelchair_damage",
            "gears",
            "bicycle_damage",
            "motorcart_damage",
            "callback_guards",
            "level_callback_guards",
            "special_callback_guards",
        ] {
            if !meta[key].is_null() && meta[key] != false {
                return Err(format!("trace is an isolated {key} fixture; use its hw-core test instead of a full-session replay"));
            }
        }
    }
    let level = resolve_number(&args, &meta, "--level", "level")?;
    let character = resolve_number(&args, &meta, "--char", "character")?;
    if hw_data::level(level).is_none() {
        return Err(format!("unknown level {level}"));
    }
    if !(1..=4).contains(&character) {
        return Err("--char must be 1..4".into());
    }
    let tas = resolve_tas(&args, &meta)?;
    let tas_frames = (tas.len() / 8) as u32;
    if meta["tas_frames"]
        .as_u64()
        .is_some_and(|v| v != tas_frames as u64)
    {
        return Err("TAS frame count differs from metadata".into());
    }
    let mut ctx = Ctx::<R>::new();
    let created = session::create(&mut ctx, level, character);
    if created != Flow::Continue {
        return Err(format!("session creation failed: {created:?}"));
    }
    let mut replay = ReplayState::new(tas_frames);
    if let Some(path) = trace {
        let file = File::open(path).map_err(|e| format!("{path}: {e}"))?;
        let mut frames = 0u32;
        let observer_fields = [
            ("bpHash", "broadphase_diagnostics"),
            ("physicsHash", "physics_diagnostics"),
            ("gameHash", "game_diagnostics"),
            ("linksHash", "links_diagnostics"),
            ("mechanicsHash", "mechanics_diagnostics"),
            ("callbacksHash", "callbacks_diagnostics"),
            ("levelHash", "level_diagnostics"),
            ("levelAddState", "level_add_diagnostics"),
            ("listenersHash", "listeners_diagnostics"),
        ];
        let mut observer_scope = [false; 9];
        for line in BufReader::new(file).lines() {
            frames = frames.checked_add(1).ok_or("too many trace frames")?;
            let expected: Value = serde_json::from_str(&line.map_err(|e| e.to_string())?)
                .map_err(|e| format!("frame {frames}: invalid JSON: {e}"))?;
            if expected["schema_version"] != 1 || expected["frame"] != frames {
                return Err(format!("record {frames}: expected diagnostic schema_version 1 and consecutive frame number"));
            }
            for (i, (field, flag)) in observer_fields.iter().enumerate() {
                let present = expected[*field].is_string();
                if meta[*flag] == true && !present {
                    return Err(format!(
                        "frame {frames}: missing {field} required by metadata {flag}"
                    ));
                }
                if frames == 1 {
                    observer_scope[i] = present;
                } else if observer_scope[i] != present {
                    return Err(format!(
                        "frame {frames}: observer scope changed for {field}"
                    ));
                }
            }
            let actual = record(
                &mut ctx,
                &mut replay,
                &tas,
                expected["state"].is_string()
                    || expected["bpState"].is_string()
                    || expected["physicsState"].is_string()
                    || expected["gameState"].is_string()
                    || expected["linksState"].is_string()
                    || expected["mechanicsState"].is_string()
                    || expected["callbacksState"].is_string()
                    || expected["levelState"].is_string()
                    || expected["listenersState"].is_string(),
                expected["bpHash"].is_string(),
                expected["physicsHash"].is_string(),
                expected["gameHash"].is_string(),
                expected["linksHash"].is_string(),
                expected["mechanicsHash"].is_string(),
                expected["callbacksHash"].is_string(),
                expected["levelHash"].is_string(),
                expected["levelAddState"].is_string(),
                expected["listenersHash"].is_string(),
            )?;
            if let Some(detail) = diff::compare(&expected, &actual) {
                let from = frames.saturating_sub(2).max(1);
                let to = frames.saturating_add(2);
                let tas_arg = args.get("--tas").cloned().unwrap_or_else(|| {
                    hw_data::TAS_STRINGS
                        .iter()
                        .find(|t| t.bits == tas)
                        .map(|t| format!("reference/tas/{}", t.file))
                        .unwrap_or_else(|| "INPUT.tas".into())
                });
                return Err(format!("first divergence: frame {frames}, {detail}\nRecord a raw Flash window:\npython tools/gen-replay-probe.py --level {level} --char {character} --tas \"{tas_arg}\" --frames {} --run divergence-{frames} --dump-from {from} --dump-to {to}{}",meta["frames"].as_u64().unwrap_or(to as u64).max(to as u64),format!("{}{}{}{}{}", if expected["listenersHash"].is_string() {" --listeners-diagnostics"} else {""}, if expected["levelAddState"].is_string() {" --level-add-diagnostics"} else {""}, if expected["bpHash"].is_string() {" --broadphase-diagnostics"} else {""}, if expected["linksHash"].is_string() {" --links-diagnostics"} else if expected["gameHash"].is_string() {" --game-diagnostics"} else if expected["physicsHash"].is_string() {" --physics-diagnostics"} else {""}, if expected["levelHash"].is_string() {" --level-diagnostics"} else if expected["callbacksHash"].is_string() {" --callbacks-diagnostics"} else if expected["mechanicsHash"].is_string() {" --mechanics-diagnostics"} else {""})));
            }
        }
        if frames == 0 {
            return Err("trace is empty".into());
        }
        for key in ["frames", "records"] {
            if meta[key].as_u64().is_some_and(|n| n != frames as u64) {
                return Err(format!(
                    "truncated/extended trace: {frames} records, metadata {key}={}",
                    meta[key]
                ));
            }
        }
        println!(
            "MATCH diagnostic-v1: {frames} frames, level {level}, character {character}, finish {}",
            replay.finish_frame
        );
        return Ok(());
    }
    let frames = args
        .get("--frames")
        .map(|s| number(s, "--frames"))
        .transpose()?
        .unwrap_or(tas_frames.saturating_add(60));
    if frames == 0 {
        return Err("--frames must be positive".into());
    }
    let from = args
        .get("--dump-from")
        .map(|s| number(s, "--dump-from"))
        .transpose()?;
    let to = args
        .get("--dump-to")
        .map(|s| number(s, "--dump-to"))
        .transpose()?;
    let (from, to) = match (from, to) {
        (None, None) => (1, 0),
        (Some(a), Some(b)) if a > 0 && a <= b && b <= frames => (a, b),
        _ => return Err("provide both dump bounds with 1 <= from <= to <= frames".into()),
    };
    let mut output: Box<dyn Write> = if let Some(path) = args.get("--out") {
        Box::new(BufWriter::new(
            File::create_new(path).map_err(|e| format!("{path}: {e}"))?,
        ))
    } else {
        Box::new(BufWriter::new(std::io::stdout()))
    };
    for frame in 1..=frames {
        let rec = record(
            &mut ctx,
            &mut replay,
            &tas,
            frame >= from && frame <= to,
            args.contains_key("--broadphase"),
            args.contains_key("--physics")
                || args.contains_key("--game")
                || args.contains_key("--links")
                || args.contains_key("--mechanics")
                || args.contains_key("--callbacks")
                || args.contains_key("--level-state"),
            args.contains_key("--game")
                || args.contains_key("--links")
                || args.contains_key("--mechanics")
                || args.contains_key("--callbacks")
                || args.contains_key("--level-state"),
            args.contains_key("--links"),
            args.contains_key("--mechanics")
                || args.contains_key("--callbacks")
                || args.contains_key("--level-state"),
            args.contains_key("--callbacks") || args.contains_key("--level-state"),
            args.contains_key("--level-state"),
            args.contains_key("--level-add-state"),
            args.contains_key("--listeners"),
        )?;
        serde_json::to_writer(&mut output, &rec).map_err(|e| e.to_string())?;
        writeln!(output).map_err(|e| e.to_string())?;
    }
    output.flush().map_err(|e| e.to_string())?;
    if let Some(path) = args.get("--out") {
        let math = if cfg!(all(target_arch = "x86", target_os = "windows", not(feature = "portable-math"))) { "flash-x87" } else { "portable" };
        let metadata = json!({"schema_version":1,"source":"hw CPU diagnostic-v1","real":real_name,"math":math,"level":level,"character":character,"tas_frames":tas_frames,"tas_bits":tas,"frames":frames,"records":frames});
        let sidecar = format!("{path}.metadata.json");
        let file = File::create_new(&sidecar).map_err(|e| format!("{sidecar}: {e}"))?;
        serde_json::to_writer_pretty(file, &metadata).map_err(|e| e.to_string())?;
    }
    eprintln!(
        "simulated {frames} frames; finish {} (diagnostic-v1)",
        replay.finish_frame
    );
    Ok(())
}
