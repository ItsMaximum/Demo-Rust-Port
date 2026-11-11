//! obby-search: Obstacle Course (level 45) / Irresponsible Dad (character 3) QE + pairmax search.
//!
//! Goal: a finish by frame 299 (under 10 s). The start area is covered by a spike ceiling
//! (special 12, y 66.6 m, x 0..24.2 m); measured with the cannon, a chest launch from below it
//! never reaches the finish, while from above it about 1.6e-4 of random launches do (left wall /
//! top border bounces, then down onto the finish shape #120). A pair-pool exhaustion (pairmax)
//! throws inside b2World.Solve after every body has been integrated and before SolveTOI, so on
//! that frame bodies move without continuous collision: the chest can pass the spikes.
//!
//! Commands:
//!   stats   --tas F                       per-frame pairs/proxies/site/chest/speed
//!   explore --run-dir D [--seeds a,b] ... Go-Explore toward pairmax and chest-above-ceiling states
mod climb;
mod climb2;
mod onarch;
mod hold;
mod mlaunch;
mod cqe;
mod flight;
mod onset;
mod osearch;
mod qe;
mod sim;
mod tune;
mod window;

use sim::{read_tas, tas_text, Fr, Sim};
use std::collections::HashMap;
use std::io::Write;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

pub const LEVEL: u32 = 45;
pub const CHAR: u32 = 3;
/// Spike ceiling underside (special 12 at y 66.6 m, 70 px thick): a living chest centre above
/// this line in the start area has passed it.
const CEILING_Y: f64 = 66.0;
const CEILING_X1: f64 = 24.5;

struct Args(HashMap<String, String>);
impl Args {
    fn parse() -> (String, Args) {
        let mut it = std::env::args().skip(1);
        let cmd = it.next().unwrap_or_default();
        let mut m = HashMap::new();
        let mut it = it.peekable();
        while let Some(f) = it.next() {
            if let Some((k, v)) = f.split_once('=') {
                m.insert(k.to_string(), v.to_string());
                continue;
            }
            let v = match it.peek() {
                Some(v) if !v.starts_with("--") => it.next().unwrap(),
                _ => "true".into(),
            };
            m.insert(f, v);
        }
        (cmd, Args(m))
    }
    fn get(&self, k: &str) -> Option<&str> {
        self.0.get(k).map(String::as_str)
    }
    fn num<T: std::str::FromStr>(&self, k: &str, d: T) -> T {
        self.get(k).and_then(|v| v.parse().ok()).unwrap_or(d)
    }
}

const COMMANDS: &[(&str, &str)] = &[
    ("stats", "per-frame pairs, proxies, throw site, chest and speeds for a TAS (--tas F)"),
    ("trace", "Dad's upper body, neck and finish state (--tas F [--from N --frames M --tail K])"),
    ("explore", "Go-Explore toward pairmax and chest-above-ceiling states"),
    ("window", "local search on lineage ends (--objective early|up|upa|upc|ups|upv|upr|upc2|upx|fast|lq|apex|osky|osky3|ostate|aft|earlykick|finish|pass|passfin|pairs)"),
    ("onset", "re-roll inputs before a pairmax onset, follow the held lock, branch Space releases in open sky"),
    ("onarch", "Go-Explore archive over onsets (--pass scores ceiling passes; needs --flight-map file:base,...)"),
    ("cqe", "archive search for a child-arm QE at the start with Dad healthy (--mines mask)"),
    ("mlaunch", "back Dad into the start mines and score the in-frame pair peak of the launch"),
    ("hold", "sweep held inputs and hold lengths after each onset (--files F), to frame 299"),
    ("tune", "mutate the frames before a launch and play to frame 299 (OBBY_COMBINED=1 for pairmax-first)"),
    ("climb", "coordinate ascent on single inputs for the in-frame pair peak"),
    ("climb2", "coordinate ascent on single inputs for the held-lock open-sky score"),
    ("flight", "teleport Dad's component into open sky with a sampled velocity and play to frame 299 (an experiment, not an input)"),
    ("branch", "random continuations of a lineage"),
    ("aftermath", "onset statistics: kick size, lock length, state after the lock"),
    ("release", "release-timing statistics for held locks"),
    ("lockctl", "how post-onset inputs change lock length"),
    ("driftcheck", "ballistic drift of onset kicks against the level"),
    ("qescan", "when torn-ligament anchors first whip faster than 1e3..1e5 m/s (--files F)"),
    ("lstate", "state at the end of each lineage (--files F)"),
    ("pairmap", "overlap counts per proxy at a frame (--tas F --frame N)"),
    ("blast", "experiment: apply a mine-style blast at a frame and point"),
    ("inject", "experiment: displace the chest at a frame"),
    ("sens", "single-input sensitivity before an onset"),
    ("launches", "first launch of each lineage"),
    ("nearceil", "lineages that approach the spike ceiling"),
    ("info", "break limits and masses"),
    ("geom", "shapes in a box (--x0 --y0 --x1 --y1)"),
];

fn print_help() {
    println!("obby-search: Obstacle Course (level 45) / Irresponsible Dad (character 3) QE and pairmax search
");
    println!("usage: obby-search <command> [options]
");
    for (name, summary) in COMMANDS {
        println!("  {name:<11} {summary}");
    }
    println!("
Common options: --run-dir DIR  --threads N  --hours H  --seed S  --seed-file F.jsonl  --window N  --max-frame N");
    println!("Seed files are JSON lines with a \"tas\" field of 0/1 bits (8 per frame). A STOP file in the run directory ends a search.");
    println!("Level and character are fixed to 45 and 3; use `hw-search` for other levels. See docs/OBSTACLE_COURSE.md.");
}

fn main() {
    let (cmd, args) = Args::parse();
    if matches!(cmd.as_str(), "" | "help" | "--help" | "-h") {
        print_help();
        return;
    }
    if let Some(spec) = args.get("--flight-map") {
        if let Err(e) = onarch::set_flight_maps(spec) {
            eprintln!("obby-search: {e}");
            std::process::exit(2);
        }
    }
    let r = std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || match cmd.as_str() {
            "stats" => stats(&args),
            "explore" => explore(&args),
            "inject" => inject(&args),
            "blast" => blast(&args),
            "cqe" => {
                let mut seeds = Vec::new();
                if let Some(p) = args.get("--seed-file") {
                    for line in std::fs::read_to_string(p).map_err(|e| e.to_string())?.lines() {
                        if let Some(m) = tas_field(line) {
                            seeds.push(m);
                        }
                    }
                }
                cqe::run(cqe::Config {
                    run_dir: args.get("--run-dir").ok_or("missing --run-dir")?.into(),
                    threads: args.num("--threads", 7),
                    hours: args.num("--hours", 2.0),
                    seed: args.num("--seed", 1),
                    max_frame: args.num("--max-frame", 100),
                    grow: args.num("--grow", 20),
                    temp: args.num("--temp", 20.0),
                    x_lo: args.num("--x-lo", 1.0),
                    x_hi: args.num("--x-hi", 6.0),
                    mines: args.num("--mines", 12),
                    seeds,
                })
            }
            "mlaunch" => {
                let mut seeds = Vec::new();
                if let Some(p) = args.get("--seed-file") {
                    for line in std::fs::read_to_string(p).map_err(|e| e.to_string())?.lines() {
                        if let Some(m) = tas_field(line) {
                            seeds.push(m);
                        }
                    }
                }
                mlaunch::run(mlaunch::Config {
                    run_dir: args.get("--run-dir").ok_or("missing --run-dir")?.into(),
                    threads: args.num("--threads", 7),
                    hours: args.num("--hours", 2.0),
                    seed: args.num("--seed", 1),
                    slack: args.num("--slack", 10),
                    approach: args.num("--approach", 40),
                    after: args.num("--after", 16),
                    pool: args.num("--pool", 3000),
                    temp: args.num("--temp", 150.0),
                    seeds,
                })
            }
            "hold" => {
                let files: Vec<String> = args.get("--files").unwrap_or("").split(',').filter(|x| !x.is_empty()).map(|x| x.to_string()).collect();
                hold::hold_cmd(&files, args.num("--max-hold", 20), args.num("--threads", 4), args.get("--out").unwrap_or("hold-out"))
            }
            "onarch" => {
                let mut seeds = Vec::new();
                for p in args.get("--seed-file").unwrap_or("").split(',').filter(|x| !x.is_empty()) {
                    for line in std::fs::read_to_string(p).map_err(|e| e.to_string())?.lines() {
                        if let Some(m) = tas_field(line) {
                            seeds.push(m);
                        }
                    }
                }
                if seeds.is_empty() {
                    return Err("onarch needs --seed-file".into());
                }
                sim::set_hang_ms(args.num("--hang-ms", 400));
                onarch::run(onarch::Config {
                    run_dir: args.get("--run-dir").ok_or("missing --run-dir")?.into(),
                    threads: args.num("--threads", 7),
                    hours: args.num("--hours", 4.0),
                    seed: args.num("--seed", 1),
                    window: args.num("--window", 20),
                    extra: args.num("--extra", 10),
                    max_frame: args.num("--max-frame", 230),
                    temp: args.num("--temp", 5.0),
                    sweep: args.num("--sweep", 15.0),
                    pass: args.get("--pass").is_some(),
                    seeds,
                })
            }
            "lstate" => {
                let files: Vec<String> = args.get("--files").unwrap_or("").split(',').filter(|x| !x.is_empty()).map(|x| x.to_string()).collect();
                qe::lstate_cmd(&files, args.num("--threads", 4))
            }
            "climb2" => climb2::climb2_cmd(
                args.get("--tas").ok_or("missing --tas")?,
                args.num("--from", 80),
                args.num("--to", 110),
                args.num("--max-frame", 130),
                args.num("--threads", 7),
                args.get("--out").unwrap_or("climb2.tas"),
            ),
            "qescan" => {
                let files: Vec<String> = args.get("--files").unwrap_or("").split(',').filter(|x| !x.is_empty()).map(|x| x.to_string()).collect();
                qe::qescan_cmd(&files, args.num("--frames", 120), args.num("--threads", 12))
            }
            "release" => {
                let files: Vec<String> = args.get("--files").unwrap_or("").split(',').filter(|x| !x.is_empty()).map(String::from).collect();
                sim::set_hang_ms(args.num("--hang-ms", 400));
                let hold = args.get("--hold").and_then(|v| v.parse().ok());
                onset::release_cmd(&files, hold, args.num("--release", 0), args.num("--span", 70), args.num("--k", 30), args.num("--threads", 14))
            }
            "trace" => onset::trace_cmd(args.get("--tas").ok_or("missing --tas")?, args.num("--tail", 0), args.num("--frames", 300), args.num("--from", 0)),
            "onset" => {
                let mut seeds = Vec::new();
                for p in args.get("--seed-file").unwrap_or("").split(',').filter(|x| !x.is_empty()) {
                    for line in std::fs::read_to_string(p).map_err(|e| e.to_string())?.lines() {
                        if let Some(m) = tas_field(line) {
                            seeds.push(m);
                        }
                    }
                }
                sim::set_hang_ms(args.num("--hang-ms", 400));
                osearch::run(osearch::Config {
                    run_dir: args.get("--run-dir").ok_or("missing --run-dir")?.into(),
                    threads: args.num("--threads", 14),
                    hours: args.num("--hours", 1.0),
                    seed: args.num("--seed", 1),
                    max_frame: args.num("--max-frame", 299),
                    window: args.num("--window", 12),
                    extra: args.num("--extra", 8),
                    lock_span: args.num("--lock-span", 60),
                    batch: args.num("--batch", 16),
                    pool: args.num("--pool", 3000),
                    temp: args.num("--temp", 30.0),
                    release: args.num("--release", 0),
                    seeds,
                })
            }
            "sens" => onset::sens_cmd(args.get("--tas").ok_or("missing --tas")?, args.num("--from", 150), args.num("--threads", 14)),
            "branch" => onset::branch_cmd(args.get("--tas").ok_or("missing --tas")?, args.num("--n", 1000), args.num("--depth", 30), args.num("--threads", 14), args.num("--seed", 1)),
            "geom" => onset::geom_cmd(args.num("--x0", 90.0), args.num("--y0", 40.0), args.num("--x1", 130.0), args.num("--y1", 65.0)),
            "flight" => {
                let pair = |k: &str, d: [f64; 2]| -> [f64; 2] {
                    args.get(k).and_then(|v| {
                        let mut it = v.split(',').filter_map(|x| x.parse::<f64>().ok());
                        Some([it.next()?, it.next()?])
                    }).unwrap_or(d)
                };
                flight::flight_cmd(
                    args.get("--tas").ok_or("missing --tas")?,
                    args.num("--frame", 255),
                    args.num("--n", 10000),
                    args.num("--threads", 14),
                    args.num("--seed", 1),
                    flight::Region { x: pair("--x", [0.0, 60.0]), y: pair("--y", [20.0, 64.0]), speed: pair("--speed", [10.0, 200.0]), angle: pair("--angle", [-90.0, 90.0]) },
                )
            }
            "driftcheck" => {
                let files: Vec<String> = args.get("--files").unwrap_or("").split(',').filter(|x| !x.is_empty()).map(String::from).collect();
                sim::set_hang_ms(args.num("--hang-ms", 400));
                flight::driftcheck_cmd(&files, args.get("--base").ok_or("missing --base")?, args.num("--threads", 14))
            }
            "info" => onset::info_cmd(),
            "lockctl" => onset::lockctl_cmd(args.get("--tas").ok_or("missing --tas")?, args.num("--n", 500), args.num("--k", 10), args.num("--hold", 16), args.num("--threads", 6)),
            "pairmap" => onset::pairmap_cmd(args.get("--tas").ok_or("missing --tas")?, args.num("--frame", 86)),
            "launches" => {
                let files: Vec<String> = args.get("--files").unwrap_or("").split(',').filter(|x| !x.is_empty()).map(String::from).collect();
                onset::launches_cmd(&files, args.num("--threads", 4))
            }
            "nearceil" => {
                let files: Vec<String> = args.get("--files").unwrap_or("").split(',').filter(|x| !x.is_empty()).map(String::from).collect();
                onset::nearceil_cmd(&files, args.num("--threads", 2))
            }
            "tune" => {
                let mut seeds = Vec::new();
                let cut: usize = args.num("--cut", 0);
                for p in args.get("--seed-file").unwrap_or("").split(',').filter(|x| !x.is_empty()) {
                    for line in std::fs::read_to_string(p).map_err(|e| e.to_string())?.lines() {
                        if let Some(mut m) = tas_field(line) {
                            if cut > 0 && m.len() > cut {
                                m.truncate(cut);
                            }
                            seeds.push(m);
                        }
                    }
                }
                seeds.sort();
                seeds.dedup();
                sim::set_hang_ms(args.num("--hang-ms", 400));
                tune::run(tune::Config {
                    run_dir: args.get("--run-dir").ok_or("missing --run-dir")?.into(),
                    threads: args.num("--threads", 7),
                    hours: args.num("--hours", 2.0),
                    seed: args.num("--seed", 1),
                    window: args.num("--window", 30),
                    tail: args.num("--tail", 0),
                    pool: args.num("--pool", 400),
                    temp: args.num("--temp", 3.0),
                    seeds,
                })
            }
            "climb" => {
                sim::set_hang_ms(args.num("--hang-ms", 400));
                climb::climb_cmd(
                    args.get("--tas").ok_or("missing --tas")?,
                    args.num("--from", 165),
                    args.num("--to", 176),
                    args.num("--extra", 4),
                    args.get("--zone").unwrap_or("corridor"),
                    args.num("--threads", 7),
                    args.get("--out").unwrap_or("runs/climb-best.tas"),
                )
            }
            "aftermath" => {
                let files: Vec<String> = args.get("--files").unwrap_or("").split(',').filter(|x| !x.is_empty()).map(String::from).collect();
                sim::set_hang_ms(args.num("--hang-ms", 400));
                onset::aftermath(&files, args.num("--horizon", 40), args.num("--threads", 14), args.get("--tail-zero").is_some())
            }
            "window" => {
                let mut seeds = Vec::new();
                for f in args.get("--seeds").unwrap_or("").split(',').filter(|x| !x.is_empty()) {
                    match read_tas(f) {
                        Ok(m) => seeds.push(m),
                        Err(e) => return Err(e),
                    }
                }
                if let Some(p) = args.get("--seed-file") {
                    // jsonl rows with a "tas" field (archive/pool/events).
                    for line in std::fs::read_to_string(p).map_err(|e| e.to_string())?.lines() {
                        if let Some(m) = tas_field(line) {
                            seeds.push(m);
                        }
                    }
                }
                sim::set_hang_ms(args.num("--hang-ms", 400));
                window::run(window::Config {
                    run_dir: args.get("--run-dir").ok_or("missing --run-dir")?.into(),
                    threads: args.num("--threads", 14),
                    hours: args.num("--hours", 1.0),
                    seed: args.num("--seed", 1),
                    max_frame: args.num("--max-frame", 299),
                    window: args.num("--window", 24),
                    extra: args.num("--extra", 10),
                    batch: args.num("--batch", 24),
                    pool: args.num("--pool", 2000),
                    temp: args.num("--temp", 300.0),
                    seeds,
                    objective: args.get("--objective").unwrap_or("pairs").to_string(),
                })
            }
            _ => Err(format!("unknown command {cmd}; run `obby-search help`")),
        })
        .unwrap()
        .join()
        .unwrap();
    if let Err(e) = r {
        eprintln!("obby-search: {e}");
        std::process::exit(2);
    }
}

/// Owner labels for Dad's, the bicycle's and the child's bodies.
pub(crate) fn owners(ctx: &sim::State) -> HashMap<u32, String> {
    let mut owner: HashMap<u32, String> = HashMap::new();
    let p = &ctx.character.primary;
    for (i, &b) in p.bodies.iter().enumerate() {
        owner.insert(b as u32, format!("dad{i}"));
    }
    for (i, &b) in p.bicycle.bodies.iter().enumerate() {
        owner.insert(b as u32, format!("bike{i}"));
    }
    for (i, &b) in ctx.character.child.bodies.iter().enumerate() {
        owner.insert(b as u32, format!("child{i}"));
    }
    owner.insert(ctx.character.child.child_seat.seat as u32, "seat".into());
    owner
}

/// Dynamic bodies faster than 300 m/s with an owner label.
fn fast_bodies(ctx: &sim::State) -> Vec<(String, u32, f64)> {
    use hw_core::NONE;
    let mut owner: HashMap<u32, String> = HashMap::new();
    let p = &ctx.character.primary;
    for (i, &b) in p.bodies.iter().enumerate() {
        owner.insert(b as u32, format!("dad{i}"));
    }
    for (i, &b) in p.bicycle.bodies.iter().enumerate() {
        owner.insert(b as u32, format!("bike{i}"));
    }
    for (i, &b) in ctx.character.child.bodies.iter().enumerate() {
        owner.insert(b as u32, format!("child{i}"));
    }
    owner.insert(ctx.character.child.child_seat.seat as u32, "seat".into());
    let w = &ctx.world;
    let mut out = Vec::new();
    let mut b = w.body_list;
    let mut guard = 0;
    while b != NONE && guard < hw_core::b2::state::MAX_BODIES {
        guard += 1;
        let bd = w.body(b);
        if bd.inv_mass > 0.0 {
            let v = bd.linear_velocity;
            let sp = (v.x * v.x + v.y * v.y).sqrt();
            if sp > 300.0 {
                out.push((owner.get(&(b as u32)).cloned().unwrap_or_else(|| "other".into()), b as u32, sp));
            }
        }
        b = bd.next;
    }
    out
}

/// The input string in a JSON line's "tas" field (any spacing).
fn tas_field(line: &str) -> Option<Vec<u8>> {
    let i = line.find("\"tas\"")?;
    let rest = &line[i + 5..];
    let q = rest.find('"')?;
    let t = &rest[q + 1..];
    let j = t.find('"')?;
    sim::parse_tas(&t[..j]).ok()
}

/// Replay --tas to frame F-1, then on frame F overwrite the chest's end-of-solve displacement and
/// velocity (lab injection), and keep playing the string; prints each frame.
fn inject(args: &Args) -> Result<(), String> {
    let masks = read_tas(args.get("--tas").ok_or("missing --tas")?)?;
    let frame: u32 = args.num("--frame", 0);
    let vx: f64 = args.num("--vx", 0.0);
    let vy: f64 = args.num("--vy", 0.0);
    let dx: f64 = args.num("--dx", vx / 30.0);
    let dy: f64 = args.num("--dy", vy / 30.0);
    let after: u32 = args.num("--after", 20);
    let sky = sim::Skyline::new(&Sim::new(LEVEL, CHAR)?.ctx);
    let mut s = Sim::new(LEVEL, CHAR)?;
    s.masks = masks;
    while s.frame() + 1 < frame {
        s.step(0);
    }
    for k in 0..=after {
        let chest = sim::chest_body(&s.ctx);
        if k == 0 {
            hw_core::lab::arm(&[chest]);
            hw_core::lab::inject(hw_core::lab::Injection { body: chest, origin: None, displacement: [dx, dy], velocity: [vx, vy], angular: None, angle_delta: None });
        }
        let f = s.step(0);
        if k == 0 {
            hw_core::lab::disarm();
            hw_core::lab::take_events();
        }
        println!(
            "frame {} site {} pairs {} alive {} chest [{:.2},{:.2}] cv [{:.1},{:.1}] above {} finish {}",
            f.frame, f.site, f.pairs, f.alive_after, f.chest_after[0], f.chest_after[1], f.chest_v[0], f.chest_v[1], sky.above(f.chest_after), s.replay.finish_frame
        );
    }
    Ok(())
}

/// `blast` (experiment, not an input): at the start of frame `--frame`, apply a mine-style blast
/// (impulse 10 * (1 - d / 2) radially to up to `--count` shapes within 2 m of (`--x`, `--y`)),
/// then report each frame's pairs (in-frame peak in the instrumented build) for `--after` frames.
fn blast(args: &Args) -> Result<(), String> {
    use hw_core::b2::{body, world};
    use hw_core::b2::math::Vec2;
    let masks = read_tas(args.get("--tas").ok_or("missing --tas")?)?;
    let frame: u32 = args.num("--frame", 85);
    let (x, y): (f64, f64) = (args.num("--x", 0.4), args.num("--y", 83.2));
    let count: i32 = args.num("--count", 30);
    let after: u32 = args.num("--after", 12);
    let strength: f64 = args.num("--strength", 10.0);
    let mut s = Sim::new(LEVEL, CHAR)?;
    s.masks = masks;
    while s.frame() + 1 < frame {
        s.step(0);
    }
    let radius = 2.0;
    let mut results = [hw_core::NONE; 64];
    let (_, n) = world::query(&mut s.ctx, Vec2::new(x - radius, y - radius), Vec2::new(x + radius, y + radius), &mut results[..], count);
    let mut hit = 0;
    for k in 0..n as usize {
        let b = s.ctx.world.shape(results[k]).body;
        if !body::is_static(&s.ctx, b) {
            let p = body::get_world_center(&s.ctx, b);
            let d = Vec2::new(p.x - x, p.y - y);
            let len = (d.x * d.x + d.y * d.y).sqrt().min(radius);
            let factor = 1.0 - len / radius;
            let a = d.y.atan2(d.x);
            body::apply_impulse(&mut s.ctx, b, Vec2::new(a.cos() * factor * strength, a.sin() * factor * strength), p);
            hit += 1;
        }
    }
    println!("blast at frame {} ({:.2},{:.2}): {} shapes found, {} impulses", frame, x, y, n, hit);
    for _ in 0..after {
        let f = s.step(0);
        println!("frame {} site {} pairs {} proxies {} alive {} chest [{:.2},{:.2}] cv [{:.1},{:.1}] vmax {:.0}", f.frame, f.site, f.pairs, f.proxies, f.alive_after, f.chest_after[0], f.chest_after[1], f.chest_v[0], f.chest_v[1], f.speed_max);
    }
    Ok(())
}

fn stats(args: &Args) -> Result<(), String> {
    let masks = read_tas(args.get("--tas").ok_or("missing --tas")?)?;
    let frames: u32 = args.num("--frames", masks.len() as u32 + 30);
    let mut s = Sim::new(LEVEL, CHAR)?;
    s.masks = masks;
    let detail = args.get("--detail").is_some();
    for _ in 0..frames {
        let f = s.step(0);
        if detail && f.pairs > args.num("--detail-pairs", 1500) {
            for (who, b, sp) in fast_bodies(&s.ctx) {
                eprintln!("  f{} {} body#{} speed {:.3e}", f.frame, who, b, sp);
            }
        }
        println!(
            "{{\"frame\":{},\"mask\":{},\"site\":{},\"pairs\":{},\"proxies\":{},\"lock\":{},\"alive\":{},\"finish\":{},\"chest\":[{:.3},{:.3}],\"cv\":[{:.1},{:.1}],\"speed_max\":{:.1},\"fast\":{},\"dej\":{},\"cej\":{},\"us\":{}}}",
            f.frame, f.mask, f.site, f.pairs, f.proxies, f.lock, f.alive_after, s.replay.finish_frame, f.chest_after[0], f.chest_after[1],
            f.chest_v[0], f.chest_v[1], f.speed_max, f.fast_bodies, f.dad_ejected, f.child_ejected, f.step_us
        );
    }
    Ok(())
}

// ---------------------------------------------------------------------------------------------
// Go-Explore toward pairmax.

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n.max(1)
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
struct Key {
    fb: u8,
    dej: bool,
    cej: bool,
    cx: i16,
    cy: i16,
    sp: u8,
    pb: u8,
}

fn speed_bucket(s: f64) -> u8 {
    if s < 1.0 { 0 } else { (s.log2().floor() as i64).clamp(0, 30) as u8 }
}

fn key(f: &Fr) -> Key {
    let c = f.chest_after;
    Key {
        fb: (f.frame / 15).min(255) as u8,
        dej: f.dad_ejected,
        cej: f.child_ejected,
        cx: if c[0].is_finite() { (c[0] / 5.0).floor() as i16 } else { -999 },
        cy: if c[1].is_finite() { (c[1] / 5.0).floor() as i16 } else { -999 },
        sp: speed_bucket(f.speed_max),
        pb: (f.pairs / 250).clamp(0, 20) as u8,
    }
}

/// Selection value of a cell: pair pressure first, then raw QE speed, then height in the start area.
fn value(k: &Key, above: bool) -> f64 {
    k.pb as f64 * 3.0 + (k.sp.min(30) as f64) * 0.3 + if above { 30.0 } else { 0.0 }
}

struct Cell {
    masks: Vec<u8>,
    value: f64,
    visits: u64,
}

struct Shared {
    cells: Mutex<HashMap<Key, Cell>>,
    stop: AtomicBool,
    frames: AtomicU64,
    rollouts: AtomicU64,
    pairmax: AtomicU64,
    above: AtomicU64,
    best_pairs: AtomicU64,
    best_speed: Mutex<f64>,
    events: Mutex<std::fs::File>,
    winner: Mutex<Option<Vec<u8>>>,
}

fn post_eject_mask(rng: &mut Rng) -> u8 {
    const POSE: [u8; 5] = [0, 1, 2, 4, 8];
    let mut m = POSE[rng.below(5) as usize];
    if rng.below(10) < 8 {
        m |= 16;
    }
    m
}

fn mounted_mask(rng: &mut Rng) -> u8 {
    const A: [u8; 12] = [4, 4, 8, 1, 2, 16, 4 | 1, 4 | 2, 16 | 1, 16 | 2, 16 | 4, 0];
    A[rng.below(A.len() as u64) as usize]
}

/// One random macro: a mask and how many frames to hold it.
fn macro_step(rng: &mut Rng, dej: bool, cej: bool) -> (u8, u32) {
    let mut m = if dej { post_eject_mask(rng) } else { mounted_mask(rng) };
    let r = rng.below(100);
    if !dej && r < 4 {
        m |= 32; // Shift: Dad ejects
    } else if !cej && r < 8 {
        m |= 64; // Ctrl: child ejects
    } else if (!dej || !cej) && r < 10 {
        m |= 128; // Z: both
    }
    let len = 1 + (rng.unit().powi(2) * 24.0) as u32;
    (m, len)
}

fn explore(args: &Args) -> Result<(), String> {
    let run_dir = std::path::PathBuf::from(args.get("--run-dir").ok_or("missing --run-dir")?);
    std::fs::create_dir_all(&run_dir).map_err(|e| e.to_string())?;
    let threads: usize = args.num("--threads", 14);
    let hours: f64 = args.num("--hours", 1.0);
    let seed: u64 = args.num("--seed", 1);
    let max_frame: u32 = args.num("--max-frame", 299);
    let rollout: u32 = args.num("--rollout", 45);
    let batch: usize = args.num("--batch", 16);
    sim::set_hang_ms(args.num("--hang-ms", 400));
    let mut seeds: Vec<Vec<u8>> = vec![Vec::new()];
    let mut resume: Vec<Vec<u8>> = Vec::new();
    if let Some(p) = args.get("--resume") {
        for line in std::fs::read_to_string(p).map_err(|e| e.to_string())?.lines() {
            if let Some(m) = tas_field(line) {
                resume.push(m);
            }
        }
        eprintln!("explore: resuming {} archive cells", resume.len());
    }
    if let Some(list) = args.get("--seeds") {
        for f in list.split(',') {
            seeds.push(read_tas(f)?);
        }
    }
    let shared = Shared {
        cells: Mutex::new(HashMap::new()),
        stop: AtomicBool::new(false),
        frames: AtomicU64::new(0),
        rollouts: AtomicU64::new(0),
        pairmax: AtomicU64::new(0),
        above: AtomicU64::new(0),
        best_pairs: AtomicU64::new(0),
        best_speed: Mutex::new(0.0),
        events: Mutex::new(std::fs::OpenOptions::new().create(true).append(true).open(run_dir.join("events.jsonl")).map_err(|e| e.to_string())?),
        winner: Mutex::new(None),
    };
    // Seed cells: replay every prefix of every seed, adding cells along the way.
    let template = Sim::new(LEVEL, CHAR)?;
    for sd in &seeds {
        let mut s = template.fork();
        s.masks = sd.clone();
        let mut cells = shared.cells.lock().unwrap();
        cells.entry(key(&Fr { frame: 0, ..Default::default() })).or_insert(Cell { masks: Vec::new(), value: 0.0, visits: 0 });
        while (s.replay.iteration as usize) < sd.len() && s.frame() < max_frame {
            let f = s.step(0);
            if f.port_abort || !f.alive_after {
                break;
            }
            let k = key(&f);
            let above = f.alive_after && f.chest_after[1] < CEILING_Y && f.chest_after[0] < CEILING_X1;
            let used = s.masks[..s.replay.iteration as usize].to_vec();
            let v = value(&k, above);
            cells.entry(k).or_insert(Cell { masks: used, value: v, visits: 0 });
        }
    }
    // Archive cells resume as-is (keys recomputed from a replay of each prefix), in parallel.
    {
        let next = std::sync::atomic::AtomicUsize::new(0);
        let (resume, template, shared) = (&resume, &template, &shared);
        std::thread::scope(|scope| {
            for _ in 0..threads {
                let next = &next;
                std::thread::Builder::new()
                    .stack_size(32 << 20)
                    .spawn_scoped(scope, move || {
                        let mut s = template.fork();
                        loop {
                            let i = next.fetch_add(1, Ordering::Relaxed);
                            if i >= resume.len() {
                                break;
                            }
                            let m = &resume[i];
                            s.restore(template);
                            s.masks = m.clone();
                            let mut last = None;
                            while (s.replay.iteration as usize) < m.len() && s.frame() < max_frame {
                                let f = s.step(0);
                                if f.port_abort || !f.alive_after || f.step_us > 80_000 {
                                    last = None;
                                    break;
                                }
                                last = Some(f);
                            }
                            if let Some(f) = last {
                                let k = key(&f);
                                let above = f.alive_after && f.chest_after[1] < CEILING_Y && f.chest_after[0] < CEILING_X1;
                                shared.cells.lock().unwrap().entry(k).or_insert(Cell { masks: m.clone(), value: value(&k, above), visits: 0 });
                            }
                        }
                    })
                    .unwrap();
            }
        });
    }
    eprintln!("explore: {} seed cells", shared.cells.lock().unwrap().len());
    let start = Instant::now();
    let deadline = start + Duration::from_secs_f64(hours * 3600.0);
    // Detached workers: a frame that never returns (a game-level hang) cannot block shutdown.
    let shared: &'static Shared = Box::leak(Box::new(shared));
    let template: &'static Sim = Box::leak(Box::new(template));
    let run_dir: &'static std::path::Path = Box::leak(run_dir.into_boxed_path());
    let beats: &'static Vec<AtomicU64> = Box::leak(Box::new((0..threads).map(|_| AtomicU64::new(0)).collect()));
    let current: &'static Vec<Mutex<Vec<u8>>> = Box::leak(Box::new((0..threads).map(|_| Mutex::new(Vec::new())).collect()));
    let mut reported = vec![false; threads];
    for w in 0..threads {
        std::thread::Builder::new()
            .stack_size(32 << 20)
            .spawn(move || worker(w as u64, seed, template, shared, run_dir, max_frame, rollout, batch, &beats[w], start, &current[w]))
            .unwrap();
    }
    let mut st = std::fs::OpenOptions::new().create(true).append(true).open(run_dir.join("stats.jsonl")).unwrap();
    let mut last_dump = Instant::now();
    loop {
        std::thread::sleep(Duration::from_secs(10));
        let stop = Instant::now() > deadline || run_dir.join("STOP").exists() || shared.stop.load(Ordering::Relaxed);
        let t = start.elapsed().as_secs_f64();
        let now_ms = start.elapsed().as_millis() as u64;
        let stuck = beats.iter().filter(|b| now_ms.saturating_sub(b.load(Ordering::Relaxed)) > 30_000).count();
        for (w, b) in beats.iter().enumerate() {
            if now_ms.saturating_sub(b.load(Ordering::Relaxed)) > 30_000 && !reported[w] {
                reported[w] = true;
                if let Ok(c) = current[w].try_lock() {
                    std::fs::write(run_dir.join(format!("stuck-w{w}.tas")), tas_text(&c)).ok();
                }
            }
        }
        let line = format!(
            "{{\"t\":{:.0},\"cells\":{},\"fps\":{:.0},\"rollouts\":{},\"best_pairs\":{},\"best_speed\":{:.1},\"pairmax_events\":{},\"above_ceiling_events\":{},\"stuck_workers\":{},\"winner\":{}}}",
            t,
            shared.cells.lock().unwrap().len(),
            shared.frames.load(Ordering::Relaxed) as f64 / t,
            shared.rollouts.load(Ordering::Relaxed),
            shared.best_pairs.load(Ordering::Relaxed),
            *shared.best_speed.lock().unwrap(),
            shared.pairmax.load(Ordering::Relaxed),
            shared.above.load(Ordering::Relaxed),
            stuck,
            shared.winner.lock().unwrap().is_some()
        );
        writeln!(st, "{line}").ok();
        eprintln!("{line}");
        if stop || last_dump.elapsed() > Duration::from_secs(120) {
            dump(shared, run_dir);
            last_dump = Instant::now();
        }
        if stop {
            shared.stop.store(true, Ordering::Relaxed);
            // Give workers a moment to finish in-flight rollouts, then exit regardless.
            std::thread::sleep(Duration::from_secs(5));
            dump(shared, run_dir);
            std::process::exit(if shared.winner.lock().unwrap().is_some() { 0 } else { 1 });
        }
    }
}

/// Archive export plus the top cells by pair pressure (for inspection while running).
fn dump(shared: &Shared, run_dir: &std::path::Path) {
    let cells = shared.cells.lock().unwrap();
    if let Ok(mut out) = std::fs::File::create(run_dir.join("archive.jsonl")) {
        for (k, c) in cells.iter() {
            writeln!(out, "{{\"key\":\"{:?}\",\"value\":{:.2},\"visits\":{},\"frames\":{},\"tas\":\"{}\"}}", k, c.value, c.visits, c.masks.len(), tas_text(&c.masks)).ok();
        }
    }
}

fn pick(shared: &Shared, rng: &mut Rng) -> Option<(Key, Vec<u8>)> {
    let mut cells = shared.cells.lock().unwrap();
    if cells.is_empty() {
        return None;
    }
    let n = cells.len();
    let k = if rng.below(6) == 0 {
        let i = rng.below(n as u64) as usize;
        *cells.keys().nth(i).unwrap()
    } else {
        // Weighted: high pair pressure / speed, few visits.
        let maxv = cells.values().map(|c| c.value).fold(0.0f64, f64::max);
        let mut total = 0.0;
        let ws: Vec<(Key, f64)> = cells
            .iter()
            .map(|(k, c)| {
                let w = ((c.value - maxv) / 3.0).exp() / (1.0 + c.visits as f64).sqrt();
                total += w;
                (*k, w)
            })
            .collect();
        let mut x = rng.unit() * total;
        let mut pick = ws[ws.len() - 1].0;
        for (k, w) in ws {
            x -= w;
            if x <= 0.0 {
                pick = k;
                break;
            }
        }
        pick
    };
    let c = cells.get_mut(&k).unwrap();
    c.visits += 1;
    Some((k, c.masks.clone()))
}

#[allow(clippy::too_many_arguments)]
fn worker(w: u64, seed: u64, template: &Sim, shared: &Shared, run_dir: &std::path::Path, max_frame: u32, rollout: u32, batch: usize, beat: &AtomicU64, start: Instant, current: &Mutex<Vec<u8>>) {
    let mut rng = Rng(seed ^ (w + 1).wrapping_mul(0x5851_f42d_4c95_7f2d));
    let mut base = template.fork();
    let mut s = template.fork();
    while !shared.stop.load(Ordering::Relaxed) {
        let Some((_, masks)) = pick(shared, &mut rng) else { break };
        // Replay the cell's prefix once.
        base.restore(template);
        base.masks.clear();
        base.masks.extend_from_slice(&masks);
        let mut ok = true;
        let mut last = Fr::default();
        *current.lock().unwrap() = masks.clone();
        while (base.replay.iteration as usize) < masks.len() {
            let f = base.step(0);
            beat.store(start.elapsed().as_millis() as u64, Ordering::Relaxed);
            shared.frames.fetch_add(1, Ordering::Relaxed);
            if f.port_abort || !f.alive_after || base.frame() >= max_frame {
                ok = false;
                break;
            }
            last = f;
        }
        if !ok {
            continue;
        }
        for _ in 0..batch {
            if shared.stop.load(Ordering::Relaxed) {
                break;
            }
            shared.rollouts.fetch_add(1, Ordering::Relaxed);
            s.restore(&base);
            let (mut dej, mut cej) = (last.dad_ejected, last.child_ejected);
            let mut frames = 0;
            'roll: while frames < rollout && s.frame() < max_frame {
                let (m, len) = macro_step(&mut rng, dej, cej);
                for _ in 0..len {
                    if frames >= rollout || s.frame() >= max_frame {
                        break 'roll;
                    }
                    // Force a fresh entry for this frame (the replay reads masks by iteration).
                    let it = s.replay.iteration as usize;
                    s.masks.truncate(it);
                    {
                        let mut c = current.lock().unwrap();
                        c.clear();
                        c.extend_from_slice(&s.masks);
                        c.push(m);
                    }
                    let f = s.step(m);
                    beat.store(start.elapsed().as_millis() as u64, Ordering::Relaxed);
                    frames += 1;
                    shared.frames.fetch_add(1, Ordering::Relaxed);
                    if f.port_abort || f.step_us > 80_000 {
                        break 'roll;
                    }
                    dej = f.dad_ejected;
                    cej = f.child_ejected;
                    let used = || s.masks[..s.replay.iteration as usize].to_vec();
                    if f.pairs as u64 > shared.best_pairs.load(Ordering::Relaxed) {
                        shared.best_pairs.fetch_max(f.pairs as u64, Ordering::Relaxed);
                    }
                    {
                        let mut bs = shared.best_speed.lock().unwrap();
                        if f.speed_max > *bs {
                            *bs = f.speed_max;
                        }
                    }
                    let above = f.alive_after && f.chest_after[1] < CEILING_Y && f.chest_after[0] < CEILING_X1;
                    if f.site != 0 || above || f.finished {
                        let kind = if f.finished { "finish" } else if f.site == 2 { "pairmax" } else if f.site != 0 { "throw" } else { "above" };
                        if f.site == 2 {
                            shared.pairmax.fetch_add(1, Ordering::Relaxed);
                        }
                        if above {
                            shared.above.fetch_add(1, Ordering::Relaxed);
                        }
                        writeln!(
                            shared.events.lock().unwrap(),
                            "{{\"kind\":\"{}\",\"frame\":{},\"site\":{},\"alive\":{},\"pairs\":{},\"chest_before\":[{:.3},{:.3}],\"chest\":[{:.3},{:.3}],\"cv\":[{:.1},{:.1}],\"speed_max\":{:.1},\"dej\":{},\"cej\":{},\"tas\":\"{}\"}}",
                            kind, f.frame, f.site, f.alive_after, f.pairs, f.chest_before[0], f.chest_before[1], f.chest_after[0], f.chest_after[1],
                            f.chest_v[0], f.chest_v[1], f.speed_max, f.dad_ejected, f.child_ejected, tas_text(&used())
                        )
                        .ok();
                        if f.finished && f.alive_before {
                            std::fs::write(run_dir.join("winner-1.tas"), tas_text(&used())).ok();
                            *shared.winner.lock().unwrap() = Some(used());
                            shared.stop.store(true, Ordering::Relaxed);
                            break 'roll;
                        }
                    }
                    if !f.alive_after {
                        break 'roll;
                    }
                    let k = key(&f);
                    let v = value(&k, above);
                    let mut cells = shared.cells.lock().unwrap();
                    match cells.get_mut(&k) {
                        None => {
                            cells.insert(k, Cell { masks: used(), value: v, visits: 0 });
                        }
                        Some(c) => {
                            // Prefer the shorter prefix for the same cell (earlier is better).
                            if used().len() < c.masks.len() {
                                c.masks = used();
                            }
                        }
                    }
                }
            }
        }
    }
}
