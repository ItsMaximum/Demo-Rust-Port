//! hw-search: launch analysis and input search on the Flash-verified CPU port.
//! Built only with the `lab` feature (research hooks); see docs/SEARCH.md.
mod analyze;
mod brute;
mod cannon;
mod ceiling;
mod events;
mod explore;
mod finish_policy;
mod perturb;
mod pair;
mod progress;
mod pool_policy;
mod qe;
mod repair_policy;
mod setup;
mod bundle;
mod sim;
mod sparse;
mod tail;
mod window_policy;

use std::collections::BTreeMap;

type Result<T> = std::result::Result<T, String>;

const HELP: &str = "hw-search analyze --level ID --char N --tas FILE_OR_BITS --from A --to B [--raw-events] [--toi-target-probe] [--raw-grip-context]
    One JSON line per frame A..B: chest solve/TOI chain, finish checks, freezes, smash.
    --raw-events preserves ordered events and exact float bits for pre-step primary CHEST.
    --toi-target-probe optionally adds fixed CHEST x exact L45 world-ceiling scan diagnostics; requires --raw-events.
    --raw-grip-context optionally adds immutable pre-input/post-end_frame arm joint edges and pending Results; requires --raw-events.
hw-search ceiling --level 45 --char 3 --bases A.tas,B.tas --run-dir FRESH_DIR
                  [--threads N] [--hours 0.1] [--window 299] [--pool 200]
                  [--edit-from 0] [--edit-to 299]
                  [--launch-pool 0] [--launch-min-frame 128] [--launch-min-speed 1024]
                  [--pressure-pool 0] [--pressure-min-pairs 1024] [--pressure-min-frame 1] [--pressure-max-frame 299]
                  [--endpoint-pool 0] [--endpoint-min-frame 1]
                  [--enumerate-cells A,B] [--enumerate-start 0] [--enumerate-end 65536]
                  [--enumerate-low5-cells C0,...,C4] [--enumerate-start 0] [--enumerate-end 32^N]
                  [--enumerate-low5-values 17,18,20,24,28]
    Optional deterministic full-eight-bit two-cell enumeration uses one full299 root.
    Low5 mode instead takes1..5 distinct raw cells, all0..31 masks each, and preserves own high3/all outside bytes.
    Low5 IDs are base32 in declared cell order, last cell fastest; defaults17 workers plus one coordinator.
    Optional low5 values select1..32 unique values0..31 in declared alphabet order; IDs use that alphabet's length as radix.
    A custom alphabet has len^N cases; baseline ID is null if any selected root low5 is absent from the alphabet.
    Low5 root is exactly2392 ASCII bits; per-case Main/AfterToi COM diagnostics require no ceiling solve or archive credit.
    Legacy case IDs are (maskA<<8)|maskB; shards are [start,end). Baseline is retained, including no-op.
    Guarded/interrupted cases are recorded separately; partial runs never certify exhaustive negatives.
    Search the full opening and ejected setup using primary Dad upward solve velocity
    and measured world-ceiling contacts. Full299 inputs, fresh replay, no state injection.
    Actual finishes are confirmed before fitness filters; pairmax remains separate telemetry.
    Edit bounds are raw input indices [from,to); every attempt still plans the full299.
    --launch-pool opts into a separate host launch archive (0 disables; maximum1024).
    Archive thresholds are frame1..299 and finite nonnegative speed; finish acceptance is unchanged.
    Admission requires positive upward speed meeting the threshold OR an actual ceiling candidate/solve observation.
    --pressure-pool opts into four bounded unscored global-pair DNA strata;0 preserves old RNG.
    Pair counts do not establish Dad force, survival, selective omission or completion.
    --endpoint-pool opts into four bounded unscored banks of observed primary COM endpoints.
    Closest finish-box distance guides parent selection; geometry alone never establishes a finish.
hw-search cannon --level ID --char N --tas PREFIX --frame F [--theta 0:360:0.25] [--logv 2:6:0.02] [--k 1] [--threads N]
    Replay PREFIX to frame F-1, then at frame F overwrite the chest's end-of-solve velocity
    (angle 0=right, 90=up; speed 10^logv m/s) and position (c0 + k*v*dt); report finishes.
    --origin X,Y also moves the start of the chest's sweep (virtual launch origin).
hw-search qe --level ID --char N --run-dir DIR --seeds A.tas,B.tas [--fp] [--uniform 0.2] [--seed-share 0]
             [--adaptive-window] [--resume-pool-file pool.jsonl] [--setup-share 0] [--repair-pool-file repair.jsonl]
             [--retain-repeated-launches]
    Sample launch lineages; --uniform is the chance of choosing a parent uniformly (0..1).
    --seed-share samples initial parents uniformly and protects them from trimming (0..1).
    --repair-share samples failed tail edits on remaining picks; --repair-per-seed bounds each bank.
    --adaptive-window permits short nonempty parents, replaying from entry zero when needed.
    --resume-pool-file restores ordinary pool scores/visits separately from protected initial parents.
    --setup-share samples all-key setup edits within the replay window (0..1; default 0).
    --repair-pool-file restores failed proposals with their original ancestor and generation.
    --retain-repeated-launches keeps eligible new raw nonfinish parents on repeated launch outcomes (default false).
    Other selections weight route fitness and visits; distinct finishing TAS strings are measured independently.
hw-search run --level ID --char N --run-dir DIR [--seeds A.tas,B.tas] [--threads N] [--hours 10]
              [--seed 1] [--min-launch 20] [--max-frame 900] [--rollout 60] [--finish-any]
    Explore grab/pose inputs for a one-frame chest launch onto the finish; stops at the first
    winner confirmed by an exact from-scratch replay (DIR/winner-1.tas). STOP file ends early.
    --finish-any also accepts a confirmed finish without a same-frame launch (run and perturb).
hw-search perturb --qe --level ID --char N --run-dir DIR --bases A.tas,B.tas
                  [--anchor 0] [--qe-persist 1] [--finish-any]
    --anchor preserves that many starting entries and keeps full base setups (default off).
    --qe-persist scores the minimum QE over that many consecutive frames (1..64).
hw-search pair --level 45 --char 3 --bases A.tas,B.tas --run-dir FRESH_DIR
               [--threads N] [--hours 2] [--seed 1] [--window 239] [--extra 24] [--pool 500]
    Fixed Dad/Obstacle occupancy: preserve60 entries, score six clean post-frame pair counts,
    maximum299 world frames; full initial bases stay protected. Continues after launches.
    Live plans have at least239 explicit entries so exact finite confirmation covers299.
    --lag-ms 15 --lag-frames 3 resets fitness on lag and bounds slow branches (lag-ms0 off).
    --threads 0 --hours 0 only exports the full protected initial pool; no replay/winner.
hw-search progress --level 45 --char 3 --bases A.tas,B.tas --run-dir FRESH_DIR
                   [--threads N] [--hours 2] [--seed 1] [--window 239] [--extra 24] [--pool 500]
                   [--objective finish|fan|launch] [--diversity-pool 0] [--route-checkpoints]
    Preserve60 entries and full protected bases; minimize worst finish-focus distance over
    six clean fresh frames with living Dad. QE/pairs are telemetry; zero distance is no win.
    Finite replays stop by299, check every real finish before gates, and continue after launches.
    --objective fan ranks actual six-frame approach/ascent through the boost shaft and fan.
    --objective launch compares measured six-frame Dad velocity with route/time demand.
    Launch fitness requires actual living-Dad survival to299. --diversity-pool 64 adds
    a separate actual position/time route archive; half the workers sample it uniformly.
    --route-checkpoints splits that capacity between measured180/240 windows and
    samples represented source families uniformly before choosing a route member.
    Same lag controls and zero-worker/zero-hour initialization contract as pair.
hw-search brute --level ID --char N --tas BASE [--depth 1] [--alphabet launch|all] [--extra 2]
                [--min-launch 60] [--threads N] [--out brute.jsonl] [--limit N]
    Enumerate every input combination on the last --depth frames of BASE (alphabet `launch`: the
    32 pose/grab combinations; `all`: all 256 masks), replaying only depth+extra frames from one
    snapshot. A launch is a living chest that moved at least --min-launch metres in a frame; a
    finish is confirmed by a from-scratch replay. Results go to --out as JSON lines.
hw-search sparse --level ID --char N --tas FINISHING_TAS --limit FRAME --run-dir DIR
                 [--threads N] [--hours 10] [--seed 1] [--kmax 4] [--span 30] [--jmin 0]
                 [--min-launch 60] [--batch 32] [--bits 255] [--sreach 40]
    Rewrite a finishing string with fewer frame-perfect key events, still finishing by --limit.
    Keeps j of the string's events and re-times or adds up to --kmax new ones within --span frames.
    Writes each improved string to DIR as a .tas file, plus finishes.jsonl and stats.jsonl;
    creating a file named STOP in DIR ends the search early.
hw-search tail --level ID --char N --tas FINISHING_TAS --cut ENTRY --limit FRAME --run-dir DIR
               [--window 30] [--k 2] [--hold MASK] [--from 0] [--max-cost 0] [--bounds x0,x1,y0,y1]
               [--min-launch 60] [--threads N] [--seed 1]
    Keep the string up to ENTRY, hold one effective input class from there, and try every tail
    with up to --k class changes at entries in [cut, cut+window). Finishing tails are written to DIR
    as .tas files with finishes.jsonl and launches.jsonl.
hw-search canon --level ID --char N --tas TAS --limit FRAME
    Print TAS with every press/release pulse the finish does not need removed; a one-line JSON
    summary (finish frame, events, frame-perfect count) goes to stderr.
hw-search confirm --level ID --char N --tas TAS [--min-launch 20]
    Replay TAS from scratch and confirm a finish on a frame where the chest launched.";

pub struct Args(BTreeMap<String, String>);

impl Args {
    fn parse(it: impl Iterator<Item = String>) -> Result<Self> {
        let mut m = BTreeMap::new();
        let mut it = it.peekable();
        while let Some(flag) = it.next() {
            if !flag.starts_with("--") {
                return Err(format!("unexpected argument {flag}"));
            }
            // --flag=value (needed for values starting with '-', e.g. --bounds=-4,29,-84,48)
            if let Some((k, v)) = flag.split_once('=') {
                m.insert(k.to_string(), v.to_string());
                continue;
            }
            let value = match it.peek() {
                Some(v) if !v.starts_with("--") => it.next().unwrap(),
                _ => "true".into(),
            };
            m.insert(flag, value);
        }
        Ok(Args(m))
    }
    pub fn get(&self, k: &str) -> Option<&str> {
        self.0.get(k).map(String::as_str)
    }
    pub fn req(&self, k: &str) -> Result<&str> {
        self.get(k).ok_or_else(|| format!("missing {k}"))
    }
    pub fn num<T: std::str::FromStr>(&self, k: &str, default: T) -> Result<T> {
        match self.get(k) {
            Some(v) => v.parse().map_err(|_| format!("bad value for {k}: {v}")),
            None => Ok(default),
        }
    }
}

/// Checkpoint members remain ordinary pool entries; they do not become initial parents.
fn parse_resume_pool(text: &str) -> Result<Vec<(Vec<u8>, f64, u64)>> {
    let mut members = Vec::new();
    for line in text.lines() {
        if let Ok(v) = serde_json::from_str::<serde_json::Value>(line) {
            if let Some(t) = v["tas"].as_str().filter(|t| !t.is_empty()) {
                let masks = sim::parse_tas(t)?;
                let score = if v["score_version"].as_u64() == Some(qe::SCORE_VERSION) {
                    v["score"].as_f64().filter(|s| s.is_finite()).unwrap_or(400.0)
                } else {
                    400.0
                };
                let visits = v["visits"].as_u64().unwrap_or(0);
                members.push((masks, score, visits));
            }
        }
    }
    Ok(members)
}

/// A repair checkpoint must load completely or fail; missing fields cannot drop history.
fn parse_repair_pool(text: &str) -> Result<Vec<(u64, Vec<u8>, u32)>> {
    let mut members = Vec::new();
    for (index, line) in text.lines().enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let at = format!("repair pool line {}", index + 1);
        let v: serde_json::Value = serde_json::from_str(line).map_err(|e| format!("{at}: {e}"))?;
        let ancestor = v["ancestor"].as_u64().ok_or_else(|| format!("{at}: ancestor must be u64"))?;
        let generation = v["generation"].as_u64()
            .and_then(|n| u32::try_from(n).ok()).filter(|&n| n > 0)
            .ok_or_else(|| format!("{at}: generation must be positive u32"))?;
        let text = v["tas"].as_str().filter(|t| !t.trim().is_empty())
            .ok_or_else(|| format!("{at}: missing nonempty tas"))?;
        let masks = sim::parse_tas(text).map_err(|e| format!("{at}: {e}"))?;
        members.push((ancestor, masks, generation));
    }
    Ok(members)
}

/// Dedicated fixed profile; unrelated mode flags cannot silently alter pair fitness.
fn pair_config(args: &Args) -> Result<pair::Config> {
    const ALLOWED: &[&str] = &["--level", "--char", "--bases", "--run-dir", "--threads", "--hours",
        "--seed", "--window", "--extra", "--pool", "--hang-ms", "--lag-ms", "--lag-frames"];
    for key in args.0.keys() {
        if !ALLOWED.contains(&key.as_str()) { return Err(format!("unsupported pair option {key}; anchor60/window6/max299/finish-any are fixed")); }
    }
    let mut bases = Vec::new();
    for input in args.req("--bases")?.split(',') {
        if input.is_empty() { return Err("empty pair base in CSV".into()); }
        bases.push(sim::read_tas(input)?);
    }
    let cfg = pair::Config { level:args.num("--level",45)?, character:args.num("--char",3)?,
        bases, run_dir:args.req("--run-dir")?.into(), threads:args.num("--threads",default_threads())?,
        hours:args.num("--hours",2.0)?, seed:args.num("--seed",1)?, window:args.num("--window",239)?,
        extra:args.num("--extra",24)?, pool_size:args.num("--pool",500)?,
        lag_ms:args.num("--lag-ms",15.0)?, lag_frames:args.num("--lag-frames",3)? };
    cfg.validate()?;
    Ok(cfg)
}

/// Reuse only fixed-profile CLI validation; no occupancy scores or pools are imported.
fn progress_config(args: &Args) -> Result<progress::Config> {
    let objective=progress::Objective::parse(args.get("--objective").unwrap_or("finish"))?;
    let diversity_pool=args.num("--diversity-pool",0)?;
    let route_checkpoints=args.num("--route-checkpoints",false)?;
    let mut fixed=args.0.clone();fixed.remove("--objective");fixed.remove("--diversity-pool");fixed.remove("--route-checkpoints");
    let c=pair_config(&Args(fixed))?;
    let config=progress::Config {level:c.level,character:c.character,bases:c.bases,run_dir:c.run_dir,
        threads:c.threads,hours:c.hours,seed:c.seed,window:c.window,extra:c.extra,pool_size:c.pool_size,
        lag_ms:c.lag_ms,lag_frames:c.lag_frames,objective,diversity_pool,route_checkpoints};
    config.validate()?;
    Ok(config)
}

fn ceiling_config(args: &Args) -> Result<ceiling::Config> {
    const ALLOWED: &[&str] = &["--level", "--char", "--bases", "--run-dir", "--threads", "--hours",
        "--seed", "--window", "--pool", "--hang-ms", "--lag-ms", "--lag-frames", "--edit-from", "--edit-to",
        "--launch-pool", "--launch-min-frame", "--launch-min-speed",
        "--pressure-pool", "--pressure-min-pairs", "--pressure-min-frame", "--pressure-max-frame",
        "--endpoint-pool", "--endpoint-min-frame", "--enumerate-cells", "--enumerate-low5-cells", "--enumerate-low5-values", "--enumerate-start", "--enumerate-end"];
    for key in args.0.keys() {
        if !ALLOWED.contains(&key.as_str()) { return Err(format!("unsupported ceiling option {key}")); }
    }
    let enumeration=ceiling_enumeration_config(args)?;let low5=enumeration.as_ref().is_some_and(|s|s.low5);
    let mut bases = Vec::new();
    for input in args.req("--bases")?.split(',') {
        if input.is_empty() { return Err("empty ceiling base in CSV".into()); }
        bases.push(if low5 {read_ceiling_low5_root(input)?}else {sim::read_tas(input)?});
    }
    let cfg = ceiling::Config { level:args.num("--level",45)?, character:args.num("--char",3)?,
        enumeration,
        bases, run_dir:args.req("--run-dir")?.into(), threads:args.num("--threads",if low5 {17}else {default_threads()})?,
        hours:args.num("--hours",0.1)?, seed:args.num("--seed",1)?, window:args.num("--window",299)?,
        edit_from:args.num("--edit-from",0)?, edit_to:args.num("--edit-to",299)?,
        launch_pool_size:args.num("--launch-pool",0)?, launch_min_frame:args.num("--launch-min-frame",128)?,
        launch_min_speed:args.num("--launch-min-speed",1024.0)?,
        pressure_pool_size:args.num("--pressure-pool",0)?, pressure_min_pairs:args.num("--pressure-min-pairs",1024)?,
        pressure_min_frame:args.num("--pressure-min-frame",1)?, pressure_max_frame:args.num("--pressure-max-frame",299)?,
        endpoint_pool_size:args.num("--endpoint-pool",0)?, endpoint_min_frame:args.num("--endpoint-min-frame",1)?,
        pool_size:args.num("--pool",200)?, lag_ms:args.num("--lag-ms",15.0)?,
        lag_frames:args.num("--lag-frames",3)? };
    cfg.validate()?;
    Ok(cfg)
}

fn parse_ceiling_low5_root(bits:&[u8])->Result<Vec<u8>> {
    if bits.len()!=2392 || !bits.iter().all(|&b|matches!(b,b'0'|b'1')) {
        return Err("low5 enumeration root must be exactly2392 ASCII0/1 bytes with no whitespace or BOM".into());}
    sim::parse_tas(std::str::from_utf8(bits).map_err(|e|e.to_string())?)
}
fn read_ceiling_low5_root(input:&str)->Result<Vec<u8>> {
    if input.bytes().all(|b|matches!(b,b'0'|b'1')) {parse_ceiling_low5_root(input.as_bytes())}
    else {parse_ceiling_low5_root(&std::fs::read(input).map_err(|e|format!("{input}: {e}"))?)}
}
fn ceiling_enumeration_config(args:&Args)->Result<Option<ceiling::EnumerationConfig>> {
    let legacy=args.get("--enumerate-cells");let low5_cells=args.get("--enumerate-low5-cells");
    if legacy.is_some() && low5_cells.is_some() {return Err("enumerate-cells and enumerate-low5-cells are mutually exclusive".into());}
    if args.get("--enumerate-low5-values").is_some() && low5_cells.is_none() {
        return Err("enumerate-low5-values requires enumerate-low5-cells".into());}
    let Some(cells)=low5_cells.or(legacy) else {
        if args.get("--enumerate-start").is_some() || args.get("--enumerate-end").is_some() {
            return Err("enumerate-start/end require enumerate-cells or enumerate-low5-cells".into());}return Ok(None);};
    let low5=low5_cells.is_some();let raw=cells.split(',').collect::<Vec<_>>();
    if (low5 && !(1..=5).contains(&raw.len())) || (!low5 && raw.len()!=2) {
        return Err("enumerate-cells requires two raw indices; enumerate-low5-cells requires1..5".into());}
    let cells=raw.into_iter().map(|s|s.parse::<usize>().map_err(|_|"invalid enumerate raw index".to_string())).collect::<Result<Vec<_>>>()?;
    let low5_values=if low5 {match args.get("--enumerate-low5-values") {
        None=>(0..32).collect::<Vec<u8>>(),
        Some(values)=>{
            let values=values.split(',').map(|s|s.parse::<u8>().map_err(|_|"invalid enumerate low5 value".to_string()))
                .collect::<Result<Vec<_>>>()?;
            if !(1..=32).contains(&values.len()) || values.iter().enumerate().any(|(i,&v)|v>31 || values[..i].contains(&v)) {
                return Err("enumerate-low5-values requires1..32 unique values in0..31".into());}values}
    }}else {Vec::new()};
    let end=if low5 {(low5_values.len() as u32).pow(cells.len() as u32)}else {65536};
    Ok(Some(ceiling::EnumerationConfig {cells,low5,low5_values,start:args.num("--enumerate-start",0)?,end:args.num("--enumerate-end",end)?}))
}
fn validate_ceiling_launch_options(command: &str, args: &Args) -> Result<()> {
    if command != "ceiling" && ["--launch-pool", "--launch-min-frame", "--launch-min-speed",
        "--pressure-pool", "--pressure-min-pairs", "--pressure-min-frame", "--pressure-max-frame",
        "--endpoint-pool", "--endpoint-min-frame", "--enumerate-cells", "--enumerate-low5-cells", "--enumerate-low5-values", "--enumerate-start", "--enumerate-end"]
        .iter().any(|flag| args.get(flag).is_some()) {
        return Err("launch, pressure and endpoint archive options are only valid for ceiling".into());
    }
    Ok(())
}

fn validate_raw_grip_context_options(command: &str, args: &Args) -> Result<()> {
    if args.get("--raw-grip-context").is_some() {
        let _: bool = args.num("--raw-grip-context", false)?;
        if command != "analyze" || !args.num("--raw-events", false)? {
            return Err("--raw-grip-context is only valid for analyze --raw-events".into());
        }
    }
    Ok(())
}

/// Documented commands: stable flags, covered by docs/SEARCH.md and docs/SEARCH_RECIPES.md.
const CURATED: &[(&str, &str)] = &[
    ("analyze", "per-frame launch diagnostics for a TAS (chest solve, TOI chain, finish checks)"),
    ("qe", "evolve launch lineages from seed TAS files (the main QE / pairmax search)"),
    ("run", "Go-Explore search for a one-frame chest launch onto the finish; stops at a confirmed winner"),
    ("perturb", "mutate known QE bases looking for a finishing launch"),
    ("brute", "exhaustive enumeration of the inputs on the last frames of a launch-prone string"),
    ("sparse", "rewrite a finishing string with as few frame-perfect key events as possible"),
    ("tail", "exhaustive low-input tails after a cut point of a finishing string"),
    ("canon", "drop every press/release pulse a finishing string does not need"),
    ("cannon", "scan synthetic launch velocities at a frame (a what-if sweep, not an input)"),
    ("confirm", "replay a TAS from scratch and confirm a same-frame launch finish"),
];

/// Research commands: flags may change, outputs are for analysis rather than winners.
const EXPERIMENTAL: &[(&str, &str)] = &[
    ("pair", "pair-count search from protected bases (Obstacle Course, level 45 / character 3)"),
    ("progress", "finish-distance search from protected bases (Obstacle Course, level 45 / character 3)"),
    ("ceiling", "search the opening for upward launches against the world ceiling"),
    ("cannon-rand", "randomised synthetic launch sweep"),
    ("route", "which world borders the chest hits on the last frame of a TAS"),
    ("profile", "per-frame cost of TAS files (step time, QE, pairs, contacts)"),
    ("sensitivity", "flip each input bit in the last frames of a launch and classify the outcome"),
    ("joints", "level joints after N frames of a TAS"),
    ("finish", "print the level's finish region and shape details"),
    ("cost", "structural frame-perfect count of TAS strings"),
    ("events", "key presses and releases of a finishing string and how far each can move"),
    ("fc", "every finish check a TAS triggers"),
];

/// Number of worker threads when --threads is not given: all logical cores but two, at least one.
pub(crate) fn default_threads() -> usize {
    std::thread::available_parallelism().map(|n| n.get().saturating_sub(2).max(1)).unwrap_or(4).min(16)
}

/// The `hw-search <command> ...` block of HELP for one command, if documented.
fn help_block(command: &str) -> Option<String> {
    let mut out = String::new();
    let mut on = false;
    for line in HELP.lines() {
        if let Some(rest) = line.strip_prefix("hw-search ") {
            on = rest.split_whitespace().next() == Some(command);
        }
        if on {
            out.push_str(line);
            out.push('\n');
        }
    }
    (!out.is_empty()).then_some(out)
}

fn print_help(command: Option<&str>) {
    match command {
        Some(c) => match help_block(c) {
            Some(b) => print!("{b}"),
            None => match CURATED.iter().chain(EXPERIMENTAL).find(|(n, _)| *n == c) {
                Some((n, summary)) => println!("hw-search {n}: {summary}
(no detailed flag reference; see docs/SEARCH.md)"),
                None => println!("unknown command {c}"),
            },
        },
        None => {
            println!("hw-search: input search and launch analysis on the Flash-verified CPU port
");
            println!("usage: hw-search <command> [options]");
            println!("       hw-search help <command>");
            println!("       hw-search x <experimental-command>
");
            println!("commands:");
            for (name, summary) in CURATED {
                println!("  {name:<10} {summary}");
            }
            println!("
common options: --level ID --char N --run-dir DIR --seeds A.tas,B.tas --threads N --hours H");
            println!("--threads defaults to your logical cores minus two (at most 16). A STOP file in the run directory ends a search early.");
            println!("Experimental research commands are listed by `hw-search x`.");
        }
    }
}

fn print_experimental_help(command: Option<&str>) {
    match command {
        Some(c) => print_help(Some(c)),
        None => {
            println!("experimental commands (run as `hw-search x <command>`; flags may change):");
            for (name, summary) in EXPERIMENTAL {
                println!("  {name:<12} {summary}");
            }
        }
    }
}

fn run() -> Result<()> {
    let mut it = std::env::args().skip(1);
    let mut command = it.next().unwrap_or_default();
    if matches!(command.as_str(), "" | "help" | "--help" | "-h") {
        print_help(it.next().as_deref());
        return Ok(());
    }
    if command == "x" {
        command = it.next().unwrap_or_default();
        if command.is_empty() || command == "help" {
            print_experimental_help(it.next().as_deref());
            return Ok(());
        }
        if !EXPERIMENTAL.iter().any(|(name, _)| *name == command) && !CURATED.iter().any(|(name, _)| *name == command) {
            return Err(format!("unknown command {command}; run `hw-search x` for the list"));
        }
    } else if !CURATED.iter().any(|(name, _)| *name == command) {
        return if EXPERIMENTAL.iter().any(|(name, _)| *name == command) {
            Err(format!("{command} is experimental; run it as `hw-search x {command}` (see `hw-search x`)"))
        } else {
            Err(format!("unknown command {command}; run `hw-search help` for the list"))
        };
    }
    let rest: Vec<String> = it.collect();
    if rest.iter().any(|a| a == "--help" || a == "-h") {
        print_help(Some(&command));
        return Ok(());
    }
    let args = Args::parse(rest.into_iter())?;
    validate_ceiling_launch_options(&command, &args)?;
    validate_raw_grip_context_options(&command, &args)?;
    if args.get("--toi-target-probe").is_some()
        && (command!="analyze" || !args.num("--raw-events",false)?){
        return Err("--toi-target-probe is only valid for analyze --raw-events".into());
    }
    if !matches!(command.as_str(), "run" | "perturb") && args.get("--finish-any").is_some() {
        return Err("--finish-any is only valid for run and perturb".into());
    }
    if command != "perturb" && (args.get("--anchor").is_some() || args.get("--qe-persist").is_some()) {
        return Err("--anchor and --qe-persist are only valid for perturb".into());
    }
    // Searches drop a branch after --lag-frames consecutive frames that each cost more than
    // --lag-ms of CPU (runaway QE lag).
    sim::set_lag_ms(args.num("--lag-ms", 15.0)?);
    sim::set_lag_frames(args.num("--lag-frames", 3)?);
    // Searches abort a single frame that hangs longer than this (runaway TOI loop).
    if matches!(command.as_str(), "run" | "perturb" | "qe" | "sparse" | "tail" | "pair" | "progress" | "ceiling") {
        sim::set_hang_ms(args.num("--hang-ms", 500)?);
    }
    match command.as_str() {
        "ceiling" => { ceiling::run(ceiling_config(&args)?)?; Ok(()) }
        "analyze" => {
            let masks = sim::read_tas(args.req("--tas")?)?;
            if args.num("--raw-grip-context",false)? {
                return analyze::run_raw_primary_grip_context(args.num("--level",0)?, args.num("--char",0)?,
                    &masks, args.num("--from",1)?, args.num("--to",masks.len() as u32+60)?,
                    args.num("--toi-target-probe",false)?);
            }
            let run=if args.num("--toi-target-probe",false)? {analyze::run_raw_primary_toi_target}
                else if args.num("--raw-events",false)? {analyze::run_raw_primary}else {analyze::run};
            run(args.num("--level", 0)?, args.num("--char", 0)?, &masks, args.num("--from", 1)?, args.num("--to", masks.len() as u32 + 60)?)
        }
        "qe" => {
            // Launch sampling around launch lineages, scored by chain progress: see qe.rs.
            let adaptive_window = args.num("--adaptive-window", false)?;
            let retain_repeated_launches = args.num("--retain-repeated-launches", false)?;
            let mut seeds = Vec::new();
            let mut seed_scores = Vec::new();
            if let Some(pool) = args.get("--pool-file") {
                // A previous qe run's pool.jsonl (or launches.jsonl): every non-empty "tas".
                for line in std::fs::read_to_string(pool).map_err(|e| format!("{pool}: {e}"))?.lines() {
                    if let Ok(v) = serde_json::from_str::<serde_json::Value>(line) {
                        if let Some(t) = v["tas"].as_str().filter(|t| !t.is_empty()) {
                            seeds.push(sim::parse_tas(t)?);
                            // Older scores conflate impacts/strands with live finish checks.
                            // Unknown scores must not dominate the updated fitness pool.
                            seed_scores.push(if v["score_version"].as_u64() == Some(qe::SCORE_VERSION) {
                                v["score"].as_f64().filter(|s| s.is_finite()).unwrap_or(400.0)
                            } else {
                                400.0
                            });
                        }
                    }
                }
            }
            if let Some(list) = args.get("--seeds") {
                for f in list.split(',') {
                    seeds.push(sim::read_tas(f)?);
                }
            }
            if let Some(dir) = args.get("--seed-dir") {
                let mut files: Vec<_> = std::fs::read_dir(dir).map_err(|e| format!("{dir}: {e}"))?.filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "tas")).collect();
                files.sort();
                for f in files {
                    seeds.push(sim::read_tas(&f.to_string_lossy())?);
                }
            }
            let resume_members = match args.get("--resume-pool-file") {
                Some(pool) => parse_resume_pool(&std::fs::read_to_string(pool).map_err(|e| format!("{pool}: {e}"))?)?,
                None => Vec::new(),
            };
            let repair_members = match args.get("--repair-pool-file") {
                Some(pool) => parse_repair_pool(&std::fs::read_to_string(pool).map_err(|e| format!("{pool}: {e}"))?)?,
                None => Vec::new(),
            };
            let winner = qe::run(qe::Config {
                level: args.num("--level", 0)?,
                character: args.num("--char", 0)?,
                seeds,
                seed_scores,
                resume_members,
                repair_members,
                run_dir: args.req("--run-dir")?.into(),
                threads: args.num("--threads", default_threads())?,
                hours: args.num("--hours", 10.0)?,
                seed: args.num("--seed", 1)?,
                min_launch: args.num("--min-launch", 60.0)?,
                max_frame: args.num("--max-frame", 330)?,
                window: args.num("--window", 12)?,
                adaptive_window,
                retain_repeated_launches,
                extra: args.num("--extra", 8)?,
                batch: args.num("--batch", 64)?,
                pool: args.num("--pool", 5000)?,
                bounds: match args.get("--bounds") {
                    Some(r) => {
                        let v: Vec<f64> = r.split(',').map(|x| x.parse::<f64>().map_err(|_| format!("bad --bounds {r}"))).collect::<Result<_>>()?;
                        if v.len() != 4 {
                            return Err("--bounds x0,x1,y0,y1".into());
                        }
                        Some([v[0], v[1], v[2], v[3]])
                    }
                    None => None,
                },
                combo: args.num("--combo", 0.0)?,
                simplify: args.num("--simplify", 0.0)?,
                fp_mode: args.get("--fp").is_some(),
                ev_slack: args.num("--ev-slack", 8)?,
                ev_temp: args.num("--ev-temp", 0.0)?,
                deep: args.num("--deep", 0.0)?,
                budget_rel: match args.get("--budget-rel") {
                    Some(v) => Some(v.parse().map_err(|_| format!("bad --budget-rel {v}"))?),
                    None => None,
                },
                edits: args.num("--edits", 0.0)?,
                temp: args.num("--temp", 60.0)?,
                uniform: args.num("--uniform", 0.2)?,
                seed_share: args.num("--seed-share", 0.0)?,
                setup_share: args.num("--setup-share", 0.0)?,
                bundle_share: args.num("--bundle-share", 0.0)?,
                repair_share: args.num("--repair-share", 0.0)?,
                repair_per_seed: args.num("--repair-per-seed", 8)?,
            })?;
            println!("{}", sim::tas_text(&winner));
            Ok(())
        }
        "brute" => {
            // Exhaustive last-frames enumeration: see brute.rs.
            let level = args.num("--level", 0)?;
            let character = args.num("--char", 0)?;
            let base = sim::read_tas(args.req("--tas")?)?;
            let alphabet: Vec<u8> = match args.get("--alphabet").unwrap_or("launch") {
                "all" => (0..=255u8).collect(),
                // pose/grab bits (up, down, space, shift, ctrl) with left/right/z from the base's
                // last frame: the bits that move a QE launch.
                "launch" => {
                    let keep = base.last().copied().unwrap_or(0) & (1 | 2 | 128);
                    (0..32u8).map(|c| keep | ((c & 1) << 2) | ((c & 2) << 2) | ((c & 4) << 2) | ((c & 8) << 2) | ((c & 16) << 2)).collect()
                }
                other => return Err(format!("unknown alphabet {other}")),
            };
            let template = sim::Sim::new(level, character)?;
            brute::run(brute::Config {
                level,
                character,
                base,
                depth: args.num("--depth", 1)?,
                alphabet,
                extra: args.num("--extra", 2)?,
                min_launch: args.num("--min-launch", 60.0)?,
                target: finish_centre(&template)?,
                threads: args.num("--threads", default_threads().min(2).max(1))?,
                out: args.get("--out").unwrap_or("brute.jsonl").into(),
                limit: args.num("--limit", u64::MAX)?,
            })
        }
        "cannon-rand" => {
            // Decoupled random cannon: see cannon::run_random.
            let masks = sim::read_tas(args.req("--tas")?)?;
            let frame: u32 = args.num("--frame", 0)?;
            let base = prefix(args.num("--level", 0)?, args.num("--char", 0)?, &masks, frame.saturating_sub(1))?;
            let mask = masks.get(base.replay.iteration as usize).copied().unwrap_or(0);
            let pair = |k: &str, d: [f64; 2]| -> Result<[f64; 2]> {
                match args.get(k) {
                    Some(v) => {
                        let p: Vec<f64> = v.split(':').map(|x| x.parse::<f64>().map_err(|_| format!("bad {k}"))).collect::<Result<_>>()?;
                        if p.len() != 2 { return Err(format!("{k} a:b")); }
                        Ok([p[0], p[1]])
                    }
                    None => Ok(d),
                }
            };
            let origin = match args.get("--origin") {
                Some(o) => {
                    let v: Vec<f64> = o.split(',').map(|x| x.parse::<f64>().map_err(|_| format!("bad --origin {o}"))).collect::<Result<_>>()?;
                    Some([v[0], v[1]])
                }
                None => None,
            };
            let n: u64 = args.num("--n", 100000)?;
            let t = std::time::Instant::now();
            let wins = cannon::run_random(&base, mask, n, pair("--dtheta", [0.0, 360.0])?, pair("--dlog", [1.0, 5.0])?, pair("--vtheta", [0.0, 360.0])?, pair("--vlog", [3.0, 6.5])?, origin, match args.get("--w-ratio") { Some(v) => Some(v.parse::<f64>().map_err(|_| "bad --w-ratio")?), None => None }, args.num("--threads", default_threads().min(2).max(1))?, args.num("--seed", 1)?);
            println!("{}", serde_json::json!({"probes": n, "successes": wins.len(), "seconds": t.elapsed().as_secs_f64(), "chest": sim::chest(&base.ctx)}));
            for w in wins { println!("{w}"); }
            Ok(())
        }
        "cannon" => {
            let masks = sim::read_tas(args.req("--tas")?)?;
            let frame: u32 = args.num("--frame", 0)?;
            let base = prefix(args.num("--level", 0)?, args.num("--char", 0)?, &masks, frame.saturating_sub(1))?;
            let mask = masks.get(base.replay.iteration as usize).copied().unwrap_or(0);
            let thetas = range(args.get("--theta").unwrap_or("0:360:0.25"))?;
            let speeds: Vec<f64> = range(args.get("--logv").unwrap_or("2:6:0.02"))?.into_iter().map(|e| 10f64.powf(e)).collect();
            let threads = args.num("--threads", default_threads())?;
            let t = std::time::Instant::now();
            let origin = match args.get("--origin") {
                Some(o) => {
                    let v: Vec<f64> = o.split(',').map(|x| x.parse::<f64>().map_err(|_| format!("bad --origin {o}"))).collect::<Result<_>>()?;
                    Some([v[0], v[1]])
                }
                None => None,
            };
            let wins = cannon::run(&base, mask, &thetas, &speeds, args.num("--k", 1.0)?, origin, threads, args.get("--all").is_some());
            eprintln!("cannon: {} probes in {:.1}s", thetas.len() * speeds.len(), t.elapsed().as_secs_f64());
            cannon::report(&base, &wins, thetas.len() * speeds.len());
            Ok(())
        }
        "run" => {
            let level = args.num("--level", 0)?;
            let character = args.num("--char", 0)?;
            let mut seeds = Vec::new();
            if let Some(list) = args.get("--seeds") {
                for f in list.split(',') {
                    seeds.push(sim::read_tas(f)?);
                }
            }
            let template = sim::Sim::new(level, character)?;
            let target = finish_centre(&template)?;
            let cfg = explore::Config {
                level,
                character,
                seeds,
                run_dir: args.req("--run-dir")?.into(),
                threads: args.num("--threads", default_threads())?,
                hours: args.num("--hours", 10.0)?,
                seed: args.num("--seed", 1)?,
                min_launch: args.num("--min-launch", 20.0)?,
                finish_any: args.num("--finish-any", false)?,
                max_frame: args.num("--max-frame", 900)?,
                rollout: args.num("--rollout", 60)?,
                target,
                site_map: match args.get("--site-map") {
                    Some(path) => std::fs::read_to_string(path)
                        .map_err(|e| format!("{path}: {e}"))?
                        .lines()
                        .filter_map(|l| {
                            let v: Vec<f64> = l.split_whitespace().filter_map(|t| t.parse().ok()).collect();
                            (v.len() == 3).then(|| ((v[0] as i32, v[1] as i32), v[2]))
                        })
                        .collect(),
                    None => Default::default(),
                },
                drive: args.get("--drive").is_some(),
                bounds: match args.get("--bounds") {
                    Some(r) => {
                        let v: Vec<f64> = r.split(',').map(|x| x.parse::<f64>().map_err(|_| format!("bad --bounds {r}"))).collect::<Result<_>>()?;
                        if v.len() != 4 {
                            return Err("--bounds x0,x1,y0,y1".into());
                        }
                        Some([v[0], v[1], v[2], v[3]])
                    }
                    None => None,
                },
                region: match args.get("--region") {
                    Some(r) => {
                        let v: Vec<f64> = r.split(',').map(|x| x.parse::<f64>().map_err(|_| format!("bad --region {r}"))).collect::<Result<_>>()?;
                        if v.len() != 4 {
                            return Err("--region x0,x1,y0,y1".into());
                        }
                        Some([v[0], v[1], v[2], v[3]])
                    }
                    None => None,
                },
            };
            let winner = explore::run(cfg)?;
            println!("{}", sim::tas_text(&winner));
            Ok(())
        }
        "pair" => {
            let winner = pair::run(pair_config(&args)?)?;
            println!("{}", sim::tas_text(&winner));
            Ok(())
        }
        "progress" => {
            let winner=progress::run(progress_config(&args)?)?;
            println!("{}",sim::tas_text(&winner));
            Ok(())
        }
        "perturb" => {
            let level = args.num("--level", 0)?;
            let character = args.num("--char", 0)?;
            let mut bases = Vec::new();
            for f in args.req("--bases")?.split(',') {
                bases.push(sim::read_tas(f)?);
            }
            let template = sim::Sim::new(level, character)?;
            let target = finish_centre(&template)?;
            let cfg = perturb::Config {
                level,
                character,
                bases,
                run_dir: args.req("--run-dir")?.into(),
                threads: args.num("--threads", default_threads())?,
                hours: args.num("--hours", 10.0)?,
                seed: args.num("--seed", 1)?,
                min_launch: args.num("--min-launch", 60.0)?,
                max_frame: args.num("--max-frame", 900)?,
                finish_any: args.num("--finish-any", false)?,
                anchor: args.num("--anchor", 0)?,
                qe_persist: args.num("--qe-persist", 1)?,
                window: args.num("--window", 40)?,
                extra: args.num("--extra", 6)?,
                target,
                bounds: match args.get("--bounds") {
                    Some(r) => {
                        let v: Vec<f64> = r.split(',').map(|x| x.parse::<f64>().map_err(|_| format!("bad --bounds {r}"))).collect::<Result<_>>()?;
                        if v.len() != 4 {
                            return Err("--bounds x0,x1,y0,y1".into());
                        }
                        Some([v[0], v[1], v[2], v[3]])
                    }
                    None => None,
                },
                pool_size: args.num("--pool", 300)?,
                qe_mode: args.get("--qe").is_some(),
                route: args.get("--route").is_some(),
                fc_mode: args.get("--score") == Some("finishcheck"),
            };
            if cfg.fc_mode {
                sim::set_finish_checks(true);
            }
            let winner = perturb::run(cfg)?;
            println!("{}", sim::tas_text(&winner));
            Ok(())
        }
        "route" => {
            // Which world borders the chest hits on the last frame of --tas.
            let template = sim::Sim::new(args.num("--level", 0)?, args.num("--char", 0)?)?;
            let r = analyze::launch_route(&template, &sim::read_tas(args.req("--tas")?)?);
            println!("{}", r.label());
            Ok(())
        }
        "profile" => {
            // Per-frame cost of a list of TAS files: one JSON line per frame
            // (step_us, qe, pairs, contacts) to calibrate the lag cut-off.
            let level = args.num("--level", 0)?;
            let character = args.num("--char", 0)?;
            let template = sim::Sim::new(level, character)?;
            for path in args.req("--tas")?.split(',') {
                let masks = sim::read_tas(path)?;
                let mut s = template.fork();
                s.masks = masks.clone();
                for _ in 0..masks.len() {
                    let f = s.step(0);
                    println!("{}", serde_json::json!({"file": path, "frame": f.frame, "us": f.step_us, "qe": f.qe, "qe_raw": f.qe_raw, "pairs": f.pairs, "contacts": f.contacts, "d": f.displacement}));
                    if f.port_abort { break; }
                }
            }
            Ok(())
        }
        "sensitivity" => {
            // For a string ending on a launch: flip each bit of each of the last --window frames,
            // replay, and classify the outcome (same launch / different launch / no launch).
            let level = args.num("--level", 0)?;
            let character = args.num("--char", 0)?;
            let masks = sim::read_tas(args.req("--tas")?)?;
            let window: usize = args.num("--window", 80)?;
            let min_launch: f64 = args.num("--min-launch", 60.0)?;
            let n = masks.len();
            let from = n.saturating_sub(window);
            let mut base = prefix(level, character, &masks, from as u32)?;
            base.masks = masks.clone();
            let run_tail = |s: &mut sim::Sim| -> Option<[f64; 2]> {
                for _ in 0..(n - s.replay.iteration as usize + 6) {
                    let f = s.step(0);
                    if f.port_abort || !f.alive_after && !f.alive_before {
                        return None;
                    }
                    if f.alive_before && f.displacement >= min_launch {
                        return Some(f.chest_after);
                    }
                }
                None
            };
            let mut w = base.fork();
            let reference = run_tail(&mut w);
            println!("reference launch end {:?}", reference);
            let names = ["left", "right", "up", "down", "space", "shift", "ctrl", "z"];
            for k in from..n {
                let mut row = String::new();
                for bit in 0..8 {
                    w.restore(&base);
                    w.masks = masks.clone();
                    // replay unchanged frames up to k
                    for _ in from..k {
                        w.step(0);
                    }
                    w.masks[k] ^= 1 << bit;
                    let out = run_tail(&mut w);
                    let c = match (out, reference) {
                        (Some(a), Some(b)) if a == b => '=',
                        (Some(_), _) => 'L',
                        (None, _) => '.',
                    };
                    row.push(c);
                }
                println!("frame {:4} input {:08b} {}", k + 1, masks[k].reverse_bits(), row);
            }
            println!("columns: {}", names.join(","));
            Ok(())
        }
        "joints" => {
            // Level joints after N frames of TAS input: type, motor, bodies (position, mass).
            let masks = match args.get("--tas") { Some(t) => sim::read_tas(t)?, None => Vec::new() };
            let s = prefix(args.num("--level", 0)?, args.num("--char", 0)?, &masks, args.num("--frame", 1)?)?;
            let w = &s.ctx.world;
            let mut j = w.joint_list;
            while j != hw_core::NONE {
                let jt = w.joint(j);
                let desc = |b: hw_core::H| {
                    let bd = w.body(b);
                    format!("{}{}@({:.2},{:.2})m={:.3}", if bd.inv_mass == 0.0 { "S" } else { "D" }, b, bd.sweep.c.x, bd.sweep.c.y, bd.mass)
                };
                println!("joint {j} type {} motor {} speed {:.2} torque {:.1} force {:.1} limit {} | {} -- {}",
                    jt.type_, jt.enable_motor.get(), jt.motor_speed, jt.max_motor_torque, jt.max_motor_force, jt.enable_limit.get(), desc(jt.body1), desc(jt.body2));
                j = jt.next;
            }
            Ok(())
        }
        "finish" => {
            let s = sim::Sim::new(args.num("--level", 0)?, args.num("--char", 0)?)?;
            let l = &s.ctx.level;
            for sp in l.specials[..l.count as usize].iter() {
                if sp.kind == hw_data::SpecialKind::FinishLine {
                    println!("finish region x ({:.3}, {:.3}) y ({:.3}, {:.3})", sp.lower.x, sp.upper.x, sp.lower.y, sp.upper.y);
                    let sh = s.ctx.world.shape(sp.shape);
                    println!("  finish shape #{} on body #{} (level body #{}), centroid ({:.2},{:.2}), handlers {:?}, filter cat {} mask {} group {}, sensor {}",
                        sp.shape, sh.body, l.level_body, sh.centroid.x, sh.centroid.y, sh.handlers, sh.filter.category_bits, sh.filter.mask_bits, sh.filter.group_index, sh.is_sensor.get());
                }
            }
            Ok(())
        }
        "sparse" => {
            // Fewest frame-perfect events: keep j base events, add k new ones; see sparse.rs.
            sparse::run(sparse::Config {
                level: args.num("--level", 0)?,
                character: args.num("--char", 0)?,
                base: sim::read_tas(args.req("--tas")?)?,
                limit: args.num("--limit", 0)?,
                run_dir: args.req("--run-dir")?.into(),
                threads: args.num("--threads", default_threads())?,
                hours: args.num("--hours", 10.0)?,
                seed: args.num("--seed", 1)?,
                min_launch: args.num("--min-launch", 60.0)?,
                kmax: args.num("--kmax", 4)?,
                span: args.num("--span", 30)?,
                batch: args.num("--batch", 32)?,
                bits: args.num("--bits", 255)?,
                jmin: args.num("--jmin", 0)?,
                sreach: args.num("--sreach", 40)?,
            })
        }
        "canon" => {
            // Drop every press/release pulse the finish does not need; print the result.
            let (level, character, limit) = (args.num("--level", 0)?, args.num("--char", 0)?, args.num("--limit", 0)?);
            let masks = sim::read_tas(args.req("--tas")?)?;
            let c = sparse::canonicalize(level, character, &masks, limit);
            let f = sparse::finish_of(level, character, &c, limit);
            let (fp, ev) = sparse::fp_count(level, character, &c, limit, args.num("--threads", default_threads())?).ok_or("no finish")?;
            eprintln!("{}", serde_json::json!({"finish": f, "events": ev, "frame_perfect": fp}));
            println!("{}", sim::tas_text(&c));
            Ok(())
        }
        "tail" => {
            // Exhaustive tails with up to k effective-input changes after a cut: see tail.rs.
            tail::run(tail::Config {
                level: args.num("--level", 0)?,
                character: args.num("--char", 0)?,
                base: sim::read_tas(args.req("--tas")?)?,
                cut: args.num("--cut", 0)?,
                window: args.num("--window", 30)?,
                k: args.num("--k", 2)?,
                limit: args.num("--limit", 0)?,
                min_launch: args.num("--min-launch", 60.0)?,
                threads: args.num("--threads", default_threads())?,
                run_dir: args.req("--run-dir")?.into(),
                seed: args.num("--seed", 1)?,
                hold: match args.get("--hold") {
                    Some(v) => Some(match v.strip_prefix("0x") {
                        Some(h) => u8::from_str_radix(h, 16),
                        None => v.parse(),
                    }
                    .map_err(|_| format!("bad --hold {v}"))?),
                    None => None,
                },
                from: args.num("--from", 0)?,
                max_cost: args.num("--max-cost", 0)?,
                bounds: match args.get("--bounds") {
                    Some(r) => {
                        let v: Vec<f64> = r.split(',').map(|x| x.parse::<f64>().map_err(|_| format!("bad --bounds {r}"))).collect::<Result<_>>()?;
                        if v.len() != 4 {
                            return Err("--bounds x0,x1,y0,y1".into());
                        }
                        Some([v[0], v[1], v[2], v[3]])
                    }
                    None => None,
                },
            })
        }
        "cost" => {
            // Aspect cost (structural frame-perfect count) of each string: see events.rs.
            for f in args.req("--tas")?.split(',') {
                let m = sim::read_tas(f)?;
                println!("{}", serde_json::json!({"tas": f, "cost": events::aspect_cost(&m), "events": events::edges(&m).len()}));
            }
            Ok(())
        }
        "events" => {
            // Key presses/releases of a finishing string and how far each can move: see events.rs.
            events::run(
                args.num("--level", 0)?,
                args.num("--char", 0)?,
                args.req("--tas")?,
                args.num("--limit", 0)?,
                args.num("--reach", 30)?,
                args.num("--threads", default_threads())?,
            )
        }
        "fc" => {
            // Every finish check a string triggers: frame, chest distance from the region, position.
            sim::set_finish_checks(true);
            let masks = sim::read_tas(args.req("--tas")?)?;
            let mut s = sim::Sim::new(args.num("--level", 0)?, args.num("--char", 0)?)?;
            s.replay = hw_core::game::session::ReplayState::new(masks.len() as u32);
            s.masks = masks.clone();
            let mut n = 0;
            for _ in 0..masks.len() + 60 {
                let f = s.step(0);
                if f.fc.is_finite() {
                    n += 1;
                    println!("frame {} finish check: chest ({:.2},{:.2}) distance {:.2} m alive_before {} moved {:.1} m", f.frame, f.fc_pos[0], f.fc_pos[1], f.fc, f.alive_before, f.displacement);
                }
                if f.port_abort { break; }
            }
            println!("{n} frames with finish checks");
            Ok(())
        }
        "confirm" => {
            let masks = sim::read_tas(args.req("--tas")?)?;
            let (frame, moved) = explore::confirm(args.num("--level", 0)?, args.num("--char", 0)?, &masks, args.num("--min-launch", 20.0)?)?;
            println!("confirmed: finish frame {frame}, chest moved {moved:.1} m in that frame");
            Ok(())
        }
        _ => Err(format!("{command}: not implemented")),
    }
}

/// Centre of the level's first FinishLine trigger region.
fn finish_centre(s: &sim::Sim) -> Result<[f64; 2]> {
    let l = &s.ctx.level;
    for sp in l.specials[..l.count as usize].iter() {
        if sp.kind == hw_data::SpecialKind::FinishLine {
            return Ok([(sp.lower.x + sp.upper.x) / 2.0, (sp.lower.y + sp.upper.y) / 2.0]);
        }
    }
    Err("level has no FinishLine".into())
}

/// Replay `masks` (open-ended input) for `frames` frames.
pub fn prefix(level: u32, character: u32, masks: &[u8], frames: u32) -> Result<sim::Sim> {
    let mut s = sim::Sim::new(level, character)?;
    s.masks = masks.to_vec();
    for _ in 0..frames {
        let f = s.step(0);
        if f.port_abort {
            return Err(format!("port-only abort at frame {}", f.frame));
        }
    }
    Ok(s)
}

/// "a:b:step" (b exclusive).
fn range(spec: &str) -> Result<Vec<f64>> {
    let p: Vec<f64> = spec.split(':').map(|x| x.parse::<f64>().map_err(|_| format!("bad range {spec}"))).collect::<Result<_>>()?;
    if p.len() != 3 || p[2] <= 0.0 {
        return Err(format!("bad range {spec}"));
    }
    let n = ((p[1] - p[0]) / p[2]).ceil().max(0.0) as usize;
    Ok((0..n).map(|i| p[0] + i as f64 * p[2]).collect())
}

fn main() {
    let result = std::thread::Builder::new()
        .name("hw-search".into())
        .stack_size(64 << 20)
        .spawn(run)
        .map_err(|e| e.to_string())
        .and_then(|h| h.join().map_err(|_| "worker panicked".into()))
        .and_then(|r| r);
    if let Err(e) = result {
        eprintln!("hw-search: {e}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::{parse_repair_pool, parse_resume_pool, Args};

    fn args(values: &[&str]) -> Args {
        Args::parse(values.iter().map(|s| (*s).to_owned())).unwrap()
    }

    #[test]
    fn raw_grip_context_cli_defaults_off_and_requires_raw_analyze_even_if_false() {
        assert!(!args(&[]).num("--raw-grip-context",false).unwrap());
        assert!(super::validate_raw_grip_context_options("analyze",&args(&[])).is_ok());
        for flags in [vec!["--raw-events","--raw-grip-context"],
            vec!["--raw-events","--raw-grip-context=false"],
            vec!["--raw-events","--raw-grip-context","--toi-target-probe"]] {
            assert!(super::validate_raw_grip_context_options("analyze",&args(&flags)).is_ok());
        }
        assert!(args(&["--raw-grip-context"]).num("--raw-grip-context",false).unwrap());
        assert!(!args(&["--raw-grip-context=false"]).num("--raw-grip-context",false).unwrap());
        for flags in [vec!["--raw-grip-context"],vec!["--raw-events=false","--raw-grip-context"],
            vec!["--raw-events","--raw-grip-context=no"]] {
            assert!(super::validate_raw_grip_context_options("analyze",&args(&flags)).is_err());
        }
        for command in ["ceiling","run","qe","perturb","joints",""] {
            assert!(super::validate_raw_grip_context_options(command,
                &args(&["--raw-events","--raw-grip-context=false"])).is_err());
            assert!(super::validate_raw_grip_context_options(command,&args(&[])).is_ok());
        }
    }

    #[test]
    fn finish_any_flag_defaults_off_and_accepts_explicit_bool() {
        assert!(!args(&[]).num("--finish-any", false).unwrap());
        assert!(args(&["--finish-any"]).num("--finish-any", false).unwrap());
        assert!(!args(&["--finish-any=false"]).num("--finish-any", false).unwrap());
        assert!(args(&["--finish-any=no"]).num("--finish-any", false).is_err());
    }

    #[test]
    fn adaptive_window_flag_is_optional_and_accepts_explicit_bool() {
        assert!(!args(&[]).num("--adaptive-window", false).unwrap());
        assert!(args(&["--adaptive-window"]).num("--adaptive-window", false).unwrap());
        assert!(!args(&["--adaptive-window=false"]).num("--adaptive-window", false).unwrap());
        assert!(args(&["--adaptive-window=no"]).num("--adaptive-window", false).is_err());
    }

    #[test]
    fn repeated_launch_parent_flag_defaults_off_and_accepts_explicit_bool() {
        assert!(!args(&[]).num("--retain-repeated-launches", false).unwrap());
        assert!(args(&["--retain-repeated-launches"]).num("--retain-repeated-launches", false).unwrap());
        assert!(!args(&["--retain-repeated-launches=false"]).num("--retain-repeated-launches", false).unwrap());
        assert!(args(&["--retain-repeated-launches=no"]).num("--retain-repeated-launches", false).is_err());
    }

    #[test]
    fn resume_path_is_separate_from_curated_pool_path() {
        let a = args(&["--pool-file", "initial.jsonl", "--resume-pool-file", "checkpoint.jsonl"]);
        assert_eq!(a.get("--pool-file"), Some("initial.jsonl"));
        assert_eq!(a.get("--resume-pool-file"), Some("checkpoint.jsonl"));
    }

    #[test]
    fn resume_members_preserve_current_scores_visits_and_raw_duplicates() {
        let members = parse_resume_pool(concat!(
            "{\"tas\":\"10000000\",\"score_version\":2,\"score\":12.5,\"visits\":18446744073709551615}\n",
            "{\"tas\":\"10000000\",\"score_version\":2,\"score\":0,\"visits\":7}\n",
        )).unwrap();
        assert_eq!(members, vec![(vec![1], 12.5, u64::MAX), (vec![1], 0.0, 7)]);
    }

    #[test]
    fn resume_score_decimals_preserve_the_observed_checkpoint_bits() {
        // These exact decimals shifted one adjacent f64 value when S13 was
        // imported into S15 without serde_json's float_roundtrip feature.
        let cases = [
            ("101.33270601102355", 0x4059_554b_0e27_21d7u64),
            ("110.60882969746983", 0x405b_a6f7_10d5_dddb),
            ("239.19956797953182", 0x406d_e662_dc63_2d60),
        ];
        let mut actual = Vec::new();
        for (decimal, _) in cases {
            let line = format!("{{\"tas\":\"10000000\",\"score_version\":2,\"score\":{decimal},\"visits\":7}}\n");
            let rows = parse_resume_pool(&line).unwrap();
            assert_eq!(rows.len(), 1);
            assert_eq!(rows[0].0, [1]);
            assert_eq!(rows[0].2, 7);
            actual.push(rows[0].1.to_bits());
        }
        assert_eq!(actual, cases.map(|(_, bits)| bits));
    }

    #[test]
    fn serialized_resume_scores_round_trip_through_the_actual_importer() {
        // Exercise both zero signs, subnormals, boundary exponents, normal
        // search scores and the largest finite value. Compare raw bits only.
        let scores = [
            0x0000_0000_0000_0000u64, 0x8000_0000_0000_0000,
            0x0000_0000_0000_0001, 0x000f_ffff_ffff_ffff,
            0x0010_0000_0000_0000, 0x3fb9_9999_9999_999a,
            0x3ff0_0000_0000_0000, 0xbff0_0000_0000_0000,
            0x4059_554b_0e27_21d7, 0x405b_a6f7_10d5_dddb,
            0x406d_e662_dc63_2d60, 0x7fef_ffff_ffff_ffff,
        ];
        for bits in scores {
            let mut score = f64::from_bits(bits);
            for _ in 0..3 {
                let line = serde_json::json!({"tas": "10000000", "score_version": 2,
                    "score": score, "visits": u64::MAX}).to_string();
                let rows = parse_resume_pool(&line).unwrap();
                assert_eq!(rows.len(), 1);
                assert_eq!(rows[0].0, [1]);
                assert_eq!(rows[0].2, u64::MAX);
                score = rows[0].1;
                assert_eq!(score.to_bits(), bits);
            }
        }
    }

    #[test]
    fn resume_members_use_existing_score_and_json_defaults() {
        let members = parse_resume_pool(concat!(
            "not json\n",
            "{\"tas\":\"\"}\n",
            "{\"score_version\":2}\n",
            "{\"tas\":\"01000000\",\"score_version\":1,\"score\":0,\"visits\":-1}\n",
            "{\"tas\":\"00100000\",\"score_version\":2,\"score\":null,\"visits\":1.5}\n",
            "{\"tas\":\"00010000\",\"score\":0}\n",
        )).unwrap();
        assert_eq!(members, vec![(vec![2], 400.0, 0), (vec![4], 400.0, 0), (vec![8], 400.0, 0)]);
        assert!(parse_resume_pool("{\"tas\":\"invalid\"}").is_err());
    }

    #[test]
    fn setup_share_defaults_off_and_repair_path_is_separate() {
        assert_eq!(args(&[]).num("--setup-share", 0.0).unwrap(), 0.0);
        assert_eq!(args(&[]).num("--bundle-share", 0.0).unwrap(), 0.0);
        let a = args(&["--setup-share", "0.25", "--resume-pool-file", "pool.jsonl", "--repair-pool-file", "repair.jsonl"]);
        assert_eq!(a.num("--setup-share", 0.0).unwrap(), 0.25);
        assert_eq!(a.get("--repair-pool-file"), Some("repair.jsonl"));
        assert_eq!(a.get("--resume-pool-file"), Some("pool.jsonl"));
        assert_eq!(args(&["--bundle-share", "0.75"]).num("--bundle-share", 0.0).unwrap(), 0.75);
    }

    #[test]
    fn repair_members_preserve_ancestry_generation_raw_bits_and_duplicates() {
        let row = "{\"ancestor\":18446744073709551615,\"generation\":4294967295,\"tas\":\"10000000 01000000\"}";
        let members = parse_repair_pool(&format!("\n{row}\n{row}\n")).unwrap();
        assert_eq!(members, vec![(u64::MAX, vec![1, 2], u32::MAX); 2]);
    }

    #[test]
    fn repair_members_never_silently_drop_nonempty_bad_rows() {
        for row in [
            "not json", "null", "{}",
            "{\"ancestor\":-1,\"generation\":1,\"tas\":\"10000000\"}",
            "{\"ancestor\":0,\"generation\":0,\"tas\":\"10000000\"}",
            "{\"ancestor\":0,\"generation\":4294967296,\"tas\":\"10000000\"}",
            "{\"ancestor\":0,\"generation\":1.5,\"tas\":\"10000000\"}",
            "{\"ancestor\":0,\"generation\":1,\"tas\":\" \"}",
            "{\"ancestor\":0,\"generation\":1,\"tas\":\"1000000\"}",
        ] {
            assert!(parse_repair_pool(&format!("\n{row}")).unwrap_err().starts_with("repair pool line 2:"));
        }
    }

    #[test]
    fn pair_cli_fixed_defaults_parse_without_simulation() {
        let bits = super::sim::tas_text(&vec![17;139]);
        let c = super::pair_config(&args(&["--bases", &bits, "--run-dir", "unused"])).unwrap();
        assert_eq!((c.level,c.character,c.threads,c.hours,c.window,c.extra,c.pool_size),(45,3,super::default_threads(),2.0,239,24,500));
        assert_eq!((c.lag_ms,c.lag_frames),(15.0,3));
        assert_eq!(c.bases,[vec![17;139]]);
    }
    #[test]
    fn pair_cli_zero_workers_requires_explicit_zero_hours() {
        let bits=super::sim::tas_text(&vec![17;139]);
        assert!(super::pair_config(&args(&["--bases",&bits,"--run-dir","unused","--threads","0"])).is_err());
        let c=super::pair_config(&args(&["--bases",&bits,"--run-dir","unused","--threads","0","--hours","0"])).unwrap();
        assert_eq!((c.threads,c.hours),(0,0.0));
    }
    #[test]
    fn pair_cli_rejects_wrong_profile_mode_flags_and_bad_workers() {
        let bits=super::sim::tas_text(&vec![17;139]);
        for (flag,value) in [("--char","2"),("--level","3214"),("--threads","19"),("--hours","NaN"),
            ("--anchor","0"),("--qe-persist","1"),("--max-frame","900"),("--qe","true"),
            ("--lag-ms","NaN"),("--lag-ms","-1"),("--lag-frames","0")] {
            assert!(super::pair_config(&args(&["--bases",&bits,"--run-dir","unused",flag,value])).is_err());
        }
    }
    #[test]
    fn ceiling_cli_has_full_opening_and_rejects_other_profiles() {
        let bits=super::sim::tas_text(&vec![17;299]);
        let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused"])).unwrap();
        assert_eq!((c.level,c.character,c.threads,c.window,c.pool_size),(45,3,super::default_threads(),299,200));
        assert_eq!((c.edit_from,c.edit_to),(0,299));
        assert_eq!((c.launch_pool_size,c.launch_min_frame,c.launch_min_speed),(0,128,1024.0));
        assert_eq!(c.bases,[vec![17;299]]);
        for (flag,value) in [("--char","2"),("--level","13196"),("--threads","19"),
            ("--hours","NaN"),("--window","300"),("--extra","299"),("--anchor","60"),
            ("--objective","launch"),("--lag-ms","-1"),("--lag-frames","0")] {
            assert!(super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused",flag,value])).is_err());
        }
        let short=super::sim::tas_text(&vec![17;298]);
        assert!(super::ceiling_config(&args(&["--bases",&short,"--run-dir","unused"])).is_err());
        assert!(super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--threads","0"])).is_err());
        let zero=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--threads","0","--hours","0"])).unwrap();
        assert_eq!((zero.threads,zero.hours),(0,0.0));
    }
    #[test] fn ceiling_enumeration_cli_default_shards_and_profile_rejection() {
        let bits=super::sim::tas_text(&vec![17;299]);let base=["--bases",bits.as_str(),"--run-dir","unused"];
        assert!(super::ceiling_config(&args(&base)).unwrap().enumeration.is_none());
        let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-cells","284,285"])).unwrap();
        assert_eq!(c.enumeration,Some(super::ceiling::EnumerationConfig {cells:vec![284,285],low5:false,low5_values:vec![],start:0,end:65536}));
        for flags in [["--enumerate-cells","284,284"],["--enumerate-cells","284,299"],["--enumerate-cells","284"],
            ["--enumerate-start","0"],["--enumerate-end","65536"]] {
            let a=args(&["--bases",&bits,"--run-dir","unused",flags[0],flags[1]]);
            assert!(super::ceiling_config(&a).is_err());assert!(super::validate_ceiling_launch_options("pair",&a).is_err());}
        let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-cells","285,284",
            "--enumerate-start","256","--enumerate-end","512"])).unwrap();
        let spec=c.enumeration.unwrap();assert_eq!(spec.start,256);assert_eq!(spec.end,512);
    }
    #[test] fn ceiling_low5_cli_defaults_domains_shards_and_native_thread_cap() {
        let bits=super::sim::tas_text(&vec![156;299]);
        for (cells,total) in [("285",32),("285,279",1024),("280,277,285",32768),("277,278,279,284,285",33_554_432)] {
            let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-low5-cells",cells])).unwrap();
            assert_eq!(c.threads,17);let spec=c.enumeration.unwrap();assert!(spec.low5);assert_eq!((spec.start,spec.end),(0,total));
        }
        let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-low5-cells","285,279,277",
            "--enumerate-start","1023","--enumerate-end","1025","--endpoint-min-frame","274","--endpoint-pool","0"])).unwrap();
        let spec=c.enumeration.unwrap();assert_eq!(spec.cells,vec![285,279,277]);assert_eq!((spec.start,spec.end),(1023,1025));
        assert_eq!((c.endpoint_pool_size,c.endpoint_min_frame),(0,274));
        for cells in ["","284,284","299","0,1,2,3,4,5","284,","-1","x"] {
            let a=args(&["--bases",&bits,"--run-dir","unused","--enumerate-low5-cells",cells]);
            assert!(super::ceiling_config(&a).is_err());assert!(super::validate_ceiling_launch_options("pair",&a).is_err());
        }
        for (flag,value) in [("--threads","18"),("--enumerate-start","32"),("--enumerate-end","33")] {
            assert!(super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-low5-cells","285",flag,value])).is_err());
        }
        assert!(super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-low5-cells","285",
            "--enumerate-cells","284,285"])).is_err());
        assert!(super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-cells","284,285","--threads","18"])).is_ok());
    }
    #[test] fn ceiling_low5_root_rejects_noncanonical_bytes_without_changing_legacy_tas_parser() {
        let bits=super::sim::tas_text(&vec![156;299]);assert_eq!(super::parse_ceiling_low5_root(bits.as_bytes()).unwrap(),vec![156;299]);
        for bad in [format!("{bits}\n"),format!(" {bits}"),format!("\u{feff}{bits}"),bits[..2391].to_string(),format!("{bits}0")] {
            assert!(super::parse_ceiling_low5_root(bad.as_bytes()).is_err());
        }
        let mut invalid=bits.into_bytes();invalid[17]=b'2';assert!(super::parse_ceiling_low5_root(&invalid).is_err());
        let spaced=super::sim::tas_text(&vec![156;299])+"\n";assert_eq!(super::sim::parse_tas(&spaced).unwrap(),vec![156;299]);
    }
    #[test] fn ceiling_low5_alphabet_cli_preserves_order_and_sets_default_domain_and_shards() {
        let bits=super::sim::tas_text(&vec![156;299]);
        let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-low5-cells","280,282,284",
            "--enumerate-low5-values","17,18,20,24,28"])).unwrap();let spec=c.enumeration.unwrap();
        assert_eq!(c.threads,17);assert_eq!(spec.cells,[280,282,284]);assert_eq!(spec.low5_values,[17,18,20,24,28]);
        assert_eq!((spec.start,spec.end),(0,125));
        let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-low5-cells","285,280",
            "--enumerate-low5-values","28,17,20","--enumerate-start","3","--enumerate-end","6"])).unwrap();
        let spec=c.enumeration.unwrap();assert_eq!(spec.low5_values,[28,17,20]);assert_eq!((spec.start,spec.end),(3,6));
        let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-low5-cells","277,278,279,284,285",
            "--enumerate-low5-values","20"])).unwrap();assert_eq!(c.enumeration.unwrap().end,1);
        let all=(0..32).map(|v|v.to_string()).collect::<Vec<_>>().join(",");
        let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-low5-cells","280,282,284",
            "--enumerate-low5-values",&all])).unwrap();let spec=c.enumeration.unwrap();
        assert_eq!(spec.low5_values,(0..32).collect::<Vec<_>>());assert_eq!(spec.end,32768);
    }
    #[test] fn ceiling_low5_alphabet_cli_rejects_malformed_values_and_invalid_custom_shards() {
        let bits=super::sim::tas_text(&vec![156;299]);
        for values in ["","true","17,17","32","255","256","-1","17,","17,,18","17, 18","x","17.0"] {
            assert!(super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-low5-cells","280",
                "--enumerate-low5-values",values])).is_err());
        }
        let too_many=(0..33).map(|v|v.to_string()).collect::<Vec<_>>().join(",");
        assert!(super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-low5-cells","280",
            "--enumerate-low5-values",&too_many])).is_err());
        for (flag,value) in [("--enumerate-start","5"),("--enumerate-end","6"),("--enumerate-end","0")] {
            assert!(super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-low5-cells","280",
                "--enumerate-low5-values","17,18,20,24,28",flag,value])).is_err());
        }
    }
    #[test] fn ceiling_low5_alphabet_cli_requires_low5_cells_and_ceiling_command() {
        let bits=super::sim::tas_text(&vec![156;299]);
        let a=args(&["--bases",&bits,"--run-dir","unused","--enumerate-low5-values","17,18,20,24,28"]);
        assert!(super::ceiling_config(&a).is_err());assert!(super::ceiling_enumeration_config(&a).is_err());
        let a=args(&["--bases",&bits,"--run-dir","unused","--enumerate-cells","280,284","--enumerate-low5-values","17"]);
        assert!(super::ceiling_config(&a).is_err());
        for command in ["pair","progress","run","qe","analyze","perturb","cannon"] {
            assert!(super::validate_ceiling_launch_options(command,&a).is_err());
        }
        assert!(super::validate_ceiling_launch_options("ceiling",&a).is_ok());
        let legacy=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-cells","280,284"])).unwrap();
        let spec=legacy.enumeration.unwrap();assert!(!spec.low5);assert!(spec.low5_values.is_empty());assert_eq!((spec.start,spec.end),(0,65536));
        let default=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--enumerate-low5-cells","280"])).unwrap();
        assert_eq!(default.enumeration.unwrap().low5_values,(0..32).collect::<Vec<_>>());
    }
    #[test]
    fn ceiling_cli_pressure_is_default_off_and_validates_even_when_disabled() {
        let bits=super::sim::tas_text(&vec![17;299]);
        let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused"])).unwrap();
        assert_eq!((c.pressure_pool_size,c.pressure_min_pairs,c.pressure_min_frame,c.pressure_max_frame),(0,1024,1,299));
        for (flag,value) in [("--pressure-pool","3"),("--pressure-pool","1028"),("--pressure-min-pairs","0"),
            ("--pressure-min-pairs","4097"),("--pressure-min-frame","0"),("--pressure-max-frame","300")] {
            let a=args(&["--bases",&bits,"--run-dir","unused",flag,value]);assert!(super::ceiling_config(&a).is_err());
            assert!(super::validate_ceiling_launch_options("pair",&a).is_err());}
        let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused","--pressure-pool","32"])).unwrap();
        assert_eq!(c.pressure_pool_size,32);
    }
    #[test]
    fn ceiling_cli_endpoint_archive_is_default_off_and_bounded() {
        let bits=super::sim::tas_text(&vec![17;299]);
        let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused"])).unwrap();
        assert_eq!((c.endpoint_pool_size,c.endpoint_min_frame),(0,1));
        for (pool,frame) in [("0","299"),("4","1"),("64","283"),("1024","299")] {
            let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused",
                "--endpoint-pool",pool,"--endpoint-min-frame",frame])).unwrap();
            assert_eq!((c.endpoint_pool_size,c.endpoint_min_frame),
                (pool.parse::<usize>().unwrap(),frame.parse::<u32>().unwrap()));
        }
        for (flag,value) in [("--endpoint-pool","-1"),("--endpoint-pool","1"),("--endpoint-pool","6"),
            ("--endpoint-pool","1028"),("--endpoint-pool","bad"),("--endpoint-min-frame","0"),
            ("--endpoint-min-frame","300"),("--endpoint-min-frame","-1"),("--endpoint-min-frame","bad")] {
            let a=args(&["--bases",&bits,"--run-dir","unused",flag,value]);
            assert!(super::ceiling_config(&a).is_err(),"accepted {flag}={value}");
        }
        for command in ["analyze","qe","run","pair","progress","perturb","confirm",""] {
            for flag in ["--endpoint-pool","--endpoint-min-frame"] {
                let a=args(&["--bases",&bits,"--run-dir","unused",flag,"1"]);
                assert!(super::validate_ceiling_launch_options(command,&a).is_err());
                assert!(super::validate_ceiling_launch_options("ceiling",&a).is_ok());
                assert!(super::pair_config(&a).is_err());
                assert!(super::progress_config(&a).is_err());
            }
        }
    }
    #[test]
    fn ceiling_cli_focused_edits_keep_the_full_replay_horizon() {
        let bits=super::sim::tas_text(&vec![17;299]);
        let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused",
            "--edit-from","128","--edit-to","143"])).unwrap();
        assert_eq!((c.edit_from,c.edit_to),(128,143));
        assert_eq!((c.window,c.bases[0].len()),(299,299));
        for (from,to) in [("143","143"),("144","143"),("0","300"),("-1","143"),("0","0")] {
            assert!(super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused",
                "--edit-from",from,"--edit-to",to])).is_err());
        }
    }
    #[test]
    fn ceiling_cli_launch_archive_is_opt_in_with_inclusive_limits() {
        let bits=super::sim::tas_text(&vec![17;299]);
        let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused",
            "--launch-pool","64","--launch-min-frame","143","--launch-min-speed","2048"])).unwrap();
        assert_eq!((c.launch_pool_size,c.launch_min_frame,c.launch_min_speed),(64,143,2048.0));
        assert_eq!((c.window,c.bases[0].len()),(299,299));
        for (pool,frame,speed) in [("0","128","1024"),("1","1","0"),("1024","299","0.5")] {
            let c=super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused",
                "--launch-pool",pool,"--launch-min-frame",frame,"--launch-min-speed",speed])).unwrap();
            assert_eq!((c.launch_pool_size,c.launch_min_frame,c.launch_min_speed),
                (pool.parse::<usize>().unwrap(),frame.parse::<u32>().unwrap(),speed.parse::<f64>().unwrap()));
        }
    }
    #[test]
    fn ceiling_cli_launch_archive_rejects_bad_values_even_when_disabled() {
        let bits=super::sim::tas_text(&vec![17;299]);
        for (flag,value) in [("--launch-pool","-1"),("--launch-pool","1025"),("--launch-pool","bad"),
            ("--launch-min-frame","0"),("--launch-min-frame","300"),("--launch-min-frame","-1"),
            ("--launch-min-frame","bad"),("--launch-min-speed","-1"),("--launch-min-speed","NaN"),
            ("--launch-min-speed","inf"),("--launch-min-speed","-inf"),("--launch-min-speed","bad")] {
            assert!(super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused",flag,value])).is_err(),
                "accepted {flag}={value}");
        }
        for flag in ["--launch-pool","--launch-min-frame","--launch-min-speed"] {
            assert!(super::ceiling_config(&args(&["--bases",&bits,"--run-dir","unused",flag])).is_err());
        }
    }
    #[test]
    fn ceiling_launch_archive_options_are_rejected_by_other_commands() {
        let bits=super::sim::tas_text(&vec![17;299]);
        let none=args(&[]);
        for command in ["analyze","qe","brute","cannon-rand","cannon","run","pair","progress",
            "perturb","route","profile","sensitivity","joints","finish","sparse","canon","tail",
            "cost","events","fc","confirm",""] {
            assert!(super::validate_ceiling_launch_options(command,&none).is_ok());
            for flag in ["--launch-pool","--launch-min-frame","--launch-min-speed"] {
                let options=args(&["--bases",&bits,"--run-dir","unused",flag,"0"]);
                assert!(super::validate_ceiling_launch_options(command,&options).is_err(),"accepted {flag} for {command}");
                assert!(super::validate_ceiling_launch_options("ceiling",&options).is_ok());
                assert!(super::pair_config(&options).is_err());
                assert!(super::progress_config(&options).is_err());
            }
        }
    }
    #[test]
    fn progress_cli_uses_fixed_profile_and_preserves_full_unscored_bases() {
        let bits=super::sim::tas_text(&vec![17;289]);
        let c=super::progress_config(&args(&["--bases",&bits,"--run-dir","unused"])).unwrap();
        assert_eq!((c.level,c.character,c.threads,c.hours,c.window,c.extra,c.pool_size),(45,3,super::default_threads(),2.0,239,24,500));
        assert_eq!(c.bases,[vec![17;289]]);
        assert_eq!(c.objective,super::progress::Objective::Finish);
        assert_eq!(c.diversity_pool,0);
        assert!(!c.route_checkpoints);
        let fan=super::progress_config(&args(&["--bases",&bits,"--run-dir","unused","--objective","fan"])).unwrap();
        assert_eq!(fan.objective,super::progress::Objective::Fan);
        let launch=super::progress_config(&args(&["--bases",&bits,"--run-dir","unused","--objective","launch"])).unwrap();
        assert_eq!(launch.objective,super::progress::Objective::Launch);
        let reserve=super::progress_config(&args(&["--bases",&bits,"--run-dir","unused","--objective","launch","--diversity-pool","64"])).unwrap();
        assert_eq!(reserve.diversity_pool,64);
        assert!(!reserve.route_checkpoints);
        let checkpoints=super::progress_config(&args(&["--bases",&bits,"--run-dir","unused","--objective","launch",
            "--diversity-pool","64","--route-checkpoints"])).unwrap();
        assert!(checkpoints.route_checkpoints);
        for capacity in ["0","1","3","63","1025"] {
            assert!(super::progress_config(&args(&["--bases",&bits,"--run-dir","unused","--objective","launch",
                "--diversity-pool",capacity,"--route-checkpoints"])).is_err());
        }
        assert!(super::progress_config(&args(&["--bases",&bits,"--run-dir","unused","--objective","fan",
            "--diversity-pool","64","--route-checkpoints"])).is_err());
        for objective in ["finish","fan"] {
            assert!(super::progress_config(&args(&["--bases",&bits,"--run-dir","unused","--objective",objective,"--diversity-pool","64"])).is_err());
        }
        assert!(super::progress_config(&args(&["--bases",&bits,"--run-dir","unused","--objective","launch","--diversity-pool","1025"])).is_err());
        assert!(super::progress_config(&args(&["--bases",&bits,"--run-dir","unused","--objective","unknown"])).is_err());
        assert!(super::pair_config(&args(&["--bases",&bits,"--run-dir","unused","--objective","fan"])).is_err());
        assert!(super::progress_config(&args(&["--bases",&bits,"--run-dir","unused","--threads","0"])).is_err());
        let c=super::progress_config(&args(&["--bases",&bits,"--run-dir","unused","--threads","0","--hours","0"])).unwrap();
        assert_eq!((c.threads,c.hours),(0,0.0));
        for(flag,value) in [("--char","2"),("--level","3214"),("--max-frame","900"),("--anchor","0"),("--qe","true")] {
            assert!(super::progress_config(&args(&["--bases",&bits,"--run-dir","unused",flag,value])).is_err());
        }
    }
}
