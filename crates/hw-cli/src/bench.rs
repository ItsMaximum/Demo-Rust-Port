//! `hw bench`: CPU replay throughput over the verification corpus (8 official TAS strings
//! and the 108 generated coverage cases), single-threaded and across worker threads.
//! Only `session::create` and `session::step_bits` + `end_frame` run; no observers.
use hw_core::game::session::{self, ReplayState};
use hw_core::{Ctx, Flow, Input};
use hw_real::Real;
use serde_json::json;
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

type Result<T> = std::result::Result<T, String>;

struct Case {
    name: String,
    level: u32,
    character: u32,
    masks: Vec<u8>,
    frames: u32,
}

fn masks(bits: &str) -> Vec<u8> {
    let bits: Vec<u8> = bits.bytes().filter(|b| !b.is_ascii_whitespace()).collect();
    bits.chunks(8)
        .map(|c| c.iter().enumerate().fold(0, |m, (k, &b)| m | (((b == b'1') as u8) << k)))
        .collect()
}

fn corpus(set: &str) -> Result<Vec<Case>> {
    let mut cases = Vec::new();
    if set == "official" || set == "all" {
        for t in hw_data::TAS_STRINGS {
            cases.push(Case {
                name: t.file.into(),
                level: t.level,
                character: t.character,
                masks: masks(t.bits),
                frames: t.frames + 60,
            });
        }
    }
    if set == "generated" || set == "all" {
        let text = std::fs::read_to_string("reference/tas/extra/plan.json")
            .map_err(|e| format!("reference/tas/extra/plan.json (run from the workspace root): {e}"))?;
        let plan: serde_json::Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
        for case in plan["cases"].as_array().ok_or("plan.json has no cases")? {
            let path = case["tas"].as_str().ok_or("case without tas")?;
            let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
            cases.push(Case {
                name: case["name"].as_str().unwrap_or_default().into(),
                level: case["level"].as_u64().unwrap_or(0) as u32,
                character: case["character"].as_u64().unwrap_or(0) as u32,
                masks: masks(&text),
                frames: case["frames"].as_u64().unwrap_or(0) as u32,
            });
        }
    }
    if cases.is_empty() {
        return Err("--cases must be official, generated or all".into());
    }
    Ok(cases)
}

/// Replays one case; returns (frames stepped, seconds spent stepping).
fn replay<R: Real>(ctx: &mut Ctx<R>, case: &Case) -> Result<(u64, f64)> {
    *ctx = Ctx::new();
    if session::create(ctx, case.level, case.character) != Flow::Continue {
        return Err(format!("{}: session::create failed", case.name));
    }
    let mut state = ReplayState::new(case.masks.len() as u32);
    let start = Instant::now();
    for _ in 0..case.frames {
        let mask = if state.input_allowed.get() && state.iteration < case.masks.len() as u32 {
            case.masks[state.iteration as usize] as u32
        } else {
            0
        };
        let _ = session::step_bits(ctx, &mut state, Input::from_mask(mask));
        ctx.end_frame();
    }
    Ok((case.frames as u64, start.elapsed().as_secs_f64()))
}

/// Runs `jobs` (case indices) on `threads` workers; returns (frames, stepping seconds, wall seconds).
fn run_threads<R: Real + Send>(cases: &[Case], jobs: &[usize], threads: usize) -> Result<(u64, f64, f64)> {
    let next = AtomicUsize::new(0);
    let start = Instant::now();
    let results: Vec<Result<(u64, f64)>> = std::thread::scope(|s| {
        let handles: Vec<_> = (0..threads)
            .map(|_| {
                std::thread::Builder::new()
                    .stack_size(64 << 20)
                    .spawn_scoped(s, || {
                        let mut ctx: Box<Ctx<R>> = Box::new(Ctx::new());
                        let (mut frames, mut seconds) = (0, 0.0);
                        loop {
                            let i = next.fetch_add(1, Ordering::Relaxed);
                            let Some(&job) = jobs.get(i) else { break };
                            let (f, t) = replay(&mut ctx, &cases[job])?;
                            frames += f;
                            seconds += t;
                        }
                        Ok((frames, seconds))
                    })
                    .unwrap()
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap_or_else(|_| Err("bench worker panicked".into()))).collect()
    });
    let wall = start.elapsed().as_secs_f64();
    let (mut frames, mut seconds) = (0, 0.0);
    for r in results {
        let (f, t) = r?;
        frames += f;
        seconds += t;
    }
    Ok((frames, seconds, wall))
}

pub fn run(args: &BTreeMap<String, String>) -> Result<()> {
    let cases = corpus(args.get("--cases").map(String::as_str).unwrap_or("all"))?;
    let real = args.get("--real").map(String::as_str).unwrap_or("f64");
    let available = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1);
    let threads: usize = match args.get("--threads") {
        Some(v) => v.parse().map_err(|_| "bad --threads")?,
        None => available,
    };
    let repeat: usize = match args.get("--repeat") {
        Some(v) => v.parse().map_err(|_| "bad --repeat")?,
        None => 1,
    };
    let math = if cfg!(feature = "portable-math") { "portable" } else { "host-x87" };
    let jobs: Vec<usize> = (0..repeat).flat_map(|_| 0..cases.len()).collect();
    let corpus_frames: u64 = cases.iter().map(|c| c.frames as u64).sum();
    println!(
        "bench: {} cases, {corpus_frames} frames per pass, real {real}, math {math}, target {}",
        cases.len(),
        std::env::consts::ARCH
    );
    let measure = |threads: usize| -> Result<(u64, f64, f64)> {
        match real {
            "f64" => run_threads::<f64>(&cases, &jobs, threads),
            _ => Err("--real must be f64".into()),
        }
    };
    let (f1, s1, w1) = measure(1)?;
    let single = f1 as f64 / s1;
    println!("1 thread: {f1} frames in {w1:.2}s wall; {single:.0} frames/s ({:.1} us/frame)", 1e6 / single);
    let (fn_, sn, wn) = measure(threads)?;
    let aggregate = fn_ as f64 / wn;
    println!(
        "{threads} threads: {fn_} frames in {wn:.2}s wall; {aggregate:.0} frames/s aggregate ({:.0} frames/s per thread while stepping, {:.2}x one thread)",
        fn_ as f64 / sn,
        aggregate / single
    );
    if let Some(out) = args.get("--out") {
        let value = json!({
            "command": "bench",
            "cases": cases.len(),
            "repeat": repeat,
            "frames_per_pass": corpus_frames,
            "real": real,
            "math": math,
            "target": std::env::consts::ARCH,
            "available_parallelism": available,
            "single_thread": { "frames": f1, "step_seconds": s1, "wall_seconds": w1, "frames_per_second": single },
            "multi_thread": { "threads": threads, "frames": fn_, "step_seconds": sn, "wall_seconds": wn, "frames_per_second": aggregate },
        });
        std::fs::write(out, serde_json::to_string_pretty(&value).unwrap()).map_err(|e| format!("{out}: {e}"))?;
        println!("wrote {out}");
    }
    Ok(())
}
