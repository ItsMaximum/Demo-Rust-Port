//! `hold`: aftermath sweep for pairmax onsets. For each onset lineage (input up to and including
//! the onset frame), play every combination of a held input for H frames after the onset and a
//! release input after that, to frame 299, and report finishes and the closest living approach.
//! Holding Space (grab) keeps an ejected Dad's lock going; releasing lets a frame complete.
use crate::sim::{tas_text, Sim, Skyline};
use crate::window::finish_distance;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

pub fn hold_cmd(files: &[String], max_hold: u32, threads: usize, out_dir: &str) -> Result<(), String> {
    let mut work: Vec<Vec<u8>> = Vec::new();
    let mut uniq = std::collections::HashSet::new();
    for f in files {
        let text = std::fs::read_to_string(f).map_err(|e| format!("{f}: {e}"))?;
        if f.ends_with(".tas") {
            let m = crate::sim::parse_tas(&text)?;
            if uniq.insert(m.clone()) {
                work.push(m);
            }
            continue;
        }
        for line in text.lines() {
            if let Some(m) = crate::tas_field(line) {
                if uniq.insert(m.clone()) {
                    work.push(m);
                }
            }
        }
    }
    std::fs::create_dir_all(out_dir).ok();
    let template = Sim::new(crate::LEVEL, crate::CHAR)?;
    let sky = Skyline::new(&template.ctx);
    const HOLDS: [u8; 6] = [16, 17, 18, 20, 24, 0];
    const RELEASES: [u8; 4] = [0, 1, 2, 4];
    let jobs: Vec<(usize, u32, u8, u8)> = (0..work.len())
        .flat_map(|i| (0..=max_hold).flat_map(move |h| HOLDS.iter().flat_map(move |&hm| RELEASES.iter().map(move |&rm| (i, h, hm, rm)))))
        .collect();
    let next = AtomicUsize::new(0);
    let best: Mutex<Vec<(f64, u32, usize, u32, u8, u8, u32, bool)>> = Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for _ in 0..threads {
            let (next, best, work, jobs, template, sky) = (&next, &best, &work, &jobs, &template, &sky);
            std::thread::Builder::new().stack_size(32 << 20).spawn_scoped(scope, move || {
                let mut s = template.fork();
                loop {
                    let k = next.fetch_add(1, Ordering::Relaxed);
                    if k >= jobs.len() {
                        break;
                    }
                    let (i, h, hm, rm) = jobs[k];
                    let mut masks = work[i].clone();
                    for _ in 0..h {
                        masks.push(hm);
                    }
                    s.restore(template);
                    s.masks = masks;
                    let mut near = f64::INFINITY;
                    let mut fin = 0u32;
                    let mut locked = 0u32;
                    let mut sky_seen = false;
                    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        while s.frame() < 299 {
                            let f = s.step(rm);
                            if f.port_abort || f.step_us > 200_000 {
                                break;
                            }
                            if f.finished && f.alive_before {
                                fin = f.frame;
                                break;
                            }
                            if f.site != 0 {
                                locked += 1;
                                continue;
                            }
                            if !f.alive_after {
                                break;
                            }
                            if sky.above(f.chest_after) {
                                sky_seen = true;
                            }
                            near = near.min(finish_distance(f.chest_after));
                        }
                    }));
                    if r.is_err() {
                        s = template.fork();
                        continue;
                    }
                    let mut b = best.lock().unwrap();
                    b.push((if fin != 0 { -1.0 } else { near }, fin, i, h, hm, rm, locked, sky_seen));
                    if fin != 0 {
                        let used = s.masks[..s.replay.iteration as usize].to_vec();
                        let path = format!("{}/finish-{}-{}-{}-{}.tas", out_dir, i, h, hm, rm);
                        std::fs::write(&path, tas_text(&used)).ok();
                        eprintln!("FINISH frame {} lineage {} hold {}x{} release {} -> {}", fin, i, h, hm, rm, path);
                    }
                }
            }).unwrap();
        }
    });
    let mut b = best.into_inner().unwrap();
    b.sort_by(|a, c| a.0.partial_cmp(&c.0).unwrap_or(std::cmp::Ordering::Equal));
    for r in b.iter().take(25) {
        println!(
            "{{\"near\":{:.2},\"finish\":{},\"lineage\":{},\"hold\":{},\"hold_mask\":{},\"release\":{},\"locked\":{},\"sky\":{}}}",
            r.0, r.1, r.2, r.3, r.4, r.5, r.6, r.7
        );
    }
    Ok(())
}
