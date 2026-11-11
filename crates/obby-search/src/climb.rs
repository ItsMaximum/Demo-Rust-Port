//! `climb`: coordinate ascent on single input entries. From a cached state before entry `--from`,
//! try every alternative input (poses x Space, eject bits kept) at each entry in [from, to) and
//! keep any change that raises the objective; repeat passes until none improves. Objective: the
//! best pair count (in-frame peak in the instrumented build) on a frame whose chest state passes
//! the zone test (`--zone start|corridor`), counting a pairmax onset in the zone as 1_000_000.
use crate::sim::{tas_text, Sim};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

fn zone_ok(zone: &str, c: [f64; 2], v: [f64; 2]) -> bool {
    match zone {
        "corridor" => c[0] > 24.5 && c[0] < 35.5 && c[1] > 67.3 && c[1] < 72.0 && v[0] > 5.0 && v[1] < -8.0,
        "any" => c[0].is_finite(),
        _ => c[0] > 0.0 && c[0] < 26.0 && c[1] > 62.0 && c[1] < 84.0 && v[1] < -15.0,
    }
}

fn score(s: &mut Sim, base: &Sim, masks: &[u8], start_iter: usize, end_frame: u32, zone: &str) -> i64 {
    s.restore(base);
    s.masks.clear();
    s.masks.extend_from_slice(masks);
    let _ = start_iter;
    let mut best = 0i64;
    while s.frame() < end_frame {
        let f = s.step(0);
        if f.port_abort || f.step_us > 80_000 {
            break;
        }
        if f.site == 2 && f.alive_before && zone_ok(zone, f.chest_before, [10.0, -20.0]) {
            return 1_000_000;
        }
        if f.site == 0 && f.alive_after && zone_ok(zone, f.chest_after, f.chest_v) {
            best = best.max(f.pairs as i64);
        }
        if f.site == 0 && !f.alive_after {
            break;
        }
    }
    best
}

pub fn climb_cmd(tas: &str, from: usize, to: usize, extra: u32, zone: &str, threads: usize, out: &str) -> Result<(), String> {
    let mut masks = crate::sim::read_tas(tas)?;
    if masks.len() < to {
        masks.resize(to, 0);
    }
    let template = Sim::new(crate::LEVEL, crate::CHAR)?;
    // Cache the state just before entry `from`.
    let mut base = template.fork();
    base.masks = masks[..from].to_vec();
    while (base.replay.iteration as usize) < from {
        base.step(0);
    }
    let end_frame = base.frame() + (to - from) as u32 + extra;
    let mut cur = {
        let mut s = template.fork();
        score(&mut s, &base, &masks, from, end_frame, zone)
    };
    eprintln!("climb: start score {} (entries {}..{}, end frame {})", cur, from, to, end_frame);
    const POSE: [u8; 9] = [0, 1, 2, 4, 8, 1 | 4, 2 | 4, 1 | 8, 2 | 8];
    let mut pass = 0;
    loop {
        pass += 1;
        let mut improved = false;
        for i in from..to {
            let ej = masks[i] & (32 | 64 | 128);
            let cands: Vec<u8> = POSE.iter().flat_map(|&p| [p | ej, p | 16 | ej]).filter(|&m| m != masks[i]).collect();
            let next = AtomicUsize::new(0);
            let results = Mutex::new(Vec::new());
            std::thread::scope(|scope| {
                for _ in 0..threads {
                    let (next, results, cands, masks, base, template) = (&next, &results, &cands, &masks, &base, &template);
                    std::thread::Builder::new().stack_size(32 << 20).spawn_scoped(scope, move || {
                        let mut s = template.fork();
                        loop {
                            let k = next.fetch_add(1, Ordering::Relaxed);
                            if k >= cands.len() {
                                break;
                            }
                            let mut m = masks.clone();
                            m[i] = cands[k];
                            let v = score(&mut s, base, &m, from, end_frame, zone);
                            results.lock().unwrap().push((v, cands[k]));
                        }
                    }).unwrap();
                }
            });
            let mut r = results.into_inner().unwrap();
            r.sort();
            if let Some(&(v, m)) = r.last() {
                if v > cur {
                    masks[i] = m;
                    cur = v;
                    improved = true;
                    eprintln!("climb: pass {} entry {} -> mask {} score {}", pass, i, m, cur);
                    std::fs::write(out, tas_text(&masks)).ok();
                    if cur >= 1_000_000 {
                        eprintln!("climb: pairmax onset in zone");
                        return Ok(());
                    }
                }
            }
        }
        if !improved {
            break;
        }
    }
    eprintln!("climb: done, score {}", cur);
    std::fs::write(out, tas_text(&masks)).ok();
    Ok(())
}
