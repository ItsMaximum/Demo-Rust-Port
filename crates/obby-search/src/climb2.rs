//! `climb2`: coordinate ascent on single inputs for the held-lock open-sky score (`osky3`). From
//! a cached state before entry `--from`, every alternative input (poses x Space, eject bits kept)
//! at each entry in [from, to) is played to the onset and scored; any improvement is kept and
//! passes repeat until none improves. Writes the best input to `--out`.
use crate::sim::{tas_text, Sim, Skyline};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

pub fn climb2_cmd(tas: &str, from: usize, to: usize, max_frame: u32, threads: usize, out: &str) -> Result<(), String> {
    let mut masks = crate::sim::read_tas(tas)?;
    let template = Sim::new(crate::LEVEL, crate::CHAR)?;
    let sky = Skyline::new(&template.ctx);
    let mut base = template.fork();
    base.masks = masks[..from.min(masks.len())].to_vec();
    while (base.replay.iteration as usize) < from {
        base.step(0);
    }
    let score_of = |m: &[u8], s: &mut Sim, br: &mut Sim| -> (f64, u32) {
        s.restore(&base);
        s.masks.clear();
        s.masks.extend_from_slice(m);
        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| crate::window::osky3_eval(s, br, &template, &sky, max_frame, 0)));
        match r {
            Ok(v) => v,
            Err(_) => {
                std::mem::forget(std::mem::replace(s, template.fork()));
                (f64::NEG_INFINITY, 0)
            }
        }
    };
    let (mut cur, mut cur_on) = {
        let (mut s, mut br) = (template.fork(), template.fork());
        score_of(&masks, &mut s, &mut br)
    };
    eprintln!("climb2: start score {:.1} onset {} (entries {}..{})", cur, cur_on, from, to);
    const POSE: [u8; 9] = [0, 1, 2, 4, 8, 1 | 4, 2 | 4, 1 | 8, 2 | 8];
    let mut pass = 0;
    loop {
        pass += 1;
        let mut improved = false;
        for i in from..to.min(masks.len()) {
            let ej = masks[i] & (32 | 64 | 128);
            let cands: Vec<u8> = POSE.iter().flat_map(|&p| [p | ej, p | 16 | ej]).filter(|&m| m != masks[i]).collect();
            let next = AtomicUsize::new(0);
            let results = Mutex::new(Vec::new());
            std::thread::scope(|scope| {
                for _ in 0..threads {
                    let (next, results, cands, masks, score_of, template) = (&next, &results, &cands, &masks, &score_of, &template);
                    std::thread::Builder::new().stack_size(32 << 20).spawn_scoped(scope, move || {
                        let (mut s, mut br) = (template.fork(), template.fork());
                        loop {
                            let k = next.fetch_add(1, Ordering::Relaxed);
                            if k >= cands.len() {
                                break;
                            }
                            let mut m = masks.clone();
                            m[i] = cands[k];
                            let (v, on) = score_of(&m, &mut s, &mut br);
                            results.lock().unwrap().push((v, on, cands[k]));
                        }
                    }).unwrap();
                }
            });
            let mut r = results.into_inner().unwrap();
            r.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            if let Some(&(v, on, m)) = r.last() {
                if v > cur {
                    masks[i] = m;
                    cur = v;
                    cur_on = on;
                    improved = true;
                    eprintln!("climb2: pass {} entry {} -> mask {} score {:.1} onset {}", pass, i, m, cur, cur_on);
                    std::fs::write(out, tas_text(&masks)).ok();
                }
            }
        }
        if !improved {
            break;
        }
    }
    eprintln!("climb2: done, score {:.1} onset {}", cur, cur_on);
    std::fs::write(out, tas_text(&masks)).ok();
    Ok(())
}
