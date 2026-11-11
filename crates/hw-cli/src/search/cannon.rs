//! `hw-search cannon`: from a real replay state, overwrite the chest's end-of-solve velocity v
//! and position (c0 + k·v·dt) at one frame and let the unchanged engine finish that step.
//! Maps which launch angles and speeds reach the finish from that state.
use super::analyze::shape_label;
use super::sim::Sim;
use hw_core::game::character::CHEST;
use hw_core::lab::{self, Event, Injection};
use serde_json::json;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

pub struct Probe {
    pub theta_deg: f64,
    pub speed: f64,
}

pub struct Outcome {
    pub theta_deg: f64,
    pub speed: f64,
    pub finished: bool,
    pub chain: Vec<String>,
    pub end: [f64; 2],
    /// AbortSite of the frame (0 = none).
    pub site: u32,
}

/// Launch direction is measured in screen terms: 0° = right, 90° = up (Box2D y is down).
pub fn fire(base: &Sim, work: &mut Sim, mask: u8, theta_deg: f64, speed: f64, k: f64, origin: Option<[f64; 2]>, want_chain: bool) -> Outcome {
    work.restore(base);
    let chest = work.ctx.character.bodies[CHEST];
    let (s, c) = theta_deg.to_radians().sin_cos();
    let v = [speed * c, -speed * s];
    lab::arm(&[chest]);
    lab::inject(Injection { body: chest, origin, displacement: [k * v[0] / 30.0, k * v[1] / 30.0], velocity: v, angular: None, angle_delta: None });
    let f = work.step(mask);
    lab::disarm();
    let events = lab::take_events();
    let mut chain = Vec::new();
    if want_chain {
        for e in &events {
            match *e {
                Event::Toi { shape1, shape2, body1, toi, .. } => {
                    let other = if body1 == chest { shape2 } else { shape1 };
                    chain.push(format!("{:.4}:{}", toi, shape_label(&work.ctx, other)));
                }
                Event::AfterToi { body, c, v, .. } if body == chest => chain.push(format!("  solved->({:.2},{:.2}) v({:.0},{:.0})", c[0], c[1], v[0], v[1])),
                Event::Finish { x, y, inside, dead, .. } => chain.push(format!("FINISH-CHECK({x:.2},{y:.2}) inside={inside} dead={dead}")),
                _ => {}
            }
        }
    }
    Outcome { theta_deg, speed, finished: f.finished && f.alive_before, chain, end: f.chest_after, site: f.site }
}

/// Decoupled launch: first-leg displacement (sweep c - c0) and post-solve velocity chosen
/// independently, as in real QE launches (joint correction sets c, the solve sets v).
pub fn fire_split(base: &Sim, work: &mut Sim, mask: u8, displacement: [f64; 2], velocity: [f64; 2], origin: Option<[f64; 2]>, spin: Option<f64>) -> (bool, [f64; 2], Vec<String>) {
    work.restore(base);
    let chest = work.ctx.character.bodies[CHEST];
    lab::arm(&[chest]);
    // spin: angular velocity (rad/s); the swept angle is set consistently (da = w * dt).
    lab::inject(Injection { body: chest, origin, displacement, velocity, angular: spin, angle_delta: spin.map(|w| w / 30.0) });
    let f = work.step(mask);
    lab::disarm();
    let events = lab::take_events();
    let mut chain = Vec::new();
    if f.finished {
        for e in &events {
            if let Event::Toi { shape1, shape2, body1, toi, .. } = *e {
                let other = if body1 == chest { shape2 } else { shape1 };
                chain.push(format!("{:.4}:{}", toi, shape_label(&work.ctx, other)));
            }
        }
    }
    (f.finished && f.alive_before, f.chest_after, chain)
}

/// Random decoupled sampling: angles uniform in the given ranges (deg, 0 = right, 90 = up),
/// lengths/speeds log-uniform. Returns (probes, successes as JSON lines).
pub fn run_random(base: &Sim, mask: u8, n: u64, dtheta: [f64; 2], dlog: [f64; 2], vtheta: [f64; 2], vlog: [f64; 2], origin: Option<[f64; 2]>, w_ratio: Option<f64>, threads: usize, seed: u64) -> Vec<String> {
    let next = AtomicUsize::new(0);
    let wins = Mutex::new(Vec::new());
    // Research: HW_CANNON_ENDS=file records every probe's end position and launch parameters.
    let ends_path = std::env::var_os("HW_CANNON_ENDS");
    let record_ends = ends_path.is_some();
    let ends: Mutex<Vec<String>> = Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for w in 0..threads {
            let (wins, next, ends) = (&wins, &next, &ends);
            std::thread::Builder::new()
                .stack_size(16 << 20)
                .spawn_scoped(scope, move || {
                    let mut work = base.fork();
                    let mut st = seed ^ (w as u64 + 1).wrapping_mul(0x9e37_79b9_7f4a_7c15);
                    let mut unit = || {
                        st = st.wrapping_add(0x9e37_79b9_7f4a_7c15);
                        let mut z = st;
                        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
                        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
                        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
                    };
                    while (next.fetch_add(1, Ordering::Relaxed) as u64) < n {
                        let td = (dtheta[0] + unit() * (dtheta[1] - dtheta[0])).to_radians();
                        let ld = 10f64.powf(dlog[0] + unit() * (dlog[1] - dlog[0]));
                        let tv = (vtheta[0] + unit() * (vtheta[1] - vtheta[0])).to_radians();
                        let sv = 10f64.powf(vlog[0] + unit() * (vlog[1] - vlog[0]));
                        let d = [ld * td.cos(), -ld * td.sin()];
                        let v = [sv * tv.cos(), -sv * tv.sin()];
                        let spin = w_ratio.map(|r| r * sv * if unit() < 0.5 { 1.0 } else { -1.0 });
                        let (ok, end, chain) = fire_split(base, &mut work, mask, d, v, origin, spin);
                        if record_ends {
                            ends.lock().unwrap().push(format!("{} {} {:.3} {:.4e} {:.3} {:.4e}", end[0], end[1], td.to_degrees(), ld, tv.to_degrees(), sv));
                        }
                        if ok {
                            wins.lock().unwrap().push(json!({"disp_theta": td.to_degrees(), "disp_len": ld, "vel_theta": tv.to_degrees(), "speed": sv, "end": end, "chain": chain}).to_string());
                        }
                    }
                })
                .unwrap();
        }
    });
    if let Some(path) = ends_path {
        std::fs::write(path, ends.into_inner().unwrap().join("
") + "
").ok();
    }
    wins.into_inner().unwrap()
}

pub fn run(base: &Sim, mask: u8, thetas: &[f64], speeds: &[f64], k: f64, origin: Option<[f64; 2]>, threads: usize, all: bool) -> Vec<Outcome> {
    let probes: Vec<Probe> = speeds
        .iter()
        .flat_map(|&speed| thetas.iter().map(move |&theta_deg| Probe { theta_deg, speed }))
        .collect();
    let next = AtomicUsize::new(0);
    let wins = Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for _ in 0..threads {
            std::thread::Builder::new()
                .stack_size(16 << 20)
                .spawn_scoped(scope, || {
                    let mut work = base.fork();
                    loop {
                        let i = next.fetch_add(1, Ordering::Relaxed);
                        let Some(p) = probes.get(i) else { break };
                        let o = fire(base, &mut work, mask, p.theta_deg, p.speed, k, origin, all);
                        if all {
                            wins.lock().unwrap().push(o);
                        } else if o.finished {
                            let o = fire(base, &mut work, mask, p.theta_deg, p.speed, k, origin, true);
                            wins.lock().unwrap().push(o);
                        }
                    }
                })
                .unwrap();
        }
    });
    let mut w = wins.into_inner().unwrap();
    w.sort_by(|a, b| a.speed.total_cmp(&b.speed).then(a.theta_deg.total_cmp(&b.theta_deg)));
    w
}

pub fn report(base: &Sim, wins: &[Outcome], probes: usize) {
    let c = super::sim::chest(&base.ctx);
    let n = wins.iter().filter(|o| o.finished).count();
    println!(
        "{}",
        json!({"frame": base.frame() + 1, "chest": c, "probes": probes, "successes": n,
               "fraction": n as f64 / probes as f64})
    );
    for o in wins {
        println!("{}", json!({"theta": o.theta_deg, "speed": o.speed, "finished": o.finished, "site": o.site, "end": o.end, "chain": o.chain}));
    }
}
