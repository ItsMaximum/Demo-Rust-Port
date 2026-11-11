//! Dad/Obstacle finish-focus distance windows; no engine-state mutation.
//! No simulation state or physics is changed. Distance zero is not a finish.
pub const ANCHOR: usize = 60;
pub const WINDOW: usize = 6;
pub const MAX_FRAME: u32 = 299;

#[derive(Clone, Copy, Debug)]
pub struct Observation {
    pub frame_before: u32, pub frame_after: u32,
    pub iteration_before: u32, pub iteration_after: u32,
    pub input_allowed_before: bool, pub input_allowed_after: bool,
    pub mask_matches_requested: bool,
    pub locked_before: bool, pub locked_after: bool,
    pub dad_alive_before: bool, pub dad_alive_after: bool,
    pub chest_live_before: bool, pub chest_live_after: bool,
    pub chest_handle_before: u32, pub chest_handle_after: u32,
    pub chest_before: [f64; 2], pub chest_after: [f64; 2],
    pub chest_velocity_after: [f64; 2],
    pub chest_frozen_before: bool, pub chest_frozen_after: bool,
    // These must come directly from ctx.level.focus, not Actors' active alias.
    pub focus_live_before: bool, pub focus_live_after: bool,
    pub focus_handle_before: u32, pub focus_handle_after: u32,
    pub focus_before: [f64; 2], pub focus_after: [f64; 2],
    pub focus_frozen_before: bool, pub focus_frozen_after: bool,
    pub finish_distance_after: f64,
    pub site: u32, pub port_abort: bool, pub lagging: bool, pub finished: bool,
    pub host_diagnostics_complete: bool,
    // Telemetry only. Neither high pair counts nor QE improves distance fitness.
    pub pairs_after: i32, pub qe: f64, pub qe_raw: f64,
}

pub fn in_world(p: [f64; 2]) -> bool {
    p[0].is_finite() && p[1].is_finite()
        && p[0] > -4.0 && p[0] < 324.0 && p[1] > -84.0 && p[1] < 164.0
}

#[derive(Clone, Copy, Debug)]
pub struct WindowScore {
    /// Smaller is better: the worst distance of six clean consecutive samples.
    pub worst_distance_m: f64,
    pub distances: [f64; WINDOW], pub pairs: [i32; WINDOW],
    pub qe: [f64; WINDOW], pub qe_raw: [f64; WINDOW],
    pub focus_positions: [[f64; 2]; WINDOW],
    pub chest_velocities: [[f64; 2]; WINDOW], pub world_frames: [u32; WINDOW],
    pub focus_handle: u32, pub chest_handle: u32,
    pub focus_position: [f64; 2], pub chest_position: [f64; 2],
}

#[derive(Clone, Copy, Debug)]
pub struct Decision {
    /// Always confirm this independently before other decisions; not a winner claim.
    pub observed_finish: bool,
    pub observed_pair_pool_exhausted: bool,
    pub clean: bool,
    pub score: Option<WindowScore>,
    pub continue_rollout: bool,
}

pub struct ProgressWindow {
    samples: [Option<Observation>; WINDOW], count: usize, next: usize,
    // Actual arena handles within one replay, not snapshot-local serialized IDs.
    last: Option<(u32, u32, u32, u32)>,
    credit_after_iteration: u32, planned_len: u32,
}
impl ProgressWindow {
    pub fn new(planned_len: usize, earliest_changed_index: usize) -> Result<Self, &'static str> {
        if !(ANCHOR + 2..=MAX_FRAME as usize).contains(&planned_len) {
            return Err("progress requires 62..299 complete entries");
        }
        if earliest_changed_index >= planned_len { return Err("no changed planned input"); }
        Ok(Self { samples: [None; WINDOW], count: 0, next: 0, last: None,
            credit_after_iteration: ANCHOR.max(earliest_changed_index) as u32,
            planned_len: planned_len as u32 })
    }
    fn reset(&mut self) { self.count = 0; self.next = 0; self.last = None; }
    pub fn observe(&mut self, o: Observation) -> Decision {
        let observed_finish = o.finished && o.frame_after <= MAX_FRAME;
        let clean = o.frame_before.checked_add(1) == Some(o.frame_after)
            && o.frame_after <= MAX_FRAME
            && o.iteration_before >= self.credit_after_iteration && o.iteration_before < self.planned_len
            && o.iteration_before.checked_add(1) == Some(o.iteration_after) && o.iteration_after <= self.planned_len
            && o.input_allowed_before && o.input_allowed_after && o.mask_matches_requested
            && !o.locked_before && !o.locked_after && o.site == 0 && !o.port_abort && !o.lagging
            && o.host_diagnostics_complete
            && o.dad_alive_before && o.dad_alive_after && o.chest_live_before && o.chest_live_after
            && o.focus_live_before && o.focus_live_after
            && o.chest_handle_before == o.chest_handle_after && o.focus_handle_before == o.focus_handle_after
            && !o.chest_frozen_before && !o.chest_frozen_after && !o.focus_frozen_before && !o.focus_frozen_after
            && in_world(o.chest_before) && in_world(o.chest_after) && in_world(o.focus_before) && in_world(o.focus_after)
            && o.finish_distance_after.is_finite() && o.finish_distance_after >= 0.0;
        let mut score = None;
        if !clean { self.reset(); } else {
            if let Some((frame, iteration, focus, chest)) = self.last {
                if frame.checked_add(1) != Some(o.frame_after) || iteration.checked_add(1) != Some(o.iteration_after)
                    || focus != o.focus_handle_after || chest != o.chest_handle_after { self.reset(); }
            }
            self.samples[self.next] = Some(o); self.next = (self.next + 1) % WINDOW;
            self.count = (self.count + 1).min(WINDOW);
            self.last = Some((o.frame_after, o.iteration_after, o.focus_handle_after, o.chest_handle_after));
            if self.count == WINDOW {
                let samples: [Observation; WINDOW] = std::array::from_fn(|i| self.samples[(self.next + i) % WINDOW].unwrap());
                let distances = samples.map(|s| s.finish_distance_after);
                let worst_distance_m = distances.iter().copied().fold(0.0_f64, f64::max);
                score = Some(WindowScore { worst_distance_m, distances,
                    pairs: samples.map(|s| s.pairs_after), qe: samples.map(|s| s.qe), qe_raw: samples.map(|s| s.qe_raw),
                    focus_positions: samples.map(|s| s.focus_after),
                    chest_velocities: samples.map(|s| s.chest_velocity_after), world_frames: samples.map(|s| s.frame_after),
                    focus_handle: o.focus_handle_after, chest_handle: o.chest_handle_after,
                    focus_position: o.focus_after, chest_position: o.chest_after });
            }
        }
        Decision { observed_finish, observed_pair_pool_exhausted: o.site == 2, clean, score,
            // A lock, missing focus or frozen body gives no fitness but is not
            // repaired or treated as death. Continue under the world-frame bound.
            continue_rollout: o.frame_after < MAX_FRAME && !o.port_abort
                && o.dad_alive_after && o.chest_live_after && in_world(o.chest_after) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn sample(n: u32, distance: f64) -> Observation {
        Observation { frame_before: n - 1, frame_after: n, iteration_before: n - 1, iteration_after: n,
            input_allowed_before: true, input_allowed_after: true, mask_matches_requested: true,
            locked_before: false, locked_after: false, dad_alive_before: true, dad_alive_after: true,
            chest_live_before: true, chest_live_after: true, chest_handle_before: 10, chest_handle_after: 10,
            chest_before: [29.0, 74.0], chest_after: [29.0, 74.0], chest_frozen_before: false, chest_frozen_after: false,
            chest_velocity_after:[0.0,0.0],
            focus_live_before: true, focus_live_after: true, focus_handle_before: 10, focus_handle_after: 10,
            focus_before: [29.0, 74.0], focus_after: [29.0, 74.0], focus_frozen_before: false, focus_frozen_after: false,
            finish_distance_after: distance, site: 0, port_abort: false, lagging: false, finished: false,host_diagnostics_complete:true,
            pairs_after: 100, qe: 1.0, qe_raw: 1.0 }
    }
    #[test] fn one_close_spike_cannot_replace_six_frame_motion() {
        let mut w = ProgressWindow::new(299, 60).unwrap();
        for n in 61..66 { assert!(w.observe(sample(n, if n == 63 { 0.0 } else { 90.0 })).score.is_none()); }
        assert_eq!(w.observe(sample(66, 90.0)).score.unwrap().worst_distance_m.to_bits(), 90.0_f64.to_bits());
        for n in 67..=72 { w.observe(sample(n, 30.0)); }
        assert_eq!(w.observe(sample(73, 30.0)).score.unwrap().worst_distance_m.to_bits(), 30.0_f64.to_bits());
    }
    #[test] fn dead_child_qe_and_pair_exhaustion_cannot_improve_stationary_dad() {
        let mut w = ProgressWindow::new(299, 60).unwrap();
        for n in 61..=66 { let mut o=sample(n,90.0); o.qe_raw=8.91e9; o.qe=102138.0; o.pairs_after=4096;
            let d=w.observe(o); if n==66 { assert_eq!(d.score.unwrap().worst_distance_m.to_bits(),90.0_f64.to_bits()); } }
        let mut abort=sample(67,0.0); abort.site=2; abort.locked_after=true;
        let d=w.observe(abort); assert!(d.observed_pair_pool_exhausted); assert!(d.score.is_none());
    }
    #[test] fn primary_chest_liveness_is_separate_from_a_live_focus_alias() {
        let mut w=ProgressWindow::new(299,60).unwrap();
        for n in 61..66 {w.observe(sample(n,10.0));}
        let mut o=sample(66,0.0); o.chest_live_after=false; o.focus_handle_before=44; o.focus_handle_after=44;
        o.focus_after=[120.0,57.0]; assert!(w.observe(o).score.is_none());
        for n in 67..72 {assert!(w.observe(sample(n,10.0)).score.is_none());}
        assert!(w.observe(sample(72,10.0)).score.is_some());
    }
    #[test] fn focus_identity_change_requires_a_new_complete_window() {
        let mut w=ProgressWindow::new(299,60).unwrap(); for n in 61..66 {w.observe(sample(n,10.0));}
        let mut o=sample(66,1.0); o.focus_handle_before=20; o.focus_handle_after=20;
        assert!(w.observe(o).score.is_none());
        for n in 67..71 {let mut o=sample(n,1.0);o.focus_handle_before=20;o.focus_handle_after=20;assert!(w.observe(o).score.is_none());}
        let mut o=sample(71,1.0);o.focus_handle_before=20;o.focus_handle_after=20;assert!(w.observe(o).score.is_some());
    }
    #[test] fn post_change_eof_and_no_progress_cannot_borrow_previous_fitness() {
        let mut w=ProgressWindow::new(286,285).unwrap();
        for n in 61..=286 {assert!(w.observe(sample(n,1.0)).score.is_none());}
        for n in 287..=299 {let mut o=sample(n,0.0);o.iteration_before=287;o.iteration_after=287;
            assert!(w.observe(o).score.is_none());}
        assert!(ProgressWindow::new(100,100).is_err());
    }
    #[test] fn unclean_frame_resets_even_when_positions_look_ideal() {
        for fault in 0..9 {let mut w=ProgressWindow::new(299,60).unwrap();for n in 61..66 {w.observe(sample(n,1.0));}
            let mut o=sample(66,0.0);match fault {0=>o.dad_alive_after=false,1=>o.focus_live_after=false,
                2=>o.focus_frozen_after=true,3=>o.lagging=true,4=>o.input_allowed_after=false,
                5=>o.mask_matches_requested=false,6=>o.finish_distance_after=f64::NAN,7=>o.host_diagnostics_complete=false,
                _=>o.iteration_after=o.iteration_before}
            assert!(w.observe(o).score.is_none());for n in 67..72 {assert!(w.observe(sample(n,1.0)).score.is_none());}
            assert!(w.observe(sample(72,1.0)).score.is_some());}
    }
    #[test] fn finish_before_gates_and_launch_continuation_are_preserved() {
        let mut w=ProgressWindow::new(299,60).unwrap();let mut o=sample(286,0.0);
        o.finished=true;o.site=2;o.locked_after=true;o.dad_alive_after=false;
        let d=w.observe(o);assert!(d.observed_finish);assert!(d.observed_pair_pool_exhausted);assert!(d.score.is_none());
        assert!(w.observe(sample(287,20.0)).continue_rollout);assert!(!w.observe(sample(299,10.0)).continue_rollout);
        let mut late=sample(300,0.0);late.finished=true;assert!(!w.observe(late).observed_finish);
    }
    #[test] fn zero_region_distance_is_not_a_finish_latch() {
        let mut w=ProgressWindow::new(299,60).unwrap();
        for n in 61..=66 {let d=w.observe(sample(n,0.0));assert!(!d.observed_finish);
            if n==66 {assert_eq!(d.score.unwrap().worst_distance_m.to_bits(),0.0_f64.to_bits());}}
    }
}
