//! Clean pair-occupancy windows and exact raw/prefix guards, without physics mutation.
//! Call observe once per actual Sim::step, with separately captured pre/post state.
pub const ANCHOR: usize = 60;
pub const WINDOW: usize = 6;
pub const MAX_FRAME: u32 = 299;
pub const MAX_PAIRS: i32 = 4096;

#[derive(Clone, Copy, Debug)]
pub struct Observation {
    pub frame_before: u32,
    pub frame_after: u32,
    pub iteration_before: u32,
    pub iteration_after: u32,
    pub input_allowed_before: bool,
    pub input_allowed_after: bool,
    pub mask_matches_requested: bool,
    pub locked_before: bool,
    pub locked_after: bool,
    pub alive_before: bool,
    pub alive_after: bool,
    pub chest_alive_before: bool,
    pub chest_alive_after: bool,
    pub chest_before: [f64; 2],
    pub chest_after: [f64; 2],
    pub pairs_after: i32,
    pub site: u32,
    pub port_abort: bool,
    pub lagging: bool,
    pub finished: bool,
    pub displacement: f64,
}

fn in_world(p: [f64; 2]) -> bool {
    p[0].is_finite() && p[1].is_finite()
        && p[0] > -4.0 && p[0] < 324.0 && p[1] > -84.0 && p[1] < 164.0
}

#[derive(Debug, PartialEq, Eq)]
pub struct Decision {
    /// Always send this event to independent exact replay BEFORE acting on other fields.
    /// It is an observation, not a verified winner; port-abort confirmations may fail.
    pub observed_finish: bool,
    pub observed_pair_pool_exhausted: bool,
    pub sustained_pairs: Option<i32>,
    /// Original Flash throws and locks are not repaired. They may continue under the
    /// fixed world-frame ceiling, but never receive clean occupancy credit.
    pub continue_rollout: bool,
}

pub struct PairWindow {
    values: [i32; WINDOW],
    count: usize,
    next: usize,
    last: Option<(u32, u32)>,
    credit_after_iteration: u32,
    planned_len: u32,
}

impl PairWindow {
    /// earliest_changed_index is zero-based. A child's scoring window starts at or
    /// after the changed entry; unchanged earlier high-count windows cannot reward it.
    pub fn new(planned_len: usize, earliest_changed_index: usize) -> Result<Self, &'static str> {
        if !(ANCHOR + 2..=MAX_FRAME as usize).contains(&planned_len) {
            return Err("pair candidates require 62..299 complete input entries");
        }
        if earliest_changed_index >= planned_len {
            return Err("no changed input in the candidate");
        }
        Ok(Self { values: [0; WINDOW], count: 0, next: 0, last: None,
            credit_after_iteration: ANCHOR.max(earliest_changed_index) as u32,
            planned_len: planned_len as u32 })
    }

    pub fn counts(&self) -> Option<[i32; WINDOW]> {
        if self.count != WINDOW { return None; }
        let mut out = [0; WINDOW];
        for (i, value) in out.iter_mut().enumerate() { *value = self.values[(self.next + i) % WINDOW]; }
        Some(out)
    }
    pub fn last_was_clean(&self) -> bool { self.last.is_some() }
    fn reset(&mut self) { self.count = 0; self.next = 0; self.last = None; }

    pub fn observe(&mut self, o: Observation) -> Decision {
        let observed_finish = o.finished && o.frame_after <= MAX_FRAME;
        let eligible = o.frame_before.checked_add(1) == Some(o.frame_after)
            && o.frame_after <= MAX_FRAME
            && o.iteration_before >= self.credit_after_iteration
            && o.iteration_before < self.planned_len
            && o.iteration_before.checked_add(1) == Some(o.iteration_after)
            && o.iteration_after <= self.planned_len
            && o.input_allowed_before && o.input_allowed_after && o.mask_matches_requested
            && !o.locked_before && !o.locked_after && o.site == 0 && !o.port_abort && !o.lagging
            && o.alive_before && o.alive_after && o.chest_alive_before && o.chest_alive_after
            && in_world(o.chest_before) && in_world(o.chest_after)
            && (0..=MAX_PAIRS).contains(&o.pairs_after);
        let mut sustained_pairs = None;
        if !eligible {
            self.reset();
        } else {
            if let Some((f, i)) = self.last {
                if f.checked_add(1) != Some(o.frame_after) || i.checked_add(1) != Some(o.iteration_after) {
                    self.reset();
                }
            }
            self.values[self.next] = o.pairs_after;
            self.next = (self.next + 1) % WINDOW;
            self.count = (self.count + 1).min(WINDOW);
            self.last = Some((o.frame_after, o.iteration_after));
            if self.count == WINDOW { sustained_pairs = self.values.iter().copied().min(); }
        }
        Decision { observed_finish, observed_pair_pool_exhausted: o.site == 2,
            sustained_pairs,
            continue_rollout: o.frame_after < MAX_FRAME && !o.port_abort
                && o.alive_after && o.chest_alive_after && in_world(o.chest_after) }
    }
}

/// CPU lag is a separate search cutoff, never a physics throw or a negative finish.
/// Every laggy frame already resets PairWindow; this bounds consecutive slow work.
#[derive(Default)]
pub struct LagGuard { consecutive: u32 }
impl LagGuard {
    pub fn observe(&mut self, lagging: bool, limit: u32) -> bool {
        self.consecutive = if lagging { self.consecutive.saturating_add(1) } else { 0 };
        self.consecutive >= limit
    }
    pub fn consecutive(&self) -> u32 { self.consecutive }
}

/// Exact current-pool TAS equality, not score equality or endpoint equality.
/// The floor applies only when the unprotected pool is full. Equal-count novel
/// literals are eligible; eviction must give the new equal-count member a place.
pub fn eligible_for_pool(raw: &[u8], score: i32, current: &[Vec<u8>], full_pool_floor: Option<i32>) -> bool {
    (ANCHOR + 2..=MAX_FRAME as usize).contains(&raw.len())
        && !current.iter().any(|m| m.as_slice() == raw)
        && full_pool_floor.map_or(true, |floor| score >= floor)
}

/// Apply after a suffix-only mutation as a defensive check. Never repair a
/// corrupted prefix silently; reject the proposal and surface an implementation bug.
pub fn valid_anchored_child(base: &[u8], child: &[u8]) -> bool {
    base.len() >= ANCHOR + 2 && child.len() >= ANCHOR + 2 && child.len() <= MAX_FRAME as usize
        && base[..ANCHOR] == child[..ANCHOR]
}

#[cfg(test)]
mod tests {
    use super::*;
    fn frame(n: u32, pairs: i32) -> Observation {
        Observation { frame_before: n - 1, frame_after: n, iteration_before: n - 1,
            iteration_after: n, input_allowed_before: true, input_allowed_after: true,
            mask_matches_requested: true, locked_before: false, locked_after: false,
            alive_before: true, alive_after: true, chest_alive_before: true, chest_alive_after: true,
            chest_before: [28.0, 74.0], chest_after: [28.0, 74.0], pairs_after: pairs,
            site: 0, port_abort: false, lagging: false, finished: false, displacement: 0.0 }
    }
    #[test] fn six_actual_frames_use_the_minimum_and_slide() {
        let mut p = PairWindow::new(299, 60).unwrap();
        for (n, c) in [(61, 10), (62, 12), (63, 11), (64, 900), (65, 14)] {
            assert_eq!(p.observe(frame(n, c)).sustained_pairs, None);
        }
        assert_eq!(p.observe(frame(66, 13)).sustained_pairs, Some(10));
        assert_eq!(p.counts(),Some([10,12,11,900,14,13]));
        assert_eq!(p.observe(frame(67, 16)).sustained_pairs, Some(11));
        assert_eq!(p.counts(),Some([12,11,900,14,13,16]));
    }
    #[test] fn single_frame_spike_has_no_high_score() {
        let mut p = PairWindow::new(299, 60).unwrap();
        for n in 61..=66 { assert_ne!(p.observe(frame(n, if n == 63 { 4096 } else { 80 })).sustained_pairs, Some(4096)); }
    }
    #[test] fn every_unclean_gate_resets_the_whole_window() {
        for fault in 0..14 {
            let mut p = PairWindow::new(299, 60).unwrap();
            for n in 61..=65 { p.observe(frame(n, 800)); }
            let mut bad = frame(66, 4096);
            match fault {
                0 => bad.iteration_after = bad.iteration_before,
                1 => bad.input_allowed_before = false, 2 => bad.input_allowed_after = false,
                3 => bad.locked_before = true, 4 => bad.locked_after = true,
                5 => bad.site = 2, 6 => bad.port_abort = true,
                7 => bad.alive_before = false, 8 => bad.alive_after = false,
                9 => bad.chest_alive_before = false, 10 => bad.chest_alive_after = false,
                11 => bad.chest_after[0] = f64::NAN, 12 => bad.mask_matches_requested = false,
                _ => bad.lagging = true,
            }
            assert_eq!(p.observe(bad).sustained_pairs, None);
            for n in 67..=71 { assert_eq!(p.observe(frame(n, 200)).sustained_pairs, None); }
            assert_eq!(p.observe(frame(72, 200)).sustained_pairs, Some(200));
        }
    }
    #[test] fn consecutive_lag_cutoff_resets_on_a_fast_frame() {
        let mut g=LagGuard::default();
        assert!(!g.observe(true,3)); assert!(!g.observe(true,3));
        assert!(!g.observe(false,3)); assert_eq!(g.consecutive(),0);
        assert!(!g.observe(true,3)); assert!(!g.observe(true,3)); assert!(g.observe(true,3));
        assert_eq!(g.consecutive(),3);
        let mut single=LagGuard::default(); assert!(single.observe(true,1));
    }
    #[test] fn duplicate_or_skipped_observations_do_not_make_six_frames() {
        let mut p = PairWindow::new(299, 60).unwrap();
        for _ in 0..10 { assert_eq!(p.observe(frame(61, 4096)).sustained_pairs, None); }
        assert_eq!(p.observe(frame(63, 4096)).sustained_pairs, None);
        for n in 64..=67 { assert_eq!(p.observe(frame(n, 4096)).sustained_pairs, None); }
        assert_eq!(p.observe(frame(68, 4096)).sustained_pairs, Some(4096));
    }
    #[test] fn changed_tail_cannot_reuse_an_earlier_high_window() {
        let mut p = PairWindow::new(299, 200).unwrap();
        for n in 61..=200 { assert_eq!(p.observe(frame(n, 4096)).sustained_pairs, None); }
        for n in 201..=205 { assert_eq!(p.observe(frame(n, 90)).sustained_pairs, None); }
        assert_eq!(p.observe(frame(206, 90)).sustained_pairs, Some(90));
    }
    #[test] fn finish_is_observed_even_when_all_fitness_gates_fail() {
        let mut p = PairWindow::new(299, 60).unwrap();
        let mut o = frame(286, 4096); o.finished = true; o.site = 2; o.lagging = true;
        o.locked_after = true; o.alive_after = false; o.chest_alive_after = false;
        let d = p.observe(o);
        assert!(d.observed_finish && d.observed_pair_pool_exhausted);
        assert_eq!(d.sustained_pairs, None); assert!(!d.continue_rollout);
    }
    #[test] fn a_launch_does_not_end_the_rollout_and_ceiling_does() {
        let mut p = PairWindow::new(299, 60).unwrap();
        let mut o = frame(286, 200); o.displacement = 60.0;
        assert!(p.observe(o).continue_rollout);
        assert!(!p.observe(frame(299, 200)).continue_rollout);
        let mut late = frame(300, 200); late.finished = true;
        assert!(!p.observe(late).observed_finish);
    }
    #[test] fn original_throw_is_diagnostic_and_never_an_occupancy_bonus() {
        let mut p = PairWindow::new(299, 60).unwrap();
        let mut o = frame(66, 4096); o.site = 2; o.locked_after = true;
        let d = p.observe(o); assert!(d.observed_pair_pool_exhausted);
        assert_eq!(d.sustained_pairs, None); assert!(d.continue_rollout);
    }
    #[test] fn equal_count_distinct_tas_is_retained_but_exact_raw_duplicate_is_not() {
        let a = vec![17; 139]; let mut b = a.clone(); b[90] = 49;
        assert!(!eligible_for_pool(&a, 500, &[a.clone()], Some(500)));
        assert!(eligible_for_pool(&b, 500, &[a], Some(500)));
        assert!(!eligible_for_pool(&b, 499, &[], Some(500)));
    }
    #[test] fn prefix_and_short_tail_contract_is_exact() {
        let a = vec![17; 139]; let mut b = a.clone(); b[61] = 49;
        assert!(valid_anchored_child(&a, &b)); b[59] ^= 16;
        assert!(!valid_anchored_child(&a, &b));
        assert!(!valid_anchored_child(&a, &a[..61]));
        assert!(PairWindow::new(61, 60).is_err());
        assert!(PairWindow::new(300, 60).is_err());
    }
}
