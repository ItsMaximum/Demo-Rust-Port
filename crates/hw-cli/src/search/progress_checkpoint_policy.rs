//! Exact actual-frame checkpoints for clean progress windows; no replay or RNG.
//!
//! The caller may pass only a score returned by ProgressWindow for this same
//! replay and actual world frame. Matching integer frames alone does not prove
//! liveness, freshness, or clean scoring. No earlier/nearest window is borrowed.

pub const FRAMES: [u32; 2] = [180, 240];

pub fn index(actual_frame: u32, world_frames: [u32; 6]) -> Option<usize> {
    let checkpoint = FRAMES.iter().position(|frame| *frame == actual_frame)?;
    let first = actual_frame - 5;
    for (offset, frame) in world_frames.iter().enumerate() {
        if *frame != first + offset as u32 {
            return None;
        }
    }
    Some(checkpoint)
}

/// Split a valid even reserve cap equally across the two exact checkpoints.
pub fn per_checkpoint_capacity(total: usize) -> Option<usize> {
    if (2..=1024).contains(&total) && total % 2 == 0 {
        Some(total / 2)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::policy::{Observation, ProgressWindow, WINDOW};

    fn sample(frame: u32, iteration_before: u32) -> Observation {
        Observation {
            frame_before: frame - 1,
            frame_after: frame,
            iteration_before,
            iteration_after: iteration_before + 1,
            input_allowed_before: true,
            input_allowed_after: true,
            mask_matches_requested: true,
            locked_before: false,
            locked_after: false,
            dad_alive_before: true,
            dad_alive_after: true,
            chest_live_before: true,
            chest_live_after: true,
            chest_handle_before: 24,
            chest_handle_after: 24,
            chest_before: [26.0, 78.0],
            chest_after: [26.0, 78.0],
            chest_velocity_after: [100.0, -16.0],
            chest_frozen_before: false,
            chest_frozen_after: false,
            focus_live_before: true,
            focus_live_after: true,
            focus_handle_before: 24,
            focus_handle_after: 24,
            focus_before: [26.0, 78.0],
            focus_after: [26.0, 78.0],
            focus_frozen_before: false,
            focus_frozen_after: false,
            finish_distance_after: 90.0,
            site: 0,
            port_abort: false,
            lagging: false,
            finished: false,
            host_diagnostics_complete: true,
            pairs_after: 100,
            qe: 1.0,
            qe_raw: 1.0,
        }
    }

    #[test]
    fn only_exact_chosen_frame_and_consecutive_actual_window_match() {
        assert_eq!(WINDOW, 6);
        assert_eq!(index(180, [175, 176, 177, 178, 179, 180]), Some(0));
        assert_eq!(index(240, [235, 236, 237, 238, 239, 240]), Some(1));
        assert_eq!(index(179, [174, 175, 176, 177, 178, 179]), None);
        assert_eq!(index(181, [176, 177, 178, 179, 180, 181]), None);
        assert_eq!(index(180, [174, 175, 176, 177, 178, 179]), None);
        assert_eq!(index(180, [175, 176, 177, 179, 178, 180]), None);
        assert_eq!(index(180, [175, 176, 177, 178, 178, 180]), None);
        assert_eq!(index(180, [0, 0, 0, 0, 0, 180]), None);
        assert_eq!(index(240, [175, 176, 177, 178, 179, 180]), None);
    }

    #[test]
    fn changed_target_minus6_has_first_six_clean_samples_at_each_checkpoint() {
        for (checkpoint_index, checkpoint) in FRAMES.into_iter().enumerate() {
            let mut window = ProgressWindow::new(299, checkpoint as usize - 6).unwrap();
            for frame in 61..checkpoint {
                assert!(window.observe(sample(frame, frame - 1)).score.is_none());
            }
            let score = window.observe(sample(checkpoint, checkpoint - 1)).score.unwrap();
            let expected: [u32; WINDOW] = std::array::from_fn(|i| checkpoint - 5 + i as u32);
            assert_eq!(score.world_frames, expected);
            assert_eq!(index(checkpoint, score.world_frames), Some(checkpoint_index));
        }
    }

    #[test]
    fn changed_target_minus5_or_target_cannot_borrow_an_earlier_window() {
        for checkpoint in FRAMES {
            for changed in [checkpoint - 5, checkpoint] {
                let mut window = ProgressWindow::new(299, changed as usize).unwrap();
                for frame in 61..=checkpoint {
                    assert!(window.observe(sample(frame, frame - 1)).score.is_none());
                }
            }
        }
    }

    #[test]
    fn finite_planned_eof_before_checkpoint_cannot_supply_last_clean_sample() {
        for checkpoint in FRAMES {
            let planned = checkpoint as usize - 1;
            let mut window = ProgressWindow::new(planned, checkpoint as usize - 6).unwrap();
            for frame in 61..=checkpoint {
                let mut observation = sample(frame, frame - 1);
                if frame == checkpoint {
                    // No requested literal slot remains at iteration planned.
                    observation.input_allowed_after = false;
                }
                assert!(window.observe(observation).score.is_none());
            }
        }
    }

    #[test]
    fn primary_death_inside_checkpoint_window_resets_its_clean_samples() {
        for checkpoint in FRAMES {
            let mut window = ProgressWindow::new(299, checkpoint as usize - 6).unwrap();
            for frame in 61..=checkpoint {
                let mut observation = sample(frame, frame - 1);
                if frame >= checkpoint - 2 {
                    // A live child/focus is deliberately left true: it cannot
                    // substitute for primary Dad survival after this death.
                    observation.dad_alive_after = false;
                    if frame > checkpoint - 2 { observation.dad_alive_before = false; }
                }
                assert!(window.observe(observation).score.is_none());
            }
        }
    }

    #[test]
    fn earlier_input_stall_uses_actual_iteration_not_world_frame_credit() {
        // One no-input-progress frame at world100 leaves iteration one behind.
        // At the same actual F180, changed174 has only five credited samples;
        // changed173 has six and may legitimately enter checkpoint zero.
        for (changed, expected) in [(174, None), (173, Some(0))] {
            let mut window = ProgressWindow::new(299, changed).unwrap();
            let mut selected = None;
            for frame in 61..=180 {
                let before = if frame <= 100 { frame - 1 } else { frame - 2 };
                let mut observation = sample(frame, before);
                if frame == 100 {
                    observation.iteration_after = before;
                }
                let decision = window.observe(observation);
                if frame == 180 {
                    assert_eq!(observation.iteration_after, 179);
                    selected = decision.score.and_then(|score| index(frame, score.world_frames));
                }
            }
            assert_eq!(selected, expected);
        }
    }

    #[test]
    fn lag_or_lock_inside_the_exact_checkpoint_window_resets_clean_score() {
        for fault in 0..2 {
            let mut window = ProgressWindow::new(299, 174).unwrap();
            for frame in 61..=180 {
                let mut observation = sample(frame, frame - 1);
                if frame == 178 {
                    if fault == 0 { observation.lagging = true; }
                    else { observation.locked_after = true; }
                }
                assert!(window.observe(observation).score.is_none());
            }
        }
    }

    #[test]
    fn input_stall_inside_the_window_cannot_fill_a_clean_six_sample_slot() {
        let mut window = ProgressWindow::new(299, 174).unwrap();
        for frame in 61..=180 {
            let before = if frame <= 178 { frame - 1 } else { frame - 2 };
            let mut observation = sample(frame, before);
            if frame == 178 { observation.iteration_after = before; }
            assert!(window.observe(observation).score.is_none());
        }
    }

    #[test]
    fn reserve_capacity_is_even_bounded_and_split_without_rounding() {
        for total in (2..=1024).step_by(2) {
            assert_eq!(per_checkpoint_capacity(total), Some(total / 2));
        }
        for total in [0, 1, 3, 1023, 1025, usize::MAX] {
            assert_eq!(per_checkpoint_capacity(total), None);
        }
    }
}
