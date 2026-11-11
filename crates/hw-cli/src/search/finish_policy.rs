//! Ranking for valid finishes in the measured frame-perfect search.

/// Return whether `candidate` improves on `current`.
///
/// Both tuples use `(structural_cost, measured_frame_perfect_count, finish_frame)`.
/// Compare measured frame-perfect count first, finish frame second, and structural
/// cost third. The caller validates that finishes are nonzero and within its frame cap.
pub fn is_better(candidate: (usize, usize, u32), current: (usize, usize, u32)) -> bool {
    (candidate.1, candidate.2, candidate.0) < (current.1, current.2, current.0)
}

#[cfg(test)]
mod tests {
    use super::is_better;

    #[test]
    fn fewer_measured_frame_perfects_wins_despite_higher_cost_and_later_finish() {
        assert!(is_better((18, 14, 250), (15, 15, 227)));
    }

    #[test]
    fn more_measured_frame_perfects_loses_despite_lower_cost_and_earlier_finish() {
        assert!(!is_better((12, 16, 217), (15, 15, 227)));
    }

    #[test]
    fn equal_frame_perfects_favor_earlier_finish_despite_higher_cost() {
        assert!(is_better((18, 15, 218), (15, 15, 227)));
        assert!(!is_better((12, 15, 230), (15, 15, 227)));
    }

    #[test]
    fn equal_frame_perfects_and_finish_favor_lower_cost() {
        assert!(is_better((14, 15, 227), (15, 15, 227)));
        assert!(!is_better((16, 15, 227), (15, 15, 227)));
    }

    #[test]
    fn identical_result_is_not_an_improvement() {
        assert!(!is_better((15, 15, 227), (15, 15, 227)));
    }
}
