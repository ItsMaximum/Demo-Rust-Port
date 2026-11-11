//! Host-side replay window eligibility. Adaptive windows can retain short inputs.

/// Prefix entries to replay before editing a parent. Empty input and a zero
/// requested window are invalid in both modes.
pub(crate) fn root(len: usize, window: usize, adaptive: bool) -> Option<usize> {
    if len == 0 || window == 0 {
        return None;
    }
    if adaptive {
        Some(len.saturating_sub(window))
    } else {
        // Preserve the fixed policy len > window + 2 without overflowing the
        // addition or subtracting a larger window from a shorter input.
        len.checked_sub(window).filter(|&prefix| prefix > 2)
    }
}

pub(crate) fn eligible(len: usize, window: usize, adaptive: bool) -> bool {
    root(len, window, adaptive).is_some()
}

#[cfg(test)]
mod tests {
    use super::{eligible, root};

    #[test]
    fn adaptive_windows_replay_short_gbe_descendants_from_the_level_start() {
        assert!(eligible(202, 223, true));
        assert_eq!(root(202, 223, true), Some(0));
        assert!(eligible(239, 223, true));
        assert_eq!(root(239, 223, true), Some(16));
    }

    #[test]
    fn fixed_windows_preserve_existing_roots_and_reject_short_parents() {
        assert!(!eligible(202, 223, false));
        assert_eq!(root(202, 223, false), None);
        assert!(eligible(227, 187, false));
        assert_eq!(root(227, 187, false), Some(40));
    }

    #[test]
    fn fixed_margin_and_adaptive_boundary_have_distinct_eligibility() {
        assert!(!eligible(12, 10, false));
        assert_eq!(root(12, 10, false), None);
        assert!(eligible(13, 10, false));
        assert_eq!(root(13, 10, false), Some(3));
        assert_eq!(root(10, 10, true), Some(0));
        assert_eq!(root(11, 10, true), Some(1));
        assert_eq!(root(12, 10, true), Some(2));
    }

    #[test]
    fn empty_inputs_and_zero_windows_cannot_be_replayed() {
        for adaptive in [false, true] {
            for (len, window) in [(0, 223), (239, 0), (0, 0)] {
                assert!(!eligible(len, window, adaptive));
                assert_eq!(root(len, window, adaptive), None);
            }
        }
    }

    #[test]
    fn windows_near_usize_limit_do_not_wrap_or_underflow() {
        assert!(!eligible(usize::MAX, usize::MAX, false));
        assert_eq!(root(usize::MAX, usize::MAX, true), Some(0));
        assert!(!eligible(usize::MAX, usize::MAX - 1, false));
        assert_eq!(root(usize::MAX, usize::MAX - 1, true), Some(1));
        assert_eq!(root(usize::MAX, usize::MAX - 3, false), Some(3));
        assert_eq!(root(1, usize::MAX, false), None);
        assert_eq!(root(1, usize::MAX, true), Some(0));
    }
}
