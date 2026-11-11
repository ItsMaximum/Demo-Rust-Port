//! Family-balanced host selection; no engine access or internal RNG.
//!
//! Each represented root gets one equally sized first-draw slot, regardless of
//! its resident count. The second draw selects a member in current vector order.

/// Select a resident by represented root, then by member within that root.
///
/// Empty input makes no draws. Every nonempty input makes exactly two calls,
/// both with positive bounds. Out-of-range callback results return None.
/// If the root draw is invalid, the second call uses bound 1 solely to preserve
/// the two-call contract; it cannot produce a selection or access a fake root.
pub fn pick(roots: &[u64], mut draw: impl FnMut(usize) -> usize) -> Option<usize> {
    if roots.is_empty() {
        return None;
    }

    let mut represented = roots.to_vec();
    represented.sort_unstable();
    represented.dedup();

    let root_slot = draw(represented.len());
    let selected_root = represented.get(root_slot).copied();
    let member_count = selected_root.map_or(1, |selected| {
        roots.iter().filter(|root| **root == selected).count()
    });
    let member_slot = draw(member_count);
    let selected_root = selected_root?;
    if member_slot >= member_count {
        return None;
    }

    roots.iter().enumerate().filter_map(|(index, root)| {
        if *root == selected_root { Some(index) } else { None }
    }).nth(member_slot)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn skewed_54_5_4_1_residents_still_have_four_equal_root_slots() {
        let mut roots = vec![4; 54];
        roots.extend([10; 5]);
        roots.extend([11; 4]);
        roots.push(99);
        let expected = [(4, 54), (10, 5), (11, 4), (99, 1)];

        for (root_slot, (root, members)) in expected.into_iter().enumerate() {
            let mut bounds = Vec::new();
            let selected = pick(&roots, |bound| {
                bounds.push(bound);
                if bounds.len() == 1 { root_slot } else { members - 1 }
            }).unwrap();
            assert_eq!(bounds, [4, members]);
            assert_eq!(roots[selected], root);
        }
    }

    #[test]
    fn sparse_unordered_roots_use_sorted_root_slots_and_vector_member_order() {
        let roots = [90, 3, 90, 8, 3, 90, 8];
        for (root_slot, member_slot, expected_index, expected_count) in [
            (0, 0, 1, 2), (0, 1, 4, 2),
            (1, 0, 3, 2), (1, 1, 6, 2),
            (2, 0, 0, 3), (2, 1, 2, 3), (2, 2, 5, 3),
        ] {
            let mut bounds = Vec::new();
            let selected = pick(&roots, |bound| {
                bounds.push(bound);
                if bounds.len() == 1 { root_slot } else { member_slot }
            });
            assert_eq!(selected, Some(expected_index));
            assert_eq!(bounds, [3, expected_count]);
        }
    }

    #[test]
    fn one_family_still_draws_a_bounded_member() {
        for member in 0..3 {
            let mut calls = 0;
            assert_eq!(pick(&[7, 7, 7], |bound| {
                assert!(bound > 0);
                calls += 1;
                if calls == 1 { assert_eq!(bound, 1); 0 }
                else { assert_eq!(bound, 3); member }
            }), Some(member));
            assert_eq!(calls, 2);
        }
        let mut bounds = Vec::new();
        assert_eq!(pick(&[u64::MAX], |bound| { bounds.push(bound); 0 }), Some(0));
        assert_eq!(bounds, [1, 1]);
    }

    #[test]
    fn empty_input_never_draws_a_zero_range_or_calls_the_callback() {
        assert_eq!(pick(&[], |_| panic!("empty selector must not draw")), None);
    }

    #[test]
    fn malformed_root_draw_returns_none_after_two_positive_bounded_calls() {
        let mut bounds = Vec::new();
        assert_eq!(pick(&[9, 3, 9], |bound| {
            assert!(bound > 0);
            bounds.push(bound);
            if bounds.len() == 1 { bound } else { 0 }
        }), None);
        assert_eq!(bounds, [2, 1]);
    }

    #[test]
    fn malformed_member_draw_returns_none_instead_of_indexing() {
        let mut bounds = Vec::new();
        assert_eq!(pick(&[9, 3, 9], |bound| {
            bounds.push(bound);
            if bounds.len() == 1 { 1 } else { usize::MAX }
        }), None);
        assert_eq!(bounds, [2, 2]);
    }
}
