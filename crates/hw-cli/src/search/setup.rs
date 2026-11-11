//! Host-side setup mutations that preserve the input prefix before a replay anchor.

use super::events::{edges, moved};

fn draw(roll: &mut impl FnMut(u64) -> u64, upper: usize) -> usize {
    (roll(upper as u64) % upper as u64) as usize
}

/// Mutate one setup input aspect while keeping the TAS length fixed.
///
/// A quarter of selections flip one bit across the initial constant-mask run.
/// Other selections retime one key edge by up to eight entries, respecting its
/// same-key neighbours. Edge selection favours the first eject press and the
/// first 96 entries, while retaining a chance to select every edge.
///
/// `roll` receives a positive exclusive upper bound. Empty strings, blocked
/// retimings and unchanged results return `None`.
pub fn mutate(masks: &[u8], roll: &mut impl FnMut(u64) -> u64) -> Option<Vec<u8>> {
    mutate_from(masks, 0, roll)
}

/// Mutate one input aspect at or after `root`, preserving its exact prefix and length.
///
/// Retimings use the full input's key edges, so a key already held at the anchor
/// does not acquire a synthetic press there. An edge cannot move before either
/// the anchor or its preceding same-key neighbour. Initial-run flips start at
/// the anchor and end at the next complete-mask change.
pub fn mutate_from(masks: &[u8], root: usize, roll: &mut impl FnMut(u64) -> u64) -> Option<Vec<u8>> {
    let first = *masks.get(root)?;
    if draw(roll, 4) == 0 {
        let bit = 1u8 << draw(roll, 8);
        let end = root + masks[root..].iter().position(|&mask| mask != first).unwrap_or(masks.len() - root);
        let mut next = masks.to_vec();
        for mask in &mut next[root..end] {
            *mask ^= bit;
        }
        return Some(next);
    }

    let all: Vec<_> = edges(masks).into_iter().filter(|edge| edge.at >= root).collect();
    if all.is_empty() {
        return None;
    }
    let index = match draw(roll, 4) {
        0 => all.iter().position(|edge| edge.bit == 7 && edge.press)
            .unwrap_or_else(|| draw(roll, all.len())),
        1 | 2 => {
            let early = all.iter().take_while(|edge| edge.at < 96).count();
            draw(roll, if early == 0 { all.len() } else { early })
        }
        _ => draw(roll, all.len()),
    };
    let edge = &all[index];
    let later = draw(roll, 2) != 0;
    let amount = 1 + draw(roll, 8);
    let to = if later {
        edge.at.saturating_add(amount).min(edge.hi)
    } else {
        edge.at.saturating_sub(amount).max(edge.lo.max(root))
    };
    if to == edge.at {
        return None;
    }
    let next = moved(masks, edge, to);
    (next != masks).then_some(next)
}

#[cfg(test)]
mod tests {
    use super::{mutate, mutate_from};
    use super::super::events;

    fn scripted(sequence: &[u64]) -> impl FnMut(u64) -> u64 + '_ {
        let mut values = sequence.iter().copied();
        move |upper| {
            let value = values.next().expect("mutation requested an unexpected draw");
            assert!(value < upper);
            value
        }
    }

    fn assert_only_bit_changed(before: &[u8], after: &[u8], bit: u8) {
        assert_eq!(before.len(), after.len());
        for (&before, &after) in before.iter().zip(after) {
            assert_eq!(before & !(1 << bit), after & !(1 << bit));
        }
    }

    #[test]
    fn first_eject_press_can_move_earlier_and_later() {
        let before = [2, 2, 2, 130, 130, 130, 130];
        let earlier = mutate(&before, &mut scripted(&[1, 0, 0, 1])).unwrap();
        let later = mutate(&before, &mut scripted(&[1, 0, 1, 1])).unwrap();
        assert_eq!(earlier, [2, 130, 130, 130, 130, 130, 130]);
        assert_eq!(later, [2, 2, 2, 2, 2, 130, 130]);
        assert_only_bit_changed(&before, &earlier, 7);
        assert_only_bit_changed(&before, &later, 7);
    }

    #[test]
    fn initial_hold_can_change_from_entry_zero() {
        let before = [9, 9, 9, 18, 18];
        let next = mutate(&before, &mut scripted(&[0, 0])).unwrap();
        assert_eq!(next, [8, 8, 8, 18, 18]);
        assert_only_bit_changed(&before, &next, 0);
    }

    #[test]
    fn retiming_stops_before_same_key_neighbours() {
        let before = [0, 1, 1, 0, 0, 1];
        let earlier = mutate(&before, &mut scripted(&[1, 1, 1, 0, 7])).unwrap();
        let later = mutate(&before, &mut scripted(&[1, 1, 1, 1, 7])).unwrap();
        assert_eq!(earlier, [0, 1, 0, 0, 0, 1]);
        assert_eq!(later, [0, 1, 1, 1, 0, 1]);
        assert_eq!(events::edges(&earlier).iter().map(|edge| edge.at).collect::<Vec<_>>(), [1, 2, 5]);
        assert_eq!(events::edges(&later).iter().map(|edge| edge.at).collect::<Vec<_>>(), [1, 4, 5]);
        assert_only_bit_changed(&before, &earlier, 0);
        assert_only_bit_changed(&before, &later, 0);
    }

    #[test]
    fn blocked_edge_returns_none() {
        let before = [0, 1, 0, 1];
        assert!(mutate(&before, &mut scripted(&[1, 1, 1, 0, 7])).is_none());
        assert!(mutate(&before, &mut scripted(&[1, 1, 1, 1, 7])).is_none());
    }

    #[test]
    fn edges_past_early_window_remain_selectable() {
        let mut before = vec![0; 104];
        before[100..].fill(1);
        let next = mutate(&before, &mut scripted(&[1, 3, 0, 0, 1])).unwrap();
        assert_eq!(next[..98], [0; 98]);
        assert_eq!(next[98..], [1; 6]);
        assert_only_bit_changed(&before, &next, 0);
        // The early-edge preference also falls back when no early edge exists.
        let fallback = mutate(&before, &mut scripted(&[1, 1, 0, 0, 1])).unwrap();
        assert_eq!(fallback, next);
    }

    #[test]
    fn absent_eject_press_falls_back_to_other_keys() {
        let before = [0, 0, 4, 4];
        let next = mutate(&before, &mut scripted(&[1, 0, 0, 0, 0])).unwrap();
        assert_eq!(next, [0, 4, 4, 4]);
        assert_only_bit_changed(&before, &next, 2);
    }

    #[test]
    fn empty_and_single_entry_strings_are_safe() {
        let mut no_draws = |_| panic!("empty input must not request random draws");
        assert!(mutate(&[], &mut no_draws).is_none());
        assert!(mutate(&[0], &mut scripted(&[1])).is_none());
        assert!(mutate(&[128], &mut scripted(&[1, 0, 0, 7])).is_none());
        assert_eq!(mutate(&[128], &mut scripted(&[1, 0, 1, 7])), Some(vec![0]));
        assert_eq!(mutate(&[1], &mut scripted(&[0, 3])), Some(vec![9]));
    }

    #[test]
    fn anchored_eject_can_move_earlier_and_later_without_changing_prefix() {
        let before = [1, 1, 3, 3, 131, 131, 131];
        let earlier = mutate_from(&before, 2, &mut scripted(&[1, 0, 0, 7])).unwrap();
        let later = mutate_from(&before, 2, &mut scripted(&[1, 0, 1, 1])).unwrap();
        assert_eq!(earlier, [1, 1, 131, 131, 131, 131, 131]);
        assert_eq!(later, [1, 1, 3, 3, 3, 3, 131]);
        assert_eq!(earlier[..2], before[..2]);
        assert_eq!(later[..2], before[..2]);
        assert_only_bit_changed(&before, &earlier, 7);
        assert_only_bit_changed(&before, &later, 7);
    }

    #[test]
    fn held_key_at_anchor_does_not_create_a_retimable_press() {
        // Left was pressed before the anchor and remains held throughout.
        // The only eligible edge is up's actual press at entry 4.
        let before = [1, 1, 1, 1, 5, 5];
        let next = mutate_from(&before, 2, &mut scripted(&[1, 3, 0, 0, 0])).unwrap();
        assert_eq!(next, [1, 1, 1, 5, 5, 5]);
        assert_only_bit_changed(&before, &next, 2);

        let mut selection_only = scripted(&[1]);
        assert!(mutate_from(&[1, 1, 1, 1], 2, &mut selection_only).is_none());
    }

    #[test]
    fn anchor_run_flip_keeps_prefix_length_and_other_key_bits() {
        let before = [9, 9, 18, 18, 18, 4];
        let next = mutate_from(&before, 3, &mut scripted(&[0, 6])).unwrap();
        assert_eq!(next, [9, 9, 18, 82, 82, 4]);
        assert_eq!(next[..3], before[..3]);
        assert_only_bit_changed(&before, &next, 6);
    }

    #[test]
    fn anchored_retiming_respects_full_input_same_key_neighbours() {
        let before = [0, 1, 1, 0, 0, 1, 1];
        let earlier = mutate_from(&before, 2, &mut scripted(&[1, 3, 0, 0, 7])).unwrap();
        let later = mutate_from(&before, 2, &mut scripted(&[1, 3, 0, 1, 7])).unwrap();
        assert_eq!(earlier, [0, 1, 0, 0, 0, 1, 1]);
        assert_eq!(later, [0, 1, 1, 1, 0, 1, 1]);
        assert_eq!(events::edges(&earlier).iter().map(|edge| edge.at).collect::<Vec<_>>(), [1, 2, 5]);
        assert_eq!(events::edges(&later).iter().map(|edge| edge.at).collect::<Vec<_>>(), [1, 4, 5]);
        assert_only_bit_changed(&before, &earlier, 0);
        assert_only_bit_changed(&before, &later, 0);

        // With an earlier anchor, the preceding key edge still bounds the move.
        let bounded = mutate_from(&before, 1, &mut scripted(&[1, 3, 1, 0, 7])).unwrap();
        assert_eq!(bounded, earlier);

        // The press at entry 1 is protected; the release at the anchor cannot
        // move earlier across either the anchor or that preceding press.
        assert!(mutate_from(&[0, 1, 0, 1], 2, &mut scripted(&[1, 3, 0, 0, 7])).is_none());
    }

    #[test]
    fn root_at_or_beyond_end_returns_without_random_draws() {
        let mut no_draws = |_| panic!("invalid anchor must not request random draws");
        assert!(mutate_from(&[], 0, &mut no_draws).is_none());
        assert!(mutate_from(&[1, 1], 2, &mut no_draws).is_none());
        assert!(mutate_from(&[1, 1], usize::MAX, &mut no_draws).is_none());
    }
}
