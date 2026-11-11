//! Host-side coherent retiming of raw eject and post-eject input bundles.
//! Every original suffix byte survives; this makes no state or fitness claim.

#[derive(Debug, PartialEq, Eq)]
pub struct Proposal {
    pub masks: Vec<u8>,
    pub before_a: usize,
    pub before_b: usize,
    pub after_a: usize,
    pub after_b: usize,
    pub delta: i32,
    pub kappa: i32,
}

fn shifted(value: usize, amount: i32) -> Option<usize> {
    if amount < 0 {
        value.checked_sub((-amount) as usize)
    } else {
        value.checked_add(amount as usize)
    }
}

/// Draw raw delta in -8..=8 and kappa in -4..=4, without clamping or fallback.
/// Invalid pairs return `None`. Valid proposals move complete raw-mask changes
/// together, keep the full suffix at its new time, and pad with the original
/// final mask to preserve both length and the exact prefix before `root`.
///
/// `roll` receives a positive exclusive upper bound. Inputs without a preceding
/// mounted run and a later complete-mask boundary return before any draws.
pub fn mutate_from(masks: &[u8], root: usize, roll: &mut impl FnMut(u64) -> u64) -> Option<Proposal> {
    let prefix = masks.get(..root)?;
    let a = masks.iter().position(|mask| mask & 128 != 0)?;
    if a == 0 {
        return None;
    }
    let prior = masks[a - 1];
    let eject = masks[a];
    let b = (a + 1..masks.len()).find(|&at| masks[at] != eject)?;
    let mut p = a - 1;
    while p > 0 && masks[p - 1] == prior {
        p -= 1;
    }

    let delta = (roll(17) % 17) as i32 - 8;
    let kappa = (roll(9) % 9) as i32 - 4;
    let suffix_shift = delta + kappa;
    if (delta == 0 && kappa == 0) || suffix_shift > 0 {
        return None;
    }
    let mounted_len = shifted(a - p, delta)?;
    let ejected_len = shifted(b - a, kappa)?;
    if mounted_len == 0 || ejected_len == 0 {
        return None;
    }
    let after_a = shifted(a, delta)?;
    let after_b = shifted(b, suffix_shift)?;
    if after_a < root {
        return None;
    }

    let mut next = Vec::with_capacity(masks.len());
    next.extend_from_slice(&masks[..p]);
    next.resize(after_a, prior);
    next.resize(after_b, eject);
    next.extend_from_slice(&masks[b..]);
    // A positive net shift would require dropping original tail bytes, so it
    // was rejected above. Negative shifts only append the exact final hold.
    if next.len() > masks.len() {
        return None;
    }
    next.resize(masks.len(), *masks.last()?);
    if next.get(..root) != Some(prefix) || next == masks {
        return None;
    }
    Some(Proposal { masks: next, before_a: a, before_b: b, after_a, after_b, delta, kappa })
}

#[cfg(test)]
mod tests {
    use super::mutate_from;

    fn scripted(values: &[u64]) -> impl FnMut(u64) -> u64 + '_ {
        let mut values = values.iter().copied();
        move |upper| {
            let value = values.next().expect("unexpected random draw");
            assert!(value < upper);
            value
        }
    }

    fn satan() -> Vec<u8> {
        // Original mounted/ejected raw masks and cuts, with a varied later tail.
        let mut masks = vec![9; 2];
        masks.extend([24; 28]);
        masks.extend([148; 8]);
        masks.extend([176, 240, 240, 131, 178, 250, 154, 186, 146, 208]);
        masks
    }

    fn raw_runs(masks: &[u8]) -> Vec<u8> {
        masks.iter().enumerate().filter_map(|(at, &mask)|
            (at == 0 || mask != masks[at - 1]).then_some(mask)).collect()
    }

    #[test]
    fn satan_eject_moves_three_bits_together_and_preserves_the_complete_tail() {
        let before = satan();
        let proposal = mutate_from(&before, 20, &mut scripted(&[6, 4])).unwrap();
        assert_eq!((proposal.before_a, proposal.before_b), (30, 38));
        assert_eq!((proposal.after_a, proposal.after_b), (28, 36));
        assert_eq!((proposal.delta, proposal.kappa), (-2, 0));
        // A unary key-edge retiming cannot make this three-bit change.
        assert_eq!(before[28] ^ proposal.masks[28], (1 << 2) | (1 << 3) | (1 << 7));
        assert_eq!(proposal.masks[28..30], [148, 148]);
        assert_eq!(proposal.masks[..20], before[..20]);
        assert_eq!(proposal.masks.len(), before.len());
        let suffix_end = proposal.after_b + before.len() - proposal.before_b;
        assert_eq!(proposal.masks[proposal.after_b..suffix_end], before[proposal.before_b..]);
        assert_eq!(proposal.masks[suffix_end..], [208, 208]);
        assert_eq!(raw_runs(&proposal.masks), raw_runs(&before));
    }

    #[test]
    fn later_eject_can_leave_the_entire_later_tail_at_its_original_time() {
        let before = satan();
        let proposal = mutate_from(&before, 20, &mut scripted(&[10, 2])).unwrap();
        assert_eq!((proposal.delta, proposal.kappa), (2, -2));
        assert_eq!((proposal.after_a, proposal.after_b), (32, 38));
        assert_eq!(proposal.masks[30..32], [24, 24]);
        assert_eq!(proposal.masks[38..], before[38..]);
        assert_eq!(proposal.masks[..20], before[..20]);
        assert_eq!(raw_runs(&proposal.masks), raw_runs(&before));
        assert_eq!(proposal.masks.len(), before.len());
    }

    #[test]
    fn gbe_moves_the_eject_and_multibit_post_boundary_in_one_proposal() {
        let mut before = vec![1; 26];
        before.extend([129; 7]);
        before.extend([136; 7]);
        before.extend([148; 8]);
        before.extend([152; 4]);
        let proposal = mutate_from(&before, 16, &mut scripted(&[6, 4])).unwrap();
        assert_eq!((proposal.before_a, proposal.before_b), (26, 33));
        assert_eq!((proposal.after_a, proposal.after_b), (24, 31));
        assert_eq!(proposal.masks[31] ^ proposal.masks[30], (1 << 0) | (1 << 3));
        assert_eq!(proposal.masks[31..before.len() - 2], before[33..]);
        assert_eq!(proposal.masks[..16], before[..16]);
        assert_eq!(proposal.masks.len(), before.len());
        assert_eq!(raw_runs(&proposal.masks), raw_runs(&before));
    }

    #[test]
    fn both_destination_and_original_prefix_must_respect_the_anchor() {
        let before = satan();
        assert!(mutate_from(&before, 29, &mut scripted(&[6, 4])).is_none());
        // Destination32 is after anchor31, but delaying original entry30 would
        // change an input already consumed by that anchor. Reject it as well.
        assert!(mutate_from(&before, 31, &mut scripted(&[10, 2])).is_none());
        let exact = mutate_from(&before, 28, &mut scripted(&[6, 4])).unwrap();
        assert_eq!(exact.masks[..28], before[..28]);
        let mut no_draws = |_| panic!("invalid root must not draw");
        assert!(mutate_from(&before, before.len() + 1, &mut no_draws).is_none());
    }

    #[test]
    fn absent_or_incomplete_bundles_return_before_random_draws() {
        for before in [vec![], vec![1], vec![128], vec![128, 16], vec![0, 128], vec![0, 0, 16]] {
            let mut no_draws = |_| panic!("incomplete bundle must not draw");
            assert!(mutate_from(&before, 0, &mut no_draws).is_none());
        }
    }

    #[test]
    fn identity_positive_net_shift_and_removed_runs_are_rejected_without_fallback() {
        let before = satan();
        assert!(mutate_from(&before, 20, &mut scripted(&[8, 4])).is_none());
        assert!(mutate_from(&before, 20, &mut scripted(&[7, 6])).is_none());
        // Short mounted/ejected runs cannot be deleted or crossed.
        assert!(mutate_from(&[0, 128, 16], 0, &mut scripted(&[7, 4])).is_none());
        assert!(mutate_from(&[0, 0, 128, 16], 0, &mut scripted(&[8, 3])).is_none());
        assert!(mutate_from(&[0, 128, 16], 0, &mut scripted(&[0, 4])).is_none());
    }

    #[test]
    fn each_complete_input_draws_the_two_raw_axes_once_and_deterministically() {
        let before = satan();
        let mut bounds = Vec::new();
        let mut values = [6, 4].into_iter();
        let first = mutate_from(&before, 20, &mut |upper| {
            bounds.push(upper);
            values.next().expect("no retry or fallback draw")
        }).unwrap();
        let second = mutate_from(&before, 20, &mut scripted(&[6, 4])).unwrap();
        assert_eq!(bounds, [17, 9]);
        assert_eq!(first, second);
        assert_eq!(before, satan());
    }
}
