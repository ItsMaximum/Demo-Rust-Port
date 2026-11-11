//! Host-side repair parents, bounded independently for each initial seed ID.
//! Admission checks raw inputs only and makes no fitness or finish claim.

use std::collections::VecDeque;

#[derive(Clone)]
pub(crate) struct Entry {
    pub(crate) id: u64,
    pub(crate) ancestor: u64,
    pub(crate) masks: Vec<u8>,
    pub(crate) generation: u32,
}

struct Family {
    ancestor: u64,
    original: Vec<u8>,
    root: usize,
    children: VecDeque<Entry>,
}

pub(crate) struct Bank {
    families: Vec<Family>,
    capacity_per_family: usize,
}

impl Bank {
    /// Seed IDs must be unique. Each root counts original entries whose bytes
    /// every descendant must preserve; full input length also remains fixed.
    pub(crate) fn new(seeds: Vec<(u64, Vec<u8>, usize)>, capacity_per_family: usize) -> Self {
        Self {
            families: seeds.into_iter().map(|(ancestor, original, root)| Family {
                ancestor, original, root, children: VecDeque::new(),
            }).collect(),
            capacity_per_family,
        }
    }

    /// Retain a distinct nonoriginal child, replacing the oldest child in its
    /// own family when full. Rejected candidates never change the bank.
    pub(crate) fn admit(&mut self, ancestor: u64, id: u64, masks: &[u8], generation: u32) -> bool {
        if self.capacity_per_family == 0 || generation == 0 {
            return false;
        }
        let Some(family) = self.families.iter_mut().find(|family| family.ancestor == ancestor) else {
            return false;
        };
        if masks.len() != family.original.len() || masks == family.original.as_slice() {
            return false;
        }
        let Some(prefix) = family.original.get(..family.root) else {
            return false;
        };
        if masks.get(..family.root) != Some(prefix)
            || family.children.iter().any(|child| child.masks.as_slice() == masks) {
            return false;
        }
        if family.children.len() == self.capacity_per_family {
            family.children.pop_front();
        }
        family.children.push_back(Entry { id, ancestor, masks: masks.to_vec(), generation });
        true
    }

    /// Restore one checkpoint row without evicting earlier imported rows. Every
    /// row must fit its original family and pass the same exact raw-input checks.
    pub(crate) fn restore_entry(&mut self, ancestor: u64, id: u64, masks: &[u8], generation: u32) -> Result<(), String> {
        let Some(family) = self.families.iter().find(|family| family.ancestor == ancestor) else {
            return Err("unknown curated ancestor".into());
        };
        if self.capacity_per_family == 0 || family.children.len() >= self.capacity_per_family {
            return Err("repair family exceeds --repair-per-seed capacity".into());
        }
        if !self.admit(ancestor, id, masks, generation) {
            return Err("invalid repair generation, full length, raw prefix, original input or exact duplicate".into());
        }
        Ok(())
    }

    pub(crate) fn count(&self, ancestor: u64) -> usize {
        self.families.iter().find(|family| family.ancestor == ancestor)
            .map_or(0, |family| family.children.len())
    }

    /// Ordinals follow FIFO order within the family; the caller can choose them
    /// uniformly from `0..count(ancestor)` without depending on child metadata.
    pub(crate) fn pick(&self, ancestor: u64, ordinal: usize) -> Option<Entry> {
        self.families.iter().find(|family| family.ancestor == ancestor)?
            .children.get(ordinal).cloned()
    }

    pub(crate) fn len(&self) -> usize {
        self.families.iter().map(|family| family.children.len()).sum()
    }

    pub(crate) fn populated_families(&self) -> usize {
        self.families.iter().filter(|family| !family.children.is_empty()).count()
    }

    /// Select a nonempty family by ordinal, skipping empty families without
    /// weighting by their number of children.
    pub(crate) fn populated_ancestor(&self, ordinal: usize) -> Option<u64> {
        self.families.iter().filter(|family| !family.children.is_empty())
            .nth(ordinal).map(|family| family.ancestor)
    }

    /// Largest generation still retained, or zero when the bank is empty.
    pub(crate) fn max_generation(&self) -> u32 {
        self.entries().map(|entry| entry.generation).max().unwrap_or(0)
    }

    /// Checkpoint order is initial family order, then FIFO order within a family.
    pub(crate) fn entries(&self) -> impl Iterator<Item = &Entry> {
        self.families.iter().flat_map(|family| family.children.iter())
    }
}

#[cfg(test)]
mod tests {
    use super::Bank;

    #[test]
    fn repairs_can_chain_two_generations_without_changing_replay_prefix_or_length() {
        let original = vec![9, 24, 24, 148, 148, 176];
        let mut bank = Bank::new(vec![(7, original.clone(), 3)], 3);
        let mut first = original.clone();
        first[4] ^= 64;
        assert!(bank.admit(7, 100, &first, 1));

        let mut parent = bank.pick(7, 0).unwrap();
        assert_eq!(parent.id, 100);
        assert_eq!(parent.ancestor, 7);
        parent.masks[5] ^= 1;
        assert_eq!(bank.pick(7, 0).unwrap().masks, first);
        assert!(bank.admit(parent.ancestor, 101, &parent.masks, parent.generation + 1));

        assert_eq!(bank.count(7), 2);
        assert_eq!(bank.populated_families(), 1);
        assert_eq!(bank.max_generation(), 2);
        for child in bank.entries() {
            assert_eq!(child.masks.len(), original.len());
            assert_eq!(child.masks[..3], original[..3]);
        }
        let second = bank.pick(7, 1).unwrap();
        assert_eq!(second.id, 101);
        assert_eq!(second.generation, 2);
        assert_eq!(second.masks, parent.masks);
    }

    #[test]
    fn changed_prefix_lengths_original_and_invalid_roots_are_rejected() {
        let original = vec![8, 16, 148, 176];
        let mut bank = Bank::new(vec![(1, original.clone(), 2)], 4);
        assert!(bank.admit(1, 10, &[8, 16, 149, 176], 1));
        assert!(!bank.admit(1, 11, &[9, 16, 149, 176], 2));
        assert!(!bank.admit(1, 12, &[8, 16, 149], 2));
        assert!(!bank.admit(1, 13, &[8, 16, 149, 176, 0], 2));
        assert!(!bank.admit(1, 14, &original, 2));
        assert_eq!(bank.len(), 1);
        assert_eq!(bank.pick(1, 0).unwrap().id, 10);

        let mut invalid_root = Bank::new(vec![(1, original, 5)], 4);
        assert!(!invalid_root.admit(1, 15, &[8, 16, 149, 176], 1));
        assert_eq!(invalid_root.len(), 0);
    }

    #[test]
    fn exact_duplicates_keep_history_but_masked_key_differences_remain_distinct() {
        let mut bank = Bank::new(vec![(7, vec![64, 64, 64, 0], 1)], 3);
        // Ctrl is held in both variants; every lower-priority raw key still
        // belongs to the candidate's identity.
        assert!(bank.admit(7, 100, &[64, 65, 64, 0], 1));
        assert!(!bank.admit(7, 101, &[64, 65, 64, 0], 2));
        assert!(bank.admit(7, 102, &[64, 66, 64, 0], 1));
        assert_eq!(bank.count(7), 2);
        let retained = bank.pick(7, 0).unwrap();
        assert_eq!(retained.id, 100);
        assert_eq!(retained.generation, 1);
        assert_eq!(bank.max_generation(), 1);
    }

    #[test]
    fn capacity_is_fifo_and_independent_for_each_initial_seed() {
        let original = vec![64, 0, 0];
        let mut bank = Bank::new(vec![(3, original.clone(), 1), (5, original.clone(), 1), (8, original, 1)], 2);
        assert!(bank.admit(3, 10, &[64, 1, 0], 1));
        assert!(bank.admit(3, 11, &[64, 2, 0], 1));
        assert!(bank.admit(3, 12, &[64, 4, 0], 2));
        assert_eq!(bank.pick(3, 0).unwrap().id, 11);
        assert_eq!(bank.pick(3, 1).unwrap().id, 12);
        assert!(bank.pick(3, 2).is_none());

        // Identical original and child bytes do not merge different seed IDs.
        assert!(bank.admit(8, 20, &[64, 2, 0], 9));
        assert!(bank.admit(8, 21, &[64, 4, 0], 1));
        assert!(bank.admit(8, 22, &[64, 8, 0], 1));
        assert_eq!(bank.pick(8, 0).unwrap().id, 21);
        assert_eq!(bank.pick(8, 1).unwrap().id, 22);
        assert_eq!(bank.pick(3, 0).unwrap().id, 11);
        assert_eq!(bank.pick(3, 1).unwrap().id, 12);
        assert_eq!(bank.len(), 4);
        assert_eq!(bank.count(3), 2);
        assert_eq!(bank.count(8), 2);
        assert_eq!(bank.populated_families(), 2);
        assert_eq!(bank.populated_ancestor(0), Some(3));
        assert_eq!(bank.populated_ancestor(1), Some(8));
        assert_eq!(bank.populated_ancestor(2), None);
        assert_eq!(bank.max_generation(), 2);
        assert_eq!(bank.entries().map(|entry| entry.id).collect::<Vec<_>>(), [11, 12, 21, 22]);
    }

    #[test]
    fn unknown_ancestors_zero_generation_and_disabled_banks_cannot_admit() {
        let mut enabled = Bank::new(vec![(1, vec![16, 0], 1)], 2);
        assert!(!enabled.admit(99, 10, &[16, 1], 1));
        assert!(!enabled.admit(1, 10, &[16, 1], 0));
        assert_eq!(enabled.count(99), 0);
        assert!(enabled.pick(99, 0).is_none());
        assert_eq!(enabled.len(), 0);

        let mut disabled = Bank::new(vec![(1, vec![16, 0], 1)], 0);
        assert!(!disabled.admit(1, 10, &[16, 1], 1));
        assert_eq!(disabled.count(1), 0);
        assert_eq!(disabled.len(), 0);
        assert_eq!(disabled.populated_families(), 0);
        assert_eq!(disabled.populated_ancestor(0), None);
        assert_eq!(disabled.max_generation(), 0);
        assert!(disabled.entries().next().is_none());
        assert!(disabled.pick(1, 0).is_none());

        let mut empty = Bank::new(Vec::new(), 2);
        assert!(!empty.admit(1, 10, &[16, 1], 1));
    }
}
