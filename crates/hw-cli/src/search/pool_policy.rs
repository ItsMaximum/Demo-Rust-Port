//! Host-side lineage insertion policy. Different TAS bytes remain different parents.

use std::collections::HashSet;

#[derive(Clone)]
pub(crate) struct Member {
    pub(crate) masks: Vec<u8>,
    pub(crate) score: f64,
    pub(crate) visits: u64,
    pub(crate) id: u64,
    pub(crate) events: usize,
}

/// Append a new exact TAS, returning true, or merge its metadata and return false.
///
/// Fresh discoveries have zero visits and must not reset an existing parent's
/// selection history. Imported duplicates add their visits; identity and event
/// metadata stay with the original parent, while better fitness refreshes it.
pub(crate) fn upsert(pool: &mut Vec<Member>, candidate: Member) -> bool {
    if let Some(existing) = pool.iter_mut().find(|member| member.masks == candidate.masks) {
        existing.score = existing.score.min(candidate.score);
        if candidate.visits != 0 {
            existing.visits = existing.visits.saturating_add(candidate.visits);
        }
        false
    } else {
        pool.push(candidate);
        true
    }
}

/// Index of the `ordinal`th surviving initial seed, regardless of pool ordering.
/// Initial IDs may have gaps after exact duplicate inputs were merged by `upsert`.
pub(crate) fn seed_index(pool: &[Member], seed_id_end: u64, ordinal: usize) -> Option<usize> {
    pool.iter().enumerate().filter(|(_, member)| member.id < seed_id_end)
        .nth(ordinal).map(|(index, _)| index)
}

/// Bound the pool, keeping protected initial seeds before descendants, then ranking
/// each group by the existing fitness policy and selection history.
/// The caller must give enough capacity for every protected initial seed.
pub(crate) fn trim(pool: &mut Vec<Member>, capacity: usize, seed_id_end: Option<u64>, event_weighted: bool) {
    if pool.len() <= capacity {
        return;
    }
    let protected = |member: &Member| seed_id_end.is_some_and(|end| member.id < end);
    let fitness = |member: &Member| {
        if event_weighted { member.events as f64 * 20.0 + member.score } else { member.score }
    };
    pool.sort_by(|a, b| protected(b).cmp(&protected(a))
        .then_with(|| fitness(a).total_cmp(&fitness(b)))
        .then(a.visits.cmp(&b.visits)));
    pool.truncate(capacity);
}

/// Repeated launch geometry is only a sampling key, not an exact state identity.
/// This opt-in gate excludes finishes, whose measurement path remains separate.
pub(crate) fn retain_repeated_launch(enabled: bool, first_outcome: bool, launch: bool, finished: bool) -> bool {
    enabled && !first_outcome && launch && !finished
}

#[derive(Debug, PartialEq, Eq)]
pub(crate) struct Insertion {
    pub(crate) inserted: bool,
    pub(crate) retained: bool,
}

/// Apply the ordinary exact-byte insertion and trimming policy, tagging only a
/// newly inserted ID that survives this trim. Existing seeds and checkpoint
/// parents keep their identity and selection history, without receiving a tag.
/// Callers holding shared locks must acquire the pool before the tracking set.
pub(crate) fn insert_tracked(pool: &mut Vec<Member>, tracked: &mut HashSet<u64>,
    candidate: Member, capacity: usize, seed_id_end: Option<u64>, event_weighted: bool) -> Insertion {
    let id = candidate.id;
    let inserted = upsert(pool, candidate);
    trim(pool, capacity, seed_id_end, event_weighted);
    let retained = inserted && pool.iter().any(|member| member.id == id);
    if retained {
        tracked.insert(id);
    }
    Insertion { inserted, retained }
}

/// Drop tags for parents evicted since the previous stats checkpoint. IDs are
/// never reused; pruning therefore cannot tag unrelated later discoveries.
pub(crate) fn prune_parent_ids(pool: &[Member], tracked: &mut HashSet<u64>) -> usize {
    let live: HashSet<_> = pool.iter().map(|member| member.id).collect();
    tracked.retain(|id| live.contains(id));
    tracked.len()
}

#[cfg(test)]
mod tests {
    use super::{insert_tracked, prune_parent_ids, retain_repeated_launch, seed_index, trim, upsert, Insertion, Member};
    use std::collections::HashSet;

    fn member(masks: &[u8], score: f64, visits: u64, id: u64, events: usize) -> Member {
        Member { masks: masks.to_vec(), score, visits, id, events }
    }

    #[test]
    fn rediscovering_the_same_tas_preserves_selection_history_and_identity() {
        let mut pool = Vec::new();
        assert!(upsert(&mut pool, member(&[2, 130, 144], 40.0, 17, 3, 8)));
        for id in 4..20 {
            assert!(!upsert(&mut pool, member(&[2, 130, 144], 60.0, 0, id, 99)));
        }
        assert_eq!(pool.len(), 1);
        assert_eq!(pool[0].masks, [2, 130, 144]);
        assert_eq!(pool[0].score, 40.0);
        assert_eq!(pool[0].visits, 17);
        assert_eq!(pool[0].id, 3);
        assert_eq!(pool[0].events, 8);
    }

    #[test]
    fn a_measured_discovery_refreshes_an_unscored_seed_without_replacing_it() {
        let mut pool = vec![member(&[2, 130, 144], 400.0, 11, 7, 8)];
        assert!(!upsert(&mut pool, member(&[2, 130, 144], 25.0, 0, 100, 8)));
        assert_eq!(pool.len(), 1);
        assert_eq!(pool[0].score, 25.0);
        assert_eq!(pool[0].visits, 11);
        assert_eq!(pool[0].id, 7);
        assert_eq!(pool[0].events, 8);
    }

    #[test]
    fn imported_duplicate_visits_accumulate_without_losing_better_fitness() {
        let mut pool = vec![member(&[2, 130, 144], 25.0, 11, 7, 8)];
        assert!(!upsert(&mut pool, member(&[2, 130, 144], 40.0, 5, 100, 99)));
        assert_eq!(pool[0].visits, 16);
        assert_eq!(pool[0].score, 25.0);
        assert_eq!(pool[0].id, 7);
        assert_eq!(pool[0].events, 8);
    }

    #[test]
    fn different_lower_priority_keys_and_lengths_remain_separate_parents() {
        // Numeric masks use ctrl bit 6 (0x40). TAS text is emitted least-significant
        // bit first, so ctrl alone is "00000010", not numeric mask 0x02. All
        // entries share that highest-priority key; pool identity still uses every
        // input bit and the entire length.
        let mut pool = Vec::new();
        assert!(upsert(&mut pool, member(&[64], 25.0, 1, 1, 1)));
        assert!(upsert(&mut pool, member(&[65], 25.0, 2, 2, 2)));
        assert!(upsert(&mut pool, member(&[64, 64], 25.0, 3, 3, 1)));
        assert_eq!(pool.len(), 3);
        assert_eq!(pool.iter().map(|parent| parent.id).collect::<Vec<_>>(), [1, 2, 3]);
        assert_eq!(pool.iter().map(|parent| parent.visits).collect::<Vec<_>>(), [1, 2, 3]);
    }

    #[test]
    fn a_rediscovered_seed_keeps_its_reservoir_identity_and_history() {
        let mut pool = vec![member(&[1, 16], 400.0, 3, 2, 4)];
        assert!(!upsert(&mut pool, member(&[1, 16], 5.0, 0, 99, 999)));
        assert!(upsert(&mut pool, member(&[2, 16], 0.0, 0, 100, 1)));
        trim(&mut pool, 1, Some(10), false);
        assert_eq!(pool.len(), 1);
        assert_eq!(seed_index(&pool, 10, 0), Some(0));
        assert_eq!(pool[0].id, 2);
        assert_eq!(pool[0].score, 5.0);
        assert_eq!(pool[0].visits, 3);
        assert_eq!(pool[0].events, 4);
    }

    #[test]
    fn protected_setups_survive_descendants_with_better_fitness() {
        let mut pool = vec![
            member(&[1], 0.0, 0, 100, 0),
            member(&[2], 400.0, 30, 2, 60),
            member(&[3], 1.0, 0, 101, 0),
            member(&[4], 300.0, 2, 7, 70),
            member(&[5], 2.0, 0, 102, 0),
        ];
        trim(&mut pool, 3, Some(10), true);
        assert_eq!(pool.iter().map(|parent| parent.id).collect::<Vec<_>>(), [2, 7, 100]);
        assert_eq!(pool[0].visits, 30);
        assert_eq!(pool[1].visits, 2);
    }

    #[test]
    fn seed_ordinals_cover_noncontiguous_initial_ids_without_descendants() {
        let pool = vec![
            member(&[1], 0.0, 0, 900, 0),
            member(&[2], 400.0, 9, 7, 10),
            member(&[3], 0.0, 0, 901, 0),
            member(&[4], 300.0, 0, 0, 8),
            member(&[5], 200.0, 100, 18, 7),
            member(&[6], 0.0, 0, 40, 0),
        ];
        let indices = (0..3).map(|ordinal| seed_index(&pool, 20, ordinal)).collect::<Vec<_>>();
        assert_eq!(indices, [Some(1), Some(3), Some(4)]);
        assert_eq!(seed_index(&pool, 20, 3), None);
        assert_eq!(seed_index(&pool, 0, 0), None);
        assert_eq!(seed_index(&[], 20, 0), None);
    }

    #[test]
    fn unprotected_trimming_keeps_fitness_and_visit_ordering() {
        let pool = vec![
            member(&[1], 20.0, 2, 0, 1),
            member(&[2], 0.0, 9, 1, 9),
            member(&[3], 20.0, 1, 2, 1),
            member(&[4], 15.0, 0, 3, 1),
        ];
        let mut plain = pool.clone();
        trim(&mut plain, 3, None, false);
        assert_eq!(plain.iter().map(|parent| parent.id).collect::<Vec<_>>(), [1, 3, 2]);

        let mut weighted = pool;
        trim(&mut weighted, 3, None, true);
        assert_eq!(weighted.iter().map(|parent| parent.id).collect::<Vec<_>>(), [3, 2, 0]);
    }

    #[test]
    fn repeated_parent_gate_never_admits_finishes_or_distinct_outcomes() {
        let mut pool = vec![member(&[64, 65], 10.0, 9, 2, 3)];
        let mut tracked = HashSet::new();
        for (enabled, first, launch, finished) in [
            (false, false, true, false), // default off
            (true, true, true, false), // ordinary distinct launch
            (true, false, true, true), // repeat finish uses finish measurement
            (true, false, false, false), // repeated finish check, no launch
        ] {
            if retain_repeated_launch(enabled, first, launch, finished) {
                insert_tracked(&mut pool, &mut tracked,
                    member(&[64, 66], 5.0, 0, 100, 3), 4, Some(10), false);
            }
            assert_eq!(pool.len(), 1);
            assert_eq!(pool[0].id, 2);
            assert_eq!(pool[0].visits, 9);
            assert_eq!(pool[0].score, 10.0);
            assert!(tracked.is_empty());
        }
        assert!(retain_repeated_launch(true, false, true, false));
    }

    #[test]
    fn shared_geometry_does_not_merge_different_raw_parents() {
        let mut pool = vec![member(&[64, 65], 10.0, 9, 2, 3)];
        let mut tracked = HashSet::new();
        // The caller has already observed the same geometric outcome. Identity
        // below is still every raw input byte, including lower priority keys.
        assert!(retain_repeated_launch(true, false, true, false));
        assert_eq!(insert_tracked(&mut pool, &mut tracked,
            member(&[64, 66], 10.0, 0, 100, 3), 4, Some(10), false),
            Insertion { inserted: true, retained: true });
        assert_eq!(pool.len(), 2);
        assert_eq!(tracked, HashSet::from([100]));
        assert_eq!(insert_tracked(&mut pool, &mut tracked,
            member(&[64, 65], 5.0, 0, 101, 99), 4, Some(10), false),
            Insertion { inserted: false, retained: false });
        assert_eq!(pool[0].id, 2);
        assert_eq!(pool[0].visits, 9);
        assert_eq!(pool[0].score, 5.0);
        assert_eq!(pool[0].events, 3);
        assert_eq!(tracked, HashSet::from([100]));
    }

    #[test]
    fn a_new_parent_can_be_inserted_and_immediately_trimmed() {
        let mut pool = vec![member(&[1], 400.0, 8, 2, 40),
            member(&[2], 10.0, 3, 20, 1)];
        let mut tracked = HashSet::new();
        assert_eq!(insert_tracked(&mut pool, &mut tracked,
            member(&[3], 20.0, 0, 100, 1), 2, Some(10), false),
            Insertion { inserted: true, retained: false });
        assert!(tracked.is_empty());
        assert_eq!(pool.iter().map(|parent| parent.id).collect::<Vec<_>>(), [2, 20]);
        assert_eq!(pool[0].visits, 8);
        assert_eq!(insert_tracked(&mut pool, &mut tracked,
            member(&[4], 5.0, 0, 101, 1), 2, Some(10), false),
            Insertion { inserted: true, retained: true });
        assert_eq!(tracked, HashSet::from([101]));
        assert_eq!(pool.iter().map(|parent| parent.id).collect::<Vec<_>>(), [2, 101]);
    }

    #[test]
    fn pruning_removes_evicted_tags_without_promoting_other_parents() {
        let mut pool = vec![member(&[1], 0.0, 5, 2, 1)];
        let mut tracked = HashSet::new();
        insert_tracked(&mut pool, &mut tracked, member(&[2], 20.0, 0, 100, 1), 2, Some(10), false);
        // A later ordinary discovery replaces the tracked parent. Its new ID
        // does not inherit the repeated-parent tag, even with the same geometry.
        upsert(&mut pool, member(&[3], 10.0, 0, 101, 1));
        trim(&mut pool, 2, Some(10), false);
        assert_eq!(prune_parent_ids(&pool, &mut tracked), 0);
        assert!(tracked.is_empty());
        assert_eq!(pool.iter().map(|parent| parent.id).collect::<Vec<_>>(), [2, 101]);
        insert_tracked(&mut pool, &mut tracked, member(&[4], 5.0, 0, 102, 1), 2, Some(10), false);
        assert_eq!(prune_parent_ids(&pool, &mut tracked), 1);
        assert_eq!(tracked, HashSet::from([102]));
        assert_eq!(prune_parent_ids(&[], &mut tracked), 0);
    }
}
