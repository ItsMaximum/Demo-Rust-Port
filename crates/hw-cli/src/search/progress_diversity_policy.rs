//! Pure host reserve admission; independent from launch-pace ranking.
//!
//! The caller owns exact raw-literal deduplication across the entire reserve.
//! Equal costs may represent distinct literals and must not be deduplicated here.
//! Positions and costs must come from the same replay's eligible living-Dad score.
//! This policy makes no simulation, finish, survival, or original-player claim.

use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CellKey {
    pub x: i16,
    pub y: i16,
    pub band: u8,
}

pub fn key(position: [f64; 2], frame: u32) -> Option<CellKey> {
    let [x, y] = position;
    if !x.is_finite() || !y.is_finite()
        || !(x > -4.0 && x < 324.0 && y > -84.0 && y < 164.0)
        || !(1..=299).contains(&frame)
    {
        return None;
    }
    Some(CellKey {
        x: (x / 4.0).floor() as i16,
        y: (y / 4.0).floor() as i16,
        band: ((frame - 1) / 30) as u8,
    })
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Resident {
    pub key: CellKey,
    pub route_cost: f64,
    pub visits: u64,
    pub id: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Admission {
    Append,
    Replace(usize),
    BelowCellFloor,
}

// Earlier IDs are older. Exact ties retain the earliest slice index.
fn more_visited_or_older(candidate: Resident, current: Resident) -> bool {
    candidate.visits > current.visits
        || (candidate.visits == current.visits && candidate.id < current.id)
}

pub fn admission(
    residents: &[Resident],
    incoming: CellKey,
    route_cost: f64,
    cap: usize,
) -> Admission {
    if cap == 0 || residents.len() > cap || !route_cost.is_finite() || route_cost < 0.0 {
        return Admission::BelowCellFloor;
    }

    let mut counts = BTreeMap::new();
    for resident in residents {
        if !resident.route_cost.is_finite() || resident.route_cost < 0.0 {
            return Admission::BelowCellFloor;
        }
        let count = counts.entry(resident.key).or_insert(0_usize);
        *count += 1;
        if *count > 2 {
            return Admission::BelowCellFloor;
        }
    }

    if counts.get(&incoming).copied().unwrap_or(0) == 2 {
        let mut worst = None;
        for (index, resident) in residents.iter().copied().enumerate() {
            if resident.key != incoming {
                continue;
            }
            if worst.map_or(true, |current: usize| {
                resident.route_cost > residents[current].route_cost
                    || (resident.route_cost == residents[current].route_cost
                        && more_visited_or_older(resident, residents[current]))
            }) {
                worst = Some(index);
            }
        }
        let worst = worst.expect("two residents in incoming cell");
        return if route_cost > residents[worst].route_cost {
            Admission::BelowCellFloor
        } else {
            Admission::Replace(worst)
        };
    }

    if residents.len() < cap {
        return Admission::Append;
    }

    // There is cell room but no global room: preserve a new trajectory cell
    // without applying any global route-cost floor or launch-pace comparison.
    let most_crowded = counts.values().copied().max().expect("nonempty full reserve");
    let mut victim = None;
    for (index, resident) in residents.iter().copied().enumerate() {
        if counts[&resident.key] != most_crowded {
            continue;
        }
        if victim.map_or(true, |current: usize| more_visited_or_older(resident, residents[current])) {
            victim = Some(index);
        }
    }
    Admission::Replace(victim.expect("resident in most crowded cell"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cell(x: i16) -> CellKey {
        CellKey { x, y: 0, band: 0 }
    }

    fn resident(x: i16, route_cost: f64, visits: u64, id: u64) -> Resident {
        Resident { key: cell(x), route_cost, visits, id }
    }

    #[test]
    fn later_route_in_new_cell_survives_without_global_pace_or_cost_floor() {
        let residents = [
            resident(0, 1.0, 2, 10),
            resident(0, 2.0, 8, 11),
            resident(1, 0.5, 99, 1),
        ];
        // A high route cost is still eligible for cell coverage. The singleton
        // is preserved even though it has more visits than the crowded cell.
        assert_eq!(admission(&residents, cell(2), 120.0, 3), Admission::Replace(1));
    }

    #[test]
    fn same_cell_uses_only_its_worst_cost_and_allows_equal_cost_new_literal() {
        let residents = [resident(0, 10.0, 99, 1), resident(0, 20.0, 0, 2)];
        assert_eq!(admission(&residents, cell(0), 21.0, 4), Admission::BelowCellFloor);
        assert_eq!(admission(&residents, cell(0), 20.0, 4), Admission::Replace(1));
        assert_eq!(admission(&residents, cell(0), 19.0, 4), Admission::Replace(1));
        // The caller must reject an exact repeated literal before this policy.
    }

    #[test]
    fn equal_same_cell_cost_evicts_most_visited_then_oldest() {
        let visits = [resident(0, 10.0, 3, 1), resident(0, 10.0, 4, 9)];
        assert_eq!(admission(&visits, cell(0), 10.0, 2), Admission::Replace(1));
        let age = [resident(0, 10.0, 4, 9), resident(0, 10.0, 4, 1)];
        assert_eq!(admission(&age, cell(0), 10.0, 2), Admission::Replace(1));
    }

    #[test]
    fn global_eviction_uses_crowding_then_visits_then_age() {
        let residents = [
            resident(0, 999.0, 1, 10),
            resident(0, 999.0, 5, 11),
            resident(1, 1.0, 7, 12),
            resident(1, 0.0, 7, 3),
            resident(2, 9999.0, 100, 0),
        ];
        assert_eq!(admission(&residents, cell(3), 10000.0, 5), Admission::Replace(3));
        assert_eq!(admission(&residents[..4], cell(3), 10000.0, 5), Admission::Append);
    }

    #[test]
    fn cell_room_can_append_but_a_full_cell_never_gets_a_third_member() {
        let one = [resident(0, 1.0, 0, 1)];
        assert_eq!(admission(&one, cell(0), 2.0, 2), Admission::Append);
        assert_eq!(admission(&one, cell(1), 2.0, 1), Admission::Replace(0));
        let two = [resident(0, 1.0, 0, 1), resident(0, 2.0, 0, 2)];
        assert_eq!(admission(&two, cell(0), 2.0, 3), Admission::Replace(1));
    }

    #[test]
    fn invalid_cost_capacity_or_existing_reserve_fails_closed() {
        for cost in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0] {
            assert_eq!(admission(&[], cell(0), cost, 2), Admission::BelowCellFloor);
        }
        assert_eq!(admission(&[], cell(0), 0.0, 0), Admission::BelowCellFloor);
        let two = [resident(0, 1.0, 0, 1), resident(0, 2.0, 0, 2)];
        assert_eq!(admission(&two, cell(1), 0.0, 1), Admission::BelowCellFloor);
        let invalid = [resident(0, f64::NAN, 0, 1)];
        assert_eq!(admission(&invalid, cell(1), 0.0, 2), Admission::BelowCellFloor);
        let over_cell = [resident(0, 1.0, 0, 1), resident(0, 2.0, 0, 2), resident(0, 3.0, 0, 3)];
        assert_eq!(admission(&over_cell, cell(1), 0.0, 4), Admission::BelowCellFloor);
    }

    #[test]
    fn negative_cells_and_time_bands_use_floor_and_inclusive_frame_bounds() {
        assert_eq!(key([-0.01, -0.01], 1), Some(CellKey { x: -1, y: -1, band: 0 }));
        assert_eq!(key([-3.99, -83.99], 30), Some(CellKey { x: -1, y: -21, band: 0 }));
        assert_eq!(key([4.0, 8.0], 31), Some(CellKey { x: 1, y: 2, band: 1 }));
        assert_eq!(key([323.99, 163.99], 299), Some(CellKey { x: 80, y: 40, band: 9 }));
    }

    #[test]
    fn nonfinite_outside_world_or_outside_frame_range_has_no_cell() {
        for position in [
            [-4.0, 0.0], [324.0, 0.0], [0.0, -84.0], [0.0, 164.0],
            [f64::NAN, 0.0], [0.0, f64::INFINITY],
        ] {
            assert_eq!(key(position, 1), None);
        }
        assert_eq!(key([0.0, 0.0], 0), None);
        assert_eq!(key([0.0, 0.0], 300), None);
    }
}
