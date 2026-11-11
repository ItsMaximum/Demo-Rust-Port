//! Host-only Dad/Obstacle Fan-route ranking. No state or physics is changed.
//!
//! Caller contract: these are all six existing clean post-change samples of
//! the same live Dad CHEST and actual level.focus handle. Existing input,
//! liveness, identity, lock, lag and finish gates remain authoritative.
//! Coordinates below describe geometry, not contact/Boost activation proof.
use serde_json::{json, Value};

pub const WINDOW: usize = 6;
// LEVEL_45 shapes72/73 flank the vertical shaft. Boost13 fills their gap:
// center(102.976,74.496), rotation-90, panels4, local half sizes5.76/1.44.
// Use the intersection of Boost's x span and the right wall's inside edge.
pub const SHAFT_X: [f64; 2] = [101.536, 104.408];
pub const SHAFT_ENTRY_Y: f64 = 80.256;
// Fan14 rotation+90: center(103.168,58.496), sensor x99.168..107.168,
// y56.096..60.896. Its lower bound joins the remaining shaft-ascent cost.
pub const FAN_X: [f64; 2] = [99.168, 107.168];
pub const FAN_Y: [f64; 2] = [56.096, 60.896];
pub const FINISH_X: [f64; 2] = [117.312, 123.712];
pub const FINISH_Y: [f64; 2] = [55.504, 58.704];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Region { Approach, Shaft, Fan, UpperExit, FinishRegion }
impl Region {
    fn name(self) -> &'static str { match self {
        Self::Approach => "lower_shaft_approach",
        Self::Shaft => "actual_shaft_coordinates",
        Self::Fan => "actual_fan_band_coordinates",
        Self::UpperExit => "actual_upper_exit_coordinates",
        Self::FinishRegion => "actual_finish_region_coordinates",
    } }
}

#[derive(Clone, Copy, Debug)]
pub struct RouteScore {
    /// Smaller is better; maximum path-to-go of six actual clean samples.
    pub ranking_value: f64,
    pub sample_costs_m: [f64; WINDOW],
    pub positions: [[f64; 2]; WINDOW],
    pub regions: [Region; WINDOW],
}
impl RouteScore {
    pub fn value(self) -> Value { json!({
        "objective":"dad-fan-route-progress-v1",
        "ranking_value":self.ranking_value,
        "ranking_f64_bits":format!("{:016x}",self.ranking_value.to_bits()),
        "smaller_is_better":true,
        "worst_six_route_path_cost_m":self.ranking_value,
        "sample_route_path_costs_m":self.sample_costs_m,
        "sample_actual_focus_positions":self.positions,
        "sample_actual_coordinate_regions":self.regions.map(Region::name),
        "shaft_x":SHAFT_X,"shaft_entry_y":SHAFT_ENTRY_Y,
        "fan_x":FAN_X,"fan_y":FAN_Y,"finish_x":FINISH_X,"finish_y":FINISH_Y,
        "projected_targets_count_as_reached":false,
        "contact_or_boost_activation_proven":false,
        "virtual_trajectory_or_completion_claim":false,
        "finish_claim":false,
        "qualification":"Host path-cost surrogate on six same-live-Dad/focus samples; actual finite finish confirmation and original Flash verification remain separate."
    }) }
}

fn inside(v: f64, range: [f64; 2]) -> bool { v >= range[0] && v <= range[1] }
fn clamp(v: f64, range: [f64; 2]) -> f64 { v.max(range[0]).min(range[1]) }
fn finish_distance(p: [f64; 2]) -> f64 {
    let dx = p[0] - clamp(p[0], FINISH_X);
    let dy = p[1] - clamp(p[1], FINISH_Y);
    (dx * dx + dy * dy).sqrt()
}
fn region(p: [f64; 2]) -> Region {
    if inside(p[0], FINISH_X) && inside(p[1], FINISH_Y) { Region::FinishRegion }
    else if inside(p[0], FAN_X) && inside(p[1], FAN_Y) { Region::Fan }
    else if p[0] > FAN_X[1] && p[0] <= FINISH_X[1]
        && p[1] >= FINISH_Y[0] && p[1] <= FAN_Y[1] { Region::UpperExit }
    else if inside(p[0], SHAFT_X) && p[1] > FAN_Y[1]
        && p[1] <= SHAFT_ENTRY_Y { Region::Shaft }
    else { Region::Approach }
}
fn sample(p: [f64; 2]) -> (f64, Region) {
    let r = region(p);
    let cost = match r {
        Region::FinishRegion | Region::Fan | Region::UpperExit => finish_distance(p),
        Region::Shaft => (p[1] - FAN_Y[1]) + finish_distance([p[0], FAN_Y[1]]),
        Region::Approach => {
            // A closest target is only a distance term, never a reached stage.
            let entry = [clamp(p[0], SHAFT_X), SHAFT_ENTRY_Y];
            let dx = p[0] - entry[0]; let dy = p[1] - entry[1];
            (dx * dx + dy * dy).sqrt() + (SHAFT_ENTRY_Y - FAN_Y[1])
                + finish_distance([entry[0], FAN_Y[1]])
        }
    };
    (cost, r)
}

/// No history is inherited: only the supplied six actual clean coordinates
/// contribute. Caller still enforces earliest_changed_index and body identity.
pub fn score(positions: [[f64; 2]; WINDOW]) -> Option<RouteScore> {
    if positions.iter().flatten().any(|v| !v.is_finite()) { return None; }
    let samples = positions.map(sample);
    let sample_costs_m = samples.map(|s| s.0);
    if sample_costs_m.iter().any(|c| !c.is_finite() || *c < 0.0) { return None; }
    let ranking_value = sample_costs_m.iter().copied().fold(0.0_f64, f64::max);
    Some(RouteScore { ranking_value, sample_costs_m, positions,
        regions: samples.map(|s| s.1) })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn at(p: [f64; 2]) -> RouteScore { score([p; WINDOW]).unwrap() }

    #[test] fn approaching_shaft_beats_the_old_floor_endpoint() {
        let old_floor = at([80.90892070807715, 82.92967190877987]);
        let approach = at([99.0, 82.9]);
        let entry = at([103.0, SHAFT_ENTRY_Y]);
        assert!(approach.ranking_value < old_floor.ranking_value);
        assert!(entry.ranking_value < approach.ranking_value);
        assert_eq!(old_floor.regions, [Region::Approach; WINDOW]);
    }
    #[test] fn real_shaft_ascent_then_upper_exit_reduce_cost() {
        let lower = at([103.0, 79.0]);
        let upper = at([103.0, 65.0]);
        let fan = at([103.0, 58.496]);
        let exit = at([113.0, 58.496]);
        assert!(upper.ranking_value < lower.ranking_value);
        assert!(fan.ranking_value < upper.ranking_value);
        assert!(exit.ranking_value < fan.ranking_value);
        assert_eq!(lower.regions, [Region::Shaft; WINDOW]);
        assert_eq!(fan.regions, [Region::Fan; WINDOW]);
        assert_eq!(exit.regions, [Region::UpperExit; WINDOW]);
    }
    #[test] fn one_frame_in_fan_cannot_replace_six_real_route_samples() {
        let floor = [80.90892070807715, 82.92967190877987];
        let mut positions = [floor; WINDOW]; positions[5] = [103.0, 58.496];
        let result = score(positions).unwrap();
        assert_eq!(result.ranking_value.to_bits(), at(floor).ranking_value.to_bits());
        assert_eq!(result.regions[0], Region::Approach);
        assert_eq!(result.regions[5], Region::Fan);
    }
    #[test] fn projection_is_not_shaft_occupancy_and_child_input_is_not_inferred() {
        let left_of_wall = at([100.0, 70.0]);
        assert_eq!(left_of_wall.regions, [Region::Approach; WINDOW]);
        assert!(left_of_wall.ranking_value > at([103.0, 70.0]).ranking_value);
    }
    #[test] fn joins_have_the_exact_same_cost_expression_at_boundary() {
        let x = 103.0;
        let entry = at([x, SHAFT_ENTRY_Y]);
        let expected = (SHAFT_ENTRY_Y - FAN_Y[1]) + finish_distance([x, FAN_Y[1]]);
        assert_eq!(entry.ranking_value.to_bits(), expected.to_bits());
        let exit = at([x, FAN_Y[1]]);
        assert_eq!(exit.ranking_value.to_bits(), finish_distance([x, FAN_Y[1]]).to_bits());
    }
    #[test] fn geometry_zero_is_never_a_finish_and_nonfinite_input_is_rejected() {
        let zero = at([120.0, 57.0]);
        assert_eq!(zero.ranking_value.to_bits(), 0.0_f64.to_bits());
        assert_eq!(zero.value()["finish_claim"], false);
        let mut p = [[103.0, 70.0]; WINDOW]; p[2][0] = f64::NAN;
        assert!(score(p).is_none());
        p[2][0] = f64::INFINITY; assert!(score(p).is_none());
    }
}
