//! Host-only deadline/launch-pace ranking; no simulation changes.
//!
//! Included by progress.rs; its route_policy sibling supplies geometry.
//! Caller must supply six existing clean consecutive post-change samples with
//! the same live primary Dad CHEST == level.focus handle. Velocities must be
//! that body's actual linear velocities, never Frame's active actor or QE.
//! Finish observation/confirmation remains before these fitness gates.
use super::route_policy::{self, Region, FAN_Y, FINISH_X, FINISH_Y, SHAFT_ENTRY_Y, SHAFT_X};
use serde_json::{json, Value};

pub const WINDOW: usize = 6;
pub const MAX_FRAME: u32 = 299;
pub const FRAMES_PER_SECOND: f64 = 30.0;

#[derive(Clone, Copy, Debug)]
pub struct LaunchScore {
    /// Smaller is better: the worst required-pace minus measured forward speed.
    /// Negative means a positive projected-pace surplus, not feasible arrival.
    pub ranking_value: f64,
    pub route_costs_m: [f64; WINDOW],
    pub required_pace_mps: [f64; WINDOW],
    pub projected_chest_speed_mps: [f64; WINDOW],
    pub pace_deficits_mps: [f64; WINDOW],
    pub positions: [[f64; 2]; WINDOW],
    pub chest_velocities: [[f64; 2]; WINDOW],
    pub world_frames: [u32; WINDOW],
    pub actual_coordinate_regions: [Region; WINDOW],
}
impl LaunchScore {
    pub fn value(self) -> Value { json!({
        "objective":"dad-route-launch-pace-v1",
        "ranking_value":self.ranking_value,
        "ranking_f64_bits":format!("{:016x}",self.ranking_value.to_bits()),
        "smaller_is_better":true,
        "ranking_units":"m/s surrogate: worst of six required path pace minus measured route-leg speed",
        "sample_route_costs_m":self.route_costs_m,
        "sample_required_pace_mps":self.required_pace_mps,
        "sample_projected_chest_speed_mps":self.projected_chest_speed_mps,
        "sample_pace_deficits_mps":self.pace_deficits_mps,
        "sample_actual_focus_positions":self.positions,
        "sample_actual_primary_chest_velocities":self.chest_velocities,
        "sample_world_frames":self.world_frames,
        "sample_actual_coordinate_regions":self.actual_coordinate_regions.map(region_name),
        "fitness_frame_ceiling_exclusive":MAX_FRAME,
        "uses_QE_or_detached_body_velocity":false,
        "projected_target_counts_as_reached":false,
        "contact_or_boost_activation_proven":false,
        "projected_arrival_or_virtual_trajectory_claim":false,
        "finish_claim":false,
        "qualification":"Measured six-frame Dad velocity projected onto a geometry route leg, compared with remaining path/time demand. No prediction that speed persists, terrain is cleared, or the finish is reachable."
    }) }
}
fn region_name(r: Region) -> &'static str { match r {
    Region::Approach => "lower_shaft_approach",
    Region::Shaft => "actual_shaft_coordinates",
    Region::Fan => "actual_fan_band_coordinates",
    Region::UpperExit => "actual_upper_exit_coordinates",
    Region::FinishRegion => "actual_finish_region_coordinates",
} }
fn clamp(v: f64, range: [f64; 2]) -> f64 { v.max(range[0]).min(range[1]) }
fn forward(p: [f64; 2], velocity: [f64; 2], region: Region) -> f64 {
    let target = match region {
        Region::Approach => [clamp(p[0], SHAFT_X), SHAFT_ENTRY_Y],
        Region::Shaft => [p[0], FAN_Y[1]],
        Region::Fan | Region::UpperExit | Region::FinishRegion =>
            [clamp(p[0], FINISH_X), clamp(p[1], FINISH_Y)],
    };
    let dx = target[0] - p[0]; let dy = target[1] - p[1];
    if dx == 0.0 && dy != 0.0 { return if dy > 0.0 { velocity[1] } else { -velocity[1] }; }
    if dy == 0.0 && dx != 0.0 { return if dx > 0.0 { velocity[0] } else { -velocity[0] }; }
    let length = (dx * dx + dy * dy).sqrt();
    // At an exact geometric target there is no defined directional bonus.
    if length == 0.0 { 0.0 }
    else { velocity[0] * (dx / length) + velocity[1] * (dy / length) }
}

/// Existing eligibility/cutoff/identity gates are mandatory caller obligations.
/// This adds no stateful history or extrapolated positions. Every sample uses
/// its own actual worldframe and the same299 deadline; EOF299 gets no fitness.
pub fn score(positions: [[f64; 2]; WINDOW], chest_velocities: [[f64; 2]; WINDOW],
    world_frames: [u32; WINDOW]) -> Option<LaunchScore> {
    if chest_velocities.iter().flatten().any(|v| !v.is_finite())
        || world_frames.iter().any(|&f| f == 0 || f >= MAX_FRAME)
        || world_frames.windows(2).any(|f| f[1] != f[0] + 1) { return None; }
    let geometry = route_policy::score(positions)?;
    let mut required_pace_mps = [0.0; WINDOW];
    let mut projected_chest_speed_mps = [0.0; WINDOW];
    let mut pace_deficits_mps = [0.0; WINDOW];
    for i in 0..WINDOW {
        let seconds_left = f64::from(MAX_FRAME - world_frames[i]) / FRAMES_PER_SECOND;
        required_pace_mps[i] = geometry.sample_costs_m[i] / seconds_left;
        projected_chest_speed_mps[i] = forward(positions[i], chest_velocities[i], geometry.regions[i]);
        pace_deficits_mps[i] = required_pace_mps[i] - projected_chest_speed_mps[i];
        if !required_pace_mps[i].is_finite() || !projected_chest_speed_mps[i].is_finite()
            || !pace_deficits_mps[i].is_finite() { return None; }
    }
    let ranking_value = pace_deficits_mps.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    Some(LaunchScore { ranking_value, route_costs_m:geometry.sample_costs_m,
        required_pace_mps, projected_chest_speed_mps, pace_deficits_mps,
        positions, chest_velocities, world_frames, actual_coordinate_regions:geometry.regions })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn frames(first: u32) -> [u32; WINDOW] { std::array::from_fn(|i| first + i as u32) }

    #[test] fn equal_motion_later_in_the_budget_has_more_required_pace() {
        let p = [[70.0, 83.0]; WINDOW]; let v = [[25.0, 0.0]; WINDOW];
        let early = score(p,v,frames(150)).unwrap(); let late = score(p,v,frames(280)).unwrap();
        assert!(early.ranking_value < late.ranking_value);
        assert_eq!(early.projected_chest_speed_mps.map(f64::to_bits),late.projected_chest_speed_mps.map(f64::to_bits));
    }
    #[test] fn saved_native178_burst_is_retained_ahead_of_late_floor_drift() {
        // Exact named-Dad words from saved native02 projection; no Flash/finish claim.
        let p = [[29.020767961267367,68.88914501382166],[30.940332482797757,69.2820340593845],
            [32.88013855418307,69.54228169234068],[34.59654684516831,69.6911473547274],
            [36.0929639109099,69.76892398905913],[37.47145591617495,69.80079652240826]];
        let v = [[70.930150210356,17.75187454746309],[55.12366462222046,13.089585730888363],
            [59.046436206460186,9.195721652698314],[52.0458412848709,5.020581082535274],
            [45.18355037445152,3.314962121562956],[41.61984128542059,1.0725314753397084]];
        let burst = score(p,v,frames(178)).unwrap();
        let floor = score([[80.90892070807715,82.92967190877987];WINDOW],
            [[4.9243160264700405,0.05993248354758149];WINDOW],frames(293)).unwrap();
        assert!(burst.ranking_value < 0.0); assert!(burst.ranking_value < floor.ranking_value);
        assert!(burst.actual_coordinate_regions.iter().all(|&r|r==Region::Approach));
    }
    #[test] fn one_velocity_spike_cannot_replace_sustained_six_sample_pace() {
        let p = [[29.0,69.0]; WINDOW]; let mut v = [[0.0,0.0]; WINDOW]; v[0]=[700.0,0.0];
        let spike = score(p,v,frames(175)).unwrap();
        let baseline = score(p,[[0.0,0.0];WINDOW],frames(175)).unwrap();
        assert_eq!(spike.ranking_value.to_bits(),baseline.ranking_value.to_bits());
    }
    #[test] fn up_is_useful_in_shaft_but_down_is_not() {
        let p = [[103.0,72.0]; WINDOW];
        let up = score(p,[[0.0,-20.0];WINDOW],frames(210)).unwrap();
        let down = score(p,[[0.0,20.0];WINDOW],frames(210)).unwrap();
        assert!(up.ranking_value < down.ranking_value);
        assert_eq!(up.projected_chest_speed_mps.map(f64::to_bits),[20.0_f64.to_bits();WINDOW]);
    }
    #[test] fn missing_worldsteps_nonfinite_velocity_and299_get_no_fitness() {
        let p = [[103.0,72.0]; WINDOW]; let v = [[0.0,-20.0];WINDOW];
        let mut f = frames(210); f[2]+=1; assert!(score(p,v,f).is_none());
        assert!(score(p,v,frames(294)).is_none());
        let mut bad = v; bad[2][0]=f64::NAN; assert!(score(p,bad,frames(210)).is_none());
    }
    #[test] fn zero_geometry_and_negative_pace_are_not_virtual_completion() {
        let s = score([[120.0,57.0];WINDOW],[[90.0,0.0];WINDOW],frames(210)).unwrap();
        assert_eq!(s.ranking_value.to_bits(),0.0_f64.to_bits());
        let value=s.value();assert_eq!(value["finish_claim"],false);
        assert_eq!(value["projected_arrival_or_virtual_trajectory_claim"],false);
        assert_eq!(value["projected_target_counts_as_reached"],false);
    }
}
