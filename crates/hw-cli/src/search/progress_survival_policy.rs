//! Terminal survival admission for Dad launch fitness; no simulation or scoring.
//!
//! Caller contract: handle and independently confirm every observed finish BEFORE
//! this gate. A finish must not be discarded because terminal survival is false.
//! These fields must all describe the same freshly evaluated literal replay.
//! Parent, child-actor, or earlier score-window evidence must never be substituted.
//! This module does not score finishes, certify original Flash, or change physics.

pub(crate) const REQUIRED_WORLD_FRAMES: u32 = 299;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct TerminalEvidence {
    /// True only for completed evaluation with reason "ceiling", a declared
    /// horizon of 299, and the same replay's full actual 299-frame evaluation.
    /// STOP, budget cancellation, or another early termination must leave false,
    /// even if the last recorded world-frame counter happens to equal 299.
    pub(crate) evaluated_to_horizon: bool,
    /// Actual world frames evaluated, not TAS length or consumed-input iteration.
    pub(crate) actual_world_frames: u32,
    /// Primary Dad's terminal state, independent of child_active / active alias.
    pub(crate) dad_alive: bool,
    /// Primary Dad's CHEST exists and is live in the actual terminal world.
    pub(crate) chest_live: bool,
    /// Primary CHEST's finite terminal position passes the caller's world bounds.
    pub(crate) chest_in_world: bool,
    /// ctx.level.focus is the same live primary CHEST, not child debris or heart.
    pub(crate) actual_focus_is_primary_chest: bool,
    /// Sticky for this replay: any host port abort prevents admission.
    pub(crate) port_abort: bool,
    /// True if this evaluation terminated at the host lag cutoff.
    pub(crate) lag_stopped: bool,
}

/// Survival eligibility only. A recorded launch remains diagnostic when false.
pub(crate) fn eligible(evidence: TerminalEvidence) -> bool {
    evidence.evaluated_to_horizon
        && evidence.actual_world_frames == REQUIRED_WORLD_FRAMES
        && evidence.dad_alive
        && evidence.chest_live
        && evidence.chest_in_world
        && evidence.actual_focus_is_primary_chest
        && !evidence.port_abort
        && !evidence.lag_stopped
}

#[cfg(test)]
mod tests {
    use super::*;

    fn living_at_299() -> TerminalEvidence {
        TerminalEvidence {
            evaluated_to_horizon: true,
            actual_world_frames: REQUIRED_WORLD_FRAMES,
            dad_alive: true,
            chest_live: true,
            chest_in_world: true,
            actual_focus_is_primary_chest: true,
            port_abort: false,
            lag_stopped: false,
        }
    }

    #[test]
    fn known_fatal_90_launch_does_not_inherit_its_live_score_window() {
        // The saved 2035 launch had a live score window ending at 89, then died
        // at world/iteration 90. Only the terminal evidence admits a parent.
        let fatal = TerminalEvidence {
            evaluated_to_horizon: false,
            actual_world_frames: 90,
            dad_alive: false,
            chest_live: false,
            chest_in_world: false,
            actual_focus_is_primary_chest: false,
            ..living_at_299()
        };
        assert!(!eligible(fatal));
        assert!(eligible(living_at_299()));
    }

    #[test]
    fn each_terminal_failure_vetoes_otherwise_living_evidence() {
        let live = living_at_299();
        let cases = [
            ("early horizon", TerminalEvidence { actual_world_frames: 298, ..live }),
            ("beyond declared horizon", TerminalEvidence { actual_world_frames: 300, ..live }),
            ("STOP at 299", TerminalEvidence { evaluated_to_horizon: false, ..live }),
            ("dead Dad", TerminalEvidence { dad_alive: false, ..live }),
            ("removed CHEST", TerminalEvidence { chest_live: false, ..live }),
            ("CHEST out of world", TerminalEvidence { chest_in_world: false, ..live }),
            ("different finish focus", TerminalEvidence { actual_focus_is_primary_chest: false, ..live }),
            ("host port abort", TerminalEvidence { port_abort: true, ..live }),
            ("lag stop", TerminalEvidence { lag_stopped: true, ..live }),
        ];
        for (name, evidence) in cases {
            assert!(!eligible(evidence), "{name}");
        }
    }

    #[test]
    fn living_child_or_camera_focus_cannot_replace_primary_dad_chest() {
        let child_focus = TerminalEvidence {
            // Dad and his CHEST can still be live while the finish focus differs.
            actual_focus_is_primary_chest: false,
            ..living_at_299()
        };
        assert!(!eligible(child_focus));

        let only_child_live = TerminalEvidence {
            dad_alive: false,
            chest_live: false,
            actual_focus_is_primary_chest: false,
            ..living_at_299()
        };
        assert!(!eligible(only_child_live));
    }

    #[test]
    fn full_input_length_cannot_certify_partial_evaluation() {
        // A full 299-entry input can be stopped without its tail being observed.
        let partial = TerminalEvidence {
            evaluated_to_horizon: false,
            actual_world_frames: 58,
            ..living_at_299()
        };
        assert!(!eligible(partial));
    }
}
