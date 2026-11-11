//! Optional host proposal filter for Dad's already-inert eject controls.
//!
//! Used by the host progress search. It never changes a TAS, physics state,
//! EOF, first-change index, or score. A certificate belongs
//! to one exact full literal and is rebuilt from that literal's fresh replay.
//! Call `observe` for every actual pre-input invocation, including retries.

pub const MAX_ENTRIES: usize = 299;
const SHIFT: u8 = 1 << 5;
const CTRL: u8 = 1 << 6;
const Z: u8 = 1 << 7;

#[derive(Clone, Copy, Debug, Default)]
pub struct PreInputState {
    pub dad_dead: bool,
    pub dad_ejected: bool,
    pub child_dead: bool,
    pub child_ejected: bool,
}

impl PreInputState {
    fn noop_keys(self) -> u8 {
        let dad = self.dad_dead || self.dad_ejected;
        let child = self.child_dead || self.child_ejected;
        (if dad { SHIFT } else { 0 })
            | (if child { CTRL } else { 0 })
            | (if dad && child { Z } else { 0 })
    }
}

/// Values captured around one returned Sim step, without changing that step.
#[derive(Clone, Copy, Debug)]
pub struct Invocation {
    pub input_index: usize,
    pub requested_mask: u8,
    pub input_allowed_before: bool,
    pub iteration_after: usize,
    pub port_abort: bool,
    pub before: PreInputState,
}

pub struct CertificateBuilder {
    literal: [u8; MAX_ENTRIES],
    len: usize,
    seen: [bool; MAX_ENTRIES],
    noop_keys: [u8; MAX_ENTRIES],
    tracking_valid: bool,
}

impl CertificateBuilder {
    pub fn new(literal: &[u8]) -> Result<Self, &'static str> {
        if literal.is_empty() || literal.len() > MAX_ENTRIES {
            return Err("eject certificate requires 1..299 exact planned entries");
        }
        let mut saved = [0; MAX_ENTRIES];
        saved[..literal.len()].copy_from_slice(literal);
        Ok(Self {
            literal: saved,
            len: literal.len(),
            seen: [false; MAX_ENTRIES],
            noop_keys: [0; MAX_ENTRIES],
            tracking_valid: true,
        })
    }

    /// The intersection includes failed/no-progress attempts at a planned slot.
    /// Such attempts contribute zero, so a later successful retry cannot revive
    /// that slot. A host port failure invalidates the whole proof, including at
    /// EOF. Past-EOF requests never create a planned-input certificate.
    pub fn observe(&mut self, invocation: Invocation) {
        if invocation.port_abort {
            self.tracking_valid = false;
        }
        let i = invocation.input_index;
        if i >= self.len {
            return;
        }
        if invocation.requested_mask != self.literal[i] {
            // This trace may belong to another literal; invalidate all slots.
            self.tracking_valid = false;
        }
        let normal_request = invocation.input_allowed_before
            && invocation.requested_mask == self.literal[i]
            && i.checked_add(1) == Some(invocation.iteration_after)
            && !invocation.port_abort;
        let keys = if normal_request { invocation.before.noop_keys() } else { 0 };
        if self.seen[i] {
            self.noop_keys[i] &= keys;
        } else {
            self.seen[i] = true;
            self.noop_keys[i] = keys;
        }
    }

    /// Use on a missing observation, replay error, or an uncertain input request.
    pub fn invalidate(&mut self) {
        self.tracking_valid = false;
    }

    /// Consumes the builder so later observations cannot silently modify proof.
    /// Unobserved slots stay zero. STOP/short observed horizons certify no tail.
    pub fn seal(self) -> NeutralEjectCertificate {
        NeutralEjectCertificate {
            literal: self.literal,
            len: self.len,
            noop_keys: if self.tracking_valid { self.noop_keys } else { [0; MAX_ENTRIES] },
        }
    }
}

#[derive(Clone)]
pub struct NeutralEjectCertificate {
    literal: [u8; MAX_ENTRIES],
    len: usize,
    noop_keys: [u8; MAX_ENTRIES],
}

impl NeutralEjectCertificate {
    /// Reject only a non-identical, equal-length proposal whose every changed
    /// bit was certified inert in its exact parent. Mixed/active edits and any
    /// length change retain the existing replay and raw first-change cutoff.
    pub fn rejects_neutral_only(&self, parent: &[u8], child: &[u8]) -> bool {
        if parent.len() != self.len || child.len() != self.len
            || parent != &self.literal[..self.len]
        {
            return false;
        }
        let mut changed = false;
        for i in 0..self.len {
            let difference = parent[i] ^ child[i];
            if difference != 0 {
                changed = true;
                if difference & !self.noop_keys[i] != 0 {
                    return false;
                }
            }
        }
        changed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observation(index: usize, mask: u8, before: PreInputState) -> Invocation {
        Invocation { input_index:index, requested_mask:mask, input_allowed_before:true,
            iteration_after:index+1, port_abort:false, before }
    }
    fn both_ejected() -> PreInputState {
        PreInputState { dad_ejected:true, child_ejected:true, ..PreInputState::default() }
    }
    fn one_slot(parent: &[u8], index: usize, state: PreInputState) -> NeutralEjectCertificate {
        let mut builder=CertificateBuilder::new(parent).unwrap();
        builder.observe(observation(index,parent[index],state));
        builder.seal()
    }

    #[test]
    fn first_ejection_and_active_controls_are_never_rejected() {
        let parent=vec![16; 80];
        let mounted=one_slot(&parent,65,PreInputState::default());
        let ejected=one_slot(&parent,65,both_ejected());
        for bit in [SHIFT,CTRL,Z] {
            let mut child=parent.clone();child[65]^=bit;
            assert!(!mounted.rejects_neutral_only(&parent,&child));
            assert!(ejected.rejects_neutral_only(&parent,&child));
        }
        for bit in [1,2,4,8,16] {
            let mut child=parent.clone();child[65]^=bit;
            assert!(!ejected.rejects_neutral_only(&parent,&child));
        }
    }

    #[test]
    fn dad_and_child_actions_remain_distinct() {
        let parent=vec![16; 80];
        let dad=one_slot(&parent,65,PreInputState {dad_ejected:true,..PreInputState::default()});
        let child_cert=one_slot(&parent,65,PreInputState {child_dead:true,..PreInputState::default()});
        for (bit,dad_expected,child_expected) in [(SHIFT,true,false),(CTRL,false,true),(Z,false,false)] {
            let mut child=parent.clone();child[65]^=bit;
            assert_eq!(dad.rejects_neutral_only(&parent,&child),dad_expected);
            assert_eq!(child_cert.rejects_neutral_only(&parent,&child),child_expected);
        }
    }

    #[test]
    fn retries_intersect_every_pre_input_state() {
        let parent=vec![16; 80];let mut child=parent.clone();child[65]^=SHIFT;
        let mut builder=CertificateBuilder::new(&parent).unwrap();
        builder.observe(observation(65,16,both_ejected()));
        builder.observe(observation(65,16,PreInputState::default()));
        builder.observe(observation(65,16,both_ejected()));
        assert!(!builder.seal().rejects_neutral_only(&parent,&child));
    }

    #[test]
    fn no_progress_abort_or_disallowed_request_cannot_certify_a_slot() {
        let parent=vec![16; 80];let mut child=parent.clone();child[65]^=SHIFT;
        for kind in 0..3 {
            let mut builder=CertificateBuilder::new(&parent).unwrap();
            builder.observe(observation(65,16,both_ejected()));
            let mut bad=observation(65,16,both_ejected());
            match kind {0=>bad.iteration_after=65,1=>bad.port_abort=true,_=>bad.input_allowed_before=false}
            builder.observe(bad);builder.observe(observation(65,16,both_ejected()));
            assert!(!builder.seal().rejects_neutral_only(&parent,&child));
        }
    }

    #[test]
    fn host_port_failure_invalidates_previously_certified_slots_including_at_eof() {
        let parent=vec![16; 80];let mut child=parent.clone();child[65]^=SHIFT;
        for failure_index in [66,80] {
            let mut builder=CertificateBuilder::new(&parent).unwrap();
            builder.observe(observation(65,16,both_ejected()));
            let mut failure=observation(failure_index,if failure_index<80 {16} else {0},both_ejected());
            failure.port_abort=true;builder.observe(failure);
            assert!(!builder.seal().rejects_neutral_only(&parent,&child));
        }
    }

    #[test]
    fn mixed_edits_are_preserved_with_their_original_raw_cutoff() {
        let parent=vec![16; 200];let cert=one_slot(&parent,151,both_ejected());
        let mut child=parent.clone();child[151]^=SHIFT;child[177]^=8;
        let original_first_change=parent.iter().zip(&child).position(|(a,b)|a!=b);
        assert!(!cert.rejects_neutral_only(&parent,&child));
        assert_eq!(original_first_change,Some(151));
        assert_eq!(parent[151],16);assert_eq!(child[151],16^SHIFT);
    }

    #[test]
    fn literal_binding_forbids_ancestor_or_length_certificate_reuse() {
        let parent=vec![16; 80];let cert=one_slot(&parent,65,both_ejected());
        let mut other_parent=parent.clone();other_parent[63]^=8;
        let mut child=other_parent.clone();child[65]^=SHIFT;
        assert!(!cert.rejects_neutral_only(&other_parent,&child));
        let mut short=parent.clone();short.pop();
        let mut long=parent.clone();long.push(16);
        assert!(!cert.rejects_neutral_only(&parent,&short));
        assert!(!cert.rejects_neutral_only(&parent,&long));
        assert!(!cert.rejects_neutral_only(&parent,&parent));
    }

    #[test]
    fn unknown_tail_and_invalid_tracking_fail_closed() {
        let parent=vec![16; 80];let mut child=parent.clone();child[70]^=CTRL;
        let cert=one_slot(&parent,65,both_ejected());
        assert!(!cert.rejects_neutral_only(&parent,&child));
        for explicit_invalid in [false,true] {
            let mut builder=CertificateBuilder::new(&parent).unwrap();
            builder.observe(observation(70,16,both_ejected()));
            if explicit_invalid {builder.invalidate();}
            else {builder.observe(observation(65,16^8,both_ejected()));}
            assert!(!builder.seal().rejects_neutral_only(&parent,&child));
        }
    }

    #[test]
    fn max_literal_and_past_eof_observations_preserve_all_bytes() {
        let parent=vec![16;MAX_ENTRIES];let mut child=parent.clone();child[298]^=SHIFT|CTRL|Z;
        let before=child.clone();let mut builder=CertificateBuilder::new(&parent).unwrap();
        builder.observe(observation(298,16,both_ejected()));
        builder.observe(observation(299,0,PreInputState::default()));
        assert!(builder.seal().rejects_neutral_only(&parent,&child));
        assert_eq!(child,before);assert_eq!(parent.len(),299);
        assert!(CertificateBuilder::new(&[]).is_err());
        assert!(CertificateBuilder::new(&vec![0;300]).is_err());
    }
}
