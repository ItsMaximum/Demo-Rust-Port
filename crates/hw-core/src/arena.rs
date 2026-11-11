//! Fixed-capacity object arena with a free list and end-of-frame recycling.
//!
//! AS3 objects stay readable after Box2D "destroys" them (DestroyBody does not clear m_sweep;
//! a queued PairRemoved may still read a destroyed shape's body; Van destroys then recreates a
//! shape inside one callback). To keep those reads exact within a frame, a destroyed slot is only
//! put back on the free list when `recycle_pending` is called at the end of the frame.
//! Objects retained by game fields remain readable after destruction while those roots exist.

use crate::{H, NONE};

/// Sticky marker in `Arena::allocations` (see `Arena::anomaly`).
pub const ANOMALY_BIT: u32 = 0x8000_0000;

#[derive(Clone, Copy)]
#[repr(C)]
pub struct Arena<T: Copy, const N: usize> {
    pub items: [T; N],
    /// 1 = allocated and not destroyed, 0 = free or destroyed.
    pub alive: [u32; N],
    /// Stack of free slots.
    pub free: [u32; N],
    pub free_len: u32,
    /// Slots destroyed this frame, recycled at end of frame.
    pub pending: [u32; N],
    pub pending_len: u32,
    /// Number of allocations ever made (debug aid / generation source).
    pub allocations: u32,
    /// Bit 0: explicit game root. Bit 1: destroyed object reachable through links.
    pub retained: [u32; N],
    pub retired_len: u32,
}

impl<T: Copy, const N: usize> Arena<T, N> {
    pub fn new(blank: T) -> Self {
        let mut free = [0u32; N];
        // pop from the end: allocate lowest indices first
        for i in 0..N {
            free[i] = (N - 1 - i) as u32;
        }
        Arena { items: [blank; N], alive: [0; N], free, free_len: N as u32, pending: [0; N], pending_len: 0, allocations: 0, retained:[0;N],retired_len:0 }
    }
    /// Allocate a slot initialised to `value`. Returns `NONE` if the port capacity is exhausted.
    #[inline]
    pub fn alloc(&mut self, value: T) -> H {
        if self.free_len == 0 {
            return NONE;
        }
        self.free_len -= 1;
        let i = self.free[self.free_len as usize];
        self.items[i as usize] = value;
        self.alive[i as usize] = 1;
        self.retained[i as usize] = 0;
        self.allocations = (self.allocations & ANOMALY_BIT) | (self.allocations.wrapping_add(1) & !ANOMALY_BIT);
        i
    }
    /// Mark destroyed; the slot keeps its contents until `recycle_pending`.
    #[inline]
    pub fn destroy(&mut self, h: H) {
        debug_assert!(self.alive[h as usize] == 1, "double destroy");
        // Outside Flash-verified territory (never reached by any verified string): a double
        // destroy, or more destroys in one frame than slots. Flag it (top bit of the unused
        // `allocations` counter, see `anomaly`) instead of corrupting the free list.
        if self.alive[h as usize] != 1 || self.pending_len as usize >= N {
            self.allocations |= ANOMALY_BIT;
            return;
        }
        self.alive[h as usize] = 0;
        self.pending[self.pending_len as usize] = h;
        self.pending_len += 1;
    }
    pub fn recycle_pending(&mut self) {
        for k in 0..self.pending_len as usize {
            let i = self.pending[k];
            if self.retained[i as usize]!=0 {self.retired_len+=1;continue;}
            self.free[self.free_len as usize] = i;
            self.free_len += 1;
        }
        self.pending_len = 0;
    }
    /// A game field keeps this AS3 object alive even after Box2D unlinks it.
    /// Most character records last the whole session; replaceable grip fields release their roots.
    pub fn retain(&mut self,h:H) {if h!=NONE {self.retained[h as usize]|=1;}}
    /// Drop an exclusive game-field root. Use with `recycle_pending_reachable`:
    /// an already retired slot must remain tracked until the next graph walk.
    pub fn release_root(&mut self, h: H) {
        if h == NONE { return; }
        let i = h as usize;
        if self.retained[i] & 1 == 0 { return; }
        self.retained[i] &= !1;
        if self.alive[i] == 0 {
            let mut pending = false;
            for k in 0..self.pending_len as usize {
                if self.pending[k] == h { pending = true; break; }
            }
            if !pending { self.retained[i] |= 2; }
        }
    }
    /// Recycle destroyed records only when neither a game root nor a reachable
    /// object points to them. Link-only retention is reconsidered each frame.
    pub fn recycle_pending_reachable(&mut self, reachable: &[crate::Flag32; N]) {
        for i in 0..N {
            if self.retained[i] & 2 != 0 {
                if self.retained[i] & 1 != 0 {
                    self.retained[i] = 1;
                } else if !reachable[i] {
                    self.retained[i] = 0;
                    self.free[self.free_len as usize] = i as u32;
                    self.free_len += 1;
                    self.retired_len -= 1;
                }
            }
        }
        for k in 0..self.pending_len as usize {
            let i = self.pending[k] as usize;
            if self.retained[i] != 0 || reachable[i].get() {
                if self.retained[i] == 0 { self.retained[i] = 2; }
                self.retired_len += 1;
            } else {
                self.free[self.free_len as usize] = i as u32;
                self.free_len += 1;
            }
        }
        self.pending_len = 0;
    }
    #[inline(always)]
    pub fn get(&self, h: H) -> &T {
        &self.items[h as usize]
    }
    #[inline(always)]
    pub fn get_mut(&mut self, h: H) -> &mut T {
        &mut self.items[h as usize]
    }
    #[inline(always)]
    pub fn is_alive(&self, h: H) -> bool {
        h != NONE && self.alive[h as usize] == 1
    }
    /// True once `destroy` saw a double destroy or a pending-list overflow (port-only state).
    #[inline]
    pub fn anomaly(&self) -> bool {
        self.allocations & ANOMALY_BIT != 0
    }
    pub fn live_count(&self) -> u32 {
        N as u32 - self.free_len - self.pending_len - self.retired_len
    }
}
