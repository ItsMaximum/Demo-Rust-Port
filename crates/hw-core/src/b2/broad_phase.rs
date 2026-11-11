//! b2BroadPhase + b2PairManager (+ b2Bound, b2Proxy, b2Pair, b2BufferedPair, b2BoundValues) of
//! Box2DFlashAS3 2.0.2, ported statement by statement. All state lives in
//! `ctx.world.broad_phase` (`BroadPhase<R>`); the pair callback is `crate::b2::contacts`.
//!
//! Throw sites (see docs/PORTING.md), each reproduced with the exact partial
//! state the AS3 leaves behind:
//! - `CreateProxy` with `m_freeProxy == 65535`  -> `AbortSite::ProxyPoolExhausted` (nothing mutated).
//! - `AddPair` with `m_freePair == 65535`       -> `PairPoolExhausted` (after the hash lookup).
//! - `AddBufferedPair`/`RemoveBufferedPair` with `m_pairBufferCount == 4096` -> `PairBufferAdd` /
//!   `PairBufferRemove` (after `SetBuffered`, before the buffer append and the Clear/SetRemoved).
//! - `Commit` when `Find` returns null -> `CommitFindNull` (`m_pairBufferCount` NOT reset).
use hw_real::as3::{b2_max, b2_min};
use hw_real::Real;

use super::math::Vec2;
use super::settings::{B2_MAX_PAIRS, B2_MAX_PROXIES, USHRT_MAX};
use super::state::{
    Bound, BroadPhase, Pair, Proxy, B2_INVALID, B2_NULL_PAIR, B2_NULL_PROXY, B2_TABLE_CAPACITY, B2_TABLE_MASK,
    E_PAIR_BUFFERED, E_PAIR_FINAL, E_PAIR_REMOVED, MAX_QUERY_RESULTS,
};
use crate::flow::{AbortSite, Flow};
use crate::{Ctx, H, NONE};

// ---- b2Bound ---------------------------------------------------------------------------------
impl Bound {
    /// b2Bound.IsLower
    #[inline(always)]
    pub fn is_lower(&self) -> bool {
        (self.value & 1) == 0
    }
    /// b2Bound.IsUpper
    #[inline(always)]
    pub fn is_upper(&self) -> bool {
        (self.value & 1) == 1
    }
}

/// b2Bound.Swap between two entries of one bounds array (all three fields are exchanged).
#[inline(always)]
fn swap_bounds<const N: usize>(bounds: &mut [Bound; N], i: usize, j: usize) {
    let temp = bounds[i];
    bounds[i] = bounds[j];
    bounds[j] = temp;
}

// ---- b2Proxy ---------------------------------------------------------------------------------
impl Proxy {
    /// b2Proxy.GetNext: the free-list link is stored in lowerBounds[0].
    #[inline(always)]
    pub fn get_next(&self) -> u32 {
        self.lower_bounds[0]
    }
    /// b2Proxy.SetNext: `lowerBounds[0] = next & 0xFFFF`.
    #[inline(always)]
    pub fn set_next(&mut self, next: u32) {
        self.lower_bounds[0] = next & 0xFFFF;
    }
    /// b2Proxy.IsValid
    #[inline(always)]
    pub fn is_valid(&self) -> bool {
        self.overlap_count != B2_INVALID
    }
}

// ---- b2Pair status bits ----------------------------------------------------------------------
impl Pair {
    #[inline(always)]
    pub fn set_buffered(&mut self) {
        self.status |= E_PAIR_BUFFERED;
    }
    #[inline(always)]
    pub fn clear_buffered(&mut self) {
        self.status &= !E_PAIR_BUFFERED;
    }
    #[inline(always)]
    pub fn is_buffered(&self) -> bool {
        (self.status & E_PAIR_BUFFERED) == E_PAIR_BUFFERED
    }
    #[inline(always)]
    pub fn set_removed(&mut self) {
        self.status |= E_PAIR_REMOVED;
    }
    #[inline(always)]
    pub fn clear_removed(&mut self) {
        self.status &= !E_PAIR_REMOVED;
    }
    #[inline(always)]
    pub fn is_removed(&self) -> bool {
        (self.status & E_PAIR_REMOVED) == E_PAIR_REMOVED
    }
    #[inline(always)]
    pub fn set_final(&mut self) {
        self.status |= E_PAIR_FINAL;
    }
    #[inline(always)]
    pub fn is_final(&self) -> bool {
        (self.status & E_PAIR_FINAL) == E_PAIR_FINAL
    }
}

// ---- b2AABB.IsValid --------------------------------------------------------------------------
/// b2AABB.IsValid: `dx >= 0 && dy >= 0 && lower.IsValid() && upper.IsValid()` (false on NaN).
#[inline(always)]
fn aabb_is_valid<R: Real>(lower: Vec2<R>, upper: Vec2<R>) -> bool {
    let dx = upper.x - lower.x;
    let dy = upper.y - lower.y;
    let valid = dx.ge(R::zero()) && dy.ge(R::zero());
    valid && lower.is_valid() && upper.is_valid()
}

// ---- b2BroadPhase ----------------------------------------------------------------------------

/// `new b2BroadPhase(worldAABB, callback)` (+ `new b2PairManager()` and `Initialize`): fills
/// `ctx.world.broad_phase` from a `BroadPhase::blank()`.
pub fn init<R: Real>(ctx: &mut Ctx<R>, lower: Vec2<R>, upper: Vec2<R>) {
    let bp = &mut ctx.world.broad_phase;
    // this.m_pairManager = new b2PairManager();
    pair_manager_init(bp);
    // m_proxyPool / m_bounds / m_queryResults arrays, m_quantizationFactor = new b2Vec2()
    bp.quantization_factor = Vec2::zero();
    // this.m_pairManager.Initialize(this, callback): the callback is the contacts module.
    bp.world_aabb_lower = lower;
    bp.world_aabb_upper = upper;
    bp.proxy_count = 0;
    let mut i = 0;
    while i < B2_MAX_PROXIES {
        bp.query_results[i] = 0;
        i += 1;
    }
    let mut axis = 0;
    while axis < 2 {
        let mut j = 0;
        while j < 2 * B2_MAX_PROXIES {
            // new b2Bound(): uint fields default to 0
            bp.bounds[axis][j] = Bound { value: 0, proxy_id: 0, stabbing_count: 0 };
            j += 1;
        }
        axis += 1;
    }
    let dx = upper.x - lower.x;
    let dy = upper.y - lower.y;
    // b2Settings.USHRT_MAX is an int constant (65535) promoted to Number.
    bp.quantization_factor.x = R::from_i32(USHRT_MAX as i32) / dx;
    bp.quantization_factor.y = R::from_i32(USHRT_MAX as i32) / dy;
    let mut i = 0;
    while i < B2_MAX_PROXIES - 1 {
        // new b2Proxy(): lowerBounds/upperBounds = [0,0], uint fields 0, userData null
        let proxy = &mut bp.proxy_pool[i];
        *proxy = Proxy { lower_bounds: [0, 0], upper_bounds: [0, 0], overlap_count: 0, time_stamp: 0, shape: NONE };
        proxy.set_next(i as u32 + 1);
        proxy.time_stamp = 0;
        proxy.overlap_count = B2_INVALID;
        proxy.shape = NONE;
        i += 1;
    }
    let proxy = &mut bp.proxy_pool[B2_MAX_PROXIES - 1];
    *proxy = Proxy { lower_bounds: [0, 0], upper_bounds: [0, 0], overlap_count: 0, time_stamp: 0, shape: NONE };
    proxy.set_next(B2_NULL_PROXY);
    proxy.time_stamp = 0;
    proxy.overlap_count = B2_INVALID;
    proxy.shape = NONE;
    bp.free_proxy = 0;
    bp.time_stamp = 1;
    bp.query_result_count = 0;
}

/// b2BroadPhase.BinarySearch(bounds, count, value). `count` is the AS3 `int` parameter;
/// `(lo + hi) / 2` is a Number division truncated by the `int` local.
fn binary_search<const N: usize>(bounds: &[Bound; N], count: i32, value: u32) -> u32 {
    let mut low: i32 = 0;
    let mut high: i32 = count - 1;
    while low <= high {
        let mid: i32 = (low + high) / 2;
        let bound = bounds[mid as usize];
        if bound.value > value {
            high = mid - 1;
        } else {
            if bound.value >= value {
                return mid as u32;
            }
            low = mid + 1;
        }
    }
    low as u32
}

/// b2BroadPhase.IncrementOverlapCount(proxyId). `m_queryResults` grows silently in AS3; the port
/// stops at `MAX_QUERY_RESULTS`; aborted queries can retain and append duplicate results.
fn increment_overlap_count<R: Real>(bp: &mut BroadPhase<R>, proxy_id: u32) -> Flow {
    let time_stamp = bp.time_stamp;
    let proxy = &mut bp.proxy_pool[proxy_id as usize];
    if proxy.time_stamp < time_stamp {
        proxy.time_stamp = time_stamp;
        proxy.overlap_count = 1;
    } else {
        proxy.overlap_count = 2;
        let n = bp.query_result_count as usize;
        if n < MAX_QUERY_RESULTS {
            bp.query_results[n] = proxy_id;
            bp.query_result_count += 1;
        } else {
            return Flow::Abort(AbortSite::PortCapacity);
        }
    }
    Flow::Continue
}

/// b2BroadPhase.IncrementTimeStamp (resets every proxy time stamp at 65535).
fn increment_time_stamp<R: Real>(bp: &mut BroadPhase<R>) {
    if bp.time_stamp == USHRT_MAX {
        let mut i = 0;
        while i < B2_MAX_PROXIES {
            bp.proxy_pool[i].time_stamp = 0;
            i += 1;
        }
        bp.time_stamp = 1;
    } else {
        bp.time_stamp += 1;
    }
}

/// b2BroadPhase.Query(lowerQueryOut, upperQueryOut, lowerValue, upperValue, bounds, boundCount, axis).
/// Returns (lowerQuery, upperQuery); overlapping proxies are pushed to `m_queryResults`.
fn query<R: Real>(bp: &mut BroadPhase<R>, lower_value: u32, upper_value: u32, axis: usize, bound_count: i32) -> (Flow, u32, u32) {
    let lower_query = binary_search(&bp.bounds[axis], bound_count, lower_value);
    let upper_query = binary_search(&bp.bounds[axis], bound_count, upper_value);
    let mut j = lower_query;
    while j < upper_query {
        let bound = bp.bounds[axis][j as usize];
        if bound.is_lower() {
            let f=increment_overlap_count(bp, bound.proxy_id);if f.is_abort(){return(f,0,0);}
        }
        j += 1;
    }
    if lower_query > 0 {
        let mut i: i32 = (lower_query - 1) as i32;
        let mut s: i32 = bp.bounds[axis][i as usize].stabbing_count as i32;
        // while(_loc13_): int truthiness
        while s != 0 {
            if i < 0 {
                // AS3 reads bounds[-1] (undefined) and throws. Reachable after earlier aborts
                // leave inconsistent stabbing counts; do not clear or repair the query state.
                return (Flow::Abort(AbortSite::BroadPhaseQueryNullBound), 0, 0);
            }
            let bound = bp.bounds[axis][i as usize];
            if bound.is_lower() {
                let proxy = bp.proxy_pool[bound.proxy_id as usize];
                // Proxy bound arrays contain AS3 int assignments during swaps. A previous
                // abort can leave a negative entry; this comparison promotes that signed
                // Number, unlike the explicit uint conversion in DestroyProxy.
                if (lower_query as i32) <= (proxy.upper_bounds[axis] as i32) {
                    let f=increment_overlap_count(bp, bound.proxy_id);if f.is_abort(){return(f,0,0);}
                    s -= 1;
                }
            }
            i -= 1;
        }
    }
    (Flow::Continue, lower_query, upper_query)
}

/// b2BroadPhase.ComputeBounds(lowerValues, upperValues, aabb): clamps to the world AABB, then
/// `uint(q * (v - lower))` (ECMAScript ToUint32) masked even (`& USHRT_MAX-1`) / odd (`& 0xFFFF | 1`).
fn compute_bounds<R: Real>(bp: &BroadPhase<R>, lower: Vec2<R>, upper: Vec2<R>) -> ([u32; 2], [u32; 2]) {
    let mut min_x = lower.x;
    let mut min_y = lower.y;
    min_x = b2_min(min_x, bp.world_aabb_upper.x);
    min_y = b2_min(min_y, bp.world_aabb_upper.y);
    min_x = b2_max(min_x, bp.world_aabb_lower.x);
    min_y = b2_max(min_y, bp.world_aabb_lower.y);
    let mut max_x = upper.x;
    let mut max_y = upper.y;
    max_x = b2_min(max_x, bp.world_aabb_upper.x);
    max_y = b2_min(max_y, bp.world_aabb_upper.y);
    max_x = b2_max(max_x, bp.world_aabb_lower.x);
    max_y = b2_max(max_y, bp.world_aabb_lower.y);
    let mut lower_values = [0u32; 2];
    let mut upper_values = [0u32; 2];
    lower_values[0] = (bp.quantization_factor.x * (min_x - bp.world_aabb_lower.x)).to_uint32() & (USHRT_MAX - 1);
    upper_values[0] = (bp.quantization_factor.x * (max_x - bp.world_aabb_lower.x)).to_uint32() & 0xFFFF | 1;
    lower_values[1] = (bp.quantization_factor.y * (min_y - bp.world_aabb_lower.y)).to_uint32() & (USHRT_MAX - 1);
    upper_values[1] = (bp.quantization_factor.y * (max_y - bp.world_aabb_lower.y)).to_uint32() & 0xFFFF | 1;
    (lower_values, upper_values)
}

/// b2BroadPhase.TestOverlap(b2BoundValues, b2Proxy)
fn test_overlap<R: Real>(bp: &BroadPhase<R>, lower_values: &[u32; 2], upper_values: &[u32; 2], proxy_id: u32) -> (Flow, bool) {
    let proxy = bp.proxy_pool[proxy_id as usize];
    let mut axis = 0;
    while axis < 2 {
        let bounds = &bp.bounds[axis];
        if proxy.upper_bounds[axis] as usize >= bounds.len() {return (Flow::Abort(AbortSite::BroadPhaseQueryNullBound),false);}
        let bound = bounds[proxy.upper_bounds[axis] as usize];
        if lower_values[axis] > bound.value {
            return (Flow::Continue,false);
        }
        if proxy.lower_bounds[axis] as usize >= bounds.len() {return (Flow::Abort(AbortSite::BroadPhaseQueryNullBound),false);}
        let bound = bounds[proxy.lower_bounds[axis] as usize];
        if upper_values[axis] < bound.value {
            return (Flow::Continue,false);
        }
        axis += 1;
    }
    (Flow::Continue,true)
}

/// b2BroadPhase.TestOverlapValidate(p1, p2) — private and unreferenced in the AS3 (kept for parity).
#[allow(dead_code)]
fn test_overlap_validate<R: Real>(bp: &BroadPhase<R>, p1: &Proxy, p2: &Proxy) -> bool {
    let mut axis = 0;
    while axis < 2 {
        let bounds = &bp.bounds[axis];
        let bound1 = bounds[p1.lower_bounds[axis] as usize];
        let bound2 = bounds[p2.upper_bounds[axis] as usize];
        if bound1.value > bound2.value {
            return false;
        }
        let bound1 = bounds[p1.upper_bounds[axis] as usize];
        let bound2 = bounds[p2.lower_bounds[axis] as usize];
        if bound1.value < bound2.value {
            return false;
        }
        axis += 1;
    }
    true
}

/// b2BroadPhase.InRange(aabb)
pub fn in_range<R: Real>(ctx: &Ctx<R>, lower: Vec2<R>, upper: Vec2<R>) -> bool {
    let bp = &ctx.world.broad_phase;
    let mut d1x = lower.x;
    let mut d1y = lower.y;
    d1x = d1x - bp.world_aabb_upper.x;
    d1y = d1y - bp.world_aabb_upper.y;
    let mut d2x = bp.world_aabb_lower.x;
    let mut d2y = bp.world_aabb_lower.y;
    d2x = d2x - upper.x;
    d2y = d2y - upper.y;
    d1x = b2_max(d1x, d2x);
    d1y = b2_max(d1y, d2y);
    b2_max(d1x, d1y).lt(R::zero())
}

/// b2BroadPhase.GetProxy(proxyId): `None` for b2_nullProxy or an invalid (free) proxy.
/// (Unreferenced by the AS3 game and Box2D code; kept for API completeness.)
pub fn get_proxy<R: Real>(ctx: &Ctx<R>, proxy_id: u32) -> Option<Proxy> {
    if proxy_id == B2_NULL_PROXY || proxy_id as usize >= B2_MAX_PROXIES {
        return None;
    }
    let proxy = ctx.world.broad_phase.proxy_pool[proxy_id as usize];
    if proxy.is_valid() == false {
        return None;
    }
    Some(proxy)
}

/// b2BroadPhase.CreateProxy(aabb, userData) -> proxyId. On an abort the returned id must not be
/// stored (the AS3 assignment `m_proxyId = CreateProxy(...)` never happens); it is `B2_NULL_PROXY`
/// for `ProxyPoolExhausted` (nothing allocated) and the allocated id for later aborts.
pub fn create_proxy<R: Real>(ctx: &mut Ctx<R>, lower: Vec2<R>, upper: Vec2<R>, shape: H) -> (Flow, u32) {
    let bp = &mut ctx.world.broad_phase;
    let proxy_id = bp.free_proxy;
    #[cfg(feature = "lab")]
    let _lab_proxy_operation = crate::lab::proxy_operation(crate::lab::ProxyOperationKind::CreateProxy, Some(proxy_id));
    if proxy_id as usize >= B2_MAX_PROXIES {
        // m_proxyPool[65535] is undefined -> null.GetNext() throws; nothing was mutated.
        return (Flow::Abort(AbortSite::ProxyPoolExhausted), B2_NULL_PROXY);
    }
    bp.free_proxy = bp.proxy_pool[proxy_id as usize].get_next();
    bp.proxy_pool[proxy_id as usize].overlap_count = 0;
    bp.proxy_pool[proxy_id as usize].shape = shape;
    let bound_count = (2 * bp.proxy_count) as u32;
    let (lower_values, upper_values) = compute_bounds(bp, lower, upper);
    let mut axis = 0;
    while axis < 2 {
        let (f, lower_index, upper_index) = query(bp, lower_values[axis], upper_values[axis], axis, bound_count as i32);
        if f.is_abort() { return (f, proxy_id); }
        let bounds = &mut bp.bounds[axis];
        let lo = lower_index as usize;
        let up = upper_index as usize;
        let bc = bound_count as usize;
        // Copy [upperIndex, boundCount) to [upperIndex + 2, ...) (via a temporary array in AS3).
        let mut i = bc;
        while i > up { i -= 1; bounds[i + 2] = bounds[i]; }
        // Copy [lowerIndex, upperIndex) to [lowerIndex + 1, ...).
        let mut i = up;
        while i > lo { i -= 1; bounds[i + 1] = bounds[i]; }
        // _loc13_++
        let up = up + 1;
        bounds[lo].value = lower_values[axis];
        bounds[lo].proxy_id = proxy_id;
        bounds[up].value = upper_values[axis];
        bounds[up].proxy_id = proxy_id;
        // bounds[lowerIndex - 1] is undefined when lowerIndex == 0; the ternary avoids reading it.
        bounds[lo].stabbing_count = if lo == 0 { 0 } else { bounds[lo - 1].stabbing_count };
        bounds[up].stabbing_count = bounds[up - 1].stabbing_count;
        let mut index = lo;
        while index < up {
            bounds[index].stabbing_count = bounds[index].stabbing_count.wrapping_add(1);
            index += 1;
        }
        let mut index = lo;
        while index < bc + 2 {
            let bound = bounds[index];
            let proxy = &mut bp.proxy_pool[bound.proxy_id as usize];
            if bound.is_lower() {
                proxy.lower_bounds[axis] = index as u32;
            } else {
                proxy.upper_bounds[axis] = index as u32;
            }
            index += 1;
        }
        axis += 1;
    }
    bp.proxy_count += 1;
    let mut i: i32 = 0;
    while i < ctx.world.broad_phase.query_result_count {
        let other = ctx.world.broad_phase.query_results[i as usize];
        match add_buffered_pair(&mut ctx.world.broad_phase, proxy_id, other) {
            Flow::Continue => {}
            f => return (f, proxy_id),
        }
        i += 1;
    }
    match pair_manager_commit(ctx) {
        Flow::Continue => {}
        f => return (f, proxy_id),
    }
    let bp = &mut ctx.world.broad_phase;
    bp.query_result_count = 0;
    increment_time_stamp(bp);
    (Flow::Continue, proxy_id)
}

/// b2BroadPhase.DestroyProxy(proxyId)
pub fn destroy_proxy<R: Real>(ctx: &mut Ctx<R>, proxy_id: u32) -> Flow {
    let bp = &mut ctx.world.broad_phase;
    let bound_count: i32 = 2 * bp.proxy_count;
    let mut axis = 0;
    while axis < 2 {
        let proxy = bp.proxy_pool[proxy_id as usize];
        let lower_index = proxy.lower_bounds[axis];
        let upper_index = proxy.upper_bounds[axis];
        let bounds = &mut bp.bounds[axis];
        if lower_index as usize>=bounds.len(){return Flow::Abort(AbortSite::BroadPhaseQueryNullBound);}
        let lower_value = bounds[lower_index as usize].value;
        if upper_index as usize>=bounds.len(){return Flow::Abort(AbortSite::BroadPhaseQueryNullBound);}
        let upper_value = bounds[upper_index as usize].value;
        let li = lower_index as usize;
        let ui = upper_index as usize;
        let bc = bound_count as usize;
        // Copy (lowerIndex, upperIndex) down by one (temporary array in AS3).
        for i in li + 1..ui { bounds[i - 1] = bounds[i]; }
        // Copy (upperIndex, boundCount) down by two.
        for i in ui + 1..bc { bounds[i - 2] = bounds[i]; }
        // Fix bound indices of the shifted proxies: [lowerIndex, boundCount - 2).
        let mut index = li;
        while index < bc - 2 {
            let bound = bounds[index];
            let other = &mut bp.proxy_pool[bound.proxy_id as usize];
            if bound.is_lower() {
                other.lower_bounds[axis] = index as u32;
            } else {
                other.upper_bounds[axis] = index as u32;
            }
            index += 1;
        }
        // Fix stabbing counts: [lowerIndex, upperIndex - 1).
        let mut index = li;
        while index < ui - 1 {
            bounds[index].stabbing_count = bounds[index].stabbing_count.wrapping_sub(1);
            index += 1;
        }
        // Query for pairs to remove (the output indices are discarded).
        crate::flow!(query(bp, lower_value, upper_value, axis, bound_count - 2).0);
        axis += 1;
    }
    let mut i: i32 = 0;
    while i < ctx.world.broad_phase.query_result_count {
        let other = ctx.world.broad_phase.query_results[i as usize];
        crate::flow!(remove_buffered_pair(&mut ctx.world.broad_phase, proxy_id, other));
        i += 1;
    }
    crate::flow!(pair_manager_commit(ctx));
    let bp = &mut ctx.world.broad_phase;
    bp.query_result_count = 0;
    increment_time_stamp(bp);
    let free_proxy = bp.free_proxy;
    let proxy = &mut bp.proxy_pool[proxy_id as usize];
    proxy.shape = NONE;
    proxy.overlap_count = B2_INVALID;
    proxy.lower_bounds[0] = B2_INVALID;
    proxy.lower_bounds[1] = B2_INVALID;
    proxy.upper_bounds[0] = B2_INVALID;
    proxy.upper_bounds[1] = B2_INVALID;
    proxy.set_next(free_proxy);
    bp.free_proxy = proxy_id;
    bp.proxy_count -= 1;
    Flow::Continue
}

/// b2BroadPhase.MoveProxy(proxyId, aabb). Silently returns for an invalid id or a non-finite /
/// inverted AABB. The four insertion-sort passes mutate bounds, stabbing counts, proxy indices and
/// the pair buffer in AS3 order, so an abort from AddBufferedPair/RemoveBufferedPair leaves the
/// half-sorted arrays behind exactly as the Flash Player does.
pub fn move_proxy<R: Real>(ctx: &mut Ctx<R>, proxy_id: u32, lower: Vec2<R>, upper: Vec2<R>) -> Flow {
    #[cfg(feature = "lab")]
    let _lab_proxy_operation = crate::lab::proxy_operation(crate::lab::ProxyOperationKind::MoveProxy, Some(proxy_id));
    if proxy_id == B2_NULL_PROXY || (B2_MAX_PROXIES as u32) <= proxy_id {
        return Flow::Continue;
    }
    if aabb_is_valid(lower, upper) == false {
        return Flow::Continue;
    }
    let bp = &mut ctx.world.broad_phase;
    let bound_count = (2 * bp.proxy_count) as u32;
    let pid = proxy_id as usize;
    // newValues (b2BoundValues)
    let (new_lower, new_upper) = compute_bounds(bp, lower, upper);
    // oldValues
    let mut old_lower = [0u32; 2];
    let mut old_upper = [0u32; 2];
    let mut axis = 0;
    while axis < 2 {
        let proxy = bp.proxy_pool[pid];
        if proxy.lower_bounds[axis] as usize>=bp.bounds[axis].len(){return Flow::Abort(AbortSite::BroadPhaseQueryNullBound);}
        old_lower[axis] = bp.bounds[axis][proxy.lower_bounds[axis] as usize].value;
        if proxy.upper_bounds[axis] as usize>=bp.bounds[axis].len(){return Flow::Abort(AbortSite::BroadPhaseQueryNullBound);}
        old_upper[axis] = bp.bounds[axis][proxy.upper_bounds[axis] as usize].value;
        axis += 1;
    }
    let mut axis = 0;
    while axis < 2 {
        let lower_index = bp.proxy_pool[pid].lower_bounds[axis];
        let upper_index = bp.proxy_pool[pid].upper_bounds[axis];
        let lower_value = new_lower[axis];
        let upper_value = new_upper[axis];
        // int deltaLower = lowerValue - bounds[lowerIndex].value (values < 2^16, so the wrapped
        // u32 difference reinterpreted as i32 is the exact Number difference).
        let delta_lower = lower_value.wrapping_sub(bp.bounds[axis][lower_index as usize].value) as i32;
        bp.bounds[axis][lower_index as usize].value = lower_value;
        let delta_upper = upper_value.wrapping_sub(bp.bounds[axis][upper_index as usize].value) as i32;
        bp.bounds[axis][upper_index as usize].value = upper_value;
        // Expanding adds overlaps due to the lower bound moving down.
        if delta_lower < 0 {
            let mut index = lower_index;
            while index > 0 && lower_value < bp.bounds[axis][(index - 1) as usize].value {
                let i = index as usize;
                let prev_proxy_id = bp.bounds[axis][i - 1].proxy_id;
                bp.bounds[axis][i - 1].stabbing_count = bp.bounds[axis][i - 1].stabbing_count.wrapping_add(1);
                if bp.bounds[axis][i - 1].is_upper() == true {
                    let(f,overlap)=test_overlap(bp, &new_lower, &new_upper, prev_proxy_id);crate::flow!(f);if overlap {
                        crate::flow!(add_buffered_pair(bp, proxy_id, prev_proxy_id));
                    }
                    let prev_proxy = &mut bp.proxy_pool[prev_proxy_id as usize];
                    prev_proxy.upper_bounds[axis] = prev_proxy.upper_bounds[axis].wrapping_add(1);
                    bp.bounds[axis][i].stabbing_count = bp.bounds[axis][i].stabbing_count.wrapping_add(1);
                } else {
                    let prev_proxy = &mut bp.proxy_pool[prev_proxy_id as usize];
                    prev_proxy.lower_bounds[axis] = prev_proxy.lower_bounds[axis].wrapping_add(1);
                    bp.bounds[axis][i].stabbing_count = bp.bounds[axis][i].stabbing_count.wrapping_sub(1);
                }
                bp.proxy_pool[pid].lower_bounds[axis] = bp.proxy_pool[pid].lower_bounds[axis].wrapping_sub(1);
                swap_bounds(&mut bp.bounds[axis], i, i - 1);
                index -= 1;
            }
        }
        // Expanding adds overlaps due to the upper bound moving up.
        if delta_upper > 0 {
            let mut index = upper_index;
            while index < bound_count.wrapping_sub(1) && bp.bounds[axis][(index + 1) as usize].value <= upper_value {
                let i = index as usize;
                let next_proxy_id = bp.bounds[axis][i + 1].proxy_id;
                bp.bounds[axis][i + 1].stabbing_count = bp.bounds[axis][i + 1].stabbing_count.wrapping_add(1);
                if bp.bounds[axis][i + 1].is_lower() == true {
                    let(f,overlap)=test_overlap(bp, &new_lower, &new_upper, next_proxy_id);crate::flow!(f);if overlap {
                        crate::flow!(add_buffered_pair(bp, proxy_id, next_proxy_id));
                    }
                    let next_proxy = &mut bp.proxy_pool[next_proxy_id as usize];
                    next_proxy.lower_bounds[axis] = next_proxy.lower_bounds[axis].wrapping_sub(1);
                    bp.bounds[axis][i].stabbing_count = bp.bounds[axis][i].stabbing_count.wrapping_add(1);
                } else {
                    let next_proxy = &mut bp.proxy_pool[next_proxy_id as usize];
                    next_proxy.upper_bounds[axis] = next_proxy.upper_bounds[axis].wrapping_sub(1);
                    bp.bounds[axis][i].stabbing_count = bp.bounds[axis][i].stabbing_count.wrapping_sub(1);
                }
                bp.proxy_pool[pid].upper_bounds[axis] = bp.proxy_pool[pid].upper_bounds[axis].wrapping_add(1);
                swap_bounds(&mut bp.bounds[axis], i, i + 1);
                index += 1;
            }
        }
        // Shrinking removes overlaps due to the lower bound moving up.
        if delta_lower > 0 {
            let mut index = lower_index;
            while index < bound_count.wrapping_sub(1) && bp.bounds[axis][(index + 1) as usize].value <= lower_value {
                let i = index as usize;
                let next_proxy_id = bp.bounds[axis][i + 1].proxy_id;
                bp.bounds[axis][i + 1].stabbing_count = bp.bounds[axis][i + 1].stabbing_count.wrapping_sub(1);
                if bp.bounds[axis][i + 1].is_upper() {
                    let(f,overlap)=test_overlap(bp, &old_lower, &old_upper, next_proxy_id);crate::flow!(f);if overlap {
                        crate::flow!(remove_buffered_pair(bp, proxy_id, next_proxy_id));
                    }
                    let next_proxy = &mut bp.proxy_pool[next_proxy_id as usize];
                    next_proxy.upper_bounds[axis] = next_proxy.upper_bounds[axis].wrapping_sub(1);
                    bp.bounds[axis][i].stabbing_count = bp.bounds[axis][i].stabbing_count.wrapping_sub(1);
                } else {
                    let next_proxy = &mut bp.proxy_pool[next_proxy_id as usize];
                    next_proxy.lower_bounds[axis] = next_proxy.lower_bounds[axis].wrapping_sub(1);
                    bp.bounds[axis][i].stabbing_count = bp.bounds[axis][i].stabbing_count.wrapping_add(1);
                }
                bp.proxy_pool[pid].lower_bounds[axis] = bp.proxy_pool[pid].lower_bounds[axis].wrapping_add(1);
                swap_bounds(&mut bp.bounds[axis], i, i + 1);
                index += 1;
            }
        }
        // Shrinking removes overlaps due to the upper bound moving down.
        if delta_upper < 0 {
            let mut index = upper_index;
            while index > 0 && upper_value < bp.bounds[axis][(index - 1) as usize].value {
                let i = index as usize;
                let prev_proxy_id = bp.bounds[axis][i - 1].proxy_id;
                bp.bounds[axis][i - 1].stabbing_count = bp.bounds[axis][i - 1].stabbing_count.wrapping_sub(1);
                if bp.bounds[axis][i - 1].is_lower() == true {
                    let(f,overlap)=test_overlap(bp, &old_lower, &old_upper, prev_proxy_id);crate::flow!(f);if overlap {
                        crate::flow!(remove_buffered_pair(bp, proxy_id, prev_proxy_id));
                    }
                    let prev_proxy = &mut bp.proxy_pool[prev_proxy_id as usize];
                    prev_proxy.lower_bounds[axis] = prev_proxy.lower_bounds[axis].wrapping_add(1);
                    bp.bounds[axis][i].stabbing_count = bp.bounds[axis][i].stabbing_count.wrapping_sub(1);
                } else {
                    let prev_proxy = &mut bp.proxy_pool[prev_proxy_id as usize];
                    prev_proxy.upper_bounds[axis] = prev_proxy.upper_bounds[axis].wrapping_add(1);
                    bp.bounds[axis][i].stabbing_count = bp.bounds[axis][i].stabbing_count.wrapping_add(1);
                }
                bp.proxy_pool[pid].upper_bounds[axis] = bp.proxy_pool[pid].upper_bounds[axis].wrapping_sub(1);
                swap_bounds(&mut bp.bounds[axis], i, i - 1);
                index -= 1;
            }
        }
        axis += 1;
    }
    Flow::Continue
}

/// b2BroadPhase.Commit
pub fn commit<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    pair_manager_commit(ctx)
}

/// Statically dispatched output storage; fixed arrays are not coerced to slices.
/// Both implementations preserve partial writes on abort.
pub trait QueryOutput {
    fn capacity(&self) -> usize;
    fn set(&mut self, index: usize, value: H);
}
impl<const N: usize> QueryOutput for [H; N] {
    #[inline] fn capacity(&self) -> usize { N }
    #[inline] fn set(&mut self, index: usize, value: H) { self[index] = value; }
}
impl QueryOutput for [H] {
    #[inline] fn capacity(&self) -> usize { self.len() }
    #[inline] fn set(&mut self, index: usize, value: H) { self[index] = value; }
}

/// b2BroadPhase.QueryAABB(aabb, userDataOut, maxCount) -> count. `out` receives shape handles
/// (proxy userData) in broad-phase order; writes stop at `out.len()` (the AS3 array grows).
pub fn query_aabb<R: Real>(ctx: &mut Ctx<R>, lower: Vec2<R>, upper: Vec2<R>, out: &mut (impl QueryOutput + ?Sized), max_count: i32) -> (Flow, i32) {
    let bp = &mut ctx.world.broad_phase;
    let (lower_values, upper_values) = compute_bounds(bp, lower, upper);
    let bound_count = 2 * bp.proxy_count;
    for axis in 0..2 {let f=query(bp, lower_values[axis], upper_values[axis], axis, bound_count).0;if f.is_abort(){return(f,0);}}
    let mut count: i32 = 0;
    let mut i: i32 = 0;
    while i < bp.query_result_count && count < max_count {
        let proxy = bp.proxy_pool[bp.query_results[i as usize] as usize];
        if (i as usize) < out.capacity() {
            out.set(i as usize, proxy.shape);
        } else {
            return (Flow::Abort(AbortSite::PortCapacity),count);
        }
        i += 1;
        count += 1;
    }
    bp.query_result_count = 0;
    increment_time_stamp(bp);
    (Flow::Continue, count)
}

/// b2BroadPhase.Validate — the AS3 body only counts bounds; no effect.
pub fn validate<R: Real>(_ctx: &Ctx<R>) {}

// ---- b2PairManager ---------------------------------------------------------------------------

/// `new b2PairManager()`: hash table of b2_nullPair, pair free list chained through `next`,
/// `m_freePair` left at its uint default (0), which happens to be the first free pair.
fn pair_manager_init<R: Real>(bp: &mut BroadPhase<R>) {
    let mut i = 0;
    while i < B2_TABLE_CAPACITY {
        bp.hash_table[i] = B2_NULL_PAIR;
        i += 1;
    }
    let mut i = 0;
    while i < B2_MAX_PAIRS {
        // new b2Pair(): uint fields 0, userData null
        bp.pairs[i] = Pair { contact: NONE, proxy_id1: 0, proxy_id2: 0, status: 0, next: 0 };
        i += 1;
    }
    let mut i = 0;
    while i < B2_MAX_PAIRS {
        // new b2BufferedPair()
        bp.pair_buffer[i].proxy_id1 = 0;
        bp.pair_buffer[i].proxy_id2 = 0;
        i += 1;
    }
    let mut i = 0;
    while i < B2_MAX_PAIRS {
        bp.pairs[i].proxy_id1 = B2_NULL_PROXY;
        bp.pairs[i].proxy_id2 = B2_NULL_PROXY;
        bp.pairs[i].contact = NONE;
        bp.pairs[i].status = 0;
        bp.pairs[i].next = i as u32 + 1;
        i += 1;
    }
    bp.pairs[B2_MAX_PAIRS - 1].next = B2_NULL_PAIR;
    bp.pair_count = 0;
    bp.pair_buffer_count = 0;
    // m_freePair is never assigned by the AS3 constructor: uint default 0.
    bp.free_pair = 0;
}

/// b2PairManager.Hash(proxyId1, proxyId2): Thomas Wang's 32-bit mix. In AS3 `>>` is an arithmetic
/// shift on ToInt32 but every shifted term is masked to its logical width, and `*= 2057` on a
/// uint is exact in a double before ToUint32, so the whole thing is plain wrapping u32 math.
pub fn hash(proxy_id1: u32, proxy_id2: u32) -> u32 {
    let mut key: u32 = ((proxy_id2 << 16) & 4294901760u32) | proxy_id1;
    key = (!key).wrapping_add((key << 15) & 4294934528u32);
    key ^= (key >> 12) & 0x0FFFFF;
    key = key.wrapping_add((key << 2) & 4294967292u32);
    key ^= (key >> 4) & 0x0FFFFFFF;
    key = key.wrapping_mul(2057);
    key ^ ((key >> 16) & 0xFFFF)
}

/// b2PairManager.Equals(pair, proxyId1, proxyId2)
#[inline(always)]
fn equals(pair: &Pair, proxy_id1: u32, proxy_id2: u32) -> bool {
    pair.proxy_id1 == proxy_id1 && pair.proxy_id2 == proxy_id2
}

/// b2PairManager.FindHash(proxyId1, proxyId2, hash) -> pair index or `B2_NULL_PAIR` (null).
fn find_hash<R: Real>(bp: &BroadPhase<R>, proxy_id1: u32, proxy_id2: u32, hash: u32) -> u32 {
    let mut index = bp.hash_table[hash as usize];
    while index != B2_NULL_PAIR && equals(&bp.pairs[index as usize], proxy_id1, proxy_id2) == false {
        index = bp.pairs[index as usize].next;
    }
    if index == B2_NULL_PAIR {
        return B2_NULL_PAIR;
    }
    index
}

/// b2PairManager.Find(proxyId1, proxyId2) -> pair index or `B2_NULL_PAIR`.
fn find<R: Real>(bp: &BroadPhase<R>, mut proxy_id1: u32, mut proxy_id2: u32) -> u32 {
    if proxy_id1 > proxy_id2 {
        let tmp = proxy_id1;
        proxy_id1 = proxy_id2;
        proxy_id2 = tmp;
    }
    let hash = hash(proxy_id1, proxy_id2) & B2_TABLE_MASK;
    find_hash(bp, proxy_id1, proxy_id2, hash)
}

/// b2PairManager.AddPair(proxyId1, proxyId2) -> pair index. Throws `PairPoolExhausted` when the
/// free list is empty (`m_pairs[65535]` undefined -> `null.next`), after the lookup.
fn add_pair<R: Real>(bp: &mut BroadPhase<R>, mut proxy_id1: u32, mut proxy_id2: u32) -> (Flow, u32) {
    if proxy_id1 > proxy_id2 {
        let tmp = proxy_id1;
        proxy_id1 = proxy_id2;
        proxy_id2 = tmp;
    }
    let hash = hash(proxy_id1, proxy_id2) & B2_TABLE_MASK;
    let found = find_hash(bp, proxy_id1, proxy_id2, hash);
    if found != B2_NULL_PAIR {
        return (Flow::Continue, found);
    }
    let pair_index = bp.free_pair;
    if pair_index as usize >= B2_MAX_PAIRS {
        lab! {
            if crate::lab::armed() {
                let proxy_slots = [crate::lab::pair_proxy_slot(&bp.proxy_pool, proxy_id1),
                    crate::lab::pair_proxy_slot(&bp.proxy_pool, proxy_id2)];
                crate::lab::record_pair_exhaustion([proxy_id1, proxy_id2], hash, pair_index,
                    bp.pair_count, bp.pair_buffer_count, proxy_slots);
            }
        }
        return (Flow::Abort(AbortSite::PairPoolExhausted), B2_NULL_PAIR);
    }
    let next_free = bp.pairs[pair_index as usize].next;
    bp.free_pair = next_free;
    let head = bp.hash_table[hash as usize];
    let pair = &mut bp.pairs[pair_index as usize];
    pair.proxy_id1 = proxy_id1;
    pair.proxy_id2 = proxy_id2;
    pair.status = 0;
    pair.contact = NONE;
    pair.next = head;
    bp.hash_table[hash as usize] = pair_index;
    bp.pair_count += 1;
    (Flow::Continue, pair_index)
}

/// b2PairManager.RemovePair(proxyId1, proxyId2) -> the removed pair's userData (`NONE` = null,
/// also when the pair was not found).
fn remove_pair<R: Real>(bp: &mut BroadPhase<R>, mut proxy_id1: u32, mut proxy_id2: u32) -> H {
    if proxy_id1 > proxy_id2 {
        let tmp = proxy_id1;
        proxy_id1 = proxy_id2;
        proxy_id2 = tmp;
    }
    let hash = hash(proxy_id1, proxy_id2) & B2_TABLE_MASK;
    let mut index = bp.hash_table[hash as usize];
    let mut prev_index = B2_NULL_PAIR; // `_loc6_` (null pair reference)
    while index != B2_NULL_PAIR {
        if equals(&bp.pairs[index as usize], proxy_id1, proxy_id2) {
            let pair_index = index;
            let next = bp.pairs[pair_index as usize].next;
            if prev_index != B2_NULL_PAIR {
                bp.pairs[prev_index as usize].next = next;
            } else {
                bp.hash_table[hash as usize] = next;
            }
            let free_pair = bp.free_pair;
            let pair = &mut bp.pairs[pair_index as usize];
            let user_data = pair.contact;
            pair.next = free_pair;
            pair.proxy_id1 = B2_NULL_PROXY;
            pair.proxy_id2 = B2_NULL_PROXY;
            pair.contact = NONE;
            pair.status = 0;
            bp.free_pair = pair_index;
            bp.pair_count -= 1;
            return user_data;
        }
        prev_index = index;
        index = bp.pairs[index as usize].next;
    }
    NONE
}

/// b2PairManager.AddBufferedPair(proxyId1, proxyId2). On a full buffer the AS3 has already
/// called `SetBuffered()` and then throws on `m_pairBuffer[4096].proxyId1 = ...` (`PairBufferAdd`);
/// `ClearRemoved()` is not reached.
fn add_buffered_pair<R: Real>(bp: &mut BroadPhase<R>, proxy_id1: u32, proxy_id2: u32) -> Flow {
    let (f, pair_index) = add_pair(bp, proxy_id1, proxy_id2);
    crate::flow!(f);
    let pi = pair_index as usize;
    if bp.pairs[pi].is_buffered() == false {
        bp.pairs[pi].set_buffered();
        let n = bp.pair_buffer_count as usize;
        if n >= B2_MAX_PAIRS {
            return Flow::Abort(AbortSite::PairBufferAdd);
        }
        bp.pair_buffer[n].proxy_id1 = bp.pairs[pi].proxy_id1;
        bp.pair_buffer[n].proxy_id2 = bp.pairs[pi].proxy_id2;
        bp.pair_buffer_count += 1;
    }
    bp.pairs[pi].clear_removed();
    // if (s_validate) ValidateBuffer(); -- s_validate is false, ValidateBuffer is empty
    Flow::Continue
}

/// b2PairManager.RemoveBufferedPair(proxyId1, proxyId2). Same full-buffer behaviour as
/// AddBufferedPair (`PairBufferRemove`; `SetRemoved()` not reached).
fn remove_buffered_pair<R: Real>(bp: &mut BroadPhase<R>, proxy_id1: u32, proxy_id2: u32) -> Flow {
    let pair_index = find(bp, proxy_id1, proxy_id2);
    if pair_index == B2_NULL_PAIR {
        return Flow::Continue;
    }
    let pi = pair_index as usize;
    if bp.pairs[pi].is_buffered() == false {
        bp.pairs[pi].set_buffered();
        let n = bp.pair_buffer_count as usize;
        if n >= B2_MAX_PAIRS {
            return Flow::Abort(AbortSite::PairBufferRemove);
        }
        bp.pair_buffer[n].proxy_id1 = bp.pairs[pi].proxy_id1;
        bp.pair_buffer[n].proxy_id2 = bp.pairs[pi].proxy_id2;
        bp.pair_buffer_count += 1;
    }
    bp.pairs[pi].set_removed();
    Flow::Continue
}

/// b2PairManager.Commit. Walks the pair buffer (re-reading `m_pairBufferCount` every iteration,
/// as the AS3 loop does), fires `contacts::pair_removed` / `contacts::pair_added` exactly where
/// `m_callback` is invoked, then runs the RemovePair sweep and resets the buffer count. If `Find`
/// returns null the AS3 throws at `null.ClearBuffered()`: earlier entries stay processed and the
/// buffer count is left as is (`CommitFindNull`).
fn pair_manager_commit<R: Real>(ctx: &mut Ctx<R>) -> Flow {
    let mut remove_count: i32 = 0;
    let mut i: i32 = 0;
    while i < ctx.world.broad_phase.pair_buffer_count {
        let bp = &mut ctx.world.broad_phase;
        let buffered = bp.pair_buffer[i as usize];
        let pair_index = find(bp, buffered.proxy_id1, buffered.proxy_id2);
        if pair_index == B2_NULL_PAIR {
            return Flow::Abort(AbortSite::CommitFindNull);
        }
        let pi = pair_index as usize;
        bp.pairs[pi].clear_buffered();
        let pair = bp.pairs[pi];
        let shape1 = proxy_user_data(bp, pair.proxy_id1);
        let shape2 = proxy_user_data(bp, pair.proxy_id2);
        if pair.is_removed() {
            if pair.is_final() == true {
                crate::flow!(crate::b2::contacts::pair_removed(ctx, shape1, shape2, pair.contact));
            }
            // `_loc5_` is a reference to the pair object: re-read it after the callback.
            let bp = &mut ctx.world.broad_phase;
            let pair = bp.pairs[pi];
            bp.pair_buffer[remove_count as usize].proxyid_set(pair.proxy_id1, pair.proxy_id2);
            remove_count += 1;
        } else if pair.is_final() == false {
            let (f, user_data) = crate::b2::contacts::pair_added(ctx, shape1, shape2);
            crate::flow!(f);
            let bp = &mut ctx.world.broad_phase;
            bp.pairs[pi].contact = user_data;
            bp.pairs[pi].set_final();
        }
        i += 1;
    }
    let bp = &mut ctx.world.broad_phase;
    let mut i: i32 = 0;
    while i < remove_count {
        let buffered = bp.pair_buffer[i as usize];
        let _ = remove_pair(bp, buffered.proxy_id1, buffered.proxy_id2);
        i += 1;
    }
    bp.pair_buffer_count = 0;
    // if (s_validate) ValidateTable(); -- never
    Flow::Continue
}

/// `m_proxyPool[proxyId].userData`. A pair only ever references live proxy ids (< 512); the
/// guard keeps the port panic-free if the SAP state was corrupted by an earlier abort.
#[inline(always)]
fn proxy_user_data<R: Real>(bp: &BroadPhase<R>, proxy_id: u32) -> H {
    if (proxy_id as usize) < B2_MAX_PROXIES {
        bp.proxy_pool[proxy_id as usize].shape
    } else {
        debug_assert!(false, "pair references an out-of-range proxy id");
        NONE
    }
}

impl super::state::BufferedPair {
    #[inline(always)]
    fn proxyid_set(&mut self, proxy_id1: u32, proxy_id2: u32) {
        self.proxy_id1 = proxy_id1;
        self.proxy_id2 = proxy_id2;
    }
}

// ---- tests -----------------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    use super::*;
    use crate::b2::state::WorldState;

    /// Values transcribed from the AS3 Hash with its `& mask` terms and int-shift semantics
    /// (independent reference program in the porting notes); (0, 0) also checked by hand:
    /// 0xFFFFFFFF -> 0xFFF00000 -> 0xFFB00000 -> 0xF04B0000 -> 0xCAA30000 -> 0xCAA3CAA3.
    #[test]
    fn hash_matches_as3() {
        assert_eq!(hash(0, 0), 3399731875);
        assert_eq!(hash(0, 1), 1068090708);
        assert_eq!(hash(1, 2), 3665319474);
        assert_eq!(hash(3, 511), 3965878313);
        assert_eq!(hash(100, 200), 457076435);
        assert_eq!(hash(511, 510), 2133804443);
        assert_eq!(hash(65535, 65535), 3176528920);
        assert_eq!(hash(3, 511) & B2_TABLE_MASK, 41);
        assert_eq!(hash(100, 200) & B2_TABLE_MASK, 3795);
    }

    fn with_ctx(f: impl FnOnce(&mut Ctx<f64>) + Send + 'static) {
        // The world state is ~1 MB: run on a thread with a generous stack.
        std::thread::Builder::new()
            .stack_size(64 << 20)
            .spawn(move || {
                let mut ctx = Box::new(Ctx { world: WorldState::blank(), scratch: crate::b2::island::StepScratch::blank(), listener_mode: 0, level: crate::game::specials::LevelState::blank(), character: crate::game::actors::Actors::blank() });
                init(&mut ctx, Vec2::new(-4.0, -84.0), Vec2::new(324.0, 164.0));
                f(&mut ctx);
            })
            .unwrap()
            .join()
            .unwrap();
    }

    #[test]
    fn proxy_free_list_order() {
        with_ctx(|ctx| {
            let bp = &ctx.world.broad_phase;
            assert_eq!(bp.free_proxy, 0);
            assert_eq!(bp.time_stamp, 1);
            for i in 0..B2_MAX_PROXIES - 1 {
                assert_eq!(bp.proxy_pool[i].get_next(), i as u32 + 1);
                assert!(!bp.proxy_pool[i].is_valid());
            }
            assert_eq!(bp.proxy_pool[B2_MAX_PROXIES - 1].get_next(), B2_NULL_PROXY);
            assert_eq!(bp.free_pair, 0);
            assert_eq!(bp.pairs[B2_MAX_PAIRS - 1].next, B2_NULL_PAIR);
            assert_eq!(bp.pairs[7].next, 8);
            assert!(bp.hash_table.iter().all(|&h| h == B2_NULL_PAIR));

            // Non-overlapping boxes: no pairs, so no contact callbacks.
            let mut ids = [0u32; 4];
            for k in 0..4 {
                let x = 10.0 * k as f64;
                let (f, id) = create_proxy(ctx, Vec2::new(x, 0.0), Vec2::new(x + 1.0, 1.0), 100 + k as u32);
                assert_eq!(f, Flow::Continue);
                ids[k] = id;
            }
            assert_eq!(ids, [0, 1, 2, 3]);
            assert_eq!(ctx.world.broad_phase.proxy_count, 4);
            assert_eq!(ctx.world.broad_phase.free_proxy, 4);
            assert_eq!(ctx.world.broad_phase.pair_count, 0);
            // Bounds are sorted, lower even / upper odd, stabbing counts consistent.
            for axis in 0..2 {
                let b = &ctx.world.broad_phase.bounds[axis];
                for i in 1..8 {
                    assert!(b[i - 1].value <= b[i].value);
                }
            }
            // Destroy 1 then 2: free list is LIFO (2 -> 1 -> 4 -> ...).
            assert_eq!(destroy_proxy(ctx, 1), Flow::Continue);
            assert_eq!(destroy_proxy(ctx, 2), Flow::Continue);
            let bp = &ctx.world.broad_phase;
            assert_eq!(bp.free_proxy, 2);
            assert_eq!(bp.proxy_pool[2].get_next(), 1);
            assert_eq!(bp.proxy_pool[1].get_next(), 4);
            assert_eq!(bp.proxy_pool[1].overlap_count, B2_INVALID);
            assert_eq!(bp.proxy_pool[1].upper_bounds, [B2_INVALID, B2_INVALID]);
            assert_eq!(bp.proxy_count, 2);
            let (f, id) = create_proxy(ctx, Vec2::new(50.0, 0.0), Vec2::new(51.0, 1.0), 200);
            assert_eq!(f, Flow::Continue);
            assert_eq!(id, 2);
            let (f, id) = create_proxy(ctx, Vec2::new(60.0, 0.0), Vec2::new(61.0, 1.0), 201);
            assert_eq!(f, Flow::Continue);
            assert_eq!(id, 1);
            let (f, id) = create_proxy(ctx, Vec2::new(70.0, 0.0), Vec2::new(71.0, 1.0), 202);
            assert_eq!(f, Flow::Continue);
            assert_eq!(id, 4);
            // The stale query results array gets results in broad-phase order.
            let mut out = [NONE; 8];
            let n = query_aabb(ctx, Vec2::new(-1.0, -1.0), Vec2::new(100.0, 2.0), &mut out, 8);
            assert_eq!(n, (Flow::Continue, 5));
        });
    }

    #[test]
    fn compute_bounds_masks() {
        with_ctx(|ctx| {
            let bp = &ctx.world.broad_phase;
            let (l, u) = compute_bounds(bp, Vec2::new(-4.0, -84.0), Vec2::new(324.0, 164.0));
            assert_eq!(l, [0, 0]);
            assert_eq!(u, [65535, 65535]);
            // Out-of-world values clamp. A NaN coordinate falls through b2Min (NaN < x is false ->
            // picks the world upper bound) and b2Max alike, so it quantizes to the world UPPER corner:
            // lower 65535 & 0xFFFE = 65534, upper 65535. (CreateProxy has no IsValid guard; MoveProxy does.)
            let (l, u) = compute_bounds(bp, Vec2::new(-1000.0, f64::NAN), Vec2::new(1000.0, f64::NAN));
            assert_eq!(l, [0, 65534]);
            assert_eq!(u, [65535, 65535]);
            assert!(l[0] & 1 == 0 && u[0] & 1 == 1);
        });
    }

    #[test]
    fn proxy_pool_exhaustion_aborts_without_mutation() {
        with_ctx(|ctx| {
            ctx.world.broad_phase.free_proxy = B2_NULL_PROXY;
            let before = ctx.world.broad_phase.proxy_count;
            let (f, id) = create_proxy(ctx, Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0), 7);
            assert_eq!(f, Flow::Abort(AbortSite::ProxyPoolExhausted));
            assert_eq!(id, B2_NULL_PROXY);
            assert_eq!(ctx.world.broad_phase.proxy_count, before);
            assert_eq!(ctx.world.broad_phase.free_proxy, B2_NULL_PROXY);
        });
    }
}
