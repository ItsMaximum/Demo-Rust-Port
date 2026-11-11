//! AS3 runtime-error model. The release Flash Player swallows an uncaught error thrown inside the
//! ENTER_FRAME handler: everything mutated before the throw stays mutated, the rest of the handler
//! is skipped, and `b2World.m_lock` stays latched. Physics/game functions that can throw return
//! `Flow`; `flow!` propagates an abort with an early return.

/// Where an AS3 error was thrown (the reachable sites are listed in `b2::broad_phase`).
#[repr(u32)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AbortSite {
    None = 0,
    /// b2BroadPhase.CreateProxy: 513th live proxy (m_proxyPool[65535] is undefined -> null.GetNext()).
    ProxyPoolExhausted = 1,
    /// b2PairManager.AddPair: 4097th pair (m_pairs[65535] undefined -> null.next).
    PairPoolExhausted = 2,
    /// b2PairManager.AddBufferedPair: m_pairBuffer[4096] undefined.
    PairBufferAdd = 3,
    /// b2PairManager.RemoveBufferedPair: m_pairBuffer[4096] undefined.
    PairBufferRemove = 4,
    /// b2PairManager.Commit: Find returned null -> null.ClearBuffered().
    CommitFindNull = 5,
    /// b2ContactManager.Destroy / PairRemoved: contact shape's body is null.
    ContactManagerNullBody = 6,
    /// b2Settings.b2Assert(false) in b2PolyAndCircleContact.
    B2AssertPolyCircle = 7,
    /// Game code dereferenced the null returned by a lock-guarded CreateBody/CreateShape.
    GameNullDeref = 8,
    /// Port-only: a fixed capacity of the port (not of Box2D) was exceeded.
    PortCapacity = 9,
    /// Port-only: explicitly unsupported game path (never maps to a Flash error).
    PortUnimplemented = 10,
    /// Corrupted broadphase stabbing counts walk to bounds[-1], then null.IsLower().
    BroadPhaseQueryNullBound = 11,
}

/// Control flow result of a function that may throw.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[must_use]
pub enum Flow {
    Continue,
    Abort(AbortSite),
}

impl Flow {
    #[inline(always)]
    pub fn is_abort(self) -> bool {
        !matches!(self, Flow::Continue)
    }
}

/// Packed status word for records: 0 = ok, otherwise the abort site plus a call-site id.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Status(pub u32);

impl Status {
    pub const OK: Status = Status(0);
    pub fn abort(site: AbortSite, call_site: u32) -> Status {
        Status(((site as u32) << 16) | (call_site & 0xFFFF))
    }
    pub fn site(self) -> u32 {
        self.0 >> 16
    }
    pub fn call_site(self) -> u32 {
        self.0 & 0xFFFF
    }
}

/// Early-return propagation of an abort.
#[macro_export]
macro_rules! flow {
    ($e:expr) => {
        match $e {
            $crate::Flow::Continue => {}
            f => return f,
        }
    };
}
