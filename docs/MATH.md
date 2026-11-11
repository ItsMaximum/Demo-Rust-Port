# Math implementation and measured scope

Ordinary `Number` arithmetic in the game is SSE2 binary64, which Rust's `f64` reproduces exactly.
Transcendental functions are different: the 32-bit Flash Player calls into code that is not plain
SSE2. `hw-flashmath` reproduces what was measured, and the workspace builds for
`i686-pc-windows-msvc` so the host CRT matches the player's.

## What each function uses

| Function | Default i686 build | `portable-math` feature / other targets |
| --- | --- | --- |
| `sin`, `cos` | the 32-bit CRT, through raw words | portable u32-limb implementation of the x87 `FSIN`/`FCOS` evaluation |
| `atan2` | the player's x87 body (`FPATAN`) via inline assembly (`host_x87.rs`) | portable implementation of the same x87 sequence (`x87_atan.rs`) |

The portable paths exist so the port does not depend on host assembly or CRT behaviour. They are
tested against the same recorded Flash inputs as the native paths.

## atan2 is x87, not the CRT

The player's atan2 takes absolute inputs, runs `FPATAN`, loads x87 pi and subtracts the angle when x
is negative, then applies the sign of y. A newer CRT wraps this differently: it changes NaN payloads
and underflow results. Flash preserves NaN payloads and returns the bare body's value in the targeted
subnormal case, so the CRT wrapper must not be transplanted into the port.

The host x87 helper also reproduces x87 NaN propagation: it keeps the measured sign and payload and
picks the larger significand when both inputs are NaN. For an equal-payload tie, the positive sign wins
(`FLD` quiets signaling NaNs before the comparison).

Comparison against the installed CRT, over 34,000,225 pairs (seed `123456789abcdef0`):

| Candidate | Mismatches (all) | Mismatches (finite inputs) |
| --- | ---: | ---: |
| `libm` | 2,691,574 | 2,686,458 |
| portable fdlibm-style `atan2` | 6,754,127 | 6,749,011 |
| x87 ordinary body only | 6,117 | 997 |

All 997 finite mismatches of the x87 body are zero or subnormal outputs, where the CRT wrapper differs
from the player. These counts compare against the CRT, not directly against Flash; Flash itself is
measured by the recorded edge traces below.

## Recorded evidence

- `reference/traces/flash32/math-v1.jsonl`: 6,047 original-player sin/cos/atan2 inputs, including 1,764
  finite large-angle inputs. Native and portable implementations match every row.
- `reference/traces/flash32/atan2-edges-v1.jsonl`: all pairs of 38 operands (1,444 rows) covering
  subnormal boundaries, signed zeros, infinities and distinct NaN payloads. Native and portable
  `atan2` match every row. Signaling NaNs were quieted before being recorded, so raw sNaN inputs are
  not measured directly.

## Portable sin/cos

Large arguments (absolute value at least 2^63) make the player's wrapper loop `FPREM1` with the
extended constant `403e:c90fdaa22168c235` (the rounded extended pi times 2^62) and retry the
instruction without rounding the remainder to f64. The instruction's internal reduction uses the
narrower pi approximation `0.C90FDAA22168C234C * 2^2`, which preserves its large-angle error. The
portable code implements this with fixed u32 limbs (`x87_wide.rs`: 67-bit truncation, 64-bit
nearest-even rounding, exact bounded-width reduction). Standalone `FSIN`/`FCOS` evaluates differently
from `FSINCOS`, so the operations are never combined.

## Reproducing the comparisons

```text
cargo run -p hw-flashmath --release --bin fuzz_atan2 -- 20000000
cargo run -p hw-flashmath --release --bin fuzz_atan2 -- 20000000 1311768467463790320 player
cargo run -p hw-flashmath --release --bin fuzz_trig -- 25000000 11400714819323198485 all
```

The first run compares the x87 body against the installed CRT; the second compares the portable
implementation against the reproduced player path (zero mismatches over 34,000,225 pairs); the third
runs 25 million mixed-distribution finite sin/cos inputs against the installed CRT (zero mismatches).
These are host differential runs, not additional Flash measurements.

## Provenance

The x87 polynomial structure and constants derive from the Bochs x86 emulator's FPU sources at commit
`c38d6a4ab10e084b7d52d6606e90b7395c6e688f`. The notice those sources carry is kept in
`crates/hw-flashmath/src/x87-LICENSE.txt` and summarised in `NOTICE`; the unmodified constant headers
are in `reference/math/bochs-x87`.
