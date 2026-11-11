# Repository layout

A guide to what is where, what depends on what, and which files are written by hand and which are generated.

```
.cargo/config.toml       default build target (i686-pc-windows-msvc)
Cargo.toml, Cargo.lock   one workspace, six crates
crates/                  the code
docs/                    documentation
reference/               data: inputs (TAS), recorded Flash traces, measured constants, math evidence
tools/                   gen-data.py, verify-corpus.ps1
.github/workflows/       CI
LICENSE-MIT, LICENSE-APACHE, NOTICE
```

Generated build output goes to `target/`, and run directories you create (searches, `hw sim --out`) belong under
`runs/`; both are ignored by git.

## Crates

| Crate | Kind | Role |
| --- | --- | --- |
| `hw-real` | library | the `Real` trait: f64 with the semantics of AS3 `Number`, plus the `rl!` bit-literal macro and AS3 helpers (`math_min`, `to_uint32`, ...) |
| `hw-flashmath` | library, 3 fuzz binaries | `sin`, `cos`, `atan2` exactly as the 32-bit Flash Player computes them (see [MATH.md](MATH.md)) |
| `hw-data` | library (`no_std`) | baked level, character and official-TAS tables (`TAS_STRINGS`), as plain numbers |
| `hw-core` | library | the port itself: Box2D physics (`b2/`), game logic (`game/`), and the state serialisers used for verification |
| `hw-cli` | binaries `hw`, `hw-search` | simulate, verify, generate inputs, benchmark; input search |
| `obby-search` | binary | research tool for one level (Obstacle Course), see [OBSTACLE_COURSE.md](OBSTACLE_COURSE.md) |

Dependencies point downward only:

```
hw-real  <-  hw-flashmath  <-  hw-core  <-  hw-cli
                  hw-data  <--/   ^
                  hw-data  <------+----------- obby-search (also uses hw-real)
```

`hw-core` and `hw-flashmath` depend on `hw-real` without its `std` feature so they stay usable without the standard
library. `hw-data` has no dependencies.

### Features

| Crate | Feature | Effect |
| --- | --- | --- |
| `hw-core`, `hw-flashmath` | `std` (default) | host-math paths and the serialisers |
| `hw-core`, `hw-flashmath`, `hw-cli` | `portable-math` | use the portable u32-limb trig instead of the host CRT and x87 code |
| `hw-core`, `hw-cli` | `lab` | research hooks (`hw-core/src/lab.rs`) that `hw-search` and `obby-search` use; off by default and recorded as having no effect on results |
| `hw-core`, `hw-cli` | `full-corpus` | compile the tests that need the full trace corpus ([GETTING_STARTED.md](GETTING_STARTED.md)) |

### Inside `hw-core`

| Path | Contents |
| --- | --- |
| `src/b2/` | the Box2DFlashAS3 2.0.2 port, one module per class family (`body`, `shapes`, `collision`, `broad_phase`, `contacts`, `joints`, `island`, `world`, `math`, `settings`, `state`) |
| `src/game/` | the game: `session` (the frame loop), `level` and `specials`, one module per character (`wheelchair`, `segway`, `bicycle`, `child`, `motorcart`) with their `*_damage` modules, plus `damage`, `composites`, `poses`, `contact_listener` |
| `src/arena.rs`, `ctx.rs`, `flow.rs`, `flag.rs`, `input.rs` | fixed-capacity object arenas, the simulation context, abort handling (`Flow`, `AbortSite`), stored booleans, input bits |
| `src/*_observer.rs`, `diagnostic.rs` | the state serialisers that produce the strings compared against Flash traces (see [TRACE_FORMAT.md](TRACE_FORMAT.md)) |
| `src/lab.rs` | the `lab` research hooks |
| `tests/` | integration tests that replay recorded traces and fixtures; `tests/support/` holds shared helpers |

### Inside `hw-cli`

`src/main.rs` implements `hw`; `diff.rs` finds the first mismatching field; `inputs.rs` is the input generator;
`bench.rs` the benchmark. `src/search/` is `hw-search`: `main.rs` has the command table and help text, and each
command lives in its own module (`qe.rs`, `explore.rs` for `run`, `perturb.rs`, `brute.rs`, `sparse.rs`, `tail.rs`,
`analyze.rs`, ...), with `*_policy.rs` modules holding their scoring and selection rules.

## `reference/`

### `reference/tas/`

A TAS file is a string of `0` and `1` characters, eight per frame (see [CLI.md](CLI.md)). Whitespace is ignored.

| Files | What they are |
| --- | --- |
| `dotd4`, `dotd6`, `dotd_pairmax_1`, `snowy`, `bmx610_v98`, `bmx1003`, `gbe_err_1`, `gbe_freeze_2` `.tas` | the eight official inputs, with their Flash traces; listed in `index.json` |
| `bmx610.tas` | a corrected version of `bmx610_v98`, listed in `launches.json` |
| `gbeqe.tas`, `satanqe.tas`, `traptracqe.tas` | QE launch strings used as search seeds |
| `gbeqe-finish-f<frame>[-fp<N>].tas`, `satanqe-finish-f<frame>[-fp<N>].tas` | found finishing runs; `f` is the finish frame and `fp` the number of frame-perfect inputs |
| `*.verification.json` | the proof record for a found run (below) |
| `obstacle-*.tas` | strings from the Obstacle Course research, named for the event they show and its frame (`obstacle-pairmax-f274`: the pair pool overflows on frame 274) |
| `extra/` | the 108 generated coverage inputs (`coverage-v1-L<level>-C<char>-<style>.tas`), each with a `.tas.metadata.json` (generator, seed, style), plus `plan.json` listing all cases and their traces |
| `generated/` | other generated inputs (`wheelchair-controls-v1.tas`) |

**`index.json`** is an array with one entry per official input: `file`, `trace_stem`, `const` (a legacy identifier
that nothing reads), `name`, `level`, `character`, `frames`, `sha256`, `finish_frame` and the paths
of its traces (`diagnostic_trace`, `physics_trace`, `character_state_trace`, `level_state_trace`,
`listeners_state_trace`). Only some of those traces are shipped. `hw verify` resolves a trace's input by `sha256`
through this file and `hw-data`'s `TAS_STRINGS` (which embeds the official strings), and `tools/verify-corpus.ps1`
walks it. `hw-cli` compiles `index.json` and `launches.json` into the binary, so editing either needs a rebuild.

**`launches.json`** lists additional launch strings the same way, with `finish_confirmed` (`"flash"` when the finish
was reproduced in the game) and `derived_from`.

**`*.verification.json`** records one verified result: level, character, finish frame and seconds, `frame_perfect`
count, key-event counts, the SHA-256 of the input, the trace it matched and that trace's hash and frame count,
the hash of the Flash Player, and which search produced it.

### `reference/traces/flash32/`

Recorded Flash traces, each `NAME.jsonl` with a `NAME.jsonl.metadata.json`. File names, record format and observer
streams are in [TRACE_FORMAT.md](TRACE_FORMAT.md). This directory holds a subset of the full corpus.

### `reference/constants/`

`constants-v1.jsonl` holds numbers measured in the game: for each character and level, the display-object
properties (position, rotation, scale, size) the game code reads, as raw 16-digit hex double bits after Flash's own
quantisation. `tools/gen-data.py` turns it into the tables in `hw-data`. The probe that recorded it is not part of
this repository.

### `reference/math/bochs-x87/`

The unmodified constant headers from the Bochs FPU sources that `hw-flashmath`'s x87 code derives from, with the
pinned commit in `revision.txt`. See [MATH.md](MATH.md) and `NOTICE`.

### `reference/hashes.json`

SHA-256 of the game build and Flash Player that all traces were recorded with.

## Generated and hand-written files

| File | How it is produced |
| --- | --- |
| `crates/hw-data/src/generated.rs`, `runtime_guides.rs` | generated by `python tools/gen-data.py` from `reference/constants/constants-v1.jsonl` and `reference/tas`; do not edit by hand |
| `crates/hw-flashmath/src/x87_constants.rs`, `x87_atan_constants.rs` | constants derived from the Bochs headers (see `NOTICE`) |
| `reference/traces/**`, `reference/constants/**` | recorded from the game; never produced by this repository's code |
| `reference/tas/extra/*.tas` | produced by `hw gen-inputs` (seeded, so reproducible); `plan.json` came from a planning script that is not included |
| `reference/tas/*-finish-*.tas`, `obstacle-*.tas` | outputs of searches, kept as results |
| everything else | written by hand |

## Conventions

- **Line endings:** `.gitattributes` sets LF for text files; `.swf` and `.png` are binary.
- **Target:** `i686-pc-windows-msvc`, set in `.cargo/config.toml` ([GETTING_STARTED.md](GETTING_STARTED.md) explains why).
- **Release profile:** fat LTO, one codegen unit, `panic = "unwind"` (searches catch panics per trial).
- **Naming:** crates are `hw-*` (the game's initials), Rust modules follow the AS3 class names in snake case
  ([PORTING.md](PORTING.md)), and trace and input names carry the level id (`L<id>`) and character (`C<1-4>`).
- **Level ids and characters:** see [GETTING_STARTED.md](GETTING_STARTED.md).
