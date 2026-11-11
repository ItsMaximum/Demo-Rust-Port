# Getting started

## Requirements

- Windows 10 or 11 with the MSVC build tools (Visual Studio Build Tools, "Desktop development with C++").
- A Rust toolchain from [rustup](https://rustup.rs) (stable is enough) and the 32-bit target:

  ```powershell
  rustup target add i686-pc-windows-msvc
  ```

The workspace sets `i686-pc-windows-msvc` as its default target in `.cargo/config.toml`. That is deliberate:
the Flash Player is a 32-bit program, and on this target the port's `sin`/`cos` use the same CRT and its
`atan2` uses the same x87 instructions. See [MATH.md](MATH.md).

Search tools keep snapshots of the whole simulation per worker, so they link with the large-address-aware
flag and can use up to 4 GB of the 32-bit address space.

## Build

```powershell
cargo build --release --workspace --features hw-cli/lab
```

This builds `hw.exe`, `hw-search.exe` and `obby-search.exe` into `target/i686-pc-windows-msvc/release/`.
`hw-search` needs the `lab` feature, which adds the research hooks the searches use (they do nothing unless a
search arms them). To build only `hw` and `hw-search`, use `cargo build --release -p hw-cli --features lab`;
`obby-search` (see [OBSTACLE_COURSE.md](OBSTACLE_COURSE.md)) is `cargo build --release -p obby-search`.

## Replay and verify

```powershell
$hw = "target/i686-pc-windows-msvc/release/hw.exe"

# Run a TAS and write one JSON record per frame (runs/ is ignored by git); the last line of console
# output reports the finish frame
New-Item -ItemType Directory -Force runs | Out-Null
& $hw sim --level 3214 --char 4 --tas reference/tas/gbeqe-finish-f227-fp15.tas --out runs/gbe.jsonl

# Compare a recorded Flash trace with the port, hash for hash
& $hw verify --trace reference/traces/flash32/tas-dotd_pairmax_1-diagnostic-v1.jsonl

# Verify every official input (listeners trace when present, else the diagnostic trace)
./tools/verify-corpus.ps1
```

A TAS is a string of `0` and `1` characters, eight per frame, in the order left, right, up, down, space,
shift, ctrl, z; see [CLI.md](CLI.md). Files in `reference/tas/` are TAS strings. Level numbers are the
game's level ids (3214 Gut Bus Extreme, 2433 A large Satan eats u, 45 Obstacle Course, ...) and characters
are 1 wheelchair, 2 segway, 3 bicycle (Irresponsible Dad), 4 motorcart.

## Search

```powershell
$search = "target/i686-pc-windows-msvc/release/hw-search.exe"
& $search help            # curated commands
& $search help qe         # flags for one command
& $search x               # experimental commands
```

Worked examples are in [SEARCH_RECIPES.md](SEARCH_RECIPES.md); concepts and command reference are in
[SEARCH.md](SEARCH.md).

## Tests

```powershell
cargo test --release --features hw-cli/lab
```

This replays the shipped Flash traces and the hand-written fixtures through the port and checks every
recorded field. The whole run takes a few minutes.

### Full trace corpus

The repository ships about 115 MB of recorded traces. The complete corpus, with the coverage inputs
for all nine levels and every official-input stream, is above 700 MB and is not included. Tests that
need a trace which is not present are compiled out unless you enable the `full-corpus` feature, so a
default `cargo test` never fails on a missing file. If you have the full set of traces, copy them
into `reference/traces/flash32/` (each trace `NAME.jsonl` with its `NAME.jsonl.metadata.json`) and run:

```powershell
cargo test --release --features hw-cli/lab,hw-core/full-corpus,hw-cli/full-corpus
```

`tools/verify-corpus.ps1 -RequireAll` likewise fails unless every official input has its listeners trace.

## Regenerating `hw-data`

`crates/hw-data/src/generated.rs` and `runtime_guides.rs` are generated from the measured constants in
`reference/constants/constants-v1.jsonl` and the official strings in `reference/tas`:

```powershell
python tools/gen-data.py
```

Measuring those constants needs the game itself and a Flash Player, which this repository does not
include; the constants file is the recorded result.
