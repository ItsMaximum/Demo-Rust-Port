# Demo Rust Port

A headless, bit-exact Rust port of the game logic of the Happy Wheels demo (Flash), with command-line
tools for replaying inputs and searching for new ones.

The port reproduces the Flash game frame by frame: Box2DFlashAS3 2.0.2 physics, the level and character
code, and the way the 32-bit Flash Player computes `sin`, `cos` and `atan2`. It is checked against traces
recorded from the real game, so a replay here gives the same state, hash for hash, as the game.
That makes it a fast, scriptable simulator for tool-assisted runs (TAS): you can replay a run in
milliseconds, and search millions of input variations for glitches such as quantum entanglement (QE) and
pair-pool overflow ("pairmax") launches.

Nothing from the game is distributed here: no SWF, no Flash Player, no decompiled code. The level and
character tables in `hw-data` are numbers measured from the game, and the traces under `reference/` are
recordings of its behaviour. See [NOTICE](NOTICE).

## What is in the box

| Path | What it is |
| --- | --- |
| `crates/hw-core` | the port: Box2D physics and game logic |
| `crates/hw-flashmath` | Flash-exact `sin`/`cos`/`atan2` |
| `crates/hw-real` | the numeric trait that gives f64 the semantics of AS3 `Number` |
| `crates/hw-data` | baked level, character and official-TAS tables |
| `crates/hw-cli` | `hw` (simulate, verify, generate inputs, benchmark) and `hw-search` (input search) |
| `crates/obby-search` | a research tool for one level (Obstacle Course, level 45, Irresponsible Dad) |
| `reference/` | official and found TAS strings, recorded Flash traces, math evidence |
| `docs/` | documentation, see below |

## Quick start

You need Windows with the MSVC build tools and a Rust toolchain. The workspace builds for
`i686-pc-windows-msvc` (32-bit), because that matches the Flash Player's CRT; see [docs/MATH.md](docs/MATH.md).

```powershell
rustup target add i686-pc-windows-msvc
cargo build --release -p hw-cli --features lab

# Replay an input and check it against a recorded Flash trace
target/i686-pc-windows-msvc/release/hw.exe verify --trace reference/traces/flash32/tas-dotd_pairmax_1-diagnostic-v1.jsonl

# Run a found input: Gut Bus Extreme (level 3214, character 4), finishing on frame 227
target/i686-pc-windows-msvc/release/hw.exe sim --level 3214 --char 4 --tas reference/tas/finishes/gbeqe-finish-f227-fp15.tas

# List the search commands
target/i686-pc-windows-msvc/release/hw-search.exe help
```

More: [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md).

## Documentation

| Document | Contents |
| --- | --- |
| [GETTING_STARTED.md](docs/GETTING_STARTED.md) | build, run, verify, run the tests, the trace corpus |
| [CLI.md](docs/CLI.md) | `hw` reference and the TAS format |
| [SEARCH.md](docs/SEARCH.md) | what QE and pairmax are, the search commands, and how they work |
| [SEARCH_RECIPES.md](docs/SEARCH_RECIPES.md) | copy-paste searches: finding a QE launch, then minimising its frame-perfect inputs |
| [OBSTACLE_COURSE.md](docs/OBSTACLE_COURSE.md) | the level 45 research: measured mechanics and what is still unsolved |
| [REPOSITORY.md](docs/REPOSITORY.md) | directory layout, crate dependencies, the `reference/` file formats, generated files |
| [TRACE_FORMAT.md](docs/TRACE_FORMAT.md) | how the Flash traces are recorded and what a match proves |
| [MATH.md](docs/MATH.md) | how the Flash-exact math is implemented and measured |
| [PORTING.md](docs/PORTING.md) | conventions for contributing to the port |

## Results

The searches in this repository found these finishing inputs. Each replays to a finish in the port and
was recorded in the real game for comparison.

| Level | Character | Finish | File |
| --- | --- | --- | --- |
| Gut Bus Extreme (3214) | 4 | frame 227 (7.57 s), 15 frame-perfect inputs | `reference/tas/finishes/gbeqe-finish-f227-fp15.tas` |
| A large Satan eats u (2433) | 4 | frame 125 (4.17 s), 23 frame-perfect inputs | `reference/tas/finishes/satanqe-finish-f125-fp23.tas` |

Obstacle Course (level 45) with Irresponsible Dad has no sub-10-second finish yet; the findings so far
are in [docs/OBSTACLE_COURSE.md](docs/OBSTACLE_COURSE.md).

## Tests

```powershell
cargo test --release --features hw-cli/lab
```

The repository ships a subset of the recorded traces (about 115 MB). Tests that need the rest are behind
the `full-corpus` feature; see [GETTING_STARTED.md](docs/GETTING_STARTED.md).

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option. Third-party
notices (Box2D, the Bochs-derived x87 code) are in [NOTICE](NOTICE).
