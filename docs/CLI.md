# The `hw` command line

`hw` replays inputs (TAS strings) through the port and verifies the result against recorded Flash
traces. Build it from the workspace root:

```powershell
cargo build -p hw-cli --release
```

The executable is `target/i686-pc-windows-msvc/release/hw.exe`. Keep the workspace's i686 target:
its CRT math is the measured reference implementation (see `docs/MATH.md`).

## Commands

```powershell
hw sim --level 13098 --char 2 --tas reference/tas/official/dotd6.tas --out dotd6-cpu.jsonl --dump-from 200 --dump-to 208
hw verify --trace reference/traces/flash32/tas-dotd_pairmax_1-diagnostic-v1.jsonl
hw gen-inputs --level 45 --char 3 --seed 12345 --frames 600 --style forward-jumpy
hw bench --threads 8
```

### `hw sim`

`--tas` takes a file or literal bits: eight characters per input frame, in the order
left, right, up, down, space, shift, ctrl, z. It runs input length + 60 frames by default;
`--frames N` sets the total, including the idle tail. Output is JSONL on stdout, or in `--out`
(an existing file is never overwritten). A file output gets an adjacent `.metadata.json` with the
exact input, so the trace can be verified later.

Observer switches add more recorded state per frame. Each of `--game`, `--links`, `--mechanics`,
`--callbacks` and `--level-state` implies the ones before it:

| Switch | Adds |
| --- | --- |
| (default) | frame, iteration, inputs, error, lock, state hash, dead, inputAllowed, finish |
| `--broadphase` | hash of the broadphase arrays |
| `--physics` | named numeric physics fields |
| `--game` | physics plus private character state and retained body/joint roots |
| `--links` | live list topology and reference identities |
| `--mechanics` | damage composites and the shared contact-result normal |
| `--callbacks` | retained shape keys, contact thresholds, ADD/result presence |
| `--level-state` | level work queues, special fields, spike impalement state |
| `--level-add-state`, `--listeners` | ordinary level ADD registration; character listener identities |

`--dump-from A --dump-to B` (inclusive, one-based) also writes raw state strings for that window.
Input iteration can stall on a Flash error; the simulation follows that iteration rather than
indexing by wall-clock frame.

`--real f64` is accepted for compatibility and is the only supported value.

### `hw verify`

`hw verify --trace FILE` replays the trace's input and compares every recorded field: input, error,
lock, dead, iteration, inputAllowed, hash, length, and finish/broadphase when present. Raw dumps compare
all their bytes. The trace's `.metadata.json` resolves the official TAS strings by their recorded
SHA-256; other sessions need `--tas`. Level, character and frame counts are checked.

The first mismatch reports a named field when raw state is available, or a hash difference otherwise.
The exit code is nonzero on a mismatch, invalid or truncated input, or a port-only error. Observer streams
that appear or disappear partway through a trace are rejected.

A MATCH means the recorded observations agree. It is not a claim about state the recorder did not
serialise; `docs/TRACE_FORMAT.md` lists what each stream covers.

`tools/verify-corpus.ps1` runs `hw verify` on every official input in `reference/tas/index.json`, using its
listeners trace when present and its diagnostic trace otherwise. `-Executable PATH` selects another build.

### `hw gen-inputs`

Emits one eight-bit entry per line. Styles: `idle`, `forward-jumpy`, `random`, `mash`, `death-seek`.
`--out` adds generator/seed/style provenance beside the TAS. The generator draws once per frame using
the SplitMix32 constants in `crates/hw-cli/src/inputs.rs`; it is coverage tooling, not randomness in the game.

### `hw bench`

`hw bench [--threads N] [--cases all|official|generated] [--repeat N] [--out FILE]` replays the
available corpus on one thread and then on N threads and reports frames per second. Run it from the
workspace root.

## Searching

Input search lives in a second binary, `hw-search`; see `docs/SEARCH.md`.
