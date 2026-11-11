# Flash traces and the verification format

The port is checked against traces recorded from the real game running in the 32-bit Flash Player 32.
A trace is a JSON-lines file with one record per game frame, produced by replaying a TAS through the
game's own replay loop while an observer serialises state. `hw verify` replays the same input through
the port and compares every record.

The recording tooling (patched game builds, a Flash Player and a decompiler) is not part of this
repository, and neither are the game files. What ships is the recorded output, so the port can be
verified without any of that. `reference/hashes.json` records the SHA-256 of the game build and Flash Player the
traces were recorded with.

## Files

Each trace `NAME.jsonl` has a sidecar `NAME.jsonl.metadata.json` with the level, character, frame
counts, the exact input (`tas_bits`) and its SHA-256, the observer streams it contains, and the hashes
of the recorder. `hw verify` reads the sidecar; official TAS strings are resolved from their SHA-256
through `reference/tas/index.json`.

Naming:

| Pattern | Meaning |
| --- | --- |
| `tas-<name>-diagnostic-v1` | an official input, per-frame hash diagnostics |
| `tas-<name>-{physics,game,level,listeners,level-add}-v1` | the same input with more observer streams |
| `coverage-v1-L<level>-C<char>-<style>[-level-v1]` | generated coverage inputs (`hw gen-inputs` styles) |
| `b2-*-v1`, `level-shapes-*`, `character-*`, `*-damage-*`, `*-guards-*` | isolated construction and fixture traces used by `hw-core` tests |
| `math-v1`, `atan2-edges-v1`, `arithmetic-edges-v1` | math and arithmetic edge cases |

## Record format

Every frame records `schema_version`, `frame`, `iteration`, the eight input bits, `err`, `lock`, a
16-digit `hash`, the state-string length, `dead` and `inputAllowed`; sessions that can finish also record
the first observed `finish` frame. Dump windows add the raw `state` string. The hash is two Jenkins
one-at-a-time lanes (seeds 0 and 2654435769) over the ASCII state string. Numbers are raw 16-digit
double bit patterns with no NaN or zero normalisation.

`iteration` counts consumed input entries; it stalls when the game throws, which is why the port follows
the iteration rather than the wall-clock frame.

## Observer streams

Each stream adds a hash per frame and a named state string inside a dump window. Each of the first
five implies the ones before it.

| Stream | Covers |
| --- | --- |
| broadphase | bounds, proxies, pairs, hash table, query and pair-buffer entries |
| physics | body transforms and sweeps, shape geometry, joints and caches, every contact's flags, TOI and manifold points |
| game | character scalar state, retained body and joint roots (including the child actor) |
| links | live list topology and normalised object references (null 0, null-contact -2, unknown -1) |
| mechanics | damage composites, active end joints, ligament inputs, the shared contact-result normal |
| callbacks | retained shape keys, dictionary counts, contact thresholds, ADD/result presence, queued results |
| level | action and removal queues, persistent special fields, spike impalement state |
| level-add, listeners | ordinary level ADD registration; character listener method and receiver identity |

## What a MATCH does and does not show

A MATCH says every serialised value agreed for every frame. It does not cover state the observers do
not serialise: the identity of unknown retained shape keys, `Dictionary` enumeration order, presentation-only
sound payloads, and special closures (reported as `other`). `hw verify` rejects traces whose declared
streams are missing, and traces where a stream appears or disappears midway.

## Isolated fixtures

Some traces isolate one subsystem: `level-shapes-*` stop after the level's shape construction and step
only the world; `character-*`, `*-damage-*` and `*-guards-*` exercise construction, damage and callback
paths with the character created in isolation. These are read by the `hw-core` integration tests rather
than by `hw verify`, and their metadata says which parts of the game were replaced.

## Full trace corpus

The repository ships a subset of the recorded traces to keep it small. The complete corpus (all coverage
inputs and all official-input streams) is larger than 700 MB. Tests that need a trace that is not present
are behind the `full-corpus` feature; see `docs/GETTING_STARTED.md`.
