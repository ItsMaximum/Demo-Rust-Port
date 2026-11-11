# Contributing

Bug reports and pull requests are welcome. This is a bit-exact port, so changes are judged by whether
replays still match the recorded Flash traces.

## Building and testing

See [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md). Before opening a pull request:

```powershell
cargo build --release --workspace
cargo test --release --features hw-cli/lab
```

If you have the full trace corpus, also run the tests with `hw-core/full-corpus,hw-cli/full-corpus`.

## Porting rules

Read [docs/PORTING.md](docs/PORTING.md) first. The short version: keep the original statement order and
operand association, use the `Real` trait helpers instead of `f64` operators with different semantics, never
panic on a hot path, and keep stored state free of pointers and native booleans.

## Changing simulation behaviour

Anything that changes a simulation result must be explained by a recorded trace. Adding a verified trace
means a `NAME.jsonl` plus its `NAME.jsonl.metadata.json` in `reference/traces/flash32/`, and, for an
official input, an entry in `reference/tas/index.json`. Keep traces small: the repository ships a subset
of the corpus on purpose.

## Search tools

New `hw-search` commands start under `hw-search x` (experimental). A command moves to the documented set
once its flags are stable, it has a help block in `crates/hw-cli/src/search/main.rs`, and
`docs/SEARCH.md` describes it.

## Style

Format with `cargo fmt` where the surrounding file already follows rustfmt; otherwise match the file.
Comments explain why, and cite the original behaviour when the code looks odd. Do not commit run output
(pools, logs, traces from local searches).
