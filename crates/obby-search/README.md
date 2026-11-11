# obby-search

Research tool for Obstacle Course (level 45) with Irresponsible Dad (character 3): a QE / pairmax
search for a finish in under 10 seconds. The notes, measured mechanics and command overview are in
[`docs/OBSTACLE_COURSE.md`](../../docs/OBSTACLE_COURSE.md); `obby-search help` lists the commands.

```powershell
cargo build --release -p obby-search
target/i686-pc-windows-msvc/release/obby-search.exe help
```

Run the build from the repository root; the binary lands in the shared `target/` directory. The workspace builds
with `panic = "unwind"`, which the tool relies on to catch a panic from a rare simulation state per trial.
