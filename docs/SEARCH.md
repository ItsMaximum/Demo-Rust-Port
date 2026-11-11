# Searching for inputs

`hw-search` looks for input strings (TAS) that reach a goal in the simulation, mostly a finish far faster
than the level normally allows. It runs many simulations in parallel on the Flash-exact port, so every
candidate it reports replays identically in the game.

This page explains the glitches the searches are built around, then the commands. For ready-made
command lines see [SEARCH_RECIPES.md](SEARCH_RECIPES.md).

## Background

### The finish check

Finishing a level fires when a contact result involving the finish plate arrives while the character's
focus body (the chest) is inside the plate's focus box, and the character is not dead. The check ignores
which body touched the plate, and it runs during the physics step, before death is processed. So a finish
only needs the *chest* to be in the box at the moment *any* body touches the plate.

### Launches

After the solver has moved the bodies, Box2D 2.0 runs a continuous-collision pass (time of impact) that stops
fast bodies at surfaces. A frame that throws part way (see pairmax below) skips that pass, so on that frame
bodies move by their velocity with no collision at all. A body moving at thousands of metres per second then crosses the level in one frame and
lands wherever its velocity points. A *launch* here means a frame where the living chest moves a large
distance (the searches use `--min-launch`, in metres). Most launches go nowhere useful; the search is for the
few that end with the chest in the finish box.

An end-of-frame position is a poor guide to where a launched chest has been: the chest can be left
"stranded" mid-flight by a time-of-impact result with no contact points. The searches therefore score the
chest's *solved* path (impact points and strand points) and any finish check that fires on the way, not just
where it stops.

### Quantum entanglement (QE)

When a limb breaks off, the game joins the two pieces with a chain of small ligament bodies. A ligament
"tear" creates a new anchor body and re-points the chain's first joint at it, but does not re-link the joint
edge lists of the bodies involved. The torn chain is then solved in a different island from the bodies it
pulls on, and the solver can no longer reconcile the two. Velocities on the chain grow without bound, by
roughly an order of magnitude every few frames (1e3 to 1e8 m/s). A QE needs the chain to be separated from the
body it was torn from (about 0.16 m is enough to break contact) and to stay in the world long enough to matter,
which in practice means an enclosure such as a pit.

Anything the whipping chain touches is thrown at tens to hundreds of metres per second, including the chest.
That is how a QE becomes a launch.

### Pairmax

The broad phase keeps at most 4096 overlapping pairs in a fixed pool. A QE can make enormous numbers of bodies
overlap at once; when the pool runs out, the game throws inside `World.Step`, after bodies have been moved and
before continuous collision, joint breaking and death checks. That is the "pairmax" (pair overflow) frame:
everything moves with no continuous collision, nothing can die, and joints do not break. If the bodies that
need new pairs keep doing so, the pool stays exhausted and every following frame throws as well (a *lock*), so the
character coasts ballistically through all geometry. A lock ends on the first frame whose synchronisation
completes, usually when a body leaves the world and its proxy is destroyed.

### Frame-perfect inputs

Search results are chaotic: changing any input before a launch changes the launch. Every key event that affects
the simulation before the launch is therefore *frame-perfect*. `canon`, `sparse` and `tail` take a finishing
string and shrink the number of such events, which is what makes a found run practical to perform.

## Commands

All commands take `--level ID` and `--char N` (see [GETTING_STARTED.md](GETTING_STARTED.md) for the ids).
`hw-search help <command>` prints the full flag list. Searches take `--threads` (default: logical cores minus two,
at most 16) and stop at `--hours` or when a file named `STOP` appears in the run directory.

| Command | What it does | Main outputs in `--run-dir` |
| --- | --- | --- |
| `analyze` | one JSON line per frame: chest solve and time-of-impact chain, finish checks, freezes | stdout |
| `qe` | evolves launch lineages from seed TAS files, scored by how far the chest's launch chain gets along a viable route | `pool.jsonl`, `launches.jsonl`, `finishes.jsonl`, `stats.jsonl`, `best-*.tas`, `winner-1.tas` |
| `run` | Go-Explore over grab and pose inputs for a one-frame chest launch onto the finish; stops at the first finish that an exact from-scratch replay confirms | `winner-N.tas`, `launches.jsonl`, `archive.jsonl`, `stats.jsonl` |
| `perturb` | mutates known QE bases looking for a finishing launch | `winner-1.tas`, `best-*.tas`, `improvements.jsonl`, `launches.jsonl`, `pool.jsonl` |
| `brute` | exhaustively tries every input on the last `--depth` frames of a launch-prone string | `--out` (default `brute.jsonl`) and a `.tas` per finish |
| `canon` | removes every press/release the finish does not need and prints the result | stdout |
| `sparse` | rewrites a finishing string with as few frame-perfect events as possible | `*.tas`, `finishes.jsonl`, `stats.jsonl` |
| `tail` | tries every low-input tail after a cut point | `finishes.jsonl`, `launches.jsonl`, `.tas` files |
| `cannon` | replays a prefix, then overwrites the chest velocity with each point of a grid and reports which launch velocities finish; a what-if map, not an input | stdout |
| `confirm` | replays a TAS from scratch and confirms a same-frame-launch finish | stdout |

Seed files are TAS strings (`--seeds a.tas,b.tas`) or, for `qe`, a previous run's `pool.jsonl` (`--pool-file`).
A seed string should end on the frame of a launch.

Every reported winner is confirmed by replaying the full input from frame 0 in a fresh simulation, with no
state carried over from the search, so a `winner-N.tas` is a complete input.

### Experimental commands

`hw-search x` lists commands that are research tools rather than stable interfaces: `pair` and `progress`
(fixed to Obstacle Course), `ceiling`, `cannon-rand`, `route`, `profile`, `sensitivity`, `joints`, `finish`,
`cost`, `events`, `fc`. Their flags may change.

## Interpreting a run

`stats.jsonl` gets one JSON object every ten seconds with the command's counters: elapsed time, simulated
frames per second, trials, population sizes, launches, finishes and the best score so far (the exact keys differ
by command, for example `best_score` for `qe` and `best_miss_m` for `run`). `launches.jsonl` records each distinct
launch: frame, distance moved, where the chest went, and the input. A winner is written as soon as it is
confirmed; the search keeps running unless the command is documented to stop (`run` stops at the first
confirmed winner). The exit code is 0 after a confirmed winner and 1 when the time runs out without one.

Scores are command-specific and are not comparable between commands. A rising best score without a winner
means the lineage is getting closer, not that a finish is near.

## Reproducibility

Searches are seeded (`--seed`), but worker threads race, so two runs with the same seed explore in a different
order. Winners do not depend on that: a `.tas` replays identically with `hw sim` every time.
