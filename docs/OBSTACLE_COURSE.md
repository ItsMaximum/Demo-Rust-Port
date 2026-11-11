# Obstacle Course (level 45) with Irresponsible Dad

Research notes and tools for one unsolved target: a QE or pairmax setup that finishes Obstacle Course
(level 45) with Irresponsible Dad (character 3) in under 10 seconds, that is, by frame 299.

**Status: no finish by frame 299 has been found.** The mechanics below are measured and the
search tooling reproduces them; the notes record what each route needs and where it falls short.
The tool is `crates/obby-search`; it is fixed to this level and character (use
[`hw-search`](SEARCH.md) for other levels).

## Building and running

`obby-search` is a member of the main workspace (see [GETTING_STARTED.md](GETTING_STARTED.md) for the toolchain):

```powershell
cargo build --release -p obby-search
target/i686-pc-windows-msvc/release/obby-search.exe help
```

Run directories (`--run-dir`) are plain folders; create them under `runs/`, which is git-ignored.
Seed files are JSON lines with a `tas` field of 0/1 bits (8 per frame). Starting points shipped in
`reference/tas/obstacle-course/` include `obstacle-dad-*.tas` and `obstacle-pairmax-f274.tas`.

Searches that score open-sky states use flight maps: the output of `flight`, which teleports Dad's
jointed component into open sky with a sampled velocity and plays the unchanged engine to frame 299.
Pass them with `--flight-map file:base_frame,file:base_frame`, where the base frame is the frame the
teleports started from.

## What the pair-pool lock does (measured)

- **Where it throws.** A pairmax throws inside `b2World.Solve`'s synchronize loop at the first body (in body-list order, newest first) whose `MoveProxy` needs a new pair once all 4096 pairs are allocated.
- **What gets skipped.** On that frame:
  - bodies after the throwing body keep stale proxies;
  - there is no `Commit`, `SolveTOI` or character post-step;
  - so there are no joint breaks, death checks or bleed-out ticks.
- **Sustained lock.**
  - While the newest body keeps crossing into unpaired overlaps, every frame throws.
  - Dad then moves through all geometry. Gravity, his joints and his existing contacts still act on him, and he cannot die.
  - In practice the newest body is often a torn-ligament anchor: `tear_ligament` reassigns `body1` without relinking the edge list, which is the QE.
- **How a lock ends.**
  - A lock ends on the first frame whose synchronize completes.
  - That usually happens when a body before the throw point leaves the world AABB. It is frozen, and `DestroyProxy` commits, which frees the buffered removals.
  - Releasing Space can sometimes end a lock early.
- **Corrupted bounds.** A throw in the middle of `MoveProxy` can leave one bound out of order. The next move repairs it.
- **Onset kicks.**
  - An onset kick moves Dad's head with his chest (displacement ratio about 1.0).
  - Kicks are bimodal: either under 2 m, or 50 m to several km.
  - The neck re-closes during a lock of about 3 or more frames.
- **Neck limit.** The neck break limit is 85 and the head mass is 0.055, so a single-frame head yank above about 50 m/s breaks it.

## Torn ligaments, QE ignition and the finish check (measured)

- **A tear makes a free chain.** After a distal break the ligament joins the upper and lower limb. A tear moves the first end joint onto a new 7 g anchor.
  - The QE chain is then anchor, node 0, node 1 and the lower limb. It is not jointed to the character any more.
  - It still sits in the upper limb's island through the stale joint edge.
- **Ignition needs separation.** While the anchor touches node 0 (radius sum 0.16 m), both are solved in one island and the tear stays dormant (anchor speed 1–100 m/s).
  - Once they separate by metres, the joint is solved against an anchor outside its island and the speed grows by about an order of magnitude every few frames (1e3 to 1e7 m/s).
  - Mine blasts and the boost shaft both produce that separation.
- **Whipping limbs push Dad.** Ligament nodes and anchors collide only with category 8. The whipping lower limbs (the child's forearms) do touch Dad's body parts, and push him at tens to hundreds of m/s.
- **The child's death is the usual onset.** When the child's torso breaks during a QE, its intestine nodes inherit the whipped chest velocity (about 1e4 m/s or more) and span most of the world for a frame.
  - That alone reaches 3500–4096 pairs. The corridor family pairmaxes at frame 178; the start families at 115–162.
- **Holding Space extends a lock.** With Dad ejected, Space (grab) keeps the pool exhausted for a few more frames (4 instead of 1 at the frame-115 onset). A lock held by a newest whipping body lasts 7–40 frames.
- **The spike ceiling can be passed.** At onsets around frames 158–162, Dad is wedged at the start's left wall and released up-right at 25–35 m/s.
  - The lock holds 7–19 frames and he comes out alive above the spike ceiling.
  - So far his velocity there is too weak (about (20, -11) m/s) for an arc to the finish.
  - That family is also dying: his torso broke at frame 92, so he bleeds out at about 242.
- **The finish check ignores which body touches the plate.** `FinishLine` fires on any RESULT on shape h120 and only checks that Dad's chest (the focus) is inside x 117.3–123.7, y 55.5–58.7 and that he is not dead. Dying is fine.
- **Port note.** In some exotic lock states the port panics in `edge_separation` (edge index -1 from NaN separations). The workspace builds with `panic = "unwind"`, so the tool can catch these per trial.

## What finishes from open sky

`flight` teleports Dad's jointed component into open sky with a sampled velocity, then plays the unchanged engine to frame 299. It is an experiment, not an input.

- **From just above the spike ceiling** (y about 64, base frame 105): velocities of about 30 to 38 m/s at 35 to 62 degrees finish. That arc carries Dad over the whole level into the finish pocket, landing between frames 224 and 293. About 1% of random open-sky states finish.
- **From above the corridor ceiling at frame 180:** about 31 m/s at about 40 degrees finishes by frames 289 to 299.
- **From the corridor around frame 252:** nothing finishes.
- **The finish pocket.** The finish plate (shape h120) sits in a pocket. Static wall h76 (x 123.7 to 127.1) rises to y 47.75 behind it.

## Families

### Springbox platform (onset around frame 249)

- The kicks throw Dad up and left, out of the world.
- The lock is held by the torn-ligament anchor created at frame 248. That anchor leaves the world at about frame 305.
- Post-onset inputs cannot shorten it (`lockctl`).
- No drift line passes through in-world open sky in a way that finishes (`driftcheck`, 2578 lineages).

### Start area (launch around frame 84)

- The child's QE tears both characters apart. Dad's waist breaks, so he is dying (bleed-out after 150 frames).
- His chest and head are launched up and right at 35 to 45 m/s toward the spike ceiling.
- At the same time the debris cluster (about 66 mutually overlapping bodies) peaks at about 2860 pairs in-frame.
- **On an experimental copy with a 2048-pair pool:**
  - That spike locks for about 40 frames.
  - Dad passes through the spike ceiling alive and arcs over the level.
  - He lands on the fan housing (x about 100) at frame 233, then bleeds out.
- This is the route that would let the chest pass the spike ceiling at the start.
- **What is missing:**
  - about 4096 pairs at the launch in the real engine;
  - a slightly faster and flatter arc that lands before the bleed-out.

## Commands

`obby-search help` lists every command with a one-line summary. The main groups:

| Group | Commands |
| --- | --- |
| Inspect a TAS | `stats`, `trace`, `pairmap`, `info`, `geom`, `qescan`, `lstate` |
| Find QE launches | `cqe`, `mlaunch`, `explore` |
| Search onsets (the first pair-pool overflow) | `window --objective ...`, `onset`, `onarch [--pass]`, `hold` |
| Tune and climb | `tune`, `climb`, `climb2` |
| Open-sky experiments | `flight`, `branch`, `driftcheck`, `blast`, `inject` |
| Onset statistics | `aftermath`, `release`, `lockctl`, `sens`, `launches`, `nearceil` |

`window --objective` selects what a search maximises: `early` (earliest onset), `earlykick` (onsets that
kick Dad), `fast`, `upr`, `upc2`, `upx` (onsets with Dad rising up-right at launch speed), `ostate` and
`osky3` (ballistic or held-lock path scored against open-sky finishing states), `aft` (held-Space
aftermath), `pass` and `passfin` (passing the spike ceiling, then the closest finish approach).

`trace` prints more with environment flags: `OBBY_TRACE_THROW` names the throwing body,
`OBBY_TRACE_BODIES`, `OBBY_TRACE_HEAD=N` and `OBBY_TRACE_COMP` show bodies, composites and broken joints,
`OBBY_TRACE_QE` shows torn-ligament anchors, `OBBY_TRACE_CONTACTS` the contacts on Dad and
`OBBY_TRACE_MINES` the start mines. `OBBY_QUIET=1` suppresses per-trial event logging in `window`.
