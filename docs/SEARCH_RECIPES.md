# Search recipes

Copy-paste command lines for the common jobs, using inputs shipped in `reference/tas/`. All of them run
from the repository root in PowerShell after building (see [GETTING_STARTED.md](GETTING_STARTED.md)).
Concepts and the full command list are in [SEARCH.md](SEARCH.md).

```powershell
$hw     = "target/i686-pc-windows-msvc/release/hw.exe"
$search = "target/i686-pc-windows-msvc/release/hw-search.exe"
```

(`Out-File -Encoding ascii` matters: Windows PowerShell 5.1's `>` writes UTF-16, which the TAS reader rejects.)

Run directories are plain folders; put them under `runs/`, which git ignores. Level ids: 3214 Gut Bus Extreme,
2433 A large Satan eats u. Character 4 is the motorcart.

How long searches take: the shipped results came from runs of hours on many threads. The time estimates below
are for the short checks, which show that a command works and what it writes, not that it will find a winner
in that time.

## 1. Replay a found run

```powershell
& $hw sim --level 3214 --char 4 --tas reference/tas/gbeqe-finish-f227-fp15.tas   # finish 227
& $hw sim --level 2433 --char 4 --tas reference/tas/satanqe-finish-f125-fp23.tas # finish 125
```

`hw sim` prints `simulated N frames; finish F` on the last line (stderr) and one JSON record per frame on
stdout. Add `--out file.jsonl` to keep the records.

Confirm that a string finishes from a one-frame chest launch, replayed from scratch:

```powershell
& $search confirm --level 3214 --char 4 --tas reference/tas/gbeqe-finish-f217.tas
# confirmed: finish frame 217, chest moved 250.5 m in that frame
```

## 2. Look at a launch

```powershell
& $search analyze --level 3214 --char 4 --tas reference/tas/gbeqe-finish-f217.tas --from 215 --to 217
```

One JSON line per frame with the chest's solved path (`chest_solve`, `chest_tois`), `displacement`,
`finish_checks`, `qe`, the held inputs and whether the frame threw (`site`).

## 3. Enumerate the inputs just before a launch (seconds)

`brute` tries every combination of inputs on the last `--depth` frames of a string that ends on its launch
frame. The shipped `gbeqe.tas` runs on past its launch at frame 220, so cut it first:

```powershell
$bits = (Get-Content -Raw reference/tas/gbeqe.tas).Trim()
New-Item -ItemType Directory -Force runs | Out-Null
Set-Content -Path runs/gbeqe-220.tas -Value $bits.Substring(0, 220 * 8) -NoNewline

& $search brute --level 3214 --char 4 --tas runs/gbeqe-220.tas --depth 2 --extra 4 --min-launch 30 --out runs/brute.jsonl
```

With the default `launch` alphabet (the 32 pose/grab combinations) this is 1,024 combinations and takes under a
second; at `--depth 3` it is 32,768 combinations in a few seconds. The summary line reports `launches`,
`distinct_launches` and `winner`. Every finish it finds is confirmed from scratch and written as a `.tas`.

## 4. Search for new launches from seed strings

`qe` evolves launch lineages from seeds that end on a launch frame:

```powershell
& $search qe --level 3214 --char 4 --run-dir runs/gbe-qe --seeds runs/gbeqe-220.tas --hours 8
```

It prints a statistics line every ten seconds and writes to `runs/gbe-qe/`: `stats.jsonl`, `launches.jsonl` (every
distinct launch), `pool.jsonl` (the live population, reusable as `--pool-file` or `--resume-pool-file`), and a
`best-*.tas` whenever a lineage improves. A confirmed finish is written as `winner-1.tas`. Create a file named
`STOP` in the run directory to end the search early. The exit code is 0 after a confirmed winner and 1 when
the time runs out without one.

A two-minute check (`--hours 0.03`) produced about nine `best-*.tas` files and no winner.

`run` explores grab and pose inputs from the same seed and stops at the first winner it can confirm:

```powershell
& $search run --level 3214 --char 4 --run-dir runs/gbe-run --seeds reference/tas/gbeqe.tas --region 15,55,30,75 --max-frame 330 --min-launch 60 --hours 8
```

In a seven-minute check this ended with a best miss of about 10 m and no winner; the results shipped here took
hours and several seeds, so expect to leave it running.

## 5. Make a found run easier to perform

Every key event that changes the simulation before the launch must land on exactly the right frame. Count them
and strip the unnecessary ones:

```powershell
& $search x cost --tas reference/tas/gbeqe-finish-f217.tas,reference/tas/gbeqe-finish-f227-fp15.tas
# cost 27, 42 events  and  cost 15, 19 events

& $search canon --level 3214 --char 4 --tas reference/tas/gbeqe-finish-f217.tas --limit 277 | Out-File -Encoding ascii runs/gbe-canon.tas
# the summary {"events":36,"finish":217,"frame_perfect":27} is printed to the console
```

`canon` takes seconds and keeps the finish frame; here it drops six press/release events but the frame-perfect
count stays at 27. `sparse` then searches for strings with fewer frame-perfect events, accepting a later finish
(`--limit`, the latest acceptable finish frame):

```powershell
& $search sparse --level 3214 --char 4 --tas runs/gbe-canon.tas --limit 227 --run-dir runs/gbe-sparse --hours 8
```

It writes each improved string to the run directory as a `.tas` with `finishes.jsonl` and `stats.jsonl`. The
shipped `gbeqe-finish-f227-fp15.tas` (15 frame-perfect events, finishing on frame 227 instead of 217) and
`satanqe-finish-f125-fp23.tas` (23 instead of 33, same frame) came from hours of this kind of search. A six-minute
run on 14 threads found no improvement over the 27-event string, so budget accordingly.

`tail` instead keeps the string up to a cut point and tries every low-input tail:

```powershell
& $search tail --level 3214 --char 4 --tas reference/tas/gbeqe-finish-f217.tas --cut 200 --window 20 --k 1 --limit 277 --run-dir runs/gbe-tail
```

(221 tails, about a second.) Raise `--k` and `--window` for a wider search; the number of tails grows quickly.

## 6. Checking a result against the game

`hw sim` and `confirm` are deterministic and bit-exact with the recorded game traces (`hw verify`), so a finish
in the port is a finish in the game for the same input. A new result can only be recorded from the game itself
with the Flash-side recording tooling, which is not part of this repository; how recorded traces are structured
is in [TRACE_FORMAT.md](TRACE_FORMAT.md).
