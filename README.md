# Sokoban Solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/UltraViewer/sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/UltraViewer/sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver for the make.fun bounty
"Build the best Sokoban solver, 21 November 2026"
(sokoban-solver-01-run-20261121).

## How it works

A level is a 1-wide hallway plus 1–4 rooms. A box pushed into the
hallway can never come back, so each room is an independent sub-puzzle:
its boxes stay in the room and end on goals inside it.

1. **Decompose.** Find the rooms with articulation points of the floor
   graph: a room is a component cut off by one door cell, balanced
   (boxes == goals), picked smallest-first so the boxes partition.
2. **Solve each room push-optimally** with A* over pushes
   (player-reachability BFS per push, Hungarian box-to-goal heuristic
   on shortest-path distances, dead-square table from reverse pull
   reachability, 2×2 freeze pruning, and a transposition table keyed by
   boxes plus the player's walking region).
3. **Stitch.** Shortest hallway walks carry the player between rooms;
   the door box, if any, is pushed in first (forced). The last room
   needs no exit walk.

Pushes add up to the level optimum, because the room optima are
independent. Output is the full move string (`u/d/l/r`, uppercase
marks a push), deterministic across runs (fixed tie-breaking,
expansion-count budgets only).

## Practice score (sealed scorer, `grade-local`)

| instance | pushes | optimal | score |
| --- | --- | --- | --- |
| i001 | 13 | 13 | 1000 |
| i002 | 16 | 16 | 1000 |
| i003 | 29 | 29 | 1000 |
| i004 | 47 | 47 | 1000 |
| i005 | 60 | 60 | 1000 |
| **total** | **165** | **165** | **5000 / 5000** |

House baseline (plain-astar 1.0.0) scores 1000/5000 on the same set
(only i001 solved). Runtime on the practice set: ~0.5 s of the 60 s
budget. Two runs write identical bytes.

## Run

```sh
MAKEFUN_INPUT_DIR=<input> MAKEFUN_OUTPUT_DIR=<output> node main.ts
```

Reads `<input>/<id>/level.txt`, writes `<output>/<id>/solution.txt`.
Node 22+ (type stripping), no dependencies, no network.

---

<sub>Entered on [make.fun](https://make.fun/?ref=crwhvsxm&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
