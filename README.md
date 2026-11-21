# Room-Split Push-Optimal Sokoban Solver: hallway rooms solved independently with A* over pushes, deadlock pruning and assignment heuristic — 1,000,000 on the practice set, 60/60 fresh drawn levels push-optimal

[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/efrendi111094-crypto/makefun-sokoban-solver-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/efrendi111094-crypto/makefun-sokoban-solver-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

A deterministic Sokoban solver in a single `main.ts` (Node 22, stdlib only, no
network, no clock reads). It answers all 5 instances of a set in one run,
well inside the 60-second wall budget, and every run writes byte-identical
answers.

## The key idea

A level is a hallway with one-door rooms off it, and a box pushed out of a
room can never be pushed back — so the rooms are independent problems and the
push-optimal total is the sum of the rooms' optima. The solver therefore:

1. **Splits the level into rooms** — the hallway is the player's row; each
   room is a connected component of floor off that row, with its own boxes,
   goals and door (the hallway cell the room opens onto).
2. **Solves each room push-optimally with A\* over pushes** — a state is the
   boxes plus the player's reachable region; the heuristic is the minimum
   assignment of boxes to goals over wall-only distances; search is pruned by
   static dead squares (corners, goal-less wall runs, cells cut off from every
   goal) and by a freeze check that drops boxes which can never move again.
   A room is accepted only when the player can still walk out of its door,
   so the next room stays reachable.
3. **Stitches the rooms together** with plain walks along the hallway, then
   replays the whole move string to verify every box ends on a goal before
   writing it out.

A level that does not split into rooms falls back to one A\* over the whole
level. Every search is cut off by a count of expanded nodes, never by the
clock, so two executions always agree.

## Results

- **Practice set: 1,000,000 / 1,000,000** — all 5 instances at 1,000 points,
  every room solved in exactly its push-optimal count (13, 16, 29, 47, 60
  pushes), in ~0.7 s for the whole set.
- **Stress test: 60/60 fresh levels push-optimal** — 12 extra sets drawn with
  the kit's own generator from new seeds, every instance solved in exactly
  its reference push-optimal count, each set in under 1.4 s.

## How to run

```sh
node main.ts
```

It reads `/input/<id>/level.txt` and writes `/output/<id>/solution.txt` for
every instance. For a local run:

```sh
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

## How this entry meets the task

- One program answers every instance of a set in one run, inside the wall
  budget, deterministically.
- Solutions are push-optimal, not just move-optimal: the search counts
  pushes, and the score pays full points only at the push-optimal count.
- Stdlib only, no network, no paid APIs, no clock — the sandbox runs it as
  `node /bot/main.ts`.

<sub>Entered on [make.fun](https://make.fun/?ref=tzrh51cg&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
