[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/qw123gyj-dev/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/qw123gyj-dev/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

# Sokoban solver

Push-optimal Sokoban solver for make.fun's `sokoban-solver-01-run-20261121`.
`node main.ts` reads each instance at `/input/<id>/level.txt` (XSB) and writes
`/output/<id>/solution.txt` (moves `u/d/l/r`, uppercase letters mark pushes).

## Approach

The levels are a 1-wide hallway with 1–4 rooms off it, and a box pushed out
of a room through its door can never be pushed back — so the rooms are
independent. The solver cuts the hallway's dead squares out of the floor,
splits the level into rooms, solves each room push-optimally on its own, and
concatenates the moves, walking the player through the hallway between rooms.
The level's push-optimum is the sum of its rooms' optima.

Per room, an A* over pushes:

- a state is the boxes' squares plus the player's reachable region (named by
  its lowest square); the cost is pushes;
- the heuristic is the min-cost assignment of boxes to goals under
  shortest-path (walls only) distances — admissible, so the first goal state
  popped is push-optimal;
- deadlock pruning: dead squares (found by pulling boxes back from the goals)
  and frozen 2×2 blocks of walls and boxes;
- a push-BFS (the same search the competition's generator uses to certify its
  rooms) is the fallback if A* ever exhausts its expansion budget.

All budgets are counted in expansions, never wall-clock time, so two runs
write identical bytes. Deterministic tie-breaking throughout (deeper `g`
first, then insertion order).

## Results

Practice set: **1,000,000 / 1,000,000** — every instance solved in exactly
its push-optimal count P* (13, 16, 29, 47, 60 pushes), versus the house
baseline plain-astar's 200,000. On 50 freshly drawn sets (250 instances) from
the competition's own generator, every instance was solved at exactly P*.
A whole practice set runs in ~0.6 s, far inside the 60 s wall budget.
