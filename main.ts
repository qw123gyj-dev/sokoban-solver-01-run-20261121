// Sokoban solver for make.fun sokoban-solver-01-run-20261121.
//
// Approach: the level is a hallway with 1-4 independent rooms (boxes can never
// leave their room: a push through a door lands in the hallway, from which no
// goal is reachable). Each room is solved push-optimally on its own and the
// moves are concatenated, the player walking through the hallway between rooms.
// Per room, an A* over pushes: a state is the boxes' squares plus the player's
// reachable region (named by its lowest square), the cost is pushes, and the
// heuristic is the min-cost assignment of boxes to goals under shortest-path
// (walls only) distances -- admissible, so the first goal state popped is
// push-optimal. Deadlock pruning: dead squares (reverse pull from the goals)
// and frozen 2x2 blocks, both of which the generator's own reference solver
// uses. A push-BFS (the reference algorithm, ported) is the fallback if A*
// ever exhausts its expansion budget. All budgets are counted in expansions,
// never wall-clock time, so two runs write identical bytes.

import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Expansion budgets (work, never the clock). The generator keeps a room only
// when its own push-BFS solves it within 250,000 states, and A* with an
// admissible heuristic expands no state that BFS would not also keep, so a
// 250,000-expansion cap per room is a guarantee, not a hope. The set-wide cap
// keeps the whole run far inside the 60 s wall budget.
const PER_ROOM_EXPANSIONS = 250_000;
const TOTAL_EXPANSIONS = 2_000_000;

type Level = {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
};

type Budget = { used: number; limit: number };

const DIRS: Array<[number, number, string, string]> = [
  [0, -1, 'u', 'U'],
  [0, 1, 'd', 'D'],
  [-1, 0, 'l', 'L'],
  [1, 0, 'r', 'R'],
];

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  return { w, h, wall, goals, boxes: boxes.sort((a, b) => a - b), player };
}

/** Squares from which a box alone can be pushed onto some goal (pulled back from the goals over open floor). */
function liveSquares(level: Level): Uint8Array {
  const { w, wall, goals } = level;
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi];
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

/** Flood fill from `from` with `occupied` in the way; marks `seen` with `stamp`, returns the lowest square reached. */
function reach(level: Level, occupied: Uint8Array, from: number, seen: Int32Array, stamp: number, stack: Int32Array): number {
  const { w, wall } = level;
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n];
    if (p < low) low = p;
    for (let k = 0; k < 4; k++) {
      const q = k === 0 ? p - w : k === 1 ? p + w : k === 2 ? p - 1 : p + 1;
      if (wall[q] === 0 && occupied[q] === 0 && seen[q] !== stamp) {
        seen[q] = stamp;
        stack[n++] = q;
      }
    }
  }
  return low;
}

/** Whether the box just pushed to `t` sits in a frozen 2x2 block of walls and boxes with a box off its goal. */
function frozenBlock(level: Level, occupied: Uint8Array, goal: Uint8Array, t: number): boolean {
  const { w, wall } = level;
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b]) || !(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b]) || (occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) return true;
  }
  return false;
}

/** Shortest-path distances (walls only, boxes ignored) from every square to every goal: dist[s * nGoals + gi]. */
function goalDistances(level: Level): Int32Array {
  const { w, wall, goals } = level;
  const n = goals.length;
  const size = wall.length;
  const dist = new Int32Array(size * n).fill(-1);
  const q = new Int32Array(size);
  for (let gi = 0; gi < n; gi++) {
    let qh = 0;
    let qt = 0;
    const g = goals[gi];
    dist[g * n + gi] = 0;
    q[qt++] = g;
    while (qh < qt) {
      const p = q[qh++];
      const d = dist[p * n + gi] + 1;
      const nb = [p - w, p + w, p - 1, p + 1];
      for (const t of nb) {
        if (!wall[t] && dist[t * n + gi] < 0) {
          dist[t * n + gi] = d;
          q[qt++] = t;
        }
      }
    }
  }
  return dist;
}

/** Min-cost assignment of boxes to goals under `dist` (branch and bound over goal permutations). */
function matchRec(boxes: number[], dist: Int32Array, nGoals: number, bi: number, sum: number, mask: number, best: number): number {
  if (sum >= best) return best;
  if (bi === boxes.length) return sum < best ? sum : best;
  const base = boxes[bi] * nGoals;
  for (let gi = 0; gi < nGoals; gi++) {
    if (mask & (1 << gi)) continue;
    const d = dist[base + gi];
    if (d < 0) continue;
    best = matchRec(boxes, dist, nGoals, bi + 1, sum + d, mask | (1 << gi), best);
  }
  return best;
}

/**
 * Admissible heuristic: each box needs at least its shortest-path distance to
 * its assigned goal in pushes, so the min-cost assignment lower-bounds the
 * pushes still needed. Infinity when no assignment exists.
 */
function heuristic(boxes: number[], dist: Int32Array, nGoals: number): number {
  if (boxes.length <= 5 && nGoals <= 20) return matchRec(boxes, dist, nGoals, 0, 0, 0, Infinity);
  let sum = 0;
  for (const b of boxes) {
    let m = Infinity;
    for (let gi = 0; gi < nGoals; gi++) {
      const d = dist[b * nGoals + gi];
      if (d >= 0 && d < m) m = d;
    }
    if (m === Infinity) return Infinity;
    sum += m;
  }
  return sum;
}

/** The shortest walk from `from` to `to` around walls and boxes, in lower LURD letters (first direction first on ties). */
function walkTo(level: Level, occupied: Uint8Array, from: number, to: number): string {
  const { w, wall } = level;
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array<string>(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: the push square is out of reach');
  let s = '';
  for (let p = to; p !== from; p = prev[p]) s = how[p] + s;
  return s;
}

/** The square the player stands on after `moves` (LURD, case aside) from `from` on a board `w` wide. */
function endOf(w: number, from: number, moves: string): number {
  let p = from;
  for (const ch of moves.toLowerCase()) p += ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
  return p;
}

type Solution = { pushes: number; moves: string };

/**
 * Push-optimal A* for one (sub-)level. Returns null when the expansion budget
 * runs out or the level has no solution. Deterministic: heap ties break by
 * deeper g first, then by insertion order.
 */
function solveAStar(level: Level, maxExp: number, budget: Budget): Solution | null {
  const { w, wall, goals } = level;
  const size = wall.length;
  const n = level.boxes.length;
  const nGoals = goals.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  if (level.boxes.some((b) => !live[b])) return null;
  const dist = goalDistances(level);
  const solved = (boxes: number[]): boolean => boxes.every((b) => goal[b] === 1);
  if (solved(level.boxes)) return { pushes: 0, moves: '' };

  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  const occupied = new Uint8Array(size);
  let stamp = 0;

  // Numeric keys stay exact while size ** (n + 1) < 2 ** 53; beyond that use
  // a string key (only reachable on a whole-level fallback with many boxes).
  const packable = Math.pow(size, n + 1) < 2 ** 53;
  const keyOf = (boxes: number[], low: number): number | string => {
    if (!packable) return `${low}|${boxes.join(',')}`;
    let k = low;
    for (const b of boxes) k = k * size + b;
    return k;
  };

  // State storage: boxes, player square, parent index, state key.
  const sBoxes: number[][] = [];
  const sPlayer: number[] = [];
  const sParent: number[] = [];
  const sKey: Array<number | string> = [];
  const best = new Map<number | string, number>();

  for (const b of level.boxes) occupied[b] = 1;
  const low0 = reach(level, occupied, level.player, seen, ++stamp, stack);
  for (const b of level.boxes) occupied[b] = 0;
  const h0 = heuristic(level.boxes, dist, nGoals);
  if (h0 === Infinity) return null;
  sBoxes.push(level.boxes.slice());
  sPlayer.push(level.player);
  sParent.push(-1);
  sKey.push(keyOf(level.boxes, low0));
  best.set(sKey[0], 0);

  // Binary heap on [f, -g, seq, state]: lowest f first, deeper g first on ties, then insertion order.
  const heap: Array<[number, number, number, number]> = [[h0, 0, 0, 0]];
  let seq = 1;
  const less = (a: [number, number, number, number], b: [number, number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      const t = heap[i]!;
      heap[i] = heap[p]!;
      heap[p] = t;
      i = p;
    }
  };
  const heapPop = (): [number, number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        const t = heap[i]!;
        heap[i] = heap[c]!;
        heap[c] = t;
        i = c;
      }
    }
    return top;
  };

  const cur: number[] = new Array<number>(n);
  const next: number[] = new Array<number>(n);
  let expanded = 0;
  let found = -1;
  while (heap.length > 0) {
    const [, negG, , s] = heapPop();
    const g = -negG;
    if (best.get(sKey[s]) !== g) continue; // stale heap entry
    const stateBoxes = sBoxes[s]!;
    for (let j = 0; j < n; j++) cur[j] = stateBoxes[j];
    if (solved(cur)) {
      found = s;
      break;
    }
    if (expanded >= maxExp || budget.used >= budget.limit) return null;
    expanded++;
    budget.used++;
    for (const b of cur) occupied[b] = 1;
    reach(level, occupied, sPlayer[s], seen, ++stamp, stack);
    const here = stamp;
    for (let bi = 0; bi < n; bi++) {
      const b = cur[bi];
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const t = b + d;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(level, occupied, goal, t);
        if (!frozen) {
          for (let j = 0; j < n; j++) next[j] = cur[j];
          next[bi] = t;
          next.sort((x, y) => x - y);
          const low = reach(level, occupied, b, mark, ++stamp, stack);
          const key = keyOf(next, low);
          const g2 = g + 1;
          const prevBest = best.get(key);
          if (prevBest === undefined || g2 < prevBest) {
            const h2 = heuristic(next, dist, nGoals);
            if (h2 !== Infinity) {
              best.set(key, g2);
              const idx = sBoxes.length;
              sBoxes.push(next.slice());
              sPlayer.push(b);
              sParent.push(s);
              sKey.push(key);
              heapPush([g2 + h2, -g2, seq++, idx]);
            }
          }
        }
        occupied[t] = 0;
        occupied[b] = 1;
      }
    }
    for (const b of cur) occupied[b] = 0;
  }
  if (found < 0) return null;

  // Reconstruct: walk to each push's square, then push.
  const path: number[] = [];
  for (let s = found; s > 0; s = sParent[s]) path.push(s);
  path.reverse();
  let moves = '';
  let player = level.player;
  const boxes = level.boxes.slice();
  for (const s of path) {
    const nb = sBoxes[s]!;
    const from = boxes.find((b) => !nb.includes(b))!;
    const to = nb.find((b) => !boxes.includes(b))!;
    const d = to - from;
    const dir = DIRS.find(([dx, dy]) => dy * w + dx === d)!;
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(level, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    moves += dir[3];
    player = from;
    for (let j = 0; j < n; j++) boxes[j] = nb[j];
  }
  return { pushes: path.length, moves };
}

/**
 * Push-BFS (the generator's reference search, ported): expands states in
 * order of pushes made, so the first solved state met is push-optimal.
 * Fallback when A* exhausts its budget.
 */
function solveBFS(level: Level, maxStates: number, budget: Budget): Solution | null {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;
  const solved = (boxes: number[]): boolean => boxes.every((b) => goal[b] === 1);
  if (level.boxes.some((b) => !live[b])) return null;
  if (solved(level.boxes)) return { pushes: 0, moves: '' };

  const keyOf = (boxes: number[], low: number): number | string => {
    if (Math.pow(size, boxes.length + 1) >= 2 ** 53) return `${low}|${boxes.join(',')}`;
    let k = low;
    for (const b of boxes) k = k * size + b;
    return k;
  };
  const statesBoxes: number[][] = [level.boxes];
  const statesPlayer: number[] = [level.player];
  const parent: number[] = [-1];
  const index = new Map<number | string, number>();
  for (const b of level.boxes) occupied[b] = 1;
  index.set(keyOf(level.boxes, reach(level, occupied, level.player, seen, ++stamp, stack), size), 0);
  for (const b of level.boxes) occupied[b] = 0;
  let found = -1;
  for (let head = 0; found < 0 && head < statesBoxes.length; head++) {
    const boxes = statesBoxes[head]!;
    if (budget.used >= budget.limit) return null;
    budget.used++;
    for (const b of boxes) occupied[b] = 1;
    reach(level, occupied, statesPlayer[head], seen, ++stamp, stack);
    const here = stamp;
    for (let bi = 0; bi < boxes.length && found < 0; bi++) {
      const b = boxes[bi];
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const t = b + d;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(level, occupied, goal, t);
        let key: number | string = -1;
        let next: number[] | null = null;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((x, y) => x - y);
          key = keyOf(next, reach(level, occupied, b, mark, ++stamp, stack));
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || index.has(key)) continue;
        if (statesBoxes.length >= maxStates) {
          for (const x of boxes) occupied[x] = 0;
          return null;
        }
        index.set(key, statesBoxes.length);
        statesBoxes.push(next!);
        statesPlayer.push(b);
        parent.push(head);
        if (solved(next!)) {
          found = statesBoxes.length - 1;
          break;
        }
      }
    }
    for (const x of boxes) occupied[x] = 0;
  }
  if (found < 0) return null;
  const path: number[] = [];
  for (let s = found; s > 0; s = parent[s]) path.push(s);
  path.reverse();
  let moves = '';
  let player = level.player;
  let boxes = level.boxes;
  for (const s of path) {
    const nextBoxes = statesBoxes[s]!;
    const from = boxes.find((b) => !nextBoxes.includes(b))!;
    const to = nextBoxes.find((b) => !boxes.includes(b))!;
    const d = to - from;
    const dir = DIRS.find(([dx, dy]) => dy * w + dx === d)!;
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(level, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    moves += dir[3];
    player = from;
    boxes = nextBoxes;
  }
  return { pushes: path.length, moves };
}

type Room = { squares: number[]; boxes: number[]; goals: number[]; done: boolean };

/**
 * The level's rooms: cut the dead squares of the player's row (the hallway: a
 * box there reaches no goal) out of the floor; each part left holding a box is
 * a room. Null when the level is no set of rooms.
 */
function roomsOf(level: Level): Room[] | null {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(level);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !live[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms: Room[] = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (const [dx, dy] of DIRS) {
        const q = squares[i] + dy * w + dx;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ squares, boxes: [], goals: [], done: false });
  }
  for (const b of boxes) rooms[part[b]]!.boxes.push(b);
  for (const g of goals) if (part[g] >= 0) rooms[part[g]]!.goals.push(g);
  if (goals.some((g) => part[g] < 0) || rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

/** Solve one room (A*, falling back to BFS), or null. */
function solveRoom(level: Level, budget: Budget): Solution | null {
  const a = solveAStar(level, PER_ROOM_EXPANSIONS, budget);
  if (a !== null) return a;
  return solveBFS(level, PER_ROOM_EXPANSIONS, budget);
}

/** Solve a level room by room; falls back to solving the whole level at once. Returns null when any part fails. */
function solveLevel(level: Level, budget: Budget): Solution | null {
  const rooms = roomsOf(level);
  if (rooms === null) return solveRoom(level, budget);
  let player = level.player;
  let pushes = 0;
  let moves = '';
  for (const room of rooms) {
    const wall = level.wall.slice();
    for (const other of rooms) {
      if (other === room) continue;
      for (const b of other.done ? other.goals : other.boxes) wall[b] = 1;
    }
    const sub: Level = {
      w: level.w,
      h: level.h,
      wall,
      goals: room.goals.slice().sort((a, b) => a - b),
      boxes: room.boxes.slice().sort((a, b) => a - b),
      player,
    };
    const solved = solveRoom(sub, budget);
    if (solved === null) return null;
    pushes += solved.pushes;
    moves += solved.moves;
    player = endOf(level.w, player, solved.moves);
    room.done = true;
  }
  return { pushes, moves };
}

const budget: Budget = { used: 0, limit: TOTAL_EXPANSIONS };
for (const id of readdirSync(I).sort()) {
  let moves = '';
  try {
    const level = parseLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
    const solved = solveLevel(level, budget);
    if (solved !== null) moves = solved.moves;
  } catch {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
