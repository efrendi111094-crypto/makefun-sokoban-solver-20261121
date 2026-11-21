// Sokoban solver for sokoban-solver-01-run-20261121.
//
// Idea: a level is a hallway with one-door rooms off it, and a box pushed out
// of a room can never come back, so every room is solved on its own and the
// push-optimal total is the sum of the rooms' optima. Each room (3-4 boxes in
// a 6x6 slot) is solved push-optimally with A* over pushes -- a state is the
// boxes plus the player's reachable region -- with an assignment heuristic
// (minimum matching of boxes to goals over wall-only distances), static dead
// squares (corners, wall runs with no goal, cells cut off from every goal)
// and a freeze check that prunes boxes which can never move again. The room
// solutions are stitched together with plain walks along the hallway.
//
// Determinism: every search is cut off by a count of expanded nodes, never
// by the wall clock, so two runs write byte-identical answers. No randomness,
// no network, no environment reads beyond the input/output directories.
//
// Written fresh for this competition. Node 22, stdlib only, erasable-syntax
// TypeScript (plain JavaScript with type annotations).
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Node budgets are counts of expanded states, so the run is reproducible.
const ROOM_NODE_CAP = 400_000;
const TOTAL_NODE_CAP = 2_000_000;

type Dir = { dx: number; dy: number; ch: string };
const DIRS: Dir[] = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];

type Grid = {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
};

function parseLevel(text: string): Grid {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = 0;
  for (let y = 0; y < h; y++) {
    const row = rows[y] ?? '';
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      if (ch === '#') wall[i] = 1;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  }
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

function idx(w: number, x: number, y: number): number {
  return y * w + x;
}

function inBounds(w: number, h: number, x: number, y: number): boolean {
  return x >= 0 && y >= 0 && x < w && y < h;
}

// BFS distances from a set of start cells over passable cells.
function bfsDist(
  w: number,
  h: number,
  passable: Uint8Array,
  starts: number[],
): Int32Array {
  const dist = new Int32Array(w * h).fill(-1);
  const q: number[] = [];
  for (const s of starts) {
    if (passable[s] && dist[s] < 0) {
      dist[s] = 0;
      q.push(s);
    }
  }
  for (let head = 0; head < q.length; head++) {
    const c = q[head] as number;
    const cx = c % w;
    const cy = (c / w) | 0;
    const nd = dist[c] as number;
    for (const d of DIRS) {
      const nx = cx + d.dx;
      const ny = cy + d.dy;
      if (!inBounds(w, h, nx, ny)) continue;
      const n = idx(w, nx, ny);
      if (!passable[n] || dist[n] >= 0) continue;
      dist[n] = nd + 1;
      q.push(n);
    }
  }
  return dist;
}

// Flood fill from `start` over passable cells that are not blocked.
// Returns the reach mask and the smallest reachable cell index (the
// canonical representative of the player's region).
function flood(
  w: number,
  h: number,
  passable: Uint8Array,
  blocked: Uint8Array,
  start: number,
): { reach: Uint8Array; min: number } {
  const reach = new Uint8Array(w * h);
  if (!passable[start] || blocked[start]) return { reach, min: -1 };
  const q: number[] = [start];
  reach[start] = 1;
  let min = start;
  for (let head = 0; head < q.length; head++) {
    const c = q[head] as number;
    if (c < min) min = c;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (const d of DIRS) {
      const nx = cx + d.dx;
      const ny = cy + d.dy;
      if (!inBounds(w, h, nx, ny)) continue;
      const n = idx(w, nx, ny);
      if (!passable[n] || blocked[n] || reach[n]) continue;
      reach[n] = 1;
      q.push(n);
    }
  }
  return { reach, min };
}

// Shortest walk (no pushes) from `from` to `to` over passable cells that are
// not blocked; returns the move string, or null when unreachable.
function walkPath(
  w: number,
  h: number,
  passable: Uint8Array,
  blocked: Uint8Array,
  from: number,
  to: number,
): string | null {
  if (from === to) return '';
  const prev = new Int32Array(w * h).fill(-1);
  const prevCh = new Array<string>(w * h).fill('');
  const q: number[] = [from];
  prev[from] = from;
  for (let head = 0; head < q.length; head++) {
    const c = q[head] as number;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (const d of DIRS) {
      const nx = cx + d.dx;
      const ny = cy + d.dy;
      if (!inBounds(w, h, nx, ny)) continue;
      const n = idx(w, nx, ny);
      if (!passable[n] || blocked[n] || prev[n] >= 0) continue;
      prev[n] = c;
      prevCh[n] = d.ch;
      if (n === to) {
        let s = '';
        let cur = n;
        while (cur !== from) {
          s = (prevCh[cur] as string) + s;
          cur = prev[cur] as number;
        }
        return s;
      }
      q.push(n);
    }
  }
  return null;
}

// All permutations of 0..n-1, for the assignment heuristic.
function permutations(n: number): number[][] {
  const out: number[][] = [];
  const a: number[] = [];
  for (let i = 0; i < n; i++) a.push(i);
  const gen = (k: number): void => {
    if (k === n) {
      out.push(a.slice());
      return;
    }
    for (let i = k; i < n; i++) {
      const t = a[k] as number;
      a[k] = a[i] as number;
      a[i] = t;
      gen(k + 1);
      const t2 = a[k] as number;
      a[k] = a[i] as number;
      a[i] = t2;
    }
  };
  gen(0);
  return out;
}

// Static dead squares: floor cells a box can never leave towards a goal.
// A cell is dead when it is not a goal and (a) no goal is reachable from it,
// (b) it is a corner, or (c) it lies on a wall run that holds no goal and is
// walled off at both ends. Only truly dead cells are marked, so pruning them
// never costs an optimal solution.
function staticDead(
  w: number,
  h: number,
  passable: Uint8Array,
  goals: number[],
  goalDists: Int32Array[],
): Uint8Array {
  const dead = new Uint8Array(w * h);
  const goalSet = new Set(goals);
  const wallAt = (x: number, y: number): boolean =>
    !inBounds(w, h, x, y) || !passable[idx(w, x, y)];
  for (let i = 0; i < w * h; i++) {
    if (!passable[i] || goalSet.has(i)) continue;
    let reachable = false;
    for (const d of goalDists) {
      if (d[i] as number >= 0) {
        reachable = true;
        break;
      }
    }
    if (!reachable) {
      dead[i] = 1;
      continue;
    }
    const x = i % w;
    const y = (i / w) | 0;
    const up = wallAt(x, y - 1);
    const dn = wallAt(x, y + 1);
    const lf = wallAt(x - 1, y);
    const rt = wallAt(x + 1, y);
    if ((up || dn) && (lf || rt)) dead[i] = 1; // corner
  }
  // Wall runs: a maximal straight run of floor cells hugging one wall, with
  // no goal on it and walled off at both ends, is dead along its length.
  const markRun = (
    cells: number[],
    bounded: boolean,
  ): void => {
    // A run holding a goal is alive: a box can slide along it onto the goal.
    if (!bounded || cells.length === 0) return;
    if (cells.some((c) => goalSet.has(c))) return;
    for (const c of cells) dead[c] = 1;
  };
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; ) {
      if (!passable[idx(w, x, y)] || !wallAt(x, y - 1)) {
        x++;
        continue;
      }
      const cells: number[] = [];
      let bounded = wallAt(x - 1, y);
      while (
        x < w &&
        passable[idx(w, x, y)] &&
        wallAt(x, y - 1)
      ) {
        cells.push(idx(w, x, y));
        x++;
      }
      bounded = bounded && wallAt(x, y);
      markRun(cells, bounded);
    }
    for (let x = 0; x < w; ) {
      if (!passable[idx(w, x, y)] || !wallAt(x, y + 1)) {
        x++;
        continue;
      }
      const cells: number[] = [];
      let bounded = wallAt(x - 1, y);
      while (
        x < w &&
        passable[idx(w, x, y)] &&
        wallAt(x, y + 1)
      ) {
        cells.push(idx(w, x, y));
        x++;
      }
      bounded = bounded && wallAt(x, y);
      markRun(cells, bounded);
    }
  }
  for (let x = 0; x < w; x++) {
    for (let y = 0; y < h; ) {
      if (!passable[idx(w, x, y)] || !wallAt(x - 1, y)) {
        y++;
        continue;
      }
      const cells: number[] = [];
      let bounded = wallAt(x, y - 1);
      while (
        y < h &&
        passable[idx(w, x, y)] &&
        wallAt(x - 1, y)
      ) {
        cells.push(idx(w, x, y));
        y++;
      }
      bounded = bounded && wallAt(x, y);
      markRun(cells, bounded);
    }
    for (let y = 0; y < h; ) {
      if (!passable[idx(w, x, y)] || !wallAt(x + 1, y)) {
        y++;
        continue;
      }
      const cells: number[] = [];
      let bounded = wallAt(x, y - 1);
      while (
        y < h &&
        passable[idx(w, x, y)] &&
        wallAt(x + 1, y)
      ) {
        cells.push(idx(w, x, y));
        y++;
      }
      bounded = bounded && wallAt(x, y);
      markRun(cells, bounded);
    }
  }
  return dead;
}

// Freeze check: a box is frozen when every push direction is blocked by a
// wall or by another frozen box. A frozen box that is not on a goal can never
// move again, so the state is dead. The fixed point is computed iteratively.
function freezeDead(
  w: number,
  h: number,
  passable: Uint8Array,
  boxes: number[],
  boxAt: Int32Array,
  goalSet: Set<number>,
): boolean {
  const n = boxes.length;
  const frozen = new Uint8Array(n);
  let changed = true;
  while (changed) {
    changed = false;
    for (let i = 0; i < n; i++) {
      if (frozen[i]) continue;
      const b = boxes[i] as number;
      const x = b % w;
      const y = (b / w) | 0;
      let stuck = true;
      for (const d of DIRS) {
        const nx = x + d.dx;
        const ny = y + d.dy;
        if (!inBounds(w, h, nx, ny)) continue;
        const dest = idx(w, nx, ny);
        if (!passable[dest]) continue;
        const bi = boxAt[dest] as number;
        if (bi < 0 || !frozen[bi]) {
          stuck = false;
          break;
        }
      }
      if (stuck) {
        frozen[i] = 1;
        changed = true;
      }
    }
  }
  for (let i = 0; i < n; i++) {
    if (frozen[i] && !goalSet.has(boxes[i] as number)) return true;
  }
  return false;
}

type Push = { box: number; dir: number };

// Push-optimal A* over one box arrangement. `passable` is the region the
// boxes may occupy; `entries` are the cells the player may start from (one
// search covers all of them). `needExit` asks that the final state still let
// the player reach one of `exitCells` (so the next room stays reachable);
// the last room of a level does not need it. Returns the push list, or null
// when the node budget runs out.
function astarPush(
  w: number,
  h: number,
  passable: Uint8Array,
  goals: number[],
  boxes0: number[],
  entries: number[],
  budget: { used: number; cap: number },
  roomCap: number,
  needExit: boolean,
  exitCells: number[],
): { pushes: Push[]; startEntry: number; expansions: number } | null {
  const n = boxes0.length;
  const goalSet = new Set(goals);
  const goalDists = goals.map((g) => bfsDist(w, h, passable, [g]));
  const dead = staticDead(w, h, passable, goals, goalDists);
  const perms = n <= 5 ? permutations(n) : null;

  const heuristic = (boxes: number[]): number => {
    if (perms !== null) {
      let best = Infinity;
      for (const p of perms) {
        let s = 0;
        for (let i = 0; i < n; i++) {
          const d = goalDists[p[i] as number]?.[boxes[i] as number] as number;
          if (d < 0) {
            s = Infinity;
            break;
          }
          s += d;
        }
        if (s < best) best = s;
      }
      return best;
    }
    let s = 0;
    for (const b of boxes) {
      let m = Infinity;
      for (const d of goalDists) {
        const v = d[b] as number;
        if (v >= 0 && v < m) m = v;
      }
      if (m === Infinity) return Infinity;
      s += m;
    }
    return s;
  };

  // Node storage: parallel arrays, index 0.. are the roots (one per entry).
  const nBoxes: number[][] = [];
  const nPlayer: number[] = [];
  const nParent: number[] = [];
  const nPushBox: number[] = [];
  const nPushDir: number[] = [];
  const nG: number[] = [];
  const best = new Map<string, number>();

  const keyOf = (boxes: number[], canonPlayer: number): string =>
    `${boxes.join(',')}|${canonPlayer}`;

  const boxMaskOf = (boxes: number[]): Uint8Array => {
    const m = new Uint8Array(w * h);
    for (const b of boxes) m[b] = 1;
    return m;
  };

  const boxAtOf = (boxes: number[]): Int32Array => {
    const m = new Int32Array(w * h).fill(-1);
    boxes.forEach((b, i) => {
      m[b] = i;
    });
    return m;
  };

  // Binary heap of [f, g, node], smallest f first, deeper g first on a tie,
  // then the earlier node: fully deterministic.
  const heap: [number, number, number][] = [];
  const heapLess = (
    a: [number, number, number],
    b: [number, number, number],
  ): boolean =>
    a[0] < b[0] ||
    (a[0] === b[0] && (a[1] > b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!heapLess(heap[i] as [number, number, number], heap[p] as [number, number, number])) break;
      const t = heap[i] as [number, number, number];
      heap[i] = heap[p] as [number, number, number];
      heap[p] = t;
      i = p;
    }
  };
  const heapPop = (): [number, number, number] => {
    const top = heap[0] as [number, number, number];
    const last = heap.pop() as [number, number, number];
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (
          c + 1 < heap.length &&
          heapLess(
            heap[c + 1] as [number, number, number],
            heap[c] as [number, number, number],
          )
        ) {
          c++;
        }
        if (
          !heapLess(
            heap[c] as [number, number, number],
            heap[i] as [number, number, number],
          )
        ) {
          break;
        }
        const t = heap[i] as [number, number, number];
        heap[i] = heap[c] as [number, number, number];
        heap[c] = t;
        i = c;
      }
    }
    return top;
  };

  const addNode = (
    boxes: number[],
    player: number,
    parent: number,
    pushBox: number,
    pushDir: number,
    g: number,
  ): void => {
    const mask = boxMaskOf(boxes);
    const canon = flood(w, h, passable, mask, player).min;
    if (canon < 0) return;
    const k = keyOf(boxes, canon);
    const prev = best.get(k);
    if (prev !== undefined && (nG[prev] as number) <= g) return;
    const id = nBoxes.length;
    nBoxes.push(boxes);
    nPlayer.push(player);
    nParent.push(parent);
    nPushBox.push(pushBox);
    nPushDir.push(pushDir);
    nG.push(g);
    best.set(k, id);
    const hv = heuristic(boxes);
    if (hv === Infinity) return;
    heapPush([g + hv, g, id]);
  };

  for (const e of entries) {
    if (!passable[e]) continue;
    addNode(boxes0.slice(), e, -1, -1, -1, 0);
  }

  let expansions = 0;
  while (heap.length > 0) {
    if (expansions >= roomCap || budget.used >= budget.cap) return null;
    const [, g, id] = heapPop();
    const boxes = nBoxes[id] as number[];
    const k = keyOf(boxes, flood(w, h, passable, boxMaskOf(boxes), nPlayer[id] as number).min);
    if (best.get(k) !== id) continue; // stale entry
    if ((nG[id] as number) !== g) continue;
    if (boxes.every((b) => goalSet.has(b))) {
      if (!needExit) {
        return finish(id);
      }
      const mask = boxMaskOf(boxes);
      const reach = flood(w, h, passable, mask, nPlayer[id] as number).reach;
      if (exitCells.some((c) => reach[c] === 1)) {
        return finish(id);
      }
      // Otherwise keep expanding: a longer route may still leave the door open.
    }
    expansions++;
    budget.used++;
    const mask = boxMaskOf(boxes);
    const reach = flood(w, h, passable, mask, nPlayer[id] as number).reach;
    const boxAt = boxAtOf(boxes);
    for (let bi = 0; bi < n; bi++) {
      const b = boxes[bi] as number;
      const bx = b % w;
      const by = (b / w) | 0;
      for (let di = 0; di < DIRS.length; di++) {
        const d = DIRS[di] as Dir;
        const nx = bx + d.dx;
        const ny = by + d.dy;
        if (!inBounds(w, h, nx, ny)) continue;
        const dest = idx(w, nx, ny);
        if (!passable[dest] || mask[dest]) continue;
        const px = bx - d.dx;
        const py = by - d.dy;
        if (!inBounds(w, h, px, py)) continue;
        const pushSquare = idx(w, px, py);
        if (!reach[pushSquare]) continue;
        const nb = boxes.slice();
        nb[bi] = dest;
        nb.sort((a, b2) => a - b2);
        if (dead[dest]) continue;
        if (freezeDead(w, h, passable, nb, boxAtOf(nb), goalSet)) continue;
        addNode(nb, b, id, b, di, g + 1);
      }
    }
  }
  return null;

  function finish(id: number): { pushes: Push[]; startEntry: number; expansions: number } {
    const pushes: Push[] = [];
    let cur = id;
    while ((nParent[cur] as number) >= 0) {
      pushes.push({ box: nPushBox[cur] as number, dir: nPushDir[cur] as number });
      cur = nParent[cur] as number;
    }
    pushes.reverse();
    return { pushes, startEntry: nPlayer[cur] as number, expansions };
  }
}

// Turn a push list into the player's move string, walking to each push
// square first. Returns the moves, the player's final cell and the boxes'
// final cells, or null when a push cannot be reached.
function pushesToMoves(
  w: number,
  h: number,
  passable: Uint8Array,
  boxes0: number[],
  entry: number,
  pushes: Push[],
): { moves: string; endPlayer: number; endBoxes: number[] } | null {
  const boxes = boxes0.slice();
  let player = entry;
  let moves = '';
  for (const p of pushes) {
    const d = DIRS[p.dir] as Dir;
    const bx = p.box % w;
    const by = (p.box / w) | 0;
    const px = bx - d.dx;
    const py = by - d.dy;
    if (!inBounds(w, h, px, py)) return null;
    const pushSquare = idx(w, px, py);
    const blocked = new Uint8Array(w * h);
    for (const b of boxes) blocked[b] = 1;
    const path = walkPath(w, h, passable, blocked, player, pushSquare);
    if (path === null) return null;
    moves += path + d.ch;
    const bi = boxes.indexOf(p.box);
    if (bi < 0) return null;
    boxes[bi] = idx(w, bx + d.dx, by + d.dy);
    player = p.box;
  }
  return { moves, endPlayer: player, endBoxes: boxes };
}

// Replay a move string on a level; true only when every step is legal and
// every box ends on a goal.
function simulate(g: Grid, moves: string): boolean {
  const boxes = new Set(g.boxes);
  let p = g.player;
  for (const ch of moves) {
    const d = DIRS.find((dd) => dd.ch === ch);
    if (d === undefined) return false;
    const x = p % g.w;
    const y = (p / g.w) | 0;
    const nx = x + d.dx;
    const ny = y + d.dy;
    if (!inBounds(g.w, g.h, nx, ny)) return false;
    const np = idx(g.w, nx, ny);
    if (g.wall[np]) return false;
    if (boxes.has(np)) {
      const bx = nx + d.dx;
      const by = ny + d.dy;
      if (!inBounds(g.w, g.h, bx, by)) return false;
      const nb = idx(g.w, bx, by);
      if (g.wall[nb] || boxes.has(nb)) return false;
      boxes.delete(np);
      boxes.add(nb);
    }
    p = np;
  }
  return g.goals.every((gl) => boxes.has(gl));
}

type Room = {
  cells: number[];
  goals: number[];
  boxes: number[];
  entries: number[];
};

// Split a level into its rooms: the hallway is the player's row, and each
// room is a connected component of floor off that row holding goals.
function detectRooms(g: Grid): Room[] {
  const { w, h } = g;
  const hy = (g.player / w) | 0;
  const goalSet = new Set(g.goals);
  const boxSet = new Set(g.boxes);
  const seen = new Uint8Array(w * h);
  const rooms: Room[] = [];
  for (let i = 0; i < w * h; i++) {
    if (g.wall[i] || ((i / w) | 0) === hy || seen[i]) continue;
    const comp: number[] = [];
    const q: number[] = [i];
    seen[i] = 1;
    for (let head = 0; head < q.length; head++) {
      const c = q[head] as number;
      comp.push(c);
      const cx = c % w;
      const cy = (c / w) | 0;
      for (const d of DIRS) {
        const nx = cx + d.dx;
        const ny = cy + d.dy;
        if (!inBounds(w, h, nx, ny)) continue;
        const n = idx(w, nx, ny);
        if (g.wall[n] || ((n / w) | 0) === hy || seen[n]) continue;
        seen[n] = 1;
        q.push(n);
      }
    }
    const goals = comp.filter((c) => goalSet.has(c));
    if (goals.length === 0) continue;
    const boxes = comp.filter((c) => boxSet.has(c));
    // The door is the hallway cell the room opens onto: a box may sit in the
    // room-side doorway cell, and the player pushes it from the hallway, so
    // the entries are hallway cells, not room cells.
    const entrySet = new Set<number>();
    for (const c of comp) {
      const cx = c % w;
      const cy = (c / w) | 0;
      for (const d of DIRS) {
        const nx = cx + d.dx;
        const ny = cy + d.dy;
        if (!inBounds(w, h, nx, ny)) continue;
        const n = idx(w, nx, ny);
        if (((n / w) | 0) === hy && !g.wall[n]) entrySet.add(n);
      }
    }
    const entries = [...entrySet].sort((a, b) => a - b);
    rooms.push({ cells: comp, goals, boxes, entries });
  }
  return rooms;
}

// Fallback for levels that do not split into rooms: one A* over the whole
// level (sum-of-minimum heuristic once there are more than 5 boxes).
function solveWhole(g: Grid, budget: { used: number; cap: number }): string {
  const { w, h } = g;
  const passable = new Uint8Array(w * h);
  for (let i = 0; i < w * h; i++) passable[i] = g.wall[i] ? 0 : 1;
  const res = astarPush(
    w,
    h,
    passable,
    g.goals,
    g.boxes,
    [g.player],
    budget,
    ROOM_NODE_CAP,
    false,
    [],
  );
  if (res === null) return '';
  const conv = pushesToMoves(w, h, passable, g.boxes, res.startEntry, res.pushes);
  if (conv === null) return '';
  return simulate(g, conv.moves) ? conv.moves : '';
}

function solveLevel(text: string, budget: { used: number; cap: number }): string {
  const g = parseLevel(text);
  const { w, h } = g;
  const floor = new Uint8Array(w * h);
  for (let i = 0; i < w * h; i++) floor[i] = g.wall[i] ? 0 : 1;

  const rooms = detectRooms(g);
  const coveredBoxes = rooms.reduce((a, r) => a + r.boxes.length, 0);
  const coveredGoals = rooms.reduce((a, r) => a + r.goals.length, 0);
  if (
    rooms.length === 0 ||
    coveredBoxes !== g.boxes.length ||
    coveredGoals !== g.goals.length ||
    rooms.some((r) => r.entries.length === 0 || r.boxes.length === 0)
  ) {
    return solveWhole(g, budget);
  }

  // Left to right by door, so the walk along the hallway stays short.
  rooms.sort((a, b) => {
    const ax = Math.min(...a.entries.map((c) => c % w));
    const bx = Math.min(...b.entries.map((c) => c % w));
    if (ax !== bx) return ax - bx;
    const ay = Math.min(...a.entries.map((c) => (c / w) | 0));
    const by = Math.min(...b.entries.map((c) => (c / w) | 0));
    return ay - by;
  });

  let player = g.player;
  const boxes = g.boxes.slice();
  let moves = '';
  for (let ri = 0; ri < rooms.length; ri++) {
    const room = rooms[ri] as Room;
    const last = ri === rooms.length - 1;
    // The search region is the room plus its hallway door cells, so the
    // player can stand in the doorway and push a box out of it.
    const passable = new Uint8Array(w * h);
    for (const c of room.cells) passable[c] = 1;
    for (const c of room.entries) passable[c] = 1;
    const res = astarPush(
      w,
      h,
      passable,
      room.goals,
      room.boxes,
      room.entries,
      budget,
      ROOM_NODE_CAP,
      !last,
      room.entries,
    );
    if (res === null) return '';
    // Walk from where the player stands to the door the solution used.
    const blocked = new Uint8Array(w * h);
    for (const b of boxes) blocked[b] = 1;
    const walk = walkPath(w, h, floor, blocked, player, res.startEntry);
    if (walk === null) return '';
    const conv = pushesToMoves(
      w,
      h,
      passable,
      room.boxes,
      res.startEntry,
      res.pushes,
    );
    if (conv === null) return '';
    moves += walk + conv.moves;
    player = conv.endPlayer;
    // The room's boxes are solved now; refresh the global box list.
    const roomBoxSet = new Set(room.boxes);
    const kept = boxes.filter((b) => !roomBoxSet.has(b));
    boxes.length = 0;
    boxes.push(...kept, ...conv.endBoxes);
    boxes.sort((a, b) => a - b);
  }
  return simulate(g, moves) ? moves : '';
}

function main(): void {
  const budget = { used: 0, cap: TOTAL_NODE_CAP };
  for (const id of readdirSync(INPUT_DIR).sort()) {
    const level = readFileSync(join(INPUT_DIR, id, 'level.txt'), 'utf8');
    const moves = solveLevel(level, budget);
    mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
    writeFileSync(join(OUTPUT_DIR, id, 'solution.txt'), `${moves}\n`);
  }
}

main();
