// Sokoban solver: push-optimal, room-decomposed.
//
// Strategy: a level is a 1-wide hallway plus 1-4 rooms. A box pushed into the
// hallway can never come back, so each room is an independent sub-puzzle: its
// boxes stay in the room and end on goals inside it. We find the rooms with
// articulation points of the floor graph, solve each room push-optimally with
// A* (player-reachability per push, Hungarian box-to-goal heuristic, dead
// squares, 2x2 freeze pruning, region-normalised transposition table), then
// stitch the room solutions together with shortest hallway walks. Pushes add
// up to the level optimum, because the room optima are independent.
//
// Deterministic: fixed direction order everywhere, heap ties broken by an
// insertion counter, expansion-count budgets only (no wall clock in any
// decision), so two runs write identical bytes.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Per-room A* expansion budget (deterministic). Rooms have 3-4 boxes and
// solve in far fewer expansions; the cap is only a safety valve.
const ROOM_EXPANSIONS = 2_000_000;
// Global fallback budget for a whole-level search (only used when the room
// decomposition cannot cover every box).
const GLOBAL_EXPANSIONS = 3_000_000;

const DIRS: [number, number, string][] = [
  [0, -1, 'u'],
  [0, 1, 'd'],
  [-1, 0, 'l'],
  [1, 0, 'r'],
];

type Level = {
  w: number;
  h: number;
  wall: boolean[]; // index = y*w+x
  goals: number[]; // sorted cell indices
  goalSet: Set<number>;
  boxes: number[]; // sorted cell indices
  player: number;
};

type Room = {
  cells: Set<number>; // room floor cells, door included
  door: number; // articulation cell the player enters through
  entry: number; // hallway cell the player stands on to enter (door neighbour)
  boxes: number[]; // sorted, initial box cells inside the room
  goals: number[]; // sorted, goal cells inside the room
};

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const wall: boolean[] = new Array(w * h).fill(true);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = 0;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      wall[i] = ch === '#';
      if ('.*+'.includes(ch)) goals.push(i);
      if ('$*'.includes(ch)) boxes.push(i);
      if ('@+'.includes(ch)) player = i;
    }
  });
  goals.sort((a, b) => a - b);
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, goalSet: new Set(goals), boxes, player };
}

function neighbours(i: number, w: number, h: number): number[] {
  const x = i % w;
  const y = (i / w) | 0;
  const out: number[] = [];
  if (x > 0) out.push(i - 1);
  if (x < w - 1) out.push(i + 1);
  if (y > 0) out.push(i - w);
  if (y < h - 1) out.push(i + w);
  return out;
}

// ---- articulation points (iterative Tarjan) -------------------------------
function articulationPoints(level: Level): Set<number> {
  const { w, h, wall } = level;
  const n = w * h;
  const disc = new Int32Array(n).fill(-1);
  const low = new Int32Array(n).fill(0);
  const parent = new Int32Array(n).fill(-1);
  const ap = new Set<number>();
  let timer = 0;
  const floor = (i: number): boolean => !wall[i];
  for (let start = 0; start < n; start++) {
    if (wall[start] || disc[start] !== -1) continue;
    // iterative DFS with an explicit stack of frames
    type Frame = { u: number; childIdx: number; children: number };
    const stack: Frame[] = [{ u: start, childIdx: 0, children: 0 }];
    disc[start] = low[start] = timer++;
    while (stack.length > 0) {
      const f = stack[stack.length - 1]!;
      const u = f.u;
      const ns = neighbours(u, w, h).filter(floor);
      if (f.childIdx < ns.length) {
        const v = ns[f.childIdx]!;
        f.childIdx++;
        if (disc[v] !== -1) {
          if (v !== parent[u]) low[u] = Math.min(low[u], disc[v]);
          continue;
        }
        parent[v] = u;
        f.children++;
        disc[v] = low[v] = timer++;
        stack.push({ u: v, childIdx: 0, children: 0 });
      } else {
        stack.pop();
        const p = parent[u];
        if (p !== -1) {
          low[p] = Math.min(low[p], low[u]);
          if (parent[p] !== -1 && low[u] >= disc[p]) ap.add(p);
          if (parent[p] === -1 && stack.length === 0) {
            // root finished below; handled by children count
          }
        } else {
          if (f.children > 1) ap.add(u);
        }
      }
    }
  }
  return ap;
}

// Connected components of the floor graph after removing `removed`.
function componentsAfterRemoval(level: Level, removed: number): number[][] {
  const { w, h, wall } = level;
  const seen = new Uint8Array(w * h);
  seen[removed] = 1;
  const comps: number[][] = [];
  for (let i = 0; i < w * h; i++) {
    if (wall[i] || seen[i]) continue;
    const comp: number[] = [];
    const q: number[] = [i];
    seen[i] = 1;
    while (q.length > 0) {
      const u = q.pop()!;
      comp.push(u);
      for (const v of neighbours(u, w, h)) {
        if (!wall[v] && !seen[v]) {
          seen[v] = 1;
          q.push(v);
        }
      }
    }
    comps.push(comp);
  }
  return comps;
}

// ---- room decomposition ---------------------------------------------------
function decompose(level: Level): Room[] | null {
  const aps = articulationPoints(level);
  const boxSet = new Set(level.boxes);
  const goalSet = level.goalSet;
  type Cand = { cells: number[]; door: number; boxes: number[]; goals: number[] };
  const cands: Cand[] = [];
  for (const v of aps) {
    for (const comp of componentsAfterRemoval(level, v)) {
      if (comp.includes(level.player)) continue;
      const cells = [...comp, v];
      const boxes = cells.filter((c) => boxSet.has(c)).sort((a, b) => a - b);
      const goals = cells.filter((c) => goalSet.has(c)).sort((a, b) => a - b);
      if (boxes.length >= 1 && boxes.length === goals.length) {
        cands.push({ cells, door: v, boxes, goals });
      }
    }
  }
  // Greedy smallest-first; a room's boxes must not overlap a picked room's.
  // The smallest balanced component per box set is that box set's room (or a
  // balanced split of it, whose optima still add up to the room optimum).
  const sorted = [...cands].sort((a, b) => a.cells.length - b.cells.length);
  const picked: Cand[] = [];
  const usedBoxes = new Set<number>();
  for (const c of sorted) {
    if (c.boxes.some((b) => usedBoxes.has(b))) continue;
    picked.push(c);
    for (const b of c.boxes) usedBoxes.add(b);
  }
  if (usedBoxes.size === level.boxes.length) {
    const rooms: Room[] = picked.map((c) => {
      const cellSet = new Set(c.cells);
      // Entry: a door neighbour outside the room. If the door holds a box,
      // the entry must be the side the box is pushed from, i.e. the
      // neighbour whose opposite cell lies inside the room.
      const doorHasBox = boxSet.has(c.door);
      const ns = neighbours(c.door, level.w, level.h)
        .filter((n) => !level.wall[n] && !cellSet.has(n))
        .sort((a, b) => a - b);
      let entry = -1;
      if (doorHasBox) {
        for (const n of ns) {
          const beyond = c.door + (c.door - n);
          if (cellSet.has(beyond)) {
            entry = n;
            break;
          }
        }
      }
      if (entry === -1) entry = ns[0] ?? -1;
      return { cells: cellSet, door: c.door, entry, boxes: c.boxes, goals: c.goals };
    });
    if (rooms.every((r) => r.entry !== -1)) {
      rooms.sort((a, b) => a.door - b.door);
      return rooms;
    }
  }
  return null;
}

// ---- per-room tables ------------------------------------------------------
function deadSquares(roomCells: Set<number>, goals: number[], level: Level): Set<number> {
  // Reverse pull reachability from the goals: a box on cell x can be pushed
  // to a live cell y when the player can stand on the far side (2x - y).
  // Cells never reached are dead (a box there can never reach a goal).
  const live = new Set<number>(goals);
  const q: number[] = [...goals];
  const { w, h, wall } = level;
  while (q.length > 0) {
    const y = q.pop()!;
    const yx = y % w;
    const yy = (y / w) | 0;
    for (const [dx, dy] of [
      [1, 0],
      [-1, 0],
      [0, 1],
      [0, -1],
    ] as const) {
      const x = y + dy * w + dx;
      if (!roomCells.has(x)) continue;
      const s = x + dy * w + dx; // cell the player stands on to push x -> y
      if (!roomCells.has(s)) continue;
      if (live.has(x)) continue;
      // x must be inside the board and not a wall (roomCells implies floor)
      void yx;
      void yy;
      void w;
      void h;
      void wall;
      live.add(x);
      q.push(x);
    }
  }
  const dead = new Set<number>();
  for (const c of roomCells) if (!live.has(c)) dead.add(c);
  return dead;
}

// BFS distances from a source over room cells (ignoring boxes).
function bfsDist(roomCells: Set<number>, src: number, level: Level): Map<number, number> {
  const dist = new Map<number, number>([[src, 0]]);
  const q: number[] = [src];
  let qi = 0;
  while (qi < q.length) {
    const u = q[qi++]!;
    for (const v of neighbours(u, level.w, level.h)) {
      if (roomCells.has(v) && !dist.has(v)) {
        dist.set(v, dist.get(u)! + 1);
        q.push(v);
      }
    }
  }
  return dist;
}

// Minimum-cost box->goal assignment. Subset DP (exact) for n <= 6, else the
// sum of per-box minima (still admissible, weaker). Costs are shortest-path
// distances ignoring boxes; unreachable = large.
function hungarian(cost: number[][]): number {
  const n = cost.length;
  if (n === 0) return 0;
  if (n <= 6) {
    const size = 1 << n;
    const INF = 1e9;
    const dp = new Array<number>(size).fill(INF);
    dp[0] = 0;
    for (let mask = 0; mask < size; mask++) {
      const k = popcount(mask); // boxes assigned so far
      if (k >= n) continue;
      for (let j = 0; j < n; j++) {
        if (mask & (1 << j)) continue;
        const nm = mask | (1 << j);
        const v = dp[mask]! + cost[k]![j]!;
        if (v < dp[nm]!) dp[nm] = v;
      }
    }
    return dp[size - 1]!;
  }
  let s = 0;
  for (let i = 0; i < n; i++) s += Math.min(...cost[i]!);
  return s;
}
function popcount(x: number): number {
  let c = 0;
  while (x) {
    x &= x - 1;
    c++;
  }
  return c;
}

// Walking region of the player over `area` cells, excluding `boxSet`.
// Returns the set and its canonical representative (smallest cell).
function playerRegion(
  area: Set<number>,
  boxSet: Set<number>,
  start: number,
  level: Level,
): { region: Set<number>; canon: number } {
  const region = new Set<number>();
  const q: number[] = [start];
  region.add(start);
  let canon = start;
  while (q.length > 0) {
    const u = q.pop()!;
    if (u < canon) canon = u;
    for (const v of neighbours(u, level.w, level.h)) {
      if (area.has(v) && !boxSet.has(v) && !region.has(v)) {
        region.add(v);
        q.push(v);
      }
    }
  }
  return { region, canon };
}

// Shortest walk (list of direction letters, lowercase) from `from` to `to`
// over `area` cells, avoiding `boxSet`. Deterministic direction order.
function walkPath(
  area: Set<number>,
  boxSet: Set<number>,
  from: number,
  to: number,
  level: Level,
): string[] | null {
  if (from === to) return [];
  const prev = new Map<number, [number, string]>();
  const q: number[] = [from];
  prev.set(from, [-1, '']);
  let qi = 0;
  while (qi < q.length) {
    const u = q[qi++]!;
    for (const [dx, dy, letter] of DIRS) {
      const v = u + dy * level.w + dx;
      if (!area.has(v) || boxSet.has(v) || prev.has(v)) continue;
      prev.set(v, [u, letter]);
      if (v === to) {
        const path: string[] = [];
        let cur = v;
        while (cur !== from) {
          const [p, l] = prev.get(cur)!;
          path.push(l);
          cur = p;
        }
        return path.reverse();
      }
      q.push(v);
    }
  }
  return null;
}

// ---- push-based A* over one room ------------------------------------------
type Push = { from: number; to: number; dir: number; letter: string };

function solveRoom(
  room: Room,
  boxes0: number[],
  level: Level,
  expansionBudget: number,
): { pushes: Push[]; expansions: number } | null {
  const area = room.cells;
  const goalSet = new Set(room.goals);
  const dead = deadSquares(area, room.goals, level);
  // Pairwise box->goal shortest distances (ignoring boxes).
  const goalDist = room.goals.map((g) => bfsDist(area, g, level));
  const INF = 1e6;
  const heuristic = (boxes: number[]): number => {
    const cost = boxes.map((b) => room.goals.map((g, gi) => goalDist[gi]!.get(b) ?? INF));
    return hungarian(cost);
  };

  // State arrays (index = state id).
  const stBoxes: number[][] = [boxes0.slice().sort((a, b) => a - b)];
  const stPlayer: number[] = [room.door]; // player cell after the push that made it
  const stG: number[] = [0];
  const stParent: number[] = [-1];
  const stPush: (Push | null)[] = [null];
  const best = new Map<string, number>();
  const keyOf = (boxes: number[], canon: number): string => `${canon}|${boxes.join(',')}`;

  // Open list: binary heap on (f, g, seq). f = g + h.
  type HeapItem = [number, number, number, number]; // f, g, seq, stateId
  const heap: HeapItem[] = [];
  let seq = 0;
  const less = (a: HeapItem, b: HeapItem): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] > b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: HeapItem): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const heapPop = (): HeapItem => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };

  const startBoxes = stBoxes[0]!;
  const startRegion = playerRegion(area, new Set(startBoxes), room.door, level);
  best.set(keyOf(startBoxes, startRegion.canon), 0);
  heapPush([heuristic(startBoxes), 0, seq++, 0]);

  let expansions = 0;
  const isDead = (boxes: number[]): boolean => {
    // A box on a dead square (not a goal) can never be solved.
    for (const b of boxes) if (!goalSet.has(b) && dead.has(b)) return true;
    // 2x2 freeze: four cells all wall/box, none a goal.
    const bs = new Set(boxes);
    for (const b of boxes) {
      const x = b % level.w;
      const y = (b / level.w) | 0;
      if (x + 1 >= level.w || y + 1 >= level.h) continue;
      const cells = [b, b + 1, b + level.w, b + level.w + 1];
      if (cells.every((c) => level.wall[c] || bs.has(c)) && cells.every((c) => !goalSet.has(c))) {
        return true;
      }
    }
    return false;
  };

  while (heap.length > 0 && expansions < expansionBudget) {
    const [, g, , sid] = heapPop();
    const boxes = stBoxes[sid]!;
    const player = stPlayer[sid]!;
    const boxSet = new Set(boxes);
    const { region, canon } = playerRegion(area, boxSet, player, level);
    if (best.get(keyOf(boxes, canon)) !== sid) continue;
    // Goal: every box on a goal and the player can walk back to the door.
    if (boxes.every((b) => goalSet.has(b)) && region.has(room.door)) {
      const pushes: Push[] = [];
      for (let t = sid; t > 0; t = stParent[t]!) pushes.push(stPush[t]!);
      pushes.reverse();
      return { pushes, expansions };
    }
    expansions++;
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi]!;
      for (const [dx, dy, letter] of DIRS) {
        const dest = b + dy * level.w + dx;
        const stand = b - dy * level.w - dx;
        if (!area.has(dest) || boxSet.has(dest)) continue;
        if (!region.has(stand)) continue;
        const nb = boxes.slice();
        nb[bi] = dest;
        nb.sort((a, b2) => a - b2);
        if (isDead(nb)) continue;
        const nRegion = playerRegion(area, new Set(nb), b, level);
        const k = keyOf(nb, nRegion.canon);
        const ng = g + 1;
        const prev = best.get(k);
        if (prev !== undefined && stG[prev]! <= ng) continue;
        const nid = stBoxes.length;
        stBoxes.push(nb);
        stPlayer.push(b);
        stG.push(ng);
        stParent.push(sid);
        stPush.push({ from: b, to: dest, dir: dy * level.w + dx, letter });
        best.set(k, nid);
        heapPush([ng + heuristic(nb), ng, seq++, nid]);
      }
    }
  }
  return null;
}

// Convert a push list into the move string for one room. The player starts
// on the door; between pushes the player walks (shortest path) to the next
// pushing cell; afterwards, unless this is the last room, the player walks
// back to the door and steps out to the entry cell.
function roomMoves(
  room: Room,
  pushes: Push[],
  boxes0: number[],
  level: Level,
  leave: boolean,
): string {
  const area = room.cells;
  const boxes = boxes0.slice().sort((a, b) => a - b);
  let player = room.door;
  let out = '';
  for (const p of pushes) {
    const stand = p.from - p.dir;
    const boxSet = new Set(boxes);
    const path = walkPath(area, boxSet, player, stand, level);
    if (path === null) throw new Error('no walk to pushing cell');
    out += path.join('');
    out += p.letter.toUpperCase();
    const bi = boxes.indexOf(p.from);
    boxes[bi] = p.to;
    boxes.sort((a, b) => a - b);
    player = p.from;
  }
  if (!leave) return out;
  // Walk back to the door and step out to the entry cell.
  const boxSet = new Set(boxes);
  const exit = walkPath(area, boxSet, player, room.door, level);
  if (exit === null) throw new Error('no walk back to door');
  out += exit.join('');
  const dx = room.entry % level.w - room.door % level.w;
  const dy = ((room.entry / level.w) | 0) - ((room.door / level.w) | 0);
  const step = DIRS.find(([x, y]) => x === dx && y === dy);
  if (!step) throw new Error('entry is not adjacent to door');
  out += step[2];
  return out;
}

// ---- whole-level solve -----------------------------------------------------
function solveLevel(level: Level): string {
  const rooms = decompose(level);
  const allArea = new Set<number>();
  for (let i = 0; i < level.w * level.h; i++) if (!level.wall[i]) allArea.add(i);

  if (rooms !== null) {
    let moves = '';
    let player = level.player;
    // Current box cells, updated by simulating every push we emit.
    const boxNow = level.boxes.slice();
    const applyPush = (from: number, to: number): void => {
      const bi = boxNow.indexOf(from);
      if (bi < 0) throw new Error('push of a box that is not there');
      boxNow[bi] = to;
      boxNow.sort((a, b) => a - b);
    };
    for (let ri = 0; ri < rooms.length; ri++) {
      const room = rooms[ri]!;
      const isLast = ri === rooms.length - 1;
      // Walk the player through the level (boxes are obstacles) to the entry.
      const boxSet = new Set(boxNow);
      const toEntry = walkPath(allArea, boxSet, player, room.entry, level);
      if (toEntry === null) throw new Error('cannot reach room entry');
      moves += toEntry.join('');
      player = room.entry;
      // The door box, if any, is pushed in first (forced, one push).
      let roomBoxes = room.boxes.slice();
      if (boxNow.includes(room.door)) {
        const dir = room.door - room.entry;
        const dest = room.door + dir;
        const letter = DIRS.find(([dx, dy]) => dy * level.w + dx === dir)![2];
        moves += letter.toUpperCase();
        applyPush(room.door, dest);
        roomBoxes = room.boxes.map((b) => (b === room.door ? dest : b)).sort((a, b) => a - b);
        player = room.door;
      } else {
        // Step from the entry onto the door.
        const dx = room.door % level.w - room.entry % level.w;
        const dy = ((room.door / level.w) | 0) - ((room.entry / level.w) | 0);
        const step = DIRS.find(([x, y]) => x === dx && y === dy);
        if (!step) throw new Error('entry is not adjacent to door');
        moves += step[2];
        player = room.door;
      }
      const res = solveRoom(room, roomBoxes, level, ROOM_EXPANSIONS);
      if (res === null) throw new Error('room search failed');
      moves += roomMoves(room, res.pushes, roomBoxes, level, !isLast);
      for (const p of res.pushes) applyPush(p.from, p.to);
      player = room.entry; // roomMoves ends with the player stepping out
    }
    return moves;
  }

  // Fallback: one global push-A* over the whole level (single-room levels).
  const whole: Room = {
    cells: allArea,
    door: level.player,
    entry: level.player,
    boxes: level.boxes.slice(),
    goals: level.goals.slice(),
  };
  const res = solveRoom(whole, level.boxes.slice(), level, GLOBAL_EXPANSIONS);
  if (res === null) return '';
  // Reconstruct moves without the room entry/exit wrapper.
  const boxes = level.boxes.slice();
  let player = level.player;
  let out = '';
  for (const p of res.pushes) {
    const stand = p.from - p.dir;
    const path = walkPath(allArea, new Set(boxes), player, stand, level);
    if (path === null) return '';
    out += path.join('');
    out += p.letter.toUpperCase();
    const bi = boxes.indexOf(p.from);
    boxes[bi] = p.to;
    boxes.sort((a, b) => a - b);
    player = p.from;
  }
  return out;
}

for (const id of readdirSync(I).sort()) {
  const text = readFileSync(join(I, id, 'level.txt'), 'utf8');
  const level = parseLevel(text);
  let moves = '';
  try {
    moves = solveLevel(level);
  } catch (e) {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
