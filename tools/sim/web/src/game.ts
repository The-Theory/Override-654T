import { latest, type Piece, type Run } from "./sim.ts"

// Reading the game state out of a run at time t

export const COLOURS = ["yellow", "red", "blue"] as const
export const GOAL_KINDS = ["red", "blue", "neutral", "midfield"] as const

export interface Place { x: number; y: number; where: number }

export function placeAt(p: Piece, t: number): Place {
  const [, x, y, where] = latest(p.track, t)
  return { x, y, where }
}

export function goalAt(run: Run, x: number, y: number): number {
  return run.goals.findIndex(([gx, gy]) => Math.hypot(gx - x, gy - y) < 0.5)
}

// Pieces on each Goal, bottom first
export function stacks(run: Run, t: number): Piece[][] {
  const out: { level: number; piece: Piece }[][] = run.goals.map(() => [])
  for (const piece of run.pieces) {
    const { x, y, where } = placeAt(piece, t)
    if (where < 2) continue
    const g = goalAt(run, x, y)
    if (g >= 0) out[g]!.push({ level: where - 2, piece })
  }
  return out.map(s => s.sort((a, b) => a.level - b.level).map(e => e.piece))
}

export function pieceName(p: Piece): string {
  if (p.cup) return "Cup"
  const halves = p.top === p.bottom ? COLOURS[p.top] : `${COLOURS[p.top]}/${COLOURS[p.bottom]}`
  return `${halves[0]!.toUpperCase()}${halves.slice(1)} Pin${p.lying ? ", lying" : ""}`
}

export function placeName(run: Run, place: Place): string {
  if (place.where === 1) return "in the claw"
  if (place.where >= 2) {
    const g = goalAt(run, place.x, place.y)
    return `${ordinal(place.where - 1)} on ${g >= 0 ? run.goals[g]![3] : "a Goal"}`
  }
  return `on the tiles at ${coord(place.x)}, ${coord(place.y)}`
}

export const coord = (v: number) => v.toFixed(1)

function ordinal(n: number): string {
  return n + (["th", "st", "nd", "rd"][n % 10 > 3 || Math.floor(n / 10) === 1 ? 0 : n % 10] ?? "th")
}

////////////////////////////////////////////////////////////////
// Log and summary

// One log line: a LemLib motion from start to end, or an instant claw action
export interface Step {
  start:  number
  end:    number
  action: string
  result: string
  claw:   boolean
  timeout: boolean
}

const MOTION = /^(\w+) (settled|TIMEOUT|exited early|cancelled) after (\d+) ms/

export function steps(run: Run): Step[] {
  return run.events.map(([t, text]): Step => {
    const m = MOTION.exec(text)
    if (!m) return { start: t, end: t, action: "Claw", result: text.replace(/^claw /, ""), claw: true, timeout: false }
    const timeout = m[2] === "TIMEOUT"
    return { start: t - Number(m[3]) / 1000, end: t, action: m[1]!, result: timeout ? "timed out" : m[2]!, claw: false, timeout }
  }).sort((a, b) => a.start - b.start)
}

export interface Summary {
  end:      number
  finished: boolean
  limit:    number
  motions:  number
  timeouts: number
  picked:   number
  stacked:  string[]   // Goal names, in order
  dropped:  number
  flips:    string[]
  red:      number
  blue:     number
  odomOff:  number     // [in] at the end
}

export function summarize(run: Run): Summary {
  const all  = steps(run)
  const claw = all.filter(s => s.claw).map(s => s.result)
  const last = run.frames.at(-1)!
  const [, red, blue] = run.score.at(-1)!
  return {
    end:      last[0],
    finished: run.finished,
    limit:    run.limit,
    motions:  all.filter(s => !s.claw).length,
    timeouts: all.filter(s => s.timeout).length,
    picked:   claw.filter(r => r.startsWith("picked up")).length,
    stacked:  claw.filter(r => r.startsWith("stacked on")).map(r => r.replace("stacked on ", "")),
    dropped:  claw.filter(r => r.startsWith("dropped")).length,
    flips:    run.toggles.flatMap(tg => tg.faces.slice(1).map(([t, face]) => `${tg.name} to ${COLOURS[face]} at ${t.toFixed(2)} s`)),
    red, blue,
    odomOff:  Math.hypot(last[1] - last[4], last[2] - last[5]),
  }
}

// What the claw has at t, e.g. "Pin in Cup"
export function holding(run: Run, t: number): string {
  const held = run.pieces.filter(p => placeAt(p, t).where === 1)
  if (held.length === 0) return "Nothing"
  if (held.length === 2) return "Pin in Cup"
  return pieceName(held[0]!)
}
