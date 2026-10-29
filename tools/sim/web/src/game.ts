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
// Steps and summary

// One step of the auton: a LemLib motion from start to end, or an instant claw action
export interface Step {
  start:   number
  end:     number
  what:    string    // in words, e.g. "Move to pose" or "Picked up Pin in Cup"
  claw:    boolean
  timeout: boolean
  early:   boolean   // a minSpeed motion that handed off before settling
}

const MOTION = /^(\w+) (settled|TIMEOUT|exited early|cancelled) after (\d+) ms/

// moveToPose -> "Move to pose"
const words = (name: string) => name.replace(/([A-Z])/g, " $1").toLowerCase().replace(/^./, c => c.toUpperCase())

export function steps(run: Run): Step[] {
  return run.events.map(([t, text]): Step => {
    const m = MOTION.exec(text)
    if (!m) {
      const what = text.replace(/^claw /, "")
      return { start: t, end: t, what: what[0]!.toUpperCase() + what.slice(1), claw: true, timeout: false, early: false }
    }
    return { start: t - Number(m[3]) / 1000, end: t, what: words(m[1]!), claw: false, timeout: m[2] === "TIMEOUT", early: m[2] === "exited early" }
  }).sort((a, b) => a.start - b.start)
}
