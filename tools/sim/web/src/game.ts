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
  return `${halves} Pin${p.lying ? ", lying" : ""}`
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
