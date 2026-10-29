// The robot's sim, built by `make web` into public/sim.js, and the run log it writes.
// Each call gets a fresh instance, since the robot code's globals only start clean once.

// [t s, x, y, heading, odom x, odom y, odom heading], every 10 ms
export type Frame = [number, number, number, number, number, number, number]

// [t s, x, y, where]: where 0 = tiles, 1 = in the claw, 2 + n = stacked n deep on its Goal
export type Sample = [number, number, number, number]

// Colour codes: 0 yellow, 1 red, 2 blue
export interface Piece {
  cup:    number
  top:    number
  bottom: number
  lying:  number
  angle:  number          // [deg] heading a lying Pin's top half points along
  track:  Sample[]
}

export interface Run {
  auton:    string
  finished: boolean                                                    // false if cut off at the time limit
  limit:    number                                                     // [s]
  robot:    { width: number; front: number; back: number }
  goals:    [number, number, number, string][]                         // [x, y, kind, name]; kind 0 red, 1 blue, 2 neutral, 3 midfield
  toggles:  { name: string; x: number; y: number; faces: [number, number][] }[]   // faces: [t, colour]
  pieces:   Piece[]
  score:    [number, number, number][]                                 // [t, red, blue]
  frames:   Frame[]
  events:   [number, string][]
}

// The last row at or before t, for logs that only note changes
export function latest<T extends readonly [number, ...unknown[]]>(rows: T[], t: number): T {
  let lo = 0, hi = rows.length - 1
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1
    if (rows[mid]![0] <= t) lo = mid
    else hi = mid - 1
  }
  return rows[lo]!
}

// Frames are 10 ms apart
export function frameAt(run: Run, t: number): Frame {
  return run.frames[Math.min(run.frames.length - 1, Math.max(0, Math.round(t * 100)))]!
}

interface Module {
  callMain(args: string[]): void
  FS: { readFile(path: string, opts: { encoding: "utf8" }): string }
}
type Factory = (opts: { print: (line: string) => void; printErr: (line: string) => void }) => Promise<Module>

export class Sim {
  private constructor(private create: Factory) {}

  // Imported from a blob so a dropped-in file loads the same way as a served one
  static async load(source: string | Blob): Promise<Sim> {
    const blob = typeof source === "string" ? await (await fetch(source)).blob() : source
    const url  = URL.createObjectURL(new Blob([blob], { type: "text/javascript" }))
    try {
      const mod = await import(/* @vite-ignore */ url) as { default: Factory }
      return new Sim(mod.default)
    } finally {
      URL.revokeObjectURL(url)
    }
  }

  async autons(): Promise<string[]> {
    const { lines } = await this.call(["--list"])
    return lines.filter(l => l.length > 0)
  }

  async run(auton: string, seconds: number): Promise<Run> {
    const { mod } = await this.call([auton, "--out", "/out", "--time", String(seconds * 1000)])
    return JSON.parse(mod.FS.readFile(`/out/${auton}.json`, { encoding: "utf8" })) as Run
  }

  private async call(args: string[]): Promise<{ mod: Module; lines: string[] }> {
    const lines: string[] = []
    const mod = await this.create({ print: l => lines.push(l), printErr: l => console.warn(l) })
    try {
      mod.callMain(args)
    } catch (e) {
      // exit() surfaces as a thrown ExitStatus
      if ((e as { name?: string }).name !== "ExitStatus") throw e
    }
    return { mod, lines }
  }
}
