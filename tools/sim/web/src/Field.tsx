import { memo } from "react"
import { COLOURS, GOAL_KINDS, pieceName, placeAt, placeName, stacks } from "./game.ts"
import { frameAt, latest, type Run } from "./sim.ts"

// Top-down field in LemLib's frame (inches, +y away from the audience). The flip below makes SVG
// y point up too, so LemLib headings (clockwise) become rotate(-heading).

const HALF = 70.2
const TILE = 2 * HALF / 6
const GOAL_SIZE = [5.6, 5.6, 4.5, 6.5]	// [in] by kind

export interface Inspect { title: string; detail: string }

interface Props {
  run: Run
  t: number
  onInspect: (what: Inspect | null) => void
}

const pts = (xy: [number, number][]) => xy.map(([x, y]) => `${x.toFixed(2)},${y.toFixed(2)}`).join(" ")

export function Field({ run, t, onInspect }: Props) {
  const now  = Math.round(t * 100)
  const cur  = frameAt(run, t)
  const hover = (title: string, detail: string) => ({
    onPointerEnter: () => onInspect({ title, detail }),
    onPointerLeave: () => onInspect(null),
  })
  const stacked = stacks(run, t)

  return (
    <svg className="field" viewBox="-74 -74 148 148">
      <g transform="scale(1 -1)">
        <Floor />

        {run.goals.map(([x, y, kind, name], i) => (
          <g key={name} transform={`translate(${x} ${y})`} {...hover(name, stackText(stacked[i]!.map(pieceName)))}>
            <rect className={`goal goal-${GOAL_KINDS[kind]}`} x={-GOAL_SIZE[kind]! / 2} y={-GOAL_SIZE[kind]! / 2}
              width={GOAL_SIZE[kind]} height={GOAL_SIZE[kind]} />
            <rect className="goal-hole" x={-1.2} y={-1.2} width={2.4} height={2.4} />
          </g>
        ))}

        {run.toggles.map(tg => {
          const face = latest(tg.faces, t)[1]
          const inward = Math.atan2(-tg.x, -tg.y) * 180 / Math.PI
          return (
            <g key={tg.name} transform={`translate(${tg.x} ${tg.y}) rotate(${-inward})`} {...hover(`${tg.name} Toggle`, `showing ${COLOURS[face]}`)}>
              <rect className={`toggle c-${COLOURS[face]}`} x={-13} y={0} width={26} height={2.4} />
            </g>
          )
        })}

        {run.pieces.map((p, i) => {
          const place = placeAt(p, t)
          const held  = place.where === 1 ? " held" : ""
          const h     = hover(pieceName(p), placeName(run, place))
          if (p.cup) return <circle key={i} className={`cup${held}`} cx={place.x} cy={place.y} r={1.58} {...h} />
          if (p.lying)
            return (
              <g key={i} transform={`translate(${place.x} ${place.y}) rotate(${-p.angle})`} {...h}>
                <rect className={`c-${COLOURS[p.top]}`} x={-1.5} y={0} width={3} height={3.2} />
                <rect className={`c-${COLOURS[p.bottom]}`} x={-1.5} y={-3.2} width={3} height={3.2} />
              </g>
            )
          return <circle key={i} className={`pin c-${COLOURS[p.top]}${held}`} cx={place.x} cy={place.y} r={1.2} {...h} />
        })}

        <Trail run={run} now={now} />

        {run.events.map(([time, text]) => {
          const f = frameAt(run, time)
          const kind = text.includes("TIMEOUT") ? "timeout" : text.startsWith("claw") ? "claw" : "motion"
          return <circle key={`${time}${text}`} className={`marker ${kind} ${time <= t ? "" : "future"}`} cx={f[1]} cy={f[2]} r={0.9}
            {...hover(text.split(" after ")[0]!, `${time.toFixed(2)} s`)} />
        })}

        <Robot run={run} x={cur[4]} y={cur[5]} heading={cur[6]} className="robot odom" />
        <Robot run={run} x={cur[1]} y={cur[2]} heading={cur[3]} className="robot" />
      </g>
    </svg>
  )
}

function stackText(names: string[]): string {
  return names.length ? `stack, bottom first: ${names.join(", ")}` : "empty"
}

// Static, so it never re-renders
const Floor = memo(function Floor() {
  const tiles = []
  for (let r = 0; r < 6; r++)
    for (let c = 0; c < 6; c++)
      tiles.push(<rect key={`${r}${c}`} className={(r + c) % 2 ? "tile alt" : "tile"} x={-HALF + c * TILE} y={-HALF + r * TILE} width={TILE} height={TILE} />)
  const m = TILE
  return (
    <>
      <rect className="wall" x={-HALF - 2} y={-HALF - 2} width={2 * HALF + 4} height={2 * HALF + 4} />
      {tiles}
      <line className="tape" x1={-HALF} y1={HALF} x2={HALF} y2={-HALF} />
      <polygon className="tape" points={pts([[m, 0], [0, m], [-m, 0], [0, -m]])} />
      <line className="tape faint" x1={-HALF} y1={-HALF} x2={HALF} y2={HALF} />
    </>
  )
})

// Where the robot has been (solid) and will go (faint), plus where odom thought it was
function Trail({ run, now }: { run: Run; now: number }) {
  const f = run.frames
  const past = f.slice(0, now + 1)
  return (
    <>
      <polyline className="trail future" points={pts(f.slice(now).map(r => [r[1], r[2]]))} />
      <polyline className="trail odom" points={pts(past.map(r => [r[4], r[5]]))} />
      <polyline className="trail" points={pts(past.map(r => [r[1], r[2]]))} />
    </>
  )
}

function Robot({ run, x, y, heading, className }: { run: Run; x: number; y: number; heading: number; className: string }) {
  const { width, front, back } = run.robot
  return (
    <g className={className} transform={`translate(${x} ${y}) rotate(${-heading})`}>
      <rect className="body" x={-width / 2} y={-back} width={width} height={front + back} />
      <line className="front" x1={-width / 2} y1={front} x2={width / 2} y2={front} />
    </g>
  )
}
