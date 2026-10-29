import { memo, useState } from "react"
import { COLOURS, GOAL_KINDS, pieceName, placeAt, placeName, stacks } from "./game.ts"
import { frameAt, latest, type Run } from "./sim.ts"

// Top-down field in LemLib's frame (inches, +y away from the audience). The flip below makes SVG
// y point up too, so LemLib headings (clockwise) become rotate(-heading).

const HALF = 70.2
const TILE = 2 * HALF / 6
const VIEW = 74
const GOAL_SIZE = [5.6, 5.6, 4.5, 6.5]	// [in] by kind

interface Tip { title: string; detail: string; x: number; y: number }

const pts = (xy: [number, number][]) => xy.map(([x, y]) => `${x.toFixed(2)},${y.toFixed(2)}`).join(" ")

export function Field({ run, t }: { run: Run; t: number }) {
  const [tip, setTip] = useState<Tip | null>(null)
  const cur = frameAt(run, t)
  const stacked = stacks(run, t)
  const hover = (title: string, detail: string, x: number, y: number) => ({
    onPointerEnter: () => setTip({ title, detail, x, y }),
    onPointerLeave: () => setTip(null),
  })

  return (
    <svg className="field" viewBox={`${-VIEW} ${-VIEW} ${2 * VIEW} ${2 * VIEW}`}>
      <g transform="scale(1 -1)">
        <Floor />

        {run.goals.map(([x, y, kind, name], i) => (
          <g key={name} transform={`translate(${x} ${y})`} {...hover(name, stackText(stacked[i]!.map(pieceName)), x, y)}>
            <rect className={`goal goal-${GOAL_KINDS[kind]}`} x={-GOAL_SIZE[kind]! / 2} y={-GOAL_SIZE[kind]! / 2}
              width={GOAL_SIZE[kind]} height={GOAL_SIZE[kind]} />
            <rect className="goal-hole" x={-1.2} y={-1.2} width={2.4} height={2.4} />
          </g>
        ))}

        {run.toggles.map(tg => {
          const face = latest(tg.faces, t)[1]
          const inward = Math.atan2(-tg.x, -tg.y) * 180 / Math.PI
          return (
            <g key={tg.name} transform={`translate(${tg.x} ${tg.y}) rotate(${-inward})`} {...hover(`${tg.name} Toggle`, `Showing ${COLOURS[face]}`, tg.x, tg.y)}>
              <rect className={`toggle c-${COLOURS[face]}`} x={-13} y={0} width={26} height={2.4} />
            </g>
          )
        })}

        <Trail run={run} now={Math.round(t * 100)} />

        {run.pieces.map((p, i) => {
          const place = placeAt(p, t)
          const held  = place.where === 1 ? " held" : ""
          const h     = hover(pieceName(p), placeName(run, place), place.x, place.y)
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

        <Robot run={run} x={cur[4]} y={cur[5]} heading={cur[6]} className="robot odom" />
        <Robot run={run} x={cur[1]} y={cur[2]} heading={cur[3]} className="robot" />
      </g>
      {tip && <Tooltip {...tip} />}
    </svg>
  )
}

function stackText(names: string[]): string {
  return names.length ? `Bottom to top: ${names.join(", ")}` : "Empty"
}

// Drawn unflipped, beside the thing hovered, kept inside the field
function Tooltip({ title, detail, x, y }: Tip) {
  const FONT = 3.4, PAD = 1.6
  const w = Math.max(title.length, detail.length) * FONT * 0.56 + 2 * PAD
  const h = 2 * FONT * 1.3 + 2 * PAD
  const left = Math.min(VIEW - w, Math.max(-VIEW, x + 3))
  const top  = Math.min(VIEW - h, Math.max(-VIEW, -y + 3))
  return (
    <g className="tip" transform={`translate(${left} ${top})`}>
      <rect width={w} height={h} />
      <text x={PAD} y={PAD + FONT}>{title}</text>
      <text className="detail" x={PAD} y={PAD + FONT * 2.3}>{detail}</text>
    </g>
  )
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
