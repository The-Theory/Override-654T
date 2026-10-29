import { useEffect, useMemo, useRef, useState, type PointerEvent } from "react"
import { Field } from "./Field.tsx"
import { coord, steps, type Step } from "./game.ts"
import { Sim, frameAt, latest, type Run } from "./sim.ts"

const SPEEDS = [0.5, 1, 2, 4]
const LIMITS = [15, 60]	// [s] auton, skills

const heading = (h: number) => `${(((h % 360) + 360) % 360).toFixed(0)}°`

export function App() {
  const [sim, setSim]         = useState<Sim | null>(null)
  const [autons, setAutons]   = useState<string[]>([])
  const [auton, setAuton]     = useState("")
  const [limit, setLimit]     = useState(15)
  const [run, setRun]         = useState<Run | null>(null)
  const [t, setT]             = useState(0)
  const [playing, setPlaying] = useState(true)
  const [speed, setSpeed]     = useState(1)
  const [status, setStatus]   = useState("Loading sim…")

  const end = run ? run.frames.at(-1)![0] : 0
  const clamp = (to: number) => Math.min(end, Math.max(0, to))
  const seek  = (to: number) => { setPlaying(false); setT(clamp(to)) }

  useEffect(() => {
    Sim.load("sim.js")
      .then(async s => {
        const names = await s.autons()
        setSim(s)
        setAutons(names)
        setAuton(names[0] ?? "")
      })
      .catch(() => setStatus("No sim.js: run `make -C tools/sim web`"))
  }, [])

  useEffect(() => {
    if (!sim || !auton) return
    void sim.run(auton, limit).then(r => {
      setRun(r)
      setT(0)
      setPlaying(true)
    })
  }, [sim, auton, limit])

  // Playback on wall-clock time, so speed holds whatever the frame rate
  useEffect(() => {
    if (!playing || !run) return
    let last: number | null = null	// rAF stamps can predate performance.now(), so start from the first one
    let frame = requestAnimationFrame(function tick(now) {
      const dt = last === null ? 0 : (now - last) / 1000 * speed
      last = now
      setT(prev => {
        const next = Math.min(end, prev + dt)
        if (next >= end) setPlaying(false)
        return next
      })
      frame = requestAnimationFrame(tick)
    })
    return () => cancelAnimationFrame(frame)
  }, [playing, speed, run, end])

  const togglePlay = () => {
    if (!playing && t >= end) setT(0)
    setPlaying(p => !p)
  }

  // Space plays/pauses; arrows step 0.1 s, or 1 s with shift
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.code === "Space") { e.preventDefault(); togglePlay() }
      if (e.code === "ArrowLeft" || e.code === "ArrowRight") {
        const step = (e.code === "ArrowLeft" ? -1 : 1) * (e.shiftKey ? 1 : 0.1)
        setPlaying(false)
        setT(prev => clamp(prev + step))	// Key repeat outruns renders, so build on the latest t
      }
    }
    window.addEventListener("keydown", onKey)
    return () => window.removeEventListener("keydown", onKey)
  })

  const log = useMemo(() => run ? steps(run) : [], [run])

  return (
    <div className="app">
      <header>
        <h1>Tide Sim</h1>
      </header>

      {run ? (
        <main>
          <nav className="card menu">
            <h2>Autons</h2>
            <ul>
              {autons.map(a => <li key={a}><button className={a === auton ? "on" : ""} onClick={() => setAuton(a)}>{a}</button></li>)}
            </ul>
            <h2>Time limit</h2>
            <Choice options={LIMITS} value={limit} onPick={setLimit} show={s => `${s} s`} />
          </nav>
          <section className="stage">
            <Score run={run} t={t} />
            <Field run={run} t={t} />
            <Readout run={run} t={t} />
            <div className="transport">
              <button className="play" onClick={togglePlay} aria-label={playing ? "Pause" : t >= end ? "Replay" : "Play"}>
                <PixelIcon rows={playing ? PAUSE : PLAY} />
              </button>
              <span className="num clock">{t.toFixed(1)} / {end.toFixed(1)} s</span>
              <Timeline log={log} t={t} end={end} onSeek={seek} />
              <Choice label="Speed" options={SPEEDS} value={speed} onPick={setSpeed} show={s => `${s}×`} />
            </div>
          </section>
          <aside>
            <StepList log={log} t={t} onSeek={seek} />
          </aside>
        </main>
      ) : <p className="status">{status}</p>}
    </div>
  )
}

////////////////////////////////////////////////////////////////
// Pieces

// Icons drawn on a pixel grid to sit with Silkscreen; # is a filled pixel
const PLAY  = ["#......", "###....", "#####..", "#######", "#####..", "###....", "#......"]
const PAUSE = ["##...##", "##...##", "##...##", "##...##", "##...##", "##...##", "##...##"]

function PixelIcon({ rows }: { rows: string[] }) {
  return (
    <svg className="icon" viewBox={`0 0 ${rows[0]!.length} ${rows.length}`} shapeRendering="crispEdges">
      {rows.flatMap((row, y) => [...row].map((c, x) => c === "#" && <rect key={`${x},${y}`} x={x} y={y} width={1} height={1} />))}
    </svg>
  )
}

function Choice<T extends string | number>({ label, options, value, onPick, show = String }: {
  label?: string; options: T[]; value: T; onPick: (v: T) => void; show?: (v: T) => string
}) {
  return (
    <div className="choice">
      {label && <span className="muted">{label}</span>}
      <div className="segmented">
        {options.map(o => <button key={o} className={o === value ? "on" : ""} onClick={() => onPick(o)}>{show(o)}</button>)}
      </div>
    </div>
  )
}

// Where the robot is under the playhead, with the legend for what's drawn
function Readout({ run, t }: { run: Run; t: number }) {
  const f = frameAt(run, t)
  const off = Math.hypot(f[1] - f[4], f[2] - f[5])
  return (
    <div className="readout">
      <span><i className="key robot" />Robot <span className="num">{coord(f[1])}, {coord(f[2])} · {heading(f[3])}</span></span>
      <span className={off > 1 ? "warn" : ""}><i className="key odom" />Odom off by <span className="num">{off.toFixed(1)} in</span></span>
    </div>
  )
}

// Motions as bars from start to end (red if they timed out), claw actions as yellow ticks
function Timeline({ log, t, end, onSeek }: { log: Step[]; t: number; end: number; onSeek: (t: number) => void }) {
  const at = (e: PointerEvent<SVGSVGElement>) => {
    const box = e.currentTarget.getBoundingClientRect()
    onSeek((e.clientX - box.left) / box.width * end)
  }
  const x = (time: number) => time / end * 1000
  return (
    <svg className="timeline" viewBox="0 0 1000 24" preserveAspectRatio="none"
      onPointerDown={e => { e.currentTarget.setPointerCapture(e.pointerId); at(e) }}
      onPointerMove={e => { if (e.buttons) at(e) }}>
      <rect className="track" width={1000} height={24} />
      {log.map((s, i) => s.claw
        ? <rect key={i} className="claw" x={x(s.start) - 1.5} y={4} width={3} height={16} />
        : <rect key={i} className={s.timeout ? "motion timeout" : "motion"} x={x(s.start) + 1} y={8} width={Math.max(2, x(s.end) - x(s.start) - 2)} height={8} />)}
      <line className="head" x1={x(t)} x2={x(t)} y1={0} y2={24} />
    </svg>
  )
}

// Score at the playhead
function Score({ run, t }: { run: Run; t: number }) {
  const [, red, blue] = latest(run.score, t)
  return <p className="score num"><span className="red">{red}</span> · <span className="blue">{blue}</span></p>
}

// Each motion and claw action in order; click one to jump to it
function StepList({ log, t, onSeek }: { log: Step[]; t: number; onSeek: (t: number) => void }) {
  const current = log.findLastIndex(s => s.start <= t)
  const row = useRef<HTMLLIElement>(null)
  useEffect(() => { row.current?.scrollIntoView({ block: "nearest" }) }, [current])
  return (
    <section className="card steps">
      <h2>Steps</h2>
      <ol>
        {log.map((s, i) => (
          <li key={i} ref={i === current ? row : undefined} onClick={() => onSeek(s.start)}
            className={[i === current ? "current" : "", s.start > t ? "future" : "", s.claw ? "claw" : ""].join(" ")}>
            <span className="num muted">{s.start.toFixed(1)}</span>
            <span>{s.what}</span>
            {s.timeout ? <span className="warn">Timed out</span>
              : s.early ? <span className="muted">Handed off</span>
              : <span className="num muted">{s.claw ? "" : `${(s.end - s.start).toFixed(1)} s`}</span>}
          </li>
        ))}
      </ol>
    </section>
  )
}
