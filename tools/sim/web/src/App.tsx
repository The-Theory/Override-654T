import { useEffect, useRef, useState, type PointerEvent } from "react"
import { Field, type Inspect } from "./Field.tsx"
import { COLOURS, coord, pieceName, stacks } from "./game.ts"
import { Sim, frameAt, latest, type Run } from "./sim.ts"

const SPEEDS = [0.25, 0.5, 1, 2, 4]
const LENGTHS = [15, 60]	// [s] auton, skills

const heading = (h: number) => `${(((h % 360) + 360) % 360).toFixed(0)}°`

export function App() {
  const [sim, setSim]         = useState<Sim | null>(null)
  const [autons, setAutons]   = useState<string[]>([])
  const [auton, setAuton]     = useState("")
  const [length, setLength]   = useState(15)
  const [run, setRun]         = useState<Run | null>(null)
  const [t, setT]             = useState(0)
  const [playing, setPlaying] = useState(true)
  const [speed, setSpeed]     = useState(1)
  const [inspect, setInspect] = useState<Inspect | null>(null)
  const [status, setStatus]   = useState("Loading sim…")

  const end = run ? run.frames.at(-1)![0] : 0

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
    void sim.run(auton, length).then(r => {
      setRun(r)
      setT(0)
      setPlaying(true)
    })
  }, [sim, auton, length])

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
  const clamp = (to: number) => Math.min(end, Math.max(0, to))
  const seek  = (to: number) => setT(clamp(to))

  // Space plays/pauses; arrows step 0.1 s, or 1 s with shift
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.code === "Space") { e.preventDefault(); togglePlay() }
      if (e.code === "ArrowLeft" || e.code === "ArrowRight") {
        setPlaying(false)
        const step = (e.code === "ArrowLeft" ? -1 : 1) * (e.shiftKey ? 1 : 0.1)
        setT(prev => clamp(prev + step))	// Key repeat outruns renders, so build on the latest t
      }
    }
    window.addEventListener("keydown", onKey)
    return () => window.removeEventListener("keydown", onKey)
  })

  return (
    <div className="app">
      <header>
        <h1>Tide Sim</h1>
        <div className="group">
          {autons.map(a => <button key={a} className={a === auton ? "on" : ""} onClick={() => setAuton(a)}>{a}</button>)}
        </div>
        <div className="group">
          {LENGTHS.map(s => <button key={s} className={s === length ? "on" : ""} onClick={() => setLength(s)}>{s} s</button>)}
        </div>
        {run && (
          <>
            <button className="play" onClick={togglePlay}>{playing ? "Pause" : "Play"}</button>
            <div className="group">
              {SPEEDS.map(s => <button key={s} className={s === speed ? "on" : ""} onClick={() => setSpeed(s)}>{s}×</button>)}
            </div>
            <span className="time num">{t.toFixed(2)} / {end.toFixed(2)} s</span>
          </>
        )}
      </header>

      {run ? (
        <main>
          <section className="stage">
            <Field run={run} t={t} onInspect={setInspect} />
            <Timeline run={run} t={t} end={end} onSeek={to => { setPlaying(false); seek(to) }} />
          </section>
          <aside>
            <Readouts run={run} t={t} />
            <Score run={run} t={t} />
            <section className="card inspect">
              {inspect ? <><h2>{inspect.title}</h2><p>{inspect.detail}</p></> : <p className="dim">Hover a piece, Goal, Toggle or marker</p>}
            </section>
            <Log run={run} t={t} onSeek={to => { setPlaying(false); seek(to) }} />
          </aside>
        </main>
      ) : <p className="status">{status}</p>}
    </div>
  )
}

////////////////////////////////////////////////////////////////
// Panels

function Timeline({ run, t, end, onSeek }: { run: Run; t: number; end: number; onSeek: (t: number) => void }) {
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
      <rect className="done" width={x(t)} height={24} />
      {run.events.map(([time, text]) => (
        <line key={`${time}${text}`} className={text.includes("TIMEOUT") ? "tick timeout" : text.startsWith("claw") ? "tick claw" : "tick"}
          x1={x(time)} x2={x(time)} y1={4} y2={20} />
      ))}
      <line className="head" x1={x(t)} x2={x(t)} y1={0} y2={24} />
    </svg>
  )
}

function Readouts({ run, t }: { run: Run; t: number }) {
  const f = frameAt(run, t)
  const error = Math.hypot(f[1] - f[4], f[2] - f[5])
  return (
    <section className="card">
      <table className="num">
        <thead><tr><th /><th>x</th><th>y</th><th>heading</th></tr></thead>
        <tbody>
          <tr><th className="robot-key">Robot</th><td>{coord(f[1])}</td><td>{coord(f[2])}</td><td>{heading(f[3])}</td></tr>
          <tr><th className="odom-key">Odom</th><td>{coord(f[4])}</td><td>{coord(f[5])}</td><td>{heading(f[6])}</td></tr>
        </tbody>
      </table>
      <p className={error > 1 ? "warn" : "dim"}>Odom off by {error.toFixed(2)} in</p>
    </section>
  )
}

function Score({ run, t }: { run: Run; t: number }) {
  const [, red, blue] = latest(run.score, t)
  const all = stacks(run, t)
  return (
    <section className="card">
      <h2>Score <span className="red num">{red}</span> <span className="blue num">{blue}</span></h2>
      <ul className="stacks">
        {run.goals.map(([, , , name], i) => all[i]!.length > 0 && (
          <li key={name}>
            <span>{name}</span>
            <span className="dots">
              {all[i]!.map((p, k) => <i key={k} className={p.cup ? "dot cup" : `dot c-${COLOURS[p.top]}`} title={pieceName(p)} />)}
            </span>
          </li>
        ))}
      </ul>
    </section>
  )
}

// Motions and claw actions; follows playback, click one to jump there
function Log({ run, t, onSeek }: { run: Run; t: number; onSeek: (t: number) => void }) {
  const last = run.events.findLastIndex(([time]) => time <= t)
  const current = useRef<HTMLLIElement>(null)
  useEffect(() => { current.current?.scrollIntoView({ block: "nearest" }) }, [last])
  return (
    <section className="card log">
      <ol>
        {run.events.map(([time, text], i) => {
          const [what, rest] = text.split(" after ")
          const cls = [i <= last ? "" : "future", i === last ? "current" : "", text.includes("TIMEOUT") ? "warn" : ""].join(" ")
          return (
            <li key={`${time}${text}`} ref={i === last ? current : undefined} className={cls} onClick={() => onSeek(time)}>
              <span className="num">{time.toFixed(2)}</span>
              <span>{what}{rest && <span className="dim"> after {rest}</span>}</span>
            </li>
          )
        })}
      </ol>
    </section>
  )
}
