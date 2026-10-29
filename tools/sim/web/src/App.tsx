import { useEffect, useMemo, useRef, useState, type PointerEvent, type ReactNode } from "react"
import { Field } from "./Field.tsx"
import { coord, holding, steps, summarize, type Step } from "./game.ts"
import { Sim, frameAt, latest, type Run } from "./sim.ts"

const SPEEDS  = [0.5, 1, 2, 4]
const LIMITS  = [15, 60]	// [s] auton, skills

const seconds = (t: number) => `${t.toFixed(2)} s`
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
        <Choice label="Auton" options={autons} value={auton} onPick={setAuton} />
        <Choice label="Time limit" options={LIMITS} value={limit} onPick={setLimit} show={s => `${s} s`} />
      </header>

      {run ? (
        <main>
          <section className="stage">
            <Field run={run} t={t} />
            <Legend />
            <div className="transport">
              <button className="play" onClick={togglePlay}>{playing ? "Pause" : t >= end ? "Replay" : "Play"}</button>
              <span className="num clock">{t.toFixed(2)} / {seconds(end)}</span>
              <Timeline log={log} t={t} end={end} onSeek={seek} />
              <Choice label="Speed" options={SPEEDS} value={speed} onPick={setSpeed} show={s => `${s}×`} />
            </div>
          </section>
          <aside>
            <RunPanel run={run} />
            <NowPanel run={run} log={log} t={t} />
            <LogPanel log={log} t={t} onSeek={seek} />
          </aside>
        </main>
      ) : <p className="status">{status}</p>}
    </div>
  )
}

////////////////////////////////////////////////////////////////
// Building blocks

function Choice<T extends string | number>({ label, options, value, onPick, show = String }: {
  label: string; options: T[]; value: T; onPick: (v: T) => void; show?: (v: T) => string
}) {
  return (
    <div className="choice">
      <span className="label">{label}</span>
      <div className="buttons">
        {options.map(o => <button key={o} className={o === value ? "on" : ""} onClick={() => onPick(o)}>{show(o)}</button>)}
      </div>
    </div>
  )
}

function Panel({ title, className = "", children }: { title: string; className?: string; children: ReactNode }) {
  return (
    <section className={`panel ${className}`}>
      <h2>{title}</h2>
      {children}
    </section>
  )
}

// Key / value rows; a third true marks the row as a problem
function Rows({ rows }: { rows: [string, ReactNode, boolean?][] }) {
  return (
    <dl>
      {rows.map(([key, value, warn]) => (
        <div key={key} className={warn ? "warn" : ""}><dt>{key}</dt><dd>{value}</dd></div>
      ))}
    </dl>
  )
}

// "A ×2, B" from ["A", "A", "B"]
function countEach(names: string[]): string {
  const counts = new Map<string, number>()
  for (const n of names) counts.set(n, (counts.get(n) ?? 0) + 1)
  return [...counts].map(([n, c]) => c > 1 ? `${n} ×${c}` : n).join(", ")
}

const Score = ({ red, blue }: { red: number; blue: number }) =>
  <span className="num"><span className="red">Red {red}</span> · <span className="blue">Blue {blue}</span></span>

////////////////////////////////////////////////////////////////
// Panels

function Legend() {
  return (
    <div className="legend">
      <span><i className="key robot" />Robot</span>
      <span><i className="key odom" />Where odom thinks it is</span>
      <span><i className="key ahead" />Path still to come</span>
      <span className="dim">Hover anything on the field for details</span>
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
    <svg className="timeline" viewBox="0 0 1000 28" preserveAspectRatio="none"
      onPointerDown={e => { e.currentTarget.setPointerCapture(e.pointerId); at(e) }}
      onPointerMove={e => { if (e.buttons) at(e) }}>
      <rect className="track" width={1000} height={28} />
      {log.map((s, i) => s.claw
        ? <rect key={i} className="claw" x={x(s.start) - 1.5} y={18} width={3} height={8} />
        : <rect key={i} className={s.timeout ? "motion timeout" : "motion"} x={x(s.start)} y={3} width={Math.max(2, x(s.end) - x(s.start) - 2)} height={12} />)}
      <line className="head" x1={x(t)} x2={x(t)} y1={0} y2={28} />
    </svg>
  )
}

// The whole run at a glance
function RunPanel({ run }: { run: Run }) {
  const s = useMemo(() => summarize(run), [run])
  const claw = [`picked up ${s.picked}`, `stacked ${s.stacked.length}`, `dropped ${s.dropped}`].join(", ")
  return (
    <Panel title={`Run: ${run.auton}`}>
      <Rows rows={[
        ["Result", s.finished ? `Finished at ${seconds(s.end)}` : `Still running at the ${s.limit} s limit`, !s.finished],
        ["Score", <Score red={s.red} blue={s.blue} />],
        ["Motions", `${s.motions}, ${s.timeouts} timed out`, s.timeouts > 0],
        ["Claw", claw],
        ["Stacked on", s.stacked.length ? countEach(s.stacked) : "Nothing"],
        ["Toggles", s.flips.length ? s.flips.join(", ") : "None flipped"],
        ["Odom at end", `${s.odomOff.toFixed(2)} in off`, s.odomOff > 1],
      ]} />
    </Panel>
  )
}

// The moment under the playhead
function NowPanel({ run, log, t }: { run: Run; log: Step[]; t: number }) {
  const f = frameAt(run, t)
  const off = Math.hypot(f[1] - f[4], f[2] - f[5])
  const [, red, blue] = latest(run.score, t)
  const motion = log.find(s => !s.claw && s.start <= t && t < s.end)
  return (
    <Panel title={`At ${seconds(t)}`}>
      <table className="num">
        <thead><tr><th /><th>x</th><th>y</th><th>heading</th></tr></thead>
        <tbody>
          <tr><th>Robot</th><td>{coord(f[1])}</td><td>{coord(f[2])}</td><td>{heading(f[3])}</td></tr>
          <tr><th>Odom</th><td>{coord(f[4])}</td><td>{coord(f[5])}</td><td>{heading(f[6])}</td></tr>
        </tbody>
      </table>
      <Rows rows={[
        ["Odom off by", `${off.toFixed(2)} in`, off > 1],
        ["Running", motion ? `${motion.action}, ${(t - motion.start).toFixed(2)} of ${seconds(motion.end - motion.start)}` : "No motion"],
        ["Claw holds", holding(run, t)],
        ["Score", <Score red={red} blue={blue} />],
      ]} />
    </Panel>
  )
}

// Every motion and claw action; the one under the playhead is highlighted, click one to jump to its start
function LogPanel({ log, t, onSeek }: { log: Step[]; t: number; onSeek: (t: number) => void }) {
  const current = log.findLastIndex(s => s.start <= t)
  const row = useRef<HTMLTableRowElement>(null)
  useEffect(() => { row.current?.scrollIntoView({ block: "nearest" }) }, [current])
  return (
    <Panel title="Log" className="log">
      <div className="scroll">
        <table>
          <thead><tr><th>Start</th><th>Action</th><th>Result</th><th>Took</th></tr></thead>
          <tbody>
            {log.map((s, i) => (
              <tr key={i} ref={i === current ? row : undefined} onClick={() => onSeek(s.start)}
                className={[i === current ? "current" : "", s.start > t ? "future" : "", s.timeout ? "warn" : ""].join(" ")}>
                <td className="num">{s.start.toFixed(2)}</td>
                <td>{s.action}</td>
                <td>{s.result}</td>
                <td className="num">{s.claw ? "" : seconds(s.end - s.start)}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </Panel>
  )
}
