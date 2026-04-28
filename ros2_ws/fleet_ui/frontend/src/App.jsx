import React, { useState, useEffect, useRef, useCallback } from 'react'

const API = '/api'

const ROBOT_PROFILES = [
  { id: 'custom', label: 'Custom (simulação local)', type: 'local' },
  { id: 'tb3_1',  label: 'TurtleBot3 #1',           type: 'ssh', host: '192.168.1.101' },
  { id: 'tb3_2',  label: 'TurtleBot3 #2',           type: 'ssh', host: '192.168.1.102' },
]

const ALL_TOPICS = ['scan', 'odom', 'imu', 'pose']

const DEFAULT_CFG = {
  command:       'record',
  robot:         'default',
  route:         'percurso_initial',
  collect:       true,
  topics:        ['scan', 'odom', 'imu', 'pose'],
  initial_pose:  [0, 0, 0],
  points:        [[0.5, 0.0, 0.0], [1.0, 0.0, 0.0]],
  return_to_start: [0, 0, 0],
}

// ── Helpers de estilo ────────────────────────────────────────────────────────
const S = {
  label:   { fontSize: '0.72rem', color: '#8b92a8', textTransform: 'uppercase', letterSpacing: '0.05em', marginBottom: '0.3rem', display: 'block' },
  input:   { background: '#0d0f14', border: '1px solid #2a3142', borderRadius: '6px', color: '#e6e9ef', padding: '0.4rem 0.6rem', fontSize: '0.85rem', outline: 'none', fontFamily: 'monospace', width: '100%', boxSizing: 'border-box' },
  numInput:{ background: '#0d0f14', border: '1px solid #2a3142', borderRadius: '6px', color: '#e6e9ef', padding: '0.4rem 0.5rem', fontSize: '0.82rem', outline: 'none', fontFamily: 'monospace', width: '70px', textAlign: 'center' },
  select:  { background: '#0d0f14', border: '1px solid #2a3142', borderRadius: '6px', color: '#e6e9ef', padding: '0.4rem 0.6rem', fontSize: '0.85rem', outline: 'none', width: '100%', boxSizing: 'border-box' },
  section: { background: '#161a22', border: '1px solid #2a3142', borderRadius: '8px', padding: '0.85rem 1rem', display: 'flex', flexDirection: 'column', gap: '0.65rem' },
  row:     { display: 'flex', gap: '0.75rem', alignItems: 'flex-start' },
  field:   { display: 'flex', flexDirection: 'column', flex: 1 },
}

function btn(bg, color, disabled = false) {
  return { background: bg, color, border: `1px solid ${color}`, borderRadius: '8px', padding: '0.45rem 0.9rem', fontSize: '0.85rem', fontWeight: 600, cursor: disabled ? 'not-allowed' : 'pointer', opacity: disabled ? 0.55 : 1, fontFamily: 'inherit' }
}

// ── Componente: toggle Record / Replay ───────────────────────────────────────
function CmdToggle({ value, onChange }) {
  const base = { padding: '0.4rem 1.2rem', fontSize: '0.85rem', fontWeight: 600, border: 'none', cursor: 'pointer', fontFamily: 'inherit' }
  return (
    <div style={{ display: 'flex', borderRadius: '8px', overflow: 'hidden', border: '1px solid #2a3142', alignSelf: 'flex-start' }}>
      {['record', 'replay'].map(cmd => (
        <button key={cmd} onClick={() => onChange(cmd)}
          style={{ ...base, background: value === cmd ? (cmd === 'record' ? '#065f46' : '#1e1b4b') : '#161a22', color: value === cmd ? (cmd === 'record' ? '#6ee7b7' : '#a5b4fc') : '#8b92a8' }}>
          {cmd === 'record' ? '⏺  Gravar' : '▶  Reproduzir'}
        </button>
      ))}
    </div>
  )
}

// ── Componente: lista de waypoints ───────────────────────────────────────────
function WaypointList({ points, onChange }) {
  const update = (i, j, val) => {
    const next = points.map((p, pi) => pi === i ? p.map((v, vi) => vi === j ? parseFloat(val) || 0 : v) : p)
    onChange(next)
  }
  const add    = () => onChange([...points, [0, 0, 0]])
  const remove = i  => onChange(points.filter((_, pi) => pi !== i))

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '0.4rem' }}>
      {points.map((p, i) => (
        <div key={i} style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
          <span style={{ fontSize: '0.75rem', color: '#4b5563', width: '20px', textAlign: 'right' }}>#{i + 1}</span>
          {['x', 'y', 'yaw'].map((lbl, j) => (
            <div key={lbl} style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '0.1rem' }}>
              <span style={{ fontSize: '0.65rem', color: '#4b5563' }}>{lbl}</span>
              <input type="number" step="0.1" value={p[j]}
                onChange={e => update(i, j, e.target.value)}
                style={S.numInput} />
            </div>
          ))}
          <button onClick={() => remove(i)} style={{ ...btn('#3a1a1a', '#f87171'), padding: '0.2rem 0.5rem', fontSize: '0.8rem', marginTop: '0.9rem' }}>✕</button>
        </div>
      ))}
      <button onClick={add} style={{ ...btn('#161a22', '#6366f1'), padding: '0.3rem 0.75rem', fontSize: '0.8rem', alignSelf: 'flex-start', marginTop: '0.2rem' }}>
        + Waypoint
      </button>
    </div>
  )
}

// ── Componente: inputs XYYaw ─────────────────────────────────────────────────
function XYYaw({ value, onChange, label }) {
  const update = (i, val) => {
    const next = [...value]; next[i] = parseFloat(val) || 0; onChange(next)
  }
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '0.3rem' }}>
      {label && <span style={S.label}>{label}</span>}
      <div style={{ display: 'flex', gap: '0.5rem', alignItems: 'center' }}>
        {['x', 'y', 'yaw'].map((lbl, i) => (
          <div key={lbl} style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '0.1rem' }}>
            <span style={{ fontSize: '0.65rem', color: '#4b5563' }}>{lbl}</span>
            <input type="number" step="0.1" value={value[i]}
              onChange={e => update(i, e.target.value)}
              style={S.numInput} />
          </div>
        ))}
      </div>
    </div>
  )
}

// ── Formulário de configuração ────────────────────────────────────────────────
function ConfigForm({ cfg, onChange, currentPose }) {
  const set = (key, val) => onChange({ ...cfg, [key]: val })

  const usarPoseAtual = () => {
    if (!currentPose?.valid) return
    set('initial_pose', [
      parseFloat(currentPose.x.toFixed(3)),
      parseFloat(currentPose.y.toFixed(3)),
      parseFloat(currentPose.yaw.toFixed(3)),
    ])
  }

  const toggleTopic = (t) => {
    const next = cfg.topics.includes(t) ? cfg.topics.filter(x => x !== t) : [...cfg.topics, t]
    set('topics', next)
  }

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>

      {/* Comando */}
      <div style={S.section}>
        <div style={S.row}>
          <div style={S.field}>
            <span style={S.label}>Comando</span>
            <CmdToggle value={cfg.command} onChange={v => set('command', v)} />
          </div>
          <div style={S.field}>
            <span style={S.label}>Robô</span>
            <select value={cfg.robot} onChange={e => set('robot', e.target.value)} style={S.select}>
              <option value="default">Único (default)</option>
              <option value="tb1">tb1</option>
              <option value="tb2">tb2</option>
            </select>
          </div>
          <div style={S.field}>
            <span style={S.label}>Nome da rota</span>
            <input value={cfg.route} onChange={e => set('route', e.target.value)}
              placeholder="percurso1" style={S.input} />
          </div>
        </div>
      </div>

      {/* Coleta e tópicos */}
      <div style={S.section}>
        <div style={S.row}>
          <label style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', cursor: 'pointer' }}>
            <input type="checkbox" checked={cfg.collect} onChange={e => set('collect', e.target.checked)}
              style={{ accentColor: '#6ee7b7', width: 16, height: 16 }} />
            <span style={{ fontSize: '0.85rem', color: cfg.collect ? '#6ee7b7' : '#8b92a8', fontWeight: 600 }}>
              Gravar bag
            </span>
          </label>
        </div>

        {cfg.collect && (
          <div>
            <span style={S.label}>Tópicos</span>
            <div style={{ display: 'flex', gap: '0.6rem', flexWrap: 'wrap' }}>
              {ALL_TOPICS.map(t => (
                <label key={t} style={{ display: 'flex', alignItems: 'center', gap: '0.35rem', cursor: 'pointer',
                  background: cfg.topics.includes(t) ? 'rgba(99,102,241,0.12)' : 'transparent',
                  border: `1px solid ${cfg.topics.includes(t) ? '#6366f1' : '#2a3142'}`,
                  borderRadius: '6px', padding: '0.3rem 0.65rem' }}>
                  <input type="checkbox" checked={cfg.topics.includes(t)} onChange={() => toggleTopic(t)}
                    style={{ accentColor: '#6366f1', margin: 0 }} />
                  <span style={{ fontSize: '0.82rem', color: cfg.topics.includes(t) ? '#e6e9ef' : '#8b92a8', fontFamily: 'monospace' }}>
                    {t}
                  </span>
                </label>
              ))}
            </div>
          </div>
        )}
      </div>

      {/* Pose inicial */}
      <div style={S.section}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '0.1rem' }}>
          <span style={S.label}>Pose inicial (initialpose)</span>
          <div style={{ display: 'flex', alignItems: 'center', gap: '0.75rem' }}>
            {currentPose?.valid && (
              <span style={{ fontSize: '0.72rem', color: '#4b5563', fontFamily: 'monospace' }}>
                atual: ({currentPose.x.toFixed(2)}, {currentPose.y.toFixed(2)}, {(currentPose.yaw * 180 / Math.PI).toFixed(1)}°)
              </span>
            )}
            <button onClick={usarPoseAtual} disabled={!currentPose?.valid}
              style={{ ...btn('#161a22', currentPose?.valid ? '#6366f1' : '#2a3142', !currentPose?.valid), fontSize: '0.72rem', padding: '0.2rem 0.55rem' }}>
              ⊕ Usar pose atual
            </button>
          </div>
        </div>
        <XYYaw value={cfg.initial_pose} onChange={v => set('initial_pose', v)} />
      </div>

      {/* Waypoints (record) ou Retornar ao início (replay) */}
      <div style={S.section}>
        {cfg.command === 'record' ? (
          <>
            <span style={S.label}>Waypoints</span>
            <WaypointList points={cfg.points} onChange={v => set('points', v)} />
          </>
        ) : (
          <XYYaw label="Retornar ao início antes do replay" value={cfg.return_to_start} onChange={v => set('return_to_start', v)} />
        )}
      </div>

    </div>
  )
}

// ── App principal ─────────────────────────────────────────────────────────────
export default function App() {
  const [cfg, setCfg]             = useState(DEFAULT_CFG)
  const [jobId, setJobId]         = useState(null)
  const [job, setJob]             = useState(null)
  const [running, setRunning]     = useState(false)
  const [error, setError]         = useState(null)
  const [showResult, setShowResult] = useState(false)
  const [status, setStatus]       = useState({ robots: [], pose: { x: 0, y: 0, yaw: 0, valid: false } })
  const [resetMsg, setResetMsg]   = useState(null)
  const [resetting, setResetting] = useState(false)

  const [connPanel, setConnPanel]             = useState(false)
  const [selectedProfile, setSelectedProfile] = useState('custom')
  const [connStatus, setConnStatus]           = useState('idle')
  const [connMsg, setConnMsg]                 = useState('')
  const [discoverPanel, setDiscoverPanel]     = useState(false)
  const [discovering, setDiscovering]         = useState(false)
  const [discovered, setDiscovered]           = useState([])
  const [discoverError, setDiscoverError]     = useState(null)
  const [subnet, setSubnet]                   = useState('')
  const [testingSSH, setTestingSSH]           = useState({})
  const [sshUser, setSshUser]                 = useState('ubuntu')
  const [extraProfiles, setExtraProfiles]     = useState([])

  const outputRef   = useRef(null)
  const pollRef     = useRef(null)
  const panelRef    = useRef(null)
  const discoverRef = useRef(null)

  useEffect(() => {
    const handler = e => {
      if (panelRef.current && !panelRef.current.contains(e.target))    setConnPanel(false)
      if (discoverRef.current && !discoverRef.current.contains(e.target)) setDiscoverPanel(false)
    }
    document.addEventListener('mousedown', handler)
    return () => document.removeEventListener('mousedown', handler)
  }, [])

  useEffect(() => {
    const t = setInterval(() =>
      fetch(`${API}/status`).then(r => r.json()).then(d => d && setStatus(d)).catch(() => {}), 300)
    return () => clearInterval(t)
  }, [])

  const startPolling = useCallback(id => {
    if (pollRef.current) clearInterval(pollRef.current)
    pollRef.current = setInterval(async () => {
      const r = await fetch(`${API}/job/${id}`).catch(() => null)
      if (!r) return
      const data = await r.json().catch(() => null)
      if (!data) return
      setJob(data)
      if (!data.running) { clearInterval(pollRef.current); setRunning(false) }
    }, 500)
  }, [])

  useEffect(() => {
    if (outputRef.current) outputRef.current.scrollTop = outputRef.current.scrollHeight
  }, [job?.lines?.length])

  const connectRobot = async () => {
    const profile = [...ROBOT_PROFILES, ...extraProfiles].find(p => p.id === selectedProfile)
    if (!profile) return
    if (profile.type === 'ssh') { setConnStatus('error'); setConnMsg(`SSH para ${profile.host} ainda não implementado.`); return }
    setConnStatus('connecting'); setConnMsg('')
    try {
      const r = await fetch(`${API}/status`, { signal: AbortSignal.timeout(4000) })
      await r.json()
      if (r.ok) { setConnStatus('connected'); setConnMsg('Backend respondendo.'); setConnPanel(false) }
      else throw new Error(`HTTP ${r.status}`)
    } catch (e) { setConnStatus('error'); setConnMsg(`Backend não respondeu: ${e.message}`) }
  }

  const discoverRobots = async () => {
    setDiscovering(true); setDiscovered([]); setDiscoverError(null)
    try {
      const url = subnet.trim() ? `${API}/discover_robots?subnet=${encodeURIComponent(subnet.trim())}` : `${API}/discover_robots`
      const r = await fetch(url, { signal: AbortSignal.timeout(40000) })
      if (!r.ok) { setDiscoverError(`Erro HTTP ${r.status}`); return }
      const data = await r.json()
      if (data.error) { setDiscoverError(data.error); return }
      setDiscovered(data.found || [])
      if (!data.found?.length) setDiscoverError(`Nenhum host encontrado em ${data.subnet_scanned || subnet || '(auto)'}.0/24`)
    } catch (e) { setDiscoverError(`Erro: ${e.message}`) }
    finally { setDiscovering(false) }
  }

  const testSSH = async ip => {
    setTestingSSH(prev => ({ ...prev, [ip]: 'testing' }))
    try {
      const r = await fetch(`${API}/test_ssh`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ host: ip, user: sshUser }) })
      const data = await r.json()
      const st = data.success && data.has_ros2 ? 'ok' : data.success ? 'no_ros' : 'error'
      setTestingSSH(prev => ({ ...prev, [ip]: st }))
      if (st === 'ok') setExtraProfiles(prev => prev.find(p => p.host === ip) ? prev : [...prev, { id: `ssh_${ip}`, label: `SSH ${ip} (${sshUser})`, type: 'ssh', host: ip, user: sshUser }])
    } catch { setTestingSSH(prev => ({ ...prev, [ip]: 'error' })) }
  }

  const run = async () => {
    setError(null)
    if (!cfg.route.trim()) { setError('Nome da rota não pode estar vazio.'); return }
    if (cfg.command === 'record' && cfg.points.length === 0) { setError('Adiciona pelo menos 1 waypoint.'); return }
    if (cfg.collect && cfg.topics.length === 0) { setError('Seleciona pelo menos 1 tópico.'); return }

    setRunning(true); setJob(null); setJobId(null)
    const r = await fetch(`${API}/run_config`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(cfg),
    }).catch(e => { setRunning(false); setError(`Erro: ${e.message}`); return null })
    if (!r) return
    const data = await r.json().catch(() => null)
    if (!data?.job_id) { setRunning(false); setError(data?.message || 'Erro ao iniciar job'); return }
    setJobId(data.job_id)
    startPolling(data.job_id)
  }

  const resetToOrigin = async () => {
    setResetting(true); setResetMsg(null)
    try {
      const r = await fetch(`${API}/go_to_point?x=0&y=0&yaw=0`, { method: 'POST' })
      const data = await r.json()
      setResetMsg(data.success ? 'ok' : (data.message || 'erro'))
    } catch (e) { setResetMsg(`erro: ${e.message}`) }
    finally { setResetting(false); setTimeout(() => setResetMsg(null), 6000) }
  }

  const pose   = status.pose || {}
  const robot  = status.robots?.[0] || {}
  const nav2Ok = status.nav2_ready === true
  const isConnected = connStatus === 'connected'

  const lineColor = line => {
    if (line.includes('[OK]'))     return '#6ee7b7'
    if (line.includes('[FALHOU]')) return '#f87171'
    if (line.includes('[TRACE]'))  return '#8b92a8'
    if (line.includes('Resumo'))   return '#fbbf24'
    if (line.includes('==='))      return '#93c5fd'
    return '#e6e9ef'
  }

  const connLight = { idle: '#4b5563', connecting: '#fbbf24', connected: '#6ee7b7', error: '#f87171' }[connStatus]
  const deg = r => (r * 180 / Math.PI).toFixed(1)

  return (
    <div style={{ fontFamily: 'system-ui, sans-serif', background: '#0d0f14', color: '#e6e9ef', height: '100vh', display: 'flex', flexDirection: 'column', overflow: 'hidden' }}>

      {/* ── Header ──────────────────────────────────────────────────────────── */}
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', padding: '0.5rem 1.25rem', borderBottom: '1px solid #2a3142', flexShrink: 0 }}>
        <h1 style={{ margin: 0, fontSize: '1.1rem', color: '#6ee7b7', fontWeight: 700 }}>Fleet UI</h1>
        <div style={{ display: 'flex', gap: '0', fontSize: '0.78rem', fontFamily: 'monospace', alignItems: 'stretch', background: '#161a22', border: '1px solid #2a3142', borderRadius: '8px', overflow: 'hidden' }}>
          {[
            { label: 'Nav2',  value: nav2Ok ? 'ON' : 'OFF',         color: nav2Ok ? '#6ee7b7' : '#f87171' },
            { label: 'Nav',   value: robot.nav_state || '—',         color: robot.nav_state === 'navigating' ? '#6ee7b7' : robot.nav_state === 'failed' ? '#f87171' : '#e6e9ef' },
            { label: 'Coleta',value: robot.collection_on ? 'ON' : 'OFF', color: robot.collection_on ? '#6ee7b7' : '#8b92a8' },
            { label: 'x',     value: pose.valid ? pose.x.toFixed(3) : '—', color: pose.valid ? '#93c5fd' : '#8b92a8' },
            { label: 'y',     value: pose.valid ? pose.y.toFixed(3) : '—', color: pose.valid ? '#93c5fd' : '#8b92a8' },
            { label: 'yaw',   value: pose.valid ? `${deg(pose.yaw)}°` : '—', color: pose.valid ? '#93c5fd' : '#8b92a8' },
          ].map(({ label, value, color }) => (
            <div key={label} style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', padding: '0.3rem 0.7rem', borderRight: '1px solid #2a3142' }}>
              <span style={{ fontSize: '0.62rem', color: '#4b5563', textTransform: 'uppercase', letterSpacing: '0.04em' }}>{label}</span>
              <span style={{ color, fontWeight: 600, marginTop: '0.05rem' }}>{value}</span>
            </div>
          ))}
          <button onClick={resetToOrigin} disabled={resetting} title="Envia robô para (0,0,0)"
            style={{ background: 'none', border: 'none', cursor: resetting ? 'not-allowed' : 'pointer', padding: '0.3rem 0.75rem', color: resetMsg === 'ok' ? '#6ee7b7' : resetMsg ? '#f87171' : '#6366f1', fontWeight: 700, fontSize: '0.8rem', opacity: resetting ? 0.6 : 1 }}>
            {resetting ? '⏳' : resetMsg === 'ok' ? '✓' : resetMsg ? '✗' : '⟳ Origem'}
          </button>
        </div>
      </div>

      {/* ── Barra de conexão ───────────────────────────────────────────────── */}
      <div style={{ display: 'flex', gap: '0.75rem', alignItems: 'center', padding: '0.4rem 1.25rem', borderBottom: '1px solid #2a3142', flexShrink: 0, position: 'relative' }}>

        <div ref={panelRef} style={{ position: 'relative' }}>
          <button onClick={() => { setConnPanel(v => !v); setDiscoverPanel(false) }}
            style={{ ...btn('#161a22', '#6366f1'), display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
            <span style={{ width: 10, height: 10, borderRadius: '50%', background: connLight, display: 'inline-block', boxShadow: connStatus === 'connected' ? `0 0 6px ${connLight}` : 'none' }} />
            Conectar robô <span style={{ fontSize: '0.7rem', opacity: 0.7 }}>{connPanel ? '▲' : '▼'}</span>
          </button>
          {connPanel && (
            <div style={{ position: 'absolute', top: '2.5rem', left: 0, zIndex: 100, background: '#161a22', border: '1px solid #2a3142', borderRadius: '10px', padding: '1rem', width: '340px', boxShadow: '0 8px 32px rgba(0,0,0,0.5)' }}>
              <div style={{ fontSize: '0.78rem', color: '#8b92a8', marginBottom: '0.75rem', textTransform: 'uppercase', letterSpacing: '0.05em' }}>Selecionar robô</div>
              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem', marginBottom: '1rem' }}>
                {[...ROBOT_PROFILES, ...extraProfiles].map(p => {
                  const isSelected = selectedProfile === p.id
                  const isExtra    = !!extraProfiles.find(ep => ep.id === p.id)
                  return (
                    <label key={p.id} onClick={() => setSelectedProfile(p.id)}
                      style={{ display: 'flex', alignItems: 'center', gap: '0.75rem', padding: '0.6rem 0.85rem', borderRadius: '8px', border: `1px solid ${isSelected ? '#6366f1' : '#2a3142'}`, background: isSelected ? 'rgba(99,102,241,0.1)' : 'transparent', cursor: 'pointer' }}>
                      <input type="radio" name="robot_profile" checked={isSelected} onChange={() => setSelectedProfile(p.id)} style={{ accentColor: '#6366f1', margin: 0 }} />
                      <div style={{ flex: 1 }}>
                        <div style={{ fontSize: '0.85rem', color: isSelected ? '#e6e9ef' : '#a0aec0', fontWeight: isSelected ? 600 : 400 }}>{p.label}</div>
                        <div style={{ fontSize: '0.72rem', color: '#4b5563', marginTop: '0.1rem' }}>
                          {isExtra ? `SSH ${p.host} · descoberto` : p.type === 'ssh' ? `SSH ${p.host} · em breve` : 'Backend local · localhost:8000'}
                        </div>
                      </div>
                      {isExtra && <span style={{ fontSize: '0.68rem', background: '#1a3a2a', color: '#6ee7b7', padding: '0.15rem 0.4rem', borderRadius: '4px' }}>ROS 2 ✓</span>}
                    </label>
                  )
                })}
              </div>
              <div style={{ display: 'flex', gap: '0.75rem' }}>
                <button onClick={connectRobot} disabled={connStatus === 'connecting'} style={{ ...btn('#065f46', '#6ee7b7'), flex: 1 }}>
                  {connStatus === 'connecting' ? '⏳ Conectando…' : 'Conectar'}
                </button>
                {connStatus === 'connected' && (
                  <button onClick={() => { setConnStatus('idle'); setConnMsg(''); setConnPanel(false) }} style={btn('#3a1a1a', '#f87171')}>Desconectar</button>
                )}
              </div>
              {connMsg && <div style={{ marginTop: '0.6rem', fontSize: '0.78rem', color: connStatus === 'error' ? '#f87171' : '#6ee7b7' }}>{connMsg}</div>}
            </div>
          )}
        </div>

        <div ref={discoverRef} style={{ position: 'relative' }}>
          <button onClick={() => { setDiscoverPanel(v => !v); setConnPanel(false) }}
            style={{ ...btn('#161a22', '#fbbf24'), display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
            🔍 Procurar robô <span style={{ fontSize: '0.7rem', opacity: 0.7 }}>{discoverPanel ? '▲' : '▼'}</span>
          </button>
          {discoverPanel && (
            <div style={{ position: 'absolute', top: '2.5rem', left: 0, zIndex: 100, background: '#161a22', border: '1px solid #2a3142', borderRadius: '10px', padding: '1rem', width: '400px', boxShadow: '0 8px 32px rgba(0,0,0,0.5)' }}>
              <div style={{ fontSize: '0.78rem', color: '#8b92a8', marginBottom: '0.75rem', textTransform: 'uppercase', letterSpacing: '0.05em' }}>Descoberta na rede</div>
              <div style={{ display: 'flex', gap: '0.5rem', marginBottom: '0.75rem' }}>
                <input placeholder="Subnet (ex: 192.168.1) — auto se vazio" value={subnet} onChange={e => setSubnet(e.target.value)} style={{ ...S.input, flex: 1 }} />
                <input placeholder="user SSH" value={sshUser} onChange={e => setSshUser(e.target.value)} style={{ ...S.input, width: '90px' }} />
              </div>
              <button onClick={discoverRobots} disabled={discovering} style={{ ...btn(discovering ? '#1a2a1a' : '#161a22', '#fbbf24'), width: '100%', marginBottom: '0.75rem' }}>
                {discovering ? '⏳ Varrendo…' : '▶ Iniciar varredura'}
              </button>
              {discoverError && <div style={{ color: '#f87171', fontSize: '0.78rem', marginBottom: '0.5rem' }}>{discoverError}</div>}
              {discovered.length > 0 && (
                <div style={{ display: 'flex', flexDirection: 'column', gap: '0.4rem', maxHeight: '220px', overflowY: 'auto' }}>
                  {discovered.map(h => {
                    const st = testingSSH[h.ip]
                    const stColor = st === 'ok' ? '#6ee7b7' : st === 'no_ros' ? '#fbbf24' : st === 'error' ? '#f87171' : '#8b92a8'
                    const stLabel = st === 'testing' ? '⏳' : st === 'ok' ? '✓ ROS 2' : st === 'no_ros' ? '⚠ sem ROS' : st === 'error' ? '✗ falhou' : 'Testar SSH'
                    return (
                      <div key={h.ip} style={{ display: 'flex', alignItems: 'center', gap: '0.6rem', background: '#0d0f14', borderRadius: '6px', padding: '0.45rem 0.7rem', border: `1px solid ${h.likely_robot ? '#2a3a2a' : '#2a3142'}` }}>
                        <span style={{ width: 8, height: 8, borderRadius: '50%', background: h.likely_robot ? '#6ee7b7' : '#4b5563', display: 'inline-block', flexShrink: 0 }} />
                        <div style={{ flex: 1, minWidth: 0 }}>
                          <div style={{ fontSize: '0.82rem', color: '#e6e9ef', fontFamily: 'monospace' }}>{h.ip}</div>
                          <div style={{ fontSize: '0.7rem', color: '#4b5563', overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>{h.hostname}</div>
                        </div>
                        {h.likely_robot && <span style={{ fontSize: '0.68rem', color: '#6ee7b7', background: '#1a3a2a', padding: '0.1rem 0.35rem', borderRadius: '4px', flexShrink: 0 }}>robô?</span>}
                        <button onClick={() => testSSH(h.ip)} disabled={st === 'testing'} style={{ ...btn('#161a22', stColor), fontSize: '0.72rem', padding: '0.25rem 0.5rem', flexShrink: 0 }}>{stLabel}</button>
                      </div>
                    )
                  })}
                </div>
              )}
            </div>
          )}
        </div>

        {isConnected && <span style={{ fontSize: '0.8rem', color: '#6ee7b7', fontFamily: 'monospace' }}>● Conectado — {[...ROBOT_PROFILES, ...extraProfiles].find(p => p.id === selectedProfile)?.label}</span>}
        {connStatus === 'error' && <span style={{ fontSize: '0.8rem', color: '#f87171', fontFamily: 'monospace' }}>✗ {connMsg}</span>}
      </div>

      {/* ── Layout principal ───────────────────────────────────────────────── */}
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '1rem', flex: 1, minHeight: 0, padding: '0.75rem 1.25rem 1rem', overflow: 'hidden' }}>

        {/* Coluna esquerda: formulário */}
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem', overflowY: 'auto', paddingRight: '0.25rem' }}>

          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.8rem', fontWeight: 600, color: '#6ee7b7', textTransform: 'uppercase', letterSpacing: '0.05em' }}>Configuração</span>
          </div>

          <ConfigForm cfg={cfg} onChange={setCfg} currentPose={pose} />

          {/* Avisos */}
          {!isConnected && (
            <div style={{ color: '#fbbf24', fontSize: '0.82rem', padding: '0.5rem 0.75rem', background: 'rgba(251,191,36,0.08)', borderRadius: '6px', border: '1px solid rgba(251,191,36,0.2)' }}>
              ⚠ Conecte um robô antes de executar.
            </div>
          )}
          {isConnected && !nav2Ok && (
            <div style={{ color: '#fbbf24', fontSize: '0.82rem', padding: '0.5rem 0.75rem', background: 'rgba(251,191,36,0.08)', borderRadius: '6px', border: '1px solid rgba(251,191,36,0.2)' }}>
              ⚠ Nav2 parece offline — verifique o terminal de simulação.
            </div>
          )}
          {error && (
            <div style={{ color: '#f87171', fontSize: '0.82rem', padding: '0.5rem 0.75rem', background: 'rgba(248,113,113,0.1)', borderRadius: '6px' }}>{error}</div>
          )}

          {/* Botões */}
          <div style={{ display: 'flex', gap: '0.75rem' }}>
            <button onClick={run} disabled={running || !isConnected}
              style={btn(running || !isConnected ? '#1a3a2a' : '#065f46', '#6ee7b7', running || !isConnected)}>
              {running ? '⏳ A executar…' : `▶ Executar ${cfg.command}`}
            </button>
            {running && (
              <button onClick={() => { if (pollRef.current) clearInterval(pollRef.current); setRunning(false) }}
                style={btn('#3a1a1a', '#f87171')}>■ Parar polling</button>
            )}
          </div>
        </div>

        {/* Coluna direita: output */}
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem', minHeight: 0 }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.8rem', fontWeight: 600, color: '#8b92a8', textTransform: 'uppercase', letterSpacing: '0.05em' }}>
              Output {jobId && <span style={{ color: '#3b82f6', fontWeight: 400 }}>#{jobId}</span>}
            </span>
            {job && !job.running && (
              <span style={{ fontSize: '0.82rem', color: job.exit_code === 0 ? '#6ee7b7' : '#f87171', fontWeight: 600 }}>
                {job.exit_code === 0 ? '✓ Sucesso' : `✗ Falhou (exit ${job.exit_code})`}
              </span>
            )}
          </div>

          <div ref={outputRef} style={{ flex: 1, background: '#161a22', border: '1px solid #2a3142', borderRadius: '8px', padding: '0.75rem 1rem', overflowY: 'auto', fontFamily: 'JetBrains Mono, Consolas, monospace', fontSize: '0.78rem', lineHeight: 1.7 }}>
            {!job && !running && <span style={{ color: '#8b92a8' }}>Aguardando execução…</span>}
            {job?.lines?.map((line, i) => (
              <div key={i} style={{ color: lineColor(line), whiteSpace: 'pre-wrap', wordBreak: 'break-word' }}>{line}</div>
            ))}
            {running && <div style={{ color: '#8b92a8', animation: 'blink 1s step-end infinite' }}>▌</div>}
          </div>

          {/* Métricas */}
          {job?.result?.bag_metrics && Object.keys(job.result.bag_metrics).length > 0 && (() => {
            const m = job.result.bag_metrics
            const dur  = m.wall_duration_s ?? m.duration_s
            const path = m.odom_path_length_m ?? m.theoretical_path_m
            const pathLabel = m.odom_path_length_m != null ? 'Percurso (odom)' : 'Percurso (teórico)'
            const cards = [
              dur  != null && { label: 'Duração',    value: `${dur} s`,   color: '#93c5fd' },
              path != null && { label: pathLabel,    value: `${path} m`,  color: '#6ee7b7' },
              m.odom_avg_speed_ms != null && { label: 'Vel. média', value: `${m.odom_avg_speed_ms} m/s`, color: '#6ee7b7' },
              m.scan_avg_valid_points != null && { label: 'Scan pts',    value: `${m.scan_avg_valid_points}`,    color: '#fbbf24' },
              m.imu_accel_mean_ms2   != null && { label: 'IMU accel',   value: `${m.imu_accel_mean_ms2} m/s²`, color: '#c4b5fd' },
            ].filter(Boolean)
            return (
              <div style={{ background: '#161a22', border: '1px solid #2a3142', borderRadius: '8px', padding: '0.75rem 1rem' }}>
                <div style={{ fontSize: '0.75rem', color: '#8b92a8', textTransform: 'uppercase', letterSpacing: '0.05em', marginBottom: '0.6rem' }}>Métricas da gravação</div>
                <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: '0.5rem' }}>
                  {cards.map(({ label, value, color }) => (
                    <div key={label} style={{ background: '#0d0f14', borderRadius: '6px', padding: '0.45rem 0.6rem' }}>
                      <div style={{ fontSize: '0.68rem', color: '#8b92a8', marginBottom: '0.15rem' }}>{label}</div>
                      <div style={{ fontSize: '0.85rem', fontWeight: 700, color, fontFamily: 'JetBrains Mono, monospace' }}>{value}</div>
                    </div>
                  ))}
                </div>
              </div>
            )
          })()}

          {job?.result?.sensor_summary && Object.keys(job.result.sensor_summary).length > 0 && (
            <div style={{ background: '#161a22', border: '1px solid #2a3142', borderRadius: '8px', padding: '0.75rem 1rem' }}>
              <div style={{ fontSize: '0.75rem', color: '#8b92a8', textTransform: 'uppercase', letterSpacing: '0.05em', marginBottom: '0.5rem' }}>Sensores gravados</div>
              <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.78rem', fontFamily: 'JetBrains Mono, Consolas, monospace' }}>
                <thead><tr>{['Tópico', 'Msgs', 'Hz est.'].map(h => <th key={h} style={{ textAlign: 'left', color: '#8b92a8', fontWeight: 500, paddingBottom: '0.3rem', borderBottom: '1px solid #2a3142' }}>{h}</th>)}</tr></thead>
                <tbody>
                  {Object.entries(job.result.sensor_summary).map(([topic, stat]) => (
                    <tr key={topic}>
                      <td style={{ color: '#93c5fd', padding: '0.2rem 0.5rem 0.2rem 0' }}>{topic}</td>
                      <td style={{ color: stat.msgs > 0 ? '#6ee7b7' : '#f87171', padding: '0.2rem 0.5rem' }}>{stat.msgs}</td>
                      <td style={{ color: '#e6e9ef', padding: '0.2rem 0' }}>{stat.hz_est != null ? `~${stat.hz_est} Hz` : '—'}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}

          {job?.result && (
            <button onClick={() => setShowResult(true)} style={{ ...btn('#161a22', '#2a3142'), fontSize: '0.78rem', width: '100%', textAlign: 'left' }}>
              ▸ Ver resultado JSON
            </button>
          )}
          {job?.error && (
            <div style={{ color: '#f87171', fontSize: '0.82rem', padding: '0.5rem 0.75rem', background: 'rgba(248,113,113,0.1)', borderRadius: '6px' }}>
              Erro interno: {job.error}
            </div>
          )}
        </div>
      </div>

      {/* Modal JSON */}
      {showResult && job?.result && (
        <div onClick={() => setShowResult(false)}
          style={{ position: 'fixed', inset: 0, zIndex: 200, background: 'rgba(0,0,0,0.75)', display: 'flex', alignItems: 'center', justifyContent: 'center', padding: '2rem' }}>
          <div onClick={e => e.stopPropagation()}
            style={{ background: '#161a22', border: '1px solid #2a3142', borderRadius: '12px', width: '100%', maxWidth: '860px', maxHeight: '85vh', display: 'flex', flexDirection: 'column', overflow: 'hidden', boxShadow: '0 16px 48px rgba(0,0,0,0.7)' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', padding: '0.85rem 1.25rem', borderBottom: '1px solid #2a3142' }}>
              <span style={{ fontSize: '0.82rem', fontWeight: 600, color: '#8b92a8', textTransform: 'uppercase', letterSpacing: '0.05em' }}>Resultado JSON</span>
              <button onClick={() => setShowResult(false)} style={{ background: 'none', border: 'none', color: '#8b92a8', cursor: 'pointer', fontSize: '1.1rem' }}>✕</button>
            </div>
            <pre style={{ flex: 1, overflowY: 'auto', overflowX: 'auto', margin: 0, padding: '1.25rem', fontFamily: 'JetBrains Mono, Consolas, monospace', fontSize: '0.82rem', lineHeight: 1.65, color: '#e6e9ef' }}>
              {JSON.stringify(job.result, null, 2)}
            </pre>
          </div>
        </div>
      )}

      <style>{`@keyframes blink { 0%,100%{opacity:1} 50%{opacity:0} }`}</style>
    </div>
  )
}
