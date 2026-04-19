import React, { useState, useEffect, useRef, useCallback } from 'react'

const API = '/api'

const ROBOT_PROFILES = [
  { id: 'custom', label: 'Custom (simulação local)', type: 'local' },
  { id: 'tb3_1',  label: 'TurtleBot3 #1',           type: 'ssh', host: '192.168.1.101' },
  { id: 'tb3_2',  label: 'TurtleBot3 #2',           type: 'ssh', host: '192.168.1.102' },
]

const makeExampleRecord = (robot) => JSON.stringify({
  command: "record", robot,
  route: "percurso1", collect: true,
  topics: ["scan", "odom", "imu", "pose"],
  initial_pose: [0, 0, 0],
  points: [[0.5,0,0],[1.0,0,0],[1.5,0.5,0],[2.0,0.5,0]]
}, null, 2)

const makeExampleReplay = (robot) => JSON.stringify({
  command: "replay", robot,
  route: "percurso1", collect: true,
  topics: ["scan", "odom", "imu", "pose"],
  initial_pose: [0, 0, 0],
  return_to_start: [0, 0, 0]
}, null, 2)

// ─────────────────────────────────────────────────────────────────────────────
// RobotPanel — painel independente por robô
// ─────────────────────────────────────────────────────────────────────────────
function RobotPanel({ robotId, isConnected, onViewResult, compact }) {
  const [config, setConfig]       = useState(makeExampleRecord(robotId))
  const [jobId, setJobId]         = useState(null)
  const [job, setJob]             = useState(null)
  const [running, setRunning]     = useState(false)
  const [parseError, setParseError] = useState(null)
  const outputRef = useRef(null)
  const pollRef   = useRef(null)

  const startPolling = useCallback((id) => {
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

  const run = async () => {
    setParseError(null)
    let cfg
    try { cfg = JSON.parse(config) }
    catch (e) { setParseError(`JSON inválido: ${e.message}`); return }
    setRunning(true); setJob(null); setJobId(null)
    const r = await fetch(`${API}/run_config`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(cfg),
    }).catch(e => { setRunning(false); setParseError(`Erro: ${e.message}`); return null })
    if (!r) return
    const data = await r.json().catch(() => null)
    if (!data?.job_id) { setRunning(false); setParseError(data?.message || 'Erro ao iniciar job'); return }
    setJobId(data.job_id); startPolling(data.job_id)
  }

  const stop = () => { if (pollRef.current) clearInterval(pollRef.current); setRunning(false) }

  const lineColor = (line) => {
    if (line.includes('[OK]'))     return '#6ee7b7'
    if (line.includes('[FALHOU]')) return '#f87171'
    if (line.includes('[TRACE]'))  return '#8b92a8'
    if (line.includes('Resumo'))   return '#fbbf24'
    if (line.includes('==='))      return '#93c5fd'
    return '#e6e9ef'
  }

  const accent = robotId === 'tb1' ? '#6366f1' : robotId === 'tb2' ? '#f59e0b' : '#6366f1'
  const fs = compact ? '0.75rem' : '0.85rem'

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem', minHeight: 0, flex: 1 }}>

      {/* Cabeçalho do painel */}
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        <span style={{ fontSize: '0.8rem', fontWeight: 700, color: accent, fontFamily: 'JetBrains Mono, monospace', textTransform: 'uppercase', letterSpacing: '0.08em' }}>
          {robotId || 'Robot'}
          {jobId && <span style={{ color: '#3b82f6', fontWeight: 400, marginLeft: '0.5rem' }}>#{jobId}</span>}
        </span>
        <div style={{ display: 'flex', gap: '0.4rem' }}>
          <button onClick={() => setConfig(makeExampleRecord(robotId))} style={btnStyle('#161a22', '#2a3142', '0.72rem')}>record</button>
          <button onClick={() => setConfig(makeExampleReplay(robotId))} style={btnStyle('#161a22', '#2a3142', '0.72rem')}>replay</button>
        </div>
      </div>

      {/* Config JSON */}
      <textarea
        value={config}
        onChange={e => { setConfig(e.target.value); setParseError(null) }}
        spellCheck={false}
        style={{
          flex: '0 0 auto', height: compact ? '160px' : '200px',
          background: '#161a22',
          border: `1px solid ${parseError ? '#f87171' : '#2a3142'}`,
          borderRadius: '8px', color: '#e6e9ef',
          fontFamily: 'JetBrains Mono, Consolas, monospace',
          fontSize: fs, padding: '0.75rem', resize: 'none', outline: 'none', lineHeight: 1.5,
        }}
      />

      {parseError && (
        <div style={{ color: '#f87171', fontSize: '0.75rem', padding: '0.4rem 0.6rem', background: 'rgba(248,113,113,0.08)', borderRadius: '6px' }}>
          {parseError}
        </div>
      )}

      {/* Botões */}
      <div style={{ display: 'flex', gap: '0.5rem' }}>
        <button onClick={run} disabled={running || !isConnected}
          title={!isConnected ? 'Conecte primeiro' : ''}
          style={btnStyle(running || !isConnected ? '#1a3a2a' : '#065f46', '#6ee7b7', fs, running || !isConnected)}>
          {running ? '⏳ Executando…' : '▶ Executar'}
        </button>
        {running && (
          <button onClick={stop} style={btnStyle('#3a1a1a', '#f87171', fs)}>■ Parar</button>
        )}
        {job && !job.running && (
          <span style={{ fontSize: '0.75rem', color: job.exit_code === 0 ? '#6ee7b7' : '#f87171', fontWeight: 600, alignSelf: 'center' }}>
            {job.exit_code === 0 ? '✓ Sucesso' : `✗ Falhou`}
          </span>
        )}
      </div>

      {/* Output log */}
      <div ref={outputRef} style={{
        flex: 1, background: '#161a22', border: '1px solid #2a3142', borderRadius: '8px',
        padding: '0.6rem 0.85rem', overflowY: 'auto',
        fontFamily: 'JetBrains Mono, Consolas, monospace', fontSize: '0.73rem', lineHeight: 1.65,
        minHeight: compact ? '80px' : '120px',
      }}>
        {!job && !running && <span style={{ color: '#4b5563' }}>Aguardando execução…</span>}
        {job?.lines?.map((line, i) => (
          <div key={i} style={{ color: lineColor(line), whiteSpace: 'pre-wrap', wordBreak: 'break-word' }}>{line}</div>
        ))}
        {running && <div style={{ color: '#4b5563', animation: 'blink 1s step-end infinite' }}>▌</div>}
      </div>

      {/* Métricas + sensor summary + ver JSON */}
      {job?.result && (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.4rem' }}>
          {job.result.bag_metrics && Object.keys(job.result.bag_metrics).length > 0 && (() => {
            const m = job.result.bag_metrics
            const dur  = m.wall_duration_s ?? m.duration_s
            const path = m.amcl_path_length_m ?? m.pose_path_length_m ?? m.tf_path_length_m ?? m.odom_path_length_m
            const speed = m.amcl_avg_speed_ms ?? m.pose_avg_speed_ms ?? m.odom_avg_speed_ms
            const cards = [
              dur   != null && { l: 'Duração',   v: `${dur} s`,       c: '#93c5fd' },
              path  != null && { l: 'Percurso',  v: `${path} m`,      c: '#6ee7b7' },
              speed != null && { l: 'Vel. média', v: `${speed} m/s`,   c: '#6ee7b7' },
              m.scan_avg_valid_points  != null && { l: 'Scan pts', v: `${m.scan_avg_valid_points}`,   c: '#fbbf24' },
              m.imu_accel_variance_ms2 != null && { l: 'IMU var',  v: `${m.imu_accel_variance_ms2}`, c: '#c4b5fd' },
            ].filter(Boolean)
            return (
              <div style={{ display: 'grid', gridTemplateColumns: `repeat(${Math.min(cards.length, 3)}, 1fr)`, gap: '0.35rem' }}>
                {cards.map(({ l, v, c }) => (
                  <div key={l} style={{ background: '#0d0f14', borderRadius: '6px', padding: '0.35rem 0.5rem', border: '1px solid #2a3142' }}>
                    <div style={{ fontSize: '0.62rem', color: '#4b5563' }}>{l}</div>
                    <div style={{ fontSize: '0.78rem', fontWeight: 700, color: c, fontFamily: 'JetBrains Mono, monospace' }}>{v}</div>
                  </div>
                ))}
              </div>
            )
          })()}

          {job.result.sensor_summary && Object.keys(job.result.sensor_summary).filter(t => job.result.sensor_summary[t].msgs > 0).length > 0 && (
            <div style={{ fontSize: '0.7rem', color: '#4b5563', fontFamily: 'monospace' }}>
              {Object.entries(job.result.sensor_summary).filter(([,s]) => s.msgs > 0).map(([t, s]) =>
                `${t.split('/').pop()}: ${s.msgs} msgs`
              ).join('  ·  ')}
            </div>
          )}

          <button onClick={() => onViewResult(job.result)}
            style={{ ...btnStyle('#161a22', '#2a3142', '0.72rem'), textAlign: 'left' }}>
            ▸ Ver resultado JSON
          </button>
        </div>
      )}
    </div>
  )
}

// ─────────────────────────────────────────────────────────────────────────────
// App principal
// ─────────────────────────────────────────────────────────────────────────────
export default function App() {
  const [status, setStatus]         = useState({ robots: [], pose: { x: 0, y: 0, yaw: 0, valid: false } })
  const [resetMsg, setResetMsg]     = useState(null)
  const [resetting, setResetting]   = useState(false)
  const [multiMode, setMultiMode]   = useState(false)
  const [modalResult, setModalResult] = useState(null)

  // ── Conexão ────────────────────────────────────────────────────────
  const [connPanel, setConnPanel]       = useState(false)
  const [selectedProfile, setSelectedProfile] = useState('custom')
  const [connStatus, setConnStatus]     = useState('idle')
  const [connMsg, setConnMsg]           = useState('')

  // ── Descoberta ────────────────────────────────────────────────────
  const [discoverPanel, setDiscoverPanel] = useState(false)
  const [discovering, setDiscovering]     = useState(false)
  const [discovered, setDiscovered]       = useState([])
  const [discoverError, setDiscoverError] = useState(null)
  const [subnet, setSubnet]               = useState('')
  const [testingSSH, setTestingSSH]       = useState({})
  const [sshUser, setSshUser]             = useState('ubuntu')
  const [extraProfiles, setExtraProfiles] = useState([])

  const panelRef    = useRef(null)
  const discoverRef = useRef(null)

  useEffect(() => {
    const handler = (e) => {
      if (panelRef.current && !panelRef.current.contains(e.target)) setConnPanel(false)
      if (discoverRef.current && !discoverRef.current.contains(e.target)) setDiscoverPanel(false)
    }
    document.addEventListener('mousedown', handler)
    return () => document.removeEventListener('mousedown', handler)
  }, [])

  useEffect(() => {
    const t = setInterval(() =>
      fetch(`${API}/status`).then(r => r.json()).then(d => d && setStatus(d)).catch(() => {}), 1000)
    return () => clearInterval(t)
  }, [])

  const connectRobot = async () => {
    const profile = ROBOT_PROFILES.find(p => p.id === selectedProfile)
    if (!profile) return
    if (profile.type === 'ssh' && !extraProfiles.find(ep => ep.id === selectedProfile)) {
      setConnStatus('error'); setConnMsg(`SSH para ${profile.host} ainda não implementado.`); return
    }
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
      const found = data.found || []
      setDiscovered(found)
      if (!found.length) setDiscoverError(`Nenhum host com SSH encontrado em ${data.subnet_scanned || subnet || '(auto)'}.0/24`)
    } catch (e) { setDiscoverError(`Erro: ${e.message}`) }
    finally { setDiscovering(false) }
  }

  const testSSH = async (ip) => {
    setTestingSSH(prev => ({ ...prev, [ip]: 'testing' }))
    try {
      const r = await fetch(`${API}/test_ssh`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ host: ip, user: sshUser }) })
      const data = await r.json()
      const st = data.success && data.has_ros2 ? 'ok' : data.success ? 'no_ros' : 'error'
      setTestingSSH(prev => ({ ...prev, [ip]: st }))
      if (st === 'ok') setExtraProfiles(prev => prev.find(p => p.host === ip) ? prev : [...prev, { id: `ssh_${ip}`, label: `SSH ${ip} (${sshUser})`, type: 'ssh', host: ip, user: sshUser }])
    } catch { setTestingSSH(prev => ({ ...prev, [ip]: 'error' })) }
  }

  const resetToOrigin = async () => {
    setResetting(true); setResetMsg(null)
    try {
      const r = await fetch(`${API}/go_to_point?x=0&y=0&yaw=0`, { method: 'POST' })
      const data = await r.json()
      setResetMsg(data.success ? 'ok' : 'erro')
    } catch { setResetMsg('erro') }
    finally { setResetting(false); setTimeout(() => setResetMsg(null), 3000) }
  }

  const pose  = status.pose || {}
  const robot = status.robots?.[0] || {}
  const deg   = r => (r * 180 / Math.PI).toFixed(1)
  const connLight = { idle: '#4b5563', connecting: '#fbbf24', connected: '#6ee7b7', error: '#f87171' }[connStatus]
  const isConnected = connStatus === 'connected'

  return (
    <div style={{ fontFamily: 'system-ui, sans-serif', background: '#0d0f14', color: '#e6e9ef', height: '100vh', display: 'flex', flexDirection: 'column', overflow: 'hidden' }}>

      {/* ── Header ── */}
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', padding: '0.5rem 1.25rem', borderBottom: '1px solid #2a3142', flexShrink: 0 }}>
        <h1 style={{ margin: 0, fontSize: '1.1rem', color: '#6ee7b7', fontWeight: 700 }}>Fleet UI</h1>
        <div style={{ display: 'flex', gap: '0', fontSize: '0.78rem', fontFamily: 'monospace', alignItems: 'stretch', background: '#161a22', border: '1px solid #2a3142', borderRadius: '8px', overflow: 'hidden' }}>
          {[
            { label: 'Nav', value: robot.nav_state || '—', color: robot.nav_state === 'navigating' ? '#6ee7b7' : robot.nav_state === 'failed' ? '#f87171' : '#e6e9ef' },
            { label: 'Coleta', value: robot.collection_on ? 'ON' : 'OFF', color: robot.collection_on ? '#6ee7b7' : '#8b92a8' },
            { label: 'x', value: pose.valid ? pose.x.toFixed(2) : '—', color: pose.valid ? '#93c5fd' : '#8b92a8' },
            { label: 'y', value: pose.valid ? pose.y.toFixed(2) : '—', color: pose.valid ? '#93c5fd' : '#8b92a8' },
            { label: 'yaw', value: pose.valid ? `${deg(pose.yaw)}°` : '—', color: pose.valid ? '#93c5fd' : '#8b92a8' },
          ].map(({ label, value, color }) => (
            <div key={label} style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', padding: '0.3rem 0.7rem', borderRight: '1px solid #2a3142' }}>
              <span style={{ fontSize: '0.6rem', color: '#4b5563', textTransform: 'uppercase', letterSpacing: '0.04em' }}>{label}</span>
              <span style={{ color, fontWeight: 600 }}>{value}</span>
            </div>
          ))}
          <button onClick={resetToOrigin} disabled={resetting} title="Envia robô para (0,0,0)"
            style={{ background: 'none', border: 'none', cursor: resetting ? 'not-allowed' : 'pointer', padding: '0.3rem 0.75rem', color: resetMsg === 'ok' ? '#6ee7b7' : resetMsg ? '#f87171' : '#6366f1', fontWeight: 700, fontSize: '0.8rem', opacity: resetting ? 0.6 : 1 }}>
            {resetting ? '⏳' : resetMsg === 'ok' ? '✓' : resetMsg ? '✗' : '⟳ Origem'}
          </button>
        </div>
      </div>

      {/* ── Barra de conexão + modo ── */}
      <div style={{ display: 'flex', gap: '0.75rem', alignItems: 'center', padding: '0.4rem 1.25rem', borderBottom: '1px solid #2a3142', flexShrink: 0 }}>

        {/* Conectar robô */}
        <div ref={panelRef} style={{ position: 'relative' }}>
          <button onClick={() => { setConnPanel(v => !v); setDiscoverPanel(false) }}
            style={{ ...btnStyle('#161a22', '#6366f1'), display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
            <span style={{ width: 9, height: 9, borderRadius: '50%', background: connLight, display: 'inline-block', boxShadow: connStatus === 'connected' ? `0 0 6px ${connLight}` : 'none' }} />
            Conectar robô
            <span style={{ fontSize: '0.65rem', opacity: 0.7 }}>{connPanel ? '▲' : '▼'}</span>
          </button>
          {connPanel && (
            <div style={{ position: 'absolute', top: '2.4rem', left: 0, zIndex: 100, background: '#161a22', border: '1px solid #2a3142', borderRadius: '10px', padding: '1rem', width: '340px', boxShadow: '0 8px 32px rgba(0,0,0,0.5)' }}>
              <div style={{ fontSize: '0.72rem', color: '#8b92a8', marginBottom: '0.6rem', textTransform: 'uppercase', letterSpacing: '0.05em' }}>Selecionar perfil</div>
              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.4rem', marginBottom: '0.75rem' }}>
                {[...ROBOT_PROFILES, ...extraProfiles].map(p => {
                  const isSSH = p.type === 'ssh' && !extraProfiles.find(ep => ep.id === p.id)
                  const isSel = selectedProfile === p.id
                  const isExtra = !!extraProfiles.find(ep => ep.id === p.id)
                  return (
                    <label key={p.id} onClick={() => setSelectedProfile(p.id)}
                      style={{ display: 'flex', alignItems: 'center', gap: '0.6rem', padding: '0.5rem 0.75rem', borderRadius: '7px', border: `1px solid ${isSel ? '#6366f1' : '#2a3142'}`, background: isSel ? 'rgba(99,102,241,0.1)' : 'transparent', cursor: 'pointer' }}>
                      <input type="radio" name="rp" checked={isSel} onChange={() => setSelectedProfile(p.id)} style={{ accentColor: '#6366f1', margin: 0 }} />
                      <div style={{ flex: 1 }}>
                        <div style={{ fontSize: '0.82rem', color: isSel ? '#e6e9ef' : '#a0aec0', fontWeight: isSel ? 600 : 400 }}>{p.label}</div>
                        <div style={{ fontSize: '0.68rem', color: '#4b5563' }}>{isExtra ? `SSH ${p.host} · descoberto` : p.type === 'ssh' ? `SSH ${p.host} · em breve` : 'localhost:8000'}</div>
                      </div>
                      {isSSH && !isExtra && <span style={{ fontSize: '0.65rem', background: '#1f2a3a', color: '#6b7280', padding: '0.1rem 0.35rem', borderRadius: '4px' }}>em breve</span>}
                      {isExtra && <span style={{ fontSize: '0.65rem', background: '#1a3a2a', color: '#6ee7b7', padding: '0.1rem 0.35rem', borderRadius: '4px' }}>ROS 2 ✓</span>}
                    </label>
                  )
                })}
              </div>
              <div style={{ display: 'flex', gap: '0.5rem' }}>
                <button onClick={connectRobot} disabled={connStatus === 'connecting'} style={{ ...btnStyle('#065f46', '#6ee7b7'), flex: 1 }}>
                  {connStatus === 'connecting' ? '⏳ Conectando…' : 'Conectar'}
                </button>
                {connStatus === 'connected' && (
                  <button onClick={() => { setConnStatus('idle'); setConnMsg(''); setConnPanel(false) }} style={btnStyle('#3a1a1a', '#f87171')}>Desconectar</button>
                )}
              </div>
              {connMsg && <div style={{ marginTop: '0.5rem', fontSize: '0.75rem', color: connStatus === 'error' ? '#f87171' : '#6ee7b7' }}>{connMsg}</div>}
            </div>
          )}
        </div>

        {/* Procurar robô */}
        <div ref={discoverRef} style={{ position: 'relative' }}>
          <button onClick={() => { setDiscoverPanel(v => !v); setConnPanel(false) }}
            style={{ ...btnStyle('#161a22', '#fbbf24'), display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
            🔍 Procurar robô
            <span style={{ fontSize: '0.65rem', opacity: 0.7 }}>{discoverPanel ? '▲' : '▼'}</span>
          </button>
          {discoverPanel && (
            <div style={{ position: 'absolute', top: '2.4rem', left: 0, zIndex: 100, background: '#161a22', border: '1px solid #2a3142', borderRadius: '10px', padding: '1rem', width: '400px', boxShadow: '0 8px 32px rgba(0,0,0,0.5)' }}>
              <div style={{ fontSize: '0.72rem', color: '#8b92a8', marginBottom: '0.6rem', textTransform: 'uppercase', letterSpacing: '0.05em' }}>Descoberta na rede</div>
              <div style={{ display: 'flex', gap: '0.4rem', marginBottom: '0.6rem' }}>
                <input placeholder="Subnet (ex: 192.168.1)" value={subnet} onChange={e => setSubnet(e.target.value)}
                  style={{ flex: 1, background: '#0d0f14', border: '1px solid #2a3142', borderRadius: '6px', color: '#e6e9ef', padding: '0.35rem 0.6rem', fontSize: '0.78rem', outline: 'none', fontFamily: 'monospace' }} />
                <input placeholder="user" value={sshUser} onChange={e => setSshUser(e.target.value)}
                  style={{ width: '80px', background: '#0d0f14', border: '1px solid #2a3142', borderRadius: '6px', color: '#e6e9ef', padding: '0.35rem 0.6rem', fontSize: '0.78rem', outline: 'none', fontFamily: 'monospace' }} />
              </div>
              <button onClick={discoverRobots} disabled={discovering} style={{ ...btnStyle(discovering ? '#1a2a1a' : '#161a22', '#fbbf24'), width: '100%', marginBottom: '0.6rem' }}>
                {discovering ? '⏳ Varrendo .1–.254…' : '▶ Iniciar varredura'}
              </button>
              {discoverError && <div style={{ color: '#f87171', fontSize: '0.75rem', marginBottom: '0.4rem' }}>{discoverError}</div>}
              {discovered.length > 0 && (
                <div style={{ display: 'flex', flexDirection: 'column', gap: '0.35rem', maxHeight: '240px', overflowY: 'auto' }}>
                  {discovered.map(h => {
                    const st = testingSSH[h.ip]
                    const stColor = st === 'ok' ? '#6ee7b7' : st === 'no_ros' ? '#fbbf24' : st === 'error' ? '#f87171' : '#8b92a8'
                    const stLabel = st === 'testing' ? '⏳' : st === 'ok' ? '✓ ROS 2' : st === 'no_ros' ? '⚠ sem ROS' : st === 'error' ? '✗ erro' : 'Testar SSH'
                    const pid = `ssh_${h.ip}`
                    const canConn = st === 'ok' && extraProfiles.find(p => p.id === pid)
                    return (
                      <div key={h.ip} style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', background: '#0d0f14', borderRadius: '6px', padding: '0.4rem 0.65rem', border: `1px solid ${h.likely_robot ? '#2a3a2a' : '#2a3142'}` }}>
                        <span style={{ width: 7, height: 7, borderRadius: '50%', background: h.likely_robot ? '#6ee7b7' : '#4b5563', display: 'inline-block', flexShrink: 0 }} />
                        <div style={{ flex: 1, minWidth: 0 }}>
                          <div style={{ fontSize: '0.78rem', color: '#e6e9ef', fontFamily: 'monospace' }}>{h.ip}</div>
                          <div style={{ fontSize: '0.67rem', color: '#4b5563', overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>{h.hostname}</div>
                        </div>
                        {h.likely_robot && <span style={{ fontSize: '0.65rem', color: '#6ee7b7', background: '#1a3a2a', padding: '0.1rem 0.3rem', borderRadius: '4px', flexShrink: 0 }}>robô?</span>}
                        <button onClick={() => testSSH(h.ip)} disabled={st === 'testing'} style={{ ...btnStyle('#161a22', stColor), fontSize: '0.68rem', padding: '0.2rem 0.45rem', flexShrink: 0 }}>{stLabel}</button>
                        {canConn && (
                          <button onClick={() => { setSelectedProfile(pid); setConnStatus('connected'); setConnMsg(`SSH ${h.ip}`); setDiscoverPanel(false) }}
                            style={{ ...btnStyle('#065f46', '#6ee7b7'), fontSize: '0.68rem', padding: '0.2rem 0.45rem', flexShrink: 0 }}>Conectar</button>
                        )}
                      </div>
                    )
                  })}
                </div>
              )}
            </div>
          )}
        </div>

        {/* Separador */}
        <div style={{ width: '1px', height: '1.5rem', background: '#2a3142' }} />

        {/* Toggle modo multi-robô */}
        <button
          onClick={() => setMultiMode(v => !v)}
          style={{
            ...btnStyle(multiMode ? 'rgba(99,102,241,0.15)' : '#161a22', multiMode ? '#a5b4fc' : '#8b92a8'),
            display: 'flex', alignItems: 'center', gap: '0.4rem',
          }}
        >
          <span style={{ fontSize: '1rem', lineHeight: 1 }}>⊞</span>
          {multiMode ? 'Modo Multi-Robô' : 'Modo Single'}
        </button>

        {/* Status de conexão */}
        {isConnected && (
          <span style={{ fontSize: '0.78rem', color: '#6ee7b7', fontFamily: 'monospace' }}>
            ● {[...ROBOT_PROFILES, ...extraProfiles].find(p => p.id === selectedProfile)?.label}
          </span>
        )}
        {connStatus === 'error' && (
          <span style={{ fontSize: '0.78rem', color: '#f87171', fontFamily: 'monospace' }}>✗ {connMsg}</span>
        )}
      </div>

      {/* ── Painéis de robô ── */}
      <div style={{
        display: 'grid',
        gridTemplateColumns: multiMode ? '1fr 1fr' : '1fr',
        gap: '1rem',
        flex: 1, minHeight: 0,
        padding: '0.75rem 1.25rem 1rem',
      }}>
        <RobotPanel
          robotId={multiMode ? 'tb1' : 'default'}
          isConnected={isConnected}
          onViewResult={setModalResult}
          compact={multiMode}
        />
        {multiMode && (
          <RobotPanel
            robotId="tb2"
            isConnected={isConnected}
            onViewResult={setModalResult}
            compact={true}
          />
        )}
      </div>

      {/* ── Modal JSON resultado ── */}
      {modalResult && (
        <div onClick={() => setModalResult(null)}
          style={{ position: 'fixed', inset: 0, zIndex: 200, background: 'rgba(0,0,0,0.75)', display: 'flex', alignItems: 'center', justifyContent: 'center', padding: '2rem' }}>
          <div onClick={e => e.stopPropagation()}
            style={{ background: '#161a22', border: '1px solid #2a3142', borderRadius: '12px', width: '100%', maxWidth: '860px', maxHeight: '85vh', display: 'flex', flexDirection: 'column', overflow: 'hidden', boxShadow: '0 16px 48px rgba(0,0,0,0.7)' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', padding: '0.85rem 1.25rem', borderBottom: '1px solid #2a3142' }}>
              <span style={{ fontSize: '0.82rem', fontWeight: 600, color: '#8b92a8', textTransform: 'uppercase', letterSpacing: '0.05em' }}>Resultado JSON</span>
              <button onClick={() => setModalResult(null)} style={{ background: 'none', border: 'none', color: '#8b92a8', cursor: 'pointer', fontSize: '1.1rem' }}>✕</button>
            </div>
            <pre style={{ flex: 1, overflowY: 'auto', margin: 0, padding: '1.25rem', fontFamily: 'JetBrains Mono, Consolas, monospace', fontSize: '0.82rem', lineHeight: 1.65, color: '#e6e9ef' }}>
              {JSON.stringify(modalResult, null, 2)}
            </pre>
          </div>
        </div>
      )}

      <style>{`@keyframes blink { 0%,100%{opacity:1} 50%{opacity:0} }`}</style>
    </div>
  )
}

function btnStyle(bg, color, fontSize = '0.85rem', disabled = false) {
  return {
    background: bg, color,
    border: `1px solid ${color}`,
    borderRadius: '8px', padding: '0.5rem 1rem',
    fontSize, fontWeight: 600,
    cursor: disabled ? 'not-allowed' : 'pointer',
    opacity: disabled ? 0.6 : 1,
    fontFamily: 'inherit',
  }
}
