import React, { useState, useEffect, useRef, useCallback } from 'react'

const API = '/api'

// Perfis de robô disponíveis.
// type:'local' → usa o backend local (simulação Custom).
// type:'ssh'   → placeholder para robôs reais via SSH (em breve).
const ROBOT_PROFILES = [
  { id: 'custom', label: 'Custom (simulação local)', type: 'local' },
  { id: 'tb3_1',  label: 'TurtleBot3 #1',           type: 'ssh', host: '192.168.1.101' },
  { id: 'tb3_2',  label: 'TurtleBot3 #2',           type: 'ssh', host: '192.168.1.102' },
]

const EXAMPLE_RECORD = JSON.stringify({
  command: "record",
  robot: "default",
  route: "percurso1",
  collect: true,
  topics: ["scan", "odom", "imu", "pose"],
  initial_pose: [0, 0, 0],
  points: [
    [0.5, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.5, 0.5, 0.0],
    [2.0, 0.5, 0.0]
  ]
}, null, 2)

const EXAMPLE_REPLAY = JSON.stringify({
  command: "replay",
  robot: "default",
  route: "percurso1",
  collect: true,
  topics: ["scan", "odom", "imu", "pose"],
  initial_pose: [0, 0, 0],
  return_to_start: [0, 0, 0]
}, null, 2)

export default function App() {
  const [config, setConfig]         = useState(EXAMPLE_RECORD)
  const [jobId, setJobId]           = useState(null)
  const [job, setJob]               = useState(null)
  const [running, setRunning]       = useState(false)
  const [parseError, setParseError] = useState(null)
  const [showResult, setShowResult] = useState(false)
  const [status, setStatus]         = useState({ robots: [], pose: { x: 0, y: 0, yaw: 0, valid: false } })
  const [resetMsg, setResetMsg]     = useState(null)
  const [resetting, setResetting]   = useState(false)

  // ── Conexão ────────────────────────────────────────────────────────
  const [connPanel, setConnPanel]       = useState(false)
  const [selectedProfile, setSelectedProfile] = useState('custom')
  const [connStatus, setConnStatus]     = useState('idle')  // idle | connecting | connected | error
  const [connMsg, setConnMsg]           = useState('')

  // ── Descoberta de robôs ───────────────────────────────────────────
  const [discoverPanel, setDiscoverPanel] = useState(false)
  const [discovering, setDiscovering]     = useState(false)
  const [discovered, setDiscovered]       = useState([])    // [{ip, hostname, likely_robot}]
  const [discoverError, setDiscoverError] = useState(null)
  const [subnet, setSubnet]               = useState('')
  const [testingSSH, setTestingSSH]       = useState({})    // ip → 'testing'|'ok'|'no_ros'|'error'
  const [sshUser, setSshUser]             = useState('ubuntu')
  const [extraProfiles, setExtraProfiles] = useState([])    // perfis adicionados via descoberta
  const discoverRef                       = useRef(null)

  const outputRef  = useRef(null)
  const pollRef    = useRef(null)
  const panelRef   = useRef(null)

  // Fecha painéis ao clicar fora
  useEffect(() => {
    const handler = (e) => {
      if (panelRef.current && !panelRef.current.contains(e.target)) setConnPanel(false)
      if (discoverRef.current && !discoverRef.current.contains(e.target)) setDiscoverPanel(false)
    }
    document.addEventListener('mousedown', handler)
    return () => document.removeEventListener('mousedown', handler)
  }, [])

  // ── Status do robot (polling rápido) ─────────────────────────────
  useEffect(() => {
    const t = setInterval(() =>
      fetch(`${API}/status`).then(r => r.json()).then(d => d && setStatus(d)).catch(() => {}),
    300)
    return () => clearInterval(t)
  }, [])

  // ── Polling do job ────────────────────────────────────────────────
  const startPolling = useCallback((id) => {
    if (pollRef.current) clearInterval(pollRef.current)
    pollRef.current = setInterval(async () => {
      const r = await fetch(`${API}/job/${id}`).catch(() => null)
      if (!r) return
      const data = await r.json().catch(() => null)
      if (!data) return
      setJob(data)
      if (!data.running) {
        clearInterval(pollRef.current)
        setRunning(false)
      }
    }, 500)
  }, [])

  // Auto-scroll output
  useEffect(() => {
    if (outputRef.current) outputRef.current.scrollTop = outputRef.current.scrollHeight
  }, [job?.lines?.length])

  // ── Conectar robô ─────────────────────────────────────────────────
  const connectRobot = async () => {
    const profile = ROBOT_PROFILES.find(p => p.id === selectedProfile)
    if (!profile) return
    if (profile.type === 'ssh') {
      setConnStatus('error')
      setConnMsg(`SSH para ${profile.host} ainda não implementado.`)
      return
    }
    // Custom: verifica se o backend está online
    setConnStatus('connecting')
    setConnMsg('')
    try {
      const r = await fetch(`${API}/status`, { signal: AbortSignal.timeout(4000) })
      await r.json()
      if (r.ok) {
        setConnStatus('connected')
        setConnMsg('Backend respondendo.')
        setConnPanel(false)
      } else {
        throw new Error(`HTTP ${r.status}`)
      }
    } catch (e) {
      setConnStatus('error')
      setConnMsg(`Backend não respondeu: ${e.message}`)
    }
  }

  const discoverRobots = async () => {
    setDiscovering(true)
    setDiscovered([])
    setDiscoverError(null)
    try {
      const url = subnet.trim() ? `${API}/discover_robots?subnet=${encodeURIComponent(subnet.trim())}` : `${API}/discover_robots`
      const r = await fetch(url, { signal: AbortSignal.timeout(40000) })
      if (!r.ok) { setDiscoverError(`Erro HTTP ${r.status}`); return }
      const data = await r.json()
      if (data.error) { setDiscoverError(data.error); return }
      const found = data.found || []
      setDiscovered(found)
      if (found.length === 0) {
        const sub = data.subnet_scanned || subnet.trim() || '(auto)'
        setDiscoverError(`Nenhum host com porta 22 aberta encontrado em ${sub}.0/24`)
      }
    } catch (e) {
      setDiscoverError(`Erro: ${e.message}`)
    } finally {
      setDiscovering(false)
    }
  }

  const testSSH = async (ip) => {
    setTestingSSH(prev => ({ ...prev, [ip]: 'testing' }))
    try {
      const r = await fetch(`${API}/test_ssh`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ host: ip, user: sshUser }),
      })
      const data = await r.json()
      setTestingSSH(prev => ({ ...prev, [ip]: data.success && data.has_ros2 ? 'ok' : data.success ? 'no_ros' : 'error' }))
      if (data.success && data.has_ros2) {
        // Adiciona como perfil seleccionável no painel Conectar
        setExtraProfiles(prev => {
          const exists = prev.find(p => p.host === ip)
          if (exists) return prev
          return [...prev, { id: `ssh_${ip}`, label: `SSH ${ip} (${sshUser})`, type: 'ssh', host: ip, user: sshUser }]
        })
      }
    } catch {
      setTestingSSH(prev => ({ ...prev, [ip]: 'error' }))
    }
  }

  const run = async () => {
    setParseError(null)
    let cfg
    try { cfg = JSON.parse(config) }
    catch (e) { setParseError(`JSON inválido: ${e.message}`); return }

    setRunning(true)
    setJob(null)
    setJobId(null)

    const r = await fetch(`${API}/run_config`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(cfg),
    }).catch(e => { setRunning(false); setParseError(`Erro: ${e.message}`); return null })
    if (!r) return

    const data = await r.json().catch(() => null)
    if (!data?.job_id) {
      setRunning(false)
      setParseError(data?.message || 'Erro ao iniciar job')
      return
    }
    setJobId(data.job_id)
    startPolling(data.job_id)
  }

  const stop = () => {
    if (pollRef.current) clearInterval(pollRef.current)
    setRunning(false)
  }

  const resetToOrigin = async () => {
    setResetting(true)
    setResetMsg(null)
    try {
      const r = await fetch(`${API}/go_to_point?x=0&y=0&yaw=0`, { method: 'POST' })
      const data = await r.json()
      setResetMsg(data.success ? 'ok' : (data.message || 'erro'))
    } catch (e) {
      setResetMsg(`erro: ${e.message}`)
    } finally {
      setResetting(false)
      setTimeout(() => setResetMsg(null), 6000)
    }
  }

  const pose    = status.pose || {}
  const robot   = status.robots?.[0] || {}
  const nav2Ok  = status.nav2_ready === true

  const lineColor = (line) => {
    if (line.includes('[OK]'))     return '#6ee7b7'
    if (line.includes('[FALHOU]')) return '#f87171'
    if (line.includes('[TRACE]'))  return '#8b92a8'
    if (line.includes('Resumo'))   return '#fbbf24'
    if (line.includes('==='))      return '#93c5fd'
    return '#e6e9ef'
  }

  const deg = r => (r * 180 / Math.PI).toFixed(1)

  // Luz de status de conexão
  const connLight = {
    idle:       '#4b5563',
    connecting: '#fbbf24',
    connected:  '#6ee7b7',
    error:      '#f87171',
  }[connStatus]

  const isConnected = connStatus === 'connected'

  return (
    <div style={{ fontFamily: 'system-ui, sans-serif', background: '#0d0f14', color: '#e6e9ef', height: '100vh', display: 'flex', flexDirection: 'column', overflow: 'hidden' }}>

      {/* Header compacto */}
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', padding: '0.5rem 1.25rem', borderBottom: '1px solid #2a3142', flexShrink: 0 }}>
        <h1 style={{ margin: 0, fontSize: '1.1rem', color: '#6ee7b7', fontWeight: 700 }}>Fleet UI</h1>

        {/* Status em linha */}
        <div style={{ display: 'flex', gap: '0', fontSize: '0.78rem', fontFamily: 'monospace', alignItems: 'stretch', background: '#161a22', border: '1px solid #2a3142', borderRadius: '8px', overflow: 'hidden' }}>
          {[
            { label: 'Nav2', value: nav2Ok ? 'ON' : 'OFF', color: nav2Ok ? '#6ee7b7' : '#f87171' },
            { label: 'Nav', value: robot.nav_state || '—', color: robot.nav_state === 'navigating' ? '#6ee7b7' : robot.nav_state === 'failed' ? '#f87171' : '#e6e9ef' },
            { label: 'Coleta', value: robot.collection_on ? 'ON' : 'OFF', color: robot.collection_on ? '#6ee7b7' : '#8b92a8' },
            { label: 'x', value: pose.valid ? pose.x.toFixed(3) : '—', color: pose.valid ? '#93c5fd' : '#8b92a8' },
            { label: 'y', value: pose.valid ? pose.y.toFixed(3) : '—', color: pose.valid ? '#93c5fd' : '#8b92a8' },
            { label: 'yaw', value: pose.valid ? `${deg(pose.yaw)}°` : '—', color: pose.valid ? '#93c5fd' : '#8b92a8' },
          ].map(({ label, value, color }) => (
            <div key={label} style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', padding: '0.3rem 0.7rem', borderRight: '1px solid #2a3142' }}>
              <span style={{ fontSize: '0.62rem', color: '#4b5563', textTransform: 'uppercase', letterSpacing: '0.04em' }}>{label}</span>
              <span style={{ color, fontWeight: 600, marginTop: '0.05rem' }}>{value}</span>
            </div>
          ))}
          <button
            onClick={resetToOrigin}
            disabled={resetting}
            title="Envia robô para (0,0,0)"
            style={{ background: 'none', border: 'none', cursor: resetting ? 'not-allowed' : 'pointer', padding: '0.3rem 0.75rem', color: resetMsg === 'ok' ? '#6ee7b7' : resetMsg ? '#f87171' : '#6366f1', fontWeight: 700, fontSize: '0.8rem', opacity: resetting ? 0.6 : 1 }}
          >
            {resetting ? '⏳' : resetMsg === 'ok' ? '✓' : resetMsg ? '✗' : '⟳ Origem'}
          </button>
        </div>
      </div>

      {/* ── Barra de conexão ────────────────────────────────────────── */}
      <div style={{ display: 'flex', gap: '0.75rem', alignItems: 'center', padding: '0.4rem 1.25rem', borderBottom: '1px solid #2a3142', flexShrink: 0, position: 'relative' }}>

        {/* Botão Conectar robô */}
        <div ref={panelRef} style={{ position: 'relative' }}>
          <button
            onClick={() => { setConnPanel(v => !v); setDiscoverPanel(false) }}
            style={{ ...btnStyle('#161a22', '#6366f1'), display: 'flex', alignItems: 'center', gap: '0.5rem' }}
          >
            <span style={{ width: 10, height: 10, borderRadius: '50%', background: connLight, display: 'inline-block', boxShadow: connStatus === 'connected' ? `0 0 6px ${connLight}` : 'none', transition: 'background 0.3s' }} />
            Conectar robô
            <span style={{ fontSize: '0.7rem', opacity: 0.7 }}>{connPanel ? '▲' : '▼'}</span>
          </button>

          {connPanel && (
            <div style={{
              position: 'absolute', top: '2.5rem', left: 0, zIndex: 100,
              background: '#161a22', border: '1px solid #2a3142', borderRadius: '10px',
              padding: '1rem', width: '360px', boxShadow: '0 8px 32px rgba(0,0,0,0.5)',
            }}>
              <div style={{ fontSize: '0.78rem', color: '#8b92a8', marginBottom: '0.75rem', textTransform: 'uppercase', letterSpacing: '0.05em' }}>
                Selecionar robô
              </div>

              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem', marginBottom: '1rem' }}>
                {[...ROBOT_PROFILES, ...extraProfiles].map(p => {
                  const isSSH      = p.type === 'ssh' && !extraProfiles.find(ep => ep.id === p.id)
                  const isSelected = selectedProfile === p.id
                  const isExtra    = !!extraProfiles.find(ep => ep.id === p.id)
                  return (
                    <label
                      key={p.id}
                      onClick={() => setSelectedProfile(p.id)}
                      style={{
                        display: 'flex', alignItems: 'center', gap: '0.75rem',
                        padding: '0.6rem 0.85rem', borderRadius: '8px',
                        border: `1px solid ${isSelected ? '#6366f1' : '#2a3142'}`,
                        background: isSelected ? 'rgba(99,102,241,0.1)' : 'transparent',
                        cursor: 'pointer',
                        transition: 'border-color 0.2s, background 0.2s',
                      }}
                    >
                      <input type="radio" name="robot_profile" value={p.id} checked={isSelected}
                        onChange={() => setSelectedProfile(p.id)} style={{ accentColor: '#6366f1', margin: 0 }} />
                      <div style={{ flex: 1 }}>
                        <div style={{ fontSize: '0.85rem', color: isSelected ? '#e6e9ef' : '#a0aec0', fontWeight: isSelected ? 600 : 400 }}>
                          {p.label}
                        </div>
                        <div style={{ fontSize: '0.72rem', color: '#4b5563', marginTop: '0.15rem' }}>
                          {isExtra ? `SSH ${p.host} · descoberto` : p.type === 'ssh' ? `SSH ${p.host} · em breve` : 'Backend local · localhost:8000'}
                        </div>
                      </div>
                      {isSSH && !isExtra && (
                        <span style={{ fontSize: '0.68rem', background: '#1f2a3a', color: '#6b7280', padding: '0.15rem 0.4rem', borderRadius: '4px' }}>
                          em breve
                        </span>
                      )}
                      {isExtra && (
                        <span style={{ fontSize: '0.68rem', background: '#1a3a2a', color: '#6ee7b7', padding: '0.15rem 0.4rem', borderRadius: '4px' }}>
                          ROS 2 ✓
                        </span>
                      )}
                    </label>
                  )
                })}
              </div>

              <div style={{ display: 'flex', gap: '0.75rem', alignItems: 'center' }}>
                <button onClick={connectRobot} disabled={connStatus === 'connecting'}
                  style={{ ...btnStyle('#065f46', '#6ee7b7'), flex: 1 }}>
                  {connStatus === 'connecting' ? '⏳ Conectando…' : 'Conectar'}
                </button>
                {connStatus === 'connected' && (
                  <button onClick={() => { setConnStatus('idle'); setConnMsg(''); setConnPanel(false) }}
                    style={btnStyle('#3a1a1a', '#f87171')}>
                    Desconectar
                  </button>
                )}
              </div>
              {connMsg && <div style={{ marginTop: '0.6rem', fontSize: '0.78rem', color: connStatus === 'error' ? '#f87171' : '#6ee7b7' }}>{connMsg}</div>}
            </div>
          )}
        </div>

        {/* Botão Procurar robô */}
        <div ref={discoverRef} style={{ position: 'relative' }}>
          <button
            onClick={() => { setDiscoverPanel(v => !v); setConnPanel(false) }}
            style={{ ...btnStyle('#161a22', '#fbbf24'), display: 'flex', alignItems: 'center', gap: '0.5rem' }}
          >
            🔍 Procurar robô
            <span style={{ fontSize: '0.7rem', opacity: 0.7 }}>{discoverPanel ? '▲' : '▼'}</span>
          </button>

          {discoverPanel && (
            <div style={{
              position: 'absolute', top: '2.5rem', left: 0, zIndex: 100,
              background: '#161a22', border: '1px solid #2a3142', borderRadius: '10px',
              padding: '1rem', width: '420px', boxShadow: '0 8px 32px rgba(0,0,0,0.5)',
            }}>
              <div style={{ fontSize: '0.78rem', color: '#8b92a8', marginBottom: '0.75rem', textTransform: 'uppercase', letterSpacing: '0.05em' }}>
                Descoberta de robôs na rede
              </div>

              <div style={{ display: 'flex', gap: '0.5rem', marginBottom: '0.75rem' }}>
                <input
                  placeholder="Subnet (ex: 192.168.1) — auto se vazio"
                  value={subnet}
                  onChange={e => setSubnet(e.target.value)}
                  style={{ flex: 1, background: '#0d0f14', border: '1px solid #2a3142', borderRadius: '6px', color: '#e6e9ef', padding: '0.4rem 0.6rem', fontSize: '0.8rem', outline: 'none', fontFamily: 'monospace' }}
                />
                <input
                  placeholder="user SSH"
                  value={sshUser}
                  onChange={e => setSshUser(e.target.value)}
                  style={{ width: '90px', background: '#0d0f14', border: '1px solid #2a3142', borderRadius: '6px', color: '#e6e9ef', padding: '0.4rem 0.6rem', fontSize: '0.8rem', outline: 'none', fontFamily: 'monospace' }}
                />
              </div>

              <button onClick={discoverRobots} disabled={discovering}
                style={{ ...btnStyle(discovering ? '#1a2a1a' : '#161a22', '#fbbf24'), width: '100%', marginBottom: '0.75rem' }}>
                {discovering ? '⏳ Varrendo rede (.1–.254)…' : '▶ Iniciar varredura'}
              </button>

              {discoverError && (
                <div style={{ color: '#f87171', fontSize: '0.78rem', marginBottom: '0.5rem' }}>{discoverError}</div>
              )}

              {discovered.length > 0 && (
                <div style={{ display: 'flex', flexDirection: 'column', gap: '0.4rem', maxHeight: '260px', overflowY: 'auto' }}>
                  {discovered.map(h => {
                    const st = testingSSH[h.ip]
                    const stColor = st === 'ok' ? '#6ee7b7' : st === 'no_ros' ? '#fbbf24' : st === 'error' ? '#f87171' : '#8b92a8'
                    const stLabel = st === 'testing' ? '⏳' : st === 'ok' ? '✓ ROS 2' : st === 'no_ros' ? '⚠ sem ROS' : st === 'error' ? '✗ falhou' : 'Testar SSH'
                    const profileId = `ssh_${h.ip}`
                    const canConnect = st === 'ok' && extraProfiles.find(p => p.id === profileId)
                    return (
                      <div key={h.ip} style={{ display: 'flex', alignItems: 'center', gap: '0.6rem', background: '#0d0f14', borderRadius: '6px', padding: '0.5rem 0.75rem', border: `1px solid ${h.likely_robot ? '#2a3a2a' : '#2a3142'}` }}>
                        <span style={{ width: 8, height: 8, borderRadius: '50%', background: h.likely_robot ? '#6ee7b7' : '#4b5563', display: 'inline-block', flexShrink: 0 }} />
                        <div style={{ flex: 1, minWidth: 0 }}>
                          <div style={{ fontSize: '0.82rem', color: '#e6e9ef', fontFamily: 'monospace' }}>{h.ip}</div>
                          <div style={{ fontSize: '0.7rem', color: '#4b5563', overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>{h.hostname}</div>
                        </div>
                        {h.likely_robot && <span style={{ fontSize: '0.68rem', color: '#6ee7b7', background: '#1a3a2a', padding: '0.1rem 0.35rem', borderRadius: '4px', flexShrink: 0 }}>robô?</span>}
                        <button onClick={() => testSSH(h.ip)} disabled={st === 'testing'}
                          style={{ ...btnStyle('#161a22', stColor), fontSize: '0.72rem', padding: '0.25rem 0.5rem', flexShrink: 0 }}>
                          {stLabel}
                        </button>
                        {canConnect && (
                          <button
                            onClick={() => {
                              setSelectedProfile(profileId)
                              setConnStatus('connected')
                              setConnMsg(`SSH ${h.ip} com ROS 2`)
                              setDiscoverPanel(false)
                            }}
                            style={{ ...btnStyle('#065f46', '#6ee7b7'), fontSize: '0.72rem', padding: '0.25rem 0.5rem', flexShrink: 0 }}
                          >
                            Conectar
                          </button>
                        )}
                      </div>
                    )
                  })}
                </div>
              )}

              {extraProfiles.length > 0 && (
                <div style={{ marginTop: '0.75rem', fontSize: '0.75rem', color: '#6ee7b7' }}>
                  ✓ {extraProfiles.length} robô(s) com ROS 2 adicionado(s) ao painel Conectar
                </div>
              )}
            </div>
          )}
        </div>

        {/* Status de conexão inline */}
        {isConnected && (
          <span style={{ fontSize: '0.8rem', color: '#6ee7b7', fontFamily: 'monospace' }}>
            ● Conectado — {[...ROBOT_PROFILES, ...extraProfiles].find(p => p.id === selectedProfile)?.label}
          </span>
        )}
        {connStatus === 'error' && (
          <span style={{ fontSize: '0.8rem', color: '#f87171', fontFamily: 'monospace' }}>✗ {connMsg}</span>
        )}
      </div>

      {/* Layout principal — ocupa o restante da viewport */}
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '1rem', flex: 1, minHeight: 0, padding: '0.75rem 1.25rem 1rem' }}>

        {/* Coluna esquerda: config */}
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ fontSize: '0.8rem', fontWeight: 600, color: '#8b92a8', textTransform: 'uppercase', letterSpacing: '0.05em' }}>
              Configuração (JSON)
            </span>
            <div style={{ display: 'flex', gap: '0.5rem' }}>
              <button onClick={() => setConfig(EXAMPLE_RECORD)} style={btnStyle('#161a22', '#2a3142')}>Exemplo record</button>
              <button onClick={() => setConfig(EXAMPLE_REPLAY)} style={btnStyle('#161a22', '#2a3142')}>Exemplo replay</button>
            </div>
          </div>

          <textarea
            value={config}
            onChange={e => { setConfig(e.target.value); setParseError(null) }}
            spellCheck={false}
            style={{
              flex: 1,
              background: '#161a22',
              border: `1px solid ${parseError ? '#f87171' : '#2a3142'}`,
              borderRadius: '8px',
              color: '#e6e9ef',
              fontFamily: 'JetBrains Mono, Consolas, monospace',
              fontSize: '0.85rem',
              padding: '1rem',
              resize: 'none',
              outline: 'none',
              lineHeight: 1.6,
            }}
          />

          {parseError && (
            <div style={{ color: '#f87171', fontSize: '0.82rem', padding: '0.5rem 0.75rem', background: 'rgba(248,113,113,0.1)', borderRadius: '6px' }}>
              {parseError}
            </div>
          )}

          {!isConnected && (
            <div style={{ color: '#fbbf24', fontSize: '0.82rem', padding: '0.5rem 0.75rem', background: 'rgba(251,191,36,0.08)', borderRadius: '6px', border: '1px solid rgba(251,191,36,0.2)' }}>
              ⚠ Conecte um robô antes de executar.
            </div>
          )}
          {isConnected && !nav2Ok && (
            <div style={{ color: '#fbbf24', fontSize: '0.82rem', padding: '0.5rem 0.75rem', background: 'rgba(251,191,36,0.08)', borderRadius: '6px', border: '1px solid rgba(251,191,36,0.2)' }}>
              ⚠ Nav2 parece offline — verifique o T2. O experimento falhará se Nav2 não estiver ativo.
            </div>
          )}

          <div style={{ display: 'flex', gap: '0.75rem' }}>
            <button
              onClick={run}
              disabled={running || !isConnected}
              title={!isConnected ? 'Conecte um robô primeiro' : ''}
              style={btnStyle(running || !isConnected ? '#1a3a2a' : '#065f46', '#6ee7b7', '1rem', running || !isConnected)}
            >
              {running ? '⏳ A executar…' : '▶ Executar'}
            </button>
            {running && (
              <button onClick={stop} style={btnStyle('#3a1a1a', '#f87171')}>
                ■ Parar polling
              </button>
            )}
          </div>
        </div>

        {/* Coluna direita: output */}
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem', minHeight: 0, overflow: 'hidden' }}>
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

          <div
            ref={outputRef}
            style={{
              flex: 1,
              background: '#161a22',
              border: '1px solid #2a3142',
              borderRadius: '8px',
              padding: '0.75rem 1rem',
              overflowY: 'auto',
              fontFamily: 'JetBrains Mono, Consolas, monospace',
              fontSize: '0.78rem',
              lineHeight: 1.7,
            }}
          >
            {!job && !running && (
              <span style={{ color: '#8b92a8' }}>Aguardando execução…</span>
            )}
            {job?.lines?.map((line, i) => (
              <div key={i} style={{ color: lineColor(line), whiteSpace: 'pre-wrap', wordBreak: 'break-word' }}>
                {line}
              </div>
            ))}
            {running && (
              <div style={{ color: '#8b92a8', animation: 'blink 1s step-end infinite' }}>▌</div>
            )}
          </div>

          {job?.result && (
            <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>

              {/* Métricas do bag */}
              {job.result.bag_metrics && Object.keys(job.result.bag_metrics).length > 0 && (() => {
                const m = job.result.bag_metrics
                const dur = m.wall_duration_s ?? m.duration_s
                // Prioridade: AMCL > SLAM /pose > TF > odom > teórico (waypoints)
                const amclPath = m.amcl_path_length_m
                const posePath = m.pose_path_length_m
                const tfPath   = m.tf_path_length_m
                const odomPath = m.odom_path_length_m
                const theoPath = m.theoretical_path_m
                const realPath = amclPath ?? posePath ?? tfPath ?? odomPath
                const path     = realPath ?? theoPath
                const pathLabel = amclPath != null ? 'Percurso real (AMCL)'
                               : posePath != null ? 'Percurso real (SLAM)'
                               : tfPath   != null ? 'Percurso real (TF)'
                               : odomPath != null ? 'Percurso (odom)'
                               : 'Percurso (teórico)'
                const pathColor = realPath != null ? '#6ee7b7' : '#fbbf24'
                const speed = amclPath != null ? m.amcl_avg_speed_ms
                            : posePath != null ? m.pose_avg_speed_ms
                            : tfPath   != null ? m.tf_avg_speed_ms
                            : m.odom_avg_speed_ms
                const cards = [
                  dur      != null && { label: 'Duração',          value: `${dur} s`,        color: '#93c5fd' },
                  path     != null && { label: pathLabel,           value: `${path} m`,       color: pathColor },
                  speed    != null && { label: 'Vel. média',        value: `${speed} m/s`,    color: pathColor },
                  theoPath != null && realPath != null && { label: 'Percurso teórico', value: `${theoPath} m`, color: '#fbbf24' },
                  m.scan_avg_valid_points  != null && { label: 'Scan pts válidos', value: `${m.scan_avg_valid_points}`,   color: '#fbbf24' },
                  m.imu_accel_mean_ms2     != null && { label: 'IMU accel média',  value: `${m.imu_accel_mean_ms2} m/s²`, color: '#c4b5fd' },
                  m.imu_accel_variance_ms2 != null && { label: 'IMU variância',    value: `${m.imu_accel_variance_ms2}`,  color: '#c4b5fd' },
                ].filter(Boolean)
                const pathUnavailable = m.path_unavailable === true && realPath == null
                return (
                  <div style={{ background: '#161a22', border: '1px solid #2a3142', borderRadius: '8px', padding: '0.75rem 1rem' }}>
                    <div style={{ fontSize: '0.75rem', color: '#8b92a8', textTransform: 'uppercase', letterSpacing: '0.05em', marginBottom: '0.6rem' }}>
                      Métricas da gravação
                    </div>
                    <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: '0.5rem' }}>
                      {cards.map(({ label, value, color }) => (
                        <div key={label} style={{ background: '#0d0f14', borderRadius: '6px', padding: '0.45rem 0.6rem' }}>
                          <div style={{ fontSize: '0.68rem', color: '#8b92a8', marginBottom: '0.15rem' }}>{label}</div>
                          <div style={{ fontSize: '0.85rem', fontWeight: 700, color, fontFamily: 'JetBrains Mono, monospace' }}>{value}</div>
                        </div>
                      ))}
                    </div>
                    {pathUnavailable && (
                      <div style={{ marginTop: '0.6rem', fontSize: '0.75rem', color: '#fbbf24', background: 'rgba(251,191,36,0.08)', border: '1px solid rgba(251,191,36,0.2)', borderRadius: '6px', padding: '0.4rem 0.6rem' }}>
                        ⚠ Percurso indisponível — adicione <code style={{ fontFamily: 'monospace', background: 'rgba(255,255,255,0.08)', padding: '0 0.25rem', borderRadius: '3px' }}>"pose"</code> nos topics para medir deslocamento real.
                      </div>
                    )}
                  </div>
                )
              })()}

              {/* Sensor summary */}
              {job.result.sensor_summary && Object.keys(job.result.sensor_summary).length > 0 && (
                <div style={{ background: '#161a22', border: '1px solid #2a3142', borderRadius: '8px', padding: '0.75rem 1rem' }}>
                  <div style={{ fontSize: '0.75rem', color: '#8b92a8', textTransform: 'uppercase', letterSpacing: '0.05em', marginBottom: '0.5rem' }}>
                    Sensores gravados
                  </div>
                  <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.78rem', fontFamily: 'JetBrains Mono, Consolas, monospace' }}>
                    <thead>
                      <tr>
                        {['Tópico', 'Msgs', 'Hz est.'].map(h => (
                          <th key={h} style={{ textAlign: 'left', color: '#8b92a8', fontWeight: 500, paddingBottom: '0.3rem', borderBottom: '1px solid #2a3142' }}>{h}</th>
                        ))}
                      </tr>
                    </thead>
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

              {/* JSON — abre modal */}
              <button
                onClick={() => setShowResult(true)}
                style={{ ...btnStyle('#161a22', '#2a3142'), fontSize: '0.78rem', width: '100%', textAlign: 'left' }}
              >
                ▸ Ver resultado JSON
              </button>
            </div>
          )}

          {job?.error && (
            <div style={{ color: '#f87171', fontSize: '0.82rem', padding: '0.5rem 0.75rem', background: 'rgba(248,113,113,0.1)', borderRadius: '6px' }}>
              Erro interno: {job.error}
            </div>
          )}
        </div>
      </div>

      {/* Modal JSON resultado */}
      {showResult && job?.result && (
        <div
          onClick={() => setShowResult(false)}
          style={{
            position: 'fixed', inset: 0, zIndex: 200,
            background: 'rgba(0,0,0,0.75)',
            display: 'flex', alignItems: 'center', justifyContent: 'center',
            padding: '2rem',
          }}
        >
          <div
            onClick={e => e.stopPropagation()}
            style={{
              background: '#161a22', border: '1px solid #2a3142', borderRadius: '12px',
              width: '100%', maxWidth: '860px', maxHeight: '85vh',
              display: 'flex', flexDirection: 'column', overflow: 'hidden',
              boxShadow: '0 16px 48px rgba(0,0,0,0.7)',
            }}
          >
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', padding: '0.85rem 1.25rem', borderBottom: '1px solid #2a3142' }}>
              <span style={{ fontSize: '0.82rem', fontWeight: 600, color: '#8b92a8', textTransform: 'uppercase', letterSpacing: '0.05em' }}>
                Resultado JSON
              </span>
              <button onClick={() => setShowResult(false)} style={{ background: 'none', border: 'none', color: '#8b92a8', cursor: 'pointer', fontSize: '1.1rem', lineHeight: 1 }}>✕</button>
            </div>
            <pre style={{
              flex: 1, overflowY: 'auto', overflowX: 'auto',
              margin: 0, padding: '1.25rem',
              fontFamily: 'JetBrains Mono, Consolas, monospace',
              fontSize: '0.82rem', lineHeight: 1.65, color: '#e6e9ef',
            }}>
              {JSON.stringify(job.result, null, 2)}
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
    background: bg,
    color,
    border: `1px solid ${color}`,
    borderRadius: '8px',
    padding: '0.5rem 1rem',
    fontSize,
    fontWeight: 600,
    cursor: disabled ? 'not-allowed' : 'pointer',
    opacity: disabled ? 0.6 : 1,
    fontFamily: 'inherit',
  }
}
