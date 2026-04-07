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
  topics: ["scan", "odom", "imu"],
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
  topics: ["scan", "odom", "imu"],
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
  const [connPanel, setConnPanel]       = useState(false)   // painel aberto?
  const [selectedProfile, setSelectedProfile] = useState('custom')
  const [connStatus, setConnStatus]     = useState('idle')  // idle | connecting | connected | error
  const [connMsg, setConnMsg]           = useState('')

  const outputRef  = useRef(null)
  const pollRef    = useRef(null)
  const panelRef   = useRef(null)

  // Fecha painel ao clicar fora
  useEffect(() => {
    const handler = (e) => {
      if (panelRef.current && !panelRef.current.contains(e.target)) setConnPanel(false)
    }
    document.addEventListener('mousedown', handler)
    return () => document.removeEventListener('mousedown', handler)
  }, [])

  // ── Status do robot (polling) ─────────────────────────────────────
  useEffect(() => {
    const t = setInterval(() =>
      fetch(`${API}/status`).then(r => r.json()).then(d => d && setStatus(d)).catch(() => {}),
    1000)
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
      setResetMsg(data.success ? 'ok' : 'erro')
    } catch {
      setResetMsg('erro')
    } finally {
      setResetting(false)
      setTimeout(() => setResetMsg(null), 3000)
    }
  }

  const pose  = status.pose || {}
  const robot = status.robots?.[0] || {}

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
  const profile     = ROBOT_PROFILES.find(p => p.id === selectedProfile)

  return (
    <div style={{ fontFamily: 'system-ui, sans-serif', background: '#0d0f14', color: '#e6e9ef', minHeight: '100vh', padding: '1.5rem' }}>

      {/* Header */}
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '1rem', borderBottom: '1px solid #2a3142', paddingBottom: '1rem' }}>
        <h1 style={{ margin: 0, fontSize: '1.4rem', color: '#6ee7b7' }}>Fleet UI</h1>
        <div style={{ display: 'flex', gap: '1.5rem', fontSize: '0.82rem', fontFamily: 'monospace', alignItems: 'center' }}>
          <span style={{ color: '#8b92a8' }}>
            Nav: <span style={{ color: robot.nav_state === 'navigating' ? '#6ee7b7' : robot.nav_state === 'failed' ? '#f87171' : '#e6e9ef' }}>
              {robot.nav_state || '—'}
            </span>
          </span>
          <span style={{ color: '#8b92a8' }}>
            Coleta: <span style={{ color: robot.collection_on ? '#6ee7b7' : '#8b92a8' }}>{robot.collection_on ? 'ON' : 'OFF'}</span>
          </span>
          <span style={{ color: '#8b92a8' }}>
            Pose: <span style={{ color: pose.valid ? '#e6e9ef' : '#8b92a8' }}>
              {pose.valid ? `x=${pose.x.toFixed(2)} y=${pose.y.toFixed(2)} yaw=${deg(pose.yaw)}°` : 'aguardando…'}
            </span>
          </span>
          <button
            onClick={resetToOrigin}
            disabled={resetting}
            title="Envia robô para (0, 0, 0)"
            style={{ ...btnStyle(resetting ? '#1a2a3a' : '#161a22', resetMsg === 'erro' ? '#f87171' : resetMsg === 'ok' ? '#6ee7b7' : '#6366f1'), fontSize: '0.78rem', padding: '0.3rem 0.75rem' }}
          >
            {resetting ? '⏳ indo…' : resetMsg === 'ok' ? '✓ indo' : resetMsg === 'erro' ? '✗ falhou' : '⟳ Reiniciar'}
          </button>
        </div>
      </div>

      {/* ── Barra de conexão ────────────────────────────────────────── */}
      <div ref={panelRef} style={{ position: 'relative', marginBottom: '1.25rem' }}>
        {/* Botão principal */}
        <div style={{ display: 'flex', alignItems: 'center', gap: '0.75rem' }}>
          <button
            onClick={() => setConnPanel(v => !v)}
            style={{ ...btnStyle('#161a22', '#6366f1'), display: 'flex', alignItems: 'center', gap: '0.5rem' }}
          >
            <span style={{ width: 10, height: 10, borderRadius: '50%', background: connLight, display: 'inline-block', boxShadow: connStatus === 'connected' ? `0 0 6px ${connLight}` : 'none', transition: 'background 0.3s' }} />
            Conectar robô
            <span style={{ fontSize: '0.7rem', opacity: 0.7 }}>{connPanel ? '▲' : '▼'}</span>
          </button>
          {isConnected && (
            <span style={{ fontSize: '0.8rem', color: '#6ee7b7', fontFamily: 'monospace' }}>
              ● Conectado — {profile?.label}
            </span>
          )}
          {connStatus === 'error' && (
            <span style={{ fontSize: '0.8rem', color: '#f87171', fontFamily: 'monospace' }}>
              ✗ {connMsg}
            </span>
          )}
        </div>

        {/* Painel dropdown */}
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
              {ROBOT_PROFILES.map(p => {
                const isSSH     = p.type === 'ssh'
                const isSelected = selectedProfile === p.id
                return (
                  <label
                    key={p.id}
                    onClick={() => !isSSH && setSelectedProfile(p.id)}
                    style={{
                      display: 'flex', alignItems: 'center', gap: '0.75rem',
                      padding: '0.6rem 0.85rem', borderRadius: '8px',
                      border: `1px solid ${isSelected ? '#6366f1' : '#2a3142'}`,
                      background: isSelected ? 'rgba(99,102,241,0.1)' : 'transparent',
                      cursor: isSSH ? 'not-allowed' : 'pointer',
                      opacity: isSSH ? 0.45 : 1,
                      transition: 'border-color 0.2s, background 0.2s',
                    }}
                  >
                    <input
                      type="radio"
                      name="robot_profile"
                      value={p.id}
                      checked={isSelected}
                      disabled={isSSH}
                      onChange={() => setSelectedProfile(p.id)}
                      style={{ accentColor: '#6366f1', margin: 0 }}
                    />
                    <div style={{ flex: 1 }}>
                      <div style={{ fontSize: '0.85rem', color: isSelected ? '#e6e9ef' : '#a0aec0', fontWeight: isSelected ? 600 : 400 }}>
                        {p.label}
                      </div>
                      {isSSH && (
                        <div style={{ fontSize: '0.72rem', color: '#4b5563', marginTop: '0.15rem' }}>
                          SSH {p.host} · em breve
                        </div>
                      )}
                      {!isSSH && (
                        <div style={{ fontSize: '0.72rem', color: '#4b5563', marginTop: '0.15rem' }}>
                          Backend local · localhost:8000
                        </div>
                      )}
                    </div>
                    {isSSH && (
                      <span style={{ fontSize: '0.68rem', background: '#1f2a3a', color: '#6b7280', padding: '0.15rem 0.4rem', borderRadius: '4px' }}>
                        em breve
                      </span>
                    )}
                  </label>
                )
              })}
            </div>

            <div style={{ display: 'flex', gap: '0.75rem', alignItems: 'center' }}>
              <button
                onClick={connectRobot}
                disabled={connStatus === 'connecting'}
                style={{ ...btnStyle('#065f46', '#6ee7b7'), flex: 1 }}
              >
                {connStatus === 'connecting' ? '⏳ Conectando…' : 'Conectar'}
              </button>
              {connStatus === 'connected' && (
                <button
                  onClick={() => { setConnStatus('idle'); setConnMsg(''); setConnPanel(false) }}
                  style={btnStyle('#3a1a1a', '#f87171')}
                >
                  Desconectar
                </button>
              )}
            </div>

            {connMsg && connStatus !== 'error' && (
              <div style={{ marginTop: '0.6rem', fontSize: '0.78rem', color: '#6ee7b7' }}>{connMsg}</div>
            )}
            {connStatus === 'error' && (
              <div style={{ marginTop: '0.6rem', fontSize: '0.78rem', color: '#f87171' }}>{connMsg}</div>
            )}
          </div>
        )}
      </div>

      {/* Layout principal */}
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '1.25rem', height: 'calc(100vh - 180px)' }}>

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
            <div>
              <button
                onClick={() => setShowResult(v => !v)}
                style={{ ...btnStyle('#161a22', '#2a3142'), fontSize: '0.78rem', width: '100%', textAlign: 'left' }}
              >
                {showResult ? '▾ Ocultar resultado JSON' : '▸ Ver resultado JSON'}
              </button>
              {showResult && (
                <pre style={{
                  background: '#161a22', border: '1px solid #2a3142', borderRadius: '0 0 8px 8px',
                  padding: '0.75rem 1rem', fontSize: '0.78rem',
                  fontFamily: 'JetBrains Mono, Consolas, monospace', color: '#e6e9ef',
                  margin: 0, overflowX: 'auto', maxHeight: '220px', overflowY: 'auto',
                }}>
                  {JSON.stringify(job.result, null, 2)}
                </pre>
              )}
            </div>
          )}

          {job?.error && (
            <div style={{ color: '#f87171', fontSize: '0.82rem', padding: '0.5rem 0.75rem', background: 'rgba(248,113,113,0.1)', borderRadius: '6px' }}>
              Erro interno: {job.error}
            </div>
          )}
        </div>
      </div>

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
