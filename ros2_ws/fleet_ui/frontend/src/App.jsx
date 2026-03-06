import React, { useState, useEffect, useRef, useCallback } from 'react'

const API = '/api'
const WS_URL = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws/status`

// Mundo padrão: -5 a 5 em x e y (metros)
const WORLD = { minX: -5, maxX: 5, minY: -5, maxY: 5 }

function pixelToWorld(px, py, width, height) {
  const x = WORLD.minX + (px / width) * (WORLD.maxX - WORLD.minX)
  const y = WORLD.maxY - (py / height) * (WORLD.maxY - WORLD.minY)
  return { x, y }
}

function worldToPixel(x, y, width, height) {
  const px = ((x - WORLD.minX) / (WORLD.maxX - WORLD.minX)) * width
  const py = (1 - (y - WORLD.minY) / (WORLD.maxY - WORLD.minY)) * height
  return { px, py }
}

export default function App() {
  const [status, setStatus] = useState({ robots: [] })
  const [wsLive, setWsLive] = useState(false)
  const [selectedRobot, setSelectedRobot] = useState('')
  const [routeName, setRouteName] = useState('r1')
  const [routeList, setRouteList] = useState([])
  const [target, setTarget] = useState(null)
  const [toast, setToast] = useState(null)
  const mapRef = useRef(null)
  const wsRef = useRef(null)

  const showToast = (msg, isError = false) => {
    setToast({ msg, error: isError })
    setTimeout(() => setToast(null), 3500)
  }

  const fetchJson = async (url, options = {}) => {
    const r = await fetch(url, { ...options, headers: { 'Content-Type': 'application/json', ...options.headers } })
    const data = await r.json().catch(() => ({}))
    return { ok: r.ok, data }
  }

  const call = async (path, method = 'POST') => {
    const url = `${API}${path}`
    const { ok, data } = await fetchJson(url, { method })
    return { ok, data }
  }

  const loadRoutes = useCallback(async () => {
    const { data } = await call(`/list_routes?robot_id=${encodeURIComponent(selectedRobot)}`, 'GET')
    setRouteList(data.route_names || [])
  }, [selectedRobot])

  useEffect(() => {
    fetch(`${API}/list_robots`)
      .then((r) => r.json())
      .then((data) => {
        const ids = data.robot_ids || []
        if (ids.length) setSelectedRobot((prev) => prev || ids[0] || '')
      })
      .catch(() => {})
  }, [])

  useEffect(() => {
    loadRoutes()
  }, [loadRoutes, selectedRobot])

  useEffect(() => {
    let closed = false
    const connect = () => {
      const ws = new WebSocket(WS_URL)
      ws.onopen = () => { setWsLive(true); wsRef.current = ws }
      ws.onclose = () => { setWsLive(false); wsRef.current = null; if (!closed) setTimeout(connect, 2000) }
      ws.onerror = () => {}
      ws.onmessage = (e) => {
        try {
          const next = JSON.parse(e.data)
          setStatus(next)
        } catch (_) {}
      }
    }
    connect()
    return () => { closed = true }
  }, [])

  useEffect(() => {
    if (wsLive) return
    const t = setInterval(() => fetch(`${API}/status`).then((r) => r.json()).then(setStatus).catch(() => {}), 2000)
    return () => clearInterval(t)
  }, [wsLive])

  const handleMapClick = (e) => {
    const canvas = mapRef.current
    if (!canvas) return
    const rect = canvas.getBoundingClientRect()
    const scaleX = canvas.width / rect.width
    const scaleY = canvas.height / rect.height
    const px = (e.clientX - rect.left) * scaleX
    const py = (e.clientY - rect.top) * scaleY
    const { x, y } = pixelToWorld(px, py, canvas.width, canvas.height)
    setTarget({ x, y })
  }

  const drawMap = useCallback((ctx, w, h) => {
    ctx.fillStyle = '#0f1219'
    ctx.fillRect(0, 0, w, h)
    const step = 40
    ctx.strokeStyle = '#1e2433'
    ctx.lineWidth = 1
    for (let i = 0; i <= w; i += step) {
      ctx.beginPath()
      ctx.moveTo(i, 0)
      ctx.lineTo(i, h)
      ctx.stroke()
    }
    for (let j = 0; j <= h; j += step) {
      ctx.beginPath()
      ctx.moveTo(0, j)
      ctx.lineTo(w, j)
      ctx.stroke()
    }
    ctx.strokeStyle = '#2a3142'
    const cx = (0 - WORLD.minX) / (WORLD.maxX - WORLD.minX) * w
    const cy = (1 - (0 - WORLD.minY) / (WORLD.maxY - WORLD.minY)) * h
    ctx.beginPath()
    ctx.moveTo(cx, 0)
    ctx.lineTo(cx, h)
    ctx.moveTo(0, cy)
    ctx.lineTo(w, cy)
    ctx.stroke()
  }, [])

  const resizeCanvas = useCallback(() => {
    const canvas = mapRef.current
    if (!canvas) return
    const rect = canvas.getBoundingClientRect()
    const dpr = window.devicePixelRatio || 1
    if (canvas.width !== rect.width * dpr || canvas.height !== rect.height * dpr) {
      canvas.width = rect.width * dpr
      canvas.height = rect.height * dpr
    }
    const ctx = canvas.getContext('2d')
    ctx.setTransform(1, 0, 0, 1, 0, 0)
    ctx.scale(dpr, dpr)
    const w = rect.width
    const h = rect.height
    drawMap(ctx, w, h)
    if (target) {
      const { px, py } = worldToPixel(target.x, target.y, w, h)
      ctx.fillStyle = '#6ee7b7'
      ctx.beginPath()
      ctx.arc(px, py, 8, 0, Math.PI * 2)
      ctx.fill()
      ctx.strokeStyle = '#0d0f14'
      ctx.lineWidth = 2
      ctx.stroke()
    }
  }, [drawMap, target])

  useEffect(() => {
    resizeCanvas()
    window.addEventListener('resize', resizeCanvas)
    return () => window.removeEventListener('resize', resizeCanvas)
  }, [resizeCanvas])

  const q = (params) => new URLSearchParams(params).toString()

  const goToPoint = async () => {
    if (!target) return
    const { ok, data } = await call(`/go_to_point?${q({ robot_id: selectedRobot, x: target.x, y: target.y, yaw: 0 })}`, 'POST')
    showToast(ok ? 'Go to point enviado.' : (data?.message || 'Erro'), !ok)
  }

  const startRecord = async () => {
    const { ok, data } = await call(`/start_record?${q({ robot_id: selectedRobot, route_name: routeName })}`, 'POST')
    showToast(ok ? 'Record iniciado.' : (data?.message || 'Erro'), !ok)
    if (ok) loadRoutes()
  }

  const stopRecord = async () => {
    const { ok, data } = await call(`/stop_record?${q({ robot_id: selectedRobot })}`, 'POST')
    showToast(ok ? 'Record parado. Rota salva.' : (data?.message || 'Erro'), !ok)
    if (ok) loadRoutes()
  }

  const playRoute = async () => {
    const { ok, data } = await call(`/play_route?${q({ robot_id: selectedRobot, route_name: routeName })}`, 'POST')
    showToast(ok ? 'Play route enviado.' : (data?.message || 'Erro'), !ok)
  }

  const cancel = async () => {
    const { ok, data } = await call(`/cancel?${q({ robot_id: selectedRobot })}`, 'POST')
    showToast(ok ? 'Cancelado.' : (data?.message || 'Erro'), !ok)
  }

  const enableCollection = async () => {
    const { ok, data } = await call(`/enable_collection?${q({ robot_id: selectedRobot })}`, 'POST')
    showToast(ok ? 'Coleta ativada.' : (data?.message || 'Erro'), !ok)
  }

  const disableCollection = async () => {
    const { ok, data } = await call(`/disable_collection?${q({ robot_id: selectedRobot })}`, 'POST')
    showToast(ok ? 'Coleta desativada.' : (data?.message || 'Erro'), !ok)
  }

  const robots = status.robots.length
    ? status.robots
    : [{ robot_id: '', nav_state: 'idle', current_route: '', collection_on: false, collection_file: '', last_error: '', bytes_written: 0 }]

  return (
    <div className="app">
      <header className="header">
        <h1>Fleet UI</h1>
        <span className={`status-badge ${wsLive ? 'live' : ''}`}>
          {wsLive ? 'Status ao vivo' : 'Polling'}
        </span>
      </header>

      <div className="grid">
        <div>
          <div className="panel">
            <h2>Mapa (clique para definir destino)</h2>
            <div className="map-container">
              <canvas
                ref={mapRef}
                className="map-canvas"
                onClick={handleMapClick}
                style={{ width: '100%', height: '100%' }}
              />
              <div className="map-overlay">
                <span>
                  {target ? `x: ${target.x.toFixed(2)} m  y: ${target.y.toFixed(2)} m` : 'Clique no mapa'}
                </span>
                <button className="btn btn-go" disabled={!target} onClick={goToPoint}>
                  Ir para ponto
                </button>
              </div>
            </div>
          </div>

          <div className="panel" style={{ marginTop: '1rem' }}>
            <h2>Controles</h2>
            <div className="controls">
              <label>
                Robô:
                <select value={selectedRobot} onChange={(e) => setSelectedRobot(e.target.value)}>
                  {robots.map((r) => (
                    <option key={r.robot_id || 'default'} value={r.robot_id}>{r.robot_id || '(default)'}</option>
                  ))}
                </select>
              </label>
              <label>
                Rota:
                <input
                  type="text"
                  value={routeName}
                  onChange={(e) => setRouteName(e.target.value)}
                  placeholder="r1"
                  list="routes"
                />
                <datalist id="routes">
                  {routeList.map((name) => <option key={name} value={name} />)}
                </datalist>
              </label>
            </div>
            <div className="controls">
              <button className="btn btn-primary" onClick={startRecord}>Iniciar gravação</button>
              <button className="btn" onClick={stopRecord}>Parar gravação</button>
              <button className="btn btn-primary" onClick={playRoute}>Reproduzir rota</button>
              <button className="btn btn-danger" onClick={cancel}>Cancelar</button>
              <button className="btn" onClick={enableCollection}>Ligar coleta</button>
              <button className="btn" onClick={disableCollection}>Desligar coleta</button>
            </div>
          </div>
        </div>

        <div className="panel">
          <h2>Status da frota</h2>
          <div className="robot-cards">
            {robots.map((r) => (
              <div key={r.robot_id} className="robot-card">
                <h3>{r.robot_id || '(default)'}</h3>
                <div className="row">
                  <span>Nav</span>
                  <span className={`nav-state ${r.nav_state}`}>{r.nav_state}</span>
                </div>
                <div className="row">
                  <span>Rota atual</span>
                  <span>{r.current_route || '—'}</span>
                </div>
                <div className="row">
                  <span>Coleta</span>
                  <span>{r.collection_on ? 'ON' : 'OFF'}</span>
                </div>
                {r.collection_file && (
                  <div className="row">
                    <span>Arquivo</span>
                    <span style={{ fontSize: '0.7rem', wordBreak: 'break-all' }}>{r.collection_file}</span>
                  </div>
                )}
                {r.last_error && <div className="error-msg">{r.last_error}</div>}
              </div>
            ))}
          </div>
        </div>
      </div>

      {toast && (
        <div className={`toast ${toast.error ? 'error' : 'success'}`}>
          {toast.msg}
        </div>
      )}
    </div>
  )
}
