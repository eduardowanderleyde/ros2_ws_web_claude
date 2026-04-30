import React, { useState, useEffect, useRef, useCallback } from 'react'

const API = '/api'

const ROBOT_PROFILES = [
  { id: 'custom', label: 'Custom (simulação local)', type: 'local' },
  { id: 'tb3_1',  label: 'TurtleBot3 #1',           type: 'ssh', host: '192.168.1.101' },
  { id: 'tb3_2',  label: 'TurtleBot3 #2',           type: 'ssh', host: '192.168.1.102' },
]

const ALL_TOPICS = ['scan', 'odom', 'imu', 'pose']

const DEFAULT_CFG = {
  command:         'record',
  robot:           'default',
  route:           'percurso_initial',
  collect:         true,
  topics:          ['scan', 'odom', 'pose'],
  initial_pose:    [0, 0, 0],
  points:          [[0.5, 0.0, 0.0], [1.0, 0.0, 0.0]],
  return_to_start: [0, 0, 0],
}

// ── Estilos base ─────────────────────────────────────────────────────────────
const S = {
  label:    { fontSize: '0.72rem', color: '#8b92a8', textTransform: 'uppercase', letterSpacing: '0.05em', marginBottom: '0.3rem', display: 'block' },
  input:    { background: '#0d0f14', border: '1px solid #2a3142', borderRadius: '6px', color: '#e6e9ef', padding: '0.4rem 0.6rem', fontSize: '0.85rem', outline: 'none', fontFamily: 'monospace', width: '100%', boxSizing: 'border-box' },
  numInput: { background: '#0d0f14', border: '1px solid #2a3142', borderRadius: '6px', color: '#e6e9ef', padding: '0.4rem 0.5rem', fontSize: '0.82rem', outline: 'none', fontFamily: 'monospace', width: '70px', textAlign: 'center' },
  select:   { background: '#0d0f14', border: '1px solid #2a3142', borderRadius: '6px', color: '#e6e9ef', padding: '0.4rem 0.6rem', fontSize: '0.85rem', outline: 'none', width: '100%', boxSizing: 'border-box' },
  section:  { background: '#161a22', border: '1px solid #2a3142', borderRadius: '8px', padding: '0.85rem 1rem', display: 'flex', flexDirection: 'column', gap: '0.65rem' },
  row:      { display: 'flex', gap: '0.75rem', alignItems: 'flex-start' },
  field:    { display: 'flex', flexDirection: 'column', flex: 1 },
}

function btn(bg, color, disabled = false) {
  return { background: bg, color, border: `1px solid ${color}`, borderRadius: '8px', padding: '0.45rem 0.9rem', fontSize: '0.85rem', fontWeight: 600, cursor: disabled ? 'not-allowed' : 'pointer', opacity: disabled ? 0.55 : 1, fontFamily: 'inherit' }
}

const HOUSE_WORLD = { x0: -5.0, y0: -5.0, x1: 7.0, y1: 7.0 }

// ── MapView ───────────────────────────────────────────────────────────────────
function MapView({ robotPose, waypoints, onAddWaypoint, onNavigateTo }) {
  // mantém compatibilidade de estado existente + novos
  const canvasRef    = useRef(null)
  const imgRef       = useRef(null)   // SLAM PNG
  const floorImgRef  = useRef(null)   // Planta estática
  const mapMeta      = useRef(null)
  const scaleRef     = useRef(1)
  const panRef       = useRef({ x: 0, y: 0 })
  const dragRef      = useRef(null)
  const [clickMode, setClickMode]       = useState('waypoint')
  const [mapOk, setMapOk]               = useState(false)
  const [showFloor, setShowFloor]       = useState(true)
  const [floorOpacity, setFloorOpacity] = useState(0.55)
  const [savedMapMeta, setSavedMapMeta] = useState(null)
  const [savingMap, setSavingMap]       = useState(false)
  const [saveMsg, setSaveMsg]           = useState(null)
  const [hoverCoord, setHoverCoord]     = useState(null)  // {x, y} em coords mundo
  const [floorOffset] = useState({ x: HOUSE_WORLD.x0, y: HOUSE_WORLD.y0 })
  const [floorScale]  = useState(HOUSE_WORLD.x1 - HOUSE_WORLD.x0)

  // Converte coords mundo → canvas
  const worldToCanvas = useCallback((wx, wy) => {
    const m = mapMeta.current
    if (!m) return null
    const px = (wx - m.origin_x) / m.resolution
    const py = m.height - (wy - m.origin_y) / m.resolution
    return {
      x: px * scaleRef.current + panRef.current.x,
      y: py * scaleRef.current + panRef.current.y,
    }
  }, [])

  // Converte canvas → coords mundo
  const canvasToWorld = useCallback((cx, cy) => {
    const m = mapMeta.current
    if (!m) return null
    const px = (cx - panRef.current.x) / scaleRef.current
    const py = (cy - panRef.current.y) / scaleRef.current
    return {
      x: parseFloat((m.origin_x + px * m.resolution).toFixed(3)),
      y: parseFloat((m.origin_y + (m.height - py) * m.resolution).toFixed(3)),
    }
  }, [])

  // Desenha tudo no canvas
  const draw = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    const W = canvas.width, H = canvas.height
    ctx.clearRect(0, 0, W, H)

    // ── Fundo com grid pontilhada ─────────────────────────────────────────────
    ctx.fillStyle = '#0f1117'
    ctx.fillRect(0, 0, W, H)
    ctx.fillStyle = 'rgba(255,255,255,0.04)'
    const gridStep = 24
    for (let gx = 0; gx < W; gx += gridStep)
      for (let gy = 0; gy < H; gy += gridStep) {
        ctx.beginPath(); ctx.arc(gx, gy, 0.8, 0, Math.PI * 2); ctx.fill()
      }

    if (!mapMeta.current) {
      if (showFloor && floorImgRef.current?.complete) {
        const fi = floorImgRef.current
        const s = Math.min(W / fi.naturalWidth, H / fi.naturalHeight) * 0.88
        ctx.globalAlpha = floorOpacity
        ctx.drawImage(fi, (W - fi.naturalWidth * s) / 2, (H - fi.naturalHeight * s) / 2,
                      fi.naturalWidth * s, fi.naturalHeight * s)
        ctx.globalAlpha = 1
      }
      ctx.fillStyle = '#6b7280'
      ctx.font = '13px system-ui'
      ctx.textAlign = 'center'
      ctx.textBaseline = 'middle'
      ctx.fillText('⏳ Aguardando mapa SLAM…', W / 2, H / 2)
      return
    }

    // ── Planta de fundo ───────────────────────────────────────────────────────
    if (showFloor && floorImgRef.current?.complete) {
      const m = mapMeta.current, fi = floorImgRef.current
      ctx.globalAlpha = floorOpacity
      ctx.imageSmoothingEnabled = true
      if (savedMapMeta) {
        const sm = savedMapMeta
        const scaleX = (sm.width * sm.resolution) / (fi.naturalWidth * m.resolution) * scaleRef.current
        const scaleY = (sm.height * sm.resolution) / (fi.naturalHeight * m.resolution) * scaleRef.current
        const cx0 = (sm.origin_x - m.origin_x) / m.resolution * scaleRef.current + panRef.current.x
        const cy0 = (m.height - (sm.origin_y - m.origin_y) / m.resolution - sm.height * sm.resolution / m.resolution) * scaleRef.current + panRef.current.y
        ctx.drawImage(fi, cx0, cy0, fi.naturalWidth * scaleX, fi.naturalHeight * scaleY)
      } else {
        const pxPerMeter = scaleRef.current / m.resolution
        const imgW_px = floorScale * pxPerMeter
        const imgH_px = (fi.naturalHeight / fi.naturalWidth) * imgW_px
        const cx0 = (floorOffset.x - m.origin_x) / m.resolution * scaleRef.current + panRef.current.x
        const cy0 = (m.height - (floorOffset.y + floorScale * (fi.naturalHeight / fi.naturalWidth) - m.origin_y) / m.resolution) * scaleRef.current + panRef.current.y
        ctx.drawImage(fi, cx0, cy0, imgW_px, imgH_px)
      }
      ctx.globalAlpha = 1
    }

    // ── Mapa SLAM ─────────────────────────────────────────────────────────────
    if (imgRef.current?.complete) {
      const m = mapMeta.current
      ctx.imageSmoothingEnabled = false
      ctx.globalAlpha = showFloor ? 0.85 : 1
      ctx.drawImage(imgRef.current,
        panRef.current.x, panRef.current.y,
        m.width * scaleRef.current, m.height * scaleRef.current)
      ctx.globalAlpha = 1
    }

    // ── Linha de percurso entre waypoints ─────────────────────────────────────
    const wps = waypoints.map(wp => worldToCanvas(wp[0], wp[1])).filter(Boolean)
    if (wps.length > 1) {
      ctx.beginPath()
      ctx.moveTo(wps[0].x, wps[0].y)
      wps.slice(1).forEach(c => ctx.lineTo(c.x, c.y))
      ctx.strokeStyle = 'rgba(99,102,241,0.4)'
      ctx.lineWidth = 2
      ctx.setLineDash([6, 4])
      ctx.stroke()
      ctx.setLineDash([])
    }

    // ── Waypoints ─────────────────────────────────────────────────────────────
    wps.forEach((c, i) => {
      // Halo exterior
      ctx.beginPath()
      ctx.arc(c.x, c.y, 13, 0, Math.PI * 2)
      ctx.fillStyle = 'rgba(99,102,241,0.18)'
      ctx.fill()
      // Círculo principal
      ctx.beginPath()
      ctx.arc(c.x, c.y, 9, 0, Math.PI * 2)
      ctx.fillStyle = '#6366f1'
      ctx.fill()
      ctx.strokeStyle = '#a5b4fc'
      ctx.lineWidth = 1.5
      ctx.stroke()
      // Número
      ctx.fillStyle = '#fff'
      ctx.font = 'bold 9px monospace'
      ctx.textAlign = 'center'
      ctx.textBaseline = 'middle'
      ctx.fillText(String(i + 1), c.x, c.y)
    })

    // ── Robô ──────────────────────────────────────────────────────────────────
    if (robotPose?.valid) {
      const c = worldToCanvas(robotPose.x, robotPose.y)
      if (c) {
        ctx.save()
        ctx.translate(c.x, c.y)
        ctx.rotate(-robotPose.yaw)
        // Anel exterior pulsante
        ctx.beginPath()
        ctx.arc(0, 0, 18, 0, Math.PI * 2)
        ctx.fillStyle = 'rgba(110,231,183,0.12)'
        ctx.fill()
        ctx.strokeStyle = 'rgba(110,231,183,0.35)'
        ctx.lineWidth = 1.5
        ctx.stroke()
        // Corpo
        ctx.shadowColor = '#6ee7b7'
        ctx.shadowBlur = 14
        ctx.beginPath()
        ctx.arc(0, 0, 11, 0, Math.PI * 2)
        ctx.fillStyle = '#065f46'
        ctx.fill()
        ctx.strokeStyle = '#6ee7b7'
        ctx.lineWidth = 2
        ctx.stroke()
        ctx.shadowBlur = 0
        // Seta de direcção
        ctx.beginPath()
        ctx.moveTo(14, 0)
        ctx.lineTo(-5, -5.5)
        ctx.lineTo(-5, 5.5)
        ctx.closePath()
        ctx.fillStyle = '#6ee7b7'
        ctx.fill()
        ctx.restore()
      }
    }

    // ── Barra de escala ───────────────────────────────────────────────────────
    if (mapMeta.current) {
      const m = mapMeta.current
      const metersPerPixel = m.resolution / scaleRef.current
      // Escolhe um valor "bonito" para a escala
      const targets = [0.1, 0.25, 0.5, 1, 2, 5, 10]
      const targetPx = 80
      const target = targets.reduce((best, t) =>
        Math.abs(t / metersPerPixel - targetPx) < Math.abs(best / metersPerPixel - targetPx) ? t : best
      , targets[0])
      const barPx = target / metersPerPixel
      const bx = 16, by = H - 22
      ctx.fillStyle = 'rgba(0,0,0,0.5)'
      ctx.fillRect(bx - 4, by - 10, barPx + 8, 18)
      ctx.strokeStyle = '#6ee7b7'
      ctx.lineWidth = 2
      ctx.beginPath()
      ctx.moveTo(bx, by); ctx.lineTo(bx + barPx, by)
      ctx.moveTo(bx, by - 4); ctx.lineTo(bx, by + 4)
      ctx.moveTo(bx + barPx, by - 4); ctx.lineTo(bx + barPx, by + 4)
      ctx.stroke()
      ctx.fillStyle = '#6ee7b7'
      ctx.font = '10px monospace'
      ctx.textAlign = 'center'
      ctx.textBaseline = 'bottom'
      ctx.fillText(`${target < 1 ? target * 100 + 'cm' : target + 'm'}`, bx + barPx / 2, by - 3)
    }
  }, [robotPose, waypoints, worldToCanvas, showFloor, floorOpacity, floorOffset, floorScale, savedMapMeta])

  // Carrega fundo: prefere slam_map.png (mapa guardado) > house_map.png (decorativo)
  useEffect(() => {
    const loadFloor = (src, meta) => {
      const img = new Image()
      img.src = src + '?t=' + Date.now()
      img.onload = () => { floorImgRef.current = img; if (meta) setSavedMapMeta(meta); draw() }
    }
    // Verifica se existe slam_map.json guardado
    fetch(`${API}/slam_map_meta`)
      .then(r => r.json())
      .then(d => {
        if (d.available) loadFloor('/slam_map.png', d)
        else loadFloor('/house_map.png', null)
      })
      .catch(() => loadFloor('/house_map.png', null))
  }, [draw])

  // Busca mapa periodicamente
  useEffect(() => {
    let alive = true
    const fetchMap = async () => {
      try {
        const r = await fetch(`${API}/map`)
        const data = await r.json()
        if (!alive || !data.available) return
        mapMeta.current = {
          resolution: data.resolution,
          origin_x:   data.origin_x,
          origin_y:   data.origin_y,
          width:       data.width,
          height:      data.height,
        }
        if (!imgRef.current) imgRef.current = new Image()
        const dataUrl = `data:image/png;base64,${data.png_b64}`
        if (imgRef.current.src !== dataUrl) {
          imgRef.current.src = dataUrl
          imgRef.current.onload = () => {
            fitMap()
            setMapOk(true)
            draw()
          }
        } else {
          draw()
        }
      } catch { /* backend offline */ }
    }
    fetchMap()
    const t = setInterval(fetchMap, 1500)
    return () => { alive = false; clearInterval(t) }
  }, [draw])

  // Redesenha quando pose ou waypoints mudam
  useEffect(() => { draw() }, [draw, robotPose, waypoints])

  // Ajusta escala/pan para caber o mapa no canvas
  const fitMap = useCallback(() => {
    const canvas = canvasRef.current
    const m = mapMeta.current
    if (!canvas || !m) return
    const W = canvas.width, H = canvas.height
    if (!W || !H) return
    const s = Math.min(W / m.width, H / m.height) * 0.88
    scaleRef.current = s
    panRef.current   = { x: (W - m.width * s) / 2, y: (H - m.height * s) / 2 }
  }, [])

  // Redimensiona canvas ao container e re-centra o mapa
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ro = new ResizeObserver(() => {
      canvas.width  = canvas.offsetWidth
      canvas.height = canvas.offsetHeight
      if (mapMeta.current) fitMap()
      draw()
    })
    // Inicializa dimensões imediatamente
    canvas.width  = canvas.offsetWidth  || 600
    canvas.height = canvas.offsetHeight || 500
    ro.observe(canvas)
    return () => ro.disconnect()
  }, [draw, fitMap])

  // Zoom com scroll
  const onWheel = useCallback(e => {
    e.preventDefault()
    const canvas = canvasRef.current
    const rect   = canvas.getBoundingClientRect()
    const mx = e.clientX - rect.left
    const my = e.clientY - rect.top
    const factor = e.deltaY < 0 ? 1.15 : 1 / 1.15
    const newScale = Math.max(0.2, Math.min(scaleRef.current * factor, 50))
    panRef.current = {
      x: mx - (mx - panRef.current.x) * (newScale / scaleRef.current),
      y: my - (my - panRef.current.y) * (newScale / scaleRef.current),
    }
    scaleRef.current = newScale
    draw()
  }, [draw])

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    canvas.addEventListener('wheel', onWheel, { passive: false })
    return () => canvas.removeEventListener('wheel', onWheel)
  }, [onWheel])

  // Pan com botão direito ou middle
  const onMouseDown = e => {
    if (e.button === 1 || e.button === 2 || e.altKey) {
      dragRef.current = { x: e.clientX - panRef.current.x, y: e.clientY - panRef.current.y }
      e.preventDefault()
    }
  }
  const onMouseMove = e => {
    // Actualiza coordenadas de hover
    const rect = canvasRef.current?.getBoundingClientRect()
    if (rect) {
      const w = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top)
      setHoverCoord(w)
    }
    if (!dragRef.current) return
    panRef.current = { x: e.clientX - dragRef.current.x, y: e.clientY - dragRef.current.y }
    draw()
  }
  const onMouseLeave = () => { dragRef.current = null; setHoverCoord(null) }
  const onMouseUp    = () => { dragRef.current = null }

  // Clique → acção (só botão esquerdo, sem drag)
  const onClick = e => {
    if (!mapMeta.current || dragRef.current) return
    const rect = canvasRef.current.getBoundingClientRect()
    const world = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top)
    if (!world) return
    if (clickMode === 'waypoint') onAddWaypoint(world.x, world.y)
    else onNavigateTo(world.x, world.y)
  }

  const modeBtn = (mode, label, active_color) => (
    <button onClick={() => setClickMode(mode)}
      style={{
        background: clickMode === mode ? active_color + '22' : 'transparent',
        color: clickMode === mode ? active_color : '#6b7280',
        border: `1.5px solid ${clickMode === mode ? active_color : '#2a3142'}`,
        borderRadius: '6px', padding: '0.3rem 0.75rem',
        fontSize: '0.75rem', fontWeight: 600, cursor: 'pointer', fontFamily: 'inherit',
        transition: 'all 0.15s',
      }}>
      {label}
    </button>
  )

  const saveBackground = async () => {
    setSavingMap(true); setSaveMsg(null)
    try {
      const r = await fetch(`${API}/save_background_map`, { method: 'POST' })
      const d = await r.json()
      setSaveMsg(d.success ? 'ok' : 'erro')
      if (d.success) {
        const img = new Image()
        img.src = '/slam_map.png?t=' + Date.now()
        img.onload = () => {
          floorImgRef.current = img
          fetch(`${API}/slam_map_meta`).then(r => r.json()).then(d => {
            if (d.available) setSavedMapMeta(d); draw()
          })
        }
      }
    } catch { setSaveMsg('erro') }
    finally { setSavingMap(false); setTimeout(() => setSaveMsg(null), 4000) }
  }

  return (
    <div style={{ display: 'flex', flexDirection: 'column', height: '100%', gap: '0.5rem' }}>

      {/* ── Barra de ferramentas ───────────────────────────────────────────── */}
      <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', flexShrink: 0,
                    background: '#161a22', borderRadius: '8px', padding: '0.4rem 0.7rem',
                    border: '1px solid #2a3142' }}>

        {/* Modos de clique */}
        {modeBtn('waypoint', '＋ Waypoint', '#6366f1')}
        {modeBtn('navigate', '→ Ir agora',  '#6ee7b7')}

        <div style={{ width: '1px', height: '20px', background: '#2a3142', margin: '0 0.2rem' }} />

        {/* Centrar */}
        {mapOk && (
          <button onClick={() => { fitMap(); draw() }}
            style={{ background: 'transparent', color: '#6b7280', border: '1.5px solid #2a3142',
                     borderRadius: '6px', padding: '0.3rem 0.65rem', fontSize: '0.75rem',
                     fontWeight: 600, cursor: 'pointer', fontFamily: 'inherit' }}>
            ⊞ Centrar
          </button>
        )}

        {/* Planta / Salvar */}
        <button onClick={() => setShowFloor(v => !v)}
          style={{ background: showFloor ? 'rgba(110,231,183,0.1)' : 'transparent',
                   color: showFloor ? '#6ee7b7' : '#6b7280',
                   border: `1.5px solid ${showFloor ? '#6ee7b7' : '#2a3142'}`,
                   borderRadius: '6px', padding: '0.3rem 0.65rem', fontSize: '0.75rem',
                   fontWeight: 600, cursor: 'pointer', fontFamily: 'inherit' }}>
          🏠 {savedMapMeta ? 'Fundo' : 'Planta'}
        </button>

        {showFloor && (
          <input type="range" min="0" max="1" step="0.05" value={floorOpacity}
            onChange={e => { setFloorOpacity(parseFloat(e.target.value)); draw() }}
            style={{ width: '56px', accentColor: '#6ee7b7', cursor: 'pointer' }} />
        )}

        {mapOk && (
          <button onClick={saveBackground} disabled={savingMap}
            style={{ background: 'transparent',
                     color: saveMsg === 'ok' ? '#6ee7b7' : saveMsg === 'erro' ? '#f87171' : '#6366f1',
                     border: `1.5px solid ${saveMsg === 'ok' ? '#6ee7b7' : saveMsg === 'erro' ? '#f87171' : '#6366f1'}`,
                     borderRadius: '6px', padding: '0.3rem 0.65rem', fontSize: '0.75rem',
                     fontWeight: 600, cursor: savingMap ? 'not-allowed' : 'pointer',
                     opacity: savingMap ? 0.6 : 1, fontFamily: 'inherit' }}>
            {savingMap ? '⏳' : saveMsg === 'ok' ? '✓ Salvo' : saveMsg === 'erro' ? '✗ Erro' : '💾 Salvar'}
          </button>
        )}

        {/* Coordenadas de hover */}
        <div style={{ marginLeft: 'auto', fontFamily: 'monospace', fontSize: '0.72rem',
                      color: hoverCoord ? '#93c5fd' : '#374151', minWidth: '120px', textAlign: 'right' }}>
          {hoverCoord
            ? `x ${hoverCoord.x.toFixed(2)}  y ${hoverCoord.y.toFixed(2)}`
            : mapOk ? 'scroll=zoom  btn2=pan' : <span style={{ color: '#fbbf24' }}>⏳ aguardando SLAM…</span>
          }
        </div>
      </div>

      {/* ── Canvas do mapa ────────────────────────────────────────────────── */}
      <canvas
        ref={canvasRef}
        onClick={onClick}
        onMouseDown={onMouseDown}
        onMouseMove={onMouseMove}
        onMouseUp={onMouseUp}
        onMouseLeave={onMouseLeave}
        onContextMenu={e => e.preventDefault()}
        style={{
          flex: 1,
          width: '100%',
          borderRadius: '10px',
          border: '1px solid #1e2535',
          boxShadow: '0 4px 24px rgba(0,0,0,0.4)',
          cursor: clickMode === 'navigate' ? 'crosshair' : 'cell',
          display: 'block',
        }}
      />
    </div>
  )
}

// ── Toggle Record/Replay ──────────────────────────────────────────────────────
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

// ── Lista de waypoints ────────────────────────────────────────────────────────
function WaypointList({ points, onChange }) {
  const update = (i, j, val) => onChange(points.map((p, pi) => pi === i ? p.map((v, vi) => vi === j ? parseFloat(val) || 0 : v) : p))
  const add    = ()  => onChange([...points, [0, 0, 0]])
  const remove = i   => onChange(points.filter((_, pi) => pi !== i))

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '0.4rem' }}>
      {points.map((p, i) => (
        <div key={i} style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
          <span style={{ fontSize: '0.75rem', color: '#6366f1', width: '20px', textAlign: 'right', fontWeight: 700 }}>#{i + 1}</span>
          {['x', 'y', 'yaw'].map((lbl, j) => (
            <div key={lbl} style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '0.1rem' }}>
              <span style={{ fontSize: '0.65rem', color: '#4b5563' }}>{lbl}</span>
              <input type="number" step="0.1" value={p[j]} onChange={e => update(i, j, e.target.value)} style={S.numInput} />
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

// ── XYYaw ─────────────────────────────────────────────────────────────────────
function XYYaw({ value, onChange, label }) {
  const update = (i, val) => { const next = [...value]; next[i] = parseFloat(val) || 0; onChange(next) }
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '0.3rem' }}>
      {label && <span style={S.label}>{label}</span>}
      <div style={{ display: 'flex', gap: '0.5rem', alignItems: 'center' }}>
        {['x', 'y', 'yaw'].map((lbl, i) => (
          <div key={lbl} style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '0.1rem' }}>
            <span style={{ fontSize: '0.65rem', color: '#4b5563' }}>{lbl}</span>
            <input type="number" step="0.1" value={value[i]} onChange={e => update(i, e.target.value)} style={S.numInput} />
          </div>
        ))}
      </div>
    </div>
  )
}

// ── Formulário de configuração ────────────────────────────────────────────────
function ConfigForm({ cfg, onChange, currentPose }) {
  const set = (key, val) => onChange({ ...cfg, [key]: val })

  const toggleTopic = t => {
    const next = cfg.topics.includes(t) ? cfg.topics.filter(x => x !== t) : [...cfg.topics, t]
    set('topics', next)
  }

  const usarPoseAtual = () => {
    if (!currentPose?.valid) return
    set('initial_pose', [
      parseFloat(currentPose.x.toFixed(3)),
      parseFloat(currentPose.y.toFixed(3)),
      parseFloat(currentPose.yaw.toFixed(3)),
    ])
  }

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>

      {/* Comando + Robô + Rota */}
      <div style={S.section}>
        <div style={S.row}>
          <div style={S.field}>
            <span style={S.label}>Comando</span>
            <CmdToggle value={cfg.command} onChange={v => set('command', v)} />
          </div>
          <div style={{ ...S.field, maxWidth: '110px' }}>
            <span style={S.label}>Robô</span>
            <select value={cfg.robot} onChange={e => set('robot', e.target.value)} style={S.select}>
              <option value="default">único</option>
              <option value="tb1">tb1</option>
              <option value="tb2">tb2</option>
            </select>
          </div>
        </div>
        <div style={S.field}>
          <span style={S.label}>Nome da rota</span>
          <input value={cfg.route} onChange={e => set('route', e.target.value)} placeholder="percurso_initial" style={S.input} />
        </div>
      </div>

      {/* Coleta */}
      <div style={S.section}>
        <label style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', cursor: 'pointer' }}>
          <input type="checkbox" checked={cfg.collect} onChange={e => set('collect', e.target.checked)} style={{ accentColor: '#6ee7b7', width: 16, height: 16 }} />
          <span style={{ fontSize: '0.85rem', color: cfg.collect ? '#6ee7b7' : '#8b92a8', fontWeight: 600 }}>Gravar bag</span>
        </label>
        {cfg.collect && (
          <div>
            <span style={S.label}>Tópicos</span>
            <div style={{ display: 'flex', gap: '0.5rem', flexWrap: 'wrap' }}>
              {ALL_TOPICS.map(t => (
                <label key={t} style={{ display: 'flex', alignItems: 'center', gap: '0.35rem', cursor: 'pointer',
                  background: cfg.topics.includes(t) ? 'rgba(99,102,241,0.12)' : 'transparent',
                  border: `1px solid ${cfg.topics.includes(t) ? '#6366f1' : '#2a3142'}`,
                  borderRadius: '6px', padding: '0.3rem 0.65rem' }}>
                  <input type="checkbox" checked={cfg.topics.includes(t)} onChange={() => toggleTopic(t)} style={{ accentColor: '#6366f1', margin: 0 }} />
                  <span style={{ fontSize: '0.82rem', color: cfg.topics.includes(t) ? '#e6e9ef' : '#8b92a8', fontFamily: 'monospace' }}>{t}</span>
                </label>
              ))}
            </div>
          </div>
        )}
      </div>

      {/* Pose inicial */}
      <div style={S.section}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
          <span style={S.label}>Pose inicial</span>
          <div style={{ display: 'flex', alignItems: 'center', gap: '0.6rem' }}>
            {currentPose?.valid && (
              <span style={{ fontSize: '0.7rem', color: '#4b5563', fontFamily: 'monospace' }}>
                ({currentPose.x.toFixed(2)}, {currentPose.y.toFixed(2)})
              </span>
            )}
            <button onClick={usarPoseAtual} disabled={!currentPose?.valid}
              style={{ ...btn('#161a22', currentPose?.valid ? '#6366f1' : '#2a3142', !currentPose?.valid), fontSize: '0.72rem', padding: '0.2rem 0.55rem' }}>
              ⊕ Pose atual
            </button>
          </div>
        </div>
        <XYYaw value={cfg.initial_pose} onChange={v => set('initial_pose', v)} />
      </div>

      {/* Waypoints / Retorno */}
      <div style={S.section}>
        {cfg.command === 'record' ? (
          <>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
              <span style={S.label}>Waypoints</span>
              <span style={{ fontSize: '0.7rem', color: '#6366f1' }}>ou clica no mapa</span>
            </div>
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
  const [cfg, setCfg]               = useState(DEFAULT_CFG)
  const [jobId, setJobId]           = useState(null)
  const [job, setJob]               = useState(null)
  const [running, setRunning]       = useState(false)
  const [error, setError]           = useState(null)
  const [showResult, setShowResult] = useState(false)
  const [status, setStatus]         = useState({ robots: [], pose: { x: 0, y: 0, yaw: 0, valid: false } })
  const [resetMsg, setResetMsg]     = useState(null)
  const [resetting, setResetting]   = useState(false)

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

  // Fecha painéis ao clicar fora
  useEffect(() => {
    const h = e => {
      if (panelRef.current && !panelRef.current.contains(e.target))    setConnPanel(false)
      if (discoverRef.current && !discoverRef.current.contains(e.target)) setDiscoverPanel(false)
    }
    document.addEventListener('mousedown', h)
    return () => document.removeEventListener('mousedown', h)
  }, [])

  // Polling de status
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

  // Adiciona waypoint vindo do clique no mapa
  const handleMapAddWaypoint = useCallback((x, y) => {
    setCfg(prev => ({ ...prev, points: [...prev.points, [x, y, 0]] }))
  }, [])

  // Navega directamente para ponto clicado no mapa
  const handleMapNavigateTo = useCallback(async (x, y) => {
    await fetch(`${API}/go_to_point?x=${x}&y=${y}&yaw=0`, { method: 'POST' }).catch(() => {})
  }, [])

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
      if (!data.found?.length) setDiscoverError(`Nenhum host encontrado em ${data.subnet_scanned || '(auto)'}.0/24`)
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
    if (!cfg.route.trim())                             { setError('Nome da rota vazio.'); return }
    if (cfg.command === 'record' && !cfg.points.length){ setError('Adiciona pelo menos 1 waypoint.'); return }
    if (cfg.collect && !cfg.topics.length)             { setError('Seleciona pelo menos 1 tópico.'); return }

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
        <div style={{ display: 'flex', fontSize: '0.78rem', fontFamily: 'monospace', alignItems: 'stretch', background: '#161a22', border: '1px solid #2a3142', borderRadius: '8px', overflow: 'hidden' }}>
          {[
            { label: 'Nav2',   value: nav2Ok ? 'ON' : 'OFF',           color: nav2Ok ? '#6ee7b7' : '#f87171' },
            { label: 'Nav',    value: robot.nav_state || '—',           color: robot.nav_state === 'navigating' ? '#6ee7b7' : robot.nav_state === 'failed' ? '#f87171' : '#e6e9ef' },
            { label: 'Coleta', value: robot.collection_on ? 'ON':'OFF', color: robot.collection_on ? '#6ee7b7' : '#8b92a8' },
            { label: 'x',      value: pose.valid ? pose.x.toFixed(3) : '—', color: pose.valid ? '#93c5fd' : '#8b92a8' },
            { label: 'y',      value: pose.valid ? pose.y.toFixed(3) : '—', color: pose.valid ? '#93c5fd' : '#8b92a8' },
            { label: 'yaw',    value: pose.valid ? `${deg(pose.yaw)}°` : '—', color: pose.valid ? '#93c5fd' : '#8b92a8' },
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

      {/* ── Barra de conexão ─────────────────────────────────────────────────── */}
      <div style={{ display: 'flex', gap: '0.75rem', alignItems: 'center', padding: '0.4rem 1.25rem', borderBottom: '1px solid #2a3142', flexShrink: 0, position: 'relative' }}>

        <div ref={panelRef} style={{ position: 'relative' }}>
          <button onClick={() => { setConnPanel(v => !v); setDiscoverPanel(false) }}
            style={{ ...btn('#161a22', '#6366f1'), display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
            <span style={{ width: 10, height: 10, borderRadius: '50%', background: connLight, display: 'inline-block', boxShadow: connStatus === 'connected' ? `0 0 6px ${connLight}` : 'none' }} />
            Conectar robô <span style={{ fontSize: '0.7rem', opacity: 0.7 }}>{connPanel ? '▲' : '▼'}</span>
          </button>
          {connPanel && (
            <div style={{ position: 'absolute', top: '2.5rem', left: 0, zIndex: 100, background: '#161a22', border: '1px solid #2a3142', borderRadius: '10px', padding: '1rem', width: '320px', boxShadow: '0 8px 32px rgba(0,0,0,0.5)' }}>
              <div style={{ fontSize: '0.78rem', color: '#8b92a8', marginBottom: '0.75rem', textTransform: 'uppercase', letterSpacing: '0.05em' }}>Selecionar robô</div>
              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem', marginBottom: '1rem' }}>
                {[...ROBOT_PROFILES, ...extraProfiles].map(p => {
                  const isSelected = selectedProfile === p.id
                  const isExtra    = !!extraProfiles.find(ep => ep.id === p.id)
                  return (
                    <label key={p.id} onClick={() => setSelectedProfile(p.id)}
                      style={{ display: 'flex', alignItems: 'center', gap: '0.75rem', padding: '0.5rem 0.85rem', borderRadius: '8px', border: `1px solid ${isSelected ? '#6366f1' : '#2a3142'}`, background: isSelected ? 'rgba(99,102,241,0.1)' : 'transparent', cursor: 'pointer' }}>
                      <input type="radio" name="rp" checked={isSelected} onChange={() => setSelectedProfile(p.id)} style={{ accentColor: '#6366f1', margin: 0 }} />
                      <div style={{ flex: 1 }}>
                        <div style={{ fontSize: '0.85rem', color: isSelected ? '#e6e9ef' : '#a0aec0', fontWeight: isSelected ? 600 : 400 }}>{p.label}</div>
                        <div style={{ fontSize: '0.72rem', color: '#4b5563', marginTop: '0.1rem' }}>
                          {isExtra ? `SSH ${p.host} · descoberto` : p.type === 'ssh' ? `SSH ${p.host} · em breve` : 'localhost:8000'}
                        </div>
                      </div>
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
            🔍 Procurar <span style={{ fontSize: '0.7rem', opacity: 0.7 }}>{discoverPanel ? '▲' : '▼'}</span>
          </button>
          {discoverPanel && (
            <div style={{ position: 'absolute', top: '2.5rem', left: 0, zIndex: 100, background: '#161a22', border: '1px solid #2a3142', borderRadius: '10px', padding: '1rem', width: '380px', boxShadow: '0 8px 32px rgba(0,0,0,0.5)' }}>
              <div style={{ fontSize: '0.78rem', color: '#8b92a8', marginBottom: '0.75rem', textTransform: 'uppercase', letterSpacing: '0.05em' }}>Descoberta na rede</div>
              <div style={{ display: 'flex', gap: '0.5rem', marginBottom: '0.75rem' }}>
                <input placeholder="Subnet (ex: 192.168.1) — auto" value={subnet} onChange={e => setSubnet(e.target.value)} style={{ ...S.input, flex: 1 }} />
                <input placeholder="user SSH" value={sshUser} onChange={e => setSshUser(e.target.value)} style={{ ...S.input, width: '80px' }} />
              </div>
              <button onClick={discoverRobots} disabled={discovering} style={{ ...btn('#161a22', '#fbbf24'), width: '100%', marginBottom: '0.75rem' }}>
                {discovering ? '⏳ Varrendo…' : '▶ Varrer rede'}
              </button>
              {discoverError && <div style={{ color: '#f87171', fontSize: '0.78rem' }}>{discoverError}</div>}
              {discovered.map(h => {
                const st = testingSSH[h.ip]
                const stColor = st === 'ok' ? '#6ee7b7' : st === 'no_ros' ? '#fbbf24' : st === 'error' ? '#f87171' : '#8b92a8'
                return (
                  <div key={h.ip} style={{ display: 'flex', alignItems: 'center', gap: '0.6rem', background: '#0d0f14', borderRadius: '6px', padding: '0.4rem 0.65rem', border: '1px solid #2a3142', marginTop: '0.4rem' }}>
                    <div style={{ flex: 1 }}>
                      <div style={{ fontSize: '0.82rem', fontFamily: 'monospace' }}>{h.ip}</div>
                      <div style={{ fontSize: '0.7rem', color: '#4b5563' }}>{h.hostname}</div>
                    </div>
                    <button onClick={() => testSSH(h.ip)} disabled={st === 'testing'} style={{ ...btn('#161a22', stColor), fontSize: '0.72rem', padding: '0.25rem 0.5rem' }}>
                      {st === 'testing' ? '⏳' : st === 'ok' ? '✓ ROS2' : st === 'no_ros' ? '⚠ sem ROS' : st === 'error' ? '✗' : 'Testar'}
                    </button>
                  </div>
                )
              })}
            </div>
          )}
        </div>

        {isConnected && <span style={{ fontSize: '0.8rem', color: '#6ee7b7', fontFamily: 'monospace' }}>● Conectado — {[...ROBOT_PROFILES, ...extraProfiles].find(p => p.id === selectedProfile)?.label}</span>}
        {connStatus === 'error' && <span style={{ fontSize: '0.8rem', color: '#f87171', fontFamily: 'monospace' }}>✗ {connMsg}</span>}
      </div>

      {/* ── Layout 3 colunas ─────────────────────────────────────────────────── */}
      <div style={{ display: 'grid', gridTemplateColumns: '300px 1fr 340px', gap: '0.75rem', flex: 1, minHeight: 0, padding: '0.75rem 1rem 0.75rem' }}>

        {/* Coluna esquerda: formulário */}
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem', overflowY: 'auto', paddingRight: '0.25rem' }}>
          <span style={{ fontSize: '0.8rem', fontWeight: 600, color: '#6ee7b7', textTransform: 'uppercase', letterSpacing: '0.05em', flexShrink: 0 }}>Configuração</span>
          <ConfigForm cfg={cfg} onChange={setCfg} currentPose={pose} onAddWaypoint={handleMapAddWaypoint} />

          {!isConnected && (
            <div style={{ color: '#fbbf24', fontSize: '0.82rem', padding: '0.5rem 0.75rem', background: 'rgba(251,191,36,0.08)', borderRadius: '6px', border: '1px solid rgba(251,191,36,0.2)' }}>
              ⚠ Conecte um robô antes de executar.
            </div>
          )}
          {isConnected && !nav2Ok && (
            <div style={{ color: '#fbbf24', fontSize: '0.82rem', padding: '0.5rem 0.75rem', background: 'rgba(251,191,36,0.08)', borderRadius: '6px', border: '1px solid rgba(251,191,36,0.2)' }}>
              ⚠ Nav2 offline — aguarda ~60s após o Terminal 2.
            </div>
          )}
          {error && <div style={{ color: '#f87171', fontSize: '0.82rem', padding: '0.5rem 0.75rem', background: 'rgba(248,113,113,0.1)', borderRadius: '6px' }}>{error}</div>}

          <div style={{ display: 'flex', gap: '0.6rem' }}>
            <button onClick={run} disabled={running || !isConnected}
              style={btn(running || !isConnected ? '#1a3a2a' : '#065f46', '#6ee7b7', running || !isConnected)}>
              {running ? '⏳ A executar…' : `▶ Executar ${cfg.command}`}
            </button>
            {running && (
              <button onClick={() => { if (pollRef.current) clearInterval(pollRef.current); setRunning(false) }}
                style={btn('#3a1a1a', '#f87171')}>■ Parar</button>
            )}
          </div>
        </div>

        {/* Coluna central: mapa */}
        <div style={{ display: 'flex', flexDirection: 'column', minHeight: 0 }}>
          <span style={{ fontSize: '0.8rem', fontWeight: 600, color: '#8b92a8', textTransform: 'uppercase', letterSpacing: '0.05em', marginBottom: '0.5rem', flexShrink: 0 }}>Mapa</span>
          <div style={{ flex: 1, minHeight: 0 }}>
            <MapView
              robotPose={pose}
              waypoints={cfg.command === 'record' ? cfg.points : []}
              onAddWaypoint={handleMapAddWaypoint}
              onNavigateTo={handleMapNavigateTo}
            />
          </div>
        </div>

        {/* Coluna direita: output */}
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem', minHeight: 0, overflow: 'hidden' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', flexShrink: 0 }}>
            <span style={{ fontSize: '0.8rem', fontWeight: 600, color: '#8b92a8', textTransform: 'uppercase', letterSpacing: '0.05em' }}>
              Output {jobId && <span style={{ color: '#3b82f6', fontWeight: 400 }}>#{jobId}</span>}
            </span>
            {job && !job.running && (
              <span style={{ fontSize: '0.82rem', color: job.exit_code === 0 ? '#6ee7b7' : '#f87171', fontWeight: 600 }}>
                {job.exit_code === 0 ? '✓ Sucesso' : `✗ exit ${job.exit_code}`}
              </span>
            )}
          </div>

          <div ref={outputRef} style={{ flex: 1, background: '#161a22', border: '1px solid #2a3142', borderRadius: '8px', padding: '0.6rem 0.85rem', overflowY: 'auto', fontFamily: 'JetBrains Mono, Consolas, monospace', fontSize: '0.75rem', lineHeight: 1.6 }}>
            {!job && !running && <span style={{ color: '#8b92a8' }}>Aguardando execução…</span>}
            {job?.lines?.map((line, i) => (
              <div key={i} style={{ color: lineColor(line), whiteSpace: 'pre-wrap', wordBreak: 'break-word' }}>{line}</div>
            ))}
            {running && <div style={{ color: '#8b92a8', animation: 'blink 1s step-end infinite' }}>▌</div>}
          </div>

          {/* Métricas */}
          {job?.result?.bag_metrics && (() => {
            const m = job.result.bag_metrics
            const cards = [
              m.wall_duration_s      != null && { label: 'Duração',      value: `${m.wall_duration_s} s`,      color: '#93c5fd' },
              m.odom_path_length_m   != null && { label: 'Percurso',     value: `${m.odom_path_length_m} m`,   color: '#6ee7b7' },
              m.odom_avg_speed_ms    != null && { label: 'Vel. média',   value: `${m.odom_avg_speed_ms} m/s`,  color: '#6ee7b7' },
              m.scan_avg_valid_points!= null && { label: 'Scan pts',     value: `${m.scan_avg_valid_points}`,  color: '#fbbf24' },
              m.imu_accel_mean_ms2   != null && { label: 'IMU accel',    value: `${m.imu_accel_mean_ms2} m/s²`,color: '#c4b5fd' },
            ].filter(Boolean)
            return cards.length > 0 && (
              <div style={{ background: '#161a22', border: '1px solid #2a3142', borderRadius: '8px', padding: '0.6rem 0.85rem', flexShrink: 0 }}>
                <div style={{ fontSize: '0.68rem', color: '#8b92a8', textTransform: 'uppercase', letterSpacing: '0.05em', marginBottom: '0.4rem' }}>Métricas</div>
                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.35rem' }}>
                  {cards.map(({ label, value, color }) => (
                    <div key={label} style={{ background: '#0d0f14', borderRadius: '5px', padding: '0.3rem 0.5rem' }}>
                      <div style={{ fontSize: '0.62rem', color: '#8b92a8' }}>{label}</div>
                      <div style={{ fontSize: '0.78rem', fontWeight: 700, color, fontFamily: 'monospace' }}>{value}</div>
                    </div>
                  ))}
                </div>
              </div>
            )
          })()}

          {job?.result && (
            <button onClick={() => setShowResult(true)} style={{ ...btn('#161a22', '#2a3142'), fontSize: '0.75rem', flexShrink: 0 }}>
              ▸ Ver resultado JSON
            </button>
          )}
        </div>
      </div>

      {/* Modal JSON */}
      {showResult && job?.result && (
        <div onClick={() => setShowResult(false)}
          style={{ position: 'fixed', inset: 0, zIndex: 200, background: 'rgba(0,0,0,0.75)', display: 'flex', alignItems: 'center', justifyContent: 'center', padding: '2rem' }}>
          <div onClick={e => e.stopPropagation()}
            style={{ background: '#161a22', border: '1px solid #2a3142', borderRadius: '12px', width: '100%', maxWidth: '800px', maxHeight: '80vh', display: 'flex', flexDirection: 'column', overflow: 'hidden' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', padding: '0.75rem 1.25rem', borderBottom: '1px solid #2a3142' }}>
              <span style={{ fontSize: '0.82rem', fontWeight: 600, color: '#8b92a8', textTransform: 'uppercase' }}>Resultado JSON</span>
              <button onClick={() => setShowResult(false)} style={{ background: 'none', border: 'none', color: '#8b92a8', cursor: 'pointer', fontSize: '1.1rem' }}>✕</button>
            </div>
            <pre style={{ flex: 1, overflowY: 'auto', margin: 0, padding: '1rem', fontFamily: 'monospace', fontSize: '0.8rem', lineHeight: 1.6, color: '#e6e9ef' }}>
              {JSON.stringify(job.result, null, 2)}
            </pre>
          </div>
        </div>
      )}

      <style>{`@keyframes blink { 0%,100%{opacity:1} 50%{opacity:0} }`}</style>
    </div>
  )
}
