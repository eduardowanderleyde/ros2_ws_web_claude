# Resultados Experimentais — Fleet Repeatability

Experimentos realizados em simulação Gazebo com TurtleBot3 Waffle, Nav2 + SLAM Toolbox, ROS 2 Jazzy.

---

## Experimento 1 — Linha reta (`teste_linha`)

**Data:** 2026-04-07  
**Rota:** ponto único em (1.37, 1.13) com yaw = -1.34 rad  
**Runs:** baseline + replay_01

### Configuração

```yaml
route: teste_linha
frame: map
poses:
  - x: 1.3728, y: 1.1267, yaw: -1.3435
```

### Resultados

| Métrica | baseline | replay_01 |
|---------|----------|-----------|
| Duração (s) | 3159.1 | 3191.4 |
| Comprimento (m) | 1.212 | 1.207 |
| Início (x, y) | (1.009, 0.296) | (1.001, 0.290) |
| Fim (x, y) | (1.373, 1.127) | (1.371, 1.136) |

| Par | RMSE (m) |
|-----|----------|
| baseline vs replay_01 | **0.0097** |

**Erro de endpoint:** 9.6 mm  
**Razão de duração:** 1.010 (100.1% do baseline)  
**Distância média ponto a ponto:** 9.7 mm

### Interpretação

Resultado excelente: RMSE de apenas **9.7 mm** entre baseline e replay. A trajetória é de curta distância em linha reta e o Nav2 reproduz o percurso com alta fidelidade. Demonstra que em condições controladas (mapa estático, sem obstáculos) a repetibilidade é muito alta.

---

## Experimento 2 — Percurso em L (`exp_val02`)

**Data:** 2026-04-06  
**Rota:** 4 waypoints formando percurso em L  
**Runs:** baseline + replay_01 + replay_02

### Configuração

```json
{
  "points": [
    [0.5, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.5, 0.5, 0.0],
    [2.0, 0.5, 0.0]
  ]
}
```

### Resultados

| Métrica | baseline | replay_01 | replay_02 |
|---------|----------|-----------|-----------|
| Duração (s) | 20.63 | 20.56 | 20.59 |
| Comprimento (m) | 2.288 | 2.304 | 2.306 |
| Início (x, y) | (0.140, 0.311) | (0.188, 0.222) | (0.236, 0.142) |
| Fim (x, y) | (1.811, 1.043) | (1.857, 1.008) | (1.868, 0.970) |

#### Matriz RMSE entre pares (m)

|           | baseline | replay_01 | replay_02 |
|-----------|----------|-----------|-----------|
| baseline  | 0.000 | **0.092** | **0.167** |
| replay_01 | 0.092 | 0.000 | **0.081** |
| replay_02 | 0.167 | 0.081 | 0.000 |

#### Vs. referência (baseline)

| Run | RMSE (m) | Erro endpoint (m) | Duração ratio | Dist. média (m) |
|-----|----------|-------------------|---------------|-----------------|
| replay_01 | 0.092 | 0.058 | 0.997 | 0.090 |
| replay_02 | 0.167 | 0.093 | 0.998 | 0.164 |

### Interpretação

O percurso em L de ~2.3 m apresenta repetibilidade moderada:

- **replay_01** ficou a **92 mm** (RMSE) do baseline — aceitável para navegação autônoma em ambiente simulado.
- **replay_02** apresentou maior desvio (**167 mm**), possivelmente por variação na posição inicial (início em x=0.236 vs 0.140 do baseline).
- A duração foi praticamente idêntica em todos os runs (razão ≈ 0.998), indicando que o Nav2 mantém velocidade consistente.
- RMSE entre os dois replays: **81 mm** — os replays são mais parecidos entre si do que com o baseline, sugerindo que a pose inicial influencia o resultado.

---

## Experimento 3 — Percurso mais longo (val01) — **inválido**

**Data:** 2026-04-06  
**Rota:** exp_val01 — percurso com mais waypoints  
**Runs:** baseline + replay_01 + replay_02

### Motivo de invalidade

| Métrica | baseline | replay_01 | replay_02 |
|---------|----------|-----------|-----------|
| Duração (s) | **45.54** | 12.56 | 12.56 |
| Comprimento (m) | **4.808** | 1.543 | 1.530 |
| Razão de duração | 1.000 | **0.276** | **0.276** |

Os replays completaram apenas **27.6%** da duração do baseline e percorreram apenas **1.5 m** dos **4.8 m** do baseline. O Nav2 não concluiu o percurso completo — possivelmente por timeout ou falha de planejamento em algum waypoint intermediário.

**RMSE baseline vs replay:** ~1.58 m (inválido — percurso incompleto).

---

## Comparativo geral

| Experimento | Percurso | N runs | RMSE médio (m) | Válido |
|-------------|----------|--------|----------------|--------|
| teste_linha | 1.21 m, linha | 2 | **0.0097** | ✓ |
| exp_val02 | 2.30 m, L shape | 3 | 0.092–0.167 | ✓ |
| exp_val01 | 4.81 m | 3 | ~1.58 | ✗ (incompleto) |

---

## Sensores gravados (exp_val02 — replay_01)

Dados da coleta MCAP (`/odom`, `/scan`, `/imu`):

| Tópico | Mensagens | Hz estimado |
|--------|-----------|-------------|
| `/imu` | 5412 | ~200 Hz |
| `/odom` | 1354 | ~50 Hz |
| `/scan` | 271 | ~10 Hz |

- **IMU a 200 Hz** — suficiente para detectar acelerações e trepidação durante a navegação.
- **Odom a 50 Hz** — resolução adequada para reconstrução de trajetória.
- **LiDAR a 10 Hz** — frequência padrão do TurtleBot3 LiDAR (HLDS HLS-LFCD2).

---

## Conclusões preliminares

1. **Alta repetibilidade em percursos curtos e simples:** RMSE < 10 mm para linha reta de ~1.2 m.
2. **Repetibilidade moderada em percursos com curvas:** RMSE 80–170 mm para percurso em L de ~2.3 m.
3. **Pose inicial impacta o resultado:** variações de ~10 cm na posição inicial (frame odom) propagam desvio na trajetória.
4. **Duração estável:** razão de duração ≈ 0.997–1.010 nos experimentos válidos, indicando que o Nav2 mantém velocidade consistente entre runs.
5. **Percursos longos (> 4 m) requerem atenção:** o exp_val01 falhou por percurso incompleto — necessário validar planejamento de trajetória antes de executar runs longos.
