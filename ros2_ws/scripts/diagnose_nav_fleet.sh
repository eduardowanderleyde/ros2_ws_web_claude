#!/usr/bin/env bash
# Diagnóstico: /odom, TF (odom->base_footprint, map->odom), /cmd_vel.
# TurtleBot3: child_frame de /odom costuma ser base_footprint; o frame "map" no TF
# só existe com AMCL localizado (ex.: 2D Pose Estimate no RViz).
# Uso: com Gazebo+Nav2 (+ fleet opcional) a correr:
#   bash scripts/diagnose_nav_fleet.sh
# Opcional: DIAG_OUT=/tmp/meus_diag.log bash scripts/diagnose_nav_fleet.sh
#
# timeout usa SIGINT para o tf2_echo terminar sem disparar tanto o "ros2 crash" do Ubuntu.
#
# Em simulação com Gazebo, TF e tópicos usam /clock. No Jazzy:
#   ros2 topic echo … --use-sim-time   (não use --ros-args nesse subcomando)
#   ros2 run tf2_ros tf2_echo --ros-args -p use_sim_time:=true -- frame1 frame2
# Robô real: DIAG_USE_WALL_CLOCK=1 bash scripts/diagnose_nav_fleet.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
INSTALL_SETUP="${WS_ROOT}/install/setup.bash"
OUT="${DIAG_OUT:-${WS_ROOT}/diagnostics/diag_$(date +%Y%m%d_%H%M%S).log}"

mkdir -p "$(dirname "$OUT")"

# Após exec, toda a saída já vai para o ficheiro + terminal.
log() { echo "$@"; }
section() {
  log ""
  log "======== $1 ========"
}

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERRO: /opt/ros/jazzy/setup.bash não encontrado." >&2
  exit 1
fi
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
if [[ -f "$INSTALL_SETUP" ]]; then
  # shellcheck source=/dev/null
  source "$INSTALL_SETUP"
else
  echo "AVISO: $INSTALL_SETUP não existe — rode colcon build no workspace." >&2
fi

if [[ "${DIAG_USE_WALL_CLOCK:-0}" == "1" ]]; then
  TOPIC_ECHO_SIM=()
  TF2_ECHO_SIM=()
else
  TOPIC_ECHO_SIM=(--use-sim-time)
  TF2_ECHO_SIM=(--ros-args -p use_sim_time:=true --)
fi

exec > >(tee -a "$OUT") 2>&1
log "Relatório: $OUT"
log "Data: $(date -Iseconds)"
log "ROS_DISTRO=${ROS_DISTRO:-}"

section "Tópicos relevantes (grep)"
ros2 topic list 2>&1 | grep -E '^/(odom|cmd_vel|map|scan|imu)' || true
log "(lista completa abaixo, primeiras 60 linhas)"
ros2 topic list 2>&1 | head -60

section "A — /odom (uma mensagem, timeout 5s)"
if ros2 topic list 2>&1 | grep -qx '/odom'; then
  ros2 topic echo /odom --once --timeout 5 "${TOPIC_ECHO_SIM[@]}" --qos-reliability reliable 2>&1 || log "[A] falhou ou sem mensagens no tempo."
else
  log "[A] SKIP: tópico /odom não existe (simulação/Nav2 provavelmente parados)."
fi

section "B1 — TF odom -> base_footprint (~5s, deve existir com Gazebo+Nav2)"
if ros2 topic list 2>&1 | grep -qx '/tf' || ros2 topic list 2>&1 | grep -qx '/tf_static'; then
  timeout -s INT 5 ros2 run tf2_ros tf2_echo "${TF2_ECHO_SIM[@]}" odom base_footprint 2>&1 | head -30 || log "[B1] falhou ou timeout."
else
  log "[B1] SKIP: sem /tf"
fi

section "B2 — TF map -> odom (~5s, AMCL; se falhar: 2D Pose Estimate no RViz)"
if ros2 topic list 2>&1 | grep -qx '/tf' || ros2 topic list 2>&1 | grep -qx '/tf_static'; then
  timeout -s INT 5 ros2 run tf2_ros tf2_echo "${TF2_ECHO_SIM[@]}" map odom 2>&1 | head -30 || log "[B2] falhou ou timeout."
  log "[B2] Nota: se \"map\" não existir, o AMCL ainda não publica map->odom. No RViz: 2D Pose Estimate."
else
  log "[B2] SKIP: sem /tf"
fi

section "B3 — TF map -> base_footprint (cadeia completa; requer B2 ok)"
if ros2 topic list 2>&1 | grep -qx '/tf' || ros2 topic list 2>&1 | grep -qx '/tf_static'; then
  timeout -s INT 5 ros2 run tf2_ros tf2_echo "${TF2_ECHO_SIM[@]}" map base_footprint 2>&1 | head -30 || log "[B3] falhou (normal se B2 falhou)."
else
  log "[B3] SKIP: sem /tf"
fi

section "C — /cmd_vel (uma mensagem, timeout 8s; vazio = normal sem goal de navegação)"
if ros2 topic list 2>&1 | grep -qx '/cmd_vel'; then
  ros2 topic echo /cmd_vel --once --timeout 8 "${TOPIC_ECHO_SIM[@]}" --qos-reliability reliable 2>&1 || log "[C] sem mensagens (esperado se o robô não está a receber comando)."
else
  log "[C] SKIP: tópico /cmd_vel não existe."
fi

section "Serviços fleet (amostra)"
ros2 service list 2>&1 | grep -E 'collection|go_to_point|play_route|fleet' | head -20 || true

section "Fim"
log "Gravado em: $OUT"
echo ""
echo "Concluído. Copie o ficheiro acima ou: cat \"$OUT\""
