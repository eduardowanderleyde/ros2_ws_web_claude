#!/usr/bin/env python3
"""
Pós-processamento de rosbag2 gravados pelo fleet_data_collector: extrai trajetória (AMCL no mapa
ou /odom), calcula métricas (duração, comprimento, RMSE entre pares, erro no ponto final vs
referência, desvio médio ponto a ponto vs referência, razão de duração), exporta CSV e gráfico.

Uso (com workspace ROS 2 sourceado):
  source install/setup.bash
  python3 scripts/analyze_runs.py collections/default/run_a collections/default/run_b
  python3 scripts/analyze_runs.py bag1 bag2 --trajectory-topic auto   # prefere /amcl_pose se existir

A **referência** é sempre o **primeiro** bag da lista (índice 0).

Dependências Python: numpy; matplotlib opcional para PNG (--no-plot se não tiver).

Requer: rosbag2 + mensagens nav_msgs / geometry_msgs (ambiente colcon).
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    HAS_MPL = True
except ImportError:
    HAS_MPL = False


def _need_ros():
    import rclpy  # noqa: F401
    from nav_msgs.msg import Odometry  # noqa: F401
    from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: F401
    from rclpy.serialization import deserialize_message  # noqa: F401


@dataclass
class RunStats:
    label: str
    bag_path: str
    traj_topic: str
    num_poses: int
    duration_sec: float
    path_length_m: float
    start_xy: Tuple[float, float]
    end_xy: Tuple[float, float]
    static_traj_warn: bool


def _expand_inputs(paths: Sequence[str]) -> List[Path]:
    import glob

    out: List[Path] = []
    for p in paths:
        if any(c in p for c in "*?["):
            out.extend(Path(x) for x in sorted(glob.glob(p)))
        else:
            out.append(Path(p))
    # dedupe, exist only
    seen = set()
    uniq: List[Path] = []
    for p in out:
        rp = p.resolve()
        if rp in seen:
            continue
        seen.add(rp)
        if p.exists():
            uniq.append(p)
        else:
            print(f"[aviso] ignorando (não existe): {p}", file=sys.stderr)
    return uniq


def _open_reader(uri: str):
    import rosbag2_py

    opts = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    for storage_id in ("mcap", "sqlite3"):
        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(
                rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
                opts,
            )
            return reader, storage_id
        except Exception:
            continue
    raise RuntimeError(f"Não foi possível abrir o bag: {uri} (tente mcap/sqlite3)")


def _topic_map(uri: str) -> Dict[str, str]:
    reader, _storage = _open_reader(uri)
    try:
        return {m.name: m.type for m in reader.get_all_topics_and_types()}
    finally:
        reader.close()


def _topic_message_counts(uri: str) -> Dict[str, int]:
    """Contagens reais (metadata do bag); /amcl_pose pode existir no schema com 0 mensagens."""
    from rosbag2_py import Info

    for storage in ("mcap", "sqlite3"):
        try:
            meta = Info().read_metadata(uri, storage)
            return {
                ti.topic_metadata.name: ti.message_count
                for ti in meta.topics_with_message_count
            }
        except Exception:
            continue
    return {}


def _bag_duration_sec(uri: str) -> Optional[float]:
    """Duração total gravada no metadata do rosbag (útil se há 1 msg na trajetória)."""
    from rosbag2_py import Info

    for storage in ("mcap", "sqlite3"):
        try:
            meta = Info().read_metadata(uri, storage)
            return float(meta.duration.nanoseconds) * 1e-9
        except Exception:
            continue
    return None


def _find_odom_topic_name(tmap: Dict[str, str]) -> str:
    cands = [n for n, typ in tmap.items() if "Odometry" in typ and "odom" in n.lower()]
    if not cands:
        raise RuntimeError("Nenhum tópico de odometria (nav_msgs/Odometry) no bag.")
    if "/odom" in cands:
        return "/odom"
    cands.sort(key=lambda n: (n.count("/"), len(n)))
    return cands[0]


def _find_amcl_topic_name(tmap: Dict[str, str]) -> Optional[str]:
    if "/amcl_pose" in tmap and "PoseWithCovarianceStamped" in tmap["/amcl_pose"]:
        return "/amcl_pose"
    for name, typ in tmap.items():
        if name.endswith("/amcl_pose") and "PoseWithCovarianceStamped" in typ:
            return name
    return None


def _resolve_trajectory_topic(
    uri: str,
    mode: str,
    *,
    tmap: Optional[Dict[str, str]] = None,
    counts: Optional[Dict[str, int]] = None,
) -> str:
    tmap = tmap if tmap is not None else _topic_map(uri)
    counts = counts if counts is not None else _topic_message_counts(uri)
    if mode == "auto":
        amcl = _find_amcl_topic_name(tmap)
        if amcl is not None and counts.get(amcl, 0) > 0:
            return amcl
        return _find_odom_topic_name(tmap)
    if mode == "amcl_pose":
        amcl = _find_amcl_topic_name(tmap)
        if amcl is None:
            raise RuntimeError(
                "Modo amcl_pose: bag sem tópico amcl_pose (geometry_msgs/PoseWithCovarianceStamped). "
                "Grave com: --topics ... amcl_pose no experiment_repeatability / enable_collection."
            )
        if counts.get(amcl, 0) == 0:
            raise RuntimeError(
                f"Modo amcl_pose: {amcl} existe no bag mas tem 0 mensagens "
                "(AMCL sem pose inicial ativa ou QoS). "
                "Defina pose no RViz ou use --initial-pose antes da coleta; recompile/reinicie o collector."
            )
        return amcl
    if mode == "odom":
        return _find_odom_topic_name(tmap)
    raise ValueError(f"trajectory-topic inválido: {mode}")


def _read_traj_xy(
    bag_dir: Path,
    topic: str,
) -> Tuple[str, np.ndarray, np.ndarray, np.ndarray, float]:
    """Retorna (topic, t_sec_rel_bag, x, y, duration_wall_sec).

    Ordenação e duração pelo timestamp de gravação no rosbag (3.º campo de read_next).
    """
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.serialization import deserialize_message
    import rosbag2_py

    uri = str(bag_dir.resolve())
    tmap = _topic_map(uri)
    if topic not in tmap:
        raise RuntimeError(f"Tópico {topic} não encontrado no bag.")
    typ = tmap[topic]

    reader, _storage = _open_reader(uri)
    filt = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(filt)

    rows: List[Tuple[int, float, float, float]] = []

    while reader.has_next():
        nxt = reader.read_next()
        if len(nxt) == 2:
            _tname, data = nxt
            bag_ns = 0
        else:
            _tname, data, bag_ns = nxt[0], nxt[1], int(nxt[2])
        if "Odometry" in typ:
            msg = deserialize_message(data, Odometry)
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
        elif "PoseWithCovarianceStamped" in typ:
            msg = deserialize_message(data, PoseWithCovarianceStamped)
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
        else:
            reader.close()
            raise RuntimeError(f"Tipo não suportado para trajetória: {typ}")
        hdr_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rows.append((bag_ns, hdr_sec, x, y))

    reader.close()

    if not rows:
        raise RuntimeError(f"Nenhuma mensagem em {topic}")

    rows.sort(key=lambda r: r[0])
    bag_ns_arr = np.array([r[0] for r in rows], dtype=np.int64)
    t_rel = (bag_ns_arr - bag_ns_arr[0]) * 1e-9
    duration_wall_sec = float((bag_ns_arr[-1] - bag_ns_arr[0]) * 1e-9)
    xs = np.array([r[2] for r in rows], dtype=np.float64)
    ys = np.array([r[3] for r in rows], dtype=np.float64)
    return topic, t_rel, xs, ys, duration_wall_sec


def _path_length(x: np.ndarray, y: np.ndarray) -> float:
    if len(x) < 2:
        return 0.0
    dx = np.diff(x)
    dy = np.diff(y)
    return float(np.sum(np.sqrt(dx * dx + dy * dy)))


def _resample_pair(
    xy1: np.ndarray, xy2: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Mesmo número de pontos (subamostragem uniforme) para comparar trajetórias."""
    n = min(len(xy1), len(xy2))
    if n < 2:
        return xy1, xy2
    i1 = np.linspace(0, len(xy1) - 1, n).astype(int)
    i2 = np.linspace(0, len(xy2) - 1, n).astype(int)
    return xy1[i1], xy2[i2]


def _rmse(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.sum((a - b) ** 2, axis=1))))


def _mean_pointwise_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Distância euclidiana média entre pontos homólogos (mesmo N)."""
    d = np.sqrt(np.sum((a - b) ** 2, axis=1))
    return float(np.mean(d))


def _safe_label_file(s: str) -> str:
    return "".join(c if c.isalnum() or c in "-_" else "_" for c in s)


def _write_trajectory_csv(path: Path, t: np.ndarray, xy: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("t_sec,x_m,y_m\n")
        for i in range(len(t)):
            f.write(f"{t[i]:.6f},{xy[i, 0]:.6f},{xy[i, 1]:.6f}\n")


def analyze_bag(
    label: str,
    bag_dir: Path,
    trajectory_mode: str,
) -> Tuple[RunStats, np.ndarray, np.ndarray]:
    uri = str(bag_dir.resolve())
    counts = _topic_message_counts(uri)
    tmap = _topic_map(uri)
    amcl = _find_amcl_topic_name(tmap)
    topic = _resolve_trajectory_topic(
        uri, trajectory_mode, tmap=tmap, counts=counts
    )
    if trajectory_mode == "auto" and amcl is not None and counts.get(amcl, 0) == 0:
        print(
            f"[aviso] {label}: {amcl} tem 0 mensagens no bag (AMCL sem publicar ou pose inicial). "
            "Métricas usam /odom. Regrave com pose inicial + collector atualizado (QoS AMCL).",
            file=sys.stderr,
        )
    topic, t, x, y, duration_wall = _read_traj_xy(bag_dir, topic)
    duration = duration_wall if len(t) > 1 else 0.0
    meta_dur = _bag_duration_sec(uri)
    if meta_dur is not None and meta_dur > 0.0 and (len(t) <= 1 or duration < 1e-9):
        duration = meta_dur
    plen = _path_length(x, y)
    xy = np.column_stack([x, y])
    static = len(x) > 50 and plen < 1e-4
    stats = RunStats(
        label=label,
        bag_path=str(bag_dir.resolve()),
        traj_topic=topic,
        num_poses=len(x),
        duration_sec=duration,
        path_length_m=plen,
        start_xy=(float(x[0]), float(y[0])),
        end_xy=(float(x[-1]), float(y[-1])),
        static_traj_warn=static,
    )
    if "amcl" in topic.lower() and len(x) < 10:
        print(
            f"[aviso] {label}: poucas amostras ({len(x)}) em {topic} — métricas de trajetória são "
            "enganosas (ex.: RMSE entre pontos únicos). Recompile/reinicie o collector (QoS amcl_pose) e regrave.",
            file=sys.stderr,
        )
    return stats, xy, t


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analisa rosbag2 do fleet: odometria, métricas e gráfico de trajetórias."
    )
    parser.add_argument(
        "bags",
        nargs="+",
        help="Pastas de rosbag (uma por run), ex.: collections/default/run1 ou glob entre aspas",
    )
    parser.add_argument(
        "--output-dir",
        default="analysis_out",
        help="Onde salvar summary.json e trajectory_overlay.png",
    )
    parser.add_argument(
        "--labels",
        nargs="*",
        help="Rótulos na legenda (mesma ordem dos bags); default run_0, run_1, ...",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Não gera PNG (útil sem matplotlib)",
    )
    parser.add_argument(
        "--no-csv",
        action="store_true",
        help="Não grava trajectory_<label>.csv por run",
    )
    parser.add_argument(
        "--trajectory-topic",
        choices=("auto", "amcl_pose", "odom"),
        default="auto",
        help="Fonte da trajetória: auto=/amcl_pose se existir senão /odom; amcl_pose=obrigatório no bag",
    )
    args = parser.parse_args()

    try:
        _need_ros()
    except ImportError as e:
        print(f"Erro: ambiente ROS 2 não disponível: {e}", file=sys.stderr)
        print("Execute: source /opt/ros/jazzy/setup.bash && source install/setup.bash", file=sys.stderr)
        return 2

    bag_paths = _expand_inputs(args.bags)
    if not bag_paths:
        print("Nenhum caminho válido.", file=sys.stderr)
        return 2

    labels = args.labels or [f"run_{i}" for i in range(len(bag_paths))]
    if len(labels) != len(bag_paths):
        print("--labels deve ter o mesmo número de entradas que os bags.", file=sys.stderr)
        return 2

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    all_stats: List[RunStats] = []
    trajectories: List[np.ndarray] = []
    times: List[np.ndarray] = []

    for label, bdir in zip(labels, bag_paths):
        print(f"Lendo {label} ... ({bdir})")
        try:
            st, xy, t_rel = analyze_bag(label, bdir, args.trajectory_topic)
            all_stats.append(st)
            trajectories.append(xy)
            times.append(t_rel)
        except Exception as ex:
            print(f"[FALHOU] {bdir}: {ex}", file=sys.stderr)
            return 1

    if not args.no_csv:
        csv_dir = out_dir / "trajectories_csv"
        for t_rel, xy, lab in zip(times, trajectories, labels):
            fn = csv_dir / f"trajectory_{_safe_label_file(lab)}.csv"
            _write_trajectory_csv(fn, t_rel, xy)
        print(f"[OK] CSVs: {csv_dir}/")

    # RMSE pairwise
    n = len(trajectories)
    rmse_matrix: List[List[Optional[float]]] = [
        [None] * n for _ in range(n)
    ]
    for i in range(n):
        for j in range(i + 1, n):
            a, b = _resample_pair(trajectories[i], trajectories[j])
            r = _rmse(a, b)
            rmse_matrix[i][j] = r
            rmse_matrix[j][i] = r
        rmse_matrix[i][i] = 0.0

    # vs primeira execução (referência)
    ref_idx = 0
    d0 = all_stats[ref_idx].duration_sec
    d0 = d0 if d0 > 1e-6 else 1e-6
    end_ref = np.array(all_stats[ref_idx].end_xy, dtype=np.float64)
    vs_reference: List[dict] = []
    for i in range(n):
        end_i = np.array(all_stats[i].end_xy, dtype=np.float64)
        final_err = float(np.linalg.norm(end_i - end_ref))
        dur_ratio = float(all_stats[i].duration_sec / d0)
        entry: dict = {
            "label": labels[i],
            "final_endpoint_error_m": final_err,
            "duration_sec": all_stats[i].duration_sec,
            "duration_ratio_vs_ref": dur_ratio,
        }
        if i != ref_idx:
            a, b = _resample_pair(trajectories[ref_idx], trajectories[i])
            entry["rmse_vs_ref_m"] = _rmse(a, b)
            entry["mean_pointwise_distance_vs_ref_m"] = _mean_pointwise_distance(a, b)
        else:
            entry["rmse_vs_ref_m"] = 0.0
            entry["mean_pointwise_distance_vs_ref_m"] = 0.0
        vs_reference.append(entry)

    def _run_to_json(s: RunStats) -> dict:
        d = asdict(s)
        d["start_xy"] = list(d["start_xy"])
        d["end_xy"] = list(d["end_xy"])
        return d

    summary = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "reference_run_index": ref_idx,
        "reference_label": labels[ref_idx],
        "runs": [_run_to_json(s) for s in all_stats],
        "pairwise_rmse_m": rmse_matrix,
        "vs_reference": vs_reference,
        "labels": labels,
    }
    summary_path = out_dir / "summary.json"
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"\n[OK] Resumo: {summary_path}")

    if not args.no_plot:
        if not HAS_MPL:
            print("[aviso] matplotlib não instalado; use: pip install matplotlib  ou  --no-plot")
        else:
            fig, ax = plt.subplots(figsize=(8, 8))
            for st, xy, lab in zip(all_stats, trajectories, labels):
                ax.plot(xy[:, 0], xy[:, 1], label=lab, linewidth=1.5)
            ax.set_aspect("equal", adjustable="datalim")
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_title("Trajetórias (mapa ou odom) — comparação entre runs")
            ax.legend(loc="best", fontsize=9)
            ax.grid(True, alpha=0.3)
            # Caixa de texto: RMSE vs referência (primeiro bag)
            if n > 1:
                lines = [f"Ref: {labels[0]}"]
                for i in range(1, n):
                    r = vs_reference[i].get("rmse_vs_ref_m", 0.0)
                    fe = vs_reference[i].get("final_endpoint_error_m", 0.0)
                    lines.append(f"{labels[i]}: RMSE={r:.3f}m  Δfim={fe:.3f}m")
                ax.text(
                    0.02,
                    0.98,
                    "\n".join(lines),
                    transform=ax.transAxes,
                    fontsize=8,
                    verticalalignment="top",
                    bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
                )
            png_path = out_dir / "trajectory_overlay.png"
            fig.savefig(png_path, dpi=150, bbox_inches="tight")
            plt.close(fig)
            print(f"[OK] Gráfico: {png_path}")

    # Imprime tabela rápida
    print("\n--- Métricas por run ---")
    for s in all_stats:
        print(
            f"  {s.label}: duração={s.duration_sec:.2f}s  comprimento={s.path_length_m:.3f}m  "
            f"poses={s.num_poses}  topic={s.traj_topic}"
        )
    if n > 1:
        print("\n--- RMSE (m) entre pares (subamostragem uniforme) ---")
        for i in range(n):
            for j in range(i + 1, n):
                r = rmse_matrix[i][j]
                print(f"  {labels[i]} vs {labels[j]}: {r:.4f} m")
        print(f"\n--- Vs referência ({labels[0]}) ---")
        for v in vs_reference:
            print(
                f"  {v['label']}: RMSE={v['rmse_vs_ref_m']:.4f} m  "
                f"dist_média_pt={v['mean_pointwise_distance_vs_ref_m']:.4f} m  "
                f"Δfim={v['final_endpoint_error_m']:.4f} m  "
                f"duração×={v['duration_ratio_vs_ref']:.3f}"
            )

    static_labels = [s.label for s in all_stats if s.static_traj_warn]
    if static_labels:
        print("\n--- Avisos ---")
        print(
            "  Trajetória (x,y) quase constante — comprimento e RMSE ~0. "
            "Se a fonte foi /odom: verifique Gazebo/ros_gz_bridge. "
            "Se /amcl_pose no bag tem 0 mensagens: pose inicial (RViz ou --initial-pose) antes da coleta e collector com QoS AMCL."
        )
        print(f"  Runs afetados: {', '.join(static_labels)}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
