"""
visualize_trajectory.py — Grafica en vivo sobre el sistema de coordenadas
del Robotat.

Por agente dibuja:
  - nube de waypoints de la trayectoria real (shared_state traj_x/traj_y)
  - estrella: waypoint que persigue ahora (shared_state target)
  - punto + estela: posicion real leida del tracker

Sliders (modo integrado, si se pasa live_params):
  - theta_offset: UNO solo, aplica a todos los agentes (lideres y followers)
  - Ra (mm) por cada lider con mode="circle"

Teclas: S = captura SVG en ./capturas, ESPACIO = pausar/reanudar robots.

Uso: integrado desde main.py (recomendado) o standalone
(python visualize_trajectory.py`, sin sliders).
"""
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider

from robotat_tracker import RobotatTracker
from leader_controller import _build_circular_trajectory

# ============================================================
# Configuracion
# ============================================================
X_LIMITS = (2200, -2200)
Y_LIMITS = (2200, -2200)
REFRESH_MS = 50
MAX_TRAIL_POINTS = 300
DEFAULT_COLORS = ["tab:red", "tab:blue", "tab:green", "tab:orange", "tab:purple", "tab:brown"]
CAPTURE_KEY = "s"
CAPTURE_DIR = "capturas"
PAUSE_KEY = " "

THETA_KEYS = ("theta_offset_deg", "theta_offset_deg_fw")   # claves que mueve el slider global
THETA_RANGE = (-180.0, 180.0)   # grados
THETA_STEP = 1.0
RA_RANGE = (100.0, 1500.0)      # mm
RA_STEP = 10.0
SLIDER_ROW_IN = 0.38            # alto por slider (pulgadas)
# ============================================================


def _save_vector_snapshot(fig, output_dir=CAPTURE_DIR):
    """Guarda la figura actual como SVG con fecha y hora en el nombre."""
    capture_dir = Path(output_dir)
    capture_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")[:-3]
    capture_path = capture_dir / f"trayectoria_{timestamp}.svg"

    fig.canvas.draw()  # termina cualquier redibujado pendiente
    fig.savefig(capture_path, format="svg", bbox_inches="tight", facecolor="white")
    return capture_path.resolve()


def _slider_specs(robots, live_params, default_colors):
    """Sliders a crear: (aid, key, vmin, vmax, step, color, label, init). aid=None = global."""
    specs = []
    if not live_params:
        return specs

    # Un solo slider de theta_offset para todos los agentes
    first = next((v[k] for v in live_params.values() for k in THETA_KEYS if k in v), None)
    if first is not None:
        specs.append((None, "theta_offset", THETA_RANGE[0], THETA_RANGE[1], THETA_STEP,
                      "tab:gray", "theta_offset (todos)", first))

    # Radio: un slider por lider en modo circulo
    for i, r in enumerate(robots):
        aid = r["agent_id"]
        if r.get("role") == "leader" and r.get("mode", "point") == "circle" \
                and "Ra" in live_params.get(aid, {}):
            color = r.get("color", default_colors[i % len(default_colors)])
            specs.append((aid, "Ra", RA_RANGE[0], RA_RANGE[1], RA_STEP,
                          color, f"id {aid} Ra", live_params[aid]["Ra"]))
    return specs


def build_and_show_plot(tracker, robots, shared_state,
                         threads=None, stop_event=None, pause_event=None,
                         live_params=None,
                         x_limits=X_LIMITS, y_limits=Y_LIMITS,
                         refresh_ms=REFRESH_MS, max_trail_points=MAX_TRAIL_POINTS):
    """
    Muestra la grafica en vivo. Bloquea hasta cerrar la ventana o terminar
    los hilos; llamar desde el hilo principal.

    tracker: RobotatTracker ya conectado (el mismo de los controladores).
    robots: lista ROBOTS de main.py.
    shared_state / live_params: dicts {agent_id: {...}} compartidos con los hilos.
    threads: si se pasa, la ventana se cierra al terminar todos.
    stop_event: se activa al cerrar la ventana para frenar los robots.
    """
    specs = _slider_specs(robots, live_params, DEFAULT_COLORS)
    n_sliders = len(specs)

    fig_h = 7.0 + SLIDER_ROW_IN * n_sliders + 0.3
    fig, ax = plt.subplots(figsize=(7, fig_h))
    if n_sliders:
        fig.subplots_adjust(bottom=(SLIDER_ROW_IN * n_sliders + 0.8) / fig_h, top=0.95)

    ax.set_xlim(*x_limits)
    ax.set_ylim(*y_limits)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title("Robotat en vivo — waypoints reales del control vs. posiciones reales")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    agent_points = {}
    agent_trails = {}
    target_marker = {}
    waypoint_scatter = {}
    drawn_traj_key = {}  # (n_puntos, version): evita redibujar la nube en cada frame

    for i, r in enumerate(robots):
        aid = r["agent_id"]
        color = r.get("color", DEFAULT_COLORS[i % len(DEFAULT_COLORS)])
        label = r.get("label") or f"{r.get('role', 'agent')} (id {aid})"

        point, = ax.plot([], [], "o", color=color, markersize=10, label=label)
        trail, = ax.plot([], [], "-", color=color, linewidth=1, alpha=0.4)
        target_pt, = ax.plot([], [], "*", color=color, markersize=14, alpha=0.9,
                              markeredgecolor="black", markeredgewidth=0.5)
        wp_scatter = ax.scatter([], [], s=8, color=color, alpha=0.25)

        agent_points[aid] = point
        agent_trails[aid] = {"line": trail, "xs": [], "ys": []}
        target_marker[aid] = target_pt
        waypoint_scatter[aid] = wp_scatter
        drawn_traj_key[aid] = None

    ax.legend(loc="upper right", fontsize=8)
    pause_text = ax.text(0.5, 0.97, "", transform=ax.transAxes, ha="center", va="top",
                         color="red", fontsize=14, fontweight="bold")

    # Sliders: escriben en live_params, los controladores lo releen cada ciclo
    sliders = []
    for k, (aid, key, vmin, vmax, step, color, label, init) in enumerate(specs):
        y_in = 0.15 + SLIDER_ROW_IN * (n_sliders - 1 - k)
        ax_s = fig.add_axes([0.32, y_in / fig_h, 0.55, 0.22 / fig_h])
        init = min(max(float(init), vmin), vmax)
        s = Slider(ax_s, label, vmin, vmax, valinit=init, valstep=step, color=color)

        def _on_change(val, aid=aid, key=key):
            if aid is None:  # global: escribe en todos los agentes
                for params in live_params.values():
                    for tk in THETA_KEYS:
                        if tk in params:
                            params[tk] = float(val)
                print(f">> todos: theta_offset = {val:.0f}")
            else:
                live_params[aid][key] = float(val)
                print(f">> agente {aid}: {key} = {val:.0f}")

        s.on_changed(_on_change)
        sliders.append(s)
    fig._sliders = sliders  # evita que matplotlib los recolecte

    def _on_close(event):
        if stop_event is not None:
            stop_event.set()

    def _on_key_press(event):
        if event.key is None:
            return

        if event.key == PAUSE_KEY:
            if pause_event is None:
                return
            if pause_event.is_set():
                pause_event.clear()
                print(">> REANUDADO")
            else:
                pause_event.set()
                print(">> PAUSADO")
            return

        if event.key.lower() != CAPTURE_KEY:
            return

        try:
            capture_path = _save_vector_snapshot(fig)
            print(f"Captura vectorial guardada en: {capture_path}")
        except OSError as exc:
            print(f"No se pudo guardar la captura vectorial: {exc}")

    fig.canvas.mpl_connect("close_event", _on_close)
    fig.canvas.mpl_connect("key_press_event", _on_key_press)
    print("Grafica en vivo: S = captura vectorial SVG, ESPACIO = pausar/reanudar robots.")

    def update(frame):
        artists = [pause_text]
        pause_text.set_text("PAUSADO (espacio para reanudar)"
                            if pause_event is not None and pause_event.is_set() else "")
        for r in robots:
            aid = r["agent_id"]
            state = shared_state.get(aid, {})

            # Redibuja la nube solo si cambio el numero de puntos o la version (slider de Ra)
            traj_x = state.get("traj_x")
            traj_y = state.get("traj_y")
            if traj_x:
                key = (len(traj_x), state.get("traj_version", 0))
                if drawn_traj_key[aid] != key:
                    waypoint_scatter[aid].set_offsets(list(zip(traj_x, traj_y)))
                    drawn_traj_key[aid] = key

            target = state.get("target")
            if target is not None:
                target_marker[aid].set_data([target[0]], [target[1]])

            pose = tracker.try_get_pose(aid)
            if pose is not None:
                x, y, _theta = pose
                agent_points[aid].set_data([x], [y])
                trail = agent_trails[aid]
                trail["xs"].append(x)
                trail["ys"].append(y)
                if len(trail["xs"]) > max_trail_points:
                    trail["xs"].pop(0)
                    trail["ys"].pop(0)
                trail["line"].set_data(trail["xs"], trail["ys"])
                artists += [agent_points[aid], trail["line"]]

            artists += [target_marker[aid], waypoint_scatter[aid]]

        if threads is not None and not any(t.is_alive() for t in threads):
            plt.close(fig)

        return artists

    ani = animation.FuncAnimation(fig, update, interval=refresh_ms, blit=False,
                                    cache_frame_data=False)
    plt.show()
    return ani


def _standalone_main():
    """Modo standalone (sin sliders). Ajustar Ra/NV/tf a lo que use main.py."""
    LEADER_AGENT_ID = 5
    Ra, NV, tf, CONTROL_DT = 250.0, 2, 30.0, 0.1

    AGENTS = [
        {"agent_id": LEADER_AGENT_ID, "role": "leader", "label": "Lider (robot1)", "color": "tab:red"},
    ]

    tracker = RobotatTracker(agent_ids=[a["agent_id"] for a in AGENTS])
    tracker.connect()

    print("Esperando pose inicial del lider para calcular la trayectoria...")
    x0, y0, theta0 = tracker.wait_for_pose(LEADER_AGENT_ID, timeout=15.0)
    traj_x, traj_y = _build_circular_trajectory(x0, y0, theta0, Ra, NV, tf, CONTROL_DT)

    shared_state = {
        LEADER_AGENT_ID: {"traj_x": traj_x, "traj_y": traj_y, "target": (traj_x[0], traj_y[0])},
    }

    try:
        build_and_show_plot(tracker, AGENTS, shared_state)
    finally:
        tracker.disconnect()


if __name__ == "__main__":
    _standalone_main()
