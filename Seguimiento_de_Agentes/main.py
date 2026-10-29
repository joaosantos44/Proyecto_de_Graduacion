"""
main.py — Orquestador del swarm.

Un solo RobotatTracker (una conexion MQTT) compartido por todos los
agentes; cada agente corre en su hilo (run_leader / run_follower).
Si SHOW_LIVE_PLOT=True, la grafica en vivo corre en el hilo principal y
muestra sliders que escriben en live_params (theta_offset y Ra).

Para agregar un agente: anadir una entrada a ROBOTS.
Nunca dejar dos entradas activas con el mismo agent_id.
"""
import threading
import time

from robotat_tracker import RobotatTracker
from leader_controller import run_leader
from follower_controller import run_follower
from visualize_trajectory import build_and_show_plot

# ============================================================
# Defaults (una entrada de ROBOTS con su propio valor los sobrescribe)
# ============================================================
DEFAULT_THETA_OFFSET_DEG = 100.0   # valor inicial de los sliders de theta_offset
DEFAULT_RA_MM = 450.0             # valor inicial del slider de radio (mm)

V_MAX_RPM_DEFAULT = 300.0         # tope de rpm para todos los agentes
RUN_TIME_S = None                 # duracion de la prueba (None = indefinido)
SHOW_LIVE_PLOT = True

# ============================================================
# Agentes
# ============================================================
ROBOTS = [
    # Lider en point:
    {
        "role": "leader",
        "agent_id": 12,
        "ip": "192.168.50.112",
        "mode": "point",          # "circle" para ver el slider de Ra
        "follow_agent_id": 2,
        "v0": 250.0,
        "eP_stop": 100.0,         # con circle usar ~650  con point usar 100
        "ePtol": 40.0,
        # "theta_offset_deg": 100.0,
    },
    # Cadena de seguidores
    {
        "role": "follower",
        "agent_id": 8,
        "ip": "192.168.50.108",
        "follow_agent_id": 12,
        "eP_stop_fw": 100.0,
        "v0_fw": 250.0,
    },
    {
        "role": "follower",
        "agent_id": 2,
        "ip": "192.168.50.102",
        "follow_agent_id": 8,
        "eP_stop_fw": 100.0,
        "v0_fw": 250.0,
    },

    # # Seguidores extra (descomentar para ampliar la cadena):
    # {"role": "follower",
    #  "agent_id": 5,
    #   "ip": "192.168.50.105",
    #  "follow_agent_id": 3,
    #  "eP_stop_fw": 100.0,
    #  "v0_fw": 250.0
    # },
    # {"role": "follower",
    #  "agent_id": 12,
    #   "ip": "192.168.50.112",
    #  "follow_agent_id": 5,
    #  "eP_stop_fw": 100.0,
    #  "v0_fw": 250.0
    # },
    # {"role": "follower",
    #  "agent_id": 12,
    #   "ip": "192.168.50.112",
    #  "follow_agent_id": 9,
    #  "eP_stop_fw": 100.0,
    #  "v0_fw": 250.0
    # },
    # {
    #     "role": "follower",
    #     "agent_id": 3,
    #     "ip": "192.168.50.103",
    #     "follow_agent_id": 2,
    #     "eP_stop_fw": 100.0,
    #     "v0_fw": 250.0,
    # },
    
]
# ============================================================


def _collect_agent_ids(robots):
    ids = set()
    for r in robots:
        ids.add(r["agent_id"])
        if r.get("follow_agent_id") is not None:
            ids.add(r["follow_agent_id"])
    return sorted(ids)


def _make_thread(robot_cfg, tracker, stop_event, shared_state, pause_event, live_params):
    role = robot_cfg["role"]
    aid = robot_cfg["agent_id"]

    # Valores iniciales de los sliders: config del robot o default global
    theta_key = "theta_offset_deg_fw" if role == "follower" else "theta_offset_deg"
    live = live_params.setdefault(aid, {})
    live[theta_key] = float(robot_cfg.get(theta_key, DEFAULT_THETA_OFFSET_DEG))
    if role == "leader":
        live["Ra"] = float(robot_cfg.get("Ra", DEFAULT_RA_MM))

    base = dict(
        tracker=tracker,
        agent_id=aid,
        ip=robot_cfg["ip"],
        stop_event=stop_event,
        pause_event=pause_event,
        run_time_s=robot_cfg.get("run_time_s", RUN_TIME_S),
        label=robot_cfg.get("label"),
        shared_state=shared_state.setdefault(aid, {}),
        live_params=live,
    )

    if role == "leader":
        kwargs = dict(base)
        kwargs["v_max_rpm"] = robot_cfg.get("v_max_rpm", V_MAX_RPM_DEFAULT)
        for k in ("mode", "NV", "tf", "target_point", "follow_agent_id",
                  "kpO", "kiO", "kdO", "v0", "alpha",
                  "lookahead_samples", "ePtol", "eP_stop", "stop_at_target",
                  "theta_sign",
                  "left_sign", "right_sign", "swap_wheels", "control_dt"):
            if k in robot_cfg:
                kwargs[k] = robot_cfg[k]
        kwargs["theta_offset_deg"] = live["theta_offset_deg"]
        kwargs["Ra"] = live["Ra"]
        target = run_leader

    elif role == "follower":
        kwargs = dict(base)
        kwargs["v_max_rpm_fw"] = robot_cfg.get("v_max_rpm_fw", V_MAX_RPM_DEFAULT)
        kwargs["follow_agent_id"] = robot_cfg["follow_agent_id"]
        for k in ("kpO_fw", "kiO_fw", "kdO_fw", "v0_fw", "alpha_fw",
                  "eP_stop_fw", "stop_at_target_fw",
                  "theta_sign_fw",
                  "left_sign_fw", "right_sign_fw", "swap_wheels_fw", "control_dt"):
            if k in robot_cfg:
                kwargs[k] = robot_cfg[k]
        kwargs["theta_offset_deg_fw"] = live["theta_offset_deg_fw"]
        target = run_follower

    else:
        raise ValueError(f"role desconocido: {role!r} (usar 'leader' o 'follower')")

    return threading.Thread(target=target, kwargs=kwargs, daemon=True)


def main():
    agent_ids = _collect_agent_ids(ROBOTS)
    print(f"Conectando al Robotat, trackeando agent_ids={agent_ids}...")

    tracker = RobotatTracker(agent_ids=agent_ids)
    tracker.connect()

    stop_event = threading.Event()
    pause_event = threading.Event()  # set = pausado (ESPACIO en la grafica)
    shared_state = {}                # {agent_id: trayectoria/objetivo real}
    live_params = {}                 # {agent_id: valores de los sliders}
    threads = [_make_thread(r, tracker, stop_event, shared_state, pause_event, live_params)
               for r in ROBOTS]

    for t in threads:
        t.start()

    try:
        if SHOW_LIVE_PLOT:
            build_and_show_plot(tracker, ROBOTS, shared_state,
                                 threads=threads, stop_event=stop_event,
                                 pause_event=pause_event, live_params=live_params)
        else:
            while any(t.is_alive() for t in threads):
                time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nCtrl+C recibido, deteniendo todos los agentes...")
    finally:
        stop_event.set()
        for t in threads:
            t.join(timeout=5.0)
        tracker.disconnect()
        print("Listo, todos los agentes detenidos.")


if __name__ == "__main__":
    main()
