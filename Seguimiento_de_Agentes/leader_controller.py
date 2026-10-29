"""
leader_controller.py — Control del lider, pensado para correr en un hilo
desde main.py con un RobotatTracker compartido.

Modos:
  - "circle": trayectoria circular + pure pursuit.
  - "point":  ir a un punto fijo (o seguir en vivo a otro agente).

shared_state: publica trayectoria/objetivo real para la grafica.
live_params:  valores que los sliders cambian en vivo
              ("theta_offset_deg", "Ra").
"""
import math
import time

from Pololu3Pi import Pololu3Pi
from unicycle_utils import inverse_kinematics_rpm, wrap_to_pi, send_wheel_cmd


def _circle_geometry(x0, y0, theta0, Ra):
    #Centro del circulo y angulo inicial del robot respecto a el.
    xc = x0 + Ra * math.cos(theta0 + math.pi / 2)
    yc = y0 + Ra * math.sin(theta0 + math.pi / 2)
    ang0 = math.atan2(y0 - yc, x0 - xc)
    return xc, yc, ang0


def _circle_points(xc, yc, ang0, Ra, NV, tf, control_dt):
    #Waypoints del circulo para un centro y radio dados.
    w_traj = NV * 2 * math.pi / tf
    n_pts = math.ceil(tf / control_dt) + 1
    traj_x = [xc + Ra * math.cos(ang0 + w_traj * (i * control_dt)) for i in range(n_pts)]
    traj_y = [yc + Ra * math.sin(ang0 + w_traj * (i * control_dt)) for i in range(n_pts)]
    return traj_x, traj_y


def _build_circular_trajectory(x0, y0, theta0, Ra, NV, tf, control_dt):
    xc, yc, ang0 = _circle_geometry(x0, y0, theta0, Ra)
    return _circle_points(xc, yc, ang0, Ra, NV, tf, control_dt)


def run_leader(tracker, agent_id, ip, stop_event,
                v_max_rpm=800.0,
                mode="point",
                Ra=500.0, NV=2, tf=30.0,
                target_point=(0.0, 0.0),
                follow_agent_id=None,
                kpO=2.7, kiO=0.000, kdO=0.55,
                v0=400.0, alpha=0.0007,
                lookahead_samples=15, ePtol=10,
                eP_stop=750.0, stop_at_target=True,
                control_dt=0.1, run_time_s=None,
                theta_sign=1, theta_offset_deg=0.0,
                left_sign=1, right_sign=1, swap_wheels=False,
                shared_state=None,
                pause_event=None,
                live_params=None,
                label=None):
    """
    Corre el control hasta que stop_event se active o pase run_time_s.

    mode="circle": avanza por la trayectoria con pure pursuit. Si cambia
        live_params["Ra"], se recalcula con el mismo centro y nuevo radio.
    mode="point": va a target_point, o a la pose en vivo de follow_agent_id.
        Frena si eP < eP_stop y retoma si el objetivo se aleja.

    theta_sign / theta_offset_deg: calibracion de orientacion del marcador.
    left_sign / right_sign / swap_wheels: calibracion de ruedas del robot.
    """
    tag = label or f"leader[{agent_id}]"
    bot = Pololu3Pi()
    bot.connect(ip=ip)

    try:
        print(f"[{tag}] esperando pose inicial...")
        x0, y0, theta0 = tracker.wait_for_pose(agent_id, timeout=15.0)
        print(f"[{tag}] pose inicial: x={x0:.1f}mm y={y0:.1f}mm theta={math.degrees(theta0):.1f}deg")

        # El circulo se arma con el heading ya calibrado
        theta0_cal = theta_sign * theta0 + math.radians(theta_offset_deg)

        xc = yc = ang0 = None
        if mode == "circle":
            xc, yc, ang0 = _circle_geometry(x0, y0, theta0_cal, Ra)
            traj_x, traj_y = _circle_points(xc, yc, ang0, Ra, NV, tf, control_dt)
        elif mode == "point":
            if follow_agent_id is not None:
                print(f"[{tag}] esperando pose inicial del agente a seguir (id={follow_agent_id})...")
                fx0, fy0, _ = tracker.wait_for_pose(follow_agent_id, timeout=15.0)
                traj_x, traj_y = [fx0], [fy0]
            else:
                traj_x, traj_y = [float(target_point[0])], [float(target_point[1])]
        else:
            raise ValueError(f"mode desconocido: {mode!r} (usar 'circle' o 'point')")

        n_pts = len(traj_x)
        lookahead_index = 0
        xg, yg = traj_x[0], traj_y[0]

        if shared_state is not None:
            shared_state['mode'] = mode
            shared_state['traj_x'] = list(traj_x)
            shared_state['traj_y'] = list(traj_y)
            shared_state['target'] = (xg, yg)
            shared_state['traj_version'] = 0  # sube al recalcular la trayectoria

        EO = 0.0
        eO_1 = 0.0
        t_start = time.monotonic()
        was_paused = False
        pause_t0 = 0.0

        print(f"[{tag}] iniciando control (v_max_rpm={v_max_rpm}, mode={mode}, "
              f"theta_sign={theta_sign}, theta_offset_deg={theta_offset_deg}, Ra={Ra})")

        while not stop_event.is_set():
            loop_t0 = time.monotonic()

            if pause_event is not None and pause_event.is_set():
                if not was_paused:
                    print(f"[{tag}] PAUSADO, motores detenidos.")
                    was_paused = True
                    pause_t0 = loop_t0
                bot.force_stop()
                EO = 0.0
                eO_1 = 0.0
                stop_event.wait(control_dt)
                continue
            if was_paused:
                t_start += time.monotonic() - pause_t0  # la pausa no cuenta en run_time_s
                was_paused = False
                print(f"[{tag}] REANUDADO.")

            if run_time_s is not None and (loop_t0 - t_start) > run_time_s:
                print(f"[{tag}] tiempo de prueba finalizado.")
                break

            # Parametros en vivo (sliders)
            if live_params is not None:
                theta_offset_deg = live_params.get("theta_offset_deg", theta_offset_deg)
                if mode == "circle":
                    new_Ra = live_params.get("Ra", Ra)
                    if new_Ra != Ra:
                        Ra = new_Ra
                        traj_x, traj_y = _circle_points(xc, yc, ang0, Ra, NV, tf, control_dt)
                        xg, yg = traj_x[lookahead_index], traj_y[lookahead_index]
                        if shared_state is not None:
                            shared_state['traj_x'] = list(traj_x)
                            shared_state['traj_y'] = list(traj_y)
                            shared_state['target'] = (xg, yg)
                            shared_state['traj_version'] = shared_state.get('traj_version', 0) + 1

            pose = tracker.try_get_pose(agent_id)
            if pose is None:
                bot.force_stop()
                print(f"[{tag}] AVISO: sin pose valida, motores detenidos.")
                time.sleep(control_dt)
                continue

            if mode == "point" and follow_agent_id is not None:
                target_pose = tracker.try_get_pose(follow_agent_id)
                if target_pose is None:
                    bot.force_stop()
                    print(f"[{tag}] AVISO: sin pose valida del agente seguido "
                          f"(id={follow_agent_id}), motores detenidos.")
                    time.sleep(control_dt)
                    continue
                xg, yg = target_pose[0], target_pose[1]
                if shared_state is not None:
                    shared_state['target'] = (xg, yg)
                    shared_state['traj_x'] = [xg]
                    shared_state['traj_y'] = [yg]

            posx, posy, theta_raw = pose
            theta = theta_sign * theta_raw + math.radians(theta_offset_deg)

            ex = xg - posx
            ey = yg - posy
            thetag = math.atan2(ey, ex)
            eP = math.hypot(ex, ey)
            eO = wrap_to_pi(thetag - theta)

            if mode == "circle":
                # Al llegar al waypoint, avanza el lookahead
                if eP <= ePtol:
                    lookahead_index = min(lookahead_index + lookahead_samples, n_pts - 1)
                    xg, yg = traj_x[lookahead_index], traj_y[lookahead_index]
                    if shared_state is not None:
                        shared_state['target'] = (xg, yg)
            else:
                if stop_at_target and eP < eP_stop:
                    send_wheel_cmd(bot, 0.0, 0.0,
                                    left_sign=left_sign, right_sign=right_sign, swap_wheels=swap_wheels)
                    EO = 0.0
                    eO_1 = 0.0
                    print(f"[{tag}] en el punto objetivo (eP={eP:5.1f}mm), motores detenidos.")
                    elapsed = time.monotonic() - loop_t0
                    time.sleep(max(0.0, control_dt - elapsed))
                    continue

            v = v0 * (1 - math.exp(-alpha * eP ** 2))
            if mode == "point":
                # Baja v suavemente si el objetivo no esta al frente (evita avanza-frena-avanza)
                v *= max(0.0, math.cos(eO))

            eO_D = eO - eO_1
            EO += eO
            w = kpO * eO + kiO * EO + kdO * eO_D
            eO_1 = eO

            v_left_rpm, v_right_rpm = inverse_kinematics_rpm(v, w, v_max_rpm)
            send_wheel_cmd(bot, v_left_rpm, v_right_rpm,
                            left_sign=left_sign, right_sign=right_sign, swap_wheels=swap_wheels)

            print(f"[{tag}] pos=({posx:7.1f},{posy:7.1f}) theta_raw={math.degrees(theta_raw):6.1f}deg "
                  f"theta={math.degrees(theta):6.1f}deg "
                  f"eP={eP:6.1f}mm eO={math.degrees(eO):6.1f}deg L={v_left_rpm:5.1f}rpm R={v_right_rpm:5.1f}rpm")

            elapsed = time.monotonic() - loop_t0
            time.sleep(max(0.0, control_dt - elapsed))

    finally:
        print(f"[{tag}] deteniendo motores...")
        bot.disconnect()  # incluye force_stop()
