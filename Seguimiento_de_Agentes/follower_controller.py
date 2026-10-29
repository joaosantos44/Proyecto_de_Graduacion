"""
follower_controller.py — Control del seguidor. Misma ley que run_leader()
en mode="point" con follow_agent_id, pero standalone. Siempre requiere follow_agent_id.

live_params: el slider de la grafica escribe "theta_offset_deg_fw".
"""
import math
import time

from Pololu3Pi import Pololu3Pi
from unicycle_utils import inverse_kinematics_rpm, wrap_to_pi, send_wheel_cmd


def run_follower(tracker, agent_id, ip, stop_event,
                  v_max_rpm_fw=800.0,
                  follow_agent_id=None,
                  kpO_fw=2.7, kiO_fw=0.000, kdO_fw=0.55,
                  v0_fw=400.0, alpha_fw=0.0001, 
                  eP_stop_fw=40.0, stop_at_target_fw=True,
                  control_dt=0.1, run_time_s=None,
                  theta_sign_fw=1, theta_offset_deg_fw=0.0,
                  left_sign_fw=1, right_sign_fw=1, swap_wheels_fw=False,
                  shared_state=None,
                  pause_event=None,
                  live_params=None,
                  label=None):
    """
    Persigue en vivo a follow_agent_id. Frena si eP < eP_stop_fw y retoma
    si el agente seguido se aleja.
    """
    if follow_agent_id is None:
        raise ValueError("run_follower requiere follow_agent_id (a que agente seguir)")

    tag = label or f"follower[{agent_id}->{follow_agent_id}]"
    bot = Pololu3Pi()
    bot.connect(ip=ip)

    try:
        print(f"[{tag}] esperando pose inicial...")
        x0, y0, theta0 = tracker.wait_for_pose(agent_id, timeout=15.0)
        print(f"[{tag}] pose inicial: x={x0:.1f}mm y={y0:.1f}mm theta={math.degrees(theta0):.1f}deg")

        print(f"[{tag}] esperando pose inicial del agente a seguir (id={follow_agent_id})...")
        fx0, fy0, _ = tracker.wait_for_pose(follow_agent_id, timeout=15.0)
        xg, yg = fx0, fy0

        if shared_state is not None:
            shared_state['mode'] = 'point'
            shared_state['traj_x'] = [xg]
            shared_state['traj_y'] = [yg]
            shared_state['target'] = (xg, yg)
            shared_state['traj_version'] = 0

        EO = 0.0
        eO_1 = 0.0
        t_start = time.monotonic()
        was_paused = False
        pause_t0 = 0.0

        print(f"[{tag}] iniciando control (v_max_rpm_fw={v_max_rpm_fw}, "
              f"theta_sign_fw={theta_sign_fw}, theta_offset_deg_fw={theta_offset_deg_fw})")

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

            # Parametro en vivo (slider)
            if live_params is not None:
                theta_offset_deg_fw = live_params.get("theta_offset_deg_fw", theta_offset_deg_fw)

            pose = tracker.try_get_pose(agent_id)
            if pose is None:
                bot.force_stop()
                print(f"[{tag}] AVISO: sin pose valida, motores detenidos.")
                time.sleep(control_dt)
                continue

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
            theta = theta_sign_fw * theta_raw + math.radians(theta_offset_deg_fw)

            ex = xg - posx
            ey = yg - posy
            thetag = math.atan2(ey, ex)
            eP = math.hypot(ex, ey)
            eO = wrap_to_pi(thetag - theta)

            if stop_at_target_fw and eP < eP_stop_fw:
                send_wheel_cmd(bot, 0.0, 0.0,
                                left_sign=left_sign_fw, right_sign=right_sign_fw, swap_wheels=swap_wheels_fw)
                EO = 0.0
                eO_1 = 0.0
                print(f"[{tag}] en el punto objetivo (eP={eP:5.1f}mm), motores detenidos.")
                elapsed = time.monotonic() - loop_t0
                time.sleep(max(0.0, control_dt - elapsed))
                continue

            v = v0_fw * (1 - math.exp(-alpha_fw * eP ** 2))
            v *= max(0.0, math.cos(eO))  # baja v si el objetivo no esta al frente

            eO_D = eO - eO_1
            EO += eO
            w = kpO_fw * eO + kiO_fw * EO + kdO_fw * eO_D
            eO_1 = eO

            v_left_rpm, v_right_rpm = inverse_kinematics_rpm(v, w, v_max_rpm_fw)
            send_wheel_cmd(bot, v_left_rpm, v_right_rpm,
                            left_sign=left_sign_fw, right_sign=right_sign_fw, swap_wheels=swap_wheels_fw)

            print(f"[{tag}] pos=({posx:7.1f},{posy:7.1f}) theta_raw={math.degrees(theta_raw):6.1f}deg "
                  f"theta={math.degrees(theta):6.1f}deg "
                  f"eP={eP:6.1f}mm eO={math.degrees(eO):6.1f}deg L={v_left_rpm:5.1f}rpm R={v_right_rpm:5.1f}rpm")

            elapsed = time.monotonic() - loop_t0
            time.sleep(max(0.0, control_dt - elapsed))

    finally:
        print(f"[{tag}] deteniendo motores...")
        bot.disconnect()  # incluye force_stop()
