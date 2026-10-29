"""
unicycle_utils.py — Cinematica de uniciclo compartida entre los controladores
fisicos (robot1 lider / robot2 seguidor).

Parametros fisicos de los Pololu3Pi (migracion de Webots):
    wheel_radius   = 32 mm
    wheel_distance = 96 - 2*6.8 mm
"""
import math

wheel_radius = 32.0            # mm
wheel_distance = 96.0 - 2 * 6.8  # mm


def send_wheel_cmd(bot, v_left_rpm, v_right_rpm,
                    left_sign=1, right_sign=1, swap_wheels=False):
    """
    Envia velocidades de rueda al robot aplicando la calibracion de signos
    de ESE robot (left_sign/right_sign/swap_wheels vienen del config de
    cada agente en main.py -- NO son globales, porque cada robot fisico
    puede tener el cableado o montaje distinto).
    """
    l = left_sign * v_left_rpm
    r = right_sign * v_right_rpm
    if swap_wheels:
        l, r = r, l
    bot.set_wheel_velocities(l, r)


def rads2rpm(x):
    return x * (60.0 / (2.0 * math.pi))


def rpm2rads(x):
    return x * (2.0 * math.pi / 60.0)


def velsat(vel, v_max):
    """Saturacion simetrica: satura |vel| a v_max conservando el signo."""
    if vel == 0:
        return 0.0
    return math.copysign(min(abs(vel), v_max), vel)


def wrap_to_pi(angle):
    """Envuelve un angulo (rad) al rango [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def inverse_kinematics_rpm(v, w, v_max_rpm):
    """
    Cinematica inversa de uniciclo -> velocidades de rueda en RPM,
    ya saturadas a +/- v_max_rpm.

    v: velocidad lineal deseada (mm/s)
    w: velocidad angular deseada (rad/s)
    Devuelve (v_left_rpm, v_right_rpm).
    """
    w_left_rads = (2 * v - w * wheel_distance) / (2 * wheel_radius)
    w_right_rads = (2 * v + w * wheel_distance) / (2 * wheel_radius)

    v_left_rpm = velsat(rads2rpm(w_left_rads), v_max_rpm)
    v_right_rpm = velsat(rads2rpm(w_right_rads), v_max_rpm)
    return v_left_rpm, v_right_rpm
