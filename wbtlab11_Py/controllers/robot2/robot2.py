"""
robot2.py — SEGUIDOR 2 — sigue a robot1 via supervisor
Migrado de robot2.m
"""
import math
from controller import Supervisor

LEADER_DEF = 'ROBOT1'

# =========================================================
# Setup
# =========================================================
TIME_STEP = 64

robot = Supervisor()

right_motor = robot.getDevice('motor_1')
left_motor = robot.getDevice('motor_2')
compass = robot.getDevice('compass')
gps = robot.getDevice('gps')

gps.enable(TIME_STEP)
compass.enable(TIME_STEP)
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


def rads2rpm(x):
    return x * (60.0 / (2.0 * math.pi))


def rpm2rads(x):
    return x * (2.0 * math.pi / 60.0)


wheel_radius = 32.0
wheel_distance = 96.0 - 2 * 6.8
v_max = 400.0


def velsat(vel):
    return math.copysign(min(abs(vel), v_max), vel) if vel != 0 else 0.0


# =========================================================
# PARAMETROS CONFIGURABLES
d_sep = 300.0    # mm - distancia deseada detras del lider
kpO = 6.0
kiO = 0.0001
kdO = 0.1
v0 = 200.0       # mm/s
alpha = 0.0001
eP_stop = 30.0   # mm - zona muerta: si esta mas cerca, frena
# =========================================================

# Primer paso y obtener nodo del lider
robot.step(TIME_STEP)
leader_node = robot.getFromDef(LEADER_DEF)
leader_trans_f = leader_node.getField('translation')

EO = 0.0
eO_1 = 0.0

# Esperar GPS propio valido
pos0 = [float('nan')] * 3
while any(math.isnan(v) for v in pos0):
    robot.step(TIME_STEP)
    pos0 = gps.getValues()

while robot.step(TIME_STEP) != -1:

    # --- Posicion del lider (en mm) ---
    lpos = leader_trans_f.getSFVec3f()
    lx = lpos[0] * 1000.0
    ly = lpos[1] * 1000.0

    # --- Posicion y orientacion propias ---
    pos = gps.getValues()
    mag = compass.getValues()
    posx = pos[0] * 1000.0
    posy = pos[1] * 1000.0
    bearing = math.atan2(mag[1], mag[0]) - 90.0
    theta = math.atan2(math.sin(-bearing), math.cos(-bearing))

    # --- Punto objetivo: en la linea entre yo y el lider, a d_sep del lider ---
    dx = lx - posx
    dy = ly - posy
    dist = math.hypot(dx, dy)

    if dist > 1e-3:
        dir_x = dx / dist
        dir_y = dy / dist
    else:
        dir_x = 0.0
        dir_y = 0.0

    # El objetivo esta d_sep mm antes del lider (en la linea que los une)
    xg = lx - d_sep * dir_x
    yg = ly - d_sep * dir_y

    # --- Error ---
    ex = xg - posx
    ey = yg - posy
    thetag = math.atan2(ey, ex)
    eP = math.hypot(ex, ey)
    eO = math.atan2(math.sin(thetag - theta), math.cos(thetag - theta))

    # --- Zona muerta: si ya esta cerca, frenar y resetear integrador ---
    if eP < eP_stop:
        v = 0.0
        w = 0.0
        EO = 0.0
        eO_1 = 0.0
    else:
        # Control lineal: acercamiento exponencial
        v = v0 * (1 - math.exp(-alpha * eP ** 2))

        # Control angular: PID
        eO_D = eO - eO_1
        EO = EO + eO
        w = kpO * eO + kiO * EO + kdO * eO_D
        eO_1 = eO

    # --- Cinematica inversa ---
    v_right = velsat(rads2rpm((2 * v + w * wheel_distance) / (2 * wheel_radius)))
    v_left = velsat(rads2rpm((2 * v - w * wheel_distance) / (2 * wheel_radius)))
    left_motor.setVelocity(-rpm2rads(v_left))
    right_motor.setVelocity(-rpm2rads(v_right))
