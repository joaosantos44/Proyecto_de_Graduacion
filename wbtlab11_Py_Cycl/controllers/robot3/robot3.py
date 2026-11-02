"""
robot3.py — CYCLIC PURSUIT — agente 3/4
Persigue un punto phi_offset radianes adelante de ROBOT4 sobre el
circulo de radio Ra centrado en el origen del arena. Cierra el ciclo:
1->2->3->4->1. Ningun agente es "lider" fijo; todos corren esta misma logica.
Migrado de cyclic_pursuit.m
"""
import math
from controller import Supervisor

NEXT_DEF = 'ROBOT4'   # DEF del agente que este robot persigue (siguiente en el ciclo)

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
# PARAMETROS CONFIGURABLES (mismo criterio para los 4 agentes)
CENTER_X = 0.0      # mm - centro del circulo de orbita (centro del arena)
CENTER_Y = 0.0      # mm
Ra = 1000.0         # mm - radio de la orbita deseada (1.0 m, margen amplio vs paredes)
N_AGENTS = 4
PHI_OFFSET = 1.5 * math.pi / N_AGENTS  # cuanto "adelanta" el objetivo (2pi/N -> agentes equiespaciados en todo el circulo)

kpO = 6.0
kiO = 0.0001
kdO = 0.1
v0 = 150.0          # mm/s
alpha = 0.0001
w_max = 6.0          # rad/s - saturacion (protege hardware, ver cyclic_pursuit.m)
# =========================================================

# Primer paso y obtener nodo del siguiente agente en el ciclo
robot.step(TIME_STEP)
next_node = robot.getFromDef(NEXT_DEF)
next_trans_f = next_node.getField('translation')

EO = 0.0
eO_1 = 0.0

# Esperar GPS propio valido
pos0 = [float('nan')] * 3
while any(math.isnan(v) for v in pos0):
    robot.step(TIME_STEP)
    pos0 = gps.getValues()

while robot.step(TIME_STEP) != -1:

    # --- Posicion del siguiente agente en el ciclo (en mm) ---
    npos = next_trans_f.getSFVec3f()
    nx = npos[0] * 1000.0
    ny = npos[1] * 1000.0

    # --- Posicion y orientacion propias ---
    pos = gps.getValues()
    mag = compass.getValues()
    posx = pos[0] * 1000.0
    posy = pos[1] * 1000.0
    bearing = math.atan2(mag[1], mag[0]) - 90.0
    theta = math.atan2(math.sin(-bearing), math.cos(-bearing))

    # --- Angulo actual del siguiente agente sobre el circulo (relativo al centro) ---
    ang_next = math.atan2(ny - CENTER_Y, nx - CENTER_X)

    # --- Punto objetivo: PHI_OFFSET adelante en el circulo ---
    ang_target = ang_next + PHI_OFFSET
    xg = CENTER_X + Ra * math.cos(ang_target)
    yg = CENTER_Y + Ra * math.sin(ang_target)

    # --- Error ---
    ex = xg - posx
    ey = yg - posy
    thetag = math.atan2(ey, ex)
    eP = math.hypot(ex, ey)
    eO = math.atan2(math.sin(thetag - theta), math.cos(thetag - theta))

    # --- Control lineal: acercamiento exponencial (nunca "llega" del todo:
    #     el objetivo siempre se mueve, por eso no hay zona muerta con v=0) ---
    if eP > 1e-6:
        v = v0 * (1 - math.exp(-alpha * eP ** 2))
    else:
        v = 0.0

    # --- Control angular: PID ---
    eO_D = eO - eO_1
    EO = EO + eO
    w = kpO * eO + kiO * EO + kdO * eO_D
    eO_1 = eO

    # --- Saturacion de velocidad angular (evita picos en hardware) ---
    w = max(-w_max, min(w_max, w))

    # --- Cinematica inversa ---
    v_right = velsat(rads2rpm((2 * v + w * wheel_distance) / (2 * wheel_radius)))
    v_left = velsat(rads2rpm((2 * v - w * wheel_distance) / (2 * wheel_radius)))
    left_motor.setVelocity(-rpm2rads(v_left))
    right_motor.setVelocity(-rpm2rads(v_right))
