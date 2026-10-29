"""
robot1.py — LIDER — trayectoria circular con pure pursuit
Migrado de robot1.m
"""
import math
from controller import Robot

# =========================================================
# Setup
# =========================================================
TIME_STEP = 64
dt = TIME_STEP / 1000.0

robot = Robot()

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
Ra = 600.0      # mm - radio del circulo
NV = 5          # numero de vueltas
tf = NV * 8      # tiempo total en segundos

kpO = 8.0
kiO = 0.0001
kdO = 0.2
v0 = 150.0       # mm/s
alpha = 0.0001

lookahead_samples = 10
ePtol = 100.0    # mm
# =========================================================

# ---- Esperar GPS valido ----
pos0 = [float('nan')] * 3
while any(math.isnan(v) for v in pos0):
    robot.step(TIME_STEP)
    pos0 = gps.getValues()
x0 = pos0[0] * 1000.0
y0 = pos0[1] * 1000.0

# ---- Esperar compass valido ----
mag0 = [float('nan')] * 3
while any(math.isnan(v) for v in mag0):
    robot.step(TIME_STEP)
    mag0 = compass.getValues()
bearing0 = math.atan2(mag0[1], mag0[0]) - 90.0
theta0 = math.atan2(math.sin(-bearing0), math.cos(-bearing0))

# ---- Calcular centro del circulo ----
xc = x0 + Ra * math.cos(theta0 + math.pi / 2)
yc = y0 + Ra * math.sin(theta0 + math.pi / 2)

# Angulo inicial: donde esta el robot respecto al centro
ang0 = math.atan2(y0 - yc, x0 - xc)

# ---- Generar trayectoria circular ----
w_traj = NV * 2 * math.pi / tf
N_pts = math.ceil(tf / dt) + 1
traj_x = [0.0] * N_pts
traj_y = [0.0] * N_pts
for i in range(N_pts):
    tk = i * dt
    ang = ang0 + w_traj * tk
    traj_x[i] = xc + Ra * math.cos(ang)
    traj_y[i] = yc + Ra * math.sin(ang)

lookahead_index = 0  # 0-based (equivale a 1 en MATLAB)
xg = traj_x[0]
yg = traj_y[0]

EO = 0.0
eO_1 = 0.0

# ---- Ciclo principal ----
while robot.step(TIME_STEP) != -1:

    pos = gps.getValues()
    mag = compass.getValues()
    posx = pos[0] * 1000.0
    posy = pos[1] * 1000.0
    bearing = math.atan2(mag[1], mag[0]) - 90.0
    theta = math.atan2(math.sin(-bearing), math.cos(-bearing))

    ex = xg - posx
    ey = yg - posy
    thetag = math.atan2(ey, ex)
    eP = math.hypot(ex, ey)
    eO = math.atan2(math.sin(thetag - theta), math.cos(thetag - theta))

    # Avanzar lookahead
    if eP <= ePtol:
        lookahead_index = min(lookahead_index + lookahead_samples, N_pts - 1)
        xg = traj_x[lookahead_index]
        yg = traj_y[lookahead_index]

    # Control lineal
    v = v0 * (1 - math.exp(-alpha * eP ** 2))

    # Control angular PID
    eO_D = eO - eO_1
    EO = EO + eO
    w = kpO * eO + kiO * EO + kdO * eO_D
    eO_1 = eO

    # Cinematica inversa
    v_right = velsat(rads2rpm((2 * v + w * wheel_distance) / (2 * wheel_radius)))
    v_left = velsat(rads2rpm((2 * v - w * wheel_distance) / (2 * wheel_radius)))
    left_motor.setVelocity(-rpm2rads(v_left))
    right_motor.setVelocity(-rpm2rads(v_right))
