% =========================================================================
% robot1.m — LIDER — trayectoria circular con pure pursuit
% =========================================================================
function robot1
TIME_STEP = 64;
dt = TIME_STEP / 1000;

right_motor = wb_robot_get_device('motor_1');
left_motor  = wb_robot_get_device('motor_2');
compass     = wb_robot_get_device('compass');
gps         = wb_robot_get_device('gps');

wb_gps_enable(gps, TIME_STEP);
wb_compass_enable(compass, TIME_STEP);
wb_motor_set_position(left_motor,  inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_velocity(left_motor,  0.0);
wb_motor_set_velocity(right_motor, 0.0);

rads2rpm = @(x) x * (60 / (2*pi));
rpm2rads = @(x) x * (2*pi / 60);
wheel_radius   = 32;
wheel_distance = 96 - 2*6.8;
v_max = 400;
velsat = @(vel) sign(vel) * min(abs(vel), v_max);

% =========================================================
% PARAMETROS CONFIGURABLES
Ra     = 600;      % mm — radio del circulo
NV     = 5;        % numero de vueltas
tf     = NV * 8;   % tiempo total en segundos

kpO   = 8;
kiO   = 0.0001;
kdO   = 0.2;
v0    = 150;       % mm/s
alpha = 0.0001;

lookahead_samples = 10;
ePtol             = 100;   % mm
% =========================================================

% ---- Esperar GPS valido ----
pos0 = [NaN NaN NaN];
while any(isnan(pos0))
    wb_robot_step(TIME_STEP);
    pos0 = wb_gps_get_values(gps);
end
x0 = pos0(1)*1000;
y0 = pos0(2)*1000;

% ---- Esperar compass valido ----
mag0 = [NaN NaN NaN];
while any(isnan(mag0))
    wb_robot_step(TIME_STEP);
    mag0 = wb_compass_get_values(compass);
end
bearing0 = atan2(mag0(2), mag0(1)) - 90;
theta0   = atan2(sin(-bearing0), cos(-bearing0));

% ---- Calcular centro del circulo ----
% Centro = Ra mm a la izquierda de la orientacion inicial
xc = x0 + Ra * cos(theta0 + pi/2);
yc = y0 + Ra * sin(theta0 + pi/2);

% Angulo inicial: donde esta el robot respecto al centro
ang0 = atan2(y0 - yc, x0 - xc);

% ---- Generar trayectoria circular ----
w_traj = NV*2*pi/tf;
N_pts  = ceil(tf/dt) + 1;
traj   = zeros(2, N_pts);
for i = 1:N_pts
    tk = (i-1)*dt;
    ang = ang0 + w_traj*tk;
    traj(1,i) = xc + Ra*cos(ang);
    traj(2,i) = yc + Ra*sin(ang);
end

lookahead_index = 1;
xg = traj(1,1);
yg = traj(2,1);

EO = 0; eO_1 = 0;

% ---- Ciclo principal ----
while wb_robot_step(TIME_STEP) ~= -1

    pos     = wb_gps_get_values(gps);
    mag     = wb_compass_get_values(compass);
    posx    = pos(1)*1000;
    posy    = pos(2)*1000;
    bearing = atan2(mag(2), mag(1)) - 90;
    theta   = atan2(sin(-bearing), cos(-bearing));

    e      = [xg - posx; yg - posy];
    thetag = atan2(e(2), e(1));
    eP     = norm(e);
    eO     = atan2(sin(thetag - theta), cos(thetag - theta));

    % Avanzar lookahead
    if eP <= ePtol
        lookahead_index = min(lookahead_index + lookahead_samples, N_pts);
        xg = traj(1, lookahead_index);
        yg = traj(2, lookahead_index);
    end

    % Control lineal
    v = v0 * (1 - exp(-alpha * eP^2));

    % Control angular PID
    eO_D = eO - eO_1;
    EO   = EO + eO;
    w    = kpO*eO + kiO*EO + kdO*eO_D;
    eO_1 = eO;

    % Cinematica inversa
    v_right = velsat(rads2rpm((2*v + w*wheel_distance) / (2*wheel_radius)));
    v_left  = velsat(rads2rpm((2*v - w*wheel_distance) / (2*wheel_radius)));
    wb_motor_set_velocity(left_motor,  -rpm2rads(v_left));
    wb_motor_set_velocity(right_motor, -rpm2rads(v_right));
    drawnow;
end
