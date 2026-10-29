% =========================================================================
% robot3.m — SEGUIDOR 3 — sigue a robot2 via supervisor
% =========================================================================
function robot3
TIME_STEP = 64;

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
d_sep  = 300;  % mm — distancia deseada detras del lider
kpO    = 6;
kiO    = 0.0001;
kdO    = 0.1;
v0     = 200;    % mm/s
alpha  = 0.0001;
eP_stop = 30;    % mm — zona muerta: si esta mas cerca, frena
% =========================================================

% Primer paso y obtener nodo del lider
wb_robot_step(TIME_STEP);
leader_node    = wb_supervisor_node_get_from_def('ROBOT2');
leader_trans_f = wb_supervisor_node_get_field(leader_node, 'translation');

EO = 0; eO_1 = 0;

% Esperar GPS propio valido
pos0 = [NaN NaN NaN];
while any(isnan(pos0))
    wb_robot_step(TIME_STEP);
    pos0 = wb_gps_get_values(gps);
end

while wb_robot_step(TIME_STEP) ~= -1

    % --- Posicion del lider (en mm) ---
    lpos = wb_supervisor_field_get_sf_vec3f(leader_trans_f);
    lx   = lpos(1)*1000;
    ly   = lpos(2)*1000;

    % --- Posicion y orientacion propias ---
    pos     = wb_gps_get_values(gps);
    mag     = wb_compass_get_values(compass);
    posx    = pos(1)*1000;
    posy    = pos(2)*1000;
    bearing = atan2(mag(2), mag(1)) - 90;
    theta   = atan2(sin(-bearing), cos(-bearing));

    % --- Punto objetivo: en la linea entre yo y el lider, a d_sep del lider ---
    % Direccion desde mi posicion hacia el lider
    dir    = [lx - posx; ly - posy];
    dist   = norm(dir);

    if dist > 1e-3
        dir_unit = dir / dist;
    else
        dir_unit = [0; 0];
    end

    % El objetivo esta d_sep mm antes del lider (en la linea que los une)
    xg = lx - d_sep * dir_unit(1);
    yg = ly - d_sep * dir_unit(2);

    % --- Error ---
    e      = [xg - posx; yg - posy];
    thetag = atan2(e(2), e(1));
    eP     = norm(e);
    eO     = atan2(sin(thetag - theta), cos(thetag - theta));

    % --- Zona muerta: si ya esta cerca, frenar y resetear integrador ---
    if eP < eP_stop
        v  = 0;
        w  = 0;
        EO = 0;
        eO_1 = 0;
    else
        % Control lineal: acercamiento exponencial
        v = v0 * (1 - exp(-alpha * eP^2));

        % Control angular: PID
        eO_D = eO - eO_1;
        EO   = EO + eO;
        w    = kpO*eO + kiO*EO + kdO*eO_D;
        eO_1 = eO;
    end

    % --- Cinematica inversa ---
    v_right = velsat(rads2rpm((2*v + w*wheel_distance) / (2*wheel_radius)));
    v_left  = velsat(rads2rpm((2*v - w*wheel_distance) / (2*wheel_radius)));
    wb_motor_set_velocity(left_motor,  -rpm2rads(v_left));
    wb_motor_set_velocity(right_motor, -rpm2rads(v_right));
    drawnow;
end
