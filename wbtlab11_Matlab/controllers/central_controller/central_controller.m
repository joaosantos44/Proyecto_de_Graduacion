% =========================================================================
% Central_controller.m
% -------------------------------------------------------------------------
% robot1 tiene supervisor=TRUE y corre este archivo.
% Los robots 2,3,4 tienen controller "<none>".
%
% robot1 controla sus propios motores normalmente Y ademas mueve a los
% robots 2,3,4 via wb_supervisor_field_set (igual que Opcion B para ellos).
% robot1 usa sus sensores GPS y Compass reales para su propio control.
% =========================================================================
function central_controller
TIME_STEP = 64;
dt = TIME_STEP / 1000;

% ---- Dispositivos propios de robot1 ----
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

% ---- Parametros de control ----
% robot1 en mm (usa GPS real), robots 2-4 en mm (convertidos a m internamente)
%         xg_rel  yg_rel  kpO   kiO     kdO    v0    alpha
params = [ 1000,    0,     6,  0.0001,  0.1,  200,  0.0001;  % robot1 (mm, rpm)
          -1000,    0,     6,  0.0001,  0.1,  0.5,  10;      % robot2 (mm->m, m/s)
              0, 1000,     6,  0.0001,  0.1,  0.5,  10;      % robot3
              0,-1000,     6,  0.0001,  0.1,  0.5,  10];     % robot4

N     = 4;
names_sup = {"robot2","robot3","robot4"};  % los que controla via supervisor

% ---- Nodos supervisor para robots 2-4 ----
for i = 1:3
    nd_s(i)      = wb_supervisor_node_get_from_def(upper(names_sup{i}));
    trans_fs(i)  = wb_supervisor_node_get_field(nd_s(i), 'translation');
    rot_fs(i)    = wb_supervisor_node_get_field(nd_s(i), 'rotation');
end

% ---- Estado robots 2-4 (metros) ----
state_s = zeros(3, 3);  % [x y theta] por robot
xg_s    = zeros(3, 1);
yg_s    = zeros(3, 1);
EO_s    = zeros(3, 1);
eO_1_s  = zeros(3, 1);

% ---- Estado robot1 ----
EO1   = 0;
eO_1_1 = 0;
xg1   = 0; yg1 = 0;
initialized = false;

% ---- Esperar GPS valido ----
pos0 = [NaN NaN NaN];
while any(isnan(pos0))
    wb_robot_step(TIME_STEP);
    pos0 = wb_gps_get_values(gps);
end
xg1 = pos0(1)*1000 + params(1,1);
yg1 = pos0(2)*1000 + params(1,2);

% Inicializar posiciones de robots 2-4
for i = 1:3
    pos       = wb_supervisor_field_get_sf_vec3f(trans_fs(i));
    rot       = wb_supervisor_field_get_sf_rotation(rot_fs(i));
    state_s(i,1) = pos(1);
    state_s(i,2) = pos(2);
    state_s(i,3) = rot(4) * sign(rot(3));
    xg_s(i) = pos(1) + params(i+1,1)/1000;
    yg_s(i) = pos(2) + params(i+1,2)/1000;
end

% ---- Ciclo principal ----
while wb_robot_step(TIME_STEP) ~= -1

    % ========================
    % ROBOT 1: control real
    % ========================
    pos = wb_gps_get_values(gps);
    mag = wb_compass_get_values(compass);
    posx    = pos(1)*1000;
    posy    = pos(2)*1000;
    bearing = atan2(mag(2), mag(1)) - 90;
    theta1  = atan2(sin(-bearing), cos(-bearing));

    e1     = [xg1 - posx; yg1 - posy];
    thetag = atan2(e1(2), e1(1));
    eP1    = norm(e1);
    eO1    = atan2(sin(thetag - theta1), cos(thetag - theta1));

    v1 = params(1,6) * (1 - exp(-params(1,7) * eP1^2));
    eO_D1  = eO1 - eO_1_1;
    EO1    = EO1 + eO1;
    w1     = params(1,3)*eO1 + params(1,4)*EO1 + params(1,5)*eO_D1;
    eO_1_1 = eO1;

    v_right1 = rads2rpm((2*v1 + w1*wheel_distance) / (2*wheel_radius));
    v_left1  = rads2rpm((2*v1 - w1*wheel_distance) / (2*wheel_radius));
    wb_motor_set_velocity(left_motor,  -rpm2rads(velsat(v_left1)));
    wb_motor_set_velocity(right_motor, -rpm2rads(velsat(v_right1)));

    % ========================
    % ROBOTS 2-4: supervisor
    % ========================
    for i = 1:3
        x     = state_s(i,1);
        y     = state_s(i,2);
        theta = state_s(i,3);

        kpO_i   = params(i+1,3); kiO_i = params(i+1,4);
        kdO_i   = params(i+1,5); v0_i  = params(i+1,6);
        alpha_i = params(i+1,7);

        e      = [xg_s(i) - x; yg_s(i) - y];
        thetag = atan2(e(2), e(1));
        eP     = norm(e);
        eO     = atan2(sin(thetag - theta), cos(thetag - theta));

        v = v0_i * (1 - exp(-alpha_i * eP^2));
        eO_D    = eO - eO_1_s(i);
        EO_s(i) = EO_s(i) + eO;
        w       = kpO_i*eO + kiO_i*EO_s(i) + kdO_i*eO_D;
        eO_1_s(i) = eO;

        state_s(i,1) = x     + dt * v * cos(theta);
        state_s(i,2) = y     + dt * v * sin(theta);
        state_s(i,3) = atan2(sin(theta + dt*w), cos(theta + dt*w));

        wb_supervisor_field_set_sf_vec3f(trans_fs(i),  [state_s(i,1), state_s(i,2), 0.037]);
        wb_supervisor_field_set_sf_rotation(rot_fs(i), [0, 0, 1, state_s(i,3)]);
    end

    drawnow;
end
