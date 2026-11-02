% =================================================================
% MT3005 - LABORATORIO 11: Control de robots móviles con ruedas
% -----------------------------------------------------------------
% Esta simulación busca ser un entorno representativo de la 
% situación real de un Pololu 3Pi dentro del Robotat. Tome en 
% consideración, sin embargo, que puede que existan ciertas discre-
% pancias en términos de escalamiento para las velocidades y las 
% ganancias de los controladores, ya que la física del robot se
% aproximó de manera sencilla.
% =================================================================
function pololu3pi_controller
addpath('robotat'); % para poder emplear las funciones del Robotat

TIME_STEP = 64;

% Dimensiones del robot
wheel_radius = 32; % mm
wheel_distance = 96 - 2*6.8; % mm

% Se obtienen los device tags/handles
right_motor = wb_robot_get_device('motor_1');
left_motor = wb_robot_get_device('motor_2');
compass = wb_robot_get_device('compass');
gps = wb_robot_get_device('gps');

% Se configuran y activan el GPS y la brújula
wb_gps_enable(gps, 10);
wb_compass_enable(compass, 10);

% Velocidad máxima, en rpm, de cada una de las ruedas. Se coloca de 
% esta manera ya que si bien la velocidad máxima admisible por los
% motores es de 800 rpm, por seguridad esta se limitará a la mitad. 
v_max = 800 / 2;

% Variables para las velocidades de las ruedas (en rpm <- OJO)
%v_left = 80;
%v_right = 50;

% Funciones de conversión entre rpm y rad/s para cambiar entre las
% dimensionales del robot y el simulador
rads2rpm = @(x) x * ( 60 / (2*pi));
rpm2rads = @(x) x * ( 2*pi / 60);

% Función de saturación para garantizar que las velocidades se
% encuentren en el rango adecuado
velsat = @(vel) sign(vel) * min(abs(vel), v_max);

% Se inicializan los motores
wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_velocity(left_motor, 0.0);
wb_motor_set_velocity(right_motor, 0.0);

% Modos ------------------------------------------------------------
ctrlsel = 2;

% Trayectoria
% points = [0, -1000, -1000, -2000;
%           0,  0,    1000,  2000];
% time_points = [0, 6, 12, 18];
% t = 0:0.01:18;

% Trayectoria
points = [0, -1000;
          0,  1000];
time_points = [0, 6];
t = 0:0.01:7;

pos0 = [NaN NaN NaN];

while any(isnan(pos0))
    wb_robot_step(TIME_STEP);
    pos0 = wb_gps_get_values(gps);
end

x0 = pos0(1)*1000;
y0 = pos0(2)*1000;

points = points + [x0; y0];

[traj, ~, ~, ~] = quinticpolytraj(points, time_points, t);

lookahead_samples = 3;
lookahead_index = 1;
ePtol = 100; % mm (antes 0.5 m)

% Control

xg = 0;
yg =  0;

kpO = 6;
kiO = 0.0001;
kdO = 0.1;

kpO_2 = 10;
kiO_2 = 0.0001;
kdO_2 = 0.5;

EO = 0;
eO_1 = 0;

v0 = 200;
alpha = 0.0001; 


% Ciclo de simulación
while wb_robot_step(TIME_STEP) ~= -1

  % Se obtiene la posición y orientación actual del robot
  pos = wb_gps_get_values(gps);
  mag = wb_compass_get_values(compass);
  posx = pos(1)*1000; 
  posy = pos(2)*1000;
  bearing = atan2(mag(2), mag(1)) - 90;
  theta = atan2(sin(-bearing), cos(-bearing));

  % Punto objetivo (pure pursuit)
  % xg = traj(1, lookahead_index);
  % yg = traj(2, lookahead_index);


switch ctrlsel
    % PID punto-a-punto
    case 1
      % Error
      e = [xg - posx; yg - posy];
      thetag = atan2(e(2), e(1));
    
      eP = norm(e);
      eO = thetag - theta;
      eO = atan2(sin(eO), cos(eO));
    
      v = v0 * (1-exp(-alpha*eP^2));
    
      % Control angular
      eO_D = eO - eO_1;
      EO = EO + eO;
      w = kpO_2*eO + kiO_2*EO + kdO_2*eO_D;
      eO_1 = eO;
    
      % Cambio de punto (pure pursuit)
      if(eP <= ePtol)
          lookahead_index = lookahead_index + lookahead_samples;
          if(lookahead_index > length(traj))
              lookahead_index = length(traj);
          end
           xg = traj(1, lookahead_index);
           yg = traj(2, lookahead_index);
      end
    case 2
        e = [xg - posx; yg - posy];
        thetag = atan2(e(2), e(1));
        
        eP = norm(e);
        eO = thetag - theta;
        eO = atan2(sin(eO), cos(eO));
        
        % Control de velocidad lineal
        %kP = v0 * (1-exp(-alpha*eP^2)) / eP;
        v = v0 * (1-exp(-alpha*eP^2));
        
        % Control de velocidad angular
        eO_D = eO - eO_1;
        EO = EO + eO;
        w = kpO*eO + kiO*EO + kdO*eO_D;
        eO_1 = eO;

    otherwise
        v = 1;
        w = 1;
end
  Len = wheel_distance; % mm
  Rad = wheel_radius;   % mm

  v_right = (2*v + w*Len) / (2*Rad);
  v_left  = (2*v - w*Len) / (2*Rad);

  % rad/s a rpm
  v_right = rads2rpm(v_right);
  v_left  = rads2rpm(v_left);

  disp([v, w, v_right, v_left, theta, posx, posy])

  % Se mapea del uniciclo de regreso al robot diferencial.
  % OJO: los resultados de estas fórmulas están en rad/s, DEBEN
  % cambiarse a rpm (usar la función auxiliar de conversión)

  % Se envían las velocidades a los motores de las ruedas (debe
  % garantizarse que estas no superan el valor máximo)
  % OJO: el signo y dimensionales empleadas para las velocidades de las
  % ruedas son específicas a la simulación, para el robot real no hay que
  % invertirlas ni convertirlas a rad/s, ya que el robot funciona en rpm.
  wb_motor_set_velocity(left_motor, -rpm2rads(velsat(v_left)));
  wb_motor_set_velocity(right_motor, -rpm2rads(velsat(v_right)));

  % Flush para gráficos
  drawnow;

end

% cleanup code goes here: write data to files, etc.
