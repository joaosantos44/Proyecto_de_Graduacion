%% Pru 1

Waypoints = [0,0,0;...
            1,1,0;...
            0,2,0];
% Orientation = [0,0,0;...
%             0,0,90;...
%             0,0,180];
% quatOrientation = quaternion(Orientation,'eulerd','ZYX','frame');
% Velocities = [1,0,0;...
%              0,1,0;...
%             -1,0,0];
TimeOfArrival = [0;1;2];
trajectory = waypointTrajectory(Waypoints,TimeOfArrival,'SampleRate',10);
truePosition = zeros(trajectory.SampleRate*trajectory.TimeOfArrival(end)-1,3); 
c = 1;
while ~isDone(trajectory)
   truePosition(c,:) = trajectory();
   c = c + 1;
end

figure
plot3(Waypoints(:,1),Waypoints(:,2),Waypoints(:,3),'LineStyle','none','Marker','o','MarkerEdgeColor','r');
hold on;
plot3(truePosition(:,1),truePosition(:,2),truePosition(:,3),'Color','k');
daspect([1 1 1]);
grid on;
xlabel('x');
ylabel('y');
zlabel('z');

%% Pru 2

Waypoints = [0,0,0;...
            1,0,1;...
            0,0,2];
% Orientation = [0,0,0;...
%             0,-90,0;...
%             0,-180,0];
% quatOrientation = quaternion(Orientation,'eulerd','ZYX','frame');
% Velocities = [1,0,0;...
%              0,0,1;...
%             -1,0,0];
TimeOfArrival = [0;1;2];
trajectory = waypointTrajectory(Waypoints,TimeOfArrival,'SampleRate',10);
truePosition = zeros(trajectory.SampleRate*trajectory.TimeOfArrival(end)-1,3); 
c = 1;
while ~isDone(trajectory)
   truePosition(c,:) = trajectory();
   c = c + 1;
end

figure
plot3(Waypoints(:,1),Waypoints(:,2),Waypoints(:,3),'LineStyle','none','Marker','o','MarkerEdgeColor','r');
hold on;
plot3(truePosition(:,1),truePosition(:,2),truePosition(:,3),'Color','k');
daspect([1 1 1]);
grid on;
xlabel('x');
ylabel('y');
zlabel('z');


%% Pru 3

% --- Parámetros ---
R = 5;              % Radio del círculo
omega = 2;          % Velocidad angular
t = linspace(0, 4*pi, 400); % Vector de tiempo (una vuelta completa)

% --- Cálculo de la trayectoria ---
x = R * cos(omega * t);
y = R * sin(omega * t);

% --- Cálculo del Gradiente (Velocidad) ---
% Derivadas: dx/dt = -R*sin(t), dy/dt = R*cos(t)
vx = -R * omega * sin(omega * t);
vy = R * omega * cos(omega * t);

% --- Configuración de la Figura ---
figure('Color', 'w');
hold on; axis equal;
grid on;
xlabel('Eje X'); ylabel('Eje Y');
title('Trayectoria Circular y Vector Gradiente (Velocidad)');
xlim([-R-2, R+2]); ylim([-R-2, R+2]);

% Dibujar la trayectoria completa de fondo (atenuada)
plot(x, y, '--', 'Color', [0.7 0.7 0.7]); 

% --- Animación ---
hPath = plot(nan, nan, 'b', 'LineWidth', 2); % Estela del recorrido
hGrad = quiver(nan, nan, nan, nan, 0, 'r', 'LineWidth', 1.5); % Vector gradiente
hPart = plot(nan, nan, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8); % La partícula

for k = 1:length(t)
    % Actualizar estela
    set(hPath, 'XData', x(1:k), 'YData', y(1:k));
    
    % Actualizar partícula
    set(hPart, 'XData', x(k), 'YData', y(k));
    
    % Actualizar vector gradiente (quiver)
    set(hGrad, 'XData', x(k), 'YData', y(k), 'UData', vx(k), 'VData', vy(k));
    
    drawnow; % Refrescar gráfico
    pause(0.05); % Control de velocidad de animación
end

legend([hPath, hGrad], {'Recorrido', 'Gradiente (Velocidad)'}, 'Location', 'northeast');



%% Pru 4

% Simulación de Dos Partículas en Trayectoria Circular
clear; clc; close all;

% --- Parámetros Partícula 1 (Azul) ---
R1 = 5;              
omega1 = 1;          
color1 = [0, 0.4470, 0.7410]; % Azul

% --- Parámetros Partícula 2 (Roja) ---
R2 = 5;              
omega2 = 1;        
color2 = [0.8500, 0.3250, 0.0980]; % Rojo/Naranja

% --- Tiempo ---
t = linspace(0, 4*pi, 150); 

% --- Cálculos Partícula 1 ---
x1 = R1 * cos(omega1 * t);
y1 = R1 * sin(omega1 * t);
vx1 = -R1 * omega1 * sin(omega1 * t);
vy1 = R1 * omega1 * cos(omega1 * t);

% --- Cálculos Partícula 2 ---
x2 = R2 * cos(omega2 * t);
y2 = R2 * sin(omega2 * t);
vx2 = -R2 * omega2 * sin(omega2 * t);
vy2 = R2 * omega2 * cos(omega2 * t);

% --- Configuración de la Figura ---
figure('Color', 'w', 'Name', 'Simulación de Dos Partículas');
hold on; axis equal; grid on;
xlabel('Eje X'); ylabel('Eje Y');
title('Movimiento Circular de Dos Partículas y sus Gradientes');
limit = max(R1, R2) + 2;
xlim([-limit, limit]); ylim([-limit, limit]);

% Dibujar órbitas de fondo
plot(R1*cos(t), R1*sin(t), ':', 'Color', [0.8 0.8 0.8]);
plot(R2*cos(t), R2*sin(t), ':', 'Color', [0.8 0.8 0.8]);

% --- Handles para Animación ---
% Partícula 1
hPath1 = plot(nan, nan, 'Color', color1, 'LineWidth', 1.5);
hGrad1 = quiver(nan, nan, nan, nan, 0, 'Color', color1, 'LineWidth', 2, 'MaxHeadSize', 0.5);
hPart1 = plot(nan, nan, 'o', 'MarkerFaceColor', color1, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);

% Partícula 2
hPath2 = plot(nan, nan, 'Color', color2, 'LineWidth', 1.5);
hGrad2 = quiver(nan, nan, nan, nan, 0, 'Color', color2, 'LineWidth', 2, 'MaxHeadSize', 0.5);
hPart2 = plot(nan, nan, 'o', 'MarkerFaceColor', color2, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);

legend([hPart1, hPart2], {'Partícula 1 (Lenta/Grande)', 'Partícula 2 (Rápida/Pequeña)'});

% --- Bucle de Animación ---
for k = 1:length(t)
    % Actualizar Partícula 1
    set(hPath1, 'XData', x1(1:k), 'YData', y1(1:k));
    set(hPart1, 'XData', x1(k), 'YData', y1(k));
    set(hGrad1, 'XData', x1(k), 'YData', y1(k), 'UData', vx1(k), 'VData', vy1(k));
    
    % Actualizar Partícula 2
    set(hPath2, 'XData', x2(1:k), 'YData', y2(1:k));
    set(hPart2, 'XData', x2(k), 'YData', y2(k));
    set(hGrad2, 'XData', x2(k), 'YData', y2(k), 'UData', vx2(k), 'VData', vy2(k));
    
    drawnow;
    pause(0.04);
end



%% Pru 4

% Simulación de Dos Partículas con Inicio Diferido
clear; clc; close all;

% --- Parámetros Partícula 1 ---
R1 = 5; 
omega1 = 1; 
color1 = [0, 0.4470, 0.7410];

% --- Parámetros Partícula 2 ---
R2 = 5; 
omega2 = 1; 
color2 = [0.8500, 0.3250, 0.0980];

% --- Tiempo ---
t = linspace(0, 4*pi, 200); 
delay_frames = 25; % Número de pasos que esperará la partícula 2

% --- Cálculos Partícula 1 ---
x1 = R1 * cos(omega1 * t);
y1 = R1 * sin(omega1 * t);
vx1 = -R1 * omega1 * sin(omega1 * t);
vy1 = R1 * omega1 * cos(omega1 * t);

% --- Cálculos Partícula 2 ---
x2 = R2 * cos(omega2 * t);
y2 = R2 * sin(omega2 * t);
vx2 = -R2 * omega2 * sin(omega2 * t);
vy2 = R2 * omega2 * cos(omega2 * t);

% --- Configuración de la Figura ---
figure('Color', 'w'); hold on; axis equal; grid on;
xlabel('Eje X'); ylabel('Eje Y');
title(['Partícula 2 inicia con un retraso de ', num2str(delay_frames), ' frames']);
limit = max(R1, R2) + 2;
xlim([-limit, limit]); ylim([-limit, limit]);

% Órbitas de fondo
plot(R1*cos(linspace(0,2*pi,100)), R1*sin(linspace(0,2*pi,100)), ':', 'Color', [0.8 0.8 0.8]);
plot(R2*cos(linspace(0,2*pi,100)), R2*sin(linspace(0,2*pi,100)), ':', 'Color', [0.8 0.8 0.8]);

% --- Handles Animación ---
hPath1 = plot(nan, nan, 'Color', color1, 'LineWidth', 1.5);
hGrad1 = quiver(nan, nan, nan, nan, 0, 'Color', color1, 'LineWidth', 2);
hPart1 = plot(nan, nan, 'o', 'MarkerFaceColor', color1, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);

hPath2 = plot(nan, nan, 'Color', color2, 'LineWidth', 1.5);
hGrad2 = quiver(nan, nan, nan, nan, 0, 'Color', color2, 'LineWidth', 2);
hPart2 = plot(nan, nan, 'o', 'MarkerFaceColor', color2, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);

% --- Bucle de Animación ---
for k = 1:length(t)
    % La Partícula 1 siempre se mueve
    set(hPath1, 'XData', x1(1:k), 'YData', y1(1:k));
    set(hPart1, 'XData', x1(k), 'YData', y1(k));
    set(hGrad1, 'XData', x1(k), 'YData', y1(k), 'UData', vx1(k), 'VData', vy1(k));
    
    % La Partícula 2 solo inicia si el contador k es mayor al delay
    if k > delay_frames
        % Usamos un índice ajustado para que empiece desde el inicio de sus vectores
        idx2 = k - delay_frames; 
        
        set(hPath2, 'XData', x2(1:idx2), 'YData', y2(1:idx2));
        set(hPart2, 'XData', x2(idx2), 'YData', y2(idx2));
        set(hGrad2, 'XData', x2(idx2), 'YData', y2(idx2), 'UData', vx2(idx2), 'VData', vy2(idx2));
    end
    
    drawnow;
    pause(0.03);
end


%% Pru 5

% Simulación de Dos Partículas con Inicio Diferido
clear; clc; close all;

% --- Parámetros Partícula 1 ---
R1 = 5; 
omega1 = 1; 
color1 = [0, 0.4470, 0.7410];

% --- Parámetros Partícula 2 ---
R2 = 5; 
omega2 = 1; 
color2 = [0.8500, 0.3250, 0.0980];

% --- Parámetros Partícula 3 ---
R3 = 5; 
omega3 = 1; 
color3 = [0.4940, 0.1840, 0.5560];

% --- Parámetros Partícula 4 ---
R4 = 5; 
omega4 = 1; 
color4 = [0.2, 1, 0];

% --- Tiempo ---
t = linspace(0, 10*pi, 500); 
delay_frames_1 = 25; % Número de pasos que esperará la partícula 2
delay_frames_2 = 50;
delay_frames_3 = 75;

% --- Cálculos Partícula 1 ---
x1 = R1 * cos(omega1 * t);
y1 = R1 * sin(omega1 * t);
vx1 = -R1 * omega1 * sin(omega1 * t);
vy1 = R1 * omega1 * cos(omega1 * t);

% --- Cálculos Partícula 2 ---
x2 = R2 * cos(omega2 * t);
y2 = R2 * sin(omega2 * t);
vx2 = -R2 * omega2 * sin(omega2 * t);
vy2 = R2 * omega2 * cos(omega2 * t);

% --- Cálculos Partícula 3 ---
x3 = R3 * cos(omega3 * t);
y3 = R3 * sin(omega3 * t);
vx3 = -R3 * omega3 * sin(omega3 * t);
vy3 = R3 * omega3 * cos(omega3 * t);

% --- Cálculos Partícula 3 ---
x4 = R4 * cos(omega4 * t);
y4 = R4 * sin(omega4 * t);
vx4 = -R4 * omega4 * sin(omega4 * t);
vy4 = R4 * omega4 * cos(omega4 * t);

% --- Configuración de la Figura ---
figure('Color', 'w'); hold on; axis equal; grid on;
xlabel('Eje X'); ylabel('Eje Y');
title(['Partícula 2, 3 y 4 inicia con un retraso de ', num2str(delay_frames_1), ' frames cada una']);
limit = max(R1, R4) + 2;
xlim([-limit, limit]); ylim([-limit, limit]);

% Órbitas de fondo
plot(R1*cos(linspace(0,2*pi,100)), R1*sin(linspace(0,2*pi,100)), ':', 'Color', [0.8 0.8 0.8]);
plot(R2*cos(linspace(0,2*pi,100)), R2*sin(linspace(0,2*pi,100)), ':', 'Color', [0.8 0.8 0.8]);
plot(R3*cos(linspace(0,2*pi,100)), R3*sin(linspace(0,2*pi,100)), ':', 'Color', [0.8 0.8 0.8]);
plot(R4*cos(linspace(0,2*pi,100)), R4*sin(linspace(0,2*pi,100)), ':', 'Color', [0.8 0.8 0.8]);

% --- Handles Animación ---
hPath1 = plot(nan, nan, 'Color', color1, 'LineWidth', 1.5);
hGrad1 = quiver(nan, nan, nan, nan, 0, 'Color', color1, 'LineWidth', 2);
hPart1 = plot(nan, nan, 'o', 'MarkerFaceColor', color1, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);

hPath2 = plot(nan, nan, 'Color', color2, 'LineWidth', 1.5);
hGrad2 = quiver(nan, nan, nan, nan, 0, 'Color', color2, 'LineWidth', 2);
hPart2 = plot(nan, nan, 'o', 'MarkerFaceColor', color2, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);

hPath3 = plot(nan, nan, 'Color', color3, 'LineWidth', 1.5);
hGrad3 = quiver(nan, nan, nan, nan, 0, 'Color', color3, 'LineWidth', 2);
hPart3 = plot(nan, nan, 'o', 'MarkerFaceColor', color3, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);

hPath4 = plot(nan, nan, 'Color', color4, 'LineWidth', 1.5);
hGrad4 = quiver(nan, nan, nan, nan, 0, 'Color', color4, 'LineWidth', 2);
hPart4 = plot(nan, nan, 'o', 'MarkerFaceColor', color4, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);

% --- Bucle de Animación ---
for k = 1:length(t)
    % La Partícula 1 siempre se mueve
    set(hPath1, 'XData', x1(1:k), 'YData', y1(1:k));
    set(hPart1, 'XData', x1(k), 'YData', y1(k));
    set(hGrad1, 'XData', x1(k), 'YData', y1(k), 'UData', vx1(k), 'VData', vy1(k));
    
    % La Partícula 2 solo inicia si el contador k es mayor al delay
    if k > delay_frames_1
        % Usamos un índice ajustado para que empiece desde el inicio de sus vectores
        idx2 = k - delay_frames_1; 
        
        set(hPath2, 'XData', x2(1:idx2), 'YData', y2(1:idx2));
        set(hPart2, 'XData', x2(idx2), 'YData', y2(idx2));
        set(hGrad2, 'XData', x2(idx2), 'YData', y2(idx2), 'UData', vx2(idx2), 'VData', vy2(idx2));
    end

    % La Partícula 3 solo inicia si el contador k es mayor al delay
    if k > delay_frames_2
        % Usamos un índice ajustado para que empiece desde el inicio de sus vectores
        idx3 = k - delay_frames_2; 
        
        set(hPath3, 'XData', x3(1:idx3), 'YData', y3(1:idx3));
        set(hPart3, 'XData', x3(idx3), 'YData', y3(idx3));
        set(hGrad3, 'XData', x3(idx3), 'YData', y3(idx3), 'UData', vx3(idx3), 'VData', vy3(idx3));
    end

    % La Partícula 4 solo inicia si el contador k es mayor al delay
    if k > delay_frames_3
        % Usamos un índice ajustado para que empiece desde el inicio de sus vectores
        idx4 = k - delay_frames_3; 
        
        set(hPath4, 'XData', x4(1:idx4), 'YData', y4(1:idx4));
        set(hPart4, 'XData', x4(idx4), 'YData', y4(idx4));
        set(hGrad4, 'XData', x4(idx4), 'YData', y4(idx4), 'UData', vx4(idx4), 'VData', vy4(idx4));
    end
    
    drawnow;
    pause(0.01);
end


%% Pru 6


% Simulación de Dos Partículas con Inicio Diferido
clear; clc; close all;

% --- Parámetros Partícula 1 ---
R1 = 5; 
omega1 = 1; 
color1 = [0, 0.4470, 0.7410];

% --- Parámetros Partícula 2 ---
R2 = 5; 
omega2 = 1; 
color2 = [0.8500, 0.3250, 0.0980];

% --- Parámetros Partícula 3 ---
R3 = 5; 
omega3 = 1; 
color3 = [0.4940, 0.1840, 0.5560];

% --- Parámetros Partícula 4 ---
R4 = 5; 
omega4 = 1; 
color4 = [0.2, 1, 0];

% --- Tiempo ---
t = linspace(0, 10*pi, 500); 
delay_frames_1 = 25; % Número de pasos que esperará la partícula 2
delay_frames_2 = 50;
delay_frames_3 = 75;

% --- Cálculos Partícula 1 ---
x1 = R1 * cos(omega1 * t);
y1 = R1 * sin(omega1 * t);
vx1 = -R1 * omega1 * sin(omega1 * t);
vy1 = R1 * omega1 * cos(omega1 * t);

% --- Cálculos Partícula 2 ---
x2 = R2 * cos(omega2 * t);
y2 = R2 * sin(omega2 * t);
vx2 = -R2 * omega2 * sin(omega2 * t);
vy2 = R2 * omega2 * cos(omega2 * t);

% --- Cálculos Partícula 3 ---
x3 = R3 * cos(omega3 * t);
y3 = R3 * sin(omega3 * t);
vx3 = -R3 * omega3 * sin(omega3 * t);
vy3 = R3 * omega3 * cos(omega3 * t);

% --- Cálculos Partícula 3 ---
x4 = R4 * cos(omega4 * t);
y4 = R4 * sin(omega4 * t);
vx4 = -R4 * omega4 * sin(omega4 * t);
vy4 = R4 * omega4 * cos(omega4 * t);

% --- Configuración de la Figura ---
figure('Color', 'w'); hold on; axis equal; grid on;
xlabel('Eje X'); ylabel('Eje Y');
title(['Partícula 2, 3 y 4 inicia con un retraso de ', num2str(delay_frames_1), ' frames cada una']);
limit = max(R1, R4) + 2;
xlim([-limit, limit]); ylim([-limit, limit]);

% Órbitas de fondo
plot(R1*cos(linspace(0,2*pi,100)), R1*sin(linspace(0,2*pi,100)), ':', 'Color', [0.8 0.8 0.8]);
plot(R2*cos(linspace(0,2*pi,100)), R2*sin(linspace(0,2*pi,100)), ':', 'Color', [0.8 0.8 0.8]);
plot(R3*cos(linspace(0,2*pi,100)), R3*sin(linspace(0,2*pi,100)), ':', 'Color', [0.8 0.8 0.8]);
plot(R4*cos(linspace(0,2*pi,100)), R4*sin(linspace(0,2*pi,100)), ':', 'Color', [0.8 0.8 0.8]);

% --- Handles Animación ---
hPath1 = plot(nan, nan, 'Color', color1, 'LineWidth', 1.5);
hGrad1 = quiver(nan, nan, nan, nan, 0, 'Color', color1, 'LineWidth', 2);
hPart1 = plot(nan, nan, 'o', 'MarkerFaceColor', color1, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);

hPath2 = plot(nan, nan, 'Color', color2, 'LineWidth', 1.5);
hGrad2 = quiver(nan, nan, nan, nan, 0, 'Color', color2, 'LineWidth', 2);
hPart2 = plot(nan, nan, 'o', 'MarkerFaceColor', color2, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);

hPath3 = plot(nan, nan, 'Color', color3, 'LineWidth', 1.5);
hGrad3 = quiver(nan, nan, nan, nan, 0, 'Color', color3, 'LineWidth', 2);
hPart3 = plot(nan, nan, 'o', 'MarkerFaceColor', color3, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);

hPath4 = plot(nan, nan, 'Color', color4, 'LineWidth', 1.5);
hGrad4 = quiver(nan, nan, nan, nan, 0, 'Color', color4, 'LineWidth', 2);
hPart4 = plot(nan, nan, 'o', 'MarkerFaceColor', color4, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);

% --- Bucle de Animación Modificado ---
% --- Parámetros de la Estela ---
% Define cuántos fotogramas hacia atrás se mostrarán en la estela.
% Experimenta con este valor.
longitud_estela = 25; 

% --- Bucle de Animación Reescrito (Estelas Cometa) ---
for k = 1:length(t);
    
    % --- ACTUALIZACIÓN DE GRÁFICOS (con longitudes finitas) ---

    % 1. Partícula 1 (Azul) - Sale primero, siempre se mueve
    % Usamos max(1, k-longitud_estela) para obtener el índice de inicio
    % de la estela "cometa".
    idx_start1 = max(1, k - longitud_estela);
    
    set(hPath1, 'XData', x1(idx_start1:k), 'YData', y1(idx_start1:k));
    set(hPart1, 'XData', x1(k), 'YData', y1(k));
    set(hGrad1, 'XData', x1(k), 'YData', y1(k), 'UData', vx1(k), 'VData', vy1(k));
    
    % 2. Partícula 2 (Naranja)
    if k > delay_frames_1
        idx2 = k - delay_frames_1; % Índice ajustado para la posición
        
        % Índice de inicio para la estela cometa de la partícula 2
        idx_start2 = max(1, idx2 - longitud_estela);
        
        set(hPath2, 'XData', x2(idx_start2:idx2), 'YData', y2(idx_start2:idx2));
        set(hPart2, 'XData', x2(idx2), 'YData', y2(idx2));
        % Los índices de vy2 estaban mal en tu original, aquí están corregidos
        set(hGrad2, 'XData', x2(idx2), 'YData', y2(idx2), 'UData', vx2(idx2), 'VData', vy2(idx2));
    end

    % 3. Partícula 3 (Morada)
    if k > delay_frames_2
        idx3 = k - delay_frames_2; 
        
        idx_start3 = max(1, idx3 - longitud_estela);
        
        set(hPath3, 'XData', x3(idx_start3:idx3), 'YData', y3(idx_start3:idx3));
        set(hPart3, 'XData', x3(idx3), 'YData', y3(idx3));
        set(hGrad3, 'XData', x3(idx3), 'YData', y3(idx3), 'UData', vx3(idx3), 'VData', vy3(idx3));
    end

    % 4. Partícula 4 (Verde) - Sale última, sigue a las otras
    if k > delay_frames_3
        idx4 = k - delay_frames_3; 
        
        idx_start4 = max(1, idx4 - longitud_estela);
        
        set(hPath4, 'XData', x4(idx_start4:idx4), 'YData', y4(idx_start4:idx4));
        set(hPart4, 'XData', x4(idx4), 'YData', y4(idx4));
        set(hGrad4, 'XData', x4(idx4), 'YData', y4(idx4), 'UData', vx4(idx4), 'VData', vy4(idx4));
    end
    
    drawnow;
    pause(0.01); % Un poco más rápido
end
