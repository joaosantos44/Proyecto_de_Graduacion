clear all;
% Codigo de LQI y LQR de MIguel para robotica en el cual se realizaron 
% modificaciones para la persecucion ciclica

%% Parámetros del sistema y definición del sistema dinámico 
% xi = [x; y; theta]  y  u = [v; omega]
% Campo vectorial del sistema dinámico
f = @(xi,u) [u(1)*cos(xi(3)); u(1)*sin(xi(3)); u(2)];

% Difeomorfismo para linealización
ell = 0.1; % en m
finv = @(xi,mu) [1,0; 0,1/ell] * [cos(xi(3)), -sin(xi(3)); sin(xi(3)), cos(xi(3))]' * [mu(1); mu(2)];

% Matrices del sistema LTI equivalente
A = zeros(2); 
B = eye(2); 

%% Parámetros de la simulación
NV = 10; % Numero de Vueltas
dt = 0.01; % período de muestreo
t0 = 0; % tiempo inicial
tf = NV*8; % tiempo final 
% Cambiar el tf para que se vea mas fluido el seguimiento
% Relacion a 1:8 con el numero de vueltas

% K = (tf-t0)/dt; % número de iteraciones
% t = t0:dt:tf; % vector de tiempo (para trayectoria y figuras)
% Se cambia el numero de iteraciones para adecuarlas a la trayectoria de
% circulo
t = t0:dt:tf;
K = length(t) - 1;


%% Inicialización y condiciones iniciales

% Agente 1
xi0 = [0; 0; 0];
u0 = [0; 0];
xi = xi0; % vector de estado 
u = u0; % vector de entradas
% Arrays para almacenar las trayectorias de las variables de estado,
% entradas y salidas del sistema
XI = zeros(numel(xi), K+1);
U = zeros(numel(u), K+1);
% Inicialización de arrays
XI(:, 1) = xi0;
U(:, 1) = u0;


% Agente 2 en este caso el que sigue al agente 1
xi1 = [-0.5; 0; 0];   % condición inicial distinta
u1 = [0; 0];
XI1 = zeros(numel(xi1), K+1);
U1  = zeros(numel(u1), K+1);
XI1(:,1) = xi1;
U1(:,1)  = u1;

% Agente 3 en este caso el que sigue al agente 2
xi2 = [-1; 0; 0];   % condición inicial distinta
u2 = [0; 0];
XI2 = zeros(numel(xi2), K+1);
U2  = zeros(numel(u2), K+1);
XI2(:,1) = xi2;
U2(:,1)  = u2;

% Agente 4 en este caso el que sigue al agente 3
xi3 = [-1.5; 0; 0];   % condición inicial distinta
u3 = [0; 0];
XI3 = zeros(numel(xi3), K+1);
U3  = zeros(numel(u3), K+1);
XI3(:,1) = xi3;
U3(:,1)  = u3;

%% Definicion de trayectoria circular
Ra = 23;
% Con la configuracion actual 22 es el maximo despues de eso el sistema
% muere porque? no se =(
xc = 0;
yc = 0;
w = NV*2*pi/tf;
% Relacion a 1:8 con el numero de vueltas
% Cambiar el valor que multiplica a pi para modificar el numero de vueltas

traj = zeros(2, K+1);

for i = 1:K+1
    tk = t(i);
    traj(1,i) = xc + Ra*cos(w*tk);
    traj(2,i) = yc + Ra*sin(w*tk);
end

xg = traj(1,1);
yg = traj(2,1);

G = zeros(2, K+1);
G(:,1) = [xg; yg];

%% Inicialización de variables para controladores
% LQR 
Q = eye(2);
R = 10*eye(2);
Klqr = lqr(A,B,Q,R);

% LQI
Cr = eye(2);
Dr = zeros(2);

AA = [A, zeros(size(Cr')); 
Cr, zeros(size(Cr,1))];
BB = [B; Dr];

QQ = eye(size(A,1) + size(Cr,1)); 
QQ(3,3) = 500; 
QQ(4,4) = 500; 

Klqi = lqr(AA, BB, QQ, R);

ref = [xg; yg];

sigma = 0;
sigma1 = 0;
sigma2 = 0;
sigma3 = 0;


% Pure pursuit
% lookahead_samples = 50;
% lookahead_index = 1;
% ePtol = 0.5;


%% Ciclo de simulación
for k = 1:K
 % LQI trayectoria
    x = xi(1:2);
    xg = traj(1, k+1);
    yg = traj(2, k+1);
    ref = [xg; yg];
    sigma = sigma + (Cr*x - ref)*dt;
    mu = -Klqi*[x; sigma];
    u = finv(xi, mu);

    % Agente 2
    x1 = xi1(1:2);

    % referencia = posición del líder
    d = 0.5; % distancia deseada

    ref1 = xi(1:2) - d*[cos(xi(3)); sin(xi(3))];
    sigma1 = sigma1 + (Cr*x1 - ref1)*dt;
    mu1 = -Klqi*[x1; sigma1];

    % dirección hacia el punto objetivo
    dir = ref1 - x1;
    % proyectar dirección
    mu1 = mu1 + 0.001 * dir;
    u1 = finv(xi1, mu1);

    % Agente 3
    x2 = xi2(1:2);

    % referencia = posición del líder
    d2 = 1; % distancia deseada

    ref2 = xi1(1:2) - d2*[cos(xi1(3)); sin(xi1(3))];
    sigma2 = sigma2 + (Cr*x2 - ref2)*dt;
    mu2 = -Klqi*[x2; sigma2];

    % dirección hacia el punto objetivo
    dir2 = ref2 - x2;
    % proyectar dirección
    mu2 = mu2 + 0.001 * dir2;
    u2 = finv(xi2, mu2);

    % Agente 4
    x3 = xi3(1:2);

    % referencia = posición del líder
    d3 = 1.5; % distancia deseada

    ref3 = xi2(1:2) - d3*[cos(xi2(3)); sin(xi2(3))];
    sigma3 = sigma3 + (Cr*x3 - ref3)*dt;
    mu3 = -Klqi*[x3; sigma3];

    % dirección hacia el punto objetivo
    dir3 = ref3 - x3;
    % proyectar dirección
    mu3 = mu3 + 0.001 * dir3;
    u3 = finv(xi3, mu3);


    % Se actualiza la trayectoria del punto que hala al robot
    G(:, k+1) = [xg; yg];
    

    % Se actualiza el estado del sistema mediante una discretización por 
    % el método de Runge-Kutta (RK4)
    k1 = f(xi, u);
    k2 = f(xi+(dt/2)*k1, u);
    k3 = f(xi+(dt/2)*k2, u);
    k4 = f(xi+dt*k3, u);
    xi = xi + (dt/6)*(k1+2*k2+2*k3+k4);

    % Actualizacion del agente 2
    k1_1 = f(xi1, u1);
    k2_1 = f(xi1+(dt/2)*k1_1, u1);
    k3_1 = f(xi1+(dt/2)*k2_1, u1);
    k4_1 = f(xi1+dt*k3_1, u1);
    xi1 = xi1 + (dt/6)*(k1_1+2*k2_1+2*k3_1+k4_1);

    % Actualizacion del agente 3
    k1_2 = f(xi2, u2);
    k2_2 = f(xi2+(dt/2)*k1_2, u2);
    k3_2 = f(xi2+(dt/2)*k2_2, u2);
    k4_2 = f(xi2+dt*k3_2, u2);
    xi2 = xi2 + (dt/6)*(k1_2+2*k2_2+2*k3_2+k4_2);

    % Actualizacion del agente 3
    k1_3 = f(xi3, u3);
    k2_3 = f(xi3+(dt/2)*k1_3, u3);
    k3_3 = f(xi3+(dt/2)*k2_3, u3);
    k4_3 = f(xi3+dt*k3_3, u3);
    xi3 = xi3 + (dt/6)*(k1_3+2*k2_3+2*k3_3+k4_3);
    
    % Se guardan las trayectorias del estado y las entradas
    XI(:, k+1) = xi;
    U(:, k+1) = u;

    % Se guardan las trayectorias del estado y las entradas Agente 2
    XI1(:, k+1) = xi1;
    U1(:, k+1)  = u1;

    % Se guardan las trayectorias del estado y las entradas Agente 3
    XI2(:, k+1) = xi2;
    U2(:, k+1)  = u2;

    % Se guardan las trayectorias del estado y las entradas Agente 4
    XI3(:, k+1) = xi3;
    U3(:, k+1)  = u3;

end

%% Animación y generación de figuras (NO modificar)
figure('WindowState', 'maximized');

colors_leader = winter(3);
colors_follower = autumn(3);
colors_follower2 = summer(3);


subplot(6,2,1);
for i = 1:3
    plot(t, XI(i,:), 'Color', colors_leader(i,:), 'LineWidth', 2); hold on;
end
hold off;
xlabel('$t$', 'Interpreter', 'latex', 'Fontsize', 20);
ylabel('$\mathbf{\xi}(t)$', 'Interpreter', 'latex', 'Fontsize', 20);
title('Leader')
l = legend('$x$', '$y$', '$\theta$', 'Location', 'southwest', 'Orientation', 'vertical');
set(l, 'Interpreter', 'latex', 'FontSize', 16);
grid minor;

subplot(6,2,3);
for i = 1:3
    plot(t, XI1(i,:), 'Color', colors_follower(i,:), 'LineWidth', 1.5); hold on;
end
hold off;
xlabel('$t$', 'Interpreter', 'latex', 'Fontsize', 20);
ylabel('$\mathbf{\xi}(t)$', 'Interpreter', 'latex', 'Fontsize', 20);
title('Follower')
l = legend('$x$', '$y$', '$\theta$', 'Location', 'southwest', 'Orientation', 'vertical');
set(l, 'Interpreter', 'latex', 'FontSize', 16);
grid minor;

subplot(6,2,5);
for i = 1:3
    plot(t, XI2(i,:), 'Color', colors_follower2(i,:), 'LineWidth', 1.5); hold on;
end
hold off;
xlabel('$t$', 'Interpreter', 'latex', 'Fontsize', 20);
ylabel('$\mathbf{\xi}(t)$', 'Interpreter', 'latex', 'Fontsize', 20);
title('Follower 2')
l = legend('$x$', '$y$', '$\theta$', 'Location', 'southwest', 'Orientation', 'vertical');
set(l, 'Interpreter', 'latex', 'FontSize', 16);
grid minor;

subplot(6,2,7);
for i = 1:2
    plot(t, U(i,:), 'Color', colors_leader(i,:), 'LineWidth', 2); hold on;
end
hold off;
xlabel('$t$', 'Interpreter', 'latex', 'Fontsize', 20);
ylabel('$\mathbf{v}(t)$', 'Interpreter', 'latex', 'Fontsize', 20);
title('Leader')
l = legend('$v$', '$\omega$', 'Location', 'northeast', 'Orientation', 'vertical');
set(l, 'Interpreter', 'latex', 'FontSize', 16);
grid minor;

subplot(6,2,9);
for i = 1:2
    plot(t, U1(i,:), 'Color', colors_follower(i,:), 'LineWidth', 1.5); hold on;
end
hold off;
xlabel('$t$', 'Interpreter', 'latex', 'Fontsize', 20);
ylabel('$\mathbf{v}(t)$', 'Interpreter', 'latex', 'Fontsize', 20);
title('Follower')
l = legend('$v$', '$\omega$', 'Location', 'northeast', 'Orientation', 'vertical');
set(l, 'Interpreter', 'latex', 'FontSize', 16);
grid minor;

subplot(6,2,11);
for i = 1:2
    plot(t, U2(i,:), 'Color', colors_follower2(i,:), 'LineWidth', 1.5); hold on;
end
hold off;
xlabel('$t$', 'Interpreter', 'latex', 'Fontsize', 20);
ylabel('$\mathbf{v}(t)$', 'Interpreter', 'latex', 'Fontsize', 20);
title('Follower 2')
l = legend('$v$', '$\omega$', 'Location', 'northeast', 'Orientation', 'vertical');
set(l, 'Interpreter', 'latex', 'FontSize', 16);
grid minor;

subplot(6,2,[2 4 6 8 10 12]);
s = max(max(abs(XI3(1:2,:))));
xlim(s*[-1, 1]+[-0.5, 0.5]);
ylim(s*[-1, 1]+[-0.5, 0.5]);
grid minor;
axis square;
hold on;

q = XI(:,1);
x = q(1); 
y = q(2); 
theta = q(3);

% Animacion del Agente 2
q1 = XI1(:,1);
x1 = q1(1); 
y1 = q1(2); 
theta1 = q1(3);

% Animacion del Agente 3
q2 = XI2(:,1);
x2 = q2(1); 
y2 = q2(2); 
theta2 = q2(3);

% Animacion del Agente 4
q3 = XI3(:,1);
x3 = q3(1); 
y3 = q3(2); 
theta3 = q3(3);

plot(traj(1,:), traj(2,:), 'k', 'LineWidth', 1.5);
pointplot = scatter(G(1,1), G(2,1), 150, [0.3010 0.7450 0.9330], 'filled', 'h');

trajplot = plot(x, y, '--k', 'LineWidth', 1);

% Animacion del Agente 2 trayectoria
trajplot1 = plot(xi1(1), xi1(2), '--r', 'LineWidth', 1);

% Animacion del Agente 3 trayectoria
trajplot2 = plot(xi2(1), xi2(2), '--b', 'LineWidth', 1);

% Animacion del Agente 4 trayectoria
trajplot3 = plot(xi3(1), xi3(2), '--g', 'LineWidth', 1);


BV = [-0.1, 0, 0.1; 0, 0.3, 0];

IV = [cos(theta-pi/2), -sin(theta-pi/2); sin(theta-pi/2), cos(theta-pi/2)] * BV;
bodyplot = fill(IV(1,:) + x, IV(2,:) + y, [0.5,0.5,0.5]);

% Animacion del Agente 2 flecha
IV1 = [cos(theta1-pi/2), -sin(theta1-pi/2); sin(theta1-pi/2), cos(theta1-pi/2)] * BV;
bodyplot1 = fill(IV1(1,:) + x1, IV1(2,:) + y1, [1,0,0]);

% Animacion del Agente 3 flecha
IV2 = [cos(theta2-pi/2), -sin(theta2-pi/2); sin(theta2-pi/2), cos(theta2-pi/2)] * BV;
bodyplot2 = fill(IV2(1,:) + x2, IV2(2,:) + y2, [0,0,1]);

% Animacion del Agente 4 flecha
IV3 = [cos(theta3-pi/2), -sin(theta3-pi/2); sin(theta3-pi/2), cos(theta3-pi/2)] * BV;
bodyplot3 = fill(IV3(1,:) + x3, IV3(2,:) + y3, [0,1,0]);

xlabel('$x$', 'Interpreter', 'latex', 'Fontsize', 20);
ylabel('$y$', 'Interpreter', 'latex', 'Fontsize', 20);
hold off;

%% Ciclo de animación
for k = 2:K+1
    q = XI(:,k);
    x = q(1); 
    y = q(2); 
    theta = q(3);
    
    % Parametros del Agente 2
    q1 = XI1(:,k);
    x1 = q1(1); 
    y1 = q1(2); 
    theta1 = q1(3);

    % Parametros del Agente 3
    q2 = XI2(:,k);
    x2 = q2(1); 
    y2 = q2(2); 
    theta2 = q2(3);

    % Parametros del Agente 3
    q3 = XI3(:,k);
    x3 = q3(1); 
    y3 = q3(2); 
    theta3 = q3(3);
    
    trajplot.XData = [trajplot.XData, x];
    trajplot.YData = [trajplot.YData, y];

    % ploteo del Agente 2
    trajplot1.XData = [trajplot1.XData, x1];
    trajplot1.YData = [trajplot1.YData, y1];

    % ploteo del Agente 3
    trajplot2.XData = [trajplot2.XData, x2];
    trajplot2.YData = [trajplot2.YData, y2];

    % ploteo del Agente 4
    trajplot3.XData = [trajplot3.XData, x3];
    trajplot3.YData = [trajplot3.YData, y3];
    
    % Punto de seguimiento
    pointplot.XData = G(1, k);
    pointplot.YData = G(2, k);

    BV = [-0.1, 0, 0.1; 0, 0.3, 0];
    IV = [cos(theta-pi/2), -sin(theta-pi/2); sin(theta-pi/2), cos(theta-pi/2)] * BV;
    bodyplot.XData = IV(1,:) + x;
    bodyplot.YData = IV(2,:) + y;

    % ploteo del Agente 2 Flecha
    IV1 = [cos(theta1-pi/2), -sin(theta1-pi/2); sin(theta1-pi/2), cos(theta1-pi/2)] * BV;
    bodyplot1.XData = IV1(1,:) + x1;
    bodyplot1.YData = IV1(2,:) + y1;

    % ploteo del Agente 3 Flecha
    IV2 = [cos(theta2-pi/2), -sin(theta2-pi/2); sin(theta2-pi/2), cos(theta2-pi/2)] * BV;
    bodyplot2.XData = IV2(1,:) + x2;
    bodyplot2.YData = IV2(2,:) + y2;

    % ploteo del Agente 4 Flecha
    IV3 = [cos(theta3-pi/2), -sin(theta3-pi/2); sin(theta3-pi/2), cos(theta3-pi/2)] * BV;
    bodyplot3.XData = IV3(1,:) + x3;
    bodyplot3.YData = IV3(2,:) + y3;
  
    pause(dt);
end