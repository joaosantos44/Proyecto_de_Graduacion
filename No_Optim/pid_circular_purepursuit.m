clear all;
% =========================================================================
% MT3005 - PURE PURSUIT CON TRAYECTORIA CIRCULAR
% -------------------------------------------------------------------------
% Control PID con acercamiento exponencial y Pure Pursuit sobre trayectoria
% circular para 4 agentes en formacion.
%
% Logica de lookahead:
%   - Existe un indice global sobre los puntos de la trayectoria circular.
%   - Cuando el lider llega a epsilon del punto actual (ePtol), el indice
%     avanza lookahead_samples posiciones.
%   - Los seguidores usan el mismo mecanismo pero apuntando al punto
%     trasero del agente previo.
%
% Diferencia frente al archivo de acercamiento exponencial:
%   - El punto de seguimiento NO avanza en tiempo real (como en el LQI),
%     sino que salta discretamente (lookahead fijo) igual que el
%     pure pursuit del codigo PID original.

% Los codigos fueron obtenidos de los diversos lavoratoriso de robotica 1
% dados por Miguel Zea y adaptados a un funcionamiento para una trayectoria
% circular
% =========================================================================

%% Parametros del sistema
f = @(xi,u) [u(1)*cos(xi(3)); u(1)*sin(xi(3)); u(2)];

%% Parametros de la simulacion
NV  = 3;
dt  = 0.01;
t0  = 0;
tf  = NV*8;
t   = t0:dt:tf;
K   = length(t) - 1;

%% Trayectoria circular del lider
Ra = 5;
xc = 0; yc = 0;
w  = NV*2*pi/tf;

traj = zeros(2, K+1);
for i = 1:K+1
    tk = t(i);
    traj(1,i) = xc + Ra*cos(w*tk);
    traj(2,i) = yc + Ra*sin(w*tk);
end

%% Inicializacion de agentes
% Agente 1 (lider)
xi0 = [Ra; 0; pi/2];
xi  = xi0; u = [0;0];
XI  = zeros(3, K+1); XI(:,1)  = xi0;
U   = zeros(2, K+1); U(:,1)   = u;

% Agente 2
d1  = 0.5;
xi1 = [Ra-d1; 0; pi/2]; u1 = [0;0];
XI1 = zeros(3, K+1); XI1(:,1) = xi1;
U1  = zeros(2, K+1); U1(:,1)  = u1;

% Agente 3
d2  = 0.5;
xi2 = [Ra-d1-d2; 0; pi/2]; u2 = [0;0];
XI2 = zeros(3, K+1); XI2(:,1) = xi2;
U2  = zeros(2, K+1); U2(:,1)  = u2;

% Agente 4
d3  = 0.5;
xi3 = [Ra-d1-d2-d3; 0; pi/2]; u3 = [0;0];
XI3 = zeros(3, K+1); XI3(:,1) = xi3;
U3  = zeros(2, K+1); U3(:,1)  = u3;

%% Parametros de control
% Velocidad lineal (acercamiento exponencial)
v0    = 2;
alpha = 10;

% Velocidad angular (PID)
kpO = 10;
kiO = 0.0001;
kdO = 0;

% Pure pursuit
lookahead_samples = 20;  % cuantos pasos adelante salta el punto
ePtol = 0.5; % tolerancia para considerar que se llego al punto

% Indices lookahead independientes por agente
li0 = 1; li1 = 1; li2 = 1; li3 = 1;

% Puntos objetivo actuales por agente
xg0 = traj(1,1); yg0 = traj(2,1);  % lider
% Los seguidores usan la posicion trasera del agente previo, no la trayectoria
% directamente, por lo que su "punto objetivo" se actualiza con la posicion
% del lider anterior en cada paso.

G  = zeros(2, K+1);
G(:,1) = [xg0; yg0];

% Estados integradores/derivativos angulares
EO0=0; eO_1_0=0;
EO1=0; eO_1_1=0;
EO2=0; eO_1_2=0;
EO3=0; eO_1_3=0;

%% Ciclo de simulacion
for k = 1:K

    % ============================================================
    % AGENTE 1 (Lider) - Pure pursuit sobre trayectoria circular
    % ============================================================
    x_r=xi(1); y_r=xi(2); theta_r=xi(3);
    e0 = [xg0-x_r; yg0-y_r];
    thetag0 = atan2(e0(2), e0(1));
    eP0_k = norm(e0);
    eO0_k = atan2(sin(thetag0-theta_r), cos(thetag0-theta_r));

    % Velocidad lineal exponencial
    if eP0_k > 1e-6
        v0_ = v0*(1-exp(-alpha*eP0_k^2));  % simplificado: kP*eP = v0*(1-exp(...))
    else
        v0_ = 0;
    end

    % Velocidad angular PID
    eO_D0=eO0_k-eO_1_0; EO0=EO0+eO0_k;
    w0_=kpO*eO0_k+kiO*EO0+kdO*eO_D0; eO_1_0=eO0_k;
    u = [v0_; w0_];

    % Avanzar lookahead del lider
    if eP0_k <= ePtol
        li0 = li0 + lookahead_samples;
        if li0 > K+1; li0 = K+1; end
        xg0 = traj(1, li0);
        yg0 = traj(2, li0);
    end
    G(:, k+1) = [xg0; yg0];

    % ============================================================
    % AGENTE 2 - Pure pursuit hacia punto trasero del lider
    % ============================================================
    x1_=xi1(1); y1_=xi1(2); theta1_=xi1(3);

    % Punto objetivo: posicion trasera del lider
    ref1 = xi(1:2) - d1*[cos(xi(3)); sin(xi(3))];

    e1 = ref1 - [x1_; y1_];
    thetag1 = atan2(e1(2), e1(1));
    eP1_k = norm(e1);
    eO1_k = atan2(sin(thetag1-theta1_), cos(thetag1-theta1_));

    if eP1_k > 1e-6
        v1_ = v0*(1-exp(-alpha*eP1_k^2));
    else
        v1_ = 0;
    end

    eO_D1=eO1_k-eO_1_1; EO1=EO1+eO1_k;
    w1_=kpO*eO1_k+kiO*EO1+kdO*eO_D1; eO_1_1=eO1_k;
    u1 = [v1_; w1_];

    % ============================================================
    % AGENTE 3 - Pure pursuit hacia punto trasero del agente 2
    % ============================================================
    x2_=xi2(1); y2_=xi2(2); theta2_=xi2(3);
    ref2 = xi1(1:2) - d2*[cos(xi1(3)); sin(xi1(3))];

    e2 = ref2 - [x2_; y2_];
    thetag2 = atan2(e2(2), e2(1));
    eP2_k = norm(e2);
    eO2_k = atan2(sin(thetag2-theta2_), cos(thetag2-theta2_));

    if eP2_k > 1e-6
        v2_ = v0*(1-exp(-alpha*eP2_k^2));
    else
        v2_ = 0;
    end

    eO_D2=eO2_k-eO_1_2; EO2=EO2+eO2_k;
    w2_=kpO*eO2_k+kiO*EO2+kdO*eO_D2; eO_1_2=eO2_k;
    u2 = [v2_; w2_];

    % ============================================================
    % AGENTE 4 - Pure pursuit hacia punto trasero del agente 3
    % ============================================================
    x3_=xi3(1); y3_=xi3(2); theta3_=xi3(3);
    ref3 = xi2(1:2) - d3*[cos(xi2(3)); sin(xi2(3))];

    e3 = ref3 - [x3_; y3_];
    thetag3 = atan2(e3(2), e3(1));
    eP3_k = norm(e3);
    eO3_k = atan2(sin(thetag3-theta3_), cos(thetag3-theta3_));

    if eP3_k > 1e-6
        v3_ = v0*(1-exp(-alpha*eP3_k^2));
    else
        v3_ = 0;
    end

    eO_D3=eO3_k-eO_1_3; EO3=EO3+eO3_k;
    w3_=kpO*eO3_k+kiO*EO3+kdO*eO_D3; eO_1_3=eO3_k;
    u3 = [v3_; w3_];

    % ============================================================
    % Integracion RK4
    % ============================================================
    k1=f(xi,u);   k2=f(xi+(dt/2)*k1,u);   k3=f(xi+(dt/2)*k2,u);   k4=f(xi+dt*k3,u);
    xi  = xi  + (dt/6)*(k1+2*k2+2*k3+k4);

    k1=f(xi1,u1); k2=f(xi1+(dt/2)*k1,u1); k3=f(xi1+(dt/2)*k2,u1); k4=f(xi1+dt*k3,u1);
    xi1 = xi1 + (dt/6)*(k1+2*k2+2*k3+k4);

    k1=f(xi2,u2); k2=f(xi2+(dt/2)*k1,u2); k3=f(xi2+(dt/2)*k2,u2); k4=f(xi2+dt*k3,u2);
    xi2 = xi2 + (dt/6)*(k1+2*k2+2*k3+k4);

    k1=f(xi3,u3); k2=f(xi3+(dt/2)*k1,u3); k3=f(xi3+(dt/2)*k2,u3); k4=f(xi3+dt*k3,u3);
    xi3 = xi3 + (dt/6)*(k1+2*k2+2*k3+k4);

    XI(:,k+1)  = xi;   U(:,k+1)  = u;
    XI1(:,k+1) = xi1;  U1(:,k+1) = u1;
    XI2(:,k+1) = xi2;  U2(:,k+1) = u2;
    XI3(:,k+1) = xi3;  U3(:,k+1) = u3;
end

%% Graficas y animacion
figure('WindowState','maximized');

colors_leader    = summer(3);
colors_follower  = spring(3);
colors_follower2 = winter(3);

subplot(6,2,1);
for i=1:3; plot(t,XI(i,:),'Color',colors_leader(i,:),'LineWidth',2); hold on; end
hold off; xlabel('$t$','Interpreter','latex','Fontsize',20);
ylabel('$\mathbf{\xi}(t)$','Interpreter','latex','Fontsize',20); title('Leader');
l=legend('$x$','$y$','$\theta$','Location','southwest'); set(l,'Interpreter','latex','FontSize',16); grid minor;

subplot(6,2,3);
for i=1:3; plot(t,XI1(i,:),'Color',colors_follower(i,:),'LineWidth',1.5); hold on; end
hold off; xlabel('$t$','Interpreter','latex','Fontsize',20);
ylabel('$\mathbf{\xi}(t)$','Interpreter','latex','Fontsize',20); title('Follower 1');
l=legend('$x$','$y$','$\theta$','Location','southwest'); set(l,'Interpreter','latex','FontSize',16); grid minor;

subplot(6,2,5);
for i=1:3; plot(t,XI2(i,:),'Color',colors_follower2(i,:),'LineWidth',1.5); hold on; end
hold off; xlabel('$t$','Interpreter','latex','Fontsize',20);
ylabel('$\mathbf{\xi}(t)$','Interpreter','latex','Fontsize',20); title('Follower 2');
l=legend('$x$','$y$','$\theta$','Location','southwest'); set(l,'Interpreter','latex','FontSize',16); grid minor;

subplot(6,2,7);
for i=1:2; plot(t,U(i,:),'Color',colors_leader(i,:),'LineWidth',2); hold on; end
hold off; xlabel('$t$','Interpreter','latex','Fontsize',20);
ylabel('$\mathbf{v}(t)$','Interpreter','latex','Fontsize',20); title('Leader');
l=legend('$v$','$\omega$','Location','northeast'); set(l,'Interpreter','latex','FontSize',16); grid minor;

subplot(6,2,9);
for i=1:2; plot(t,U1(i,:),'Color',colors_follower(i,:),'LineWidth',1.5); hold on; end
hold off; xlabel('$t$','Interpreter','latex','Fontsize',20);
ylabel('$\mathbf{v}(t)$','Interpreter','latex','Fontsize',20); title('Follower 1');
l=legend('$v$','$\omega$','Location','northeast'); set(l,'Interpreter','latex','FontSize',16); grid minor;

subplot(6,2,11);
for i=1:2; plot(t,U2(i,:),'Color',colors_follower2(i,:),'LineWidth',1.5); hold on; end
hold off; xlabel('$t$','Interpreter','latex','Fontsize',20);
ylabel('$\mathbf{v}(t)$','Interpreter','latex','Fontsize',20); title('Follower 2');
l=legend('$v$','$\omega$','Location','northeast'); set(l,'Interpreter','latex','FontSize',16); grid minor;

subplot(6,2,[2 4 6 8 10 12]);
s = max(max(abs([XI(1:2,:), XI1(1:2,:), XI2(1:2,:), XI3(1:2,:)])));
xlim(s*[-1,1]+[-0.5,0.5]); ylim(s*[-1,1]+[-0.5,0.5]);
grid minor; axis square; hold on;

plot(traj(1,:), traj(2,:), 'k', 'LineWidth', 1.5);
pointplot = scatter(G(1,1), G(2,1), 150, [0.3010 0.7450 0.9330], 'filled', 'h');

q=XI(:,1);  trajplot  = plot(q(1), q(2), '--k', 'LineWidth', 1);
q=XI1(:,1); trajplot1 = plot(q(1), q(2), '--r', 'LineWidth', 1);
q=XI2(:,1); trajplot2 = plot(q(1), q(2), '--b', 'LineWidth', 1);
q=XI3(:,1); trajplot3 = plot(q(1), q(2), '--g', 'LineWidth', 1);

BV = [-0.1,0,0.1; 0,0.3,0];
for ag = 1:4
    switch ag
        case 1; qq=XI(:,1);  clr=[0.5,0.5,0.5];
        case 2; qq=XI1(:,1); clr=[1,0,0];
        case 3; qq=XI2(:,1); clr=[0,0,1];
        case 4; qq=XI3(:,1); clr=[0,1,0];
    end
    IV=[cos(qq(3)-pi/2),-sin(qq(3)-pi/2);sin(qq(3)-pi/2),cos(qq(3)-pi/2)]*BV;
    switch ag
        case 1; bp  = fill(IV(1,:)+qq(1),IV(2,:)+qq(2),clr);
        case 2; bp1 = fill(IV(1,:)+qq(1),IV(2,:)+qq(2),clr);
        case 3; bp2 = fill(IV(1,:)+qq(1),IV(2,:)+qq(2),clr);
        case 4; bp3 = fill(IV(1,:)+qq(1),IV(2,:)+qq(2),clr);
    end
end
xlabel('$x$','Interpreter','latex','Fontsize',20);
ylabel('$y$','Interpreter','latex','Fontsize',20); hold off;

%% Ciclo de animacion
for k = 2:K+1
    q=XI(:,k);  x=q(1); y=q(2); theta=q(3);
    trajplot.XData=[trajplot.XData,x]; trajplot.YData=[trajplot.YData,y];
    IV=[cos(theta-pi/2),-sin(theta-pi/2);sin(theta-pi/2),cos(theta-pi/2)]*BV;
    bp.XData=IV(1,:)+x; bp.YData=IV(2,:)+y;

    q=XI1(:,k); x=q(1); y=q(2); theta=q(3);
    trajplot1.XData=[trajplot1.XData,x]; trajplot1.YData=[trajplot1.YData,y];
    IV=[cos(theta-pi/2),-sin(theta-pi/2);sin(theta-pi/2),cos(theta-pi/2)]*BV;
    bp1.XData=IV(1,:)+x; bp1.YData=IV(2,:)+y;

    q=XI2(:,k); x=q(1); y=q(2); theta=q(3);
    trajplot2.XData=[trajplot2.XData,x]; trajplot2.YData=[trajplot2.YData,y];
    IV=[cos(theta-pi/2),-sin(theta-pi/2);sin(theta-pi/2),cos(theta-pi/2)]*BV;
    bp2.XData=IV(1,:)+x; bp2.YData=IV(2,:)+y;

    q=XI3(:,k); x=q(1); y=q(2); theta=q(3);
    trajplot3.XData=[trajplot3.XData,x]; trajplot3.YData=[trajplot3.YData,y];
    IV=[cos(theta-pi/2),-sin(theta-pi/2);sin(theta-pi/2),cos(theta-pi/2)]*BV;
    bp3.XData=IV(1,:)+x; bp3.YData=IV(2,:)+y;

    pointplot.XData=G(1,k); pointplot.YData=G(2,k);
    pause(dt);
end
