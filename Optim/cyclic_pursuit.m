clear all;
% =========================================================================
% MT3005 - PERSECUCION CICLICA ORBITAL
% =========================================================================
% Cada agente persigue un punto que esta phi_offset radianes adelante
% de su objetivo en el circulo de radio Ra.
% Con phi_offset = 0  -> convergencia pura al centro
% Con phi_offset > 0  -> orbita estable (cada agente sigue un punto
%                        adelante de su objetivo sobre el circulo)
% =========================================================================

%% ---- PARAMETROS CONFIGURABLES ----
N_agents = 8;      % numero de agentes
Ra = 2;      % m — radio de la orbita deseada

if N_agents <= 5         % rad — cuanto adelanta el punto objetivo en el circulo
    phi_offset = pi/4;   % pi/4 = 45 grados, pi/2 = 90 grados
else                     % mas grande = orbita mas estable, menos convergencia
    phi_offset = pi/12;
end     

NV = 15;
tf = NV * 8;

%% Parametros del sistema
f = @(xi,u) [u(1)*cos(xi(3)); u(1)*sin(xi(3)); u(2)];

dt = 0.01;
t = 0:dt:tf;
K = length(t) - 1;

%% Parametros del controlador
v0 = 2;
alpha = 10;
kpO = 10;
kiO = 0.0001;
kdO = 0;

%% Inicializacion: agentes equiespaciados en el circulo
w_max = 2;   % rad/s — saturacion de velocidad angular (protege hardware)

agents = struct();
for i = 1:N_agents
    ang_i  = 2*pi*(i-1)/N_agents;
    agents(i).xi       = [Ra*cos(ang_i); Ra*sin(ang_i); ang_i + pi/2];
    agents(i).u        = [0; 0];
    agents(i).EO       = 0;
    agents(i).eO_1     = 0;
    agents(i).XI       = zeros(3, K+1);
    agents(i).U        = zeros(2, K+1);
    agents(i).XI(:,1)  = agents(i).xi;
    agents(i).U(:,1)   = agents(i).u;
end

%% Ciclo de simulacion
for k = 1:K

    for i = 1:N_agents
        xi_i   = agents(i).xi;
        x_     = xi_i(1);
        y_     = xi_i(2);
        theta_ = xi_i(3);

        % Posicion del siguiente agente
        i_next  = mod(i, N_agents) + 1;
        xi_next = agents(i_next).xi;
        xn = xi_next(1);
        yn = xi_next(2);

        % Angulo actual del siguiente agente sobre el circulo
        % atan2 es estable aqui porque solo usa el angulo instantaneo,
        % no lo acumula. El salto de 2pi no afecta porque cos/sin
        % son periodicos: cos(ang+2pi) = cos(ang)
        ang_next = atan2(yn, xn);

        % Punto objetivo: phi_offset radianes adelante en el circulo
        % No hay salto porque el resultado pasa por cos/sin directamente
        ang_target = ang_next + phi_offset;
        xg = Ra * cos(ang_target);
        yg = Ra * sin(ang_target);

        % Error
        e      = [xg - x_; yg - y_];
        thetag = atan2(e(2), e(1));
        eP     = norm(e);
        eO     = atan2(sin(thetag - theta_), cos(thetag - theta_));

        % Control lineal
        if eP > 1e-6
            v_ = v0 * (1 - exp(-alpha * eP^2));
        else
            v_ = 0;
        end

        % Control angular PID
        eO_D = eO - agents(i).eO_1;
        agents(i).EO  = agents(i).EO + eO;
        w_ = kpO*eO + kiO*agents(i).EO + kdO*eO_D;
        agents(i).eO_1 = eO;

        % Saturacion de velocidad angular (evita picos en hardware)
        w_ = max(-w_max, min(w_max, w_));

        agents(i).u = [v_; w_];
    end

    % RK4
    for i = 1:N_agents
        xi_i = agents(i).xi;
        u_i  = agents(i).u;
        k1 = f(xi_i, u_i);
        k2 = f(xi_i+(dt/2)*k1, u_i);
        k3 = f(xi_i+(dt/2)*k2, u_i);
        k4 = f(xi_i+dt*k3,     u_i);
        agents(i).xi        = xi_i + (dt/6)*(k1+2*k2+2*k3+k4);
        agents(i).XI(:,k+1) = agents(i).xi;
        agents(i).U(:,k+1)  = agents(i).u;
    end
end

%% =========================================================
% FIGURA 1 — Animacion de trayectorias
% =========================================================
colors = lines(N_agents);
BV     = [-0.05, 0, 0.05; 0, 0.15, 0];

fig1 = figure('Name','Trayectorias','WindowState','maximized');
all_pos = cell2mat(arrayfun(@(a) a.XI(1:2,:), agents, 'UniformOutput', false));
s_ax = max(max(abs(all_pos))) * 1.15;
xlim([-s_ax, s_ax]); ylim([-s_ax, s_ax]);
axis square; grid minor; hold on;

th_ref = linspace(0, 2*pi, 200);
plot(Ra*cos(th_ref), Ra*sin(th_ref), '--', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5);
scatter(0, 0, 100, 'k', '+', 'LineWidth', 2);

for i = 1:N_agents
    ang_i = 2*pi*(i-1)/N_agents;
    scatter(Ra*cos(ang_i), Ra*sin(ang_i), 100, colors(i,:), 'o', 'LineWidth', 2.5);
end

trajplots = gobjects(N_agents,1);
bodyplots = gobjects(N_agents,1);
leg_labels = arrayfun(@(i) sprintf('Agente %d',i), 1:N_agents, 'UniformOutput',false);

for i = 1:N_agents
    q = agents(i).XI(:,1);
    trajplots(i) = plot(q(1), q(2), '-', 'Color', colors(i,:), 'LineWidth', 1.5);
    IV = [cos(q(3)-pi/2),-sin(q(3)-pi/2); sin(q(3)-pi/2),cos(q(3)-pi/2)] * BV;
    bodyplots(i) = fill(IV(1,:)+q(1), IV(2,:)+q(2), colors(i,:), 'EdgeColor','none');
end

legend(trajplots, leg_labels, 'Location','northeast','FontSize',13);
xlabel('$x$ (m)','Interpreter','latex','Fontsize',18);
ylabel('$y$ (m)','Interpreter','latex','Fontsize',18);
title(sprintf('Persecucion ciclica — %d agentes, Ra=%.1f m, \\phi=%.2f rad', ...
    N_agents, Ra, phi_offset), 'Fontsize', 14);
hold off;

%% =========================================================
% FIGURA 2 — Estados xi(t) de cada agente (subplots separados)
% =========================================================
state_labels = {'$x$ (m)', '$y$ (m)', '$\theta$ (rad)'};
state_names  = {'x', 'y', '\theta'};

fig2 = figure('Name','Estados','WindowState','maximized');
for i = 1:N_agents
    subplot(N_agents, 1, i);
    lh = gobjects(3,1);
    styles = {'-', '--', ':'};
    for s = 1:3
        lh(s) = plot(t, agents(i).XI(s,:), ...
            styles{s}, 'Color', colors(i,:), 'LineWidth', 1.8);
        hold on;
    end
    hold off;
    ylabel('$\xi(t)$','Interpreter','latex','Fontsize',14);
    title(sprintf('Agente %d', i), 'Fontsize', 13);
    legend(lh, {'$x$','$y$','$\theta$'}, ...
        'Interpreter','latex','FontSize',12,'Location','best','Orientation','horizontal');
    grid minor;
    if i == N_agents
        xlabel('$t$ (s)','Interpreter','latex','Fontsize',14);
    end
end
sgtitle('Estados \xi(t) por agente', 'Interpreter','latex','Fontsize',16);

%% =========================================================
% FIGURA 3 — Entradas u(t): v y omega separados por agente
% =========================================================
fig3 = figure('Name','Entradas','WindowState','maximized');

% Subplot superior: velocidad lineal v de todos los agentes
subplot(2,1,1);
for i = 1:N_agents
    plot(t, agents(i).U(1,:), '-', 'Color', colors(i,:), 'LineWidth', 1.8);
    hold on;
end
hold off;
ylabel('$v$ (m/s)','Interpreter','latex','Fontsize',16);
title('Velocidad lineal $v(t)$','Interpreter','latex','Fontsize',14);
legend(leg_labels, 'Location','best','FontSize',12,'Orientation','horizontal');
grid minor;

% Subplot inferior: velocidad angular omega de todos los agentes
subplot(2,1,2);
for i = 1:N_agents
    plot(t, agents(i).U(2,:), '-', 'Color', colors(i,:), 'LineWidth', 1.8);
    hold on;
end
hold off;
ylabel('$\omega$ (rad/s)','Interpreter','latex','Fontsize',16);
xlabel('$t$ (s)','Interpreter','latex','Fontsize',16);
title('Velocidad angular $\omega(t)$','Interpreter','latex','Fontsize',14);
legend(leg_labels, 'Location','best','FontSize',12,'Orientation','horizontal');
grid minor;

sgtitle('Entradas $u(t)$ por agente', 'Interpreter','latex','Fontsize',16);

%% =========================================================
% ANIMACION en Figura 1
% =========================================================
figure(fig1);
for k = 2:K+1
    for i = 1:N_agents
        q = agents(i).XI(:,k);
        trajplots(i).XData = [trajplots(i).XData, q(1)];
        trajplots(i).YData = [trajplots(i).YData, q(2)];
        IV = [cos(q(3)-pi/2),-sin(q(3)-pi/2); sin(q(3)-pi/2),cos(q(3)-pi/2)] * BV;
        bodyplots(i).XData = IV(1,:)+q(1);
        bodyplots(i).YData = IV(2,:)+q(2);
    end
    pause(dt);
end