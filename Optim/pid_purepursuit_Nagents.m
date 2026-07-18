clear all;
% =========================================================================
% MT3005 - PURE PURSUIT CON N AGENTES (REFACTORIZADO)
% -------------------------------------------------------------------------
% Cambia solo N_agents y d_sep para escalar la formacion.
% El lider usa lookahead discreto sobre la trayectoria circular.
% Los seguidores apuntan al punto trasero del agente anterior.
% =========================================================================

%% ---- PARAMETROS CONFIGURABLES ----
N_agents = 4;
d_sep    = 0.5;

%% Parametros del sistema
f = @(xi,u) [u(1)*cos(xi(3)); u(1)*sin(xi(3)); u(2)];

%% Parametros de simulacion
NV = 10;
dt = 0.01;
t0 = 0;
tf = NV*8;
t  = t0:dt:tf;
K  = length(t) - 1;

%% Trayectoria circular
Ra = 10;
xc = 0; yc = 0;
w  = NV*2*pi/tf;

traj = zeros(2, K+1);
for i = 1:K+1
    traj(1,i) = xc + Ra*cos(w*t(i));
    traj(2,i) = yc + Ra*sin(w*t(i));
end

%% Parametros del controlador
v0    = 2;
alpha = 10;

kpO = 10;
kiO = 0.0001;
kdO = 0;

% Pure pursuit
lookahead_samples = 50;
lookahead_index   = 1;
ePtol             = 0.5;

xg = traj(1,1);
yg = traj(2,1);
G  = zeros(2, K+1);
G(:,1) = [xg; yg];

%% Inicializacion de agentes
agents = struct();
for i = 1:N_agents
    if i == 1
        agents(i).xi = [Ra; 0; pi/2];
    else
        agents(i).xi = [Ra - (i-1)*d_sep; 0; pi/2];
    end
    agents(i).u    = [0; 0];
    agents(i).EO   = 0;
    agents(i).eO_1 = 0;
    agents(i).XI   = zeros(3, K+1);
    agents(i).U    = zeros(2, K+1);
    agents(i).XI(:,1) = agents(i).xi;
    agents(i).U(:,1)  = agents(i).u;
end

%% Ciclo de simulacion
for k = 1:K

    for i = 1:N_agents
        xi_i = agents(i).xi;
        x_ = xi_i(1); y_ = xi_i(2); theta_ = xi_i(3);

        if i == 1
            % Lider: apunta al punto lookahead sobre la trayectoria circular
            e = [xg - x_; yg - y_];
        else
            % Seguidor: apunta al punto trasero del agente anterior
            xi_prev = agents(i-1).xi;
            ref = xi_prev(1:2) - d_sep*[cos(xi_prev(3)); sin(xi_prev(3))];
            e = ref - [x_; y_];
        end

        thetag = atan2(e(2), e(1));
        eP = norm(e);
        eO = atan2(sin(thetag - theta_), cos(thetag - theta_));

        % Velocidad lineal: acercamiento exponencial
        if eP > 1e-6
            v_ = v0 * (1 - exp(-alpha*eP^2));
        else
            v_ = 0;
        end

        % Velocidad angular: PID
        eO_D = eO - agents(i).eO_1;
        agents(i).EO  = agents(i).EO + eO;
        w_ = kpO*eO + kiO*agents(i).EO + kdO*eO_D;
        agents(i).eO_1 = eO;

        agents(i).u = [v_; w_];
    end

    % Avanzar lookahead del lider (salto discreto)
    eP_leader = norm([xg - agents(1).xi(1); yg - agents(1).xi(2)]);
    if eP_leader <= ePtol
        lookahead_index = min(lookahead_index + lookahead_samples, K+1);
        xg = traj(1, lookahead_index);
        yg = traj(2, lookahead_index);
    end
    G(:, k+1) = [xg; yg];

    % Integracion RK4
    for i = 1:N_agents
        xi_i = agents(i).xi;
        u_i  = agents(i).u;
        k1 = f(xi_i, u_i);
        k2 = f(xi_i+(dt/2)*k1, u_i);
        k3 = f(xi_i+(dt/2)*k2, u_i);
        k4 = f(xi_i+dt*k3,     u_i);
        agents(i).xi = xi_i + (dt/6)*(k1+2*k2+2*k3+k4);
        agents(i).XI(:, k+1) = agents(i).xi;
        agents(i).U(:,  k+1) = agents(i).u;
    end
end

%% Graficas y animacion
figure('WindowState','maximized');
n_plots = min(N_agents, 3);
cmaps   = {summer(3), spring(3), winter(3), copper(3), cool(3)};

for i = 1:n_plots
    subplot(n_plots*2, 2, (i-1)*2+1);
    cmap = cmaps{mod(i-1,numel(cmaps))+1};
    for s = 1:3
        plot(t, agents(i).XI(s,:), 'Color', cmap(s,:), 'LineWidth', 1.5); hold on;
    end
    hold off;
    if i==1; title('Leader'); else; title(sprintf('Follower %d', i-1)); end
    xlabel('$t$','Interpreter','latex','Fontsize',16);
    ylabel('$\xi(t)$','Interpreter','latex','Fontsize',16);
    legend('$x$','$y$','$\theta$','Interpreter','latex','FontSize',12,'Location','southwest');
    grid minor;

    subplot(n_plots*2, 2, (i-1)*2+1+n_plots*2);
    for s = 1:2
        plot(t, agents(i).U(s,:), 'Color', cmap(s,:), 'LineWidth', 1.5); hold on;
    end
    hold off;
    if i==1; title('Leader u'); else; title(sprintf('Follower %d u', i-1)); end
    xlabel('$t$','Interpreter','latex','Fontsize',16);
    ylabel('$v(t)$','Interpreter','latex','Fontsize',16);
    legend('$v$','$\omega$','Interpreter','latex','FontSize',12,'Location','northeast');
    grid minor;
end

subplot(n_plots*2, 2, [2, 4, 6, 2+n_plots*2, 4+n_plots*2, 6+n_plots*2]);
all_pos = cell2mat(arrayfun(@(a) a.XI(1:2,:), agents, 'UniformOutput', false));
s_ax = max(max(abs(all_pos)));
xlim(s_ax*[-1,1]+[-0.5,0.5]); ylim(s_ax*[-1,1]+[-0.5,0.5]);
grid minor; axis square; hold on;

plot(traj(1,:), traj(2,:), 'k', 'LineWidth', 1.5);
pointplot = scatter(G(1,1), G(2,1), 150, [0.3010 0.7450 0.9330], 'filled', 'h');

colors_anim = lines(N_agents);
BV = [-0.1,0,0.1; 0,0.3,0];
trajplots = gobjects(N_agents,1);
bodyplots = gobjects(N_agents,1);

for i = 1:N_agents
    q = agents(i).XI(:,1);
    trajplots(i) = plot(q(1), q(2), '--', 'Color', colors_anim(i,:), 'LineWidth', 1);
    IV = [cos(q(3)-pi/2),-sin(q(3)-pi/2); sin(q(3)-pi/2),cos(q(3)-pi/2)] * BV;
    bodyplots(i) = fill(IV(1,:)+q(1), IV(2,:)+q(2), colors_anim(i,:));
end
xlabel('$x$','Interpreter','latex','Fontsize',20);
ylabel('$y$','Interpreter','latex','Fontsize',20);
leg_labels = ["Leader", arrayfun(@(i) sprintf("Follower %d",i), 1:N_agents-1, 'UniformOutput',false)];
legend(trajplots, leg_labels, 'Location','northeast');
hold off;

%% Ciclo de animacion
for k = 2:K+1
    for i = 1:N_agents
        q = agents(i).XI(:,k);
        trajplots(i).XData = [trajplots(i).XData, q(1)];
        trajplots(i).YData = [trajplots(i).YData, q(2)];
        IV = [cos(q(3)-pi/2),-sin(q(3)-pi/2); sin(q(3)-pi/2),cos(q(3)-pi/2)] * BV;
        bodyplots(i).XData = IV(1,:) + q(1);
        bodyplots(i).YData = IV(2,:) + q(2);
    end
    pointplot.XData = G(1,k);
    pointplot.YData = G(2,k);
    pause(dt);
end
