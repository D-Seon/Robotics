%% Computer Problem Set 5: Inverse Kinematics — Problems II
% 2-Link Planar Manipulator (a1=a2=1)
% 8 desired hand positions solved via Newton-Raphson (Jacobian IK)
% Results:
%   1. Solve IK for all 8 points
%   2. Plot 8 desired positions
%   3. Animate robot moving through all 8 targets sequentially

clear; clc; close all;

%% ── Robot & solver parameters ────────────────────────────────────────
a1 = 1.0;  a2 = 1.0;
tolerance = 0.01;
max_iter  = 300;

%% ── 8 Points: [x_des, y_des, theta1_0, theta2_0] ────────────────────
points = [
    1.2,  0.8,   0.5,  0.5;
    0.5,  1.5,   0.8,  0.6;
   -0.8,  1.2,   1.8,  0.5;
   -1.5,  0.5,   2.2,  0.4;
   -1.2, -0.8,  -2.5,  0.5;
   -0.5, -1.5,  -2.0,  0.6;
    0.8, -1.2,  -0.8,  0.5;
    1.5, -0.5,  -0.4,  0.4;
];

n_pts = size(points, 1);

%% ── 1. Solve IK for all 8 points ────────────────────────────────────
fprintf('========== IK Solutions (Newton-Raphson) ==========\n');
fprintf('%-6s %-14s %-14s %-14s %-14s %-10s %-8s\n',...
        'Point','x_des','y_des','theta1','theta2','error','iters');
fprintf('%s\n', repmat('-',1,75));

results = zeros(n_pts, 5);  % [theta1, theta2, error, iters, converged]
all_theta = cell(n_pts,1);
all_error = cell(n_pts,1);

for p = 1:n_pts
    X_des  = points(p,1:2)';
    theta0 = points(p,3:4)';
    [th_hist, err_hist] = run_IK(theta0, X_des, a1, a2, tolerance, max_iter);
    all_theta{p} = th_hist;
    all_error{p} = err_hist;
    tf = th_hist(:,end);
    ef = err_hist(end);
    conv = ef <= tolerance;
    results(p,:) = [tf(1), tf(2), ef, length(err_hist)-1, conv];
    fprintf('  P%-4d (%-5.1f,%-5.1f)  theta1=%7.4f  theta2=%7.4f  err=%.5f  %3d iter  %s\n',...
            p, points(p,1), points(p,2), tf(1), tf(2), ef, length(err_hist)-1,...
            choose(conv,'CONV','DIVG'));
end

%% ── 2. Static figure: 8 desired positions + convergence summary ──────
figure('Name','Problems II — Overview','Color','w','Position',[30 50 1380 560]);

% subplot 1: workspace + 8 target points
subplot(1,3,1);
hold on;
tw = linspace(0,2*pi,400);
fill((a1+a2)*cos(tw),(a1+a2)*sin(tw),[0.94 0.97 1],'EdgeColor',[0.7 0.7 0.9],'LineWidth',1.2);
fill(abs(a1-a2)*cos(tw),abs(a1-a2)*sin(tw),[1 1 1],'EdgeColor',[0.8 0.8 0.8],'LineWidth',1);

colors8 = lines(n_pts);
for p=1:n_pts
    xd=points(p,1); yd=points(p,2);
    scatter(xd,yd,120,colors8(p,:),'filled','MarkerEdgeColor','k','LineWidth',1.2);
    text(xd+0.07,yd+0.07,sprintf('P%d',p),'FontSize',10,'FontWeight','bold','Color',colors8(p,:));
end
axis equal; grid on; box on;
xlim([-2.4 2.4]); ylim([-2.4 2.4]);
xlabel('X (m)','FontSize',11,'FontWeight','bold');
ylabel('Y (m)','FontSize',11,'FontWeight','bold');
title('8 Desired Hand Positions','FontSize',12,'FontWeight','bold');
legend('Workspace','Unreachable','Location','southeast','FontSize',9);

% subplot 2: error convergence for all 8 points
subplot(1,3,2);
hold on;
for p=1:n_pts
    n_e = length(all_error{p});
    semilogy(0:n_e-1, all_error{p}, '-o','Color',colors8(p,:),'LineWidth',1.8,...
             'MarkerSize',4,'MarkerFaceColor',colors8(p,:),'DisplayName',sprintf('P%d',p));
end
yline(tolerance,'k--','LineWidth',2,'Label','  tol=0.01','LabelVerticalAlignment','bottom');
xlabel('Iteration','FontSize',11,'FontWeight','bold');
ylabel('||error||','FontSize',11,'FontWeight','bold');
title('Hand Error vs Iteration (All Points)','FontSize',12,'FontWeight','bold');
legend('Location','northeast','FontSize',9);
grid on; box on; set(gca,'FontSize',10);

% subplot 3: final joint angles bar chart
subplot(1,3,3);
b = bar(1:n_pts, [results(:,1), results(:,2)], 'grouped');
b(1).FaceColor = [0.85 0.33 0.10];
b(2).FaceColor = [0.0  0.45 0.74];
xlabel('Point #','FontSize',11,'FontWeight','bold');
ylabel('Joint Angle (rad)','FontSize',11,'FontWeight','bold');
title('Final Joint Angles \theta_1, \theta_2','FontSize',12,'FontWeight','bold');
legend('\theta_1','\theta_2','Location','best','FontSize',10);
grid on; box on; set(gca,'FontSize',10);
xticks(1:n_pts);

sgtitle('Problems II — 2-Link Planar Manipulator IK (Newton-Raphson)',...
        'FontSize',14,'FontWeight','bold');

%% ── 3. Animation: robot visits all 8 target points sequentially ──────
animate_all_points(all_theta, points, a1, a2, colors8);


%% ====================================================================
%%  FUNCTIONS
%% ====================================================================

function [theta_hist, error_hist] = run_IK(theta0, X_des, a1, a2, tol, max_iter)
    theta      = theta0;
    theta_hist = theta;
    error_hist = [];
    for i = 1:max_iter
        [x,y]  = fk(theta(1),theta(2),a1,a2);
        dx     = X_des - [x;y];
        err    = norm(dx);
        error_hist(end+1) = err;
        if err <= tol; break; end
        detJ = sin(theta(2));
        if abs(detJ) < 1e-6
            warning('Singularity P iter=%d',i); break;
        end
        t1=theta(1); t2=theta(2);   % 자코비안 역행렬
        Jinv = (1/detJ)*[cos(t1+t2), sin(t1+t2);
                         -cos(t1)-cos(t1+t2), -sin(t1)-sin(t1+t2)];
        theta = theta + Jinv*dx;
        theta_hist = [theta_hist, theta];
    end
    [x,y]=fk(theta(1),theta(2),a1,a2);
    error_hist(end+1) = norm(X_des-[x;y]);
    theta_hist = [theta_hist, theta];
end

function [x,y] = fk(t1,t2,a1,a2)    % 순기구학
    x = a1*cos(t1) + a2*cos(t1+t2);
    y = a1*sin(t1) + a2*sin(t1+t2);
end

function s = choose(cond, a, b)
    if cond; s=a; else; s=b; end
end

function animate_all_points(all_theta, points, a1, a2, colors8)
    n_pts = size(points,1);
    FRAMES = 40;   % interpolation frames per IK step (higher = slower)

    fig = figure('Name','Problems II — Robot Animation',...
                 'Color','k','Position',[480 60 900 900]);
    ax  = axes('Parent',fig,'Color',[0.04 0.04 0.10],...
               'XColor','w','YColor','w','FontSize',11);
    hold(ax,'on'); axis(ax,'equal');
    xlim(ax,[-2.5 2.5]); ylim(ax,[-2.5 2.5]);
    grid(ax,'on'); ax.GridColor=[0.22 0.22 0.28]; ax.GridAlpha=0.55;
    xlabel(ax,'X (m)','Color','w','FontSize',12);
    ylabel(ax,'Y (m)','Color','w','FontSize',12);

    % workspace boundary
    tw=linspace(0,2*pi,300);
    plot(ax,(a1+a2)*cos(tw),(a1+a2)*sin(tw),'--','Color',[0.35 0.35 0.55],'LineWidth',1.2);

    % draw all 8 target points (static)
    h_targets = gobjects(n_pts,1);
    for p=1:n_pts
        h_targets(p) = scatter(ax,points(p,1),points(p,2),160,...
                               colors8(p,:),'filled',...
                               'MarkerEdgeColor','w','LineWidth',1.2);
        text(ax,points(p,1)+0.09,points(p,2)+0.09,sprintf('P%d',p),...
             'Color',colors8(p,:),'FontSize',10.5,'FontWeight','bold');
    end

    % trajectory trace line (full path, drawn progressively)
    traj_h  = plot(ax,NaN,NaN,'-','Color',[0.55 0.55 0.55],'LineWidth',1.2);

    % robot graphic handles
    lnk1_h  = plot(ax,[0 0],[0 0],'LineWidth',10,'Color',[0.88 0.72 0.10]);
    lnk1b_h = plot(ax,[0 0],[0 0],'LineWidth', 3,'Color',[1.00 0.96 0.55]);
    lnk2_h  = plot(ax,[0 0],[0 0],'LineWidth', 8,'Color',[0.20 0.78 0.42]);
    lnk2b_h = plot(ax,[0 0],[0 0],'LineWidth', 3,'Color',[0.60 1.00 0.70]);
    j0_h = scatter(ax,0,0,250,[0.90 0.90 0.90],'filled','MarkerEdgeColor','k','LineWidth',1.8);
    j1_h = scatter(ax,0,0,190,[0.20 0.55 1.00],'filled','MarkerEdgeColor','w','LineWidth',1.6);
    j2_h = scatter(ax,0,0,160,[1.00 0.38 0.15],'filled','MarkerEdgeColor','w','LineWidth',1.6);
    g0_h = plot(ax,[0 0],[0 0],'Color',[1 0.55 0.05],'LineWidth',3.5);
    g1_h = plot(ax,[0 0],[0 0],'Color',[1 0.55 0.05],'LineWidth',3.5);
    g2_h = plot(ax,[0 0],[0 0],'Color',[1 0.55 0.05],'LineWidth',3.5);

    % active target highlight ring
    ring_h = scatter(ax,NaN,NaN,350,'w','LineWidth',2.5);

    % info text
    info_h  = text(ax,-2.42, 2.40,'','Color','w','FontSize',10.5,...
                   'FontWeight','bold','VerticalAlignment','top');
    title_h = title(ax,'','Color','w','FontSize',12,'FontWeight','bold');

    sx=[]; sy=[];   % accumulated EE trajectory

    for p = 1:n_pts
        theta_hist = all_theta{p};
        n_steps    = size(theta_hist,2);
        X_des      = points(p,1:2);
        pt_col     = colors8(p,:);

        % highlight current target
        set(ring_h,'XData',X_des(1),'YData',X_des(2),...
                   'MarkerEdgeColor',pt_col,'SizeData',360);
        % colour-code links for this target
        set(lnk1_h,'Color', pt_col*0.85 + [0.1 0.1 0.1]);
        set(lnk2_h,'Color', min(pt_col*0.6 + [0.3 0.4 0.3],1));

        for k = 1:n_steps-1
            t1s=theta_hist(1,k); t2s=theta_hist(2,k);
            t1e=theta_hist(1,k+1); t2e=theta_hist(2,k+1);

            for f = 1:FRAMES
                % cosine ease-in/ease-out
                alpha = (1 - cos(pi*(f-1)/(FRAMES-1))) / 2;
                t1 = t1s + alpha*(t1e-t1s);
                t2 = t2s + alpha*(t2e-t2s);

                x1=a1*cos(t1);        y1=a1*sin(t1);
                x2=x1+a2*cos(t1+t2); y2=y1+a2*sin(t1+t2);

                sx(end+1)=x2; sy(end+1)=y2;

                % links
                set(lnk1_h, 'XData',[0 x1],'YData',[0 y1]);
                set(lnk1b_h,'XData',[0 x1],'YData',[0 y1]);
                set(lnk2_h, 'XData',[x1 x2],'YData',[y1 y2]);
                set(lnk2b_h,'XData',[x1 x2],'YData',[y1 y2]);
                % joints
                set(j0_h,'XData',0, 'YData',0);
                set(j1_h,'XData',x1,'YData',y1);
                set(j2_h,'XData',x2,'YData',y2);
                % gripper
                ang=t1+t2; gl=0.21; gw=0.08;
                perp=[cos(ang+pi/2) sin(ang+pi/2)];
                fwd =[cos(ang)      sin(ang)];
                set(g0_h,'XData',[x2+perp(1)*gw x2-perp(1)*gw],...
                         'YData',[y2+perp(2)*gw y2-perp(2)*gw]);
                set(g1_h,'XData',[x2+perp(1)*gw x2+perp(1)*gw+fwd(1)*gl],...
                         'YData',[y2+perp(2)*gw y2+perp(2)*gw+fwd(2)*gl]);
                set(g2_h,'XData',[x2-perp(1)*gw x2-perp(1)*gw+fwd(1)*gl],...
                         'YData',[y2-perp(2)*gw y2-perp(2)*gw+fwd(2)*gl]);
                % EE trajectory (colour changes per point)
                set(traj_h,'XData',sx,'YData',sy,'Color',[pt_col 0.55]);
                % info
                cur_err=norm(X_des'-[x2;y2]);
                set(info_h,'String',...
                    sprintf('Point : %d / %d\nIter  : %d / %d\n\theta_1  : %6.4f rad\n\theta_2  : %6.4f rad\nError : %.5f',...
                            p, n_pts, k-1, n_steps-2, t1, t2, cur_err));
                set(title_h,'String',...
                    sprintf('Moving to Point %d  —  Target (%.1f, %.1f)',p,X_des(1),X_des(2)),...
                    'Color',pt_col);
                drawnow;
                pause(0.016);
            end
        end

        % brief pause at each target + flash ring
        for fl=1:6
            set(ring_h,'SizeData',300+100*mod(fl,2));
            drawnow; pause(0.08);
        end
    end

    set(title_h,'String','✓  All 8 Points Reached!','Color','cyan');
    set(info_h,'String','Done — all targets visited.','Color','cyan');
end