%% Computer Problem Set 5: Inverse Kinematics
% 2-Link Planar Manipulator
% Case 1: start (0.75, 0.75) - run until convergence
% Case 2: start (0.72, 0.72) - run for 20 iterations

clear; clc; close all;

%% Parameters
a1 = 1.0;  a2 = 1.0;
X_desired = [-0.5; 1.5];
tolerance = 0.01;

%% Run IK for both cases
[theta_hist1, error_hist1] = run_IK([0.75; 0.75], X_desired, a1, a2, tolerance, 200);
[theta_hist2, error_hist2] = run_IK([0.72; 0.72], X_desired, a1, a2, tolerance, 20);

%% Print results
fprintf('=== CASE 1: start (0.75, 0.75) ===\n');
fprintf('Converged in %d iterations\n', size(theta_hist1,2)-1);
fprintf('Final theta1=%.5f, theta2=%.5f\n', theta_hist1(1,end), theta_hist1(2,end));
[xf,yf]=fk(theta_hist1(1,end),theta_hist1(2,end),a1,a2);
fprintf('Final EE: (%.5f, %.5f), error=%.6f\n\n', xf, yf, error_hist1(end));

fprintf('=== CASE 2: start (0.72, 0.72) ===\n');
fprintf('Ran for %d iterations\n', size(theta_hist2,2)-1);
fprintf('Final theta1=%.5f, theta2=%.5f\n', theta_hist2(1,end), theta_hist2(2,end));
[xf2,yf2]=fk(theta_hist2(1,end),theta_hist2(2,end),a1,a2);
fprintf('Final EE: (%.5f, %.5f), error=%.6f\n', xf2, yf2, error_hist2(end));

%% ── CASE 1 Figure ──────────────────────────────────────────────────
figure('Name','Case 1','Color','w','Position',[30 80 1350 520]);

subplot(1,3,1);
plot_error(error_hist1, tolerance, 'b', 'Case 1 — Hand Error vs Iteration');

subplot(1,3,2);
plot_joints(theta_hist1, 'Case 1 — Joint Angles vs Iteration');

subplot(1,3,3);
plot_trajectory(theta_hist1, a1, a2, X_desired, 'Case 1 — Workspace Trajectory');

sgtitle('Case 1  |  Start: (\theta_1,\theta_2)=(0.75, 0.75)  →  Convergence',...
        'FontSize',14,'FontWeight','bold');

%% ── CASE 2 Figure ──────────────────────────────────────────────────
figure('Name','Case 2','Color','w','Position',[30 640 1350 520]);

subplot(1,3,1);
plot_error(error_hist2, tolerance, 'm', 'Case 2 — Hand Error (20 iter)');

subplot(1,3,2);
plot_joints(theta_hist2, 'Case 2 — Joint Angles (20 iter)');

subplot(1,3,3);
plot_trajectory(theta_hist2, a1, a2, X_desired, 'Case 2 — Workspace Trajectory');

sgtitle('Case 2  |  Start: (\theta_1,\theta_2)=(0.72, 0.72)  →  20 Iterations',...
        'FontSize',14,'FontWeight','bold');

%% ── ANIMATION: Case 1 ──────────────────────────────────────────────
animate_robot(theta_hist1, error_hist1, a1, a2, X_desired,...
              'Case 1  |  Start (0.75, 0.75)  →  Convergence');

%% ── ANIMATION: Case 2 ──────────────────────────────────────────────
animate_robot(theta_hist2, error_hist2, a1, a2, X_desired,...
              'Case 2  |  Start (0.72, 0.72)  →  20 Iterations');


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
            warning('Singularity at iter %d — stopping.',i); break;
        end
        t1=theta(1); t2=theta(2);
        Jinv = (1/detJ)*[ cos(t1+t2),               sin(t1+t2);
                          -cos(t1)-cos(t1+t2),  -sin(t1)-sin(t1+t2)];
        theta = theta + Jinv*dx;
        theta_hist = [theta_hist, theta];
    end
    % record final state error
    [x,y] = fk(theta(1),theta(2),a1,a2);
    error_hist(end+1) = norm(X_des-[x;y]);
    theta_hist = [theta_hist, theta];  % duplicate last for clean indexing
end

function [x,y] = fk(t1,t2,a1,a2)
    x = a1*cos(t1) + a2*cos(t1+t2);
    y = a1*sin(t1) + a2*sin(t1+t2);
end

% ── static plots ──────────────────────────────────────────────────────

function plot_error(error_hist, tol, col, ttl)
    iters = 0:length(error_hist)-1;
    semilogy(iters, error_hist, '-o','Color',col,'LineWidth',2,...
             'MarkerSize',5,'MarkerFaceColor',col);
    hold on;
    yline(tol,'r--','LineWidth',1.8,'Label','  tol = 0.01','LabelVerticalAlignment','bottom');
    xlabel('Iteration','FontSize',12,'FontWeight','bold');
    ylabel('||error||','FontSize',12,'FontWeight','bold');
    title(ttl,'FontSize',12,'FontWeight','bold');
    grid on; box on; set(gca,'FontSize',11);
end

function plot_joints(theta_hist, ttl)
    iters = 0:size(theta_hist,2)-1;
    plot(iters, theta_hist(1,:),'r-s','LineWidth',2,'MarkerSize',5,'MarkerFaceColor','r'); hold on;
    plot(iters, theta_hist(2,:),'b-^','LineWidth',2,'MarkerSize',5,'MarkerFaceColor','b');
    xlabel('Iteration','FontSize',12,'FontWeight','bold');
    ylabel('Angle (rad)','FontSize',12,'FontWeight','bold');
    title(ttl,'FontSize',12,'FontWeight','bold');
    legend('\theta_1','\theta_2','Location','best','FontSize',11);
    grid on; box on; set(gca,'FontSize',11);
end

function plot_trajectory(theta_hist, a1, a2, X_des, ttl)
    hold on;
    % workspace boundary
    tw = linspace(0,2*pi,300);
    plot((a1+a2)*cos(tw),(a1+a2)*sin(tw),'--','Color',[0.75 0.75 0.75],'LineWidth',1);

    % end-effector path with colour gradient
    n = size(theta_hist,2);
    EE = zeros(2,n);
    for k=1:n
        [EE(1,k),EE(2,k)] = fk(theta_hist(1,k),theta_hist(2,k),a1,a2);
    end
    for k=1:n-1
        c = (k-1)/(max(n-2,1));
        plot(EE(1,k:k+1),EE(2,k:k+1),'-','Color',[c 0.25 1-c],'LineWidth',2.2);
    end

    % final robot pose
    draw_robot_static(theta_hist(:,end),a1,a2);

    % markers
    plot(EE(1,1),EE(2,1),'go','MarkerSize',11,'MarkerFaceColor','g','DisplayName','Start EE');
    plot(X_des(1),X_des(2),'r*','MarkerSize',14,'LineWidth',2,'DisplayName','Goal');

    axis equal; grid on; box on;
    xlim([-2.4 2.4]); ylim([-2.4 2.4]);
    xlabel('X (m)','FontSize',11,'FontWeight','bold');
    ylabel('Y (m)','FontSize',11,'FontWeight','bold');
    title(ttl,'FontSize',12,'FontWeight','bold');
    legend('Workspace','','','Start EE','Goal','Location','best','FontSize',9);
end

function draw_robot_static(theta, a1, a2)
    t1=theta(1); t2=theta(2);
    p0=[0 0];
    p1=[a1*cos(t1), a1*sin(t1)];
    p2=[p1(1)+a2*cos(t1+t2), p1(2)+a2*sin(t1+t2)];
    % links
    line([p0(1) p1(1)],[p0(2) p1(2)],'LineWidth',6,'Color',[0.85 0.7 0.15]);
    line([p0(1) p1(1)],[p0(2) p1(2)],'LineWidth',2,'Color',[1 0.95 0.5]);
    line([p1(1) p2(1)],[p1(2) p2(2)],'LineWidth',5,'Color',[0.2 0.75 0.4]);
    line([p1(1) p2(1)],[p1(2) p2(2)],'LineWidth',2,'Color',[0.6 1 0.7]);
    % joints
    circ(p0(1),p0(2),0.06,[0.2 0.2 0.2]);
    circ(p1(1),p1(2),0.055,[0.2 0.55 1]);
    circ(p2(1),p2(2),0.045,[1 0.38 0.15]);
    % gripper
    gripper_draw(p2(1),p2(2),t1+t2,0.19,0.07);
end

function circ(x,y,r,col)
    th=linspace(0,2*pi,60);
    fill(x+r*cos(th),y+r*sin(th),col,'EdgeColor','w','LineWidth',1.4);
end

function gripper_draw(x,y,ang,gl,gw)
    perp=[cos(ang+pi/2) sin(ang+pi/2)];
    fwd =[cos(ang)      sin(ang)];
    col =[1 0.55 0.05];
    line([x+perp(1)*gw x-perp(1)*gw],[y+perp(2)*gw y-perp(2)*gw],'Color',col,'LineWidth',3);
    line([x+perp(1)*gw x+perp(1)*gw+fwd(1)*gl],[y+perp(2)*gw y+perp(2)*gw+fwd(2)*gl],'Color',col,'LineWidth',3);
    line([x-perp(1)*gw x-perp(1)*gw+fwd(1)*gl],[y-perp(2)*gw y-perp(2)*gw+fwd(2)*gl],'Color',col,'LineWidth',3);
end

% ── animation ─────────────────────────────────────────────────────────

function animate_robot(theta_hist, error_hist, a1, a2, X_des, fig_title)
    fig = figure('Name',fig_title,'Color','k','Position',[500 80 820 820]);
    ax  = axes('Parent',fig,'Color',[0.04 0.04 0.10],...
               'XColor','w','YColor','w','FontSize',11);
    hold(ax,'on'); axis(ax,'equal');
    xlim(ax,[-2.5 2.5]); ylim(ax,[-2.5 2.5]);
    grid(ax,'on'); ax.GridColor=[0.25 0.25 0.3]; ax.GridAlpha=0.6;
    title(ax,fig_title,'Color','w','FontSize',12,'FontWeight','bold');
    xlabel(ax,'X (m)','Color','w','FontSize',11);
    ylabel(ax,'Y (m)','Color','w','FontSize',11);

    % workspace ring
    tw=linspace(0,2*pi,300);
    plot(ax,(a1+a2)*cos(tw),(a1+a2)*sin(tw),'--','Color',[0.35 0.35 0.5],'LineWidth',1);

    % goal
    plot(ax,X_des(1),X_des(2),'r*','MarkerSize',20,'LineWidth',2.5);
    text(ax,X_des(1)+0.09,X_des(2)+0.09,...
         sprintf('Goal\n(%.1f, %.1f)',X_des(1),X_des(2)),...
         'Color',[1 0.4 0.4],'FontSize',10,'FontWeight','bold');

    % animated objects
    traj_h  = plot(ax,NaN,NaN,'-','Color',[0.15 0.75 1],'LineWidth',1.8);
    lnk1_h  = plot(ax,[0 0],[0 0],'LineWidth',9, 'Color',[0.88 0.72 0.10]);
    lnk1b_h = plot(ax,[0 0],[0 0],'LineWidth',3, 'Color',[1.0  0.96 0.55]);
    lnk2_h  = plot(ax,[0 0],[0 0],'LineWidth',7, 'Color',[0.20 0.78 0.42]);
    lnk2b_h = plot(ax,[0 0],[0 0],'LineWidth',3, 'Color',[0.60 1.00 0.70]);
    j0_h = scatter(ax,0,0,220,[0.9 0.9 0.9],'filled','MarkerEdgeColor','k','LineWidth',1.5);
    j1_h = scatter(ax,0,0,170,[0.2 0.55 1.0],'filled','MarkerEdgeColor','w','LineWidth',1.5);
    j2_h = scatter(ax,0,0,140,[1.0 0.38 0.15],'filled','MarkerEdgeColor','w','LineWidth',1.5);
    g0_h = plot(ax,[0 0],[0 0],'Color',[1 0.55 0.05],'LineWidth',3.5);
    g1_h = plot(ax,[0 0],[0 0],'Color',[1 0.55 0.05],'LineWidth',3.5);
    g2_h = plot(ax,[0 0],[0 0],'Color',[1 0.55 0.05],'LineWidth',3.5);

    % info panel
    info_h = text(ax,-2.4, 2.35,'','Color','w','FontSize',10.5,...
                  'FontWeight','bold','VerticalAlignment','top');

    % error bar background
    eb_bg = fill(ax,[0 0 0 0],[0 0 0 0],[0.15 0.15 0.2],...
                 'EdgeColor','none','FaceAlpha',0.7);
    eb_h  = fill(ax,[0 0 0 0],[0 0 0 0],[0.2 0.8 0.3],...
                 'EdgeColor','none','FaceAlpha',0.9);
    text(ax,-2.4,-2.1,'Error','Color',[0.8 0.8 0.8],'FontSize',9);

    n = size(theta_hist,2);
    FRAMES = 45;   % interpolation frames per IK step  ← tune here for speed
    sx=[]; sy=[];

    for k = 1:n-1
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

            % links (double-line for 3-D feel)
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
            % trajectory trace
            set(traj_h,'XData',sx,'YData',sy);
            % error mini-bar (bottom-left, width=1 unit, height proportional)
            cur_err = norm(X_des-[x2;y2]);
            max_err = max(error_hist);
            bar_w=1.2; bx=-2.4; by=-2.35; bar_h=0.55;
            set(eb_bg,'XData',[bx bx+bar_w bx+bar_w bx],...
                      'YData',[by by by+bar_h by+bar_h]);
            fill_w = bar_w * min(cur_err/max_err,1);
            ecol = [min(1,2*cur_err/max_err), min(1,2*(1-cur_err/max_err)), 0.1];
            set(eb_h,'XData',[bx bx+fill_w bx+fill_w bx],...
                     'YData',[by by by+bar_h by+bar_h],'FaceColor',ecol);
            % info text
            set(info_h,'String',...
                sprintf('Iter : %d / %d\n\theta_1 : %.4f rad\n\theta_2 : %.4f rad\nError : %.5f',...
                        k-1, n-2, t1, t2, cur_err));
            drawnow;
            pause(0.018);
        end
    end
    % final title update
    [xf,yf]=fk(theta_hist(1,end),theta_hist(2,end),a1,a2);
    fin_err=norm(X_des-[xf;yf]);
    if fin_err<=0.01
        done_str='✓  CONVERGED';  done_col='cyan';
    else
        done_str='✗  NOT CONVERGED';  done_col=[1 0.6 0.2];
    end
    title(ax,sprintf('%s  —  %s  (error=%.4f)',fig_title,done_str,fin_err),...
          'Color',done_col,'FontSize',11,'FontWeight','bold');
end