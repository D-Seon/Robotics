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
              'Case 1  |  Start (0.75, 0.75)  →  Convergence', tolerance);

%% ── ANIMATION: Case 2 ──────────────────────────────────────────────
animate_robot(theta_hist2, error_hist2, a1, a2, X_desired,...
              'Case 2  |  Start (0.72, 0.72)  →  20 Iterations', tolerance);

%% ====================================================================
%%  FUNCTIONS
%% ====================================================================

function [theta_hist, error_hist] = run_IK(theta0, X_des, a1, a2, tol, max_iter)
    % -----------------------------------------------------------------
    % 역기구학 연산 함수 (Newton-Raphson 수치 해석)
    % -----------------------------------------------------------------
    theta      = theta0;
    theta_hist = theta;
    error_hist = [];
    for i = 1:max_iter
        % 순기구학 계산 및 오차 도출
        [x,y]  = fk(theta(1),theta(2),a1,a2);
        dx     = X_des - [x;y];
        err    = norm(dx);
        error_hist(end+1) = err;
        
        if err <= tol; break; end
        
        % 자코비안 행렬식 검사
        detJ = sin(theta(2));
        if abs(detJ) < 1e-6
            warning('Singularity at iter %d — stopping.',i); break;
        end
        
        % 역자코비안을 통한 관절각 변위 산출 및 업데이트 (θ_new = θ_old + J^-1 * dX)
        t1=theta(1); t2=theta(2);
        Jinv = (1/detJ)*[ cos(t1+t2),               sin(t1+t2);
                          -cos(t1)-cos(t1+t2),  -sin(t1)-sin(t1+t2)];
        theta = theta + Jinv*dx;
        theta_hist = [theta_hist, theta];
    end
    [x,y] = fk(theta(1),theta(2),a1,a2);
    error_hist(end+1) = norm(X_des-[x;y]);
    theta_hist = [theta_hist, theta];  
end

function [x,y] = fk(t1,t2,a1,a2)
    % -----------------------------------------------------------------
    % 순기구학 공식 (Forward Kinematics)
    % -----------------------------------------------------------------
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
    tw = linspace(0,2*pi,300);
    plot((a1+a2)*cos(tw),(a1+a2)*sin(tw),'--','Color',[0.75 0.75 0.75],'LineWidth',1);
    n = size(theta_hist,2);
    EE = zeros(2,n);
    for k=1:n
        [EE(1,k),EE(2,k)] = fk(theta_hist(1,k),theta_hist(2,k),a1,a2);
    end
    for k=1:n-1
        c = (k-1)/(max(n-2,1));
        plot(EE(1,k:k+1),EE(2,k:k+1),'-','Color',[c 0.25 1-c],'LineWidth',2.2);
    end
    draw_robot_static(theta_hist(:,end),a1,a2);
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
    line([p0(1) p1(1)],[p0(2) p1(2)],'LineWidth',6,'Color',[0.85 0.7 0.15]);
    line([p0(1) p1(1)],[p0(2) p1(2)],'LineWidth',2,'Color',[1 0.95 0.5]);
    line([p1(1) p2(1)],[p1(2) p2(2)],'LineWidth',5,'Color',[0.2 0.75 0.4]);
    line([p1(1) p2(1)],[p1(2) p2(2)],'LineWidth',2,'Color',[0.6 1 0.7]);
    circ(p0(1),p0(2),0.06,[0.2 0.2 0.2]);
    circ(p1(1),p1(2),0.055,[0.2 0.55 1]);
    circ(p2(1),p2(2),0.045,[1 0.38 0.15]);
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

% ── 애니메이션 함수 업그레이드 (실시간 에러 선그래프 임베딩) ──────────────────
function animate_robot(theta_hist, error_hist, a1, a2, X_des, fig_title, tol)
    % 2개의 서브플롯을 배치할 수 있도록 창 가로 해상도를 1300으로 확대
    fig = figure('Name',fig_title,'Color','k','Position',[300 80 1300 700]);
    
    % [좌측 subplot] 로봇 기하학 애니메이션 뷰컨트롤
    ax_robot = subplot(1, 2, 1, 'Color', [0.04 0.04 0.10], 'XColor', 'w', 'YColor', 'w', 'FontSize', 11);
    hold(ax_robot, 'on'); axis(ax_robot, 'equal');
    xlim(ax_robot, [-2.5 2.5]); ylim(ax_robot, [-2.5 2.5]);
    grid(ax_robot, 'on'); ax_robot.GridColor = [0.25 0.25 0.3]; ax_robot.GridAlpha = 0.6;
    title(ax_robot, 'Robot Workspace Motion', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel(ax_robot, 'X (m)'); ylabel(ax_robot, 'Y (m)');
    
    % 작업공간 경계선 및 목푯값 세팅
    tw = linspace(0, 2*pi, 300);
    plot(ax_robot, (a1+a2)*cos(tw), (a1+a2)*sin(tw), '--', 'Color', [0.35 0.35 0.5], 'LineWidth', 1);
    plot(ax_robot, X_des(1), X_des(2), 'r*', 'MarkerSize', 20, 'LineWidth', 2.5);
    text(ax_robot, X_des(1)+0.09, X_des(2)+0.09, 'Goal', 'Color', [1 0.4 0.4], 'FontWeight', 'bold');
    
    % 로봇 그래픽 핸들들 생성
    traj_h  = plot(ax_robot, NaN, NaN, '-', 'Color', [0.15 0.75 1], 'LineWidth', 1.8);
    lnk1_h  = plot(ax_robot, [0 0], [0 0], 'LineWidth', 9, 'Color', [0.88 0.72 0.10]);
    lnk1b_h = plot(ax_robot, [0 0], [0 0], 'LineWidth', 3, 'Color', [1.0  0.96 0.55]);
    lnk2_h  = plot(ax_robot, [0 0], [0 0], 'LineWidth', 7, 'Color', [0.20 0.78 0.42]);
    lnk2b_h = plot(ax_robot, [0 0], [0 0], 'LineWidth', 3, 'Color', [0.60 1.00 0.70]);
    j0_h = scatter(ax_robot, 0, 0, 220, [0.9 0.9 0.9], 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    j1_h = scatter(ax_robot, 0, 0, 170, [0.2 0.55 1.0], 'filled', 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
    j2_h = scatter(ax_robot, 0, 0, 140, [1.0 0.38 0.15], 'filled', 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
    g0_h = plot(ax_robot, [0 0], [0 0], 'Color', [1 0.55 0.05], 'LineWidth', 3.5);
    g1_h = plot(ax_robot, [0 0], [0 0], 'Color', [1 0.55 0.05], 'LineWidth', 3.5);
    g2_h = plot(ax_robot, [0 0], [0 0], 'Color', [1 0.55 0.05], 'LineWidth', 3.5);
    info_h = text(ax_robot, -2.4, 2.35, '', 'Color', 'w', 'FontSize', 10.5, 'FontWeight', 'bold', 'VerticalAlignment', 'top');

    % [우측 subplot] 실시간 에러 수렴 선그래프 화면 구성
    ax_err = subplot(1, 2, 2, 'Color', [0.07 0.07 0.12], 'XColor', 'w', 'YColor', 'w', 'FontSize', 11);
    hold(ax_err, 'on');
    grid(ax_err, 'on'); ax_err.GridColor = [0.3 0.3 0.3];
    xlabel(ax_err, 'Iteration', 'FontWeight', 'bold');
    ylabel(ax_err, '||Error|| (Log Scale)', 'FontWeight', 'bold');
    title(ax_err, 'Real-time Error Convergence', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
    
    % 에러 한계선(yline) 설정 및 동적 선그래프 객체 생성
    yline(ax_err, tol, 'r--', 'LineWidth', 1.5);
    text(ax_err, 0.2, tol*1.2, 'tol = 0.01', 'Color', 'r', 'FontSize', 9);
    live_err_h = plot(ax_err, NaN, NaN, '-o', 'Color', [0.2 0.8 1], 'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', [0.2 0.8 1]);
    
    % 전역 윈도우 타이틀 상단 설정
    sgtitle(fig_title, 'Color', 'w', 'FontSize', 14, 'FontWeight', 'bold');
    
    % 애니메이션 파라미터 제어
    n = size(theta_hist,2);
    FRAMES = 35; % 보간 스텝 수 (속도를 높이려면 수치를 내리세요)
    sx = []; sy = [];
    animated_iters = [];
    animated_errors = [];

    for k = 1:n-1
        t1s = theta_hist(1,k);   t2s = theta_hist(2,k);
        t1e = theta_hist(1,k+1); t2e = theta_hist(2,k+1);
        
        for f = 1:FRAMES
            alpha = (1 - cos(pi*(f-1)/(FRAMES-1))) / 2; % 부드러운 가감속 효과
            t1 = t1s + alpha*(t1e-t1s);
            t2 = t2s + alpha*(t2e-t2s);
            
            x1 = a1*cos(t1);        y1 = a1*sin(t1);
            x2 = x1 + a2*cos(t1+t2); y2 = y1 + a2*sin(t1+t2);
            sx(end+1) = x2; sy(end+1) = y2;
            
            % 그래픽 요소 링크 좌표 업데이트
            set(lnk1_h, 'XData', [0 x1], 'YData', [0 y1]);
            set(lnk1b_h, 'XData', [0 x1], 'YData', [0 y1]);
            set(lnk2_h, 'XData', [x1 x2], 'YData', [y1 y2]);
            set(lnk2b_h, 'XData', [x1 x2], 'YData', [y1 y2]);
            set(j1_h, 'XData', x1, 'YData', y1);
            set(j2_h, 'XData', x2, 'YData', y2);
            
            % 그리퍼 연산 및 업데이트
            ang = t1+t2; gl = 0.21; gw = 0.08;
            perp = [cos(ang+pi/2) sin(ang+pi/2)];
            fwd  = [cos(ang)      sin(ang)];
            set(g0_h, 'XData', [x2+perp(1)*gw x2-perp(1)*gw], 'YData', [y2+perp(2)*gw y2-perp(2)*gw]);
            set(g1_h, 'XData', [x2+perp(1)*gw x2+perp(1)*gw+fwd(1)*gl], 'YData', [y2+perp(2)*gw y2+perp(2)*gw+fwd(2)*gl]);
            set(g2_h, 'XData', [x2-perp(1)*gw x2-perp(1)*gw+fwd(1)*gl], 'YData', [y2-perp(2)*gw y2-perp(2)*gw+fwd(2)*gl]);
            set(traj_h, 'XData', sx, 'YData', sy);
            
            % 실시간 텍스트 정보 수정
            cur_err = norm(X_des-[x2;y2]);
            set(info_h, 'String', sprintf('Iter : %d / %d\n\\theta_1 : %.4f rad\n\\theta_2 : %.4f rad\nError : %.5f', k-1, n-2, t1, t2, cur_err));
            
            % 스텝이 바뀔 때 우측 창에 에러 선그래프 동적 생성 반영
            if f == FRAMES
                animated_iters(end+1) = k-1;
                animated_errors(end+1) = error_hist(k);
                set(live_err_h, 'XData', animated_iters, 'YData', animated_errors);
                
                % 오차가 로그 스케일로 예쁘게 정렬되도록 축 동적제어
                if length(animated_iters) > 1
                    set(ax_err, 'YScale', 'log');
                    xlim(ax_err, [0 max(n-2, 5)]);
                    ylim(ax_err, [0.001 max(error_hist)*1.5]);
                end
            end
            
            drawnow;
            pause(0.005); % 쾌적한 애니메이션 출력을 위한 미세 딜레이 조정
        end
    end
    
    % 마지막 최종 스텝 보정값 추가 반영
    animated_iters(end+1) = n-1;
    animated_errors(end+1) = error_hist(end);
    set(live_err_h, 'XData', animated_iters, 'YData', animated_errors);
    
    % 수렴 유무 타이틀 가시성 변경 반영
    [xf,yf] = fk(theta_hist(1,end), theta_hist(2,end), a1, a2);
    fin_err = norm(X_des-[xf;yf]);
    if fin_err <= tol
        sgtitle(sprintf('%s  —  ✓ CONVERGED (Error=%.5f)', fig_title, fin_err), 'Color', 'cyan', 'FontSize', 14, 'FontWeight', 'bold');
    else
        sgtitle(sprintf('%s  —  ✗ NOT CONVERGED (Error=%.5f)', fig_title, fin_err), 'Color', [1 0.5 0.2], 'FontSize', 14, 'FontWeight', 'bold');
    end
end