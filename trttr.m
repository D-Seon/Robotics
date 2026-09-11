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
    [x,y] = fk(theta(1),theta(2),a1,a2);
    error_hist(end+1) = norm(X_des-[x;y]);
    theta_hist = [theta_hist, theta];  
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
    phi = t1 + t2;
    
    gold_colors = {[0.25, 0.18, 0.05], [0.70, 0.52, 0.15], [0.88, 0.71, 0.28], [0.98, 0.88, 0.55], [1.0, 0.98, 0.85]};
    widths = [16, 12, 8, 4, 1.5];
    for k = 1:5
        line([p0(1) p1(1)],[p0(2) p1(2)],'Color',gold_colors{k},'LineWidth',widths(k));
        line([p1(1) p2(1)],[p1(2) p2(2)],'Color',gold_colors{k},'LineWidth',widths(k));
    end
    
    circ(p0(1),p0(2),0.05,[0.2 0.2 0.2]);
    circ(p1(1),p1(2),0.045,[0.9, 0.75, 0.20]);
    circ(p2(1),p2(2),0.035,[0.0, 0.9, 1.0]);
    
    [w_x, w_y, c_x, c_y, p_x, p_y] = get_gripper_geometry_data();
    draw_patch_part(w_x, w_y, p2(1), p2(2), phi, [0.30, 0.25, 0.15], [0.1, 0.1, 0.1]);
    draw_patch_part(c_x, c_y, p2(1), p2(2), phi, [0.85, 0.75, 0.45], [0.45, 0.38, 0.2]);
    draw_patch_part(c_x, -c_y, p2(1), p2(2), phi, [0.85, 0.75, 0.45], [0.45, 0.38, 0.2]);
    draw_patch_part(p_x, p_y, p2(1), p2(2), phi, [0.0, 0.9, 1.0], 'none');
    draw_patch_part(p_x, -p_y, p2(1), p2(2), phi, [0.0, 0.9, 1.0], 'none');
end

function draw_patch_part(lx, ly, ex, ey, ephi, fcol, ecol)
    R = [cos(ephi), -sin(ephi); sin(ephi), cos(ephi)];
    g_coords = R * [lx; ly] + [ex; ey];
    patch('XData', g_coords(1,:), 'YData', g_coords(2,:), 'FaceColor', fcol, 'EdgeColor', ecol);
end

function circ(x,y,r,col)
    th=linspace(0,2*pi,60);
    fill(x+r*cos(th),y+r*sin(th),col,'EdgeColor','w','LineWidth',1);
end

function [wrist_x, wrist_y, claw_base_x, claw_base_y, pad_base_x, pad_base_y] = get_gripper_geometry_data()
    wrist_x = [-0.03,  0.03,  0.03, -0.03]; wrist_y = [-0.05, -0.05,  0.05,  0.05];
    claw_base_x = [0.03, 0.11, 0.19, 0.23, 0.20, 0.12, 0.03]; claw_base_y = [0.05, 0.09, 0.07, 0.02, -0.01, 0.03, 0.01];
    pad_base_x = [0.15, 0.23, 0.20]; pad_base_y = [0.07, 0.02, -0.01];
end

% ── 애니메이션 함수 업그레이드 (가로 2분할 실시간 에러 그래프 임베딩) ──────────────────
function animate_robot(theta_hist, error_hist, a1, a2, X_des, fig_title, tol)
    % 듀얼 배치를 위해 창 가로 해상도를 1350으로 가로 확장 세팅
    fig = figure('Name',fig_title,'Color',[0.08, 0.09, 0.11],'Position',[300 80 1350 700]);
    
    % [좌측 subplot] 로봇 기하학 시뮬레이터 뷰 영역
    ax_robot = subplot(1, 2, 1, 'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], 'FontSize', 11);
    hold(ax_robot, 'on'); axis(ax_robot, 'equal');
    xlim(ax_robot, [-2.5 2.5]); ylim(ax_robot, [-2.5 2.5]);
    grid(ax_robot, 'on'); ax_robot.GridColor = [0.25, 0.28, 0.35]; ax_robot.GridAlpha = 0.4;
    title(ax_robot, 'SCREEN 01: Robot Workspace Motion', 'Color', [0.9, 0.95, 1.0], 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Segoe UI');
    xlabel(ax_robot, 'X (m)'); ylabel(ax_robot, 'Y (m)');
    
    % 가동 영역 및 산업용 베이스 받침대 드로잉 고정
    tw = linspace(0, 2*pi, 300);
    plot(ax_robot, (a1+a2)*cos(tw), (a1+a2)*sin(tw), '--', 'Color', [0.35 0.35 0.5], 'LineWidth', 1);
    base_plate_x = [-0.45, 0.45, 0.30, -0.30]; base_plate_y = [-0.60, -0.60, -0.15, -0.15];
    base_neck_x = [-0.15, 0.15, 0.10, -0.10];  base_neck_y = [-0.15, -0.15, 0.0, 0.0];
    fill(ax_robot, base_plate_x, base_plate_y, [0.18, 0.20, 0.23], 'EdgeColor', [0.1, 0.11, 0.12], 'LineWidth', 2);
    fill(ax_robot, base_neck_x, base_neck_y, [0.55, 0.58, 0.62], 'EdgeColor', [0.3, 0.32, 0.35], 'LineWidth', 1.5);
    rectangle(ax_robot, 'Position', [-0.18, -0.18, 0.36, 0.36], 'Curvature', [1, 1], 'FaceColor', [0.35, 0.38, 0.42], 'LineWidth', 1.5);
    
    % 목표 표적점 마킹
    plot(ax_robot, X_des(1), X_des(2), 'r*', 'MarkerSize', 20, 'LineWidth', 2.5);
    text(ax_robot, X_des(1)+0.09, X_des(2)+0.09, 'Goal', 'Color', [1 0.4 0.4], 'FontWeight', 'bold');
    
    % 로봇 핸들 객체 선언
    traj_h  = plot(ax_robot, NaN, NaN, '-', 'Color', [1.0, 0.2, 0.6], 'LineWidth', 2);
    gold_colors = {[0.25, 0.18, 0.05], [0.70, 0.52, 0.15], [0.88, 0.71, 0.28], [0.98, 0.88, 0.55], [1.0, 0.98, 0.85]};
    widths = [26, 21, 14, 7, 2.5];
    lnk1_layers = []; lnk2_layers = [];
    for k = 1:5
        lnk1_layers(k) = plot(ax_robot, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
        lnk2_layers(k) = plot(ax_robot, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
    end
    
    h_wrist  = patch(ax_robot, 'XData', [], 'YData', [], 'FaceColor', [0.30, 0.25, 0.15], 'EdgeColor', [0.1, 0.1, 0.1]);
    h_claw_L = patch(ax_robot, 'XData', [], 'YData', [], 'FaceColor', [0.85, 0.75, 0.45], 'EdgeColor', [0.45, 0.38, 0.2]);
    h_claw_R = patch(ax_robot, 'XData', [], 'YData', [], 'FaceColor', [0.85, 0.75, 0.45], 'EdgeColor', [0.45, 0.38, 0.2]);
    h_pad_L  = patch(ax_robot, 'XData', [], 'YData', [], 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none'); 
    h_pad_R  = patch(ax_robot, 'XData', [], 'YData', [], 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');
    
    scatter(ax_robot, 0, 0, 220, [0.2 0.2 0.2], 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    j2_cap = scatter(ax_robot, 0, 0, 170, [0.9, 0.75, 0.20], 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    wrist_cap = scatter(ax_robot, 0, 0, 110, [0.0, 0.9, 1.0], 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    info_h = text(ax_robot, -2.4, 2.35, '', 'Color', 'w', 'FontSize', 10.5, 'FontWeight', 'bold', 'VerticalAlignment', 'top', 'FontName', 'Courier New');

    % [우측 subplot] 실시간 오차 수렴 곡선 모니터 뷰 영역
    ax_err = subplot(1, 2, 2, 'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], 'FontSize', 11);
    hold(ax_err, 'on'); grid(ax_err, 'on'); ax_err.GridColor = [0.25, 0.28, 0.35];
    xlabel(ax_err, 'Iteration step', 'FontWeight', 'bold');
    ylabel(ax_err, '||Error|| (Log Scale)', 'FontWeight', 'bold');
    title(ax_err, 'SCREEN 02: Real-time Error Convergence Trace', 'Color', [0.9, 0.95, 1.0], 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Segoe UI');
    
    % 수렴 한계점 가이드 가로 점선 설정 및 동적 플롯 핸들 확보
    yline(ax_err, tol, 'r--', 'LineWidth', 1.5);
    text(ax_err, 0.5, tol*1.3, 'tol = 0.01', 'Color', 'r', 'FontSize', 9, 'FontWeight', 'bold');
    live_err_h = plot(ax_err, NaN, NaN, '-o', 'Color', [0.2 0.8 1], 'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', [0.2 0.8 1]);
    
    % 전역 메인 헤드 타이틀 지정
    sgtitle(fig_title, 'Color', 'w', 'FontSize', 14, 'FontWeight', 'bold');
    
    % 루프 시뮬레이션 데이터 준비
    [w_x, w_y, c_x, c_y, p_x, p_y] = get_gripper_geometry_data();
    n = size(theta_hist,2);
    FRAMES = 35;   
    sx = []; sy = [];
    live_iters = [];
    live_errors = [];

    for k = 1:n-1
        t1s = theta_hist(1,k);   t2s = theta_hist(2,k);
        t1e = theta_hist(1,k+1); t2e = theta_hist(2,k+1);
        
        for f = 1:FRAMES
            alpha = (1 - cos(pi*(f-1)/(FRAMES-1))) / 2;
            t1 = t1s + alpha*(t1e-t1s);
            t2 = t2s + alpha*(t2e-t2s);
            
            x1 = a1*cos(t1);        y1 = a1*sin(t1);
            x2 = x1 + a2*cos(t1+t2); y2 = y1 + a2*sin(t1+t2);
            phi = t1 + t2;
            sx(end+1) = x2; sy(end+1) = y2;
            
            % 5중 메탈 암 링크 드로잉 좌표 리프레시
            for m = 1:5
                set(lnk1_layers(m), 'XData', [0 x1], 'YData', [0 y1]);
                set(lnk2_layers(m), 'XData', [x1 x2], 'YData', [y1 y2]);
            end
            set(j2_cap, 'XData', x1, 'YData', y1);
            set(wrist_cap, 'XData', x2, 'YData', y2);
            
            % 2x2 회전 변환 데이터 전개 후 집게 패치 적용
            R = [cos(phi), -sin(phi); sin(phi), cos(phi)];
            wr_g   = R * [w_x; w_y] + [x2; y2];
            cl_L_g = R * [c_x; c_y] + [x2; y2];
            cl_R_g = R * [c_x; -c_y] + [x2; y2];
            pd_L_g = R * [p_x; p_y] + [x2; y2];
            pd_R_g = R * [p_x; -p_y] + [x2; y2];
            
            set(h_wrist, 'XData', wr_g(1,:), 'YData', wr_g(2,:));
            set(h_claw_L, 'XData', cl_L_g(1,:), 'YData', cl_L_g(2,:));
            set(h_claw_R, 'XData', cl_R_g(1,:), 'YData', cl_R_g(2,:));
            set(h_pad_L, 'XData', pd_L_g(1,:), 'YData', pd_L_g(2,:));
            set(h_pad_R, 'XData', pd_R_g(1,:), 'YData', pd_R_g(2,:));
            
            set(traj_h, 'XData', sx, 'YData', sy);
            
            % 실시간 오차 스칼라 획득 및 정보 패널 주입
            cur_err = norm(X_des-[x2;y2]);
            set(info_h, 'String', sprintf('Iter : %d / %d\n\\theta_1 : %.4f rad\n\\theta_2 : %.4f rad\nError : %.5f', k-1, n-2, t1, t2, cur_err));
            
            % 수치 최적화 스텝이 1단계 완전히 변환될 때마다 우측 모니터 그래프에 실시간 동적 플롯 바인딩
            if f == FRAMES
                live_iters(end+1) = k-1;
                live_errors(end+1) = error_hist(k);
                set(live_err_h, 'XData', live_iters, 'YData', live_errors);
                
                % 수렴 경향성 가시화를 위해 Y축을 로그 스케일로 바인딩 제어
                if length(live_iters) > 1
                    set(ax_err, 'YScale', 'log');
                    xlim(ax_err, [0 max(n-2, 5)]);
                    ylim(ax_err, [0.001 max(error_hist)*1.5]);
                end
            end
            
            drawnow;
            pause(0.004); % 듀얼 렌더링 부하 조정을 위한 pause 연산 주기 최적화
        end
    end
    
    % 마지막 최종 스텝 보정 이음새 누적
    live_iters(end+1) = n-1;
    live_errors(end+1) = error_hist(end);
    set(live_err_h, 'XData', live_iters, 'YData', live_errors);
    
    % 루프 종료 검증 최종 판단 피드백 세팅
    [xf,yf] = fk(theta_hist(1,end), theta_hist(2,end), a1, a2);
    fin_err = norm(X_des-[xf;yf]);
    if fin_err <= tol
        sgtitle(sprintf('%s  —  ✓ CONVERGED (Error=%.5f)', fig_title, fin_err), 'Color', 'cyan', 'FontSize', 14, 'FontWeight', 'bold');
    else
        sgtitle(sprintf('%s  —  ✗ NOT CONVERGED (Error=%.5f)', fig_title, fin_err), 'Color', [1 0.5 0.2], 'FontSize', 14, 'FontWeight', 'bold');
    end
end