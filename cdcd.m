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
animate_all_points(all_theta, points, a1, a2, colors8, tolerance);

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
        t1=theta(1); t2=theta(2);   
        Jinv = (1/detJ)*[cos(t1+t2), sin(t1+t2);
                         -cos(t1)-cos(t1+t2), -sin(t1)-sin(t1+t2)];
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

function s = choose(cond, a, b)
    if cond; s=a; else; s=b; end
end

function [wrist_x, wrist_y, claw_base_x, claw_base_y, pad_base_x, pad_base_y] = get_gripper_geometry_data()
    wrist_x = [-0.03,  0.03,  0.03, -0.03]; wrist_y = [-0.05, -0.05,  0.05,  0.05];
    claw_base_x = [0.03, 0.11, 0.19, 0.23, 0.20, 0.12, 0.03]; claw_base_y = [0.05, 0.09, 0.07, 0.02, -0.01, 0.03, 0.01];
    pad_base_x = [0.15, 0.23, 0.20]; pad_base_y = [0.07, 0.02, -0.01];
end

% ── 애니메이션 함수 (포인트별 반복 횟수 오버레이 매핑 구조 적용) ──
function animate_all_points(all_theta, points, a1, a2, colors8, tol)
    n_pts = size(points,1);
    FRAMES = 40;   
    
    fig = figure('Name','Problems II — Robot Advanced HUD Animation',...
                 'Color',[0.08, 0.09, 0.11],'Position',[100 60 1400 700]);
    
    %% --- [좌측 SCREEN 01: 물리 타스크 공간 시뮬레이터] ---
    ax_robot = subplot(1, 2, 1, 'Parent', fig, 'Color', [0.12, 0.14, 0.18], ...
                  'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
                  'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
    hold(ax_robot,'on'); axis(ax_robot,'equal');
    xlim(ax_robot,[-2.5 2.5]); ylim(ax_robot,[-2.5 2.5]);
    grid(ax_robot,'on');
    xlabel(ax_robot,'X (m)'); ylabel(ax_robot,'Y (m)');
    
    tw=linspace(0,2*pi,300);
    plot(ax_robot,(a1+a2)*cos(tw),(a1+a2)*sin(tw),'--','Color',[0.35 0.35 0.55],'LineWidth',1.2);
    base_plate_x = [-0.45, 0.45, 0.30, -0.30]; base_plate_y = [-0.60, -0.60, -0.15, -0.15];
    base_neck_x = [-0.15, 0.15, 0.10, -0.10];  base_neck_y = [-0.15, -0.15, 0.0, 0.0];
    fill(ax_robot, base_plate_x, base_plate_y, [0.18, 0.20, 0.23], 'EdgeColor', [0.1, 0.11, 0.12], 'LineWidth', 2);
    fill(ax_robot, base_neck_x, base_neck_y, [0.55, 0.58, 0.62], 'EdgeColor', [0.3, 0.32, 0.35], 'LineWidth', 1.5);
    rectangle(ax_robot, 'Position', [-0.18, -0.18, 0.36, 0.36], 'Curvature', [1, 1], 'FaceColor', [0.35, 0.38, 0.42], 'LineWidth', 1.5);
    
    h_targets = gobjects(n_pts,1);
    for p=1:n_pts
        h_targets(p) = scatter(ax_robot,points(p,1),points(p,2),160,...
                               colors8(p,:),'filled',...
                               'MarkerEdgeColor','w','LineWidth',1.2);
        text(ax_robot,points(p,1)+0.09,points(p,2)+0.09,sprintf('P%d',p),...
             'Color',colors8(p,:),'FontSize',10.5,'FontWeight','bold');
    end
    
    traj_h  = plot(ax_robot,NaN,NaN,'-','Color',[1.0, 0.2, 0.6],'LineWidth',2.0);
    
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
    
    scatter(ax_robot,0,0,250,[0.2, 0.2, 0.2],'filled','MarkerEdgeColor','k','LineWidth',1.8);
    j2_cap = scatter(ax_robot,0,0,190,[0.9, 0.75, 0.20],'filled','MarkerEdgeColor','k','LineWidth',1.6);
    wrist_cap = scatter(ax_robot,0,0,130,[0.0, 0.9, 1.0],'filled','MarkerEdgeColor','k','LineWidth',1.6);
    
    ring_h = scatter(ax_robot,NaN,NaN,350,'w','LineWidth',2.5);
    
    info_h  = text(ax_robot,-2.42, 2.40,'','Color','w','FontSize',10.5,...
                   'FontWeight','bold','VerticalAlignment','top','FontName','Courier New');

    %% --- [우측 SCREEN 02: 실시간 오차 수렴 트레이닝 차트 (X축 최적화)] ---
    ax_err = subplot(1, 2, 2, 'Parent', fig, 'Color', [0.12, 0.14, 0.18], ...
                  'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
                  'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
    hold(ax_err, 'on'); grid(ax_err, 'on');
    xlabel(ax_err, 'Iteration step', 'FontWeight', 'bold');
    ylabel(ax_err, 'Position Kinematics Error ||e||', 'FontWeight', 'bold');
    title(ax_err, 'SCREEN 02: Real-time Multi-Target Convergence Overlay', 'Color', [0.9, 0.95, 1.0], 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Segoe UI');
    
    set(ax_err, 'YScale', 'log');
    
    yline(ax_err, tol, 'r--', 'LineWidth', 1.8);
    text(ax_err, 0.5, tol * 1.3, 'Tolerance = 0.01', 'Color', 'r', 'FontSize', 9, 'FontWeight', 'bold');
    
    % ★ 변경: X축 가동 범위를 개별 이터레이션 크기(0 ~ 20)로 컴팩트하게 제한하여 상세하게 관찰
    xlim(ax_err, [0, 20]); 
    ylim(ax_err, [0.001, 2.5]);

    sgtitle('Problems II — Full 2-Link Planar Manipulator Multi-Target IK HUD Simulator', 'Color', 'w', 'FontSize', 14, 'FontWeight', 'bold');

    [w_x, w_y, c_x, c_y, p_x, p_y] = get_gripper_geometry_data();
    
    sx=[]; sy=[];   
    
    %% --- 시뮬레이션 순차 구동 엔진 구동 ---
    for p = 1:n_pts
        theta_hist = all_theta{p};
        n_steps    = size(theta_hist,2);
        X_des      = points(p,1:2);
        pt_col     = colors8(p,:);
        
        % ★ 포인트마다 새 선 객체를 끊어 생성하여 누적하면서도, X데이터를 이터레이션(0부터 시작)으로 매핑
        live_err_h = plot(ax_err, NaN, NaN, '-o', 'Color', pt_col, 'LineWidth', 2, ...
                          'MarkerSize', 4, 'MarkerFaceColor', pt_col, 'DisplayName', sprintf('P%d', p));
        
        % 포인트 세션용 개별 가로/세로 데이터 공간 초기화
        pt_time_axis = [];
        pt_error_history = [];
        
        set(ring_h,'XData',X_des(1),'YData',X_des(2),'MarkerEdgeColor',pt_col,'SizeData',360);
        
        for k = 1:n_steps-1
            t1s=theta_hist(1,k); t2s=theta_hist(2,k);
            t1e=theta_hist(1,k+1); t2e=theta_hist(2,k+1);
            
            [cx_init, cy_init] = fk(t1s, t2s, a1, a2);
            step_error = norm(X_des' - [cx_init; cy_init]);
            
            % ★ 변경 핵심: 전역 시간 프레임 대신 '현재 포인트의 이터레이션 인덱스(k-1)'를 X축으로 사용
            current_iter = k - 1;
            pt_time_axis(end+1) = current_iter;
            
            if step_error < 0.001; step_error = 0.001; end
            pt_error_history(end+1) = step_error;
            
            % 동적 선 데이터 바인딩
            set(live_err_h, 'XData', pt_time_axis, 'YData', pt_error_history);
            
            % 혹시 연산 횟수가 20회 넘는 특이 포인트 등장 시 차트 우측 자동 조절 확장
            if current_iter > 18
                xlim(ax_err, [0, current_iter + 3]);
            end
            
            for f = 1:FRAMES
                alpha = (1 - cos(pi*(f-1)/(FRAMES-1))) / 2;
                t1 = t1s + alpha*(t1e-t1s);
                t2 = t2s + alpha*(t2e-t2s);
                
                x1=a1*cos(t1);        y1=a1*sin(t1);
                x2=x1+a2*cos(t1+t2); y2=y1+a2*sin(t1+t2);
                phi = t1 + t2;
                sx(end+1)=x2; sy(end+1)=y2;
                
                for m = 1:5
                    set(lnk1_layers(m), 'XData',[0 x1],'YData',[0 y1]);
                    set(lnk2_layers(m), 'XData',[x1 x2],'YData',[y1 y2]);
                end
                set(j2_cap, 'XData', x1, 'YData', y1);
                set(wrist_cap, 'XData', x2, 'YData', y2);
                
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
                
                set(traj_h,'XData',sx,'YData',sy,'Color',[pt_col 0.65]);
                
                cur_err=norm(X_des'-[x2;y2]);
                set(info_h,'String',...
                    sprintf('Active Point : %d / %d\nOptimizer It : %d / %d\nJoint Ang #1 : %6.4f rad\nJoint Ang #2 : %6.4f rad\nLinear Error : %.5f m',...
                            p, n_pts, k-1, n_steps-2, t1, t2, cur_err));
                drawnow;
                pause(0.003); 
            end
        end
        
        % 루프 마감 최종 스텝 오차 적재 보정
        [cx_final, cy_final] = fk(theta_hist(1,end), theta_hist(2,end), a1, a2);
        final_step_err = norm(X_des' - [cx_final; cy_final]);
        if final_step_err < 0.001; final_step_err = 0.001; end
        pt_time_axis(end+1) = n_steps - 1;
        pt_error_history(end+1) = final_step_err;
        set(live_err_h, 'XData', pt_time_axis, 'YData', pt_error_history);
        
        for fl=1:6
            set(ring_h,'SizeData',300+100*mod(fl,2));
            drawnow; pause(0.05);
        end
    end
    
    % 전 과정 완료 후 예쁜 구분을 위해 범례(Legend) 텍스트를 상단에 고정 표시
    legend(ax_err, 'Location', 'northeast', 'TextColor', 'w', 'BackgroundColor', [0.15 0.17 0.22]);
    set(info_h,'String','✓ SEQUENCE DONE: All targets successfully visited.','Color','cyan');
end