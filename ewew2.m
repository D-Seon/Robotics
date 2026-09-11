% Real-Time Numerical IK (Newton-Raphson) Visual Simulation (MATLAB R2026a)
% Shared Workspace with Silver (Elbow-Down) & Gold (Elbow-Up) Visuals
clear; clc; close all;

%% 1. 로봇 물리 및 문제 2번 8개 목표점 조건 설정
l1 = 1.0; % 링크 1 길이 (m)
l2 = 1.0; % 링크 2 길이 (m)

% 문제 2번: 8개의 타스크 영역 목표 좌표 (X, Y)
p2_targets = [ 1.2,  0.8; 
               0.5,  1.5; 
              -0.8,  1.2; 
              -1.5, -0.5; 
              -1.2, -0.8; 
              -0.5, -1.5; 
               0.8, -1.2; 
               1.5, -0.5 ];

% 각 목표점별 Newton-Raphson 연산용 초기 각도값 구속 조건
p2_starts = [ 0.5,  0.5; 
              0.5,  0.5; 
              1.8,  0.5; 
              2.2,  0.4; 
             -2.5,  0.5; 
             -2.0,  0.6; 
             -0.8,  0.5; 
             -0.4,  0.4 ];

max_iter_per_point = 25; % 각 지점당 수치해석 매핑 반복수 (실시간 가속용)
t_global = linspace(0, 2*pi, size(p2_targets, 1) * max_iter_per_point);

%% 2. 메인 피겨 및 듀얼 가로형 레이아웃 구축
fig = figure('Name', 'Real-Time Numerical IK Full Manipulator Dashboard', ...
             'NumberTitle', 'off', ...
             'Color', [0.08, 0.09, 0.11], ...
             'Position', [100, 150, 1250, 580]);

%% 3. [좌측 스크린 - 은색 로봇 암 (-\theta_2 해)]
ax1 = subplot(1, 2, 1, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax1, 'on'); hold(ax1, 'on'); axis(ax1, 'equal');
xlim(ax1, [-2.5, 2.5]); ylim(ax1, [-2.5, 2.5]);
title(ax1, 'SCREEN 01: Numerical Elbow-Down (-\theta_2)', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 8개 목표 타겟 점 사전 마킹
plot(ax1, p2_targets(:,1), p2_targets(:,2), 'cx', 'MarkerSize', 8, 'LineWidth', 2);
hand_trace_D = plot(ax1, NaN, NaN, '-', 'Color', [1.0, 0.2, 0.6], 'LineWidth', 2.5);

%% 4. [우측 스크린 - 금색 로봇 암 (+\theta_2 해)]
ax2 = subplot(1, 2, 2, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax2, 'on'); hold(ax2, 'on'); axis(ax2, 'equal');
xlim(ax2, [-2.5, 2.5]); ylim(ax2, [-2.5, 2.5]);
title(ax2, 'SCREEN 02: Numerical Elbow-Up (+\theta_2)', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

plot(ax2, p2_targets(:,1), p2_targets(:,2), 'cx', 'MarkerSize', 8, 'LineWidth', 2);
hand_trace_U = plot(ax2, NaN, NaN, '-', 'Color', [1.0, 0.2, 0.6], 'LineWidth', 2.5);

%% 5. 고정 베이스 스탠드 오버레이 (Static Bases)
base_plate_x = [-0.45, 0.45, 0.30, -0.30]; base_plate_y = [-0.60, -0.60, -0.15, -0.15];
base_neck_x = [-0.15, 0.15, 0.10, -0.10];  base_neck_y = [-0.15, -0.15, 0.0, 0.0];

% 좌우측 스탠드 렌더링
fill(ax1, base_plate_x, base_plate_y, [0.18, 0.20, 0.23], 'EdgeColor', 'none');
fill(ax1, base_neck_x, base_neck_y, [0.55, 0.58, 0.62], 'EdgeColor', 'none');
rectangle(ax1, 'Position', [-0.18, -0.18, 0.36, 0.36], 'Curvature', [1, 1], 'FaceColor', [0.35, 0.38, 0.42]);

fill(ax2, base_plate_x, base_plate_y, [0.18, 0.20, 0.23], 'EdgeColor', 'none');
fill(ax2, base_neck_x, base_neck_y, [0.55, 0.58, 0.62], 'EdgeColor', 'none');
rectangle(ax2, 'Position', [-0.18, -0.18, 0.36, 0.36], 'Curvature', [1, 1], 'FaceColor', [0.35, 0.38, 0.42]);

%% 6. 요청하신 오리지널 입체 메탈릭 링크 레이어 빌드
silver_colors = {[0.18, 0.19, 0.20], [0.48, 0.50, 0.53], [0.72, 0.75, 0.78], [0.90, 0.93, 0.96], [1.0, 1.0, 1.0]};
gold_colors   = {[0.25, 0.18, 0.05], [0.70, 0.52, 0.15], [0.88, 0.71, 0.28], [0.98, 0.88, 0.55], [1.0, 0.98, 0.85]};
widths = [26, 21, 14, 7, 2.5];

link1_layers_D = []; link2_layers_D = [];
link1_layers_U = []; link2_layers_U = [];
for k = 1:5
    link1_layers_D(k) = plot(ax1, [0, 0], [0, 0], 'Color', silver_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
    link2_layers_D(k) = plot(ax1, [0, 0], [0, 0], 'Color', silver_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
    link1_layers_U(k) = plot(ax2, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
    link2_layers_U(k) = plot(ax2, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
end

%% 7. 오리지널 다각형 집게(Gripper) 장치 패치 매핑
wrist_x = [-0.03,  0.03,  0.03, -0.03]; wrist_y = [-0.05, -0.05,  0.05,  0.05];
claw_base_x = [0.03, 0.11, 0.19, 0.23, 0.20, 0.12, 0.03]; claw_base_y = [0.05, 0.09, 0.07, 0.02, -0.01, 0.03, 0.01];
pad_base_x = [0.15, 0.23, 0.20]; pad_base_y = [0.07, 0.02, -0.01];

% 좌측 은색 로봇용 집게
h_wrist_D  = patch(ax1, 'XData', wrist_x, 'YData', wrist_y, 'FaceColor', [0.25, 0.27, 0.30]);
h_claw_L_D = patch(ax1, 'XData', claw_base_x, 'YData', claw_base_y, 'FaceColor', [0.75, 0.78, 0.82]);
h_claw_R_D = patch(ax1, 'XData', claw_base_x, 'YData', -claw_base_y, 'FaceColor', [0.75, 0.78, 0.82]);
h_pad_L_D  = patch(ax1, 'XData', pad_base_x, 'YData', pad_base_y, 'FaceColor', [1.0, 0.45, 0.0], 'EdgeColor', 'none');
h_pad_R_D  = patch(ax1, 'XData', pad_base_x, 'YData', -pad_base_y, 'FaceColor', [1.0, 0.45, 0.0], 'EdgeColor', 'none');

% 우측 금색 로봇용 집게
h_wrist_U  = patch(ax2, 'XData', wrist_x, 'YData', wrist_y, 'FaceColor', [0.30, 0.25, 0.15]);
h_claw_L_U = patch(ax2, 'XData', claw_base_x, 'YData', claw_base_y, 'FaceColor', [0.85, 0.75, 0.45]);
h_claw_R_U = patch(ax2, 'XData', claw_base_x, 'YData', -claw_base_y, 'FaceColor', [0.85, 0.75, 0.45]);
h_pad_L_U  = patch(ax2, 'XData', pad_base_x, 'YData', pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');
h_pad_R_U  = patch(ax2, 'XData', pad_base_x, 'YData', -pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');

h_j2_cap_D = plot(ax1, 0, 0, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.8, 0.82, 0.85], 'MarkerEdgeColor', [0.1, 0.1, 0.1]);
h_j2_cap_U = plot(ax2, 0, 0, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.9, 0.75, 0.20], 'MarkerEdgeColor', [0.1, 0.1, 0.1]);

%% 8. 실시간 역기구학 수치 해석 및 애니메이션 루프 전개
x_hist_D = []; y_hist_D = [];
x_hist_U = []; y_hist_U = [];
global_step_idx = 0;

for p = 1:size(p2_targets, 1)
    target = p2_targets(p, :)';
    
    % 각 포인트 진입 시 지정된 독립 초기 각도 세팅
    theta_D = p2_starts(p, :)';
    theta_U = p2_starts(p, :)';
    
    for iter = 1:max_iter_per_point
        global_step_idx = global_step_idx + 1;
        
        %% ================= [단계 1: 수치해석적 IK (Newton-Raphson 법)] =================
        % 1) 현재 각도 기반 정기구학 말단 위치 연산 (FK)
        x_cur_D = l1*cos(theta_D(1)) + l2*cos(theta_D(1)+theta_D(2));
        y_cur_D = l1*sin(theta_D(1)) + l2*sin(theta_D(1)+theta_D(2));
        
        x_cur_U = l1*cos(theta_U(1)) + l2*cos(theta_U(1)+theta_U(2));
        y_cur_U = l1*sin(theta_U(1)) + l2*sin(theta_U(1)+theta_U(2));
        
        % 2) 목표 점과의 타스크 공간 잔차 오차(Error Vector) 도출
        dx_D = target(1) - x_cur_D; dy_D = target(2) - y_cur_D;
        dx_U = target(1) - x_cur_U; dy_U = target(2) - y_cur_U;
        
        % 3) 수치해석 자코비안 역행렬 수식 대입 및 각도 변화량 보정
        % --- SCREEN 01 (Elbow-Down 해 강제 유도용 내부 수식 제어) ---
        s2_D = sin(theta_D(2)); s12_D = sin(theta_D(1)+theta_D(2)); c12_D = cos(theta_D(1)+theta_D(2)); c1_D = cos(theta_D(1)); s1_D = sin(theta_D(1));
        dth1_D = (c12_D*dx_D + s12_D*dy_D) / s2_D;
        dth2_D = ((-c1_D-c12_D)*dx_D + (-s1_D-s12_D)*dy_D) / s2_D;
        
        % --- SCREEN 02 (Elbow-Up 해 강제 유도용 내부 수식 제어) ---
        s2_U = sin(theta_U(2)); s12_U = sin(theta_U(1)+theta_U(2)); c12_U = cos(theta_U(1)+theta_U(2)); c1_U = cos(theta_U(1)); s1_U = sin(theta_U(1));
        dth1_U = (c12_U*dx_U + s12_U*dy_U) / s2_U;
        dth2_U = ((-c1_U-c12_U)*dx_U + (-s1_U-s12_U)*dy_U) / s2_U;
        
        % 4) 뉴턴-랩슨 점진적 관절각 업데이트 업데이트
        theta_D = theta_D + [dth1_D; dth2_D];
        theta_U = theta_U + [dth1_U; dth2_U];
        
        %% ================= [단계 2: 실시간 그래픽 변환 및 렌더링] =================
        % 기하학적 관절 2(팔꿈치) 링크 끝단 좌표 연산
        x1_D = l1 * cos(theta_D(1)); y1_D = l1 * sin(theta_D(1));
        x1_U = l1 * cos(theta_U(1)); y1_U = l1 * sin(theta_U(1));
        
        % 최종 마젠타색 궤도 드로잉 축적용 히스토리 어레이 누적
        x_hist_D = [x_hist_D, x_cur_D]; y_hist_D = [y_hist_D, y_cur_D];
        x_hist_U = [x_hist_U, x_cur_U]; y_hist_U = [y_hist_U, y_cur_U];
        
        % --- 물리 메탈릭 암 링크 가시화 동적 리프레시 ---
        for k = 1:5
            set(link1_layers_D(k), 'XData', [0, x1_D], 'YData', [0, y1_D]);
            set(link2_layers_D(k), 'XData', [x1_D, x_cur_D], 'YData', [y1_D, y_cur_D]);
            
            set(link1_layers_U(k), 'XData', [0, x1_U], 'YData', [0, y1_U]);
            set(link2_layers_U(k), 'XData', [x1_U, x_cur_U], 'YData', [y1_U, y_cur_U]);
        end
        set(h_j2_cap_D, 'XData', x1_D, 'YData', y1_D);
        set(h_j2_cap_U, 'XData', x1_U, 'YData', y1_U);
        
        % --- 집게 팩토리 모션 및 2x2 회전 변환 행렬 매핑 ---
        grip_control = 0.65 + 0.35 * sin(t_global(global_step_idx));
        claw_dyn_y = claw_base_y * grip_control; pad_dyn_y = pad_base_y * grip_control;
        
        phi_D = theta_D(1) + theta_D(2); phi_U = theta_U(1) + theta_U(2);
        R_D = [cos(phi_D), -sin(phi_D); sin(phi_D), cos(phi_D)];
        R_U = [cos(phi_U), -sin(phi_U); sin(phi_U), cos(phi_U)];
        
        % 은색 집게 매트릭스 변환
        wr_g_D = R_D * [wrist_x; wrist_y] + [x_cur_D; y_cur_D];
        cl_L_g_D = R_D * [claw_base_x; claw_dyn_y] + [x_cur_D; y_cur_D]; cl_R_g_D = R_D * [claw_base_x; -claw_dyn_y] + [x_cur_D; y_cur_D];
        pd_L_g_D = R_D * [pad_base_x; pad_dyn_y] + [x_cur_D; y_cur_D];   pd_R_g_D = R_D * [pad_base_x; -pad_dyn_y] + [x_cur_D; y_cur_D];
        set(h_wrist_D, 'XData', wr_g_D(1,:), 'YData', wr_g_D(2,:));
        set(h_claw_L_D, 'XData', cl_L_g_D(1,:), 'YData', cl_L_g_D(2,:)); set(h_claw_R_D, 'XData', cl_R_g_D(1,:), 'YData', cl_R_g_D(2,:));
        set(h_pad_L_D, 'XData', pd_L_g_D(1,:), 'YData', pd_L_g_D(2,:));   set(h_pad_R_D, 'XData', pd_R_g_D(1,:), 'YData', pd_R_g_D(2,:));
        
        % 금색 집게 매트릭스 변환
        wr_g_U = R_U * [wrist_x; wrist_y] + [x_cur_U; y_cur_U];
        cl_L_g_U = R_U * [claw_base_x; claw_dyn_y] + [x_cur_U; y_cur_U]; cl_R_g_U = R_U * [claw_base_x; -claw_dyn_y] + [x_cur_U; y_cur_U];
        pd_L_g_U = R_U * [pad_base_x; pad_dyn_y] + [x_cur_U; y_cur_U];   pd_R_g_U = R_U * [pad_base_x; -pad_dyn_y] + [x_cur_U; y_cur_U];
        set(h_wrist_U, 'XData', wr_g_U(1,:), 'YData', wr_g_U(2,:));
        set(h_claw_L_U, 'XData', cl_L_g_U(1,:), 'YData', cl_L_g_U(2,:)); set(h_claw_R_U, 'XData', cl_R_g_U(1,:), 'YData', cl_R_g_U(2,:));
        set(h_pad_L_U, 'XData', pd_L_g_U(1,:), 'YData', pd_L_g_U(2,:));   set(h_pad_R_U, 'XData', pd_R_g_U(1,:), 'YData', pd_R_g_U(2,:));
        
        % --- 실시간 마젠타 네온 수렴 궤적선 가이드 업데이트 ---
        set(hand_trace_D, 'XData', x_hist_D, 'YData', y_hist_D);
        set(hand_trace_U, 'XData', x_hist_U, 'YData', y_hist_U);
        
        % 루프 프레임 가속 드로잉 싱크 최적화 (`pause(0.015)`)
        drawnow;
        pause(0.015);
    end
end