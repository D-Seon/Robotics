% Dual-Solution Entire Trajectory Envelope Overlay Simulation (MATLAB R2026a)
% Simultaneous Coordinated Tracking (Gold Elbow-Up vs Silver Elbow-Down)
clear; clc; close all;

%% 1. 로봇 물리 및 타원 궤적 파라미터 설정
l1 = 1.0; % 링크 1 길이 (m)
l2 = 1.0; % 링크 2 길이 (m)

% 목표 타원 방정식 (x^2/2^2 + y^2/1.3^2 = 1)
a = 2.0;  % 장반경
b = 1.3;  % 단반경
num_steps = 240;
t = linspace(0, 2*pi, num_steps);

X_target = a * cos(t);
Y_target = b * sin(t);

%% 2. 시각화 그래픽스 창 설정 (단일 스크린 통합 레이아웃)
fig = figure('Name', 'Dual-IK Solution Kinematic Envelope Analysis', ...
             'NumberTitle', 'off', ...
             'Color', [0.08, 0.09, 0.11], ...
             'Position', [200, 100, 800, 750]);

ax = axes('Parent', fig, ...
          'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
          'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax, 'on'); hold(ax, 'on'); axis(ax, 'equal');
xlim(ax, [-2.5, 2.5]); ylim(ax, [-2.5, 2.5]);
title(ax, 'Unified Workspace: Elbow-Up (Gold) vs Elbow-Down (Silver)', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 배경 가이드: 목표 타원 선
plot(ax, X_target, Y_target, ':', 'Color', [0.4, 0.45, 0.5], 'LineWidth', 1.2);

% 두 로봇 핸드가 공동으로 완성해나갈 마젠타색 최종 궤적선
hand_trace = plot(ax, NaN, NaN, '-', 'Color', [1.0, 0.2, 0.6], 'LineWidth', 3.0);

%% 3. 고정 공용 베이스 받침대 (Shared Base Stand)
base_plate_x = [-0.45, 0.45, 0.30, -0.30]; base_plate_y = [-0.60, -0.60, -0.15, -0.15];
base_neck_x = [-0.15, 0.15, 0.10, -0.10];  base_neck_y = [-0.15, -0.15, 0.0, 0.0];
fill(ax, base_plate_x, base_plate_y, [0.18, 0.20, 0.23], 'EdgeColor', [0.1, 0.11, 0.12], 'LineWidth', 2);
fill(ax, base_neck_x, base_neck_y, [0.55, 0.58, 0.62], 'EdgeColor', [0.3, 0.32, 0.35], 'LineWidth', 1.5);
rectangle(ax, 'Position', [-0.18, -0.18, 0.36, 0.36], 'Curvature', [1, 1], 'FaceColor', [0.35, 0.38, 0.42], 'LineWidth', 1.5);

%% 4. 듀얼 로봇 암 메탈릭 멀티 레이어 빌드
silver_colors = {[0.18, 0.19, 0.20], [0.48, 0.50, 0.53], [0.72, 0.75, 0.78], [0.90, 0.93, 0.96], [1.0, 1.0, 1.0]};
gold_colors   = {[0.25, 0.18, 0.05], [0.70, 0.52, 0.15], [0.88, 0.71, 0.28], [0.98, 0.88, 0.55], [1.0, 0.98, 0.85]};
widths = [26, 21, 14, 7, 2.5];

link1_layers_U = []; link2_layers_U = []; % Gold (Up)
link1_layers_D = []; link2_layers_D = []; % Silver (Down)

for k = 1:5
    % Elbow-Up (금색 포즈)
    link1_layers_U(k) = plot(ax, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
    link2_layers_U(k) = plot(ax, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
    % Elbow-Down (은색 포즈)
    link1_layers_D(k) = plot(ax, [0, 0], [0, 0], 'Color', silver_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
    link2_layers_D(k) = plot(ax, [0, 0], [0, 0], 'Color', silver_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
end

%% 5. 독립형 집게(Gripper) 및 조인트 캡 구조체 패치 세팅
wrist_x = [-0.03,  0.03,  0.03, -0.03]; wrist_y = [-0.05, -0.05,  0.05,  0.05];
claw_base_x = [0.03, 0.11, 0.19, 0.23, 0.20, 0.12, 0.03]; claw_base_y = [0.05, 0.09, 0.07, 0.02, -0.01, 0.03, 0.01];
pad_base_x = [0.15, 0.23, 0.20]; pad_base_y = [0.07, 0.02, -0.01];

% --- 금색 집게 (Elbow-Up) ---
h_wrist_U  = patch(ax, 'XData', wrist_x, 'YData', wrist_y, 'FaceColor', [0.30, 0.25, 0.15]);
h_claw_L_U = patch(ax, 'XData', claw_base_x, 'YData', claw_base_y, 'FaceColor', [0.85, 0.75, 0.45]);
h_claw_R_U = patch(ax, 'XData', claw_base_x, 'YData', -claw_base_y, 'FaceColor', [0.85, 0.75, 0.45]);
h_pad_L_U  = patch(ax, 'XData', pad_base_x, 'YData', pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');
h_pad_R_U  = patch(ax, 'XData', pad_base_x, 'YData', -pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');

% --- 은색 집게 (Elbow-Down) ---
h_wrist_D  = patch(ax, 'XData', wrist_x, 'YData', wrist_y, 'FaceColor', [0.22, 0.24, 0.26]);
h_claw_L_D = patch(ax, 'XData', claw_base_x, 'YData', claw_base_y, 'FaceColor', [0.70, 0.72, 0.75]);
h_claw_R_D = patch(ax, 'XData', claw_base_x, 'YData', -claw_base_y, 'FaceColor', [0.70, 0.72, 0.75]);
h_pad_L_D  = patch(ax, 'XData', pad_base_x, 'YData', pad_base_y, 'FaceColor', [1.0, 0.40, 0.0], 'EdgeColor', 'none');
h_pad_R_D  = patch(ax, 'XData', pad_base_x, 'YData', -pad_base_y, 'FaceColor', [1.0, 0.40, 0.0], 'EdgeColor', 'none');

% 움직이는 팔꿈치용 마커 볼트 캡들
h_j2_cap_U = plot(ax, 0, 0, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.9, 0.75, 0.20], 'MarkerEdgeColor', [0.1, 0.1, 0.1]);
h_j2_cap_D = plot(ax, 0, 0, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.7, 0.72, 0.75], 'MarkerEdgeColor', [0.1, 0.1, 0.1]);

%% 6. 실시간 초고속 동시 제어 및 전체 궤도 포즈 오버레이 루프
x_history = []; y_history = [];

for i = 1:num_steps
    x = X_target(i);
    y = Y_target(i);
    
    %% [공통 역기구학 베이스 데이터 연산]
    D = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2);
    if D > 1, D = 1; elseif D < -1, D = -1; end
    
    %% [해의 분리 계산]
    % 1) Elbow-Up (+\theta_2) 각도 계산
    theta2_U = acos(D); 
    theta1_U = atan2(y, x) - atan2(l2 * sin(theta2_U), l1 + l2 * cos(theta2_U));
    
    % 2) Elbow-Down (-\theta_2) 각도 계산
    theta2_D = -acos(D); 
    theta1_D = atan2(y, x) - atan2(l2 * sin(theta2_D), l1 + l2 * cos(theta2_D));
    
    %% [순기구학 변환을 통한 각각의 글로벌 좌표 매핑]
    % 금색 로봇 암(Up) 좌표
    x1_U = l1 * cos(theta1_U); y1_U = l1 * sin(theta1_U);
    x2_U = x1_U + l2 * cos(theta1_U + theta2_U); y2_U = y1_U + l2 * sin(theta1_U + theta2_U);
    phi_U = theta1_U + theta2_U;
    
    % 은색 로봇 암(Down) 좌표
    x1_D = l1 * cos(theta1_D); y1_D = l1 * sin(theta1_D);
    x2_D = x1_D + l2 * cos(theta1_D + theta2_D); y2_D = y1_D + l2 * sin(theta1_D + theta2_D);
    phi_D = theta1_D + theta2_D;
    

    
    %% [메인 실시간 그래픽 렌더링 동기화 업데이트]
    % 금색 로봇 팔 갱신
    for k = 1:5
        set(link1_layers_U(k), 'XData', [0, x1_U], 'YData', [0, y1_U]);
        set(link2_layers_U(k), 'XData', [x1_U, x2_U], 'YData', [y1_U, y2_U]);
    end
    set(h_j2_cap_U, 'XData', x1_U, 'YData', y1_U);
    
    % 은색 로봇 팔 갱신
    for k = 1:5
        set(link1_layers_D(k), 'XData', [0, x1_D], 'YData', [0, y1_D]);
        set(link2_layers_D(k), 'XData', [x1_D, x2_D], 'YData', [y1_D, y2_D]);
    end
    set(h_j2_cap_D, 'XData', x1_D, 'YData', y1_D);
    
    %% 듀얼 집게 손가락 다이내믹 모션 제어 및 변환 matrix
    grip_control = 0.65 + 0.35 * sin(t(i));
    claw_dyn_y = claw_base_y * grip_control; pad_dyn_y = pad_base_y * grip_control;
    
    R_U = [cos(phi_U), -sin(phi_U); sin(phi_U), cos(phi_U)];
    R_D = [cos(phi_D), -sin(phi_D); sin(phi_D), cos(phi_D)];
    
    % 금색 집게 매핑
    wr_g_U = R_U * [wrist_x; wrist_y] + [x2_U; y2_U];
    cl_L_g_U = R_U * [claw_base_x; claw_dyn_y] + [x2_U; y2_U]; cl_R_g_U = R_U * [claw_base_x; -claw_dyn_y] + [x2_U; y2_U];
    pd_L_g_U = R_U * [pad_base_x; pad_dyn_y] + [x2_U; y2_U];   pd_R_g_U = R_U * [pad_base_x; -pad_dyn_y] + [x2_U; y2_U];
    set(h_wrist_U, 'XData', wr_g_U(1,:), 'YData', wr_g_U(2,:));
    set(h_claw_L_U, 'XData', cl_L_g_U(1,:), 'YData', cl_L_g_U(2,:)); set(h_claw_R_U, 'XData', cl_R_g_U(1,:), 'YData', cl_R_g_U(2,:));
    set(h_pad_L_U, 'XData', pd_L_g_U(1,:), 'YData', pd_L_g_U(2,:));   set(h_pad_R_U, 'XData', pd_R_g_U(1,:), 'YData', pd_R_g_U(2,:));
    
    % 은색 집게 매핑
    wr_g_D = R_D * [wrist_x; wrist_y] + [x2_D; y2_D];
    cl_L_g_D = R_D * [claw_base_x; claw_dyn_y] + [x2_D; y2_D]; cl_R_g_D = R_D * [claw_base_x; -claw_dyn_y] + [x2_D; y2_D];
    pd_L_g_D = R_D * [pad_base_x; pad_dyn_y] + [x2_D; y2_D];   pd_R_g_D = R_D * [pad_base_x; -pad_dyn_y] + [x2_D; y2_D];
    set(h_wrist_D, 'XData', wr_g_D(1,:), 'YData', wr_g_D(2,:));
    set(h_claw_L_D, 'XData', cl_L_g_D(1,:), 'YData', cl_L_g_D(2,:)); set(h_claw_R_D, 'XData', cl_R_g_D(1,:), 'YData', cl_R_g_D(2,:));
    set(h_pad_L_D, 'XData', pd_L_g_D(1,:), 'YData', pd_L_g_D(2,:));   set(h_pad_R_D, 'XData', pd_R_g_D(1,:), 'YData', pd_R_g_D(2,:));
    
    %% 최종 말단 타원 궤적 동적 덮어쓰기 갱신
    x_history = [x_history, x2_U]; y_history = [y_history, y2_U];
    set(hand_trace, 'XData', x_history, 'YData', y_history);
    
    %% 초고속 스피드 동기화 (`pause(0.005)`)
    drawnow;
    pause(0.005);
end