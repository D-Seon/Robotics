% Trajectory Analysis: Link-1 End Joint under Fixed Positive Theta2 (MATLAB R2026a)
clear; clc; close all;

%% 1. 로봇 물리 및 고정각 설정
l1 = 1.0; % 링크 1 길이 (m)
l2 = 1.0; % 링크 2 길이 (m)

% ★ 관절 2 (\theta_2)를 양의 해(Elbow-Up) 상수로 고정 (예: 45도)
theta2_fixed_deg = 45; 
theta2_fixed = deg2rad(theta2_fixed_deg);

% 타원 궤적 정보 (타겟 방위각 참조용)
a = 2.0; b = 1.3;
num_steps = 240;
t = linspace(0, 2*pi, num_steps);
X_ellipse = a * cos(t);
Y_ellipse = b * sin(t);

%% 2. 시각화 그래픽스 환경 구축 (분할 스크린 레이아웃)
fig = figure('Name', 'Link-1 Joint Trajectory under Fixed Positive \theta_2', ...
             'NumberTitle', 'off', ...
             'Color', [0.08, 0.09, 0.11], ...
             'Position', [100, 200, 1250, 550]);

%% 3. [좌측 스크린 - 물리 공간 및 링크 1 말단 궤적 동시 표기]
ax1 = subplot(1, 2, 1, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax1, 'on'); hold(ax1, 'on'); axis(ax1, 'equal');
xlim(ax1, [-2.5, 2.5]); ylim(ax1, [-2.5, 2.5]);
title(ax1, 'SCREEN 01: Workspace with Link-1 Trajectory', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 참조용 타원 기준선 (회색 점선)
plot(ax1, X_ellipse, ':', 'Color', [0.4, 0.45, 0.5], 'LineWidth', 1);

% ★ 이번 코딩의 핵심: 링크 1 말단(관절 2)이 그리는 실제 궤적선 (네온 그린)
link1_trace_ax1 = plot(ax1, NaN, NaN, '-', 'Color', [0.0, 1.0, 0.5], 'LineWidth', 2.5);

% 산업용 베이스 고정 받침대 드로잉
base_plate_x = [-0.45, 0.45, 0.30, -0.30]; base_plate_y = [-0.60, -0.60, -0.15, -0.15];
base_neck_x = [-0.15, 0.15, 0.10, -0.10];  base_neck_y = [-0.15, -0.15, 0.0, 0.0];
fill(ax1, base_plate_x, base_plate_y, [0.18, 0.20, 0.23], 'EdgeColor', [0.1, 0.11, 0.12], 'LineWidth', 2);
fill(ax1, base_neck_x, base_neck_y, [0.55, 0.58, 0.62], 'EdgeColor', [0.3, 0.32, 0.35], 'LineWidth', 1.5);
rectangle(ax1, 'Position', [-0.18, -0.18, 0.36, 0.36], 'Curvature', [1, 1], 'FaceColor', [0.35, 0.38, 0.42], 'LineWidth', 1.5);

%% 4. [우측 스크린 - 링크 1 말단(관절 2)만의 순수 궤적 확대 뷰]
ax2 = subplot(1, 2, 2, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax2, 'on'); hold(ax2, 'on'); axis(ax2, 'equal');
xlim(ax2, [-1.2, 1.2]); ylim(ax2, [-1.2, 1.2]); 
xlabel(ax2, 'X1 Position (m)', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
ylabel(ax2, 'Y1 Position (m)', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
title(ax2, 'SCREEN 02: Pure Link-1 End Trajectory (x_1, y_1)', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 반지름 1m 물리적 경계선
th_circle = linspace(0, 2*pi, 100);
plot(ax2, cos(th_circle), sin(th_circle), '--', 'Color', [0.3, 0.35, 0.45], 'LineWidth', 1);

% 우측 스크린 링크 1 실시간 추적선 (마젠타 핑크)
link1_trace_ax2 = plot(ax2, NaN, NaN, '-', 'Color', [1.0, 0.2, 0.6], 'LineWidth', 2.5);
h_joint_dot = plot(ax2, NaN, NaN, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [1.0, 0.2, 0.6], 'MarkerEdgeColor', [1,1,1]);

%% 5. 아노다이징 골드 메탈릭 암 레이어 빌드
gold_colors = {[0.25, 0.18, 0.05], [0.70, 0.52, 0.15], [0.88, 0.71, 0.28], [0.98, 0.88, 0.55], [1.0, 0.98, 0.85]};
widths = [26, 21, 14, 7, 2.5];

link1_layers = []; link2_layers = [];
for k = 1:5
    link1_layers(k) = plot(ax1, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
    link2_layers(k) = plot(ax1, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
end

%% 6. 집게(Gripper) 및 조인트 피벗 볼트 세팅
wrist_x = [-0.03,  0.03,  0.03, -0.03]; wrist_y = [-0.05, -0.05,  0.05,  0.05];
claw_base_x = [0.03, 0.11, 0.19, 0.23, 0.20, 0.12, 0.03]; claw_base_y = [0.05, 0.09, 0.07, 0.02, -0.01, 0.03, 0.01];
pad_base_x = [0.15, 0.23, 0.20]; pad_base_y = [0.07, 0.02, -0.01];

h_wrist  = patch(ax1, 'XData', wrist_x, 'YData', wrist_y, 'FaceColor', [0.30, 0.25, 0.15]);
h_claw_L = patch(ax1, 'XData', claw_base_x, 'YData', claw_base_y, 'FaceColor', [0.85, 0.75, 0.45]);
h_claw_R = patch(ax1, 'XData', claw_base_x, 'YData', -claw_base_y, 'FaceColor', [0.85, 0.75, 0.45]);
h_pad_L  = patch(ax1, 'XData', pad_base_x, 'YData', pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');
h_pad_R  = patch(ax1, 'XData', pad_base_x, 'YData', -pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');

h_j2_cap = plot(ax1, 0, 0, 'o', 'MarkerSize', 11, 'MarkerFaceColor', [0.9, 0.75, 0.20], 'MarkerEdgeColor', [0.1, 0.1, 0.1]);

%% 7. 오버레이 HUD 정보창 레이아웃 배치
hud_box = annotation('textbox', [0.14, 0.68, 0.17, 0.18], 'Color', [0.98, 0.88, 0.55], ...
                     'BackgroundColor', [0.15, 0.17, 0.22, 0.85], 'EdgeColor', [0.6, 0.4, 0.1], ...
                     'FontName', 'Courier New', 'FontSize', 9, 'FontWeight', 'bold');

%% 8. 타원 방위각 연동 및 실시간 모션 루프
x1_history = []; y1_history = [];

for i = 1:num_steps
    % 현재 프레임의 타원 타겟 좌표 추출
    x_target = X_ellipse(i);
    y_target = Y_ellipse(i);
    
    %% [타원 방위각 연동 역기구학 해석]
    gamma = atan2(y_target, x_target);
    alpha = atan2(l2 * sin(theta2_fixed), l1 + l2 * cos(theta2_fixed));
    
    % \theta_2가 고정된 구조에서 타원 방향을 향하는 \theta_1 연산
    theta1 = gamma - alpha; 
    
    %% [순기구학 기반 각 링크별 말단 좌표 도출]
    % ★ 링크 1의 말단 좌표 (x1, y1) -> 관절 2 위치
    x1 = l1 * cos(theta1); 
    y1 = l1 * sin(theta1);
    
    % 링크 2의 말단 좌표 (x2, y2) -> 핸드 위치
    x2 = x1 + l2 * cos(theta1 + theta2_fixed); 
    y2 = y1 + l2 * sin(theta1 + theta2_fixed);
    phi = theta1 + theta2_fixed;
    
    % 링크 1 말단 데이터 누적 기록
    x1_history = [x1_history, x1];
    y1_history = [y1_history, y1];
    
    %% [SCREEN 01 - 전체 로봇 그래픽 갱신]
    for k = 1:5
        set(link1_layers(k), 'XData', [0, x1], 'YData', [0, y1]);
        set(link2_layers(k), 'XData', [x1, x2], 'YData', [y1, y2]);
    end
    set(h_j2_cap, 'XData', x1, 'YData', y1);
    
    % 집게 방위각 매핑 변환
    R = [cos(phi), -sin(phi); sin(phi), cos(phi)];
    wr_g = R * [wrist_x; wrist_y] + [x2; y2];
    cl_L_g = R * [claw_base_x; claw_base_y] + [x2; y2]; cl_R_g = R * [claw_base_x; -claw_base_y] + [x2; y2];
    pd_L_g = R * [pad_base_x; pad_base_y] + [x2; y2];   pd_R_g = R * [pad_base_x; -pad_base_y] + [x2; y2];
    
    set(h_wrist, 'XData', wr_g(1,:), 'YData', wr_g(2,:));
    set(h_claw_L, 'XData', cl_L_g(1,:), 'YData', cl_L_g(2,:)); set(h_claw_R, 'XData', cl_R_g(1,:), 'YData', cl_R_g(2,:));
    set(h_pad_L, 'XData', pd_L_g(1,:), 'YData', pd_L_g(2,:));   set(h_pad_R, 'XData', pd_R_g(1,:), 'YData', pd_R_g(2,:));
    
    % SCREEN 01용 링크 1 말단 실시간 궤적 라인 업데이트
    set(link1_trace_ax1, 'XData', x1_history, 'YData', y1_history);
    
    %% [SCREEN 02 - 링크 1 말단만의 독립 궤적 드로잉]
    set(link1_trace_ax2, 'XData', x1_history, 'YData', y1_history);
    set(h_joint_dot, 'XData', x1, 'YData', y1); 
    
    %% HUD 대시보드 갱신
    hud_string = sprintf(['  [LINK-1 TARGET]\n  ------------------\n  Locked \\theta_2: %3d^o\n  Active \\theta_1: %5.1f^o\n\n  [JOINT 2 COORD]\n  X1 Position: %5.2f m\n  Y1 Position: %5.2f m'], ...
                           theta2_fixed_deg, rad2deg(theta1), x1, y1);
    set(hud_box, 'String', hud_string);
    
    %% 프레임 동기화 및 딜레이 속도 제어
    drawnow;
    pause(0.015);
end