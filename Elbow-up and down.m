% Dual-Screen (Subplot) 2-Link Manipulator Simulation (MATLAB R2026a)
clear; clc; close all;

%% 1. 로봇 물리 및 궤적 파라미터 설정
l1 = 1.0; % 링크 1 길이 (m)
l2 = 1.0; % 링크 2 길이 (m)

% 타원 궤적 파라미터 (x^2/2^2 + y^2/1.3^2 = 1)
a = 2.0;  % 장반경
b = 1.3;  % 단반경

num_steps = 240;
t = linspace(0, 2*pi, num_steps);

X_traj = a * cos(t);
Y_traj = b * sin(t);

%% 2. 메인 피겨 창 설정 (가로로 긴 듀얼 스크린 레이아웃)
fig = figure('Name', 'Dual-Screen Inverse Kinematics Analysis', ...
             'NumberTitle', 'off', ...
             'Color', [0.08, 0.09, 0.11], ...
             'Position', [100, 200, 1200, 550]); % 창 크기를 가로로 넓게 설정

%% 3. [좌측 스크린 - Elbow-Down (-\theta_2) 은색 로봇 암]
ax1 = subplot(1, 2, 1, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax1, 'on'); hold(ax1, 'on'); axis(ax1, 'equal');
xlim(ax1, [-2.5, 2.5]); ylim(ax1, [-2.5, 2.5]);
title(ax1, 'SCREEN 01: Elbow-Down Solution (-\theta_2)', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');
plot(ax1, X_traj, ':', 'Color', [0.0, 0.8, 1.0], 'LineWidth', 1.5); % 타원 가이드
hand_trace_D = plot(ax1, X_traj(1), Y_traj(1), '-', 'Color', [1.0, 0.2, 0.6], 'LineWidth', 2.0); % 궤적 히스토리

%% 4. [우측 스크린 - Elbow-Up (+\theta_2) 금색 로봇 암]
ax2 = subplot(1, 2, 2, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax2, 'on'); hold(ax2, 'on'); axis(ax2, 'equal');
xlim(ax2, [-2.5, 2.5]); ylim(ax2, [-2.5, 2.5]);
title(ax2, 'SCREEN 02: Elbow-Up Solution (+\theta_2)', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');
plot(ax2, X_traj, ':', 'Color', [0.0, 0.8, 1.0], 'LineWidth', 1.5); % 타원 가이드
hand_trace_U = plot(ax2, X_traj(1), Y_traj(1), '-', 'Color', [1.0, 0.2, 0.6], 'LineWidth', 2.0); % 궤적 히스토리

%% 5. 양쪽 스크린 고정용 베이스 받침대 (Static Base Stands) 그리기
base_plate_x = [-0.45, 0.45, 0.30, -0.30]; base_plate_y = [-0.60, -0.60, -0.15, -0.15];
base_neck_x = [-0.15, 0.15, 0.10, -0.10];  base_neck_y = [-0.15, -0.15, 0.0, 0.0];

% 왼쪽 스크린 베이스
fill(ax1, base_plate_x, base_plate_y, [0.18, 0.20, 0.23], 'EdgeColor', [0.1, 0.11, 0.12], 'LineWidth', 2);
fill(ax1, base_neck_x, base_neck_y, [0.55, 0.58, 0.62], 'EdgeColor', [0.3, 0.32, 0.35], 'LineWidth', 1.5);
rectangle(ax1, 'Position', [-0.18, -0.18, 0.36, 0.36], 'Curvature', [1, 1], 'FaceColor', [0.35, 0.38, 0.42], 'LineWidth', 1.5);

% 오른쪽 스크린 베이스
fill(ax2, base_plate_x, base_plate_y, [0.18, 0.20, 0.23], 'EdgeColor', [0.1, 0.11, 0.12], 'LineWidth', 2);
fill(ax2, base_neck_x, base_neck_y, [0.55, 0.58, 0.62], 'EdgeColor', [0.3, 0.32, 0.35], 'LineWidth', 1.5);
rectangle(ax2, 'Position', [-0.18, -0.18, 0.36, 0.36], 'Curvature', [1, 1], 'FaceColor', [0.35, 0.38, 0.42], 'LineWidth', 1.5);

%% 6. 메탈릭 실버(좌) 및 아노다이징 골드(우) 암 레이어 빌드
silver_colors = {[0.18, 0.19, 0.20], [0.48, 0.50, 0.53], [0.72, 0.75, 0.78], [0.90, 0.93, 0.96], [1.0, 1.0, 1.0]};
gold_colors   = {[0.25, 0.18, 0.05], [0.70, 0.52, 0.15], [0.88, 0.71, 0.28], [0.98, 0.88, 0.55], [1.0, 0.98, 0.85]};
widths = [26, 21, 14, 7, 2.5];

link1_layers_down = []; link2_layers_down = [];
link1_layers_up = [];   link2_layers_up = [];

for k = 1:5
    link1_layers_down(k) = plot(ax1, [0, 0], [0, 0], 'Color', silver_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
    link2_layers_down(k) = plot(ax1, [0, 0], [0, 0], 'Color', silver_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
    
    link1_layers_up(k)   = plot(ax2, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
    link2_layers_up(k)   = plot(ax2, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
end

%% 7. 듀얼 독립 집게(Gripper) 및 피벗 볼트 캡 패치 구성
wrist_x = [-0.03,  0.03,  0.03, -0.03]; wrist_y = [-0.05, -0.05,  0.05,  0.05];
claw_base_x = [0.03, 0.11, 0.19, 0.23, 0.20, 0.12, 0.03]; claw_base_y = [0.05, 0.09, 0.07, 0.02, -0.01, 0.03, 0.01];
pad_base_x = [0.15, 0.23, 0.20]; pad_base_y = [0.07, 0.02, -0.01];

% --- 좌측 은색 집게 ---
h_wrist_D  = patch(ax1, 'XData', wrist_x, 'YData', wrist_y, 'FaceColor', [0.25, 0.27, 0.30]);
h_claw_L_D = patch(ax1, 'XData', claw_base_x, 'YData', claw_base_y, 'FaceColor', [0.75, 0.78, 0.82]);
h_claw_R_D = patch(ax1, 'XData', claw_base_x, 'YData', -claw_base_y, 'FaceColor', [0.75, 0.78, 0.82]);
h_pad_L_D  = patch(ax1, 'XData', pad_base_x, 'YData', pad_base_y, 'FaceColor', [1.0, 0.45, 0.0], 'EdgeColor', 'none');
h_pad_R_D  = patch(ax1, 'XData', pad_base_x, 'YData', -pad_base_y, 'FaceColor', [1.0, 0.45, 0.0], 'EdgeColor', 'none');

% --- 우측 금색 집게 ---
h_wrist_U  = patch(ax2, 'XData', wrist_x, 'YData', wrist_y, 'FaceColor', [0.30, 0.25, 0.15]);
h_claw_L_U = patch(ax2, 'XData', claw_base_x, 'YData', claw_base_y, 'FaceColor', [0.85, 0.75, 0.45]);
h_claw_R_U = patch(ax2, 'XData', claw_base_x, 'YData', -claw_base_y, 'FaceColor', [0.85, 0.75, 0.45]);
h_pad_L_U  = patch(ax2, 'XData', pad_base_x, 'YData', pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');
h_pad_R_U  = patch(ax2, 'XData', pad_base_x, 'YData', -pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');

% 조인트 볼트 헤드
h_j2_cap_D = plot(ax1, 0, 0, 'o', 'MarkerSize', 11, 'MarkerFaceColor', [0.8, 0.82, 0.85], 'MarkerEdgeColor', [0.1, 0.1, 0.1]);
h_j2_cap_U = plot(ax2, 0, 0, 'o', 'MarkerSize', 11, 'MarkerFaceColor', [0.9, 0.75, 0.20], 'MarkerEdgeColor', [0.1, 0.1, 0.1]);

%% 8. 각각의 스크린을 위한 독립 HUD 텍스트 박스 배치
hud_box_D = annotation('textbox', [0.14, 0.68, 0.16, 0.18], 'Color', [0.0, 0.95, 1.0], ...
                       'BackgroundColor', [0.15, 0.17, 0.22, 0.85], 'EdgeColor', [0.0, 0.5, 0.7], ...
                       'FontName', 'Courier New', 'FontSize', 9, 'FontWeight', 'bold');
                   
hud_box_U = annotation('textbox', [0.55, 0.68, 0.16, 0.18], 'Color', [0.98, 0.88, 0.55], ...
                       'BackgroundColor', [0.15, 0.17, 0.22, 0.85], 'EdgeColor', [0.6, 0.4, 0.1], ...
                       'FontName', 'Courier New', 'FontSize', 9, 'FontWeight', 'bold');

%% 9. 실시간 분할 스크린 구동 제어 루프
x_history = []; y_history = [];

for i = 1:num_steps
    x = X_traj(i);
    y = Y_traj(i);
    
    %% 역기구학 해 분리 연산
    D = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2);
    if D > 1, D = 1; elseif D < -1, D = -1; end
    
    % SCREEN 01용 해 (Elbow-Down)
    theta2_D = -acos(D); 
    theta1_D = atan2(y, x) - atan2(l2 * sin(theta2_D), l1 + l2 * cos(theta2_D));
    
    % SCREEN 02용 해 (Elbow-Up)
    theta2_U = acos(D); 
    theta1_U = atan2(y, x) - atan2(l2 * sin(theta2_U), l1 + l2 * cos(theta2_U));
    
    %% 글로벌 좌표 유도
    x1_D = l1 * cos(theta1_D); y1_D = l1 * sin(theta1_D);
    x2_D = x1_D + l2 * cos(theta1_D + theta2_D); y2_D = y1_D + l2 * sin(theta1_D + theta2_D);
    phi_D = theta1_D + theta2_D;
    
    x1_U = l1 * cos(theta1_U); y1_U = l1 * sin(theta1_U);
    x2_U = x1_U + l2 * cos(theta1_U + theta2_U); y2_U = y1_U + l2 * sin(theta1_U + theta2_U);
    phi_U = theta1_U + theta2_U;
    
    %% [SCREEN 01 그래픽 갱신]
    for k = 1:5
        set(link1_layers_down(k), 'XData', [0, x1_D], 'YData', [0, y1_D]);
        set(link2_layers_down(k), 'XData', [x1_D, x2_D], 'YData', [y1_D, y2_D]);
    end
    set(h_j2_cap_D, 'XData', x1_D, 'YData', y1_D);
    
    %% [SCREEN 02 그래픽 갱신]
    for k = 1:5
        set(link1_layers_up(k), 'XData', [0, x1_U], 'YData', [0, y1_U]);
        set(link2_layers_up(k), 'XData', [x1_U, x2_U], 'YData', [y1_U, y2_U]);
    end
    set(h_j2_cap_U, 'XData', x1_U, 'YData', y1_U);
    
    %% 집게 애니메이션 계산 및 매핑
    grip_control = 0.65 + 0.35 * sin(t(i));
    claw_dyn_y = claw_base_y * grip_control; pad_dyn_y = pad_base_y * grip_control;
    
    R_D = [cos(phi_D), -sin(phi_D); sin(phi_D), cos(phi_D)];
    R_U = [cos(phi_U), -sin(phi_U); sin(phi_U), cos(phi_U)];
    
    % 좌측 스크린 집게 변환
    wr_g_D = R_D * [wrist_x; wrist_y] + [x2_D; y2_D];
    cl_L_g_D = R_D * [claw_base_x; claw_dyn_y] + [x2_D; y2_D]; cl_R_g_D = R_D * [claw_base_x; -claw_dyn_y] + [x2_D; y2_D];
    pd_L_g_D = R_D * [pad_base_x; pad_dyn_y] + [x2_D; y2_D];   pd_R_g_D = R_D * [pad_base_x; -pad_dyn_y] + [x2_D; y2_D];
    
    set(h_wrist_D, 'XData', wr_g_D(1,:), 'YData', wr_g_D(2,:));
    set(h_claw_L_D, 'XData', cl_L_g_D(1,:), 'YData', cl_L_g_D(2,:)); set(h_claw_R_D, 'XData', cl_R_g_D(1,:), 'YData', cl_R_g_D(2,:));
    set(h_pad_L_D, 'XData', pd_L_g_D(1,:), 'YData', pd_L_g_D(2,:));   set(h_pad_R_D, 'XData', pd_R_g_D(1,:), 'YData', pd_R_g_D(2,:));
    
    % 우측 스크린 집게 변환
    wr_g_U = R_U * [wrist_x; wrist_y] + [x2_U; y2_U];
    cl_L_g_U = R_U * [claw_base_x; claw_dyn_y] + [x2_U; y2_U]; cl_R_g_U = R_U * [claw_base_x; -claw_dyn_y] + [x2_U; y2_U];
    pd_L_g_U = R_U * [pad_base_x; pad_dyn_y] + [x2_U; y2_U];   pd_R_g_U = R_U * [pad_base_x; -pad_dyn_y] + [x2_U; y2_U];
    
    set(h_wrist_U, 'XData', wr_g_U(1,:), 'YData', wr_g_U(2,:));
    set(h_claw_L_U, 'XData', cl_L_g_U(1,:), 'YData', cl_L_g_U(2,:)); set(h_claw_R_U, 'XData', cl_R_g_U(1,:), 'YData', cl_R_g_U(2,:));
    set(h_pad_L_U, 'XData', pd_L_g_U(1,:), 'YData', pd_L_g_U(2,:));   set(h_pad_R_U, 'XData', pd_R_g_U(1,:), 'YData', pd_R_g_U(2,:));
    
    %% 히스토리 궤적 플롯 누적 업데이트
    x_history = [x_history, x]; y_history = [y_history, y];
    set(hand_trace_D, 'XData', x_history, 'YData', y_history);
    set(hand_trace_U, 'XData', x_history, 'YData', y_history);
    
    %% 개별 스크린 전용 HUD 데이터 갱신
    hud_string_D = sprintf(['  [CH.01 ELBOW-DOWN]\n  ------------------\n  X_task  : %5.2f m\n  Y_task  : %5.2f m\n  Theta 1 : %5.1f deg\n  Theta 2 : %5.1f deg'], ...
                           x, y, rad2deg(theta1_D), rad2deg(theta2_D));
    set(hud_box_D, 'String', hud_string_D);
    
    hud_string_U = sprintf(['  [CH.02 ELBOW-UP]\n  ------------------\n  X_task  : %5.2f m\n  Y_task  : %5.2f m\n  Theta 1 : %5.1f deg\n  Theta 2 : %5.1f deg'], ...
                           x, y, rad2deg(theta1_U), rad2deg(theta2_U));
    set(hud_box_U, 'String', hud_string_U);
    
    %% 드로잉 리프레시 및 타임 딜레이
    drawnow;
    pause(0.1);
end