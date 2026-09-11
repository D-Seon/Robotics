% Link-1 End Joint (x1, y1) Trajectory Analysis (MATLAB R2026a)
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

%% 2. 메인 피겨 창 설정 (듀얼 가로형 레이아웃)
fig = figure('Name', 'Link-1 End Joint Trajectory HUD', ...
             'NumberTitle', 'off', ...
             'Color', [0.08, 0.09, 0.11], ...
             'Position', [100, 200, 1250, 550]);

%% 3. [좌측 스크린 - 전체 물리 공간 및 링크 1 궤적 동시 표기]
ax1 = subplot(1, 2, 1, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax1, 'on'); hold(ax1, 'on'); axis(ax1, 'equal');
xlim(ax1, [-2.5, 2.5]); ylim(ax1, [-2.5, 2.5]);
title(ax1, 'SCREEN 01: Workspace with Link-1 Trajectory', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 핸드가 따라가는 타원 목표 가이드라인 (하늘색 점선)
plot(ax1, X_traj, ':', 'Color', [0.0, 0.8, 1.0], 'LineWidth', 1.2);

% ★ 링크 1 끝단(관절 2)이 그리는 궤적 라인 (네온 그린 컬러)
link1_trace_ax1 = plot(ax1, NaN, NaN, '-', 'Color', [0.0, 1.0, 0.5], 'LineWidth', 2.5);

% 베이스 스탠드 플롯
base_plate_x = [-0.45, 0.45, 0.30, -0.30]; base_plate_y = [-0.60, -0.60, -0.15, -0.15];
base_neck_x = [-0.15, 0.15, 0.10, -0.10];  base_neck_y = [-0.15, -0.15, 0.0, 0.0];
fill(ax1, base_plate_x, base_plate_y, [0.18, 0.20, 0.23], 'EdgeColor', [0.1, 0.11, 0.12], 'LineWidth', 2);
fill(ax1, base_neck_x, base_neck_y, [0.55, 0.58, 0.62], 'EdgeColor', [0.3, 0.32, 0.35], 'LineWidth', 1.5);
rectangle(ax1, 'Position', [-0.18, -0.18, 0.36, 0.36], 'Curvature', [1, 1], 'FaceColor', [0.35, 0.38, 0.42], 'LineWidth', 1.5);

%% 4. [우측 스크린 - 링크 1 끝단(관절 2)만의 독립 궤적 현미경 뷰]
ax2 = subplot(1, 2, 2, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax2, 'on'); hold(ax2, 'on'); axis(ax2, 'equal');
xlim(ax2, [-1.2, 1.2]); ylim(ax2, [-1.2, 1.2]); % 링크 1 길이가 1m이므로 궤적 범위에 맞춤
xlabel(ax2, 'X1 Position (m)', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
ylabel(ax2, 'Y1 Position (m)', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
title(ax2, 'SCREEN 02: Pure Link-1 End Trajectory (x_1, y_1)', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 링크 1 중심 원점 가이드라인 (반지름 1인 원형 물리 한계선)
th_circle = linspace(0, 2*pi, 100);
plot(ax2, cos(th_circle), sin(th_circle), '--', 'Color', [0.3, 0.35, 0.45], 'LineWidth', 1);

% 우측 창에서 실시간으로 그려질 링크 1 궤적선 (마젠타 핑크 컬러)
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

%% 6. 집게(Gripper) 및 하이라이트 조인트 볼트 세팅
wrist_x = [-0.03,  0.03,  0.03, -0.03]; wrist_y = [-0.05, -0.05,  0.05,  0.05];
claw_base_x = [0.03, 0.11, 0.19, 0.23, 0.20, 0.12, 0.03]; claw_base_y = [0.05, 0.09, 0.07, 0.02, -0.01, 0.03, 0.01];
pad_base_x = [0.15, 0.23, 0.20]; pad_base_y = [0.07, 0.02, -0.01];

h_wrist  = patch(ax1, 'XData', wrist_x, 'YData', wrist_y, 'FaceColor', [0.30, 0.25, 0.15]);
h_claw_L = patch(ax1, 'XData', claw_base_x, 'YData', claw_base_y, 'FaceColor', [0.85, 0.75, 0.45]);
h_claw_R = patch(ax1, 'XData', claw_base_x, 'YData', -claw_base_y, 'FaceColor', [0.85, 0.75, 0.45]);
h_pad_L  = patch(ax1, 'XData', pad_base_x, 'YData', pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');
h_pad_R  = patch(ax1, 'XData', pad_base_x, 'YData', -pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');

h_j2_cap = plot(ax1, 0, 0, 'o', 'MarkerSize', 11, 'MarkerFaceColor', [0.9, 0.75, 0.20], 'MarkerEdgeColor', [0.1, 0.1, 0.1]);

%% 7. 대시보드 HUD 텍스트 박스 배치
hud_box = annotation('textbox', [0.14, 0.68, 0.17, 0.18], 'Color', [0.98, 0.88, 0.55], ...
                     'BackgroundColor', [0.15, 0.17, 0.22, 0.85], 'EdgeColor', [0.6, 0.4, 0.1], ...
                     'FontName', 'Courier New', 'FontSize', 9, 'FontWeight', 'bold');

%% 8. 실시간 연동 시뮬레이션 루프
x1_history = []; y1_history = []; % 링크 1 끝단 좌표 저장 배열

for i = 1:num_steps
    x = X_traj(i);
    y = Y_traj(i);
    
    %% [역기구학 연산 - 조건: 세타2가 양(+)의 해]
    D = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2);
    if D > 1, D = 1; elseif D < -1, D = -1; end
    
    theta2 = acos(D); % 양의 해 (Elbow-Up) 고정
    theta1 = atan2(y, x) - atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));
    
    %% [순기구학 글로벌 좌표 매핑]
    % ★ 링크 1의 끝단 좌표 (x1, y1) 계산
    x1 = l1 * cos(theta1); 
    y1 = l1 * sin(theta1);
    
    % 링크 2의 끝단(핸드) 좌표 (x2, y2) 계산
    x2 = x1 + l2 * cos(theta1 + theta2); 
    y2 = y1 + l2 * sin(theta1 + theta2);
    phi = theta1 + theta2;
    
    % 링크 1 끝단 히스토리 누적
    x1_history = [x1_history, x1];
    y1_history = [y1_history, y1];
    
    %% [SCREEN 01 - 전체 그래픽 요소 리프레시]
    for k = 1:5
        set(link1_layers(k), 'XData', [0, x1], 'YData', [0, y1]);
        set(link2_layers(k), 'XData', [x1, x2], 'YData', [y1, y2]);
    end
    set(h_j2_cap, 'XData', x1, 'YData', y1);
    
    % 집게 기하 렌더링
    grip_control = 0.65 + 0.35 * sin(t(i));
    claw_dyn_y = claw_base_y * grip_control; pad_dyn_y = pad_base_y * grip_control;
    R = [cos(phi), -sin(phi); sin(phi), cos(phi)];
    
    wr_g = R * [wrist_x; wrist_y] + [x2; y2];
    cl_L_g = R * [claw_base_x; claw_dyn_y] + [x2; y2]; cl_R_g = R * [claw_base_x; -claw_dyn_y] + [x2; y2];
    pd_L_g = R * [pad_base_x; pad_dyn_y] + [x2; y2];   pd_R_g = R * [pad_base_x; -pad_dyn_y] + [x2; y2];
    
    set(h_wrist, 'XData', wr_g(1,:), 'YData', wr_g(2,:));
    set(h_claw_L, 'XData', cl_L_g(1,:), 'YData', cl_L_g(2,:)); set(h_claw_R, 'XData', cl_R_g(1,:), 'YData', cl_R_g(2,:));
    set(h_pad_L, 'XData', pd_L_g(1,:), 'YData', pd_L_g(2,:));   set(h_pad_R, 'XData', pd_R_g(1,:), 'YData', pd_R_g(2,:));
    
    % SCREEN 01용 링크 1 끝단 실시간 궤적 업데이트
    set(link1_trace_ax1, 'XData', x1_history, 'YData', y1_history);
    
    %% [SCREEN 02 - 링크 1 끝단만의 궤적 곡선 드로잉]
    set(link1_trace_ax2, 'XData', x1_history, 'YData', y1_history);
    set(h_joint_dot, 'XData', x1, 'YData', y1); % 현재 링크1 끝점 마커 위치
    
    %% HUD 데이터 대시보드 출력
    hud_string = sprintf(['  [MONITOR SYSTEM]\n  ------------------\n  Hand X  : %5.2f m\n  Hand Y  : %5.2f m\n\n  [LINK 1 END]\n  Joint2 X: %5.2f m\n  Joint2 Y: %5.2f m'], ...
                           x2, y2, x1, y1);
    set(hud_box, 'String', hud_string);
    
    %% 프레임 동기화 및 딜레이
    drawnow;
    pause(0.015);
end