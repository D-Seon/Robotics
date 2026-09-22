% Fixed-Theta1 (0 deg) with Full-Circle Positive-Theta2 Robot Hand Trajectory (MATLAB R2026a)
% Synchronized Initial State: x = 2.0, y = 0 (theta1 = 0, theta2 = 0)
clear; clc; close all;

%% 1. 로봇 물리 및 초기 자세 동기화 고정각 설정
l1 = 1.0; % 링크 1 길이 (m)
l2 = 1.0; % 링크 2 길이 (m)

% ★ 초기 위치 x=2, y=0 자세를 만족하기 위해 \theta_1을 0도로 완벽 고정합니다.
theta1_fixed_deg = 0; 
theta1_fixed = deg2rad(theta1_fixed_deg);

% ★ 관절 2 (\theta_2) 구동 범위 설정: 양의 해 영역을 따라 한 바퀴 전체(0에서 +2*pi)를 회전하도록 설정
num_steps = 240;
theta2_range = linspace(0, 2*pi, num_steps);

%% 2. 시각화 그래픽스 환경 구축 (듀얼 가로형 레이아웃)
fig = figure('Name', 'Synchronized \theta_1 Control & Full-Circle +\theta_2 Analysis Dashboard', ...
             'NumberTitle', 'off', ...
             'Color', [0.08, 0.09, 0.11], ...
             'Position', [100, 200, 1250, 550]);

%% 3. [좌측 스크린 - 물리 공간 및 링크 2 말단(핸드) 궤적]
ax1 = subplot(1, 2, 1, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax1, 'on'); hold(ax1, 'on'); axis(ax1, 'equal');
xlim(ax1, [-2.2, 2.2]); ylim(ax1, [-2.2, 2.2]);
title(ax1, sprintf('SCREEN 01: Hand Trajectory (Fixed \\theta_1 = %d^\\circ)', theta1_fixed_deg), ...
      'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% ★ 링크 2의 말단(핸드)이 그리는 실시간 궤적선 (네온 시안 컬러)
hand_trace = plot(ax1, NaN, NaN, '-', 'Color', [0.0, 0.95, 1.0], 'LineWidth', 2.5);

% 산업용 베이스 받침대 드로잉
base_plate_x = [-0.45, 0.45, 0.30, -0.30]; base_plate_y = [-0.60, -0.60, -0.15, -0.15];
base_neck_x = [-0.15, 0.15, 0.10, -0.10];  base_neck_y = [-0.15, -0.15, 0.0, 0.0];
fill(ax1, base_plate_x, base_plate_y, [0.18, 0.20, 0.23], 'EdgeColor', [0.1, 0.11, 0.12], 'LineWidth', 2);
fill(ax1, base_neck_x, base_neck_y, [0.55, 0.58, 0.62], 'EdgeColor', [0.3, 0.32, 0.35], 'LineWidth', 1.5);
rectangle(ax1, 'Position', [-0.18, -0.18, 0.36, 0.36], 'Curvature', [1, 1], 'FaceColor', [0.35, 0.38, 0.42], 'LineWidth', 1.5);

%% 4. [우측 스크린 - 실시간 관절 각도 변위 차트]
ax2 = subplot(1, 2, 2, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax2, 'on'); hold(ax2, 'on');
xlim(ax2, [0, num_steps]); ylim(ax2, [-10, 370]); % 양의 영역 360도 분석을 위해 y축 범위 세팅
xlabel(ax2, 'Simulation Time Step', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
ylabel(ax2, 'Joint Angles (degree)', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
title(ax2, 'SCREEN 02: Real-Time Joint Curves', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 실시간 궤적 플롯 (Line 속성 오류 방지 문법)
curve_theta1 = plot(ax2, NaN, NaN, '--', 'Color', [1.0, 0.4, 0.0], 'LineWidth', 2.0, 'DisplayName', '\theta_1 (Locked 0^o)');
curve_theta2 = plot(ax2, NaN, NaN, '-', 'Color', [0.0, 1.0, 0.5], 'LineWidth', 2.5, 'DisplayName', '\theta_2 (Active Positive)');
legend(ax2, 'TextColor', [0.8, 0.85, 0.9], 'Color', [0.15, 0.17, 0.22], 'EdgeColor', [0.3, 0.35, 0.45]);

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

%% 8. 순기구학 연동 실시간 구동 제어 루프
x2_history = []; y2_history = [];
t1_history = []; t2_history = [];
time_axis = [];

for i = 1:num_steps
    % 현재 프레임의 관절각 상태 매핑
    theta1 = theta1_fixed; 
    theta2 = theta2_range(i); % 0도에서 +360도(+2*pi)까지 순회
    
    % 차트용 히스토리 배열 누적
    time_axis = [time_axis, i];
    t1_history = [t1_history, theta1_fixed_deg];
    t2_history = [t2_history, rad2deg(theta2)];
    
    %% [순기구학 수학 연산]
    % 링크 1 끝단 좌표 (0도로 완전히 누워 X축 상의 (1.0, 0) 위치에 고정)
    x1 = l1 * cos(theta1); 
    y1 = l1 * sin(theta1);
    
    % 링크 2 말단(로봇 핸드) 좌표 계산
    x2 = x1 + l2 * cos(theta1 + theta2); 
    y2 = y1 + l2 * sin(theta1 + theta2);
    phi = theta1 + theta2;
    
    % 로봇 핸드 말단부의 시뮬레이션 궤적 저장
    x2_history = [x2_history, x2];
    y2_history = [y2_history, y2];
    
    %% [SCREEN 01 - 전체 그래픽 요소 리프레시]
    for k = 1:5
        set(link1_layers(k), 'XData', [0, x1], 'YData', [0, y1]);
        set(link2_layers(k), 'XData', [x1, x2], 'YData', [y1, y2]);
    end
    set(h_j2_cap, 'XData', x1, 'YData', y1);
    
    % 집게 오프셋 변환 행렬 매핑
    R = [cos(phi), -sin(phi); sin(phi), cos(phi)];
    wr_g = R * [wrist_x; wrist_y] + [x2; y2];
    cl_L_g = R * [claw_base_x; claw_base_y] + [x2; y2]; cl_R_g = R * [claw_base_x; -claw_base_y] + [x2; y2];
    pd_L_g = R * [pad_base_x; pad_base_y] + [x2; y2];   pd_R_g = R * [pad_base_x; -pad_base_y] + [x2; y2];
    
    set(h_wrist, 'XData', wr_g(1,:), 'YData', wr_g(2,:));
    set(h_claw_L, 'XData', cl_L_g(1,:), 'YData', cl_L_g(2,:)); set(h_claw_R, 'XData', cl_R_g(1,:), 'YData', cl_R_g(2,:));
    set(h_pad_L, 'XData', pd_L_g(1,:), 'YData', pd_L_g(2,:));   set(h_pad_R, 'XData', pd_R_g(1,:), 'YData', pd_R_g(2,:));
    
    % ★ 링크 2 말단부(집게 핸드) 궤적 실선 가이드 라인 업데이트
    set(hand_trace, 'XData', x2_history, 'YData', y2_history);
    
    %% [SCREEN 02 - 우측 차트 실시간 드로잉 갱신]
    set(curve_theta1, 'XData', time_axis, 'YData', t1_history);
    set(curve_theta2, 'XData', time_axis, 'YData', t2_history);
    
    %% HUD 대시보드 데이터 출력
    hud_string = sprintf(['  [MONITOR HUD]\n  ------------------\n  Start At: (2.0, 0)\n  Locked \\theta_1:   %2d^o\n  Active \\theta_2: +%5.1f^o'], ...
                           theta1_fixed_deg, rad2deg(theta2));
    set(hud_box, 'String', hud_string);
    
    %% 애니메이션 동기화 프레임 속도 제어
    drawnow;
    pause(0.015);
end
