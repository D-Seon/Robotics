% Pure Gripper Tracking Simulation without Links (MATLAB R2026a)
% Synchronized Initial State: x = 2.0, y = 0 (theta1 = 0, theta2 = 0)
clear; clc; close all;

%% 1. 로봇 물리 및 동시 구동 각도 범위 설정
l1 = 1.0; % 링크 1 길이 (m)
l2 = 1.0; % 링크 2 길이 (m)

num_steps = 300; % 정교한 곡선 묘사를 위해 스텝 수 최적화

% 초기 자세 (0, 0)에서 출발하여 두 관절이 동시에 회전하도록 속도 비율 차등 주입
theta1_range = linspace(0, 2*pi, num_steps);
theta2_range = linspace(0, 4*pi, num_steps);

%% 2. 시각화 그래픽스 환경 구축 (듀얼 가로형 레이아웃)
fig = figure('Name', 'Pure Gripper Navigation HUD', ...
             'NumberTitle', 'off', ...
             'Color', [0.08, 0.09, 0.11], ...
             'Position', [100, 200, 1250, 550]);

%% 3. [좌측 스크린 - 링크 없이 오직 집게와 핸드 궤적만 시각화]
ax1 = subplot(1, 2, 1, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax1, 'on'); hold(ax1, 'on'); axis(ax1, 'equal');
xlim(ax1, [-2.5, 2.5]); ylim(ax1, [-2.5, 2.5]);
title(ax1, 'SCREEN 01: Pure Gripper Path Navigation', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% ★ 두 관절의 기하학적 합성으로 그려질 실시간 핸드 궤적선 (네온 시안 컬러)
hand_trace = plot(ax1, NaN, NaN, '-', 'Color', [0.0, 0.95, 1.0], 'LineWidth', 2.5);

%% 4. [우측 스크린 - 실시간 듀얼 관절 각도 변위 차트]
ax2 = subplot(1, 2, 2, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax2, 'on'); hold(ax2, 'on');
xlim(ax2, [0, num_steps]); ylim(ax2, [-10, 730]); 
xlabel(ax2, 'Simulation Time Step', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
ylabel(ax2, 'Joint Angles (degree)', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
title(ax2, 'SCREEN 02: Dual-Servo Motor Trajectory Curves', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 실시간 궤적 플롯 객체 선언 (DisplayName 속성 적용)
curve_theta1 = plot(ax2, NaN, NaN, '-', 'Color', [0.0, 1.0, 0.5], 'LineWidth', 2.2, 'DisplayName', '\theta_1 (Speed: 1x)');
curve_theta2 = plot(ax2, NaN, NaN, '-', 'Color', [1.0, 0.4, 0.0], 'LineWidth', 2.2, 'DisplayName', '\theta_2 (Speed: 2x)');
legend(ax2, 'TextColor', [0.8, 0.85, 0.9], 'Color', [0.15, 0.17, 0.22], 'EdgeColor', [0.3, 0.35, 0.45]);

%% 5. 링크 레이어를 제외한 집게(Gripper)만의 정교한 다각형 패치 구축
% 집게 로컬 기준점 (0,0) 구조 설계 데이터
wrist_x = [-0.03,  0.03,  0.03, -0.03]; wrist_y = [-0.05, -0.05,  0.05,  0.05];
claw_base_x = [0.03, 0.11, 0.19, 0.23, 0.20, 0.12, 0.03]; claw_base_y = [0.05, 0.09, 0.07, 0.02, -0.01, 0.03, 0.01];
pad_base_x = [0.15, 0.23, 0.20]; pad_base_y = [0.07, 0.02, -0.01];

% 아노다이징 골드 스타일 하이테크 집게 팩토리 패치 매핑 (ax1에 직접 빌드)
h_wrist  = patch(ax1, 'XData', wrist_x, 'YData', wrist_y, 'FaceColor', [0.30, 0.25, 0.15], 'EdgeColor', [0.1, 0.1, 0.1]);
h_claw_L = patch(ax1, 'XData', claw_base_x, 'YData', claw_base_y, 'FaceColor', [0.85, 0.75, 0.45], 'EdgeColor', [0.45, 0.38, 0.2]);
h_claw_R = patch(ax1, 'XData', claw_base_x, 'YData', -claw_base_y, 'FaceColor', [0.85, 0.75, 0.45], 'EdgeColor', [0.45, 0.38, 0.2]);
h_pad_L  = patch(ax1, 'XData', pad_base_x, 'YData', pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none'); 
h_pad_R  = patch(ax1, 'XData', pad_base_x, 'YData', -pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');

% 집게 중심 코어 유도용 피벗 캡 장식 (미니 테크니컬 포인트)
h_wrist_cap = plot(ax1, 0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.00, 0.80, 1.00], 'MarkerEdgeColor', [0.1, 0.1, 0.1], 'LineWidth', 1.2);

%% 6. 오버레이 HUD 정보창 레이아웃 배치
hud_box = annotation('textbox', [0.14, 0.72, 0.17, 0.14], 'Color', [0.0, 0.95, 1.0], ...
                     'BackgroundColor', [0.15, 0.17, 0.22, 0.85], 'EdgeColor', [0.0, 0.5, 0.7], ...
                     'FontName', 'Courier New', 'FontSize', 9, 'FontWeight', 'bold');

%% 7. 순기구학(Forward Kinematics) 기반 실시간 루프 애니메이션
x2_history = []; y2_history = [];
t1_history = []; t2_history = [];
time_axis = [];

for i = 1:num_steps
    % 매 프레임의 고유 모터 제어 각도 매핑
    theta1 = theta1_range(i);
    theta2 = theta2_range(i);
    
    % 차트용 데이터 누적
    time_axis = [time_axis, i];
    t1_history = [t1_history, rad2deg(theta1)];
    t2_history = [t2_history, rad2deg(theta2)];
    
    %% [순기구학 기반 집게 기준 중심 좌표 (x2, y2) 및 방위각 유도]
    % 링크는 그리지 않지만, 집게가 매달려 있을 가상의 공간 좌표 연산은 필수적입니다.
    x1 = l1 * cos(theta1); 
    y1 = l1 * sin(theta1);
    
    x2 = x1 + l2 * cos(theta1 + theta2); 
    y2 = y1 + l2 * sin(theta1 + theta2);
    phi = theta1 + theta2; % 집게의 최종 글로벌 회전 방위각
    
    % 궤적 저장 배열 업데이트
    x2_history = [x2_history, x2];
    y2_history = [y2_history, y2];
    
    %% [SCREEN 01 - 집게 그래픽스 좌표 행렬 실시간 동적 변환]
    % 집게가 허공에서 실시간으로 완벽하게 회전 및 이동할 수 있도록 2x2 회전 변환 행렬 매핑
    R = [cos(phi), -sin(phi); sin(phi), cos(phi)];
    
    wr_g   = R * [wrist_x; wrist_y] + [x2; y2];
    cl_L_g = R * [claw_base_x; claw_base_y] + [x2; y2];
    cl_R_g = R * [claw_base_x; -claw_base_y] + [x2; y2];
    pd_L_g = R * [pad_base_x; pad_base_y] + [x2; y2];
    pd_R_g = R * [pad_base_x; -pad_base_y] + [x2; y2];
    
    % 패치 객체에 변환 좌표 실시간 주입
    set(h_wrist, 'XData', wr_g(1,:), 'YData', wr_g(2,:));
    set(h_claw_L, 'XData', cl_L_g(1,:), 'YData', cl_L_g(2,:));
    set(h_claw_R, 'XData', cl_R_g(1,:), 'YData', cl_R_g(2,:));
    set(h_pad_L, 'XData', pd_L_g(1,:), 'YData', pd_L_g(2,:));
    set(h_pad_R, 'XData', pd_R_g(1,:), 'YData', pd_R_g(2,:));
    set(h_wrist_cap, 'XData', x2, 'YData', y2);
    
    % 집게가 남기는 복합 순기구학 궤적 실선 가이드라인 업데이트
    set(hand_trace, 'XData', x2_history, 'YData', y2_history);
    
    %% [SCREEN 02 - 우측 차트 변위 선도 업데이트]
    set(curve_theta1, 'XData', time_axis, 'YData', t1_history);
    set(curve_theta2, 'XData', time_axis, 'YData', t2_history);
    
    %% HUD 대시보드 스펙 정보 실시간 리프레시
    hud_string = sprintf(['  [GRIPPER HUD]\n  ------------------\n  Start At: (2.0, 0)\n\n  [NAV POSITION]\n  Coord X : %5.2f m\n  Coord Y : %5.2f m'], ...
                           x2, y2);
    set(hud_box, 'String', hud_string);
    
    %% 애니메이션 동기화 및 딜레이 스피드 제어
    drawnow;
    pause(0.015);
end