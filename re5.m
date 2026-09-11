% Full Planar Manipulator Simultaneous FK Simulation (MATLAB R2026a)
% Synchronized Initial State: x = 2.0, y = 0 (theta1 = 0, theta2 = 0)
clear; clc; close all;

%% 1. 로봇 물리 및 동시 구동 각도 범위 설정
l1 = 1.0; % 링크 1 길이 (m)
l2 = 1.0; % 링크 2 길이 (m)

num_steps = 300; % 복합 곡선을 정밀하게 표현하기 위해 프레임 수 확장

% ★ 초기 자세 (0, 0)에서 출발하여 두 모터가 동시에 회전합니다.
% 관절 1이 1바퀴(2*pi) 돌 때, 관절 2는 속도를 높여 2바퀴(4*pi)를 돌도록 조합했습니다.
theta1_range = linspace(0, 2*pi, num_steps);
theta2_range = linspace(0, 4*pi, num_steps);

%% 2. 시각화 그래픽스 환경 구축 (듀얼 가로형 레이아웃)
fig = figure('Name', 'Synchronized FK Full Manipulator HUD', ...
             'NumberTitle', 'off', ...
             'Color', [0.08, 0.09, 0.11], ...
             'Position', [100, 200, 1250, 550]);

%% 3. [좌측 스크린 - 물리 타스크 공간 시뮬레이터]
ax1 = subplot(1, 2, 1, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax1, 'on'); hold(ax1, 'on'); axis(ax1, 'equal');
xlim(ax1, [-2.5, 2.5]); ylim(ax1, [-2.5, 2.5]);
title(ax1, 'SCREEN 01: Full Mechanical Workspace', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% ★ 로봇 핸드(집게)가 순기구학 합성으로 그려나갈 실시간 궤적선 (네온 마젠타 컬러)
hand_trace = plot(ax1, NaN, NaN, '-', 'Color', [1.0, 0.2, 0.6], 'LineWidth', 2.5);

% 산업용 중후한 고정 베이스 받침대 플레이트 드로잉
base_plate_x = [-0.45, 0.45, 0.30, -0.30]; base_plate_y = [-0.60, -0.60, -0.15, -0.15];
base_neck_x = [-0.15, 0.15, 0.10, -0.10];  base_neck_y = [-0.15, -0.15, 0.0, 0.0];
fill(ax1, base_plate_x, base_plate_y, [0.18, 0.20, 0.23], 'EdgeColor', [0.1, 0.11, 0.12], 'LineWidth', 2);
fill(ax1, base_neck_x, base_neck_y, [0.55, 0.58, 0.62], 'EdgeColor', [0.3, 0.32, 0.35], 'LineWidth', 1.5);
rectangle(ax1, 'Position', [-0.18, -0.18, 0.36, 0.36], 'Curvature', [1, 1], 'FaceColor', [0.35, 0.38, 0.42], 'LineWidth', 1.5);

%% 4. [우측 스크린 - 실시간 듀얼 서보 모터 변위 차트]
ax2 = subplot(1, 2, 2, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax2, 'on'); hold(ax2, 'on');
xlim(ax2, [0, num_steps]); ylim(ax2, [-10, 730]); 
xlabel(ax2, 'Simulation Time Step', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
ylabel(ax2, 'Joint Angles (degree)', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
title(ax2, 'SCREEN 02: Dual-Motor Command Inputs', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 실시간 궤적 플롯 객체 선언 (R2026a 속성 인식을 위해 DisplayName 문법 적용)
curve_theta1 = plot(ax2, NaN, NaN, '-', 'Color', [0.0, 1.0, 0.5], 'LineWidth', 2.2, 'DisplayName', '\theta_1 (Speed: 1x)');
curve_theta2 = plot(ax2, NaN, NaN, '-', 'Color', [1.0, 0.4, 0.0], 'LineWidth', 2.2, 'DisplayName', '\theta_2 (Speed: 2x)');
legend(ax2, 'TextColor', [0.8, 0.85, 0.9], 'Color', [0.15, 0.17, 0.22], 'EdgeColor', [0.3, 0.35, 0.45]);

%% 5. 아노다이징 골드 메탈릭 [링크 1 & 링크 2] 레이어 빌드
gold_colors = {[0.25, 0.18, 0.05], [0.70, 0.52, 0.15], [0.88, 0.71, 0.28], [0.98, 0.88, 0.55], [1.0, 0.98, 0.85]};
widths = [26, 21, 14, 7, 2.5];

link1_layers = []; link2_layers = [];
for k = 1:5
    link1_layers(k) = plot(ax1, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
    link2_layers(k) = plot(ax1, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
end

%% 6. 요청 반영: 정교한 다각형 [집게(Gripper)] 패치 구조물 구축
% 집게 로컬 좌표 데이터 정의 (손목 브래킷 중심점 0,0 기준)
wrist_x = [-0.03,  0.03,  0.03, -0.03]; wrist_y = [-0.05, -0.05,  0.05,  0.05];
claw_base_x = [0.03, 0.11, 0.19, 0.23, 0.20, 0.12, 0.03]; claw_base_y = [0.05, 0.09, 0.07, 0.02, -0.01, 0.03, 0.01];
pad_base_x = [0.15, 0.23, 0.20]; pad_base_y = [0.07, 0.02, -0.01];

% 하이테크 골드-블루 조합 집게 패치 초기화 선언
h_wrist  = patch(ax1, 'XData', wrist_x, 'YData', wrist_y, 'FaceColor', [0.30, 0.25, 0.15], 'EdgeColor', [0.1, 0.1, 0.1]);
h_claw_L = patch(ax1, 'XData', claw_base_x, 'YData', claw_base_y, 'FaceColor', [0.85, 0.75, 0.45], 'EdgeColor', [0.45, 0.38, 0.2]);
h_claw_R = patch(ax1, 'XData', claw_base_x, 'YData', -claw_base_y, 'FaceColor', [0.85, 0.75, 0.45], 'EdgeColor', [0.45, 0.38, 0.2]);
h_pad_L  = patch(ax1, 'XData', pad_base_x, 'YData', pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none'); 
h_pad_R  = patch(ax1, 'XData', pad_base_x, 'YData', -pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');

% 핵심 조인트 마커 포인트 캡들
h_j2_cap = plot(ax1, 0, 0, 'o', 'MarkerSize', 11, 'MarkerFaceColor', [0.9, 0.75, 0.20], 'MarkerEdgeColor', [0.1, 0.1, 0.1]);
h_wrist_cap = plot(ax1, 0, 0, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.0, 0.9, 1.0], 'MarkerEdgeColor', [0.1, 0.1, 0.1]);

%% 7. 오버레이 HUD 정보창 레이아웃 배치
hud_box = annotation('textbox', [0.14, 0.68, 0.17, 0.18], 'Color', [0.98, 0.88, 0.55], ...
                     'BackgroundColor', [0.15, 0.17, 0.22, 0.85], 'EdgeColor', [0.6, 0.4, 0.1], ...
                     'FontName', 'Courier New', 'FontSize', 9, 'FontWeight', 'bold');

%% 8. 순기구학(FK) 실시간 렌더링 동시 구동 제어 루프
x2_history = []; y2_history = [];
t1_history = []; t2_history = [];
time_axis = [];

for i = 1:num_steps
    % 현재 프레임의 독립 관절 커맨드 각도 매핑
    theta1 = theta1_range(i);
    theta2 = theta2_range(i);
    
    % 우측 차트용 히스토리 기록
    time_axis = [time_axis, i];
    t1_history = [t1_history, rad2deg(theta1)];
    t2_history = [t2_history, rad2deg(theta2)];
    
    %% [★ 핵심 순기구학(Forward Kinematics) 기하 공식 적용]
    % 링크 1 끝단 좌표 (관절 2의 위치)
    x1 = l1 * cos(theta1); 
    y1 = l1 * sin(theta1);
    
    % 링크 2 끝단 좌표 (로봇 핸드 및 집게 기준 중심점 위치)
    x2 = x1 + l2 * cos(theta1 + theta2); 
    y2 = y1 + l2 * sin(theta1 + theta2);
    phi = theta1 + theta2; % 집게가 바라볼 글로벌 최종 지향 각도
    
    % 첫 프레임(i=1)일 때 x2 = 2.0, y2 = 0 상태에서 한 치의 오차 없이 깔끔하게 시작합니다.
    x2_history = [x2_history, x2];
    y2_history = [y2_history, y2];
    
    %% [SCREEN 01 - 로봇 링크 및 집게 그래픽스 실시간 동적 변환]
    % 입체 메탈 링크 빔 업데이트
    for k = 1:5
        set(link1_layers(k), 'XData', [0, x1], 'YData', [0, y1]);
        set(link2_layers(k), 'XData', [x1, x2], 'YData', [y1, y2]);
    end
    set(h_j2_cap, 'XData', x1, 'YData', y1);
    set(h_wrist_cap, 'XData', x2, 'YData', y2);
    
    % 2x2 회전 변환 행렬을 적용해 집게 파트를 로봇 끝단 좌표(x2, y2) 위에 안착 및 회전 제어
    R = [cos(phi), -sin(phi); sin(phi), cos(phi)];
    
    wr_g   = R * [wrist_x; wrist_y] + [x2; y2];
    cl_L_g = R * [claw_base_x; claw_base_y] + [x2; y2];
    cl_R_g = R * [claw_base_x; -claw_base_y] + [x2; y2];
    pd_L_g = R * [pad_base_x; pad_base_y] + [x2; y2];
    pd_R_g = R * [pad_base_x; -pad_base_y] + [x2; y2];
    
    % 변환된 글로벌 행렬 좌표를 패치 객체에 실시간 업데이트
    set(h_wrist, 'XData', wr_g(1,:), 'YData', wr_g(2,:));
    set(h_claw_L, 'XData', cl_L_g(1,:), 'YData', cl_L_g(2,:));
    set(h_claw_R, 'XData', cl_R_g(1,:), 'YData', cl_R_g(2,:));
    set(h_pad_L, 'XData', pd_L_g(1,:), 'YData', pd_L_g(2,:));
    set(h_pad_R, 'XData', pd_R_g(1,:), 'YData', pd_R_g(2,:));
    
    % 집게가 수놓으며 그리는 순기구학 합성 (X, Y) 궤적선 동적 업데이트
    set(hand_trace, 'XData', x2_history, 'YData', y2_history);
    
    %% [SCREEN 02 - 우측 차트 모터 신호선 업데이트]
    set(curve_theta1, 'XData', time_axis, 'YData', t1_history);
    set(curve_theta2, 'XData', time_axis, 'YData', t2_history);
    
    %% HUD 대시보드 데이터 실시간 리프레시
    hud_string = sprintf(['  [FORWARD KINE]\n  ------------------\n  Start At: (2.0, 0)\n  \\theta_1 Speed: 1x\n  \\theta_2 Speed: 2x\n\n  [GRIPPER POSITION]\n  Hand X  : %5.2f m\n  Hand Y  : %5.2f m'], ...
                           x2, y2);
    set(hud_box, 'String', hud_string);
    
    %% 애니메이션 프레임 동기화 및 딜레이 속도 제어
    drawnow;
    pause(0.015);
end