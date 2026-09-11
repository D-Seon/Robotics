% Isolated Link-1 & Theta1 Trajectory Simulation (MATLAB R2026a)
% Synchronized Initial State: x = 2.0, y = 0 (theta1 = 0, theta2 = 0)
clear; clc; close all;

%% 1. 로봇 물리 및 원래 타원 궤적 파라미터 설정
l1 = 1.0; % 링크 1 길이 (m) - 시각화 대상
l2 = 1.0; % 링크 2 길이 (m) - 내부 역기구학 연산용

% 원래 제공된 타원 조건 (x^2/2^2 + y^2/1.3^2 = 1)
a = 2.0;  % 장반경
b = 1.3;  % 단반경

num_steps = 240;
t = linspace(0, 2*pi, num_steps);

% 초기값 x=2.0, y=0을 만족하는 마스터 타원 가이드라인
X_ellipse = a * cos(t);
Y_ellipse = b * sin(t);

%% 2. 시각화 그래픽스 환경 구축 (산업용 다크 대시보드)
fig = figure('Name', 'Isolated Link-1 & \theta_1 Analysis HUD', ...
             'NumberTitle', 'off', ...
             'Color', [0.08, 0.09, 0.11], ...
             'Position', [100, 200, 1250, 550]);

%% 3. [좌측 스크린 - 링크 2가 생략된 물리 공간 모션 뷰]
ax1 = subplot(1, 2, 1, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax1, 'on'); hold(ax1, 'on'); axis(ax1, 'equal');
xlim(ax1, [-2.5, 2.5]); ylim(ax1, [-2.5, 2.5]);
title(ax1, 'SCREEN 01: Isolated Link-1 Physical Motion (+ \theta_2 Mode)', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 참조용 타원 목표 가이드라인 (회색 점선)
plot(ax1, X_ellipse, ':', 'Color', [0.4, 0.45, 0.5], 'LineWidth', 1);

% ★ 이번 코딩의 핵심: 링크 1의 말단(관절 2)이 그리는 실제 공간 궤적선 (네온 그린)
link1_trace = plot(ax1, NaN, NaN, '-', 'Color', [0.0, 1.0, 0.5], 'LineWidth', 2.5);

% 묵직한 산업용 베이스 고정 받침대 오버레이
base_plate_x = [-0.45, 0.45, 0.30, -0.30]; base_plate_y = [-0.60, -0.60, -0.15, -0.15];
base_neck_x = [-0.15, 0.15, 0.10, -0.10];  base_neck_y = [-0.15, -0.15, 0.0, 0.0];
fill(ax1, base_plate_x, base_plate_y, [0.18, 0.20, 0.23], 'EdgeColor', [0.1, 0.11, 0.12], 'LineWidth', 2);
fill(ax1, base_neck_x, base_neck_y, [0.55, 0.58, 0.62], 'EdgeColor', [0.3, 0.32, 0.35], 'LineWidth', 1.5);
rectangle(ax1, 'Position', [-0.18, -0.18, 0.36, 0.36], 'Curvature', [1, 1], 'FaceColor', [0.35, 0.38, 0.42], 'LineWidth', 1.5);

%% 4. [우측 스크린 - 링크 1 관절각 \theta_1의 시간축 변위 차트]
ax2 = subplot(1, 2, 2, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax2, 'on'); hold(ax2, 'on');
xlim(ax2, [0, num_steps]); ylim(ax2, [-190, 190]);
xlabel(ax2, 'Simulation Time Step', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
ylabel(ax2, 'Joint-1 Angle \theta_1 (degree)', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
title(ax2, 'SCREEN 02: Real-Time \theta_1 Inverse Kinematics Curve', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 실시간 궤적 플롯 객체 선언 (Line 클래스 속성 오류 완벽 방지)
curve_theta1 = plot(ax2, NaN, NaN, '-', 'Color', [0.0, 1.0, 0.5], 'LineWidth', 2.5, 'DisplayName', '\theta_1 (Active Link-1)');
legend(ax2, 'TextColor', [0.8, 0.85, 0.9], 'Color', [0.15, 0.17, 0.22], 'EdgeColor', [0.3, 0.35, 0.45]);

%% 5. 아노다이징 골드 메탈릭 [링크 1] 레이어 빌드 (★링크 2 및 집게는 코드에서 전면 생략)
gold_colors = {[0.25, 0.18, 0.05], [0.70, 0.52, 0.15], [0.88, 0.71, 0.28], [0.98, 0.88, 0.55], [1.0, 0.98, 0.85]};
widths = [26, 21, 14, 7, 2.5];

link1_layers = [];
for k = 1:5
    link1_layers(k) = plot(ax1, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
end

% 링크 1의 최종 끝단인 '관절 2 피벗 볼트 캡'만 시각화 포인트로 남겨둠
h_j2_cap = plot(ax1, NaN, NaN, 'o', 'MarkerSize', 11, 'MarkerFaceColor', [0.9, 0.75, 0.20], 'MarkerEdgeColor', [0.1, 0.1, 0.1]);

%% 6. 오버레이 HUD 정보창 레이아웃 배치
hud_box = annotation('textbox', [0.14, 0.68, 0.17, 0.18], 'Color', [0.98, 0.88, 0.55], ...
                     'BackgroundColor', [0.15, 0.17, 0.22, 0.85], 'EdgeColor', [0.6, 0.4, 0.1], ...
                     'FontName', 'Courier New', 'FontSize', 9, 'FontWeight', 'bold');

%% 7. 실시간 역기구학 루프 연산 및 시뮬레이션 전개
x1_history = []; y1_history = [];
t1_history = []; time_axis = [];

for i = 1:num_steps
    % 현재 스텝에서 보이지 않는 가상의 가이드 핸드가 도달해야 할 타원 좌표 (x, y)
    x_target = X_ellipse(i);
    y_target = Y_ellipse(i);
    
    %% [역기구학 연산 코어 - 조건: 세타2가 양(+)의 해인 아래 꺾임 모드]
    D = (x_target^2 + y_target^2 - l1^2 - l2^2) / (2 * l1 * l2);
    if D > 1, D = 1; elseif D < -1, D = -1; end % 수치적 예외 처리
    
    theta2 = acos(D); % 양의 해 고정 (Elbow-Down 형상 유도)
    theta1 = atan2(y_target, x_target) - atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));
    
    % 우측 선도 매핑용 데이터 기록
    time_axis = [time_axis, i];
    t1_history = [t1_history, rad2deg(theta1)];
    
    %% [순기구학 수식을 통한 링크 1 말단 좌표 도출]
    % 관절 1의 회전 성분으로 결정되는 링크 1 끝점 (x1, y1)
    x1 = l1 * cos(theta1); 
    y1 = l1 * sin(theta1);
    
    % 링크 1 끝단의 물리 공간 이동 역사 누적
    x1_history = [x1_history, x1];
    y1_history = [y1_history, y1];
    
    %% [SCREEN 01 - 오직 링크 1에 대해서만 그래픽 데이터 실시간 리프레시]
    for k = 1:5
        set(link1_layers(k), 'XData', [0, x1], 'YData', [0, y1]);
    end
    set(h_j2_cap, 'XData', x1, 'YData', y1); % 움직이는 관절 2 조인트 피벗 기둥
    
    % 링크 1의 끝단이 그려내는 실시간 네온 그린 궤적 업데이트
    set(link1_trace, 'XData', x1_history, 'YData', y1_history);
    
    %% [SCREEN 02 - 우측 차트 변위 선도 업데이트]
    set(curve_theta1, 'XData', time_axis, 'YData', t1_history);
    
    %% HUD 대시보드 스펙 정보 실시간 리프레시
    hud_string = sprintf(['  [LINK-1 ONLY MAP]\n  ------------------\n  Start At: (2.0, 0)\n  Link 1  : VISIBLE\n  Link 2  : HIDDEN\n\n  [JOINT 2 POSITION]\n  Coord X1: %5.2f m\n  Coord Y1: %5.2f m'], ...
                           x1, y1);
    set(hud_box, 'String', hud_string);
    
    %% 애니메이션 동기화 및 딜레이 속도 제어
    drawnow;
    pause(0.015);
end