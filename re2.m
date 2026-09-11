% Isolated Link-2 & Gripper Elliptical Trajectory Simulation (MATLAB R2026a)
% Synchronized Initial State: x = 2.0, y = 0 (theta1 = 0, theta2 = 0)
clear; clc; close all;

%% 1. 로봇 물리 및 타원 궤적 파라미터 설정
l1 = 1.0; % 링크 1 길이 (m) - 내부 연산용
l2 = 1.0; % 링크 2 길이 (m) - 시각화 대상

% 타원 궤적 파라미터 (x^2/2^2 + y^2/1.3^2 = 1)
a = 2.0;  % 장반경
b = 1.3;  % 단반경

num_steps = 240;
t = linspace(0, 2*pi, num_steps);

% 초기 위치 x=2.0, y=0 만족하는 타원 가이드라인 데이터
X_ellipse = a * cos(t);
Y_ellipse = b * sin(t);

%% 2. 시각화 그래픽스 환경 구축 (듀얼 가로형 레이아웃)
fig = figure('Name', 'Isolated Link-2 & Gripper Kinematic HUD', ...
             'NumberTitle', 'off', ...
             'Color', [0.08, 0.09, 0.11], ...
             'Position', [100, 200, 1250, 550]);

%% 3. [좌측 스크린 - 링크 1이 생략된 물리 공간 모션 뷰]
ax1 = subplot(1, 2, 1, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax1, 'on'); hold(ax1, 'on'); axis(ax1, 'equal');
xlim(ax1, [-2.5, 2.5]); ylim(ax1, [-2.5, 2.5]);
title(ax1, 'SCREEN 01: Isolated Link-2 & Gripper Motion', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 참조용 타원 기준 가이드라인 (회색 점선)
plot(ax1, X_ellipse, ':', 'Color', [0.4, 0.45, 0.5], 'LineWidth', 1);

% ★ 로봇 핸드(집게 끝)가 그리며 나아가는 실시간 궤적선 (네온 마젠타 컬러)
hand_trace = plot(ax1, NaN, NaN, '-', 'Color', [1.0, 0.2, 0.6], 'LineWidth', 2.5);

%% 4. [우측 스크린 - 실시간 역기구학 관절각 추적 차트]
ax2 = subplot(1, 2, 2, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax2, 'on'); hold(ax2, 'on');
xlim(ax2, [0, num_steps]); ylim(ax2, [-190, 190]);
xlabel(ax2, 'Simulation Time Step', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
ylabel(ax2, 'Joint Angles (degree)', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
title(ax2, 'SCREEN 02: Real-Time Inverse Kinematics Curves', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 실시간 궤적 플롯 객체 선언 (R2026a Line 클래스 속성오류 방지 문법)
curve_theta1 = plot(ax2, NaN, NaN, '--', 'Color', [1.0, 0.4, 0.0], 'LineWidth', 2.0, 'DisplayName', '\theta_1 (Hidden Link)');
curve_theta2 = plot(ax2, NaN, NaN, '-', 'Color', [0.0, 1.0, 0.5], 'LineWidth', 2.5, 'DisplayName', '\theta_2 (Visible Link)');
legend(ax2, 'TextColor', [0.8, 0.85, 0.9], 'Color', [0.15, 0.17, 0.22], 'EdgeColor', [0.3, 0.35, 0.45]);

%% 5. 아노다이징 골드 메탈릭 [링크 2] 레이어 빌드 (★링크 1 레이어는 생성 안 함)
gold_colors = {[0.25, 0.18, 0.05], [0.70, 0.52, 0.15], [0.88, 0.71, 0.28], [0.98, 0.88, 0.55], [1.0, 0.98, 0.85]};
widths = [26, 21, 14, 7, 2.5];

link2_layers = [];
for k = 1:5
    link2_layers(k) = plot(ax1, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
end

%% 6. 집게(Gripper) 및 구동 조인트 피벗 볼트 세팅
wrist_x = [-0.03,  0.03,  0.03, -0.03]; wrist_y = [-0.05, -0.05,  0.05,  0.05];
claw_base_x = [0.03, 0.11, 0.19, 0.23, 0.20, 0.12, 0.03]; claw_base_y = [0.05, 0.09, 0.07, 0.02, -0.01, 0.03, 0.01];
pad_base_x = [0.15, 0.23, 0.20]; pad_base_y = [0.07, 0.02, -0.01];

h_wrist  = patch(ax1, 'XData', wrist_x, 'YData', wrist_y, 'FaceColor', [0.30, 0.25, 0.15]);
h_claw_L = patch(ax1, 'XData', claw_base_x, 'YData', claw_base_y, 'FaceColor', [0.85, 0.75, 0.45]);
h_claw_R = patch(ax1, 'XData', claw_base_x, 'YData', -claw_base_y, 'FaceColor', [0.85, 0.75, 0.45]);
h_pad_L  = patch(ax1, 'XData', pad_base_x, 'YData', pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');
h_pad_R  = patch(ax1, 'XData', pad_base_x, 'YData', -pad_base_y, 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');

% 링크 2의 시작점인 '관절 2 피벗 캡' (공중에서 회전축 역할을 시각적으로 명시)
h_j2_cap = plot(ax1, NaN, NaN, 'o', 'MarkerSize', 11, 'MarkerFaceColor', [0.9, 0.75, 0.20], 'MarkerEdgeColor', [0.1, 0.1, 0.1]);

%% 7. 오버레이 HUD 정보창 레이아웃 배치
hud_box = annotation('textbox', [0.14, 0.68, 0.17, 0.18], 'Color', [0.98, 0.88, 0.55], ...
                     'BackgroundColor', [0.15, 0.17, 0.22, 0.85], 'EdgeColor', [0.6, 0.4, 0.1], ...
                     'FontName', 'Courier New', 'FontSize', 9, 'FontWeight', 'bold');

%% 8. 실시간 역기구학 연동 및 애니메이션 루프
x_history = []; y_history = [];
t1_history = []; t2_history = [];
time_axis = [];

for i = 1:num_steps
    % 현재 프레임의 타원 위 목표 타스크 좌표 (x, y)
    x_target = X_ellipse(i);
    y_target = Y_ellipse(i);
    
    %% [역기구학 해석연산]
    D = (x_target^2 + y_target^2 - l1^2 - l2^2) / (2 * l1 * l2);
    if D > 1, D = 1; elseif D < -1, D = -1; end % 수치 에러 방지
    
    % 양의 해(Elbow-Up) 구조 추적
    theta2 = acos(D); 
    theta1 = atan2(y_target, x_target) - atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));
    
    % 우측 차트용 변위 어레이 누적 기록
    time_axis = [time_axis, i];
    t1_history = [t1_history, rad2deg(theta1)];
    t2_history = [t2_history, rad2deg(theta2)];
    
    %% [순기구학 기반 링크 2 양 끝단 좌표 매핑]
    % 링크 1은 그리지 않지만, 링크 2가 시작되는 좌표인 관절 2(x1, y1) 정보는 연산해야 합니다.
    x1 = l1 * cos(theta1); 
    y1 = l1 * sin(theta1);
    
    % 링크 2의 말단이자 최종 핸드 지점(x2, y2) -> 타원 위의 점과 정밀 일치
    x2 = x1 + l2 * cos(theta1 + theta2); 
    y2 = y1 + l2 * sin(theta1 + theta2);
    phi = theta1 + theta2; % 집게 방위각
    
    % 핸드 누적 흔적 업데이트용 배열 저장
    x_history = [x_history, x2];
    y_history = [y_history, y2];
    
    %% [SCREEN 01 - 링크 2 및 집게 그래픽스 실시간 업데이트]
    % ★ 오직 링크 2에 대해서만 레이어 빔 좌표를 주입하여 갱신합니다.
    for k = 1:5
        set(link2_layers(k), 'XData', [x1, x2], 'YData', [y1, y2]);
    end
    set(h_j2_cap, 'XData', x1, 'YData', y1); % 공중에 떠서 움직이는 관절 2 기둥
    
    % 집게 오프셋 회전 변환
    R = [cos(phi), -sin(phi); sin(phi), cos(phi)];
    wr_g = R * [wrist_x; wrist_y] + [x2; y2];
    cl_L_g = R * [claw_base_x; claw_base_y] + [x2; y2]; cl_R_g = R * [claw_base_x; -claw_base_y] + [x2; y2];
    pd_L_g = R * [pad_base_x; pad_base_y] + [x2; y2];   pd_R_g = R * [pad_base_x; -pad_base_y] + [x2; y2];
    
    set(h_wrist, 'XData', wr_g(1,:), 'YData', wr_g(2,:));
    set(h_claw_L, 'XData', cl_L_g(1,:), 'YData', cl_L_g(2,:)); set(h_claw_R, 'XData', cl_R_g(1,:), 'YData', cl_R_g(2,:));
    set(h_pad_L, 'XData', pd_L_g(1,:), 'YData', pd_L_g(2,:));   set(h_pad_R, 'XData', pd_R_g(1,:), 'YData', pd_R_g(2,:));
    
    % 실시간 타원 궤적 흔적 라인 업데이트
    set(hand_trace, 'XData', x_history, 'YData', y_history);
    
    %% [SCREEN 02 - 우측 차트 변위 선도 업데이트]
    set(curve_theta1, 'XData', time_axis, 'YData', t1_history);
    set(curve_theta2, 'XData', time_axis, 'YData', t2_history);
    
    %% HUD 대시보드 스펙 정보 실시간 리프레시
    hud_string = sprintf(['  [ISOLATED MODE]\n  ------------------\n  Start At: (2.0, 0)\n  Link 1  : HIDDEN\n  Link 2  : VISIBLE\n\n  [CURRENT HAND]\n  X Coord : %5.2f m\n  Y Coord : %5.2f m'], ...
                           x2, y2);
    set(hud_box, 'String', hud_string);
    
    %% 애니메이션 동기화 및 딜레이 스피드 제어
    drawnow;
    pause(0.015);
end