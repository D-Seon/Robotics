% IK-to-FK Duality Verification & Elliptical Trajectory Simulation (MATLAB R2026a)
% Synchronized Initial State: x = 2.0, y = 0 (theta1 = 0, theta2 = 0)
clear; clc; close all;

%% 1. 로봇 물리 및 타원 궤적 파라미터 설정
l1 = 1.0; % 링크 1 길이 (m)
l2 = 1.0; % 링크 2 길이 (m)

% 목표 타원 방정식 파라미터 (x^2/2^2 + y^2/1.3^2 = 1)
a = 2.0;  % 장반경
b = 1.3;  % 단반경

num_steps = 240;
t = linspace(0, 2*pi, num_steps);

% [마스터 목표 데이터] 우리가 도달해야 할 타원 좌표 배열
X_target = a * cos(t);
Y_target = b * sin(t);

%% 2. 시각화 그래픽스 환경 구축 (듀얼 가로형 레이아웃)
fig = figure('Name', 'IK-to-FK Kinematic Loop Verification Dashboard', ...
             'NumberTitle', 'off', ...
             'Color', [0.08, 0.09, 0.11], ...
             'Position', [100, 200, 1250, 550]);

%% 3. [좌측 스크린 - 물리 공간 및 FK 복원 궤적 시각화]
ax1 = subplot(1, 2, 1, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax1, 'on'); hold(ax1, 'on'); axis(ax1, 'equal');
xlim(ax1, [-2.5, 2.5]); ylim(ax1, [-2.5, 2.5]);
title(ax1, 'SCREEN 01: FK Reconstructed Ellipse Space', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 참조용 원래 목표 타원 가이드라인 (회색 점선)
plot(ax1, X_target, Y_target, ':', 'Color', [0.5, 0.55, 0.6], 'LineWidth', 1.5);

% ★ 중요: 각도를 전달받은 정기구학(FK) 공식이 실시간으로 복원해내는 궤적선 (네온 마젠타)
fk_reconstructed_trace = plot(ax1, NaN, NaN, '-', 'Color', [1.0, 0.2, 0.6], 'LineWidth', 2.5);

% 중후한 베이스 고정 스탠드 오버레이 드로잉
base_plate_x = [-0.45, 0.45, 0.30, -0.30]; base_plate_y = [-0.60, -0.60, -0.15, -0.15];
base_neck_x = [-0.15, 0.15, 0.10, -0.10];  base_neck_y = [-0.15, -0.15, 0.0, 0.0];
fill(ax1, base_plate_x, base_plate_y, [0.18, 0.20, 0.23], 'EdgeColor', [0.1, 0.11, 0.12], 'LineWidth', 2);
fill(ax1, base_neck_x, base_neck_y, [0.55, 0.58, 0.62], 'EdgeColor', [0.3, 0.32, 0.35], 'LineWidth', 1.5);
rectangle(ax1, 'Position', [-0.18, -0.18, 0.36, 0.36], 'Curvature', [1, 1], 'FaceColor', [0.35, 0.38, 0.42], 'LineWidth', 1.5);

%% 4. [우측 스크린 - 역기구학(IK)이 짜낸 타원 전용 관절 변위 차트]
ax2 = subplot(1, 2, 2, 'Parent', fig, ...
              'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
              'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
grid(ax2, 'on'); hold(ax2, 'on');
xlim(ax2, [0, num_steps]); ylim(ax2, [-190, 190]);
xlabel(ax2, 'Simulation Time Step', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
ylabel(ax2, 'Joint Angles (degree)', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
title(ax2, 'SCREEN 02: IK Computed Elliptical Joint Profiles', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 실시간 궤적 플롯 객체 선언 
curve_theta1 = plot(ax2, NaN, NaN, '-', 'Color', [0.0, 1.0, 0.5], 'LineWidth', 2.2, 'DisplayName', '\theta_1 (IK Output)');
curve_theta2 = plot(ax2, NaN, NaN, '-', 'Color', [1.0, 0.4, 0.0], 'LineWidth', 2.2, 'DisplayName', '\theta_2 (IK Output)');
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
hud_box = annotation('textbox', [0.14, 0.68, 0.17, 0.18], 'Color', [0.0, 0.95, 1.0], ...
                     'BackgroundColor', [0.15, 0.17, 0.22, 0.85], 'EdgeColor', [0.0, 0.4, 0.6], ...
                     'FontName', 'Courier New', 'FontSize', 9, 'FontWeight', 'bold');

%% 8. 역기구학(IK) 도출 ➔ 정기구학(FK) 변환 실시간 시뮬레이션 루프
xfk_history = []; yfk_history = [];
t1_history = []; t2_history = [];
time_axis = [];

for i = 1:num_steps
    % 1) 타원 궤적상의 마스터 목표 좌표 획득
    x_ellipse = X_target(i);
    y_ellipse = Y_target(i);
    
    %% ================= [단계 1: 역기구학(IK) 연산] =================
    % 목표 좌표(x, y)를 추종하기 위한 각도 계산 (Elbow-Down 양의 해 고정)
    D = (x_ellipse^2 + y_ellipse^2 - l1^2 - l2^2) / (2 * l1 * l2);
    if D > 1, D = 1; elseif D < -1, D = -1; end % 예외처리
    
    theta2_calc = acos(D); 
    theta1_calc = atan2(y_ellipse, x_ellipse) - atan2(l2 * sin(theta2_calc), l1 + l2 * cos(theta2_calc));
    
    % 차트 누적 데이터 기록
    time_axis = [time_axis, i];
    t1_history = [t1_history, rad2deg(theta1_calc)];
    t2_history = [t2_history, rad2deg(theta2_calc)];
    
    %% ================= [단계 2: 정기구학(FK) 연산] =================
    % ★ IK가 구한 각도(theta1_calc, theta2_calc)를 FK 공식에 그대로 대입하여 궤적을 복원합니다.
    x1_fk = l1 * cos(theta1_calc); 
    y1_fk = l1 * sin(theta1_calc);
    
    x2_fk = x1_fk + l2 * cos(theta1_calc + theta2_calc); 
    y2_fk = y1_fk + l2 * sin(theta1_calc + theta2_calc);
    phi_fk = theta1_calc + theta2_calc; % 집게 지향각
    
    % 정기구학이 최종 뱉어낸 실시간 복원 좌표 저장
    xfk_history = [xfk_history, x2_fk];
    yfk_history = [yfk_history, y2_fk];
    
    %% [SCREEN 01 - FK 좌표 기반 로봇 물리 암 및 집게 실시간 동적 변환]
    for k = 1:5
        set(link1_layers(k), 'XData', [0, x1_fk], 'YData', [0, y1_fk]);
        set(link2_layers(k), 'XData', [x1_fk, x2_fk], 'YData', [y1_fk, y2_fk]);
    end
    set(h_j2_cap, 'XData', x1_fk, 'YData', y1_fk);
    
    % 회전 매트릭스를 통한 집게 좌표 동적 오프셋 변환
    R = [cos(phi_fk), -sin(phi_fk); sin(phi_fk), cos(phi_fk)];
    wr_g = R * [wrist_x; wrist_y] + [x2_fk; y2_fk];
    cl_L_g = R * [claw_base_x; claw_base_y] + [x2_fk; y2_fk]; cl_R_g = R * [claw_base_x; -claw_base_y] + [x2_fk; y2_fk];
    pd_L_g = R * [pad_base_x; pad_base_y] + [x2_fk; y2_fk];   pd_R_g = R * [pad_base_x; -pad_base_y] + [x2_fk; y2_fk];
    set(h_wrist, 'XData', wr_g(1,:), 'YData', wr_g(2,:));
    set(h_claw_L, 'XData', cl_L_g(1,:), 'YData', cl_L_g(2,:)); set(h_claw_R, 'XData', cl_R_g(1,:), 'YData', cl_R_g(2,:));
    set(h_pad_L, 'XData', pd_L_g(1,:), 'YData', pd_L_g(2,:));   set(h_pad_R, 'XData', pd_R_g(1,:), 'YData', pd_R_g(2,:));
    
    % ★ FK 공식 연산 결과가 그리는 마젠타색 복원 가이드 궤적선 실시간 갱신
    set(fk_reconstructed_trace, 'XData', xfk_history, 'YData', yfk_history);
    
    %% [SCREEN 02 - 우측 차트 변위 선도 업데이트]
    set(curve_theta1, 'XData', time_axis, 'YData', t1_history);
    set(curve_theta2, 'XData', time_axis, 'YData', t2_history);
    
    %% HUD 대시보드 스펙 정보 갱신
    hud_string = sprintf(['  [IK->FK LOOP]\n  ------------------\n  Start At: (2.0, 0)\n  \\theta_1 Curve: Calc\n  \\theta_2 Curve: Calc\n\n  [FK RECOVERY X,Y]\n  Recov X : %5.2f m\n  Recov Y : %5.2f m'], ...
                           x2_fk, y2_fk);
    set(hud_box, 'String', hud_string);
    
    %% 애니메이션 속도 제어
    drawnow;
    pause(0.015);
end