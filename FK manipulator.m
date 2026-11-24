% Real-time 2-Link Planar Manipulator Simulation (All Start at X=2.0, Y=0)
clear; clc; close all;

%% 1. 파라미터 및 타스크 궤적 설정
L1 = 1.0; % 링크 1 길이 (m)
L2 = 1.0; % 링크 2 길이 (m)

num_steps = 200;
t = linspace(0, 2*pi, num_steps); 
X_task = 2.0 * cos(t);
Y_task = 1.3 * sin(t);

%% [중요 조건] 초기값 X=2.0, Y=0 (즉, 두 링크가 일직선인 상태)을 위한 고정각 설정
% 집게가 (2.0, 0)에 위치하기 위해 두 관절 모두 0도(라디안)에서 시작 및 고정해야 합니다.
theta1_fixed = 0;  % 2번 화면에서 고정될 \theta_1 값 (0도)
theta2_fixed = 0;  % 3번 화면에서 고정될 \theta_2 값 (0도)

% 모든 화면의 궤적 기록용 배열 초기화
X_rot1_hist = zeros(1, num_steps); Y_rot1_hist = zeros(1, num_steps);
X_rot2_hist = zeros(1, num_steps); Y_rot2_hist = zeros(1, num_steps);
X_rot3_hist = zeros(1, num_steps); Y_rot3_hist = zeros(1, num_steps);
X_rot4_hist = zeros(1, num_steps); Y_rot4_hist = zeros(1, num_steps);

%% 2. 그래픽 창 및 Subplot 레이아웃 설정
fig = figure('Name', 'Manipulator Trajectory Simulation', 'Position', [50, 50, 1200, 900]);

% 1번 화면: 역기구학 연동 구동 및 타스크 궤적 누적 (파란 점)
ax1 = subplot(2,2,1); grid on; hold on; axis equal; xlim([-2.5, 2.5]); ylim([-2.5, 2.5]);
xlabel('X (m)'); ylabel('Y (m)'); title('1번: 로봇 핸드 타스크 궤적 (IK 연동)');
h_traj1 = plot(nan, nan, 'b.', 'MarkerSize', 6); 

% 2번 화면: \theta_1 = 0 고정, \theta_2만 변화하며 그리는 궤적 (빨간 점)
ax2 = subplot(2,2,2); grid on; hold on; axis equal; xlim([-2.5, 2.5]); ylim([-2.5, 2.5]);
xlabel('X (m)'); ylabel('Y (m)'); title('2번: \theta_1 고정(0^\circ) / \theta_2 가변 궤적');
h_traj2 = plot(nan, nan, 'r.', 'MarkerSize', 6); 

% 3번 화면: \theta_2 = 0 고정, \theta_1만 변화하며 그리는 궤적 (초록 점)
ax3 = subplot(2,2,3); grid on; hold on; axis equal; xlim([-2.5, 2.5]); ylim([-2.5, 2.5]);
xlabel('X (m)'); ylabel('Y (m)'); title('3번: \theta_2 고정(0^\circ) / \theta_1 가변 궤적');
h_traj3 = plot(nan, nan, 'g.', 'MarkerSize', 6); 

% 4번 화면: 정기구학 복원 궤적 검증 누적 (자홍색 점)
ax4 = subplot(2,2,4); grid on; hold on; axis equal; xlim([-2.5, 2.5]); ylim([-2.5, 2.5]);
xlabel('X (m)'); ylabel('Y (m)'); title('4번: 정기구학(FK) 궤적 복원 검증');
h_traj4 = plot(nan, nan, 'm.', 'MarkerSize', 6); 

sgtitle('Real-Time 2-Link Manipulator Simulation (All Start Position: X=2.0, Y=0) (R2026a)', 'FontSize', 14, 'FontWeight', 'bold');

%% 3. 베이스 및 매니퓰레이터 그래픽 객체 생성
for ax = [ax1, ax2, ax3, ax4]
    draw_base(ax);
end

[h_l1_a1, h_j1_a1, h_l2_a1, h_j2_a1, h_g_a1] = create_robot_shapes(ax1);
[h_l1_a2, h_j1_a2, h_l2_a2, h_j2_a2, h_g_a2] = create_robot_shapes(ax2);
[h_l1_a3, h_j1_a3, h_l2_a3, h_j2_a3, h_g_a3] = create_robot_shapes(ax3);
[h_l1_a4, h_j1_a4, h_l2_a4, h_j2_a4, h_g_a4] = create_robot_shapes(ax4);

h_txt1 = text(ax1, -2.3, 2.2, '', 'FontSize', 9, 'BackgroundColor', 'w', 'EdgeColor', 'k');

%% 4. 실시간 구동 루프
for i = 1:num_steps
    % 1번 & 4번을 위한 실시간 목표 좌표 분석
    X = X_task(i);
    Y = Y_task(i);
    
    %% 1번 & 4번 화면: 역기구학(IK) 구동
    cos_theta2 = (X^2 + Y^2 - L1^2 - L2^2) / (2 * L1 * L2);
    cos_theta2 = max(min(cos_theta2, 1), -1); 
    th2_main = atan2(sqrt(1 - cos_theta2^2), cos_theta2);
    th1_main = atan2(Y, X) - atan2(L2*sin(th2_main), L1 + L2*cos(th2_main));
    
    x1_m = L1*cos(th1_main);          y1_m = L1*sin(th1_main);
    x2_m = x1_m + L2*cos(th1_main + th2_main); y2_m = y1_m + L2*sin(th1_main + th2_main);
    
    X_rot1_hist(i) = X;    Y_rot1_hist(i) = Y;
    X_rot4_hist(i) = x2_m; Y_rot4_hist(i) = y2_m;
    
    %% 2번 화면: \theta_1 = 0 고정, \theta_2가 0에서부터 회전 시작
    th2_moving = t(i); 
    
    x1_ax2 = L1 * cos(theta1_fixed); % X=1, Y=0 지점에 조인트2 고정
    y1_ax2 = L1 * sin(theta1_fixed); 
    x2_ax2 = x1_ax2 + L2 * cos(theta1_fixed + th2_moving);
    y2_ax2 = y1_ax2 + L2 * sin(theta1_fixed + th2_moving);
    
    X_rot2_hist(i) = x2_ax2;
    Y_rot2_hist(i) = y2_ax2;
    
    %% 3번 화면: \theta_2 = 0 고정, \theta_1이 0에서부터 회전 시작 (일직선 상태로 회전)
    th1_moving = t(i); 
    
    x1_ax3 = L1 * cos(th1_moving); 
    y1_ax3 = L1 * sin(th1_moving);
    x2_ax3 = x1_ax3 + L2 * cos(th1_moving + theta2_fixed); 
    y2_ax3 = y1_ax3 + L2 * sin(th1_moving + theta2_fixed);
    
    X_rot3_hist(i) = x2_ax3;
    Y_rot3_hist(i) = y2_ax3;
    
    %% 데이터 업데이트 및 플롯 그리기
    set(h_traj1, 'XData', X_rot1_hist(1:i), 'YData', Y_rot1_hist(1:i));
    set(h_traj2, 'XData', X_rot2_hist(1:i), 'YData', Y_rot2_hist(1:i));
    set(h_traj3, 'XData', X_rot3_hist(1:i), 'YData', Y_rot3_hist(1:i));
    set(h_traj4, 'XData', X_rot4_hist(1:i), 'YData', Y_rot4_hist(1:i));
    
    % 각 서브플롯 로봇 형상 갱신
    update_robot_pose(th1_main, th2_main, x1_m, y1_m, x2_m, y2_m, h_l1_a1, h_j1_a1, h_l2_a1, h_j2_a1, h_g_a1);
    update_robot_pose(theta1_fixed, th2_moving, x1_ax2, y1_ax2, x2_ax2, y2_ax2, h_l1_a2, h_j1_a2, h_l2_a2, h_j2_a2, h_g_a2);
    update_robot_pose(th1_moving, theta2_fixed, x1_ax3, y1_ax3, x2_ax3, y2_ax3, h_l1_a3, h_j1_a3, h_l2_a3, h_j2_a3, h_g_a3);
    update_robot_pose(th1_main, th2_main, x1_m, y1_m, x2_m, y2_m, h_l1_a4, h_j1_a4, h_l2_a4, h_j2_a4, h_g_a4);
    
    set(h_txt1, 'String', sprintf('실시간 메인 관절각\n\\theta_1: %.1f^\\circ\n\\theta_2: %.1f^\\circ', rad2deg(th1_main), rad2deg(th2_main)));
    
    drawnow;
    pause(0.015);
end

%% ================= 로봇 외형 렌더링 헬퍼 함수들 =================
function [h_l1, h_j1, h_l2, h_j2, h_g] = create_robot_shapes(ax)
    metal_gray = [0.45, 0.45, 0.45]; joint_dark = [0.2, 0.2, 0.2];
    h_l1 = fill(ax, nan, nan, metal_gray, 'EdgeColor', [0.1,0.1,0.1], 'LineWidth', 1.5);
    h_j1 = fill(ax, nan, nan, [0.15, 0.15, 0.15], 'EdgeColor', 'k', 'LineWidth', 1.2); 
    h_l2 = fill(ax, nan, nan, metal_gray, 'EdgeColor', [0.1,0.1,0.1], 'LineWidth', 1.5);
    h_j2 = fill(ax, nan, nan, joint_dark, 'EdgeColor', 'k', 'LineWidth', 1.2);
    h_g  = fill(ax, nan, nan, joint_dark, 'EdgeColor', 'k', 'LineWidth', 1.2);
end

function draw_base(ax)
    base_x = [-0.25, -0.20, 0.20, 0.25,  0.35, -0.35]; base_y = [ 0.00, -0.30, -0.30, 0.00, -0.45, -0.45];
    fill(ax, base_x, base_y, [0.28, 0.28, 0.28], 'EdgeColor', 'k', 'LineWidth', 1.5);
    fill(ax, [-0.45, 0.45, 0.45, -0.45], [-0.45, -0.45, -0.55, -0.55], [0.15, 0.15, 0.15], 'EdgeColor', 'k');
    plot(ax, [-0.5, 0.5], [-0.55, -0.55], 'k-', 'LineWidth', 2);
    for x = -0.45:0.05:0.45, plot(ax, [x, x+0.03], [-0.55, -0.62], 'k-'); end
end

function update_robot_pose(th1, th2, x1, y1, x2, y2, h_l1, h_j1, h_l2, h_j2, h_g)
    W = 0.14; link_template = [0, -W/2; 0, W/2; 1.0, W/3; 1.0, -W/3]; 
    r_j = 0.15; ang = linspace(0, 2*pi, 25); joint_template = [r_j*cos(ang); r_j*sin(ang)]';
    grip_template = [0, 0; 0.08, 0.12; 0.22, 0.12; 0.12, 0.02; 0.22, -0.12; 0.08, -0.12];

    R1 = [cos(th1), -sin(th1); sin(th1), cos(th1)];
    pts_l1 = (R1 * link_template')';
    set(h_l1, 'XData', pts_l1(:,1), 'YData', pts_l1(:,2));
    set(h_j1, 'XData', joint_template(:,1), 'YData', joint_template(:,2));

    th12 = th1 + th2;
    R12 = [cos(th12), -sin(th12); sin(th12), cos(th12)];
    pts_l2 = (R12 * link_template')';
    set(h_l2, 'XData', pts_l2(:,1) + x1, 'YData', pts_l2(:,2) + y1);
    set(h_j2, 'XData', joint_template(:,1) + x1, 'YData', joint_template(:,2) + y1);

    pts_g = (R12 * grip_template')';
    set(h_g, 'XData', pts_g(:,1) + x2, 'YData', pts_g(:,2) + y2);
end