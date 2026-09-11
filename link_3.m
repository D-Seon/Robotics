clear; clc; close all;

%% 1. 파라미터 및 환경 설정
L1 = 1.0; L2 = 1.0; L3 = 1.0;   % 모든 링크 길이 = 1.0m
W = 0.12;                       % 링크 두께
GripSize = 0.15;                % 집게발 크기

dt = 0.01;                      % Time Interval (Delta t = 0.01초)
T_final = 4.0;                  % 총 구동 시간 (2초)
t = 0:dt:T_final;               % 시간 벡터 생성
N = length(t);

% 주파수 설정
f1 = 1.0;                       % 관절 1 주파수 (1.0 Hz)
f2 = 0.5;                       % 관절 2 주파수 (0.5 Hz)
f3 = 0.25;                      % 관절 3 주파수 (0.25 Hz)

%% 2. 관절 궤적 입력 함수 정의 (시간에 따른 각도 변화)
theta1 = 2 * pi * f1 * t; 
theta2 = 2 * pi * f2 * t + pi/3; 
theta3 = 2 * pi * f3 * t + pi/6; 

%% 3. 시각화 창 설정 
figure('Name', '3-Link Manipulator Simulation', 'Color', 'w', 'Position', [100 100 850 850]);
hold on; grid on; axis equal;

% 로봇이 최대 확장했을 때(1.0 + 1.0 + 1.0 = 3.0m)를 고려한 화면 범위 설정
xlim([-3.5 3.5]); ylim([-3.5 3.5]);
xlabel('X [m]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y [m]', 'FontSize', 12, 'FontWeight', 'bold');
title('3-Link Manipulator: End-Effector Trajectory (Y vs X)', 'FontSize', 14, 'FontWeight', 'bold');

%% 4. 시각화 객체(그래픽 요소) 초기화
h_traj = plot(0, 0, 'b-', 'LineWidth', 1.8); % 말단 장치 궤적 선 (청색 실선)

% 3개 링크의 구분을 위해 그라데이션 색상 배치 (진한 색 -> 연한 색)
h_link1 = patch(0, 0, [0.1 0.1 0.1], 'EdgeColor', 'k', 'LineWidth', 1.2);
h_link2 = patch(0, 0, [0.4 0.4 0.4], 'EdgeColor', 'k', 'LineWidth', 1.2);
h_link3 = patch(0, 0, [0.7 0.7 0.7], 'EdgeColor', 'k', 'LineWidth', 1.2);

% 말단 장치 집게발
h_grip1 = plot([0 0], [0 0], 'k', 'LineWidth', 2.5);
h_grip2 = plot([0 0], [0 0], 'k', 'LineWidth', 2.5);

% 조인트 포인트 마커 표시
plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'w', 'LineWidth', 2); % Base (0,0)
h_joint1 = plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 1.5); % Joint 1
h_joint2 = plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'LineWidth', 1.5); % Joint 2

%% 5. 시뮬레이션 및 딜레이 루프 실행
traj_x = zeros(1, N);
traj_y = zeros(1, N);

for i = 1:N
    % 현재 시간에서의 관절 각도 계산
    th1 = theta1(i);
    th2 = theta2(i);
    th3 = theta3(i);
    
    % 누적 절대각 계산 (정기구학 수식 유도용)
    th12 = th1 + th2;
    th123 = th1 + th2 + th3;
    
    % --- 정기구학(Forward Kinematics) 좌표 계산 ---
    x0 = 0; y0 = 0;                                 % 베이스 원점
    x1 = L1 * cos(th1);                             % 관절 1 위치 (링크1의 끝)
    y1 = L1 * sin(th1);
    x2 = x1 + L2 * cos(th12);                       % 관절 2 위치 (링크2의 끝)
    y2 = y1 + L2 * sin(th12);
    x3 = x2 + L3 * cos(th123);                      % 말단 장치 최종 위치 (링크3의 끝)
    y3 = y2 + L3 * sin(th123);
    
    % 궤적 저장 및 업데이트
    traj_x(i) = x3;
    traj_y(i) = y3;
    set(h_traj, 'XData', traj_x(1:i), 'YData', traj_y(1:i));
    
    % 조인트 마커 위치 업데이트 (정확한 관절 중심점에 일치)
    set(h_joint1, 'XData', x1, 'YData', y1);
    set(h_joint2, 'XData', x2, 'YData', y2);
    
    % ---  링크 1 그래픽 업데이트 (베이스 원점 기준) ---
    R1 = [cos(th1) -sin(th1); sin(th1) cos(th1)];
    pts1 = R1 * [0, L1, L1, 0; W/2, W/2, -W/2, -W/2];
    set(h_link1, 'XData', pts1(1,:) + x0, 'YData', pts1(2,:) + y0);
    
    % ---  링크 2 그래픽 업데이트 (관절 1 위치 x1, y1 기준) ---
    R2 = [cos(th12) -sin(th12); sin(th12) cos(th12)];
    pts2 = R2 * [0, L2, L2, 0; W/2, W/2, -W/2, -W/2];
    set(h_link2, 'XData', pts2(1,:) + x1, 'YData', pts2(2,:) + y1);
    
    % ---  링크 3 그래픽 업데이트 (관절 2 위치 x2, y2 기준) ---
    R3 = [cos(th123) -sin(th123); sin(th123) cos(th123)];
    pts3 = R3 * [0, L3, L3, 0; W/2, W/2, -W/2, -W/2];
    set(h_link3, 'XData', pts3(1,:) + x2, 'YData', pts3(2,:) + y2); 
    
    % ---  집게발(Gripper) 그래픽 업데이트 (말단 위치 x3, y3 기준) ---
    g1 = R3 * [0, GripSize*cos(pi/6); W/2, W/2 + GripSize*sin(pi/6)];
    g2 = R3 * [0, GripSize*cos(-pi/6); -W/2, -W/2 + GripSize*sin(-pi/6)];
    set(h_grip1, 'XData', g1(1,:) + x3, 'YData', g1(2,:) + y3);
    set(h_grip2, 'XData', g2(1,:) + x3, 'YData', g2(2,:) + y3);
    
    % 타이틀 실시간 정보 업데이트
    title(sprintf('3-Link Manipulator Simulation | Time: %.2fs', t(i)));
    
    % 실시간 그래픽 렌더링 및 속도 딜레이 설정
    drawnow;            
    pause(0.015);       
end