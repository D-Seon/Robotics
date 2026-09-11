clear; clc; close all;

%% 1. 파라미터 및 환경 설정
L1 = 1.0;               % 첫 번째 링크 길이 (고정)
W = 0.12;               % 링크 두께
GripSize = 0.15;        % 집게발 크기

dt = 0.01;              % Time Interval (Delta t)
T_final = 2.0;          % 총 구동 시간 (2초)
t = 0:dt:T_final;       % 시간 벡터 생성
N = length(t);

f1 = 0.5;               % 회전 주파수 (0.5 Hz)
f2 = 1.0;               % 직선 운동 주파수 (1.0 Hz)

%% 2. 관절 궤적 입력 함수 정의
% 세타1: 2*pi*f1*t -> 2초 동안 1바퀴 회전 (0 ~ 2*pi)
theta1 = 2 * pi * f1 * t; 

% r (링크 2의 길이): 0.5*f2*t + 0.5 -> t=0일 때 0.5m, t=2일 때 1.5m로 선형 증가
r = 0.5 * f2 * t + 0.5; 

%% 3. 시각화 창 설정 
figure('Name', 'RP Manipulator Simulation', 'Color', 'w', 'Position', [100 100 850 850]);
hold on; grid on; axis equal;

% 로봇이 최대 확장했을 때(1.0 + 1.5 = 2.5m)를 고려한 화면 범위 설정
xlim([-2.8 2.8]); ylim([-2.8 2.8]);
xlabel('X [m]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y [m]', 'FontSize', 12, 'FontWeight', 'bold');
title('RP Manipulator: End-Effector Trajectory (Y vs X)', 'FontSize', 14, 'FontWeight', 'bold');

%% 4. 시각화 객체(그래픽 요소) 초기화
% 말단 장치 궤적 선 (점선 스타일)
h_traj = plot(0, 0, 'r--', 'LineWidth', 2); 

% 링크 1 (진한 회색 패치)
h_link1 = patch(0, 0, [0.2 0.2 0.2], 'EdgeColor', 'k', 'LineWidth', 1.5);

% 링크 2 (Prismatic - 연한 회색 패치, 내부 슬라이더 느낌)
h_link2 = patch(0, 0, [0.6 0.6 0.6], 'EdgeColor', 'k', 'LineWidth', 1.2);

% 말단 장치 집게발 (검은색 두꺼운 선으로 뚜렷하게 분리)
h_grip1 = plot([0 0], [0 0], 'k', 'LineWidth', 2.5);
h_grip2 = plot([0 0], [0 0], 'k', 'LineWidth', 2.5);

% 관절 포인트 표시 (중앙 Base 및 Joint 1)
plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'w', 'LineWidth', 2); % 원점 Base
h_joint1 = plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', [0.3 0.6 0.9], 'LineWidth', 1.5);

%% 5. 시뮬레이션 루프 실행
traj_x = zeros(1, N);
traj_y = zeros(1, N);

for i = 1:N
    th1 = theta1(i);
    current_r = r(i); % 실시간으로 늘어나는 링크 2의 길이
    
    % --- 정기구학(Forward Kinematics) 좌표 계산 ---
    % 원점(Base)은 모니터 정중앙 (0,0)
    x0 = 0; y0 = 0;
    
    % Joint 1 위치 (첫 번째 회전 링크의 끝점)
    x1 = L1 * cos(th1);
    y1 = L1 * sin(th1);
    
    % 말단 장치(End-Effector) 위치
    % 2축이 Prismatic joint이므로 방향은 th1과 같고, 거리는 L1 + r이 됨
    x2 = (L1 + current_r) * cos(th1);
    y2 = (L1 + current_r) * sin(th1);
    
    % 궤적 데이터 저장 및 업데이트
    traj_x(i) = x2;
    traj_y(i) = y2;
    set(h_traj, 'XData', traj_x(1:i), 'YData', traj_y(1:i));
    
    % --- 그래픽 업데이트: 링크 1 (회전축) ---
    R1 = [cos(th1) -sin(th1); sin(th1) cos(th1)];
    pts1 = R1 * [0, L1, L1, 0; W/2, W/2, -W/2, -W/2];
    set(h_link1, 'XData', pts1(1,:) + x0, 'YData', pts1(2,:) + y0);
    
    % --- 그래픽 업데이트: 링크 2 (직선 정밀 제어축) ---
    % 두께를 링크 1보다 살짝 가늘게 하여 '들어가고 나가는 모양'을 시각화
    W2 = W * 0.7; 
    pts2 = R1 * [L1, L1+current_r, L1+current_r, L1; W2/2, W2/2, -W2/2, -W2/2];
    set(h_link2, 'XData', pts2(1,:) + x0, 'YData', pts2(2,:) + y0);
    
    % Joint 1 센터 마커 위치 업데이트
    set(h_joint1, 'XData', x1, 'YData', y1);
    
    % --- 그래픽 업데이트: 집게발(Gripper) ---
    % 말단 장치에서 뚜렷하게 분리되어 보이도록 방향 벡터 기준 그리기
    g1 = R1 * [L1+current_r, L1+current_r+GripSize*cos(pi/6); W2/2, W2/2+GripSize*sin(pi/6)];
    g2 = R1 * [L1+current_r, L1+current_r+GripSize*cos(-pi/6); -W2/2, -W2/2+GripSize*sin(-pi/6)];
    set(h_grip1, 'XData', g1(1,:) + x0, 'YData', g1(2,:) + y0);
    set(h_grip2, 'XData', g2(1,:) + x0, 'YData', g2(2,:) + y0);
    
    % 실시간 타이틀 업데이트
    title(sprintf('RP Manipulator Simulation | Time: %.2fs | Length r: %.2fm', t(i), current_r));
    
    drawnow;
    pause(0.01); % 실시간 속도감 유지
end