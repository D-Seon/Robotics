clear; clc; close all;

%% 1. 파라미터 설정
L1 = 1.0; % 링크 1 길이
L2 = 1.0; % 링크 2 길이
W = 0.15; % 링크 두께 (두껍게 설정)
GripSize = 0.2; % 집게발 크기

% 시나리오 설정
cases = [2, 200;  % Case 1: 2초, 200샘플
         1, 100]; % Case 2: 1초, 100샘플

for c = 1:2
    T_final = cases(c, 1);
    N_samples = cases(c, 2);
    t = linspace(0, T_final, N_samples);
    
    % 관절 궤적 입력 함수 (2:3 리사주 패턴)
    theta1 = 2 * pi * t;                
    theta2 = pi * t + pi/2;             
    
    %% 2. 시각화 준비
    fig = figure('Name', sprintf('Case %d: Manipulator Simulation', c), 'Color', 'w');
    hold on; grid on; axis equal;
    xlim([-2.5 2.5]); ylim([-2.5 2.5]);
    xlabel('X [m]'); ylabel('Y [m]');
    
    % 궤적 저장을 위한 변수
    traj_x = []; traj_y = [];
    h_traj = plot(0, 0, 'r:', 'LineWidth', 1.5); % 궤적 점선
    
    % 링크 및 집게발 객체(Patch) 초기화
    % 링크 1
    h_link1 = patch(0, 0, [0.2 0.2 0.2], 'EdgeColor', 'k', 'LineWidth', 1.5);
    % 링크 2
    h_link2 = patch(0, 0, [0.4 0.4 0.4], 'EdgeColor', 'k', 'LineWidth', 1.5);
    % 집게발 1, 2
    h_grip1 = plot([0 0], [0 0], 'k', 'LineWidth', 3);
    h_grip2 = plot([0 0], [0 0], 'k', 'LineWidth', 3);
    
    %% 3. 시뮬레이션 루프
    for i = 1:N_samples
        th1 = theta1(i);
        th2 = theta2(i);
        th12 = th1 + th2; % 절대각
        
        % 관절 좌표 계산
        x0 = 0; y0 = 0;
        x1 = L1 * cos(th1);
        y1 = L1 * sin(th1);
        x2 = x1 + L2 * cos(th12);
        y2 = y1 + L2 * sin(th12);
        
        % 궤적 업데이트
        traj_x = [traj_x, x2];
        traj_y = [traj_y, y2];
        set(h_traj, 'XData', traj_x, 'YData', traj_y);
        
        % --- 링크 1 그리기 (두꺼운 직사각형) ---
        R1 = [cos(th1) -sin(th1); sin(th1) cos(th1)];
        link1_pts = [0, L1, L1, 0; W/2, W/2, -W/2, -W/2];
        link1_rot = R1 * link1_pts;
        set(h_link1, 'XData', link1_rot(1,:) + x0, 'YData', link1_rot(2,:) + y0);
        
        % --- 링크 2 그리기 (두꺼운 직사각형) ---
        R2 = [cos(th12) -sin(th12); sin(th12) cos(th12)];
        link2_pts = [0, L2, L2, 0; W/2, W/2, -W/2, -W/2];
        link2_rot = R2 * link2_pts;
        set(h_link2, 'XData', link2_rot(1,:) + x1, 'YData', link2_rot(2,:) + y1);
        
        % --- 집게발(Gripper) 계산 및 그리기 ---
        % 말단 장치에서 양옆으로 벌어진 두 개의 발
        grip_base_rot = R2 * [L2; 0]; % 말단 지점
        
        % 집게발 1 (위쪽)
        g1_pts = R2 * [L2, L2 + GripSize*cos(pi/6); W/2, W/2 + GripSize*sin(pi/6)];
        set(h_grip1, 'XData', g1_pts(1,:) + x1, 'YData', g1_pts(2,:) + y1);
        
        % 집게발 2 (아래쪽)
        g2_pts = R2 * [L2, L2 + GripSize*cos(-pi/6); -W/2, -W/2 + GripSize*sin(-pi/6)];
        set(h_grip2, 'XData', g2_pts(1,:) + x1, 'YData', g2_pts(2,:) + y1);
        
        title(sprintf('Time: %.2f s (Case %d)', t(i), c));
        drawnow;
        if c == 1, pause(0.01); else, pause(0.02); end
    end
end