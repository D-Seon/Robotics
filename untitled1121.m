clear; clc; close all;

%% 1. 파라미터 설정
L1 = 1.0; L2 = 1.0;     % 링크 길이
W = 0.15;               % 링크 두께
GripSize = 0.2;         % 집게발 크기

% 시간 벡터 설정
t2 = linspace(0, 2, 200); % 시나리오 3용 (2초, 200샘플)
t1 = linspace(0, 1, 100); % 시나리오 1, 2, 4용 (1초, 100샘플)
t2_50 = linspace(0, 2, 50);

% 초기 자세 조건: theta1 = 0, theta2 = pi/2
scenarios = { 
    % 시나리오 1: Link 1은 0으로 고정, Link 2만 pi/2에서 시작하여 회전
    'Scenario 1: Link 2 Only (Fixed th1=0)', repmat(0, 1, 100), 2*pi*t1 + pi/2, t1;

    % 시나리오 2: Link 2는 pi/2로 고정, Link 1만 0에서 시작하여 회전
    'Scenario 2: Link 1 Only (Fixed th2=pi/2)', 2*pi*t1, repmat(pi/2, 1, 100), t1;

    % 시나리오 3: 전체 리사주 패턴 (2초) - 시작점 theta1=0, theta2=pi/2
    'Scenario 3: Full Lissajous (T=2s, 200pts)', 2*pi*1*t2, 2*pi*0.5*t2 + pi/2, t2;

    % 시나리오 4: 부분 리사주 패턴 (1초) - 시작점 theta1=0, theta2=pi/2
    'Scenario 4: Rough Lissajous (T=2s, 50pts)', 2*pi*1*t2_50, 2*pi*0.5*t2_50 + pi/2, t2_50
    };

%% 2. 시뮬레이션 실행
for s = 1:4
    current_title = scenarios{s, 1};
    th1_array = scenarios{s, 2};
    th2_array = scenarios{s, 3};
    t_array = scenarios{s, 4};
    N = length(t_array);

    figure('Name', current_title, 'Color', 'w', 'Position', [100 100 800 800]);
    hold on; grid on; axis equal;
    xlim([-2.5 2.5]); ylim([-2.5 2.5]);
    xlabel('X [m]'); ylabel('Y [m]');

    h_traj = plot(0, 0, 'r:', 'LineWidth', 1.2); % 궤적 선
    h_link1 = patch(0, 0, [0.2 0.2 0.2], 'EdgeColor', 'k');
    h_link2 = patch(0, 0, [0.4 0.4 0.4], 'EdgeColor', 'k');
    h_grip1 = plot([0 0], [0 0], 'k', 'LineWidth', 2);
    h_grip2 = plot([0 0], [0 0], 'k', 'LineWidth', 2);

    traj_x = []; traj_y = [];

    for i = 1:N
        th1 = th1_array(i);
        th2 = th2_array(i);
        th12 = th1 + th2; % 절대각 계산

        % 정기구학 좌표 계산
        x1 = L1 * cos(th1); y1 = L1 * sin(th1);
        x2 = x1 + L2 * cos(th12); y2 = y1 + L2 * sin(th12);

        % 궤적 업데이트
        traj_x = [traj_x, x2]; traj_y = [traj_y, y2];
        set(h_traj, 'XData', traj_x, 'YData', traj_y);

        % 링크 1 그리기
        R1 = [cos(th1) -sin(th1); sin(th1) cos(th1)];
        pts1 = R1 * [0, L1, L1, 0; W/2, W/2, -W/2, -W/2];
        set(h_link1, 'XData', pts1(1,:), 'YData', pts1(2,:));

        % 링크 2 그리기
        R2 = [cos(th12) -sin(th12); sin(th12) cos(th12)];
        pts2 = R2 * [0, L2, L2, 0; W/2, W/2, -W/2, -W/2];
        set(h_link2, 'XData', pts2(1,:) + x1, 'YData', pts2(2,:) + y1);

        % 집게발(Gripper) 그리기
        g1 = R2 * [L2, L2 + GripSize*cos(pi/6); W/2, W/2 + GripSize*sin(pi/6)];
        g2 = R2 * [L2, L2 + GripSize*cos(-pi/6); -W/2, -W/2 + GripSize*sin(-pi/6)];
        set(h_grip1, 'XData', g1(1,:) + x1, 'YData', g1(2,:) + y1);
        set(h_grip2, 'XData', g2(1,:) + x1, 'YData', g2(2,:) + y1);

        title([current_title, sprintf(' (Time: %.2fs)', t_array(i))]);
        drawnow;
        pause(0.01);
    end
end