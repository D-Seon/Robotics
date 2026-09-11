clear; clc; close all;

% 기본 파라미터
L1 = 1.0; L2 = 1.0; W = 0.15; GripSize = 0.2;
t = linspace(0, 2, 100); % 2초 동안 100샘플

%% 시나리오 1: Link 1 고정, Link 2만 회전
figure('Name', 'Scenario 1: Link 2 Rotation Only', 'Color', 'w');
for i = 1:length(t)
    th1 = pi/4; % 45도로 고정
    th2 = 2*pi*t(i); % Link 2만 회전
    draw_robot(th1, th2, L1, L2, W, GripSize, 'Link 1 Fixed, Link 2 Rotating');
    drawnow;
end

%% 시나리오 2: Link 2 고정, Link 1만 회전
figure('Name', 'Scenario 2: Link 1 Rotation Only', 'Color', 'w');
for i = 1:length(t)
    th1 = 2*pi*t(i); % Link 1만 회전
    th2 = 0; % Link 2를 일직선으로 고정
    draw_robot(th1, th2, L1, L2, W, GripSize, 'Link 2 Fixed, Link 1 Rotating');
    drawnow;
end


% 로봇 그리기 보조 함수
function draw_robot(th1, th2, L1, L2, W, GripSize, txt)
cla; hold on; grid on; axis equal; xlim([-2.5 2.5]); ylim([-2.5 2.5]);
th12 = th1 + th2;
x0 = 0; y0 = 0;
x1 = L1 * cos(th1); y1 = L1 * sin(th1);
x2 = x1 + L2 * cos(th12); y2 = y1 + L2 * sin(th12);

% Link 1 (Patch)
R1 = [cos(th1) -sin(th1); sin(th1) cos(th1)];
pts1 = R1 * [0, L1, L1, 0; W/2, W/2, -W/2, -W/2];
patch(pts1(1,:), pts1(2,:), [0.2 0.2 0.2]);

% Link 2 (Patch)
R2 = [cos(th12) -sin(th12); sin(th12) cos(th12)];
pts2 = R2 * [0, L2, L2, 0; W/2, W/2, -W/2, -W/2];
patch(pts2(1,:) + x1, pts2(2,:) + y1, [0.4 0.4 0.4]);

% Gripper
g1 = R2 * [L2, L2+GripSize*cos(pi/6); W/2, W/2+GripSize*sin(pi/6)];
g2 = R2 * [L2, L2+GripSize*cos(-pi/6); -W/2, -W/2+GripSize*sin(-pi/6)];
plot(g1(1,:) + x1, g1(2,:) + y1, 'k', 'LineWidth', 2);
plot(g2(1,:) + x1, g2(2,:) + y1, 'k', 'LineWidth', 2);
title(txt);
end