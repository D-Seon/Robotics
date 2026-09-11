%% MATLAB 2D 2-Link Manipulator: High-Speed 2nd Link Rotation
clear; clc; close all;

%% 1. 로봇 및 애니메이션 파라미터 설정
L1 = 0.45; % 링크 1 길이
L2 = 0.35; % 링크 2 길이
clawLen = 0.08; % 집게발 길이

numSteps = 100; % 더 부드러운 움직임을 위해 스텝 수 증가
q1_angles = linspace(0, 2*pi, numSteps);     % 1번 링크: 1바퀴 회전
q2_angles = linspace(0, 2 * 2*pi, numSteps); % 2번 링크: 2바퀴 회전 (배속)

%% 2. 시각화 창 설정
fig = figure('Color', 'w', 'Position', [100 100 800 800]);
ax = axes('Parent', fig); hold on; grid on; axis equal;

limit = 0.9;
axis([-limit limit -limit limit]);
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('2-Link Manipulator: 2x Speed 2nd Link Rotation', 'FontSize', 15);

% 디자인 색상
colorL1 = [0.1 0.4 0.8];     
colorL2 = [0.3 0.3 0.3];     
colorJoint = [1.0 0.0 0.0];  

% 궤적 객체 (2번 링크의 속도가 빨라지면 독특한 '꽃잎' 모양의 궤적이 생깁니다)
trail1 = animatedline('Color', [0.7 0.8 1.0], 'LineWidth', 1); % 팔꿈치 궤적
trail2 = animatedline('Color', [1.0 0.4 0.4], 'LineWidth', 2); % 집게발 궤적

hObjs = [];

%% 3. 배속 회전 애니메이션 루프
for k = 1:numSteps
    if ~isempty(hObjs), delete(hObjs(isvalid(hObjs))); hObjs = []; end
    
    % 각도 적용 (q2가 q1보다 2배 빠르게 변화)
    q1 = q1_angles(k);
    q2 = q2_angles(k);
    
    % 순기구학 계산
    x0 = 0; y0 = 0; 
    x1 = L1 * cos(q1);
    y1 = L1 * sin(q1);
    
    phi = q1 + q2; % 전체 방향각
    x2 = x1 + L2 * cos(phi);
    y2 = y1 + L2 * sin(phi);
    
    % 집게발 좌표
    gap = deg2rad(25);
    cx1 = x2 + clawLen * cos(phi + gap); cy1 = y2 + clawLen * sin(phi + gap);
    cx2 = x2 + clawLen * cos(phi - gap); cy2 = y2 + clawLen * sin(phi - gap);
    
    % 궤적 업데이트
    addpoints(trail1, x1, y1);
    addpoints(trail2, x2, y2);
    
    % 그리기
    h1 = plot([x0 x1], [y0 y1], 'Color', colorL1, 'LineWidth', 12);
    h2 = plot([x1 x2], [y1 y2], 'Color', colorL2, 'LineWidth', 8);
    h3 = plot(x0, y0, 'ko', 'MarkerFaceColor', [0 0 0], 'MarkerSize', 14);
    h4 = plot(x1, y1, 'ko', 'MarkerFaceColor', colorJoint, 'MarkerSize', 9);
    h5 = plot(x2, y2, 'ko', 'MarkerFaceColor', colorJoint, 'MarkerSize', 6);
    h6 = plot([x2 cx1], [y2 cy1], 'Color', [0.4 0.4 0.4], 'LineWidth', 3);
    h7 = plot([x2 cx2], [y2 cy2], 'Color', [0.4 0.4 0.4], 'LineWidth', 3);
    
    hObjs = [h1, h2, h3, h4, h5, h6, h7];

    drawnow limitrate;
    pause(0.01);
end