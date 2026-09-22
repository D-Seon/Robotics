%% MATLAB 2D 2-Link Manipulator: Link 1 (Theta 1) Focus Trace
clear; clc; close all;

%% 1. 로봇 및 애니메이션 파라미터 설정
L1 = 0.50; % 링크 1 길이 (주동 팔)
L2 = 0.35; % 링크 2 길이 (움직임 제한됨)
clawLen = 0.07;

numSteps = 150; % 애니메이션 정밀도
% Theta 1 (Link 1)만 0에서 360도까지 회전
theta1_angles = linspace(0, 2*pi, numSteps); 
% Theta 2 (Joint 2)는 0도로 고정하여 Link 2의 상대적 움직임을 제한
theta2_fixed = 0; 

%% 2. 시각화 창 설정 (중앙 배치)
fig = figure('Color', 'w', 'Position', [100 100 800 800]);
ax = axes('Parent', fig); hold on; grid on; axis equal;

limit = 1.0;
axis([-limit limit -limit limit]);
xlabel('X Axis (m)'); ylabel('Y Axis (m)');
title('Manipulator Movement: Focus on \theta_1 (Link 1)', 'FontSize', 15);

% 디자인 색상
colorLink1 = [0.1 0.4 0.8];     % 링크 1 (메인 동력 - 파랑)
colorLink2 = [0.4 0.4 0.4];     % 링크 2 (고정된 상태 - 회색)
colorJoint1 = [0.1 0.1 0.1];    % 베이스 관절
colorJoint2 = [1.0 0.0 0.0];    % 궤적을 그리는 Joint 2 (빨강)

% 궤적 객체: Link 1의 끝단(즉, Joint 2의 위치)을 추적
theta1_trail = animatedline('Color', colorJoint2, 'LineWidth', 2, 'LineStyle', '-');

hObjs = []; % 그래픽 핸들 저장용

%% 3. Theta 1 주동 애니메이션 루프
for k = 1:numSteps
    % 이전 프레임 삭제
    if ~isempty(hObjs), delete(hObjs(isvalid(hObjs))); hObjs = []; end
    
    % 현재 각도 설정
    t1 = theta1_angles(k);
    t2 = theta2_fixed; 
    
    % --- 순기구학 계산 ---
    % 베이스 (중심)
    x0 = 0; y0 = 0;
    
    % [관절 2 위치] Link 1의 끝점
    x1 = L1 * cos(t1);
    y1 = L1 * sin(t1);
    
    % [말단 위치] Link 2의 끝점 (Link 1과 일직선 유지)
    phi = t1 + t2;
    x2 = x1 + L2 * cos(phi);
    y2 = y1 + L2 * sin(phi);
    
    % 집게발 좌표
    gap = deg2rad(20);
    cx1 = x2 + clawLen * cos(phi + gap); cy1 = y2 + clawLen * sin(phi + gap);
    cx2 = x2 + clawLen * cos(phi - gap); cy2 = y2 + clawLen * sin(phi - gap);
    
    % --- 궤적 업데이트: Joint 2의 이동 경로 ---
    addpoints(theta1_trail, x1, y1);
    
    % --- 시각화 그리기 ---
    % 링크 1 (Theta 1에 의해 회전)
    h1 = plot([x0 x1], [y0 y1], 'Color', colorLink1, 'LineWidth', 12);
    % 링크 2 (1번 링크에 고정되어 따라감)
    h2 = plot([x1 x2], [y1 y2], 'Color', colorLink2, 'LineWidth', 7);
    
    % 관절 포인트
    h3 = plot(x0, y0, 'ko', 'MarkerFaceColor', colorJoint1, 'MarkerSize', 12); % 베이스
    h4 = plot(x1, y1, 'ko', 'MarkerFaceColor', colorJoint2, 'MarkerSize', 10); % Joint 2
    
    % 집게발
    h5 = plot([x2 cx1], [y2 cy1], 'Color', colorLink2, 'LineWidth', 3);
    h6 = plot([x2 cx2], [y2 cy2], 'Color', colorLink2, 'LineWidth', 3);
    
    hObjs = [h1, h2, h3, h4, h5, h6];

    % 실시간 각도 표시 텍스트
    h7 = text(-0.9, -0.9, sprintf('\\theta_1: %.1f^\\circ', rad2deg(t1)), 'FontSize', 12, 'FontWeight', 'bold');
    hObjs = [hObjs, h7];

    drawnow limitrate;
    pause(0.01);
end

legend(theta1_trail, 'Trajectory of Link 1 End (Joint 2)', 'Location', 'northeast');
