%% MATLAB 2D 2-Link Manipulator: Pure Circular Trace (R2026a)
clear; clc; close all;

%% 1. 로봇 및 애니메이션 파라미터 설정
L1 = 0.45; % 링크 1 길이
L2 = 0.35; % 링크 2 길이
clawLen = 0.08; % 집게발 길이

numSteps = 120; % 애니메이션 정밀도 (프레임 수)
q1_fixed = deg2rad(45); % 1번 링크 고정 각도 (45도)
q2_angles = linspace(0, 2*pi, numSteps); % 2번 링크 0~360도 회전

%% 2. 시각화 창 설정
fig = figure('Color', 'w', 'Position', [100 100 800 800]);
ax = axes('Parent', fig); hold on; grid on; axis equal;

limit = 0.9;
axis([-limit limit -limit limit]);
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('2nd Link Pure Rotation Trace (1st Link Fixed)', 'FontSize', 15);

% 디자인 색상 설정
colorL1 = [0.2 0.2 0.2];     % 링크 1 (검정)
colorL2 = [0.1 0.4 0.8];     % 링크 2 (파랑)
colorJoint = [1.0 0.0 0.0];  % 관절 (빨강)
colorTrail = [1.0 0.5 0.5];  % 궤적 잔상 (연빨강)

% 궤적 라인 객체 (실시간 누적)
trail = animatedline('Color', colorTrail, 'LineWidth', 2);

% 그래픽 객체 핸들 저장 배열 초기화
hObjs = [];

%% 3. 회전 애니메이션 루프
for k = 1:numSteps
    % --- 이전 프레임의 로봇 형상 삭제 ---
    if ~isempty(hObjs)
        delete(hObjs(isvalid(hObjs)));
        hObjs = []; 
    end
    
    % 현재 각도 (1번 고정, 2번 회전)
    q1 = q1_fixed;
    q2 = q2_angles(k);
    
    % --- 순기구학 (좌표 계산) ---
    x0 = 0; y0 = 0; 
    x1 = L1 * cos(q1);
    y1 = L1 * sin(q1);
    
    % 2번 링크의 방향각은 q1 + q2
    phi = q1 + q2;
    x2 = x1 + L2 * cos(phi);
    y2 = y1 + L2 * sin(phi);
    
    % 2중 집게발 좌표
    gap = deg2rad(25);
    cx1 = x2 + clawLen * cos(phi + gap);
    cy1 = y2 + clawLen * sin(phi + gap);
    cx2 = x2 + clawLen * cos(phi - gap);
    cy2 = y2 + clawLen * sin(phi - gap);
    
    % --- 실시간 궤적 업데이트 (붉은 실선 잔상) ---
    addpoints(trail, x2, y2);
    
    % --- 로봇 형상 그리기 ---
    % 링크 1 & 2
    h1 = plot([x0 x1], [y0 y1], 'Color', colorL1, 'LineWidth', 12);
    h2 = plot([x1 x2], [y1 y2], 'Color', colorL2, 'LineWidth', 8);
    
    % 관절 및 베이스 포인트
    h3 = plot(x0, y0, 'ko', 'MarkerFaceColor', [0 0 0], 'MarkerSize', 14);
    h4 = plot(x1, y1, 'ko', 'MarkerFaceColor', colorJoint, 'MarkerSize', 9);
    h5 = plot(x2, y2, 'ko', 'MarkerFaceColor', colorJoint, 'MarkerSize', 6);
    
    % 2중 집게발 (말단 장치)
    h6 = plot([x2 cx1], [y2 cy1], 'Color', [0.4 0.4 0.4], 'LineWidth', 3);
    h7 = plot([x2 cx2], [y2 cy2], 'Color', [0.4 0.4 0.4], 'LineWidth', 3);
    
    % 다음 프레임에서 삭제할 객체 관리
    hObjs = [h1, h2, h3, h4, h5, h6, h7];

    % 화면 업데이트 속도 조절
    drawnow limitrate;
    pause(0.015);
end

% 회전 완료 후 중심점 텍스트 안내 (선택 사항)
text(x1 + 0.05, y1 + 0.05, 'Rotation Center', 'FontSize', 10, 'Color', colorJoint, 'FontWeight', 'bold');