%% Robotics System Toolbox: Orthogonal Dual Rotation (R2026a Verified)
clear; clc; close all;

%% 1. 로봇 모델 및 디자인 설정
robot = rigidBodyTree('DataFormat','column');
deepBlack = [0.1 0.1 0.1];
flameRed = [1.0 0.0 0.1];
darkSilver = [0.4 0.4 0.4];

%% 2. 링크 및 관절 설계
% --- Base (Z축 회전: 좌우 Yaw) ---
baseLink = rigidBody('base_link');
jBase = rigidBodyJoint('j_base', 'revolute');
jBase.JointAxis = [0 0 1]; 
baseLink.Joint = jBase;
addVisual(baseLink, "Cylinder", [0.12 0.04], trvec2tform([0 0 0.02]), 'FaceColor', deepBlack);
addBody(robot, baseLink, 'base');

% --- Link 1 (상하 Pitch 관절 기반) ---
L1 = 0.45; body1 = rigidBody('link1');
j1 = rigidBodyJoint('j1','revolute');
setFixedTransform(j1, trvec2tform([0 0 0.04])); 
j1.JointAxis = [0 1 0]; 
body1.Joint = j1;
addVisual(body1, "Cylinder", [0.03 L1], trvec2tform([0 0 L1/2]), 'FaceColor', deepBlack);
addVisual(body1, "Sphere", 0.05, trvec2tform([0 0 0]), 'FaceColor', flameRed);
addBody(robot, body1, 'base_link');

% --- Link 2 (Y축 회전: 상하 Pitch) ---
L2 = 0.35; body2 = rigidBody('link2');
j2 = rigidBodyJoint('j2','revolute');
setFixedTransform(j2, trvec2tform([0 0 L1])); 
j2.JointAxis = [0 1 0]; 
j2.PositionLimits = [-deg2rad(150), deg2rad(150)]; % 충돌 방지
body2.Joint = j2;
addVisual(body2, "Cylinder", [0.025 L2], trvec2tform([0 0 L2/2]), 'FaceColor', deepBlack);
addVisual(body2, "Sphere", 0.045, trvec2tform([0 0 0]), 'FaceColor', flameRed);
addBody(robot, body2, 'link1');

% --- 4중 집게발 ---
clawBase = rigidBody('claw_base');
setFixedTransform(clawBase.Joint, trvec2tform([0 0 L2]));
addVisual(clawBase, "Cylinder", [0.06 0.02], trvec2tform([0 0 0.01]), 'FaceColor', darkSilver);
addBody(robot, clawBase, 'link2');

for i = 1:4
    clawName = ['claw_' num2str(i)];
    claw = rigidBody(clawName);
    cJoint = rigidBodyJoint([clawName '_j'], 'revolute');
    angleTform = axang2tform([0 0 1 (i-1)*pi/2]);
    setFixedTransform(cJoint, angleTform * trvec2tform([0.04 0 0.02]));
    cJoint.JointAxis = [0 1 0]; 
    claw.Joint = cJoint;
    addVisual(claw, "Cylinder", [0.005 0.07], trvec2tform([0 0 0.035]), 'FaceColor', deepBlack); 
    addVisual(claw, "Sphere", 0.008, trvec2tform([0 0 0.07]), 'FaceColor', deepBlack);
    addBody(robot, claw, 'claw_base');
end

%% 3. 직교 회전 데이터 생성
numSteps = 60; % 구간당 스텝 수

% Phase 1: 2번 관절 상하(Pitch) 왕복
p1_angle = linspace(-deg2rad(120), deg2rad(120), numSteps);
% Phase 2: 베이스 관절 좌우(Yaw) 왕복
p2_angle = linspace(-deg2rad(90), deg2rad(90), numSteps);

totalSteps = numSteps * 2;
all_q = zeros(7, totalSteps);

% 데이터 할당
for k = 1:numSteps
    % Phase 1: 상하 회전 (Y축)
    all_q(1, k) = 0; 
    all_q(2, k) = pi/6; % 1번 링크 고정
    all_q(3, k) = p1_angle(k); 
    all_q(4:7, k) = pi/8;
    
    % Phase 2: 좌우 회전 (Z축)
    idx = k + numSteps;
    all_q(1, idx) = p2_angle(k);
    all_q(2, idx) = pi/6;
    all_q(3, idx) = 0; % 2번 관절 중립 고정
    all_q(4:7, idx) = pi/8;
end

%% 4. 시뮬레이션 구동
fig = figure('Color', [0.9 0.9 0.9], 'Position', [100 100 1000 800]);
ax = axes('Parent', fig); hold on; axis equal; view(135, 25);
grid on; axis([-0.8 0.8 -0.8 0.8 0 0.9]);
xlabel('X Axis'); ylabel('Y Axis'); zlabel('Z Axis');

trail = animatedline('Color', [1 0 0], 'LineWidth', 2);
camlight; lighting phong; material shiny;

for k = 1:totalSteps
    currentTform = getTransform(robot, all_q(:, k), 'claw_base');
    currentPos = currentTform(1:3, 4);
    addpoints(trail, currentPos(1), currentPos(2), currentPos(3));
    
    show(robot, all_q(:, k), 'Visuals', 'on', 'Frames', 'off', 'PreservePlot', false, 'Parent', ax);
    
    % [오류 수정 및 타이틀 개선]
    if k <= numSteps
        title('Phase 1: Vertical Rotation (Pitch)', 'FontSize', 16);
    else
        title('Phase 2: Horizontal Rotation (Yaw)', 'FontSize', 16);
    end
    
    drawnow limitrate;
    pause(0.015);
end
