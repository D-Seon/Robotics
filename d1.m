% High-Fidelity 3D-Style 2-Link Manipulator Simulation (MATLAB R2026a)
clear; clc; close all;

%% 1. 로봇 물리 및 궤적 파라미터 설정
l1 = 1.0; % 링크 1 길이 (m)
l2 = 1.0; % 링크 2 길이 (m)

% 타원 궤적 파라미터 (x^2/2^2 + y^2/1.3^2 = 1)
a = 2.0;  % 장반경 (초기치 x=2.0 만족)
b = 1.3;  % 단반경

% 시간 및 스텝 설정 (부드러운 구동을 위해 240 프레임 구성)
num_steps = 240;
t = linspace(0, 2*pi, num_steps);

% 타스크 공간 궤적 (X, Y) 생성
X_traj = a * cos(t);
Y_traj = b * sin(t);

%% 2. 시각화 그래픽스 환경 구축 (Dark Industrial Theme)
fig = figure('Name', 'Cybernetic 2-Link Manipulator HUD', ...
             'NumberTitle', 'off', ...
             'Color', [0.08, 0.09, 0.11]); % 깊은 우주 테마 다크 백그라운드
         
ax = axes('Parent', fig, ...
          'Color', [0.12, 0.14, 0.18], ...
          'XColor', [0.4, 0.45, 0.55], ...
          'YColor', [0.4, 0.45, 0.55], ...
          'GridColor', [0.25, 0.28, 0.35], ...
          'GridAlpha', 0.4);
      
grid on; hold on;
axis equal;
xlim([-2.5, 2.5]);
ylim([-2.5, 2.5]);
xlabel('X-Axis (Workspace / m)', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
ylabel('Y-Axis (Workspace / m)', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
title('PRO-MANIPULATOR Series-2: Elliptical Path Tracking', 'Color', [0.9, 0.95, 1.0], 'FontSize', 12, 'FontName', 'Segoe UI', 'FontWeight', 'bold');

% 타원형 타스크 가이드라인 (야광 그리드 느낌의 네온 네이비 블루 점선)
plot(X_traj, Y_traj, ':', 'Color', [0.0, 0.8, 1.0], 'LineWidth', 1.5);

% 실시간 핸드 궤적 히스토리 라인 (네온 마젠타 글로우)
hand_trace = plot(X_traj(1), Y_traj(1), '-', 'Color', [1.0, 0.2, 0.6], 'LineWidth', 2.0);

%% 3. 멋진 고정용 베이스 받침대 (Static Base Stand) 그리기
% 하단 고정 플레이트 (묵직한 주철 메탈 질감)
base_plate_x = [-0.45, 0.45, 0.30, -0.30];
base_plate_y = [-0.60, -0.60, -0.15, -0.15];
fill(base_plate_x, base_plate_y, [0.18, 0.20, 0.23], 'EdgeColor', [0.1, 0.11, 0.12], 'LineWidth', 2);

% 지지 볼트 디테일 (좌우 대칭 구조)
fill([-0.38, -0.34, -0.34, -0.38], [-0.58, -0.58, -0.54, -0.54], [0.3, 0.32, 0.35], 'EdgeColor', [0.1, 0.1, 0.1]);
fill([ 0.34,  0.38,  0.38,  0.34], [-0.58, -0.58, -0.54, -0.54], [0.3, 0.32, 0.35], 'EdgeColor', [0.1, 0.1, 0.1]);

% 베이스 메인 회전 목(Neck) 파트
base_neck_x = [-0.15, 0.15, 0.10, -0.10];
base_neck_y = [-0.15, -0.15, 0.0, 0.0];
fill(base_neck_x, base_neck_y, [0.55, 0.58, 0.62], 'EdgeColor', [0.3, 0.32, 0.35], 'LineWidth', 1.5);

% 1축 메탈 서보 모터 기어 (코퍼/골드 레이어)
rectangle('Position', [-0.18, -0.18, 0.36, 0.36], 'Curvature', [1, 1], ...
          'FaceColor', [0.82, 0.50, 0.15], 'EdgeColor', [0.5, 0.3, 0.1], 'LineWidth', 1.5);
rectangle('Position', [-0.12, -0.12, 0.24, 0.24], 'Curvature', [1, 1], ...
          'FaceColor', [1.0, 0.68, 0.25], 'EdgeColor', [0.6, 0.4, 0.1]);

%% 4. 메탈릭 실버 로봇 팔 (Layered Lines) 및 집게(Gripper) 오브젝트 정의
% [Layered Line 기법] 두께가 다른 은색 음영 라인을 겹쳐 2D에서 완벽한 원통형 광택 렌더링 구현
silver_colors = {
    [0.18, 0.19, 0.20], ... % 1. 가장 외곽 어두운 음영 테두리 (가장 두꺼움)
    [0.48, 0.50, 0.53], ... % 2. 기본 실버 메탈 바디
    [0.72, 0.75, 0.78], ... % 3. 중간 밝기 메탈 광택
    [0.90, 0.93, 0.96], ... % 4. 강한 스펙큘러 하이라이트
    [1.00, 1.00, 1.00]      % 5. 핵심 코어 반사광 (가장 얇음)
};
widths = [26, 21, 14, 7, 2.5];

link1_layers = []; % 빈 배열로 안전하게 초기화
link2_layers = [];

for k = 1:5
    % 겹쳐 그릴 은색 라인들을 배열에 차례대로 담아줍니다.
    link1_layers(k) = plot([0, 0], [0, 0], 'Color', silver_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
    link2_layers(k) = plot([0, 0], [0, 0], 'Color', silver_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
end

% 집게 로컬 좌표계 디자인 (집게 손바닥 기준점 0,0 에서 뻗어나가는 형태)
% 손목 브래킷 브릿지
wrist_x = [-0.03,  0.03,  0.03, -0.03];
wrist_y = [-0.05, -0.05,  0.05,  0.05];

% 집게 발톱 (Claw) - 날렵하고 멋진 하이테크 곡선형 핑거
claw_base_x = [0.03, 0.11, 0.19, 0.23, 0.20, 0.12, 0.03];
claw_base_y = [0.05, 0.09, 0.07, 0.02, -0.01, 0.03, 0.01]; % 왼편(상부) claw

% 집게 발톱 끝 고대비 네온 오렌지 패드 (실제 물건을 안전하게 잡는 특수 고무 패드 느낌)
pad_base_x = [0.15, 0.23, 0.20];
pad_base_y = [0.07, 0.02, -0.01];

% 집게 그래픽스 패치 생성 (초기 상태 플롯)
h_wrist = patch('XData', wrist_x, 'YData', wrist_y, 'FaceColor', [0.25, 0.27, 0.30], 'EdgeColor', [0.1, 0.1, 0.1], 'LineWidth', 1.2);
h_claw_L = patch('XData', claw_base_x, 'YData', claw_base_y, 'FaceColor', [0.75, 0.78, 0.82], 'EdgeColor', [0.3, 0.32, 0.35], 'LineWidth', 1.2);
h_claw_R = patch('XData', claw_base_x, 'YData', -claw_base_y, 'FaceColor', [0.75, 0.78, 0.82], 'EdgeColor', [0.3, 0.32, 0.35], 'LineWidth', 1.2);
h_pad_L = patch('XData', pad_base_x, 'YData', pad_base_y, 'FaceColor', [1.0, 0.45, 0.0], 'EdgeColor', 'none');
h_pad_R = patch('XData', pad_base_x, 'YData', -pad_base_y, 'FaceColor', [1.0, 0.45, 0.0], 'EdgeColor', 'none');

% 관절 캡 장식 (황금빛 알루미늄 피벗 볼트 포인트)
h_joint1_cap = plot(0, 0, 'o', 'MarkerSize', 14, 'MarkerFaceColor', [0.90, 0.70, 0.10], 'MarkerEdgeColor', [0.1, 0.1, 0.1], 'LineWidth', 1.5);
h_joint2_cap = plot(0, 0, 'o', 'MarkerSize', 12, 'MarkerFaceColor', [0.90, 0.70, 0.10], 'MarkerEdgeColor', [0.1, 0.1, 0.1], 'LineWidth', 1.5);
h_wrist_cap  = plot(0, 0, 'o', 'MarkerSize',  8, 'MarkerFaceColor', [0.00, 0.80, 1.00], 'MarkerEdgeColor', [0.1, 0.1, 0.1], 'LineWidth', 1.5);

%% 5. 실시간 정보 출력 윈도우 (HUD Dashboard)
hud_box = annotation('textbox', [0.15, 0.68, 0.28, 0.20], ...
                     'String', '', ...
                     'Color', [0.0, 0.95, 1.0], ...
                     'BackgroundColor', [0.15, 0.17, 0.22, 0.75], ...
                     'EdgeColor', [0.0, 0.5, 0.7], ...
                     'FontName', 'Courier New', ...
                     'FontSize', 9.5, ...
                     'FontWeight', 'bold');

%% 6. 핵심 제어 시뮬레이션 루프
x_history = [];
y_history = [];

for i = 1:num_steps
    % 목표 위치 추출
    x = X_traj(i);
    y = Y_traj(i);
    
    %% [역기구학 계산]
    D = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2);
    if D > 1, D = 1; elseif D < -1, D = -1; end % 수치 특이점 제어
    
    % Elbow-down 형상 적용
    theta2 = -acos(D); 
    theta1 = atan2(y, x) - atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));
    
    %% [순기구학 및 각 관절 글로벌 좌표 유도]
    x1 = l1 * cos(theta1);
    y1 = l1 * sin(theta1);
    x2 = x1 + l2 * cos(theta1 + theta2);
    y2 = y1 + l2 * sin(theta1 + theta2);
    
    % 핸드의 각도 (집게가 향할 방위각)
    phi = theta1 + theta2;
    
    %% [입체 메탈 암(Arm) 업데이트]
    for k = 1:5
        set(link1_layers(k), 'XData', [0, x1], 'YData', [0, y1]);
        set(link2_layers(k), 'XData', [x1, x2], 'YData', [y1, y2]);
    end
    
    % 관절 피벗 볼트 중심점 동적 추적
    set(h_joint2_cap, 'XData', x1, 'YData', y1);
    set(h_wrist_cap, 'XData', x2, 'YData', y2);
    
    %% [동적 3D-Feel 집게 구동 알고리즘]
    % 타원 궤적의 상단/우측 구간에 따라 집게의 조임 간격(Grip Gap)을 능동적으로 연동 제어
    grip_control = 0.65 + 0.35 * sin(t(i)); % 0.3 ~ 1.0 사이에서 오르내림 (실제 집게 장치 구동 모사)
    
    claw_dyn_y = claw_base_y * grip_control;
    pad_dyn_y = pad_base_y * grip_control;
    
    % 글로벌 좌표 회전 및 변환 행렬 구성 [2x2]
    R = [cos(phi), -sin(phi); sin(phi), cos(phi)];
    
    % 각 손가락 부위별 회전 행렬 매핑 연산
    wr_g = R * wrist_loc_transform(wrist_x, wrist_y) + [x2; y2];
    cl_L_g = R * [claw_base_x;  claw_dyn_y] + [x2; y2];
    cl_R_g = R * [claw_base_x; -claw_dyn_y] + [x2; y2];
    pd_L_g = R * [pad_base_x;   pad_dyn_y]  + [x2; y2];
    pd_R_g = R * [pad_base_x;  -pad_dyn_y]  + [x2; y2];
    
    % 집게 패치 동적 재생성
    set(h_wrist, 'XData', wr_g(1,:), 'YData', wr_g(2,:));
    set(h_claw_L, 'XData', cl_L_g(1,:), 'YData', cl_L_g(2,:));
    set(h_claw_R, 'XData', cl_R_g(1,:), 'YData', cl_R_g(2,:));
    set(h_pad_L, 'XData', pd_L_g(1,:), 'YData', pd_L_g(2,:));
    set(h_pad_R, 'XData', pd_R_g(1,:), 'YData', pd_R_g(2,:));
    
    %% [궤적 그리기]
    x_history = [x_history, x2];
    y_history = [y_history, y2];
    set(hand_trace, 'XData', x_history, 'YData', y_history);
    
    %% [HUD 정보창 갱신]
    hud_string = sprintf([ ...
        '  [SYSTEM MONITOR]\n', ...
        '  ====================\n', ...
        '  Joint 1 Angle : %6.1f deg\n', ...
        '  Joint 2 Angle : %6.1f deg\n', ...
        '  Wrist Coord X : %6.3f m\n', ...
        '  Wrist Coord Y : %6.3f m\n', ...
        '  Grip Aperture : %5.1f %%\n', ...
        '  ====================\n', ...
        '  STATUS        : RUNNING...'], ...
        rad2deg(theta1), rad2deg(theta2), x2, y2, grip_control * 100);
    set(hud_box, 'String', hud_string);
    
    %% 프레임 렌더링 동기화 및 지연 속도 제어
    drawnow;
    pause(0.02);
end

%% 손목 변환 도우미 함수 정의
function out = wrist_loc_transform(wx, wy)
    out = [wx; wy];
end