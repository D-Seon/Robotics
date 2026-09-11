%% Computer Problem Set 5: Inverse Kinematics with Advanced Visualization
% 2-Link Planar Manipulator (a1=a2=1)
% 8 target positions solved via Newton-Raphson (Jacobian IK)
% Visualized with Advanced HUD, Real-time Error Graph, and Futuristic Gripper

clear; clc; close all;

%% ── Robot & solver parameters ────────────────────────────────────────
a1 = 1.0;  a2 = 1.0; % 링크 길이 (m)
tolerance = 0.01;      % IK 수렴 허용 오차 (m)
max_iter  = 300;      % 포인트당 최대 반복 횟수

%% ── 8 Points Definition: [x_des, y_des, theta1_guess, theta2_guess] ──
points = [
    1.2,  0.8,   0.5,  0.5;
    0.5,  1.5,   0.8,  0.6;
   -0.8,  1.2,   1.8,  0.5;
   -1.5,  0.5,   2.2,  0.4;
   -1.2, -0.8,  -2.5,  0.5;
   -0.5, -1.5,  -2.0,  0.6;
    0.8, -1.2,  -0.8,  0.5;
    1.5, -0.5,  -0.4,  0.4;
];
n_pts = size(points, 1);

%% ── 1. Initialize Visualizer (Dual-Screen Layout) ────────────────────
fig = figure('Name', 'Advanced IK Solver HUD', 'NumberTitle', 'off', ...
             'Color', [0.08, 0.09, 0.11], 'Position', [50, 100, 1400, 700]);

% 8개 포인트 색상 정의 (tab10 스타일 프리셋)
colors8 = lines(n_pts);

% 시각화 환경 구축 및 그래픽스 핸들 객체 수신
[ax1, ax2, visual_objs] = setup_visualization_environment(fig, a1, a2, points, colors8, tolerance, max_iter);

% 오버레이 HUD 텍스트박스 선언
hud_box = annotation('textbox', [0.13, 0.65, 0.18, 0.22], 'Color', [0.98, 0.88, 0.55], ...
                     'BackgroundColor', [0.15, 0.17, 0.22, 0.85], 'EdgeColor', [0.6, 0.4, 0.1], ...
                     'FontName', 'Courier New', 'FontSize', 9, 'FontWeight', 'bold');

%% ── 2. Sequential IK Solving & Animation Loop ────────────────────────
fprintf('========== IK Solutions (Newton-Raphson) ==========\n');
fprintf('%-6s %-14s %-14s %-14s %-14s %-10s %-8s\n',...
        'Point','x_des','y_des','theta1','theta2','error','iters');
fprintf('%s\n', repmat('-',1,75));

% 초기 로봇 상태 (Home Position) 설정
current_theta = [0; 0];
[x_curr, y_curr] = fk(current_theta(1), current_theta(2), a1, a2);

full_ee_history = [x_curr; y_curr]; % 전체 누적 궤적선 저장용
total_time_steps = 0;              % 에러 차트 가로축 프레임 카운터

% 집게 로컬 고유 기하학 데이터 수신 (오류 해결용 핵심 좌표 데이터 고정)
[wrist_x, wrist_y, claw_base_x, claw_base_y, pad_base_x, pad_base_y] = define_gripper_geometry();

for p = 1:n_pts
    % 현재 타깃 포인트 정보 추출
    X_des  = points(p,1:2)';
    theta_guess = points(p,3:4)';
    pt_color = colors8(p,:);
    
    % 현재 타깃 하이라이트 링 업데이트
    setup_point_ui(ax1, p, X_des, pt_color, visual_objs.h_ring);
    
    %% >> 수치해석 최적화 루프 초기 세팅
    theta = theta_guess; 
    conv_history = []; 
    opt_active = true;
    current_iter = 1;
    
    %% >> 뉴턴-랩슨 최적화 및 동적 그래픽스 동시 구동 루프
    while opt_active && (current_iter <= max_iter)
        
        [x, y]  = fk(theta(1), theta(2), a1, a2);
        X_curr = [x; y];
        dx     = X_des - X_curr;
        err    = norm(dx);
        
        % HUD 데이터용 기록
        conv_history = [conv_history, err];
        
        %% >> 실시간 그래픽스 동적 변환 프레임 처리
        % 1. 우측 화면: 실시간 에러 트레이스 선그래프 업데이트
        total_time_steps = total_time_steps + 1;
        update_error_graph(ax2, visual_objs.h_err_trace, total_time_steps, err, pt_color);
        
        % 2. 좌측 화면: 아노다이징 골드 암 및 회전 변환 집게 패치 업데이트
        full_ee_history = [full_ee_history, X_curr];
        update_robot_graphics(visual_objs, full_ee_history, theta, a1, a2, ...
                              wrist_x, wrist_y, claw_base_x, claw_base_y, pad_base_x, pad_base_y);
        
        % 3. HUD 대시보드 리프레시
        update_hud(hud_box, p, X_des, theta, err, current_iter, max_iter, conv_history);
        
        drawnow;
        pause(0.01); % 애니메이션 프레임 속도 동기화 딜레이
        
        %% >> 수렴 유무 판정 및 자코비안 역행렬 계산
        if err <= tolerance
            opt_active = false;
        else
            detJ = sin(theta(2));
            if abs(detJ) < 1e-6
                warning('Singularity encountered at P%d iter=%d', p, current_iter);
                opt_active = false;
            else
                % 뉴턴-랩슨 공식 기반 다음 관절각 갱신 연산
                t1 = theta(1); t2 = theta(2);
                Jinv = (1/detJ) * [ cos(t1+t2),              sin(t1+t2);
                                   -cos(t1)-cos(t1+t2), -sin(t1)-sin(t1+t2) ];
                theta = theta + Jinv * dx;
                current_iter = current_iter + 1;
            end
        end
    end
    
    %% >> 콘솔창 정밀 수치 텍스트 출력
    [xf, yf] = fk(theta(1), theta(2), a1, a2);
    final_err = norm(X_des - [xf; yf]);
    is_conv = final_err <= tolerance;
    
    fprintf('  P%-4d (%-5.1f,%-5.1f)  theta1=%7.4f  theta2=%7.4f  err=%.5f  %3d iter  %s\n',...
            p, points(p,1), points(p,2), theta(1), theta(2), final_err, current_iter-1,...
            choose(is_conv,'CONV','DIVG'));
        
    % 타깃 안착 성공 시 링 플래시 효과 연출
    flash_target_visuals(visual_objs.h_ring, pt_color);
end

% 최종 완료 타이틀 텍스트 표기
set(visual_objs.h_final_title, 'String', '✓ ALL 8 TARGETS REACHED!', 'Color', 'cyan');

%% ====================================================================
%%  핵심 계산 및 시각화 지원 함수 블록 (Support Functions)
%% ====================================================================

function [x, y] = fk(t1, t2, a1, a2) 
    % 기하학 기반 고속 순기구학 계산 공식
    x = a1 * cos(t1) + a2 * cos(t1 + t2);
    y = a1 * sin(t1) + a2 * sin(t1 + t2);
end

function s = choose(cond, a, b)
    if cond; s=a; else; s=b; end
end

function [wrist_x, wrist_y, claw_base_x, claw_base_y, pad_base_x, pad_base_y] = define_gripper_geometry()
    % 다각형 집게(Gripper) 패치 처리를 위한 고유 로컬 좌표 정의 데이터
    wrist_x = [-0.03,  0.03,  0.03, -0.03]; 
    wrist_y = [-0.05, -0.05,  0.05,  0.05];
    
    claw_base_x = [0.03, 0.11, 0.19, 0.23, 0.20, 0.12, 0.03]; 
    claw_base_y = [0.05, 0.09, 0.07, 0.02, -0.01, 0.03, 0.01];
    
    pad_base_x = [0.15, 0.23, 0.20]; 
    pad_base_y = [0.07, 0.02, -0.01];
end

function [ax1, ax2, visual_objs] = setup_visualization_environment(fig, a1, a2, points, colors8, tolerance, max_iter)
    %% ── SCREEN 01: 좌측 타스크 공간 물리 시뮬레이터 ──
    ax1 = subplot(1, 2, 1, 'Parent', fig, ...
                  'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
                  'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
    hold(ax1, 'on'); axis(ax1, 'equal'); grid(ax1, 'on');
    xlim(ax1, [-2.5, 2.5]); ylim(ax1, [-2.5, 2.5]);
    title(ax1, 'SCREEN 01: Full Mechanical Workspace', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');
    
    % 로봇 작업 가동 영역 경계선
    tw = linspace(0, 2*pi, 300);
    plot(ax1, (a1+a2)*cos(tw), (a1+a2)*sin(tw), '--', 'Color', [0.35, 0.35, 0.5], 'LineWidth', 1);
    
    % 고정식 인더스트리얼 베이스 플레이트 (첫 번째 코드 이식)
    base_plate_x = [-0.45, 0.45, 0.30, -0.30]; base_plate_y = [-0.60, -0.60, -0.15, -0.15];
    base_neck_x = [-0.15, 0.15, 0.10, -0.10];  base_neck_y = [-0.15, -0.15, 0.0, 0.0];
    fill(ax1, base_plate_x, base_plate_y, [0.18, 0.20, 0.23], 'EdgeColor', [0.1, 0.11, 0.12], 'LineWidth', 2);
    fill(ax1, base_neck_x, base_neck_y, [0.55, 0.58, 0.62], 'EdgeColor', [0.3, 0.32, 0.35], 'LineWidth', 1.5);
    rectangle(ax1, 'Position', [-0.18, -0.18, 0.36, 0.36], 'Curvature', [1, 1], 'FaceColor', [0.35, 0.38, 0.42], 'LineWidth', 1.5);
    
    % 실시간 매니퓰레이터 기하 합성 패스선 (네온 마젠타)
    visual_objs.h_traj = plot(ax1, NaN, NaN, '-', 'Color', [1.0, 0.2, 0.6], 'LineWidth', 2.0);

    % 8개 타깃 마커 정적 렌더링
    n_pts = size(points,1);
    for p = 1:n_pts
        plot(ax1, points(p,1), points(p,2), 'x', 'MarkerSize', 12, 'LineWidth', 2, 'Color', colors8(p,:));
        text(ax1, points(p,1)+0.09, points(p,2)+0.09, sprintf('P%d', p), ...
             'Color', colors8(p,:), 'FontSize', 10, 'FontWeight', 'bold');
    end
    
    % 실시간 표적 활성화 추적 링
    visual_objs.h_ring = plot(ax1, NaN, NaN, 'o', 'MarkerSize', 25, 'LineWidth', 2.5, 'Color', 'w');

    % 입체 메탈 질감 표현용 아노다이징 다중 플롯 레이어 트리거
    gold_colors = {[0.25, 0.18, 0.05], [0.70, 0.52, 0.15], [0.88, 0.71, 0.28], [0.98, 0.88, 0.55], [1.0, 0.98, 0.85]};
    widths = [26, 21, 14, 7, 2.5];
    visual_objs.link1_layers = []; visual_objs.link2_layers = [];
    for k = 1:5
        visual_objs.link1_layers(k) = plot(ax1, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
        visual_objs.link2_layers(k) = plot(ax1, [0, 0], [0, 0], 'Color', gold_colors{k}, 'LineWidth', widths(k), 'HandleVisibility', 'off');
    end

    % 집게 패치 구조물 동적 업데이트 객체 배정 (YData 차트 꼬임 원천 차단형 빈 배열 바인딩)
    visual_objs.h_wrist  = patch(ax1, 'XData', [], 'YData', [], 'FaceColor', [0.30, 0.25, 0.15], 'EdgeColor', [0.1, 0.1, 0.1]);
    visual_objs.h_claw_L = patch(ax1, 'XData', [], 'YData', [], 'FaceColor', [0.85, 0.75, 0.45], 'EdgeColor', [0.45, 0.38, 0.2]);
    visual_objs.h_claw_R = patch(ax1, 'XData', [], 'YData', [], 'FaceColor', [0.85, 0.75, 0.45], 'EdgeColor', [0.45, 0.38, 0.2]);
    visual_objs.h_pad_L  = patch(ax1, 'XData', [], 'YData', [], 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none'); 
    visual_objs.h_pad_R  = patch(ax1, 'XData', [], 'YData', [], 'FaceColor', [0.0, 0.9, 1.0], 'EdgeColor', 'none');

    % 핀 조인트 아노다이징 캡 구성
    plot(ax1, 0, 0, 'o', 'MarkerSize', 15, 'MarkerFaceColor', [0.2, 0.2, 0.2], 'MarkerEdgeColor', 'k'); 
    visual_objs.h_j2_cap = plot(ax1, 0, 0, 'o', 'MarkerSize', 11, 'MarkerFaceColor', [0.9, 0.75, 0.20], 'MarkerEdgeColor', [0.1, 0.1, 0.1]);
    visual_objs.h_wrist_cap = plot(ax1, 0, 0, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.0, 0.9, 1.0], 'MarkerEdgeColor', [0.1, 0.1, 0.1]);
    
    visual_objs.h_final_title = text(ax1, -2.45, 2.4, '', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold', 'VerticalAlignment', 'top');

    %% ── SCREEN 02: 우측 수치 최적화 모니터 실시간 차트 ──
    ax2 = subplot(1, 2, 2, 'Parent', fig, ...
                  'Color', [0.12, 0.14, 0.18], 'XColor', [0.4, 0.45, 0.55], 'YColor', [0.4, 0.45, 0.55], ...
                  'GridColor', [0.25, 0.28, 0.35], 'GridAlpha', 0.4);
    hold(ax2, 'on'); grid(ax2, 'on');
    ylabel(ax2, 'Hand Error ||e|| (m)', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
    xlabel(ax2, 'Total Animation Steps', 'Color', [0.6, 0.65, 0.75], 'FontName', 'Segoe UI');
    title(ax2, 'SCREEN 02: Real-time IK Optimization Trace', 'Color', [0.9, 0.95, 1.0], 'FontName', 'Segoe UI', 'FontWeight', 'bold');
    
    set(ax2, 'YScale', 'log'); % 뉴턴-랩슨의 급격한 에러 저하를 보기 위해 로그 스케일 필수 세팅
    
    % 전역 누적 에러 궤적선 핸들 객체 선언
    visual_objs.h_err_trace = plot(ax2, NaN, NaN, '-', 'Color', 'w', 'LineWidth', 1.5);
    
    % 허용 기준 오차 가이드 가로 점선 레이아웃 추가
    yline(ax2, tolerance, '--', 'Color', [1, 0.4, 0.4], 'LineWidth', 2);
    text(ax2, 10, tolerance * 1.1, ' tol = 0.01', 'Color', [1, 0.4, 0.4], 'FontSize', 9, 'VerticalAlignment', 'bottom');
    
    xlim(ax2, [0, 400]);
    ylim(ax2, [0.001, 2]);
end

function update_error_graph(ax2, h_err_trace, total_steps, err, pt_color)
    % 우측 실시간 수렴 모니터 축에 데이터를 동적으로 이어 붙이는 함수
    current_x = get(h_err_trace, 'XData');
    current_y = get(h_err_trace, 'YData');
    
    new_x = [current_x, total_steps];
    new_y = [current_y, err];
    
    % 로그 차트가 깨지는 것을 마스킹하기 위해 하한 에러치 0.001로 고정 클램핑 연산
    new_y(new_y < 0.001) = 0.001; 
    
    set(h_err_trace, 'XData', new_x, 'YData', new_y, 'Color', pt_color);
    
    % 가로축 가동 범위 자동 우측 스크롤 연산 적용
    if total_steps > 350
        set(ax2, 'Xlim', [total_steps - 300, total_steps + 50]);
    end
end

function update_robot_graphics(visual_objs, ee_history, theta, a1, a2, wrist_x, wrist_y, claw_base_x, claw_base_y, pad_base_x, pad_base_y)
    % 기하학 데이터 및 동적 각도를 받아 골드 아노다이징 링크 및 다각형 패치 집게를 투사하는 공간 제어 연산 함수
    t1 = theta(1); t2 = theta(2);
    x1 = a1 * cos(t1);      y1 = a1 * sin(t1);
    x2 = x1 + a2 * cos(t1+t2); y2 = y1 + a2 * sin(t1+t2);
    phi = t1 + t2; % 글로벌 최종 집게 방향 지향각
    
    % 1. 5중 메탈 질감 링크 업그레이드 데이터 리프레시
    for k = 1:5
        set(visual_objs.link1_layers(k), 'XData', [0, x1], 'YData', [0, y1]);
        set(visual_objs.link2_layers(k), 'XData', [x1, x2], 'YData', [y1, y2]);
    end
    
    % 2. 관절 동적 조인트 캡 위치 동기화
    set(visual_objs.h_j2_cap, 'XData', x1, 'YData', y1);
    set(visual_objs.h_wrist_cap, 'XData', x2, 'YData', y2);
    
    % 3. 에어로 마젠타 말단 궤적 흔적선 갱신
    set(visual_objs.h_traj, 'XData', ee_history(1,:), 'YData', ee_history(2,:));
    
    % 4. [오류 완벽 수정] 수치 인자로 들어온 고유 기하 로컬 좌표 정보와 2x2 회전 및 평행이동 변환 연산 수행
    set_part_transformed(visual_objs.h_wrist,  wrist_x, wrist_y, x2, y2, phi);
    set_part_transformed(visual_objs.h_claw_L, claw_base_x, claw_base_y, x2, y2, phi);
    set_part_transformed(visual_objs.h_claw_R, claw_base_x, -claw_base_y, x2, y2, phi);
    set_part_transformed(visual_objs.h_pad_L,  pad_base_x, pad_base_y, x2, y2, phi);
    set_part_transformed(visual_objs.h_pad_R,  pad_base_x, -pad_base_y, x2, y2, phi);
end

function set_part_transformed(h_patch, local_x, local_y, ee_x, ee_y, ee_phi)
    % 2x2 오일러 삼각 회전행렬 공간 수학 공식 그대로 적용하여 세로 결합(vertcat) 연산 진행
    global_coords = [cos(ee_phi), -sin(ee_phi); sin(ee_phi), cos(ee_phi)] * [local_x; local_y] + [ee_x; ee_y];
    set(h_patch, 'XData', global_coords(1,:), 'YData', global_coords(2,:));
end

function update_hud(hud_box, p, X_des, theta, err, current_iter, max_iter, conv_history)
    % 실시간 HUD 디지털 전광판 판넬 정보창 스트링 마운트
    if length(conv_history) > 2
        diff_err = conv_history(end) - conv_history(end-1);
        if err <= 0.01; prog_str = 'TARGET REACHED'; 
        elseif diff_err < -1e-4; prog_str = 'CONVERGING...'; 
        elseif diff_err > 1e-4; prog_str = 'DIVERGING!'; 
        else; prog_str = 'STALLED'; 
        end
    else
        prog_str = 'INITIALIZING...'; 
    end
    
    hud_string = sprintf(['   [IK STATUS: POINT %d/8]\n   ---------------------\n   Goal: (%.1f, %.1f)\n\n   [SOLVER INFO]\n   Iter : %d / %d\n   Error: %.5f\n   \\theta_1 : %6.4f rad\n   \\theta_2 : %6.4f rad\n\n   PROGRESS: %s'], ...
                           p, X_des(1), X_des(2), current_iter, max_iter, err, theta(1), theta(2), prog_str);
    
    set(hud_box, 'String', hud_string);
    set(hud_box, 'Color', choose(err<=0.01, 'cyan', [0.98, 0.88, 0.55]));
end

function flash_target_visuals(h_ring, pt_color)
    % 표적 매칭에 성공한 순간 링 크기가 팝업 애니메이션을 그리며 페이드되는 시각 연출 효과
    for fl = 1:6
        s_base = 25;
        set(h_ring, 'MarkerSize', s_base + 10 * mod(fl,2), 'LineWidth', 2.5 + 1.5 * mod(fl,2));
        drawnow;
        pause(0.05);
    end
    set(h_ring, 'MarkerSize', s_base, 'LineWidth', 2.5);
end