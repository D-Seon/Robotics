%% Computer Problem Set 5: Inverse Kinematics — Problems II
% 2-Link Planar Manipulator (a1=a2=1)
% 8 desired hand positions solved via Newton-Raphson (Jacobian IK)

clear; clc; close all;

%% ── Robot & solver parameters ────────────────────────────────────────
a1 = 1.0;  a2 = 1.0;
tolerance = 0.01;
max_iter  = 300;

%% ── 8 Points: [x_des, y_des, theta1_0, theta2_0] ────────────────────
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

%% ── 1. Solve IK for all 8 points ────────────────────────────────────
fprintf('========== IK Solutions (Newton-Raphson) ==========\n');
fprintf('%-6s %-14s %-14s %-14s %-14s %-10s %-8s\n',...
        'Point','x_des','y_des','theta1','theta2','error','iters');
fprintf('%s\n', repmat('-',1,75));

results = zeros(n_pts, 5);  % [theta1, theta2, error, iters, converged]
all_theta = cell(n_pts,1);
all_error = cell(n_pts,1);

for p = 1:n_pts
    X_des  = points(p,1:2)';
    theta0 = points(p,3:4)';
    
    % 수식 주석이 포함된 역기구학 연산 함수 호출
    [th_hist, err_hist] = run_IK(theta0, X_des, a1, a2, tolerance, max_iter);
    
    all_theta{p} = th_hist;
    all_error{p} = err_hist;
    
    tf = th_hist(:,end);
    ef = err_hist(end);
    conv = ef <= tolerance;
    
    results(p,:) = [tf(1), tf(2), ef, length(err_hist)-1, conv];
    
    fprintf('  P%-4d (%-5.1f,%-5.1f)  theta1=%7.4f  theta2=%7.4f  err=%.5f  %3d iter  %s\n',...
            p, points(p,1), points(p,2), tf(1), tf(2), ef, length(err_hist)-1,...
            choose(conv,'CONV','DIVG'));
end

%% ── 2. 에러 수렴 시각화 그래프 (Error Convergence Plot) ──────────────────
figure('Name','IK Error Convergence Summary','Color','w','Position',[200 200 800 500]);
hold on;

% 8개 포인트 색상 정의
colors8 = lines(n_pts);

% 각 포인트별 에러 역사를 선그래프로 그리기
for p = 1:n_pts
    n_e = length(all_error{p});
    % 세로축을 로그 스케일(semilogy)로 지정하여 에러가 0으로 수렴하는 모습을 직관적으로 인지
    semilogy(0:n_e-1, all_error{p}, '-o', 'Color', colors8(p,:), 'LineWidth', 2, ...
             'MarkerSize', 5, 'MarkerFaceColor', colors8(p,:), 'DisplayName', sprintf('Point %d', p));
end

% 허용 오차 한계선(Tolerance Line) 표시
yline(tolerance, 'k--', 'LineWidth', 2, 'Color', [0.8 0 0], ...
      'Label', ' 허용 오차 한계선 (Tolerance = 0.01)', 'LabelVerticalAlignment', 'bottom', 'FontSize', 11);

% 그래프 스타일 설정
xlabel('반복 횟수 (Iteration)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('위치 오차 노름 (||Error||)', 'FontSize', 12, 'FontWeight', 'bold');
title('뉴턴-랩슨 역기구학: 반복당 말단 장치 오차 수렴 그래프', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 10);
grid on; box on;
set(gca, 'FontSize', 11);

%% ====================================================================
%%  수학적 핵심 함수 (Math-Annotated Functions)
%% ====================================================================

function [theta_hist, error_hist] = run_IK(theta0, X_des, a1, a2, tol, max_iter)
    theta      = theta0;
    theta_hist = theta;
    error_hist = [];
    
    for i = 1:max_iter
        % -----------------------------------------------------------------
        % ① 순기구학(FK) 및 오차(Error) 계산
        %    - 현재 관절각(theta)을 기하학적 공식에 대입해 현재 위치 [x; y] 연산
        %    - 오차 벡터(dX) 및 오차의 크기(유클리드 노름, err) 산출
        % -----------------------------------------------------------------
        [x, y] = fk(theta(1), theta(2), a1, a2);
        dx     = X_des - [x; y];               % dX = X_desired - X_current
        err    = norm(dx);                     % err = sqrt(dx^2 + dy^2)
        error_hist(end+1) = err;               % 에러 그래프를 위한 데이터 누적
        
        % 목표 허용오차(Tolerance) 만족 시 루프 탈출 (수렴 성공)
        if err <= tol; break; end
        
        % -----------------------------------------------------------------
        % ② 자코비안 행렬식(Determinant) 계산 및 특이점(Singularity) 검사
        %    - det(J) = a1 * a2 * sin(θ2) 
        %    - 만약 이 값이 0에 가까우면 역행렬이 존재하지 않음 (로봇이 일자로 펴진 상태 등)
        % -----------------------------------------------------------------
        detJ = sin(theta(2));
        if abs(detJ) < 1e-6
            warning('특이점 상태 감지 (Singularity) - Iteration: %d', i); 
            break;
        end
        
        % -----------------------------------------------------------------
        % ③ 분석적 자코비안 역행렬(Inverse Jacobian) 계산
        %               1      [     cos(θ1+θ2)             sin(θ1+θ2)      ]
        %    J_inv = ─────── * [                                            ]
        %            sin(θ2)   [ -cos(θ1)-cos(θ1+θ2)   -sin(θ1)-sin(θ1+θ2)  ]
        % -----------------------------------------------------------------
        t1 = theta(1); 
        t2 = theta(2);
        Jinv = (1/detJ) * [ cos(t1+t2),              sin(t1+t2);
                           -cos(t1)-cos(t1+t2), -sin(t1)-sin(t1+t2) ];
        
        % -----------------------------------------------------------------
        % ④ 뉴턴-랩슨 관절각 업데이트 (역기구학 핵심 수치 연산)
        %    - 오차를 줄이기 위한 미소 관절각 변화량 연산: dθ = J_inv * dX
        %    - 새로운 관절각 갱신: θ_new = θ_old + dθ
        % -----------------------------------------------------------------
        theta = theta + Jinv * dx;
        theta_hist = [theta_hist, theta];
    end
    
    % 최종 결과 반영 확인용 계산
    [x, y] = fk(theta(1), theta(2), a1, a2);
    error_hist(end+1) = norm(X_des - [x; y]);
    theta_hist = [theta_hist, theta];
end

function [x, y] = fk(t1, t2, a1, a2)
    % -----------------------------------------------------------------
    % [순기구학 공식 (Forward Kinematics)]
    %  관절 공간(Joint Space) -> 작업 공간(Task Space) 변환 기하학 식
    %  x = a1*cos(θ1) + a2*cos(θ1 + θ2)
    %  y = a1*sin(θ1) + a2*sin(θ1 + θ2)
    % -----------------------------------------------------------------
    x = a1*cos(t1) + a2*cos(t1+t2);
    y = a1*sin(t1) + a2*sin(t1+t2);
end

function s = choose(cond, a, b)
    if cond; s=a; else; s=b; end
end