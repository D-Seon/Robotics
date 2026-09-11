%% ============================================================
%  2-Link Planar Manipulator — Inverse Kinematics Simulation
%  MATLAB R2026a 최적화 버전
%
%  타스크 궤적 : 타원  x^2/2.0^2 + y^2/1.3^2 = 1
%  초기값     : x = 2.0, y = 0
%  링크 길이   : L1 = L2 = 1 m
%  역기구학   : Elbow-Up 해 (θ2 > 0)
% ============================================================

clear; clc; close all;

%% ── 1. 파라미터 설정 ─────────────────────────────────────────
L1 = 1.0;          % 링크 1 길이 [m]
L2 = 1.0;          % 링크 2 길이 [m]
a  = 2.0;          % 타원 장반축 (x 방향)
b  = 1.3;          % 타원 단반축 (y 방향)
N  = 360;          % 궤적 분할 수

%% ── 2. 타원 궤적 생성 (t=0 → x=2.0, y=0) ───────────────────
t  = linspace(0, 2*pi, N+1);
Xd = a .* cos(t);
Yd = b .* sin(t);

%% ── 3. 역기구학 계산 ─────────────────────────────────────────
%  코사인 법칙:
%    D        = (x²+y² - L1²-L2²) / (2·L1·L2)
%    θ2       = atan2(+√(1−D²), D)   ← Elbow-Up
%    θ1       = atan2(y,x) − atan2(L2·sinθ2, L1+L2·cosθ2)

theta1 = zeros(1, N+1);
theta2 = zeros(1, N+1);

for i = 1:N+1
    x = Xd(i);
    y = Yd(i);
    D = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2);

    if abs(D) > 1                        % 도달 불가 예외 처리
        warning('Step %d: 도달 불가 (D=%.4f). 이전 각도 유지.', i, D);
        if i > 1
            theta1(i) = theta1(i-1);
            theta2(i) = theta2(i-1);
        end
        continue
    end

    th2 = atan2(sqrt(1 - D^2), D);      % Elbow-Up (+)
    th1 = atan2(y, x) - atan2(L2*sin(th2), L1 + L2*cos(th2));
    theta1(i) = th1;
    theta2(i) = th2;
end

%% ── 4. 순기구학 검증 ─────────────────────────────────────────
X_fk = L1.*cos(theta1) + L2.*cos(theta1 + theta2);
Y_fk = L1.*sin(theta1) + L2.*sin(theta1 + theta2);
maxErr = max(sqrt((X_fk - Xd).^2 + (Y_fk - Yd).^2));

%% ── 5. 애니메이션 창 (R2026a: tiledlayout + dot-notation) ───
fig = figure(Name="2-Link Manipulator (R2026a)", ...
             Color="w", Position=[80 60 920 760]);

% ── tiledlayout으로 메인 애니메이션 축 생성 (R2026a 권장) ──
tl  = tiledlayout(fig, 1, 1, TileSpacing="compact", Padding="compact");
ax  = nexttile(tl);

hold(ax, "on");
grid(ax, "on");
axis(ax, "equal");
xlim(ax, [-2.8, 2.8]);
ylim(ax, [-2.8, 2.8]);
xlabel(ax, "X [m]", FontSize=12);
ylabel(ax, "Y [m]", FontSize=12);
title(ax, "2-Link Manipulator — 타원 궤적 역기구학 애니메이션", FontSize=13);

% 목표 타원 (점선)
plot(ax, Xd, Yd, "--", Color=[0.6 0.6 0.6], LineWidth=1.4, ...
     DisplayName="목표 궤적 (타원)");

% 원점 (베이스)
plot(ax, 0, 0, "ks", MarkerSize=12, MarkerFaceColor=[0.25 0.25 0.25], ...
     HandleVisibility="off");

% 링크 및 EE 그래픽 오브젝트 (R2026a: Name=Value 인수 스타일)
hLink1 = plot(ax, [0 0], [0 0], "b-o", LineWidth=3.5, MarkerSize=8, ...
              MarkerFaceColor="b", DisplayName="링크 1");
hLink2 = plot(ax, [0 0], [0 0], "r-o", LineWidth=3.5, MarkerSize=8, ...
              MarkerFaceColor="r", DisplayName="링크 2");
hEE    = plot(ax, 0, 0, "ko",   MarkerSize=10, MarkerFaceColor="k", ...
              DisplayName="End-Effector");
hTrace = plot(ax, NaN, NaN, "k-", LineWidth=1.6, DisplayName="EE 궤적");

legend(ax, Location="northeast", FontSize=10);

% 정보 텍스트 박스 (R2026a: 인수 스타일)
hTxt = text(ax, -2.65, 2.55, "", ...
            FontSize=10, FontName="Courier New", ...
            BackgroundColor=[0.96 0.96 0.96], EdgeColor="k", Margin=4);

% 궤적 버퍼
traceX = nan(1, N+1);
traceY = nan(1, N+1);

%% ── 6. 애니메이션 루프 ───────────────────────────────────────
for i = 1:N+1
    J1x = L1 * cos(theta1(i));
    J1y = L1 * sin(theta1(i));
    J2x = J1x + L2 * cos(theta1(i) + theta2(i));
    J2y = J1y + L2 * sin(theta1(i) + theta2(i));

    % R2026a: dot-notation이 set()보다 빠름
    hLink1.XData = [0,  J1x];
    hLink1.YData = [0,  J1y];
    hLink2.XData = [J1x, J2x];
    hLink2.YData = [J1y, J2y];
    hEE.XData    = J2x;
    hEE.YData    = J2y;

    traceX(i) = J2x;
    traceY(i) = J2y;
    hTrace.XData = traceX;
    hTrace.YData = traceY;

    hTxt.String = sprintf( ...
        " \x03B81 = %+7.2f\xB0\n \x03B82 = %+7.2f\xB0\n X  = %6.3f m\n Y  = %6.3f m", ...
        rad2deg(theta1(i)), rad2deg(theta2(i)), J2x, J2y);

    drawnow limitrate;
    pause(0.008);        % 속도 조절: 값 줄이면 빠름
end

%% ── 7. 결과 정적 플롯 (R2026a: tiledlayout 2×1) ─────────────
fig2 = figure(Name="관절각 & FK 검증", Color="w", Position=[80 60 900 680]);
tl2  = tiledlayout(fig2, 2, 1, TileSpacing="compact", Padding="compact");

% ── 서브플롯 1: 관절각 시계열 ──
ax1 = nexttile(tl2);
steps = 0:N;
plot(ax1, steps, rad2deg(theta1), "b-", LineWidth=1.6, DisplayName="\theta_1");
hold(ax1, "on");
plot(ax1, steps, rad2deg(theta2), "r-", LineWidth=1.6, DisplayName="\theta_2");
xlabel(ax1, "Step",       FontSize=11);
ylabel(ax1, "각도 [deg]", FontSize=11);
title(ax1,  "역기구학 관절각: \theta_1, \theta_2", FontSize=12);
legend(ax1, Location="best", FontSize=10);
grid(ax1, "on");

% ── 서브플롯 2: FK 검증 ──
ax2 = nexttile(tl2);
plot(ax2, Xd,   Yd,   "k--", LineWidth=2.0, DisplayName="목표 궤적 (타원)");
hold(ax2, "on");
plot(ax2, X_fk, Y_fk, "r-",  LineWidth=1.2, DisplayName="FK 검증 궤적");
xlabel(ax2, "X [m]", FontSize=11);
ylabel(ax2, "Y [m]", FontSize=11);
title(ax2,  "순기구학 검증 (목표 ↔ FK)", FontSize=12);
legend(ax2, Location="best", FontSize=10);
grid(ax2, "on");
axis(ax2, "equal");

%% ── 8. 콘솔 출력 ────────────────────────────────────────────
fprintf("\n=== 시뮬레이션 완료 (MATLAB R2026a) ===\n");
fprintf("  최대 FK 오차 : %.2e m\n", maxErr);
fprintf("  링크 길이    : L1 = %.1f m, L2 = %.1f m\n", L1, L2);
fprintf("  타원 반축    : a  = %.1f m, b  = %.1f m\n", a, b);
fprintf("=========================================\n");
