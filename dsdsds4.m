%% ============================================================
%  2-Link Manipulator — 순기구학 (FK) 로봇 핸드 궤적 시각화
%  MATLAB R2026a 최적화 버전
%
%  타스크 궤적 : 타원  x²/2.0² + y²/1.3² = 1
%  초기값     : x = 2.0, y = 0
%  링크 길이   : L1 = L2 = 1 m
%
%  순기구학:
%    X = L1·cos(θ₁) + L2·cos(θ₁+θ₂)
%    Y = L1·sin(θ₁) + L2·sin(θ₁+θ₂)
%
%  θ₁, θ₂ 동시 구동 → 로봇 핸드(X,Y) 타원 궤적 추적
% ============================================================

clear; clc; close all;

%% ── 1. 파라미터 ─────────────────────────────────────────────
L1 = 1.0;
L2 = 1.0;
a  = 2.0;
b  = 1.3;
N  = 360;

%% ── 2. 타원 궤적 생성 ───────────────────────────────────────
t  = linspace(0, 2*pi, N+1);
Xd = a .* cos(t);
Yd = b .* sin(t);

%% ── 3. 역기구학 — Elbow-Up / Elbow-Down 두 해 ───────────────
theta1_up = zeros(1, N+1);
theta2_up = zeros(1, N+1);
theta1_dn = zeros(1, N+1);
theta2_dn = zeros(1, N+1);

for i = 1:N+1
    x = Xd(i);
    y = Yd(i);
    D = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2);

    if abs(D) > 1
        if i > 1
            theta1_up(i) = theta1_up(i-1);  theta2_up(i) = theta2_up(i-1);
            theta1_dn(i) = theta1_dn(i-1);  theta2_dn(i) = theta2_dn(i-1);
        end
        continue
    end

    th2_up = atan2(+sqrt(1 - D^2), D);
    th1_up = atan2(y, x) - atan2(L2*sin(th2_up), L1 + L2*cos(th2_up));

    th2_dn = atan2(-sqrt(1 - D^2), D);
    th1_dn = atan2(y, x) - atan2(L2*sin(th2_dn), L1 + L2*cos(th2_dn));

    theta1_up(i) = th1_up;  theta2_up(i) = th2_up;
    theta1_dn(i) = th1_dn;  theta2_dn(i) = th2_dn;
end

%% ── 4. 순기구학 계산 (θ₁+θ₂ 동시 적용) ─────────────────────
% 관절2 위치 (링크1 끝)
J1x_up = L1 .* cos(theta1_up);
J1y_up = L1 .* sin(theta1_up);
J1x_dn = L1 .* cos(theta1_dn);
J1y_dn = L1 .* sin(theta1_dn);

% 로봇 핸드 위치 (링크2 끝 = EE)
EEx_up = J1x_up + L2 .* cos(theta1_up + theta2_up);
EEy_up = J1y_up + L2 .* sin(theta1_up + theta2_up);
EEx_dn = J1x_dn + L2 .* cos(theta1_dn + theta2_dn);
EEy_dn = J1y_dn + L2 .* sin(theta1_dn + theta2_dn);

% FK 오차 확인
err_up = max(hypot(EEx_up - Xd, EEy_up - Yd));
err_dn = max(hypot(EEx_dn - Xd, EEy_dn - Yd));
fprintf("FK 오차 (Elbow-Up)  : %.2e m\n", err_up);
fprintf("FK 오차 (Elbow-Down): %.2e m\n", err_dn);

%% ══════════════════════════════════════════════════════════════
%  Figure 1 — θ₁+θ₂ 동시 구동 애니메이션 (좌: Up / 우: Down)
%% ══════════════════════════════════════════════════════════════
fig1 = figure(Name="FK Full Manipulator Animation (R2026a)", ...
              Color="w", Position=[40 50 1340 720]);

tl1 = tiledlayout(fig1, 1, 2, TileSpacing="compact", Padding="compact");
title(tl1, "2-Link Manipulator — θ₁+θ₂ 동시 구동 → 순기구학 핸드(X,Y) 궤적", ...
      FontSize=14, FontWeight="bold");

% ────────────────────────────────────────────────────────────
% 왼쪽: Elbow-Up
% ────────────────────────────────────────────────────────────
axU = nexttile(tl1, 1);
hold(axU, "on"); grid(axU, "on"); axis(axU, "equal");
xlim(axU, [-2.8, 2.8]); ylim(axU, [-2.8, 2.8]);
xlabel(axU, "X [m]", FontSize=11);
ylabel(axU, "Y [m]", FontSize=11);
title(axU, "Elbow-Up  (θ₂ > 0)", FontSize=12, ...
      Color=[0 0.45 0.74], FontWeight="bold");

% 목표 타원
plot(axU, Xd, Yd, "--", Color=[0.75 0.75 0.75], LineWidth=1.3, ...
     DisplayName="목표 궤적 (타원)");
% 베이스
plot(axU, 0, 0, "ks", MarkerSize=11, MarkerFaceColor=[0.2 0.2 0.2], ...
     HandleVisibility="off");

% 링크1 (파랑)
hL1u = plot(axU, [0 0], [0 0], "-o", LineWidth=4, MarkerSize=8, ...
            Color=[0 0.45 0.74], MarkerFaceColor=[0 0.45 0.74], ...
            DisplayName="링크1 (θ₁)");
% 링크2 (하늘색)
hL2u = plot(axU, [0 0], [0 0], "-o", LineWidth=4, MarkerSize=8, ...
            Color=[0 0.75 0.90], MarkerFaceColor=[0 0.75 0.90], ...
            DisplayName="링크2 (θ₂)");
% 로봇 핸드 EE (검정 원)
hEEu = plot(axU, 0, 0, "o", MarkerSize=11, MarkerFaceColor="k", ...
            MarkerEdgeColor="k", DisplayName="로봇 핸드 (EE)");
% EE 궤적 누적 (진한 파랑)
hTru = plot(axU, NaN, NaN, "-", LineWidth=2.0, ...
            Color=[0 0.2 0.6], DisplayName="핸드 궤적 (FK)");

legend(axU, Location="northeast", FontSize=9);
hTxtU = text(axU, -2.65, 2.55, "", FontSize=9, FontName="Courier New", ...
             BackgroundColor=[0.92 0.96 1.0], EdgeColor=[0 0.45 0.74], Margin=4);

% ────────────────────────────────────────────────────────────
% 오른쪽: Elbow-Down
% ────────────────────────────────────────────────────────────
axD = nexttile(tl1, 2);
hold(axD, "on"); grid(axD, "on"); axis(axD, "equal");
xlim(axD, [-2.8, 2.8]); ylim(axD, [-2.8, 2.8]);
xlabel(axD, "X [m]", FontSize=11);
ylabel(axD, "Y [m]", FontSize=11);
title(axD, "Elbow-Down (θ₂ < 0)", FontSize=12, ...
      Color=[0.77 0.18 0.1], FontWeight="bold");

plot(axD, Xd, Yd, "--", Color=[0.75 0.75 0.75], LineWidth=1.3, ...
     DisplayName="목표 궤적 (타원)");
plot(axD, 0, 0, "ks", MarkerSize=11, MarkerFaceColor=[0.2 0.2 0.2], ...
     HandleVisibility="off");

% 링크1 (빨강)
hL1d = plot(axD, [0 0], [0 0], "-o", LineWidth=4, MarkerSize=8, ...
            Color=[0.77 0.18 0.1], MarkerFaceColor=[0.77 0.18 0.1], ...
            DisplayName="링크1 (θ₁)");
% 링크2 (주황)
hL2d = plot(axD, [0 0], [0 0], "-o", LineWidth=4, MarkerSize=8, ...
            Color=[0.95 0.55 0.1], MarkerFaceColor=[0.95 0.55 0.1], ...
            DisplayName="링크2 (θ₂)");
% 로봇 핸드 EE
hEEd = plot(axD, 0, 0, "o", MarkerSize=11, MarkerFaceColor="k", ...
            MarkerEdgeColor="k", DisplayName="로봇 핸드 (EE)");
% EE 궤적
hTrd = plot(axD, NaN, NaN, "-", LineWidth=2.0, ...
            Color=[0.5 0.05 0.0], DisplayName="핸드 궤적 (FK)");

legend(axD, Location="northeast", FontSize=9);
hTxtD = text(axD, -2.65, 2.55, "", FontSize=9, FontName="Courier New", ...
             BackgroundColor=[1.0 0.93 0.93], EdgeColor=[0.77 0.18 0.1], Margin=4);

% 궤적 버퍼
eeXu = nan(1,N+1);  eeYu = nan(1,N+1);
eeXd = nan(1,N+1);  eeYd = nan(1,N+1);

%% ── 5. 애니메이션 루프 ───────────────────────────────────────
for i = 1:N+1
    % ── Elbow-Up ──
    hL1u.XData = [0,         J1x_up(i)];
    hL1u.YData = [0,         J1y_up(i)];
    hL2u.XData = [J1x_up(i), EEx_up(i)];
    hL2u.YData = [J1y_up(i), EEy_up(i)];
    hEEu.XData = EEx_up(i);
    hEEu.YData = EEy_up(i);
    eeXu(i) = EEx_up(i);  eeYu(i) = EEy_up(i);
    hTru.XData = eeXu;
    hTru.YData = eeYu;

    % ── Elbow-Down ──
    hL1d.XData = [0,         J1x_dn(i)];
    hL1d.YData = [0,         J1y_dn(i)];
    hL2d.XData = [J1x_dn(i), EEx_dn(i)];
    hL2d.YData = [J1y_dn(i), EEy_dn(i)];
    hEEd.XData = EEx_dn(i);
    hEEd.YData = EEy_dn(i);
    eeXd(i) = EEx_dn(i);  eeYd(i) = EEy_dn(i);
    hTrd.XData = eeXd;
    hTrd.YData = eeYd;

    % ── 텍스트 ──
    hTxtU.String = sprintf( ...
        " th1 = %+7.2f deg\n th2 = %+7.2f deg\n X   = %+6.3f m\n Y   = %+6.3f m", ...
        rad2deg(theta1_up(i)), rad2deg(theta2_up(i)), EEx_up(i), EEy_up(i));
    hTxtD.String = sprintf( ...
        " th1 = %+7.2f deg\n th2 = %+7.2f deg\n X   = %+6.3f m\n Y   = %+6.3f m", ...
        rad2deg(theta1_dn(i)), rad2deg(theta2_dn(i)), EEx_dn(i), EEy_dn(i));

    drawnow limitrate;
    pause(0.008);
end

%% ══════════════════════════════════════════════════════════════
%  Figure 2 — 정적 결과 그래프 (3분할)
%% ══════════════════════════════════════════════════════════════
fig2 = figure(Name="FK 결과 정적 그래프 (R2026a)", Color="w", ...
              Position=[40 50 1200 800]);
tl2  = tiledlayout(fig2, 2, 2, TileSpacing="compact", Padding="compact");
title(tl2, "순기구학 결과 — θ₁+θ₂ 동시 구동 핸드 궤적", ...
      FontSize=13, FontWeight="bold");

steps = 0:N;

% ── (1,1) θ₁ 시계열 ──────────────────────────────────────────
ax21 = nexttile(tl2, 1);
plot(ax21, steps, rad2deg(theta1_up), "b-",  LineWidth=1.8, ...
     DisplayName="θ₁  Elbow-Up");
hold(ax21, "on");
plot(ax21, steps, rad2deg(theta1_dn), "r--", LineWidth=1.8, ...
     DisplayName="θ₁  Elbow-Down");
yline(ax21, 0, "k:", LineWidth=1.0, HandleVisibility="off");
xlabel(ax21, "Step", FontSize=11);
ylabel(ax21, "theta1 [deg]", FontSize=11);
title(ax21, "θ₁ 시계열", FontSize=11, FontWeight="bold");
legend(ax21, Location="best", FontSize=9);
grid(ax21, "on");

% ── (1,2) θ₂ 시계열 ──────────────────────────────────────────
ax22 = nexttile(tl2, 2);
plot(ax22, steps, rad2deg(theta2_up), "b-",  LineWidth=1.8, ...
     DisplayName="θ₂  Elbow-Up (+)");
hold(ax22, "on");
plot(ax22, steps, rad2deg(theta2_dn), "r--", LineWidth=1.8, ...
     DisplayName="θ₂  Elbow-Down (−)");
yline(ax22, 0, "k:", LineWidth=1.0, HandleVisibility="off");
xlabel(ax22, "Step", FontSize=11);
ylabel(ax22, "theta2 [deg]", FontSize=11);
title(ax22, "θ₂ 시계열", FontSize=11, FontWeight="bold");
legend(ax22, Location="best", FontSize=9);
grid(ax22, "on");

% ── (2,1) FK 핸드 궤적 — Elbow-Up ────────────────────────────
ax23 = nexttile(tl2, 3);
plot(ax23, Xd,     Yd,     "k--", LineWidth=2.0, DisplayName="목표 궤적 (타원)");
hold(ax23, "on");
plot(ax23, EEx_up, EEy_up, "b-",  LineWidth=1.5, DisplayName="FK 핸드 궤적 (Up)");
plot(ax23, EEx_up(1), EEy_up(1), "bo", MarkerSize=9, MarkerFaceColor="b", ...
     HandleVisibility="off");
xlabel(ax23, "X [m]", FontSize=11);
ylabel(ax23, "Y [m]", FontSize=11);
title(ax23, "FK 핸드 궤적 — Elbow-Up", FontSize=11, FontWeight="bold");
legend(ax23, Location="best", FontSize=9);
grid(ax23, "on"); axis(ax23, "equal");

% ── (2,2) FK 핸드 궤적 — Elbow-Down ─────────────────────────
ax24 = nexttile(tl2, 4);
plot(ax24, Xd,     Yd,     "k--", LineWidth=2.0, DisplayName="목표 궤적 (타원)");
hold(ax24, "on");
plot(ax24, EEx_dn, EEy_dn, "r-",  LineWidth=1.5, DisplayName="FK 핸드 궤적 (Down)");
plot(ax24, EEx_dn(1), EEy_dn(1), "ro", MarkerSize=9, MarkerFaceColor="r", ...
     HandleVisibility="off");
xlabel(ax24, "X [m]", FontSize=11);
ylabel(ax24, "Y [m]", FontSize=11);
title(ax24, "FK 핸드 궤적 — Elbow-Down", FontSize=11, FontWeight="bold");
legend(ax24, Location="best", FontSize=9);
grid(ax24, "on"); axis(ax24, "equal");

%% ── 6. 콘솔 출력 ────────────────────────────────────────────
fprintf("\n=============================================\n");
fprintf("  순기구학 (FK) 핸드 궤적 — 시뮬레이션 완료\n");
fprintf("---------------------------------------------\n");
fprintf("  [Elbow-Up  ] X 범위: %.3f ~ %.3f m\n", min(EEx_up), max(EEx_up));
fprintf("  [Elbow-Up  ] Y 범위: %.3f ~ %.3f m\n", min(EEy_up), max(EEy_up));
fprintf("  [Elbow-Down] X 범위: %.3f ~ %.3f m\n", min(EEx_dn), max(EEx_dn));
fprintf("  [Elbow-Down] Y 범위: %.3f ~ %.3f m\n", min(EEy_dn), max(EEy_dn));
fprintf("  FK 오차 (Up)  : %.2e m\n", err_up);
fprintf("  FK 오차 (Down): %.2e m\n", err_dn);
fprintf("=============================================\n");