%% ============================================================
%  2-Link Manipulator — θ₁ 역기구학 두 가지 해 시각화
%  MATLAB R2026a 최적화 버전
%
%  타스크 궤적 : 타원  x²/2.0² + y²/1.3² = 1
%  초기값     : x = 2.0, y = 0
%  링크 길이   : L1 = L2 = 1 m
%
%  ★ θ₁ 두 해:
%     Elbow-Up   (+해) : θ₂ > 0 → θ₁ = atan2(y,x) − atan2(L2·sinθ₂, L1+L2·cosθ₂)
%     Elbow-Down (−해) : θ₂ < 0 → θ₁ = atan2(y,x) − atan2(L2·sinθ₂, L1+L2·cosθ₂)
%     (θ₂의 부호에 따라 θ₁도 달라짐)
% ============================================================

clear; clc; close all;

%% ── 1. 파라미터 ─────────────────────────────────────────────
L1 = 1.0;
L2 = 1.0;
a  = 2.0;
b  = 1.3;
N  = 360;

%% ── 2. 타원 궤적 ────────────────────────────────────────────
t  = linspace(0, 2*pi, N+1);
Xd = a .* cos(t);
Yd = b .* sin(t);

%% ── 3. 역기구학 — θ₁, θ₂ 두 가지 해 계산 ───────────────────
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

    % Elbow-Up  : θ₂ > 0
    th2_up = atan2(+sqrt(1 - D^2), D);
    th1_up = atan2(y, x) - atan2(L2*sin(th2_up), L1 + L2*cos(th2_up));

    % Elbow-Down: θ₂ < 0
    th2_dn = atan2(-sqrt(1 - D^2), D);
    th1_dn = atan2(y, x) - atan2(L2*sin(th2_dn), L1 + L2*cos(th2_dn));

    theta1_up(i) = th1_up;  theta2_up(i) = th2_up;
    theta1_dn(i) = th1_dn;  theta2_dn(i) = th2_dn;
end

%% ── 4. 순기구학 검증 ─────────────────────────────────────────
Xfk_up = L1.*cos(theta1_up) + L2.*cos(theta1_up + theta2_up);
Yfk_up = L1.*sin(theta1_up) + L2.*sin(theta1_up + theta2_up);
Xfk_dn = L1.*cos(theta1_dn) + L2.*cos(theta1_dn + theta2_dn);
Yfk_dn = L1.*sin(theta1_dn) + L2.*sin(theta1_dn + theta2_dn);

fprintf("FK 오차 (Elbow-Up)  : %.2e m\n", max(hypot(Xfk_up-Xd, Yfk_up-Yd)));
fprintf("FK 오차 (Elbow-Down): %.2e m\n", max(hypot(Xfk_dn-Xd, Yfk_dn-Yd)));

%% ══════════════════════════════════════════════════════════════
%  Figure 1 — θ₁ 두 해 동시 애니메이션 (좌: Elbow-Up / 우: Elbow-Down)
%% ══════════════════════════════════════════════════════════════
fig1 = figure(Name="θ₁ Dual Solution Animation (R2026a)", ...
              Color="w", Position=[40 60 1300 700]);

tl1 = tiledlayout(fig1, 1, 2, TileSpacing="compact", Padding="compact");
title(tl1, "2-Link Manipulator — θ₁ 역기구학 두 가지 해 (링크1 궤적 강조)", ...
      FontSize=14, FontWeight="bold");

% ── 왼쪽 축: Elbow-Up ────────────────────────────────────────
axU = nexttile(tl1, 1);
hold(axU, "on");
grid(axU, "on");
axis(axU, "equal");
xlim(axU, [-2.8, 2.8]);
ylim(axU, [-2.8, 2.8]);
xlabel(axU, "X [m]", FontSize=11);
ylabel(axU, "Y [m]", FontSize=11);
title(axU, "Elbow-Up  (θ₁ — +해 기반)", FontSize=12, ...
      Color=[0 0.45 0.74], FontWeight="bold");

% 목표 타원
plot(axU, Xd, Yd, "--", Color=[0.75 0.75 0.75], LineWidth=1.2, ...
     DisplayName="목표 궤적 (타원)");
% 베이스
plot(axU, 0, 0, "ks", MarkerSize=11, MarkerFaceColor=[0.2 0.2 0.2], ...
     HandleVisibility="off");

% 링크1 (θ₁ 담당, 강조색)
hL1u = plot(axU, [0 0], [0 0], "-o", LineWidth=5, MarkerSize=9, ...
            Color=[0 0.45 0.74], MarkerFaceColor=[0 0.45 0.74], ...
            DisplayName="링크1 (θ₁)");
% 관절2 궤적 (링크1 끝점)
hJ1u = plot(axU, NaN, NaN, "-", LineWidth=2.2, ...
            Color=[0 0.45 0.74], DisplayName="관절2 궤적 (링크1 끝)");

legend(axU, Location="northeast", FontSize=9);
hTxtU = text(axU, -2.65, 2.55, "", FontSize=9, FontName="Courier New", ...
             BackgroundColor=[0.92 0.96 1.0], EdgeColor=[0 0.45 0.74], Margin=4);

% ── 오른쪽 축: Elbow-Down ────────────────────────────────────
axD = nexttile(tl1, 2);
hold(axD, "on");
grid(axD, "on");
axis(axD, "equal");
xlim(axD, [-2.8, 2.8]);
ylim(axD, [-2.8, 2.8]);
xlabel(axD, "X [m]", FontSize=11);
ylabel(axD, "Y [m]", FontSize=11);
title(axD, "Elbow-Down (θ₁ — −해 기반)", FontSize=12, ...
      Color=[0.77 0.18 0.1], FontWeight="bold");

plot(axD, Xd, Yd, "--", Color=[0.75 0.75 0.75], LineWidth=1.2, ...
     DisplayName="목표 궤적 (타원)");
plot(axD, 0, 0, "ks", MarkerSize=11, MarkerFaceColor=[0.2 0.2 0.2], ...
     HandleVisibility="off");

hL1d = plot(axD, [0 0], [0 0], "-o", LineWidth=5, MarkerSize=9, ...
            Color=[0.77 0.18 0.1], MarkerFaceColor=[0.77 0.18 0.1], ...
            DisplayName="링크1 (θ₁)");
% 관절2 궤적 (링크1 끝점)
hJ1d = plot(axD, NaN, NaN, "-", LineWidth=2.2, ...
            Color=[0.77 0.18 0.1], DisplayName="관절2 궤적 (링크1 끝)");

legend(axD, Location="northeast", FontSize=9);
hTxtD = text(axD, -2.65, 2.55, "", FontSize=9, FontName="Courier New", ...
             BackgroundColor=[1.0 0.93 0.93], EdgeColor=[0.77 0.18 0.1], Margin=4);

% 궤적 버퍼
j1xU = nan(1,N+1); j1yU = nan(1,N+1);   % 관절2(링크1 끝) 궤적 Up
j1xD = nan(1,N+1); j1yD = nan(1,N+1);   % 관절2(링크1 끝) 궤적 Down

%% ── 5. 애니메이션 루프 ───────────────────────────────────────
for i = 1:N+1
    % Elbow-Up 관절 좌표
    J1Ux = L1 * cos(theta1_up(i));
    J1Uy = L1 * sin(theta1_up(i));
    J2Ux = J1Ux + L2 * cos(theta1_up(i) + theta2_up(i));
    J2Uy = J1Uy + L2 * sin(theta1_up(i) + theta2_up(i));

    % Elbow-Down 관절 좌표
    J1Dx = L1 * cos(theta1_dn(i));
    J1Dy = L1 * sin(theta1_dn(i));
    J2Dx = J1Dx + L2 * cos(theta1_dn(i) + theta2_dn(i));
    J2Dy = J1Dy + L2 * sin(theta1_dn(i) + theta2_dn(i));

    % ── Elbow-Up 업데이트 ──
    hL1u.XData = [0,    J1Ux];   hL1u.YData = [0,    J1Uy];
    j1xU(i) = J1Ux;  j1yU(i) = J1Uy;
    hJ1u.XData = j1xU;            hJ1u.YData = j1yU;

    % ── Elbow-Down 업데이트 ──
    hL1d.XData = [0,    J1Dx];   hL1d.YData = [0,    J1Dy];
    j1xD(i) = J1Dx;  j1yD(i) = J1Dy;
    hJ1d.XData = j1xD;            hJ1d.YData = j1yD;

    % ── 텍스트 ──
    hTxtU.String = sprintf( ...
        " th1 = %+7.2f deg  (<<)\n J2x = %5.3f m\n J2y = %5.3f m", ...
        rad2deg(theta1_up(i)), J1Ux, J1Uy);
    hTxtD.String = sprintf( ...
        " th1 = %+7.2f deg  (<<)\n J2x = %5.3f m\n J2y = %5.3f m", ...
        rad2deg(theta1_dn(i)), J1Dx, J1Dy);

    drawnow limitrate;
    pause(0.008);
end

%% ══════════════════════════════════════════════════════════════
%  Figure 2 — θ₁ 두 해 비교 정적 그래프 (4분할)
%% ══════════════════════════════════════════════════════════════
fig2 = figure(Name="θ₁ 두 해 비교 (R2026a)", Color="w", ...
              Position=[40 60 1100 820]);
tl2  = tiledlayout(fig2, 2, 2, TileSpacing="compact", Padding="compact");
title(tl2, "θ₁ Elbow-Up(+) vs Elbow-Down(−) 비교", ...
      FontSize=13, FontWeight="bold");

steps = 0:N;

% ── (1,1) θ₁ 두 해 시계열 비교 ──────────────────────────────
ax21 = nexttile(tl2, 1);
plot(ax21, steps, rad2deg(theta1_up), "b-",  LineWidth=2.2, ...
     DisplayName="θ₁  Elbow-Up (+)");
hold(ax21, "on");
plot(ax21, steps, rad2deg(theta1_dn), "r--", LineWidth=2.2, ...
     DisplayName="θ₁  Elbow-Down (−)");
yline(ax21, 0, "k:", LineWidth=1.2, HandleVisibility="off");
xlabel(ax21, "Step",           FontSize=11);
ylabel(ax21, "theta1 [deg]",   FontSize=11);
title(ax21,  "θ₁ 두 해 시계열",    FontSize=12, FontWeight="bold");
legend(ax21, Location="best",  FontSize=9);
grid(ax21, "on");

% ── (1,2) θ₁ 두 해 차이(Δθ₁) ────────────────────────────────
ax22 = nexttile(tl2, 2);
delta_th1 = rad2deg(theta1_up - theta1_dn);
plot(ax22, steps, delta_th1, "m-", LineWidth=2.0, ...
     DisplayName="Delta theta1 (Up − Down)");
yline(ax22, 0, "k:", LineWidth=1.2, HandleVisibility="off");
xlabel(ax22, "Step",                  FontSize=11);
ylabel(ax22, "Delta theta1 [deg]",    FontSize=11);
title(ax22,  "θ₁ 두 해의 차이 (Up − Down)", FontSize=12, FontWeight="bold");
legend(ax22, Location="best", FontSize=9);
grid(ax22, "on");

% ── (2,1) 관절2(링크1 끝점) 궤적 비교 ──────────────────────
ax23 = nexttile(tl2, 3);
% 링크1 끝점(관절2) 좌표
J1x_up = L1 .* cos(theta1_up);
J1y_up = L1 .* sin(theta1_up);
J1x_dn = L1 .* cos(theta1_dn);
J1y_dn = L1 .* sin(theta1_dn);

% 반지름 1 원 (링크1 끝점이 그리는 이론 궤적)
th_c = linspace(0, 2*pi, 500);
plot(ax23, cos(th_c), sin(th_c), "k:", LineWidth=1.2, ...
     DisplayName="반지름 1 원 (이론)");
hold(ax23, "on");
plot(ax23, J1x_up, J1y_up, "b-", LineWidth=2.0, ...
     DisplayName="관절2 궤적 — Elbow-Up");
plot(ax23, J1x_dn, J1y_dn, "r--", LineWidth=2.0, ...
     DisplayName="관절2 궤적 — Elbow-Down");
plot(ax23, 0, 0, "ks", MarkerSize=10, MarkerFaceColor=[0.2 0.2 0.2], ...
     HandleVisibility="off");
xlabel(ax23, "X [m]", FontSize=11);
ylabel(ax23, "Y [m]", FontSize=11);
title(ax23, "관절2 궤적 비교 (링크1 끝점, L1=1m)", FontSize=12, FontWeight="bold");
legend(ax23, Location="best", FontSize=9);
grid(ax23, "on");
axis(ax23, "equal");
xlim(ax23, [-1.5, 1.5]);
ylim(ax23, [-1.5, 1.5]);

% ── (2,2) θ₁ 위상 궤적 (극좌표 스타일: step vs θ₁ 원형) ────
ax24 = nexttile(tl2, 4);
plot(ax24, steps, rad2deg(theta1_up), "b-", LineWidth=1.8, ...
     DisplayName="θ₁ Elbow-Up");
hold(ax24, "on");
plot(ax24, steps, rad2deg(theta1_dn), "r-", LineWidth=1.8, ...
     DisplayName="θ₁ Elbow-Down");
% 초기값 마킹
plot(ax24, 0,   rad2deg(theta1_up(1)), "bo", MarkerSize=9, ...
     MarkerFaceColor="b", HandleVisibility="off");
plot(ax24, 0,   rad2deg(theta1_dn(1)), "ro", MarkerSize=9, ...
     MarkerFaceColor="r", HandleVisibility="off");
xlabel(ax24, "Step",           FontSize=11);
ylabel(ax24, "theta1 [deg]",   FontSize=11);
title(ax24,  "θ₁ 두 해 오버레이 (초기값 표시)", FontSize=12, FontWeight="bold");
legend(ax24, Location="best",  FontSize=9);
grid(ax24, "on");

%% ── 6. 콘솔 출력 ────────────────────────────────────────────
fprintf("\n============================================\n");
fprintf("  theta1 역기구학 두 가지 해 — 시뮬레이션 완료\n");
fprintf("--------------------------------------------\n");
fprintf("  [Elbow-Up  (+)] theta1 범위: %+.1f deg ~ %+.1f deg\n", ...
        min(rad2deg(theta1_up)), max(rad2deg(theta1_up)));
fprintf("  [Elbow-Down(-)] theta1 범위: %+.1f deg ~ %+.1f deg\n", ...
        min(rad2deg(theta1_dn)), max(rad2deg(theta1_dn)));
fprintf("  theta1 최대 차이 (Up-Down): %.2f deg\n", max(abs(delta_th1)));
fprintf("  FK 오차 (Up)  : %.2e m\n", max(hypot(Xfk_up-Xd, Yfk_up-Yd)));
fprintf("  FK 오차 (Down): %.2e m\n", max(hypot(Xfk_dn-Xd, Yfk_dn-Yd)));
fprintf("============================================\n");