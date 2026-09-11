%% ============================================================
%  2-Link Manipulator — θ₂ 역기구학 두 가지 해 시각화
%  MATLAB R2026a 최적화 버전
%
%  타스크 궤적 : 타원  x²/2.0² + y²/1.3² = 1
%  초기값     : x = 2.0, y = 0
%  링크 길이   : L1 = L2 = 1 m
%
%  ★ θ₂ 두 해:
%     Elbow-Up   (+해) : θ₂ = atan2(+√(1−D²), D)  → θ₂ > 0
%     Elbow-Down (−해) : θ₂ = atan2(−√(1−D²), D)  → θ₂ < 0
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

%% ── 3. 역기구학 — 두 가지 해 ────────────────────────────────
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
%  Figure 1 — 두 해 동시 애니메이션 (좌: Elbow-Up / 우: Elbow-Down)
%% ══════════════════════════════════════════════════════════════
fig1 = figure(Name="θ₂ Dual Solution Animation (R2026a)", ...
              Color="w", Position=[40 60 1300 700]);

tl1 = tiledlayout(fig1, 1, 2, TileSpacing="compact", Padding="compact");
title(tl1, "2-Link Manipulator — θ₂ 역기구학 두 가지 해", ...
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
title(axU, "Elbow-Up  (θ₂ > 0, +해)", FontSize=12, ...
      Color=[0 0.3 0.8], FontWeight="bold");

plot(axU, Xd, Yd, "--", Color=[0.7 0.7 0.7], LineWidth=1.2, ...
     DisplayName="목표 궤적");
plot(axU, 0, 0, "ks", MarkerSize=11, MarkerFaceColor=[0.2 0.2 0.2], ...
     HandleVisibility="off");
hL1u  = plot(axU, [0 0], [0 0], "b-o", LineWidth=3.5, MarkerSize=8, ...
             MarkerFaceColor="b", DisplayName="링크1");
hL2u  = plot(axU, [0 0], [0 0], "c-o", LineWidth=3.5, MarkerSize=8, ...
             MarkerFaceColor="c", DisplayName="링크2");
hEEu  = plot(axU, 0, 0, "ko", MarkerSize=10, MarkerFaceColor="k", ...
             DisplayName="EE");
hTru  = plot(axU, NaN, NaN, "b-", LineWidth=1.5, DisplayName="EE 궤적");
legend(axU, Location="northeast", FontSize=9);
hTxtU = text(axU, -2.6, 2.5, "", FontSize=9, FontName="Courier New", ...
             BackgroundColor=[0.94 0.97 1.0], EdgeColor=[0 0.3 0.8], Margin=4);

% ── 오른쪽 축: Elbow-Down ────────────────────────────────────
axD = nexttile(tl1, 2);
hold(axD, "on");
grid(axD, "on");
axis(axD, "equal");
xlim(axD, [-2.8, 2.8]);
ylim(axD, [-2.8, 2.8]);
xlabel(axD, "X [m]", FontSize=11);
ylabel(axD, "Y [m]", FontSize=11);
title(axD, "Elbow-Down (θ₂ < 0, −해)", FontSize=12, ...
      Color=[0.8 0.15 0.1], FontWeight="bold");

plot(axD, Xd, Yd, "--", Color=[0.7 0.7 0.7], LineWidth=1.2, ...
     DisplayName="목표 궤적");
plot(axD, 0, 0, "ks", MarkerSize=11, MarkerFaceColor=[0.2 0.2 0.2], ...
     HandleVisibility="off");
hL1d  = plot(axD, [0 0], [0 0], "r-o", LineWidth=3.5, MarkerSize=8, ...
             MarkerFaceColor="r", DisplayName="링크1");
hL2d  = plot(axD, [0 0], [0 0], "m-o", LineWidth=3.5, MarkerSize=8, ...
             MarkerFaceColor="m", DisplayName="링크2");
hEEd  = plot(axD, 0, 0, "ko", MarkerSize=10, MarkerFaceColor="k", ...
             DisplayName="EE");
hTrd  = plot(axD, NaN, NaN, "r-", LineWidth=1.5, DisplayName="EE 궤적");
legend(axD, Location="northeast", FontSize=9);
hTxtD = text(axD, -2.6, 2.5, "", FontSize=9, FontName="Courier New", ...
             BackgroundColor=[1.0 0.94 0.94], EdgeColor=[0.8 0.15 0.1], Margin=4);

% 궤적 버퍼
txU = nan(1, N+1);  tyU = nan(1, N+1);
txD = nan(1, N+1);  tyD = nan(1, N+1);

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

    % Elbow-Up dot-notation 업데이트
    hL1u.XData = [0,    J1Ux];   hL1u.YData = [0,    J1Uy];
    hL2u.XData = [J1Ux, J2Ux];   hL2u.YData = [J1Uy, J2Uy];
    hEEu.XData = J2Ux;            hEEu.YData = J2Uy;
    txU(i) = J2Ux;  tyU(i) = J2Uy;
    hTru.XData = txU;             hTru.YData = tyU;

    % Elbow-Down dot-notation 업데이트
    hL1d.XData = [0,    J1Dx];   hL1d.YData = [0,    J1Dy];
    hL2d.XData = [J1Dx, J2Dx];   hL2d.YData = [J1Dy, J2Dy];
    hEEd.XData = J2Dx;            hEEd.YData = J2Dy;
    txD(i) = J2Dx;  tyD(i) = J2Dy;
    hTrd.XData = txD;             hTrd.YData = tyD;

    % 정보 텍스트
    hTxtU.String = sprintf( ...
        " th1 = %+7.2f deg\n th2 = %+7.2f deg (+)\n X   = %5.3f m\n Y   = %5.3f m", ...
        rad2deg(theta1_up(i)), rad2deg(theta2_up(i)), J2Ux, J2Uy);
    hTxtD.String = sprintf( ...
        " th1 = %+7.2f deg\n th2 = %+7.2f deg (-)\n X   = %5.3f m\n Y   = %5.3f m", ...
        rad2deg(theta1_dn(i)), rad2deg(theta2_dn(i)), J2Dx, J2Dy);

    drawnow limitrate;
    pause(0.008);
end

%% ══════════════════════════════════════════════════════════════
%  Figure 2 — θ₂ 두 해 비교 정적 그래프
%% ══════════════════════════════════════════════════════════════
fig2 = figure(Name="θ₂ 두 해 비교 (R2026a)", Color="w", ...
              Position=[40 60 1100 820]);
tl2  = tiledlayout(fig2, 2, 2, TileSpacing="compact", Padding="compact");
title(tl2, "θ₂ Elbow-Up(+) vs Elbow-Down(−) 비교", ...
      FontSize=13, FontWeight="bold");

steps = 0:N;

% ── (1,1) θ₂ 두 해 시계열 비교 ──
ax21 = nexttile(tl2, 1);
plot(ax21, steps, rad2deg(theta2_up), "b-",  LineWidth=2.0, ...
     DisplayName="θ₂  Elbow-Up (+)");
hold(ax21, "on");
plot(ax21, steps, rad2deg(theta2_dn), "r--", LineWidth=2.0, ...
     DisplayName="θ₂  Elbow-Down (−)");
yline(ax21, 0, "k:", LineWidth=1.2, HandleVisibility="off");
xlabel(ax21, "Step",           FontSize=11);
ylabel(ax21, "theta2 [deg]",   FontSize=11);
title(ax21,  "θ₂ 두 해 시계열",    FontSize=11);
legend(ax21, Location="best",  FontSize=9);
grid(ax21, "on");

% ── (1,2) θ₁ 두 해 시계열 비교 ──
ax22 = nexttile(tl2, 2);
plot(ax22, steps, rad2deg(theta1_up), "b-",  LineWidth=2.0, ...
     DisplayName="θ₁  Elbow-Up");
hold(ax22, "on");
plot(ax22, steps, rad2deg(theta1_dn), "r--", LineWidth=2.0, ...
     DisplayName="θ₁  Elbow-Down");
xlabel(ax22, "Step",           FontSize=11);
ylabel(ax22, "theta1 [deg]",   FontSize=11);
title(ax22,  "θ₁ 두 해 시계열",    FontSize=11);
legend(ax22, Location="best",  FontSize=9);
grid(ax22, "on");

% ── (2,1) Elbow-Up FK 검증 ──
ax23 = nexttile(tl2, 3);
plot(ax23, Xd,     Yd,     "k--", LineWidth=2.0, DisplayName="목표 궤적");
hold(ax23, "on");
plot(ax23, Xfk_up, Yfk_up, "b-",  LineWidth=1.3, DisplayName="FK (Elbow-Up)");
xlabel(ax23, "X [m]", FontSize=11);
ylabel(ax23, "Y [m]", FontSize=11);
title(ax23,  "FK 검증 — Elbow-Up (θ₂ > 0)", FontSize=11);
legend(ax23, Location="best", FontSize=9);
grid(ax23, "on");
axis(ax23, "equal");

% ── (2,2) Elbow-Down FK 검증 ──
ax24 = nexttile(tl2, 4);
plot(ax24, Xd,     Yd,     "k--", LineWidth=2.0, DisplayName="목표 궤적");
hold(ax24, "on");
plot(ax24, Xfk_dn, Yfk_dn, "r-",  LineWidth=1.3, DisplayName="FK (Elbow-Down)");
xlabel(ax24, "X [m]", FontSize=11);
ylabel(ax24, "Y [m]", FontSize=11);
title(ax24,  "FK 검증 — Elbow-Down (θ₂ < 0)", FontSize=11);
legend(ax24, Location="best", FontSize=9);
grid(ax24, "on");
axis(ax24, "equal");

%% ── 6. 콘솔 출력 ────────────────────────────────────────────
fprintf("\n============================================\n");
fprintf("  theta2 역기구학 두 가지 해 — 시뮬레이션 완료\n");
fprintf("--------------------------------------------\n");
fprintf("  [Elbow-Up  (+)] theta2 범위: %+.1f deg ~ %+.1f deg\n", ...
        min(rad2deg(theta2_up)), max(rad2deg(theta2_up)));
fprintf("  [Elbow-Down(-)] theta2 범위: %+.1f deg ~ %+.1f deg\n", ...
        min(rad2deg(theta2_dn)), max(rad2deg(theta2_dn)));
fprintf("  FK 오차 (Up)  : %.2e m\n", max(hypot(Xfk_up-Xd, Yfk_up-Yd)));
fprintf("  FK 오차 (Down): %.2e m\n", max(hypot(Xfk_dn-Xd, Yfk_dn-Yd)));
fprintf("============================================\n");