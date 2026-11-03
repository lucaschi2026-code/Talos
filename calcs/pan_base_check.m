%% Talos pan servo and base check: shaft moment, tipping, turntable play
% The pan servo is the only joint whose output shaft carries the arm's full
% bending moment from one side (each pitch joint has an idler on the far side).
% Statics with the arm held straight out (the worst case). Talos masses and
% centres of mass are the CAD values printed by design_check.py; SO-101 values
% come from its URDF via so101_compare.py.

clear; clc;

%% Constants
g = 9.81;  % gravity, m/s^2

%% Talos (CAD): mass (kg) and horizontal CoM distance from the pan axis (m)
names  = ["base (fixed)", "shoulder link", "upper arm + elbow servo", ...
          "forearm + wrist-flex servo", "wrist + wrist-roll servo", ...
          "gripper + jaw servo", "payload"];
m      = [0.2256, 0.0999, 0.1145, 0.1192, 0.0927, 0.1028, 0.200];
x      = [-0.0090, 0.0000, 0.0819, 0.2114, 0.2886, 0.3527, 0.420];  % payload at the tip
moving = 2:7;  % everything the pan servo carries
arm    = 2:6;  % same, without the payload

%% SO-101 for comparison (URDF masses, arm straight out, from so101_compare.py)
% Its shoulder axis sits 30.4 mm forward of the pan axis, so reaching forward
% (which its joint limits allow) every lever arm about the pan axis grows by that much.
% Order: shoulder link, upper arm, forearm, wrist, gripper, moving jaw, payload at the tool point
so_m = [0.100, 0.103, 0.104, 0.079, 0.087, 0.012, 0.200];
so_x = [0.0308, 0.1194, 0.2445, 0.3200, 0.3676, 0.3953, 0.4408];

%% Check: the CAD moving mass should match design_check.py (529 g)
assert(abs(sum(m(arm)) - 0.529) < 0.002, 'Moving mass does not match the CAD')

%% 1. Overturning moment on the pan servo's output shaft
M_pan   = g * sum(m(moving) .* x(moving));
M_pan0  = g * sum(m(arm) .* x(arm));
M_so    = g * sum(so_m .* so_x);
M_so0   = g * sum(so_m(1:end-1) .* so_x(1:end-1));
F_axial = g * sum(m(moving));

%% 2. Base tipping about the front edge of the 12-sided plinth
apothem = 0.066 * cosd(15);              % base axis -> front edge, m
M_net   = g * sum(m .* (x - apothem));   % > 0 means the arm tips forward
bolt_x  = 0.054 * cosd(45);              % rear M4 bolts sit this far behind the axis
F_bolt  = max(M_net, 0) / (apothem + bolt_x) / 2;  % per bolt, two rear bolts

%% 3. Turntable play -> tip error
% If the servo shaft lets the turntable tilt, a rim lift d at radius r_tt tilts
% the whole arm by d / r_tt, and the reach multiplies that at the tip.
r_tt  = 0.037;                        % turntable radius, m
reach = 0.420;                        % pan axis -> gripper tip, arm straight, m
gain  = reach / r_tt;
lift  = [0.05, 0.10, 0.20, 0.30];     % rim lift, mm
tip   = gain * lift;                  % tip error, mm

%% Results
fprintf('Pan shaft overturning moment, arm straight out\n');
fprintf('  Talos:  %.2f N*m no payload, %.2f N*m with 200 g at the tip\n', M_pan0, M_pan);
fprintf('  SO-101: %.2f N*m no payload, %.2f N*m with 200 g at the tip\n', M_so0, M_so);
fprintf('  Talos / SO-101: %.2fx loaded, %.2fx unloaded\n', M_pan/M_so, M_pan0/M_so0);
fprintf('  Axial load on the pan servo: %.1f N\n\n', F_axial);
fprintf('Base tipping about the front edge (%.0f mm out)\n', 1000*apothem);
fprintf('  Whole-arm CoM: %.0f mm from the base axis\n', 1000*sum(m .* x)/sum(m));
if M_net > 0
    fprintf('  TIPS: net overturning %.2f N*m, needs %.1f N per rear M4 bolt\n\n', M_net, F_bolt);
else
    fprintf('  Stable, margin %.2f N*m\n\n', -M_net);
end
fprintf('Turntable rim lift -> tip error (x%.1f)\n', gain);
disp(table(lift', tip', 'VariableNames', {'Rim_lift_mm', 'Tip_error_mm'}))
disp(table(names', m', (g*m.*x)', 'VariableNames', {'Mass', 'kg', 'Pan_moment_Nm'}))

%% Pan shaft moment vs payload at the tip
p = linspace(0, 0.3, 61);  % kg
plot(1000*p, M_pan0 + g*p*x(end), 'LineWidth', 1.5); hold on
plot(1000*p, M_so0 + g*p*so_x(end), '--', 'LineWidth', 1.5);
xline(200, ':k', '200 g target');
xlabel('Payload at the tip (g)'); ylabel('Pan shaft overturning moment (N*m)');
title('Pan servo shaft moment, arm straight out');
legend('Talos (CAD)', 'SO-101', 'Location', 'northwest'); grid on
theme(gcf, 'light');
exportgraphics(gcf, 'pan_moment.png', 'Resolution', 150);
