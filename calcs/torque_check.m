%% Talos torque check: shoulder and elbow, arm held straight out (worst case)
% Statics: tau = g * sum(m_i * x_i), where x_i is each mass's horizontal
% distance from the joint axis. Masses and CoM offsets are the lumped CAD values
% printed by design_check.py (each link includes the servo mounted on it).
% Replace them with kitchen-scale values once parts are printed.

clear; clc;

%% Constants
g          = 9.81;  % gravity, m/s^2
tau_stall  = 2.94;  % STS3215 12 V stall torque, N*m (30 kg*cm)
limit_frac = 1/3;   % keep continuous load under this fraction of stall

%% Geometry (m)
L1    = 0.116;  % shoulder axis -> elbow axis (upper arm)
L2    = 0.135;  % elbow axis -> wrist-flex axis (forearm)
L_tip = 0.169;  % wrist-flex axis -> gripper tip (63 mm wrist + 106 mm gripper)

%% Masses (kg), in this order: upper arm, forearm, wrist, gripper, payload
names = ["upper arm + elbow servo", "forearm + wrist-flex servo", ...
         "wrist + wrist-roll servo", "gripper + jaw servo", "payload"];
m     = [0.114, 0.119, 0.092, 0.101, 0.200];

% Centre-of-mass offsets (m), each measured from the START of its own segment
d_upper   = 0.0810;     % from shoulder axis (elbow servo pulls COM toward the elbow)
d_fore    = 0.0938;     % from elbow axis
d_wrist   = 0.0382;     % from wrist-flex axis
d_gripper = 0.1017;     % from wrist-flex axis
d_payload = L_tip;      % from wrist-flex axis (held at the tip)

%% Torque about each joint
x_shoulder   = [d_upper, L1 + d_fore, L1 + L2 + d_wrist, L1 + L2 + d_gripper, L1 + L2 + d_payload];
tau_shoulder = g * sum(m .* x_shoulder);
x_elbow      = [0, d_fore, L2 + d_wrist, L2 + d_gripper, L2 + d_payload];
tau_elbow    = g * sum(m .* x_elbow);

%% Check: the payload alone should match a hand calc
assert(abs(x_shoulder(5) - (L1 + L2 + L_tip)) < 1e-9, 'Payload distance from shoulder is wrong')

%% Results
fprintf('Reach (shoulder to tip): %.0f mm\n', 1000*(L1 + L2 + L_tip));
fprintf('Shoulder: %.2f N*m = %.0f%% of stall\n', tau_shoulder, 100*tau_shoulder/tau_stall);
fprintf('Elbow:    %.2f N*m = %.0f%% of stall\n', tau_elbow, 100*tau_elbow/tau_stall);
fprintf('Limit:    %.2f N*m (%.0f%% of stall)\n', limit_frac*tau_stall, 100*limit_frac);
disp(table(names', m', (g*m.*x_shoulder)', 'VariableNames', {'Mass', 'kg', 'Shoulder_Nm'}))

%% Shoulder torque vs shoulder angle (elbow straight, 0 deg = horizontal)
theta = linspace(-90, 90, 181);  % deg
plot(theta, tau_shoulder*cosd(theta), 'LineWidth', 1.5); hold on
yline(limit_frac*tau_stall, '--r', '1/3 stall');
yline(tau_stall, '-k', 'stall');
xlabel('Shoulder angle from horizontal (deg)'); ylabel('Shoulder torque (N*m)');
title('Talos shoulder torque, arm straight'); legend('load', 'Location', 'south'); grid on
xlim([-90 90]); ylim([0 1.1*tau_stall]); theme(gcf, 'light');
exportgraphics(gcf, 'shoulder_torque.png', 'Resolution', 150);
