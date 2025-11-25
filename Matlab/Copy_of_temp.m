% INDEX
theta = deg2rad(-5);   % rotation in radians (here 30 degrees)
Rx = [1 0 0;
      0 cos(theta) -sin(theta);
      0 sin(theta)  cos(theta)];
d = 1e-3*[0;43.16;14.83];
backtol0 = 1e-3*[0;0;-17];

% Homogeneous transformation matrix
T1 = [Rx, d;
     0 0 0 1];
Tb2l0 = [eye(3), backtol0;
        0 0 0 1];
T = Tb2l0*T1;

T(1:3,4)
% --- Plot settings ---
figure; hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
% Length of axis arrows
L = 0.02; 
% --- World frame (identity) ---
O = [0 0 0]';
% World axes
quiver3(O(1), O(2), O(3), L, 0, 0, 'r', 'LineWidth', 2); % X
quiver3(O(1), O(2), O(3), 0, L, 0, 'g', 'LineWidth', 2); % Y
quiver3(O(1), O(2), O(3), 0, 0, L, 'b', 'LineWidth', 2); % Z
% --- Transformed frame ---
R = T(1:3,1:3);
p = T(1:3,4);
% Axes of transformed frame
quiver3(p(1), p(2), p(3), L*R(1,1), L*R(2,1), L*R(3,1), 'r', 'LineWidth', 2);
quiver3(p(1), p(2), p(3), L*R(1,2), L*R(2,2), L*R(3,2), 'g', 'LineWidth', 2);
quiver3(p(1), p(2), p(3), L*R(1,3), L*R(2,3), L*R(3,3), 'b', 'LineWidth', 2);
title('World Frame and Transformed Frame');


% RING
theta = deg2rad(5);   % rotation in radians (here 30 degrees)
Rx = [1 0 0;
      0 cos(theta) -sin(theta);
      0 sin(theta)  cos(theta)];
d = 1e-3*[0;-44.95;14.96];
backtol0 = 1e-3*[0;0;-17];

% Homogeneous transformation matrix
T1 = [Rx, d;
     0 0 0 1];
Tb2l0 = [eye(3), backtol0;
        0 0 0 1];

T = Tb2l0*T1;

T(1:3,4)

% --- Plot settings ---
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
% Length of axis arrows
L = 0.02; 
% --- World frame (identity) ---
O = [0 0 0]';
% World axes
quiver3(O(1), O(2), O(3), L, 0, 0, 'r', 'LineWidth', 2); % X
quiver3(O(1), O(2), O(3), 0, L, 0, 'g', 'LineWidth', 2); % Y
quiver3(O(1), O(2), O(3), 0, 0, L, 'b', 'LineWidth', 2); % Z
% --- Transformed frame ---
R = T(1:3,1:3);
p = T(1:3,4);
% Axes of transformed frame
quiver3(p(1), p(2), p(3), L*R(1,1), L*R(2,1), L*R(3,1), 'r', 'LineWidth', 2);
quiver3(p(1), p(2), p(3), L*R(1,2), L*R(2,2), L*R(3,2), 'g', 'LineWidth', 2);
quiver3(p(1), p(2), p(3), L*R(1,3), L*R(2,3), L*R(3,3), 'b', 'LineWidth', 2);
title('World Frame and Transformed Frame');