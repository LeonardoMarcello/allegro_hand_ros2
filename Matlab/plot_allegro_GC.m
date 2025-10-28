close all
%% Plot Allegro Hand Joint States from ROS2 bag
% Make sure you have ROS Toolbox installed
addpath(genpath("description"))

% Path to the rosbag folder (ros2 bag record creates a folder)
bagFolder = './bag/allegro_hand_bag';
urdfFile = "description/urdf/allegro_hand_description_right_B.urdf";
% Open the ros2 bag
bag = ros2bag(bagFolder);

% Select the /allegroHand/joint_states topic
ts = select(bag, 'Topic', '/allegroHand_0/joint_states');

% Read all messages
msgs = readMessages(ts);

% Number of messages
N = numel(msgs);

% Extract joint names (from the first message)
jointNames = msgs{1}.name;   % cell array of 16 joint names
numJoints = numel(jointNames);

% Preallocate arrays
positions = zeros(N, numJoints);
velocity = zeros(N, numJoints);
efforts   = zeros(N, numJoints);
time      = zeros(N, 1);

% Loop through messages
% vector(N,i) -> N: timestamp, i: ith-joint
for i = 1:N
    msg = msgs{i};
    positions(i,:) = msg.position;
    velocities(i,:) = msg.velocity;
    efforts(i,:)   = msg.effort;
    time(i) = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
end

% Normalize time to start at 0
time = time - time(1);


%% 
pwm_limits = [-240,240];
tau2pwm = 1.43 * 1000;
% estimate tau friction
Kd = [0.004*sqrt(80), 0.008*sqrt(90), 0.008*sqrt(90), 0.004*sqrt(80)];
tau_fric = zeros(size(efforts));
for i = 1:16
    tau_fric(:,i) = - velocities(:,i) * Kd(mod(i-1,4)+1) * 0.015; 
end
%%
l = 0.05145; m = 0.0405;
tau_g = zeros(size(efforts));
for i = 1:4
    % iterate on finger
    C123 =cos(positions(:,4*(i-1)+2) + pi/2 + ...
                positions(:,4*(i-1)+3) + ...
                positions(:,4*(i-1)+4));
    tau_g(:, 4*(i-1) + 4) = l*m*9.81*C123;
end
mean(efforts(:,8)./tau_g(:,8))
std(efforts(:,8)-tau_g(:,8))
%% Plot joint positions
figure('Name','Joint Positions','NumberTitle','off');
for j = 1:16
    subplot(4,4,j);  % 4 rows, 1 column, index from 1 to 4
    plot(time, positions(:,j), 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Position [rad]');
    title(['Joint ', jointNames{j}]);
    grid on;
end
%% Plot joint velocity
figure('Name','Joint Velocities','NumberTitle','off');
for j = 1:16
    subplot(4,4,j);  % 4 rows, 1 column, index from 1 to 4
    plot(time, velocities(:,j), 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Velocity [rad/s]');
    title(['Joint ', jointNames{j}]);
    grid on;
end
%% Plot joint efforts
figure('Name','Joint Efforts','NumberTitle','off');
for j = 1:16
    subplot(4,4,j);  % 4 rows, 1 column, index from 1 to 4
    hold on
    plot(time, efforts(:,j), 'LineWidth', 1.5);
    plot(time, efforts(:,j) - tau_fric(:,j), 'LineWidth', 1.5);
    plot(time, tau_g(:,j), 'LineWidth', 1.5);
    % yline(pwm_limits(1)/tau2pwm)
    % yline(pwm_limits(2)/tau2pwm)
    xlabel('Time [s]');
    ylabel('Joint Effort [Nm]');
    title(['Allegro Hand Joint Efforts ', jointNames{j}]);
    grid on;
end




%% Plot tau3 CG
figure('Name','Joint3 barycenter','NumberTitle','off');
q1 = positions(:,6);
q2 = positions(:,7);
q3 = positions(:,8);
m3 = 0.0057;
plot(time, efforts(:,8)./(m3*9.81*sin(q1+q2+q3)), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('l3 [m]');
title('Allegro Hand Last link barycenter');
legend('l3', 'Interpreter', 'none');
grid on;


