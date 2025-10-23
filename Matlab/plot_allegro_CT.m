close all
%% Plot Allegro Hand Joint States from ROS2 bag
% Make sure you have ROS Toolbox installed
addpath(genpath("description"))

ANIMATE = true;
EXPORT = false;

% Path to the rosbag folder (ros2 bag record creates a folder)
bagFolder = './bag/allegro_hand_torque_bag';
% bagFolder = './bag/allegro_hand_bag';
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
efforts   = zeros(N, numJoints);
time      = zeros(N, 1);

% Loop through messages
for i = 1:N
    msg = msgs{i};
    positions(i,:) = msg.position;
    efforts(i,:)   = msg.effort;
    time(i) = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
end

% Normalize time to start at 0
time = time - time(1);


%% 
pwm_limits = [-240,240];
tau2pwm = 1.43 * 1000;

%% write to csv
if EXPORT    
    % Build column names
    timeHeader = "time";
    posHeaders = "pos_" + string(jointNames(:))';
    velHeaders = "vel_" + string(jointNames(:))';
    accHeaders = "acc_" + string(jointNames(:))';
    effHeaders = "eff_" + string(jointNames(:))';
    headers = [timeHeader, posHeaders, velHeaders, accHeaders, effHeaders];  % now all row vectors
    
    data_times = time
    data_pos = positions
    date_effort = efforts
    
    % Convert to table
    T = array2table(data, 'VariableNames', headers);
    
    % Write to CSV
    outputFile = fullfile(bagFolder, 'allegro_joint_data.csv');
    writetable(T, outputFile);
    fprintf('✅ Exported time series to %s\n', outputFile);
    
    outputFile = fullfile('.', 'allegro_joint_data.csv');
    writetable(T, outputFile);
    fprintf('✅ Exported time series to %s\n', outputFile);
end
%% Plot joint positions
figure('Name','Joint Positions','NumberTitle','off');
for j = 5:8
    plot(time, positions(:,j), 'LineWidth', 1.5);
    hold on;
end
xlabel('Time [s]');
ylabel('Joint Position [rad]');
title('Allegro Hand Joint Positions');
legend(jointNames([5 6 7 8]), 'Interpreter', 'none');
grid on;

%% Plot joint efforts
figure('Name','Joint Efforts','NumberTitle','off');
for j = 5:8
    plot(time, efforts(:,j), 'LineWidth', 1.5);
    hold on;
end
yline(pwm_limits(1)/tau2pwm)
yline(pwm_limits(2)/tau2pwm)
xlabel('Time [s]');
ylabel('Joint Effort [Nm]');
title('Allegro Hand Joint Efforts');
legend(jointNames([5 6 7 8]), 'Interpreter', 'none');
grid on;


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


%% Animate with synchronized video (fast, fixed)
if ANIMATE
    hand = importrobot(urdfFile);
    hand.DataFormat = 'row';
    hand.Gravity = [0 0 -9.81];

    % Video writer setup
    videoFile = 'allegro_hand_animation.avi';
    vw = VideoWriter(videoFile, 'Motion JPEG AVI');
    vw.FrameRate = 30;
    open(vw);

    % Create figure
    fig2 = figure('Name','Allegro Hand 3D','NumberTitle','off');
    ax = axes(fig2);
    axis equal; hold on;
    lighting gouraud; camlight;
    
    % --- Set a good view (rotate the camera) ---
    % Common useful presets:
    view(ax, [135 25]);      % angled front view (good default)
    % view(ax, [0 0]);       % straight front view
    % view(ax, [180 0]);     % back view
    % view(ax, [90 0]);      % side view
    % Adjust to your liking
    
    % Optionally fix camera position for stability
    camproj(ax, 'perspective');
    camup(ax, [0 0 1]);
    
    % First frame (normal mode)
    show(hand, positions(1,:), 'Parent', ax, ...
        'PreservePlot', false, 'Frames', 'off');
    drawnow;

    timeText = text(ax, -0.08, 0.08, 0.1, ...
        sprintf('Time: %.2f s', 0), 'FontSize', 12, ...
        'FontWeight', 'bold', 'Color', 'r');

    fprintf("Starting fast animation...\n");

    % Precompute video timestamps
    t_video = 0:1/vw.FrameRate:time(end);
    idx = 1;

    % Main loop
    for t = t_video
        % Find closest message index
        while idx < N && time(idx) < t
            idx = idx + 1;
        end

        % Reorder joints
        reordered_pos = positions(idx,:);
        reordered_pos = reordered_pos([1 2 3 4 13 14 15 16 5 6 7 8 9 10 11 12]);

        % Fast update (explicitly disable PreservePlot)
        show(hand, reordered_pos, 'Parent', ax, ...
            'FastUpdate', true, 'PreservePlot', false, 'Frames', 'off');

        % Update time text
        set(timeText, 'String', sprintf('Time: %.2f s', time(idx)));

        drawnow limitrate;
        writeVideo(vw, getframe(fig2));
    end

    close(vw);
    fprintf("Ended fast animation. Video saved: %s\n", videoFile);
end
