close all
%% Plot Allegro Hand Joint States from ROS2 bag
% Make sure you have ROS Toolbox installed
addpath(genpath("description"))

ANIMATE = false;
EXPORT = false;

% Path to the rosbag folder (ros2 bag record creates a folder)
bagFolder = './bag/allegro_hand_torque_bag';
% bagFolder = './bag/allegro_hand_bag';
urdfFile = "description/urdf/allegro_hand_description_right_B.urdf";
% Open the ros2 bag
bag = ros2bag(bagFolder);

% Select the /allegroHand/joint_states topic
% Select the /allegroHand/joint_cmd topic
ts = select(bag, 'Topic', '/allegroHand_0/joint_states');
ts_cmd = select(bag, 'Topic', '/allegroHand_0/joint_cmd');


% STATES ------------------------------
% Read all messages
msgs = readMessages(ts);

% Number of messages
N = numel(msgs);

% Extract joint names (from the first message)
jointNames = msgs{1}.name;   % cell array of 16 joint names
numJoints = numel(jointNames);

% Preallocate arrays
positions = zeros(N, numJoints);
velocities = zeros(N, numJoints);
efforts   = zeros(N, numJoints);
time      = zeros(N, 1);

% Loop through messages
for i = 1:N
    msg = msgs{i};
    positions(i,:) = msg.position;
    velocities(i,:) = msg.velocity;
    efforts(i,:)   = msg.effort;
    time(i) = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
end

% Normalize time to start at 0
t0 = time(1);
time = time - t0;

% CMD ------------------------------
% Read all messages
msgs = readMessages(ts_cmd);

% Number of messages
N = numel(msgs);

% Extract joint names (from the first message)
jointNames = msgs{1}.name;   % cell array of 16 joint names
numJoints = numel(jointNames);

% Preallocate arrays
positions_des = zeros(N, numJoints);
time_des      = zeros(N, 1);

% Loop through messages
for i = 1:N
    msg = msgs{i};
    positions_des(i,:) = msg.position;
    time_des(i) = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
end

% Normalize time to start at 0
time_des = time_des - t0;

% RESAMPLE
% positions_des_resampled = zeros(size(positions));
% 
% % Initialize index for positions_des
% j = j;
% for i = 1:size(time,1)
%     % Move j forward while next desired time is <= current time
%     while j < size(time_des,1) && time_des(j+1) <= time(i)
%         j = j + 1;
%     end
%     % Assign the most recent value
%     positions_des_resampled(i,:) = positions_des(j,:);
% end

positions_des_ori = positions_des;
positions_des_resampled = zeros(size(positions));
for i = 1:size(time,1)
    idx = find(time_des <= time(i), 1, 'last');  % last time_des <= time(i)
    if isempty(idx)
        idx = 1;  % or NaN if you want undefined before first sample
    end
    positions_des_resampled(i,:) = positions_des(idx,:);
end
positions_des = positions_des_resampled;
%% 
pwm_limits = [-240,240];
tau2pwm = 1.43 * 1000;
% estimate tau 
Kp = [1, 1, 1, 1];
Kd = [0.04, 0.15, 0.04, 0.04];
tau_PD = zeros(size(efforts));
for i = 1:16
    kps =  Kp(mod(i-1,4)+1); 
    kds = Kd(mod(i-1,4)+1); 
    tau_PD(:,i) = (positions_des(:,i) - positions(:,i)) * kps - kds * velocities(:,i); 
end

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
%% Plot joint velocities
figure('Name','Joint velocities','NumberTitle','off');
for j = 5:8
    plot(time, velocities(:,j), 'LineWidth', 1.5);
    hold on;
end
xlabel('Time [s]');
ylabel('Joint Position [rad/s]');
title('Allegro Hand Joint velocities');
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

%% Plot tau3 PD
figure('Name','Joint8 error','NumberTitle','off');
hold on
plot(time, positions_des(:,8)-positions(:,8),'LineWidth', 1.5);
% plot(time_des, positions_des_ori(:,8),'.', 'LineWidth', 1.5);
% plot(time, efforts(:,8) - tau_PD(:,8), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('position error [rad]');
title('Allegro Hand Position Des');
legend('l3', 'Interpreter', 'none');
grid on;

figure('Name','Joint8 target','NumberTitle','off');
for j = 5:8
    subplot(2, 2, j-4);
    hold on
    plot(time, positions(:,j),'LineWidth', 1.5);
    plot(time, positions_des(:,j), 'LineWidth', 1.5);
end
xlabel('Time [s]');
ylabel('position error [rad]');
title('Allegro Hand Position Des');
legend('l3', 'Interpreter', 'none');
grid on;
%% Plot tau3 PD
figure('Name','Joint8 PD','NumberTitle','off');

hold on
plot(time, efforts(:,8), 'LineWidth', 1.5);
plot(time, tau_PD(:,8),'LineWidth', 1.5);
% plot(time, efforts(:,8) - tau_PD(:,8), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('tau [Nm]');
title('Allegro Hand Last tau PD');
legend('l3', 'Interpreter', 'none');
grid on;



figure('Name','Joint8 PD','NumberTitle','off');
for j = 5:8
    subplot(2, 2, j-4);
    plot(time, efforts(:,j) - tau_PD(:,j), 'LineWidth', 1.5);
    grid on;
end
% plot(time, efforts(:,8) - tau_PD(:,8), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('l3 [m]');
title('Allegro Hand Last tau delta');
legend('Delta Tau', 'Interpreter', 'none');

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


% FROM BAG TORQUE
        % link 1:
        %         inertial:
        %                 symb: [1,1,1,1,1,1,1,1,1,1]
        %                 mass: 0.0176
        %                 CoM_x: 0.039300360175544974
        %                 CoM_y: 0.14474135382200007
        %                 CoM_z: 0.0023285583194372237
        % link 2:
        %         inertial:
        %                 symb: [1,1,1,1,1,1,1,1,1,1]
        %                 mass: 0.086
        %                 CoM_x: -0.10961734069424044
        %                 CoM_y: -0.0385729796498439
        %                 CoM_z: 0.0949318299919504
        % link 3:
        %         inertial:
        %                 symb: [1,1,1,1,1,1,1,1,1,1]
        %                 mass: 0.0367
        %                 CoM_x: -0.01643311574700469
        %                 CoM_y: -0.00132690823116909
        %                 CoM_z: 0.14909133209271838
        % link 4:
        %         inertial:
        %                 symb: [1,1,1,1,1,1,1,1,1,1]
        %                 mass: 0.0057
        %                 CoM_x: 0.2985621861304473
        %                 CoM_y: 0.029334558839954766
        %                 CoM_z: -6.754609317758603e-22
        
 % FROM BAG GRAVITY COMP
        %  link 0:
        %         inertial:
        %                 symb: [1,1,1,1,1,1,1,1,1,1]
        %                 mass: 0.0176
        %                 CoM_x: 0.05464461951491161
        %                 CoM_y: 0.10929927027490996
        %                 CoM_z: 0.08699215525231409
        % link 1:
        %         inertial:
        %                 symb: [1,1,1,1,1,1,1,1,1,1]
        %                 mass: 0.086
        %                 CoM_x: -0.1547754457577204
        %                 CoM_y: -0.01987271220813074
        %                 CoM_z: 7.288636329298263e-07
        %                 Fs: 0.0
        % link 2:
        %         inertial:
        %                 symb: [1,1,1,1,1,1,1,1,1,1]
        %                 mass: 0.0367
        %                 CoM_x: -0.016454213005063542
        %                 CoM_y: 0.0031797519940895734
        %                 CoM_z: 0.1490608872266794
        % link 3:
        %         inertial:
        %                 symb: [1,1,1,1,1,1,1,1,1,1]
        %                 mass: 0.0057
        %                 CoM_x: 0.29986509789636356
        %                 CoM_y: 0.008992521759715474
        %                 CoM_z: -1.8474474025812295e-27