addpath(genpath("description"))
close all

%% Matlab
% Load URDF
robot = importrobot("description/urdf/allegro_hand_description_right_B.urdf");
robot.DataFormat = 'column';               % use column vector for q

% Create a figure
figure;

% Show the robot in home configuration
show(robot, Frames="off");

% Set view and lighting
axis equal
view(3);
light;

% %% --- Highlight Centers of Mass for links 4–7 ---
% % Define which links to highlight (by name or index)
% link_indices = [12 13 14 15];
% 
% hold on
% for i = link_indices
%     link = robot.Bodies{i};
% 
%     % Get CoM position in world frame
%     T = getTransform(robot, robot.homeConfiguration, link.Name);
%     com_local = link.CenterOfMass(:);           % [x y z] in link frame
%     com_world = T(1:3,1:3) * com_local + T(1:3,4);
% 
%     % Plot small red sphere at CoM
%     [xs, ys, zs] = sphere(10);
%     r = 0.005; % sphere radius
%     surf(r*xs + com_world(1), ...
%          r*ys + com_world(2), ...
%          r*zs + com_world(3), ...
%          'FaceColor', 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.8);
% 
%     % Label the link name
%     text(com_world(1)+0.1, com_world(2), com_world(3), ...
%          sprintf('  %s CoM', link.Name), ...
%          'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold');
% end
% 
% hold off


function printRobotTFTree(robot, q)
% PRINTROBOTTFTREE Prints the robot's TF tree hierarchically
%
% robot : RigidBodyTree object
% q     : joint configuration (column vector)

if nargin < 2
    q = zeros(robot.NumBodies,1); % default home config
end

% Build parent-child map
parentMap = containers.Map();
for i = 1:robot.NumBodies
    link = robot.Bodies{i};
    parentMap(link.Name) = link.Parent.Name;
end

% Find root links (child of base)
rootLinks = {};
for i = 1:robot.NumBodies
    link = robot.Bodies{i};
    if strcmp(link.Parent.Name, robot.BaseName)
        rootLinks{end+1} = link.Name;
    end
end

% Recursive function to print tree
function printLink(linkName, indent)
    % Print link name
    fprintf('%s%s\n', indent, linkName);
    
    % Find children
    children = {};
    for j = 1:robot.NumBodies
        l = robot.Bodies{j};
        if strcmp(l.Parent.Name, linkName)
            children{end+1} = l.Name;
        end
    end
    
    % Print transform relative to parent
    if ~strcmp(linkName, robot.BaseName)
        T = getTransform(robot, q, linkName, parentMap(linkName));
        fprintf('%s  Transform relative to parent:\n', indent);
        disp(T);
    end
    
    % Recursive call
    for k = 1:length(children)
        printLink(children{k}, [indent '  ']);
    end
end

% Print each root link
for i = 1:length(rootLinks)
    printLink(rootLinks{i}, '');
end

end
function printRobotTFTreeASCII(robot, q)
% Prints the robot TF tree hierarchically in ASCII format
%
% robot : RigidBodyTree
% q     : joint configuration (column vector)

if nargin < 2
    q = zeros(robot.NumBodies,1);
end

% Build parent-child mapping
childrenMap = containers.Map();
for i = 1:robot.NumBodies
    parent = robot.Bodies{i}.Parent.Name;
    if ~isKey(childrenMap, parent)
        childrenMap(parent) = {};
    end
    cm = childrenMap(parent);
    cm{end+1} = robot.Bodies{i}.Name;
end

% Recursive function to print tree
function printLink(linkName, prefix, isLast)
    connector = '├─ ';
    nextPrefix = [prefix '│  '];
    if isLast
        connector = '└─ ';
        nextPrefix = [prefix '   '];
    end
    fprintf('%s%s%s\n', prefix, connector, linkName);
    
    % Print children
    if isKey(childrenMap, linkName)
        kids = childrenMap(linkName);
        for k = 1:length(kids)
            printLink(kids{k}, nextPrefix, k==length(kids));
        end
    end
end

% Find root links (children of base)
rootLinks = childrenMap(robot.BaseName);

% Print each root link
for i = 1:length(rootLinks)
    printLink(rootLinks{i}, '', i==length(rootLinks));
end

end


q = zeros(16,1);             % home position
printRobotTFTree(robot, q);       

% %% Denavit
% fingerDH
