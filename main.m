% clear all variables and figures
close all
clear
clc

% Add sub-folders
addpath(genpath(pwd));

%% Set up environment (Obstacles)
env = cell(0);

% a ground plane
ground = collisionBox(1.2, 1.2, 0.01);
Tground = trvec2tform([0.0 0.0 -0.1]);
ground.Pose = Tground;
env = {ground};
clear ground

% a sphere 
s = collisionSphere(0.05)
Ts = trvec2tform([0.3 0 0.3])
s.Pose = Ts;
env = [env; {s}];
clear s

%% Set up manipulator (universal UR5)
ur5 = loadrobot("universalUR5", "DataFormat","row");
currentRobotJConfig = homeConfiguration(ur5);
endEffector = "tool0";

% The official ur5 doesn't have a collision model. So we manually add them.
setUpCollisionDetection(ur5);

%% Set up visualisation perspective
figure("Name","Visualization of Task-space Tracking with Minimum Discontinuities",...
    "Units","normalized",...
    "OuterPosition",[0, 0, 1, 1],...
    "Visible","on");

% Show the manipulator
show(ur5,currentRobotJConfig,'PreservePlot',false,'Frames','off');

% Show all obstacles
hold on
for i = 1:length(env)
    show(env{i});
end
axis([-0.5 1 -0.75 0.75 -0.5 1.0]);
view(45, 20);
grid off


%% Set up task-space path
taskInit = trvec2tform([0.3,0.2,0.05])*axang2tform([0 1 0 pi]);
taskMed1 = trvec2tform([0.35,0.4,0.25])*axang2tform([0 1 0 pi/2])*axang2tform([0 0 1 pi/4]);
taskMed2 = trvec2tform([0.5,0.0,0.1])*axang2tform([1 1 0 -pi/4]);
taskFinal = trvec2tform([0.3,-0.3,0.5])*axang2tform([0 1 0 0]);

% Collect the desired EE orientations
timeInterval = [0; 30];
[taskWaypoints,~] = transformtraj(taskInit,taskFinal,...
    timeInterval,timeInterval(1):0.1:timeInterval(2)); 

% 44waypoints mean the poses are represented by homogeneous matrices
all_44waypoints(:, :, 1) = taskInit; 
all_44waypoints(:, :, 2) = taskMed1;
all_44waypoints(:, :, 3) = taskMed2;
all_44waypoints(:, :, 4) = taskFinal;

% Visualise the discrete task-space waypoints
for i = 1:size(all_44waypoints, 3)
    visualiseAFrame(all_44waypoints(:, :, i));
end

all_waypoints = tform2trvec(all_44waypoints); % n*3 matrix, each row is a position of the waypoint
[xf, yf, zf] = cubic3D(all_waypoints(:, 1)', all_waypoints(:, 2)', all_waypoints(:, 3)', ...
    linspace(timeInterval(1), timeInterval(2), size(all_44waypoints, 3)), ...
    timeInterval(1):0.1:timeInterval(2));
taskPositionWaypoints = [xf;yf;zf];

% Combine the position and orientation
for i = 1:size(taskWaypoints, 3)
    taskWaypoints(1:3, 4, i) = taskPositionWaypoints(:, i);
end

% % Visualize all task-space waypoints (now they are dense)
% for i = 1:size(taskWaypoints, 3)
%     visualiseAFrame(taskWaypoints(:, :, i));
% end

%% Collect all Inverse Kinematic Solutions
vertices = [];
for i = 1:size(taskWaypoints, 3)
    allIk = analyticUR5IK(taskWaypoints(:, :, i));
    for j = size(allIk, 1):-1:1
        inCollision = checkCollision(ur5,allIk(j, :),env, ...
            "Exhaustive","on");
        if any(inCollision)
            allIk(j, :) = [];
        end
    end
    vertices = [vertices; 
        WaypointStates(allIk, taskWaypoints(:, :, i))];
end

%% Define possible states for each task-space waypoint
epsilon_cont = 0.1; % Joint-space discrete connectivity threshold
defineStates(vertices, epsilon_cont);

%% Construct Segments of waypoints
nodes = [];
startindex = 1;
for i = 1:size(vertices, 1)
    if ~isequal(unique(vertices(startindex).gIK_(:, 1)), unique(vertices(i).gIK_(:, 1)))
        nodes = [nodes; struct('startindex_', startindex, 'endindex_', i-1, 'possible_color_', vertices(startindex).gIK_(:, 1))];
        startindex = i;
    end
end
nodes = [nodes; struct('startindex_', startindex, 'endindex_', size(vertices, 1), 'possible_color_', vertices(startindex).gIK_(:, 1))];

%% Solve the problem
solution = newLineSolver(nodes);

%% We choose the first solution
solution = solution(1, 2:end);
% We combine them to form the waypoints
ctrlpoints = [];
for i = 1:size(nodes, 1)
    for j = nodes(i).startindex_:nodes(i).endindex_
        k = find(vertices(j).gIK_(:, 1) == solution(i));
        if ~isempty(k)
            ctrlpoints = [ctrlpoints; [vertices(j).gIK_(k, 2:end), solution(i)]];
        end
    end
end

%% We insert the undesirable paths when two consecutive waypoints are discrete
i = size(ctrlpoints, 1);
while i > 1
    if max(abs(wrapToPi(ctrlpoints(i, 1:6) - ctrlpoints(i-1, 1:6)))) > epsilon_cont
        % We run a biRRT planner
        planner = manipulatorRRT(ur5, env);
        planner.MaxConnectionDistance = 0.3;
        planner.ValidationDistance = 0.1;
        
        startConfig = ctrlpoints(i-1, 1:6);
        goalConfig = ctrlpoints(i, 1:6);

        rng('default');
        path = plan(planner,startConfig,goalConfig);
        shortenedPath = shorten(planner,path,20);
        interpStates = interpolate(planner, shortenedPath);
        ctrlpoints = [ctrlpoints(1:i-1, :); 
            [interpStates(2:end-1, :), repmat([-1], [size(interpStates, 1)-2, 1])]; 
            ctrlpoints(i:end, :)];
    end 
    i = i-1;
end

jointConfigArray = ctrlpoints;

% Re-interpolate the path (containing the reconfiguration motion) to
% generate a smooth manipulator trajectory. 
jointConfigArray = jointCubic(jointConfigArray);

%% Visualize Motion
for i=1:size(jointConfigArray, 1)
    poseNow = getTransform(ur5, jointConfigArray(i, 1:6), endEffector);
    show(ur5,jointConfigArray(i, 1:6),'PreservePlot',false,'Frames','off');
    if jointConfigArray(i, 7) == -1
        jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4), '.', 'Color', [0.6, 0.6, 0.6], 'MarkerSize',20);
    elseif jointConfigArray(i, 7) == 1
        jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'.', 'Color', [0.2, 0.7, 0.2],'MarkerSize',20);
    elseif jointConfigArray(i, 7) == 2
        jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'.', 'Color', [0.2, 0.2, 0.7],'MarkerSize',20);
    elseif jointConfigArray(i, 7) == 6
        jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'.', 'Color', [0.7, 0.2, 0.2],'MarkerSize',20);
    elseif jointConfigArray(i, 7) == 7
        jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'.', 'Color', [0.2, 0.7, 0.7],'MarkerSize',20);
    end
    
    drawnow;
end
 