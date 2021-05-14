clear all;
clc;
close all;
rosshutdown;
%% Start Dobot Magician Node
rosinit;

%% Start Dobot ROS
dobot = myDobotMagician();

%% Initilise Robot
dobot.InitaliseRobot();

%% Define publishers and subscribers
safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
jointStateSubscriber = rossubscriber('/dobot_magician/joint_states'); % Create a ROS Subscriber to the topic joint_states
endEffectorStateSubscriber = rossubscriber('/dobot_magician/end_effector_poses');
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
pause(1); %Allow some time for MATLAB to start the subscribers

%% Define waypoints

end_effector_rotation = [0,0,0];

% Cordinates: XYZ
waypointCoords{1} = [0.2067         0    0.1350];        % q = zeros(1,4)
waypointCoords{2} = [0.1710   -0.1177    0.1376];
waypointCoords{3} = [0.0882   -0.1875    0.1383];
waypointCoords{4} = [-0.0078   -0.2064    0.1379];
waypointCoords{5} = [-0.0145   -0.2993    0.09];       % right above sponge
waypointCoords{6} = [-0.0163   -0.2991    0.03];       % ready to close gripper
waypointCoords{7} = [0.2057   -0.2312    0.037]; 
waypointCoords{8} = [0.2057   0.2312    0.037]; 

% Poses: TR
for i=1:length(waypointCoords)
    waypointPoses{i} = eul2tr(end_effector_rotation) * transl(waypointCoords{i}(1),waypointCoords{i}(2),waypointCoords{i}(3));
end

%% Current safety status
currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data

%% Publish custom joint target
joint_target = deg2rad([0,0,0,0]);
dobot.PublishTargetJoint(joint_target);

%% Publish custom end effector pose
waypointIndex = 6;
end_effector_position = waypointCoords{waypointIndex};
end_effector_rotation = [0,0,0];  
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

%% Control gripper
% 0 = off
% 1 = on and open
% 2 = on and closed
gripperState = 0;

switch gripperState
    case 0
        toolStateMsg.Data = [0,0];
        send(toolStatePub,toolStateMsg);
    case 1
        toolStateMsg.Data = [1,0];
        send(toolStatePub,toolStateMsg);
    case 2
        toolStateMsg.Data = [1,1];
        send(toolStatePub,toolStateMsg);
    otherwise
        toolStateMsg.Data = [0,0];
        send(toolStatePub,toolStateMsg);
end

%% Control gripper
toolStateMsg.Data = [1,0];      % Input is an array [x,y] x: pump on(1)/off(0) y: open(0)/close(1)
send(toolStatePub,toolStateMsg);

%% E-STOP
dobot.EStopRobot();         % while robot is moving, send this. Must reinitialise to use robot again.

%% Get current join state
currentJointState = jointStateSubscriber.LatestMessage.Position % Get the latest message

%% Get current end effector state

currentEndEffectorPoseMsg = endEffectorStateSubscriber.LatestMessage;
currentEndEffectorPosition = [currentEndEffectorPoseMsg.Pose.Position.X,
                              currentEndEffectorPoseMsg.Pose.Position.Y,
                              currentEndEffectorPoseMsg.Pose.Position.Z];
currentEndEffectorQuat = [currentEndEffectorPoseMsg.Pose.Orientation.W,
                          currentEndEffectorPoseMsg.Pose.Orientation.X,
                          currentEndEffectorPoseMsg.Pose.Orientation.Y,
                          currentEndEffectorPoseMsg.Pose.Orientation.Z]';
euler = quat2eul(currentEndEffectorQuat);
currentEndEffectorPose = transl(currentEndEffectorPosition)*eul2tr(euler)

%% Solve IK

waypointIndex = 1;
% % using waypoint coords
% joint_target = IKdobot_real_inputXYZ(waypointCoords{waypointIndex})

% using waypoint poses
joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})

dobot.PublishTargetJoint(joint_target);

% %% Pick and Place script
% 
% pickupPosReached = false;
% 
% toolStateMsg.Data = [1,0];      % Input is an array [x,y] x: pump on(1)/off(0) y: open(0)/close(1)
% send(toolStatePub,toolStateMsg);
% pause(2);
% 
% % while~(pickupPosReached)
%     for i=1:length(waypoints)
%         end_effector_position = waypointCoords{i};      % above the sponge and its container
%         dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
%         pause(2);
%     end
% %     pickupPosReached = true
% % end
% 
% toolStateMsg.Data = [1,1];      % Input is an array [x,y] x: pump on(1)/off(0) y: open(0)/close(1)
% send(toolStatePub,toolStateMsg);
% pause(2);
% 

%% Pick and Place script using custom IK Solver


toolStateMsg.Data = [1,0];      % Input is an array [x,y] x: pump on(1)/off(0) y: open(0)/close(1)
send(toolStatePub,toolStateMsg);
pause(2);

waypointIndex = 1;
joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})
dobot.PublishTargetJoint(joint_target);
pause(2);

waypointIndex = 4;
joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})
dobot.PublishTargetJoint(joint_target);
pause(2);

waypointIndex = 6;
joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})
dobot.PublishTargetJoint(joint_target);
pause(2);
    
toolStateMsg.Data = [1,1];      % Input is an array [x,y] x: pump on(1)/off(0) y: open(0)/close(1)
send(toolStatePub,toolStateMsg);
pause(2);

waypointIndex = 4;
joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})
dobot.PublishTargetJoint(joint_target);
pause(2);

waypointIndex = 1;
joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})
dobot.PublishTargetJoint(joint_target);
pause(2);

for i=1:3
    waypointIndex = 7;
    joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})
    dobot.PublishTargetJoint(joint_target);
    pause(2);
    waypointIndex = 8;
    joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})
    dobot.PublishTargetJoint(joint_target);
    pause(2);
end

waypointIndex = 4;
joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})
dobot.PublishTargetJoint(joint_target);
pause(2);

waypointIndex = 6;
joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})
dobot.PublishTargetJoint(joint_target);
pause(2);

toolStateMsg.Data = [1,0];      % Input is an array [x,y] x: pump on(1)/off(0) y: open(0)/close(1)
send(toolStatePub,toolStateMsg);
pause(2);

waypointIndex = 4;
joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})
dobot.PublishTargetJoint(joint_target);
pause(2);

waypointIndex = 1;
joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})
dobot.PublishTargetJoint(joint_target);
pause(2);

%% Pick and Place script using custom IK Solver

pickupPosReached = false;

toolStateMsg.Data = [1,0];      % Input is an array [x,y] x: pump on(1)/off(0) y: open(0)/close(1)
send(toolStatePub,toolStateMsg);
pause(2);

while~(pickupPosReached)
    for i=1:length(waypointCoords)
        waypointIndex = i;
%         joint_target = IKdobot_real_inputXYZ(waypointCoords{waypointIndex})
        joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})
        dobot.PublishTargetJoint(joint_target);
        pause(2);
    end
    pickupPosReached = true
end

toolStateMsg.Data = [1,1];      % Input is an array [x,y] x: pump on(1)/off(0) y: open(0)/close(1)
send(toolStatePub,toolStateMsg);
pause(2);
pickupPosReached = false;

while~(pickupPosReached)
    for i=length(waypointCoords):-1:1
        waypointIndex = i;
%         joint_target = IKdobot_real_inputXYZ(waypointCoords{waypointIndex})
        joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})
        dobot.PublishTargetJoint(joint_target);
        pause(2);
    end
    pickupPosReached = true
end
pickupPosReached = false;

while~(pickupPosReached)
    for i=1:5
        waypointIndex = 2;
%         joint_target = IKdobot_real_inputXYZ(waypointCoords{waypointIndex})
        joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})
        dobot.PublishTargetJoint(joint_target);
        pause(2);
        waypointIndex = 7;
%         joint_target = IKdobot_real_inputXYZ(waypointCoords{waypointIndex})
        joint_target = IKdobot_real_inputTR(waypointPoses{waypointIndex})
        dobot.PublishTargetJoint(joint_target);
        pause(2);
    end
    pickupPosReached = true
end
pickupPosReached = false;

%% Wipe Defined area

steps = 50;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
qMatrix = zeros(steps,4);

wipeLocation = [0.225 0];
wipeRadius = 0.025;
wipeHeight = 0.055;

for i=1:steps
    x(1,i) = wipeLocation(1) + wipeRadius*sin(delta*i)
    x(2,i) = wipeLocation(2) + wipeRadius*cos(delta*i)
    x(3,i) = wipeHeight                     
    theta(1,i) = 0;                     % Roll angle 
    theta(2,i) = 0;                     % Pitch angle
    theta(3,i) = 0;                     % Yaw angle
    T = [rpy2r(theta(1,i),theta(2,i),theta(3,i)) x(:,i);    % create transformation of first point and angle
        zeros(1,3)  1 ];
    qMatrix(i,:) = IKdobot_real_inputTR(T);
%     qMatrix(i,:) = dobot.model.ikcon(T)
end

% for publishing target join for real dobot because it can't publish a matrix of joints
pauseTime = 0.03;
for i=1:steps
    joint_target = qMatrix(i,:)
    dobot.PublishTargetJoint(joint_target);
    pause(pauseTime);
end


%%

steps = 50;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
qMatrix = zeros(steps,5);

% Wiping locations
wipeHeight = 0.126;           % height of surface above base of dobot
% Wiping areas
wipeCenter{1} = [0.125, 0.2];
wipeCenter{2} = [0.225, 0];
wipeCenter{3} = [0.125, -0.2];

wipeRadiusIncrement = 0.03;
wipeRadiusMin = 0.03;
wipeRadiusMax = 0.09;

for a=1:length(wipeCenter)
    for wipeRadius=wipeRadiusMin:wipeRadiusIncrement:wipeRadiusMax     % wipe in circles of increasing size
        for i=1:steps
            x(1,i) = wipeCenter{a}(1) + wipeRadius*sin(delta*i);
            x(2,i) = wipeCenter{a}(2) + wipeRadius*cos(delta*i);
            x(3,i) = wipeHeight;                     
            theta(1,i) = 0;                     % Roll angle 
            theta(2,i) = 0;                     % Pitch angle
            theta(3,i) = 0;                     % Yaw angle
            T = [rpy2r(theta(1,i),theta(2,i),theta(3,i)) x(:,i);    % create transformation of first point and angle
                zeros(1,3)  1 ];
            qMatrix(i,:) = IKdobot_inputTransform(T);
%             qMatrix(i,:) = dobot.model.ikcon(T)
        end

        dobot.model.plot(qMatrix,'trail','r-');
    end
end
