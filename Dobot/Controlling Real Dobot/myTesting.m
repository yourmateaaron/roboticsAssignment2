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
waypoints{1} = [0.2067         0    0.1350];        % q = zeros(1,4)
waypoints{2} = [0.1710   -0.1177    0.1376];
waypoints{3} = [0.0882   -0.1875    0.1383];
waypoints{4} = [-0.0078   -0.2064    0.1379];
waypoints{5} = [-0.0145   -0.2993    0.0718];       % right above sponge
waypoints{6} = [-0.0163   -0.2991    0.0255];       % ready to close gripper

% Poses: TR
for i=1:length(waypoints)
    waypoints{i} = eul2tr(end_effector_rotation) * transl(waypoints{i}(1),waypoints{i}(2),waypoints{i}(3));
end

%% Current safety status
currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data

%% Publish custom joint target
joint_target = deg2rad([0,0,0,0]);
dobot.PublishTargetJoint(joint_target);

%% Publish custom end effector pose
waypointIndex = 1;
end_effector_position = waypoints{waypointIndex};
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

waypointIndex = 4;
% % using waypoint coords
% joint_target = IKdobot_real_inputXYZ(waypoints{waypointIndex})

% using waypoint poses
joint_target = IKdobot_real_inputTR(waypoints{waypointIndex})

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
%         end_effector_position = waypoints{i};      % above the sponge and its container
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

pickupPosReached = false;

toolStateMsg.Data = [1,0];      % Input is an array [x,y] x: pump on(1)/off(0) y: open(0)/close(1)
send(toolStatePub,toolStateMsg);
pause(2);

while~(pickupPosReached)
    for i=1:length(waypoints)
        waypointIndex = i;
%         joint_target = IKdobot_real_inputXYZ(waypoints{waypointIndex})
        joint_target = IKdobot_real_inputTR(waypoints{waypointIndex})
        dobot.PublishTargetJoint(joint_target);
        pause(2);
    end
    pickupPosReached = true
end

toolStateMsg.Data = [1,1];      % Input is an array [x,y] x: pump on(1)/off(0) y: open(0)/close(1)
send(toolStatePub,toolStateMsg);
pause(2);

