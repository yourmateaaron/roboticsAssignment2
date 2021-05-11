clear all;
clc;
close all;

%% Create model

myDobot = Dobot()

%% Define waypoints
% Got waypoints from playing with real robot by unlocking the joints

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

%% Solve IK and plot

waypointIndex = 6;
[q_model,q_real] = IKdobot_inputTransform(waypoints{waypointIndex})
myDobot.model.plot(q_model)

