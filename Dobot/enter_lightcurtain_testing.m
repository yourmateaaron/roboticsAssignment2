clear all
clc
clf
% set(0,'DefaultFigureWindowStyle','docked')

%% Create model

dobot = Dobot()

%% Define waypoints
% Got waypoints from playing with real robot by unlocking the joints

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


%% 
% intBrick = intBrick();
intPerson = intPerson();


%%
cubeCenter_cage = [0, 0, 0];
cubeSides_cage = 2;

plotOptions.plotEdges = true;
[vertex_cage,faces_cage,faceNormals_cage] = RectangularPrism(cubeCenter_cage-cubeSides_cage/2, cubeCenter_cage+cubeSides_cage/2,plotOptions);

%%
% intPerson.crossLightCurtain(15);
steps = 45;

% define the transformations for two poses
T1 = waypointPoses{7};      % first pose
T2 = waypointPoses{8};      % second pose

% use custom IK to solve for joint angles of each pose
q1 = IKdobot_inputTransform(T1)         % sovle for joint angles
q2 = IKdobot_inputTransform(T2)         % sove for joint angles

% Use joint interpolation to move between the two poses and plot path
qMatrix = jtraj(q1,q2,steps);

for a = 1: steps
    if(intPerson.person{1,1}.base(2,4) < 1 && intPerson.person{1,1}.base(2,4) > -1 )
        disp('Light Curtain triggered');
        break;
    else
        dobot.model.animate(qMatrix(a,:));
        intPerson.PlotSingleStep();
        disp('Environment is safe');
    end
end


% if(intPerson.person{1,1}.base(2,4) < 1 && intPerson.person{1,1}.base(2,4) > -1 )
%     disp('inside light curtain')
% else
%     disp('outside light curtain')
% end
