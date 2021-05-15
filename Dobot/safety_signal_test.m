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

%% Solve IK and plot

%clf
% view([60 30]);

waypointIndex = 1;
[q_model, q_real] = IKdobot_inputTransform(waypointPoses{waypointIndex})
dobot.model.plot(q_model);
tr_model = dobot.model.fkine(q_model)
%% Create cube for simulated cage

cubeCenter_cage = [0, 0, 0];
cubeSides_cage = 1;

plotOptions.plotEdges = true;
[vertex_cage,faces_cage,faceNormals_cage] = RectangularPrism(cubeCenter_cage-cubeSides_cage/2, cubeCenter_cage+cubeSides_cage/2,plotOptions);

axis equal
camlight
view([85 30])
%% Create cube for simulated interupt object
cubeCenter_int = [0.8, 0.8, 0.8];
cubeSides_int = 0.1;

plotOptions.plotEdges = true;
[vertex_int,faces_int,faceNormals_int] = RectangularPrism(cubeCenter_int-cubeSides_int/2, cubeCenter_int+cubeSides_int/2,plotOptions);
%% Operate robot and detect any interupt object enter environment 
steps = 20;

% define the transformations for two poses
T1 = waypointPoses{1};      % first pose
T2 = waypointPoses{6};      % second pose

% use custom IK to solve for joint angles of each pose
q1 = IKdobot_inputTransform(T1)         % sovle for joint angles
q2 = IKdobot_inputTransform(T2)         % sove for joint angles
dobot.model.animate(q1);  % plot first pose
pause(2)

% Use joint interpolation to move between the two poses and plot path
qMatrix = jtraj(q1,q2,steps);

result = true(steps,1);     %create a logical vector
int_collision = false;
for a = 1: steps
    for faceIndex_int = 1:size(faces_int,1)-1
     for faceIndex_cage = 1:size(faces_cage,1)
 
                vertOnPlane_cage = vertex_cage(faces_cage(faceIndex_cage,1)',:);      % simulated cage cube
                vertOnPlane_int_1 = vertex_int(faces_int(faceIndex_int,1)',:);         % simulated interupt object cube
                vertOnPlane_int_2 = vertex_int(faces_int(faceIndex_int,2)',:);
        
                [intersectP,check] = LinePlaneIntersection(faceNormals_cage(faceIndex_cage,:),vertOnPlane_cage,vertOnPlane_int_1,vertOnPlane_int_2); 
                if check==1 && IsIntersectionPointInsideTriangle(intersectP,vertex_cage(faces_cage(faceIndex_cage,:)',:))
                    result = true;
                    int_collision = true;
                    
                end
                end
     end
    if int_collision == true           %stop if any object is within the environment
        disp('Environment_interupted');
        break;
    else
    dobot.model.animate(qMatrix(a,:));
    disp('Environment_safe');
    end
end
