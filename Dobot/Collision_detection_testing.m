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


%% Create cube for collision detection

cubeCenter = [0.175, -0.3, 0.25];
cubeSides = 0.3;

plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(cubeCenter-cubeSides/2, cubeCenter+cubeSides/2,plotOptions);

axis equal
camlight
view([85 30])

%% Teach

dobot.model.teach;

%% Check for collision of current joint state

% Get transform for every joint
% waypointIndex = 2;
% q = IKdobot_inputTransform(waypointPoses{waypointIndex});
q = dobot.model.getpos();
dobot.model.plot(q);

tr = zeros(4,4,dobot.model.n+1);
tr(:,:,1) = dobot.model.base;
L = dobot.model.links;
for i = 1:dobot.model.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha)
end

% Go through each link and each triangle face and check for intersections

for i = 1:size(tr,3)-1
    for faceIndex = 1:size(faces,1)
        vertOnPlane = vertex(faces(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
        if check==1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
            plot3(intersectP(1),intersectP(2),intersectP(3),'*g');
            display('intersection');
        end
    end
end

%% Detect collisions between two waypoints
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

% Find all the joint states in trajectory that are in collision
result = true(steps,1);     %create a logical vector
robotInCollision = false;
% go through each step of the trajectory and check the result to see if it is in collision
for a = 1: steps
    result(a) = false;
        tr = GetLinkPoses(dobot.model.n,dobot.model.base,dobot.model.links,qMatrix(a,:));        % Get the transform of every joint (i.e. start and end of every link)
        for i = 1:size(tr,3)-1
            for faceIndex = 1:size(faces,1)
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                if check==1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                    plot3(intersectP(1),intersectP(2),intersectP(3),'*g');
%                     display('intersection');
                    result = true;      % set step of logical vector true if incollision
                    robotInCollision = true;
                end
            end
        end
    
    % break out of for loop if collision is detected
    if robotInCollision == true;
        disp('Collision Detected!! Robot STOPPED');
        break;
    else
        dobot.model.animate(qMatrix(a,:));
        disp('Robot is operating safely');
    end
    
end