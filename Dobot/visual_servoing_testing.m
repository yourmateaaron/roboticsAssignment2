clear all
clc
set(0,'DefaultFigureWindowStyle','docked')
clf

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


%% Vision

% clf
% view([60 30]);

waypointIndex = 2;
[q_model, q_real] = IKdobot_inputTransform(waypointPoses{waypointIndex})
dobot.model.plot(q_model);

% definitions of target square with corners
p1Star = [662 362]; 
p2Star = [362 362]; 
p3Star = [362 662]; 
p4Star = [662 662];
pStar = [p1Star' p2Star' p3Star' p4Star'];

% cartesian coords of target's corners
P1 = [0.23, 0.03, 0]; 
P2 = [0.23, -0.03, 0]; 
P3 = [0.17, -0.03, 0]; 
P4 = [0.17, 0.03, 0];
P = [P1' P2' P3' P4'];

% add the camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'DobotCamera');

% given frame rate
fps = 25;

% define values
% gain of the controller
lambda = 0.6;
% depth of the IBVS
depth = mean(P(1,:));

% eye in hand config
tr = dobot.GetLinkPoses(q_model)
Tc0 = tr(:,:,5) * trotx(pi)

% eye to hand config
% Tc0 = transl(0.5,0.5,0.5) * trotx(3*pi/4) *troty(-pi/8)


% plot camera and points
cam.T = Tc0;

% Display points in 3D and the camera
cam.plot_camera('Tcam',Tc0, 'label','scale',0.06);
plot_sphere(P, 0.02, 'b')
lighting gouraud
light

%camera view and plotting
cam.clf()
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
