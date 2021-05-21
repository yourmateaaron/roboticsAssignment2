function [waypointCoords,waypointPoses,wipeSequence,steps] = GeneralWipe_data(steps_number)
% generate data of GeneralWipe Motion with input as number of step

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
wipeSequence = [1,4,6,4,1,7,8];
steps = steps_number;