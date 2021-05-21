function [qMatrix,steps] = Wipe_Eclipse(wipeLocation,wipeRadius,wipeHeight,number_step)
% generate trajectory qMatrix for a wipe in eclipse area motion
% wipeLocation - location of eclipse
% wipeRadius - Radius of the eclipse
% wipeHeight - height of the eclipse
% number_step - number of step for the trajectory
steps = number_step;
delta = 2*pi/steps; % Small angle change
qMatrix = zeros(steps,5);


for i=1:steps
    x(1,i) = wipeLocation(1) + wipeRadius(1)*sin(delta*i);
    x(2,i) = wipeLocation(2) + wipeRadius(2)*cos(delta*i);
    x(3,i) = wipeHeight    ;                 
    theta(1,i) = 0;                     % Roll angle 
    theta(2,i) = 0;                     % Pitch angle
    theta(3,i) = 0;                     % Yaw angle
    T = [rpy2r(theta(1,i),theta(2,i),theta(3,i)) x(:,i);    % create transformation of first point and angle
        zeros(1,3)  1 ];
    qMatrix(i,:) = IKdobot_inputTransform(T);    
end
end