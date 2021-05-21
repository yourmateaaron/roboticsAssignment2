function [qMatrix]  = RMRC_motion(waypointCoords,startWaypointIndex,endWaypointIndex,model,t_val,deltaT_val,epsilon_val)
x1 = waypointCoords{startWaypointIndex}';
x2 = waypointCoords{endWaypointIndex}';

% Set parameters for the simulation
t = t_val;             % Total time (s)
deltaT = deltaT_val;      % Control frequency
steps = t/deltaT;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
epsilon = epsilon_val;      % Threshold value for manipulability/Damped Least Squares
% W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
W = diag([1 1 1 0.1 0.1]);

% Allocate array data
m = zeros(steps,1);             % Array for Measure of Manipulability
qMatrix = zeros(steps,5);       % Array for joint angles
qdot = zeros(steps,5);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory
positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting trajectory error

% Set up trajectory, intial pose
s = lspb(0,1,steps);            % trapezoidal trajectory scalar
for i=1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*x2;     % Points in x-y-z
    theta(1,i) = 0;                     % Roll angle 
    theta(2,i) = 0;                     % Pitch angle
    theta(3,i) = 0;                     % Yaw angle
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);    % create transformation of first point and angle
        zeros(1,3)  1 ];
% q0 = zeros(1,5);                                      % initial guess for joint angles
% qMatrix(1,:) = dobot.model.ikcon(T,q0);
qMatrix(1,:) = IKdobot_inputTransform(T);               % solve joint angles to achieve first waypoint                   

% Track trajectory with RMRC
for i=1:steps-1
    T = model.fkine(qMatrix(i,:));                   % FK for pose at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                       % position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1)); % next RPY angles then convert to rotation matrix (desired rotation)
    Ra = T(1:3,1:3);                                    % actual rotation matrix
    Rdot = (1/deltaT) * (Rd - Ra);                      % rotation matrix error
    S = Rdot * Ra';                                     % skew symmetrix matrix to extract linear and angular velocity
    linear_velocity = (1/deltaT) * deltaX;
%     angular_velocity = [S(3,2); S(1,3); S(2,1)];
    angular_velocity = [S(3,2); S(1,3)];
    deltaTheta = tr2rpy(Rd * Ra');
    xdot = W*[linear_velocity;angular_velocity];        % calculate end-effector velocity to reach next waypoint
    J = model.jacob0(qMatrix(i,:));             % get jacobian at current joint state
    J = J(1:5,:);           % take first 5 rows to make square matrix
    mom(i) = real(sqrt(det(J*J')));
    if mom(i) < epsilon                       % if manipulability is less than given threshold
        lambda = (1 - mom(i)/epsilon)*5E-2;
    else
        lamda = 0;
    end
    invJ = inv(J'*J + lambda *eye(5))*J';      % DLS inverse
    qdot(i,:) = (invJ * xdot)';                 % solve the RMRC equation
    
    for j = 1:5                     % Loop through joints 1 to 5
        % If next joint angle is lower than joint limit, stop the model
        if qMatrix(i,j) + deltaT*qdot(i,j) < model.qlim(j,1)
            qdot(i,j) = 0;
        % If next joint angle is greater than joint limit, stop the model
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > model.qlim(j,2)
            qdot(i,j) = 0;
        end
    end
    
%     update next joint state based on joint velocities
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);
    positionError(:,i) = x(:,i+1) - T(1:3,4);           % For plotting
    angleError(:,i) = deltaTheta;                       % For plotting

end
end