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
waypointCoords{5} = [-0.0145   -0.2993    0.0718];       % right above sponge
waypointCoords{6} = [-0.0163   -0.2991    0.0255];       % ready to close gripper
waypointCoords{7} = [0.4   0.1177    0.1376];


% Poses: TR
for i=1:length(waypointCoords)
    waypointPoses{i} = eul2tr(end_effector_rotation) * transl(waypointCoords{i}(1),waypointCoords{i}(2),waypointCoords{i}(3));
end

%% Solve IK and plot (custom)

% clf
% view([60 30]);

waypointIndex = 1;
[q_model, q_real] = IKdobot_inputTransform(waypointPoses{waypointIndex})
dobot.model.plot(q_model);
tr_model = dobot.model.fkine(q_model)

%% Solve IK and plot (ikcon)
% 
% clf
% view([45 30]);

waypointIndex = 1;
q_model_ikcon = dobot.model.ikcon(waypointPoses{waypointIndex})
dobot.model.plot(q_model_ikcon);
tr_model_ikcon = dobot.model.fkine(q_model_ikcon)

%% Move Dobot using Joint Interpolation

clf
view([60 30]);

steps = 50;

% define the transformations for two poses
T1 = waypointPoses{1};      % first pose
T2 = waypointPoses{3};      % second pose

% use custom IK to solve for joint angles of each pose
q1 = IKdobot_inputTransform(T1)         % sovle for joint angles
q2 = IKdobot_inputTransform(T2)         % sove for joint angles
dobot.model.plot(q1,'trail','r-');  % plot first pose
pause(2)

% Use joint interpolation to move between the two poses and plot path
qMatrix = jtraj(q1,q2,steps);
dobot.model.plot(qMatrix,'trail','r-');       % not plotting trail??

%% Wipe pre-defined area

clf
view([0 0]);

steps = 50;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
qMatrix = zeros(steps,5);

wipeLocation = [0.2 0];
wipeRadius = 0.05;
wipeHeight = 0.126;

for i=1:steps
    x(1,i) = wipeLocation(1) + wipeRadius*sin(delta*i)
    x(2,i) = wipeLocation(2) + wipeRadius*cos(delta*i)
    x(3,i) = wipeHeight                     
    theta(1,i) = 0;                     % Roll angle 
    theta(2,i) = 0;                     % Pitch angle
    theta(3,i) = 0;                     % Yaw angle
    T = [rpy2r(theta(1,i),theta(2,i),theta(3,i)) x(:,i);    % create transformation of first point and angle
        zeros(1,3)  1 ];
    qMatrix(i,:) = IKdobot_inputTransform(T);
%     qMatrix(i,:) = dobot.model.ikcon(T)
end

dobot.model.plot(qMatrix,'trail','r-');

% % for publishing target join for real dobot because it can't publish a matrix of joints
% pauseTime = 0.2;
% for i=1:steps
%     dobot.model.plot(qMatrix(i,:),'trail','r-');
%     pause(pauseTime);
% end

%% Wipe Multiple pre-defined areas

clf
view([60 30]);

steps = 25;   % No. of steps for simulation
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


%% Move Dobot model using RMRC

clf
view([60 30]);

startWaypointIndex = 5;
endWaypointIndex = 1;
x1 = waypointCoords{startWaypointIndex}';
x2 = waypointCoords{endWaypointIndex}';

% Set parameters for the simulation
t = 10;             % Total time (s)
deltaT = 0.25;      % Control frequency
steps = t/deltaT;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
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
    T = dobot.model.fkine(qMatrix(i,:));                   % FK for pose at current joint state
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
    J = dobot.model.jacob0(qMatrix(i,:));             % get jacobian at current joint state
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
        if qMatrix(i,j) + deltaT*qdot(i,j) < dobot.model.qlim(j,1)
            qdot(i,j) = 0;
        % If next joint angle is greater than joint limit, stop the model
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > dobot.model.qlim(j,2)
            qdot(i,j) = 0;
        end
    end
    
%     update next joint state based on joint velocities
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);
    positionError(:,i) = x(:,i+1) - T(1:3,4);           % For plotting
    angleError(:,i) = deltaTheta;                       % For plotting

end

dobot.model.plot(qMatrix,'trail','r-')


% % plot RMRC results
% for i = 1:5
%     figure(2)
%     subplot(3,2,i)
%     plot(qMatrix(:,i),'k','LineWidth',1)
%     title(['Joint ', num2str(i)])
%     ylabel('Angle (rad)')
%     refline(0,dobot.model.qlim(i,1));
%     refline(0,dobot.model.qlim(i,2));
%     
%     figure(3)
%     subplot(3,2,i)
%     plot(qdot(:,i),'k','LineWidth',1)
%     title(['Joint ',num2str(i)]);
%     ylabel('Velocity (rad/s)')
%     refline(0,0)
% end
% 
% figure(4)
% subplot(2,1,1)
% plot(positionError'*1000,'LineWidth',1)
% refline(0,0)
% xlabel('Step')
% ylabel('Position Error (mm)')
% legend('X-Axis','Y-Axis','Z-Axis')
% 
% subplot(2,1,2)
% plot(angleError','LineWidth',1)
% refline(0,0)
% xlabel('Step')
% ylabel('Angle Error (rad)')
% legend('Roll','Pitch','Yaw')
% figure(5)
% plot(m,'k','LineWidth',1)
% refline(0,epsilon)
% title('Manipulability')

%% Wipe table RMRC

clf
view([60 30]);

% Set parameters for the simulation
t = 10;             % Total time (s)
deltaT = 0.2;      % Control frequency
steps = t/deltaT;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
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

wipeCenter{1} = [0 0.2];
wipeCenter{2} = [0.15 0.15];
wipeCenter{3} = [0.2 0];
wipeCenter{4} = [0.15 -0.15];

for a=1:length(wipeCenter)
    for r=0.03:0.04:0.11      % wipe in circles of increasing size
        % Set up trajectory, intial pose
        s = lspb(0,1,steps);            % trapezoidal trajectory scalar
        for i=1:steps
            x(1,i) = wipeCenter{a}(1) + r*cos(delta*i);
            x(2,i) = wipeCenter{a}(2) + r*sin(delta*i);
            x(3,i) = 0.135;                            
            theta(1,i) = 0;                     % Roll angle 
            theta(2,i) = 0;                     % Pitch angle
            theta(3,i) = 0;                     % Yaw angle
        end

        T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);    % create transformation of first point and angle
                zeros(1,3)  1 ];
%         q0 = zeros(1,5);                                      % initial guess for joint angles
%         qMatrix(1,:) = dobot.model.ikcon(T,q0);
        qMatrix(1,:) = IKdobot_inputTransform(T);               % solve joint angles to achieve first waypoint                   

        % Track trajectory with RMRC
        for i=1:steps-1
            T = dobot.model.fkine(qMatrix(i,:));                   % FK for pose at current joint state
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
            J = dobot.model.jacob0(qMatrix(i,:));             % get jacobian at current joint state
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
                if qMatrix(i,j) + deltaT*qdot(i,j) < dobot.model.qlim(j,1)
                    qdot(i,j) = 0;
                % If next joint angle is greater than joint limit, stop the model
                elseif qMatrix(i,j) + deltaT*qdot(i,j) > dobot.model.qlim(j,2)
                    qdot(i,j) = 0;
                end
            end

        %     update next joint state based on joint velocities
            qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);
            positionError(:,i) = x(:,i+1) - T(1:3,4);           % For plotting
            angleError(:,i) = deltaTheta;                       % For plotting

        end

        dobot.model.plot(qMatrix,'trail','r-')
    end
end