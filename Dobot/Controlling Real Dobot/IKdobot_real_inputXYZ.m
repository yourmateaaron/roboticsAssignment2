function [q_real] = IKdobot_real_inputXYZ(XYZ)
    
    % Link lengths
    d1 = 0.138;
    a2 = 0.135;
    a3 = 0.147;
    a4 = 0.0597;
    
    % Extract position and rotation
    x = XYZ(1)
    y = XYZ(2);
    z = XYZ(3);
    rpy = [0,0,0];      % end effector no rotation
    
%     z = z - d1;
    
    l = sqrt(x.^2 + y.^2) - a4;
    D = sqrt(l.^2 + z.^2);

    theta1 = atan(z/l);
    theta2 = acos((a2.^2 + D.^2 - a3.^2)/(2*a2*D));
    alpha = theta1 + theta2;
    beta = acos((a2.^2 + a3.^2 - D.^2)/(2*a2*a3));

    % q1
    q_real(1)= atan2(y,x);
    q_model(1) = q_real(1);

    % q2
    q_model(2) =  pi/2 - alpha;
    q_real(2) = q_model(2);

    % q3
    q_real(3) = pi - beta - alpha;
    q_model(3) = pi/2 - q_model(2) + q_real(3); 

    % q4
    q_real(4) = rpy(3) - q_real(1);             % desired rotation - the angle that was rotated to face the point
    q_model(4) = pi - q_model(2) - q_model(3);

end

