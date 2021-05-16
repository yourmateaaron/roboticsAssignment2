function [q_model, q_real] = IKdobot_inputTransform(TR)
    
    %DH 
    d1 = 0.138;
    a2 = 0.135;
    a3 = 0.147;
    a4 = 0.04;
    d5 = -0.13;
    
    % Extract position and rotation
    x = TR(1,4);
    y = TR(2,4);
    z = TR(3,4);
    rpy = tr2rpy(TR);
    
    z_model = z-d1-d5;
    z_real = z;
  
    l = sqrt(x.^2 + y.^2) - a4;
    D_model = sqrt(l.^2 + z_model.^2);
    D_real = sqrt(l.^2 + z_real.^2);

    theta1_model = atan2(z_model,l);
    theta1_real = atan2(z_real,l);
    
    theta2_model = acos((a2.^2 + D_model.^2 - a3.^2)/(2*a2*D_model));
    theta2_real = acos((a2.^2 + D_real.^2 - a3.^2)/(2*a2*D_real));
    
    alpha_model = theta1_model + theta2_model;
    alpha_real = theta1_real + theta2_real;
    
    beta_model = acos((a2.^2 + a3.^2 - D_model.^2)/(2*a2*a3));
    beta_real = acos((a2.^2 + a3.^2 - D_real.^2)/(2*a2*a3));

    % q1
    q_model(1)= atan2(y,x);
    q_real(1) = q_model(1);

    % q2
    q_real(2) =  pi/2 - alpha_real;
    q_model(2) = pi/2 - alpha_model;

    % q3
    q_real(3) = pi - beta_real - alpha_real;
%     q_model(3) = pi/2 - q_model(2) + q_real(3); %###
    q_model(3) = pi/2 - q_model(2) + (pi - beta_model - alpha_model);

    % q4
    q_real(4) = rpy(3) - q_real(1);
    q_model(4) = pi - q_model(2) - q_model(3);


    % q5
    q_model(5) = q_real(4);
    
    % joint limits
    qlim{1} = [-135 135]*pi/180;
    qlim{2} = [5 80]*pi/180;
    qlim{3} = [15 170]*pi/180;
    qlim{4} = [-90 90]*pi/180;
    qlim{5} = [-85 85]*pi/180;

    [qlim{1};qlim{2};qlim{3};qlim{4};qlim{5}];
    
    for i=1:length(q_model)
        if(q_model(i)<qlim{i}(1))
            q_model(i) = qlim{i}(1);
        elseif(q_model(i)>qlim{i}(2))
            q_model(i) = qlim{i}(2);
        end
    end


end

