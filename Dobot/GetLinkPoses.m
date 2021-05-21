function [ transforms ] = GetLinkPoses( link_number,base_pose,links, q )
    %% GetLinkPoses
% Gets the transform for every joint
% link_number - number of robot links
% base_pose - transform of base of robot
% q - robot joint angles
% link -  seriallink robot model
% transforms - list of transforms
    transforms = zeros(4,4,link_number+1);
    transforms(:,:,1) = base_pose;
    L = links;
    
    for i = 1:link_number
        transforms(:,:,i+1) = transforms(:,:,i) * trotz(q(i)+L(i).offset)...
                                                * transl(0,0,L(i).d)...
                                                * transl(L(i).a,0,0)...
                                                * trotx(L(i).alpha);
    end
end
