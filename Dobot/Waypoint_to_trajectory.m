function [qMatrix] = Waypoint_to_trajectory(waypointPoses,Sequence,k,steps)
% Generate trajectory qMatrix for a set of waypoint and its senquence
% waypointPoses - Poses of way points
% Sequence - Sequence of a set of waypoints
% k - index of sequence
% steps - number of step to perform the trajectory
   T1 = waypointPoses{Sequence(k)};
    if(k == length(Sequence))
        T2 = waypointPoses{1};
    else
        T2 = waypointPoses{Sequence(k+1)};
    end
    q1 = IKdobot_inputTransform(T1);
    q2 = IKdobot_inputTransform(T2);
    qMatrix = jtraj(q1,q2,steps);