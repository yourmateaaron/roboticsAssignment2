% was added to the example Dobot class


function endEffectorPose = GetEndEffectorPose(self)
            currentEndEffectorPoseMsg = self.endEffectorStateSub.LatestMessage;
            currentEndEffectorPosition = [currentEndEffectorPoseMsg.Pose.Position.X,
                                          currentEndEffectorPoseMsg.Pose.Position.Y,
                                          currentEndEffectorPoseMsg.Pose.Position.Z];
            currentEndEffectorQuat = [currentEndEffectorPoseMsg.Pose.Orientation.W,
                                      currentEndEffectorPoseMsg.Pose.Orientation.X,
                                      currentEndEffectorPoseMsg.Pose.Orientation.Y,
                                      currentEndEffectorPoseMsg.Pose.Orientation.Z]';
            euler = quat2eul(currentEndEffectorQuat);
            endEffectorPose = transl(currentEndEffectorPosition)*eul2tr(euler);
end