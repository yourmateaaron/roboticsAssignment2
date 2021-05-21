function [interupt_state] = Interuption_checking(vertex_interupt,vertex_cage,faces_interupt,faces_cage,faceNormals_cage)
% function for simulate collision checking between the robot cage aka 
% light curain and an interuption obstacles ( interuption cube)
interupt_state = 0;
for faceIndex_interupt = 1:size(faces_interupt,1)-1
        for faceIndex_cage = 1:size(faces_cage,1)
 
                vertOnPlane_cage = vertex_cage(faces_cage(faceIndex_cage,1)',:);      % simulated cage cube
                vertOnPlane_interupt_1 = vertex_interupt(faces_interupt(faceIndex_interupt,1)',:);         % simulated interupt object cube
                vertOnPlane_interupt_2 = vertex_interupt(faces_interupt(faceIndex_interupt,2)',:);
        
                [intersectP,check] = LinePlaneIntersection(faceNormals_cage(faceIndex_cage,:),vertOnPlane_cage,vertOnPlane_interupt_1,vertOnPlane_interupt_2); 
                if check==1 && IsIntersectionPointInsideTriangle(intersectP,vertex_cage(faces_cage(faceIndex_cage,:)',:))
                    
                    interupt_state = 1;
                    
                end
                end
end
end