function [collide_state] = Collision_checking(vertex,faces,faceNormals,tr)
% collision checking between robot and an obstacles (cube)
% vertex - vertex of the cube
% faces - faces of the cube
% faceNormals - faceNormals of the cube ( RectangularPrism file)
% tr - trajectory transformation
collide_state = 0;
 for i = 1:size(tr,3)-1
            for faceIndex = 1:size(faces,1)
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                if check==1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                    collide_state = 1;
                
                end
            end
 end
end