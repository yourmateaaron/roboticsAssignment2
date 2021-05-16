%% Enviroment
clear
clf
% Dobot
robot = Dobot;
hold on;

PlaceObject('Table.ply',[0,0,0]);
PlaceObject('Fence.ply',[0,0,-1]);
PlaceObject('Human.ply',[0,0,-1]);
PlaceObject('polesy.ply',[0,0,-1]);
PlaceObject('sponge.ply',[0,0,0]);
PlaceObject('EStop.ply',[0.3,-0.3,0]);
PlaceObject('WarningSign.ply',[0,0,-1]);
PlaceObject('Siren.ply',[0,0,-1]);




% Base
 surf([-5,-5;5,5],[-5,5;-5,5],[-1,-1;-1,-1],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

surf([-5,-5;5,5],[-5,5;-5,5],[-1,-1;1000,1000],'CData',imread('sky.jpg'),'FaceColor','texturemap');
surf([-5,-5;5,5],[-5,5;-5,5],[1000,1000;-1,-1],'CData',imread('sky.jpg'),'FaceColor','texturemap');
surf([-5,-5;5,5],[-5,5;-5,5],[-1,1000;-1,1000],'CData',imread('sky.jpg'),'FaceColor','texturemap');
surf([-5,-5;5,5],[-5,5;-5,5],[1000,-1;1000,-1],'CData',imread('sky.jpg'),'FaceColor','texturemap');




