%% Enviroment

% Fence
[f,v,data] = plyread('Fence.ply','tri');
% vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

Fence_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

hold on

% Table
[f,v,data] = plyread('Table.ply','tri');
% vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

Table_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on

% Human
[f,v,data] = plyread('Human.ply','tri');
% vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

Human_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

axis([-5,5,-5,5,0,5]);
axis equal;

% Fire Extinguisher
[f,v,data] = plyread('fire extinguisher.ply','tri');
% vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

Fire_extinguisher_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

% Button
[f,v,data] = plyread('stop button.ply','tri');
% vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

Button_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

% Sponge
[f,v,data] = plyread('Sponge.ply','tri');
% vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

Sponge_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

% Base
surf([-5,-5;5,5],[-5,5;-5,5],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
hold on



% Dobot
robot = Dobot;

